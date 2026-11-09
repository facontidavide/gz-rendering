/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <algorithm>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Scene.h>

#include <gz/common/Console.hh>

#include "gz/rendering/filament/FilamentArrowVisual.hh"
#include "gz/rendering/filament/FilamentAxisVisual.hh"
#include "gz/rendering/filament/FilamentCamera.hh"
#include "gz/rendering/filament/FilamentCapsule.hh"
#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentDepthCamera.hh"
#include "gz/rendering/filament/FilamentDynamicMesh.hh"
#include "gz/rendering/filament/FilamentGrid.hh"
#include "gz/rendering/filament/FilamentGpuRays.hh"
#include "gz/rendering/filament/FilamentBoundingBoxCamera.hh"
#include "gz/rendering/filament/FilamentHeightmap.hh"
#include "gz/rendering/filament/FilamentSegmentationCamera.hh"
#include "gz/rendering/filament/FilamentThermalCamera.hh"
#include "gz/rendering/filament/FilamentWideAngleCamera.hh"
#include "gz/rendering/filament/FilamentLight.hh"
#include "gz/rendering/filament/FilamentMarker.hh"
#include "gz/rendering/filament/FilamentMaterial.hh"
#include "gz/rendering/filament/FilamentMesh.hh"
#include "gz/rendering/filament/FilamentMeshFactory.hh"
#include "gz/rendering/filament/FilamentParticleEmitter.hh"
#include "gz/rendering/filament/FilamentRayQuery.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentRenderTarget.hh"
#include "gz/rendering/filament/FilamentScene.hh"
#include "gz/rendering/filament/FilamentVisual.hh"
#include "gz/rendering/filament/FilamentWireBox.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
FilamentScene::FilamentScene(unsigned int _id, const std::string &_name)
  : BaseScene(_id, _name)
{
}

//////////////////////////////////////////////////
FilamentScene::~FilamentScene()
{
}

//////////////////////////////////////////////////
void FilamentScene::Destroy()
{
  // meshes detached from every visual but still referenced by the user
  // keep their renderables (and thus material instances) alive; ogre2 drops
  // them with its scene manager, do the same. Attached ones go with their
  // visual in Clear(). (copy: Destroy unregisters)
  const std::set<FilamentMesh *> leftover = this->meshes;
  for (FilamentMesh *mesh : leftover)
  {
    if (mesh->HasParent())
      continue;
    auto keepAlive = mesh->shared_from_this();
    mesh->Destroy();
  }
  BaseScene::Destroy();
}

//////////////////////////////////////////////////
void FilamentScene::Fini()
{
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  if (engine && this->ambientLight)
  {
    engine->destroy(this->ambientLight);
    this->ambientLight = nullptr;
  }
  if (engine && this->scene)
  {
    engine->destroy(this->scene);
    this->scene = nullptr;
  }
}

//////////////////////////////////////////////////
RenderEngine *FilamentScene::Engine() const
{
  return FilamentRenderEngine::Instance();
}

//////////////////////////////////////////////////
VisualPtr FilamentScene::RootVisual() const
{
  return this->rootVisual;
}

//////////////////////////////////////////////////
math::Color FilamentScene::AmbientLight() const
{
  return this->ambient;
}

//////////////////////////////////////////////////
VisualPtr FilamentScene::VisualAt(const CameraPtr &_camera,
    const math::Vector2i &_mousePos)
{
  return _camera ? _camera->VisualAt(_mousePos) : VisualPtr();
}

//////////////////////////////////////////////////
void FilamentScene::SetAmbientLight(const math::Color &_color)
{
  this->ambient = _color;
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  if (!engine || !this->scene)
    return;
  if (this->ambientLight)
    engine->destroy(this->ambientLight);
  // A single-band SH is a uniform irradiance: classic constant ambient term.
  // Intensity calibrated against ogre2, whose ambient term measures as
  // ~0.32 * ambient * albedo (linear) on a white plane.
  filament::math::float3 sh[1] = {FilamentConversions::ToFloat3(_color)};
  this->ambientLight = filament::IndirectLight::Builder()
      .irradiance(1, sh)
      .intensity(1.0f / 3.0f)
      .build(*engine);
  this->scene->setIndirectLight(this->ambientLight);
}

//////////////////////////////////////////////////
filament::Scene *FilamentScene::Native() const
{
  return this->scene;
}

//////////////////////////////////////////////////
const std::set<FilamentMesh *> &FilamentScene::Meshes() const
{
  return this->meshes;
}

//////////////////////////////////////////////////
void FilamentScene::RegisterMesh(FilamentMesh *_mesh)
{
  this->meshes.insert(_mesh);
}

//////////////////////////////////////////////////
void FilamentScene::UnregisterMesh(FilamentMesh *_mesh)
{
  this->meshes.erase(_mesh);
}

//////////////////////////////////////////////////
const std::vector<FilamentParticleEmitter *> &
    FilamentScene::ParticleEmitters() const
{
  return this->particleEmitters;
}

//////////////////////////////////////////////////
void FilamentScene::RegisterParticleEmitter(FilamentParticleEmitter *_emitter)
{
  this->particleEmitters.push_back(_emitter);
}

//////////////////////////////////////////////////
void FilamentScene::UnregisterParticleEmitter(
    FilamentParticleEmitter *_emitter)
{
  auto &v = this->particleEmitters;
  v.erase(std::remove(v.begin(), v.end(), _emitter), v.end());
}

//////////////////////////////////////////////////
ParticleEmitterPtr FilamentScene::CreateParticleEmitterImpl(unsigned int _id,
    const std::string &_name)
{
  FilamentParticleEmitterPtr visual(new FilamentParticleEmitter);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
DirectionalLightPtr FilamentScene::CreateDirectionalLightImpl(
    unsigned int _id, const std::string &_name)
{
  FilamentDirectionalLightPtr light(new FilamentDirectionalLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
PointLightPtr FilamentScene::CreatePointLightImpl(unsigned int _id,
    const std::string &_name)
{
  FilamentPointLightPtr light(new FilamentPointLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
SpotLightPtr FilamentScene::CreateSpotLightImpl(unsigned int _id,
    const std::string &_name)
{
  FilamentSpotLightPtr light(new FilamentSpotLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
CameraPtr FilamentScene::CreateCameraImpl(unsigned int _id,
    const std::string &_name)
{
  FilamentCameraPtr camera(new FilamentCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
DepthCameraPtr FilamentScene::CreateDepthCameraImpl(unsigned int _id,
    const std::string &_name)
{
  FilamentDepthCameraPtr camera(new FilamentDepthCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
ThermalCameraPtr FilamentScene::CreateThermalCameraImpl(
    unsigned int _id, const std::string &_name)
{
  FilamentThermalCameraPtr camera(new FilamentThermalCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
SegmentationCameraPtr FilamentScene::CreateSegmentationCameraImpl(
    unsigned int _id, const std::string &_name)
{
  FilamentSegmentationCameraPtr camera(new FilamentSegmentationCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
BoundingBoxCameraPtr FilamentScene::CreateBoundingBoxCameraImpl(
    unsigned int _id, const std::string &_name)
{
  FilamentBoundingBoxCameraPtr camera(new FilamentBoundingBoxCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
GpuRaysPtr FilamentScene::CreateGpuRaysImpl(unsigned int _id,
    const std::string &_name)
{
  FilamentGpuRaysPtr rays(new FilamentGpuRays);
  bool result = this->InitObject(rays, _id, _name);
  return (result) ? rays : nullptr;
}

//////////////////////////////////////////////////
WideAngleCameraPtr FilamentScene::CreateWideAngleCameraImpl(
    unsigned int _id, const std::string &_name)
{
  FilamentWideAngleCameraPtr camera(new FilamentWideAngleCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
VisualPtr FilamentScene::CreateVisualImpl(unsigned int _id,
    const std::string &_name)
{
  FilamentVisualPtr visual(new FilamentVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
ArrowVisualPtr FilamentScene::CreateArrowVisualImpl(unsigned int _id,
    const std::string &_name)
{
  auto object = std::shared_ptr<FilamentArrowVisual>(new FilamentArrowVisual);
  bool result = this->InitObject(object, _id, _name);
  return (result) ? object : nullptr;
}

//////////////////////////////////////////////////
AxisVisualPtr FilamentScene::CreateAxisVisualImpl(unsigned int _id,
    const std::string &_name)
{
  auto object = std::shared_ptr<FilamentAxisVisual>(new FilamentAxisVisual);
  bool result = this->InitObject(object, _id, _name);
  return (result) ? object : nullptr;
}

//////////////////////////////////////////////////
GeometryPtr FilamentScene::CreateBoxImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_box");
}

//////////////////////////////////////////////////
GeometryPtr FilamentScene::CreateConeImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_cone");
}

//////////////////////////////////////////////////
GeometryPtr FilamentScene::CreateCylinderImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_cylinder");
}

//////////////////////////////////////////////////
GeometryPtr FilamentScene::CreatePlaneImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_plane");
}

//////////////////////////////////////////////////
GeometryPtr FilamentScene::CreateSphereImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_sphere");
}

//////////////////////////////////////////////////
MeshPtr FilamentScene::CreateMeshImpl(unsigned int _id,
    const std::string &_name, const std::string &_meshName)
{
  MeshDescriptor descriptor(_meshName);
  return this->CreateMeshImpl(_id, _name, descriptor);
}

//////////////////////////////////////////////////
MeshPtr FilamentScene::CreateMeshImpl(unsigned int _id,
    const std::string &_name, const MeshDescriptor &_desc)
{
  FilamentMeshPtr mesh(new FilamentMesh);
  if (!this->InitObject(mesh, _id, _name))
    return nullptr;
  if (!this->meshFactory->Load(mesh, _desc))
  {
    mesh->Destroy();
    return nullptr;
  }
  return mesh;
}

//////////////////////////////////////////////////
CapsulePtr FilamentScene::CreateCapsuleImpl(unsigned int _id,
    const std::string &_name)
{
  auto capsule = std::shared_ptr<FilamentCapsule>(new FilamentCapsule);
  bool result = this->InitObject(capsule, _id, _name);
  return (result) ? capsule : nullptr;
}

//////////////////////////////////////////////////
GridPtr FilamentScene::CreateGridImpl(unsigned int _id,
    const std::string &_name)
{
  auto object = std::shared_ptr<FilamentGrid>(new FilamentGrid);
  bool result = this->InitObject(object, _id, _name);
  return (result) ? object : nullptr;
}

//////////////////////////////////////////////////
MarkerPtr FilamentScene::CreateMarkerImpl(unsigned int _id,
    const std::string &_name)
{
  auto object = std::shared_ptr<FilamentMarker>(new FilamentMarker);
  bool result = this->InitObject(object, _id, _name);
  return (result) ? object : nullptr;
}

//////////////////////////////////////////////////
LidarVisualPtr FilamentScene::CreateLidarVisualImpl(unsigned int,
    const std::string &)
{
  return nullptr;
}

//////////////////////////////////////////////////
FrustumVisualPtr FilamentScene::CreateFrustumVisualImpl(unsigned int,
    const std::string &)
{
  return nullptr;
}

//////////////////////////////////////////////////
HeightmapPtr FilamentScene::CreateHeightmapImpl(unsigned int _id,
    const std::string &_name, const HeightmapDescriptor &_desc)
{
  FilamentHeightmapPtr heightmap(new FilamentHeightmap(_desc));
  this->InitObject(heightmap, _id, _name);
  if (heightmap->Entity().isNull())
  {
    heightmap->Destroy();
    return nullptr;
  }
  return heightmap;
}

//////////////////////////////////////////////////
WireBoxPtr FilamentScene::CreateWireBoxImpl(unsigned int _id,
    const std::string &_name)
{
  auto object = std::shared_ptr<FilamentWireBox>(new FilamentWireBox);
  bool result = this->InitObject(object, _id, _name);
  return (result) ? object : nullptr;
}

//////////////////////////////////////////////////
std::shared_ptr<FilamentDynamicMesh> FilamentScene::CreateDynamicMesh()
{
  auto mesh = std::make_shared<FilamentDynamicMesh>();
  const unsigned int objId = this->CreateObjectId();
  this->InitObject(mesh, objId, this->CreateObjectName(objId, "DynamicMesh"));
  return mesh;
}

//////////////////////////////////////////////////
MaterialPtr FilamentScene::CreateMaterialImpl(unsigned int _id,
    const std::string &_name)
{
  FilamentMaterialPtr material(new FilamentMaterial);
  bool result = this->InitObject(material, _id, _name);
  return (result) ? material : nullptr;
}

//////////////////////////////////////////////////
RenderTexturePtr FilamentScene::CreateRenderTextureImpl(unsigned int _id,
    const std::string &_name)
{
  FilamentRenderTexturePtr renderTexture(new FilamentRenderTexture);
  bool result = this->InitObject(renderTexture, _id, _name);
  return (result) ? renderTexture : nullptr;
}

//////////////////////////////////////////////////
RenderWindowPtr FilamentScene::CreateRenderWindowImpl(unsigned int,
    const std::string &)
{
  return nullptr;
}

//////////////////////////////////////////////////
RayQueryPtr FilamentScene::CreateRayQueryImpl(unsigned int _id,
    const std::string &_name)
{
  auto rayQuery = std::shared_ptr<FilamentRayQuery>(new FilamentRayQuery);
  bool result = this->InitObject(rayQuery, _id, _name);
  return (result) ? rayQuery : nullptr;
}

//////////////////////////////////////////////////
COMVisualPtr FilamentScene::CreateCOMVisualImpl(unsigned int,
    const std::string &)
{
  return nullptr;
}

//////////////////////////////////////////////////
InertiaVisualPtr FilamentScene::CreateInertiaVisualImpl(unsigned int,
    const std::string &)
{
  return nullptr;
}

//////////////////////////////////////////////////
JointVisualPtr FilamentScene::CreateJointVisualImpl(unsigned int,
    const std::string &)
{
  return nullptr;
}

//////////////////////////////////////////////////
LightVisualPtr FilamentScene::CreateLightVisualImpl(unsigned int,
    const std::string &)
{
  return nullptr;
}

//////////////////////////////////////////////////
LightStorePtr FilamentScene::Lights() const
{
  return this->lights;
}

//////////////////////////////////////////////////
SensorStorePtr FilamentScene::Sensors() const
{
  return this->sensors;
}

//////////////////////////////////////////////////
VisualStorePtr FilamentScene::Visuals() const
{
  return this->visuals;
}

//////////////////////////////////////////////////
MaterialMapPtr FilamentScene::Materials() const
{
  return this->materials;
}

//////////////////////////////////////////////////
bool FilamentScene::LoadImpl()
{
  return true;
}

//////////////////////////////////////////////////
bool FilamentScene::InitImpl()
{
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  if (!engine)
  {
    gzerr << "Filament engine not initialized" << std::endl;
    return false;
  }
  this->scene = engine->createScene();
  this->CreateStores();
  this->CreateRootVisual();
  this->meshFactory = FilamentMeshFactoryPtr(
      new FilamentMeshFactory(this->SharedThis()));
  return true;
}

//////////////////////////////////////////////////
bool FilamentScene::InitObject(FilamentObjectPtr _object, unsigned int _id,
    const std::string &_name)
{
  _object->id = _id;
  _object->name = _name;
  _object->scene = this->SharedThis();
  _object->Load();
  _object->Init();
  return true;
}

//////////////////////////////////////////////////
void FilamentScene::CreateRootVisual()
{
  if (this->rootVisual)
    return;
  this->rootVisual = FilamentVisualPtr(new FilamentVisual);
  unsigned int rootId = this->CreateObjectId();
  std::string rootName = this->CreateObjectName(rootId, "_ROOT_");
  if (!this->InitObject(this->rootVisual, rootId, rootName))
  {
    gzerr << "Unable to create root visual" << std::endl;
    this->rootVisual = nullptr;
  }
}

//////////////////////////////////////////////////
void FilamentScene::CreateStores()
{
  this->lights = FilamentLightStorePtr(new FilamentLightStore);
  this->sensors = FilamentSensorStorePtr(new FilamentSensorStore);
  this->visuals = FilamentVisualStorePtr(new FilamentVisualStore);
  this->materials = FilamentMaterialMapPtr(new FilamentMaterialMap);
}

//////////////////////////////////////////////////
FilamentScenePtr FilamentScene::SharedThis()
{
  ScenePtr sharedBase = this->shared_from_this();
  return std::dynamic_pointer_cast<FilamentScene>(sharedBase);
}
