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

#include <cstdlib>

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <utils/EntityManager.h>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Matrix4.hh>

#include "gz/rendering/filament/FilamentCamera.hh"
#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentMesh.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentScene.hh"

using namespace gz;
using namespace rendering;

namespace
{
  /// \brief gz matrices are row-major accessors, filament mat4 is indexed
  /// [column][row]
  filament::math::mat4 ToFilament(const math::Matrix4d &_m)
  {
    filament::math::mat4 out;
    for (int c = 0; c < 4; ++c)
      for (int r = 0; r < 4; ++r)
        out[c][r] = _m(r, c);
    return out;
  }
}

//////////////////////////////////////////////////
FilamentCamera::FilamentCamera()
{
}

//////////////////////////////////////////////////
FilamentCamera::~FilamentCamera()
{
}

//////////////////////////////////////////////////
void FilamentCamera::Destroy()
{
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  if (engine)
  {
    if (this->view)
      engine->destroy(this->view);
    if (!this->cameraEntity.isNull())
    {
      engine->destroyCameraComponent(this->cameraEntity);
      engine->getTransformManager().destroy(this->cameraEntity);
      ::utils::EntityManager::get().destroy(this->cameraEntity);
    }
  }
  this->view = nullptr;
  this->camera = nullptr;
  this->cameraEntity = ::utils::Entity();
  if (this->renderTexture)
  {
    this->renderTexture->Destroy();
    this->renderTexture.reset();
  }
  BaseCamera::Destroy();
}

//////////////////////////////////////////////////
filament::View *FilamentCamera::View() const
{
  return this->view;
}

//////////////////////////////////////////////////
filament::Camera *FilamentCamera::Native() const
{
  return this->camera;
}

//////////////////////////////////////////////////
RenderTargetPtr FilamentCamera::RenderTarget() const
{
  return this->renderTexture;
}

//////////////////////////////////////////////////
void FilamentCamera::Init()
{
  BaseCamera::Init();
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  auto &tm = engine->getTransformManager();

  this->cameraEntity = ::utils::EntityManager::get().create();
  tm.create(this->cameraEntity, tm.getInstance(this->entity),
      FilamentConversions::CameraFix());
  this->camera = engine->createCamera(this->cameraEntity);
  // exposure 1: pixel = radiance, no photographic scaling
  this->camera->setExposure(1.0f);

  this->view = engine->createView();
  this->view->setScene(this->scene->Native());
  this->view->setCamera(this->camera);
  this->view->setVisibleLayers(0xff, 0x1);
  // Sensors want plain radiance: no tone mapping, AA, dithering, bloom...
  this->view->setPostProcessingEnabled(false);
  // ponytail: FIL_NOSHADOW=1 is a spike benchmarking knob, drop before merge
  const char *noShadow = std::getenv("FIL_NOSHADOW");
  this->view->setShadowingEnabled(!(noShadow && *noShadow));

  this->CreateRenderTexture();
  this->Reset();
}

//////////////////////////////////////////////////
VisualPtr FilamentCamera::VisualAt(const math::Vector2i &_mousePos)
{
  if (!this->view || _mousePos.X() < 0 || _mousePos.Y() < 0 ||
      static_cast<unsigned int>(_mousePos.X()) >= this->ImageWidth() ||
      static_cast<unsigned int>(_mousePos.Y()) >= this->ImageHeight())
  {
    return VisualPtr();
  }

  // The query is answered by the next render of this view. Its callback can
  // fire after this function returns if the driver is slow, so the result
  // lives in a shared state rather than on the stack.
  struct Result { ::utils::Entity entity; bool done{false}; };
  auto result = std::make_shared<Result>();
  // filament picking coordinates have their origin at the bottom-left
  this->view->pick(static_cast<uint32_t>(_mousePos.X()),
      this->ImageHeight() - 1u - static_cast<uint32_t>(_mousePos.Y()),
      [result](const filament::View::PickingQueryResult &_r)
      {
        result->entity = _r.renderable;
        result->done = true;
      });
  this->PreRender();
  this->Render();
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  for (int i = 0; i < 10 && !result->done; ++i)
  {
    engine->flushAndWait();
    engine->pumpMessageQueues();
  }
  if (!result->done || result->entity.isNull())
    return VisualPtr();

  for (FilamentMesh *mesh : this->scene->Meshes())
  {
    if (mesh->Entity() == result->entity)
      return mesh->Parent();
  }
  return VisualPtr();
}

//////////////////////////////////////////////////
void FilamentCamera::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->renderTexture = std::dynamic_pointer_cast<FilamentRenderTexture>(base);
  this->renderTexture->SetFormat(PF_R8G8B8);
  this->renderTexture->SetWidth(this->ImageWidth());
  this->renderTexture->SetHeight(this->ImageHeight());
}

//////////////////////////////////////////////////
bool FilamentCamera::Orthographic() const
{
  // Sensor subclasses never get here with CPT_ORTHOGRAPHIC: their
  // SetProjectionType resolves to BaseCamera<FilamentCamera>, which only
  // stores the type, like ogre2 (only Ogre2Camera honours it)
  return this->projectionType == CPT_ORTHOGRAPHIC;
}

//////////////////////////////////////////////////
math::Matrix4d FilamentCamera::WithDepthRows(math::Matrix4d _matrix,
    double _near, double _far)
{
  const double invd = 1.0 / (_far - _near);
  _matrix(2, 0) = 0.0;
  _matrix(2, 1) = 0.0;
  if (!math::equal(_matrix(3, 3), 0.0))
  {
    // orthographic
    _matrix(2, 2) = -2.0 * invd;
    _matrix(2, 3) = -(_far + _near) * invd;
    return _matrix;
  }
  _matrix(2, 2) = -(_far + _near) * invd;
  _matrix(2, 3) = -2.0 * _far * _near * invd;
  _matrix(3, 0) = 0.0;
  _matrix(3, 1) = 0.0;
  _matrix(3, 2) = -1.0;
  return _matrix;
}

//////////////////////////////////////////////////
math::Matrix4d FilamentCamera::ProjectionMatrix() const
{
  if (this->customProjection)
    return this->projectionMatrix;
  if (this->Orthographic())
  {
    // ogre2 never sets an ortho window: ogre's default is 1000 units tall
    const double height = 1000.0;
    math::Matrix4d m = math::Matrix4d::Zero;
    m(0, 0) = 2.0 / (height * this->AspectRatio());
    m(1, 1) = 2.0 / height;
    m(3, 3) = 1.0;
    return WithDepthRows(m, this->NearClipPlane(), this->FarClipPlane());
  }
  return BaseCamera::ProjectionMatrix();
}

//////////////////////////////////////////////////
void FilamentCamera::SetProjectionMatrix(const math::Matrix4d &_matrix)
{
  BaseCamera::SetProjectionMatrix(_matrix);
  this->customProjection = true;
}

//////////////////////////////////////////////////
void FilamentCamera::SetProjectionType(CameraProjectionType _type)
{
  BaseCamera::SetProjectionType(_type);
  // like Ogre2Camera: a type change drops the custom matrix
  this->customProjection = false;
}

//////////////////////////////////////////////////
void FilamentCamera::SetClampedProjection(double _renderNear,
    double _renderFar)
{
  if (this->customProjection || this->Orthographic())
  {
    // image plane rows as given, depth rows from the clip planes: render
    // with the render ones, cull with the camera's own (ogre2 rebuilds the
    // depth row from its clip distances as well)
    const math::Matrix4d m = this->ProjectionMatrix();
    this->camera->setCustomProjection(
        ToFilament(WithDepthRows(m, _renderNear, _renderFar)),
        ToFilament(WithDepthRows(m, this->NearClipPlane(),
            this->FarClipPlane())),
        this->NearClipPlane(), this->FarClipPlane());
    return;
  }
  using filament::Camera;
  const double hfovDeg = this->HFOV().Degree();
  const double aspect = this->AspectRatio();
  this->camera->setCustomProjection(
      Camera::projection(Camera::Fov::HORIZONTAL, hfovDeg, aspect,
          _renderNear, _renderFar),
      Camera::projection(Camera::Fov::HORIZONTAL, hfovDeg, aspect,
          this->NearClipPlane(), this->FarClipPlane()),
      this->NearClipPlane(), this->FarClipPlane());
}

//////////////////////////////////////////////////
void FilamentCamera::UpdateProjection()
{
  if (!this->camera)
    return;
  if (this->customProjection || this->Orthographic())
  {
    this->SetClampedProjection(this->NearClipPlane(), this->FarClipPlane());
    return;
  }
  const double aspect = this->AspectRatio();
  const double hfovDeg = this->HFOV().Degree();
  this->camera->setProjection(hfovDeg, aspect, this->NearClipPlane(),
      this->FarClipPlane(), filament::Camera::Fov::HORIZONTAL);
}

//////////////////////////////////////////////////
void FilamentCamera::Render()
{
  this->UpdateProjection();
  const math::Color bg = this->scene->BackgroundColor();
  this->renderTexture->SetClearColor(FilamentConversions::ToFloat4(bg));
  this->view->setVisibleLayers(0xff,
      FilamentConversions::MaskToLayers(this->VisibilityMask()));
  this->renderTexture->Render(this->view);
  if (!this->scene->Projectors().empty())
  {
    // projected textures go on the scene, before the render passes
    const math::Matrix4d inverse =
        (this->ProjectionMatrix() * this->ViewMatrix()).Inverse();
    filament::math::mat4f invViewProj;
    for (int c = 0; c < 4; ++c)
    {
      for (int r = 0; r < 4; ++r)
        invViewProj[c][r] = static_cast<float>(inverse(r, c));
    }
    this->renderTexture->ApplyProjectors(this->scene->Projectors(),
        invViewProj, this->VisibilityMask());
  }
  this->renderTexture->ApplyRenderPasses(this->renderTexture->RenderPasses());
}
