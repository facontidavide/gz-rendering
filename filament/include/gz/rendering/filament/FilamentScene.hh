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
#ifndef GZ_RENDERING_FILAMENT_FILAMENTSCENE_HH_
#define GZ_RENDERING_FILAMENT_FILAMENTSCENE_HH_

#include <set>
#include <string>

#include "gz/rendering/config.hh"
#include "gz/rendering/base/BaseScene.hh"
#include "gz/rendering/filament/FilamentRenderTypes.hh"
#include "gz/rendering/filament/Export.hh"

namespace filament
{
  class IndirectLight;
  class Scene;
}

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    //
    class GZ_RENDERING_FILAMENT_VISIBLE FilamentScene :
      public BaseScene
    {
      protected: FilamentScene(unsigned int _id, const std::string &_name);

      public: virtual ~FilamentScene();

      public: virtual void Fini() override;

      public: virtual void Destroy() override;

      public: virtual RenderEngine *Engine() const override;

      public: virtual VisualPtr RootVisual() const override;

      public: virtual math::Color AmbientLight() const override;

      /// \brief GPU picking through the camera.
      /// ponytail: no RayQuery yet, so picking is camera-only
      public: virtual VisualPtr VisualAt(const CameraPtr &_camera,
          const math::Vector2i &_mousePos) override;

      public: virtual void SetAmbientLight(const math::Color &_color) override;

      /// \brief Native filament scene
      public: filament::Scene *Native() const;

      /// \brief Meshes currently alive in this scene; used by sensors that
      /// need to render every mesh with an override material.
      public: const std::set<FilamentMesh *> &Meshes() const;

      public: void RegisterMesh(FilamentMesh *_mesh);

      public: void UnregisterMesh(FilamentMesh *_mesh);

      protected: virtual DirectionalLightPtr CreateDirectionalLightImpl(
                     unsigned int _id, const std::string &_name) override;

      protected: virtual PointLightPtr CreatePointLightImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual SpotLightPtr CreateSpotLightImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual CameraPtr CreateCameraImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual DepthCameraPtr CreateDepthCameraImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual ThermalCameraPtr CreateThermalCameraImpl(
                     unsigned int _id, const std::string &_name) override;

      protected: virtual SegmentationCameraPtr CreateSegmentationCameraImpl(
                     unsigned int _id, const std::string &_name) override;

      protected: virtual GpuRaysPtr CreateGpuRaysImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual WideAngleCameraPtr CreateWideAngleCameraImpl(
                     unsigned int _id, const std::string &_name) override;

      protected: virtual VisualPtr CreateVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual ArrowVisualPtr CreateArrowVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual AxisVisualPtr CreateAxisVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual GeometryPtr CreateBoxImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual GeometryPtr CreateConeImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual GeometryPtr CreateCylinderImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual GeometryPtr CreatePlaneImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual GeometryPtr CreateSphereImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual MeshPtr CreateMeshImpl(unsigned int _id,
                     const std::string &_name,
                     const std::string &_meshName);

      protected: virtual MeshPtr CreateMeshImpl(unsigned int _id,
                     const std::string &_name,
                     const MeshDescriptor &_desc) override;

      protected: virtual CapsulePtr CreateCapsuleImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual GridPtr CreateGridImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual MarkerPtr CreateMarkerImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual LidarVisualPtr CreateLidarVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual FrustumVisualPtr CreateFrustumVisualImpl(
                     unsigned int _id, const std::string &_name) override;

      protected: virtual HeightmapPtr CreateHeightmapImpl(unsigned int _id,
                     const std::string &_name,
                     const HeightmapDescriptor &_desc) override;

      protected: virtual WireBoxPtr CreateWireBoxImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual MaterialPtr CreateMaterialImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual RenderTexturePtr CreateRenderTextureImpl(
                     unsigned int _id, const std::string &_name) override;

      protected: virtual RenderWindowPtr CreateRenderWindowImpl(
                     unsigned int _id, const std::string &_name) override;

      protected: virtual RayQueryPtr CreateRayQueryImpl(
                     unsigned int _id, const std::string &_name) override;

      protected: virtual COMVisualPtr CreateCOMVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual InertiaVisualPtr CreateInertiaVisualImpl(
                     unsigned int _id, const std::string &_name) override;

      protected: virtual JointVisualPtr CreateJointVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual LightVisualPtr CreateLightVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual LightStorePtr Lights() const override;

      protected: virtual SensorStorePtr Sensors() const override;

      protected: virtual VisualStorePtr Visuals() const override;

      protected: virtual MaterialMapPtr Materials() const override;

      protected: virtual bool LoadImpl() override;

      protected: virtual bool InitImpl() override;

      protected: bool InitObject(FilamentObjectPtr _object, unsigned int _id,
                     const std::string &_name);

      protected: void CreateRootVisual();

      protected: void CreateStores();

      private: FilamentScenePtr SharedThis();

      protected: filament::Scene *scene{nullptr};

      protected: filament::IndirectLight *ambientLight{nullptr};

      protected: math::Color ambient;

      protected: FilamentVisualPtr rootVisual;

      protected: FilamentMeshFactoryPtr meshFactory;

      protected: FilamentMaterialMapPtr materials;

      protected: FilamentSensorStorePtr sensors;

      protected: FilamentVisualStorePtr visuals;

      protected: FilamentLightStorePtr lights;

      protected: std::set<FilamentMesh *> meshes;

      private: friend class FilamentRenderEngine;
      private: friend class FilamentMesh;
    };
    }
  }
}
#endif
