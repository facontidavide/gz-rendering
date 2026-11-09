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
#ifndef GZ_RENDERING_FILAMENT_FILAMENTCAMERA_HH_
#define GZ_RENDERING_FILAMENT_FILAMENTCAMERA_HH_

#include <utils/Entity.h>

#include "gz/rendering/config.hh"
#include "gz/rendering/base/BaseCamera.hh"
#include "gz/rendering/filament/FilamentRenderTarget.hh"
#include "gz/rendering/filament/FilamentRenderTypes.hh"
#include "gz/rendering/filament/FilamentSensor.hh"
#include "gz/rendering/filament/Export.hh"

namespace filament
{
  class Camera;
  class View;
}

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    //
    class GZ_RENDERING_FILAMENT_VISIBLE FilamentCamera :
      public BaseCamera<FilamentSensor>
    {
      protected: FilamentCamera();

      public: virtual ~FilamentCamera();

      public: virtual void Render() override;

      public: virtual void Destroy() override;

      public: filament::View *View() const;

      /// \brief Picking through filament::View::pick (renders one frame)
      public: virtual VisualPtr VisualAt(const math::Vector2i &_mousePos)
          override;

      public: filament::Camera *Native() const;

      protected: virtual RenderTargetPtr RenderTarget() const override;

      protected: virtual void Init() override;

      protected: virtual void CreateRenderTexture();

      /// \brief Render with [_renderNear, _renderFar] but cull against the
      /// camera's own clip planes. With a closer near plane this emulates
      /// ogre2's depth clamp: geometry crossing the near plane is drawn,
      /// objects entirely in front of it are culled.
      protected: void SetClampedProjection(double _renderNear,
                     double _renderFar);

      /// \brief Push HFOV / aspect / near / far into the filament camera
      protected: void UpdateProjection();

      protected: filament::View *view{nullptr};

      protected: filament::Camera *camera{nullptr};

      /// \brief Child of the node entity, rotated into filament's camera
      /// convention (-Z forward, Y up)
      protected: ::utils::Entity cameraEntity;

      protected: FilamentRenderTexturePtr renderTexture;

      private: friend class FilamentScene;
    };
    }
  }
}
#endif
