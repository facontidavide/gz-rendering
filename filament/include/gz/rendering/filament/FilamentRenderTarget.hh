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
#ifndef GZ_RENDERING_FILAMENT_FILAMENTRENDERTARGET_HH_
#define GZ_RENDERING_FILAMENT_FILAMENTRENDERTARGET_HH_

#include <vector>

#include <math/vec4.h>

#include "gz/rendering/config.hh"
#include "gz/rendering/base/BaseRenderTarget.hh"
#include "gz/rendering/filament/FilamentObject.hh"
#include "gz/rendering/filament/FilamentRenderTypes.hh"
#include "gz/rendering/filament/Export.hh"

namespace filament
{
  class RenderTarget;
  class Texture;
  class View;
  class Viewport;
}

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Offscreen color+depth attachment pair. Rendering goes through
    /// Renderer::renderStandaloneView, readback through readPixels.
    class GZ_RENDERING_FILAMENT_VISIBLE FilamentRenderTarget :
      public virtual BaseRenderTarget<FilamentObject>
    {
      protected: FilamentRenderTarget();

      public: virtual ~FilamentRenderTarget();

      public: virtual void Destroy() override;

      public: virtual void Copy(Image &_image) const override;

      /// \brief Render _view into this target (synchronously submits)
      /// \param[in] _viewport Region to draw into; nullptr: whole target
      /// \param[in] _clear Clear the color attachment first (the whole
      /// attachment, on every backend); false keeps what is there
      public: void Render(filament::View *_view,
          const filament::Viewport *_viewport = nullptr, bool _clear = true);

      /// \brief Raw clear color written to the attachment before rendering
      public: void SetClearColor(const filament::math::float4 &_color);

      public: filament::RenderTarget *Native() const;

      /// \brief Color attachment, e.g. to sample it in a later pass
      public: filament::Texture *ColorTexture() const;

      /// \brief Run the enabled filament passes of _passes (Gaussian noise)
      /// over the color attachment, in order. Afterwards Native() and
      /// ColorTexture() name the attachments holding the result.
      public: void ApplyRenderPasses(const std::vector<RenderPassPtr> &_passes);

      /// \brief Passes added with AddRenderPass
      public: const std::vector<RenderPassPtr> &RenderPasses() const;

      /// \brief 8 bit formats are sRGB encoded by default (camera images);
      /// false stores values as is (e.g. segmentation labels)
      public: void SetSrgb(bool _srgb);

      protected: virtual void RebuildImpl() override;

      protected: void DestroyTarget();

      /// \brief Create a color + depth attachment pair for the current
      /// size and format
      private: void BuildAttachments(filament::RenderTarget *&_target,
          filament::Texture *&_color, filament::Texture *&_depth) const;

      protected: filament::Texture *colorTexture{nullptr};

      protected: filament::Texture *depthTexture{nullptr};

      protected: filament::RenderTarget *target{nullptr};

      /// \brief Second attachment set for ApplyRenderPasses, built on
      /// first use
      private: filament::RenderTarget *scratchTarget{nullptr};

      private: filament::Texture *scratchColor{nullptr};

      private: filament::Texture *scratchDepth{nullptr};

      protected: filament::math::float4 clearColor{0.0f, 0.0f, 0.0f, 1.0f};

      protected: bool srgb{true};

      /// \brief Readback staging buffer of Copy(), kept across frames
      private: mutable std::vector<unsigned char> readBuffer;
    };

    class GZ_RENDERING_FILAMENT_VISIBLE FilamentRenderTexture :
      public virtual BaseRenderTexture<FilamentRenderTarget>
    {
      protected: FilamentRenderTexture();

      public: virtual ~FilamentRenderTexture();

      private: friend class FilamentScene;
    };
    }
  }
}
#endif
