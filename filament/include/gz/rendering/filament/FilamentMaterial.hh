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
#ifndef GZ_RENDERING_FILAMENT_FILAMENTMATERIAL_HH_
#define GZ_RENDERING_FILAMENT_FILAMENTMATERIAL_HH_

#include <array>
#include <string>

#include "gz/rendering/config.hh"
#include "gz/rendering/base/BaseMaterial.hh"
#include "gz/rendering/filament/FilamentObject.hh"
#include "gz/rendering/filament/FilamentRenderTypes.hh"
#include "gz/rendering/filament/Export.hh"

namespace filament
{
  class MaterialInstance;
  class Texture;
}

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief A gz material is one instance each of the gz_lit and gz_unlit
    /// filament materials; parameters are pushed in PreRender.
    class GZ_RENDERING_FILAMENT_VISIBLE FilamentMaterial :
      public BaseMaterial<FilamentObject>
    {
      protected: FilamentMaterial();

      public: virtual ~FilamentMaterial();

      public: virtual void Destroy() override;

      public: virtual void PreRender() override;

      public: virtual bool HasTexture() const override;

      public: virtual std::string Texture() const override;

      public: virtual void SetTexture(const std::string &_texture) override;

      public: virtual void SetTexture(const std::string &_texture,
          const std::shared_ptr<const common::Image> &_img) override;

      public: virtual std::shared_ptr<const common::Image> TextureData()
          const override;

      public: virtual void ClearTexture() override;

      public: virtual void SetRoughness(const float _roughness) override;

      public: virtual float Roughness() const override;

      public: virtual void SetMetalness(const float _metalness) override;

      public: virtual float Metalness() const override;

      /// \brief Active material instance (lit or unlit)
      public: filament::MaterialInstance *Instance() const;

      /// \brief Diffuse texture on the GPU, nullptr if none
      public: filament::Texture *FilamentTexture() const;

      /// \brief Max anisotropy of the texture sampler: 1 like ogre2's
      /// materials, 8 on heightmaps like Terra
      public: float anisotropy{1.0f};

      protected: virtual void Init() override;

      protected: void Update(filament::MaterialInstance *_instance);

      protected: filament::MaterialInstance *lit{nullptr};

      protected: filament::MaterialInstance *unlit{nullptr};

      protected: filament::Texture *texture{nullptr};

      protected: std::string loadedTexture;

      protected: std::string textureName;

      /// \brief Last values pushed to the instances, to skip redundant
      /// uniform uploads
      protected: bool pushed{false};

      protected: std::array<float, 11> pushedKey{};

      protected: filament::Texture *pushedTexture{nullptr};

      protected: std::shared_ptr<const common::Image> textureData;

      /// \brief gz::common::Pbr defaults, like ogre2 after Reset()
      protected: float roughness{0.5f};

      protected: float metalness{0.0f};

      private: friend class FilamentScene;
    };
    }
  }
}
#endif
