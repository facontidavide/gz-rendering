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
#ifndef GZ_RENDERING_FILAMENT_FILAMENTMESH_HH_
#define GZ_RENDERING_FILAMENT_FILAMENTMESH_HH_

#include <utils/Entity.h>

#include "gz/rendering/config.hh"
#include "gz/rendering/base/BaseMesh.hh"
#include "gz/rendering/filament/FilamentGeometry.hh"
#include "gz/rendering/filament/FilamentObject.hh"
#include "gz/rendering/filament/FilamentRenderTypes.hh"
#include "gz/rendering/filament/Export.hh"

namespace filament
{
  class IndexBuffer;
  class MaterialInstance;
  class VertexBuffer;
}

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief A mesh is one filament renderable entity; each submesh is one
    /// primitive of that renderable.
    class GZ_RENDERING_FILAMENT_VISIBLE FilamentMesh :
      public BaseMesh<FilamentGeometry>
    {
      protected: FilamentMesh();

      public: virtual ~FilamentMesh();

      public: virtual void Destroy() override;

      public: virtual ::utils::Entity Entity() const override;

      public: virtual void SetLayers(uint8_t _layers) override;

      public: virtual void PreRender() override;

      /// \brief Temporarily render every submesh with _instance
      public: void OverrideMaterial(filament::MaterialInstance *_instance);

      /// \brief Undo OverrideMaterial
      public: void RestoreMaterials();

      /// \brief Register a submesh built by the mesh factory
      public: void AddSubMesh(FilamentSubMeshPtr _subMesh);

      protected: virtual SubMeshStorePtr SubMeshes() const override;

      protected: virtual void Init() override;

      protected: ::utils::Entity entity;

      protected: FilamentSubMeshStorePtr subMeshes;

      private: friend class FilamentScene;
      private: friend class FilamentMeshFactory;
      private: friend class FilamentSubMesh;
    };

    class GZ_RENDERING_FILAMENT_VISIBLE FilamentSubMesh :
      public BaseSubMesh<FilamentObject>
    {
      protected: FilamentSubMesh();

      public: virtual ~FilamentSubMesh();

      public: virtual void Destroy() override;

      public: virtual void PreRender() override;

      protected: virtual void SetMaterialImpl(MaterialPtr _material) override;

      /// \brief Bind _instance as the primitive's material instance
      protected: void ApplyInstance(filament::MaterialInstance *_instance);

      protected: FilamentMesh *mesh{nullptr};

      protected: size_t index{0u};

      protected: filament::VertexBuffer *vertexBuffer{nullptr};

      protected: filament::IndexBuffer *indexBuffer{nullptr};

      /// \brief Instance currently bound on the GPU side
      protected: filament::MaterialInstance *bound{nullptr};

      private: friend class FilamentScene;
      private: friend class FilamentMesh;
      private: friend class FilamentMeshFactory;
    };
    }
  }
}
#endif
