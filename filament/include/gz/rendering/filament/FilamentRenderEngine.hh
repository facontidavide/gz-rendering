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
#ifndef GZ_RENDERING_FILAMENT_FILAMENTRENDERENGINE_HH_
#define GZ_RENDERING_FILAMENT_FILAMENTRENDERENGINE_HH_

#include <map>
#include <memory>
#include <string>

#include <gz/common/SingletonT.hh>

#include "gz/rendering/RenderEnginePlugin.hh"
#include "gz/rendering/base/BaseRenderEngine.hh"
#include "gz/rendering/filament/FilamentRenderTypes.hh"
#include "gz/rendering/filament/Export.hh"

namespace filament
{
  class Engine;
  class Material;
  class Renderer;
  class Texture;
}

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    //
    class FilamentRenderEnginePrivate;

    /// \brief Plugin for loading the filament render engine
    class GZ_RENDERING_FILAMENT_VISIBLE FilamentRenderEnginePlugin :
      public RenderEnginePlugin
    {
      public: FilamentRenderEnginePlugin();

      public: ~FilamentRenderEnginePlugin() = default;

      public: std::string Name() const override;

      public: RenderEngine *Engine() const override;
    };

    /// \brief Render engine backed by Google Filament.
    /// Experimental: supports visuals, meshes, lights, cameras and depth
    /// cameras rendered offscreen (Vulkan by default, OpenGL/EGL optional).
    class GZ_RENDERING_FILAMENT_VISIBLE FilamentRenderEngine :
      public virtual BaseRenderEngine,
      public common::SingletonT<FilamentRenderEngine>
    {
      private: FilamentRenderEngine();

      public: virtual ~FilamentRenderEngine();

      public: virtual void Destroy() override;

      public: virtual bool IsEnabled() const override;

      public: virtual std::string Name() const override;

      public: virtual rendering::GraphicsAPI GraphicsAPI() const override;

      /// \brief Native filament engine
      public: filament::Engine *FilamentEngine() const;

      /// \brief Shared filament renderer
      public: filament::Renderer *Renderer() const;

      /// \brief Get (and lazily load) a compiled material package by name,
      /// e.g. "gz_lit", looked up under the engine resource path.
      public: filament::Material *Material(const std::string &_name);

      /// \brief Get (and lazily load, cached) a 2D texture from an image file
      public: filament::Texture *Texture(const std::string &_path);

      /// \brief Get the resource path used to find compiled materials
      public: std::string ResourcePath() const;

      protected: virtual bool LoadImpl(
          const std::map<std::string, std::string> &_params) override;

      protected: virtual bool InitImpl() override;

      protected: virtual SceneStorePtr Scenes() const override;

      protected: virtual ScenePtr CreateSceneImpl(unsigned int _id,
                     const std::string &_name) override;

      /// \brief Singleton setup
      private: friend class common::SingletonT<FilamentRenderEngine>;

      private: std::unique_ptr<FilamentRenderEnginePrivate> dataPtr;
    };
    }
  }
}
#endif
