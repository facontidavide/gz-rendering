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
#include <fstream>
#include <iterator>
#include <vector>

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/Renderer.h>
#include <filament/Texture.h>
#include <backend/PixelBufferDescriptor.h>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Image.hh>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>

#include "gz/rendering/GraphicsAPI.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentScene.hh"

using namespace gz;
using namespace rendering;

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    class FilamentRenderEnginePrivate
    {
      public: filament::Engine *engine{nullptr};
      public: filament::Renderer *renderer{nullptr};
      public: FilamentSceneStorePtr scenes;
      public: std::map<std::string, filament::Material *> materials;
      public: std::map<std::string, filament::Texture *> textures;
      public: std::string backend{"opengl"};
      public: std::string resourcePath;
    };
    }
  }
}

//////////////////////////////////////////////////
FilamentRenderEnginePlugin::FilamentRenderEnginePlugin()
{
}

//////////////////////////////////////////////////
std::string FilamentRenderEnginePlugin::Name() const
{
  return FilamentRenderEngine::Instance()->Name();
}

//////////////////////////////////////////////////
RenderEngine *FilamentRenderEnginePlugin::Engine() const
{
  return FilamentRenderEngine::Instance();
}

//////////////////////////////////////////////////
FilamentRenderEngine::FilamentRenderEngine()
  : dataPtr(new FilamentRenderEnginePrivate)
{
#ifdef FILAMENT_RESOURCE_PATH
  this->dataPtr->resourcePath = FILAMENT_RESOURCE_PATH;
#endif
  std::string envPath;
  if (common::env("GZ_RENDERING_FILAMENT_RESOURCE_PATH", envPath) &&
      !envPath.empty())
  {
    this->dataPtr->resourcePath = envPath;
  }
}

//////////////////////////////////////////////////
FilamentRenderEngine::~FilamentRenderEngine()
{
}

//////////////////////////////////////////////////
void FilamentRenderEngine::Destroy()
{
  BaseRenderEngine::Destroy();

  if (this->dataPtr->scenes)
  {
    this->dataPtr->scenes->RemoveAll();
    this->dataPtr->scenes.reset();
  }

  if (this->dataPtr->engine)
  {
    for (auto &it : this->dataPtr->textures)
      this->dataPtr->engine->destroy(it.second);
    this->dataPtr->textures.clear();
    for (auto &it : this->dataPtr->materials)
      this->dataPtr->engine->destroy(it.second);
    this->dataPtr->materials.clear();
    if (this->dataPtr->renderer)
      this->dataPtr->engine->destroy(this->dataPtr->renderer);
    this->dataPtr->renderer = nullptr;
    filament::Engine::destroy(&this->dataPtr->engine);
    this->dataPtr->engine = nullptr;
  }
}

//////////////////////////////////////////////////
bool FilamentRenderEngine::IsEnabled() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
std::string FilamentRenderEngine::Name() const
{
  return "filament";
}

//////////////////////////////////////////////////
rendering::GraphicsAPI FilamentRenderEngine::GraphicsAPI() const
{
  return this->dataPtr->backend == "opengl" ? rendering::GraphicsAPI::OPENGL
                                            : rendering::GraphicsAPI::VULKAN;
}

//////////////////////////////////////////////////
filament::Engine *FilamentRenderEngine::FilamentEngine() const
{
  return this->dataPtr->engine;
}

//////////////////////////////////////////////////
filament::Renderer *FilamentRenderEngine::Renderer() const
{
  return this->dataPtr->renderer;
}

//////////////////////////////////////////////////
std::string FilamentRenderEngine::ResourcePath() const
{
  return this->dataPtr->resourcePath;
}

//////////////////////////////////////////////////
filament::Material *FilamentRenderEngine::Material(const std::string &_name)
{
  auto it = this->dataPtr->materials.find(_name);
  if (it != this->dataPtr->materials.end())
    return it->second;

  std::string path = common::joinPaths(this->dataPtr->resourcePath,
      "materials", _name + ".filamat");
  std::ifstream in(path, std::ios::binary);
  if (!in)
  {
    gzerr << "Unable to open filament material package [" << path << "]"
          << std::endl;
    return nullptr;
  }
  std::vector<char> data((std::istreambuf_iterator<char>(in)),
                          std::istreambuf_iterator<char>());
  filament::Material *mat = filament::Material::Builder()
      .package(data.data(), data.size())
      .build(*this->dataPtr->engine);
  if (!mat)
  {
    gzerr << "Failed to build filament material [" << _name << "]"
          << std::endl;
    return nullptr;
  }
  this->dataPtr->materials[_name] = mat;
  return mat;
}

//////////////////////////////////////////////////
filament::Texture *FilamentRenderEngine::Texture(const std::string &_path,
    std::shared_ptr<const common::Image> _data)
{
  auto it = this->dataPtr->textures.find(_path);
  if (it != this->dataPtr->textures.end())
    return it->second;

  // Empty path: 1x1 white texture, bound when a material has no texture so
  // every declared sampler is set.
  common::Image img;
  if (_path.empty())
  {
    const unsigned char white[4] = {255, 255, 255, 255};
    img.SetFromData(white, 1, 1, common::Image::RGBA_INT8);
  }
  else if (!_data)
  {
    img.Load(_path);
  }
  // common::Image is not safely copyable: read in-memory data in place
  const common::Image &src = _data ? *_data : img;
  if (!src.Valid())
  {
    gzerr << "Unable to load texture image [" << _path << "]" << std::endl;
    return nullptr;
  }

  const unsigned int w = src.Width();
  const unsigned int h = src.Height();
  std::vector<unsigned char> rgba = src.RGBAData();
  if (rgba.size() != static_cast<size_t>(w) * h * 4u)
  {
    gzerr << "Unexpected texture data size for [" << _path << "]"
          << std::endl;
    return nullptr;
  }

  // filament textures have their origin at the bottom-left; image rows are
  // stored top-down. Flip so UVs can be used as authored.
  auto *buf = new unsigned char[rgba.size()];
  const size_t rowBytes = static_cast<size_t>(w) * 4u;
  for (unsigned int y = 0; y < h; ++y)
  {
    std::memcpy(buf + static_cast<size_t>(y) * rowBytes,
                rgba.data() + static_cast<size_t>(h - 1 - y) * rowBytes,
                rowBytes);
  }

  // full mip chain, like ogre2 (it auto-generates mipmaps for albedo
  // maps): minified textures average instead of aliasing
  auto &engine = *this->dataPtr->engine;
  const auto format = filament::Texture::InternalFormat::SRGB8_A8;
  const bool mips = filament::Texture::isTextureFormatMipmappable(engine,
      format);
  uint8_t levels = 1u;
  while (mips && (std::max(w, h) >> levels) > 0u)
    ++levels;
  filament::Texture *tex = filament::Texture::Builder()
      .width(w).height(h).levels(levels)
      .sampler(filament::Texture::Sampler::SAMPLER_2D)
      .format(format)
      .usage(filament::Texture::Usage::DEFAULT |
             (levels > 1u ? filament::Texture::Usage::GEN_MIPMAPPABLE :
                            filament::Texture::Usage::NONE))
      .build(engine);
  filament::Texture::PixelBufferDescriptor desc(buf, rgba.size(),
      filament::Texture::Format::RGBA, filament::Texture::Type::UBYTE,
      [](void *_b, size_t, void *) { delete[] static_cast<unsigned char *>(_b); });
  tex->setImage(engine, 0, std::move(desc));
  if (levels > 1u)
    tex->generateMipmaps(engine);
  this->dataPtr->textures[_path] = tex;
  return tex;
}

//////////////////////////////////////////////////
bool FilamentRenderEngine::LoadImpl(
    const std::map<std::string, std::string> &_params)
{
  auto it = _params.find("backend");
  if (it != _params.end())
    this->dataPtr->backend = common::lowercase(it->second);
  // Compatibility with the ogre2 style switches used by gz-sim / tests
  it = _params.find("vulkan");
  if (it != _params.end() && it->second == "1")
    this->dataPtr->backend = "vulkan";
  it = _params.find("opengl");
  if (it != _params.end() && it->second == "1")
    this->dataPtr->backend = "opengl";
  return true;
}

//////////////////////////////////////////////////
bool FilamentRenderEngine::InitImpl()
{
  // OpenGL by default: sensors are readback bound and filament's Vulkan
  // readPixels allocates a staging buffer per call (~4 ms per readback on
  // NVIDIA, vs ~0.3 ms on OpenGL). Vulkan stays available on request.
  filament::Engine::Backend backend = filament::Engine::Backend::OPENGL;
  if (this->dataPtr->backend == "vulkan")
    backend = filament::Engine::Backend::VULKAN;

  this->dataPtr->engine = filament::Engine::Builder()
      .backend(backend)
      .build();
  if (!this->dataPtr->engine && backend == filament::Engine::Backend::VULKAN)
  {
    // e.g. no Vulkan loader or ICD on this machine
    gzwarn << "Filament Vulkan backend unavailable, falling back to OpenGL"
           << std::endl;
    this->dataPtr->backend = "opengl";
    this->dataPtr->engine = filament::Engine::Builder()
        .backend(filament::Engine::Backend::OPENGL)
        .build();
  }
  if (!this->dataPtr->engine)
  {
    gzerr << "Failed to create filament engine (backend: "
          << this->dataPtr->backend << ")" << std::endl;
    return false;
  }
  this->dataPtr->renderer = this->dataPtr->engine->createRenderer();
  this->dataPtr->scenes = FilamentSceneStorePtr(new FilamentSceneStore);

  gzmsg << "Filament render engine initialized (backend: "
        << this->dataPtr->backend << ", resources: "
        << this->dataPtr->resourcePath << ")" << std::endl;
  return true;
}

//////////////////////////////////////////////////
SceneStorePtr FilamentRenderEngine::Scenes() const
{
  return this->dataPtr->scenes;
}

//////////////////////////////////////////////////
ScenePtr FilamentRenderEngine::CreateSceneImpl(unsigned int _id,
    const std::string &_name)
{
  auto scene = FilamentScenePtr(new FilamentScene(_id, _name));
  this->dataPtr->scenes->Add(scene);
  return scene;
}

// Register this plugin
GZ_ADD_PLUGIN(rendering::FilamentRenderEnginePlugin,
              rendering::RenderEnginePlugin)
GZ_ADD_PLUGIN_ALIAS(rendering::FilamentRenderEnginePlugin,
                    "gz::rendering::filament::Plugin")
