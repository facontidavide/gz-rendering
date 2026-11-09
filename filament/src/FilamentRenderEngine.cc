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
#include <cstring>
#include <fstream>
#include <iterator>
#include <vector>

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/Renderer.h>
#include <filament/SwapChain.h>
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
      /// \brief 1x1 headless swap chain for BeginFrame/EndFrame, lazy
      public: filament::SwapChain *headless{nullptr};
      public: bool inFrame{false};
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
    if (this->dataPtr->headless)
      this->dataPtr->engine->destroy(this->dataPtr->headless);
    this->dataPtr->headless = nullptr;
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
void FilamentRenderEngine::BeginFrame()
{
  if (this->dataPtr->inFrame)
    return;
  if (!this->dataPtr->headless)
  {
    this->dataPtr->headless =
        this->dataPtr->engine->createSwapChain(1, 1, 0);
  }
  // the frame skipper may ask to skip; rendering anyway is allowed and the
  // first render() then does the deferred frame setup
  this->dataPtr->renderer->beginFrame(this->dataPtr->headless);
  this->dataPtr->inFrame = true;
}

//////////////////////////////////////////////////
void FilamentRenderEngine::EndFrame()
{
  if (!this->dataPtr->inFrame)
    return;
  this->dataPtr->renderer->endFrame();
  this->dataPtr->inFrame = false;
}

//////////////////////////////////////////////////
bool FilamentRenderEngine::InFrame() const
{
  return this->dataPtr->inFrame;
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
    std::shared_ptr<const common::Image> _data, bool _srgb)
{
  const std::string key = _srgb ? _path : _path + "\n(linear)";
  auto it = this->dataPtr->textures.find(key);
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
  const auto format = _srgb ? filament::Texture::InternalFormat::SRGB8_A8 :
      filament::Texture::InternalFormat::RGBA8;
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
  this->dataPtr->textures[key] = tex;
  return tex;
}

namespace
{
  /// \brief Decode a BC1 (DXT1) or BC3 (DXT5) surface to RGBA8
  void DecodeBc(const uint8_t *_src, unsigned int _w, unsigned int _h,
      bool _bc3, uint8_t *_dst)
  {
    const unsigned int bw = (_w + 3u) / 4u;
    const unsigned int bh = (_h + 3u) / 4u;
    for (unsigned int by = 0; by < bh; ++by)
    {
      for (unsigned int bx = 0; bx < bw; ++bx)
      {
        const uint8_t *b = _src + (by * bw + bx) * (_bc3 ? 16u : 8u);
        uint8_t alpha[16];
        std::fill(alpha, alpha + 16, 255);
        if (_bc3)
        {
          int a[8] = {b[0], b[1]};
          for (int i = 2; i < 8; ++i)
          {
            a[i] = a[0] > a[1] ? ((8 - i) * a[0] + (i - 1) * a[1]) / 7 :
                i < 6 ? ((6 - i) * a[0] + (i - 1) * a[1]) / 5 :
                i == 6 ? 0 : 255;
          }
          uint64_t bits = 0;
          for (int k = 0; k < 6; ++k)
            bits |= static_cast<uint64_t>(b[2 + k]) << (8 * k);
          for (int p = 0; p < 16; ++p)
            alpha[p] = static_cast<uint8_t>(a[(bits >> (3 * p)) & 7u]);
          b += 8;
        }
        const unsigned int c[2] = {b[0] | (b[1] << 8u), b[2] | (b[3] << 8u)};
        int pal[4][4];
        for (int i = 0; i < 2; ++i)
        {
          pal[i][0] = ((c[i] >> 11) & 31) * 255 / 31;
          pal[i][1] = ((c[i] >> 5) & 63) * 255 / 63;
          pal[i][2] = (c[i] & 31) * 255 / 31;
          pal[i][3] = 255;
        }
        const bool four = _bc3 || c[0] > c[1];
        for (int k = 0; k < 3; ++k)
        {
          pal[2][k] = four ? (2 * pal[0][k] + pal[1][k]) / 3 :
              (pal[0][k] + pal[1][k]) / 2;
          pal[3][k] = four ? (pal[0][k] + 2 * pal[1][k]) / 3 : 0;
        }
        pal[2][3] = 255;
        pal[3][3] = four ? 255 : 0;
        const uint32_t idx = b[4] | (b[5] << 8u) | (b[6] << 16u) |
            (static_cast<uint32_t>(b[7]) << 24u);
        for (unsigned int p = 0; p < 16; ++p)
        {
          const unsigned int x = bx * 4u + p % 4u;
          const unsigned int y = by * 4u + p / 4u;
          if (x >= _w || y >= _h)
            continue;
          const int *col = pal[(idx >> (2u * p)) & 3u];
          uint8_t *d = _dst + (static_cast<size_t>(y) * _w + x) * 4u;
          d[0] = static_cast<uint8_t>(col[0]);
          d[1] = static_cast<uint8_t>(col[1]);
          d[2] = static_cast<uint8_t>(col[2]);
          d[3] = _bc3 ? alpha[p] : static_cast<uint8_t>(col[3]);
        }
      }
    }
  }
}

//////////////////////////////////////////////////
filament::Texture *FilamentRenderEngine::CubeTexture(const std::string &_path,
    bool _srgb)
{
  const std::string key = (_srgb ? "\n(cube srgb)" : "\n(cube)") + _path;
  auto it = this->dataPtr->textures.find(key);
  if (it != this->dataPtr->textures.end())
    return it->second;

  unsigned int size = 1u;
  std::vector<uint8_t> faces(6u * 4u, 0u);
  if (!_path.empty())
  {
    std::string path = _path;
    if (!common::exists(path))
    {
      path = common::joinPaths(this->dataPtr->resourcePath, "textures",
          common::basename(_path));
    }
    std::ifstream file(path, std::ios::binary);
    std::vector<uint8_t> dds((std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>());
    auto u32 = [&](size_t _o)
    {
      return static_cast<uint32_t>(dds[_o] | (dds[_o + 1] << 8u) |
          (dds[_o + 2] << 16u) | (dds[_o + 3] << 24u));
    };
    const bool isDds = dds.size() >= 128u &&
        std::equal(dds.begin(), dds.begin() + 4, "DDS ");
    const std::string fourcc = isDds ?
        std::string(dds.begin() + 84, dds.begin() + 88) : "";
    const bool bc3 = fourcc == "DXT5";
    const uint32_t w = isDds ? u32(16) : 0u;
    const uint32_t mips = isDds ? std::max(1u, u32(28)) : 0u;
    // block-compressed bytes of one face, mip chain included
    size_t faceBytes = 0u;
    for (uint32_t l = 0; l < mips; ++l)
    {
      const size_t s = std::max(1u, ((w >> l) + 3u) / 4u);
      faceBytes += s * s * (bc3 ? 16u : 8u);
    }
    if (!isDds || (fourcc != "DXT1" && !bc3) || !(u32(112) & 0x200u) ||
        u32(12) != w || w == 0u || dds.size() < 128u + 6u * faceBytes)
    {
      gzerr << "Unable to load cube map [" << _path << "]: only DXT1/DXT5 "
            << "DDS cube maps are supported" << std::endl;
      return nullptr;
    }
    size = w;
    const size_t faceSize = static_cast<size_t>(size) * size * 4u;
    std::vector<uint8_t> ogre(6u * faceSize);
    for (int f = 0; f < 6; ++f)
    {
      DecodeBc(dds.data() + 128u + f * faceBytes, size, size, bc3,
          ogre.data() + f * faceSize);
    }
    // Ogre samples its (left-handed) cube maps at (x, y, -z). Filament
    // samples at (x, y, z): mirror the faces through the z = 0 plane.
    // +-X faces flip horizontally, +-Y faces vertically, +Z and -Z swap
    // and flip horizontally.
    faces.resize(6u * faceSize);
    const int src[6] = {0, 1, 2, 3, 5, 4};
    for (int f = 0; f < 6; ++f)
    {
      const bool flipV = f == 2 || f == 3;
      for (unsigned int y = 0; y < size; ++y)
      {
        for (unsigned int x = 0; x < size; ++x)
        {
          const unsigned int sx = flipV ? x : size - 1u - x;
          const unsigned int sy = flipV ? size - 1u - y : y;
          std::memcpy(faces.data() + f * faceSize + (y * size + x) * 4u,
              ogre.data() + src[f] * faceSize + (sy * size + sx) * 4u, 4u);
        }
      }
    }
  }

  auto &engine = *this->dataPtr->engine;
  uint8_t levels = 1u;
  while ((size >> levels) > 0u)
    ++levels;
  filament::Texture *tex = filament::Texture::Builder()
      .width(size).height(size).levels(levels)
      .sampler(filament::Texture::Sampler::SAMPLER_CUBEMAP)
      .format(_srgb ? filament::Texture::InternalFormat::SRGB8_A8 :
          filament::Texture::InternalFormat::RGBA8)
      .usage(filament::Texture::Usage::DEFAULT |
             filament::Texture::Usage::GEN_MIPMAPPABLE)
      .build(engine);
  auto *buf = new uint8_t[faces.size()];
  std::memcpy(buf, faces.data(), faces.size());
  tex->setImage(engine, 0, 0, 0, 0, size, size, 6,
      filament::Texture::PixelBufferDescriptor(buf, faces.size(),
      filament::Texture::Format::RGBA, filament::Texture::Type::UBYTE,
      [](void *_b, size_t, void *) { delete[] static_cast<uint8_t *>(_b); }));
  if (levels > 1u)
    tex->generateMipmaps(engine);
  this->dataPtr->textures[key] = tex;
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
  // ponytail: spike knob to measure Filament's automatic instancing
  // (batches primitives sharing geometry and MaterialInstance)
  const char *autoInst = std::getenv("FIL_AUTOINSTANCE");
  this->dataPtr->engine->setAutomaticInstancingEnabled(autoInst && *autoInst);
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
