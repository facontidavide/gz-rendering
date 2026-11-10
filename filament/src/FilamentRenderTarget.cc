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

#include <cstring>
#include <utility>
#include <vector>

#include <filament/Engine.h>
#include <filament/RenderTarget.h>
#include <filament/Renderer.h>
#include <filament/Texture.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <backend/PixelBufferDescriptor.h>

#include <gz/common/Console.hh>

#include "gz/rendering/Image.hh"
#include "gz/rendering/PixelFormat.hh"
#include "gz/rendering/filament/FilamentGaussianNoisePass.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/Utils.hh"
#include "gz/rendering/filament/FilamentRenderTarget.hh"

using namespace gz;
using namespace rendering;

namespace
{
  filament::Engine *Engine()
  {
    return FilamentRenderEngine::Instance()->FilamentEngine();
  }
}

//////////////////////////////////////////////////
FilamentRenderTarget::FilamentRenderTarget()
{
}

//////////////////////////////////////////////////
FilamentRenderTarget::~FilamentRenderTarget()
{
}

//////////////////////////////////////////////////
void FilamentRenderTarget::Destroy()
{
  this->DestroyTarget();
  BaseRenderTarget::Destroy();
}

//////////////////////////////////////////////////
void FilamentRenderTarget::DestroyTarget()
{
  auto *engine = Engine();
  if (!engine)
    return;
  for (auto *t : {this->target, this->scratchTarget})
  {
    if (t)
      engine->destroy(t);
  }
  for (auto *t : {this->colorTexture, this->depthTexture,
                  this->scratchColor, this->scratchDepth})
  {
    if (t)
      engine->destroy(t);
  }
  this->target = nullptr;
  this->colorTexture = nullptr;
  this->depthTexture = nullptr;
  this->scratchTarget = nullptr;
  this->scratchColor = nullptr;
  this->scratchDepth = nullptr;
}

//////////////////////////////////////////////////
filament::Texture *FilamentRenderTarget::ColorTexture() const
{
  return this->colorTexture;
}

//////////////////////////////////////////////////
void FilamentRenderTarget::RebuildImpl()
{
  this->DestroyTarget();
  if (this->width == 0u || this->height == 0u)
    return;
  this->BuildAttachments(this->target, this->colorTexture,
      this->depthTexture);
}

//////////////////////////////////////////////////
void FilamentRenderTarget::BuildAttachments(filament::RenderTarget *&_target,
    filament::Texture *&_color, filament::Texture *&_depth) const
{
  auto *engine = Engine();

  filament::Texture::InternalFormat colorFormat;
  switch (this->format)
  {
    case PF_FLOAT32_R:
      colorFormat = filament::Texture::InternalFormat::R32F;
      break;
    case PF_FLOAT32_RGBA:
      colorFormat = filament::Texture::InternalFormat::RGBA32F;
      break;
    case PF_L8:
      // like ogre2, which renders grey scale natively into R8 unorm: the
      // linear red channel, no sRGB encoding
      colorFormat = filament::Texture::InternalFormat::R8;
      break;
    default:
      colorFormat = this->srgb ? filament::Texture::InternalFormat::SRGB8_A8
                               : filament::Texture::InternalFormat::RGBA8;
      break;
  }

  _color = filament::Texture::Builder()
      .width(this->width).height(this->height).levels(1)
      .format(colorFormat)
      .usage(filament::Texture::Usage::COLOR_ATTACHMENT |
             filament::Texture::Usage::SAMPLEABLE |
             filament::Texture::Usage::BLIT_SRC)
      .build(*engine);
  _depth = filament::Texture::Builder()
      .width(this->width).height(this->height).levels(1)
      .format(filament::Texture::InternalFormat::DEPTH32F)
      .usage(filament::Texture::Usage::DEPTH_ATTACHMENT)
      .build(*engine);
  _target = filament::RenderTarget::Builder()
      .texture(filament::RenderTarget::AttachmentPoint::COLOR, _color)
      .texture(filament::RenderTarget::AttachmentPoint::DEPTH, _depth)
      .build(*engine);
}

//////////////////////////////////////////////////
const std::vector<RenderPassPtr> &FilamentRenderTarget::RenderPasses() const
{
  return this->renderPasses;
}

//////////////////////////////////////////////////
void FilamentRenderTarget::ApplyRenderPasses(
    const std::vector<RenderPassPtr> &_passes)
{
  for (const RenderPassPtr &pass : _passes)
  {
    auto *noise = dynamic_cast<FilamentGaussianNoisePass *>(pass.get());
    if (!noise || !noise->IsEnabled() || !this->target)
      continue;
    // ping-pong: draw from the current attachments into a second set, which
    // then becomes the current one
    if (!this->scratchTarget)
    {
      this->BuildAttachments(this->scratchTarget, this->scratchColor,
          this->scratchDepth);
    }
    filament::View *view = noise->ColorView(this->colorTexture);
    if (!view)
      continue;
    std::swap(this->target, this->scratchTarget);
    std::swap(this->colorTexture, this->scratchColor);
    std::swap(this->depthTexture, this->scratchDepth);
    this->Render(view);
  }
}

//////////////////////////////////////////////////
void FilamentRenderTarget::SetClearColor(const filament::math::float4 &_color)
{
  this->clearColor = _color;
}

//////////////////////////////////////////////////
filament::RenderTarget *FilamentRenderTarget::Native() const
{
  return this->target;
}

//////////////////////////////////////////////////
void FilamentRenderTarget::Render(filament::View *_view,
    const filament::Viewport *_viewport, bool _clear)
{
  if (!this->target || !_view)
    return;
  auto *renderer = FilamentRenderEngine::Instance()->Renderer();
  filament::Renderer::ClearOptions clear;
  clear.clearColor = this->clearColor;
  clear.clear = _clear;
  clear.discard = _clear;
  renderer->setClearOptions(clear);
  _view->setRenderTarget(this->target);
  _view->setViewport(_viewport ? *_viewport :
      filament::Viewport{0, 0, this->width, this->height});
  renderer->renderStandaloneView(_view);
}

//////////////////////////////////////////////////
void FilamentRenderTarget::SetSrgb(bool _srgb)
{
  if (this->srgb == _srgb)
    return;
  this->srgb = _srgb;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void FilamentRenderTarget::Copy(Image &_image) const
{
  if (!this->target)
  {
    gzerr << "Render target not built" << std::endl;
    return;
  }
  if (_image.Width() != this->width || _image.Height() != this->height)
  {
    gzerr << "Invalid image dimensions" << std::endl;
    return;
  }

  const PixelFormat dstFormat = _image.Format();
  if (dstFormat == PF_BAYER_RGGB8 || dstFormat == PF_BAYER_BGGR8 ||
      dstFormat == PF_BAYER_GBRG8 || dstFormat == PF_BAYER_GRBG8)
  {
    // render target holds RGB: read that back, then mosaic on the CPU
    Image colorImage(this->width, this->height, PF_R8G8B8);
    this->Copy(colorImage);
    convertRGBToBayer(colorImage, PixelBuffer(_image));
    return;
  }

  using PF = filament::backend::PixelDataFormat;
  using PT = filament::backend::PixelDataType;
  PF readFormat;
  PT readType;
  size_t readBpp;
  switch (this->format)
  {
    case PF_FLOAT32_R:
      readFormat = PF::R; readType = PT::FLOAT; readBpp = 4u;
      break;
    case PF_FLOAT32_RGBA:
      readFormat = PF::RGBA; readType = PT::FLOAT; readBpp = 16u;
      break;
    case PF_L8:
      readFormat = PF::R; readType = PT::UBYTE; readBpp = 1u;
      break;
    default:
      readFormat = PF::RGBA; readType = PT::UBYTE; readBpp = 4u;
      break;
  }

  const size_t rowBytes = readBpp * this->width;
  // resize keeps the allocation (and skips zeroing) from frame to frame
  std::vector<unsigned char> &buf = this->readBuffer;
  buf.resize(rowBytes * this->height);
  filament::backend::PixelBufferDescriptor desc(buf.data(), buf.size(),
      readFormat, readType);
  auto *renderer = FilamentRenderEngine::Instance()->Renderer();
  renderer->readPixels(this->target, 0, 0, this->width, this->height,
      std::move(desc));
  Engine()->flushAndWait();

  // Offscreen render targets read back top row first on both the Vulkan and
  // OpenGL backends (verified empirically), matching gz image layout.
  unsigned char *dst = _image.Data<unsigned char>();
  const PixelFormat imgFormat = _image.Format();
  const size_t dstBpp = PixelUtil::BytesPerPixel(imgFormat);
  for (unsigned int y = 0; y < this->height; ++y)
  {
    const unsigned char *src = buf.data() + static_cast<size_t>(y) * rowBytes;
    unsigned char *row = dst + static_cast<size_t>(y) * this->width * dstBpp;
    if (imgFormat == PF_R8G8B8 && readBpp == 4u)
    {
      // one 4-byte store per pixel; the next pixel overwrites the alpha
      for (unsigned int x = 0; x + 1u < this->width; ++x)
        std::memcpy(row + x * 3u, src + x * 4u, 4u);
      std::memcpy(row + (this->width - 1u) * 3u,
          src + (this->width - 1u) * 4u, 3u);
    }
    else if (imgFormat == PF_B8G8R8 && readBpp == 4u)
    {
      for (unsigned int x = 0; x < this->width; ++x)
      {
        row[x * 3u + 0u] = src[x * 4u + 2u];
        row[x * 3u + 1u] = src[x * 4u + 1u];
        row[x * 3u + 2u] = src[x * 4u + 0u];
      }
    }
    else if (dstBpp == readBpp)
    {
      std::memcpy(row, src, rowBytes);
    }
    else
    {
      gzerr << "Unsupported image format for copy: " << imgFormat
            << std::endl;
      return;
    }
  }
}

//////////////////////////////////////////////////
FilamentRenderTexture::FilamentRenderTexture()
{
}

//////////////////////////////////////////////////
FilamentRenderTexture::~FilamentRenderTexture()
{
}
