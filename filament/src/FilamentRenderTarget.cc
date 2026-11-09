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
  if (this->target)
    engine->destroy(this->target);
  if (this->colorTexture)
    engine->destroy(this->colorTexture);
  if (this->depthTexture)
    engine->destroy(this->depthTexture);
  this->target = nullptr;
  this->colorTexture = nullptr;
  this->depthTexture = nullptr;
}

//////////////////////////////////////////////////
void FilamentRenderTarget::RebuildImpl()
{
  this->DestroyTarget();
  if (this->width == 0u || this->height == 0u)
    return;
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
    default:
      colorFormat = filament::Texture::InternalFormat::SRGB8_A8;
      break;
  }

  this->colorTexture = filament::Texture::Builder()
      .width(this->width).height(this->height).levels(1)
      .format(colorFormat)
      .usage(filament::Texture::Usage::COLOR_ATTACHMENT |
             filament::Texture::Usage::SAMPLEABLE |
             filament::Texture::Usage::BLIT_SRC)
      .build(*engine);
  this->depthTexture = filament::Texture::Builder()
      .width(this->width).height(this->height).levels(1)
      .format(filament::Texture::InternalFormat::DEPTH32F)
      .usage(filament::Texture::Usage::DEPTH_ATTACHMENT)
      .build(*engine);
  this->target = filament::RenderTarget::Builder()
      .texture(filament::RenderTarget::AttachmentPoint::COLOR,
               this->colorTexture)
      .texture(filament::RenderTarget::AttachmentPoint::DEPTH,
               this->depthTexture)
      .build(*engine);
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
void FilamentRenderTarget::Render(filament::View *_view)
{
  if (!this->target || !_view)
    return;
  auto *renderer = FilamentRenderEngine::Instance()->Renderer();
  filament::Renderer::ClearOptions clear;
  clear.clearColor = this->clearColor;
  clear.clear = true;
  clear.discard = true;
  renderer->setClearOptions(clear);
  _view->setRenderTarget(this->target);
  _view->setViewport({0, 0, this->width, this->height});
  renderer->renderStandaloneView(_view);
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
    default:
      readFormat = PF::RGBA; readType = PT::UBYTE; readBpp = 4u;
      break;
  }

  const size_t rowBytes = readBpp * this->width;
  std::vector<unsigned char> buf(rowBytes * this->height);
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
      for (unsigned int x = 0; x < this->width; ++x)
      {
        row[x * 3u + 0u] = src[x * 4u + 0u];
        row[x * 3u + 1u] = src[x * 4u + 1u];
        row[x * 3u + 2u] = src[x * 4u + 2u];
      }
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
