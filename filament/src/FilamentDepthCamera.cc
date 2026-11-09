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

#include <cmath>
#include <cstring>
#include <limits>

#include <backend/PixelBufferDescriptor.h>
#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Renderer.h>
#include <filament/View.h>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>

#include "gz/rendering/Image.hh"
#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentDepthCamera.hh"
#include "gz/rendering/filament/FilamentMesh.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentRenderTarget.hh"
#include "gz/rendering/filament/FilamentScene.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
FilamentDepthCamera::FilamentDepthCamera()
{
}

//////////////////////////////////////////////////
FilamentDepthCamera::~FilamentDepthCamera()
{
}

//////////////////////////////////////////////////
void FilamentDepthCamera::Init()
{
  FilamentCamera::Init();
  // depth values must not be touched by shadows or any screen-space effect
  this->view->setShadowingEnabled(false);
  this->view->setPostProcessingEnabled(false);
}

//////////////////////////////////////////////////
RenderTargetPtr FilamentDepthCamera::RenderTarget() const
{
  return this->renderTexture;
}

//////////////////////////////////////////////////
void FilamentDepthCamera::Destroy()
{
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  if (engine && this->depthInstance)
    engine->destroy(this->depthInstance);
  this->depthInstance = nullptr;
  if (this->colorTexture)
  {
    this->colorTexture->Destroy();
    this->colorTexture.reset();
  }
  FilamentCamera::Destroy();
}

//////////////////////////////////////////////////
void FilamentDepthCamera::CreateDepthTexture()
{
  if (!this->depthInstance)
  {
    filament::Material *mat =
        FilamentRenderEngine::Instance()->Material("gz_depth");
    if (!mat)
    {
      gzerr << "gz_depth material not available" << std::endl;
      return;
    }
    this->depthInstance = mat->createInstance();
  }
  this->renderTexture->SetFormat(PF_FLOAT32_R);
  this->renderTexture->SetWidth(this->ImageWidth());
  this->renderTexture->SetHeight(this->ImageHeight());
}

namespace
{
  /// \brief The depth pass renders with a closer near plane and a farther far
  /// plane than requested, so geometry just outside [near, far] is still
  /// rasterized and can be reported as -inf / +inf instead of being clipped
  /// (ogre2 gets the near case from depth clamping, which filament only
  /// enables for shadow passes).
  constexpr double kNearScale = 0.1;
  constexpr double kFarScale = 1.1;
}

//////////////////////////////////////////////////
void FilamentDepthCamera::Render()
{
  if (!this->depthInstance)
    this->CreateDepthTexture();

  this->SetClampedProjection(this->NearClipPlane() * kNearScale,
      this->FarClipPlane() * kFarScale);

  this->view->setVisibleLayers(0xff,
      FilamentConversions::ToLayers(this->VisibilityMask()));

  // Color pass, only needed for the colored point cloud
  if (this->newRgbPointCloud.ConnectionCount() > 0u)
  {
    if (!this->colorTexture)
    {
      this->colorTexture = std::dynamic_pointer_cast<FilamentRenderTexture>(
          this->scene->CreateRenderTexture());
      this->colorTexture->SetFormat(PF_R8G8B8A8);
    }
    // SetWidth/SetHeight mark the target dirty (rebuilt in PreRender)
    if (this->colorTexture->Width() != this->ImageWidth() ||
        this->colorTexture->Height() != this->ImageHeight())
    {
      this->colorTexture->SetWidth(this->ImageWidth());
      this->colorTexture->SetHeight(this->ImageHeight());
    }
    this->colorTexture->PreRender();
    this->colorTexture->SetClearColor(
        FilamentConversions::ToFloat4(this->scene->BackgroundColor()));
    this->colorTexture->Render(this->view);
  }

  // Depth pass: every mesh drawn with the linear-depth material.
  // Background reads as "beyond far".
  const float inf = std::numeric_limits<float>::infinity();
  this->renderTexture->SetClearColor({inf, inf, inf, 1.0f});
  const auto &meshes = this->scene->Meshes();
  for (FilamentMesh *m : meshes)
    m->OverrideMaterial(this->depthInstance);
  this->renderTexture->Render(this->view);
  for (FilamentMesh *m : meshes)
    m->RestoreMaterials();
}

//////////////////////////////////////////////////
void FilamentDepthCamera::PostRender()
{
  FilamentCamera::PostRender();

  const unsigned int width = this->ImageWidth();
  const unsigned int height = this->ImageHeight();
  const size_t len = static_cast<size_t>(width) * height;
  this->depthImage.resize(len);

  // queue both readbacks, wait once (rows come back top row first, the
  // gz image layout)
  const bool wantCloud = this->newRgbPointCloud.ConnectionCount() > 0u;
  auto *renderer = FilamentRenderEngine::Instance()->Renderer();
  this->depthRaw.resize(len);
  renderer->readPixels(this->renderTexture->Native(), 0, 0, width, height,
      filament::backend::PixelBufferDescriptor(this->depthRaw.data(),
      len * sizeof(float), filament::backend::PixelDataFormat::R,
      filament::backend::PixelDataType::FLOAT));
  const unsigned char *rgba = nullptr;
  if (wantCloud && this->colorTexture)
  {
    this->colorRaw.resize(len * 4u);
    renderer->readPixels(this->colorTexture->Native(), 0, 0, width, height,
        filament::backend::PixelBufferDescriptor(this->colorRaw.data(),
        this->colorRaw.size(), filament::backend::PixelDataFormat::RGBA,
        filament::backend::PixelDataType::UBYTE));
    rgba = this->colorRaw.data();
  }
  FilamentRenderEngine::Instance()->FilamentEngine()->flushAndWait();
  const float *src = this->depthRaw.data();
  if (wantCloud)
    this->pointCloud.resize(len * 4u);

  // Same clamping rules as ogre2's depth_camera_fs.glsl: beyond far is
  // judged on the euclidean range, nearer than near on the forward distance.
  const float minVal = -std::numeric_limits<float>::infinity();
  const float maxVal = std::numeric_limits<float>::infinity();
  const double nearClip = this->NearClipPlane();
  const double farClip = this->FarClipPlane();
  const double tol = 1e-6;
  const double fx = (width * 0.5) / std::tan(this->HFOV().Radian() * 0.5);
  const double cx = width * 0.5;
  const double cy = height * 0.5;
  // ogre2 also writes the background color for clamped points; it
  // gamma-encodes it, the color pass target here is sRGB as well
  const math::Color bg = this->scene->BackgroundColor();
  const uint32_t bgPacked =
      (static_cast<uint32_t>(std::round(FilamentConversions::LinearToSrgb(bg.R()) * 255.0)) << 24u) |
      (static_cast<uint32_t>(std::round(FilamentConversions::LinearToSrgb(bg.G()) * 255.0)) << 16u) |
      (static_cast<uint32_t>(std::round(FilamentConversions::LinearToSrgb(bg.B()) * 255.0)) << 8u) |
      255u;
  for (unsigned int v = 0; v < height; ++v)
  {
    for (unsigned int u = 0; u < width; ++u)
    {
      const size_t i = static_cast<size_t>(v) * width + u;
      const double d = src[i];
      // camera frame: X forward, Y left, Z up
      double p[3] = {d, -(u + 0.5 - cx) / fx * d, -(v + 0.5 - cy) / fx * d};
      bool clamped = false;
      if (std::isinf(d) ||
          std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]) > farClip - tol)
      {
        p[0] = p[1] = p[2] = maxVal;
        clamped = true;
      }
      else if (d < nearClip + tol)
      {
        p[0] = p[1] = p[2] = minVal;
        clamped = true;
      }
      this->depthImage[i] = static_cast<float>(p[0]);

      if (!wantCloud)
        continue;
      float *out = &this->pointCloud[i * 4u];
      out[0] = static_cast<float>(p[0]);
      out[1] = static_cast<float>(p[1]);
      out[2] = static_cast<float>(p[2]);
      const uint32_t packed = (clamped || !rgba) ? bgPacked :
          (static_cast<uint32_t>(rgba[i * 4u]) << 24u) |
          (static_cast<uint32_t>(rgba[i * 4u + 1u]) << 16u) |
          (static_cast<uint32_t>(rgba[i * 4u + 2u]) << 8u) | 255u;
      std::memcpy(&out[3], &packed, sizeof(float));
    }
  }

  this->newDepthFrame(this->depthImage.data(), width, height, 1u, "FLOAT32");
  if (wantCloud)
  {
    this->newRgbPointCloud(this->pointCloud.data(), width, height, 4u,
        "PF_FLOAT32_RGBA");
  }
}

//////////////////////////////////////////////////
const float *FilamentDepthCamera::DepthData() const
{
  return this->depthImage.data();
}

//////////////////////////////////////////////////
common::ConnectionPtr FilamentDepthCamera::ConnectNewDepthFrame(
    std::function<void(const float *, unsigned int, unsigned int,
    unsigned int, const std::string &)> _subscriber)
{
  return this->newDepthFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
common::ConnectionPtr FilamentDepthCamera::ConnectNewRgbPointCloud(
    std::function<void(const float *, unsigned int, unsigned int,
    unsigned int, const std::string &)> _subscriber)
{
  return this->newRgbPointCloud.Connect(_subscriber);
}
