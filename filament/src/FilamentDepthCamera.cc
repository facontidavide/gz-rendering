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
#include <random>
#include <string>
#include <type_traits>
#include <vector>

#include <backend/PixelBufferDescriptor.h>
#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Renderer.h>
#include <filament/View.h>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Rand.hh>

#include "gz/rendering/Image.hh"
#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentDepthCamera.hh"
#include "gz/rendering/filament/FilamentGaussianNoisePass.hh"
#include "gz/rendering/filament/FilamentMesh.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentRenderTarget.hh"
#include "gz/rendering/filament/FilamentScene.hh"
#include "FilamentParticles.hh"

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
  // no shadow maps for the depth pass, nor any screen-space effect
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

namespace
{
  /// \brief Whether the depth target holds the depth's float bits as RGBA8
  /// (gz_depth_packed) instead of R32F. Vulkan readPixels converts R32F
  /// texel by texel but copies RGBA8 rows as they are; OpenGL reads both
  /// directly, so it keeps R32F.
  // ponytail: RenderTarget()->Copy() of a depth camera then reads the packed
  // bytes on Vulkan; nothing (gz-sensors, tests) copies it, unpack if needed
  bool PackedDepth()
  {
    return FilamentRenderEngine::Instance()->FilamentEngine()->getBackend()
        == filament::Engine::Backend::VULKAN;
  }
}

//////////////////////////////////////////////////
void FilamentDepthCamera::CreateDepthTexture()
{
  if (!this->depthInstance)
  {
    const std::string matName = PackedDepth() ? "gz_depth_packed" :
        "gz_depth";
    filament::Material *mat =
        FilamentRenderEngine::Instance()->Material(matName);
    if (!mat)
    {
      gzerr << matName << " material not available" << std::endl;
      return;
    }
    this->depthInstance = mat->createInstance();
  }
  if (PackedDepth())
  {
    this->renderTexture->SetFormat(PF_R8G8B8A8);
    this->renderTexture->SetSrgb(false);
  }
  else
  {
    this->renderTexture->SetFormat(PF_FLOAT32_R);
  }
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
      FilamentConversions::MaskToLayers(this->VisibilityMask()));

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
    // the color and the depth pass share one frame (one command buffer,
    // one submit), closed after the depth pass
    FilamentRenderEngine::Instance()->BeginFrame();
    // ogre2 renders this pass with its shadow node, like a camera
    this->view->setShadowingEnabled(true);
    this->colorTexture->Render(this->view);
    this->view->setShadowingEnabled(false);
  }

  // Depth pass: every mesh drawn with the linear-depth material, no
  // particles (PostRender adds their effect). Background reads as "beyond
  // far".
  this->view->setVisibleLayers(0xff,
      FilamentConversions::ToSensorLayers(this->VisibilityMask()));
  const float inf = std::numeric_limits<float>::infinity();
  // packed: the bytes of +inf (0x7f800000), least significant first
  if (PackedDepth())
    this->renderTexture->SetClearColor({0.0f, 0.0f, 128.0f / 255.0f, 127.0f / 255.0f});
  else
    this->renderTexture->SetClearColor({inf, inf, inf, 1.0f});
  const auto &meshes = this->scene->Meshes();
  for (FilamentMesh *m : meshes)
    m->OverrideMaterial(this->depthInstance);
  this->renderTexture->Render(this->view);
  FilamentRenderEngine::Instance()->EndFrame();
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
  const bool packedDepth = PackedDepth();
  renderer->readPixels(this->renderTexture->Native(), 0, 0, width, height,
      filament::backend::PixelBufferDescriptor(this->depthRaw.data(),
      len * sizeof(float),
      packedDepth ? filament::backend::PixelDataFormat::RGBA :
                    filament::backend::PixelDataFormat::R,
      packedDepth ? filament::backend::PixelDataType::UBYTE :
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
  double fx = (width * 0.5) / std::tan(this->HFOV().Radian() * 0.5);
  double fy = fx;
  double cx = width * 0.5;
  double cy = height * 0.5;
  if (this->customProjection)
  {
    // pinhole of the custom matrix (ogre2 interpolates the frustum corners
    // of its inverse): x_ndc = (P00 x + P02 z) / -z, same for y
    const math::Matrix4d proj = FilamentCamera::ProjectionMatrix();
    fx = proj(0, 0) * width * 0.5;
    fy = proj(1, 1) * height * 0.5;
    cx = width * 0.5 * (1.0 - proj(0, 2));
    cy = height * 0.5 * (1.0 + proj(1, 2));
  }
  // ogre2 also writes the background color for clamped points; it
  // gamma-encodes it, the color pass target here is sRGB as well
  const math::Color bg = this->scene->BackgroundColor();
  const uint32_t bgPacked =
      (static_cast<uint32_t>(std::round(FilamentConversions::LinearToSrgb(bg.R()) * 255.0)) << 24u) |
      (static_cast<uint32_t>(std::round(FilamentConversions::LinearToSrgb(bg.G()) * 255.0)) << 16u) |
      (static_cast<uint32_t>(std::round(FilamentConversions::LinearToSrgb(bg.B()) * 255.0)) << 8u) |
      255u;
  // particles: half resolution depth of the nearest particle, like ogre2's
  // particle pass, sampled nearest
  namespace fp = filament_particles;
  fp::Noise noise;
  const unsigned int pw = std::max(width / 2u, 1u);
  const unsigned int ph = std::max(height / 2u, 1u);
  bool particles = fp::FindNoise(*this->scene, noise);
  if (particles)
  {
    this->particleDepth.resize(static_cast<size_t>(pw) * ph);
    const math::Pose3d world = this->WorldPose();
    particles = fp::Raster(*this->scene, world.Pos(),
        world.Rot().XAxis(), world.Rot().YAxis(), world.Rot().ZAxis(),
        fx * pw / width, fy * ph / height, pw, ph,
        nearClip * kNearScale, this->particleDepth.data());
    // ponytail: particles assume a centered principal point, off-center
    // custom matrices shift them by (cx, cy) - center; pass both if needed
  }
  // per frame seed, like ogre2's rnd uniform (gz::math::Rand stays seedable)
  std::minstd_rand rng;
  if (particles)
  {
    rng.seed(static_cast<unsigned int>(
        math::Rand::IntUniform(1, std::numeric_limits<int>::max())));
  }
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  // ray slopes per column and row: -(u + 0.5 - cx) / fx * d below is
  // colK[u] * d, the same double operations in the same order
  std::vector<double> colK(width), rowK(height);
  for (unsigned int u = 0; u < width; ++u)
    colK[u] = -(u + 0.5 - cx) / fx;
  for (unsigned int v = 0; v < height; ++v)
    rowK[v] = -(v + 0.5 - cy) / fy;

  // Fast path without particles: the same clamping and packing as the loop
  // below, with no per pixel divisions and nothing else in the loop.
  auto convert = [&](auto _cloud)
  {
    constexpr bool cloud = decltype(_cloud)::value;
    const double farT = farClip - tol;
    const double nearT = nearClip + tol;
    float *depthOut = this->depthImage.data();
    float *cloudOut = this->pointCloud.data();
    for (unsigned int v = 0; v < height; ++v)
    {
      const double kb = rowK[v];
      const size_t row = static_cast<size_t>(v) * width;
      for (unsigned int u = 0; u < width; ++u)
      {
        const size_t i = row + u;
        const double d = src[i];
        const double p1 = colK[u] * d;
        const double p2 = kb * d;
        float x = static_cast<float>(d);
        float y = static_cast<float>(p1);
        float z = static_cast<float>(p2);
        bool clamped = false;
        if (std::isinf(d) || std::sqrt(d * d + p1 * p1 + p2 * p2) > farT)
        {
          x = y = z = maxVal;
          clamped = true;
        }
        else if (d < nearT)
        {
          x = y = z = minVal;
          clamped = true;
        }
        depthOut[i] = x;
        if constexpr (cloud)
        {
          float *out = cloudOut + i * 4u;
          out[0] = x;
          out[1] = y;
          out[2] = z;
          const uint32_t packed = (clamped || !rgba) ? bgPacked :
              (static_cast<uint32_t>(rgba[i * 4u]) << 24u) |
              (static_cast<uint32_t>(rgba[i * 4u + 1u]) << 16u) |
              (static_cast<uint32_t>(rgba[i * 4u + 2u]) << 8u) | 255u;
          std::memcpy(&out[3], &packed, sizeof(float));
        }
      }
    }
  };
  if (!particles)
  {
    if (wantCloud)
      convert(std::true_type{});
    else
      convert(std::false_type{});
  }

  for (unsigned int v = 0; particles && v < height; ++v)
  {
    for (unsigned int u = 0; u < width; ++u)
    {
      const size_t i = static_cast<size_t>(v) * width + u;
      double d = src[i];
      bool particlePixel = false;
      if (particles)
      {
        const float pd = this->particleDepth[
            std::min(ph - 1u, (2u * v + 1u) * ph / (2u * height)) * pw +
            std::min(pw - 1u, (2u * u + 1u) * pw / (2u * width))];
        particlePixel = std::isfinite(pd);
        if (particlePixel && pd < d)
        {
          // ranges along the pixel ray
          const double a = (u + 0.5 - cx) / fx;
          const double b = (v + 0.5 - cy) / fy;
          const double rayLen = std::sqrt(1.0 + a * a + b * b);
          d = fp::Scatter(uniform(rng), noise, pd * rayLen, d * rayLen) /
              rayLen;
        }
      }
      // camera frame: X forward, Y left, Z up
      double p[3] = {d, -(u + 0.5 - cx) / fx * d, -(v + 0.5 - cy) / fy * d};
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
      // ogre2 keeps the color of particle pixels
      const uint32_t packed = ((clamped && !particlePixel) || !rgba) ?
          bgPacked :
          (static_cast<uint32_t>(rgba[i * 4u]) << 24u) |
          (static_cast<uint32_t>(rgba[i * 4u + 1u]) << 16u) |
          (static_cast<uint32_t>(rgba[i * 4u + 2u]) << 8u) | 255u;
      std::memcpy(&out[3], &packed, sizeof(float));
    }
  }

  FilamentGaussianNoisePass::ApplyDepth(*this, this->depthImage.data(),
      wantCloud ? this->pointCloud.data() : nullptr);

  this->newDepthFrame(this->depthImage.data(), width, height, 1u, "FLOAT32");
  if (wantCloud)
  {
    this->newRgbPointCloud(this->pointCloud.data(), width, height, 4u,
        "PF_FLOAT32_RGBA");
  }
}

//////////////////////////////////////////////////
math::Matrix4d FilamentDepthCamera::ProjectionMatrix() const
{
  if (!this->customProjection)
    return FilamentCamera::ProjectionMatrix();
  return WithDepthRows(FilamentCamera::ProjectionMatrix(),
      this->NearClipPlane(), this->FarClipPlane());
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
