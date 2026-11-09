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
#include <cmath>
#include <cstring>
#include <limits>

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <utils/EntityManager.h>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>

#include "gz/rendering/Image.hh"
#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentGpuRays.hh"
#include "gz/rendering/filament/FilamentMesh.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentRenderTarget.hh"
#include "gz/rendering/filament/FilamentScene.hh"

using namespace gz;
using namespace rendering;

namespace
{
  using filament::math::float3;

  /// \brief Cube face axes in the sensor frame (X forward, Y left, Z up):
  /// forward and up. Right is forward x up.
  struct Face { float3 fwd; float3 up; };
  const std::array<Face, 6> kFaces = {{
    {{1, 0, 0}, {0, 0, 1}}, {{-1, 0, 0}, {0, 0, 1}},
    {{0, 1, 0}, {0, 0, 1}}, {{0, -1, 0}, {0, 0, 1}},
    {{0, 0, 1}, {-1, 0, 0}}, {{0, 0, -1}, {1, 0, 0}}}};

  /// \brief Filament camera transform for a face: camera looks down -Z with
  /// Y up, so its basis columns are (right, up, -forward).
  filament::math::mat4f FaceTransform(const Face &_f)
  {
    const float3 r = cross(_f.fwd, _f.up);
    return filament::math::mat4f(
        filament::math::float4{r, 0.0f},
        filament::math::float4{_f.up, 0.0f},
        filament::math::float4{-_f.fwd, 0.0f},
        filament::math::float4{0.0f, 0.0f, 0.0f, 1.0f});
  }

  // Same near/far widening as the depth camera, see FilamentDepthCamera.cc
  constexpr double kNearScale = 0.1;
  constexpr double kFarScale = 1.1;
}

//////////////////////////////////////////////////
FilamentGpuRays::FilamentGpuRays()
{
  // range, retro, unused (same layout as ogre2)
  this->channels = 3u;
}

//////////////////////////////////////////////////
FilamentGpuRays::~FilamentGpuRays()
{
}

//////////////////////////////////////////////////
void FilamentGpuRays::Init()
{
  BaseGpuRays::Init();
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  auto &tm = engine->getTransformManager();
  this->cameraEntity = ::utils::EntityManager::get().create();
  tm.create(this->cameraEntity, tm.getInstance(this->entity),
      FaceTransform(kFaces[0]));
  this->camera = engine->createCamera(this->cameraEntity);
  this->view = engine->createView();
  this->view->setScene(this->scene->Native());
  this->view->setCamera(this->camera);
  this->view->setPostProcessingEnabled(false);
  this->view->setShadowingEnabled(false);

  filament::Material *mat =
      FilamentRenderEngine::Instance()->Material("gz_depth");
  if (mat)
    this->depthInstance = mat->createInstance();
  else
    gzerr << "gz_depth material not available" << std::endl;

  this->renderTexture = std::dynamic_pointer_cast<FilamentRenderTexture>(
      this->scene->CreateRenderTexture());
  this->renderTexture->SetFormat(PF_FLOAT32_R);
}

//////////////////////////////////////////////////
void FilamentGpuRays::Destroy()
{
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  if (engine)
  {
    if (this->view)
      engine->destroy(this->view);
    if (this->depthInstance)
      engine->destroy(this->depthInstance);
    if (!this->cameraEntity.isNull())
    {
      engine->destroyCameraComponent(this->cameraEntity);
      engine->getTransformManager().destroy(this->cameraEntity);
      ::utils::EntityManager::get().destroy(this->cameraEntity);
    }
  }
  this->view = nullptr;
  this->depthInstance = nullptr;
  this->camera = nullptr;
  this->cameraEntity = ::utils::Entity();
  if (this->renderTexture)
  {
    this->renderTexture->Destroy();
    this->renderTexture.reset();
  }
  BaseGpuRays::Destroy();
}

//////////////////////////////////////////////////
RenderTargetPtr FilamentGpuRays::RenderTarget() const
{
  return this->renderTexture;
}

//////////////////////////////////////////////////
void FilamentGpuRays::UpdateRays()
{
  const unsigned int hCount = std::max(this->RangeCount(), 1);
  const unsigned int vCount = std::max(this->VerticalRangeCount(), 1);
  const double hMin = this->AngleMin().Radian();
  const double hMax = this->AngleMax().Radian();
  const double vMin = this->VerticalAngleMin().Radian();
  const double vMax = this->VerticalAngleMax().Radian();
  const std::array<double, 8> key = {static_cast<double>(hCount),
      static_cast<double>(vCount), hMin, hMax, vMin, vMax, 0.0, 0.0};
  if (key == this->rayKey && !this->rays.empty())
    return;
  this->rayKey = key;

  // Texel density: about two texels per ray step in the denser direction
  const double hStep = hCount > 1 ? (hMax - hMin) / (hCount - 1) : 0.0;
  const double vStep = vCount > 1 ? (vMax - vMin) / (vCount - 1) : 0.0;
  double minStep = GZ_PI_2 / 256.0;
  if (hStep > 1e-6)
    minStep = std::min(minStep, hStep);
  if (vStep > 1e-6)
    minStep = std::min(minStep, vStep);
  // ponytail: nearest-texel sampling, so range error grows toward the face
  // edges; raise the 2x oversampling if a test needs tighter ranges.
  this->faceRes = static_cast<unsigned int>(std::clamp(
      std::ceil(2.0 * GZ_PI_2 / minStep), 64.0, 2048.0));
  const unsigned int n = this->faceRes;

  this->rays.resize(static_cast<size_t>(hCount) * vCount);
  this->faceUsed.fill(false);
  for (unsigned int j = 0; j < vCount; ++j)
  {
    const double v = vMin + j * vStep;
    for (unsigned int i = 0; i < hCount; ++i)
    {
      const double h = hMin + i * hStep;
      const float3 d{static_cast<float>(std::cos(v) * std::cos(h)),
                     static_cast<float>(std::cos(v) * std::sin(h)),
                     static_cast<float>(std::sin(v))};
      // the face is the axis with the largest component
      int face = 0;
      float best = -1.0f;
      for (int f = 0; f < 6; ++f)
      {
        const float c = dot(d, kFaces[f].fwd);
        if (c > best)
        {
          best = c;
          face = f;
        }
      }
      const Face &fc = kFaces[face];
      const float3 r = cross(fc.fwd, fc.up);
      const float a = dot(d, r) / best;      // [-1, 1] across the face
      const float b = dot(d, fc.up) / best;  // [-1, 1] bottom to top
      const unsigned int px = std::min(n - 1, static_cast<unsigned int>(
          std::max(0.0f, (a * 0.5f + 0.5f) * n)));
      // image rows are stored top row first
      const unsigned int py = std::min(n - 1, static_cast<unsigned int>(
          std::max(0.0f, (0.5f - b * 0.5f) * n)));
      this->rays[static_cast<size_t>(j) * hCount + i] =
          {face, py * n + px, 1.0f / best};
      this->faceUsed[face] = true;
    }
  }
}

//////////////////////////////////////////////////
void FilamentGpuRays::Render()
{
  if (!this->depthInstance)
    return;
  this->UpdateRays();
  const unsigned int n = this->faceRes;

  this->camera->setProjection(90.0, 1.0,
      this->NearClipPlane() * kNearScale, this->FarClipPlane() * kFarScale,
      filament::Camera::Fov::HORIZONTAL);
  this->view->setVisibleLayers(0xff,
      FilamentConversions::ToLayers(this->VisibilityMask()));

  this->renderTexture->SetWidth(n);
  this->renderTexture->SetHeight(n);
  this->renderTexture->PreRender();
  const float inf = std::numeric_limits<float>::infinity();
  this->renderTexture->SetClearColor({inf, inf, inf, 1.0f});

  auto &tm = FilamentRenderEngine::Instance()->FilamentEngine()
      ->getTransformManager();
  const auto camInst = tm.getInstance(this->cameraEntity);
  const auto &meshes = this->scene->Meshes();
  for (FilamentMesh *m : meshes)
    m->OverrideMaterial(this->depthInstance);
  Image img(n, n, PF_FLOAT32_R);
  for (int f = 0; f < 6; ++f)
  {
    if (!this->faceUsed[f])
      continue;
    tm.setTransform(camInst, FaceTransform(kFaces[f]));
    this->renderTexture->Render(this->view);
    this->renderTexture->Copy(img);
    const float *src = img.Data<float>();
    this->faceDepth[f].assign(src, src + static_cast<size_t>(n) * n);
  }
  for (FilamentMesh *m : meshes)
    m->RestoreMaterials();
}

//////////////////////////////////////////////////
void FilamentGpuRays::PostRender()
{
  const unsigned int hCount = std::max(this->RangeCount(), 1);
  const unsigned int vCount = std::max(this->VerticalRangeCount(), 1);
  const unsigned int channels = this->Channels();
  this->scan.assign(static_cast<size_t>(hCount) * vCount * channels, 0.0f);
  if (this->rays.size() != static_cast<size_t>(hCount) * vCount)
    return;

  const float nearClip = static_cast<float>(this->NearClipPlane());
  const float farClip = static_cast<float>(this->FarClipPlane());
  for (size_t k = 0; k < this->rays.size(); ++k)
  {
    const Ray &ray = this->rays[k];
    const std::vector<float> &depth = this->faceDepth[ray.face];
    float range = depth.empty() ? std::numeric_limits<float>::infinity() :
        depth[ray.texel] * ray.invCos;
    if (!std::isfinite(range) || range > farClip)
      range = this->dataMaxVal;
    else if (range < nearClip)
      range = this->dataMinVal;
    this->scan[k * channels] = range;
  }
  this->newFrame(this->scan.data(), hCount, vCount, channels,
      "PF_FLOAT32_RGB");
}

//////////////////////////////////////////////////
const float *FilamentGpuRays::Data() const
{
  return this->scan.data();
}

//////////////////////////////////////////////////
void FilamentGpuRays::Copy(float *_dataDest)
{
  std::memcpy(_dataDest, this->scan.data(), this->scan.size() * sizeof(float));
}

//////////////////////////////////////////////////
common::ConnectionPtr FilamentGpuRays::ConnectNewGpuRaysFrame(
    std::function<void(const float *, unsigned int, unsigned int,
    unsigned int, const std::string &)> _subscriber)
{
  return this->newFrame.Connect(_subscriber);
}
