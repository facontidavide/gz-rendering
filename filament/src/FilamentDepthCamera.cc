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
#include <limits>

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/View.h>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>

#include "gz/rendering/Image.hh"
#include "gz/rendering/filament/FilamentDepthCamera.hh"
#include "gz/rendering/filament/FilamentMesh.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
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

//////////////////////////////////////////////////
void FilamentDepthCamera::Render()
{
  if (!this->depthInstance)
    this->CreateDepthTexture();
  this->UpdateProjection();
  // background reads as "beyond far"
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

  Image img(width, height, PF_FLOAT32_R);
  this->renderTexture->Copy(img);
  const float *src = img.Data<float>();

  const float inf = std::numeric_limits<float>::infinity();
  const float nearClip = static_cast<float>(this->NearClipPlane());
  const float farClip = static_cast<float>(this->FarClipPlane());
  const float tol = 1e-4f;
  for (size_t i = 0; i < len; ++i)
  {
    float d = src[i];
    if (d > farClip - tol)
      d = inf;
    else if (d < nearClip + tol)
      d = -inf;
    this->depthImage[i] = d;
  }

  this->newDepthFrame(this->depthImage.data(), width, height, 1u, "FLOAT32");

  if (this->newRgbPointCloud.ConnectionCount() > 0u)
  {
    // Pinhole back-projection into the gz camera frame (X fwd, Y left, Z up)
    this->pointCloud.resize(len * 4u);
    const double fx = (width * 0.5) / std::tan(this->HFOV().Radian() * 0.5);
    const double cx = width * 0.5;
    const double cy = height * 0.5;
    for (unsigned int v = 0; v < height; ++v)
    {
      for (unsigned int u = 0; u < width; ++u)
      {
        const size_t i = static_cast<size_t>(v) * width + u;
        const float d = this->depthImage[i];
        float *p = &this->pointCloud[i * 4u];
        if (std::isinf(d))
        {
          p[0] = p[1] = p[2] = d;
        }
        else
        {
          p[0] = d;
          p[1] = static_cast<float>(-(u + 0.5 - cx) / fx * d);
          p[2] = static_cast<float>(-(v + 0.5 - cy) / fx * d);
        }
        p[3] = 0.0f;  // packed RGBA color: not rendered by this engine yet
      }
    }
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
