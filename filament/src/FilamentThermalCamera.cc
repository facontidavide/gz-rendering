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
#include <variant>

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Camera.h>
#include <filament/View.h>

#include <gz/common/Console.hh>

#include "gz/rendering/Image.hh"
#include "gz/rendering/Material.hh"
#include "gz/rendering/PixelFormat.hh"
#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentMesh.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentRenderTarget.hh"
#include "gz/rendering/filament/FilamentScene.hh"
#include "gz/rendering/filament/FilamentThermalCamera.hh"

using namespace gz;
using namespace rendering;

namespace
{
  /// \brief float to 8 bit unorm and back, like a RGBA8 render target
  float Unorm8(double _v)
  {
    return std::round(std::clamp(static_cast<float>(_v), 0.0f, 1.0f) *
        255.0f) / 255.0f;
  }

  /// \brief "temperature" user data as Kelvin, -1 when absent or a string
  /// (heat signature texture, unsupported)
  float Temperature(const VisualPtr &_visual)
  {
    Variant v = _visual->UserData("temperature");
    if (const float *f = std::get_if<float>(&v))
      return std::max(*f, 0.0f);
    if (const double *d = std::get_if<double>(&v))
      return std::max(static_cast<float>(*d), 0.0f);
    if (const int *i = std::get_if<int>(&v))
      return std::max(static_cast<float>(*i), 0.0f);
    return -1.0f;
  }
}

//////////////////////////////////////////////////
FilamentThermalCamera::FilamentThermalCamera()
{
}

//////////////////////////////////////////////////
FilamentThermalCamera::~FilamentThermalCamera()
{
}

//////////////////////////////////////////////////
void FilamentThermalCamera::Init()
{
  FilamentCamera::Init();
  this->view->setShadowingEnabled(false);
  this->view->setPostProcessingEnabled(false);
}

//////////////////////////////////////////////////
void FilamentThermalCamera::Destroy()
{
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  if (engine)
  {
    for (auto *mi : this->idInstances)
      engine->destroy(mi);
  }
  this->idInstances.clear();
  FilamentCamera::Destroy();
}

//////////////////////////////////////////////////
PixelFormat FilamentThermalCamera::ImageFormat() const
{
  return this->format;
}

//////////////////////////////////////////////////
void FilamentThermalCamera::SetImageFormat(PixelFormat _format, bool)
{
  // only 8 and 16 bit outputs, like ogre2
  this->format = _format == PF_L8 ? PF_L8 : PF_L16;
}

//////////////////////////////////////////////////
RenderTargetPtr FilamentThermalCamera::RenderTarget() const
{
  return this->renderTexture;
}

//////////////////////////////////////////////////
filament::MaterialInstance *FilamentThermalCamera::IdInstance(uint32_t _id)
{
  if (_id < this->idInstances.size())
    return this->idInstances[_id];
  filament::Material *mat =
      FilamentRenderEngine::Instance()->Material("gz_flat");
  if (!mat)
  {
    gzerr << "gz_flat material not available" << std::endl;
    return nullptr;
  }
  while (this->idInstances.size() <= _id)
  {
    const uint32_t id = static_cast<uint32_t>(this->idInstances.size());
    filament::MaterialInstance *mi = mat->createInstance();
    mi->setParameter("color", filament::math::float3{
        ((id >> 16) & 0xff) / 255.0f, ((id >> 8) & 0xff) / 255.0f,
        (id & 0xff) / 255.0f});
    this->idInstances.push_back(mi);
  }
  return this->idInstances[_id];
}

//////////////////////////////////////////////////
uint16_t FilamentThermalCamera::Encode(const float _rgba[4]) const
{
  // float math in the order of thermal_camera_fs.glsl (rgbToTemp is
  // always on in ogre2)
  const int bitDepth = this->ImageFormat() == PF_L8 ? 8 : 16;
  const float bitMax = static_cast<float>((1 << bitDepth) - 1);
  const float res = static_cast<float>(this->resolution);
  float temp = this->ambient;
  float heatRange = this->ambientRange;
  float dNorm = 0.5f;
  const bool isHeatSource =
      _rgba[1] == 0.0f && _rgba[2] == 0.0f && _rgba[3] == 0.0f;
  if (isHeatSource)
  {
    temp = _rgba[0] * bitMax * res;
    heatRange = this->heatSourceTempRange;
  }
  else
  {
    const float gray =
        _rgba[0] * 0.299f + _rgba[1] * 0.587f + _rgba[2] * 0.114f;
    dNorm = 1.0f - gray;
  }
  const float delta = (1.0f - dNorm) * heatRange;
  temp = temp - heatRange / 2.0f + delta;
  temp = std::clamp(temp, this->minTemp, this->maxTemp);
  temp /= res;
  temp /= bitMax;
  return static_cast<uint16_t>(
      std::round(std::clamp(temp, 0.0f, 1.0f) * bitMax));
}

//////////////////////////////////////////////////
void FilamentThermalCamera::Render()
{
  if (!this->textureCreated ||
      this->renderTexture->Width() != this->ImageWidth() ||
      this->renderTexture->Height() != this->ImageHeight())
  {
    this->renderTexture->SetFormat(PF_R8G8B8);
    this->renderTexture->SetSrgb(false);
    this->renderTexture->SetWidth(this->ImageWidth());
    this->renderTexture->SetHeight(this->ImageHeight());
    this->textureCreated = true;
  }
  this->renderTexture->PreRender();
  // ogre2 renders thermal with depth clamp, see SetClampedProjection
  this->SetClampedProjection(this->NearClipPlane() * 1e-3,
      this->FarClipPlane());
  // id 0: background, ogre2 clears its color pass to (0, 0, 0, 1)
  this->renderTexture->SetClearColor({0.0f, 0.0f, 0.0f, 1.0f});
  this->view->setVisibleLayers(0xff,
      FilamentConversions::ToLayers(this->VisibilityMask()));

  const int bitDepth = this->ImageFormat() == PF_L8 ? 8 : 16;
  const double bitMax = (1 << bitDepth) - 1.0;
  this->idValue.clear();
  const float bg[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  this->idValue.push_back(this->Encode(bg));

  std::vector<FilamentMesh *> overridden;
  for (FilamentMesh *mesh : this->scene->Meshes())
  {
    VisualPtr visual = mesh->Parent();
    if (!visual)
      continue;
    const float temp = Temperature(visual);
    for (unsigned int i = 0; i < mesh->SubMeshCount(); ++i)
    {
      float rgba[4];
      if (temp >= 0.0f)
      {
        // heat source: normalized temperature through an 8 bit channel
        rgba[0] = Unorm8((temp / this->resolution) / bitMax);
        rgba[1] = rgba[2] = rgba[3] = 0.0f;
      }
      else
      {
        // other objects: flat diffuse color, white without a material
        MaterialPtr mat = mesh->SubMeshByIndex(i)->Material();
        const math::Color c = mat ? mat->Diffuse() : math::Color::White;
        rgba[0] = Unorm8(c.R());
        rgba[1] = Unorm8(c.G());
        rgba[2] = Unorm8(c.B());
        rgba[3] = 1.0f;
      }
      const uint32_t id = static_cast<uint32_t>(this->idValue.size());
      this->idValue.push_back(this->Encode(rgba));
      mesh->OverrideMaterial(i, this->IdInstance(id));
    }
    overridden.push_back(mesh);
  }
  this->renderTexture->Render(this->view);
  for (FilamentMesh *m : overridden)
    m->RestoreMaterials();
}

//////////////////////////////////////////////////
void FilamentThermalCamera::PostRender()
{
  if (this->newThermalFrame.ConnectionCount() == 0u)
    return;
  const unsigned int width = this->ImageWidth();
  const unsigned int height = this->ImageHeight();
  Image image(width, height, PF_R8G8B8);
  this->renderTexture->Copy(image);
  const uint8_t *src = image.Data<uint8_t>();
  this->thermalImage.resize(static_cast<size_t>(width) * height);
  for (size_t i = 0; i < this->thermalImage.size(); ++i)
  {
    const uint32_t id = (static_cast<uint32_t>(src[i * 3]) << 16) |
        (static_cast<uint32_t>(src[i * 3 + 1]) << 8) | src[i * 3 + 2];
    this->thermalImage[i] =
        id < this->idValue.size() ? this->idValue[id] : this->idValue[0];
  }
  this->newThermalFrame(this->thermalImage.data(), width, height, 1u,
      PixelUtil::Name(this->ImageFormat()));
}

//////////////////////////////////////////////////
common::ConnectionPtr FilamentThermalCamera::ConnectNewThermalFrame(
    std::function<void(const uint16_t *, unsigned int, unsigned int,
    unsigned int, const std::string &)> _subscriber)
{
  return this->newThermalFrame.Connect(_subscriber);
}
