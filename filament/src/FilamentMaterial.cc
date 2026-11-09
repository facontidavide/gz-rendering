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
#include <array>
#include <cmath>

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>

#include <gz/common/Console.hh>

#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentMaterial.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentScene.hh"
#include "gz/rendering/filament/FilamentMesh.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
FilamentMaterial::FilamentMaterial()
{
}

//////////////////////////////////////////////////
FilamentMaterial::~FilamentMaterial()
{
}

//////////////////////////////////////////////////
void FilamentMaterial::Destroy()
{
  BaseMaterial::Destroy();
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  // filament refuses to destroy an instance a renderable still uses
  // ponytail: scans every mesh of the scene, a per-material user list if
  // destroying materials of big scenes ever shows up in profiles
  filament::Material *fallback =
      FilamentRenderEngine::Instance()->Material("gz_unlit");
  if (engine && fallback && this->scene)
  {
    for (FilamentMesh *mesh : this->scene->Meshes())
    {
      for (auto *mi : {this->lit, this->unlit})
      {
        if (mi)
          mesh->ReplaceInstance(mi, fallback->getDefaultInstance());
      }
    }
  }
  if (engine)
  {
    if (this->lit)
      engine->destroy(this->lit);
    if (this->unlit)
      engine->destroy(this->unlit);
  }
  this->lit = nullptr;
  this->unlit = nullptr;
}

//////////////////////////////////////////////////
bool FilamentMaterial::HasTexture() const
{
  return !this->textureName.empty();
}

//////////////////////////////////////////////////
std::string FilamentMaterial::Texture() const
{
  return this->textureName;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetTexture(const std::string &_texture)
{
  this->SetTexture(_texture, nullptr);
}

//////////////////////////////////////////////////
void FilamentMaterial::SetTexture(const std::string &_texture,
    const std::shared_ptr<const common::Image> &_img)
{
  this->textureName = _texture;
  this->textureData = _texture.empty() ? nullptr : _img;
  // force a reload even when only the image data changed
  this->loadedTexture.clear();
}

//////////////////////////////////////////////////
std::shared_ptr<const common::Image> FilamentMaterial::TextureData() const
{
  return this->textureData;
}

//////////////////////////////////////////////////
void FilamentMaterial::ClearTexture()
{
  this->SetTexture("", nullptr);
}

//////////////////////////////////////////////////
filament::MaterialInstance *FilamentMaterial::Instance() const
{
  return this->LightingEnabled() ? this->lit : this->unlit;
}

//////////////////////////////////////////////////
void FilamentMaterial::Init()
{
  BaseMaterial::Init();
  auto *engine = FilamentRenderEngine::Instance();
  filament::Material *litMat = engine->Material("gz_lit");
  filament::Material *unlitMat = engine->Material("gz_unlit");
  if (!litMat || !unlitMat)
  {
    gzerr << "Filament base materials not available" << std::endl;
    return;
  }
  this->lit = litMat->createInstance();
  this->unlit = unlitMat->createInstance();
  this->Reset();
  this->PreRender();
}

//////////////////////////////////////////////////
void FilamentMaterial::PreRender()
{
  BaseMaterial::PreRender();
  if (!this->lit || !this->unlit)
    return;

  if (this->HasTexture() && this->Texture() != this->loadedTexture)
  {
    this->texture = FilamentRenderEngine::Instance()->Texture(
        this->Texture(), this->TextureData());
    this->loadedTexture = this->Texture();
  }
  else if (!this->HasTexture())
  {
    this->texture = nullptr;
    this->loadedTexture.clear();
  }

  // Pushing parameters dirties the instance's uniform buffer, which filament
  // then re-uploads: only do it when something changed.
  const math::Color d = this->Diffuse();
  const math::Color s = this->Specular();
  const std::array<float, 11> key = {d.R(), d.G(), d.B(),
      static_cast<float>(this->Transparency()), s.R(), s.G(), s.B(),
      this->Roughness(), this->Metalness(),
      this->LightingEnabled() ? 1.0f : 0.0f, 0.0f};
  if (this->pushed && key == this->pushedKey &&
      this->texture == this->pushedTexture)
  {
    return;
  }
  this->Update(this->lit);
  this->Update(this->unlit);
  this->pushed = true;
  this->pushedKey = key;
  this->pushedTexture = this->texture;
}

//////////////////////////////////////////////////
void FilamentMaterial::Update(filament::MaterialInstance *_instance)
{
  math::Color diffuse = this->Diffuse();
  diffuse.A(static_cast<float>(1.0 - this->Transparency()));
  _instance->setParameter("baseColor", FilamentConversions::ToFloat4(diffuse));
  _instance->setParameter("hasTexture", this->texture ? 1 : 0);
  filament::TextureSampler sampler(
      filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR,
      filament::TextureSampler::MagFilter::LINEAR,
      filament::TextureSampler::WrapMode::REPEAT);
  _instance->setParameter("baseColorMap", this->texture ? this->texture :
      FilamentRenderEngine::Instance()->Texture(""), sampler);
  if (_instance == this->lit)
  {
    _instance->setParameter("roughness",
        std::clamp(this->Roughness(), 0.05f, 1.0f));
    _instance->setParameter("metallic",
        std::clamp(this->Metalness(), 0.0f, 1.0f));
    // ogre2 scales the specular lobe by the gz specular color. filament's
    // dielectric F0 is 0.16 * reflectance^2 (0.04 at the default 0.5), so
    // F0 = 0.04 * specular gives reflectance = 0.5 * sqrt(specular).
    const math::Color s = this->Specular();
    const double spec = std::clamp((s.R() + s.G() + s.B()) / 3.0, 0.0, 1.0);
    _instance->setParameter("reflectance",
        static_cast<float>(0.5 * std::sqrt(spec)));
  }
}
