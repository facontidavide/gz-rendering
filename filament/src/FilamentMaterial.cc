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

#include <filament/Engine.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>

#include <gz/common/Console.hh>

#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentMaterial.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"

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
    this->texture = FilamentRenderEngine::Instance()->Texture(this->Texture());
    this->loadedTexture = this->Texture();
  }
  else if (!this->HasTexture())
  {
    this->texture = nullptr;
    this->loadedTexture.clear();
  }

  this->Update(this->lit);
  this->Update(this->unlit);
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
  }
}
