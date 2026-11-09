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

  // PBR maps. Data maps are linear, color maps (emissive, light) sRGB like
  // ogre2; the environment map is a cube map.
  auto *engine = FilamentRenderEngine::Instance();
  std::array<filament::Texture *, MAP_COUNT> mapTextures{};
  for (int i = 0; i < MAP_COUNT; ++i)
  {
    Map &m = this->maps[i];
    if (!m.name.empty() && !m.loaded)
    {
      m.texture = i == ENVIRONMENT ? engine->CubeTexture(m.name, true) :
          engine->Texture(m.name, m.data, i == EMISSIVE || i == LIGHT);
      m.loaded = true;
    }
    mapTextures[i] = m.texture;
  }

  // Pushing parameters dirties the instance's uniform buffer, which filament
  // then re-uploads: only do it when something changed.
  const math::Color d = this->Diffuse();
  const math::Color s = this->Specular();
  const math::Color e = this->Emissive();
  const std::array<float, 16> key = {d.R(), d.G(), d.B(),
      static_cast<float>(this->Transparency()), s.R(), s.G(), s.B(),
      this->Roughness(), this->Metalness(),
      this->LightingEnabled() ? 1.0f : 0.0f, e.R(), e.G(), e.B(),
      static_cast<float>(this->lightMapUvSet), 0.0f, 0.0f};
  if (this->pushed && key == this->pushedKey &&
      this->texture == this->pushedTexture && mapTextures == this->pushedMaps)
  {
    return;
  }
  this->Update(this->lit);
  this->Update(this->unlit);
  this->pushed = true;
  this->pushedKey = key;
  this->pushedTexture = this->texture;
  this->pushedMaps = mapTextures;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetRoughness(const float _roughness)
{
  this->roughness = _roughness;
}

//////////////////////////////////////////////////
float FilamentMaterial::Roughness() const
{
  return this->roughness;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetMetalness(const float _metalness)
{
  this->metalness = _metalness;
}

//////////////////////////////////////////////////
float FilamentMaterial::Metalness() const
{
  return this->metalness;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetMap(MapSlot _slot, const std::string &_name,
    const std::shared_ptr<const common::Image> &_img)
{
  Map &m = this->maps[_slot];
  m.name = _name;
  m.data = _name.empty() ? nullptr : _img;
  m.texture = nullptr;
  m.loaded = false;
}

//////////////////////////////////////////////////
bool FilamentMaterial::HasNormalMap() const
{
  return !this->maps[NORMAL].name.empty();
}

//////////////////////////////////////////////////
std::string FilamentMaterial::NormalMap() const
{
  return this->maps[NORMAL].name;
}

//////////////////////////////////////////////////
std::shared_ptr<const common::Image> FilamentMaterial::NormalMapData() const
{
  return this->maps[NORMAL].data;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetNormalMap(const std::string &_name,
    const std::shared_ptr<const common::Image> &_img)
{
  this->SetMap(NORMAL, _name, _img);
}

//////////////////////////////////////////////////
void FilamentMaterial::ClearNormalMap()
{
  this->SetMap(NORMAL, "", nullptr);
}

//////////////////////////////////////////////////
bool FilamentMaterial::HasRoughnessMap() const
{
  return !this->maps[ROUGHNESS].name.empty();
}

//////////////////////////////////////////////////
std::string FilamentMaterial::RoughnessMap() const
{
  return this->maps[ROUGHNESS].name;
}

//////////////////////////////////////////////////
std::shared_ptr<const common::Image> FilamentMaterial::RoughnessMapData() const
{
  return this->maps[ROUGHNESS].data;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetRoughnessMap(const std::string &_name,
    const std::shared_ptr<const common::Image> &_img)
{
  this->SetMap(ROUGHNESS, _name, _img);
}

//////////////////////////////////////////////////
void FilamentMaterial::ClearRoughnessMap()
{
  this->SetMap(ROUGHNESS, "", nullptr);
}

//////////////////////////////////////////////////
bool FilamentMaterial::HasMetalnessMap() const
{
  return !this->maps[METALNESS].name.empty();
}

//////////////////////////////////////////////////
std::string FilamentMaterial::MetalnessMap() const
{
  return this->maps[METALNESS].name;
}

//////////////////////////////////////////////////
std::shared_ptr<const common::Image> FilamentMaterial::MetalnessMapData() const
{
  return this->maps[METALNESS].data;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetMetalnessMap(const std::string &_name,
    const std::shared_ptr<const common::Image> &_img)
{
  this->SetMap(METALNESS, _name, _img);
}

//////////////////////////////////////////////////
void FilamentMaterial::ClearMetalnessMap()
{
  this->SetMap(METALNESS, "", nullptr);
}

//////////////////////////////////////////////////
bool FilamentMaterial::HasEnvironmentMap() const
{
  return !this->maps[ENVIRONMENT].name.empty();
}

//////////////////////////////////////////////////
std::string FilamentMaterial::EnvironmentMap() const
{
  return this->maps[ENVIRONMENT].name;
}

//////////////////////////////////////////////////
std::shared_ptr<const common::Image> FilamentMaterial::EnvironmentMapData() const
{
  return this->maps[ENVIRONMENT].data;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetEnvironmentMap(const std::string &_name,
    const std::shared_ptr<const common::Image> &_img)
{
  this->SetMap(ENVIRONMENT, _name, _img);
}

//////////////////////////////////////////////////
void FilamentMaterial::ClearEnvironmentMap()
{
  this->SetMap(ENVIRONMENT, "", nullptr);
}

//////////////////////////////////////////////////
bool FilamentMaterial::HasEmissiveMap() const
{
  return !this->maps[EMISSIVE].name.empty();
}

//////////////////////////////////////////////////
std::string FilamentMaterial::EmissiveMap() const
{
  return this->maps[EMISSIVE].name;
}

//////////////////////////////////////////////////
std::shared_ptr<const common::Image> FilamentMaterial::EmissiveMapData() const
{
  return this->maps[EMISSIVE].data;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetEmissiveMap(const std::string &_name,
    const std::shared_ptr<const common::Image> &_img)
{
  this->SetMap(EMISSIVE, _name, _img);
}

//////////////////////////////////////////////////
void FilamentMaterial::ClearEmissiveMap()
{
  this->SetMap(EMISSIVE, "", nullptr);
}

//////////////////////////////////////////////////
bool FilamentMaterial::HasLightMap() const
{
  return !this->maps[LIGHT].name.empty();
}

//////////////////////////////////////////////////
std::string FilamentMaterial::LightMap() const
{
  return this->maps[LIGHT].name;
}

//////////////////////////////////////////////////
unsigned int FilamentMaterial::LightMapTexCoordSet() const
{
  return this->lightMapUvSet;
}

//////////////////////////////////////////////////
std::shared_ptr<const common::Image> FilamentMaterial::LightMapData() const
{
  return this->maps[LIGHT].data;
}

//////////////////////////////////////////////////
void FilamentMaterial::SetLightMap(const std::string &_name,
    const std::shared_ptr<const common::Image> &_img, unsigned int _uvSet)
{
  this->SetMap(LIGHT, _name, _img);
  this->lightMapUvSet = _name.empty() ? 0u : _uvSet;
}

//////////////////////////////////////////////////
void FilamentMaterial::ClearLightMap()
{
  this->SetLightMap("", nullptr, 0u);
}

//////////////////////////////////////////////////
MaterialType FilamentMaterial::Type() const
{
  return MaterialType::MT_PBS;
}

//////////////////////////////////////////////////
filament::Texture *FilamentMaterial::FilamentTexture() const
{
  return this->texture;
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
  sampler.setAnisotropy(this->anisotropy);
  auto *engine = FilamentRenderEngine::Instance();
  _instance->setParameter("baseColorMap", this->texture ? this->texture :
      engine->Texture(""), sampler);
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

    // PBR maps, with ogre2's (Ogre PBS) semantics: roughness and metalness
    // maps (red channel) scale the scalars, the emissive map is tinted by the
    // emissive color when that is not black, the light map takes the
    // emissive slot and is modulated by the diffuse color, the environment
    // map is a reflection cube map. Unset maps get a 1x1 default and a
    // cleared flag bit.
    filament::Texture *white = engine->Texture("");
    int flags = 0;
    for (int i = 0; i < MAP_COUNT; ++i)
    {
      filament::Texture *t = this->maps[i].texture;
      if (t)
        flags |= 1 << i;
      static const char *kNames[MAP_COUNT] = {"normalMap", "roughnessMap",
          "metalnessMap", "emissiveMap", "lightMap", "environmentMap"};
      if (i == ENVIRONMENT)
      {
        filament::TextureSampler cube(
            filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR,
            filament::TextureSampler::MagFilter::LINEAR,
            filament::TextureSampler::WrapMode::CLAMP_TO_EDGE);
        _instance->setParameter(kNames[i], t ? t : engine->CubeTexture(""),
            cube);
        _instance->setParameter("environmentMapLod",
            t ? static_cast<float>(t->getLevels() - 1) : 0.0f);
      }
      else
      {
        _instance->setParameter(kNames[i], t ? t : white, sampler);
      }
    }
    _instance->setParameter("mapFlags", flags);
    _instance->setParameter("lightMapUvSet",
        static_cast<int>(this->lightMapUvSet));
    const math::Color e = this->Emissive();
    filament::math::float3 emissive = {e.R(), e.G(), e.B()};
    // ogre2 only applies the emissive color when it is not black
    const bool emissiveConstant = e.R() > 0 || e.G() > 0 || e.B() > 0;
    filament::math::float3 mapTint = emissiveConstant ? emissive :
        filament::math::float3{1.0f, 1.0f, 1.0f};
    _instance->setParameter("emissive", emissive);
    _instance->setParameter("emissiveMapTint", mapTint);
  }
}
