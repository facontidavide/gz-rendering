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

#include <filament/Engine.h>
#include <filament/LightManager.h>
#include <filament/Scene.h>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>

#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentLight.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentScene.hh"

using namespace gz;
using namespace rendering;

namespace
{
  filament::LightManager &Lights()
  {
    return FilamentRenderEngine::Instance()->FilamentEngine()
        ->getLightManager();
  }
}

//////////////////////////////////////////////////
FilamentLight::FilamentLight()
  : type(filament::LightManager::Type::DIRECTIONAL)
{
}

//////////////////////////////////////////////////
FilamentLight::FilamentLight(filament::LightManager::Type _type)
  : type(_type)
{
}

//////////////////////////////////////////////////
FilamentLight::~FilamentLight()
{
}

//////////////////////////////////////////////////
math::Color FilamentLight::DiffuseColor() const
{
  return this->diffuse;
}

//////////////////////////////////////////////////
void FilamentLight::SetDiffuseColor(const math::Color &_color)
{
  this->diffuse = _color;
  this->UpdateLight();
}

//////////////////////////////////////////////////
math::Color FilamentLight::SpecularColor() const
{
  return this->specular;
}

//////////////////////////////////////////////////
void FilamentLight::SetSpecularColor(const math::Color &_color)
{
  this->specular = _color;
}

//////////////////////////////////////////////////
double FilamentLight::AttenuationConstant() const
{
  return this->attenConstant;
}

//////////////////////////////////////////////////
void FilamentLight::SetAttenuationConstant(double _value)
{
  this->attenConstant = _value;
  this->UpdateLight();
}

//////////////////////////////////////////////////
double FilamentLight::AttenuationLinear() const
{
  return this->attenLinear;
}

//////////////////////////////////////////////////
void FilamentLight::SetAttenuationLinear(double _value)
{
  this->attenLinear = _value;
  this->UpdateLight();
}

//////////////////////////////////////////////////
double FilamentLight::AttenuationQuadratic() const
{
  return this->attenQuadratic;
}

//////////////////////////////////////////////////
void FilamentLight::SetAttenuationQuadratic(double _value)
{
  this->attenQuadratic = _value;
  this->UpdateLight();
}

//////////////////////////////////////////////////
double FilamentLight::AttenuationRange() const
{
  return this->attenRange;
}

//////////////////////////////////////////////////
void FilamentLight::SetAttenuationRange(double _range)
{
  this->attenRange = _range;
  this->UpdateLight();
}

//////////////////////////////////////////////////
bool FilamentLight::CastShadows() const
{
  return this->castShadows;
}

//////////////////////////////////////////////////
void FilamentLight::SetCastShadows(bool _castShadows)
{
  this->castShadows = _castShadows;
  this->UpdateLight();
}

//////////////////////////////////////////////////
double FilamentLight::Intensity() const
{
  return this->intensity;
}

//////////////////////////////////////////////////
void FilamentLight::SetIntensity(double _intensity)
{
  this->intensity = _intensity;
  this->UpdateLight();
}

//////////////////////////////////////////////////
void FilamentLight::Destroy()
{
  if (!this->entity.isNull())
  {
    if (this->scene && this->scene->Native())
      this->scene->Native()->remove(this->entity);
    Lights().destroy(this->entity);
  }
  BaseLight::Destroy();
}

//////////////////////////////////////////////////
void FilamentLight::Init()
{
  BaseLight::Init();
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  filament::LightManager::Builder(this->type)
      .castShadows(this->castShadows)
      .build(*engine, this->entity);
  this->scene->Native()->addEntity(this->entity);
  this->UpdateLight();
}

//////////////////////////////////////////////////
void FilamentLight::UpdateLight()
{
  auto &lm = Lights();
  if (this->entity.isNull() || !lm.hasComponent(this->entity))
    return;
  auto inst = lm.getInstance(this->entity);
  lm.setColor(inst, FilamentConversions::ToFloat3(this->diffuse));
  lm.setShadowCaster(inst, this->castShadows);
  // With the camera exposure fixed at 1, a directional light of pi lux on a
  // Lambertian surface gives albedo * cos(theta): the classic diffuse term.
  // ogre2 measures as ~0.8 * albedo * cos(theta) (linear) at default
  // roughness, hence the extra factor.
  constexpr double kOgre2Match = 0.8;
  const float power =
      static_cast<float>(GZ_PI * kOgre2Match * this->intensity);
  if (this->type == filament::LightManager::Type::DIRECTIONAL ||
      this->type == filament::LightManager::Type::SUN)
  {
    lm.setIntensity(inst, power);
  }
  else
  {
    lm.setIntensityCandela(inst, power);
    lm.setFalloff(inst, static_cast<float>(this->attenRange));
  }
}

//////////////////////////////////////////////////
FilamentDirectionalLight::FilamentDirectionalLight()
  : FilamentLight(filament::LightManager::Type::DIRECTIONAL)
{
}

//////////////////////////////////////////////////
FilamentDirectionalLight::~FilamentDirectionalLight()
{
}

//////////////////////////////////////////////////
math::Vector3d FilamentDirectionalLight::Direction() const
{
  return this->direction;
}

//////////////////////////////////////////////////
void FilamentDirectionalLight::SetDirection(const math::Vector3d &_dir)
{
  this->direction = _dir;
  this->UpdateLight();
}

//////////////////////////////////////////////////
void FilamentDirectionalLight::UpdateLight()
{
  FilamentLight::UpdateLight();
  auto &lm = Lights();
  if (this->entity.isNull() || !lm.hasComponent(this->entity))
    return;
  lm.setDirection(lm.getInstance(this->entity),
      normalize(FilamentConversions::Convert(this->direction)));
}

//////////////////////////////////////////////////
FilamentPointLight::FilamentPointLight()
  : FilamentLight(filament::LightManager::Type::POINT)
{
}

//////////////////////////////////////////////////
FilamentPointLight::~FilamentPointLight()
{
}

//////////////////////////////////////////////////
FilamentSpotLight::FilamentSpotLight()
  : FilamentLight(filament::LightManager::Type::SPOT)
{
}

//////////////////////////////////////////////////
FilamentSpotLight::~FilamentSpotLight()
{
}

//////////////////////////////////////////////////
math::Vector3d FilamentSpotLight::Direction() const
{
  return this->direction;
}

//////////////////////////////////////////////////
void FilamentSpotLight::SetDirection(const math::Vector3d &_dir)
{
  this->direction = _dir;
  this->UpdateLight();
}

//////////////////////////////////////////////////
math::Angle FilamentSpotLight::InnerAngle() const
{
  return this->innerAngle;
}

//////////////////////////////////////////////////
void FilamentSpotLight::SetInnerAngle(const math::Angle &_angle)
{
  this->innerAngle = _angle;
  this->UpdateLight();
}

//////////////////////////////////////////////////
math::Angle FilamentSpotLight::OuterAngle() const
{
  return this->outerAngle;
}

//////////////////////////////////////////////////
void FilamentSpotLight::SetOuterAngle(const math::Angle &_angle)
{
  this->outerAngle = _angle;
  this->UpdateLight();
}

//////////////////////////////////////////////////
double FilamentSpotLight::Falloff() const
{
  return this->falloff;
}

//////////////////////////////////////////////////
void FilamentSpotLight::SetFalloff(double _falloff)
{
  this->falloff = _falloff;
}

//////////////////////////////////////////////////
void FilamentSpotLight::UpdateLight()
{
  FilamentLight::UpdateLight();
  auto &lm = Lights();
  if (this->entity.isNull() || !lm.hasComponent(this->entity))
    return;
  auto inst = lm.getInstance(this->entity);
  lm.setDirection(inst, normalize(FilamentConversions::Convert(this->direction)));
  // gz angles are full cone angles; filament wants half angles
  lm.setSpotLightCone(inst,
      static_cast<float>(this->innerAngle.Radian() * 0.5),
      static_cast<float>(this->outerAngle.Radian() * 0.5));
}
