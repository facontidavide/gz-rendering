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
#include <filament/Scene.h>
#include <filament/TransformManager.h>

#include <gz/common/Console.hh>

#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentGeometry.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentScene.hh"
#include "gz/rendering/filament/FilamentVisual.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
FilamentVisual::FilamentVisual()
{
}

//////////////////////////////////////////////////
FilamentVisual::~FilamentVisual()
{
}

//////////////////////////////////////////////////
void FilamentVisual::SetVisible(bool _visible)
{
  this->visible = _visible;
  this->UpdateLayers();
  for (unsigned int i = 0; i < this->ChildCount(); ++i)
  {
    auto child = std::dynamic_pointer_cast<Visual>(this->ChildByIndex(i));
    if (child)
      child->SetVisible(_visible);
  }
}

//////////////////////////////////////////////////
void FilamentVisual::SetVisibilityFlags(uint32_t _flags)
{
  BaseVisual::SetVisibilityFlags(_flags);
  this->UpdateLayers();
}

//////////////////////////////////////////////////
void FilamentVisual::UpdateLayers()
{
  const uint8_t layers = this->visible ?
      FilamentConversions::ToLayers(this->VisibilityFlags()) : 0u;
  for (auto it = this->geometries->Begin(); it != this->geometries->End(); ++it)
    (*it)->SetLayers(layers);
}

//////////////////////////////////////////////////
GeometryStorePtr FilamentVisual::Geometries() const
{
  return this->geometries;
}

//////////////////////////////////////////////////
bool FilamentVisual::AttachGeometry(GeometryPtr _geometry)
{
  if (!_geometry)
  {
    gzerr << "Cannot attach null geometry." << std::endl;
    return false;
  }
  FilamentGeometryPtr derived =
      std::dynamic_pointer_cast<FilamentGeometry>(_geometry);
  if (!derived)
  {
    gzerr << "Cannot attach geometry created by another render-engine"
          << std::endl;
    return false;
  }
  if (derived->Entity().isNull())
  {
    gzerr << "Cannot attach a geometry without a filament entity"
          << std::endl;
    return false;
  }
  derived->SetParent(this->SharedThis());
  auto &tm = FilamentRenderEngine::Instance()->FilamentEngine()
      ->getTransformManager();
  tm.setParent(tm.getInstance(derived->Entity()),
               tm.getInstance(this->entity));
  this->scene->Native()->addEntity(derived->Entity());
  derived->SetLayers(this->visible ?
      FilamentConversions::ToLayers(this->VisibilityFlags()) : 0u);
  return true;
}

//////////////////////////////////////////////////
bool FilamentVisual::DetachGeometry(GeometryPtr _geometry)
{
  FilamentGeometryPtr derived =
      std::dynamic_pointer_cast<FilamentGeometry>(_geometry);
  if (!derived)
  {
    gzerr << "Cannot detach geometry created by another render-engine"
          << std::endl;
    return false;
  }
  if (!derived->Entity().isNull())
  {
    auto &tm = FilamentRenderEngine::Instance()->FilamentEngine()
        ->getTransformManager();
    tm.setParent(tm.getInstance(derived->Entity()), {});
    this->scene->Native()->remove(derived->Entity());
  }
  derived->SetParent(nullptr);
  return true;
}

//////////////////////////////////////////////////
void FilamentVisual::Init()
{
  BaseVisual::Init();
  this->geometries = FilamentGeometryStorePtr(new FilamentGeometryStore);
}

//////////////////////////////////////////////////
FilamentVisualPtr FilamentVisual::SharedThis()
{
  return std::dynamic_pointer_cast<FilamentVisual>(this->shared_from_this());
}
