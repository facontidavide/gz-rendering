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

#include <cstdlib>

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <utils/EntityManager.h>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>

#include "gz/rendering/filament/FilamentCamera.hh"
#include "gz/rendering/filament/FilamentConversions.hh"
#include "gz/rendering/filament/FilamentMesh.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentScene.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
FilamentCamera::FilamentCamera()
{
}

//////////////////////////////////////////////////
FilamentCamera::~FilamentCamera()
{
}

//////////////////////////////////////////////////
void FilamentCamera::Destroy()
{
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  if (engine)
  {
    if (this->view)
      engine->destroy(this->view);
    if (!this->cameraEntity.isNull())
    {
      engine->destroyCameraComponent(this->cameraEntity);
      engine->getTransformManager().destroy(this->cameraEntity);
      ::utils::EntityManager::get().destroy(this->cameraEntity);
    }
  }
  this->view = nullptr;
  this->camera = nullptr;
  this->cameraEntity = ::utils::Entity();
  if (this->renderTexture)
  {
    this->renderTexture->Destroy();
    this->renderTexture.reset();
  }
  BaseCamera::Destroy();
}

//////////////////////////////////////////////////
filament::View *FilamentCamera::View() const
{
  return this->view;
}

//////////////////////////////////////////////////
filament::Camera *FilamentCamera::Native() const
{
  return this->camera;
}

//////////////////////////////////////////////////
RenderTargetPtr FilamentCamera::RenderTarget() const
{
  return this->renderTexture;
}

//////////////////////////////////////////////////
void FilamentCamera::Init()
{
  BaseCamera::Init();
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  auto &tm = engine->getTransformManager();

  this->cameraEntity = ::utils::EntityManager::get().create();
  tm.create(this->cameraEntity, tm.getInstance(this->entity),
      FilamentConversions::CameraFix());
  this->camera = engine->createCamera(this->cameraEntity);
  // exposure 1: pixel = radiance, no photographic scaling
  this->camera->setExposure(1.0f);

  this->view = engine->createView();
  this->view->setScene(this->scene->Native());
  this->view->setCamera(this->camera);
  this->view->setVisibleLayers(0xff, 0x1);
  // Sensors want plain radiance: no tone mapping, AA, dithering, bloom...
  this->view->setPostProcessingEnabled(false);
  // ponytail: FIL_NOSHADOW=1 is a spike benchmarking knob, drop before merge
  const char *noShadow = std::getenv("FIL_NOSHADOW");
  this->view->setShadowingEnabled(!(noShadow && *noShadow));

  this->CreateRenderTexture();
  this->Reset();
}

//////////////////////////////////////////////////
VisualPtr FilamentCamera::VisualAt(const math::Vector2i &_mousePos)
{
  if (!this->view || _mousePos.X() < 0 || _mousePos.Y() < 0 ||
      static_cast<unsigned int>(_mousePos.X()) >= this->ImageWidth() ||
      static_cast<unsigned int>(_mousePos.Y()) >= this->ImageHeight())
  {
    return VisualPtr();
  }

  // The query is answered by the next render of this view. Its callback can
  // fire after this function returns if the driver is slow, so the result
  // lives in a shared state rather than on the stack.
  struct Result { ::utils::Entity entity; bool done{false}; };
  auto result = std::make_shared<Result>();
  // filament picking coordinates have their origin at the bottom-left
  this->view->pick(static_cast<uint32_t>(_mousePos.X()),
      this->ImageHeight() - 1u - static_cast<uint32_t>(_mousePos.Y()),
      [result](const filament::View::PickingQueryResult &_r)
      {
        result->entity = _r.renderable;
        result->done = true;
      });
  this->PreRender();
  this->Render();
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
  for (int i = 0; i < 10 && !result->done; ++i)
  {
    engine->flushAndWait();
    engine->pumpMessageQueues();
  }
  if (!result->done || result->entity.isNull())
    return VisualPtr();

  for (FilamentMesh *mesh : this->scene->Meshes())
  {
    if (mesh->Entity() == result->entity)
      return mesh->Parent();
  }
  return VisualPtr();
}

//////////////////////////////////////////////////
void FilamentCamera::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->renderTexture = std::dynamic_pointer_cast<FilamentRenderTexture>(base);
  this->renderTexture->SetFormat(PF_R8G8B8);
  this->renderTexture->SetWidth(this->ImageWidth());
  this->renderTexture->SetHeight(this->ImageHeight());
}

//////////////////////////////////////////////////
void FilamentCamera::SetClampedProjection(double _renderNear,
    double _renderFar)
{
  using filament::Camera;
  const double hfovDeg = this->HFOV().Degree();
  const double aspect = this->AspectRatio();
  this->camera->setCustomProjection(
      Camera::projection(Camera::Fov::HORIZONTAL, hfovDeg, aspect,
          _renderNear, _renderFar),
      Camera::projection(Camera::Fov::HORIZONTAL, hfovDeg, aspect,
          this->NearClipPlane(), this->FarClipPlane()),
      this->NearClipPlane(), this->FarClipPlane());
}

//////////////////////////////////////////////////
void FilamentCamera::UpdateProjection()
{
  if (!this->camera)
    return;
  const double aspect = this->AspectRatio();
  const double hfovDeg = this->HFOV().Degree();
  this->camera->setProjection(hfovDeg, aspect, this->NearClipPlane(),
      this->FarClipPlane(), filament::Camera::Fov::HORIZONTAL);
}

//////////////////////////////////////////////////
void FilamentCamera::Render()
{
  this->UpdateProjection();
  const math::Color bg = this->scene->BackgroundColor();
  this->renderTexture->SetClearColor(FilamentConversions::ToFloat4(bg));
  this->view->setVisibleLayers(0xff,
      FilamentConversions::ToLayers(this->VisibilityMask()));
  this->renderTexture->Render(this->view);
  this->renderTexture->ApplyRenderPasses(this->renderTexture->RenderPasses());
}
