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
#include <limits>
#include <map>
#include <memory>
#include <vector>

#include <filament/Box.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/RenderableManager.h>
#include <filament/VertexBuffer.h>
#include <geometry/SurfaceOrientation.h>
#include <math/quat.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <gz/common/Console.hh>
#include <gz/common/Material.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/SubMesh.hh>

#include "gz/rendering/Material.hh"
#include "gz/rendering/filament/FilamentMaterial.hh"
#include "gz/rendering/filament/FilamentMesh.hh"
#include "gz/rendering/filament/FilamentMeshFactory.hh"
#include "gz/rendering/filament/FilamentRenderEngine.hh"
#include "gz/rendering/filament/FilamentScene.hh"

using namespace gz;
using namespace rendering;

namespace
{
  /// \brief GPU buffers of one common::SubMesh, shared by all the
  /// visuals that use it: one vertex array per distinct mesh instead of one
  /// per visual is ~2.5 ms/frame at 2500 boxes (GL and Vulkan)
  struct SharedGeometry
  {
    /// \brief Pins the source so its address (the cache key) is not reused
    std::shared_ptr<common::SubMesh> source;
    filament::VertexBuffer *vb{nullptr};
    filament::IndexBuffer *ib{nullptr};
    filament::math::float3 bmin;
    filament::math::float3 bmax;
    ~SharedGeometry()
    {
      auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();
      if (engine)
      {
        engine->destroy(this->vb);
        engine->destroy(this->ib);
      }
    }
  };

  // ponytail: keyed by SubMesh address, so editing a common::SubMesh in
  // place after a visual was built from it is not picked up by later
  // visuals; key on a content hash if that ever matters
  std::map<const common::SubMesh *, std::weak_ptr<SharedGeometry>> &Cache()
  {
    static std::map<const common::SubMesh *,
        std::weak_ptr<SharedGeometry>> cache;
    return cache;
  }

  using filament::math::float2;
  using filament::math::float3;
  using filament::math::float4;
  using filament::math::quatf;
  using filament::math::uint3;

  template <typename T>
  void FreeBuffer(void *_buf, size_t, void *)
  {
    delete[] static_cast<T *>(_buf);
  }

  bool PrimitiveType(common::SubMesh::PrimitiveType _in,
      filament::RenderableManager::PrimitiveType &_out)
  {
    switch (_in)
    {
      case common::SubMesh::TRIANGLES:
        _out = filament::RenderableManager::PrimitiveType::TRIANGLES;
        return true;
      case common::SubMesh::LINES:
        _out = filament::RenderableManager::PrimitiveType::LINES;
        return true;
      case common::SubMesh::POINTS:
        _out = filament::RenderableManager::PrimitiveType::POINTS;
        return true;
      default:
        return false;
    }
  }
}

//////////////////////////////////////////////////
FilamentMeshFactory::FilamentMeshFactory(FilamentScenePtr _scene)
  : scene(_scene)
{
}

//////////////////////////////////////////////////
FilamentMeshFactory::~FilamentMeshFactory()
{
}

//////////////////////////////////////////////////
bool FilamentMeshFactory::Load(FilamentMeshPtr _mesh,
    const MeshDescriptor &_desc)
{
  MeshDescriptor desc = _desc;
  desc.Load();
  if (!desc.mesh)
  {
    gzerr << "Cannot load null mesh [" << _desc.meshName << "]" << std::endl;
    return false;
  }
  const common::Mesh &common = *desc.mesh;
  auto *engine = FilamentRenderEngine::Instance()->FilamentEngine();

  struct Prim
  {
    FilamentSubMeshPtr sub;
    filament::RenderableManager::PrimitiveType type;
    MaterialPtr material;
  };
  std::vector<Prim> prims;

  float3 bmin{std::numeric_limits<float>::max()};
  float3 bmax{std::numeric_limits<float>::lowest()};

  for (unsigned int i = 0; i < common.SubMeshCount(); ++i)
  {
    auto s = common.SubMeshByIndex(i).lock();
    if (!s)
      continue;
    if (!desc.subMeshName.empty() && s->Name() != desc.subMeshName)
      continue;

    filament::RenderableManager::PrimitiveType type;
    if (!PrimitiveType(s->SubMeshPrimitiveType(), type))
    {
      gzwarn << "Skipping submesh [" << s->Name()
             << "] with unsupported primitive type" << std::endl;
      continue;
    }
    const unsigned int nv = s->VertexCount();
    const unsigned int ni = s->IndexCount();
    if (nv == 0u || ni == 0u)
      continue;

    std::shared_ptr<SharedGeometry> geom = Cache()[s.get()].lock();
    if (!geom)
    {
      geom = std::make_shared<SharedGeometry>();
      geom->source = s;
      geom->bmin = float3{std::numeric_limits<float>::max()};
      geom->bmax = float3{std::numeric_limits<float>::lowest()};
      // ponytail: centerSubMesh ignored (only used by some mesh viewers)
      const math::Vector3d center = math::Vector3d::Zero;

      // positions
      auto *pos = new float3[nv];
      for (unsigned int j = 0; j < nv; ++j)
      {
        math::Vector3d v = s->Vertex(j) - center;
        pos[j] = {static_cast<float>(v.X()), static_cast<float>(v.Y()),
                  static_cast<float>(v.Z())};
        geom->bmin = min(geom->bmin, pos[j]);
        geom->bmax = max(geom->bmax, pos[j]);
      }

      // indices
      auto *idx = new uint32_t[ni];
      for (unsigned int j = 0; j < ni; ++j)
        idx[j] = static_cast<uint32_t>(s->Index(j));

      // tangent frames (quaternions) from normals, or from geometry
      auto *tangents = new float4[nv];
      {
        filament::geometry::SurfaceOrientation::Builder builder;
        builder.vertexCount(nv);
        std::vector<float3> normals;
        if (s->NormalCount() == nv)
        {
          normals.resize(nv);
          for (unsigned int j = 0; j < nv; ++j)
          {
            math::Vector3d n = s->Normal(j);
            normals[j] = {static_cast<float>(n.X()), static_cast<float>(n.Y()),
                          static_cast<float>(n.Z())};
          }
          builder.normals(normals.data());
        }
        else if (type == filament::RenderableManager::PrimitiveType::TRIANGLES)
        {
          builder.positions(pos);
          builder.triangleCount(ni / 3u);
          builder.triangles(reinterpret_cast<const uint3 *>(idx));
        }
        else
        {
          // lines/points: an arbitrary frame is fine
          normals.assign(nv, float3{0.0f, 0.0f, 1.0f});
          builder.normals(normals.data());
        }
        auto *orientation = builder.build();
        std::vector<quatf> quats(nv);
        orientation->getQuats(quats.data(), nv);
        delete orientation;
        for (unsigned int j = 0; j < nv; ++j)
          tangents[j] = {quats[j].x, quats[j].y, quats[j].z, quats[j].w};
      }

      // texture coordinates (flip V: gz/Ogre are top-left, filament is GL)
      auto *uv = new float2[nv];
      const bool hasUv = s->TexCoordCount() == nv;
      for (unsigned int j = 0; j < nv; ++j)
      {
        if (hasUv)
        {
          math::Vector2d t = s->TexCoord(j);
          uv[j] = {static_cast<float>(t.X()), 1.0f - static_cast<float>(t.Y())};
        }
        else
        {
          uv[j] = {0.0f, 0.0f};
        }
      }

      using AT = filament::VertexBuffer::AttributeType;
      using VA = filament::VertexAttribute;
      geom->vb = filament::VertexBuffer::Builder()
          .vertexCount(nv)
          .bufferCount(3)
          .attribute(VA::POSITION, 0, AT::FLOAT3)
          .attribute(VA::TANGENTS, 1, AT::FLOAT4)
          .attribute(VA::UV0, 2, AT::FLOAT2)
          .build(*engine);
      geom->vb->setBufferAt(*engine, 0, filament::VertexBuffer::BufferDescriptor(
          pos, sizeof(float3) * nv, &FreeBuffer<float3>));
      geom->vb->setBufferAt(*engine, 1, filament::VertexBuffer::BufferDescriptor(
          tangents, sizeof(float4) * nv, &FreeBuffer<float4>));
      geom->vb->setBufferAt(*engine, 2, filament::VertexBuffer::BufferDescriptor(
          uv, sizeof(float2) * nv, &FreeBuffer<float2>));

      geom->ib = filament::IndexBuffer::Builder()
          .indexCount(ni)
          .bufferType(filament::IndexBuffer::IndexType::UINT)
          .build(*engine);
      geom->ib->setBuffer(*engine, filament::IndexBuffer::BufferDescriptor(
          idx, sizeof(uint32_t) * ni, &FreeBuffer<uint32_t>));
      Cache()[s.get()] = geom;
    }
    bmin = min(bmin, geom->bmin);
    bmax = max(bmax, geom->bmax);

    FilamentSubMeshPtr sub(new FilamentSubMesh);
    sub->vertexBuffer = geom->vb;
    sub->indexBuffer = geom->ib;
    sub->geometry = geom;
    sub->mesh = _mesh.get();
    sub->index = prims.size();

    // material: from the imported mesh if any, else a default one
    MaterialPtr material;
    if (const auto matIdx = s->GetMaterialIndex())
    {
      common::MaterialPtr commonMat = common.MaterialByIndex(matIdx.value());
      if (commonMat)
      {
        material = this->scene->CreateMaterial();
        material->CopyFrom(*commonMat);
      }
    }
    if (!material)
    {
      // ogre2 draws material-less meshes with its default datablock: flat
      // unlit white
      material = this->scene->CreateMaterial();
      material->SetLightingEnabled(false);
    }

    prims.push_back({sub, type, material});
  }

  if (prims.empty())
  {
    gzerr << "Mesh [" << _desc.meshName << "] has no usable submeshes"
          << std::endl;
    return false;
  }

  filament::Box box;
  box.set(bmin, bmax);
  filament::RenderableManager::Builder builder(prims.size());
  builder.boundingBox(box)
      .culling(true)
      .castShadows(true)
      .receiveShadows(true);
  for (size_t i = 0; i < prims.size(); ++i)
  {
    builder.geometry(i, prims[i].type, prims[i].sub->vertexBuffer,
        prims[i].sub->indexBuffer);
  }
  builder.build(*engine, _mesh->entity);
  // BaseMesh::Clone recreates the mesh from it
  _mesh->meshDescriptor = _desc;

  for (auto &p : prims)
  {
    _mesh->AddSubMesh(p.sub);
    // created for this submesh only: owned (destroyed when replaced), like
    // the clone ogre2's SetMaterial(mat) makes
    p.sub->SetMaterial(p.material, false);
    p.sub->ownsMaterial = true;
  }
  return true;
}
