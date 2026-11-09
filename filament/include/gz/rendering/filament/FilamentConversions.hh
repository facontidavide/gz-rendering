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
#ifndef GZ_RENDERING_FILAMENT_FILAMENTCONVERSIONS_HH_
#define GZ_RENDERING_FILAMENT_FILAMENTCONVERSIONS_HH_

#include <array>
#include <cmath>

#include <math/mat4.h>
#include <math/quat.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "gz/rendering/config.hh"

namespace gz
{
  namespace rendering
  {
    inline namespace GZ_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Conversion helpers between gz-math and filament::math types.
    namespace FilamentConversions
    {
      inline filament::math::float3 Convert(const math::Vector3d &_v)
      {
        return {static_cast<float>(_v.X()), static_cast<float>(_v.Y()),
                static_cast<float>(_v.Z())};
      }

      inline filament::math::quatf Convert(const math::Quaterniond &_q)
      {
        return {static_cast<float>(_q.W()), static_cast<float>(_q.X()),
                static_cast<float>(_q.Y()), static_cast<float>(_q.Z())};
      }

      /// \brief Local transform matrix T * R * S.
      inline filament::math::mat4f Convert(const math::Pose3d &_pose,
          const math::Vector3d &_scale)
      {
        return filament::math::mat4f::translation(Convert(_pose.Pos())) *
               filament::math::mat4f(Convert(_pose.Rot())) *
               filament::math::mat4f::scaling(Convert(_scale));
      }

      /// \brief gz colors are linear: ogre2 feeds them to the shaders as-is
      /// and renders into an sRGB target, so do the same here.
      inline filament::math::float3 ToFloat3(const math::Color &_c)
      {
        return {_c.R(), _c.G(), _c.B()};
      }

      inline filament::math::float4 ToFloat4(const math::Color &_c)
      {
        return {_c.R(), _c.G(), _c.B(), _c.A()};
      }

      /// \brief Filament layer of particle emitters. It stands for gz bit
      /// 0x00100000, ogre2's Ogre2ParticleEmitter::kParticleVisibilityFlags.
      constexpr uint8_t kParticleLayer = 0x80u;

      /// \brief gz visibility flags (32 bits) to filament layers (8 bits).
      /// Each gz bit gets its own layer the first time it is seen; the
      /// particle bit always maps to kParticleLayer.
      inline uint8_t ToLayers(uint32_t _flags)
      {
        // Values with many bits set (GZ_VISIBILITY_ALL and friends) mean
        // "everything": all layers, without allocating any.
        if (__builtin_popcount(_flags) > 8)
          return 0xffu;
        // ponytail: process-wide first-come allocation of gz bits to the
        // filament layers 0..6; bits past the 6th distinct one share layer
        // 6, so scenes using more than 6 distinct bits can over-render.
        // Upgrade to a per-scene table with re-layering if that ever matters.
        static std::array<int8_t, 32> layerOf = [] {
          std::array<int8_t, 32> a{};
          a.fill(-1);
          return a;
        }();
        static int8_t nextLayer = 0;
        constexpr uint32_t kParticleFlag = 0x00100000u;
        uint8_t layers = (_flags & kParticleFlag) ? kParticleLayer : 0u;
        for (int b = 0; b < 32; ++b)
        {
          if (!(_flags & (1u << b)) || (1u << b) == kParticleFlag)
            continue;
          if (layerOf[b] < 0)
            layerOf[b] = nextLayer < 6 ? nextLayer++ : 6;
          layers |= static_cast<uint8_t>(1u << layerOf[b]);
        }
        return layers;
      }

      /// \brief Layers seen by sensors that do not render particles as
      /// geometry (depth, lidar, thermal, segmentation, bounding boxes)
      inline uint8_t ToSensorLayers(uint32_t _flags)
      {
        return ToLayers(_flags) & static_cast<uint8_t>(~kParticleLayer);
      }

      /// \brief sRGB transfer function, for values written outside of a
      /// filament render pass (e.g. background-colored point cloud entries)
      inline double LinearToSrgb(double _x)
      {
        return _x < 0.0031308 ? _x * 12.92
                              : 1.055 * std::pow(_x, 1.0 / 2.4) - 0.055;
      }

      /// \brief Rotation from the gz camera frame (X forward, Z up) to the
      /// filament camera frame (-Z forward, Y up). Same matrix as the one
      /// used by BaseCamera::ViewMatrix().
      inline filament::math::mat4f CameraFix()
      {
        return filament::math::mat4f(
            filament::math::float4{0.0f, -1.0f, 0.0f, 0.0f},
            filament::math::float4{0.0f, 0.0f, 1.0f, 0.0f},
            filament::math::float4{-1.0f, 0.0f, 0.0f, 0.0f},
            filament::math::float4{0.0f, 0.0f, 0.0f, 1.0f});
      }
    }
    }
  }
}
#endif
