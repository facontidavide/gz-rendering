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

      /// \brief gz visibility flags (32 bits) to filament layers (8 bits).
      /// ponytail: bits 0-6 map 1:1, bits 7-31 all fold into layer 7; a
      /// camera masking only some of the high bits sees all of them.
      inline uint8_t ToLayers(uint32_t _flags)
      {
        return static_cast<uint8_t>((_flags & 0x7fu) |
            ((_flags & ~0x7fu) ? 0x80u : 0u));
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
