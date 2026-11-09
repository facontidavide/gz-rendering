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

// Sensor parity dump: builds a fixed scene, renders depth cameras (depth +
// point cloud) and GPU lidars with the given engine and writes the raw float
// outputs plus per-sensor timings to <outdir>. Compare two engines with
// filament/tools/parity_compare.py.
//
// usage: filament_parity <engine> <outdir> [timingFrames] [engineParam]
// engineParam is set to "1", e.g. "opengl" for filament

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>

#include <gz/rendering/DepthCamera.hh>
#include <gz/rendering/GpuRays.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/SegmentationCamera.hh>
#include <gz/rendering/Visual.hh>

using namespace gz;
using namespace rendering;

namespace
{
  /// \brief Counter used to give every other object a segmentation label
  int g_added = 0;

  void Add(ScenePtr _scene, GeometryPtr _geom, const math::Pose3d &_pose,
      const math::Vector3d &_scale)
  {
    VisualPtr v = _scene->CreateVisual();
    v->AddGeometry(_geom);
    v->SetLocalPose(_pose);
    v->SetLocalScale(_scale);
    // labels 1..3 on most objects, some left unlabeled (background)
    if (g_added++ % 4 != 0)
      v->SetUserData("label", 1 + g_added % 3);
    _scene->RootVisual()->AddChild(v);
  }

  void Write(const std::string &_dir, const std::string &_name,
      const float *_data, unsigned int _w, unsigned int _h, unsigned int _c,
      std::ofstream &_index)
  {
    std::ofstream f(_dir + "/" + _name + ".f32", std::ios::binary);
    f.write(reinterpret_cast<const char *>(_data),
        static_cast<std::streamsize>(sizeof(float)) * _w * _h * _c);
    _index << _name << " " << _w << " " << _h << " " << _c << "\n";
  }
  void BuildScene(ScenePtr _scene)
  {
    // floor and a back wall (the wall sits inside every far clip)
    Add(_scene, _scene->CreateBox(), {0, 0, -0.05, 0, 0, 0}, {20, 20, 0.1});
    Add(_scene, _scene->CreateBox(), {7, 0, 1.5, 0, 0, 0}, {0.2, 10, 3});
    // primitives at assorted poses, near and far
    Add(_scene, _scene->CreateBox(), {2, 0.5, 0.5, 0, 0, 0.4}, {1, 1, 1});
    Add(_scene, _scene->CreateBox(), {3.5, -1.5, 0.8, 0.3, 0.5, 0.9},
        {0.6, 1.2, 0.8});
    Add(_scene, _scene->CreateSphere(), {2.5, -0.8, 1.2, 0, 0, 0},
        {0.8, 0.8, 0.8});
    Add(_scene, _scene->CreateCylinder(), {4, 1.8, 0.75, 0, 0, 0},
        {0.5, 0.5, 1.5});
    Add(_scene, _scene->CreateCylinder(), {-2, 1, 0.3, GZ_PI / 2, 0, 0.3},
        {0.4, 0.4, 2});
    Add(_scene, _scene->CreateSphere(), {-1.5, -2, 0.6, 0, 0, 0},
        {1.2, 1.2, 1.2});
    Add(_scene, _scene->CreateBox(), {0, -3, 1, 0, 0, 0.7}, {2, 0.3, 2});
    // thin pole and an object closer than some near clips
    Add(_scene, _scene->CreateCylinder(), {1.2, -0.6, 1, 0, 0, 0},
        {0.05, 0.05, 2});
    Add(_scene, _scene->CreateBox(), {0.35, 0.3, 0.9, 0, 0, 0},
        {0.1, 0.1, 0.1});
    // a two-link model: one panoptic instance spread over child visuals
    VisualPtr model = _scene->CreateVisual("model");
    model->SetLocalPosition(3, 0.8, 0);
    for (int i = 0; i < 2; ++i)
    {
      VisualPtr link = _scene->CreateVisual();
      link->AddGeometry(_scene->CreateBox());
      link->SetLocalPosition(0, 0.5 * i, 0.3 + 0.6 * i);
      link->SetLocalScale(0.4, 0.4, 0.6);
      link->SetUserData("label", 7);
      model->AddChild(link);
    }
    _scene->RootVisual()->AddChild(model);
  }

  void WriteBytes(const std::string &_dir, const std::string &_name,
      const uint8_t *_data, unsigned int _w, unsigned int _h,
      std::ofstream &_index)
  {
    std::vector<float> f(_data, _data + static_cast<size_t>(_w) * _h * 3);
    Write(_dir, _name, f.data(), _w, _h, 3, _index);
  }



  /// \brief Median and p90 wall time of _frames calls, in ms
  std::pair<double, double> Time(const std::function<void()> &_f, int _frames)
  {
    for (int i = 0; i < 5; ++i)
      _f();
    std::vector<double> t;
    for (int i = 0; i < _frames; ++i)
    {
      auto t0 = std::chrono::steady_clock::now();
      _f();
      t.push_back(std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t0).count());
    }
    std::sort(t.begin(), t.end());
    return {t[t.size() / 2], t[t.size() * 9 / 10]};
  }

  struct DepthCfg
  {
    std::string name; math::Pose3d pose; unsigned int w, h;
    double hfov, nearClip, farClip;
  };

  struct RaysCfg
  {
    std::string name; math::Pose3d pose;
    unsigned int hCount, vCount;
    double hMin, hMax, vMin, vMax, nearClip, farClip; bool clamp;
  };
}

int main(int _argc, char **_argv)
{
  if (_argc < 3)
  {
    std::cerr << "usage: filament_parity <engine> <outdir> [frames] [param]\n";
    return 1;
  }
  common::Console::SetVerbosity(2);
  const std::string dir = _argv[2];
  const int frames = _argc > 3 ? std::atoi(_argv[3]) : 50;
  std::map<std::string, std::string> params;
  if (_argc > 4)
    params[_argv[4]] = "1";
  RenderEngine *engine = rendering::engine(_argv[1], params);
  if (!engine)
    return 1;
  ScenePtr scene = engine->CreateScene("scene");
  BuildScene(scene);
  std::ofstream index(dir + "/index.txt");
  std::ofstream timing(dir + "/timing.txt");

  const std::vector<DepthCfg> depthCfgs = {
    {"depthA", {0, 0, 1, 0, 0.2, 0}, 320, 240, GZ_PI / 2, 0.1, 10},
    {"depthB", {-1, 2, 0.5, 0, 0.1, -0.6}, 640, 480, GZ_PI / 3, 0.5, 6},
    {"depthC", {0, 0, 0.9, 0.2, 0.3, 0.4}, 256, 256, 1.2, 0.3, 4},
  };
  for (const auto &c : depthCfgs)
  {
    DepthCameraPtr cam = scene->CreateDepthCamera(c.name);
    cam->SetLocalPose(c.pose);
    cam->SetImageWidth(c.w);
    cam->SetImageHeight(c.h);
    cam->SetAspectRatio(static_cast<double>(c.w) / c.h);
    cam->SetHFOV(c.hfov);
    cam->SetNearClipPlane(c.nearClip);
    cam->SetFarClipPlane(c.farClip);
    cam->CreateDepthTexture();
    scene->RootVisual()->AddChild(cam);

    std::vector<float> depth(c.w * c.h), cloud(c.w * c.h * 4);
    auto dc = cam->ConnectNewDepthFrame(
        [&](const float *_d, unsigned int, unsigned int, unsigned int,
            const std::string &) {
          std::memcpy(depth.data(), _d, depth.size() * sizeof(float)); });
    auto t = Time([&] { cam->Update(); }, frames);
    timing << c.name << "_depth " << t.first << " " << t.second << "\n";
    auto pc = cam->ConnectNewRgbPointCloud(
        [&](const float *_d, unsigned int, unsigned int, unsigned int,
            const std::string &) {
          std::memcpy(cloud.data(), _d, cloud.size() * sizeof(float)); });
    t = Time([&] { cam->Update(); }, frames);
    timing << c.name << "_depth+cloud " << t.first << " " << t.second << "\n";
    Write(dir, c.name + "_depth", depth.data(), c.w, c.h, 1, index);
    Write(dir, c.name + "_cloud", cloud.data(), c.w, c.h, 4, index);
    // sensors stay alive: ogre2 asserts in DestroySensor on depth cameras
    dc.reset();
    pc.reset();
  }

  {
    SegmentationCameraPtr seg = scene->CreateSegmentationCamera("seg");
    seg->SetLocalPose({0, 0, 1, 0, 0.2, 0});
    seg->SetImageWidth(320);
    seg->SetImageHeight(240);
    seg->SetAspectRatio(320.0 / 240.0);
    seg->SetHFOV(GZ_PI / 2);
    seg->SetBackgroundLabel(23);
    scene->RootVisual()->AddChild(seg);
    std::vector<uint8_t> img(320 * 240 * 3), labels(img.size());
    auto conn = seg->ConnectNewSegmentationFrame(
        [&](const uint8_t *_d, unsigned int, unsigned int, unsigned int,
            const std::string &) {
          std::memcpy(img.data(), _d, img.size()); });
    for (auto type : {SegmentationType::ST_SEMANTIC,
                      SegmentationType::ST_PANOPTIC})
    {
      const std::string t =
          type == SegmentationType::ST_SEMANTIC ? "sem" : "pan";
      seg->SetSegmentationType(type);
      seg->EnableColoredMap(false);
      auto tm = Time([&] { seg->Update(); }, frames);
      timing << "seg_" << t << " " << tm.first << " " << tm.second << "\n";
      WriteBytes(dir, "seg_" + t, img.data(), 320, 240, index);
      seg->EnableColoredMap(true);
      seg->Update();
      WriteBytes(dir, "seg_" + t + "_colored", img.data(), 320, 240, index);
      seg->LabelMapFromColoredBuffer(labels.data());
      WriteBytes(dir, "seg_" + t + "_fromcolored", labels.data(), 320, 240,
          index);
    }
    conn.reset();
  }

  const std::vector<RaysCfg> raysCfgs = {
    {"lidar2d", {0, 0, 0.5, 0, 0, 0}, 720, 1,
        -GZ_PI, GZ_PI, 0, 0, 0.1, 12, false},
    {"vlp16", {0, 0, 1.0, 0, 0, 0}, 1800, 16,
        -GZ_PI, GZ_PI, -0.2618, 0.2618, 0.3, 10, false},
    {"vlp16clamp", {0, 0, 1.0, 0, 0, 0}, 1800, 16,
        -GZ_PI, GZ_PI, -0.2618, 0.2618, 0.3, 10, true},
    {"tilted", {0.5, -0.5, 1.2, 0.3, 0.2, 0.5}, 128, 64,
        -GZ_PI / 4, GZ_PI / 4, -0.5236, 0.1745, 0.2, 8, false},
    {"os1_64", {0, 0, 1.2, 0, 0, 0}, 1024, 64,
        -GZ_PI, GZ_PI, -0.3927, 0.3927, 0.3, 20, false},
  };
  for (const auto &c : raysCfgs)
  {
    GpuRaysPtr rays = scene->CreateGpuRays(c.name);
    rays->SetLocalPose(c.pose);
    rays->SetNearClipPlane(c.nearClip);
    rays->SetFarClipPlane(c.farClip);
    rays->SetClamp(c.clamp);
    rays->SetAngleMin(c.hMin);
    rays->SetAngleMax(c.hMax);
    rays->SetRayCount(c.hCount);
    rays->SetVerticalRayCount(c.vCount);
    if (c.vCount > 1)
    {
      rays->SetVerticalAngleMin(c.vMin);
      rays->SetVerticalAngleMax(c.vMax);
    }
    scene->RootVisual()->AddChild(rays);

    const unsigned int ch = rays->Channels();
    std::vector<float> scan(c.hCount * c.vCount * ch);
    auto conn = rays->ConnectNewGpuRaysFrame(
        [&](const float *_d, unsigned int, unsigned int, unsigned int,
            const std::string &) {
          std::memcpy(scan.data(), _d, scan.size() * sizeof(float)); });
    auto t = Time([&] { rays->Update(); }, frames);
    timing << c.name << " " << t.first << " " << t.second << "\n";
    // keep the range channel only
    std::vector<float> range(c.hCount * c.vCount);
    for (size_t i = 0; i < range.size(); ++i)
      range[i] = scan[i * ch];
    Write(dir, c.name, range.data(), c.hCount, c.vCount, 1, index);
    conn.reset();
  }

  engine->DestroyScene(scene);
  std::cout << "wrote " << dir << std::endl;
  return 0;
}
