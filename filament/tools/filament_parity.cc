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
// PARITY_WIDEANGLE=1: only the wide-angle camera geometry checks (RGB images,
// per-object centroids, cutoff circle radius, Project3d), see WideAngle()

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <sstream>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/math/Pose3.hh>

#include <gz/rendering/BoundingBoxCamera.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/DepthCamera.hh>
#include <gz/rendering/Heightmap.hh>
#include <gz/rendering/Light.hh>
#include <gz/rendering/Image.hh>
#include <gz/rendering/GpuRays.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/SegmentationCamera.hh>
#include <gz/rendering/ThermalCamera.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/WideAngleCamera.hh>

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
    // assorted diffuse colors (thermal uses them for non heat sources),
    // every third object is a heat source
    MaterialPtr mat = _scene->CreateMaterial();
    mat->SetDiffuse(0.1 + 0.08 * g_added, 0.9 - 0.07 * g_added,
        0.3 + 0.05 * (g_added % 5));
    v->SetMaterial(mat);
    if (g_added % 3 == 0)
      v->SetUserData("temperature", 280.0f + 7.3f * g_added);
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
    // floor and a back wall (the wall sits inside every far clip); with
    // PARITY_HEIGHTMAP=<image> a 20 x 20 x 3 m heightmap replaces the floor,
    // its mid height at z = 0
    if (const char *hm = std::getenv("PARITY_HEIGHTMAP"))
    {
      auto data = std::make_shared<common::ImageHeightmap>();
      if (data->Load(hm) != 0)
        std::cerr << "cannot load heightmap " << hm << std::endl;
      HeightmapDescriptor desc;
      desc.SetData(data);
      desc.SetSize({20, 20, 3});
      desc.SetPosition({0, 0, -1.5});
      desc.SetSampling(2u);
      HeightmapTexture tex;
      tex.SetSize(2.0);
      tex.SetDiffuse(std::string(PROJECT_SOURCE_PATH) +
          "/test/media/materials/textures/texture.png");
      desc.AddTexture(tex);
      VisualPtr v = _scene->CreateVisual();
      v->AddGeometry(_scene->CreateHeightmap(desc));
      v->SetUserData("label", 9);
      _scene->RootVisual()->AddChild(v);
      ++g_added;
    }
    else
    {
      Add(_scene, _scene->CreateBox(), {0, 0, -0.05, 0, 0, 0}, {20, 20, 0.1});
    }
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

    // optional asset meshes (PARITY_MESHES=a.dae:b.obj:...), with their own
    // materials, scaled to ~0.9 m and set on the floor in an arc in view
    const char *list = std::getenv("PARITY_MESHES");
    std::vector<std::string> paths;
    std::stringstream ss(list ? list : "");
    for (std::string p; std::getline(ss, p, ':');)
      if (!p.empty())
        paths.push_back(p);
    for (size_t k = 0; k < paths.size(); ++k)
    {
      const common::Mesh *m = common::MeshManager::Instance()->Load(paths[k]);
      if (!m)
      {
        std::cerr << "cannot load " << paths[k] << std::endl;
        continue;
      }
      const math::Vector3d size = m->Max() - m->Min();
      const double scale = 0.9 / std::max({size.X(), size.Y(), size.Z()});
      const double a = paths.size() > 1 ?
          -0.7 + 1.4 * k / (paths.size() - 1) : 0.0;
      VisualPtr v = _scene->CreateVisual();
      v->AddGeometry(_scene->CreateMesh(paths[k]));
      v->SetLocalPose({1.8 + 0.8 * std::cos(a), 2.6 * std::sin(a),
          -m->Min().Z() * scale, 0, 0, 0.5 * k});
      v->SetLocalScale(scale);
      v->SetUserData("label", 4 + static_cast<int>(k % 2));
      if (k % 2 == 0)
        v->SetUserData("temperature", 300.0f + 4.1f * k);
      _scene->RootVisual()->AddChild(v);
    }
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

  /// \brief sRGB byte to linear [0, 1]
  double Linear(uint8_t _v)
  {
    const double x = _v / 255.0;
    return x <= 0.04045 ? x / 12.92 : std::pow((x + 0.055) / 1.055, 2.4);
  }

  /// \brief Wide-angle camera geometry, comparable across engines whose
  /// shading differs: for every lens, an RGB image of the main scene
  /// (wa_<lens>_rgb), then in an empty scene the cutoff circle radius along
  /// the center row on a white background (wa_cutoff.txt), the centroid and
  /// area of every sphere rendered alone on black next to Project3d of its
  /// center (wa_geom.txt), and an L8 image of the first lens (wa_l8).
  void WideAngle(RenderEngine *_engine, ScenePtr _scene,
      const std::string &_dir, std::ofstream &_index,
      std::ofstream &_timing, int _frames)
  {
    struct Cfg
    {
      std::string name; MappingFunctionType type; double hfov, cutoff;
      unsigned int w, h;
    };
    const std::vector<Cfg> cfgs = {
      // the integration test lens
      {"custom", MFT_CUSTOM, 2.6, GZ_PI, 320, 240},
      {"equidistant", MFT_EQUIDISTANT, 3.0, 1.4, 320, 320},
      // HFOV pi trips an assert in ogre2 (degenerate frustum)
      {"stereographic", MFT_STEREOGRAPHIC, 3.0, 1.3, 320, 320},
      {"equisolid", MFT_EQUISOLID_ANGLE, 2.8, 1.2, 480, 320},
      {"orthographic", MFT_ORTHOGRAPHIC, 2.4, 1.0, 320, 320},
    };
    auto make = [&](ScenePtr _s, const Cfg &_c, const std::string &_name)
    {
      WideAngleCameraPtr cam = _s->CreateWideAngleCamera(_name);
      CameraLens lens;
      if (_c.type == MFT_CUSTOM)
        lens.SetCustomMappingFunction(1.05, 4.0, AFT_TAN, 1.0, 0.0);
      lens.SetType(_c.type);
      lens.SetCutOffAngle(_c.cutoff);
      lens.SetScaleToHFOV(true);
      cam->SetLens(lens);
      cam->SetHFOV(_c.hfov);
      cam->SetImageWidth(_c.w);
      cam->SetImageHeight(_c.h);
      cam->SetAspectRatio(static_cast<double>(_c.w) / _c.h);
      _s->RootVisual()->AddChild(cam);
      return cam;
    };

    // lit main scene, for images worth looking at
    _scene->SetAmbientLight(0.3, 0.3, 0.3);
    _scene->SetBackgroundColor(0.35, 0.45, 0.6);
    DirectionalLightPtr sun = _scene->CreateDirectionalLight();
    sun->SetDirection(-0.5, 0.3, -1);
    sun->SetDiffuseColor(0.9, 0.9, 0.85);
    _scene->RootVisual()->AddChild(sun);
    for (const auto &c : cfgs)
    {
      WideAngleCameraPtr cam = make(_scene, c, "wa_" + c.name);
      cam->SetLocalPose({0, 0, 1, 0, 0.2, 0});
      Image img = cam->CreateImage();
      auto tm = Time([&] { cam->Capture(img); }, _frames);
      _timing << "wa_" << c.name << " " << tm.first << " " << tm.second
              << "\n";
      WriteBytes(_dir, "wa_" + c.name + "_rgb", img.Data<uint8_t>(), c.w,
          c.h, _index);
      if (&c == &cfgs[0])
      {
        WideAngleCameraPtr l8 = make(_scene, c, "wa_l8cam");
        l8->SetLocalPose({0, 0, 1, 0, 0.2, 0});
        l8->SetImageFormat(PF_L8);
        Image mono = l8->CreateImage();
        l8->Capture(mono);
        std::vector<float> f(mono.Data<uint8_t>(),
            mono.Data<uint8_t>() + c.w * c.h);
        Write(_dir, "wa_l8", f.data(), c.w, c.h, 1, _index);
      }
    }

    ScenePtr geom = _engine->CreateScene("wa_geom");
    geom->SetAmbientLight(1, 1, 1);
    std::vector<WideAngleCameraPtr> cams;
    for (const auto &c : cfgs)
    {
      cams.push_back(make(geom, c, "wa_geom_" + c.name));
      cams.back()->SetLocalPose({0, 0, 0, 0.1, 0.15, 0.3});
    }

    // cutoff circle: left / right crossing of half intensity, center row
    std::ofstream cut(_dir + "/wa_cutoff.txt");
    cut.precision(10);
    geom->SetBackgroundColor(1, 1, 1);
    for (size_t k = 0; k < cfgs.size(); ++k)
    {
      Image img = cams[k]->CreateImage();
      cams[k]->Capture(img);
      const unsigned int w = cfgs[k].w;
      const uint8_t *row = img.Data<uint8_t>() + (cfgs[k].h / 2) * w * 3;
      auto cross = [&](int _from, int _step)
      {
        for (int x = _from; x >= 0 && x + _step >= 0 &&
             x + _step < static_cast<int>(w); x += _step)
        {
          const double a = Linear(row[x * 3]);
          const double b = Linear(row[(x + _step) * 3]);
          if (a >= 0.5 && b < 0.5)
            return x + 0.5 + _step * (a - 0.5) / (a - b);
        }
        return -1.0;
      };
      const double left = cross(w / 2, -1);
      const double right = cross(w / 2 - 1, 1);
      cut << cfgs[k].name << " " << left << " " << right << " "
          << (left < 0 || right < 0 ? -1.0 : (right - left) / 2) << "\n";
    }

    // spheres one at a time on black: centroid of the pixels brighter than
    // half the brightest one, next to Project3d of the sphere center
    geom->SetBackgroundColor(0, 0, 0);
    const std::vector<math::Vector3d> points = {
      {3, 0, 0}, {2, 2, 0.5}, {1, -2, 1}, {0.5, 1, -1.5}, {2, 0, 2},
      {-0.5, 2.5, 0.3}, {1, -1.2, -1.2}};
    std::ofstream out(_dir + "/wa_geom.txt");
    out.precision(10);
    MaterialPtr white = geom->CreateMaterial();
    white->SetAmbient(1, 1, 1);
    white->SetDiffuse(1, 1, 1);
    // flat color, so the masks do not depend on the shading model
    white->SetLightingEnabled(false);
    for (size_t p = 0; p < points.size(); ++p)
    {
      VisualPtr v = geom->CreateVisual();
      v->AddGeometry(geom->CreateSphere());
      v->SetLocalPosition(points[p]);
      v->SetLocalScale(0.6);
      v->SetMaterial(white);
      geom->RootVisual()->AddChild(v);
      for (size_t k = 0; k < cfgs.size(); ++k)
      {
        Image img = cams[k]->CreateImage();
        cams[k]->Capture(img);
        const uint8_t *d = img.Data<uint8_t>();
        const size_t n = static_cast<size_t>(cfgs[k].w) * cfgs[k].h;
        int maxSum = 0;
        for (size_t i = 0; i < n; ++i)
          maxSum = std::max(maxSum, d[i * 3] + d[i * 3 + 1] + d[i * 3 + 2]);
        double sx = 0, sy = 0, area = 0;
        for (size_t i = 0; i < n && maxSum > 0; ++i)
        {
          if (2 * (d[i * 3] + d[i * 3 + 1] + d[i * 3 + 2]) > maxSum)
          {
            sx += i % cfgs[k].w + 0.5;
            sy += i / cfgs[k].w + 0.5;
            area += 1;
          }
        }
        const math::Vector3d proj = cams[k]->Project3d(points[p]);
        out << cfgs[k].name << " " << p << " "
            << (area > 0 ? sx / area : -1) << " "
            << (area > 0 ? sy / area : -1) << " " << area << " "
            << proj.X() << " " << proj.Y() << " " << proj.Z() << "\n";
      }
      geom->DestroyVisual(v);
    }
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

  if (std::getenv("PARITY_WIDEANGLE"))
  {
    WideAngle(engine, scene, dir, index, timing, frames);
    std::cout << "wrote " << dir << std::endl;
    return 0;
  }

  {
    // RGB overview for eyeballing, not compared
    scene->SetAmbientLight(0.3, 0.3, 0.3);
    DirectionalLightPtr sun = scene->CreateDirectionalLight();
    sun->SetDirection(-0.5, -0.5, -1);
    sun->SetDiffuseColor(0.9, 0.9, 0.9);
    scene->RootVisual()->AddChild(sun);
    CameraPtr cam = scene->CreateCamera("rgb");
    cam->SetLocalPose({-8, 8, 5, 0, 0.4, -GZ_PI / 4});
    cam->SetImageWidth(640);
    cam->SetImageHeight(480);
    cam->SetAspectRatio(640.0 / 480.0);
    cam->SetHFOV(GZ_PI / 3);
    scene->RootVisual()->AddChild(cam);
    Image img = cam->CreateImage();
    cam->Capture(img);
    common::Image png;
    png.SetFromData(img.Data<unsigned char>(), 640, 480,
        common::Image::RGB_INT8);
    png.SavePNG(dir + "/rgb.png");
  }

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

  {
    BoundingBoxCameraPtr bb = scene->CreateBoundingBoxCamera("bbox");
    bb->SetLocalPose({0, 0, 1, 0, 0.2, 0});
    bb->SetImageWidth(320);
    bb->SetImageHeight(240);
    bb->SetAspectRatio(320.0 / 240.0);
    bb->SetHFOV(GZ_PI / 2);
    scene->RootVisual()->AddChild(bb);
    std::vector<BoundingBox> boxes;
    auto conn = bb->ConnectNewBoundingBoxes(
        [&](const std::vector<BoundingBox> &_b) { boxes = _b; });
    for (auto type : {BoundingBoxType::BBT_VISIBLEBOX2D,
                      BoundingBoxType::BBT_FULLBOX2D,
                      BoundingBoxType::BBT_BOX3D})
    {
      const std::string t = type == BoundingBoxType::BBT_VISIBLEBOX2D ?
          "visible2d" : type == BoundingBoxType::BBT_FULLBOX2D ? "full2d" :
          "box3d";
      bb->SetBoundingBoxType(type);
      auto tm = Time([&] { bb->Update(); }, frames);
      timing << "bbox_" << t << " " << tm.first << " " << tm.second << "\n";
      // one row per box: center xyz, size xyz, rotation wxyz, label
      std::vector<float> rows;
      for (const BoundingBox &b : boxes)
      {
        const math::Quaterniond q = b.Orientation();
        for (double v : {b.Center().X(), b.Center().Y(), b.Center().Z(),
             b.Size().X(), b.Size().Y(), b.Size().Z(), q.W(), q.X(), q.Y(),
             q.Z(), static_cast<double>(b.Label())})
          rows.push_back(static_cast<float>(v));
      }
      Write(dir, "bbox_" + t, rows.data(), 11, boxes.size(), 1, index);
      std::vector<uint8_t> img(320 * 240 * 3, 0);
      for (const BoundingBox &b : boxes)
        bb->DrawBoundingBox(img.data(), math::Color::Green, b);
      WriteBytes(dir, "bboxdraw_" + t, img.data(), 320, 240, index);
    }
    conn.reset();
  }

  for (PixelFormat fmt : {PF_L16, PF_L8})
  {
    const std::string name = fmt == PF_L16 ? "thermal16" : "thermal8";
    ThermalCameraPtr th = scene->CreateThermalCamera(name);
    th->SetLocalPose({0, 0, 1, 0, 0.2, 0});
    th->SetImageWidth(320);
    th->SetImageHeight(240);
    th->SetAspectRatio(320.0 / 240.0);
    th->SetHFOV(GZ_PI / 2);
    th->SetNearClipPlane(0.5);
    th->SetFarClipPlane(10);
    th->SetImageFormat(fmt);
    th->SetAmbientTemperature(296.0f);
    th->SetAmbientTemperatureRange(4.0f);
    th->SetHeatSourceTemperatureRange(2.0f);
    th->SetLinearResolution(fmt == PF_L16 ? 0.01f : 3.0f);
    th->SetMinTemperature(250.0f);
    th->SetMaxTemperature(fmt == PF_L16 ? 400.0f : 600.0f);
    scene->RootVisual()->AddChild(th);
    std::vector<uint16_t> img(320 * 240);
    auto conn = th->ConnectNewThermalFrame(
        [&](const uint16_t *_d, unsigned int, unsigned int, unsigned int,
            const std::string &) {
          std::memcpy(img.data(), _d, img.size() * sizeof(uint16_t)); });
    auto tm = Time([&] { th->Update(); }, frames);
    timing << name << " " << tm.first << " " << tm.second << "\n";
    std::vector<float> f(img.begin(), img.end());
    Write(dir, name, f.data(), 320, 240, 1, index);
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
