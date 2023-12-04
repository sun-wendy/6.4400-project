#include "SplineViewerApp.hpp"

#include <fstream>

#include "gloo/cameras/ArcBallCameraNode.hpp"
#include "gloo/lights/AmbientLight.hpp"
#include "gloo/lights/PointLight.hpp"
#include "gloo/components/LightComponent.hpp"

#include "CurveNode.hpp"
#include "PatchNode.hpp"
#include "Surface.hpp"
#include "NURBSNode.hpp"

namespace GLOO {

SplineViewerApp::SplineViewerApp(const std::string& app_name,
                                 glm::ivec2 window_size,
                                 const std::string& filename)
    : Application(app_name, window_size), filename_(filename) {
}

void SplineViewerApp::SetupScene() {
  SceneNode& root = scene_->GetRootNode();

  std::cout << "HERE" << std::endl;

  LoadFile(filename_, root);

  auto camera_node = make_unique<ArcBallCameraNode>();
  scene_->ActivateCamera(camera_node->GetComponentPtr<CameraComponent>());
  root.AddChild(std::move(camera_node));

  auto ambient_light = std::make_shared<AmbientLight>();
  ambient_light->SetAmbientColor(glm::vec3(0.7f));
  root.CreateComponent<LightComponent>(ambient_light);

  auto point_light = std::make_shared<PointLight>();
  point_light->SetDiffuseColor(glm::vec3(0.9f, 0.9f, 0.9f));
  point_light->SetSpecularColor(glm::vec3(1.0f, 1.0f, 1.0f));
  point_light->SetAttenuation(glm::vec3(1.0f, 0.09f, 0.032f));
  auto point_light_node = make_unique<SceneNode>();
  point_light_node->CreateComponent<LightComponent>(point_light);
  point_light_node->GetTransform().SetPosition(glm::vec3(0.0f, 4.0f, 5.f));
  root.AddChild(std::move(point_light_node));
}

void SplineViewerApp::LoadFile(const std::string& filename, SceneNode& root) {
  std::fstream fs(GetAssetDir() + filename);
  if (!fs) {
    std::cerr << "ERROR: Unable to open file " + filename + "!" << std::endl;
    return;
  }

  std::string spline_type;
  std::getline(fs, spline_type);

  SplineBasis spline_basis;
  std::vector<glm::vec3> control_points;
  std::vector<float> knots;
  int degree;
  std::vector<float> weights;

  std::string line;
  while (std::getline(fs, line)) {
    if (line == "control points") {
      while (std::getline(fs, line)) {
        if (line == "knots") {
          break;
        }
        std::stringstream ss(line);
        float x, y, z, w;
        ss >> x >> y >> z >> w;
        control_points.push_back(glm::vec3(x, y, z));
        weights.push_back(w);
      }
    }
    if (line == "knots") {
      std::getline(fs, line);
      std::stringstream ss(line);
      float knot;
      while (ss >> knot) {
        knots.push_back(knot);
      }
    }
    if (line == "degree") {
      std::getline(fs, line);
      std::stringstream ss(line);
      ss >> degree;
    }
  }

  // For debugging
  std::cout << "Control points: " << std::endl;
  for (size_t i = 0; i < control_points.size(); i++) {
    std::cout << control_points[i].x << " " << control_points[i].y << " " << control_points[i].z << std::endl;
  }
  std::cout << "Knots: " << std::endl;
  for (size_t i = 0; i < knots.size(); i++) {
    std::cout << knots[i] << std::endl;
  }
  std::cout << "Degree: " << degree << std::endl;
  std::cout << "Weights: " << std::endl;
  for (size_t i = 0; i < weights.size(); i++) {
    std::cout << weights[i] << std::endl;
  }

  // Set up a NURBS node for the loaded file
  auto nurbs_node = make_unique<NURBSNode>(degree, control_points, weights, knots, NURBSBasis::NURBS);
  root.AddChild(std::move(nurbs_node));

  // std::string line;
  // for (size_t i = 0; std::getline(fs, line); i++) {
  //   std::stringstream ss(line);
  //   float x, y, z;
  //   ss >> x >> y >> z;
  //   control_points.push_back(glm::vec3(x, y, z));
  // }

  // SplineBasis spline_basis;
  // if (spline_type == "Bezier curve" or spline_type == "Bezier patch") {
  //   spline_basis = SplineBasis::Bezier;
  // } else if (spline_type == "B-Spline curve" or spline_type == "B-Spline patch") {
  //   spline_basis = SplineBasis::BSpline;
  // } else {
  //   std::cerr << "ERROR: Spline basis type invalid" << std::endl;
  //   return;
  // }

  // TODO: set up patch or curve nodes here.
  // The first line of the user-specified file is spline_type, and the specified
  // control points are in control_points, a std::vector of glm::vec3 objects.
  // Depending on the specified spline type, create the appropriate node(s)
  // parameterized by the control points.

  // if (spline_type == "Bezier curve" or spline_type == "B-Spline curve") {  // Curve
  //     if (control_points.size() > 4) {  // Multiple curves
  //       std::vector<glm::vec3> new_control_points;
  //       if (spline_basis == SplineBasis::Bezier) {
  //         // Beizer
  //         int num_curves = (control_points.size() + (control_points.size() / 4)) / 4;
  //         for (size_t i = 0; i < num_curves; i++) {
  //           new_control_points = {control_points[i*3], control_points[i*3+1], control_points[i*3+2], control_points[i*3+3]};
  //           auto curve_node = make_unique<CurveNode>(new_control_points, spline_basis);
  //           root.AddChild(std::move(curve_node));
  //         }
  //       } else {
  //         // B-Spline
  //         for (size_t i = 0; i < control_points.size() - 3; i++) {
  //           new_control_points = {control_points[i], control_points[i+1], control_points[i+2], control_points[i+3]};
  //           auto curve_node = make_unique<CurveNode>(new_control_points, spline_basis);
  //           root.AddChild(std::move(curve_node));
  //         }
  //       }
  //   } else {  // Single curve
  //     auto curve_node = make_unique<CurveNode>(control_points, spline_basis);
  //     root.AddChild(std::move(curve_node));
  //   }
  // } else {  // Patch
  //     auto surface = make_unique<Surface>();
  //     for (size_t i = 0; i < control_points.size() - 15; i += 16) {
  //       std::vector<glm::vec3> new_control_points;
  //       for (size_t j = 0; j < 16; j++) {
  //         new_control_points.push_back(control_points[i+j]);
  //       }
  //       auto patch_node = make_unique<PatchNode>(new_control_points, spline_basis);
  //       // root.AddChild(std::move(patch_node));
  //       surface->AddChild(std::move(patch_node));
  //     }
  //     root.AddChild(std::move(surface));
  // }
}
}  // namespace GLOO
