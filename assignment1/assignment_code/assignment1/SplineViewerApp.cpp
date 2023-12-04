#include "SplineViewerApp.hpp"

#include <fstream>

#include "gloo/external.hpp" // take in user inputs
#include "gloo/cameras/ArcBallCameraNode.hpp"
#include "gloo/lights/AmbientLight.hpp"
#include "gloo/lights/PointLight.hpp"
#include "gloo/components/LightComponent.hpp"

#include "CurveNode.hpp"
#include "PatchNode.hpp"
#include "Surface.hpp"
#include "NURBSNode.hpp"


namespace {
const std::vector<std::string> kJointNames = {"Root",
                                              "Chest",
                                              "Waist",
                                              "Neck",
                                              "Right hip",
                                              "Right leg",
                                              "Right knee",
                                              "Right foot",
                                              "Left hip",
                                              "Left leg",
                                              "Left knee",
                                              "Left foot",
                                              "Right collarbone",
                                              "Right shoulder",
                                              "Right elbow",
                                              "Left collarbone",
                                              "Left shoulder",
                                              "Left elbow"};
}

namespace GLOO {

SplineViewerApp::SplineViewerApp(const std::string& app_name,
                                 glm::ivec2 window_size,
                                 const std::string& filename)
    : Application(app_name, window_size), filename_(filename), slider_values_(kJointNames.size(), 0.0f) {
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
  std::vector<float> knots;
  int degree;
  // std::vector<float> weights;

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
        weights_.push_back(w);
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

  // Set up a NURBS node for the loaded file
  auto nurbs_node = make_unique<NURBSNode>(degree, control_points, weights_, knots, NURBSBasis::NURBS);
  nurbs_node_ptr_ = nurbs_node.get();
  root.AddChild(std::move(nurbs_node));
}

void SplineViewerApp::DrawGUI() {
  bool modified = false;
  ImGui::Begin("Control Panel");
  for (size_t i = 0; i < control_points.size(); i++) {
    ImGui::Text("Ctrl Pt %i", i);
    ImGui::PushID((int)i);
    modified |= ImGui::SliderFloat("Weight", &weights_[i], 0, 10);
    ImGui::PopID();
  }
  ImGui::End();
  if (modified) {
    nurbs_node_ptr_->OnWeightChanged(weights_);
      // std::cout << "1: " << slider_values_[0] << std::endl;
    // skeletal_node_ptr_->OnJointChanged();
  }
}
}  // namespace GLOO

