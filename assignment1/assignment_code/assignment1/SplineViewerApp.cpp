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


namespace GLOO {

SplineViewerApp::SplineViewerApp(const std::string& app_name,
                                 glm::ivec2 window_size,
                                 const std::string& filename)
    : Application(app_name, window_size), filename_(filename), slider_values_(10, 0.0f) {
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
  bool change_control_pt_selection = false;
  // int u8_one = 1;

  ImGui::Begin("Control Panel");
  for (size_t i = 0; i < control_points.size(); i++) {
    ImGui::Text("Ctrl Pt %zu", i);
    ImGui::PushID((int)i);
    // modified |= ImGui::SliderFloat("Weight", &weights_[i], 0, 10);
    modified |= ImGui::InputFloat("Weight", &weights_[i], 1.0, 1.0);
    ImGui::PopID();
  }
  ImGui::Text("Selected control point");
  ImGui::PushID((int)control_points.size());
  change_control_pt_selection |= ImGui::SliderInt("Selected point", &selected_control_pt, 0, control_points.size()-1);
  ImGui::PopID();

  // ImGui::SmallButton("Insert Circle");
  // ImGui::InputScalar("input u8",      ImGuiDataType_U8,     &u8_v, (int)1 ? &u8_one  : NULL, "%u");
  ImGui::End();

  if (modified) {
    nurbs_node_ptr_->OnWeightChanged(weights_);
  }
  if (change_control_pt_selection){
    nurbs_node_ptr_->ChangeSelectedControlPoint(selected_control_pt);
  }
}
}  // namespace GLOO

