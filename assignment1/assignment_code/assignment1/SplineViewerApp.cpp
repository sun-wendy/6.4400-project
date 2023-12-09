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
#include "NURBSCircle.hpp"
#include "NURBSSurface.hpp"


namespace GLOO {

SplineViewerApp::SplineViewerApp(const std::string& app_name,
                                 glm::ivec2 window_size,
                                 const std::string& filename)
    : Application(app_name, window_size), slider_values_(10, 0.0f), filename_(filename) {
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


  std::getline(fs, spline_type_);

  if (spline_type_ == "NURBS curve"){
    std::vector<float> knots;
    int degree;
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
    auto nurbs_node = make_unique<NURBSNode>(degree, control_points, weights_, knots, NURBSBasis::NURBS, 'R', true);
    nurbs_node_ptr_ = nurbs_node.get();
    root.AddChild(std::move(nurbs_node));
  } else{
      int degreeU;
      int degreeV;
      int numRows; // num rows
      int numCols; // num cols
      std::vector<float> knotsU;
      std::vector<float> knotsV;

      std::string line;
      while (std::getline(fs, line)) {
        if (line == "dimensions") {
          std::getline(fs, line);
          std::stringstream ss(line);
          int i, j;
          ss >> numRows >> numCols;
        }

        if (line == "control points") {
          while (std::getline(fs, line)) {
            if (line == "knots U") {
              break;
            }
            std::stringstream ss(line);
            float x, y, z, w;
            ss >> x >> y >> z >> w;
            control_points.push_back(glm::vec3(x, y, z));
            weights_.push_back(w);
          }
        }
        if (line == "knots U") {
          std::getline(fs, line);
          std::stringstream ss(line);
          float knot;
          while (ss >> knot) {
            knotsU.push_back(knot);
          }
        }
        if (line == "knots V") {
          std::getline(fs, line);
          std::stringstream ss(line);
          float knot;
          while (ss >> knot) {
            knotsV.push_back(knot);
          }
        }
        if (line == "degree") {
          std::getline(fs, line);
          std::stringstream ss(line);
          ss >> degreeU >> degreeV;
        }
      }

    // for debugging:
    std::cout << "Control points: " << std::endl;
    for (size_t i = 0; i < control_points.size(); i++) {
      std::cout << control_points[i].x << " " << control_points[i].y << " " << control_points[i].z << " " << weights_[i] << std::endl;
    }
    std::cout << "Num rows: " << numRows << std::endl;
    std::cout << "Num cols: " << numCols << std::endl;
    std::cout << "Degree U: " << degreeU << std::endl;
    std::cout << "Degree V: " << degreeV << std::endl;

    std::cout << "Knots U: " << std::endl;
    for (size_t i = 0; i < knotsU.size(); i++) {
      std::cout << knotsU[i] << ' ';
    }
    std::cout << std::endl;

    std::cout << "Knots V: " << std::endl;
    for (size_t i = 0; i < knotsV.size(); i++) {
      std::cout << knotsV[i] << ' ';
    }
    std::cout << std::endl;


    auto surface_node = make_unique<NURBSSurface>(numRows, numCols, control_points, weights_, knotsU, knotsV, degreeU, degreeV);
    surface_node_ptr_ = surface_node.get();
    root.AddChild(std::move(surface_node));
  }

}
void SplineViewerApp::DrawGUI(){
  if (spline_type_ == "NURBS curve"){
    DrawSplineGUI();
  }
}
void SplineViewerApp::DrawSplineGUI() {
  bool change_control_pt_selection = false; // change which control point is selected
  bool modified = false; // change the selected control point's location
  bool control_point_button_pushed_clamped = false; // curve goes through the first and last control points. ADDS a new control point.
  bool control_point_button_pushed_unclamped = false; // curve doesn't go through the first and last control points. ADDS a new control point.
  bool clamp_ends = false; // curve goes through the first and last control points. Does NOT add a new control point.
  bool unclamp_ends = false;  // curve doesn't go through the first and last control points. Does NOT add a new control point.
  bool circle_button_pushed = false; // adds circle
  bool change_circle_selection = false; // select the circle that you want to move
  bool print_things = false; // prints out curve information in the format of a .spline file

  // CONTROL PANEL GUI
  ImGui::Begin("Control Panel");
  // editing existing control points
  ImGui::Text("Selected control point:");
  ImGui::PushID((int)0);
  change_control_pt_selection |= ImGui::SliderInt("", &selected_control_pt, 0, nurbs_node_ptr_->GetControlPointsLocations().size()-1);
  ImGui::PopID();
  ImGui::Text("Weight of selected control point:");
  ImGui::PushID((int)1);
  modified |= ImGui::InputFloat("", &weights_[selected_control_pt], 1.0, 1.0);
  ImGui::PopID();
  clamp_ends |= ImGui::SmallButton("Clamp ends");
  unclamp_ends |= ImGui::SmallButton("Unclamp ends");
  // adding new control point
  ImGui::Text("Add a control point:");
  ImGui::PushID((int)2);
  ImGui::InputFloat4("X, Y, Z, W", &control_point_settings_[0], 1.0, 1.0);
  ImGui::PopID();
  control_point_button_pushed_clamped |= ImGui::SmallButton("Add control point w/ clamped ends");
  control_point_button_pushed_unclamped |= ImGui::SmallButton("Add control point w/o clamped ends");
  ImGui::Text("");
  // editing existing circle
  ImGui::Text("Circle:");
  ImGui::Text("Selected circle: (-1 = none selected)");
  ImGui::PushID((int)3);
  change_circle_selection |= ImGui::SliderInt("", &selected_circle, -1, nurbs_circle_ptrs_.size()-1);
  ImGui::PopID();
  // adding new circle
  ImGui::Text("Add a circle:");
  ImGui::InputFloat4("X, Y, Z, R", &circle_settings_[0], 1.0, 1.0);
  circle_button_pushed |= ImGui::SmallButton("Insert Circle");
  ImGui::Text("");
  ImGui::Text("Print control points info:");
  print_things |= ImGui::SmallButton("Print info!");
  ImGui::End();

  if (change_control_pt_selection){ // change which control point is selected
    nurbs_node_ptr_->ChangeSelectedControlPoint(selected_control_pt);
  }

  if (modified) { // change the selected control point's location
    nurbs_node_ptr_->OnWeightChanged(weights_);
  }

  if (control_point_button_pushed_clamped){  // curve goes through the first and last control points. ADDS a new control point.
    nurbs_node_ptr_->AddNewControlPoint(glm::vec3(control_point_settings_[0],control_point_settings_[1],control_point_settings_[2]), control_point_settings_[3], true);
    weights_.push_back(control_point_settings_[3]);
  } 
  if (control_point_button_pushed_unclamped){ // curve doesn't go through the first and last control points. ADDS a new control point.
    nurbs_node_ptr_->AddNewControlPoint(glm::vec3(control_point_settings_[0],control_point_settings_[1],control_point_settings_[2]), control_point_settings_[3], false);
    weights_.push_back(control_point_settings_[3]);
  }
  if (clamp_ends){ // curve goes through the first and last control points. Does NOT add a new control point.
    nurbs_node_ptr_->CalcKnotVector(true, false);
  }
  if (unclamp_ends){ // curve doesn't go through the first and last control points. Does NOT add a new control point.
    nurbs_node_ptr_->CalcKnotVector(false, false);
  }

  if (circle_button_pushed){  // adds circle
    SceneNode& root = scene_->GetRootNode();
    auto circle = make_unique<NURBSCircle>(glm::vec3(circle_settings_[0], circle_settings_[1], circle_settings_[2]), circle_settings_[3]);
    nurbs_circle_ptrs_.push_back(circle.get());
    root.AddChild(std::move(circle));
  }

  if (change_circle_selection){ // select the circle that you want to move
    if (selected_circle == -1){
      nurbs_node_ptr_->ChangeEditStatus(true);
      for (int i = 0; i < nurbs_circle_ptrs_.size(); i++){
        nurbs_circle_ptrs_[i]->GetNurbsNodePtr()->ChangeEditStatus(false);
      }
    } else{
      nurbs_node_ptr_->ChangeEditStatus(false);
      for (int i = 0; i < nurbs_circle_ptrs_.size(); i++){
        nurbs_circle_ptrs_[i]->GetNurbsNodePtr()->ChangeEditStatus(false);
      }
      nurbs_circle_ptrs_[selected_circle]->GetNurbsNodePtr()->ChangeEditStatus(true);
    }
  }

  if (print_things){ // prints out curve information in the format of a .spline file
    std::cout << "NURBS curve" << std::endl;
    std::vector<glm::vec3> control_points_locations = nurbs_node_ptr_->GetControlPointsLocations();
    std::vector<float> control_points_weights = nurbs_node_ptr_->GetWeights();
    std::cout << "control points" << std::endl;
    for (size_t i = 0; i < control_points_locations.size(); i++) {
      std::cout << control_points_locations[i].x << " " << control_points_locations[i].y << " " << control_points_locations[i].z << " " << control_points_weights[i] << std::endl;
    }

    std::cout << "knots" << std::endl;
    std::vector<float> knots = nurbs_node_ptr_->GetKnotVector();
    for(int i=0; i < knots.size(); i++)
        std::cout << knots[i] << ' ';
    std::cout << std::endl;

    std::cout << "degree" << std::endl;
    std::cout << nurbs_node_ptr_->GetDegree() << std::endl;
  }
}
}  // namespace GLOO

