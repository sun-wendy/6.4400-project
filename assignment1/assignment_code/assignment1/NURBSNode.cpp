#include "NURBSNode.hpp"
#include <string>

#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/shaders/SimpleShader.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {
NURBSNode::NURBSNode(int degree, std::vector<glm::vec3> control_points, std::vector<float> weights, std::vector<float> knots, NURBSBasis spline_basis, char curve_type, bool curve_being_edited) {
    degree_ = degree;
    control_pts_ = control_points;
    knots_ = knots;
    spline_basis_ = spline_basis;
    weights_ = weights;
    curve_type_ = curve_type;
    curve_being_edited_ = curve_being_edited;

    // Initialize the VertexObjects and shaders used to render the control points,
    // the curve, and the tangent line.
    sphere_mesh_ = PrimitiveFactory::CreateSphere(0.15f, 25, 25);
    curve_polyline_ = std::make_shared<VertexObject>();
    tangent_line_ = std::make_shared<VertexObject>();
    shader_ = std::make_shared<PhongShader>();
    polyline_shader_ = std::make_shared<SimpleShader>();

    control_point_nodes_ = std::vector<SceneNode*>();
    selected_control_point_ = 0;

    InitCurveAndControlPoints();
    PlotControlPoints();
}

// adapted from https://www.codeproject.com/Articles/1095142/Generate-and-understand-NURBS-curves
float NURBSNode::CalcNip(int control_point_i, float time_u){ // Dynamic Programming approach to calculate basis
    int i = control_point_i; // converting variable names to what is used in the textbook
    int p = degree_;
    std::vector<float> U = knots_;
    float u = time_u;

    std::vector<float> N(p + 1);
    float saved;
    float temp;

    int m = U.size() - 1;
    if ((i == 0 && u == U[0]) || (i == (m - p - 1) && u == U[m])){ 
        return 1.0;
    }
    if (u < U[i] || u >= U[i + p + 1]){ // avoid division by 0
        return 0.0;
    }

    for (int j = 0; j <= p; j++){
        if (u >= U[i + j] && u < U[i + j + 1]){
            N[j] = 1.0;
        }
        else{
            N[j] = 0.0;
        }
    }

    for (int k = 1; k <= p; k++){
        if (N[0] == 0){
            saved = 0.0;
        }
        else{
            saved = ((u - U[i]) * N[0]) / (U[i + k] - U[i]);
        }
        for (int j = 0; j < p - k + 1; j++){
            float Uleft = U[i + j + 1];
            float Uright = U[i + j + k + 1];

            if (N[j + 1] == 0){
                N[j] = saved;
                saved = 0.0;
            }
            else{
                temp = N[j + 1] / (Uright - Uleft);
                N[j] = saved + (Uright - u) * temp;
                saved = (u - Uleft) * temp;
            }
        }
    }
    return N[0];
}


NURBSPoint NURBSNode::EvalCurve(float t) { // evaluates the curve at time t. In many textbooks, the variable "u" is used instead.
    NURBSPoint curve_point;
    curve_point.P = glm::vec3(0.0f);
    curve_point.T = glm::vec3(0.0f);
    float rationalWeight = 0.0;

    for (int i = 0; i < control_pts_.size(); i++){
        float temp = CalcNip(i, t) * weights_[i];
        rationalWeight += temp;
    }

    for (int i = 0; i < control_pts_.size(); i++){
        float temp = CalcNip(i, t);
        curve_point.P += control_pts_[i] * weights_[i] * temp/rationalWeight;
        // curve_point.P.x += control_pts_[i].x * weights_[i] * temp/rationalWeight;
        // curve_point.P.y += control_pts_[i].y * weights_[i] * temp/rationalWeight;
        // curve_point.P.z += control_pts_[i].z * weights_[i] * temp/rationalWeight;
    }
    return curve_point;
}

void NURBSNode::InitCurveAndControlPoints() {
    // initialize curve
    float start = knots_[degree_];
    float end = knots_[knots_.size()-degree_-1];
    float interval_length = end-start;
    auto positions = make_unique<PositionArray>();
    for (int i = 0; i < N_SUBDIV_; i++) {
        float t = ((float)i / (N_SUBDIV_ - 1)) * interval_length + start;
        NURBSPoint curve_point = EvalCurve(t);
        positions->push_back(curve_point.P);
    }

    auto indices = make_unique<IndexArray>();
    for (int i = 0; i < N_SUBDIV_ - 1; i++) {
        indices->push_back(i);
        indices->push_back(i + 1);
    }

    curve_polyline_->UpdatePositions(std::move(positions));
    curve_polyline_->UpdateIndices(std::move(indices));

    auto polyline_node = make_unique<SceneNode>();
    polyline_node->CreateComponent<ShadingComponent>(polyline_shader_);

    auto& rc = polyline_node->CreateComponent<RenderingComponent>(curve_polyline_);
    rc.SetDrawMode(DrawMode::Lines);

    glm::vec3 color(1.f, 1.f, 0);
    auto material = std::make_shared<Material>(color, color, color, 0);
    polyline_node->CreateComponent<MaterialComponent>(material);

    AddChild(std::move(polyline_node));

    // initialize control points
    for (int i = 0; i < control_pts_.size(); i++) {
        auto point_node = make_unique<SceneNode>();
        point_node->GetTransform().SetPosition(control_pts_[i]);

        point_node->CreateComponent<ShadingComponent>(shader_);
        
        auto& rc = point_node->CreateComponent<RenderingComponent>(sphere_mesh_);
        rc.SetDrawMode(DrawMode::Triangles);

        glm::vec3 color(1.f, 0.f, 0);
        auto material = std::make_shared<Material>(color, color, color, 0);
        point_node->CreateComponent<MaterialComponent>(material);

        control_point_nodes_.push_back(point_node.get());
        AddChild(std::move(point_node));
    }
    ChangeSelectedControlPoint(0);
}


void NURBSNode::PlotCurve() {
    // Call to update the curve
    float start = knots_[degree_];
    float end = knots_[knots_.size()-degree_-1];
    float interval_length = end-start;
    auto positions = make_unique<PositionArray>();
    for (int i = 0; i < N_SUBDIV_; i++) {
        float t = ((float)i / (N_SUBDIV_ - 1)) * interval_length + start;
        NURBSPoint curve_point = EvalCurve(t);
        positions->push_back(curve_point.P);
    }

    auto indices = make_unique<IndexArray>();
    for (int i = 0; i < N_SUBDIV_ - 1; i++) {
        indices->push_back(i);
        indices->push_back(i + 1);
    }

    curve_polyline_->UpdatePositions(std::move(positions));
    curve_polyline_->UpdateIndices(std::move(indices));
}

void NURBSNode::PlotControlPoints() {
    for (int i = 0; i < control_pts_.size(); i++) {
        control_point_nodes_[i]->GetTransform().SetPosition(control_pts_[i]);
    }
}

void NURBSNode::ChangeEditStatus(bool curve_being_edited){
    curve_being_edited_ = curve_being_edited;
}

void NURBSNode::Update(double delta_time) {
  if (curve_type_ == 'R' && curve_being_edited_){
    // Prevent multiple toggle.
    static bool prev_released = true;
    if (InputManager::GetInstance().IsKeyPressed('W')) {
        // if (prev_released) { // Interpret the control points as the other basis
            control_pts_[selected_control_point_].y += 0.05;
            PlotControlPoints();
            PlotCurve();
        // }
        prev_released = false;
    } else if (InputManager::GetInstance().IsKeyPressed('A')) {
        // if (prev_released) {
            control_pts_[selected_control_point_].x -= 0.05;
            PlotControlPoints();
            PlotCurve();
        // }
        prev_released = false;
    } else if (InputManager::GetInstance().IsKeyPressed('S')) {
        // if (prev_released) {
            control_pts_[selected_control_point_].y -= 0.05;
            PlotControlPoints();
            PlotCurve();
        // }
        prev_released = false;
    } else if (InputManager::GetInstance().IsKeyPressed('D')) {
        // if (prev_released) {
            control_pts_[selected_control_point_].x += 0.05;
            PlotControlPoints();
            PlotCurve();
        // }
        prev_released = false;
    }else {
        prev_released = true;
    }
  }
  else if (curve_type_ == 'C' && curve_being_edited_){
    static bool prev_released = true;
    if (InputManager::GetInstance().IsKeyPressed('W')) {
        // if (prev_released) { // Interpret the control points as the other basis
            for (int i = 0; i < control_pts_.size(); i++){
                control_pts_[i].y += 0.05;
            }
            PlotControlPoints();
            PlotCurve();
        // }
        prev_released = false;
    } else if (InputManager::GetInstance().IsKeyPressed('A')) {
        // if (prev_released) {
            for (int i = 0; i < control_pts_.size(); i++){
                control_pts_[i].x -= 0.05;
            }
            PlotControlPoints();
            PlotCurve();
        // }
        prev_released = false;
    } else if (InputManager::GetInstance().IsKeyPressed('S')) {
        // if (prev_released) {
            for (int i = 0; i < control_pts_.size(); i++){
                control_pts_[i].y -= 0.05;
            }
            PlotControlPoints();
            PlotCurve();
        // }
        prev_released = false;
    } else if (InputManager::GetInstance().IsKeyPressed('D')) {
        // if (prev_released) {
            for (int i = 0; i < control_pts_.size(); i++){
                control_pts_[i].x += 0.05;
            }
            PlotControlPoints();
            PlotCurve();
        // }
        prev_released = false;
    }else {
        prev_released = true;
    }
  }
}

// void NURBSNode::ChangeControlPointLocation(char key){
//     if (&key == "W"){
//         control_pts_[selected_control_point_].y += 0.05;
//         PlotControlPoints();
//         PlotCurve();
//     }
// } 

std::vector<glm::vec3> NURBSNode::GetControlPointsLocations(){
    return control_pts_;
}

std::vector<float> NURBSNode::GetWeights(){
    return weights_;
}

void NURBSNode::ChangeSelectedControlPoint(int new_selected_control_point){
    Material& material = control_point_nodes_[selected_control_point_]->GetComponentPtr<MaterialComponent>()->GetMaterial();
    glm::vec3 green_color(0.f, 1.f, 0.f); // green
    glm::vec3 red_color(1.f, 0.f, 0.f); // red
    material.SetAmbientColor(red_color);
    material.SetDiffuseColor(red_color);
    material.SetSpecularColor(red_color);

    selected_control_point_ = new_selected_control_point;
    Material& material2 = control_point_nodes_[selected_control_point_]->GetComponentPtr<MaterialComponent>()->GetMaterial();
    material2.SetAmbientColor(green_color);
    material2.SetDiffuseColor(green_color);
    material2.SetSpecularColor(green_color);
}

void NURBSNode::OnWeightChanged(std::vector<float> new_weights){
    weights_ = new_weights;
    PlotCurve();
}

void NURBSNode::UpdateControlPoints(std::vector<glm::vec3> new_control_points){
    control_pts_ = new_control_points;
    for (int i = 0; i < control_pts_.size(); i++) {
        control_point_nodes_[i]->GetTransform().SetPosition(control_pts_[i]);
    }
    PlotCurve();
}
}  // namespace GLOO
