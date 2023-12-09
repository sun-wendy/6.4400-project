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

std::vector<glm::vec3> NURBSNode::GetControlPointsLocations(){
    return control_pts_;
}

std::vector<float> NURBSNode::GetWeights(){
    return weights_;
}

std::vector<float> NURBSNode::GetKnotVector(){
    return knots_;
}

int NURBSNode::GetDegree(){
    return degree_;
}

// Adapted from https://www.codeproject.com/Articles/1095142/Generate-and-understand-NURBS-curves
// Dynamic Programming approach to calculate value of basis functions at time u
float NURBSNode::CalcNip(int control_point_i, float time_u){ 
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

// 
std::vector<float> NURBSNode::CalcKnotVector(bool clamped_ends, bool adding_new_point){
    float n;
    if (adding_new_point){ // eg when a new control point is added, expand the knot vector
        n = knots_.size();
    } else {
        n = knots_.size() - 1;
    }
    std::vector<float> new_knots;
    new_knots.push_back(0);
    for (int i = 1; i <= n; i++){
        new_knots.push_back(i/n);
    }

    if (clamped_ends){ // if the ends are clamped, the curve goes through the first and last control points
        for (int i = 0; i <= degree_; i++){
            new_knots[i] = 0.0;
            new_knots[n-i] = 1.0;
        }
    }

    knots_ = new_knots;
    PlotControlPoints();
    PlotCurve();

    return knots_;
}

// Evaluates the curve at time t. In many textbooks, the variable "u" is used instead.
NURBSPoint NURBSNode::EvalCurve(float t) { 
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
    }
    return curve_point;
}

// Initial rendering of curve and control points. Fills in all relavant vectors.
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

// Re-render the curve (when control points or knot vector are edited)
void NURBSNode::PlotCurve() {
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

// Re-render the control points (when control points or knot vector are edited)
void NURBSNode::PlotControlPoints() {
    for (int i = 0; i < control_pts_.size(); i++) {
        control_point_nodes_[i]->GetTransform().SetPosition(control_pts_[i]);
    }
}

void NURBSNode::ChangeEditStatus(bool curve_being_edited){
    curve_being_edited_ = curve_being_edited;
}

// Keyboard inputs (WASDZX) to edit the location of control point(s)
void NURBSNode::Update(double delta_time) {
  if (curve_type_ == 'R' && curve_being_edited_){ // Regular (move just the selected control point)
    // Prevent multiple toggle.
    if (InputManager::GetInstance().IsKeyPressed('W')) {
        control_pts_[selected_control_point_].y += 0.05;
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('A')) {
        control_pts_[selected_control_point_].x -= 0.05;
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('S')) {
        control_pts_[selected_control_point_].y -= 0.05;
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('D')) {
        control_pts_[selected_control_point_].x += 0.05;
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('Z')){
        control_pts_[selected_control_point_].z -= 0.05;
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('X')){
        control_pts_[selected_control_point_].z += 0.05;
        PlotControlPoints();
        PlotCurve();
    }
  }
  else if (curve_type_ == 'C' && curve_being_edited_){ // Circle (move all control points on the circle)
    if (InputManager::GetInstance().IsKeyPressed('W')) {
        for (int i = 0; i < control_pts_.size(); i++){
            control_pts_[i].y += 0.05;
        }
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('A')) {
        for (int i = 0; i < control_pts_.size(); i++){
            control_pts_[i].x -= 0.05;
        }
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('S')) {
        for (int i = 0; i < control_pts_.size(); i++){
            control_pts_[i].y -= 0.05;
        }
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('D')) {
        for (int i = 0; i < control_pts_.size(); i++){
            control_pts_[i].x += 0.05;
        }
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('Z')) {
        for (int i = 0; i < control_pts_.size(); i++){
            control_pts_[i].z -= 0.05;
        }
        PlotControlPoints();
        PlotCurve();
    } else if (InputManager::GetInstance().IsKeyPressed('X')) {
        for (int i = 0; i < control_pts_.size(); i++){
            control_pts_[i].x += 0.05;
        }
        PlotControlPoints();
        PlotCurve();
    }
  }
}

// Changes which control point is selected (the user can change the location of the selected control point)   
void NURBSNode::ChangeSelectedControlPoint(int new_selected_control_point){
    // Deselect previous control point and make it red again
    Material& material = control_point_nodes_[selected_control_point_]->GetComponentPtr<MaterialComponent>()->GetMaterial();
    glm::vec3 red_color(1.f, 0.f, 0.f); // red
    material.SetAmbientColor(red_color);
    material.SetDiffuseColor(red_color);
    material.SetSpecularColor(red_color);
    
    // Select new control point and highlight it in green
    selected_control_point_ = new_selected_control_point;
    Material& material2 = control_point_nodes_[selected_control_point_]->GetComponentPtr<MaterialComponent>()->GetMaterial();
    glm::vec3 green_color(0.f, 1.f, 0.f); // green
    material2.SetAmbientColor(green_color);
    material2.SetDiffuseColor(green_color);
    material2.SetSpecularColor(green_color);
}

// Updates the weights of the CURRENT control points
void NURBSNode::OnWeightChanged(std::vector<float> new_weights){
    weights_ = new_weights;
    PlotCurve();
}

// Updates the positions of the CURRENT control points // Unused Functions
void NURBSNode::UpdateControlPointsPositions(std::vector<glm::vec3> new_control_points){
    control_pts_ = new_control_points;
    for (int i = 0; i < control_pts_.size(); i++) {
        control_point_nodes_[i]->GetTransform().SetPosition(control_pts_[i]);
    }
    PlotCurve();
}

// Add a NEW control point
void NURBSNode::AddNewControlPoint(glm::vec3 control_point_loc, float weight, bool clamped_ends){
    // Add new control point sphere
    control_pts_.push_back(control_point_loc);
    auto point_node = make_unique<SceneNode>();
    point_node->GetTransform().SetPosition(control_point_loc);
    point_node->CreateComponent<ShadingComponent>(shader_);
    auto& rc = point_node->CreateComponent<RenderingComponent>(sphere_mesh_);
    rc.SetDrawMode(DrawMode::Triangles);
    glm::vec3 color(1.f, 0.f, 0);
    auto material = std::make_shared<Material>(color, color, color, 0);
    point_node->CreateComponent<MaterialComponent>(material);
    control_point_nodes_.push_back(point_node.get());
    AddChild(std::move(point_node));

    // Updates weight vec
    weights_.push_back(weight);

    CalcKnotVector(clamped_ends, true);

}

}  // namespace GLOO
