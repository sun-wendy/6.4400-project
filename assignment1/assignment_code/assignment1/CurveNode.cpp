#include "CurveNode.hpp"

#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/shaders/SimpleShader.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {
CurveNode::CurveNode(std::vector<glm::vec3> control_points, SplineBasis spline_basis) {
  // TODO: this node should represent a single spline curve.
  // Think carefully about what data defines a curve and how you can
  // render it.
  glm::mat4x3 matrix;
  for (int i = 0; i < 4; i++) {
    // std::cout<<control_points.size()<<std::endl;
    matrix[i] = control_points[i];
  }
  control_pts_matrix_ = matrix;
  spline_basis_ = spline_basis;

  bool b_signal;

  // Initialize the VertexObjects and shaders used to render the control points,
  // the curve, and the tangent line.
  sphere_mesh_ = PrimitiveFactory::CreateSphere(0.015f, 25, 25);
  curve_polyline_ = std::make_shared<VertexObject>();
  tangent_line_ = std::make_shared<VertexObject>();
  shader_ = std::make_shared<PhongShader>();
  polyline_shader_ = std::make_shared<SimpleShader>();

  control_point_nodes_ = std::vector<SceneNode*>();

  InitCurve();
  PlotControlPoints();
  PlotTangentLine();
}

void CurveNode::Update(double delta_time) {
  // Prevent multiple toggle.
  static bool prev_released = true;

  if (InputManager::GetInstance().IsKeyPressed('T')) {
    if (prev_released) {
      // TODO: implement toggling spline bases.
      ToggleSplineBasis();
    }
    prev_released = false;
  } else if (InputManager::GetInstance().IsKeyPressed('B')) {
    if (prev_released) {
      // TODO: implement converting conrol point geometry from Bezier to
      // B-Spline basis.
      b_signal = true;
      ConvertGeometry();
    }
    prev_released = false;
  } else if (InputManager::GetInstance().IsKeyPressed('Z')) {
    if (prev_released) {
      // TODO: implement converting conrol point geometry from B-Spline to
      // Bezier basis.
      b_signal = false;
      ConvertGeometry();
    }
    prev_released = false;
  } else {
    prev_released = true;
  }
}

void CurveNode::ToggleSplineBasis() {
  // TODO: implement toggling between Bezier and B-Spline bases.
  if (spline_basis_ == SplineBasis::Bezier) {
    spline_basis_ = SplineBasis::BSpline;

    glm::vec3 color(0, 1.f, 0);
    auto material = std::make_shared<Material>(color, color, color, 0);
    for (int i = 0; i < 4; i++) {
      control_point_nodes_[i]->CreateComponent<MaterialComponent>(material);
    }
  } else {
    spline_basis_ = SplineBasis::Bezier;

    glm::vec3 color(1.f, 0, 0);
    auto material = std::make_shared<Material>(color, color, color, 0);
    for (int i = 0; i < 4; i++) {
      control_point_nodes_[i]->CreateComponent<MaterialComponent>(material);
    }
  }
  PlotCurve();
}

void CurveNode::ConvertGeometry() {
  // TODO: implement converting the control points between bases.
  glm::mat4x4 B_1 = glm::mat4(1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1);
  glm::mat4x4 B_2 = glm::mat4(1/6.0, 2/3.0, 1/6.0, 0.0, -1/2.0, 0.0, 1/2.0, 0, 1/2.0, -1, 1/2.0, 0, -1/6.0, 1/2.0, -1/2.0, 1/6.0);
  glm::mat4x3 G = control_pts_matrix_;

  if (b_signal) {
    glm::mat4x4 B_2_inv = glm::inverse(B_2);
    glm::mat4x3 new_G = G * B_1 * B_2_inv;
    control_pts_matrix_ = new_G;
  } else {
    glm::mat4x4 B_1_inv = glm::inverse(B_1);
    glm::mat4x3 new_G = G * B_2 * B_1_inv;
    control_pts_matrix_ = new_G;
  }

  PlotCurve();
}

CurvePoint CurveNode::EvalCurve(float t) {
  // TODO: implement evaluating the spline curve at parameter value t.
  glm::mat4x3 G = control_pts_matrix_;

  glm::mat4x4 B;
  if (spline_basis_ == SplineBasis::Bezier) {
    B = glm::mat4(1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1);
  } else {
    B = glm::mat4(1/6.0, 2/3.0, 1/6.0, 0.0, -1/2.0, 0.0, 1/2.0, 0, 1/2.0, -1, 1/2.0, 0, -1/6.0, 1/2.0, -1/2.0, 1/6.0);
  }

  glm::vec4 monomial = glm::vec4(1, t, t*t, t*t*t);

  glm::vec3 P = G * B * monomial;
  glm::vec4 d_monomial = glm::vec4(0, 1, 2*t, 3*t*t);
  glm::vec3 T = G * B * d_monomial;
  return CurvePoint{P, T};
}

void CurveNode::InitCurve() {
  // TODO: create all of the  nodes and components necessary for rendering the
  // curve, its control points, and its tangent line. You will want to use the
  // VertexObjects and shaders that are initialized in the class constructor.

  auto positions = make_unique<PositionArray>();
  for (int i = 0; i < N_SUBDIV_; i++) {
    float t = (float)i / (N_SUBDIV_-1);
    CurvePoint curve_point = EvalCurve(t);
    positions->push_back(curve_point.P);
  }

  auto indices = make_unique<IndexArray>();
  for (int i = 0; i < N_SUBDIV_-1; i++) {
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
}

void CurveNode::PlotCurve() {
  // TODO: plot the curve by updating the positions of its VertexObject.
  auto positions = make_unique<PositionArray>();
  for (int i = 0; i < N_SUBDIV_; i++) {
    float t = (float)i / (N_SUBDIV_-1);
    CurvePoint curve_point = EvalCurve(t);
    positions->push_back(curve_point.P);
  }
  curve_polyline_->UpdatePositions(std::move(positions));

  for (int i = 0; i < 4; i++) {
    control_point_nodes_[i]->GetTransform().SetPosition(control_pts_matrix_[i]);
  }

  auto tangent_positions = make_unique<PositionArray>();
  float t_mid = 0.5;
  CurvePoint curve_point = EvalCurve(t_mid);
  tangent_positions->push_back(curve_point.P - (0.1f * glm::normalize(curve_point.T)));
  tangent_positions->push_back(curve_point.P + (0.1f * glm::normalize(curve_point.T)));
  tangent_line_->UpdatePositions(std::move(tangent_positions));
}

void CurveNode::PlotControlPoints() {
  // TODO: plot the curve control points.
  auto point_node_one = make_unique<SceneNode>();
  auto point_node_two = make_unique<SceneNode>();
  auto point_node_three = make_unique<SceneNode>();
  auto point_node_four = make_unique<SceneNode>();

  control_point_nodes_.insert(control_point_nodes_.end(), {point_node_one.get(), point_node_two.get(), point_node_three.get(), point_node_four.get()});

  std::shared_ptr<Material> material;
  if (spline_basis_ == SplineBasis::Bezier) {
    glm::vec3 color(1.f, 0, 0);
    material = std::make_shared<Material>(color, color, color, 0);
  } else {
    glm::vec3 color(0, 1.f, 0);
    material = std::make_shared<Material>(color, color, color, 0);
  }
  // auto material = std::make_shared<Material>(color, color, color, 0);

  for (int i = 0; i < 4; i++) {
    control_point_nodes_[i]->GetTransform().SetPosition(control_pts_matrix_[i]);
    control_point_nodes_[i]->CreateComponent<ShadingComponent>(shader_);
    control_point_nodes_[i]->CreateComponent<RenderingComponent>(sphere_mesh_);
    control_point_nodes_[i]->CreateComponent<MaterialComponent>(material);
  }

  AddChild(std::move(point_node_one));
  AddChild(std::move(point_node_two));
  AddChild(std::move(point_node_three));
  AddChild(std::move(point_node_four));
}

void CurveNode::PlotTangentLine() {
  // TODO: implement plotting a line tangent to the curve.
  // Below is a sample implementation for rendering a line segment
  // onto the screen. Note that this is just an example. This code
  // currently has nothing to do with the spline.

  auto positions = make_unique<PositionArray>();
  float t_mid = 0.5;
  CurvePoint curve_point = EvalCurve(t_mid);
  positions->push_back(curve_point.P - (0.1f * glm::normalize(curve_point.T)));
  positions->push_back(curve_point.P + (0.1f * glm::normalize(curve_point.T)));

  auto indices = make_unique<IndexArray>();
  indices->push_back(0);
  indices->push_back(1);

  tangent_line_->UpdatePositions(std::move(positions));
  tangent_line_->UpdateIndices(std::move(indices));

  auto shader = std::make_shared<SimpleShader>();

  auto tangent_line_node = make_unique<SceneNode>();
  tangent_line_node->CreateComponent<ShadingComponent>(shader);

  auto& rc = tangent_line_node->CreateComponent<RenderingComponent>(tangent_line_);
  rc.SetDrawMode(DrawMode::Lines);

  glm::vec3 color(1.f, 1.f, 1.f);
  auto material = std::make_shared<Material>(color, color, color, 0);
  tangent_line_node->CreateComponent<MaterialComponent>(material);

  AddChild(std::move(tangent_line_node));
}
}  // namespace GLOO
