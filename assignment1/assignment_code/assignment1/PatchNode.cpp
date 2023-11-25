#include "PatchNode.hpp"

#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {
PatchNode::PatchNode(std::vector<glm::vec3> control_points, SplineBasis spline_basis) {
  patch_mesh_ = std::make_shared<VertexObject>();
  shader_ = std::make_shared<PhongShader>();

  // TODO: this node should represent a single tensor product patch.
  // Think carefully about what data defines a patch and how you can
  // render it.

  glm::mat4 x_matrix;
  glm::mat4 y_matrix;
  glm::mat4 z_matrix;
  for (int i = 0; i < 4; i++) {
    x_matrix[i] = glm::vec4(control_points[i][0], control_points[i+4][0], control_points[i+8][0], control_points[i+12][0]);
    y_matrix[i] = glm::vec4(control_points[i][1], control_points[i+4][1], control_points[i+8][1], control_points[i+12][1]);
    z_matrix[i] = glm::vec4(control_points[i][2], control_points[i+4][2], control_points[i+8][2], control_points[i+12][2]);
  }

  Gs_ = {x_matrix, y_matrix, z_matrix};
  spline_basis_ = spline_basis;

  patch_mesh_ = std::make_shared<VertexObject>();
  shader_ = std::make_shared<PhongShader>();

  PlotPatch();
}

void PatchNode::Update(double delta_time) {
  // static bool prev_released = true;

  // if (InputManager::GetInstance().IsKeyPressed('M')) {
  //   if (prev_released) {
  auto material = GetChild(0).GetComponentPtr<MaterialComponent>()->GetMaterial();
  PatchPoint patch_point = EvalPatch(0.5, 0.5);
  material.SetDiffuseColor(patch_point.N);
  GetChild(0).GetComponentPtr<MaterialComponent>()->SetMaterial(std::make_shared<Material>(material));
  //   }
  //   prev_released = false;
  // }
}

PatchPoint PatchNode::EvalPatch(float u, float v) {
  glm::mat4 x_coor = Gs_[0];
  glm::mat4 y_coor = Gs_[1];
  glm::mat4 z_coor = Gs_[2];

  glm::mat4 B;
  if (spline_basis_ == SplineBasis::Bezier) {
    B = glm::mat4(1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1);
  } else {
    B = glm::mat4(1/6.0, 2/3.0, 1/6.0, 0.0, -1/2.0, 0.0, 1/2.0, 0, 1/2.0, -1, 1/2.0, 0, -1/6.0, 1/2.0, -1/2.0, 1/6.0);
  }
  glm::mat4 B_transpose = glm::transpose(B);

  glm::vec4 u_vec = glm::vec4(1, u, u*u, u*u*u);
  glm::vec4 v_vec = glm::vec4(1, v, v*v, v*v*v);

  float P_x = glm::dot(u_vec * B_transpose * x_coor * B, v_vec);
  float P_y = glm::dot(u_vec * B_transpose * y_coor * B, v_vec);
  float P_z = glm::dot(u_vec * B_transpose * z_coor * B, v_vec);
  glm::vec3 P = glm::vec3(P_x, P_y, P_z);

  glm::vec4 d_u_vec = glm::vec4(0, 1, 2*u, 3*u*u);
  glm::vec4 d_v_vec = glm::vec4(0, 1, 2*v, 3*v*v);

  float dP_du_x = glm::dot(d_u_vec * B_transpose * x_coor * B, v_vec);
  float dP_du_y = glm::dot(d_u_vec * B_transpose * y_coor * B, v_vec);
  float dP_du_z = glm::dot(d_u_vec * B_transpose * z_coor * B, v_vec);
  glm::vec3 dP_du = glm::vec3(dP_du_x, dP_du_y, dP_du_z);

  float dP_dv_x = glm::dot(u_vec * B_transpose * x_coor * B, d_v_vec);
  float dP_dv_y = glm::dot(u_vec * B_transpose * y_coor * B, d_v_vec);
  float dP_dv_z = glm::dot(u_vec * B_transpose * z_coor * B, d_v_vec);
  glm::vec3 dP_dv = glm::vec3(dP_dv_x, dP_dv_y, dP_dv_z);

  glm::vec3 N = -glm::normalize(glm::cross(dP_du, dP_dv));

  return PatchPoint{P, N};
}

void PatchNode::PlotPatch() {

  auto positions = make_unique<PositionArray>();
  auto normals = make_unique<NormalArray>();
  auto indices = make_unique<IndexArray>();

  // TODO: fill "positions", "normals", and "indices"
  float width_triangle = 1.0f / N_SUBDIV_;
  for (int i = 0; i < N_SUBDIV_; i++) {
    for (int j = 0; j < N_SUBDIV_; j++) {
      float left_u = (i + 1) * width_triangle;
      float left_v = j * width_triangle;
      PatchPoint p0 = EvalPatch(left_u, left_v);
      PatchPoint p1 = EvalPatch(left_u, left_v + width_triangle);
      PatchPoint p2 = EvalPatch(left_u - width_triangle, left_v);
      PatchPoint p3 = EvalPatch(left_u - width_triangle, left_v + width_triangle);

      positions->push_back(p0.P);
      positions->push_back(p1.P);
      positions->push_back(p2.P);
      positions->push_back(p3.P);

      normals->push_back(p0.N);
      normals->push_back(p1.N);
      normals->push_back(p2.N);
      normals->push_back(p3.N);

      int pos_id = (N_SUBDIV_ * i + j) * 4;
      indices->push_back(pos_id);
      indices->push_back(pos_id + 1);
      indices->push_back(pos_id + 2);
      indices->push_back(pos_id + 2);
      indices->push_back(pos_id + 1);
      indices->push_back(pos_id + 3);
    }
  }

  patch_mesh_->UpdatePositions(std::move(positions));
  patch_mesh_->UpdateNormals(std::move(normals));
  patch_mesh_->UpdateIndices(std::move(indices));

  auto patch_single_node = make_unique<SceneNode>();
  patch_single_node->CreateComponent<ShadingComponent>(shader_);

  auto& rc = patch_single_node->CreateComponent<RenderingComponent>(patch_mesh_);
  rc.SetDrawMode(DrawMode::Triangles);

  patch_single_node->CreateComponent<MaterialComponent>(std::make_shared<Material>(Material::GetDefault()));

  AddChild(std::move(patch_single_node));

  // std::cout<<"Plotting patch"<<std::endl;
}
}  // namespace GLOO
