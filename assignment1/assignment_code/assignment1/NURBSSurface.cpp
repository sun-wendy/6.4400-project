#include "NURBSSurface.hpp"
#include "NURBSNode.hpp"

#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/InputManager.hpp"

#include "gloo/debug/PrimitiveFactory.hpp"
namespace GLOO {
NURBSSurface::NURBSSurface(int numRows, int numCols, std::vector<glm::vec3> control_points, std::vector<float> weights, std::vector<float> knotsU, std::vector<float> knotsV, int degreeU, int degreeV) {
    numRows_ = numRows;
    numCols_ = numCols;
    control_points_ = control_points;
    weights_ = weights;
    knotsU_ = knotsU;
    knotsV_ = knotsV;
    degreeU_ = degreeU;
    degreeV_ = degreeV;

    patch_mesh_ = std::make_shared<VertexObject>();
    sphere_mesh_ = PrimitiveFactory::CreateSphere(0.1f, 25, 25);
    shader_ = std::make_shared<PhongShader>();
    PlotSurface();
    InitControlPoints();
}

// Useful for indexing into control points
int NURBSSurface::getIndex(int i, int j){
    return numCols_ * i + j;
}

float NURBSSurface::CalcNip(int control_point_i, int degree, float time_u, std::vector<float> knots){ 
    int i = control_point_i; // converting variable names to what is used in the textbook
    int p = degree;
    std::vector<float> U = knots;
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

// refercing NURBS book and NURBS book implementation https://github.com/RoberAgro/nurbspy/blob/master/nurbspy/nurbs_surface.py#L412
std::vector<float> NURBSSurface::get_derivative(float u, float v, int order_u, int order_v){
 // TO DO
}



// Formula from Wikipedia and Springer - The NURBS Book
NURBSPoint NURBSSurface::EvalPatch(float u, float v){
    NURBSPoint curve_point;
    curve_point.P = glm::vec3(0.0f);
    curve_point.T = glm::vec3(0.0f);
    float rationalWeight = 0.0;

    for (int p = 0; p < numRows_; p++){
        for (int q = 0; q < numCols_; q++){
            float Npn = CalcNip(p, degreeU_, u, knotsU_);
            float Nqm = CalcNip(q, degreeV_, v, knotsV_);
            rationalWeight += Npn * Nqm * weights_[getIndex(p,q)];
        }
    }

    for (int i = 0; i < numRows_; i++){
        for (int j = 0; j < numCols_; j++){
            float Nin = CalcNip(i, degreeU_, u, knotsU_);
            float Njm = CalcNip(j, degreeV_, v, knotsV_);
            
            if (rationalWeight != 0){
                curve_point.P += control_points_[getIndex(i,j)] * Nin * Njm * weights_[getIndex(i,j)] / rationalWeight;
            }

        }
    }

// TO DO -- calculate surface normals

    return curve_point;
} 

void NURBSSurface::InitControlPoints(){
        // initialize control points
    for (int i = 0; i < control_points_.size(); i++) {
        auto point_node = make_unique<SceneNode>();
        point_node->GetTransform().SetPosition(control_points_[i]);

        point_node->CreateComponent<ShadingComponent>(shader_);
        
        auto& rc = point_node->CreateComponent<RenderingComponent>(sphere_mesh_);
        rc.SetDrawMode(DrawMode::Triangles);

        glm::vec3 color(1.f, 0.f, 0);
        auto material = std::make_shared<Material>(color, color, color, 0);
        point_node->CreateComponent<MaterialComponent>(material);

        control_point_nodes_.push_back(point_node.get());
        AddChild(std::move(point_node));
    }
}

void NURBSSurface::PlotSurface(){
  auto positions = make_unique<PositionArray>();
  auto normals = make_unique<NormalArray>();
  auto indices = make_unique<IndexArray>();

  // TODO: fill "positions", "normals", and "indices"
  float width_triangle = 1.0f / N_SUBDIV_;
  for (int i = 0; i < N_SUBDIV_; i++) {
    for (int j = 0; j < N_SUBDIV_; j++) {
      float left_u = (i + 1) * width_triangle;
      float left_v = j * width_triangle;
      NURBSPoint p0 = EvalPatch(left_u, left_v);
      NURBSPoint p1 = EvalPatch(left_u, left_v + width_triangle);
      NURBSPoint p2 = EvalPatch(left_u - width_triangle, left_v);
      NURBSPoint p3 = EvalPatch(left_u - width_triangle, left_v + width_triangle);

      positions->push_back(p0.P);
      positions->push_back(p1.P);
      positions->push_back(p2.P);
      positions->push_back(p3.P);

      normals->push_back(p0.T);
      normals->push_back(p1.T);
      normals->push_back(p2.T);
      normals->push_back(p3.T);

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

}




}