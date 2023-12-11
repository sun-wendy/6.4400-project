#ifndef NURBS_SURFACE_H_
#define NURBS_SURFACE_H_

#include <string>
#include <vector>

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"

#include "NURBSNode.hpp"

namespace GLOO {
// struct PatchPoint {
//   glm::vec3 P;
//   glm::vec3 N;
// };

class NURBSSurface : public SceneNode {
 public:
  NURBSSurface(int numRows, int numCols, std::vector<glm::vec3> control_points, std::vector<float> weights, std::vector<float> knotsU, std::vector<float> knotsV, int degreeU, int degreeV);
  void Update(double delta_time) override;
  void ChangeSelectedControlPoint(int new_selected_control_point);
  void OnWeightChanged(std::vector<float> new_weights);
  void UpdateSurface();
  void PlotControlPoints();
  std::vector<glm::vec3> GetControlPointsLocations();
  std::vector<float> GetWeights();


 private:
    int numRows_;
    int numCols_;
    std::vector<glm::vec3> control_points_;
    std::vector<float> weights_;
    std::vector<float> knotsU_;
    std::vector<float> knotsV_;
    int degreeU_;
    int degreeV_;
    int selected_control_point_;

    int getIndex(int i, int j);
    NURBSPoint EvalPatch(float u, float v); 
    float CalcNip(int control_point_i, int degree, float time_u, std::vector<float> knots);
    void PlotSurface();
    void InitControlPoints();
    // std::vector<float> get_derivative(float u, float v, int order_u, int order_v);
    float Nip_prime(int control_point_i, int degree, float time_u, std::vector<float> knots);
    std::vector<std::vector<float>> BasisFunctionsDerivatives(int spanIndex, int degree,  int derivative, const std::vector<float> knotVector, float paramT);
    int GetKnotSpanIndex(int degree, const std::vector<float> knotVector, float paramT);

    std::vector<std::vector<glm::vec4>> ComputeDerivatives(int degreeU, int degreeV, int derivative, const std::vector<float> knotVectorU, const std::vector<float> knotVectorV, float u, float v, std::vector<std::vector<glm::vec4>> controlPointsHomo);
    std::vector<std::vector<glm::vec3>> ComputeRationalSurfaceDerivatives(int derivative, float u, float v);
    float binomial(int n, int k);
    glm::vec3 Normal(float u, float v);
    //   void PlotPatch();
    //   PatchPoint EvalPatch(float u, float v);

    // std::vector<glm::mat4> Gs_;
    std::shared_ptr<VertexObject> patch_mesh_;
    std::shared_ptr<ShaderProgram> shader_;
    std::shared_ptr<VertexObject> sphere_mesh_;
    std::vector<SceneNode *> control_point_nodes_;

    const int N_SUBDIV_ = 50;
};
}  // namespace GLOO

#endif