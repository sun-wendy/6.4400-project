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
//   void Update(double delta_time) override;

 private:
    int numRows_;
    int numCols_;
    std::vector<glm::vec3> control_points_;
    std::vector<float> weights_;
    std::vector<float> knotsU_;
    std::vector<float> knotsV_;
    int degreeU_;
    int degreeV_;

    int getIndex(int i, int j);
    NURBSPoint EvalPatch(float u, float v); 
    float CalcNip(int control_point_i, int degree, float time_u, std::vector<float> knots);
    void PlotSurface();
    void InitControlPoints();
    std::vector<float> get_derivative(float u, float v, int order_u, int order_v);
    
//   void PlotPatch();
//   PatchPoint EvalPatch(float u, float v);

  std::vector<glm::mat4> Gs_;
  std::shared_ptr<VertexObject> patch_mesh_;
  std::shared_ptr<ShaderProgram> shader_;
  std::shared_ptr<VertexObject> sphere_mesh_;
  std::vector<SceneNode*> control_point_nodes_;

  const int N_SUBDIV_ = 50;
};
}  // namespace GLOO

#endif