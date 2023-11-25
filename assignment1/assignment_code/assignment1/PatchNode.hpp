#ifndef PATCH_NODE_H_
#define PATCH_NODE_H_

#include <string>
#include <vector>

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"

#include "CurveNode.hpp"

namespace GLOO {
struct PatchPoint {
  glm::vec3 P;
  glm::vec3 N;
};

class PatchNode : public SceneNode {
 public:
  PatchNode(std::vector<glm::vec3> control_points, SplineBasis spline_basis);
  void Update(double delta_time) override;

 private:
  void PlotPatch();
  PatchPoint EvalPatch(float u, float v);

  std::vector<glm::mat4> Gs_;
  SplineBasis spline_basis_;

  std::shared_ptr<VertexObject> patch_mesh_;
  std::shared_ptr<ShaderProgram> shader_;

  const int N_SUBDIV_ = 50;
};
}  // namespace GLOO

#endif
