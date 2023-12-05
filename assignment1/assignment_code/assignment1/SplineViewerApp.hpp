#ifndef SPLINE_VIEWER_APP_H_
#define SPLINE_VIEWER_APP_H_

#include "gloo/Application.hpp"
#include "NURBSNode.hpp"

namespace GLOO {
class SplineViewerApp : public Application {
 public:
  SplineViewerApp(const std::string& app_name,
                  glm::ivec2 window_size,
                  const std::string& filename);
  void SetupScene() override;

protected:
  void DrawGUI() override;

 private:
  void LoadFile(const std::string& filename, SceneNode& root);
  std::vector<float> slider_values_;
  std::vector<float> weights_;
  std::vector<glm::vec3> control_points;
  NURBSNode* nurbs_node_ptr_;
  int selected_control_pt = 0;
  int u8_v = 0;

  std::string filename_;
};
}  // namespace GLOO

#endif
