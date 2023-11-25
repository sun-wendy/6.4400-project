#ifndef SPLINE_VIEWER_APP_H_
#define SPLINE_VIEWER_APP_H_

#include "gloo/Application.hpp"

namespace GLOO {
class SplineViewerApp : public Application {
 public:
  SplineViewerApp(const std::string& app_name,
                  glm::ivec2 window_size,
                  const std::string& filename);
  void SetupScene() override;

 private:
  void LoadFile(const std::string& filename, SceneNode& root);

  std::string filename_;
};
}  // namespace GLOO

#endif
