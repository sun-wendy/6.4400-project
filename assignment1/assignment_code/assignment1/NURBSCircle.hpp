#ifndef NURBS_CIRCLE_H_
#define NURBS_CIRCLE_H_

#include <string>
#include <vector>

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"

namespace GLOO {

class NURBSCircle : public SceneNode {
    public:
        NURBSCircle(glm::vec3 center, float radius);
        std::vector<glm::vec3> GetControlPoints(glm::vec3 center, float radius);
        std::vector<float> GetKnots();
        int GetDegree();
        std::vector<float> GetWeights();
    
    private:
        std::vector<glm::vec3> control_pts_;
        std::vector<float> knots_;
        int degree_;
        std::vector<float> weights_;
};
} // namespace GLOO

#endif
