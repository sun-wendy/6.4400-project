#include "NURBSCircle.hpp"
#include "NURBSNode.hpp"

namespace GLOO {

NURBSCircle::NURBSCircle(glm::vec3 center, float radius) {
    control_pts_ = GetControlPoints(center, radius);
    knots_ = GetKnots();
    degree_ = GetDegree();
    weights_ = GetWeights();
    // Create a NURBS node for the circle
    auto nurbs_circle_node = make_unique<NURBSNode>(degree_, control_pts_, weights_, knots_, NURBSBasis::NURBS);
    AddChild(std::move(nurbs_circle_node));
}

int NURBSCircle::GetDegree() {
    return 2;
}

std::vector<float> NURBSCircle::GetKnots() {
    std::vector<float> knots;

    for (int i = 0; i < 3; i++) {
        knots.push_back(0.0f);
    }

    for (int i = 0; i < 2; i++) {
        knots.push_back(glm::pi<float>() / 2.0f);
    }

    for (int i = 0; i < 2; i++) {
        knots.push_back(glm::pi<float>());
    }

    for (int i = 0; i < 2; i++) {
        knots.push_back(3.0f * glm::pi<float>() / 2.0f);
    }

    for (int i = 0; i < 3; i++) {
        knots.push_back(2.0f * glm::pi<float>());
    }

    return knots;
}

std::vector<float> NURBSCircle::GetWeights() {
    std::vector<float> weights;

    for (int i = 0; i < 4; i++) {
        weights.push_back(1.0f);
        weights.push_back(1.0f / sqrt(2.0f));
    }
    weights.push_back(1.0f);

    return weights;
}

std::vector<glm::vec3> NURBSCircle::GetControlPoints(glm::vec3 center, float radius){
    std::vector<glm::vec3> control_points;
    control_points.push_back(glm::vec3(center.x + radius, center.y, center.z));
    control_points.push_back(glm::vec3(center.x + radius, center.y + radius, center.z));
    control_points.push_back(glm::vec3(center.x, center.y + radius, center.z));
    control_points.push_back(glm::vec3(center.x - radius, center.y + radius, center.z));
    control_points.push_back(glm::vec3(center.x - radius, center.y, center.z));
    control_points.push_back(glm::vec3(center.x - radius, center.y - radius, center.z));
    control_points.push_back(glm::vec3(center.x, center.y - radius, center.z));
    control_points.push_back(glm::vec3(center.x + radius, center.y - radius, center.z));
    control_points.push_back(glm::vec3(center.x + radius, center.y, center.z));
    return control_points;
}
} // namespace GLOO
