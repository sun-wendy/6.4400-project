#ifndef NURBS_NODE_H_
#define NURBS_NODE_H_

#include <string>
#include <vector>

#include "gloo/SceneNode.hpp"
#include "gloo/VertexObject.hpp"
#include "gloo/shaders/ShaderProgram.hpp"

namespace GLOO {

enum class NURBSBasis { NURBS };

struct NURBSPoint {
    glm::vec3 P;
    glm::vec3 T;
};

class NURBSNode : public SceneNode {
 public:
    NURBSNode(int degree, std::vector<glm::vec3> control_points, std::vector<float> weights, std::vector<float> knots, NURBSBasis spline_basis, char curve_type, bool curve_being_edited);
    void OnWeightChanged(std::vector<float> new_weights);
    void ChangeSelectedControlPoint(int new_selected_control_point);
    void Update(double delta_time) override;
    NURBSPoint EvalCurve(float t);
    void InitCurveAndControlPoints();
    // void InitCurve();
    void PlotCurve();
    void PlotControlPoints();
    // void PlotTangentLine();
    float CalcNip(int control_point_i, float time_u);
    void UpdateControlPoints(std::vector<glm::vec3> new_control_points);
    void ChangeEditStatus(bool curve_being_edited);
    std::vector<glm::vec3> GetControlPointsLocations();
    std::vector<float> GetWeights();
    
    // void ChangeControlPointLocation(char key);

 private:
    // NURBSPoint EvalCurve(float t);
    // void InitCurveAndControlPoints();
    // void InitCurve();
    // void PlotCurve();
    // void PlotControlPoints();
    // void PlotTangentLine();
    // float CalcNip(int control_point_i, float time_u);
    
    std::vector<glm::vec3> control_pts_;
    std::vector<float> knots_;
    NURBSBasis spline_basis_;
    int degree_;
    std::vector<float> weights_;

    std::shared_ptr<VertexObject> sphere_mesh_;
    std::shared_ptr<VertexObject> curve_polyline_;
    std::shared_ptr<VertexObject> tangent_line_;
    std::shared_ptr<ShaderProgram> shader_;
    std::shared_ptr<ShaderProgram> polyline_shader_;
    std::vector<SceneNode*> control_point_nodes_;
    int selected_control_point_;
    char curve_type_;
    bool curve_being_edited_;

    const int N_SUBDIV_ = 50;
};
}  // namespace GLOO

#endif
