#include "NURBSNode.hpp"

#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/shaders/SimpleShader.hpp"
#include "gloo/InputManager.hpp"

namespace GLOO {
NURBSNode::NURBSNode(int degree, std::vector<glm::vec3> control_points, std::vector<float> knots, NURBSBasis spline_basis) {
    degree_ = degree;
    control_pts_ = control_points;
    knots_ = knots;
    spline_basis_ = spline_basis;

    // Initialize the VertexObjects and shaders used to render the control points,
    // the curve, and the tangent line.
    sphere_mesh_ = PrimitiveFactory::CreateSphere(0.015f, 25, 25);
    curve_polyline_ = std::make_shared<VertexObject>();
    tangent_line_ = std::make_shared<VertexObject>();
    shader_ = std::make_shared<PhongShader>();
    polyline_shader_ = std::make_shared<SimpleShader>();

    control_point_nodes_ = std::vector<SceneNode*>();

    InitCurve();
    // PlotControlPoints();
    // PlotTangentLine();
}

std::vector<float> NURBSNode::CalcBasisFunc(float t) {
    std::vector<float> basis_func(control_pts_.size());
    
    // Initialize basis functions for the first knot span
    for (int i = 0; i <= degree_; i++) {
        basis_func[i] = (t - knots_[i]) / (knots_[i+degree_] - knots_[i]);
    }

    // Recursively calculate basis functions for the remaining knot spans
    for (int i = degree_ + 1; i < control_pts_.size(); i++) {
        for (int j = 0; j <= i - degree_ - 1; j++) {
            basis_func[i] = (t - knots_[i]) / (knots_[i+degree_] - knots_[i]) * basis_func[i-1] + (knots_[i+1] - t) / (knots_[i+1] - knots_[i-degree_+1]) * basis_func[i-degree_];
        }
    }

    return basis_func;
}

NURBSPoint NURBSNode::EvalCurve(float t) {
    NURBSPoint curve_point;
    curve_point.P = glm::vec3(0.0f);
    curve_point.T = glm::vec3(0.0f);

    std::vector<float> basis_func = CalcBasisFunc(t);

    for (int i = 0; i < control_pts_.size(); i++) {
        curve_point.P += basis_func[i] * control_pts_[i];
        curve_point.T += basis_func[i] * control_pts_[i];
    }

    return curve_point;
}

void NURBSNode::InitCurve() {
    auto positions = make_unique<PositionArray>();
    for (int i = 0; i < N_SUBDIV_; i++) {
        float t = (float)i / (N_SUBDIV_ - 1);
        NURBSPoint curve_point = EvalCurve(t);
        positions->push_back(curve_point.P);
    }

    auto indices = make_unique<IndexArray>();
    for (int i = 0; i < N_SUBDIV_ - 1; i++) {
        indices->push_back(i);
        indices->push_back(i + 1);
    }

    curve_polyline_->UpdatePositions(std::move(positions));
    curve_polyline_->UpdateIndices(std::move(indices));

    auto polyline_node = make_unique<SceneNode>();
    polyline_node->CreateComponent<ShadingComponent>(polyline_shader_);

    auto& rc = polyline_node->CreateComponent<RenderingComponent>(curve_polyline_);
    rc.SetDrawMode(DrawMode::Lines);

    glm::vec3 color(1.f, 1.f, 0);
    auto material = std::make_shared<Material>(color, color, color, 0);
    polyline_node->CreateComponent<MaterialComponent>(material);

    AddChild(std::move(polyline_node));
}
}  // namespace GLOO
