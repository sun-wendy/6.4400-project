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

float NURBSNode::CalcNip(int control_point_i, float time_u){
    int i = control_point_i;
    int p = degree_;
    std::vector<float> U = knots_;
    float u = time_u;

    std::vector<float> N(p + 1);
    float saved;
    float temp;

    int m = U.size() - 1;
    if ((i == 0 && u == U[0]) || (i == (m - p - 1) && u == U[m])){
        return 1.0;
    }
    

    if (u < U[i] || u >= U[i + p + 1]){
        return 0.0;
    }

    for (int j = 0; j <= p; j++)
    {
        if (u >= U[i + j] && u < U[i + j + 1])
            N[j] = 1.0;
        else
            N[j] = 0.0;
    }

    for (int k = 1; k <= p; k++)
    {
        if (N[0] == 0)
            saved = 0.0;
        else
            saved = ((u - U[i]) * N[0]) / (U[i + k] - U[i]);

        for (int j = 0; j < p - k + 1; j++)
        {
            float Uleft = U[i + j + 1];
            float Uright = U[i + j + k + 1];

            if (N[j + 1] == 0)
            {
                N[j] = saved;
                saved = 0.0;
            }
            else
            {
                temp = N[j + 1] / (Uright - Uleft);
                N[j] = saved + (Uright - u) * temp;
                saved = (u - Uleft) * temp;
            }
        }
    }
    return N[0];
}


NURBSPoint NURBSNode::EvalCurve(float t) {
    NURBSPoint curve_point;
    // CalcNip(int control_point_i, float time_u)
    
    curve_point.P = glm::vec3(0.0f);
    curve_point.T = glm::vec3(0.0f);

    for (int i = 0; i < control_pts_.size(); i++){
        float temp = CalcNip(i, t);
        curve_point.P.x += control_pts_[i].x * temp;
        curve_point.P.y += control_pts_[i].y * temp;
        curve_point.P.z += control_pts_[i].z * temp;
    }

    return curve_point;
}

void NURBSNode::InitCurve() {
    float start = knots_[degree_];
    float end = knots_[knots_.size()-degree_-1];
    float interval_length = end-start;
    auto positions = make_unique<PositionArray>();
    for (int i = 0; i < N_SUBDIV_; i++) {
        float t = ((float)i / (N_SUBDIV_ - 1)) * interval_length + start;
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
