#include "NURBSSurface.hpp"
#include "NURBSNode.hpp"

#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/InputManager.hpp"

#include "gloo/debug/PrimitiveFactory.hpp"
namespace GLOO {
NURBSSurface::NURBSSurface(int numRows, int numCols, std::vector<glm::vec3> control_points, std::vector<float> weights, std::vector<float> knotsU, std::vector<float> knotsV, int degreeU, int degreeV) {
    numRows_ = numRows;
    numCols_ = numCols;
    control_points_ = control_points;
    weights_ = weights;
    knotsU_ = knotsU;
    knotsV_ = knotsV;
    degreeU_ = degreeU;
    degreeV_ = degreeV;
    selected_control_point_ = 0;

    patch_mesh_ = std::make_shared<VertexObject>();
    sphere_mesh_ = PrimitiveFactory::CreateSphere(0.1f, 25, 25);
    shader_ = std::make_shared<PhongShader>();
    PlotSurface();
    InitControlPoints();
}

void NURBSSurface::ChangeSelectedControlPoint(int new_selected_control_point){
    // Deselect previous control point and make it red again
    Material& material = control_point_nodes_[selected_control_point_]->GetComponentPtr<MaterialComponent>()->GetMaterial();
    glm::vec3 red_color(1.f, 0.f, 0.f); // red
    material.SetAmbientColor(red_color);
    material.SetDiffuseColor(red_color);
    material.SetSpecularColor(red_color);
    
    // Select new control point and highlight it in green
    selected_control_point_ = new_selected_control_point;
    Material& material2 = control_point_nodes_[selected_control_point_]->GetComponentPtr<MaterialComponent>()->GetMaterial();
    glm::vec3 green_color(0.f, 1.f, 0.f); // green
    material2.SetAmbientColor(green_color);
    material2.SetDiffuseColor(green_color);
    material2.SetSpecularColor(green_color);
}


void NURBSSurface::OnWeightChanged(std::vector<float> new_weights){
    weights_ = new_weights;
    UpdateSurface();
}

void NURBSSurface::Update(double delta_time) {
    // Prevent multiple toggle.
    if (InputManager::GetInstance().IsKeyPressed('W')) {
        control_points_[selected_control_point_].y += 0.05;
        PlotControlPoints();
        UpdateSurface();
    } else if (InputManager::GetInstance().IsKeyPressed('A')) {
        control_points_[selected_control_point_].x -= 0.05;
        PlotControlPoints();
        UpdateSurface();
    } else if (InputManager::GetInstance().IsKeyPressed('S')) {
        control_points_[selected_control_point_].y -= 0.05;
        PlotControlPoints();
        UpdateSurface();
    } else if (InputManager::GetInstance().IsKeyPressed('D')) {
        control_points_[selected_control_point_].x += 0.05;
        PlotControlPoints();
        UpdateSurface();
    } else if (InputManager::GetInstance().IsKeyPressed('Z')){
        control_points_[selected_control_point_].z -= 0.05;
        PlotControlPoints();
        UpdateSurface();
    } else if (InputManager::GetInstance().IsKeyPressed('X')){
        control_points_[selected_control_point_].z += 0.05;
        PlotControlPoints();
        UpdateSurface();
    }
}

std::vector<glm::vec3> NURBSSurface::GetControlPointsLocations(){
    return control_points_;
}

std::vector<float> NURBSSurface::GetWeights(){
    return weights_;
}

void NURBSSurface::PlotControlPoints() {
    for (int i = 0; i < control_points_.size(); i++) {
        control_point_nodes_[i]->GetTransform().SetPosition(control_points_[i]);
    }
}

// Useful for indexing into control points
int NURBSSurface::getIndex(int i, int j){
    return numCols_ * i + j;
}

float NURBSSurface::CalcNip(int control_point_i, int degree, float time_u, std::vector<float> knots){ 
    int i = control_point_i; // converting variable names to what is used in the textbook
    int p = degree;
    std::vector<float> U = knots;
    float u = time_u;

    std::vector<float> N(p + 1);
    float saved;
    float temp;

    int m = U.size() - 1;
    if ((i == 0 && u == U[0]) || (i == (m - p - 1) && u == U[m])){ 
        return 1.0;
    }
    if (u < U[i] || u >= U[i + p + 1]){ // avoid division by 0
        return 0.0;
    }

    for (int j = 0; j <= p; j++){
        if (u >= U[i + j] && u < U[i + j + 1]){
            N[j] = 1.0;
        }
        else{
            N[j] = 0.0;
        }
    }

    for (int k = 1; k <= p; k++){
        if (N[0] == 0){
            saved = 0.0;
        }
        else{
            saved = ((u - U[i]) * N[0]) / (U[i + k] - U[i]);
        }
        for (int j = 0; j < p - k + 1; j++){
            float Uleft = U[i + j + 1];
            float Uright = U[i + j + k + 1];

            if (N[j + 1] == 0){
                N[j] = saved;
                saved = 0.0;
            }
            else{
                temp = N[j + 1] / (Uright - Uleft);
                N[j] = saved + (Uright - u) * temp;
                saved = (u - Uleft) * temp;
            }
        }
    }
    return N[0];
}



std::vector<std::vector<float>> NURBSSurface::BasisFunctionsDerivatives(int spanIndex, int degree,  int derivative, const std::vector<float> knotVector, float paramT)
{

	std::vector<std::vector<float>> derivatives(derivative + 1, std::vector<float>(degree + 1));
	std::vector<std::vector<float>> ndu(degree + 1,std::vector<float>(degree + 1));

	ndu[0][0] = 1.0;

	std::vector<float> left(degree + 1);
	std::vector<float> right(degree + 1);

	float saved = 0.0;
	float temp = 0.0;

	for (int j = 1; j <= degree; j++)
	{
		left[j] = paramT - knotVector[spanIndex + 1 - j];
		right[j] = knotVector[spanIndex + j] - paramT;

		saved = 0.0;
		for (int r = 0; r < j; r++)
		{
			ndu[j][r] = right[r + 1] + left[j - r];
			temp = ndu[r][j - 1] / ndu[j][r];

			ndu[r][j] = saved + right[r + 1] * temp;
			saved = left[j - r] * temp;
		}
		ndu[j][j] = saved;
	}

	for (int j = 0; j <= degree; j++)
	{
		derivatives[0][j] = ndu[j][degree];
	}

	std::vector<std::vector<float>> a(2,std::vector<float>(degree + 1));
	for (int r = 0; r <= degree; r++)
	{
		int s1 = 0; 
		int s2 = 1;
		a[0][0] = 1.0;

		for (int k = 1; k <= derivative; k++)
		{
			float d = 0.0;
			int rk = r - k;
			int pk = degree - k;

			if (r >= k)
			{
				a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
				d = a[s2][0] * ndu[rk][pk];
			}

			int j1 = 0;
			int j2 = 0;

			if (rk >= -1)
				j1 = 1;
			else
				j1 = -rk;

			if (r - 1 <= pk)
				j2 = k - 1;
			else
				j2 = degree - r;

			for (int j = j1; j <= j2; j++)
			{
				a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
				d += a[s2][j] * ndu[rk + j][pk];
			}
			if (r <= pk)
			{
				a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
				d += a[s2][k] * ndu[r][pk];
			}
			derivatives[k][r] = d;

			int temp = s1; 
			s1 = s2; 
			s2 = temp;
		}
	}

	int r = degree;
	for (int k = 1; k <= derivative; k++)
	{
		for (int j = 0; j <= degree; j++)
		{
			derivatives[k][j] *= r;
		}
		r *= degree - k;
	}
	return derivatives;
}

// https://github.com/BIMCoderLiang/LNLib/blob/f715aaf05b7dfaa8b11f3508d9f507cbbe3ec860/src/LNLib/Algorithm/Polynomials.cpp#L11
int NURBSSurface::GetKnotSpanIndex(int degree, const std::vector<float> knotVector, float paramT)
{

	int n = knotVector.size() - degree - 2;


	if (paramT >= knotVector[n + 1])
	{
		return n;
	}
	if (paramT <= knotVector[degree])
	{
		return degree;
	}

	int low = 0;
	int high = n + 1;
	int mid = static_cast<int>(floor((low + high) / 2.0));

	while (paramT < knotVector[mid] || 
		   paramT >= knotVector[mid + 1])
	{
		if (paramT < knotVector[mid])
		{
			high = mid;
		}
		else
		{
			low = mid;
		}	
		mid = static_cast<int>(floor((low + high) / 2.0));
	}
	return mid;
}

//https://github.com/BIMCoderLiang/LNLib/blob/f715aaf05b7dfaa8b11f3508d9f507cbbe3ec860/src/LNLib/include/BsplineSurface.h#L24
    std::vector<std::vector<glm::vec4>> NURBSSurface::ComputeDerivatives(int degreeU, int degreeV, int derivative, const std::vector<float> knotVectorU, const std::vector<float> knotVectorV, float u, float v, std::vector<std::vector<glm::vec4>> controlPointsHomo){

			std::vector<std::vector<glm::vec4>> derivatives(derivative + 1, std::vector<glm::vec4>(derivative + 1));

			int uSpanIndex = GetKnotSpanIndex(degreeU, knotVectorU, u);
			std::vector<std::vector<float>> Nu = BasisFunctionsDerivatives(uSpanIndex, degreeU, derivative, knotVectorU, u);

			int vSpanIndex = GetKnotSpanIndex(degreeV, knotVectorV, v);
			std::vector<std::vector<float>> Nv = BasisFunctionsDerivatives(vSpanIndex, degreeV, derivative, knotVectorV, v);

			int du = std::min(derivative, degreeU);
			int dv = std::min(derivative, degreeV);

			std::vector<glm::vec4> temp(degreeV + 1);

			for (int k = 0; k <= du; k++)
			{
				for (int s = 0; s <= degreeV; s++)
				{
					temp[s] = glm::vec4();
					for (int r = 0; r <= degreeU; r++)
					{
						temp[s] += Nu[k][r] * controlPointsHomo[uSpanIndex - degreeU + r][vSpanIndex - degreeV + s];
					}
				}
				int dd = std::min(derivative, dv);
				for (int l = 0; l <= dd; l++)
				{
					for (int s = 0; s <= degreeV; s++)
					{
						derivatives[k][l] += Nv[l][s] * temp[s];
					}
				}
			}
			return derivatives;
		}

std::vector<std::vector<glm::vec3>> NURBSSurface::ComputeRationalSurfaceDerivatives(int derivative, float u, float v)
{
	int degreeU = degreeU_;
	int degreeV = degreeV_;
	std::vector<float> knotVectorU = knotsU_;
	std::vector<float> knotVectorV = knotsV_;
	std::vector<std::vector<glm::vec4>> controlPointsHomo;

    for (int i = 0; i < numRows_; i++){
        std::vector<glm::vec4> row;
        for (int j = 0; j < numCols_; j++){
            glm::vec3 point = control_points_[getIndex(i,j)];
            float weight = weights_[getIndex(i,j)];
            row.push_back(glm::vec4(point.x, point.y, point.z, weight));
        }
        controlPointsHomo.push_back(row);
    }

	std::vector<std::vector<glm::vec3>> derivatives(derivative + 1, std::vector<glm::vec3>(derivative + 1));
	std::vector<std::vector<glm::vec4>> ders = ComputeDerivatives(degreeU, degreeV, derivative, knotVectorU, knotVectorV, u, v, controlPointsHomo);
	std::vector<std::vector<glm::vec3>> Aders(derivative + 1, std::vector<glm::vec3>(derivative + 1));
	std::vector<std::vector<float>> wders(derivative + 1, std::vector<float>(derivative + 1));
	for (int i = 0; i < ders.size(); i++)
	{
		for (int j = 0; j < ders[0].size(); j++)
		{
			Aders[i][j] = glm::vec3(ders[i][j]);
			wders[i][j] = ders[i][j][3]; // w coordinate 
		}
	}

	for (int k = 0; k <= derivative; k++)
	{
		for (int l = 0; l <= derivative - k; l++)
		{
			glm::vec3 v = Aders[k][l];
			for ( int j = 1; j <= l; j++)
			{
				v = v - binomial(l, j) * wders[0][j] * derivatives[k][l - j];
			}

			for (int i = 1; i <= k; i++)
			{
				v = v - binomial(k, i) * wders[i][0] * derivatives[k - i][l];

				glm::vec3 v2(0);
				for (int j = 1; j <= l; j++)
				{
					v2 = v2 + binomial(l, j) * wders[i][j] * derivatives[k - i][l - j];
				}
				v = v - binomial(k, i) * v2;
			}
			derivatives[k][l] = v / wders[0][0];
		}
	}
	return derivatives;
}


float NURBSSurface::binomial(int n, int k) {
   if (k == 0 || k == n)
   return 1.0;
   return binomial(n - 1, k - 1) + binomial(n - 1, k);
}

glm::vec3 NURBSSurface::Normal(float u, float v)
{
	int degreeU = degreeU_;
	int degreeV = degreeV_;
	std::vector<float> knotVectorU = knotsU_;
	std::vector<float> knotVectorV = knotsV_;
	std::vector<std::vector<glm::vec4>> controlPointsHomo;

    for (int i = 0; i < numRows_; i++){
        std::vector<glm::vec4> row;
        for (int j = 0; j < numCols_; j++){
            glm::vec3 point = control_points_[getIndex(i,j)];
            float weight = weights_[getIndex(i,j)];
            row.push_back(glm::vec4(point.x, point.y, point.z, weight));
        }
        controlPointsHomo.push_back(row);
    }

	std::vector<std::vector<glm::vec3>> derivatives = ComputeRationalSurfaceDerivatives(1, u, v);

    glm::vec3 dP_du = derivatives[1][0];
    glm::vec3 dP_dv = derivatives[0][1];


    return -glm::normalize(glm::cross(dP_du, dP_dv));

    // return glm::cross(glm::normalize(derivatives[1][0]), glm::normalize(derivatives[0][1]));
}


// refercing NURBS book and NURBS book implementation https://github.com/RoberAgro/nurbspy/blob/master/nurbspy/nurbs_surface.py#L412
float NURBSSurface::Nip_prime(int control_point_i, int degree, float time_u, std::vector<float> knots){
    int i = control_point_i;
    int p = degree;
    std::vector<float> U = knots;
    float u = time_u;

    float Nip1 = CalcNip(i, p-1, u, U) * p; // i , p-1
    float Nip2 =  CalcNip(i+1, p-1, u, U) * p; // i+1, p-1
    

    float denom1 = (U[i+p]-U[i]);
    float denom2 = (U[i+p+1]-U[i+1]);

    // std::cout << "denom 1 " << denom1 << std::endl; 
    // std::cout << "denom 2 " << denom2 << std::endl; 

    if (denom1 == 0){
        denom1 = 0;
    }else{
        denom1 = 1/denom1;
    }
    if (denom2 == 0){
        denom2 = 0;
    }else{
        denom2 = 1/denom2;
    }

    return (denom1) * Nip1 - (denom2) * Nip2;
}



// refercing NURBS book and NURBS book implementation https://github.com/RoberAgro/nurbspy/blob/master/nurbspy/nurbs_surface.py#L412
// std::vector<float> NURBSSurface::get_derivative(float u, float v, int order_u, int order_v){
//  // TO DO

//  glm::vec3 derivU(0.0f);
//  for (int i = 0; i < numRows_; i++){
//     for (int j = 0; j < numCols_; j++){
//        derivU += Nip_prime(i, degreeU_, u,knotsU_) * CalcNip(j, degreeV_, v, knotsV_) * control_points_[getIndex(i,j)];
//     }
//  }

// glm::vec3 derivV(0.0f);
//  for (int i = 0; i < numRows_; i++){
//     for (int j = 0; j < numCols_; j++){
//        derivV += CalcNip(i, degreeU_, u,knotsU_) * Nip_prime(j, degreeV_, v, knotsV_) * control_points_[getIndex(i,j)];
//     }
//  }

// }



// Formula from Wikipedia and Springer - The NURBS Book
NURBSPoint NURBSSurface::EvalPatch(float u, float v){
    // std::cout <<  "u " << u << " v " << v << std::endl; 
    NURBSPoint curve_point;
    curve_point.P = glm::vec3(0.0f);
    curve_point.T = glm::vec3(0.0f);
    float rationalWeight = 0.0;

    for (int p = 0; p < numRows_; p++){
        for (int q = 0; q < numCols_; q++){
            float Npn = CalcNip(p, degreeU_, u, knotsU_);
            float Nqm = CalcNip(q, degreeV_, v, knotsV_);
            rationalWeight += Npn * Nqm * weights_[getIndex(p,q)];
        }
    }

    for (int i = 0; i < numRows_; i++){
        for (int j = 0; j < numCols_; j++){
            float Nin = CalcNip(i, degreeU_, u, knotsU_);
            float Njm = CalcNip(j, degreeV_, v, knotsV_);
            
            if (rationalWeight != 0){
                curve_point.P += control_points_[getIndex(i,j)] * Nin * Njm * weights_[getIndex(i,j)] / rationalWeight;
            }

        }
    }

    curve_point.T = Normal(u,v);
    

//  glm::vec3 derivU(0.0f);
//  for (int i = 0; i < numRows_; i++){
//     for (int j = 0; j < numCols_; j++){
//        derivU += control_points_[getIndex(i,j)] * Nip_prime(i, degreeU_, u,knotsU_) * CalcNip(j, degreeV_, v, knotsV_);
//     //    std::cout <<  "NIP prime " << Nip_prime(i, degreeU_, u,knotsU_) << std::endl; 
//     //    std::cout <<  "Nip " << CalcNip(j, degreeV_, v, knotsV_) << std::endl; 
//     //    std::cout <<  Nip_prime(i, degreeU_, u,knotsU_) * CalcNip(j, degreeV_, v, knotsV_) << std::endl; 
//     }
//  }

// // std::cout << derivU.x << " " << derivU.y << " " << derivU.z << std::endl; 

// std::cout << "here2" << std::endl; 
//  glm::vec3 derivV(0.0f);
//  for (int i = 0; i < numRows_; i++){
//     for (int j = 0; j < numCols_; j++){
//        derivV += CalcNip(i, degreeU_, u,knotsU_) * Nip_prime(j, degreeV_, v, knotsV_) * control_points_[getIndex(i,j)];
//     }
//  }
// //  std::cout << derivV.x << " " << derivV.y << " " << derivV.z << std::endl; 

//  curve_point.T = glm::normalize(glm::cross(derivU, derivV));

//   std::cout << "tangent  " << curve_point.T.x << " " << curve_point.T.y << " " << curve_point.T.z << std::endl; 

return curve_point;
} 

void NURBSSurface::InitControlPoints(){
        // initialize control points
    for (int i = 0; i < control_points_.size(); i++) {
        auto point_node = make_unique<SceneNode>();
        point_node->GetTransform().SetPosition(control_points_[i]);

        point_node->CreateComponent<ShadingComponent>(shader_);
        
        auto& rc = point_node->CreateComponent<RenderingComponent>(sphere_mesh_);
        rc.SetDrawMode(DrawMode::Triangles);

        glm::vec3 color(1.f, 0.f, 0);
        auto material = std::make_shared<Material>(color, color, color, 0);
        point_node->CreateComponent<MaterialComponent>(material);

        control_point_nodes_.push_back(point_node.get());
        AddChild(std::move(point_node));
    }

    Material& material = control_point_nodes_[0]->GetComponentPtr<MaterialComponent>()->GetMaterial();
    glm::vec3 green_color(0.f, 1.f, 0.f); // green
    material.SetAmbientColor(green_color);
    material.SetDiffuseColor(green_color);
    material.SetSpecularColor(green_color);
}

void NURBSSurface::PlotSurface(){
  auto positions = make_unique<PositionArray>();
  auto normals = make_unique<NormalArray>();
  auto indices = make_unique<IndexArray>();

  // TODO: fill "positions", "normals", and "indices"
  float width_triangle = 1.0f / N_SUBDIV_;
  for (int i = 0; i < N_SUBDIV_; i++) {
    for (int j = 0; j < N_SUBDIV_; j++) {
      float left_u = (i + 1) * width_triangle;
      float left_v = j * width_triangle;
      NURBSPoint p0 = EvalPatch(left_u, left_v);
      NURBSPoint p1 = EvalPatch(left_u, left_v + width_triangle);
      NURBSPoint p2 = EvalPatch(left_u - width_triangle, left_v);
      NURBSPoint p3 = EvalPatch(left_u - width_triangle, left_v + width_triangle);

      positions->push_back(p0.P);
      positions->push_back(p1.P);
      positions->push_back(p2.P);
      positions->push_back(p3.P);

    //   std::cout << "tangent  " << p0.T.x << " " << p0.T.y << " " << p0.T.z << std::endl; 

      normals->push_back(p0.T);
      normals->push_back(p1.T);
      normals->push_back(p2.T);
      normals->push_back(p3.T);

      int pos_id = (N_SUBDIV_ * i + j) * 4;
      indices->push_back(pos_id);
      indices->push_back(pos_id + 1);
      indices->push_back(pos_id + 2);
      indices->push_back(pos_id + 2);
      indices->push_back(pos_id + 1);
      indices->push_back(pos_id + 3);
    }
  }


  patch_mesh_->UpdatePositions(std::move(positions));
  patch_mesh_->UpdateNormals(std::move(normals));
  patch_mesh_->UpdateIndices(std::move(indices));

  auto patch_single_node = make_unique<SceneNode>();
  patch_single_node->CreateComponent<ShadingComponent>(shader_);

  auto& rc = patch_single_node->CreateComponent<RenderingComponent>(patch_mesh_);
  rc.SetDrawMode(DrawMode::Triangles);

  patch_single_node->CreateComponent<MaterialComponent>(std::make_shared<Material>(Material::GetDefault()));

  AddChild(std::move(patch_single_node));

}


void NURBSSurface::UpdateSurface(){
    auto positions = make_unique<PositionArray>();
  auto normals = make_unique<NormalArray>();
  auto indices = make_unique<IndexArray>();

  // TODO: fill "positions", "normals", and "indices"
  float width_triangle = 1.0f / N_SUBDIV_;
  for (int i = 0; i < N_SUBDIV_; i++) {
    for (int j = 0; j < N_SUBDIV_; j++) {
      float left_u = (i + 1) * width_triangle;
      float left_v = j * width_triangle;
      NURBSPoint p0 = EvalPatch(left_u, left_v);
      NURBSPoint p1 = EvalPatch(left_u, left_v + width_triangle);
      NURBSPoint p2 = EvalPatch(left_u - width_triangle, left_v);
      NURBSPoint p3 = EvalPatch(left_u - width_triangle, left_v + width_triangle);

      positions->push_back(p0.P);
      positions->push_back(p1.P);
      positions->push_back(p2.P);
      positions->push_back(p3.P);

    //   std::cout << "tangent  " << p0.T.x << " " << p0.T.y << " " << p0.T.z << std::endl; 

      normals->push_back(p0.T);
      normals->push_back(p1.T);
      normals->push_back(p2.T);
      normals->push_back(p3.T);

      int pos_id = (N_SUBDIV_ * i + j) * 4;
      indices->push_back(pos_id);
      indices->push_back(pos_id + 1);
      indices->push_back(pos_id + 2);
      indices->push_back(pos_id + 2);
      indices->push_back(pos_id + 1);
      indices->push_back(pos_id + 3);
    }
  }


  patch_mesh_->UpdatePositions(std::move(positions));
  patch_mesh_->UpdateNormals(std::move(normals));
  patch_mesh_->UpdateIndices(std::move(indices));
}



}