/*!
 *
 * \author VaHiD AzIzI
 *
 */


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

double dot_product(Util::Vector a, Util::Vector b)
{
	double ans = a.x*b.x + a.y*b.y + a.z*b.z;
	return ans;
}


/*
Util::Vector triple_product(Util::Vector a, Util::Vector b, Util::Vector c)
{
	Util::Vector ans = b*(dot_product(a, c)) - a*(dot_product(b, c));
	return ans;
}
*/

Util::Vector per_product(Util::Vector a)
{
	Util::Vector ans = {-a.z, a.y, a.x};
	return ans;
}

Util::Vector ahh_normalize(Util::Vector a)
{
	Util::Vector ans = {0.0f, 0.0f, 0.0f};
	//printf("LEN: %d\n", a.length());
	if (a.length() == 0.0f)
		return ans;
	else
		ans = {a.x/fabs(a.length()), a.y/fabs(a.length()), a.z/fabs(a.length())};
	return ans;
}

void find_closest_edge(std::vector<Util::Vector> simplex, double& edge_dist, Util::Vector& edge_normal, int& edge_ind)
{
	for (int i = 0; i < simplex.size(); i++) {
		int j = 0;
		(i+1 == simplex.size()) ? j = 0 : j = i+1;
		Util::Vector a = simplex[i];
		Util::Vector b = simplex[j];
		Util::Vector c = b - a;
		//printf("A: %f, %f, %f\n", a.x, a.y, a.z);
		//printf("B: %f, %f, %f\n", b.x, b.y, b.z);
		//printf("C: %f, %f, %f\n", c.x, c.y, c.z);
		Util::Vector n = per_product(c);
		//printf("N: %f, %f, %f\n", n.x, n.y, n.z);
		n = normalize(n);
		//printf("NNORM: %f, %f, %f\n", n.x, n.y, n.z);
		double d = dot_product(n, a);
		//printf("D: %f\n", d);

		if (d < edge_dist || i == 0) {
			edge_dist = d;
			edge_normal = n;
			edge_ind = i;
		}
		//printf("FIND EDGE DIST: %f\n", edge_dist);
	}
}

//TODO : Remove function while submitting
void printSimplex(std::vector<Util::Vector> simplex) {
	for(int i=0; i<simplex.size(); i++){
		std::cout<<simplex[i]<<", ";
	}
	std::cout<<std::endl;
}

Util::Vector getFarthestPoint(const std::vector<Util::Vector>& polygon, Util::Vector d) {


	float maxDist = polygon[0] * d;
	int indx = 0;
	for(int i=1; i<polygon.size(); i++){
		if(maxDist < (polygon[i] * d)) {
			maxDist = polygon[i] * d;
			indx = i;
		}
	}

	return polygon[indx];
}

Util::Vector getSupport(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, Util::Vector d) {

	return (getFarthestPoint(shapeA, d) - getFarthestPoint(shapeB, -d));
}

Util::Vector updateDirection(std::vector<Util::Vector>& simplex, Util::Vector d) {

	Util::Vector a = -(simplex[simplex.size()-1]);
	if (simplex.size() == 2) {

		Util::Vector vectorAB = simplex[0] + a;
		float perpABx = vectorAB.z*((a.x * vectorAB.z) - (a.z * vectorAB.x));
		float perpABy = vectorAB.x*(- (a.x * vectorAB.z) + (a.z * vectorAB.x));
		Util::Vector perpendAB = Util::Vector(perpABx, 0, perpABy);
		d = perpendAB;
	} else {

		d = a;
	}

	return d;
}

bool isOriginEnclosed(std::vector<Util::Vector>& simplex, Util::Vector& d) {

	if(simplex.size() == 3){

		Util::Vector a = -simplex[2];
		Util::Vector b = -simplex[1];
		Util::Vector c = -simplex[0];
		Util::Vector vectorAB = simplex[1] + a;
		Util::Vector vectorAC = simplex[0] + a;

		float perpABx = vectorAB.z*((a.x * vectorAB.z) - (a.z * vectorAB.x));
		float perpABy = vectorAB.x*(- (a.x * vectorAB.z) + (a.z * vectorAB.x));
		Util::Vector perpendAB = Util::Vector(perpABx, 0, perpABy);

		float perpACx = vectorAC.z*((a.x * vectorAC.z) - (a.z * vectorAC.x));
		float perpACy = vectorAC.x*(- (a.x * vectorAC.z) + (a.z * vectorAC.x));
		Util::Vector perpendAC = Util::Vector(perpACx, 0, perpACy);

		if(perpendAB * c < 0) {
			if (perpendAC * b < 0) {
				return true;
			} else {

				simplex.erase(simplex.begin()+1);
				d = perpendAC;
			}
		} else {

			simplex.erase(simplex.begin());
			d = perpendAB;
		}
	} else {

		d = updateDirection(simplex, d);
	}
	return false;
}

bool GJK(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, std::vector<Util::Vector>& simplex) {


	Util::Vector d = Util::Vector(1,0,-1);
	simplex.push_back(getSupport(shapeA, shapeB, d));

	while(1) {

		if (((-(simplex.back())) * d) > 0) {

			return false;
		} else {

			if (isOriginEnclosed(simplex, d))
			return true;
			else
			simplex.push_back(getSupport(shapeA, shapeB, d));

		}

	}
}

void EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector> simplex)
{
	double edge_dist = 0.0f;
	Util::Vector edge_normal = {0.0f, 0.0f, 0.0f};
	int edge_ind = 0;

	const double TOLERANCE = 0.01f;
	double dist = 0.0f;

	do {
		find_closest_edge(simplex, edge_dist, edge_normal, edge_ind);
		//printf("EDGE DIST: %f\n EDGE NORMAL: %f, %f, %f\n EDGE IND: %d\n", edge_dist, edge_normal.x, edge_normal.y, edge_normal.z, edge_ind);
		Util::Vector supportp = getSupport(_shapeA, _shapeB, edge_normal);
		//printf("SUPPORTP: %f, %f, %f\n", supportp.x, supportp.y,
		//       supportp.z);
		dist = dot_product(edge_normal, supportp);
		//printf("DIST: %f\n", dist);
		simplex.insert(simplex.begin()+edge_ind+1, supportp);
	} while (dist - edge_dist > TOLERANCE);

	return_penetration_depth = dist;
	return_penetration_vector = edge_normal;
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	simplex.clear();
	return_penetration_depth = 0;

	if(GJK(_shapeA, _shapeB, simplex)) {
		EPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, simplex);
		return true;
	}
	return false;
}
