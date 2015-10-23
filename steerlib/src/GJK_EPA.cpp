/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
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

void EPA(std::vector<Util::Vector> simplex, float& penetration_depth, Util::Vector& penetration_vector) {

	//TODO: Implement EPA here
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	simplex.clear();
	return_penetration_depth = 0;

	if(GJK(_shapeA, _shapeB, simplex)) {
		EPA(simplex, return_penetration_depth, return_penetration_vector);
		return true;
	}
    return false;
}
