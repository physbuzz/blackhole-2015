#pragma once
#include <iostream>
#include <vector>
#include "glm/glm.hpp"

//structure that will store the ray propagation information. 
struct RayPropagator{
	RayPropagator(glm::dvec3 x,glm::dvec3 n,double r): x(x),n(n),r(r){}
	glm::dvec3 x; //start position (t=0)
	glm::dvec3 n; //direction (NORMALIZED. vec=n*t+x, where t is unitless & parameterizes the line)
	double r; //path length so far.
};

struct CollisionInformation{
	CollisionInformation(): x(glm::dvec3(0,0,0)),m(glm::dvec3(0,0,0)),t(0),collides(false),isinternal(false){}
	CollisionInformation(glm::dvec3 x,glm::dvec3 m,double t,bool collides,bool isinternal): x(x),m(m),t(t),collides(collides),isinternal(isinternal){}
	glm::dvec3 x; //position of the collision
	glm::dvec3 m; //normal of the collision, returned with the sign convention that dot(m,ray.n)<0.
	double t; //distance to the collision
	bool collides; //true if the ray intersects the object at all
	bool isinternal; //true if the ray is "in, going out". The meaning of this changes depending on the object, but a unique answer is guaranteed. (this is needed for determining how to apply index of refraction)

};


//MISC UTILS
int floatToRGB(double r,double g,double b,double a=0);
//Shortcut for ((double)rand())/RAND_MAX
double random2();
//Shortcut for ((double)rand())/RAND_MAX*(b-a)+a
double random2(double a,double b);

//writes all the binary data given to a file.
void writeCharFile(std::string name,std::vector<char> data);


//MISC VECTOR FUNCTIONS

//Orthonormalizes b wrt. a. returns the normalized version of b-(a.b)*a/(a.a)
glm::dvec3 orthonormalize(glm::dvec3 a,glm::dvec3 b);

void orthonormalizethree(glm::dvec3& a,glm::dvec3& b,glm::dvec3& c);


//REFLECTION/TRANSMISSION
//alpha is assumed to be the ratio of n1/n2 where n1 is the index on the side the normal points into.

//innerval is assumed to be the absolute value of the dot product of the ray direction and surface normal.
double reflectionCoefficient(double alpha,double innerval);

//innerval is assumed to be the absolute value of the dot product of the ray direction and surface normal.
double transmissionCoefficient(double alpha,double innerval);

double diffuseCoefficient(glm::dvec3 incident,glm::dvec3 normal);

glm::dvec3 reflectionDirection(glm::dvec3 incident,glm::dvec3 normal);

glm::dvec3 transmissionDirection(glm::dvec3 incident,glm::dvec3 normal,double alpha);


//COLLISION FUNCTIONS

CollisionInformation hitSphere(RayPropagator ray,glm::dvec3 c,double r);

CollisionInformation hitSphereCap(RayPropagator ray,glm::dvec3 c,double r,glm::dvec3 dir,double nu);

CollisionInformation hitCylinderEdge(RayPropagator ray,glm::dvec3 cx,glm::dvec3 cn,double d,double r);

CollisionInformation hitPlane(RayPropagator ray,glm::dvec3 cx,glm::dvec3 dir);

CollisionInformation hitPlaneRect(RayPropagator ray,glm::dvec3 cx,glm::dvec3 dir1,glm::dvec3 dir2);

//CAMERA SAMPLING FUNCTIONS	

int circleSamples(int j=0);
double circleSampleX(int i, int j=0);
double circleSampleY(int i, int j=0);

RayPropagator camToWorldspace(RayPropagator ray,glm::dvec3 pos,glm::dvec3 right,glm::dvec3 up,glm::dvec3 backwards);

//position on screen (l,h) (unitless), towards point i, focal length f, imaging plane distance d, lens distance L.
RayPropagator getCamRayUnfocused(double l,double h,int i,double f,double d,double L,double lenseR,int j=0);
RayPropagator getCamRayFocused(double l,double h,int i,double f,double d,double L,double lenseR,int j=0);



