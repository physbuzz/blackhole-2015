#pragma once
#include "RayUtil.h"
#include "ImageUtil.h"
#include <iostream>
#include <vector>
struct CollisionType{
	int objtype; //0 is ignore. 1 is plane. 2 is sphere. 3 is sphere cap. 4 is cylinder edge. 5 is image plane.
	glm::dvec3 pos;
	glm::dvec3 dir1;
	glm::dvec3 dir2;
	double d;
	double r;
	double nu;
	int imgnum;
	double alpha;
	int reflecttype; //1 is perfect reflection. 2 is reflection according to alpha. 0 is no reflection.
	bool checkerboard;
	double irradiance; //a positive value gives constant irradiance over the surface. A value of zero gives zero, and less than zero gives irradiance calculated by path to light.
	double diffusivity;
};

const int RENDERSTYLE_FAST=1;
const int RENDERSTYLE_FOCUS1=2;
const int RENDERSTYLE_FOCUSREAL=3;
const int RENDERSTYLE_UNFOCUSED=4;

const int RENDERPOINTSET_DEFAULT=0;
const int RENDERPOINTSET_LARGE=1;

const int RENDERFALLOFF_SQUARE=1;
const int RENDERFALLOFF_LINEAR=2;


class RenderManager
{
	std::vector<DoubleImage> images;
	//the internal state of this is such that the first three objects are reserved for the possibility of a "physical" camera lens.
	std::vector<CollisionType> objs;
	std::vector<CollisionInformation> objsc;

	int camLensObj;

	int renderstyle;
	int renderpointset;
	int renderfalloff;

	double brightnessMultiplier;
	glm::dvec3 lightpos;
	double lightbrightness;

	glm::dvec3 camerax;
	glm::dvec3 cameraright;
	glm::dvec3 cameraup;
	glm::dvec3 cameraforward;

	double camFocus;
	double camD;
	double camL;
	double camR;

	void repositionCamLens();
	void createCamLens();
	void removeCamLens();
	int getNextCollision(RayPropagator x);
	void setLens(glm::dvec3 pos,glm::dvec3 dir,double focal,double size,double alpha,int reflecttype,double irradiance,double diffusivity,int i=-1);
	double lerp(double t,double a,double b);
	double getUVIntensity(int imgnum,double x,double y);
public:
	//sets the camera's position and orientation. (right,up,back) is orthonormalized automatically.
	void setCamOrientation(glm::dvec3 pos,glm::dvec3 right,glm::dvec3 up,glm::dvec3 back); 

	//sets the camera's orientation through Euler angles
	//void setCamByAngle(glm::dvec3 pos,double a,double b,double c);

	//L-d is the screen-lens distance. R is the aperature radius. focus is the lense's focal length
	void setCamFocus(double focus,double d,double L,double R);

	//one of "RENDERSTYLE_..."
	void setCamRenderStyle(int i);

	//one of "RENDERPOINTSET_..."
	void setCamRenderPoints(int j);

	//One of "RENDERFALLOFF_..."
	void setRenderFalloff(int arg);

	//changing the "brightnessMultiplier" parameter will multiply all sources in the scene by its amount. 
	//Set to 10 by default. Does not change the brightness of the light.
	void setBrightnessMult(double arg);

	//sets the brightness of the light.
	void setLightBrightness(double arg);

	//there is one and only one light in the scene here.
	void setLightPos(glm::dvec3 arg);


	//Takes a picture and returns a black and white image
	std::vector<double> snap(int width,int height,double mult,double aspect);

	void addPlane(glm::dvec3 pos,glm::dvec3 normal,double alpha,int reflecttype,double irradiance,double diffusivity);
	void addSphere(glm::dvec3 pos,double radius,double alpha,int reflecttype,double irradiance,double diffusivity);
	void addLens(glm::dvec3 pos,glm::dvec3 dir,double focal,double size,double alpha,int reflecttype,double irradiance,double diffusivity);

	int addImage(DoubleImage intensities);
	void addImagePlane(glm::dvec3 pos,glm::dvec3 dir1,glm::dvec3 dir2,int image,double alpha,int reflecttype,double irradiance,double diffusivity);
	void clearObjects();
	RenderManager();
	~RenderManager();
	double getRayIntensity(RayPropagator x, int n=0);

	CollisionType& getObject(int i);
};

