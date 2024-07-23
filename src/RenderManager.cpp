#include "RenderManager.h"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <limits>
#include <math.h>
#include "glm/glm.hpp"
#include "ImageUtil.h"
#include "RayUtil.h"
using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::ofstream;
using glm::dvec3;
using glm::normalize;
using glm::dot;
using glm::cross;
using glm::length;

RenderManager::RenderManager(): images(),objs(),objsc(),camLensObj(0),
renderstyle(RENDERSTYLE_FAST),renderpointset(RENDERPOINTSET_DEFAULT),renderfalloff(RENDERFALLOFF_SQUARE),
brightnessMultiplier(10),lightpos(-3,5,0),lightbrightness(20),
camerax(0,2,1),cameraright(1,0,0),cameraup(0,1,0),cameraforward(0,0,-1),
camFocus(0.33),camD(0.2),camL(0.5),camR(0.7)
{
	//reserve the first three objects
	CollisionType mynull;
	mynull.objtype=0;
	objs.push_back(mynull);
	objs.push_back(mynull);
	objs.push_back(mynull);
	objsc.push_back(CollisionInformation());
	objsc.push_back(CollisionInformation());
	objsc.push_back(CollisionInformation());
}

RenderManager::~RenderManager()
{
}

void RenderManager::repositionCamLens(){
	createCamLens();
}
void RenderManager::createCamLens(){
	removeCamLens();
	setLens(camerax+camL*cameraforward,cameraforward,camFocus,camR*1.1,0.5,2,0,0,0);
}
void RenderManager::removeCamLens(){
	objs[0].objtype=0;
	objs[1].objtype=0;
	objs[2].objtype=0;
}
double RenderManager::lerp(double t,double a,double b) {
	return (b-a)*t+a;
}
double RenderManager::getUVIntensity(int imgnum,double x,double y) {
	if(x<=0||y<=0||x>=1||y>=1)
		return 1;
	DoubleImage &img=images[imgnum];
	x*=img.getWidth()-1;
	y*=img.getHeight()-1;
	int i=(int)x;
	int j=(int)y;
	double ifrac=x-i;
	double jfrac=y-j;
	return lerp(jfrac,lerp(ifrac,img.get(i,j),img.get(i+1,j)),lerp(ifrac,img.get(i,j+1),img.get(i+1,j+1)));
}

int RenderManager::getNextCollision(RayPropagator x){
	int n=-1;
	double mint=-1;
	for(int i=0;i<objs.size();i++){

		//failsafe:
		objsc[i]=CollisionInformation();
		objsc[i].collides=false;

		if(objs[i].objtype==1){
			objsc[i]=hitPlane(x,objs[i].pos,objs[i].dir1);
		} else if(objs[i].objtype==2){
			objsc[i]=hitSphere(x,objs[i].pos,objs[i].r);
		} else if(objs[i].objtype==3){
			objsc[i]=hitSphereCap(x,objs[i].pos,objs[i].r,objs[i].dir1,objs[i].nu);
		} else if(objs[i].objtype==4){
			objsc[i]=hitCylinderEdge(x,objs[i].pos,objs[i].dir1,objs[i].d,objs[i].r);
		} else if(objs[i].objtype==5) {
			objsc[i]=hitPlaneRect(x,objs[i].pos,objs[i].dir1,objs[i].dir2);
		}

		if(objsc[i].collides){
			if(n<0){
				n=i;
				mint=objsc[i].t;
			} else {
				if(objsc[i].t<mint){
					n=i;
					mint=objsc[i].t;
				}
			}
		}
	}
	return n;
}

int RenderManager::addImage(DoubleImage intensities) {
	images.push_back(intensities);
	return images.size()-1;
}
void RenderManager::addImagePlane(glm::dvec3 pos,glm::dvec3 dir1,dvec3 dir2,int image,double alpha,int reflecttype,double irradiance,double diffusivity) {
	CollisionType a;
	a.alpha=alpha;
	a.reflecttype=reflecttype;
	a.irradiance=irradiance;
	a.diffusivity=diffusivity;
	a.imgnum=image;
	a.pos=pos;
	a.dir1=dir1;
	a.dir2=dir2;
	a.objtype=5;
	objs.push_back(a);
	objsc.push_back(CollisionInformation());
}
void RenderManager::addPlane(glm::dvec3 pos,glm::dvec3 normal,double alpha,int reflecttype,double irradiance,double diffusivity){
	CollisionType a;
	a.pos=pos;
	a.dir1=normal;
	a.alpha=alpha;
	a.reflecttype=reflecttype;
	a.irradiance=irradiance;
	a.objtype=1;
	a.diffusivity=diffusivity;
	objs.push_back(a);
	objsc.push_back(CollisionInformation());
}

void RenderManager::addSphere(glm::dvec3 pos,double radius,double alpha,int reflecttype,double irradiance,double diffusivity){
	CollisionType a;
	a.objtype=2;
	a.pos=pos;
	a.r=radius;
	a.alpha=alpha;
	a.reflecttype=reflecttype;
	a.irradiance=irradiance;
	a.diffusivity=diffusivity;
	objs.push_back(a);
	objsc.push_back(CollisionInformation());
}

void RenderManager::setLens(glm::dvec3 pos,glm::dvec3 dir,double focal,double radius,double alpha,int reflecttype,double irradiance,double diffusivity,int i){
	if(i<0){
		i=objs.size();
		for(int q=0;q<3;q++){
			objs.push_back(CollisionType());
			objsc.push_back(CollisionInformation());
		}
	} else if(i+2>=objs.size()){
		cout<<"SetLens called with bad index."<<endl;
		return;
	}
	CollisionType &part1=objs[i];
	CollisionType &part2=objs[i+1];
	CollisionType &part3=objs[i+2];

	CollisionType a;
	a.alpha=alpha;
	a.reflecttype=reflecttype;
	a.irradiance=irradiance;
	a.diffusivity=diffusivity;


	//1/focal=(n-1)*2/ rad2
	double rad2=abs(focal*(1/alpha-1)*2);

	if(focal>0){
		if(rad2<radius){
			cout<<"addLens given too small a focal length"<<endl;
			return;
		}
		double x=sqrt(rad2*rad2-radius*radius);

		a.objtype=3;
		a.pos=pos-dir*x;
		a.dir1=dir;
		a.nu=radius/(rad2);
		a.nu=sqrt(1-a.nu*a.nu);
		a.r=rad2;
		part1=a;

		a.objtype=3;
		a.pos=pos+dir*x;
		a.dir1=-dir;
		a.nu=radius/(rad2);
		a.nu=sqrt(1-a.nu*a.nu);
		a.r=rad2;
		part2=a;

		cout<<"thickness: "<<2*(rad2-x)<<endl;

		part3.objtype=0;
	}
	else if(focal<0){
		focal=-focal;
		if(rad2<radius){
			cout<<"addLens given too small a focal length"<<endl;
			return;
		}

		double d0=0.01;
		double x=d0/2+rad2;
		a.objtype=3;
		a.pos=pos-dir*x;
		a.dir1=dir;
		a.nu=radius/(rad2);
		a.nu=sqrt(1-a.nu*a.nu);
		a.r=rad2;
		part1=a;

		a.objtype=3;
		a.pos=pos+dir*x;
		a.dir1=-dir;
		a.nu=radius/(rad2);
		a.nu=sqrt(1-a.nu*a.nu);
		a.r=rad2;
		part2=a;


		a.objtype=4;
		a.pos=pos;
		a.dir1=dir;
		a.r=radius;
		a.d=rad2-sqrt(rad2*rad2-radius*radius);
		part3=a;
	}
}

void RenderManager::addLens(glm::dvec3 pos,glm::dvec3 dir,double focal,double radius,double alpha,int reflecttype,double irradiance,double diffusivity){
	setLens(pos,dir,focal,radius,alpha,reflecttype,irradiance,diffusivity);
}


//sets the camera's position and orientation. (right,up,back) is orthonormalized automatically.
void RenderManager::setCamOrientation(glm::dvec3 pos,glm::dvec3 right,glm::dvec3 up,glm::dvec3 forward){
	orthonormalizethree(right,up,forward);
	camerax=pos;
	cameraright=right;
	cameraup=up;
	cameraforward=forward;
	if(renderstyle==RENDERSTYLE_FOCUSREAL) {
		repositionCamLens();
	}
}

//L-d is the screen-lens distance. R is the aperature radius. focus is the lense's focal length
void RenderManager::setCamFocus(double focus,double d,double L,double R){
	camFocus=focus;
	camD=d;
	camL=L;
	camR=R;
	if(renderstyle==RENDERSTYLE_FOCUSREAL) {
		repositionCamLens();
	}
}

//one of "RENDERSTYLE_..."
void RenderManager::setCamRenderStyle(int i) {
	renderstyle=i;
	if(renderstyle==RENDERSTYLE_FAST || renderstyle==RENDERSTYLE_UNFOCUSED){
		removeCamLens();
	} else if(renderstyle==RENDERSTYLE_FOCUS1) {
		removeCamLens();
	} else if(renderstyle==RENDERSTYLE_FOCUSREAL) {
		createCamLens();
	} 
}

//one of "RENDERPOINTSET_..."
void RenderManager::setCamRenderPoints(int j){
	renderpointset=j;
}
//one of "RENDERPOINTSET_..."
void RenderManager::setRenderFalloff(int j){
	renderfalloff=j;
}

//changing the "brightnessMultiplier" parameter will multiply all sources in the scene by its amount. 
//Set to 10 by default. Does not change the brightness of the light.
void RenderManager::setBrightnessMult(double arg){
	brightnessMultiplier=arg;
}

//sets the brightness of the light.
void RenderManager::setLightBrightness(double arg){
	lightbrightness=arg;
}

//there is one and only one light in the scene here.
void RenderManager::setLightPos(glm::dvec3 arg){
	lightpos=arg;
}

//Takes a picture and returns a black and white image
std::vector<double> RenderManager::snap(int width,int height,double mult,double aspect){
	vector<double> ret(width*height);
	for(int fsize=0;fsize<1;fsize++) {
		for(int x=0;x<width;x++){
			for(int y=0;y<height;y++){
				double l=(x*2.0/width-1)*mult*aspect;
				double h=(y*2.0/height-1)*mult;
				double mag=0;
				if(renderstyle==RENDERSTYLE_FAST){
					RayPropagator p=RayPropagator(dvec3(l*camD,h*camD,camD),normalize(dvec3(l*(-camR-camD),h*(-camR-camD),(camL-camD))),0);
					p=camToWorldspace(p,camerax,cameraright,cameraup,cameraforward);
					mag=getRayIntensity(p);
				} else if(renderstyle==RENDERSTYLE_FOCUS1) {
					double magtotal=0;
					for(int i=0; i<circleSamples(renderpointset);i++){
						RayPropagator p=getCamRayFocused(l,h,i,camFocus,camD,camL,camR,renderpointset);
						//cout<<"before"<<p.n.z<<endl;
						p=camToWorldspace(p,camerax,cameraright,cameraup,cameraforward);
						//cout<<p.n.z<<endl;
						magtotal+=getRayIntensity(p);
					}
					mag=magtotal/circleSamples(renderpointset);
				} else if(renderstyle==RENDERSTYLE_FOCUSREAL) {
					double magtotal=0;
					for(int i=0; i<circleSamples(renderpointset);i++){
						RayPropagator p=getCamRayUnfocused(l,h,i,camFocus,camD,camL,camR,renderpointset);
						p=camToWorldspace(p,camerax,cameraright,cameraup,cameraforward);
						magtotal+=getRayIntensity(p);
					}
					mag=magtotal/circleSamples(renderpointset);
				} else if(renderstyle==RENDERSTYLE_UNFOCUSED) {
					double magtotal=0;
					for(int i=0; i<circleSamples(renderpointset);i++){
						RayPropagator p=getCamRayUnfocused(l,h,i,camFocus,camD,camL,camR,renderpointset);
						p=camToWorldspace(p,camerax,cameraright,cameraup,cameraforward);
						//cout<<p.n.z<<endl;
						magtotal+=getRayIntensity(p);
					}
					mag=magtotal/circleSamples(renderpointset);
				}
				ret[y*width+x]=mag;
			}
		}
	}
	return ret;
}

CollisionType& RenderManager::getObject(int i) {
	return objs[i];
}

double RenderManager::getRayIntensity(RayPropagator x, int n){
	if(n>10)
		return 0;


	int i=getNextCollision(x);
	if(i<0)
		return 0;

	CollisionInformation objcol=objsc[i];


	double newt=x.r+objcol.t;
	double mag=0;
	double multiplier=1;
	if(objs[i].objtype==5)
		multiplier=getUVIntensity(objs[i].imgnum,dot(objcol.x-objs[i].pos,objs[i].dir1)/dot(objs[i].dir1,objs[i].dir1),dot(objcol.x-objs[i].pos,objs[i].dir2)/dot(objs[i].dir2,objs[i].dir2));

	double falloffmultiplier=1;
	if(renderfalloff==RENDERFALLOFF_LINEAR)
		falloffmultiplier=1/newt;
	if(renderfalloff==RENDERFALLOFF_SQUARE)
		falloffmultiplier=1/(newt*newt);

	if(objs[i].diffusivity>0){
		dvec3 dirToLight=normalize(lightpos-objcol.x);
		mag=objs[i].diffusivity*dot(objcol.m,dirToLight)*lightbrightness*falloffmultiplier*multiplier;
	} else if(objs[i].diffusivity<0){
		dvec3 dirToLight=normalize(lightpos-objcol.x);
		int odd=int(abs(floor(objcol.x.x)+floor(objcol.x.z)+floor(objcol.x.y-0.1)))%2;
		double darksquare=.1;
		mag=dot(objcol.m,dirToLight)*(odd+darksquare)/(1+darksquare)*lightbrightness*falloffmultiplier*multiplier;
	}

	if(objs[i].irradiance>0)
		mag+=objs[i].irradiance/(newt*newt)*multiplier;


	if(objs[i].reflecttype==0){
		return mag;
	} else if(objs[i].reflecttype==1) {
		dvec3 newdir=reflectionDirection(x.n,objcol.m);
		return mag+getRayIntensity(RayPropagator(objcol.x+newdir*0.001,newdir,newt),n+1);
	} else if(objs[i].reflecttype==2) {
		dvec3 newdir1=reflectionDirection(x.n,objcol.m);
		double alpha=objs[i].alpha;
		if(objcol.isinternal)
			alpha=1/alpha;
		double mag1=reflectionCoefficient(alpha,dot(x.n,objcol.m));
		double transmit2=getRayIntensity(RayPropagator(objcol.x+newdir1*0.001,newdir1,newt),n+1);

		mag+=mag1*transmit2;

		if(mag1<.999)
		{
			double mag2=1-mag1;
			dvec3 newdir2=transmissionDirection(x.n,objcol.m,alpha);
			//for(int k=0;k<10;k++)
			//cout<<objsc[i].x.z<<endl;
			double transmit=getRayIntensity(RayPropagator(objcol.x+newdir2*0.001,newdir2,newt),n+1);
			mag+=mag2*transmit;
		}
		return mag;

	}


	return 0;
}

/*
void RenderManager::renderImage(){
	dvec3 spherepos(1,3.4,-4.3);
	dvec3 spherepos2(1,2.4,-2.3);
	double sphererad=2;
	addPlane(dvec3(0,0,0),dvec3(0,1,0),1,0,0,-1);
	addSphere(spherepos,sphererad,1,1,0,0.4);
	//addSphere(spherepos2,7,8,2,0);
	addSphere(spherepos2-dvec3(1,1,0),0.7,1,1,0,0.4);
	addLens(dvec3(-1,2,-1),normalize(dvec3(-.1,0,1)),3,1.5,.8,2,0,0);



	dvec3 lightpos(-3,5,0);
	dvec3 planenormal(0,1,0);


	int w=800;
	int h=800;
	Image img(w,h);
	dvec3 camerax(0,2,1);
	dvec3 cameraright(1,0,0);
	dvec3 cameraup(0,1,0);
	dvec3 cameraforward(0,0,-1);
	int j=1;
	for(int fsize=0;fsize<1;fsize++) {
		for(int x=0;x<w;x++){
			for(int y=0;y<h;y++){
				double l=(x*2.0/w-1)*1.2;
				double h=(y*2.0/w-1)*1.2;
				double magtotal=0;
				RayPropagator p=camToWorldspace(RayPropagator(dvec3(l*d,h*d,-d),normalize(dvec3(l,h,1)),0),camerax,cameraright,cameraup,cameraforward);
				double mag=getRayIntensity(p);
				for(int i=0; i<circleSamples(j);i++){
					RayPropagator p=camToWorldspace(getCamRayFocused(l,h,i,0.325+fsize*.001,0.2,0.5,0.3,j),camerax,cameraright,cameraup,cameraforward);
					magtotal+=getRayIntensity(p);
				}
				double mag=magtotal/circleSamples(j);
				img.put(x,y,floatToRGB(mag,mag,mag));


			}
		}
		img.save("sphere"+std::to_string(3)+".bmp");
	}

}*/
