#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <limits>
#include <math.h>
#include "glm/glm.hpp"
#include "ImageUtil.h"
#include "CImg.h"
#include "RayUtil.h"
using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::ofstream;



//commands used from code output by Mathematica
double Power(double x, int n){
    double ret=1;
    for(int i=0;i<n;i++)
        ret*=x;
    return ret;
}
double Power(double x, double n){
    return pow(x,n);
}
double Sqrt(double x){
    return sqrt(x);
}

glm::dvec4 accelerationVector(glm::dvec4 pos, glm::dvec4 vel){
    double t,x,y,z,dt,dx,dy,dz;
    t=pos.x; x=pos.y; y=pos.z; z=pos.w; 
    dt=vel.x; dx=vel.y; dy=vel.z; dz=vel.w; 

    double ddt=-((dt*(dx*x + dy*y + dz*z))/((Power(x,2) + Power(y,2) + Power(z,2))*(-1 + Sqrt(Power(x,2) + Power(y,2) + Power(z,2)))));
    
    double ddx=(x*(Power(dy,2)*Power(y,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + 2*dy*dz*y*z*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + Power(dz,2)*Power(z,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + 2*dy*dz*y*z*(-2 + 3*Power(x,2) + 3*Power(y,2) + 3*Power(z,2)) - Power(dz,2)*(2*(-1 + Power(x,2) + Power(y,2))*(Power(x,2) + Power(y,2)) + (Power(x,2) + Power(y,2))*Power(z,2) - Power(z,4)) + Power(dt,2)*(-1 + Power(x,2) + Power(y,2) + Power(z,2))*(-Power(x,2) - Power(y,2) - Power(z,2) + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) + 2*dx*x*(dy*y + dz*z)*(-2 + 3*Power(x,2) + 3*Power(y,2) + 3*Power(z,2) + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) - Power(dy,2)*(2*Power(x,4) - Power(y,4) + (-2 + Power(y,2))*Power(z,2) + 2*Power(z,4) + Power(x,2)*(-2 + Power(y,2) + 4*Power(z,2))) + Power(dx,2)*(Power(x,4) - 2*(-1 + Power(y,2) + Power(z,2))*(Power(y,2) + Power(z,2)) + Power(x,2)*(-Power(y,2) - Power(z,2) + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))))))/(2.*(-1 + Power(x,2) + Power(y,2) + Power(z,2))*Power(Power(x,2) + Power(y,2) + Power(z,2),2.5));
    
    double ddy=(y*(Power(dy,2)*Power(y,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + 2*dy*dz*y*z*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + Power(dz,2)*Power(z,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + 2*dy*dz*y*z*(-2 + 3*Power(x,2) + 3*Power(y,2) + 3*Power(z,2)) - Power(dz,2)*(2*(-1 + Power(x,2) + Power(y,2))*(Power(x,2) + Power(y,2)) + (Power(x,2) + Power(y,2))*Power(z,2) - Power(z,4)) + Power(dt,2)*(-1 + Power(x,2) + Power(y,2) + Power(z,2))*(-Power(x,2) - Power(y,2) - Power(z,2) + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) + 2*dx*x*(dy*y + dz*z)*(-2 + 3*Power(x,2) + 3*Power(y,2) + 3*Power(z,2) + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) - Power(dy,2)*(2*Power(x,4) - Power(y,4) + (-2 + Power(y,2))*Power(z,2) + 2*Power(z,4) + Power(x,2)*(-2 + Power(y,2) + 4*Power(z,2))) + Power(dx,2)*(Power(x,4) - 2*(-1 + Power(y,2) + Power(z,2))*(Power(y,2) + Power(z,2)) + Power(x,2)*(-Power(y,2) - Power(z,2) + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))))))/(2.*(-1 + Power(x,2) + Power(y,2) + Power(z,2))*Power(Power(x,2) + Power(y,2) + Power(z,2),2.5));
    
    double ddz=(z*(Power(dy,2)*Power(y,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + 2*dy*dz*y*z*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + Power(dz,2)*Power(z,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + 2*dy*dz*y*z*(-2 + 3*Power(x,2) + 3*Power(y,2) + 3*Power(z,2)) - Power(dz,2)*(2*(-1 + Power(x,2) + Power(y,2))*(Power(x,2) + Power(y,2)) + (Power(x,2) + Power(y,2))*Power(z,2) - Power(z,4)) + Power(dt,2)*(-1 + Power(x,2) + Power(y,2) + Power(z,2))*(-Power(x,2) - Power(y,2) - Power(z,2) + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) + 2*dx*x*(dy*y + dz*z)*(-2 + 3*Power(x,2) + 3*Power(y,2) + 3*Power(z,2) + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) - Power(dy,2)*(2*Power(x,4) - Power(y,4) + (-2 + Power(y,2))*Power(z,2) + 2*Power(z,4) + Power(x,2)*(-2 + Power(y,2) + 4*Power(z,2))) + Power(dx,2)*(Power(x,4) - 2*(-1 + Power(y,2) + Power(z,2))*(Power(y,2) + Power(z,2)) + Power(x,2)*(-Power(y,2) - Power(z,2) + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))))))/(2.*(-1 + Power(x,2) + Power(y,2) + Power(z,2))*Power(Power(x,2) + Power(y,2) + Power(z,2),2.5));

    return glm::dvec4(ddt,ddx,ddy,ddz);
}
double tvelocity(glm::dvec4 pos, glm::dvec4 vel){
    double t,x,y,z,dt0,dx0,dy0,dz0;
    t=pos.x; x=pos.y; y=pos.z; z=pos.w; 
    dt0=vel.x; dx0=vel.y; dy0=vel.z; dz0=vel.w; 
    double var= Sqrt((2*dx0*dz0*x*z*(-1 + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) + 2*dy0*y*(dx0*x + dz0*z)*(-1 + Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) + Power(dy0,2)*(Power(x,4) + Power(z,2) + Power(Power(y,2) + Power(z,2),2) - 2*Power(x,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) - Power(y,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) - 2*Power(z,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + Power(x,2)*(1 + 2*Power(y,2) + 2*Power(z,2))) + Power(dz0,2)*(Power(x,4) + Power(y,4) + Power(z,4) - Power(z,2)*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)) + Power(y,2)*(1 + 2*Power(z,2) - 2*Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) + Power(x,2)*(1 + 2*Power(y,2) + 2*Power(z,2) - 2*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)))) + Power(dx0,2)*(Power(x,4) + (Power(y,2) + Power(z,2))*(1 + Power(y,2) + Power(z,2) - 2*Sqrt(Power(x,2) + Power(y,2) + Power(z,2))) + Power(x,2)*(2*Power(y,2) + 2*Power(z,2) - Sqrt(Power(x,2) + Power(y,2) + Power(z,2)))))/((Power(x,2) + Power(y,2) + Power(z,2))*(1 + Power(x,2) + Power(y,2) + Power(z,2) - 2*Sqrt(Power(x,2) + Power(y,2) + Power(z,2)))))/Sqrt(1 - 1/Sqrt(Power(x,2) + Power(y,2) + Power(z,2)));

    return var;
}

void correctLightlikeRay(glm::dvec4& pos,glm::dvec4 &vel){
    vel.x=tvelocity(pos,vel);
}

bool integrateRayStep(glm::dvec4 &pos, glm::dvec4 &vel,double dTau){
    vel+=accelerationVector(pos,vel)*dTau;
    pos+=vel*dTau;
    if(pos.y*pos.y+pos.z*pos.z+pos.w*pos.w<1.2)
        return false;
    return true;
}


glm::dvec2 CartesianToPolar(double x, double y, double z){
    return glm::dvec2(atan2(y,x),atan2(sqrt(x*x+y*y),z));
}
glm::dvec2 CartesianToPolar(glm::dvec4 v){
    return CartesianToPolar(v.y,v.z,v.w);
}
double lerp(double a, double b, double t) {
    return (b-a)*t+a;
}


int getImageColor(cimg_library::CImg<unsigned char> &image, double phi, double theta){
    double xval=image.width()*(phi+3.141592)/(2*3.141592);
    double yval=image.height()*(theta)/(3.141592);
    int x=floor(xval);
    int y=floor(yval);
    double interpx=xval-x;
    double interpy=yval-y;

    if(x<0 || x>=image.width() || y<0 || y>=image.height())
        return floatToRGB(0.0,0.0,0.0);


    double r00=(double(image(x,y,0,0))/255.0);
    double g00=(double(image(x,y,0,1))/255.0);
    double b00=(double(image(x,y,0,2))/255.0);
    if(x+1>=image.width() || y+1>=image.height())
        return floatToRGB(r00,g00,b00);

    double r10=(double(image(x+1,y,0,0))/255.0);
    double g10=(double(image(x+1,y,0,1))/255.0);
    double b10=(double(image(x+1,y,0,2))/255.0);

    double r01=(double(image(x,y+1,0,0))/255.0);
    double g01=(double(image(x,y+1,0,1))/255.0);
    double b01=(double(image(x,y+1,0,2))/255.0);

    double r11=(double(image(x+1,y+1,0,0))/255.0);
    double g11=(double(image(x+1,y+1,0,1))/255.0);
    double b11=(double(image(x+1,y+1,0,2))/255.0);

    double r=lerp(lerp(r00,r10,interpx),lerp(r01,r11,interpx),interpy);
    double g=lerp(lerp(g00,g10,interpx),lerp(g01,g11,interpx),interpy);
    double b=lerp(lerp(b00,b10,interpx),lerp(b01,b11,interpx),interpy);

    return floatToRGB(r,g,b);
}
int getImageColor(cimg_library::CImg<unsigned char> &image,glm::dvec2 ang){
    return getImageColor(image,ang.x,ang.y);
}

int main(){

/*
	cimg_library::CImg<unsigned char> image("lenna.bmp");
	DoubleImage dimg=DoubleImage(image.width(),image.height());
	for(int a=0;a<image.width();a++) {
		for(int b=0;b<image.height();b++) {
			if(i==0)
				dimg.put(a,b,double(image(a,b,0,0))/255);
			else if(i==1)
				dimg.put(a,b,double(image(a,b,0,1))/255);
			else if(i==2)
				dimg.put(a,b,double(image(a,b,2))/255);
		}
	}*/


	//dvec3 spherepos(0,3.4,4.3);
	//dvec3 spherepos2(1,2.4,2.3);
	//double sphererad=2;
	//x.addPlane(dvec3(0,0,0),dvec3(0,1,0),1,0,0,-1);
	//x.addSphere(spherepos,sphererad,1,1,0,0.4);
	//x.addSphere(spherepos2,0.4,1,1,0,0.4);
	//x.addSphere(spherepos2-dvec3(1,1,0),0.7,1,1,0,0.4);
	//addLens(dvec3(-1,2,-1),normalize(dvec3(-.1,0,1)),3,1.5,.8,2,0,0);
    
    glm::dvec2 pol=CartesianToPolar(-0.4,-0.4,-0.8);
    std::cout<<pol.x<<", "<<pol.y<<std::endl;

	cimg_library::CImg<unsigned char> image("jupiter.bmp");

    
	int imgnamecount=0;
	for(double q=0;q<2*3.141592;q+=2*3.1415923/100) {
        double f=1.2;
        glm::dvec4 pos=9.0*glm::dvec4(0.0,cos(q),sin(q),0.0);
        glm::dvec4 uvec=3.0*glm::dvec4(0.0,cos(q+f),sin(q+f),0.0);
        glm::dvec4 vvec=3.0*glm::dvec4(0.0,0.0,0.0,1.0);
        glm::dvec4 forwardvec=1.0*glm::dvec4(0.0,-sin(q+f),cos(q+f),0.0);


        int w=1200;
        int h=1200;
		Image img=Image(w,h);
        double dtau=0.05;
		for(int i=0;i<w;i++) {
			for(int j=0;j<h;j++) {

                glm::dvec4 position=pos;
                glm::dvec4 velocity=forwardvec+(double(i)/w-0.5)*uvec+(double(j)/h-0.5)*vvec;
                correctLightlikeRay(position,velocity);
                //std::cout<<"foo: "<<velocity.x<<std::endl;
                int n=0;
                bool c=true;
                while(n<200 && c){
                    c=integrateRayStep(position,velocity,dtau);
                    n++;
                }
                if(c)
				    img.put(i,j,getImageColor(image,CartesianToPolar(velocity)));
                else
				    img.put(i,j,floatToRGB(0.0,0.0,0.0));
                //std::cout<<"rfinal: "<<sqrt(position.y*position.y+position.z*position.z+position.w*position.w)<<std::endl;
			}
            std::cout<<"Line: "<<i<<std::endl;
            
		}
		string add;
		if(imgnamecount<10)
			add="0"+std::to_string(imgnamecount);
		else
			add=std::to_string(imgnamecount);

		img.save("color"+add+".bmp");
		//cout<<"Saved!"<<endl;
		imgnamecount++;
	}

	//myimg.save("snapshot1.bmp");
	
	/*	cimg_library::CImg<unsigned char> image("lenna.bmp");
	DoubleImage dimg=DoubleImage(image.width(),image.height());
	for(int a=0;a<image.width();a++) {
		for(int b=0;b<image.height();b++) {
			dimg.put(a,b,double(image(a,b,0,0))/255);
		}
	}
	int imgnum=x.addImage(dimg);
	x.addImagePlane(dvec3(-3,6,2),dvec3(4,0,1),dvec3(0,-5,0),imgnum,0,0,0.1,.4);*/

	//x.addLens(dvec3(0,2,-.3),normalize(dvec3(0,0,1)),0.33,.33,.5,2,0,0);
	/*
	int size1=400;
	int size2=200;
	Image myimg=Image(size1,size2);
		x.setCamFocus(.385,.15,.6,.1);
	x.setCamRenderStyle(RENDERSTYLE_UNFOCUSED);
	myimg=Image(x.snap(size1,size2,1,2),size1,size2);
	myimg.save("unfocus.bmp");
		
	x.setCamRenderStyle(RENDERSTYLE_FAST);
	myimg=Image(x.snap(size1,size2,1,2),size1,size2);
	myimg.save("fast.bmp");
	for(int i=0;i<42;i++) {
		std::string start="test";
		std::string add="";
		if(i<10)
			add+="0";
		x.setCamFocus(.385+i*0.001,.15,.6,.1);

		
		x.setCamRenderStyle(RENDERSTYLE_FOCUSREAL);
		myimg=Image(x.snap(size1,size2,1,2),size1,size2);
		myimg.save("real"+add+std::to_string(i)+".bmp");
		
		x.setCamRenderStyle(RENDERSTYLE_FOCUS1);
		myimg=Image(x.snap(size1,size2,1,2),size1,size2);
		myimg.save("ideal"+add+std::to_string(i)+".bmp");
	}*/
	return 0;
}

