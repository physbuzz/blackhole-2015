#include "RayUtil.h"
#include <fstream>
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

double random2(){
	return ((double)rand())/RAND_MAX;
}
double random2(double a,double b){
	return random2()*(b-a)+a;
}
void writeCharFile(string name,vector<char> data){
	ofstream output(name.c_str(),ofstream::out|ofstream::binary);
	output.write(&data[0],sizeof(char)*data.size());
	output.close();
}
int floatToRGB(double r,double g,double b,double a){
	int a1=(int)(255*a);
	int r1=(int)(255*r);
	int g1=(int)(255*g);
	int b1=(int)(255*b);
	a1=a1>0?(a1>255?255:a1):0;
	r1=r1>0?(r1>255?255:r1):0;
	g1=g1>0?(g1>255?255:g1):0;
	b1=b1>0?(b1>255?255:b1):0;
	return (a1<<24)|(r1<<16)|(g1<<8)|b1;
}


//returns the gram-schmidt orthonormalized vector b.
dvec3 orthonormalize(dvec3 a,dvec3 b){
	//return b-(a.b)*a/(a.a)
	double l=dot(a,b)/dot(a,a);
	dvec3 c(b.x-l*a.x,b.y-l*a.y,b.z-l*a.z);
	double d=length(c);
	return dvec3(c.x/d,c.y/d,c.z/d);
}
void orthonormalizethree(dvec3& a,dvec3& b, dvec3& c){
	//return b-(a.b)*a/(a.a)
	double l=dot(a,b)/dot(a,a);
	a=normalize(a);
	b=normalize(b-a*dot(a,b));
	c=normalize(c-a*dot(a,c)-b*dot(b,c));
}

double reflectionCoefficient(double alpha,double innerval){
	innerval=abs(innerval);
	if(1-alpha*alpha*(1-innerval*innerval)<0)
		return 1;
	double disc=sqrt(1-alpha*alpha*(1-innerval*innerval));
	double b1=(alpha*innerval-disc)/(alpha*innerval+disc);
	double b2=(alpha*disc-innerval)/(alpha*disc+innerval);
	return (b1*b1+b2*b2)/2.0;
}

double transmissionCoefficient(double alpha,double innerval) {
	return 1-reflectionCoefficient(alpha,innerval);
}

double diffuseCoefficient(glm::dvec3 incident,glm::dvec3 normal) {
	return abs(dot(incident,normal));
}

glm::dvec3 reflectionDirection(glm::dvec3 incident,glm::dvec3 normal) {
	return incident-2*dot(incident,normal)*normal;
}

glm::dvec3 transmissionDirection(glm::dvec3 incident,glm::dvec3 normal,double alpha) {
	double innerval=dot(normal,incident);
	double disc=sqrt(1-alpha*alpha*(1-innerval*innerval));
	return alpha*incident+normal*(alpha*abs(innerval)-disc);
}

CollisionInformation hitSphere(RayPropagator ray,glm::dvec3 c,double r) {
	dvec3 diff=ray.x-c;
	//we have the equation that t must satisfy t^2+2t(x-c).n+||x-c||^2-r^2=0
	//the equation t^2+2ta+b=0 has the nice solution t=-a (+-) sqrt(a^2-b)
	double a=dot(diff,ray.n);
	double b=dot(diff,diff)-r*r;

	double c2=a*a-b;
	if(c2>0) {
		c2=sqrt(c2);
		if(-a-c2>0){ //collision from the outside
			double t=-a-c2;
			dvec3 pos=ray.x+ray.n*t;
			dvec3 m=normalize(pos-c);
			return CollisionInformation(pos,m,t,true,false);
		} else if(-a+c2>0) { //collision from the inside
			double t=-a+c2;
			dvec3 pos=ray.x+ray.n*t;
			dvec3 m=normalize(pos-c);
			return CollisionInformation(pos,-m,t,true,true);
		} else {
			return CollisionInformation(dvec3(),dvec3(),0,false,false);
		}
	} else {
		return CollisionInformation(dvec3(),dvec3(),0,false,false);
	}
}

CollisionInformation hitSphereCap(RayPropagator ray,glm::dvec3 c,double r,glm::dvec3 dir,double nu){


	dvec3 diff=ray.x-c;
	//we have the equation that t must satisfy t^2+2t(x-c).n+||x-c||^2-r^2=0
	//the equation t^2+2ta+b=0 has the nice solution t=-a (+-) sqrt(a^2-b)
	double a=dot(diff,ray.n);
	double b=dot(diff,diff)-r*r;

	double c2=a*a-b;
	if(c2>0) {
		c2=sqrt(c2);
		if(-a-c2>0){ //collision from the outside
			double t=-a-c2;
			dvec3 pos=ray.x+ray.n*t;
			dvec3 m=normalize(pos-c);
			double scalar=dot(m,dir);
			if(scalar>nu)
				return CollisionInformation(pos,m,t,true,false);
		}
		if(-a+c2>0) { //collision from the inside
			double t=-a+c2;
			dvec3 pos=ray.x+ray.n*t;
			dvec3 m=normalize(pos-c);
			double scalar=dot(m,dir);
			if(scalar>nu)
				return CollisionInformation(pos,-m,t,true,true);
		}
			
	}
	return CollisionInformation(dvec3(),dvec3(),0,false,false);

	/*
	CollisionInformation ret=hitSphere(ray,c,r);
	double scalar=dot(ret.m,dir);
	if(ret.isinternal)
		scalar=-scalar;
	if(scalar>nu)
		return ret;
	return CollisionInformation(dvec3(),dvec3(),0,false,false);*/
}

CollisionInformation hitCylinderEdge(RayPropagator ray,glm::dvec3 cx,glm::dvec3 cn,double d,double r){
	//This derives from necessitating that t satisfies:
	//||(ray.n*t+ray.x-cx) cross cn||^2=r^2
	//which states that (ray.n*t+ray.x-cx) lies on an infinite cylinder radius r direction n passing through the origin.
	dvec3 diff=ray.x-cx;
	dvec3 c1=cross(ray.n,cn);
	dvec3 c2=cross(diff,cn);
	double a=dot(c1,c1);
	double b=dot(c1,c2);
	double c=dot(c2,c2)-r*r;
	//the above expands into the equation at^2+2tb+c=0 whose solution is (-b (+-) sqrt(b^2-ac))/a
	double disc=b*b-a*c;
	if(disc>0){
		disc=sqrt(disc)/a;
		b=-b/a;
		double t1=b-disc;
		if(t1>0){
			dvec3 pos1=ray.n*t1+ray.x;
			double proj1=dot(pos1-cx,cn);
			if(-d<proj1 && proj1<d)
				return CollisionInformation(pos1,normalize(pos1-cx-proj1*cn),t1,true,false);
		}

		double t2=b+disc;
		if(t2>0){
			dvec3 pos2=ray.n*t1+ray.x;
			double proj2=dot(pos2-cx,cn);
			if(-d<proj2 && proj2<d)
				return CollisionInformation(pos2,normalize(pos2-cx-proj2*cn),t2,true,true);
		}
	}
	return CollisionInformation(dvec3(),dvec3(),0,false,false);
}

CollisionInformation hitPlane(RayPropagator ray,glm::dvec3 cx, glm::dvec3 dir) {
	double nPn=dot(ray.n,dir);
	if(nPn==0)
		return CollisionInformation(dvec3(),dvec3(),0,false,false);

	double t=dot(cx-ray.x,dir)/nPn;
	if(t<=0)
		return CollisionInformation(dvec3(),dvec3(),0,false,false);
	
	bool isinternal=(nPn>0);
	if(isinternal)
		dir=-dir;

	return CollisionInformation(ray.n*t+ray.x,dir,t,true,isinternal);
}
CollisionInformation hitPlaneRect(RayPropagator ray,glm::dvec3 cx,glm::dvec3 dir1,glm::dvec3 dir2) {
	CollisionInformation ret=hitPlane(ray,cx,normalize(cross(dir1,dir2)));
	double a=dot(ret.x-cx,dir1)/dot(dir1,dir1);
	double b=dot(ret.x-cx,dir2)/dot(dir2,dir2);
	if(a<0||a>1||b<0||b>1)
		ret.collides=false;
	return ret;
}

int circleSamples(int j) {
	if(j==0)
		return 15;
	else if(j==1)
		return 79;
}
double circleSampleX(int i,int j) {
	if(j==0) {
		double arr[]={0.2878510032706436,-0.6687808596081815,-0.9721682260034288,
			-0.13916048677223714,-0.016174198803043716,-0.333850924877801,
			0.7504514538175662,-0.45057467203415236,0.12145620062790785,
			0.9341137165557489,0.8604509966585572,0.5188617963021662,
			0.7888316210785602,0.07531179546974975,-0.24022193701704175};
		return arr[i];
	}
	else if(j==1){
		double arr[]={-0.24354938302743312,0.024062614535854454,-0.5220951189504413,
			0.42348598173012286,0.2049355414533327,0.5082629099970841,
			0.02891816344256748,0.21324925251298232,-0.04672821755517864,
			0.63430832689891,-0.0882707661019082,-0.1849057015120663,
			-0.8553839377794463,0.05075349418142849,-0.03751063254320863,
			0.7764143380053432,0.43855218472316393,-0.28722986367862546,
			0.24013791705083332,0.08150820237775891,-0.5976242882252096,
			0.8242499239863736,-0.40702362887634447,-0.43182247915754735,
			-0.31476581405540394,-0.3021074272772748,0.9134106925052339,
			0.2975242920151788,-0.5426805572453848,0.27837405926108527,
			0.02489077902074488,-0.5358791795570088,-0.7679916611534918,
			0.45641114619124945,-0.6798115103651545,-0.5035913500924472,
			-0.0421822971698953,0.4385259964851893,-0.8562908676645522,
			0.2550936072056671,0.9708589600793918,0.6874075095712846,
			0.8136275368149288,0.18202934286123318,0.510044022415014,
			0.3252669833806725,-0.12431240798030174,0.16641973923823805,
			0.6113714444321863,-0.8302460959991516,-0.22362442699431062,
			0.972715108757285,0.15289248641886344,-0.27957810129344196,
			-0.26525118658009594,-0.31809976526925476,0.7240049130319401,
			0.4987250079410379,-0.2099888405222865,-0.6686207439941354,
			-0.7265795391854968,0.20468395446504983,-0.2924742009712653,
			0.5840507830482666,-0.8052447897532282,0.8756846228137816,
			-0.41871930482745734,0.096811634500928,0.46072110437494684,
			-0.6322518530077281,0.47344248688870305,-0.07580439000156636,
			0.17598719837728938,0.4468686346951518,-0.44074721989166443,
			0.3623454227043381,-0.21069444300233897,-0.5191892355915173,
			-0.297693416911462};
		return arr[i];
	}
	return 0;
}
double circleSampleY(int i,int j){
	if(j==0){
		double arr[]={-0.15139117634794763,0.3352321478450544,-0.003376270949100757,
			0.6304400714102729,0.737613385870242,0.6952951285881652,
			0.6313326526101322,-0.5931852276246175,0.20444842525790508,
			-0.04559903351939276,-0.42683620512735265,-0.7266809799864933,
			-0.21145398719324593,0.42203750467015544,-0.11497976264032772};
		return arr[i];

	}
	else if(j==1){
		double arr[]={-0.4438606582395179,0.7924613274522763,
			-0.7414159015129083,0.7101556001503191,0.4117128269186838,
			0.5896349514770929,-0.9973181475162542,-0.3473048382123052,
			-0.21675408828687592,0.4579973546417895,-0.2250326381407577,
			-0.06190253455510275,0.14020822069455718,0.848966968319643,
			-0.6908737278122161,0.41809007228375616,0.25073053851692206,
			-0.0037059545598729926,-0.9545834412819598,-0.7603729540820403,
			-0.6344759232777837,-0.3579616511589201,0.8381334911275342,
			-0.08474318516366974,0.7668843553368414,-0.5064628095695176,
			0.025775715835821167,0.4634768270822258,-0.054479606543667014,
			0.3687155728947267,0.16788007065805566,0.8139816389183623,
			-0.613375033381709,0.7432481733654499,0.6295333116436121,
			-0.08627979806469499,-0.9787064559387111,-0.3012365729829991,
			0.3777867871048848,0.09040887984529666,-0.18906124091972476,
			0.6735378121664755,0.4254134699367942,-0.30694664774688496,
			0.36377454807751697,-0.4937495863236352,0.20571495032262055,
			-0.14248139037060126,0.3206563906300879,-0.34890802066382554,
			-0.4571594025130734,-0.006775070647144776,0.5774096804171833,
			-0.04270096074962204,0.35113693622060405,0.6677782162466963,
			0.23906920791067643,-0.27345461512915303,0.842519223237395,
			0.5517708819621712,-0.585171782452818,0.08156204306783543,
			0.18358806881280199,-0.4016849634963733,0.2762015204377728,
			0.2697496429560573,0.684179432232344,0.13423791917488126,
			-0.3446201456867706,0.03940122139819424,-0.2942873977922895,
			-0.6836581024074229,-0.5669852384935994,0.6078721713889834,
			0.15435318987287783,-0.907899152596356,-0.7821909471138988,
			0.8481188902669294,-0.7753076730949213};
			return arr[i];
	}
	return 0;
}

RayPropagator camToWorldspace(RayPropagator ray,glm::dvec3 pos,glm::dvec3 right,glm::dvec3 up,glm::dvec3 backwards) {
	return RayPropagator(pos+ray.x.x*right+ray.x.y*up+ray.x.z*backwards,ray.n.x*right+ray.n.y*up+ray.n.z*backwards,ray.r);
}

RayPropagator getCamRayUnfocused(double l,double h,int i,double f,double d,double L,double lenseR,int j){
	dvec3 pos1=dvec3(l*d,h*d,d);
	dvec3 pos2=dvec3(circleSampleX(i,j)*lenseR,circleSampleY(i,j)*lenseR,L);
	return RayPropagator(pos1, normalize(pos2-pos1),0);
}
RayPropagator getCamRayFocused(double l,double h,int i,double f,double d,double L,double lenseR,int j){
	dvec3 pos1(circleSampleX(i,j)*lenseR,circleSampleY(i,j)*lenseR,L);
	double s=1/(1/f-1/(L-d));
	//dvec3 pos2(l*d*(1-s/f),h*d*(1-s/f),-L-s);
	dvec3 pos2(-l*d*s/(L-d),-h*d*s/(L-d),s+L);
	dvec3 dir=normalize(pos2-pos1);
	if(dir.z<0)
		dir=-dir;
	return RayPropagator(pos1,dir,0);
}



