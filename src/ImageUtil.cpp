#include "ImageUtil.h"
#include "RayUtil.h"

using std::vector;
using std::ofstream;
using std::string;



Image::Image(int w,int h): width(w),height(h),data(vector<char>(w*h*4,0)){
}
Image::Image(vector<double> arr,int w,int h) {
	createFromArray(arr,w,h);
}
void Image::createFromArray(std::vector<double> arr,int width,int height) {
	data=vector<char>(width*height*4,0);
	this->width=width;
	this->height=height;
	for(int x=0;x<width;x++) {
		for(int y=0;y<height;y++) {
			double a=arr[y*width+x];
			put(x,y,floatToRGB(a,a,a));
		}
	}
}
void Image::put(int x,int y,int c){
	if(x>=0&&x<width && y>=0&&y<height)
		*((int *)&data[(x+y*width)*4])=c;

}
int Image::get(int x,int y){
	if(x>=0&&x<width && y>=0&&y<height)
		return *((int *)&data[(x+y*width)*4]);
	else
		return 0;
}
void Image::save(string filename){
	int k=width*height;
	int s=4*k;
	int filesize=54+s;

	//disgusting code assuming little endian
	vector<char> data2(filesize);
	data2[0]='B';
	data2[1]='M';
	*((int *)&data2[2])=filesize;
	*((int *)&data2[10])=54;

	*((int *)&data2[14])=40; //header size
	*((int *)&data2[18])=width; //bitmap width in pixels
	*((int *)&data2[22])=height; //bitmap height in pixels
	*((int *)&data2[26])=1; //"Must be set to 1."
	*((int *)&data2[28])=32; //bits per pixel
	*((int *)&data2[30])=0; //compression method (0=none)
	*((int *)&data2[34])=s; //size of raw bitmap data in bytes
	*((int *)&data2[38])=100; //horizontal rez in pixels/meter
	*((int *)&data2[42])=100; //vertical rez in pixels/meter
	*((int *)&data2[46])=0; //"the number of colors in the color palette, or 0 to default to 2n."
	*((int *)&data2[50])=0; //"the number of important colors used, or 0 when every color is important; generally ignored."
	for(int x=0;x<data.size();x++){
		data2[x+54]=data[x];
	}

	writeCharFile(filename,data2);
}


DoubleImage::DoubleImage() : width(0),height(0),data(){

}
DoubleImage::DoubleImage(int w,int h) : width(w),height(h),data(w*h){

}
DoubleImage::DoubleImage(vector<double> arr,int w,int h) {
	createFromArray(arr,w,h);
}
void DoubleImage::createFromArray(std::vector<double> arr,int width,int height) {
	 if(arr.size()<width*height) {
		std::cout<<"DoubleImage given invalid array size"<<std::endl;
		return;
	}
	data=arr;
	this->width=width;
	this->height=height;
}
const std::vector<double> DoubleImage::getData() {
	return data;
}
void DoubleImage::put(int x,int y,double c) {
	data[y*width+x]=c;
}
double DoubleImage::get(int x,int y) {
	return data[y*width+x];
}
int DoubleImage::getWidth() {
	return width;
}
int DoubleImage::getHeight() {
	return height;
}