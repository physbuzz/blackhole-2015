#pragma once
#include <vector>
#include <string>

class Image{
	int width;
	int height;

	std::vector<char> data;
public:
	Image(int w,int h);
	Image(std::vector<double> arr,int w,int h);
	void createFromArray(std::vector<double> arr,int width,int height);
	void put(int x,int y,int c);
	int get(int x,int y);
	void save(std::string filename);
};
class DoubleImage{
	int width;
	int height;

	std::vector<double> data;
public:
	int getWidth();
	int getHeight();
	DoubleImage();
	DoubleImage(int w,int h);
	DoubleImage(std::vector<double> arr,int w,int h);
	void createFromArray(std::vector<double> arr,int width,int height);
	const std::vector<double> getData();
	void put(int x,int y,double c);
	double get(int x,int y);
};