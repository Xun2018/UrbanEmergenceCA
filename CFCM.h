#pragma once
#include <iostream>
#include <cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>  
#include <string>  

using namespace std;
using namespace cv;


class CFCM
{
public:
	CFCM(void);
	~CFCM(void);

	void loadImage(string addr, int flags = 1);  
    void loadImage(Mat& img);  
  
    void init(int Nc);  
    int cluster(int Nc, double m, double eps);  
    void showOrgImage(string win_name, Mat img);  
    void showClusterResult(string win_name, Mat img);  
  
private:  
    void initImgVec();  
    void initCentroid(int Nc);  
    void updateFuzzyMat();  
    void updateCentroid();  
      
public:  
    Mat m_uImg;  
    Mat m_dImg;  
    Mat m_imgVec;  
    Mat m_fuzzyMat; //Np * Nc  
    Mat m_centrMat; //Nc * channels  
    int m_Np;  
    int m_channels;  
    int m_Nc;  
    double m_fuzzyVal;  
    bool m_init;  

};


