#include "fuzzy_clustering.hpp"

#include <cxcore.h>
#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <time.h>
#include <sstream>

//#include <glog/logging.h>

#include <opencv2/core/core.hpp>
#define NOMINMAX
#include <windows.h>
#include "CFCM.h" 

using namespace std;
using namespace cv;


void test()
{
	//google::InitGoogleLogging (argv[0]);
	//google::InstallFailureSignalHandler ();

	//  static unsigned int number_points = 17;
	//  static unsigned int dimension_point = 4;
	static unsigned int number_clusters = 2;
	//
	//  srand((unsigned)time(0));
	//
	//  cv::Mat dataset (number_points, dimension_point, CV_32FC1);
	//  for (int j = 0; j < dataset.rows; ++j) {
	//    for (int  i = 0; i < dataset.cols; ++i) {
	//      dataset.at<float> (j, i) = (float) rand() / (float) RAND_MAX;
	//    }
	//  }

	cv::Mat dataset = (cv::Mat_<float> (4, 2)
		<< 
		0, 0,
		5, 4,
		//    3.3, 2,
		//    15, 10,
		//    7.5, 6,
		//    30, 40.3,
		//    50, 60,
		//    70, 80,
		//    90, 100,
		//    100, 200.252,
		//    150, 151,
		//    200, 205,
		100, 150,
		200, 102);

	float fuzziness = 2.0;    // initial: 1.1
	float epsilon = 0.01;
	SoftCDistType dist_type = kSoftCDistL2;
	SoftCInitType init_type = kSoftCInitKmeansPP;

	SoftC::Fuzzy f (dataset, number_clusters, fuzziness, epsilon, dist_type, init_type);

	// Note: This does not mean iteration initization.
	unsigned int num_iterations = 100;
	f.clustering (num_iterations);

	std::cout << "### Results ### " << std::endl;

	cv::Mat centroids = f.get_centroids_ ();
	std::cout << centroids << std::endl;

	cv::Mat memberships = f.get_membership_ ();
	std::cout << memberships << std::endl;

}

void test2()
{
	static unsigned int number_clusters = 2;

	srand((unsigned)time(0));

	static unsigned int number_points = 1844*2132;
	static unsigned int dimension_point = 13;
	cv::Mat dataset (number_points, dimension_point, CV_32FC1);
	for (int j = 0; j < dataset.rows; ++j) {
		for (int  i = 0; i < dataset.cols; ++i) {
			dataset.at<float> (j, i) = (float) rand() / (float) RAND_MAX;
		}
	}


	//float fuzziness = 2.0;    // initial: 1.1
	float fuzziness = 2;    // initial: 1.1
	double epsilon = 0.00001;
	SoftCDistType dist_type = kSoftCDistL2;
	SoftCInitType init_type = kSoftCInitKmeansPP;

	double timestart, timeend,timeused;
	timestart=GetTickCount();

	SoftC::Fuzzy f (dataset, number_clusters, fuzziness, epsilon, dist_type, init_type);
	// Note: This does not mean iteration initization.
	unsigned int num_iterations = 100000;
	f.clustering (num_iterations);

	timeend=GetTickCount();

	cout<<(timeend-timestart)/1000<<"s"<<endl;

	cv::Mat memberships = f.get_membership_ ();
	cout<<memberships.rows<<endl;
	cout<<memberships.cols<<endl;

	system("pause");

	//return 0;
}

void fuzzyKmeansUnal(cv::Mat data,cv::Mat u,cv::Mat c,int k,float epsilon,int Iter,int m,float &J){
	Mat d,totald;
	bool stop=false;
	int cont=0;

	d=Mat::zeros(data.rows,k,CV_32F);
	srand(time(0));

	float e,f;
	for(int i=0;i<u.rows;i++){
		for(int j=0;j<u.cols;j++){
			e=rand();
			f=e/RAND_MAX;
			u.at<float>(i,j)=f;
		}
	}

	float sumant=0;
	while(cont<Iter && stop==false){



		for (int j=0;j<data.cols;j++){
			for (int i=0;i<k;i++){
				float sumX=0;
				float sumu=0;
				for (int l=0;l<data.rows;l++){

					sumX=sumX+(pow(u.at<float>(l,i),m)*(data.at<float>(l,j)));
					sumu=sumu+pow(u.at<float>(l,i),m);


				}
				c.at<float>(i,j)=sumX/sumu;

			}
		}
		for (int i=0;i<data.rows;i++){

			for (int j=0;j<k;j++){
				d.at<float>(i,j)=sqrt(pow((data.at<float>(i,0)-c.at<float>(j,0)),2)+pow((data.at<float>(i,1)-c.at<float>(j,1)),2));

			}

		}  


		totald=Mat::zeros(data.rows,1,CV_32F);
		for (int i=0;i<data.rows;i++){
			for (int j=0;j<k;j++){

				totald.at<float>(i,0)=totald.at<float>(i,0)+(pow(1/d.at<float>(i,j),(2/(m-1))));

			}
		}

		for (int i=0;i<data.rows;i++){
			for( int j=0;j<k;j++){


				u.at<float>(i,j)=pow(1/d.at<float>(i,j),(2/(m-1)))/totald.at<float>(i,0);
				// u.at<float>(i,j)=1;



			}
		}

		float sumj=0;
		for (int i=0;i<data.rows;i++){
			float sumb=0;
			for (int j=0;j<k;j++){
				sumb=sumb+pow(u.at<float>(i,j),m)*(pow((data.at<float>(i,0)-c.at<float>(j,0)),2)+pow((data.at<float>(i,1)-c.at<float>(j,1)),2));
			}

			sumj=sumj+sumb;
		}

		if(abs(sumant-sumj)<epsilon){
			stop=true;
			J=sumj;
		}

		cout<<abs(sumant-sumj)<<endl;

		sumant=sumj;
		J=sumj;
		cont++;
	}


	//cout<<endl;
	cout<<"El algoritmo se detuvo en "<<cont-1<<" iteraciones"<<endl;
	//cout<<"fcn: "<<J<<endl;
}

int test3()
{



	int Iter=100000000;

	int k=2;
	int m=2;
	float epsilon=0.00001;
	float J;

	Mat u,c;
	//data=Mat::zeros(7,2,CV_32F);
	//data.at<float>(0,0)=1;
	//data.at<float>(1,0)=1.5;
	//data.at<float>(2,0)=3;
	//data.at<float>(3,0)=5;
	//data.at<float>(4,0)=3.5;
	//data.at<float>(5,0)=4.5;
	//data.at<float>(6,0)=3.5;
	//data.at<float>(0,1)=1;
	//data.at<float>(1,1)=2;
	//data.at<float>(2,1)=4;
	//data.at<float>(3,1)=7;
	//data.at<float>(4,1)=5;
	//data.at<float>(5,1)=5;
	//data.at<float>(6,1)=4.5;

	static unsigned int number_points = 16000000;
	static unsigned int dimension_point = 17;
	cv::Mat data (number_points, dimension_point, CV_32FC1);
	for (int j = 0; j < data.rows; ++j) {
		for (int  i = 0; i < data.cols; ++i) {
			data.at<float> (j, i) = (float) rand() / (float) RAND_MAX;
		}
	}

	u=Mat::zeros(data.rows,k,CV_32F);
	c=Mat::zeros(k,2,CV_32F);
	fuzzyKmeansUnal( data,u,c, k,epsilon, Iter, m, J);
	cout<<"centroides"<<endl;
	cout<<c<<endl;
	//cout<<d<<endl;
	//cout<<totald<<endl;
	cout<<endl;
	cout<<"Matriz de pertenencia"<<endl;
	cout<<u<<endl;
	cout<<endl;
	cout<<"fcn: "<<J<<endl;

	system("pause");
	return 0;
}

void test4()
{
	Mat img;  
    CFCM fcm;  
    fcm.loadImage("timg.tiff");  
    fcm.showOrgImage("org", img);  
    waitKey(0);  
  
    fcm.cluster(2, 2, 0.001);  
    fcm.showClusterResult("result", img);  
    waitKey(0);  



	system("pause");
}


//--------------------------------------------------------------------------->






int main( int argc, char** argv )
{
	test4();

}