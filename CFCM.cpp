#include "StdAfx.h"
#include "CFCM.h"
#include <time.h>

CFCM::CFCM()
{
	m_init = false;
}

CFCM::~CFCM()
{
	m_dImg.release();
}


Mat sum_cols(Mat m)
{
	int cols = m.cols;
	Mat new_m (1, cols, m.type());
	for (int i = 0; i<cols; i++)
	{
		Mat m_col = m.col(i);
		Scalar s = cv::sum(m_col);
		new_m.col(i) = s;
	}
	return new_m;
}

Mat sum_rows(Mat m)
{
	int rows = m.rows;
	Mat new_m (rows, 1, m.type());
	for (int i=0; i<rows; i++)
	{
		Mat m_row = m.row(i);
		Scalar s = cv::sum(m_row);
		new_m.row(i) = s;
	}
	return new_m;
}

double max_diff_ratio(Mat m_old, Mat m_new)
{
	Mat diff = cv::abs(m_new - m_old);
	Mat divs = cv::abs(m_old) + Scalar::all(0.0001);
	Mat r = diff.mul(1.0/divs);
	double max_v, min_v;
	minMaxIdx(r, &min_v, &max_v);
	return max_v;
}

void CFCM::loadImage(string addr, int flags)
{
	m_uImg = imread(addr, flags);

	m_channels = m_uImg.channels();
	m_dImg = Mat_<Vec3d>(m_uImg);
}

void CFCM::loadImage(Mat& img)
{
	m_dImg = Mat_<Vec3d>(img);
}

void CFCM::init(int Nc)
{
	cout<<"init...";
	m_init = true;
	m_Np = m_dImg.rows * m_dImg.cols;
	m_channels = m_dImg.channels();
	initImgVec();
	initCentroid(Nc);
	cout<<"done!"<<endl;
}

void CFCM::initImgVec()
{
	m_imgVec = m_dImg.reshape(1, m_Np);
}

void CFCM::initCentroid(int Nc)
{
	m_Nc = Nc;
	m_centrMat = Mat(m_Nc, m_channels, CV_64FC1);

	Mat m_r = m_centrMat.reshape(m_channels, m_Nc);

	int rows = m_uImg.rows;
	int cols = m_uImg.cols;

	RNG rng(m_Nc);
	rng.fill(m_centrMat, RNG::UNIFORM, Scalar(0.0), Scalar(256.0));
}

int CFCM::cluster(int _Nc, double _fuzzyVal, double eps)
{
	m_fuzzyVal = _fuzzyVal;

	if(!m_init)
		init(_Nc);

	cout<<"starting cluster...";
	Mat old_fuzzy;
	m_fuzzyMat.copyTo(old_fuzzy);
	double max_v; 

	clock_t start = clock();
	int count = 0;
	do 
	{
		updateFuzzyMat();
		updateCentroid();
		max_v = max_diff_ratio(old_fuzzy, m_fuzzyMat);
		m_fuzzyMat.copyTo(old_fuzzy);
		count++;
	} while (max_v>eps);
	clock_t finish = clock();

	cout<<"done!"<<endl;

	cout<<"duration: "<<(double)finish-start/CLOCKS_PER_SEC<<endl;
	cout<<"iteration: "<<count<<endl;
	return count;
}

void CFCM::updateFuzzyMat()
{
	Mat p_vec = m_imgVec.reshape(m_channels, m_Np);
	Mat p_mat = repeat(p_vec, 1, m_Nc);
	Mat c_vec = m_centrMat.reshape(m_channels, 1);
	Mat c_mat = repeat(c_vec, m_Np, 1);
	Mat p_sub_c = p_mat - c_mat;

	vector<Mat> planes;
	Mat plane_sqsum(m_Np, m_Nc, CV_64FC1, Scalar::all(0));
	Mat pc_norm, m_pow;
	split(p_sub_c, planes);
	for (int i=0; i<planes.size(); i++)
	{
		cv::pow(planes.at(i), 2.0, m_pow);
		plane_sqsum += m_pow;
		cv::sqrt(plane_sqsum, pc_norm);
	}

	Mat pc_mat;
	cv::pow(pc_norm, 2.0/(m_fuzzyVal-1), pc_mat);

	Mat r_vec = sum_rows(1.0 / pc_mat);

	Mat r_mat = repeat(r_vec, 1, m_Nc);
	m_fuzzyMat = 1.0/(pc_mat.mul(r_mat));
}

void CFCM::updateCentroid()
{
	Mat m_pow;
	cv::pow(m_fuzzyMat, m_fuzzyVal, m_pow);
	Mat c_vec = m_pow.t() * m_imgVec; 
	Mat c_sum = sum_cols(m_pow).t(); 
	Mat c_s = repeat(c_sum, 1, m_channels);
	m_centrMat = c_vec.mul(1.0/c_s);
	
	// set one clustering cetnter as 0 vector
	for (int jj = 0; jj < m_centrMat.cols; jj++)
	{
		m_centrMat.at<uchar>(1, jj) = 0;
	}
}

void CFCM::showOrgImage(string win_name, Mat img)
{
	m_uImg.copyTo(img);
	imshow(win_name, img);
}

void CFCM::showClusterResult(string win_name, Mat img)
{
	img = Mat(m_uImg.size(), m_uImg.type());

	Mat img_r = img.reshape(m_channels, m_Np);
	
	double max_v, min_v;
	int max_p[2], min_p[2];
	Mat centroid = m_centrMat.reshape(m_channels, m_Nc);
	for (int i=0; i<m_Np; i++)
	{
		Mat row_m = m_fuzzyMat.row(i);
		minMaxIdx(row_m, &min_v, &max_v, min_p, max_p);
		int c_p = max_p[1];
		Vec3b v = (Vec3b)centroid.at<Vec3d>(c_p,0);

		img_r.at<Vec3b>(i,0) = v;
	}

	imshow(win_name, img);
	string win_name1=win_name+".jpg";
	imwrite(win_name1, img);
}


