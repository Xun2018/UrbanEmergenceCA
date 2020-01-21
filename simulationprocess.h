#ifndef SIMULATIONPROCESS_H
#define SIMULATIONPROCESS_H

#include <vector>
#include <iostream>

class TiffDataRead;

using namespace std;

class SimulationProcess 
{

public:
	SimulationProcess(string _configfile);
	SimulationProcess(string _configfile,string _demandfile);
	void runFLUS();
	~SimulationProcess();


private:
	void init();
	bool readImageData();
	bool imageOpen(string filename);
	bool imageOpenConver2uchar(string filename);
	void startloop();
	void runloop2();
	bool getparameters();
	bool getcellstatistic();
	void saveResult(string filename);


private:
	size_t _rows;
	size_t _cols;
	int nType;
	int numWindows;
	int sizeWindows;
	int looptime;
	double degree;
	bool isRestrictExit;
	bool isSave;
	int isbreak;
	int* goalNum;
	int* mIminDis2goal;
	string savepath;
	bool isMultGoal;

	string configfile;
	string demandfile;

protected:
	vector<TiffDataRead*> imgList;

	vector<int> typeIndex;

	vector<int> multipleYear;

	vector<int> multiDemand;

	int* saveCount;
	int* val;
	double* mdNeiborhoodProbability;
	double* mdRoulette;
	double* probability;
	double* sProbability;
	double* normalProbability;
	double* mdNeighIntensity;
	unsigned char* temp;
	double** t_filecost;
	int** direction;
	short** Colour;

};

#endif // SIMULATIONPROCESS_H
