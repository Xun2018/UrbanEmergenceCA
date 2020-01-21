#include "stdafx.h"
#include <simulationprocess.h>
#include <nntrain.h>
#include <TiffDataRead.h>
#include <TiffDataWrite.h>
#include <iostream>  
#include <string>  
#include <list>  
#include <vector>  
#include <map>  
#include <windows.h>

#ifdef _RANK
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <stack>
#endif 

int _tmain(int argc, _TCHAR* argv[])
{
	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");  

	SimulationProcess* sp=new SimulationProcess("bflusSimulationLog.txt","bflusLandDemand.csv");
	sp->runFLUS();

	system("pause");
	return 0;
}

