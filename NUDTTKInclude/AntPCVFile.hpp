#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
namespace NUDTTK
{
	class AntPCVFile
	{
	public:
		AntPCVFile(void);
	public:
		~AntPCVFile(void);
	public:
		void   clear();
		bool   isValidEpochLine(string strLine, vector<double> &dataList);
		bool   open(string  strPCVFileName);
		bool   write(string  strPCVFileName,bool bInt = false);
		double getPCVValue(double Elevation, double Azimuth, int nLagrange = 2);
	public:
		vector<double> m_listElevation;
		vector<double> m_listAzimuth;
		double**       m_data; 
	};
}
