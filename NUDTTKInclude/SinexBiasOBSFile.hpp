#pragma once
#include "structDef.hpp"
#include "Matrix.hpp"
#include <vector>
#include <limits>
#include <map>

//  Copyright 2019, The National University of Defense Technology at ChangSha

using namespace std;

namespace NUDTTK
{	
	//SINEX_BIAS Version 1.00 Stefan Schaer 2016.12.07
	struct SinexBiasOBSLine
	{
		//  OBS  G063 G01           C1C       2018:021:00000 2018:022:00000 ns                 -1.2508      0.0049
		char   szBiasName[4 + 1]; // A4
		char   szSVNName[4 + 1]; // A4
		char   szPRNName[3 + 1]; // A4
		char   szStationName[9 + 1]; // A4
		char   szObs1Name[4 + 1]; // A4
		char   szObs2Name[4 + 1]; // A4
		char   szUnitName[4 + 1]; // A4
		GPST   t0;
		GPST   t1;
		double obsvalue;
		double obsrms;
	};

	typedef map<string, vector<SinexBiasOBSLine>> SinexBiasOBSMap; // string=szPRNName+szObs1Name, - G01C1C 将2个名字合作为索引关键字, 便于存储数据
	
	class SinexBiasOBSFile
	{
	public:
		SinexBiasOBSFile(void);
	public:
		~SinexBiasOBSFile(void);
	public:
		bool isEmpty();
		bool isValidEpochLine(string strLine,SinexBiasOBSLine& dcbline); 
		bool open(string strSinexBiasOBSFileName);
		bool getOBSCorrectValue_Day(GPST t, string PRNName, string Obs1Name, double &obsvalue);
	    DayTime doy2daytime(int year, int doy, double second);	
	public:
		SinexBiasOBSMap  m_data; 
	};
}
