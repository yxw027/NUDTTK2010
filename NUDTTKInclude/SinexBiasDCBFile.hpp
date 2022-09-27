#pragma once
#include "structDef.hpp"
#include "Matrix.hpp"
#include <vector>
#include <limits>
#include <map>

//  Copyright 2018, The National University of Defense Technology at ChangSha

using namespace std;

namespace NUDTTK
{	
	struct SinexBiasDCBLine
	{
		//  DSB  G063 G01           C1C  C1W  2018:001:00000 2018:002:00000 ns                 -1.1066      0.0191
		char   szBiasName[4 + 1]; // A4
		char   szSVNName[4 + 1]; // A4
		char   szPRNName[3 + 1]; // A4
		char   szStationName[9 + 1]; // A4
		char   szObs1Name[4 + 1]; // A4
		char   szObs2Name[4 + 1]; // A4
		char   szUnitName[4 + 1]; // A4
		GPST   t0;
		GPST   t1;
		double dcbvalue;
		double dcbrms;
	};

	typedef map<string, vector<SinexBiasDCBLine>> SinexBiasDCBMap; // string=szPRNName+szObs1Name+szObs2Name, - G01C1CC1W 将3个名字合作为索引关键字, 便于存储数据
    //typedef map<string, SinexBiasDCBLine> SinexBiasDCBMap; // string=szPRNName+szObs1Name+szObs2Name, - G01C1CC1W 将3个名字合作为索引关键字, 便于存储数据

	class SinexBiasDCBFile
	{
	public:
		SinexBiasDCBFile(void);
	public:
		~SinexBiasDCBFile(void);
	public:
		bool isEmpty();
		bool isValidEpochLine(string strLine,SinexBiasDCBLine& dcbline); 
		bool open(string strSinexBiasDCBFileName);
		bool getDCBCorrectValue_Month(GPST t, string PRNName, string Obs1Name, string Obs2Name, double &dcbvalue);
		bool getDCBCorrectValue_Day(GPST t, string PRNName, string Obs1Name, string Obs2Name, double &dcbvalue);
	    DayTime doy2daytime(int year, int doy, double second);	
	public:
		SinexBiasDCBMap  m_data; 
	};
}
