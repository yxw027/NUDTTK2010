#pragma once
#include "structDef.hpp"
#include <string>
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct Eopc04Line
	{
		int     month;    // 月
		int     day;      // 日
		int     mjd;      // 修正的儒略日
		double  x;        // 极移
		double  y;        // 极移
		double  ut1_utc;
		double  lod;
		double  psi;
		double  eps; 

		Eopc04Line() 
		{
			month = MONTH_UNKNOWN;
			day   = -1;
		}
	};

	// 一年的地球旋转记录
	struct Eopc04YearRecord
	{
		int                 year;
		int                 interval;        // 单位：天
		int                 mjd_first;       // 初始儒略日
		string              strText[17];
		vector<Eopc04Line>  eopc04Linelist;      
	};

	class Eopc04File
	{
	public:
		Eopc04File(void);
	public:
		~Eopc04File(void);
	public:
		void clear();
		bool isValidEpochLine(string strLine,Eopc04Line& eopc04line);
		bool open(string strEopc04fileName);
		bool getPoleOffset(UTC t, double& x, double& y);
		bool getNutationCorrect(UTC t, double& psi, double& eps);
		bool getUT1_UTC(UTC t, double& ut1_utc, double& ut1_utc_rate);
		bool getUT1_UTC(UTC t, double& ut1_utc);
	public:
		vector<Eopc04YearRecord> m_data;
	};
}

