#pragma once
#include "structDef.hpp"
#include <string>
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct Eopc04TotalYearLine
	{
		int     year;     // 年
		int     month;    // 月
		int     day;      // 日
		int     mjd;      // 修正的儒略日
		double  x;        // 极移
		double  y;        // 极移
		double  ut1_utc;
		double  lod;
		double  psi;
		double  eps; 

		Eopc04TotalYearLine() 
		{
			month = MONTH_UNKNOWN;
			day   = -1;
		}
	};
	class Eopc04TotalFile
	{
	public:
		Eopc04TotalFile(void);
	public:
		~Eopc04TotalFile(void);
	public:
		void clear();
		bool isValidEpochLine(string strLine, Eopc04TotalYearLine& eopc04line);
		bool open(string strEopc04TotalfileName);
		bool getPoleOffset(UTC t, double& x, double& y);
		bool getNutationCorrect(UTC t, double& psi, double& eps);
		bool getUT1_UTC(UTC t, double& ut1_utc, double& ut1_utc_rate);
		bool getUT1_UTC(UTC t, double& ut1_utc);
	public:
		vector<Eopc04TotalYearLine> m_data;
	};
}

