#pragma once
#include "structDef.hpp"
#include <string>
#include <vector>

namespace NUDTTK
{
	struct EopRapidFileIAU2000Line
	{
		int     year;    // 年
		int     month;   // 月
		int     day;     // 日
		double  mjd;     // 修正的儒略日
		double  pm_x;    // 单位: 角秒     
		double  pm_y;    // 单位: 角秒    
		double  ut1_utc; // 单位: 秒

		EopRapidFileIAU2000Line() 
		{
			month =  MONTH_UNKNOWN;
			day   = -1;
		}
	};

	class eopRapidFileIAU2000
	{
	public:
		eopRapidFileIAU2000(void);
	public:
		~eopRapidFileIAU2000(void);
	public:
		void clear();
		bool isValidEpochLine(string strLine,EopRapidFileIAU2000Line& eopLine);
		bool open(string strEopRapidFileName);
		bool getPoleOffset(UTC t, double& x, double& y);
		bool getUT1_UTC(UTC t, double& ut1_utc, double& ut1_utc_rate);
		bool getUT1_UTC(UTC t, double& ut1_utc);
	public:
		vector<EopRapidFileIAU2000Line> m_data;
	};
}
