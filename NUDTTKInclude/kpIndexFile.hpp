#pragma once
#include "structDef.hpp"
#include <string>
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	namespace Geodyn
	{
		struct KpIndexLine
		{
			int     year;
			int     month;
			int     day;
			int     hour;
			double  kpIndex; 

			KpIndexLine() 
			{
				year  = -1;
				month = -1;
				day   = -1;
				hour  = -1;
			}
			DayTime getTime()
			{
				DayTime t(year, month, day, hour, 0, 0);
				return t;
			}
		};

		class kpIndexFile
		{
		public:
			kpIndexFile(void);
		public:
			~kpIndexFile(void);
		public:
			bool isValidEpochLine(string strLine, KpIndexLine& kpLine);
			bool open(string  strKpIndexFileName);
			bool getKpIndex(DayTime t,double& kpIndex);
			bool getKpIndex_nrlmsise00(DayTime t, double& kpm_day, double& kp_0h, double& kp_3h, double& kp_6h, double& kp_9h, double& kpm_12_33h, double& kpm_36_57h);
		public:
			vector<KpIndexLine> m_data;
		};
	}
}
