#pragma once
#include <string.h>
#include <vector>
#include "structDef.hpp"

using namespace std;
//读取JB2008大气模型输入参数：DTC

namespace NUDTTK
{
	namespace Geodyn
	{
		struct DTCFileLine
		{
			int year;
			int day;
			int DTC[24];
			DayTime t;
			DayTime doy2daytime(int year, int doy, int hour, int minute);
			
			DTCFileLine()
			{
				year = 0;
				day  = 0;
			}
		};
			
		class DTCFile
		{
		public:
			DTCFile(void);
		public:
			~DTCFile(void);
		public:
			bool isValidEpochLine(string strLine, DTCFileLine &dtcline);
			bool open(string strDTCFileName);
			bool getDTC(DayTime t,double &dtc);
			bool write(string strDTCFileName);
		public:
			vector<DTCFileLine> m_data;
		};
	}
}