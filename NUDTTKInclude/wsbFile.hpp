#pragma once
#include <string>
#include <vector>
#include <string.h>
#include "structDef.hpp"

namespace NUDTTK
{
	struct wsbFileLine
	{
		int year;
		int month;
		int day;
		double wsb[40];
		
		wsbFileLine()
		{
			year  = 0;
			day   = 0;
			month = 0;
		}
	};
		
	class wsbFile
	{
		public:
			wsbFile(void);
		public:
			~wsbFile(void);
		public:
			bool isValidEpochLine(string strLine, wsbFileLine &wsbline);
			bool open(string strWSBFileName);
			bool getWSB(DayTime t, wsbFileLine &wsbline);
		public:
			vector<wsbFileLine> m_data;
	};

}