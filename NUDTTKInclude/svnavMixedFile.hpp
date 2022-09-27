#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>
#include "TimeCoordConvert.hpp"
//  Copyright 2017, The National University of Defense Technology at ChangSha
using namespace std;

namespace NUDTTK
{
	struct SvNavMixedLine
	{
		char    cSys;
		int     id_PRN;
		int     id_SVN;
		int     id_CHAN;
		char    szBlock[20 + 1];
		int     mass;
		char    yawBiasFlag;
		float   yawRate;
		int     y0;
		int     d0;
		int     h0;
		int     m0;
		int     y1;
		int     d1;
		int     h1;
		int     m1;
        DayTime t0;
        DayTime t1;
        DayTime doy2daytime(int year, int doy, int hour, int minute);

		SvNavMixedLine()
		{
			id_PRN  = 0;
		    id_SVN  = 0;
			id_CHAN = 0;
			y0 = 0;
			d0 = 0;
			h0 = 0;
			m0 = 0;
			y1 = 0;
			d1 = 0;
			h1 = 0;
			m1 = 0;
		}
	};

	class svnavMixedFile
	{
	public:
		svnavMixedFile(void);
	public:
		~svnavMixedFile(void);
	public:
		void clear();
		bool isValidEpochLine(string strLine, SvNavMixedLine& mixedLine);
		bool open(string strSvNavMixedFileName);
		bool getSvNavInfo(DayTime t, string satName, SvNavMixedLine& svNavMixedLine);
		bool write(string strSvNavMixedFileName);
	public:
		vector<SvNavMixedLine> m_data;
	};
}
