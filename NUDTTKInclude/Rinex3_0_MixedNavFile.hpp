#pragma once
#include "structDef.hpp"
#include "constDef.hpp"
#include <windows.h>
#include <limits>
#include <vector>
#include <map>
#include "TimeCoordConvert.hpp"
#include "Rinex2_1_NavFile.hpp"

namespace NUDTTK
{
	struct TimeToUTC
	{
		char        CorrType[4 + 1];        // 不同系统Correction type	
		DELTA_UTC   deltaUTC;  
		TimeToUTC()
		{
			memset(this, 0, sizeof(TimeToUTC));
			deltaUTC.dA0  = DBL_MAX;
		}
	};
	typedef map<string, NavDatumList>    NavSysPRNMap;
    struct Rinex3_0_MixedNavHeader 
	{
		char        szRinexVersion[20 + 1];      // 3.03d 3.03e     | F9.3, A1, 10X
		char        szFileType[20 + 1];         // ('N' for Navigation data) | A1,19X
		char        szSatSystem[20 + 1];        // M:混合星历
		char        szProName[20 + 1];          // A20
		char        szAgencyName[20 + 1];       // A20
		char        szFileDate[20 + 1];         // 文件创建日期
		vector<TimeToUTC>        TimeSysCorr;         // 系统时间与UTC或其它系统之间的差值
		double		pdIonAlpha[4];
		double		pdIonBeta[4];
		long		lnLeapSeconds;

		Rinex3_0_MixedNavHeader()
		{
			memset(this, 0, sizeof(Rinex3_0_MixedNavHeader));
			pdIonAlpha[0] = DBL_MAX;
			pdIonBeta[0]  = DBL_MAX;
			lnLeapSeconds = LONG_MAX;
		}
	};


	class Rinex3_0_MixedNavFile
	{
	public:
		Rinex3_0_MixedNavFile(void);
	public:
		~Rinex3_0_MixedNavFile(void);
	public:
		void   clear();
		bool   isEmpty();
		int    isValidEpochLine(char& cSatSys, string strLine, FILE * pNavfile = NULL);
		bool   open(string  strNavfileName);
		bool   getEphemeris(DayTime T, int nPRN, POSCLK &posclk, double threshold_span_max = 86400.0);
		double getClock(DayTime T, Rinex2_1_NavDatum navDatum);
		POSCLK getPosition(DayTime T, Rinex2_1_NavDatum navDatum);
		bool   exportSP3File(string strSP3fileName, DayTime T_Begin, DayTime T_End, double spanSeconds = 300);
        bool   exportCLKFile(string strCLKfileName, DayTime T_Begin, DayTime T_End, double spanSeconds = 120);
	public:
		Rinex3_0_MixedNavHeader             m_header;
		NavSysPRNMap           m_data;
		double              m_EARTH_W_BD;
		double              m_GM_EARTH_BD;
		double              m_EARTH_W_GPS;
		double              m_GM_EARTH_GPS;
	};
}
