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
    struct Rinex3_0_NavHeader_iGMAS 
	{
		char        szRinexVersion[20 + 1];      // 3.03d 3.03e     | F9.3, A1, 10X
		char        szFileType[20 + 1];         // ('N' for Navigation data) | A1,19X
		char        szSatSystem[20 + 1];        // C:BeiDou
		char        szProName[20 + 1];          // A20
		char        szAgencyName[20 + 1];       // A20
		char        szFileDate[20 + 1];         // 文件创建日期
		char        szTimeToUTC[4 + 1];         // 系统时间与UTC或其它系统之间的差值
		double		pdIonAlpha[4];
		double		pdIonBeta[4];
		DELTA_UTC   deltaUTC;
		long		lnLeapSeconds;

		Rinex3_0_NavHeader_iGMAS()
		{
			memset(this, 0, sizeof(Rinex3_0_NavHeader_iGMAS));
			pdIonAlpha[0] = DBL_MAX;
			pdIonBeta[0]  = DBL_MAX;
			deltaUTC.dA0  = DBL_MAX;
			lnLeapSeconds = LONG_MAX;
		}
	};


	class Rinex3_0_NavFile_iGMAS
	{
	public:
		Rinex3_0_NavFile_iGMAS(void);
	public:
		~Rinex3_0_NavFile_iGMAS(void);
	public:
		void   clear();
		bool   isEmpty();
		int    isValidEpochLine(string strLine, FILE * pNavfile = NULL);
		bool   open(string  strNavfileName);
		bool   getEphemeris(DayTime T, int nPRN, Rinex2_1_NavDatum& navDatum, double threshold_span_max = 86400.0);
		bool   getEphemeris(DayTime T, int nPRN, POSCLK &posclk);
		double getClock(DayTime T, Rinex2_1_NavDatum navDatum);
		POSCLK getPosition(DayTime T, Rinex2_1_NavDatum navDatum);
		bool   exportSP3File(string strSP3fileName, DayTime T_Begin, DayTime T_End, double spanSeconds = 300);
        bool  exportCLKFile(string strCLKfileName, DayTime T_Begin, DayTime T_End, double spanSeconds = 120);
		bool   exportSP3File_GPST(string strSP3fileName, GPST T_Begin, GPST T_End, double spanSeconds = 300);
		bool   exportCLKFile_GPST(string strCLKfileName, GPST T_Begin, GPST T_End, double spanSeconds = 120);
	public:
		Rinex3_0_NavHeader_iGMAS  m_header;
		NavSatMap           m_data;
		double              m_EARTH_W;
		double              m_GM_EARTH;
	};
}
