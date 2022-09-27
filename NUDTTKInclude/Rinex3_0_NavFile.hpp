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
    struct Rinex3_0_NavHeader 
	{
		char        szRinexVersion[20 + 1];
		char        szFileType[20 + 1];         // ('N' for Navigation data) | A1,19X
		char        szSatSystem[20 + 1];        // C:BeiDou
		char        szProName[20 + 1];
		char        szAgencyName[20 + 1];       
		char        szFileDate[20 + 1];         // 文件创建日期
		char        szTimeToUTC[4 + 1];         // 系统时间与UTC或其它系统之间的差值
		double		pdIonAlpha[4];
		double		pdIonBeta[4];
		DELTA_UTC   deltaUTC;
		long		lnLeapSeconds;

		Rinex3_0_NavHeader()
		{
			memset(this, 0, sizeof(Rinex3_0_NavHeader));
			pdIonAlpha[0] = DBL_MAX;
			pdIonBeta[0]  = DBL_MAX;
			deltaUTC.dA0  = DBL_MAX;
			lnLeapSeconds = LONG_MAX;
		}
	};


	class Rinex3_0_NavFile
	{
	public:
		Rinex3_0_NavFile(void);
	public:
		~Rinex3_0_NavFile(void);
	public:
		void   clear();
		bool   isEmpty();
		int    isValidEpochLine(string strLine, FILE * pNavfile = NULL);
		bool   open(string  strNavfileName);
    	bool   write(string strNavfileName);
		void   writedouble(FILE* pNavfile, double value);
		bool   getEphemeris(DayTime T, int nPRN, Rinex2_1_NavDatum& navDatum, double threshold_span_max = 86400.0);
		bool   getEphemeris(DayTime T, int nPRN, POSCLK &posclk);
		double getClock(DayTime T, Rinex2_1_NavDatum navDatum);
		POSCLK getPosition(DayTime T, Rinex2_1_NavDatum navDatum);
		bool   exportSP3File(string strSP3fileName, DayTime T_Begin, DayTime T_End, double spanSeconds = 300);
        bool   exportCLKFile(string strCLKfileName, DayTime T_Begin, DayTime T_End, double spanSeconds = 120);
		bool   exportSP3File_GPST(string strSP3fileName, GPST T_Begin, GPST T_End, double spanSeconds = 300);
		bool   exportCLKFile_GPST(string strCLKfileName, GPST T_Begin, GPST T_End, double spanSeconds = 120);
	public:
		Rinex3_0_NavHeader  m_header;
		NavSatMap           m_data;
		int                 m_typeSatSystem;
		double              m_EARTH_W;
		double              m_GM_EARTH;
	};
}
