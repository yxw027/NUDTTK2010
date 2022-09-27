#pragma once
#include "structDef.hpp"
#include "constDef.hpp"
#include <windows.h>
#include <limits>
#include <vector>
#include <map>
#include "TimeCoordConvert.hpp"
#include "CLKFile.hpp"

namespace NUDTTK
{
	struct DELTA_UTC
	{
		double	dA0;
		double	dA1;
		long	lnT;
		long	lnW;
	};

    struct Rinex2_1_NavHeader 
	{
		char        szRinexVersion[20 + 1];
		char        szFileType[20 + 1];         // ('N' for Navigation data) | A1,19X
		char        szFileDate[20 + 1];         // 文件创建日期
		double		pdIonAlpha[4];
		double		pdIonBeta[4];
		DELTA_UTC   deltaUTC;
		long		lnLeapSeconds;

		Rinex2_1_NavHeader()
		{
			memset(this, 0, sizeof(Rinex2_1_NavHeader));
			lnLeapSeconds = LONG_MAX;
		}
	};

	struct Rinex2_1_NavDatum 
	{
		//PRN / EPOCH / SV CLK
		char       cSatSys;
		BYTE       bySatPRN;              
		DayTime    tmTimeOfClock;
		double	   dSVClockBias;
		double	   dSVClockDrift;
		double	   dSVClockDriftRate;
		//BROADCAST ORBIT - 1
		double	   dIODE;
		double	   dCrs;
		double	   dDetla_n;
		double	   dM0;
		//BROADCAST ORBIT - 2
		double	   dCuc;
		double	   dEccentricity;
		double	   dCus;
		double	   dSqrt_A;
		//BROADCAST ORBIT - 3
		double	   dTOE;
		double	   dCic;
		double	   dOMEGA;
		double	   dCis;
		//BROADCAST ORBIT - 4
		double	   d_i0;
		double	   dCrc;
		double	   d_omega;
		double	   dOMEGADOT;
		//BROADCAST ORBIT - 5
		double	   didot;
		double	   dCodesOnL2Channel;
		double	   dWeek;
		double	   dL2PDataFlag;
		//BROADCAST ORBIT - 6
		double	   dSVAccuracy;
		double	   dSVHealth;
		double	   dTGD;
		double	   dIODC;
		//BROADCAST ORBIT - 7
		double	   dTransmissionTimeOfMessage;
		double	   dFitInterval;
		double	   dSpare1;
		double	   dSpare2;

		Rinex2_1_NavDatum()
		{
			cSatSys                    = 'G';//默认GPS，张厚喆/05/17
			dSVClockBias               = DBL_MAX;
			dSVClockDrift              = DBL_MAX;
			dSVClockDriftRate          = DBL_MAX;
			dIODE                      = DBL_MAX;
			dCrs                       = DBL_MAX;
			dDetla_n                   = DBL_MAX;
			dM0                        = DBL_MAX;
			dCuc                       = DBL_MAX;
			dEccentricity              = DBL_MAX;
			dCus                       = DBL_MAX;
			dSqrt_A                    = DBL_MAX;
			dTOE                       = DBL_MAX;
			dCic                       = DBL_MAX;
			dOMEGA                     = DBL_MAX;
			dCis                       = DBL_MAX;
			d_i0                       = DBL_MAX;
			dCrc                       = DBL_MAX;
			d_omega                    = DBL_MAX;
			dOMEGADOT                  = DBL_MAX;
			didot                      = DBL_MAX;
			dCodesOnL2Channel          = DBL_MAX;
			dWeek                      = DBL_MAX;
			dL2PDataFlag               = DBL_MAX;
			dSVAccuracy                = DBL_MAX;
			dSVHealth                  = DBL_MAX;
			dTGD                       = DBL_MAX;
			dIODC                      = DBL_MAX;
			dTransmissionTimeOfMessage = DBL_MAX;
			dFitInterval               = DBL_MAX;
			dSpare1                    = DBL_MAX;
			dSpare2                    = DBL_MAX;
		}

		WeekTime getWeekTime_toe()
		{
			WeekTime t_wk;
			t_wk.week   = int(dWeek);
			t_wk.second = dTOE;
			return t_wk;
		}
	};

	typedef vector<Rinex2_1_NavDatum> NavDatumList;
	typedef map<int, NavDatumList>    NavSatMap;

	class Rinex2_1_NavFile
	{
	public:
		Rinex2_1_NavFile(void);
	public:
		~Rinex2_1_NavFile(void);
	public:
		void   clear();
		bool   isEmpty();
		int    isValidEpochLine(string strLine, FILE * pNavfile = NULL);
		bool   open(string  strNavfileName);
    	bool   write(string strNavfileName);
		void   writedouble(FILE* pNavfile, double value);
		bool   getEphemeris(DayTime T, int nPRN, Rinex2_1_NavDatum& navDatum, double threshold_span_max = 7200.0, bool onForwad = false);
		bool   getEphemeris(DayTime T, int nPRN, POSCLK &posclk, double threshold_span_max = 7200.0, bool onForwad = false);
		double getClock(DayTime T, Rinex2_1_NavDatum navDatum);
		POSCLK getPosition(DayTime T, Rinex2_1_NavDatum navDatum);
		void   exportSP3File(string strSP3fileName, DayTime T_Begin, DayTime T_End, double spanSeconds = 300, double threshold_span_max = 7200.0, bool onForwad = false);
        void   exportCLKFile(string strCLKfileName, DayTime T_Begin, DayTime T_End, double spanSeconds = 120, double threshold_span_max = 7200.0, bool onForwad = false);
	public:
		Rinex2_1_NavHeader  m_header;
		NavSatMap           m_data;
		int                 m_typeSatSystem;
		double              m_EARTH_W;
		double              m_GM_EARTH;
	};
}
