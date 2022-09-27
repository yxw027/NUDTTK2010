#pragma once
#include "structDef.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		// 科学轨道数据文件行单元
		struct CHORBLine 
		{
			long    lnJD2000;           // 1 - 6  I6  Time tag (10**-1 d since J2000.0) 
			__int64 lnSecond;           // 7 - 17 I11 Time tag (10**-6 s since 0 hours)
			POS3D   pos;
			POS3D   vel;
			double  angleRoll;          // 90 - 96 I7 Roll angle   (10**-3 deg)
			double  anglePitch;         // 97 - 103 I7 Pitch angle (10**-3 deg)
			double  angleYaw;           // 104 - 110 I7 Yaw angle  (10**-3 deg)
			double  densityNeutralGas;  // Neutral gas density (10**-16 g/cm**3)
			char    cManeuverFlag;     	// Maneuver flag (M = yes, else blank)
			char    cLandWaterFlag;     // Land/water flag (L = Land, W = Water)
			char    cAscendFlag;        // Ascending/descending arc flag (A = ascending, D = descending)
			char    cEclipseFlag;       // Eclipse flag (E = satellite in Earth's shadow, else blank)
		
			TDT gettime()
			{   
				// 根据 CHAMP 轨道的特殊时间标记(高精度儒略日, 天和秒分开存储), 保精度的换算成 TDT 时间
				// 分开算先算天再加上秒的部分
				double jd2000 = lnJD2000 * 0.1; 
				double jd = jd2000 + 2451545.0; // J2000 对应儒略日 2451545.0
				TDT t_TDT = TimeCoordConvert::JD2DayTime(jd);
				double sec = lnSecond * 0.000001;
				t_TDT = t_TDT + sec;
				return t_TDT;
			}
		};

		class champORBFile
		{
		public:
			champORBFile(void);
		public:
			~champORBFile(void);
		public:
			void    clear();
			bool    isEmpty();
			int     isValidEpochLine(string strLine, FILE * pCHORBFile = NULL);
			string  dataline2string(CHORBLine orbLine);
			bool    open(string  strCHORBFileName);
			bool    write(string strCHORBFileName);
			bool    append(champORBFile& CHORBfile);
			bool    append(string strCHORBFileName);
			bool    cutdata(TDT t0,TDT t1);
			bool    cutdata();
			bool    getCHORBLine(TDT t, CHORBLine& orbLine,int nLagrange = 8);
			bool    orbComparision_RTN(GPST t, TimePosVelClock posvel, POS6D& error);
			bool    orbComparision_XYZ(GPST t, TimePosVelClock posvel, POS6D& error);
		public:
			int     m_type;
			vector<CHORBLine> m_data;
		};
	}
}
