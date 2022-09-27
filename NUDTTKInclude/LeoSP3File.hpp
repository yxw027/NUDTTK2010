#pragma once
#include "structDef.hpp"
#include <windows.h>
#include <vector>
#include <map>

namespace NUDTTK
{
	namespace DORIS
	{
		struct LeoSP3Header
		{
			//Line 1
			char        szSP3Version[1+1];      // A2
			char        szPosVelFlag[2+1];      // A1
			TAI		    tmStart;
			long        nNumberofEpochs; 
			char        szDataType[5+1];        // A5
			char        szCoordinateSys[5+1];   // A5--ITR97  IGS00 
			char        szOrbitType[3+1];       // A3--HLM
			char        szAgency[4+1];          // A4--IGS 轨道发布机构
			//Line 2
			char        szLine2Symbols[2+1];    // A2    
			WeekTime    tmGPSWeek;
			double      dEpochInterval;
			long        nModJulDaySt;           // 轨道数据首历元约化儒略日的整数部分
			double      dFractionalDay;         // 轨道数据首历元儒略日的小数部分
			//Line 3--7
			char        szLine3Symbols[2+1];     // A2   
			int         bNumberofSats;           //卫星颗数
			char        NameofSat[3+1];          //A3
			//Line 8--12
			char        szLine8Symbols[2+1];    // A2   
			int         pbySatAccuracy;         // 卫星的精度
			//Line 13-14
			char        szLine13Symbols[2+1];   // A2  
			char        szFileType[2+1];        // A2 FileType
			char        szTimeSystem[3+1];      // A3
			//Line 15-16
			char        szLine15Symbols[2+1];   // A2 
			double      dBaseforPosVel;
			double      dBaseforClkRate;
			//Line 17-18
			char        szLine17Symbols[2+1];   // A2
			//Line 19-22
			char        szLine19Symbols[2+1];   // A2
			char        szLine19Comment[57+1];  // A57
			char        szLine20Comment[57+1];  // A57
			char        szLine21Comment[57+1];  // A57
			char        szLine22Comment[57+1];  // A57

			LeoSP3Header()
			{
				memset(this,0,sizeof(LeoSP3Header));
			}
		};

		struct LeoSP3Datum
		{
			POS3D   pos;
			POS3D   vel;
			double  clk;     
		};
		// 星历数据历元结构
        struct LeoSP3Epoch
		{
			TAI          t;
			LeoSP3Datum  sp3;  
			double getMJD();
		};

		// 主要用于单颗低轨用户卫星(非导航星座)轨道文件解析, 该类由刘俊宏完成
		class LeoSP3File
		{
		public:
			LeoSP3File(void);
		public:
			~LeoSP3File(void);
		public:
			void clear();
			bool isEmpty();
			int  isValidEpochLine(string strLine,FILE * pSP3File = NULL);
			bool open(string  strSP3FileName);
			bool write(string strSP3FileName);
			bool getEphemeris(TAI t, POS6D &posvel, int nLagrange = 9, double factor_pos = 1000.0, double factor_vel = 0.1);
			bool add(string strSP3FileName);
			void split(string strName, double period = 86400.0);
		public:
			LeoSP3Header           m_header;
			vector<LeoSP3Epoch>    m_data;
		};
	}
}
