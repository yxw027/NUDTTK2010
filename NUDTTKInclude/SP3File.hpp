#pragma once
#include "structDef.hpp"
#include <vector>
#include <windows.h>
#include <map>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct SP3Header
	{
		//Line 1
		char        szSP3Version[2+1];      // A2
		char        szPosVelFlag[1+1];      // A1
		GPST        tmStart;
		long        nNumberofEpochs; 
		char        szDataType[5+1];        // A5
		char        szCoordinateSys[5+1];   // A5--ITR97  IGS00 
		char        szOrbitType[3+1];       // A3--HLM
		char        szAgency[4+1];          // A4--IGS 轨道发布机构
		//Line 2
		char        szLine2Symbols[2+1];    // A2    
		GPSWeekTime tmGPSWeek;
		double      dEpochInterval;
		long        nModJulDaySt;           // 轨道数据首历元约化儒略日的整数部分
		double      dFractionalDay;         // 轨道数据首历元儒略日的小数部分
		//Line 3--7
		char        szLine3Symbols[2+1];    // A2   
		BYTE        bNumberofSats;
		vector<string> pstrSatNameList;
		//Line 8--12
		char        szLine8Symbols[2+1];    // A2   
		vector<BYTE> pbySatAccuracyList;    // 卫星的精度
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

		SP3Header()
		{
			//memset(this,0,sizeof(SP3Header));
			//pstrSatNameList.clear();
		}
		~SP3Header()
		{
		}

		char getSatSystemChar() // 2012/04/03, 增加北斗系统的考虑
		{
			if(szFileType[0] == 'C')
				return 'C';
			else if(szFileType[0] == 'M')	// 2014/03/23, 增加混合系统的考虑
				return 'M'; 
			else
				return 'G';
		}
	};

	struct SP3Datum
	{
		POS3D   pos;
		POS3D   vel;
		double  clk;     // 钟差改正
		double  clkrate; // 钟的变化率
		bool    valid;   // 有效标记
	};

	typedef map<string, SP3Datum> SP3SatMap; // 单个时刻的不同卫星星历数据列表

    struct SP3Epoch // 星历数据历元结构
	{
		GPST t;
		SP3SatMap sp3; 
		double getMJD();
	};

	class SP3File
	{
	public:
		SP3File(void);
	public:
		~SP3File(void);
	public:
		static int getSatPRN(string strSatName);
	public:
		void   clear();
		bool   isEmpty();
		int    isValidEpochLine(string strLine, FILE * pSP3file = NULL);
		double getEpochSpan();
		bool   open(string  strSp3FileName);
		bool   openV(string  strSp3FileName); // 具有速度标识的sp3轨道文件
		bool   write(string strSp3FileName);
		bool   getEphemeris(GPST t,   string name, SP3Datum& sp3Datum, int nLagrange = 9);
		bool   getEphemeris_0(GPST t, string name, SP3Datum& sp3Datum, int nLagrange = 9);
		bool   getEphemeris(GPST t, int nPRN, SP3Datum& sp3Datum, int nLagrange = 9, char cSatSystem = 'G');
		bool   getEphemeris_PathDelay(GPST t, POSCLK receiverPosClock, string name, double& delay, SP3Datum& sp3Datum,double threshold = 1.0E-07);
		bool   getEphemeris_PathDelay(GPST t, POSCLK receiverPosClock, int nPRN, double& delay, SP3Datum& sp3Datum,double threshold = 1.0E-07);

		bool   getClock(GPST t, string name, double &clk, double &clkrate, int nLagrange = 2);
		void   exportCLKFile(string strCLKfileName, DayTime T_Begin, DayTime T_End, double spanSeconds = 30);
	public:
        SP3Header         m_header;
		vector<SP3Epoch>  m_data;
	};
}
