#pragma once
#include "structDef.hpp"
#include <vector>
#include <windows.h>
#include <map>
#include <limits>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	// 测站的名字和坐标
	struct CLKStaNamePos
	{
		char    szName[4 + 1];
		char    szID[20 + 1];
		__int64 lnX; // 单位毫米, 11位, 普通整型 INT_MAX = 2147483647 会溢出, 这里使用64位整型
		__int64 lnY;
		__int64 lnZ;
	};

	// 钟差文件头信息
	struct CLKHeader
	{
		char                    szRinexVersion[20 + 1]; // 文件类型
		char                    cFileType;
		char                    szProgramName[20 + 1];
		char                    szAgencyName[20 + 1];
		char                    szFileDate[20 + 1];
		int                     LeapSecond;             // 自1980年1月6日的跳秒
		int                     ClockDataTypeCount;     // 钟差数据类型个数
		vector<string>          pstrClockDataTypeList;
		char                    szACShortName[3 + 1];   // AC - 分析中心
		char                    szACFullName[55 + 1];
		int                     nStaCount;              
		char                    szStaCoordFrame[50 + 1];
		vector<CLKStaNamePos>   pStaPosList;     
		BYTE                    bySatCount;
		vector<string>          pszSatList; 
		CLKHeader()
		{
			//memset(this, 0, sizeof(CLKHeader));
			LeapSecond = INT_MAX;
			ClockDataTypeCount = INT_MAX;
			nStaCount = INT_MAX;
			bySatCount = 0;
		}
	};

	struct CLKDatum
	{
		string name; 
		int    count;
		double clkBias;
		double clkBiasSigma;
		double clkRate;
		double clkRateSigma;
		double clkAcc;
		double clkAccSigma;
		CLKDatum()
		{
			/*  
				memset 使得 string 内部指针 _Bx._Ptrr 值为 0, _Myres 为 0, 在这种情况下当 string 对象被赋值为小字符串(字节数小于等于16的字符串)时,
				因新申请的内存在后来得不到释放, 所以这块内存被泄露了,根据 string 类内存管理算法得知这块内存大小总是 16 个字节.
				但当被赋值为大字符串(字节数大于16的字符串)时, 反而没有内存泄露, 这是因为新申请的内存在析构或下次赋值时总能被释放.
			*/

			//memset(this, 0, sizeof(CLKDatum)); // 2014/10/19 

			name = "";
			count = 0;
			clkBias = DBL_MAX;
			clkBiasSigma = DBL_MAX;
			clkRate = DBL_MAX;
			clkRateSigma = DBL_MAX;
			clkAcc = DBL_MAX;
			clkAccSigma = DBL_MAX;
		}
	};

	typedef map<string, CLKDatum>  CLKMap;

	struct CLKEpoch
	{
		GPST     t;
		CLKMap   ASList; 
		CLKMap   ARList;
	};

	class CLKFile
	{
	public:
		CLKFile(void);
	public:
		~CLKFile(void);
	public:
		void   clear();
		bool   isEmpty();
		int    isValidEpochLine(string strLine, FILE * pCLKFile = NULL);
		int    isValidEpochLine_rinex304(string strLine, FILE * pCLKFile = NULL);
		double getEpochSpan();
		bool   open(string  strCLKFileName);
		bool   open_rinex304(string  strCLKFileName);
		bool   open_LeoClk(string  strCLKFileName);  // 低轨卫星钟差，存在大的间断现象
		bool   write(string strCLKFileName);
		bool   getSatClock(GPST t, int nPRN, CLKDatum& clkDatum, int nLagrange = 3, char cSatSystem = 'G');
		bool   getSatClock(GPST t,   string name, CLKDatum& clkDatum, int nLagrange = 3);
		bool   getSatClock_0(GPST t, string name, CLKDatum& clkDatum, int nLagrange = 3);
		
		bool   getStaClock(GPST t, string name, CLKDatum& clkDatum, int nLagrange = 3);
		bool   getLeoClock(GPST t, string name, CLKDatum& clkDatum, int nLagrange = 3); // 低轨卫星钟差，存在大的间断现象
	public:
		CLKHeader          m_header;
		vector<CLKEpoch>   m_data;
	};
}
