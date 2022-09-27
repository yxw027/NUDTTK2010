#pragma once
#include "structDef.hpp"
#include <vector>
#include <limits>
#include <windows.h>
#include <map>
#include "Rinex2_1_ObsFile.hpp"

//  Copyright 2014, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct Rinex2_1_MixedObsHeader
	{
		char           szRinexVersion[20 + 1];        // 文件类型
		char           szFileType[20 + 1];            // A1,19X  'O' for Observation Data 
		char           szSatlliteSystem[20 + 1];      // A1,19X  blank or 'G': GPS
		char           szProgramName[20 + 1];         // 程序名称
		char           szProgramAgencyName[20 + 1];   // 程序机构名称
		char           szFileDate[20 + 1];            // 文件创建日期
		char           szObserverName[20 + 1];        // 观测者名称，默认 OBSERVER
		char           szObserverAgencyName[40 + 1];  // 观测机构名称
		char           szRecNumber[20 + 1];           // 接收机编号
		char           szRecType[20 + 1];             // 接收机类型
		char           szRecVersion[20 + 1];          // 接收机软件版本
		char		   szMarkNumber[20 + 1];
		char		   szMarkName[60 + 1];
		char           szAntNumber[20 + 1];           // 天线编号
		char           szAntType[20 + 1];             // 天线类型
		char           szTimeType[3 + 1];             // 时间类型
		POS3D          ApproxPos;                     // 概略位置
		POS3D          AntOffset;                     // 天线偏移
		DayTime		   tmStart;                       // 起始时间
		DayTime		   tmEnd;                         // 终止时间
		BYTE		   bL1WaveLengthFact;             // 1-整波长, 2-半波长,0-(in L2): Single frequency instrument 
		BYTE		   bL2WaveLengthFact;
		BYTE		   bL5WaveLengthFact;
		double         Interval;                     // 观测数据采样间隔
		int            LeapSecond;                   // 自1980年1月6日的跳秒
		BYTE           bySatCount;                   // 卫星(或测站)个数
		vector<string> pstrSatList;                  // 卫星(或测站)列表, 2014/03/22, 针对Mixed格式进行调整
		BYTE           byObsTypes;                   // 观测数据类型个数
		vector<BYTE>   pbyObsTypeList;				 // 不同观测数据的序列
		vector<string> pstrCommentList;              // 注释行序列, 2008/07/27		
		
		Rinex2_1_MixedObsHeader()
		{
			//memset(this,0,sizeof(Rinex2_1_ObsHeader));
			LeapSecond = INT_MAX;
			Interval   = DBL_MAX;
			bL1WaveLengthFact = 100;
		}

		char getSatSystemChar() // 2012/04/09, 增加北斗系统的考虑
		{
			if(szSatlliteSystem[0] == 'G' || szSatlliteSystem[0] == ' ')
				return 'G';
			else if(szSatlliteSystem[0] == 'M')	// 2013/06/26, 增加混合系统的考虑
				return 'M'; 
			else
				return 'C';
		}
		void init(Rinex2_1_ObsHeader m_header);
	};
	
	typedef map<string, Rinex2_1_ObsTypeList> Rinex2_1_MixedSatMap; // 单个时刻的不同卫星观测数据列表

    struct Rinex2_1_MixedObsEpoch
	{
		DayTime                  t;
        BYTE                     byEpochFlag; 
		BYTE                     bySatCount;		
		Rinex2_1_MixedSatMap     obs;             
	};

	class Rinex2_1_MixedObsFile
	{
	public:
		Rinex2_1_MixedObsFile(void);
	public:
		~Rinex2_1_MixedObsFile(void);
	public:
		void    clear();
        bool    isEmpty();
        int     isValidEpochLine(string strLine, FILE * pObsfile = NULL);
		bool	open(string  strObsfileName, string strSystem = "G+C");
		bool	open_5Obs(string  strObsfileName, string strSystem = "G+C");
		bool    write(string strObsfileName_noExp);
		bool    write_5Obs(string strObsfileName_noExp);
		bool    write(string strObsfileName_noExp, string& strObsfileName_all);
		bool    write_5Obs(string strObsfileName_noExp, string& strObsfileName_all);
	public:
		Rinex2_1_MixedObsHeader          m_header;
		vector<Rinex2_1_MixedObsEpoch>   m_data;
	};
}
