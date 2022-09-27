#pragma once
#include "structDef.hpp"
#include <vector>
#include <limits>
#include <windows.h>
#include <map>
#include "Rinex2_1_ObsFile.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha

using namespace std;

namespace NUDTTK
{	
	struct Rinex3_0_MaskString
	{
		static const char szVerType[];			
		static const char szPgmRunDate[];
		static const char szComment[];
		static const char szMarkerName[];
		static const char szMarkerNum[];
		static const char szMarkerType[];
		static const char szObservAgency[];		
		static const char szRecTypeVers[];
		static const char szAntType[];
		static const char szApproxPosXYZ[];
		static const char szAntDeltaHEN[];
		static const char szAntDeltaXYZ[];
		static const char szAntPhaseCenter[];
		static const char szAntBSightXYZ[];
		static const char szZeroDirAZI[];
		static const char szZeroDirXYZ[];
		static const char szCerterOfMassXYZ[];
		static const char szSysTypeOfObs[];
		static const char szSignalStrUnit[];
		static const char szInterval[];
		static const char szTmOfFirstObs[];	
		static const char szTmOfLastObs[];
		static const char szRecClockOffApp[];
		static const char szSysDCBSApp[];
		static const char szSysPCVSApp[];
		static const char szIonoCorr[];
		static const char szTimeSysCorr[];
		static const char szSysScaleFac[];
		static const char szLeapSec[];
		static const char szNumsOfSv[];
		static const char szPRNNumOfObs[];		
		static const char szEndOfHead[];		
	};		
	struct AntPhaCen
	{
		char          szSatlliteSystem[1 + 1];        // 卫星系统
		char          szObsCode[3 + 1];               // Observation Code
		POS3D         PhaCen;                         // 天线相位中心的位置(NEU for fix station，XYZ for vehicle)(单位m)
	};
	struct SysObsTyp
	{
		char          szSatlliteSystem[1 + 1];        // 卫星系统	
		int           ObsTypCount;                    // 观测数据种类型个数(使用之前需要乘以ScaFactor的观测数据类型个数)
		int           ScaFactor;                      // 使用之前，观测数据需要乘以ScaFactor，（1，10，100，1000）
		vector<string> ObsTypelist;                   // 观测数据类型列表（RINEX3.0最多可以放13种观测数据类型,每种观测数据占3个字符）
	};
	struct DCBPCVApp
	{
		char          szSatlliteSystem[1 + 1];        // 卫星系统
		char          szCorPro[17 + 1];               // 修正程序名
		char          szCorSou[40 + 1];               // 修正来源
	};	
	struct Rinex3_0_ObsHeader                         // 用于储存文件头信息
	{
		double         RinexVersion;                  // 文件格式版本，F9.2,11X
		char           szFileType[1 + 1];             // A1,19X  'O' for Observation Data 
		char           szSatlliteSystem[1 + 1];       // A1,19X  'G': GPS，‘R ’:GLONASS,'E':Galileo,'S':SBAS payload,‘D':DORIS,'M':Mixed,'C':COMPASS(北斗)
		char           szProgramName[20 + 1];         // 程序名称
		char           szProgramAgencyName[20 + 1];   // 程序机构名称
		char           szFileDate[20 + 1];            // 文件创建日期，推荐为UTC，未知为LCL
		char           szCommentLine[60 + 1];         // 注释行				
		char		   szMarkName[60 + 1];            // 天线生产者名
		char		   szMarkNumber[20 + 1];          // 天线生产者编号
        char		   szMarkType[20 + 1];            // 天线类型
		char           szObserverName[20 + 1];        // 观测者名称
		char           szObserverAgencyName[40 + 1];  // 观测机构名称
		char           szRecNumber[20 + 1];           // 接收机编号
		char           szRecType[20 + 1];             // 接收机类型
		char           szRecVersion[20 + 1];          // 接收机软件版本
		char           szAntNumber[20 + 1];           // 天线编号
		char           szAntType[20 + 1];             // 天线类型
		POS3D          ApproxPosXYZ;                  // 参考点位置（地固系，单位m）3F14.4
		POS3D          AntDeltaHEN;                   // 天线距参考点的高度和水平偏差(单位m)
		POS3D          AntDeltaXYZ;                   // 运动载体的天线在参考坐标系(星体系，body-fixed)的位置(单位m)		
		POS3D          AntDirection;                  // 天线指向
		double         AntZeroDirAZI;                 // 天线零方位角(角度，从北向开始的角度)
		POS3D          AntZeroDirXYZ;                 // 运动载体的天线零指向
		POS3D          AntCenOfMas;                   // 质心（星体系，单位m）
		vector<AntPhaCen> AntPhaCentList;             // 各卫星系统的天线相位中心列表
		vector<SysObsTyp> SysObsTypList;              // 各卫星系统的观测数据类型列表
		vector<SysObsTyp> SysSclFacList;              // 各卫星系统使用之前需要乘以ScaFactor的观测数据类型列表
		vector<DCBPCVApp> SysDCBAppList;              // DCB修正
		vector<DCBPCVApp> SysPCVAppList;              // PCV修正		
		char           szSignalSteUnit[20 + 1];       // 信号强度单位(S/N dbHz)
		double         Interval;                      // 观测数据采样间隔
		DayTime		   tmStart;                       // 起始时间
		DayTime		   tmEnd;                         // 终止时间
		char           szTimeSystem[3+1];             // 时间系统，GPS，GLO，GAL，DOR，BDT
		int            RecClokOffApp;                 // 接收机钟差修正		
		int            LeapSecond;                    // 自1980年1月6日的跳秒
		int            SatCount;                      // 观测卫星数
		Rinex3_0_ObsHeader()
		{
			memset(this,0,sizeof(Rinex3_0_ObsHeader));	
			LeapSecond      = INT_MAX;
			RecClokOffApp   = INT_MAX;
			SatCount        = INT_MAX;
			Interval        = DBL_MAX;
			ApproxPosXYZ.x  = DBL_MAX;
			AntDeltaHEN.x   = DBL_MAX;
			AntDeltaXYZ.x   = DBL_MAX;
			AntDirection.x  = DBL_MAX;
			AntZeroDirAZI   = DBL_MAX;
			AntZeroDirXYZ.x = DBL_MAX;
			AntCenOfMas.x   = DBL_MAX;
		}
	};	    
	struct SysObs                                                 //  单个时刻的某一卫星系统的观测数据
	{
		char                     szSatlliteSystem[1 + 1];         // 卫星系统
		Rinex2_1_SatMap          sobs;                            // 观测数据
	};
    struct Rinex3_0_ObsEpoch
    {
		char                     szRecordIdentifier[1 + 1];       // 数据记录标记		
		DayTime                  t;                               // 时间
        BYTE                     byEpochFlag;                     // 数据质量标记，0:OK，1:断电，>1:特殊事件
    	int                      EpochSatCount;                   // 当前时刻数据个数
		double                   RecClockOffset;                  // 接收机钟差		
    	vector<SysObs>           obs;                             // 观测数据	
		Rinex3_0_ObsEpoch()
		{
			memset(this,0,sizeof(Rinex3_0_ObsEpoch));	
		}
    };	
    class Rinex3_0_ObsFile
	{
	public:
		Rinex3_0_ObsFile(void);
	public:
		~Rinex3_0_ObsFile(void);
	public:		
		void    clear();
		bool    isEmpty();
		int     isValidEpochLine(string strLine, FILE * pObsfile = NULL);
		bool    open(string  strObsfileName, bool bOn_BDT2GPST = false);
		bool    write(string strObsfileName_noExp);
		bool    write(string strObsfileName_noExp, string& strObsfileName_all);	
		bool    rinex3_0T2_1File(Rinex2_1_ObsFile &obsFile, char szSatlliteSystem[], int mark_obstype = 0);
	public:
		Rinex3_0_ObsHeader            m_header;
		vector<Rinex3_0_ObsEpoch>     m_data; 
	};
}

