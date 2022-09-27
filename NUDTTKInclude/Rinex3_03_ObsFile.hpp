#pragma once
#include "structDef.hpp"
#include <vector>
#include <limits>
#include <windows.h>
#include <map>
#include "Rinex2_1_ObsFile.hpp"

//  Copyright 2018, The National University of Defense Technology at ChangSha

using namespace std;

namespace NUDTTK
{	
	struct Rinex3_03_MaskString
	{
		static const char szVerType[];	
		static const char szPgmRunDate[];
		static const char szComment[];
		static const char szMarkerName[];
		static const char szMarkerNum[];
		static const char szMarkerType[]; // +
		static const char szObservAgency[];		
		static const char szRecTypeVers[];
		static const char szAntType[];
		static const char szApproxPosXYZ[];
		static const char szAntDeltaHEN[];
		static const char szAntDeltaXYZ[]; // +
		static const char szAntPhaseCenter[]; // +
		static const char szAntBSightXYZ[];// +
		static const char szZeroDIRAzimuth[];// +
		static const char szZeroDIRXYZ[];// +
		static const char szCOMXYZ[];// +
		static const char szSysObsTypes[];// +
		static const char szSignalStrengthUnit[];// +
		static const char szInterval[];
		static const char szTmOfFirstObs[];
		static const char szTmOfLastObs[];
		static const char szRCVClockOffsetApplied[];// +
		static const char szSysDCBSApplied[];// +
		static const char szSysPCVSApplied[];// +
		static const char szSysScaleFactor[];// +
		static const char szSysPhaseShift[]; // +
		static const char szGlonassSlotFrq[];// +
		static const char szGlonassCodePhaseBias[];// +
	    static const char szLeapSec[];
		static const char szNumsOfSat[];
		static const char szPRNNumOfObs[];		
		static const char szEndOfHead[];	
	};

	struct Rinex3_03_AntPhaseCenter
	{
		char          cSatSys;                   // 卫星系统
		char          szObsCode[3 + 1];          // Observation Code
		POS3D         phaseCenter;               // 天线相位中心的位置(NEU for fix station，XYZ for vehicle)(单位m)
	};

	struct Rinex3_03_SysObsType
	{
		char           cSatSys;                  // 卫星系统	
		int            obsTypeCount;             // 观测数据种类型个数(使用之前需要乘以ScaFactor的观测数据类型个数)
		int            scaleFactor;              // 使用之前，观测数据需要乘以scaleFactor，（1，10，100，1000）
		vector<string> obsTypeList;              // 观测数据类型列表（超过12种观测数据类型,换行）
	};

	struct Rinex3_03_SysPhaseShift
	{
		char           cSatSys;                  // 卫星系统	
		char           obsType[3 + 1];           // 观测类型
		double         phaseShift;               // Correction applied (cycles)
        int            satCount;                 // 卫星数目
		vector<string> satList;                  // 卫星列表（超过10颗换行）
	};

	struct Rinex3_03_CorrApplied
	{
		char          cSatSys;                   // 卫星系统
		char          szNameProgram[17 + 1];          // 修正程序名
		char          szNameURL[40 + 1];              // 修正来源
	};
	struct Rinex3_03_ObsHeader                        // 用于储存文件头信息
	{
		char           rinexVersion[10 + 1];                  // 文件格式版本，F9.2,11X
		char           szFileType[1 + 1];             // A1,19X  'O' for Observation Data 
		char           cSatSys;                       // A1,19X  'M':Mixed;'G':GPS;'R':GLONASS;'S':SBAS;'E':Galileo;'C':BeiDou;'J':QZSS;'I':IRNSS
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
		POS3D          approxPosXYZ;                  // 参考点位置（地固系，单位m）3F14.4
		POS3D          antDeltaHEN;                   // 天线距参考点的高度和水平偏差(单位m)
		POS3D          antDeltaXYZ;                   // 运动载体的天线在参考坐标系(星体系，body-fixed)的位置(单位m)		
		POS3D          antBSightXYZ;                  // 天线指向
		double         antZeroDIRAzimuth;             // 天线零方位角(角度，从北向开始的角度)
		POS3D          antZeroDIRXYZ;                 // 运动载体的天线零指向
		POS3D          antCOMXYZ;                     // 质心（星体系，单位m）3F14.4
		vector<Rinex3_03_AntPhaseCenter> phaseCenterList;    // 各卫星系统的天线相位中心列表
		vector<Rinex3_03_SysObsType>     sysObsTypeList;     // 各卫星系统的观测数据类型列表
		vector<Rinex3_03_SysObsType>     sysScaleFactorList; // 各卫星系统使用之前需要乘以ScaFactor的观测数据类型列表
		vector<Rinex3_03_CorrApplied>    sysDCBSAppliedList; // DCB修正
		vector<Rinex3_03_CorrApplied>    sysPCVSAppliedList; // PCV修正
		vector<Rinex3_03_SysPhaseShift>  sysPhaseShiftList;  // PhaseShift修正
		char           szSignalStrengthUnit[20 + 1];  // 信号强度单位(S/N dbHz)
		double         interval;                      // 观测数据采样间隔
		DayTime		   tmStart;                       // 起始时间
		DayTime		   tmEnd;                         // 终止时间
		char           szTimeSystem[3+1];             // 时间系统，GPS，GLO，GAL，DOR，BDT
		int            rcvClockOffsetApplied;         // 接收机钟差修正		
		int            leapSecond;                    // 自1980年1月6日的跳秒
		int            satCount;                      // 观测卫星数
		
		Rinex3_03_ObsHeader()
		{
			memset(this,0,sizeof(Rinex3_03_ObsHeader));	
			leapSecond            = INT_MAX;
			rcvClockOffsetApplied = INT_MAX;
			satCount              = INT_MAX;
			interval              = DBL_MAX;
			approxPosXYZ.x        = DBL_MAX;
			approxPosXYZ.y        = DBL_MAX;
			approxPosXYZ.z        = DBL_MAX;
			antDeltaHEN.x         = DBL_MAX;
			antDeltaHEN.y         = DBL_MAX;
			antDeltaHEN.z         = DBL_MAX;
			antDeltaXYZ.x         = DBL_MAX;
			antDeltaXYZ.y         = DBL_MAX;
			antDeltaXYZ.z         = DBL_MAX;
			antBSightXYZ.x        = DBL_MAX;
			antBSightXYZ.y        = DBL_MAX;
			antBSightXYZ.z        = DBL_MAX;
			antZeroDIRAzimuth     = DBL_MAX;
			antZeroDIRXYZ.x       = DBL_MAX;
			antZeroDIRXYZ.y       = DBL_MAX;
			antZeroDIRXYZ.z       = DBL_MAX;
			antCOMXYZ.x           = DBL_MAX;
			antCOMXYZ.y           = DBL_MAX;
			antCOMXYZ.z           = DBL_MAX;
		}
	};	

	typedef map<string, Rinex2_1_ObsTypeList> Rinex3_03_SatMap;   // 单个时刻的不同卫星观测数据列表

	struct Rinex3_03_SysObs                                                 //  单个时刻的某一卫星系统的观测数据
	{
		char                     cSatSys; // 卫星系统
		Rinex3_03_SatMap         obsList; // 观测数据
	};

    struct Rinex3_03_ObsEpoch
    {
		char                     cRecordId[1+1];      // 数据记录标记		
		DayTime                  t;                    // 时间
        BYTE                     byEpochFlag;          // 数据质量标记，0:OK，1:断电，>1:特殊事件
    	int                      satCount;             // 卫星数据个数
		double                   clock;                // 接收机钟差, 单位: s		
    	vector<Rinex3_03_SysObs> obs;                  // 观测数据	
		
		Rinex3_03_ObsEpoch()
		{
			memset(this,0,sizeof(Rinex3_03_ObsEpoch));	
		}
    };	

	class Rinex3_03_ObsFile
	{
	public:
		Rinex3_03_ObsFile(void);
	public:
		~Rinex3_03_ObsFile(void);
	public:		
		void    clear();
		bool    isEmpty();
		int     isValidEpochLine(string strLine, FILE * pObsfile = NULL);
		bool    open(string  strObsfileName, bool on_BDT2GPST = false);
		bool    write(string strObsfileName_noExp);
		bool    write(string strObsfileName_noExp, string& strObsfileName_all);	
	public:
		Rinex3_03_ObsHeader            m_header;
		vector<Rinex3_03_ObsEpoch>     m_data; 
	};
}
