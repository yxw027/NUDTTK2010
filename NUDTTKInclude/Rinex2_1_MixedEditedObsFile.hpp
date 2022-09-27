#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Rinex2_1_MixedObsFile.hpp"
#include "Rinex2_1_EditedObsFile.hpp"

using namespace NUDTTK;
namespace NUDTTK
{
    struct Rinex2_1_MixedEditedObsLine
	{
		int                        nObsTime;       // 观测历元序号, 方便POD索引 (20070722)
		string                     satName;        // 卫星名称, 2014/12/01, 针对Mixed格式进行调整
        double                     Azimuth;        // 观测方位角
        double                     Elevation;      // 观测高度角
		double                     ReservedField;  // 保留字段, 暂不使用, 为了编写算法使用保留
		Rinex2_1_EditedObsTypeList obsTypeList;    // 不同类型观测数据列表
		
		Rinex2_1_MixedEditedObsLine()
		{
			nObsTime  = INT_MAX;
			Azimuth   = DBL_MAX;
			Elevation = DBL_MAX;
			ReservedField = DBL_MAX;
		}

		Rinex2_1_MixedEditedObsLine(Rinex2_1_MixedSatMap::iterator it)
		{
			nObsTime  = INT_MAX;
			Azimuth   = 0.0;
			Elevation = 0.0;
			satName   = it->first;
			obsTypeList.clear();				
			for(Rinex2_1_ObsTypeList::iterator jt = it->second.begin(); jt != it->second.end(); ++jt)
			{
				obsTypeList.push_back(Rinex2_1_EditedObsDatum(jt));
			}
		}
	};
	
	typedef map<string,  Rinex2_1_MixedEditedObsLine> Rinex2_1_MixedEditedObsSatMap;    // 不同卫星观测数据列表, 2014/12/01, 针对Mixed格式进行调整
	typedef map<DayTime, Rinex2_1_MixedEditedObsLine> Rinex2_1_MixedEditedObsEpochMap;  // 单个卫星的不同时刻观测数据列表, 2014/12/01, 针对Mixed格式进行调整

	// 编辑后观测数据历元结构
	struct Rinex2_1_MixedEditedObsEpoch 
	{
		DayTime                       t;
		BYTE                          byEpochFlag;
		BYTE                          bySatCount;
		double                        clock;                   // 测站钟差, 单位: 米
		double                        tropZenithDelayPriori_H; // 测站对流层干分量天顶延迟先验值
        double                        tropZenithDelayPriori_W; // 测站对流层湿分量天顶延迟先验值
		double                        tropZenithDelayEstimate; // 测站对流层湿分量天顶延迟估计值
		double                        temperature;             // 测站温度
		double                        humidity;                // 测站湿度
		double                        pressure;                // 测站压强
		Rinex2_1_MixedEditedObsSatMap editedObs;               // 2014/12/01, 针对Mixed格式进行调整 

		Rinex2_1_MixedEditedObsEpoch()
		{
			clock = 0; 
			tropZenithDelayPriori_H = 0; 
			tropZenithDelayPriori_W = 0;   
			tropZenithDelayEstimate = 0; 
			temperature = 0;             
			humidity = 0;                
			pressure = 0;                
		}
		
		void load(Rinex2_1_MixedObsEpoch obsEpoch)
		{
			// 前2项与 obs 保持相同
			t = obsEpoch.t;
			byEpochFlag = obsEpoch.byEpochFlag;
			bySatCount = obsEpoch.bySatCount;
			// 遍历每颗可视GPS卫星的观测数据
			editedObs.clear();
			for(Rinex2_1_MixedSatMap::iterator it = obsEpoch.obs.begin(); it != obsEpoch.obs.end(); ++it)
			{
				editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(it->first, Rinex2_1_MixedEditedObsLine(it))); // 2014/12/01, 针对Mixed格式进行调整
			}
		}
	};

	//编辑后双频观测数据类型预定义结构
	struct MixObsDualReqPreDefine
	{
		int          typ_GPSobs_L1;            // GPS第一个频点的观测数据类型 
		int          typ_GPSobs_L2;            // GPS第二个频点的观测数据类型
		int          typ_BDSobs_L1;            // 北斗第一个频点的观测数据类型 
		int          typ_BDSobs_L2;            // 北斗第二个频点的观测数据类型
		int          typ_GALobs_L1;            // Galileo第一个频点的观测数据类型 
		int          typ_GALobs_L2;            // Galileo第二个频点的观测数据类型

		MixObsDualReqPreDefine()
		{
			typ_GPSobs_L1            = TYPE_OBS_L1;
			typ_GPSobs_L2            = TYPE_OBS_L2;
			typ_BDSobs_L1            = TYPE_OBS_L1;
			typ_BDSobs_L2            = TYPE_OBS_L2;
			typ_GALobs_L1            = TYPE_OBS_L1;
			typ_GALobs_L2            = TYPE_OBS_L2;
		}
	};

	// 编辑后观测数据卫星(或测站)结构
	struct Rinex2_1_MixedEditedObsSat 
	{
		string                          satName;      // 2014/12/01, 针对Mixed格式进行调整
		Rinex2_1_MixedEditedObsEpochMap editedObs; 
	};

	class Rinex2_1_MixedEditedObsFile
	{
	public:
		Rinex2_1_MixedEditedObsFile(void);
	public:
		~Rinex2_1_MixedEditedObsFile(void);
	public:
		void        clear();
		bool        isEmpty();
		bool        cutdata(DayTime t_Begin,DayTime t_End);
		int         isValidEpochLine(string strLine, FILE * pEditedObsfile = NULL);
		bool        open(string  strEditedObsfileName, string strSystem = "G+C");
		bool        write(string strEditedObsfileName);
		bool        getEditedObsEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist); 
        bool        getEditedObsEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End); 
	    bool        getEditedObsSatList(vector<Rinex2_1_MixedEditedObsSat>& editedObsSatlist);
 		bool        getEditedObsSatList(vector<Rinex2_1_MixedEditedObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End); 
		static bool getEditedObsSatList(vector<Rinex2_1_MixedEditedObsEpoch> editedObsEpochlist, vector<Rinex2_1_MixedEditedObsSat>& editedObsSatlist);
		bool        datalist_sat2epoch(vector<Rinex2_1_MixedEditedObsSat> &editedObsSatlist, vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist);
		//为处理多系统观测数据添加
		bool        mainFunc_editedMixObsFile(string strEditedMixObsFileName, string systemFlag, Rinex2_1_EditedObsFile &editedObsFile_gps, Rinex2_1_EditedObsFile &editedObsFile_bds, char RecType_gps = 'N'); //系统扩展后需要作相应的更改  昌虓  2016/12/16
		bool        getEditedObsEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist, char cSys);
		static bool multiGNSS2SingleSysEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& mixedEditedObsEpochlist, vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist, char cSystem = 'G');
	public:
		MixObsDualReqPreDefine                   m_DualFreqPreDefine;
		Rinex2_1_MixedObsHeader                  m_header;
		vector<Rinex2_1_MixedEditedObsEpoch>     m_data;
	};
}

