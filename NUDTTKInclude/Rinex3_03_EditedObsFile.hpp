#pragma once
#include "structDef.hpp"
#include <vector>
#include <limits>
#include <windows.h>
#include <map>
#include "Rinex3_03_ObsFile.hpp"
#include "Rinex2_1_EditedObsFile.hpp"

//  Copyright 2018, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct Rinex3_03_EditedObsLine
	{
		int     nObsTime;           // 观测历元序号, 方便POD索引 (20070722)
		string  satName;            // 卫星名称, 2014/12/01, 针对Mixed格式进行调整
		float   Elevation;
		float   Azimuth;
		float   gmfh;               // + 对流层干分量映射函数（偏导数）
		float   gmfw;               // + 对流层湿分量映射函数（偏导数），用于对流层湿分量估计
		BYTE    mark_GNSSSatShadow; // + 进入地影标记, 默认 0-未进入地影, 1-进入地影，2-星历缺失或其它?
		double  ReservedField;      // 保留字段, 暂不使用, 为了编写算法使用保留
		Rinex2_1_EditedObsTypeList obsTypeList; // 不同类型观测数据列表
		
		Rinex3_03_EditedObsLine()
		{
			nObsTime  = INT_MAX;
			satName   = "";
			Azimuth   = 0.0f;
			Elevation = 0.0f;
			gmfh      = 0.0f;
			gmfw      = 0.0f;
			mark_GNSSSatShadow = 2;
			ReservedField = DBL_MAX;
		}

		Rinex3_03_EditedObsLine(Rinex3_03_SatMap::iterator it)
		{
			nObsTime  = INT_MAX;
			Azimuth   = 0.0f;
			Elevation = 0.0f;
			gmfh      = 0.0f;
			gmfw      = 0.0f;
			mark_GNSSSatShadow = 2;
			satName   = it->first;
			obsTypeList.clear();				
			for(Rinex2_1_ObsTypeList::iterator jt = it->second.begin(); jt != it->second.end(); ++jt)
			{
				obsTypeList.push_back(Rinex2_1_EditedObsDatum(jt));
			}
		}
	};

	typedef map<string,  Rinex3_03_EditedObsLine> Rinex3_03_EditedObsSatMap;    // 不同卫星观测数据列表
	typedef map<DayTime, Rinex3_03_EditedObsLine> Rinex3_03_EditedObsEpochMap;  // 单个卫星的不同时刻观测数据列表

	struct Rinex3_03_SysEditedObs                                               //  单个时刻的某一卫星系统的观测数据
	{
		char                      cSatSys; // 卫星系统
		Rinex3_03_EditedObsSatMap obsList;      // 观测数据
	};

	// 编辑后观测数据历元结构
	struct Rinex3_03_EditedObsEpoch 
	{
		char                           cRecordId[1+1];          // +数据记录标记
		DayTime                        t;
		BYTE                           byEpochFlag;             // +数据质量标记，0:OK，1:断电，>1:特殊事件
		int                            satCount;
		double                         clock;                   // 测站钟差, 单位: 秒
		double                         tropZenithDelayPriori_H; // 测站对流层干分量天顶延迟先验值
        double                         tropZenithDelayPriori_W; // 测站对流层湿分量天顶延迟先验值
		double                         tropZenithDelayEst;      // 测站对流层湿分量天顶延迟估计值
		double                         tropGradNSEst;           // +测站对流层南北梯度参数
		double                         tropGradEWEst;           // +测站对流层东西梯度参数
		double                         temperature;             // 测站温度
		double                         humidity;                // 测站湿度
		double                         pressure;                // 测站压强

		vector<Rinex3_03_SysEditedObs> editedObs;               // 针对3.03格式进行调整
		
		Rinex3_03_EditedObsEpoch()
		{
			clock                   = 0.0; 
			tropZenithDelayPriori_H = 0.0; 
			tropZenithDelayPriori_W = 0.0;   
			tropZenithDelayEst      = 0.0; 
			tropGradNSEst           = 0.0;
			tropGradEWEst           = 0.0;
			temperature             = 0.0;             
			humidity                = 0.0;                
			pressure                = 0.0;                
		}
		
		void load(Rinex3_03_ObsEpoch obsEpoch)
		{
			// 前5项保持与obsEpoch相同
			cRecordId[0] = obsEpoch.cRecordId[0];
			cRecordId[1] = '\0';
			t            = obsEpoch.t;
			byEpochFlag  = obsEpoch.byEpochFlag;
			satCount     = obsEpoch.satCount;
			clock        = obsEpoch.clock; // 单位: 秒,与obs统一
			// 遍历每颗可视GNSS卫星观测数据
			editedObs.resize(obsEpoch.obs.size());
			for(size_t s_i = 0; s_i < obsEpoch.obs.size(); s_i++)
			{
				editedObs[s_i].cSatSys = obsEpoch.obs[s_i].cSatSys;
				editedObs[s_i].obsList.clear();
				for(Rinex3_03_SatMap::iterator it = obsEpoch.obs[s_i].obsList.begin(); it != obsEpoch.obs[s_i].obsList.end(); ++it)
					editedObs[s_i].obsList.insert(Rinex3_03_EditedObsSatMap::value_type(it->first, Rinex3_03_EditedObsLine(it))); 
			}
		}
	};

	// 编辑后观测数据卫星(或测站)结构
	struct Rinex3_03_EditedObsSat 
	{
		string                            satName;      
		Rinex3_03_EditedObsEpochMap       editedObs; 
	};

	class Rinex3_03_EditedObsFile
	{
	public:
		Rinex3_03_EditedObsFile(void);
	public:
		~Rinex3_03_EditedObsFile(void);
	public:
		void        clear();
		bool        isEmpty();
		bool        cutdata(DayTime t_Begin,DayTime t_End);
		int         isValidEpochLine(string strLine, FILE * pEditedObsfile = NULL);
		bool        open(string  strEditedObsfileName);
		bool        write(string strEditedObsfileName);
		bool        getEditedObsEpochList(vector<Rinex3_03_EditedObsEpoch>& editedObsEpochlist); 
		int        getEditedObsEpochList(vector<Rinex3_03_EditedObsEpoch>& editedObsEpochlist, char cSatSys); 
        bool        getEditedObsEpochList(vector<Rinex3_03_EditedObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End); 
	    bool        getEditedObsSatList(vector<Rinex3_03_EditedObsSat>& editedObsSatlist);
		bool        getEditedObsSatList(vector<Rinex3_03_EditedObsSat>& editedObsSatlist, char cSatSys);
 		bool        getEditedObsSatList(vector<Rinex3_03_EditedObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End); 
		static bool getEditedObsSatList(vector<Rinex3_03_EditedObsEpoch> editedObsEpochlist, vector<Rinex3_03_EditedObsSat>& editedObsSatlist);
		bool        datalist_sat2epoch(vector<Rinex3_03_EditedObsSat> editedObsSatlist, vector<Rinex3_03_EditedObsEpoch>& editedObsEpochlist);
	public:
		Rinex3_03_ObsHeader              m_header;
		vector<Rinex3_03_EditedObsEpoch> m_data;
	};
}
