#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Rinex2_1_ObsFile.hpp"

using namespace NUDTTK;
namespace NUDTTK
{
	struct Rinex2_1_EditedObsDatum
	{
		Rinex2_1_ObsDatum obs;           // 编辑后观测数据, 周跳点相位数据可以进行修复, 其他保持不变
		BYTE              byEditedMark1; // 编辑后标记1，正常点标记、野值标记、周跳标记
		BYTE              byEditedMark2; // 编辑后标记2，野值和周跳的分类信息
		
		Rinex2_1_EditedObsDatum()
		{
			byEditedMark1 = TYPE_EDITEDMARK_UNKNOWN;
			byEditedMark2 = 0;
		}

		Rinex2_1_EditedObsDatum(Rinex2_1_ObsDatum obsDatum)
		{
			byEditedMark1 = TYPE_EDITEDMARK_UNKNOWN;
			byEditedMark2 = 0;
			obs           = obsDatum;
		}

		Rinex2_1_EditedObsDatum(Rinex2_1_ObsTypeList::iterator it)
		{
			byEditedMark1 = TYPE_EDITEDMARK_UNKNOWN;
			byEditedMark2 = 0;
			obs.data      = it->data;
			obs.lli       = it->lli;
			obs.ssi       = it->ssi;
		}
	};

	typedef vector<Rinex2_1_EditedObsDatum> Rinex2_1_EditedObsTypeList;  // 不同类型观测数据序列
	
    struct Rinex2_1_EditedObsLine
	{
		int                        nObsTime;       // 观测历元序号, 方便POD索引 (20070722)
		BYTE                       Id;             // 卫星号
        double                     Azimuth;        // 观测方位角
        double                     Elevation;      // 观测高度角
		double                     ReservedField;  // 保留字段, 暂不使用, 为了编写算法使用保留
		Rinex2_1_EditedObsTypeList obsTypeList;    // 不同类型观测数据列表
		
		Rinex2_1_EditedObsLine()
		{
			nObsTime  = INT_MAX;
			Azimuth   = DBL_MAX;
			Elevation = DBL_MAX;
			ReservedField = DBL_MAX;
		}

		Rinex2_1_EditedObsLine(Rinex2_1_SatMap::iterator it)
		{
			nObsTime  = INT_MAX;
			Azimuth   = 0.0;
			Elevation = 0.0;
			Id        = it->first;
			obsTypeList.clear();				
			for(Rinex2_1_ObsTypeList::iterator jt = it->second.begin(); jt != it->second.end(); ++jt)
			{
				obsTypeList.push_back(Rinex2_1_EditedObsDatum(jt));
			}
		}
	};
	
	typedef map<BYTE,    Rinex2_1_EditedObsLine> Rinex2_1_EditedObsSatMap;    // 不同卫星观测数据列表
	typedef map<DayTime, Rinex2_1_EditedObsLine> Rinex2_1_EditedObsEpochMap;  // 单个卫星的不同时刻观测数据列表

	// 编辑后观测数据历元结构
	struct Rinex2_1_EditedObsEpoch
	{
		DayTime                  t;
		BYTE                     byEpochFlag;
		BYTE                     bySatCount;
		double                   clock;                   // 测站钟差, 单位: 米
		double                   tropZenithDelayPriori_H; // 测站对流层干分量天顶延迟先验值
        double                   tropZenithDelayPriori_W; // 测站对流层湿分量天顶延迟先验值
		double                   tropZenithDelayEstimate; // 测站对流层湿分量天顶延迟估计值
		double                   temperature;             // 测站温度
		double                   humidity;                // 测站湿度
		double                   pressure;                // 测站压强
		Rinex2_1_EditedObsSatMap editedObs; 

		Rinex2_1_EditedObsEpoch()
		{
			clock = 0; 
			tropZenithDelayPriori_H = 0; 
			tropZenithDelayPriori_W = 0;   
			tropZenithDelayEstimate = 0; 
			temperature = 0;             
			humidity = 0;                
			pressure = 0;                
		}
		
		void load(Rinex2_1_ObsEpoch obsEpoch)
		{
			// 前2项与 obs 保持相同
			t = obsEpoch.t;
			byEpochFlag = obsEpoch.byEpochFlag;
			bySatCount = obsEpoch.bySatCount;
			// 遍历每颗可视GPS卫星的观测数据
			editedObs.clear();
			for(Rinex2_1_SatMap::iterator it = obsEpoch.obs.begin(); it != obsEpoch.obs.end(); ++it)
			{
				editedObs.insert(Rinex2_1_EditedObsSatMap::value_type(it->first, Rinex2_1_EditedObsLine(it)));
			}
		}
	};

	// 编辑后观测数据卫星(或测站)结构
	struct Rinex2_1_EditedObsSat
	{
		BYTE                       Id;
		Rinex2_1_EditedObsEpochMap editedObs; 
	};

	class Rinex2_1_EditedObsFile
	{
	public:
		Rinex2_1_EditedObsFile(void);
	public:
		~Rinex2_1_EditedObsFile(void);
	public:
		void        clear();
		bool        isEmpty();
		bool        cutdata(DayTime t_Begin,DayTime t_End);
		int         isValidEpochLine(string strLine, FILE * pEditedObsfile = NULL);
		bool        open(string  strEditedObsfileName);
		bool        write(string strEditedObsfileName);
		bool        write_4Obs(string strEditedObsfileName, int Freq1 = 1, int Freq2 =2, char RecType = 'N');
		bool        write(string strEditedObsfileName, int condition);
		bool        getEditedObsEpochList(vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist); 
        bool        getEditedObsEpochList(vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End); 
	    bool        getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
 		bool        getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End); 
		static bool getEditedObsSatList(vector<Rinex2_1_EditedObsEpoch> editedObsEpochlist, vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
		bool        datalist_sat2epoch(vector<Rinex2_1_EditedObsSat> &editedObsSatlist, vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist);
	public:
		Rinex2_1_ObsHeader                  m_header;
		vector<Rinex2_1_EditedObsEpoch>     m_data;
	};
}

