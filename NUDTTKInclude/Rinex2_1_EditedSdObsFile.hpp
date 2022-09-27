#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Rinex2_1_EditedObsFile.hpp"
using namespace NUDTTK;

namespace NUDTTK
{
	struct Rinex2_1_EditedSdObsLine
	{
		int                        nObsTime;       // 观测历元序号, 方便POD索引 (20070722)
		BYTE                       Id;             // 卫星号
		double                     Azimuth_A;      // A观测方位角, 长基线情况下会有区别
		double                     Elevation_A;    // A观测高度角, 长基线情况下会有区别
		double                     Azimuth_B;      // B观测方位角, 长基线情况下会有区别
		double                     Elevation_B;    // B观测高度角, 长基线情况下会有区别
		double                     ReservedField;  // 保留字段, 暂不使用, 为了编写算法使用保留
		Rinex2_1_EditedObsTypeList obsTypeList;    // 不同类型观测数据列表
		
		Rinex2_1_EditedSdObsLine()
		{
			nObsTime    = INT_MAX;
			Azimuth_A   = DBL_MAX;
			Elevation_A = DBL_MAX;
			Azimuth_B   = DBL_MAX;
			Elevation_B = DBL_MAX;
			ReservedField = DBL_MAX;
		}
	};
	typedef map<BYTE,    Rinex2_1_EditedSdObsLine> Rinex2_1_EditedSdObsSatMap;    // 不同卫星观测数据列表
	typedef map<DayTime, Rinex2_1_EditedSdObsLine> Rinex2_1_EditedSdObsEpochMap;  // 单个卫星的不同时刻观测数据列表

	// 编辑后观测数据卫星结构
	struct Rinex2_1_EditedSdObsSat
	{
		BYTE                         Id;
		Rinex2_1_EditedSdObsEpochMap editedObs; 
	};
	// 编辑后观测数据历元结构
	struct Rinex2_1_EditedSdObsEpoch
	{
		DayTime                    t;
		BYTE                       byEpochFlag;
		BYTE                       bySatCount;
		double                     A_clock;                   // A测站钟差, 单位: 米
		double                     A_tropZenithDelayPriori_H; // A测站对流层干分量天顶延迟先验值
        double                     A_tropZenithDelayPriori_W; // A测站对流层湿分量天顶延迟先验值
		double                     A_tropZenithDelayEstimate; // A测站对流层湿分量天顶延迟估计值
		double                     A_temperature;             // A测站温度
		double                     A_humidity;                // A测站湿度
		double                     A_pressure;                // A测站压强
		double                     B_clock;                   // B测站钟差, 单位: 米
		double                     B_tropZenithDelayPriori_H; // B测站对流层干分量天顶延迟先验值
        double                     B_tropZenithDelayPriori_W; // B测站对流层湿分量天顶延迟先验值
		double                     B_tropZenithDelayEstimate; // B测站对流层湿分量天顶延迟估计值
		double                     B_temperature;             // B测站温度
		double                     B_humidity;                // B测站湿度
		double                     B_pressure;                // B测站压强		
		Rinex2_1_EditedSdObsSatMap editedObs; 
	};
	class Rinex2_1_EditedSdObsFile
	{
	public:
		Rinex2_1_EditedSdObsFile(void);
	public:
		~Rinex2_1_EditedSdObsFile(void);
	public:
		void        clear();
		bool        isEmpty();
		bool        cutdata(DayTime t_Begin,DayTime t_End);
		int         isValidEpochLine(string strLine, FILE * pEditedSdObsFile = NULL);
		bool        open(string  strEditedSdObsFileName);
		bool        write(string strEditedSdObsFileName);
		bool        getEditedObsEpochList(vector<Rinex2_1_EditedSdObsEpoch>& editedObsEpochlist); 
		bool        getEditedObsEpochList(vector<Rinex2_1_EditedSdObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End); 
		bool        getEditedObsSatList(vector<Rinex2_1_EditedSdObsSat>& editedObsSatlist);
		bool        getEditedObsSatList(vector<Rinex2_1_EditedSdObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End);
		static bool getEditedObsSatList(vector<Rinex2_1_EditedSdObsEpoch>& editedObsEpochlist, vector<Rinex2_1_EditedSdObsSat>& editedObsSatlist);
	public:
		Rinex2_1_ObsHeader                    m_header;
		vector<Rinex2_1_EditedSdObsEpoch>     m_data;
	};
}

