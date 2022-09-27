#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Rinex2_1_EditedObsFile.hpp"

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		struct Rinex2_1_EditedSdObsLine
		{
			int                        nObsTime;       // 观测历元序号, 方便POD索引 (20070722)
			BYTE                       Id;             // 卫星(或测站)号
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
		
		typedef map<BYTE,    Rinex2_1_EditedSdObsLine> Rinex2_1_EditedSdObsSatMap;    // 不同卫星(或测站)观测数据列表
		typedef map<DayTime, Rinex2_1_EditedSdObsLine> Rinex2_1_EditedSdObsEpochMap;  // 单个卫星的不同时刻观测数据列表

		// 编辑后观测数据卫星(或测站)结构
		struct Rinex2_1_EditedSdObsSat
		{
			BYTE                         Id;
			Rinex2_1_EditedSdObsEpochMap editedObs; 
		};

		// 编辑后LEO观测数据历元结构
		struct Rinex2_1_LeoEditedSdObsEpoch
		{
			DayTime                    t;
			BYTE                       byEpochFlag;
			BYTE                       bySatCount;
			BYTE                       A_byRAIMFlag;
			double                     A_pdop;
			POS3D                      A_pos;
			POS3D                      A_vel;
			double                     A_clock;
			BYTE                       B_byRAIMFlag;
			double                     B_pdop;
			POS3D                      B_pos;
			POS3D                      B_vel;
			double                     B_clock;
			Rinex2_1_EditedSdObsSatMap editedObs; 
		};

		class LeoNetPODEditedSdObsFile
		{
		public:
			LeoNetPODEditedSdObsFile(void);
		public:
			~LeoNetPODEditedSdObsFile(void);
		public:
			void        clear();
			bool        isEmpty();
			bool        cutdata(DayTime t_Begin,DayTime t_End);
			int         isValidEpochLine(string strLine, FILE * pEditedSdObsFile = NULL);
			bool        open(string  strEditedSdObsFileName);
			bool        write(string strEditedSdObsFileName);
			bool        getEditedObsEpochList(vector<Rinex2_1_LeoEditedSdObsEpoch>& editedObsEpochlist); 
			bool        getEditedObsEpochList(vector<Rinex2_1_LeoEditedSdObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End); 
			bool        getEditedObsSatList(vector<Rinex2_1_EditedSdObsSat>& editedObsSatlist);
 			bool        getEditedObsSatList(vector<Rinex2_1_EditedSdObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End);
			static bool getEditedObsSatList(vector<Rinex2_1_LeoEditedSdObsEpoch>& editedObsEpochlist, vector<Rinex2_1_EditedSdObsSat>& editedObsSatlist);
		public:
			Rinex2_1_ObsHeader                    m_header;
			vector<Rinex2_1_LeoEditedSdObsEpoch>  m_data;
		};
	}
}
