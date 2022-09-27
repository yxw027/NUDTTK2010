#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Rinex2_1_MixedObsFile.hpp"
#include "Rinex2_1_LeoEditedSdObsFile.hpp"
#include "SP3File.hpp"

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SpaceborneGPSProd
	{
		struct Rinex2_1_MixedEditedSdObsLine
		{
			int                        nObsTime;       // 观测历元序号, 方便POD索引 (20070722)
			string                     satName;        // 卫星名称, 2015/03/09, 针对Mixed格式进行调整
			double                     Azimuth_A;      // A观测方位角, 长基线情况下会有区别
			double                     Elevation_A;    // A观测高度角, 长基线情况下会有区别
			double                     Azimuth_B;      // B观测方位角, 长基线情况下会有区别
			double                     Elevation_B;    // B观测高度角, 长基线情况下会有区别
			double                     ReservedField;  // 保留字段, 暂不使用, 为了编写算法使用保留
			Rinex2_1_EditedObsTypeList obsTypeList;    // 不同类型观测数据列表
			
			Rinex2_1_MixedEditedSdObsLine()
			{
				nObsTime    = INT_MAX;
				Azimuth_A   = DBL_MAX;
				Elevation_A = DBL_MAX;
				Azimuth_B   = DBL_MAX;
				Elevation_B = DBL_MAX;
				ReservedField = DBL_MAX;
			}
		};
		
		typedef map<string,  Rinex2_1_MixedEditedSdObsLine> Rinex2_1_MixedEditedSdObsSatMap;    // 不同卫星(或测站)观测数据列表, 2015/03/09, 针对Mixed格式进行调整
		typedef map<DayTime, Rinex2_1_MixedEditedSdObsLine> Rinex2_1_MixedEditedSdObsEpochMap;  // 单个卫星的不同时刻观测数据列表, 2015/03/09, 针对Mixed格式进行调整

		// 编辑后观测数据卫星(或测站)结构
		struct Rinex2_1_MixedEditedSdObsSat
		{
			string                             satName;     //卫星名称, 2015/03/09, 针对Mixed格式进行调整
			Rinex2_1_MixedEditedSdObsEpochMap  editedObs;   //2015/03/09, 针对Mixed格式进行调整
		};

		// 编辑后LEO观测数据历元结构
		struct Rinex2_1_LeoMixedEditedSdObsEpoch
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
			Rinex2_1_MixedEditedSdObsSatMap editedObs; //2015/03/09, 针对Mixed格式进行调整
		};

		class Rinex2_1_LeoMixedEditedSdObsFile
		{
		public:
			Rinex2_1_LeoMixedEditedSdObsFile(void);
		public:
			~Rinex2_1_LeoMixedEditedSdObsFile(void);
		public:
			void        clear();
			bool        isEmpty();
			bool        cutdata(DayTime t_Begin,DayTime t_End);
			int         isValidEpochLine(string strLine, FILE * pEditedSdObsFile = NULL);
			bool        open(string  strMixedEditedSdObsFileName, string strSystem = "G+C");
			bool        write(string strMixedEditedSdObsFileName);
			bool        getEditedObsEpochList(vector<Rinex2_1_LeoMixedEditedSdObsEpoch>& editedObsEpochlist); 
			bool        getEditedObsEpochList(vector<Rinex2_1_LeoMixedEditedSdObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End); 
			bool        getEditedObsSatList(vector<Rinex2_1_MixedEditedSdObsSat>& editedObsSatlist);
 			bool        getEditedObsSatList(vector<Rinex2_1_MixedEditedSdObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End);
			static bool getEditedObsSatList(vector<Rinex2_1_LeoMixedEditedSdObsEpoch> editedObsEpochlist, vector<Rinex2_1_MixedEditedSdObsSat>& editedObsSatlist);
			static bool mixedGNSS2SingleSysEpochList(vector<Rinex2_1_LeoMixedEditedSdObsEpoch> mixedEditedSdObsEpochlist, vector<Rinex2_1_LeoEditedSdObsEpoch>& editedSdObsEpochlist, char cSystem = 'G');
		public:
			Rinex2_1_MixedObsHeader                    m_header;
			vector<Rinex2_1_LeoMixedEditedSdObsEpoch>  m_data;
		};
	}
}
