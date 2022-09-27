#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Rinex2_1_LeoEditedObsFile.hpp"
#include "Rinex2_1_MixedEditedObsFile.hpp"

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SpaceborneGPSPreproc
	{
		// 编辑后LEO观测数据历元结构
		struct Rinex2_1_LeoMixedEditedObsEpoch
		{
			DayTime                  t;
			BYTE                     byEpochFlag;
			BYTE                     bySatCount;
            //=====与 Rinex2_1_EditedObsEpoch 定义不同的地方
			BYTE                     byRAIMFlag;
			double                   pdop;
			POS3D                    pos;
			POS3D                    vel;
			double                   clock;
            //==============================================
			Rinex2_1_MixedEditedObsSatMap editedObs; 

			Rinex2_1_LeoMixedEditedObsEpoch()
			{
				byRAIMFlag = 0;
				pos.x = 0;
				pos.y = 0;
				pos.z = 0;
				vel.x = 0;
				vel.y = 0;
				vel.z = 0;
				clock = 0;    
				pdop  = 0;               
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

		class Rinex2_1_LeoMixedEditedObsFile
		{
		public:
			Rinex2_1_LeoMixedEditedObsFile(void);
		public:
			~Rinex2_1_LeoMixedEditedObsFile(void);
		public:
			void        clear();
			bool        isEmpty();
			bool        cutdata(DayTime t_Begin,DayTime t_End);
			int         isValidEpochLine(string strLine, FILE * pEditedObsfile = NULL);
			bool        open(string  strEditedObsfileName, string strSystem = "G+C");
			bool        write(string strEditedObsfileName);
			bool        open_5Obs(string  strEditedObsfileName, string strSystem = "G+C");  // obs数据小数点后保留5位有效数字
			bool        write_5Obs(string strEditedObsfileName);                            // obs数据小数点后保留5位有效数字
			bool        getEditedObsEpochList(vector<Rinex2_1_LeoMixedEditedObsEpoch>& editedObsEpochlist); 
			bool        getEditedObsEpochList(vector<Rinex2_1_LeoMixedEditedObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End); 
			bool        getEditedObsSatList(vector<Rinex2_1_MixedEditedObsSat>& editedObsSatlist);
 			bool        getEditedObsSatList(vector<Rinex2_1_MixedEditedObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End); 
			static bool getEditedObsSatList(vector<Rinex2_1_LeoMixedEditedObsEpoch>& editedObsEpochlist, vector<Rinex2_1_MixedEditedObsSat>& editedObsSatlist);
			static bool mixedGNSS2SingleSysEpochList(vector<Rinex2_1_LeoMixedEditedObsEpoch>& mixedEditedObsEpochlist, vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist, char cSystem = 'G');
		public:
			Rinex2_1_MixedObsHeader                  m_header;
			vector<Rinex2_1_LeoMixedEditedObsEpoch>  m_data;
		};
	}
}
