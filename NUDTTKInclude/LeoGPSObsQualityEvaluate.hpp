#pragma once
#include "structDef.hpp"
#include "Rinex2_1_LeoEditedObsFile.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	namespace SpaceborneGPSPreproc
	{
		struct QERESIDUAL
		{
			DayTime  t;
			double   Azimuth;    
			double   Elevation;
			double   obs;
			double   fit;
			double   res;
			double   CN0;  //信噪比
			double   CN0_P2;  //信噪比
			QERESIDUAL()
			{
				Azimuth   = 0;
				Elevation = 0;
				res       = 0;
				CN0       = 0;
				CN0_P2    = 0;
			}
		};

		struct QERESIDUAL_ARC
		{
			vector<QERESIDUAL> resList;
			double rms_arc;
		};

		struct QERESIDUAL_SAT
		{
			vector<QERESIDUAL_ARC> arcList;
			BYTE                   id_sat;
			double                 rms_sat;
		};

		typedef map<BYTE,    int> QEAbNormalObsCountMap; // 异常信息列表


		struct SLIPINFO_LEO
		{
			BYTE                   id_sat;
			BYTE                   byEditedMark2;
			GPST                   t;
		};

		struct QERESIDUAL_LEO
		{
			vector<QERESIDUAL_SAT> satInfoList_P1;
			vector<QERESIDUAL_SAT> satInfoList_P2;
			vector<QERESIDUAL_SAT> satInfoList_L;
			double                 rms_P1;
			double                 rms_P2;
			double                 rms_L;
			double                 ratio_P_normal;
			double                 ratio_L_normal;
			double                 ratio_SLip;
		    int                    count_SLip;     // 测试代码
			double		           mean_ObsSatNum; // 平均可视卫星数(QESatCounts)
			QEAbNormalObsCountMap  AbnormalObsCount;
			vector<SLIPINFO_LEO>   slipList;//周跳列表
		};

		struct LeoGPSObsQEDefine
		{
			bool         bOn_RobustCalculate;     // 是否使用鲁棒计算
			bool         bOn_MAD_Phase;           // 电离层闪烁的部分很难消除, 暂时通过MAD方法克服
			double       robustFactor;
			int          phaseQEType;             // 0- evaluate_phase_poly;  1- evaluate_phase_vondrak;  
			double       vondrak_LIF_eps;
			double       vondrak_LIF_max;
			double       vondrak_LIF_min;
			unsigned int vondrak_LIF_width;
			double       poly_halfwidth;
			int          poly_order;

			double       threhold_var_iono;      // 电离层变化阈值

			LeoGPSObsQEDefine()
			{
				bOn_RobustCalculate = false;
				bOn_MAD_Phase       = true; 
				robustFactor        = 4.0;
				phaseQEType         = 1;
				vondrak_LIF_eps     = 1.0E-8;
				vondrak_LIF_max     = 1.0;
				vondrak_LIF_min     = 0.05;
				vondrak_LIF_width   = 30;
				poly_halfwidth      = 60;
				poly_order          = 3;

				threhold_var_iono   = DBL_MAX; // 用于剔除少量拟合残差较大的弧段, 怀疑是由于电离层变化剧烈或不规则引起
			}
		};

		class LeoGPSObsQualityEvaluate
		{
		public:
			LeoGPSObsQualityEvaluate(void);
		public:
			~LeoGPSObsQualityEvaluate(void);
		public:
			LeoGPSObsQEDefine m_QEDefine;
		public:	
			void setOQEResPath(string strOQEResPath, string strSatName = "");
			bool mainFunc(string strEdtedObsFilePath, int flag = 0,int type_obs_L1 = TYPE_OBS_L1,int type_obs_L2 = TYPE_OBS_L2, char cSystem = 'G');
			string  m_strOQEResPath;
			string  m_strSatName;
		private:
			bool evaluate_code_multipath(vector<Rinex2_1_EditedObsSat>& editedObsSatlist, int index_P1, int index_P2, int index_L1, int index_L2, int index_S1, int index_S2, double frequence_L1 = GPS_FREQUENCE_L1,double frequence_L2 = GPS_FREQUENCE_L2);
			bool evaluate_phase_poly(vector<Rinex2_1_EditedObsSat>& editedObsSatlist,  int index_P1, int index_P2, int index_L1, int index_L2, int index_S1, int index_S2, double frequence_L1 = GPS_FREQUENCE_L1,double frequence_L2 = GPS_FREQUENCE_L2, double poly_halfwidth = 60, int poly_order = 3);
			bool evaluate_phase_vondrak(vector<Rinex2_1_EditedObsSat>& editedObsSatlist, int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1 = GPS_FREQUENCE_L1,double frequence_L2 = GPS_FREQUENCE_L2, double vondrak_LIF_eps = 1.0E-8, double vondrak_LIF_max = 1.0,double vondrak_LIF_min = 0.05,unsigned int vondrak_LIF_width = 30); // 1.0E-12
		public:
			QERESIDUAL_LEO m_QEInfo;
		};
	}
}
