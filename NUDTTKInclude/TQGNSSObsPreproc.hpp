#pragma once
#include "Rinex2_1_ObsFile.hpp"
#include "Rinex2_1_MixedObsFile.hpp"
#include "Rinex2_1_LeoMixedEditedObsFile.hpp"
#include "Rinex2_1_LeoEditedObsFile.hpp" 
#include "Rinex2_1_NavFile.hpp"
#include "Rinex3_03_EditedObsFile.hpp"
#include "MathAlgorithm.hpp"
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "svnavFile.hpp"
#include "igs05atxFile.hpp"
#include "Rinex2_1_LeoEditedObsFile.hpp"
#include "GNSSObsPreproc.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha

using namespace NUDTTK::SpaceborneGPSPreproc;
namespace NUDTTK
{
	namespace TQGNSSPod
	{	
		// 系统结构
		struct TQGNSSPRE_MixedSys
		{
			// 需要初始化部分
			char                  cSys;             // 系统标识
			string                name_C1;               // 伪码
			string                name_C2;
			string                name_L1;              // 相位
			string                name_L2;
		    string                name_S1;             // 信噪比 
			string                name_S2;
			string                nameFreq_L1;         // 频率名称"G01"，主要用于PCV修正
			string                nameFreq_L2;   

			double                                  freq_L1;
			double                                  freq_L2;
			// 与系统有关的预定义部分
			bool									bOn_GNSSSAT_AntPCOPCV;
			bool									bOn_GNSSSAT_Clk;
			bool									bOn_GNSSSAT_Relativity;

			//频率索引
			int					index_C1;
			int					index_C2;
			int					index_L1;
			int					index_L2;

			TQGNSSPRE_MixedSys()
			{
				name_C1  = "";
				name_C2  = "";
				name_L1  = "";
				name_L2  = "";
				name_S1  = "";
				name_S2  = "";
				nameFreq_L1 = "";
				nameFreq_L2 = "";
				freq_L1                = GPS_FREQUENCE_L1;
				freq_L2                = GPS_FREQUENCE_L2;
			}
		};

		struct TQGNSSObsPreprocDefine
		{
			double       max_ionosphere;           // 电离层延迟组合最大值，用于伪距野值剔除
	        double       min_ionosphere;           // 电离层延迟组合最小值
			double       min_elevation;            // 最低观测仰角,度
			unsigned int min_arcpointcount;        // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			double       max_arclengh;             // 相邻连续弧段间隔秒数阈值, 选择依据是最大连续跟踪弧段的时间, 一般不会超过半个轨道周期(秒)
			double       vondrak_PIF_eps;          // vondrak 拟合参数
			double       vondrak_PIF_max;          // vondrak 拟合参数
			double       vondrak_PIF_min;          // vondrak 拟合参数
			unsigned int vondrak_PIF_width;        // vondrak 拟合参数
			double       threshold_gap;            // 弧段内的中断间隔阈值, 控制不要出现较大的中断(秒)
			double       max_pdop;                 // 几何精度因子阈值, 超过该值的观测点将不参与运算
			int			 threshold_gpssatcount;
			double       threshold_editrms_code;   // 伪码残差阈值
			double       threshold_editrms_phase;  // 相位残差阈值
			bool         bOn_RaimSPP;              // RAIM检查-单点
			bool         bOn_RaimArcChannelBias;   // RAIM检查-弧段
			bool         bOn_GNSSSAT_AntPCOPCV;    // 导航天线相位中心
			bool         bOn_PhaseWindUp;          // 相位缠绕  
			bool         bOn_GNSSSAT_Clock;
			bool	     bOn_GNSSSAT_Relativity;
            bool         bOn_ClockEliminate;
	 
			TQGNSSObsPreprocDefine()
			{
				max_ionosphere				= 30.0 + 5;
		        min_ionosphere				= 0.0 - 5;
				min_elevation				= 5.0;
				min_arcpointcount			= 10;
				max_arclengh				= 2000.0;
				vondrak_PIF_eps				= 1.0E-10;
				vondrak_PIF_max				= 1.5;
				vondrak_PIF_min				= 0.3;
				vondrak_PIF_width			= 30;
				threshold_gap				= 300;
				max_pdop					= 8.0;
				threshold_gpssatcount		= 3;      
				threshold_editrms_code		= 2.50;
			    threshold_editrms_phase		= 0.10;
				bOn_RaimSPP					= true;
				bOn_RaimArcChannelBias      = true;
				bOn_GNSSSAT_AntPCOPCV       = true;
				bOn_PhaseWindUp				= true;	
				bOn_GNSSSAT_Clock			= true ;
				bOn_GNSSSAT_Relativity		= true;
			} 
		};

		struct spp_results
		{
			DayTime t;
			POSCLK   posclk;
			int eyeableGPSCount;
			double pdop;
			double rms_res;
		};

		class TQGNSSObsPreproc
		{
		public:
			TQGNSSObsPreproc(void);
		public:
			~TQGNSSObsPreproc(void);
		public:
			void    setPreprocPath(string strPreprocPath);
			void    setSP3File(SP3File sp3File); 
			void    setCLKFile(CLKFile clkFile); 
			void    setObsFile(Rinex2_1_ObsFile obsFile);
			bool    loadSP3File(string  strSp3FileName);
			bool    loadCLKFile(string  strCLKFileName);
			bool    loadCLKFile_rinex304(string  strCLKFileName);
			bool    loadMixedObsFile(string  strObsFileName);
			bool    loadMixedObsFile_5Obs(string  strObsFileName);
			void    setHeoOrbitList(vector<TimePosVel> heoOrbitList);
			bool    getHeoOrbitPosVel(GPST t, TimePosVel& orbit, unsigned int nLagrange = 8);

			// 编辑函数+定位函数，重新梳理，修改
			bool    mainFuncHeoGNSSObsEdit(Rinex2_1_LeoMixedEditedObsFile  &mixedEditedObsFile);	// 暂时主要功能为转换为EDT文件
			bool    mixedObsSPP(Rinex2_1_LeoMixedEditedObsEpoch obsEpoch, POSCLK& posclk, int& eyeableSatCount, double& pdop, double& rms_res, double threshold = 1.0E-003);
	        // mixedObsSPP 重点，定位，PDOP
		private:
			bool    getEditedObsEpochList(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist); 
			bool    getEditedObsEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist); // +
			bool    getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
			bool    datalist_epoch2sat(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist, vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
		    bool    datalist_sat2epoch(vector<Rinex2_1_EditedObsSat>& editedObsSatlist, vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist);
			vector<Rinex2_1_LeoEditedObsEpoch>  m_editedObsEpochList;
			string  m_strPreprocPath;
		public:
			TQGNSSObsPreprocDefine		m_PreprocessorDefine;
			CLKFile											m_clkFile; // 精密钟差数据文件
			SP3File											m_sp3File; // 精密星历数据文件
			Rinex2_1_ObsFile							    m_obsFile; // 单系统原始观测数据		
			Rinex2_1_MixedObsFile                           m_mixedObsFile; // 混合系统观测数据
			POS3D											m_pcoAnt;  // 天线偏心量		
			POS3D											m_posStation;		 // 测站位置坐标 
			svnavMixedFile							m_svnavMixedFile;    // +
			vector<TimePosVel>					m_heoOrbitList; // 动力学参考轨道
			int												m_countSlip;
			int												m_countRestSlip; // 保留为被删除掉的周跳
			Matrix											m_matAxisAnt2Body;  // 天线系到星固系的转换矩阵，用于天线可能安装在非天顶方向
			// 
			vector<TQGNSSPRE_MixedSys> m_preMixedSysList;
			vector<spp_results>  SPP_resultsList;
		};
	}
}