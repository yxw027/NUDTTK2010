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
namespace NUDTTK
{
	namespace SpaceborneGPSPreproc
	{	

		// 频率1和2在处理时主用, 频率3在进行三频处理时才使用
		struct PRE_Freq
		{
			double freq_L1;
			double freq_L2;
			double freq_L3;
		};
		// 系统结构
		struct HeoGNSSPRE_MixedSys
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
			PRE_Freq              freqSys;             // GLONASS的频率信息存储在freqSatList中, 程序里自动赋值

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
			map<string, PRE_Freq> freqSatList; // 主要针对GLONASS，string类型为卫星名

			HeoGNSSPRE_MixedSys()
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

		struct HeoGNSSObsPreprocDefine
		{
			double       max_ionosphere;           // 电离层延迟组合最大值，用于伪距野值剔除
	        double       min_ionosphere;           // 电离层延迟组合最小值
			double       min_elevation;            // 最低观测仰角,度
			unsigned int min_arcpointcount;        // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			double       max_arclengh;             // 相邻连续弧段间隔秒数阈值, 选择依据是最大连续跟踪弧段的时间, 一般不会超过半个轨道周期(秒)
			double       threshold_slipsize_mw;    // melbourne wuebbena 组合量大周跳的探测阈值(周)
			double       threshold_rms_mw;         // melbourne wuebbena 组合量的均方根阈值, 以宽巷周跳为单位
			double       threshold_slipsize_P1_L1; // P1 - L1 组合量大周跳的探测阈值(米)
			double       threshold_rms_P1_L1;
			double       vondrak_PIF_eps;          // vondrak 拟合参数
			double       vondrak_PIF_max;          // vondrak 拟合参数
			double       vondrak_PIF_min;          // vondrak 拟合参数
			unsigned int vondrak_PIF_width;        // vondrak 拟合参数
			double       threshold_gap;            // 弧段内的中断间隔阈值, 控制不要出现较大的中断(秒)
			double       max_pdop;                 // 几何精度因子阈值, 超过该值的观测点将不参与运算
			double       threshold_res_raim;       // RAIM 检验残差阈值
			int			  threshold_gpssatcount;
			double       threshold_editrms_code;   // 伪码残差阈值
			double       threshold_editrms_phase;  // 相位残差阈值
			double       threshold_ClockJumpSize;  // 钟跳大小阈值, 单位: 米
			bool			bOn_ClockEliminate;       // SY1卫星的数据处理需要补偿钟差
			bool         bOn_ClockJumpDiagnose;    // SY1卫星钟跳识别开关
			bool         bOn_L2SNRLostDiagnose;    // 诊断L2失锁现象
			bool         bOn_SlipEditedMWDiagnose; // 验证相位和伪码的匹配性, 主要是针对sy1号卫星, 2009/12/23 
			bool         bOn_IonosphereFree;       // 是否用消电离层组合探测周跳
			bool         bOn_POutlierAccordingL;   // 如果相位为野值, 则判定伪码也是野值
			bool         bOn_RaimSPP;              // RAIM检查-单点
			bool         bOn_RaimArcChannelBias;   // RAIM检查-弧段
			bool         bOn_Slip_L1_L2;           // 对于 sy1 这种轨道高度较高的卫星可以考虑1增加 L1-L2 方法, CHAMP 和 GRACE 卫星轨道高度太低, 不可以
			bool         bOn_GNSSSAT_AntPCOPCV;    // 导航天线相位中心
			bool         bOn_PhaseWindUp;          // 相位缠绕  
			int			typ_BDSobs_L1;            // 北斗第一个频点的观测数据类型 
			int			typ_BDSobs_L2;            // 北斗第二个频点的观测数据类型

			//解算开关
			bool    bOn_GNSSSAT_Clock;
			bool	   bOn_GNSSSAT_Relativity;

			 
			HeoGNSSObsPreprocDefine()
			{
				max_ionosphere						= 30.0 + 5;
		        min_ionosphere						= 0.0 - 5;
				min_elevation							= 5.0;
				min_arcpointcount					= 10;
				max_arclengh							= 2000.0;
				threshold_slipsize_mw				= 3.0; // 此处主要依靠编辑, 这里可以取大一些
				threshold_rms_mw					= 1.0;
				threshold_slipsize_P1_L1			= 10;  // 只能探测大周跳, 单位 m
				threshold_rms_P1_L1				=  5;
				vondrak_PIF_eps						= 1.0E-10;
				vondrak_PIF_max						= 1.5;
				vondrak_PIF_min						= 0.3;
				vondrak_PIF_width					= 30;
				threshold_gap							= 300;
				max_pdop								= 8.0;
				threshold_res_raim					= 3.5;
				threshold_gpssatcount				= 3;      
				threshold_editrms_code			= 2.50;
			    threshold_editrms_phase			= 0.10;
				bOn_ClockEliminate					= false;
				bOn_ClockJumpDiagnose		= false;
				bOn_SlipEditedMWDiagnose	= false;
				bOn_IonosphereFree				= true;
				bOn_POutlierAccordingL			= false;
				bOn_RaimSPP							= true;
				bOn_RaimArcChannelBias    = true;
				bOn_Slip_L1_L2					= false;
				bOn_GNSSSAT_AntPCOPCV = true;
				bOn_PhaseWindUp				= true;
				typ_BDSobs_L1						= TYPE_OBS_L1;
				typ_BDSobs_L2						= TYPE_OBS_L5;		

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

		class HeoGNSSObsPreproc
		{
		public:
			HeoGNSSObsPreproc(void);
		public:
			~HeoGNSSObsPreproc(void);
		public:
			void    setPreprocPath(string strPreprocPath);
			void    setSP3File(SP3File sp3File); 
			void    setCLKFile(CLKFile clkFile); 
			void    setObsFile(Rinex2_1_ObsFile obsFile);
			bool    loadSP3File(string  strSp3FileName);
			bool    loadCLKFile(string  strCLKFileName);
			bool    loadCLKFile_rinex304(string  strCLKFileName);
			bool    loadObsFile(string  strObsFileName);
			bool    loadMixedObsFile(string  strObsFileName);
			void    setAntPhaseCenterOffset(POS3D posRTN);
			void    setAntPhaseCenterOffset(POS3D posBody, Matrix matAxisBody2RTN);
			BYTE    obsPreprocInfo2EditedMark1(int obsPreprocInfo);
			BYTE    obsPreprocInfo2EditedMark2(int obsPreprocInfo);
            // TQ高轨卫星数据处理
			void    setHeoOrbitList(vector<TimePosVel> heoOrbitList);
			bool    getHeoOrbitPosVel(GPST t, TimePosVel& orbit, unsigned int nLagrange = 8);

			// sk 添加
			bool    mainFuncHeoGNSSObsEdit_new(Rinex2_1_LeoMixedEditedObsFile  &mixedEditedObsFile);	// 
			bool    mixedObsSPP(Rinex2_1_LeoMixedEditedObsEpoch obsEpoch, POSCLK& posclk, int& eyeableSatCount, double& pdop, double& rms_res, double threshold = 1.0E-003);


			bool    pdopMixedObsSPP(int index_P1_GPS, int index_P2_GPS, int index_P1_BDS, int index_P2_BDS, Rinex2_1_LeoMixedEditedObsEpoch obsEpoch, int& eyeableSatCount, double& pdop);
			bool    mainFuncTQObs2Edt(Rinex2_1_LeoEditedObsFile  &editedObsFile,int type_obs_L1 = TYPE_OBS_L1,int type_obs_L2 = TYPE_OBS_L2);
			bool    mainFuncHeoGNSSObsEdit(string  strMixedObsFileName,  Rinex2_1_LeoMixedEditedObsFile  &mixedEditedObsFile);	
			bool    TQSinglePointPositioning_PIF(char cSatSystem, int index_P1, int index_P2,double frequence_L1,double frequence_L2, Rinex2_1_LeoEditedObsEpoch obsEpoch, POSCLK& posclk, int& eyeableGPSCount, double& pdop, double& rms_res, double threshold = 1.0E-002);
            bool    TQMixedSinglePointPositioning_PIF(int index_P1_GPS, int index_P2_GPS, int index_P1_BDS, int index_P2_BDS, Rinex2_1_LeoMixedEditedObsEpoch obsEpoch, POSCLK& posclk, int& eyeableSatCount, double& pdop, double& rms_res, double threshold = 1.0E-002);
		private:
			bool    getEditedObsEpochList(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist); 
			bool    getEditedObsEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist); // +
			bool    getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
			bool    datalist_epoch2sat(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist, vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
		    bool    datalist_sat2epoch(vector<Rinex2_1_EditedObsSat>& editedObsSatlist, vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist);
			vector<Rinex2_1_LeoEditedObsEpoch>  m_editedObsEpochList;
			string  m_strPreprocPath;
		public:
			HeoGNSSObsPreprocDefine		m_PreprocessorDefine;
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
			vector<HeoGNSSPRE_MixedSys> m_preMixedSysList;
			vector<spp_results>  SPP_resultsList;
		};
	}
}