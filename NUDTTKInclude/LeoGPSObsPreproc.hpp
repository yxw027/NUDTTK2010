#pragma once
#include "Rinex2_1_ObsFile.hpp"
#include "Rinex2_1_MixedObsFile.hpp"
#include "Rinex2_1_LeoMixedEditedObsFile.hpp"
#include "Rinex2_1_LeoEditedObsFile.hpp" 
#include "Rinex2_1_NavFile.hpp"
#include "MathAlgorithm.hpp"
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "svnavFile.hpp"
#include "igs05atxFile.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	namespace SpaceborneGPSPreproc
	{
		enum TYPE_LEOGPSOBSPREPROC_INFO
		{   
			LEOGPSOBSPREPROC_UNKNOWN              = 00,
			LEOGPSOBSPREPROC_NORMAL               = 10,
			// 野值类型
			LEOGPSOBSPREPROC_OUTLIER_BLANKZERO    = 20, // 观测数据缺失
			LEOGPSOBSPREPROC_OUTLIER_COUNT        = 21, // 观测个数太少标记不可用
			LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION = 22, // 信噪比+高度截止角
			LEOGPSOBSPREPROC_OUTLIER_IONOMAXMIN   = 23, // 电离层探测伪码野值
			LEOGPSOBSPREPROC_OUTLIER_MW           = 24, // MW组合探测野值
			LEOGPSOBSPREPROC_OUTLIER_VONDRAK      = 25, // Vondrak滤波超差标记为野值
			LEOGPSOBSPREPROC_OUTLIER_RAIM         = 26, // RAIM检验
			LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS    = 27, // 星历数据不足
			LEOGPSOBSPREPROC_OUTLIER_MWRMS        = 28, // MW无周跳区间标准差超差
			LEOGPSOBSPREPROC_OUTLIER_L1_L2        = 29, // L1 - L2探测野值  采样间隔的电离层历元差阈值 0.10
			// 周跳类型
			LEOGPSOBSPREPROC_NEWARCBEGIN          = 30,
			LEOGPSOBSPREPROC_SLIP_MW              = 31,//MW组合探测周跳
			LEOGPSOBSPREPROC_SLIP_IFAMB           = 32,//无电离层模糊度
			LEOGPSOBSPREPROC_SLIP_L1_L2           = 33 //无电离层组合探测周跳
		};

		enum TYPE_LEOGPSOBSEDIT_INFO
		{   
			LEOGPSOBSEDIT_UNKNOWN              = 00,
			LEOGPSOBSEDIT_NORMAL               = 10,
			// 野值类型
			LEOGPSOBSEDIT_OUTLIER_BLANKZERO    = 20,
			LEOGPSOBSEDIT_OUTLIER_COUNT        = 21, 
			LEOGPSOBSEDIT_OUTLIER_SNRELEVATION = 22, // 信噪比+高度截止角
			LEOGPSOBSEDIT_OUTLIER_EPHEMERIS    = 23, // 包括GPS星历和LEO卫星星历
			LEOGPSOBSEDIT_OUTLIER_PIF          = 24, // 伪码编辑
			LEOGPSOBSEDIT_OUTLIER_LIF          = 25, // 相位编辑
			LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT  = 26, // 编辑过程中, GPS卫星的个数限制
			// 针对sy1的伪码相位不匹配问题, 添加
			LEOGPSOBSEDIT_OUTLIER_MW           = 27, 
			LEOGPSOBSEDIT_OUTLIER_IFAMB        = 28, 
			LEOGPSOBSEDIT_OUTLIER_MWRMS        = 29,

			// 周跳类型
			LEOGPSOBSEDIT_NEWARCBEGIN          = 30,
			LEOGPSOBSEDIT_SLIP_LIF             = 31,
			// 针对sy1的伪码相位不匹配问题, 添加
			LEOGPSOBSEDIT_SLIP_MW              = 32,
			LEOGPSOBSEDIT_SLIP_IFAMB           = 33
		};
		struct LeoGPSObsPreprocDefine
		{
			double       max_ionosphere;           // 电离层延迟组合最大值，用于伪距野值剔除
	        double       min_ionosphere;           // 电离层延迟组合最小值
			double       min_elevation;            // 最低观测仰角,度
			unsigned int min_arcpointcount;        // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			double       max_arclengh;             // 相邻连续弧段间隔秒数阈值, 选择依据是最大连续跟踪弧段的时间, 一般不会超过半个轨道周期(秒)
			double       threshold_slipsize_mw;    // melbourne wuebbena 组合量 大周跳的探测阈值(周) 默认3.0 //Tslip 谷老师论文
			double       threshold_rms_mw;         // melbourne wuebbena 组合量 的均方根阈值 默认1.0
			double       threshold_slipsize_P1_L1; // P1 - L1 组合量 大周跳的探测阈值(米)10.0
			double       threshold_rms_P1_L1;      // P1 - L1 组合量 的均方根阈值(米)5.0
			double       vondrak_PIF_eps;          // vondrak 拟合参数
			double       vondrak_PIF_max;          // vondrak 拟合参数
			double       vondrak_PIF_min;          // vondrak 拟合参数
			unsigned int vondrak_PIF_width;        // vondrak 拟合参数
			double       threshold_gap;            // 弧段内的中断间隔阈值, 控制不要出现较大的中断(秒) 默认300s
			double       max_pdop;                 // 几何精度因子阈值, 超过该值的观测点将不参与运算
			double       threshold_res_raim;       // RAIM 检验残差阈值
			int          threshold_gpssatcount;    // gps可视卫星阈值
			double       threshold_editrms_code;   // 伪码残差阈值
			double       threshold_editrms_phase;  // 相位残差阈值
			double       threshold_ClockJumpSize;  // 钟跳大小阈值, 单位: 米
			bool         bOn_ClockEliminate;       // SY1卫星的数据处理需要补偿钟差
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
			int          typ_BDSobs_L1;            // 北斗第一个频点的观测数据类型 
			int          typ_BDSobs_L2;            // 北斗第二个频点的观测数据类型
			 
			LeoGPSObsPreprocDefine()
			{
				max_ionosphere           = 30.0 + 5;
		        min_ionosphere           = 0.0 - 5;
				min_elevation            = 5.0;
				min_arcpointcount        = 10;
				max_arclengh             = 2000.0;
				threshold_slipsize_mw    = 3.0; //  单位 周（宽巷）//利用 threshold_slipsize_wm 对 threshold_outlier 的上界进行控制
				threshold_rms_mw         = 1.0; // 谷老师博士论文44页  1@P //无周跳弧段的内符合诊断, 主要针对 CHAMP 卫星
				threshold_slipsize_P1_L1 = 10;  // 只能探测大周跳, 单位 m
				threshold_rms_P1_L1      = 5.0;  // 应对同时刻都发生周跳，MW组合无法探测，增加无电离层线性组合进行周跳检验 3@P
				vondrak_PIF_eps          = 1.0E-10;//平滑因子，越小平滑程度越强
				vondrak_PIF_max          = 1.5;
				vondrak_PIF_min          = 0.3;
				vondrak_PIF_width        = 30;
				threshold_gap            = 300; // 默认300s==5min
				max_pdop                 = 8.0;
				threshold_res_raim       = 3.0; // RAIM检测阈值默认3m 针对BD2\BD3需要取大一些，改成7.0-15.0左右，GPS可为3.5左右
				threshold_gpssatcount    = 3;   // gps可视卫星数阈值 
				threshold_editrms_code   = 2.50;
			    threshold_editrms_phase  = 0.10;
				bOn_ClockEliminate       = false;
				bOn_ClockJumpDiagnose    = false;
				bOn_SlipEditedMWDiagnose = false;
				bOn_IonosphereFree       = true;//是否使用消电离层组合探测周跳，默认是！
				bOn_POutlierAccordingL   = false;
				bOn_RaimSPP              = true;
				bOn_RaimArcChannelBias   = true;
				bOn_Slip_L1_L2           = false;
				bOn_GNSSSAT_AntPCOPCV    = true;
				bOn_PhaseWindUp          = true;
				typ_BDSobs_L1            = TYPE_OBS_L1;
				typ_BDSobs_L2            = TYPE_OBS_L5;				
				unloadPreprocessParaSy1();
			} 

			void loadPreprocessParaSy1()
			{
				max_ionosphere           = 10.0;
		        min_ionosphere           =-10.0;
				bOn_ClockEliminate       = true;
				bOn_ClockJumpDiagnose    = true;
				threshold_ClockJumpSize  = 4000;    // 2009/01/01, 钟跳的大小阈值
				bOn_L2SNRLostDiagnose    = true;
				bOn_SlipEditedMWDiagnose = true;    // sy1卫星必须考虑
				bOn_POutlierAccordingL   = true;
				bOn_RaimSPP              = false;
				bOn_RaimArcChannelBias   = false;
				bOn_Slip_L1_L2           = true;
			}

			void unloadPreprocessParaSy1()
			{
				max_ionosphere           = 30.0 + 5;
		        min_ionosphere           = 0.0 - 5;
				bOn_ClockEliminate       = false;
				bOn_ClockJumpDiagnose    = false;
				threshold_ClockJumpSize  = 4000;    // 2009/01/01, 钟跳的大小阈值
				bOn_L2SNRLostDiagnose    = false;
				bOn_SlipEditedMWDiagnose = true;
				bOn_POutlierAccordingL   = false;
				bOn_RaimSPP              = true;//RAIM检查单点
				bOn_RaimArcChannelBias   = true;//RAIM检查弧段
				bOn_Slip_L1_L2           = false;//针对轨道比较高的卫星开
			}
		};

		/*struct SLIPINFO
		{
			GPST                   t;
			BYTE                   id_sat;
			double                 size;
		};*/

		class LeoGPSObsPreproc
		{
		public:
			LeoGPSObsPreproc(void);
		public:
			~LeoGPSObsPreproc(void);
		public:
			// 双频 GPS 观测数据预处理
			void    setPreprocPath(string strPreprocPath);
			void    setPreprocSatName(string strSatName);
			void    setSP3File(SP3File sp3File); 
			void    setCLKFile(CLKFile clkFile); 
			void    setObsFile(Rinex2_1_ObsFile obsFile);
			bool    loadSP3File(string  strSp3FileName);
			bool    loadCLKFile(string  strCLKFileName);
			bool    loadObsFile(string  strObsFileName);
			void    setAntPhaseCenterOffset(POS3D posRTN);
			void    setAntPhaseCenterOffset(POS3D posBody, Matrix matAxisBody2RTN);
			BYTE    obsPreprocInfo2EditedMark1(int obsPreprocInfo);
			BYTE    obsPreprocInfo2EditedMark2(int obsPreprocInfo);
			bool    SinglePointPositioning_PIF(int index_P1, int index_P2,double frequence_L1,double frequence_L2, Rinex2_1_LeoEditedObsEpoch obsEpoch, POSCLK& posclk, int& eyeableGPSCount, double& pdop, double& rms_res, double threshold = 1.0E-002);
			bool    RaimEstChannelBias_PIF(int index_P1,int index_P2,double frequence_L1,double frequence_L2, Rinex2_1_LeoEditedObsEpoch obsEpoch, int nPRN, POSCLK& posclk, double& channelBias, double threshold = 1.0E-002);
			int     RaimSPP_PIF(int index_P1, int index_P2,double frequence_L1,double frequence_L2, Rinex2_1_LeoEditedObsEpoch& obsEpoch, POSCLK& posclk, double& pdop, double& rms_res);
			bool    detectRaimArcChannelBias_PIF(int index_P1,int index_P2, vector<Rinex2_1_LeoEditedObsEpoch> &obsEpochList, double threshold = 0.20);
			bool    detectL2SNRLost(int index_S2, int index_P1, int index_P2, int index_L1, int index_L2, Rinex2_1_EditedObsSat& obsSat);
			bool    detectCodeOutlier_ionosphere(int index_P1, int index_P2, Rinex2_1_EditedObsSat& obsSat,double frequence_L1 = GPS_FREQUENCE_L1,double frequence_L2 = GPS_FREQUENCE_L2);
			bool    detectPhaseSlip(int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1,double frequence_L2, Rinex2_1_EditedObsSat& obsSat);
			bool    mainFuncDFreqGPSObsPreproc(Rinex2_1_LeoEditedObsFile  &editedObsFile,double FREQUENCE_L1 = GPS_FREQUENCE_L1,double FREQUENCE_L2 = GPS_FREQUENCE_L2);
            // TQ高轨卫星数据处理
			bool    mainFuncTQObs2Edt(Rinex2_1_LeoEditedObsFile  &editedObsFile,int type_obs_L1 = TYPE_OBS_L1,int type_obs_L2 = TYPE_OBS_L2);
			bool    TQSinglePointPositioning_PIF(int index_P1, int index_P2,double frequence_L1,double frequence_L2, Rinex2_1_LeoEditedObsEpoch obsEpoch, POSCLK& posclk, int& eyeableGPSCount, double& pdop, double& rms_res, double threshold = 1.0E-002);
			// 双频 GPS 观测数据编辑主要函数
			void    setLeoOrbitList(vector<TimePosVel> leoOrbitList);
			bool    getLeoOrbitPosVel(GPST t, TimePosVel& orbit, unsigned int nLagrange = 8);
			bool    pdopSPP(int index_P1, int index_P2, Rinex2_1_LeoEditedObsEpoch obsEpoch, int& eyeableGPSCount, double& pdop);
			bool    obsEdited_LIF(int index_L1, int index_L2, double frequence_L1,double frequence_L2,int nPRN, Rinex2_1_LeoEditedObsEpoch epoch_j_1, Rinex2_1_LeoEditedObsEpoch epoch_j, double &res, bool &slipFlag);
			bool    mainFuncDFreqGPSObsEdit(Rinex2_1_LeoEditedObsFile  &editedObsFile,int type_obs_L1 = TYPE_OBS_L1,int type_obs_L2 = TYPE_OBS_L2);
			// 多系统观测数据预处理
			bool    pdopMixedObsSPP(int index_P1_GPS, int index_P2_GPS, int index_P1_BDS, int index_P2_BDS, Rinex2_1_LeoMixedEditedObsEpoch obsEpoch, int& eyeableSatCount, double& pdop);
			bool    mainFuncMixedObsPreproc(string  strMixedObsFileName, Rinex2_1_LeoMixedEditedObsFile  &mixedEditedObsFile,bool bOn_edit = true,int FLAG_BDS = 1);	
			// 单频数据处理
			bool    detectPhaseSlip_L1(int index_P1, int index_L1, Rinex2_1_EditedObsSat& obsSat);
			bool    mainFuncSFreqGPSObsPreproc(Rinex2_1_LeoEditedObsFile  &editedObsFile);
			bool    obsEdited_GRAPHIC(int index_P1, int index_L1, int nPRN, Rinex2_1_LeoEditedObsEpoch epoch_j_1, Rinex2_1_LeoEditedObsEpoch epoch_j, double &res, bool &slipFlag);
			bool    mainFuncSFreqGPSObsEdit(Rinex2_1_LeoEditedObsFile  &editedObsFile);
		private:
			bool    getEditedObsEpochList(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist); 
			bool    getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
			bool    datalist_epoch2sat(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist, vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
		    bool    datalist_sat2epoch(vector<Rinex2_1_EditedObsSat>& editedObsSatlist, vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist);
			vector<Rinex2_1_LeoEditedObsEpoch>  m_editedObsEpochList;
			string  m_strPreprocPath;
			string  m_SatName;
		public:
			LeoGPSObsPreprocDefine m_PreprocessorDefine;
			CLKFile                m_clkFile; // 精密钟差数据文件
			SP3File                m_sp3File; // 精密星历数据文件
			Rinex2_1_ObsFile       m_obsFile; // 单系统原始观测数据			
			POS3D                  m_pcoAnt;  // 天线偏心量		
			vector<TimePosVel>     m_leoOrbitList; // 动力学参考轨道
			int                    m_countSlip;
			int                    m_countRestSlip; // 保留为被删除掉的周跳
			Matrix                 m_matAxisAnt2Body;  // 天线系到星固系的转换矩阵，用于天线可能安装在非天顶方向
		};
	}
}