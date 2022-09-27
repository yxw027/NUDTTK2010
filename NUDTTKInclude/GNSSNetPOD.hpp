#pragma once
#include "SatdynBasic.hpp"
#include "constDef.hpp"
#include "structDef.hpp"
#include "MathAlgorithm.hpp"
#include "lambda.hpp"
#include <map>
#include "Rinex3_03_EditedObsFile.hpp"
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "TROZPDFile.hpp"
#include "svnavMixedFile.hpp"
#include "GNSSYawAttitudeModel.hpp"
#include "igs05atxFile.hpp"
#include "Sinex2_0_File.hpp"
#include"StaOceanLoadingDisplacementFile.hpp"

using namespace NUDTTK;
using namespace NUDTTK::Geodyn;
using namespace NUDTTK::Math;
using namespace NUDTTK::LAMBDA;

//  Copyright 2017, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	namespace GPSPod
	{
		// 观测数据历元结构元素，储存伪码数据，相位数据 +
		struct NETPOD_ObsEqEpochElement
		{
			string name;           // 卫星名, 如G01			
			float  Azimuth;        // 观测方位角
           float  Elevation;      // 观测高度角
			double obs_code;       // 伪码观测数据，IF
			double obs_phase;      // 相位观测数据，IF
			double ambiguity_IF;    // IF模糊度
			double ambiguity_MW;    // MW模糊度 +
			float  weightCode;        // 伪码观测权值
			float  weightPhase;       // 相位观测权值
			int    id_Ambiguity;      // 模糊度序号
			BYTE    mark_GNSSSatShadow;      // 进入地影标记, 默认 0 — 未进入地影

			NETPOD_ObsEqEpochElement()
			{
				mark_GNSSSatShadow = 0;
			}
		};
		// 观测数据历元结构
		struct NETPOD_ObsEqEpoch
		{
			int                               id_Epoch;            // 观测历元序号
			int                               validIndex;          // 有效时间标签  (用于相位观测方程设计矩阵索引有效位置参数号）
			int                               eyeableSatCount;     // 可视卫星数
			map<string, NETPOD_ObsEqEpochElement>  obsList;       // 不同卫星观测列表
		};

		// 定轨历元结构元素，储存残差，视线矢量 +
		struct NETPOD_PodEqEpochElement
		{
			string name;              // 卫星名, 如G01			
			float  oc_code;           // 伪码残差
			float  oc_phase;          // 相位残差
			float  rw_code;           // 伪码鲁棒权
			float  rw_phase;          // 相位鲁棒权
			float  weightCode;         // 伪码观测权值
			float  weightPhase;        // 相位观测权值
			bool   bEphemeris;         // 该历元卫星星历获取成功标识
			POSCLK vecLos_A;          // 视线矢量, 定轨时使用
			TimePosVel pvEphemeris;          // 记录GNSS卫星轨道位置检索结果 ???
			double  obscorrected_value;      // 观测数据改正量-相位
			double  obscorrected_value_code;  // 观测数据改正量-相位
			Matrix  interpRtPartial_A;       // 偏导数, 定轨时使用 ???  

			NETPOD_PodEqEpochElement()
			{
				rw_code  = 1.0f;
				rw_phase = 1.0f;
				oc_code  = 0.0f;
				oc_phase = 0.0f;
			}
		};
		// 定轨历元结构
		struct NETPOD_PodEqEpoch
		{
			int                            id_Epoch;            // 观测历元序号
			int                            validIndex;          // 有效时间标签  (用于相位观测方程设计矩阵索引有效位置参数号）
			int                            eyeableSatCount;     // 可视卫星数
			map<string, NETPOD_PodEqEpochElement>  obsList;       // 不同卫星观测列表
		};

		//// 需要存入二进制文件数据格式
		//struct tagStaEpochSatData
		//{
		//	string name;           // 卫星名, 如G01			
		//	float  Azimuth;        // 观测方位角
  //         float  Elevation;      // 观测高度角
		//	double obs_code;       // 伪码观测数据，IF
		//	double obs_phase;      // 相位观测数据，IF
		//	double ambiguity_IF;    // IF模糊度
		//	double ambiguity_MW;    // MW模糊度 +
		//	float  weightCode;        // 伪码观测权值
		//	float  weightPhase;       // 相位观测权值
		//	int    id_Ambiguity;      // 模糊度序号
		//	float  oc_code;        // 伪码残差
		//	float  oc_phase;       // 相位残差
		//	float  rw_code;           // 伪码鲁棒权
		//	float  rw_phase;          // 相位鲁棒权
		//	bool   bEphemeris;        // 该历元卫星星历获取成功标识
		//	POSCLK vecLos_A;          // 视线矢量, 定轨时使用
		//	TimePosVel pvEphemeris;          // 记录GNSS卫星轨道位置检索结果 ???
		//	double  obscorrected_value;      // 观测数据改正量-相位
		//	double  obscorrected_value_code;  // 观测数据改正量-相位
		//	Matrix  interpRtPartial_A;       // 偏导数, 定轨时使用 ???  
		//	BYTE    mark_GNSSSatShadow;      // 进入地影标记, 默认 0 — 未进入地影

		//	tagStaEpochSatData()
		//	{
		//		mark_GNSSSatShadow = 0;
		//	}

		//};

		// 观测方程弧段元素, 用于存储相位数据，保留
		struct NETPOD_ObsEqArcElement
		{   
			// 用于构造法方程
			int    nObsTime;     // 观测时间序号
			double LIF;          // 原始相位无电离层组合观测量
			double MW;          // 原始相位MW组合观测量
			double L1_L2;
			double dt;
			float  wL;           // 先验观测权值, 与高度角有关
			float  rw;           // 编辑后的权值
			float  oc;
			NETPOD_ObsEqArcElement()
			{
				rw = 1.0f;
				oc = 0.0f;
				wL = 1.0f;
			}
		};

		// 弧段数据在存储时，放入测站数据结构，便于在基线处理时直接检索，保留
		struct NETPOD_ObsEqArc 
		{
			double       ambiguity_IF; 
			double       ambiguity_MW; // +
			short int    id_Ambiguity; 
			//GPST         t0;
			//GPST         t1;
			//string       nameSta;
			string       nameSat;
			string       nameFreq;     // 便于查找频间差数据 
			int          count;
			map<int, NETPOD_ObsEqArcElement> obsList;
			bool updateRobustWeight(double threshold_slip, double threshold_rms, unsigned int threshold_count_np = 20, double robustfactor = 3.0);
		};


		struct NETPOD_DownSamplingElement // 保留
		{
			// 轨道计算部分
			POS3D  vecLos;
			Matrix interpRtPartial;
			// 对流层修正部分
			short int id_zenithDelay_0;     
	       double    zenithDelay;           // 修正值
			double    zenithDelay_partial_0; // 偏导数
			double    zenithDelay_partial_1;
			// 残差编辑部分
			float  oc_LIF;
			float  oc_PIF;
			float  rwP;
			float  rwL;
		};
		struct NETPOD_ObsElement   // 保留
		{
			double    LIF;
			double    PIF;
			short int id_ambiguity;
			float     wL;
			float     wP;
			float     zenithDelay_wmf;            // 用于对流层修正计算
			double    corrected_value;
			double    corrected_value_code;       // 改正量-伪码
			vector<NETPOD_DownSamplingElement> ds; // 高采样和低采样的部分区分，采用Matrix或vector等动态结构，防止占用空间
		};

		typedef map<string, NETPOD_ObsElement> NETPOD_SatMap; // 保留

		typedef map<string, NETPOD_SatMap> NETPOD_StaMap; // 保留

		struct NETPOD_TropZenithDelay  // 保留
		{
			GPST    t;                  // 对流层估计的时间
			double  zenithDelay_Init;    // 对流层湿分量估计初值
			double  zenithDelay_Est;     // 对流层估计结果
			int     count;              // 有效历元个数
		};

		// 卫星数据结构
		struct NETPOD_SatDatum  // 保留
		{
			int                index;             // 待估参数块的位置(卫星序号 0,1,2,..)
			int                index_0;           // 首个待估参数位置
			int                count_obs;         // 观测数据个数
			SatdynBasicDatum    dynamicDatum_Init;  // 初始轨道
			SatdynBasicDatum    dynamicDatum_Est;   // 估计的轨道结果

			// PCO估计
			bool      bOnEstPco_X;
			bool      bOnEstPco_Y;
			bool      bOnEstPco_Z;
			double    sigmaPco; 
			POS3D     pco_Est;      // 位置估计结果
			// PCV估计
			bool      bOnEstPcv;

			vector<TimePosVel>  REF_OrbList;       // 用于轨道插值的参考点序列, 采样率等步长（adamsCowell积分步长，75s）
			vector<Matrix>      REF_RtPartialList; // 用于偏导数插值
			bool getEphemeris(TDT t, TimePosVel& interpOrb, int nLagrange = 9);
			bool getRtPartial(TDT t, Matrix& interpRtPartial);
			bool getEphemeris_PathDelay(TDT t, POSCLK staPosClk, double& delay, TimePosVel& gnssOrb, Matrix& gnssRtPartial, double threshold = 1.0E-07);
		};
		typedef map<string, NETPOD_SatDatum> NETPOD_SatDatumMap; // 保留
		
		struct NETPOD_SatOrbEstParameter  // 保留
		{
			NETPOD_SatDatumMap satParaList;
			GPST   t0_xpyput1;
			double xp;           // 单位: 弧度
			double xpDot;        // 单位: 弧度/秒
			double yp;
			double ypDot;
			double ut1;          // 单位: 弧度
			double ut1Dot;       // 单位: 弧度/秒
			
			NETPOD_SatOrbEstParameter()
			{
				xp     = 0;
				xpDot  = 0;
				yp     = 0;
				ypDot  = 0;
				ut1    = 0;
				ut1Dot = 0;
			}

			// 计算地球旋转矩阵改进量
			void getEst_EOP(GPST t, Matrix &matEst_EP, Matrix &matEst_ER)
			{
				double spanSeconds = t - t0_xpyput1;
				double delta_xp = xp + xpDot * spanSeconds;
				double delta_yp = yp + ypDot * spanSeconds;
				matEst_EP.Init(3,3);
				matEst_EP.SetElement(0, 0,  1);
				matEst_EP.SetElement(0, 2,  delta_xp);
				matEst_EP.SetElement(1, 1,  1);
				matEst_EP.SetElement(1, 2, -delta_yp);
				matEst_EP.SetElement(2, 0, -delta_xp);
				matEst_EP.SetElement(2, 1,  delta_yp);
				matEst_EP.SetElement(2, 2,  1);

				double delta_ut1 = ut1 + ut1Dot * spanSeconds;
				matEst_ER.Init(3,3);
				matEst_ER.SetElement(0, 0,  1);
				matEst_ER.SetElement(0, 1,  delta_ut1);
				matEst_ER.SetElement(1, 0, -delta_ut1);
				matEst_ER.SetElement(1, 1,  1);
				matEst_ER.SetElement(2, 2,  1);
			}
		};
		// 卫星数据结构，用于轨道拟合
		struct NETPOD_SatSp3FitDatum       // 保留
		{
			SatdynBasicDatum    dynamicDatum_Init;     // 初始轨道, 从sp3文件读取
			SatdynBasicDatum    dynamicDatum_Est;      // 估计的轨道结果
			vector<TimePosVel>  sp3orbitList_ECEF;     // sp3 轨道, 地固系
            vector<TimePosVel>  fitorbitList_ECEF;     // 拟合轨道, 地固系
			double              fitrms_X;              // 轨道拟合精度 
			double              fitrms_Y;
			double              fitrms_Z;
			double              fitrms_R;
			double              fitrms_T;
			double              fitrms_N;
			double              fitrms_Total;				
            
			bool getInterpOrb_Fit(GPST t, TimePosVel& interpOrbit, int nLagrange = 9);
		};

		typedef map<string, NETPOD_SatSp3FitDatum> NETPOD_SatSp3FitDatumMap; // 保留

		struct NETPOD_Sp3FitParameter // 保留
		{
			NETPOD_SatSp3FitDatumMap satParaList;
			GPST   t0_xpyput1;
			double xp;           // 单位: 弧度
			double xpDot;        // 单位: 弧度/秒
			double yp;
			double ypDot;
			double ut1;          // 单位: 弧度
			double ut1Dot;       // 单位: 弧度/秒

			double meanFitRms_X; // 轨道拟合精度 
			double meanFitRms_Y;
			double meanFitRms_Z;
			double meanFitRms_R;
			double meanFitRms_T;
			double meanFitRms_N;
			double meanFitRms_Total;

			// 计算地球旋转矩阵改进量
			void getEst_EOP(GPST t, Matrix &matEst_EP, Matrix &matEst_ER)
			{
				double spanSeconds = t - t0_xpyput1;
				double delta_xp = xp + xpDot * spanSeconds;
				double delta_yp = yp + ypDot * spanSeconds;
				matEst_EP.Init(3,3);
				matEst_EP.SetElement(0, 0,  1);
				matEst_EP.SetElement(0, 2,  delta_xp);
				matEst_EP.SetElement(1, 1,  1);
				matEst_EP.SetElement(1, 2, -delta_yp);
				matEst_EP.SetElement(2, 0, -delta_xp);
				matEst_EP.SetElement(2, 1,  delta_yp);
				matEst_EP.SetElement(2, 2,  1);

				double delta_ut1 = ut1 + ut1Dot * spanSeconds;
				matEst_ER.Init(3,3);
				matEst_ER.SetElement(0, 0,  1);
				matEst_ER.SetElement(0, 1,  delta_ut1);
				matEst_ER.SetElement(1, 0, -delta_ut1);
				matEst_ER.SetElement(1, 1,  1);
				matEst_ER.SetElement(2, 2,  1);
			}

			NETPOD_Sp3FitParameter()
			{
				xp     = 0;
				xpDot  = 0;
				yp     = 0;
				ypDot  = 0;
				ut1    = 0;
				ut1Dot = 0;
			}
		};


		// 频率信息结构
		struct NETPOD_Freq // 保留
		{
			double                                  freq_L1;
			double                                  freq_L2;
			NETPOD_Freq()
			{
				freq_L1 = 0.0;
				freq_L2 = 0.0;
			}
		};

      // 整网定轨参数结构, 待增加参数
		struct NETPOD_DEF
		{
			int                          max_OrbIterationCount;      // 轨道改进次数阈值		
			unsigned int                  min_ArcPointCount;         // 弧段观测个数阈值, 个数小于该阈值的弧段将被删除
			int                          min_EyeableSatCount;       // 最小可视卫星个数, 个数小于该阈值的历元将不参与解算
			double                       max_ArcInterval;     // 历元间隔阈值, 超出此阈值认为是新弧段(单位：秒)
           bool                          on_UsedInShadow; // +
		   bool                          on_WeightGEO;
		   float                         weightInShadow; // +
		   float                         weightGEO;
			bool                         on_AmbiguityFixing;        // 是否进行固定整周模糊度
			bool                         bOn_GEOSolut;             // 是否解算GEO卫星	
			bool                         bOn_IGSOSolut;            // 是否解算IGSO卫星
			bool                         bOn_Used_delta_u;          // 是否使用Delta_u作为太阳光压的输入
			double                       apriorityRms_TZD_abs;      // 先验测站电离层天顶延迟精度, 用于电离层天顶延迟绝对约束方程加权控制
			double                       apriorityRms_TZD_rel;      // 用于相邻电离层天顶延迟参数的相对约束方程加权控制
			double                       apriorityWet_TZD;          // 测站对流层湿分量估计先验值
			double                       apriorityWet_TZD_period;   // 测站对流层湿分量估计周期
			double                       min_Wet_TZD_ncount;        // 对流层估计所需的最少数据个数

			float                        min_Elevation;             // 最小高度角
			float                        priorsigma_LIF;            // 用于相位加权控制
			bool                         bOn_WeightElevation;       // 是否进行高度角加权 +
			bool                         bOn_StaSolidTideCor;       // 是否进行测站坐标的固体潮改正 +
			bool                         bOn_StaOceanTidesCor;      // 是否进行测站坐标的海潮改正 +
			bool                         bOn_GraRelativity;         // 是否进行引力引起的相对论改正 +


			bool                     bOnEst_StaTropZenithDelay;      // 测站对流层估计开关 +
			bool                     bOnEst_StaPos;           // 测站位置估计
			bool                     bOnEst_SatEphemeris;       // 卫星轨道估计

			double                   max_FitRms_Total;        // 拟合残差最大阈值

			NETPOD_DEF()
			{
				max_OrbIterationCount     = 10;				
				min_ArcPointCount         = 12;
				min_EyeableSatCount      = 3;
				max_ArcInterval          = 180;
				on_UsedInShadow          = false;
				on_WeightGEO             = false;
				weightInShadow           = 0.0;
				weightGEO                = 0.0;
				on_AmbiguityFixing       = false;
				bOn_GEOSolut             = false;
				bOn_IGSOSolut            = false;
				bOn_Used_delta_u          = false;
				apriorityRms_TZD_abs      = 0.5;
				apriorityRms_TZD_rel      = 0.05;
				apriorityWet_TZD          = 0;                      // 测站对流层湿分量估计先验值
				apriorityWet_TZD_period   = 3600 * 2;               // 周期                
				min_Wet_TZD_ncount        = 12;
				min_Elevation = 5.0f;
				priorsigma_LIF            = 0.005f;
				bOn_WeightElevation       = false;
				bOn_StaSolidTideCor  = true;
				bOn_StaOceanTidesCor = true;
				bOn_GraRelativity    = true;
				bOnEst_StaTropZenithDelay = true;
				bOnEst_StaPos = false;
				bOnEst_SatEphemeris = false;
				max_FitRms_Total    = 0.5;
			}
		};

		// 多系统预定义, 作为统一多系统选择输入, [注: 选择后不同测站的多系统数据可能不全]
		struct NETPOD_MixedSys
		{
			// 需要初始化部分
			char                     cSys;                         // 系统标识
			float                   wSys;                         // 系统权值, 默认 1.0
			string                   name_C1;    // 伪码
			string                   name_C2;
			string                   name_L1;    // 相位
			string                   name_L2;
			string                   nameFreq_L1;                  // 用于PCV修正
            string                   nameFreq_L2;
			NETPOD_Freq               freqSys;
			map<string, NETPOD_Freq>   freqSatList;                  // 主要针对GLONASS，string类型为卫星名

			// 不需要初始化部分
			int                      iSys;                         // 记录数据在m_editedMixedObsFile存储的位置
			int                      index_C1;
			int                      index_C2;
			int                      index_L1;
			int                      index_L2;
			float                    priorsigma_PIF;               // 用于伪码加权控制

			
			// 与系统有关的预定义部分, 使用时需要通过sysFreqName索引 
			bool                     on_GNSSPhaseWindUp;   
			bool                     on_GNSSRelativity;        // 是否进行 GNSS 卫星相对论改正 +
		    bool                      on_GNSSAntPCO;       // 是否进行 GNSS 卫星天线 PCO 修正 +
			bool                     on_GNSSAntPCV;       // 是否进行 GNSS 卫星天线 PCV 修正 +
			bool                     on_GNSSRecARP;       // 是否进行接收机天线 ARP 修正
			bool                     on_GNSSRecPCOPCV;    // 是否进行接收机天线 PCO/PCV 修正

			TYPE_SOLARPRESSURE_MODEL       solarPressure_Model;       // 每个系统的太阳光压模型
			TYPE_SOLARPRESSURE_MODEL      solarPressure_Model_BDYF;       // yaw-fixed姿态太阳光压模型，2015/11/13，刘俊宏	
			TYPE_SOLARPRESSURE_MODEL      solarPressure_Model_BDYS;       // yaw-steering姿态太阳光压模型
			double                       period_SolarPressure;  


			string getSysFreqName()
			{
				char sysFreqName[4];
				sprintf(sysFreqName, "%1c%1c%1c", cSys, name_L1[1], name_L2[1]); // "Gij" "Cij" "Eij" "Rxx"
				sysFreqName[3] = '\0';
				return sysFreqName;
			}

			NETPOD_MixedSys()
			{
				wSys = 1.0;
				index_C1 = -1;
				index_C2 = -1;
				index_L1 = -1;
				index_L2 = -1;
				name_C1  = "";
				name_C2  = "";
				name_L1  = "";
				name_L2  = "";
				nameFreq_L1 = "";
				nameFreq_L2 = "";
				on_GNSSPhaseWindUp = true;
				on_GNSSRelativity = true;
				on_GNSSAntPCO = true;
				on_GNSSAntPCV  = true;
				priorsigma_PIF = 0.50f;
			}
		};
		// 多系统数据
		struct NETPOD_MixedSysData        // - 
		{
			// 不需要初始化部分
			char                      cSys;                   // 系统标识
			float                    wSys;                   // 系统权值, 默认 1.0
			string                    name_C1;                // "C1" "P1" "P2"
			string                    name_C2;                
			string                    name_L1;                // "L1" "L2"
			string                    name_L2;                
			string                    nameFreq_L1;            // 频率名称"G01"，主要用于PCV修正
			string                    nameFreq_L2;  
			NETPOD_Freq                freqSys;
			map<string, NETPOD_Freq>    freqSatList;            // 主要针对GLONASS，string类型为卫星名
			float                     priorsigma_PIF;         // 用于伪码加权控制
			int                       index_C1;               // 频率1和2在处理时主用
			int                       index_C2;
			int                       index_L1;
			int                       index_L2;
			
			bool                     on_GNSSPhaseWindUp;   
			bool                     on_GNSSRelativity;        // 是否进行 GNSS 卫星相对论改正 +
		    bool                     on_GNSSAntPCO;       // 是否进行 GNSS 卫星天线 PCO 修正 +
			bool                     on_GNSSAntPCV;       // 是否进行 GNSS 卫星天线 PCV 修正 +
			bool                     on_GNSSRecARP;       // 是否进行接收机天线 ARP 修正
			bool                     on_GNSSRecPCOPCV;    // 是否进行接收机天线 PCO/PCV 修正
			
			// 过程计算存储部分, 节省内存，不存观测数据
			string                               sysFreqName;                 // 记录系统和频率信息, 用于索引相关的预定义
			int                                  iSys;                       // 记录数据在m_editedMixedObsFile存储的位置

			vector<Rinex3_03_EditedObsEpoch>        editedObsEpochList;          // 多系统历元格式观测数据，处理结束后清空
			vector<Rinex3_03_EditedObsSat>          editedObsSatlist;           // 多系统卫星格式观测数据，处理结束后清空 

			//map<int, NETPOD_ObsEqEpoch>            P_IFEpochList;               // 参与解算的消电离层伪码（历元结构），换成map格式方便索引，int为nObsTime
			
			map<int, NETPOD_ObsEqEpoch>             mapNEQEpochList;            // 历元法方程观测数据(包括伪码和相位历元数据), 与editedObsEpochlist相对应，int为nObsTime + 写入文件后清空
			map<int, NETPOD_PodEqEpoch>             mapPODEpochList;            // 历元法方程定轨数据(包括残差、视线矢量数据), 与editedObsEpochlist相对应，int为nObsTime + 写入文件后清空
			
			vector<NETPOD_ObsEqArc>                 L_IFArcList;               // 参与解算的消电离层相位数据（弧段结构）->历元结构，处理结束后清空
			
			//string                                pathEditedObsEpochListFile;    // 单系统历元观测数据文件目录 +
			//map<int, NETPOD_ObsElement>             mapNEQEpochList;              // 历元法方程数据, 与editedObsEpochlist相对应，int为nObsTime
			int                                   id_Ambiguity_0;               // 模糊度起始编号
			map<string, int>                       mapObsCount;                  // 每颗卫星的观测数据个数

			NETPOD_MixedSysData()
			{
			}
		};

		// 钟差解部分
		struct GNSSPOD_CLKSOL // 测站数据结果里
		{
			GPST    t;
			double  clock;
			int     sqlMark;          // 解标记: 0-插值获得, 未经过解算; 1-伪码解; 2-相位解
			int     eyeableSatCount; 
			int     validIndex;       // 记录有效解索引序号, 无效为-1 
			double   sigma; 
			map<int, double> mapN_ca; // 记录交叉项 cc与ca
		};

		typedef map<int,   POS3D>    StaPosMap;

		// 测站数据结构 + 
		struct NETPOD_StaDatum
		{
			// 输入部分
			string                     staName;
			bool                       bOnUsed;
			POS6D                      pv0_ITRF;                    // ITRF下测站先验坐标
			//Rinex3_03_EditedObsFile    m_editedMixedObsFile;           // 测站观测数据文件
			string                     pathRinex3_03_EditedObsFile;   // 测站观测数据文件目录 +
			int                        count_MixedEpoch;             // 观测数据历元个数
			GPST                       t0;                         // 观测数据起始时间
           GPST                        t1;                         // 观测数据起始时间
		   	ENU                        arp0_ENU;                   // 天线参考点ARP在测站坐标系下的坐标(ENU)

			// 过程计算存储部分: 多系统数据, 与多系统的预定义 m_dataMixedSysList 呼应
			// 存在问题: 预定义 m_dataMixedSysList 有 N 个系统, 但数据中可能只有 M 个系统 dataMixedSysList, 不匹配
			//           每个测站数据文件类型索引号 "index_xx" 可能是不同的，需要与 dataMixedSysList 绑定
			// 解决办法: m_dataMixedSysList 中只定义类型选取之类的必要输入控制信息, 得到的有用索引信息放入 dataMixedSysList 中去
			vector<NETPOD_MixedSysData>  dataMixedSysList;
			bool                       on_RecPCVFromIGS;      // 是否采用IGS接收机天线PCV
			AntCorrectionBlk            pcvRecFromIGS;         //  IGS接收机天线PCV结构

			vector<string>              satGnssNameList;       // GNSS卫星列表

			// 位置估计块
			char      szAntType[20 + 1];
			ENU       arp_Ant;	
			bool      bOnEstPos;     // 测站位置估计开关
			int		  indexEstPos_0;  // 测站位置在整个估计参数列表中的位置
			double    sigmaPos;      // 控制测站位置估计的约束条件，2012.12.09，刘俊宏
			POS3D     pos_Est;      // 位置估计结果

			// 钟差估计块
			vector<GNSSPOD_CLKSOL>       clkList; // 钟差解部分
			vector<int>                 validMixedEpochList; // 记录钟差有效解对应的时刻位置

			// 模糊度参数
			int count_EstAmbiguity;   // 当前测站模糊度个数

			vector<POS3D>     corrTideList;      // 存储测站坐标潮汐改正量[地固系], 避免反复计算

			// 解算过程测站坐标
			StaPosMap                 staPosList;     //测站A在惯性系下的坐标列表
			StaPosMap                 staECEFPosList; //测站A在地固系下的坐标列表
			//map<GPST, POS3D> staJ2000PosList; // 兼顾高低采样率
			//map<GPST, POS3D> staECEFPosList;  // 兼顾高低采样率

			// 测站位置潮汐改正写在文件里 记录每个时刻的观测权值信息 L_IFArcList staPosList staECEFPosList？
			
			// 对流层估计
			int       indexZenith_0;  // 测站首个对流层估计参数在整个估计参数列表中的位置	
			vector<NETPOD_TropZenithDelay> zenithDelayEstList;

			// 系统偏差估计
			int       indexSysBias_0; // 测站首个系统偏差参数在整个估计参数列表中的位置	

			
			// 地球旋转参数矩阵
			vector<Matrix>            matPR_NRList;
			vector<Matrix>            matERList_0;    // 概略地球旋转矩阵
			vector<Matrix>            matEPList_0;    // 概略极移旋转矩阵
		    vector<Matrix>            matEPList;      // 改进后的地球旋转矩阵
			vector<Matrix>            matERList;      // 改进后的地球旋转矩阵

			void init(double period_TropZenithDelay = 3600 * 2.0, double apriorityWet_TZD = 0);	
			int  getIndexZenithDelayEstList(DayTime t);      // 确定时刻t在对流层参数列表中的位置
		};

		typedef map<string, NETPOD_StaDatum> NETPOD_StaDatumMap;

		// GNSS 整网精密定轨类(SatdynBasic的派生类)
		class GNSSNetPOD : public SatdynBasic
		{
			public:
				GNSSNetPOD(void);
			public:
				~GNSSNetPOD(void);
			private:
				void weighting_Elevation(float Elevation, float& weight_P_IF, float& weight_L_IF);
				bool adamsCowell_ac(TDT t0_Interp, TDT t1_Interp, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 75.0, int q = 11);			
			public:
				bool sp3Fit(string strSp3FilePath, NETPOD_Sp3FitParameter& paraSp3Fit, bool bOnEst_EOP = true, string outputSp3FitFilePath = "");
				bool mainGNSSPOD(string inputSp3FilePath, string outputSp3FilePath, string outputClkFilePath, string outputTrozpdFilePath, GPST t0, GPST t1, double h_clk = 30.0, double h_sp3 = 30.0);
			
			private:
				bool correctSta_ObsEpoch(NETPOD_StaDatumMap::iterator it_Sta); // 观测数据修正
				bool updateSta_ObsEpoch(NETPOD_StaDatumMap::iterator it_Sta); // 初始化观测数据历元信息 
			private:
				double  m_stepAdamsCowell;
				SP3File m_sp3FileJ2000;                    	// 记录J2000下Gnss卫星轨道
				map<string, AntCorrectionBlk>  m_mapGnssPCVBlk; //  记录Gnss卫星的PCV检索结果
			public:
				svnavMixedFile                 m_svnavMixedFile;	     // 用于获得卫星类型
				GNSSYawAttitudeModel           m_gymMixed;
				SP3File                        m_sp3File;
				igs05atxFile			       m_atxFile;		         // 天线修正文件
				NETPOD_StaDatumMap             m_mapStaDatum;          // 测站数据
				vector<NETPOD_MixedSys>        m_dataMixedSysList;     // 混合系统定义 +
				Sinex2_0_File                  snxFile;                // 测站坐标文件 +
				StaOceanLoadingDisplacementFile  m_staOldFile;          // 海潮文件 +
				NETPOD_DEF                      m_netPodParaDefine;
				TROZPDFile                      zpdFile;              // 对流层文件 +	
				NETPOD_SatOrbEstParameter        paraSatOrbEst; // 轨道求解参数结构
		};
	}
}
