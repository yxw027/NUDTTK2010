#pragma once
#include "Rinex3_03_EditedObsFile.hpp"
#include "Rinex3_0_ObsFile.hpp"
#include "dynPODStructDef.hpp"
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "jplEphFile.hpp"
#include "TimeCoordConvert.hpp"
#include "svnavMixedFile.hpp"
#include "GNSSYawAttitudeModel.hpp"
#include "igs05atxFile.hpp"
#include "StaOceanLoadingDisplacementFile.hpp"
#include "ParaEliminationNEQ.hpp"

//  Copyright 2017, The National University of Defense Technology at ChangSha
using namespace NUDTTK::Math;
using namespace NUDTTK::Geodyn;
namespace NUDTTK
{
	namespace GNSSPrecisePointPositioning
	{
		// 观测方程历元元素, 主要用于存储伪码数据
		struct PPP_ObsEqElement 
		{ 
			//int  id_Ambiguity;  // 模糊度序号
			double obs;           
			float  oc;
			float  rw;            // 鲁棒权

			PPP_ObsEqElement()
			{
				rw = 1.0f;
				oc = 0.0f;
			}
		};

		struct PPP_ObsEqEpoch
		{
			int                          id_Epoch;    // 观测时间序号, 用于索引关键数据
			map<string, PPP_ObsEqElement>  obsList;     // 不同卫星观测列表
		};

		struct PPP_CleanArcElement
		{   
			// 用于构造法方程
			int    nObsTime;     // 观测时间序号
			double LIF;
			float  wL;           // 先验观测权值, 与高度角有关
			float  rw;           // 编辑后的权值
			float  oc;
            // 用于周跳探测与修复
			double dt;
			double MW;           // 用于计算宽巷模糊度
			double iono_L1_L2;
			//int  flagEdited;   // 用于周跳分析
			//double clock;      // 钟差

			PPP_CleanArcElement()
			{
				rw = 1.0f;
				oc = 0.0f;
				wL = 1.0f;
			}
		};

		// 需要弧段数据结构, 历元观测数据的构造需要弧段结构
		struct PPP_CleanArc 
		{
			string    nameSat;
			string    nameFreq;     // 便于查找频间差数据 
			double    ambiguity; 
			double    ambiguity_MW; 
			short int id_Ambiguity; 
			int       count; 
			int       markSlip;     // 0-新弧段; 1-周跳（未修复）; 2-周跳（已修复）
			int       slip_L1;
			int       slip_L2;
			double    freq_L1;      // 记录频率信息, 便于后面使用
			double    freq_L2;

			map<int, PPP_CleanArcElement> obsList;

			int  updateRobustWeight(vector<int> &slipIndexList, double threshold_slip, double threshold_rms, size_t threshold_count = 20, double ratio_ocEdit = 3, double max_ocPhaseEdit = DBL_MAX);
			bool backwordIonoPolyfit(double t_Extra, double &iono_Extra, int m = 3, double max_Span = 1200.0);
			bool forwordIonoPolyfit(double &iono_0, int m = 3, double max_Span = 1200.0);
		};

		// 用于求解法方程
		struct PPP_NEQElement
		{
			double  obscorrected_value;      // 观测数据改正量-相位
			//double  obscorrected_value_code; 
			POSCLK  vecLos;                  // 视线矢量, 单星定轨时使用
			float   Elevation;
			float   Azimuth;
			float   weightCode;              // 伪码观测权值
			float   weightPhase;             // 相位观测权值
			float   gmfh;                    // 对流层干分量映射函数（偏导数）
			float   gmfw;                    // 对流层湿分量映射函数（偏导数），用于对流层湿分量估计
			BYTE    mark_GNSSSatShadow;      // 进入地影标记, 默认 0 — 未进入地影
			bool    bEphemeris;
			bool    bEdited;
			
			PPP_NEQElement()
			{
				gmfh = 0.0f;
				gmfw = 0.0f;
				mark_GNSSSatShadow = 0;
				bEphemeris = false;
				bEdited = false;
			}
		};

		struct PPP_NEQEpoch
		{
			int                         validIndex;      // 有效时间标签  (用于相位观测方程设计矩阵索引有效位置参数号）
			int                         eyeableSatCount;
			// 运动学定位参数
			bool                        sppMark;         // 单点定位成功标记
			double                      pdop;            // 几何精度因子
			
			map<string, PPP_NEQElement> mapDatum;
		};

		struct PPP_SOLEpoch
		{
			// 静态部分
			GPST    t;
			double  clock;
			int     pppMark;         // 解标记: 0-插值获得, 未经过解算; 1-伪码解; 2-相位解
			double  pdop;  
            // 动态扩展部分
			POS3D   pos;
			int     eyeableSatCount; 
			double  sigma;
		};

		struct PPP_SOL
		{
			POS3D pos0; 
			POS3D posEst;  
			vector<PPP_SOLEpoch> solEpochEstList; 
			vector<TropZenithDelayEstPara>   zenithDelayEstList; // 对流层天顶延迟参数，每 2-6 小时估计 1 次
			vector<TropGradEstPara>         troGradEstList;     // 对流层水平梯度参数，每24小时估计 1 次,2014/10/28	
			map<string, double>             freqBiasList;       // 频间偏差参数, 根据频率区分, string参数为nameFreq
			int getIndex_ZenithDelayEstList(GPST t);
			int getIndex_TroGradEstList(GPST t);                 
		};

		struct SPP_SOLEpoch
		{
			GPST    t;
			POS3D   pos;       // 每一时刻一个位置
			double  clock;
			int     sppMark;  // 记录该时刻的单点定位完成情况, 0: 插值获得; 1: 有效解(伪码约束解)
			double  pdop;  
			double  sigma;
			int     eyeableSatCount;
		};

		struct SPP_SOL
		{
			vector<TropZenithDelayEstPara>   zenithDelayEstList; // 对流层天顶延迟参数，每 2-6 小时估计 1 次
			vector<TropGradEstPara>         troGradEstList;     // 对流层水平梯度参数，每24小时估计 1 次,2014/10/28	
			vector<SPP_SOLEpoch>            posclkEstList;                 // 钟差参数每个有效时刻估计一次
			int getIndexZenithDelayEstList(GPST t);
			int getIndexTroGradEstList(GPST t);                 // 2014/10/28
		};
		struct PPP_Freq
		{
			double                                  freq_L1;
			double                                  freq_L2;
			//string                                  nameFreq;   // 频率名称 "G" "C" "E" "Rxx"
		};

		struct PPP_MixedSys
		{
			// 需要初始化部分
			char                                    cSys;                         // 系统标识
			double                                  wSys;                         // 系统权值, 默认 1.0
			//char                                  recType;                      // 接收机类型，主要针对GPS数据
			string                                  name_C1;    // 伪码
			string                                  name_C2;
			string                                  name_L1;    // 相位
			string                                  name_L2;
			string                                  nameFreq_L1;                  // 用于PCV修正
            string                                  nameFreq_L2;
			PPP_Freq                                freqSys;
			map<string, PPP_Freq>                   freqSatList;                  // 主要针对GLONASS，string类型为卫星名
			float                                   priorsigma_PIF;               // 用于伪码加权控制

			// 不需要初始化部分
			int                                     index_C1;
			int                                     index_C2;
			int                                     index_L1;
			int                                     index_L2;
			int                                     iSys;                         // 记录数据在m_editedMixedObsFile存储的位置
			vector<Rinex3_03_EditedObsEpoch>        editedObsEpochList;           // 针对3_03混合格式修改, 谷德峰, 2018/06/14 
			vector<Rinex3_03_EditedObsSat>          editedObsSatlist;             // 针对3_03混合格式修改, 谷德峰, 2018/06/14 
			map<int, PPP_ObsEqEpoch>                P_IFEpochList;                // 参与解算的消电离层伪码数据（历元结构），换成map格式方便索引，int为nObsTime
			vector<PPP_CleanArc>                    L_IFArcList;                  // 参与解算的消电离层相位数据（弧段结构） 
			map<int, PPP_NEQEpoch>                  mapNEQEpochList;              // 历元法方程数据, 与editedObsEpochlist相对应，int为nObsTime
			int                                     id_Ambiguity_0;               // 模糊度起始编号
			map<string, double>                     mapSlipFixing_L1;             // 每颗卫星的周跳修复值
			map<string, double>                     mapSlipFixing_L2;
			map<string, int>                        mapObsCount;                  // 每颗卫星的观测数据个数

			string getSysFreqName()
			{
				// 针对3_03混合格式修改, 谷德峰, 2018/06/14 
				char sysFreqName[4];
				sprintf(sysFreqName, "%1c%1c%1c", cSys, name_L1[1], name_L2[1]); // "Gij" "Cij" "Eij" "Rxx"
				sysFreqName[3] = '\0';
				return sysFreqName;
			}

			// 注: oc残差部分, 伪码O-C残差在P_IFEpochList中, 相位O-C残差在L_IFArcList中
			PPP_MixedSys()
			{
				wSys = 1.0;
				//recType = 'N';
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
				priorsigma_PIF = 0.50f;
			}
		};

		struct PPP_DEF
		{
			bool                         on_KineSolving;        // 是否采用运动学
			bool                         on_RINEX_3_02;
			bool                         on_WeightElevation;
			bool                         on_WeightGEO;
			bool                         on_PhaseWindUp;
			bool                         on_SolidTides;
			bool                         on_OceanTides;
			bool                         on_GnssRelativity;
			bool                         on_GnssPCV;
			bool                         on_RecARP;
			bool                         on_RecPCV;
			bool                         on_GraRelativity;
			bool                         on_UsedInShadow;
			bool                         on_EstTropZenithDelay;
			bool                         on_EstTropGradient;
			bool                         on_SlipEditedInfo;
			float                        weightGEO;
			BYTE                         min_EyeableSatCount;   // 最小可视卫星个数
			double                       max_ArcInterval;
			size_t                       min_ArcPointCount;
			float                        min_Elevation;
			double                       max_ArcLengh;
			double                       max_WeightPDOP;         // 几何精度因子阈值(加权后), 超过该值的观测点将不参与运算, 运动学定位才使用
			double                       max_Sigma;              // 最终相位几何精度因子权值, 主要依赖相位
			double                       max_ocPhaseEdit;
			float                        weightInShadow;
			double                       period_WetTropZenithDelay;
			double                       period_TropGradient;
			double                       ratio_ocEdit;
			float                        priorsigma_LIF;         // 用于相位加权控制
			float                        priorsigma_TZD_abs;     // 先验对流层绝对约束
			float                        priorsigma_TZDGrad_abs;
			bool                         spp_sf; // 单频
			
			PPP_DEF()
			{
				spp_sf                    = false;
				on_KineSolving            = false;
				on_WeightElevation        = true;
				on_WeightGEO              = true;
				on_PhaseWindUp            = true;
				weightGEO                 = 1.0f / 3.0f;
				min_EyeableSatCount       = 2;
				if(on_KineSolving)
					min_EyeableSatCount   = 4;
				max_ArcInterval           = 3600.0;
				min_ArcPointCount         = 20;
				on_UsedInShadow           = false;
				weightInShadow            = 0.2f;
				on_EstTropZenithDelay     = true;
				period_WetTropZenithDelay = 3600.0 * 2;
				on_EstTropGradient        = true;
				period_TropGradient       = 3600.0 * 24;
				on_SolidTides             = true;
				on_OceanTides             = true;
				min_Elevation             = 5.0f;
				on_GnssRelativity         = true;
				on_GnssPCV                = true;
				on_RecARP                 = true;
				on_RecPCV                 = true;
				on_RINEX_3_02             = true;
				on_GraRelativity          = true;
				ratio_ocEdit              = 3.0;
				max_ocPhaseEdit           = DBL_MAX;
				on_SlipEditedInfo         = false;
				max_ArcLengh              = 3600.0;
				max_WeightPDOP            = 4.5;
				max_Sigma                 = 4.5;
				priorsigma_LIF            = 0.005f;
				priorsigma_TZD_abs        = 0.2f;   // 通过先验模型计算的对流层湿分量天顶延迟有可能超过10cm
				priorsigma_TZDGrad_abs    = 0.004f; // 梯度的估计量级较小一般不超过2mm
			}
		};

		// 多系统(BDS+GPS)、GLONASS频分多址，动态和静态能否统一? 相邻时刻参数的约束？
		class GNSSPPP
		{
		public:
			GNSSPPP(void);
		public:
			~GNSSPPP(void);
		public:
			void weighting_Elevation(float Elevation, float& weight_P_IF, float& weight_L_IF);
			bool pdopEpoch_MixedGNSS(double& pdop, vector<POSCLK> vecLosList, vector<float> wList);
			bool mainPPP_MixedGNSS(PPP_SOL& pppSol, string outputEditedObsFilePath = "");
			bool mainPPP_MixedGNSS_ParaElim(PPP_SOL& pppSol, string outputEditedObsFilePath = "");
			bool mainSPP_MixedGNSS(SPP_SOL& sppSol, string outputEditedObsFilePath = "", char cSystem = 'G');  // 多系统标准单点定位,依赖于数据预处理
			bool SinglePointPositioning_PIF(int index_P1, int index_P2, POSCLK& posclk, Rinex3_03_EditedObsEpoch obsEpoch, int& eyeableGPSCount, double& pdop, double& rms_res, double FREQUENCE_L1 = GPS_FREQUENCE_L1, double FREQUENCE_L2 = GPS_FREQUENCE_L2, char cSatSystem = 'G', double threshold = 1.0E-002);
			bool SinglePointPositioning_SF(int index_P1, POSCLK& posclk, Rinex3_03_EditedObsEpoch obsEpoch, int& eyeableGPSCount, double& pdop, double& rms_res, char cSatSystem = 'G', double threshold = 1.0E-002);

		public:
			JPLEphFile                   m_jplEphFile;         // JPL DE405星历数据文件
			TimeCoordConvert             m_timeCoordConvert;   // 时间坐标系转换
		public:
			PPP_DEF                      m_pppDefine;
			CLKFile                      m_clkFile;		       // 精密钟差数据文件
			SP3File                      m_sp3File;		       // 精密星历数据文件
            svnavMixedFile               m_svnavMixedFile;
			GNSSYawAttitudeModel         m_gymMixed;
			igs05atxFile			     m_atxFile;		       // 天线修正文件
			StaOceanTide                 m_staOceanTide;       // 海潮数据
			Rinex3_03_EditedObsFile      m_editedMixedObsFile; // 针对3_03混合格式修改, 谷德峰, 2018/06/14 
			vector<PPP_MixedSys>         m_dataMixedSysList;
		};
	}
}
