#pragma once
#include"SatdynBasic.hpp"
#include"constDef.hpp"
#include"structDef.hpp"
#include"MathAlgorithm.hpp"
#include"Rinex2_1_EditedObsFile.hpp"
#include"Rinex2_1_EditedSdObsFile.hpp"
#include"dynPODStructDef.hpp"
#include"lambda.hpp"
#include"igs05atxFile.hpp"
#include"SP3file.hpp"
#include<windows.h>
#include<map>
#include"StaOceanLoadingDisplacementFile.hpp"
#include"svnavFile.hpp"
#include"CLKfile.hpp"
#include"TROZPDFile.hpp"
#include"Troposphere_model.hpp"
#include "GPSYawAttitudeModel1995.hpp"
#include "svnavMixedFile.hpp"
#include "GNSSYawAttitudeModel.hpp"

using namespace NUDTTK;
using namespace NUDTTK::Geodyn;
using namespace NUDTTK::Math;
using namespace NUDTTK::LAMBDA;
//  Copyright 2012, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	namespace GPSPod
	{
		// 整网定轨参数结构
		struct GPSMeoSatDynPODPara
		{
			int                          max_OrbitIterativeNum;     // 轨道改进次数最大值		
			unsigned int                 min_arcpointcount;         // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			int                          min_eyeableGNSScount;      // 单历元最小可视卫星个数, 个数小于 min_eyeableGPScount 的历元将不参与解算
			double                       min_elevation;
			double                       period_SolarPressure;  
			double                       apriorityRms_PIF;          // 先验无电离层码观测精度, 用于伪码和相位加权控制
			double                       apriorityRms_LIF;          // 先验无电离层相位观测精度, 用于伪码和相位加权控制
			double                       apriorityRms_STA;          // 先验测站位置坐标精度, 用于测站坐标约束方程加权控制
			double                       apriorityRms_TZD_abs;      // 先验测站电离层天顶延迟精度, 用于电离层天顶延迟绝对约束方程加权控制
			double                       apriorityRms_TZD_rel;      // 用于相邻电离层天顶延迟参数的相对约束方程加权控制
			double                       apriorityWet_TZD;          // 测站对流层湿分量估计先验值
			double                       apriorityWet_TZD_period;   // 测站对流层湿分量估计周期
			double                       min_Wet_TZD_ncount;        // 对流层估计所需的最少数据个数
			double                       max_arclengh;    
			bool                         bOn_AmbiguityFix;          // 是否固定整周模糊度
			bool                         bOn_StaSolidTideCor;       // 是否进行测站坐标的固体潮改正
			bool                         bOn_GPSRelativity;         // 是否进行 GPS 卫星相对论改正,轨道偏心改正
			bool                         bOn_GraRelativity;         // 是否进行引力引起的相对论改正
			bool                         bOn_GPSAntPCOPCV;          // 是否进行 GPS 卫星天线 PCO/PCV 修正
			bool                         bOn_RecAntARP;             // 是否进行接收机天线 ARP 修正
			bool                         bOn_RecAntPCOPCV;          // 是否进行接收机天线 PCO/PCV 修正
			bool                         bOn_PhaseWindUp;
			bool						 bOn_TropDelayCorrection;	// 是否进行对流层延迟修正(经验模型修正)
			bool                         bOnEst_StaPos;             // 是否进行测站位置改进
			bool                         bOnEst_StaTropZenithDelay; // 是否进行测站对流层天顶延迟估计
			bool                         bOn_StaOceanTides;
			bool                         bOnEst_ERP;                // 是否进行地球旋转参数估计
			bool                         bOn_WeightElevation;       // 是否进行高度角加权
			bool                         bOn_GYM95;                 // 是否使用GYM95模型获取卫星姿态，用于相位缠绕和卫星天线相位中心修正
			bool                         bOn_Used_delta_u;          // 是否使用Delta_u作为太阳光压的输入
			TYPE_SOLARPRESSURE_MODEL     solarPressure_Model;       // 太阳光压模型
			double                       maxdistance_Ambfixed_short;// 短基线长度阈值, 用于初始模糊度不固定
			double                       maxdistance_Ambfixed_long; // 长基线长度阈值, 大于该长度的基线模糊度不固定
			double                       threshold_slip_DDDEdit;
			double                       threshold_outlier_DDDEdit;
			double                       robustfactor_OC_edited;
			bool                         bOnConstraints_GPSEphemeris; 
			double                       apriorityRms_GPSEphemeris; // GPS先验星历的精度
			double                       max_var_MW;
			double                       threhold_LAMBDA_ksb_MW;
			double                       threhold_LAMBDA_ksb_NA;
			double                       threhold_MW_Decimal;         // 宽巷模糊度浮点解小数部分
			double                       threhold_NA_Decimal;         // 窄巷模糊度浮点解小数部分
			double                       min_arclengh_min_5;          // 最小弧段长度
			bool                         bOn_DebugInfo_AmFixed;
			bool                         bOnConstraints_SolarPressure4; 
			double                       apriorityRms_SolarPressure4;
			int                          sampleSpan;
			int                          min_countobs_clkest;        // 钟差估计时, 每个测站和卫星钟对应的最小观测数据个数
			int                          zpdProductInterval;         // 输出的对流层产品采样间隔(s)
			//////////////////////////////////////////////////////////此部分参数由刘俊宏增加用于兼容北斗,2014/09/29					
			int                          OrbitIterativeNum;          // 在不固定模糊度的情况下，控制迭代次数
			bool                         bOn_StaWeight;              // 是否根据测站加权
			bool                         bOn_SatWeight;              // 是否根据卫星加权
			bool                         bOn_SDEdited;               // 是否进行单差编辑
			bool                         bOn_GEOSolut;               // 是否解算GEO卫星			
			bool                         bOn_GPSzpd;                 // 是否使用GPS估计的对流层参数
			bool                         bOn_ArcEdited;              // 是否需要根据调整弧段的权值
			bool                         bOn_RefSat;
			bool                         bOn_GEOSRP_DC1;              // 是否估计GEO卫星光压DC1参数
			bool                         bOn_GEOSRP_DS1;              // 是否估计GEO卫星光压DS1参数			
			bool                         bOn_GEOSRP_YC1;              // 是否估计GEO卫星光压YC1参数
			bool                         bOn_GEOSRP_YS1;              // 是否估计GEO卫星光压YS1参数
			bool                         bOn_GEOSRP_BC1;              // 是否估计GEO卫星光压BC1参数
			bool                         bOn_GEOSRP_BS1;              // 是否估计GEO卫星光压BS1参数	
			TYPE_SOLARPRESSURE_MODEL     solarPressure_Model_BDYF;       // yaw-fixed姿态太阳光压模型，2015/11/13，刘俊宏	
			TYPE_SOLARPRESSURE_MODEL     solarPressure_Model_BDYS;       // yaw-steering姿态太阳光压模型
			///////////////////////////////////////////////////////////
			vector<string>               clkFixedStaNameList;        // 指定钟差参考站列表, 谷德峰, 2015/09/06  
			// ***********************************************************************************************
			bool                         bOn_EstSatPCO_X;               // 卫星PCO估计开关,邵凯，2019.09.25
			bool                         bOn_EstSatPCO_Y;               
			bool                         bOn_EstSatPCO_Z;               
			bool                         bOn_EstSatPCV;                 // 卫星PCV估计开关
			TYPE_OCEANTIDE_MODEL         oceanTideType_Model;        // 海潮类型
			bool                         bOn_BDSYawAttitude;         // 是否使用北斗偏航姿态模型获取卫星姿态，用于相位缠绕和卫星天线相位中心修正
			bool                         on_UsedInShadow;            // 是否使用地影期间数据
		    double                       weightInShadow;              // 地影期间数据降权处理
			// **********************************************************************************************8
			double                       period_EarthIrradiance;      // 地球辐射, 2017/10/03
			TYPE_EARTHRADIATION_MODEL    earthRadiation_Model;        // 地球反照辐射压模型
			TYPE_SATELLITESHAPE_MODEL    satelliteShape_Model;        // 卫星形状模型
			bool                         bOn_earthRadiation;          // 是否估计地球反照辐射开关
			bool                         bOn_earthRadiation_noest;          // 是否计算地球反照辐射开关
			// ***********************************************************************************************

			GPSMeoSatDynPODPara()
			{
				max_OrbitIterativeNum     = 10;				
				min_arcpointcount         = 12;
				min_eyeableGNSScount      = 3;
				min_elevation             = 7.0;
				period_SolarPressure      = 3600 * 24.0;
				apriorityRms_PIF          = 1.0;
				apriorityRms_LIF          = 0.01;
				apriorityRms_STA          = 1.0;			
				apriorityRms_TZD_abs      = 0.5;
				apriorityRms_TZD_rel      = 0.05;
				apriorityWet_TZD          = 0;                      // 测站对流层湿分量估计先验值
				apriorityWet_TZD_period   = 3600 * 2;               // 周期                
				min_Wet_TZD_ncount        = 12;
				max_arclengh              = 3600;

				bOn_AmbiguityFix          = true;
				min_arclengh_min_5        = 60;  // 1小时
				threhold_LAMBDA_ksb_MW    = 3.0;
				threhold_LAMBDA_ksb_NA    = 3.0;
				max_var_MW                = 2.0;  
				threhold_MW_Decimal       = 0.30; // 剔除部分双差浮点解偏离整数解过大的点, 减少模糊度固定错误概率 
				threhold_NA_Decimal       = 0.40; // 窄巷模糊度浮点解较为准确, 可适当放宽其条件
				bOn_DebugInfo_AmFixed     = false;
				min_countobs_clkest       = 1;

				bOn_GPSRelativity         = true;
				bOn_GraRelativity         = false;
				bOn_GPSAntPCOPCV          = true;
				bOn_RecAntARP             = true;
				bOn_RecAntPCOPCV          = true;
				bOn_PhaseWindUp           = true;
				bOn_TropDelayCorrection   = true;
				bOnEst_StaPos             = true;
				bOnEst_StaTropZenithDelay = true;
				bOnEst_ERP                = true;
				bOn_StaSolidTideCor       = true;
				bOn_StaOceanTides         = true;
				bOn_WeightElevation       = false;
				bOn_GYM95                 = false;
				bOn_Used_delta_u          = true;
				solarPressure_Model       = TYPE_SOLARPRESSURE_9PARA;

				maxdistance_Ambfixed_short= 2500000.0;
				maxdistance_Ambfixed_long = 5000000.0;
				threshold_slip_DDDEdit    = 0.15;
				threshold_outlier_DDDEdit = 0.15;
				robustfactor_OC_edited    = 2.5;
				bOnConstraints_GPSEphemeris = true;
				apriorityRms_GPSEphemeris   = 0.30;
				bOnConstraints_SolarPressure4 = false;
				apriorityRms_SolarPressure4   = 1.0E-8;
				sampleSpan                  = 120;
				zpdProductInterval          = 3600 * 2;				
				OrbitIterativeNum           = 3;	 // 迭代多次？？？
				
				bOn_StaWeight               = false;
				bOn_SatWeight               = false;
				bOn_SDEdited                = false;
				bOn_GEOSolut                = true;
				bOn_GPSzpd                  = false;
				bOn_ArcEdited               = false;
				bOn_RefSat                  = false;
				bOn_GEOSRP_DC1              = false;
				bOn_GEOSRP_DS1              = true;
				bOn_GEOSRP_YC1              = false;
				bOn_GEOSRP_YS1              = false;	
				bOn_GEOSRP_BC1              = true;
				bOn_GEOSRP_BS1              = false;		
				solarPressure_Model_BDYF    = TYPE_SOLARPRESSURE_9PARA;
				solarPressure_Model_BDYS    = TYPE_SOLARPRESSURE_9PARA;
				bOn_EstSatPCO_X                = false;
				bOn_EstSatPCO_Y                = false;
				bOn_EstSatPCO_Z                = false;
				bOn_EstSatPCV                  = false;
				oceanTideType_Model         = TYPE_OCEANTIDE_CSR4;
				bOn_BDSYawAttitude          = false;
				on_UsedInShadow             = true;
				weightInShadow              = 0.2;
				earthRadiation_Model        = TYPE_EARTHRADIATION_ANALYTICAL;
				satelliteShape_Model        = TYPE_SATELLITESHAPE_BALL;
				period_EarthIrradiance      = 3600 * 24.0;
				bOn_earthRadiation          = false;
				bOn_earthRadiation_noest    = true;
			}
		};

		// 钟差数据结构
		struct CLKEST_ObsElement
		{
			string          name_Sta;
			int             id_Sat;
			int             id_Ambiguity;
			int             id_Arc;
			int             nObsTime;
			double          distance; // 几何距离, 修正对流层, 偏心, 地球旋转影响
			double          obs_L_IF;
			double          weight_L_IF;
			double          obs_P_IF;
			double          weight_P_IF;
			double          ambiguity;
			double          oc_P_IF;
			double          oc_L_IF;
			//double          nadir; // 天底角
			// 2016/01/11, 增加对流层估计, 谷德峰
			int             id_zenithDelay_0;     
			double          zenithDelay;           // 修正值
			double          zenithDelay_partial_0; // 偏导数
			double          zenithDelay_partial_1; 
		};
		// 钟差估计历元数据结构
		struct CLKEST_Epoch
		{
			vector<CLKEST_ObsElement> obsList;
			string                    name_FixedSta;        // 主站名称
			GPST                      t;                    // 历元时刻
			bool                      bSuccess;
			vector<int>               id_SatList;           // 有效卫星列表
			vector<double>            clkBias_SatList;      // 卫星钟差估计结果列表
			vector<double>            clkBiasSigma_SatList; // 卫星钟差估计结果列表
			vector<string>            name_StaList;         // 有效测站列表
			vector<double>            clkBias_StaList;      // 测站钟差估计结果列表
			vector<double>            clkBiasSigma_StaList; // 测站钟差估计结果列表
			Matrix                    matN_c;               // 行列数固定, 供5min采样率可使用, 30sec采样率不用节约内存
			Matrix                    matN_cc_inv;          // 行列数固定, 供5min采样率可使用, 30sec采样率不用节约内存
			Matrix                    matN_cb;              // 行列数固定, 供5min采样率可使用, 30sec采样率不用节约内存
			Matrix                    matdc;
			vector<int>               indexAmbList;         // 记录每个观测数据对应的模糊度序号
			map<int, int>             mapIdSat;
			map<string, int>          mapIdSta;             // 不含参考站数据
			map<string, int>          mapIndexZenithDelay;  // 记录每个测站对应的天顶对流层延迟序号, 至多 2 个, mapIndexZenithDelay[name] 和 mapIndexZenithDelay[name] + 1
			//map<int, int>             mapIndexSatDyn;       // 记录每颗卫星对应的动力学参数起点
			//map<int, int>             mapIndexSatDyn;       // 记录每颗卫星对应的PCOPCV参数起点
			int                       k0_SatEphemeris;      // 力学参数起始位置
			int                       k0_SatPCOPCV;         // PCOPCV参数起始位置
			int                       k0_ERP;			
			bool create_Sat_Sta_List(string name_FixedSta, int count_obs_Min = 1); // 根据 obsList 生成id_SatList_t和name_StaList_t
			CLKEST_Epoch()
			{
				name_FixedSta = "";
				bSuccess = false;
			}
		}; 
		// 钟差估计参数结构
		struct CLKEST_Parameter
		{
			map<int, int>         mapIndexSat;      // 有效卫星索引列表
			map<string, int>      mapIndexSta;      // 有效测站索引列表
			vector<CLKEST_Epoch>  clkEpochList_5min;
			vector<CLKEST_Epoch>  clkEpochList;

			Matrix                matN_bb;          // 模糊度、对流层
			Matrix                matN_b;
			Matrix                matdb;
		};

		typedef struct {int valid; int id_arc; double obscorrected_value;} cvElement; // correctedvalueElement
		typedef struct {map<int, cvElement> mapDatum;} cvEpoch;   // correctedvalueEpoch

        struct NETEST_Parameter
		{
			string             pathSp3File_input;    // 先验sp3轨道文件, 非空
			string             pathSp3File_output;   // 输出sp3轨道文件
			string             pathTroFile_input;    // 先验对流层文件
			string             pathTroFile_output;   // 输出对流层文件
			string             pathSinexFile_input;  // 先验测站解文件
			string             pathSinexFile_output; // 输出测站解文件
			SatOrbEstParameter paraSatOrbEst;        // 轨道估计结果
			CLKEST_Parameter   paraClkEst;           // 钟差估计结果
			GPST               t0;
			GPST               t1;                   
			double             h_sp3;                // 轨道间隔
			double             h_clk;                // 钟差间隔

			NETEST_Parameter()
			{
				h_sp3 = 900.0;
				h_clk = 300.0;
			};

		};

		struct CLKEST_AmbFixElement
		{   
			int    nObsTime;      // 观测时间序号, 与降采样中的数据时刻一致
			BYTE   id_Sat;        // 卫星号
			int    id_Ambiguity;
			double ambiguity_mw;  // 宽巷模糊度观测值
			double ambiguity_IF;  // 消电离层组合模糊度序号
			int    id_Arc_A;          // 非差索引需要
			int    id_Arc_B;          // 非差索引需要
			int    id_Ambiguity_GL_A; // 非差索引需要
			int    id_Ambiguity_GL_B; // 非差索引需要
			
			CLKEST_AmbFixElement()
			{
			}
		};

		typedef map<int, CLKEST_AmbFixElement> CLKEST_AmbFixElementMap;

		struct CLKEST_AmbFixArc 
		{
			BYTE                    id_Sat;   
			int                     id_Ambiguity;
			double                  ambiguity_mw;
			double                  ambiguity_IF;
			int                     id_Ambiguity_GL_A;   // 非差索引需要
			int                     id_Ambiguity_GL_B;   // 非差索引需要
			CLKEST_AmbFixElementMap obsList;
		};

		struct CLKEST_AmbFixEpoch 
		{
			int                          nObsTime;             // 观测时间序号
			int                          id_DDObs;             // 双差GPS参考卫星的观测数据序号
			int                          id_DDRefSat;          // 双差GPS参考卫星序号
			int                          id_DDRefAmbiguity;    // 双差参考模糊度序号
			int                          id_DDRefAmbiguity_GL_A;  // 非差索引需要
			int                          id_DDRefAmbiguity_GL_B;  // 非差索引需要
			vector<CLKEST_AmbFixElement> obsSatList;        
			
			CLKEST_AmbFixEpoch()
			{
				id_DDRefSat       =  NULL_PRN;
				id_DDRefAmbiguity = -1;
				id_DDObs          = -1;
			}
		};

		struct CLKEST_AMBIGUITY_SECTION
		{
			int                          id_t0;                        // 起始时刻
			int                          id_t1;                        // 结束时刻
			vector<CLKEST_AmbFixArc>     mw_ArcList;                   // 记录分段后弧段数据
			vector<CLKEST_AmbFixEpoch>   mw_EpochList;                 // 记录分段后时刻数据,  构造双差时需要根据时刻进行
			bool                         bDDAmFixed_MW;                // 标记宽巷模糊度整体固定是否成功
			Matrix                       matDDFixedFlag_MW;            // 标记每个宽巷模糊度固定是否成功, 0: 未固定, 1: 已固定
			vector<double>               ambiguity_DD_MW_List;         // 每个宽巷模糊度固定结果, 未固定的采用浮点解
			vector<double>               ambiguity_DD_MW_Float_List;   // 每个宽巷模糊度浮点解
			bool                         bDDAmFixed_NA;                // 标记窄巷模糊度整体固定是否成功
			Matrix                       matDDFixedFlag_NA;            // 标记每个窄巷模糊度固定是否成功
			vector<double>               ambiguity_DD_NA_List; 
			vector<double>               ambiguity_DD_NA_Float_List;   // 每个窄巷模糊度浮点解

			bool ArcList2EpochList(vector<CLKEST_AmbFixArc>& arcList, vector<CLKEST_AmbFixEpoch>& epochList, int min_arcpointcount = 20, bool bOn_Loop = true);
		};

		struct CLKEST_DD_AmbEq
		{
			int    id_DDRefAmbiguity_GL_A;
			int    id_DDRefAmbiguity_GL_B;
			int    id_Ambiguity_GL_A;
			int    id_Ambiguity_GL_B;
			double ambiguity_DD_IF;
			// 用于检核
			double ambiguity_DD_MW;
			double ambiguity_DD_NA;
			double ambiguity_DD_MW_Float;
			double ambiguity_DD_NA_Float;
			string nameSta_A;
			string nameSta_B;
			int    id_REFSat;
			int    id_Sat;
		};

		struct CLKEST_FCBs_Eq
		{
			string staName;
			double mwFCBs;
			double naFCBs;
			int    SD_MW_INT;
			int    SD_NA_INT;
			double mwFCBs_cos; // 三角函数变换后的FCBs
			double mwFCBs_sin;
			double naFCBs_cos;
			double naFCBs_sin;
			bool   bUsed_mw;   // 求均值数据使用标记
			bool   bUsed_na;
		};

		struct CLKEST_FCBs_Sat
		{
			int                     id_Sat_REF;
			int                     id_Sat;
			vector<CLKEST_FCBs_Eq>  eqFCBsList;
			double                  mwFCBs_mean; // 平均宽巷FCBs
			double                  mwFCBs_var;
			double                  naFCBs_mean; // 平均窄巷FCBs
			double                  naFCBs_var;

			void robustStatMeanFCBs_mw(); // 计算平均宽巷FCBs及其标准差, 并将每个弧段的宽巷FCBs连续化
			void robustStatMeanFCBs_na(); // 计算平均窄巷FCBs及其标准差, 并将每个弧段的窄巷FCBs连续化
		};

		// GPS 整网精密定轨类(SatdynBasic的派生类)
		class GPSMeoSatDynPOD : public SatdynBasic
		{
		public:
			GPSMeoSatDynPOD(void);			
		public:
			~GPSMeoSatDynPOD(void);
		private:
			void weighting_Elevation(double Elevation, double& weight_P_IF, double& weight_L_IF);
			bool adamsCowell_ac(TDT t0_Interp, TDT t1_Interp, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 75.0, int q = 11);			
		public:
			static bool getStaBaselineList_MiniPath(vector<POS3D> staList, vector<int>& staBaseLineIdList_A, vector<int>& staBaseLineIdList_B);
			static bool generateEditedSdObsFile(Rinex2_1_EditedObsFile editedObsFile_A, Rinex2_1_EditedObsFile editedObsFile_B, Rinex2_1_EditedSdObsFile& editedSdObsFile);
			static bool lambdaSelected(Matrix matAFloat, Matrix matQahat, Matrix& matSqnorm, Matrix& matAFixed, Matrix matSelectedFlag);
		public:	
			bool obsTriDiFFEdited_LIF(int index_L1, int index_L2, int id_sat, Rinex2_1_EditedSdObsEpoch epoch_j_1, Rinex2_1_EditedSdObsEpoch epoch_j, map<int, cvElement> &mapCvDatum_j_1, map<int, cvElement> &mapCvDatum_j, double &max_res_ddd, bool &slipFlag, double threshold_slip = 0.15);
			bool mainTriDiFFEdited(SP3File &sp3File, Rinex2_1_EditedSdObsFile &editedSdObsFile, POS3D posAnt_A, POS3D posAnt_B, string outputFileName = "");
			bool mainSDEpochDiffEdited(SP3File &sp3File, Rinex2_1_EditedSdObsFile &editedSdObsFile, POS3D posAnt_A, POS3D posAnt_B, string outputFileName = "");
			bool sp3Fit(string strSp3FilePath, Sp3FitParameter& paraSp3Fit, bool bOnEst_EOP = true, string outputSp3FitFilePath = "");
			bool mainNetPod_dd(string inputSp3FilePath, string outputSp3FilePath, SatOrbEstParameter& paraSatOrbEst, GPST t0, GPST t1, double h_sp3 = 900.0);
			bool mainClkEst_iterator(string inputSp3FilePath, string inputTrozpdFilePath, string inputSinexFilePath, string outputClkFilePath, CLKEST_Parameter &paraClkEst, GPST t0, GPST t1, double h_clk = 30.0, bool bUsed_Phase = true);
			bool mainClkEst_zd(string inputSp3FilePath, string inputTrozpdFilePath, string inputSinexFilePath, string outputClkFilePath, CLKEST_Parameter &paraClkEst, GPST t0, GPST t1, double h_clk = 30.0, string outputTrozpdFilePath = "", string outputSp3FilePath = "", double h_sp3 = 900.0);
			//// PCO+PCV估计，2019/09/24, 邵凯
			//bool mainClkEst_zd_pcopcv(string inputSp3FilePath, string inputTrozpdFilePath, string inputSinexFilePath, string outputClkFilePath, CLKEST_Parameter &paraClkEst, GPST t0, GPST t1, double h_clk = 30.0, string outputTrozpdFilePath = "", string outputSp3FilePath = "", double h_sp3 = 900.0);
			// 钟差估计
			bool mainClkEst_Gamit(string inputSp3FilePath, string outputClkFilePath, CLKEST_Parameter &paraClkEst, GPST t0, GPST t1, double h_clk = 300.0);
			// 测试代码
			bool initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength = 3600 * 3);
			bool dynamicPOD_pos(vector<TimePosVel>  orbitlist, SatdynBasicDatum &dynamicDatum, GPST t_forecastBegin, GPST t_forecastEnd,  vector<TimePosVel> &orbitlist_forecast, double interval = 30.0, bool bforecast = true);
		public:
			GPSMeoSatDynPODPara             m_podParaDefine;
			map<int, AntCorrectionBlk>      m_mapGPSAntCorrectionBlk;
			StaDatumMap                     m_mapStaDatum;
			vector<StaBaselineDatum>        m_staBaseLineList;
			svnavFile                       m_svnavFile;	      // GPS天线偏移
			igs05atxFile			        m_AtxFile;			  // 天线修正文件
			StaOceanLoadingDisplacementFile m_staOldFile;         // 海潮文件
			GPSYawAttitudeModel1995         m_GYM95;              // 获取姿态控制模式下的卫星姿态
			// 邵凯，2019/09/30，新模型文件
			svnavMixedFile                  m_svnavMixedFile;
			GNSSYawAttitudeModel            m_gymMixed;						
			map<string, AntCorrectionBlk>   m_mapGnssPCVBlk;// 获取卫星PCV信息

		public: //此部分参数由刘俊宏增加用于兼容北斗,2014/09/29
			char                            cSatSystem;	
			double                          FREQUENCE_L1;
			double                          FREQUENCE_L2;
			bool                            getFREQUENCE();
		};
	}
}
;