#pragma once
#include "SatdynBasic.hpp"
#include "MathAlgorithm.hpp"
#include "structDef.hpp"
#include "Rinex2_1_LeoEditedObsFile.hpp"
#include "Rinex2_1_LeoEditedSdObsFile.hpp"
#include "Rinex2_1_LeoMixedEditedSdObsFile.hpp"
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "svnavFile.hpp"
#include "TimeAttitudeFile.hpp"
#include "AntPCVFile.hpp"
#include "GPSLeoSatDynPOD.hpp"
#include "lambda.hpp"
#include "igs05atxFile.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace NUDTTK::Geodyn;
using namespace NUDTTK::SpaceborneGPSPreproc;
using namespace NUDTTK::SpaceborneGPSPod;
using namespace NUDTTK::Math;
using namespace NUDTTK::LAMBDA;
namespace NUDTTK
{
	namespace SpaceborneGPSProd
	{
		struct GPSLeoSatFormDynPRODPara
		{
			int                          max_OrbitIterativeNum;       // 轨道改进次数最大值
			double                       max_arclengh;    
			unsigned int                 min_arcpointcount;           // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			double                       min_elevation;
			int                          min_eyeableGPScount;         // 最小可视卫星个数, 个数小于 min_eyeableGPScount 的历元将不参与解算
			double                       apriorityRms_PIF;            // 先验无电离层码观测精度, 用于伪码和相位加权控制
			double                       apriorityRms_LIF;            // 先验无电离层相位观测精度, 用于伪码和相位加权控制
			double                       threhold_LAMBDA_ksb_MW;
			double                       threhold_LAMBDA_ksb_NA;
			bool                         bOn_GPSRelativity;           // 是否进行 GPS 卫星相对论改正
			bool                         bOn_RecRelativity;           // 接收机相对论修正
			bool                         bOn_GPSAntPCO;               // 是否进行 GPS 卫星天线偏移修正
			bool                         bOn_LEOAntPCO;               // 是否进行 LEO 卫星天线偏移修正
			bool                         bOn_LEOAntPCV;               // 是否进行 LEO 卫星天线相位中心修正, 只修正相对的PCV
			bool                         bOnEst_LEOAntRPCO_X;         // 是否进行 LEO 卫星 X 方向相对天线偏移估计
			bool                         bOnEst_LEOAntRPCO_Y;         // 是否进行 LEO 卫星 Y 方向相对天线偏移估计
			bool                         bOnEst_LEOAntRPCO_Z;         // 是否进行 LEO 卫星 Z 方向相对天线偏移估计
			double                       apriorityRms_LEOAntRPCO;     // 先验相对 PCO 精度, 用于伪方程约束
			bool                         bOn_WeightElevation;         // 是否进行高度角加权
			int                          ambiguityFixedType_MW;       // 宽巷模糊度固定方法: 0-利用历元双差关系; 1-单差弧段直接构造双差[推荐方法] 
			double                       max_var_MW;                  // 宽巷模糊度弧段方差阈值
			double                       threhold_MW_Decimal;         // 宽巷模糊度浮点解小数部分
			double                       threhold_NA_Decimal;         // 窄巷模糊度浮点解小数部分
			double                       threhold_MW_ReREFRatio;      // 宽巷参考模糊度更换比例
			bool                         bOn_DebugInfo_AmFixed;
			bool                         bOn_LEOAntMWV;

			GPSLeoSatFormDynPRODPara()
			{
				max_OrbitIterativeNum       = 5;
				max_arclengh                = 2000.0;
				min_arcpointcount           = 40;
				min_elevation               = 5.0;
				apriorityRms_PIF            = 0.5;
				apriorityRms_LIF            = 0.002;
				bOn_GPSRelativity           = true;
				bOn_RecRelativity           = true;
				bOn_GPSAntPCO               = true;
				bOn_LEOAntPCO               = true;
				bOn_LEOAntPCV               = true;
				bOnEst_LEOAntRPCO_X         = false;
				bOnEst_LEOAntRPCO_Y         = false;
				bOnEst_LEOAntRPCO_Z         = false;
				apriorityRms_LEOAntRPCO     = 1.0;
				bOn_WeightElevation         = false;
				min_eyeableGPScount         = 3;
                // 模糊度固定相关参数配置
				ambiguityFixedType_MW       = 1;    
				threhold_LAMBDA_ksb_MW      = 3.0;  
				threhold_LAMBDA_ksb_NA      = 5.0;
				max_var_MW                  = 0.5;  // 推荐值 GRACE 0.5 宽巷波长, 其他编队如果伪码噪声偏大, 需要适当放宽
				threhold_MW_Decimal         = 0.30; // 剔除部分双差浮点解偏离整数解过大的点, 减少模糊度固定错误概率 
				threhold_NA_Decimal         = 0.50; // 窄巷模糊度浮点解较为准确, 可适当放宽其条件
				bOn_DebugInfo_AmFixed       = false;

				/*ambiguityFixedType_MW     = 0;
				threhold_LAMBDA_ksb         = 2.5;*/
				threhold_MW_Decimal         = 0.40; // 兼顾原有1m软件测试使用, 适当放宽宽巷浮点解检测, 避免20060107 04:49:20-05:23:00 4个宽巷模糊度浮点解小数部分在0.3-0.4之间被删除
				threhold_MW_ReREFRatio      = 1.5;

				bOn_LEOAntMWV               = true;          
			}
		};
		struct MixedGNSSPRODDatum
		{
			char cSatSystem;                                              // 系统标记
			vector<Rinex2_1_LeoEditedSdObsEpoch>  editedObsEpochlist;     // 单系统单差历元格式观测数据, 非混合格式
			vector<Rinex2_1_EditedSdObsSat>       editedObsSatlist;       // 单系统单差卫星格式观测数据, 非混合格式
			map<int, PODEpoch>                    mapDynEpochList;   
			vector<AMBIGUITY_SECTION>             amSectionList;			
			int                                   ambiguityIndexBegin;		
			map<int, double>                      mapWindupPrev;          // 记录Windup修正数据
			vector<int>                           mixedEpochIdList;       // 记录当前历元序列在[混合格式]中的历元序号, 便于相互索引
			vector<int>                           epochIdList;            // 记录混合历元序列在[当前格式]中的历元序号
			vector<O_CResEpoch>                   m_ocResP_IFEpochList;   // 无电离层伪码O-C残差
			vector<O_CResEpoch>                   m_ocResL_IFEpochList;   // 无电离层相位O-C残差
			double   FREQUENCE_L1;
			double   FREQUENCE_L2;
			double   WAVELENGTH_L1;
			double   WAVELENGTH_L2;
			double   WAVELENGTH_W;
			double   WAVELENGTH_N;
			double   coeff_mw;
			double   coefficient_IF;
			double   coefficient_L1;
			double   coefficient_L2;
			int      index_P1;
			int      index_P2;
			int      index_L1;
			int      index_L2;
			double   weightSystem;                                        // 系统间的权系数之比, 默认为1.0
			double   sysBias;                                             // 系统间偏差，以GPS为参考系统，其值为0			
			
			MixedGNSSPRODDatum(double frequence1 = GPS_FREQUENCE_L1, double frequence2 = GPS_FREQUENCE_L2)
			{
				weightSystem   = 1.0;
				FREQUENCE_L1   = frequence1;
				FREQUENCE_L2   = frequence2;
				WAVELENGTH_L1  = SPEED_LIGHT / FREQUENCE_L1;
				WAVELENGTH_L2  = SPEED_LIGHT / FREQUENCE_L2;
				WAVELENGTH_W   = SPEED_LIGHT / (FREQUENCE_L1 - FREQUENCE_L2);
				WAVELENGTH_N   = SPEED_LIGHT / (FREQUENCE_L1 + FREQUENCE_L2);
				coeff_mw       = WAVELENGTH_L2 * pow(FREQUENCE_L2, 2) / (pow(FREQUENCE_L1, 2) - pow(FREQUENCE_L2, 2));
				coefficient_IF = 1 / (1 - pow(FREQUENCE_L1 / FREQUENCE_L2, 2));
				coefficient_L1 = 1 / (1 - pow(FREQUENCE_L2 / FREQUENCE_L1, 2)); 
				coefficient_L2 = 1 / (1 - pow(FREQUENCE_L1 / FREQUENCE_L2, 2)); 
				sysBias        = 0;				
			}
		};

		struct PROD_DD_AmbEq
		{
			int    id_DDREFAmbiguity_GL;
			int    id_Ambiguity_GL;
			double ambiguity_DD_MW;

			// 用于检核
			double ambiguity_DD_MW_Float;
		};

		class GPSLeoSatFormDynPROD
		{
		public:
			GPSLeoSatFormDynPROD(void);
		public:
			~GPSLeoSatFormDynPROD(void);
		public:
			bool loadSP3File(string  strSp3FileName);
			bool loadCLKFile(string  strCLKFileName);
			bool loadCLKFile_rinex304(string strCLKFileName);
			CLKFile getRelativeClkFile(); 
			bool mainFuncSdPreproc(Rinex2_1_LeoEditedSdObsFile& editedSdObsFile);
			bool mainFuncMixedSdPreproc(Rinex2_1_LeoMixedEditedSdObsFile& editedSdObsFile);
			void weighting_Elevation(double Elevation, double& weight_P_IF, double& weight_L_IF);
			bool lambdaSelected(Matrix matAFloat, Matrix matQahat, Matrix& matSqnorm, Matrix& matAFixed, Matrix matSelectedFlag);
			// 模糊度浮点解--动力学方法
			bool dynamicPROD_amfloat_IF(string editedSdObsFilePath, SatdynBasicDatum &dynamicDatum_A, SatdynBasicDatum &dynamicDatum_B, GPST t0_forecast, GPST t1_forecast, vector<TimePosVel> &forecastOrbList_A, vector<TimePosVel> &forecastOrbList_B, double interval = 30.0,  bool bForecast = true, bool bResEdit = true, bool bClkSolve = false);
			bool dynamicPROD_amfixed_mw_L1(string editedSdObsFilePath, SatdynBasicDatum &dynamicDatum_A, SatdynBasicDatum &dynamicDatum_B, GPST t0_forecast, GPST t1_forecast, vector<TimePosVel> &forecastOrbList_A, vector<TimePosVel> &forecastOrbList_B, double interval = 30.0,  bool bForecast = true, bool bResEdit = true, bool bClkSolve = false);
			// 模糊度固定解--动力学方法
			bool dynamicPROD_amfixed_mw(string editedSdObsFilePath, SatdynBasicDatum &dynamicDatum_A, SatdynBasicDatum &dynamicDatum_B, GPST t0_forecast, GPST t1_forecast, vector<TimePosVel> &forecastOrbList_A, vector<TimePosVel> &forecastOrbList_B, double interval = 30.0,  bool bForecast = true, bool bResEdit = true, bool bClkSolve = false);
            bool dynamicPROD_amfixed_prior(string editedSdObsFilePath, string editedSdClockFilePath, string priorOrbitFilePath_A, string priorOrbitFilePath_B, SatdynBasicDatum &dynamicDatum_A, SatdynBasicDatum &dynamicDatum_B, GPST t0_forecast, GPST t1_forecast, vector<TimePosVel> &forecastOrbList_A, vector<TimePosVel> &forecastOrbList_B, double interval = 30.0,  bool bForecast = true, bool bResEdit = true, bool bClkSolve = false);
			bool dynamicMixedPROD_amfixed_mw(string editedMixedSdObsFilePath, SatdynBasicDatum &dynamicDatum_A, SatdynBasicDatum &dynamicDatum_B, GPST t0_forecast, GPST t1_forecast, vector<TimePosVel> &forecastOrbList_A, vector<TimePosVel> &forecastOrbList_B, double interval = 30.0,  bool bForecast = true, bool bResEdit = true, bool bClkSolve = false);
		    // 研究测试函数
			bool dynamicPROD_amfixed_mw_varest(string editedSdObsFilePath, SatdynBasicDatum &dynamicDatum_A, SatdynBasicDatum &dynamicDatum_B, GPST t0_forecast, GPST t1_forecast, vector<TimePosVel> &forecastOrbList_A, vector<TimePosVel> &forecastOrbList_B, double interval = 30.0,  bool bForecast = true, bool bResEdit = true, bool bClkSolve = false);
		    AntPCVFile                  m_mwvFile;   // mw残差中心
			vector<O_CResArc>           m_mwResArcList_amfloat;
			vector<O_CResArc>           m_mwResArcList_amfixed;
		private:
			bool loadEditedSdObsFile(string  strEditedSdObsFileName);
			bool loadMixedEditedSdObsFile(string  strMixedEditedSdObsFileName);
			CLKFile m_recRelativeClkFile;                    // 精密相对钟差解算结果
		public:
			GPSLeoSatDynPOD             m_dynPOD;
			GPSLeoSatFormDynPRODPara    m_prodParaDefine;
			svnavFile                   m_svnavFile;         // GPS天线偏移
			igs05atxFile			    m_AtxFile;		     // 天线修正文件(2013/04/18, 鞠冰)
			CLKFile                     m_clkFile;           // 精密钟差数据文件
			SP3File                     m_sp3File;           // 精密星历数据文件
			Rinex2_1_LeoEditedObsFile   m_editedObsFile_A;   // A卫星原始观测数据
			POS3D                       m_pcoAnt_A;          // A卫星天线偏心量
			Matrix                      m_matAxisBody2RTN_A; // 星固系到轨道系的固定准换矩阵, 用于存在固定偏差角度的三周稳定控制, 2015/03/09
			TimeAttitudeFile            m_attFile_A;         // A卫星姿态文件
			AntPCVFile                  m_pcvFile_A;         // A卫星天线相位中心
			Rinex2_1_LeoEditedObsFile   m_editedObsFile_B;   // B卫星原始观测数据
			POS3D                       m_pcoAnt_B;          // B卫星天线偏心量
			Matrix                      m_matAxisBody2RTN_B; // 星固系到轨道系的固定准换矩阵, 用于存在固定偏差角度的三周稳定控制, 2015/03/09
			TimeAttitudeFile            m_attFile_B;         // B卫星姿态文件
			AntPCVFile                  m_pcvFile_B;         // B卫星天线相位中心
			Rinex2_1_LeoEditedSdObsFile m_editedSdObsFile;
			vector<O_CResEpoch>         m_ocResP_IFEpochList;
			vector<O_CResEpoch>         m_ocResL_IFEpochList;
			// Mixed 混合观测数据相关变量
			Rinex2_1_LeoMixedEditedObsFile   m_editedMixedObsFile_A;   // A卫星混合格式原始观测数据
			Rinex2_1_LeoMixedEditedObsFile   m_editedMixedObsFile_B;   // B卫星混合格式原始观测数据
			Rinex2_1_LeoMixedEditedSdObsFile m_editedMixedSdObsFile;   // A、B卫星混合格式单差观测数据
			vector<MixedGNSSPRODDatum>       m_dataMixedGNSSlist;  
		public:
			POS3D                       m_pcoAntEst;         // 天线相对偏心量估计结果
		};
	}
}

