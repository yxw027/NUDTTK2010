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
#include"TROZPDFile.hpp"
#include"StaOceanLoadingDisplacementFile.hpp"
#include"OceanTidesLoading.hpp"
#include"Troposphere_model.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace NUDTTK::Geodyn;
using namespace NUDTTK::Math;
using namespace NUDTTK::LAMBDA;
namespace NUDTTK
{
	namespace BDPod
	{
		struct BDSatDynPODPara
		{
			int                          max_OrbitIterativeNum;     // 轨道改进次数最大值		
			unsigned int                 min_arcpointcount;         // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			int                          min_eyeableGPSCount;         // 单历元最少卫星个数
			TYPE_SOLARPRESSURE_MODEL     solarPressure_Model;       // 太阳光压模型
			double                       period_SolarPressure;      // 太阳光压周期
			double                       max_FitRms_Total;
			double                       min_elevation;
			double                       apriorityRms_PIF;          // 先验无电离层码观测精度, 用于伪码和相位加权控制
			double                       apriorityRms_LIF;          // 先验无电离层相位观测精度, 用于伪码和相位加权控制
			double                       apriorityRms_STA;          // 先验测站位置坐标精度, 用于测站坐标约束方程加权控制
			double                       apriorityRms_TZD_abs;      // 先验测站电离层天顶延迟精度, 用于电离层天顶延迟绝对约束方程加权控制
			double                       apriorityRms_TZD_rel;      // 用于相邻电离层天顶延迟参数的相对约束方程加权控制
			double                       apriorityWet_TZD;          // 测站对流层湿分量估计先验值
			double                       apriorityWet_TZD_period;   // 测站对流层湿分量估计周期
			double                       min_Wet_TZD_ncount;        // 对流层估计所需的最少数据个数
			double                       max_arclengh;    
			double                       threhold_LAMBDA_ksb;
			bool                         bOn_AmbiguityFix;          // 是否固定窄巷整周模糊度
			bool                         bOn_StaSolidTideCor;       // 是否进行测站坐标的固体潮改正
			bool                         bOn_StaOceanTides;         // 是否进行测站坐标的海潮改正
			bool                         bOn_BDTroPriModCor;        // 是否进行对流层先验模型修正
			bool                         bOn_BDSRelativity;         // 是否进行 BD 卫星相对论改正,轨道偏心改正
			bool                         bOn_BDSGraRelativity;      // 是否进行引力引起的相对论改正
			bool                         bOn_RecAntARP;             // 是否进行接收机天线 ARP 修正
			bool                         bOn_BDSAntPCO;             // 是否进行 BD 卫星天线偏移修正
			bool                         bOn_RecAntPCOPCV;          // 是否进行接收机天线 PCO/PCV 修正
			bool                         bOn_PhaseWindUp;
			bool                         bOnEst_StaPos;             // 是否进行测站位置改进
			bool                         bOnEst_StaTropZenithDelay; // 是否进行测站对流层天顶延迟估计
			bool                         bOnEst_ERP;                // 是否进行地球旋转参数估计
			bool                         bOn_WeightElevation;       // 是否对高度角进行加权
			bool                         bOn_DDDEdit;               // 是否进行三差数据编辑
			double                       threshold_slip_DDDEdit;    // 三差数据编辑周跳探测阈值
			double                       threshold_outlier_DDDEdit; // 三差数据编辑野值探测阈值
			double                       maxdistance_Ambfixed_short;// 短基线长度阈值, 用于初始模糊度不固定
			double                       maxdistance_Ambfixed_long; // 长基线长度阈值, 大于该长度的基线模糊度不固定
			double                       robustfactor_OC_edited;
			double                       bOnConstraints_GPSEphemeris; 
			double                       apriorityRms_GPSEphemeris; // GPS先验星历的精度
			int                          sampleSpan;                // 输入数据的采样间隔(s)
			int                          zpdProductInterval;        // 输出的对流层产品采样间隔(s)
			int                          OrbitIterativeNum;          //在不固定模糊度的情况下，控制迭代次数
		

			BDSatDynPODPara()
			{
				max_OrbitIterativeNum     	= 8;				
				min_arcpointcount         	= 10;
				min_eyeableGPSCount       	= 3;
				solarPressure_Model       	= TYPE_SOLARPRESSURE_5PARA;
				period_SolarPressure      	= 3600 * 24.0;
				min_elevation             	= 5.0;
				apriorityRms_PIF          	= 0.5;
				apriorityRms_LIF          	= 0.002;
				apriorityRms_STA          	= 10.0;
				apriorityRms_TZD_abs      	= 0.5;
				apriorityRms_TZD_rel      	= 0.02;
				apriorityWet_TZD          	= 0;                      // 测站对流层湿分量估计先验值
				apriorityWet_TZD_period   	= 3600 * 2;               // 周期
				min_Wet_TZD_ncount        	= 20;                     // 相当于40分钟的数据，少于此值则合并估计区间
				max_arclengh              	= 3600;
				threhold_LAMBDA_ksb       	= 2.5;
				max_FitRms_Total          	= 20;
				bOn_AmbiguityFix          	= false;                   // true固定整周模糊度;false,不固定整周模糊度
				bOn_StaSolidTideCor       	= true; 
				bOn_StaOceanTides         	= false;
				bOn_BDTroPriModCor        	= false;
				bOn_BDSRelativity         	= true;
				bOn_BDSGraRelativity     	= false;//20140929
				bOn_RecAntARP             	= true;
				bOn_BDSAntPCO             	= false;
				bOn_RecAntPCOPCV          	= false;				
				bOn_PhaseWindUp           	= true;
				bOnEst_StaPos             	= true;
				bOnEst_StaTropZenithDelay 	= true;
				bOnEst_ERP                	= false;
				bOn_WeightElevation       	= false;
				bOn_DDDEdit               	= true;
				threshold_slip_DDDEdit    	= 0.15;
				threshold_outlier_DDDEdit 	= 0.15;
				maxdistance_Ambfixed_short	= 2500000.0;
				maxdistance_Ambfixed_long 	= 5000000.0;
				robustfactor_OC_edited    	= 2.5;
				bOnConstraints_GPSEphemeris = true;
				apriorityRms_GPSEphemeris   = 1.0;
				sampleSpan                	= 60;
				zpdProductInterval        	= 3600 * 2;
				OrbitIterativeNum         	= 2;
			}
		};

		typedef struct {int valid; int id_arc; double obscorrected_value;} cvElement; // correctedvalueElement
		typedef struct {map<int, cvElement> mapDatum;} cvEpoch;   // correctedvalueEpoch

		class BDSatDynPOD : public SatdynBasic
		{
		public:
			BDSatDynPOD(void);
		public:
			~BDSatDynPOD(void);
		public:			
	        bool sp3Fit(string strSp3FilePath, Sp3FitParameter& paraSp3Fit, bool bOnEst_EOP = true);
			//static double graRelativityCorrect(POS3D satPos, POS3D staPos, double gamma = 1);			
			static bool obsSingleDifferencePreproc(Rinex2_1_EditedObsFile& editedObsFile_A, Rinex2_1_EditedObsFile& editedObsFile_B, Rinex2_1_EditedSdObsFile& editedSdObsFile);

			// 三差数据编辑
			bool obsTriDiFFEdited_LIF(int index_L1, int index_L2, int id_sat, Rinex2_1_EditedSdObsEpoch epoch_j_1, Rinex2_1_EditedSdObsEpoch epoch_j, map<int, cvElement> &mapCvDatum_j_1, map<int, cvElement> &mapCvDatum_j, double &max_res_ddd, bool &slipFlag, double threshold_slip = 0.15);
			bool mainTriDiFFEdited(SP3File &sp3File, Rinex2_1_EditedSdObsFile &editedSdObsFile, POS3D posAnt_A, POS3D posAnt_B, string outputFileName = "");

			void weighting_Elevation(double Elevation, double& weight_P_IF, double& weight_L_IF);		
			bool adamsCowell_ac(TDT t0_Interp, TDT t1_Interp, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 75.0, int q = 11);
			bool mainNetPod_dd(string inputSp3FilePath, string outputSp3FilePath, SatOrbEstParameter& paraSatOrbEst, GPST t0, GPST t1, double h_sp3 = 300.0,bool bResEdit = true);
			
			
			// 测试代码


			//bool initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength = 3600 * 3);
			//bool dynamicPOD_pos(vector<TimePosVel>  orbitlist, SatdynBasicDatum &dynamicDatum, GPST t_forecastBegin, GPST t_forecastEnd,  vector<TimePosVel> &orbitlist_forecast, double interval = 30.0, bool bforecast = true);
		public:
			BDSatDynPODPara                 m_podParaDefine;
			map<int, AntCorrectionBlk>      m_mapBDSAntCorrectionBlk;
			StaDatumMap                     m_mapStaDatum;
			vector<StaBaselineDatum>        m_staBaseLineList;
			igs05atxFile			        m_AtxFile;			  // 天线修正文件(2013/09/16, 刘俊宏)
			StaOceanLoadingDisplacementFile m_staOldFile;         // 海潮文件
		};
	}
}
;