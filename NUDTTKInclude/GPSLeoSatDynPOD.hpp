#pragma once
#include "structDef.hpp"
#include "SatdynBasic.hpp"
#include "dynPODStructDef.hpp"
#include "MathAlgorithm.hpp"
#include "Rinex2_1_LeoEditedObsFile.hpp"
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "svnavFile.hpp"
#include "igs05atxFile.hpp"
#include "TimeAttitudeFile.hpp"
#include "AntPCVFile.hpp"
#include "Ionex1_0_File.hpp"
#include "Rinex2_1_LeoMixedEditedObsFile.hpp"
#include "GPSYawAttitudeModel1995.hpp"
#include "svnavMixedFile.hpp"
#include "GNSSYawAttitudeModel.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
using namespace NUDTTK::Geodyn;
using namespace NUDTTK::SpaceborneGPSPreproc;
using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		// 2014/10/28, 谷德峰定义, 用于电离层网格比例因子估计 
		struct IonGridRatioAlpha
		{	
			double  x;           // 自变量x: 时间或轨道高度           
			double  alpha_priori;// 电离层比例因子
			double  alpha;       // 电离层比例因子
			int     count;       // 有效历元个数
		};

		struct IonGridRatioAlphaEstPara
		{
			bool                          bOnEst_IONRatio_Alpha; // 是否估计电离层比例因子, 2014/10/27, 谷德峰
			vector<IonGridRatioAlpha>     ionRatioAlphaList;
			int                           mark_IonRatio_x;       // 自变量选择: 0-轨道高度; 1-历元时间
			double                        period_IonRatio_x;
			double                        weight_IonRatio;
			
			bool                          bOnEst_IONSLMHeight;      // 是否估计电离层比例因子, 2016/7/1, 谷德峰
			double                        ionSLMHeight_priori; 
			double                        ionSLMHeight;
			
			IonGridRatioAlphaEstPara()
			{
				bOnEst_IONRatio_Alpha = false;
				mark_IonRatio_x = 1;
				period_IonRatio_x = 3600 * 3;
				weight_IonRatio = 0.1;
				ionRatioAlphaList.clear();
				
				bOnEst_IONSLMHeight = false;
				ionSLMHeight_priori = 80000;
				ionSLMHeight = 80000;
			}

			void init(double x0, double x1, int n = 1, double addSingleLayerHeight = 80000);	
			int  getIndexIonRatioAlphaList(double h); // 确定高度h在电离层比例因子参数列表中的位置
		};

		struct GPSLeoSatDynPODPara
		{
			int                          max_OrbitIterativeNum;       // 轨道改进次数最大值
			TYPE_SOLARPRESSURE_MODEL     solarPressureType;           // 太阳光压模型类型
			TYPE_ATMOSPHEREDRAG_MODEL    atmosphereDragType;          // 大气阻力模型类型
			double                       period_SolarPressure;  
			double                       period_AtmosphereDrag;
			double                       period_EmpiricalAcc;
			double                       period_RadialEmpForce;       // R方向常值经验力参数周期
			double                       period_EarthIrradiance;      // 地球反照辐射参数周期
			double                       min_arcpoint_addpara;        // 补偿参数的最小点数, 2013/04/22
			double                       min_elevation;
			double                       apriorityRms_PIF;            // 先验无电离层码观测精度, 用于伪码和相位加权控制
			double                       apriorityRms_LIF;            // 先验无电离层相位观测精度, 用于伪码和相位加权控制
			double                       max_arclengh;    
			unsigned int                 min_arcpointcount;           // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			int                          min_eyeableGPScount;         // 最小可视卫星个数, 个数小于 min_eyeableGPScount 的历元将不参与解算
		    double                       threshold_initDynDatumEst;   // 初轨确定间隔
			int                          flag_UseSingleFrequency;     // 是否使用单频观测数据, 0: 未使用; 1: 1/2(L1+C1); 2: 网格数据修正
			bool                         bOn_WeightElevation;         // 是否进行高度角加权
			bool                         bOn_GPSRelativity;           // 是否进行 GPS 卫星相对论改正
			bool                         bOn_RecRelativity;           // 接收机相对论修正
			bool                         bOn_GPSAntPCO;               // 是否进行 GPS 卫星天线偏移修正
			bool                         bOn_LEOAntPCV_IGS;           // 是否进行 LEO 卫星天线相位中心修正, 来自 IGS 天线文件
			bool                         bOn_LEOAntPCO;               // 是否进行 LEO 卫星天线质心偏移修正
			bool                         bOn_LEOAntPCV;               // 是否进行 LEO 卫星天线相位中心修正, 来自在轨校准结果
			bool                         bOn_LEOAntCRV;               // 是否进行 LEO 卫星伪码残差中心修正, 来自在轨校准结果
			bool                         bOn_PhaseWindUp;
			bool                         bOnEst_LEOAntPCO_X;          // 是否进行 LEO 卫星 X 方向天线偏移估计
			bool                         bOnEst_LEOAntPCO_Y;          // 是否进行 LEO 卫星 Y 方向天线偏移估计
			bool                         bOnEst_LEOAntPCO_Z;          // 是否进行 LEO 卫星 Z 方向天线偏移估计			
			bool                         bOnCst_Maneuver_R;		  	  // 是否添加机动力 R 方向约束
			bool						 bOnCst_Empirical_R;		  // 是否添加经验力 R 方向约束
			bool						 bOnCst_Empirical_T;		  // 是否添加经验力 T 方向约束
			bool						 bOnCst_Empirical_N;		  // 是否添加经验力 N 方向约束
			int                          cst_empirical_type;          // 经验力约束的类型, 0: 相对约束; 1: 绝对零约束;  2: 求和零均值约束; 3: 相对约束 + 求和零均值约束; 4: 绝对约束 + 求和零均值约束
			double                       apriorityRms_LEOAntPCO;    // 先验PCO精度, 用于伪方程约束
			double						 apriorityRms_Maneuver_R;	// 机动力相对约束方程精度，2014/5/9，鞠 冰
			double                       apriorityRms_EmpiricalRel_R; // 经验力R方向相对约束方程精度，2014/10/07，谷德峰
			double                       apriorityRms_EmpiricalRel_T; // 经验力T方向相对约束方程精度，2015/04/03，鞠 冰
			double                       apriorityRms_EmpiricalRel_N; // 经验力N方向相对约束方程精度，2015/04/03，鞠 冰

			bool                         bOn_OutputCovMatrix;         // 保存相关系数矩阵文件
			double                       robustfactor_OC_edited;
			bool                         bOnEst_SysBias;              // 是否估计系统间偏差，混合系统定轨需要

			IonGridRatioAlphaEstPara     ionRatioAlpha;               // 2014/10/28, 谷德峰定义, 用于电离层比例因子估计  

			double                       threshold_rms_oc_code;
			double                       threshold_max_adjustpos;
			// 2021.03.28，加速度计数据尺度参数和时间漂移项约束开关
			bool                         bOnCst_NonconserForce_Scale;
			double                       apriorityScale_X;              // // 2019/06/20,韦春博，先验尺度参数值
			double                       apriorityScale_Y;
			double                       apriorityScale_Z;
			double                       apriorityRms_NonconserForce_X; // 尺度因子R方向绝对约束方程精度，2019/04/10，韦春博
			double                       apriorityRms_NonconserForce_Y; // 尺度因子T方向绝对约束方程精度，2019/04/10，韦春博
			double                       apriorityRms_NonconserForce_Z; // 尺度因子N方向绝对约束方程精度，2019/04/10，韦春博
			// 时间漂移项的约束
			bool                         bOnCst_NonconserForce_c1Drift;
			bool                         bOnCst_NonconserForce_c2Drift;
			double                       apriorityRms_NonconserForce_c1X;
			double                       apriorityRms_NonconserForce_c1Y;
			double                       apriorityRms_NonconserForce_c1Z;
			double                       apriorityRms_NonconserForce_c2X;
			double                       apriorityRms_NonconserForce_c2Y;
			double                       apriorityRms_NonconserForce_c2Z;
			// 尺度参数的约束
			bool                         bOnCst_NonconserForce_BiasX;
			bool                         bOnCst_NonconserForce_BiasY;
			bool                         bOnCst_NonconserForce_BiasZ;
			double                       apriorityRms_NonconserForce_BiasX;
			double                       apriorityRms_NonconserForce_BiasY;
			double                       apriorityRms_NonconserForce_BiasZ;
            
			GPSLeoSatDynPODPara()
			{
				max_OrbitIterativeNum       = 8;
				solarPressureType           = TYPE_SOLARPRESSURE_1PARA;
				atmosphereDragType          = TYPE_ATMOSPHEREDRAG_J71_GEODYN;
				period_AtmosphereDrag       = 3600 *  3.0;
				period_SolarPressure        = 3600 * 24.0; 
				period_EmpiricalAcc         = 3600 *  1.5;
				period_EarthIrradiance      = 3600 * 24.0;
				period_RadialEmpForce       = 3600 * 24.0;
				min_arcpoint_addpara        = 30; // 暂定为 30 个有效点, 如果 10 秒钟采样相当于 5 分钟
				min_elevation               = 5.0;
				apriorityRms_PIF            = 0.50;//2
				apriorityRms_LIF            = 0.002;//500
				max_arclengh                = 2000.0;
				min_arcpointcount           = 30;
				threshold_initDynDatumEst   = 300.0;//初轨确定间隔
				bOn_WeightElevation         = false;
				bOn_GPSRelativity           = true;
				bOn_RecRelativity           = true;
				bOn_GPSAntPCO               = true;
				bOn_PhaseWindUp             = true;
				bOn_LEOAntPCO               = true;
				bOn_LEOAntPCV               = true;
				bOn_LEOAntCRV               = true;
				bOn_LEOAntPCV_IGS           = false;
				bOnEst_LEOAntPCO_X          = false;
				bOnEst_LEOAntPCO_Y          = false;
				bOnEst_LEOAntPCO_Z          = false;
				bOnCst_Maneuver_R	        = false;
				bOnCst_Empirical_R		    = true;
				bOnCst_Empirical_T		    = false;
				bOnCst_Empirical_N		    = false;
				cst_empirical_type          = 0;
				apriorityRms_LEOAntPCO      = 1.0;
				apriorityRms_Maneuver_R	    = 1E-6;
				apriorityRms_EmpiricalRel_R = 1E-9; //谷德峰
				apriorityRms_EmpiricalRel_T = 1E-6;
				apriorityRms_EmpiricalRel_N = 1E-6;

				flag_UseSingleFrequency     = 0;
				bOn_OutputCovMatrix         = false;
				min_eyeableGPScount         = 3;
				robustfactor_OC_edited      = 3.0;
				bOnEst_SysBias              = true;

				threshold_rms_oc_code       = 100.0;

				threshold_max_adjustpos     = 5.0E-3;
				if(flag_UseSingleFrequency != 0)
					threshold_max_adjustpos = 5.0E-2;
				// 2021.03.28，加速度计数据尺度参数和时间漂移项约束开关
				bOnCst_NonconserForce_Scale   = true;  // 2019/05/21,韦春博，尺度参数约束开关
				apriorityScale_X              = 1.0;    // 2019/06/20,韦春博，先验尺度参数值
				apriorityScale_Y              = 1.0;
				apriorityScale_Z              = 1.0;
				apriorityRms_NonconserForce_X = 0.1;    // 2019/04/10,韦春博，尺度参数约束，参考GRACE TN-02
				apriorityRms_NonconserForce_Y = 0.02;
				apriorityRms_NonconserForce_Z = 0.02;
				// 增加对时间漂移项的约束，防止时间漂移项过大，造成加速度计数据出现明显的漂移
				bOnCst_NonconserForce_c1Drift   = true;  // 2019/05/21,韦春博，时间漂移项约束开关
				bOnCst_NonconserForce_c2Drift   = true;  // 2019/05/21,韦春博，时间漂移项约束开关
				apriorityRms_NonconserForce_c1X = 5E-10;  // 时间漂移项约束
				apriorityRms_NonconserForce_c1Y = 5E-9;
				apriorityRms_NonconserForce_c1Z = 5E-12;
				apriorityRms_NonconserForce_c2X = 5E-13;
				apriorityRms_NonconserForce_c2Y = 5E-12;
				apriorityRms_NonconserForce_c2Z = 5E-13;
				// 增加对偏差参数的约束
				bOnCst_NonconserForce_BiasX       = false;
				bOnCst_NonconserForce_BiasY       = false;
				bOnCst_NonconserForce_BiasZ       = false;
				apriorityRms_NonconserForce_BiasX = 1E-7;
				apriorityRms_NonconserForce_BiasY = 1E-9;
				apriorityRms_NonconserForce_BiasZ = 1E-9;
			}
		};

		struct MixedGNSSPODDatum
		{
			char cSatSystem;                                            // 系统标记
			vector<Rinex2_1_LeoEditedObsEpoch>  editedObsEpochlist;     // 单系统历元格式观测数据, 非混合格式
			vector<Rinex2_1_EditedObsSat>       editedObsSatlist;       // 单系统卫星格式观测数据, 非混合格式
			map<int, PODEpoch>                  mapDynEpochList;    
			vector<ObsEqEpoch>                  P_IFEpochList;       
			vector<ObsEqArc>                    L_IFArcList;
			int                                 ambiguityIndexBegin;
			vector<ObsEqEpoch>                  L_IFEpochList;
			map<int, double>                    mapWindupPrev;          // 记录Windup修正数据
			vector<int>                         mixedEpochIdList;       // 记录当前历元序列在[混合格式]中的历元序号, 便于相互索引
			vector<int>                         epochIdList;            // 记录混合历元序列在[当前格式]中的历元序号
			vector<O_CResEpoch>                 ocResP_IFEpochList;     // 无电离层伪码O-C残差
			vector<O_CResArc>                   ocResL_IFArcList;       // 无电离层相位O-C残差
			double   FREQUENCE_L1;
			double   FREQUENCE_L2;
			double   WAVELENGTH_L1;
			double   WAVELENGTH_L2;
			double   coefficient_L1;
			double   coefficient_L2;
			int      index_P1;
			int      index_P2;
			int      index_L1;
			int      index_L2;
			double   weightSystem;                                        // 系统间的权系数之比, 默认为1.0
			double   sysBias;                                             // 系统间偏差，以GPS为参考系统，其值为0			
			
			MixedGNSSPODDatum(double frequence1 = GPS_FREQUENCE_L1, double frequence2 = GPS_FREQUENCE_L2)
			{
				weightSystem   = 1.0;
				FREQUENCE_L1   = frequence1;
				FREQUENCE_L2   = frequence2;
				WAVELENGTH_L1  = SPEED_LIGHT / FREQUENCE_L1;
				WAVELENGTH_L2  = SPEED_LIGHT / FREQUENCE_L2;
				coefficient_L1 = 1 / (1 - pow(FREQUENCE_L2 / FREQUENCE_L1, 2)); 
				coefficient_L2 = 1 / (1 - pow(FREQUENCE_L1 / FREQUENCE_L2, 2)); 
				sysBias        = 0;				
			}
		};

		class GPSLeoSatDynPOD : public SatdynBasic
		{
		public:
			GPSLeoSatDynPOD(void);
		public:
			~GPSLeoSatDynPOD(void);
		public:
			void    setSP3File(SP3File sp3File); 
			void    setCLKFile(CLKFile clkFile); 
			CLKFile getRecClkFile(); 
			void    setStepAdamsCowell(double step);
			double  getStepAdamsCowell();
			bool    loadSP3File(string  strSp3FileName);
			bool    loadCLKFile(string  strCLKFileName); 
			bool    loadCLKFile_rinex304(string  strCLKFileName);
            bool    adamsCowell_Interp_Leo(vector<TDT> interpTimelist, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist, vector<Matrix> &matRtPartiallist, double h = 10.0, int q = 11);
            bool    initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength = 3600 * 3);
			void    weighting_Elevation(double Elevation, double& weight_P_IF, double& weight_L_IF);
			bool    dynamicPOD_pos(vector<TimePosVel> obsOrbitList, SatdynBasicDatum &dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0, bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true);
			bool    dynamicPOD_phase(string editedObsFilePath, SatdynBasicDatum &dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0,  bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true, bool bOn_NotUsedPhase = false);
			bool    dynamicPOD_L1(string editedObsFilePath, SatdynBasicDatum &dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0,  bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true, bool bOn_NotUsedPhase = false);
			void    orbitExtrapolation(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0);
			void    orbitExtrapolation(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVelAcc> &forecastOrbList, double interval = 30.0);
			void    orbitExtrapolation_jerk(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVelAccJerk> &forecastOrbList, double interval = 30.0, bool bECEF = true);
		
			// Mixed 混合观测数据相关处理函数
			bool    dynamicMixedPOD_phase(string editedMixedObsFilePath, SatdynBasicDatum &dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0,  bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true);
		private:
			double  m_stepAdamsCowell; // 20150308, 谷德峰, 轨道积分步长统一修改
			bool    loadEditedObsFile(string  strEditedObsFileName);
			CLKFile m_recClkFile; // 精密钟差解算结果

			// Mixed 混合观测数据相关处理函数
			bool loadMixedEditedObsFile(string  strMixedEditedObsFileName);
		public:
			GPSLeoSatDynPODPara       m_podParaDefine;
			CLKFile                   m_clkFile;			     // 精密钟差数据文件
			SP3File                   m_sp3File;			     // 精密星历数据文件
			Rinex2_1_LeoEditedObsFile m_editedObsFile;		     // 原始观测数据
			POS3D                     m_pcoAnt;				     // 天线偏心量
			svnavFile                 m_svnavFile;			     // GPS天线偏移
			igs05atxFile			  m_AtxFile;			     // 天线修正文件(2013/04/18, 鞠冰)
			AntPCVFile                m_pcvFile;			     // 天线相位中心, 在轨估计结果
			AntPCVFile                m_crvFile_IF;			     // 伪码残差变化, 在轨估计结果
			Ionex1_0_File             m_ionFile;                 // IGS电离层网格产品
			//TimeAttitudeFile        m_attFile;			     // 姿态文件
			//Matrix                  m_matAxisBody2RTN;         // 星固系到轨道系的固定准换矩阵, 用于存在固定偏差角度的三轴稳定控制, 2015/03/09
			vector<O_CResEpoch>       m_ocResP_IFEpochList;	     // 无电离层伪码O-C残差
			vector<O_CResArc>         m_ocResL_IFArcList;	     // 无电离层相位O-C残差
			svnavMixedFile                  m_svnavMixedFile;
			GNSSYawAttitudeModel            m_gymMixed;		
			// Mixed 混合观测数据相关变量
			Rinex2_1_LeoMixedEditedObsFile m_editedMixedObsFile; // 混合格式原始观测数据
			vector<MixedGNSSPODDatum>      m_dataMixedGNSSlist;  
			Matrix                         m_matAxisAnt2Body;    // 天线系到星固系的转移矩阵，用于天线可能安装在非天顶方向的情况
		public:
			POS3D                     m_pcoAntEst;			     // 估计量: 天线偏心量估计结果
		};
	}
}



