#pragma once
#include "structDef.hpp"
#include "SatdynBasic.hpp"
#include "dynPODStructDef.hpp"
#include "MathAlgorithm.hpp"
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "igs05atxFile.hpp"
#include "AntPCVFile.hpp"
#include "svnavMixedFile.hpp"
#include "GNSSYawAttitudeModel.hpp"
#include "GNSS_AttitudeFile.hpp"
#include "graceKBR1BFile.hpp"
#include "gracefoKBR1BFile.hpp"
#include "lambda.hpp"
#include "wsbFile.hpp"
#include "TimeAttitudeFile.hpp"
#include "LeoNetPODEditedSdObsFile.hpp"
#include "Rinex2_1_LeoMixedEditedObsFile.hpp"
#include "Ionex1_0_File.hpp"

//  Copyright 2018, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
using namespace NUDTTK::Geodyn;
using namespace NUDTTK::SpaceborneGPSPreproc;
using namespace NUDTTK::Math;
using namespace NUDTTK::LAMBDA;

namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		// 弧段数据在存储时，放入测站数据结构，便于在基线处理时直接检索
		struct LEOPOD_ObsArc 
		{
			double             ambiguity_IF; 
			double             ambiguity_MW; // +
			double             rms_mw;          // 弧段 MW 变化方差
			double             ambFixedWL;      // 宽巷模糊度固定解：宽巷模糊度固定解通过 MW 组合方式计算
			double             res_WL;          // 宽巷模糊度固定解残差
			bool               bOn_fixedWL;     // 该弧段宽巷模糊度固定成功与否
			int                id_Ambiguity; 
			GPST               t0;
			GPST               t1;
			GPST               t;               // 弧段中间时刻
			BYTE               id_Sat;
			double             mean_elevation;  // 平均高度角，邵凯，2020.7.22
			double             rms_res;          // 弧段相位残差RMS
			ObsEqArcElementMap obsList;
			LEOPOD_ObsArc()
			{
				rms_mw     = 0.0;
				bOn_fixedWL = false;
				mean_elevation = 0.0;
				rms_res = 0.0;
			}
			bool updateRobustWeight(double threshold_slip, double threshold_rms, unsigned int threshold_count_np = 20, double robustfactor = 3.0);
			bool updateRobustWeight_New(double threshold_slip, double threshold_rms, unsigned int threshold_count_np = 20, double robustfactor = 3.0);
		};

		// 钟差解部分
		struct LEOPOD_CLKSOL
		{
			GPST    t;
			double  clock;
			POS3D   pos;              // 概略位置
			POS3D   vel;              // 速度信息
			int     sqlMark;          // 解标记: 0-插值获得, 未经过解算; 1-伪码解; 2-相位解
			double  pdop;
			int     eyeableSatCount; 
			int     validIndex;       // 记录有效解索引序号, 无效为-1 
			double  sigma; 
			map<int, double> mapN_ca; // 记录交叉项
		    map<int, Matrix> mapN_cb; // 记录交叉项,运动学 +
		};

		struct LEOPOD_ZD_AmbEq
		{
			GPST     t0;
			GPST     t1;
			int      id_Ambiguity;
			BYTE     id_Sat;
				//double   rms_MW;
			double   ambiguity_MW;
			double   ambiguity_IF; // IF 浮点解
			double   ambiguity_Fixed_IF; // IF 固定解
			double   ambiguity_Fixed_MW; // WL固定解
			double   ambiguity_NL; // NL 浮点解
			double   ambiguity_Fixed_NL; // NL固定解 
			double   res_NL;         // NL残差
		    double   mw_rms; // +
			double   res_rms; // + 定轨残差RMS
			bool     flag_fixed_nl; // 宽巷模糊度固定是否成功标签
			LEOPOD_ZD_AmbEq()
			{
				flag_fixed_nl = false;
			}
			//double   mw_res; // +
		};

		struct LEOPOD_SD_AmbEq
		{
			GPST     t0;
			GPST     t1;
			int      id_Ambiguity_A;
			int      id_Ambiguity_B;
			BYTE     id_Sat;
			BYTE     id_Sat_B; // +
			double   rms_mw;    // 单差mw_rms值
			double   ambiguity_MW;
			double   ambiguity_IF;
            double   ambiguity_SD_IF;
		};

		// 模糊度双差约束方程
		struct LEOPOD_DD_AmbEq
		{
			int    id0_Ambiguity_A;
			int    id0_Ambiguity_B;
			int    id1_Ambiguity_A;
			int    id1_Ambiguity_B;
			double ambiguity_DD_IF;
		};

		// 多系统预定义, 作为统一多系统选择输入, [注: 选择后不同测站的多系统数据可能不全]
		struct LEOPOD_MixedSys
		{
			// 需要初始化部分
			char                         cSys;        // 系统标识
			double                       wSys;        // 系统权值, 默认 1.0
			char                         recType;     // 用于不同卫星不同类型接收机的识别处理
			string                       name_C1;
			string                       name_C2;
			string                       name_L1;
			string                       name_L2;
			string                       nameFreq_L1; // 频率名称"G01"，主要用于PCV修正
			string                       nameFreq_L2;  
			double                       freq_L1;
			double                       freq_L2;
			// 与系统有关的预定义部分, 使用时需要通过sysFreqName索引 
			bool                         on_PhaseWindUp;   
			bool                         on_GnssRelativity;
			bool                         on_GnssPCV;
			bool                         on_RecPCV;
			bool                         on_RecCRV;
			bool                         on_RecCRVP1;
			bool                         on_RecCRVP2;
			float                        priorsigma_PIF;              // 用于伪码加权控制

			string getSysFreqName()
			{
				char sysFreqName[4];
				sprintf(sysFreqName, "%1c%1c%1c", cSys, name_L1[1], name_L2[1]); // "Gij" "Cij" "Eij" "Rxx"
				sysFreqName[3] = '\0';
				return sysFreqName;
			}
			
			LEOPOD_MixedSys()
			{
				wSys = 1.0;
				recType = 'N';
				on_PhaseWindUp = true;
				on_GnssRelativity = true;
				on_GnssPCV = true;
				on_RecCRV  = false;
				on_RecPCV  = false;
				on_RecCRVP1 = false;
				on_RecCRVP2 = false;
				priorsigma_PIF = 0.50f;
			}
		};

		struct LEOPOD_MixedSysData
		{
			// 不需要初始化部分
			char                                cSys;                   // 系统标识
			double                              wSys;                   // 系统权值, 默认 1.0
			string                              name_C1;                // "C1" "P1" "P2"
			string                              name_C2;                
			string                              name_L1;                // "L1" "L2"
			string                              name_L2;                
			string                              nameFreq_L1;            // 频率名称"G01"，主要用于PCV修正
			string                              nameFreq_L2;  
			double                              freq_L1;
			double                              freq_L2;
			int                                 index_C1;               // 频率1和2在处理时主用
			int                                 index_C2;               // 这四个C1C2L1L2默认为-1
			int                                 index_L1;
			int                                 index_L2;
			float                               priorsigma_PIF;         // 用于伪码加权控制

           // 过程计算存储部分
			string                              sysFreqName;            // 记录系统和频率信息, 用于索引相关的预定义
			int                                 iSys;
			vector<Rinex2_1_LeoEditedObsEpoch>   editedObsEpochlist;     // 单系统历元格式观测数据, 非混合格式
			vector<Rinex2_1_EditedObsSat>        editedObsSatlist;       // 单系统卫星格式观测数据, 非混合格式
			map<int, PODEpoch>                  mapDynEpochList;    
			vector<ObsEqEpoch>                  P_IFEpochList;       
			vector<LEOPOD_ObsArc>               L_IFArcList;
			vector<ObsEqEpoch>                  L_IFEpochList;
			//map<int, double>                    mapWindupPrev;          // 记录Windup修正数据
			vector<int>                         mixedEpochIdList;       // 记录当前历元序列在[混合格式]中的历元序号, 便于相互索引
			vector<int>                         epochIdList;            // 记录混合历元序列在[当前格式]中的历元序号
			vector<O_CResEpoch>                 ocResP_IFEpochList;     // 无电离层伪码O-C残差
			vector<O_CResArc>                   ocResL_IFArcList;       // 无电离层相位O-C残差
			double                              ocResRMS_P_IF;
			double                              ocResRMS_L_IF;
			POS3D                               pcoRecEst;              // 天线偏移估计结果，与系统和频点有关
			double                              sysBias;                // 系统间偏差，以GPS为参考系统，其值为 0
			
			// 非差模糊度浮点解信息, 用于不同测站间索引形成双差，系统间不进行组合
			int                                 n0_AmbiguitySys;        // 每个系统模糊度参数的起点

			LEOPOD_MixedSysData()
			{
				sysBias = 0.0;
				wSys    = 1.0;
				ocResRMS_P_IF = 0.0;
				ocResRMS_L_IF = 0.0;
			}
		};
		// 弧段间单差模糊度结果
		struct BetweenPass_SD_AmbEq_q
		{
			double   delta_bIJ;       // 弧段 I 和弧段 J 间单差值  delta_bIJ = bI - bJ, 窄巷
			double   delta_MWbIJ;     // 弧段 I 和弧段 J 间, 宽巷模糊度单差值
			double   delta_IFbIJ;     // 弧段 I 和弧段 J 间, IF模糊度单差值
			double   res;             // rms值,单差窄巷模糊度
			double   res_WL;          // 单差宽巷模糊度残差
			bool     fixed;           // 固定成功标记
			bool     fixed_wl;        // 宽巷固定成功标记
			bool     fixed_nl;        // 窄巷固定成功标记
			int      id_Ambiguity_A;  // 第一个弧段模糊度序号
			int      id_Ambiguity_B;  // 第二个弧段模糊度序号
			double   delta_nl_fixed;  // 单差窄巷模糊度固定值
			double   delta_wl_fixed;  // 单差宽巷模糊度固定值
			double   constraint_AB;   // A-B 模糊度约束
			BetweenPass_SD_AmbEq_q()
			{
				res = 0.0;
				res_WL = 0.0;
				delta_nl_fixed = 0;
				delta_wl_fixed = 0;
				fixed          = false;
				constraint_AB  = 0.0;
			}
		};
		//// 不同姿态模型数据
		//enum TYPE_ATT_MODEL
		//{
		//	TYPE_ATT_Body2J2000   = 1,            // 星固坐标系到惯性坐标系, GRACE,CHAMP
		//	TYPE_ATT_Body2ECEF    = 2             // 星固坐标系到地固坐标系，Swarm
		//};
		struct LEOPOD_SD_Section  // 邵凯 +
		{
			GPST   t0;
			GPST   t1;
			bool   on_Fixed_mw;        // 记录模糊度是否固定
			float  max_Decimal_mw;     // 记录模糊度固定的最大小数部分
			Matrix matSDFixedFlag_mw;  // 记录模糊度固定成功标记
			Matrix matSDFixed_mw;      // 记录模糊度固定值
			int    count_Fixed_mw;     // mw固定个数
			bool   on_Fixed_NA;
			float  max_Decimal_NA;
			Matrix matSDFixedFlag_NA;
			Matrix matSDFixed_NA;
			int    count_Fixed_NA;     // NA固定个数
			vector<double>     WL_ResList; // 记录WL小数残差
			vector<double>     N1_ResList; // 记录N1小数残差
			vector<LEOPOD_ZD_AmbEq> ambZDEqList;
            vector<LEOPOD_SD_AmbEq> ambSDEqList;
		};
		// 测站数据结构, 包括输入数据和输出轨道解部分
		struct LEOPOD_StaDatum 
		{
			// 输入部分
			GPST      t0;                     // 起始时间
			GPST      t1;                     // 结束时间
			string    staName;
			string    pathMixedEditedObsFile;
			string    pathFolder;
			size_t    count_MixedEpoch;    
			double    orbOutputInterval;
			bool      on_IsUsed_InitDynDatum;// 是否使用已经有的初轨
			double    ArcLength_InitDynDatum;// 初轨时间长度 3个小时默认
			SatdynBasicDatum dynDatum_Init; // 初始值动力学轨道信息
			double    period_SolarPressure; // 目前网解卫星编队轨道应当高度相近, 分段区间的定义大致统一
			double    period_AtmosphereDrag;
			double    period_EarthIrradiance; // + 地球反照辐射
			double    period_RadialEmpAcc; // +
			double    period_TangentialEmpAcc; // +
			double    period_NormalEmpAcc; // +
			double    period_EmpiricalAcc;
			bool      on_Cst_EmpAcc_R;//R方向脉冲力量非常值约束
			bool      on_Cst_EmpAcc_T;//T方向脉冲力约束
			bool      on_Cst_EmpAcc_N;//N方向脉冲力约束
			bool      on_Cst_Maneuver_R;       // R方向的机动力约束, R 和 T 方向不能同时估计, 为何？邵凯，2020.9.5提问
			bool      on_Cst_Maneuver_T;       // T方向的机动力约束
			bool      on_Cst_Maneuver_N;       // N方向的机动力约束
			bool      on_Cst_Radial_EmpAcc;    // R方向经验力常值参数约束 +
			bool      on_Cst_Tangential_EmpAcc;// T方向经验力常值参数约束 +
			bool      on_Cst_Normal_EmpAcc;    // N方向经验力常值参数约束 +
			float         priorsigma_RecPCO;
			float         priorsigma_RadialEmpAcc;  // R方向常值经验力先验值
			float         priorsigma_TangentialEmpAcc;  // T方向常值经验力先验
			float         priorsigma_NormalEmpAcc;  // N方向常值经验力先验
			float         priorsigma_EmpAcc_R;
			float         priorsigma_EmpAcc_T;
			float         priorsigma_EmpAcc_N;
			float         priorsigma_EmpAcc_Sum;
			double        priorsigma_ManeuverAcc;
			bool      on_RecPCVFromIGS;
			bool      on_EstRecPCO_X; // 是否进行 X 方向天线PCO估计
			bool      on_EstRecPCO_Y; // 是否进行 Y 方向天线PCO估计
			bool      on_EstRecPCO_Z; // 是否进行 Z 方向天线PCO估计
			bool      on_EstSysBias;  // 是否估计系统间偏差，混合系统定轨需要
			bool      on_UseWSB;     // 是否使用WSB数据
			bool      flag_amb;     // 模糊度固定相关标识
			int       k_amb;        // 记录模糊度固定下迭代的次数 +
			vector<int> no_NA_List; // 第一次NA无法固定对应的单差区间号
			//TYPE_ATT_MODEL         attModelType; // 姿态数据类型
			vector<BetweenPass_SD_AmbEq_q>  sdAmb_List;  // 弧段间单差模糊度列表，用于非差模糊度固定方法，邵凯，2019/07/06
			TimeAttitudeFile        attFile;
			map<string, AntPCVFile> pcvRecFileList; // 天线相位中心变化，与系统和频点有关，默认不初始化时为空
			map<string, AntPCVFile> crvRecFileList; // 伪码残差中心变化，与系统和频点有关，默认不初始化时为空
			map<string, POS3D>      pcoRecList;     // 天线相位中心偏移，与系统和频点有关，默认不初始化时为空
			Matrix                  m_matAxisBody2RTN;    // 星固系到轨道系的固定准换矩阵, 用于存在固定偏差角度的三轴稳定控制, 2015/03/09	
			Matrix                  m_matAxisAnt2Body;    // 天线系到星固系的转移矩阵，用于天线可能安装在非天顶方向的情况，2021/06/07
			SatMacroFile            m_satMacroFile;      // 卫星宏模型文件

			map<string, AntPCVFile> rcvRecFileList; // 接收机RCV变化，与系统和频点有关系，定义GP1为GPS的P1频点CRV
			// 过程计算存储部分: 多系统数据, 与多系统的预定义 m_dataMixedSysList 呼应
			// 存在问题: 预定义 m_dataMixedSysList 有 N 个系统, 但数据中可能只有 M 个系统 dataMixedSysList, 不匹配
			//           每个测站数据文件类型索引号 "index_xx" 可能是不同的，需要与 dataMixedSysList 绑定
			//           天线PCO估计开关是和系统定义绑定还是测站（目前建议和测站绑定）
			// 解决办法: m_dataMixedSysList 中只定义类型选取之类的必要输入控制信息, 得到的有用索引信息放入 dataMixedSysList 中去
			vector<LEOPOD_MixedSysData> dataMixedSysList;
			AntCorrectionBlk            pcvRecFromIGS;
			vector<Matrix>              attMatrixList;//
			vector<int>                 attFlagList;
			//vector<POS3D>               attXYZBodyList[3];
			vector<POS3D>               attXYZAntList[3];
			vector<int>                 validMixedEpochList; // 记录钟差有效解对应的时刻位置
			vector<string>              satGnssNameList;
			vector<TimePosVel>          interpOrbitlist;     // 插值序列
			vector<Matrix>              interpRtPartiallist; // 插值偏导数序列
			vector<TDT>                 interpTimelist;      // 插值TDT时间序列
			vector<TimePosVel>          acOrbitList;         // 用于插值轨道序列 getEphemeris
			vector<Matrix>              acRtPartialList;     // 用于插值偏导数序列 getInterpRtPartial

			// 输出部分
			int                   n0_EstParameters;         // 记录合并后待估参数的分块起点, 便于分块检索
			int                   count_EstParameters;      // 不含钟差: count_EstDynParameters + count_EstAmbiguity + count_EstRecPCO + count_EstSysBias
			int                   count_EstRecPCO;          // PCO参数个数
			int                   count_EstAmbiguity;       // 模糊度参数个数
			int                   count_EstSysBias;         // 系统差参数个数
			int                   count_EstDynParameters;   // 动力学参数个数
			int                   count_EstClockParameters; // 钟差参数个数
			int                   n0_SolarPressure;         // 分块内部位置, 记录太阳光压参数起始位置
			int                   n0_EarthIrradiance;       // 分块内部位置, 记录地球反照辐射参数起始位置 +
			int                   n0_AtmosphereDrag;        // 分块内部位置, 记录大气阻力参数起始位置
			int                   n0_RadialForce;           // 分块内部位置, 记录R方向经验力参数起始位置 + 
			int                   n0_TangentialForce;       // 分块内部位置, 记录T方向经验力参数起始位置 + 
			int                   n0_NormalForce;           // 分块内部位置, 记录N方向经验力参数起始位置 + 
			int                   n0_EmpiricalForce;        // 分块内部位置, 记录经验力参数在起始位置
			int                   n0_ManeuverForce;         // 分块内部位置, 记录机动力参数在起始位置
			// 2021.03.28，韦春博修改，修改加速度计数据定轨程序，将尺度和偏差参数的估计分开
			int                   n0_ScaleParaBegin;         // 记录加速度计标校参数在整个待估动力学参数列表中的位置
			int                   n0_xBiasParaBegin;         // 记录加速度计x方向标校参数在整个待估动力学参数列表中的位置
			int                   n0_yBiasParaBegin;         // 记录加速度计y方向标校参数在整个待估动力学参数列表中的位置
			int                   n0_zBiasParaBegin;         // 加速度计z方向标校参数在整个待估动力学参数列表中的位置
			int                   n0_c1DriftParaBegin;       // 时间漂移项c1
			int                   n0_c2DriftParaBegin;       // 时间漂移项c2
			//
            int                   n0_RecPCO;                // 分块内部位置, 记录PCO参数起始位置
			int                   n0_SysBias;               // 分块内部位置, 记录系统偏差参数起始位置
			int                   n0_Ambiguity;             // 分块内部位置, 记录模糊度参数起始位置
			SatdynBasicDatum      dynDatum_Est;  // 估计值动力学轨道信息
			vector<TimePosVel>    orbList; // 轨道估计结果（包含预报部分）
			vector<LEOPOD_CLKSOL> clkList; // 钟差估计结果（动力学）+ 轨道估计结果（运动学）
			Matrix                matN_aa; // 动力学参数、模糊度、天线偏移、系统偏差
			Matrix                matN_a;
			Matrix                matN_c;
			vector<double>        N_cc;
			vector<double>        N_cc_Inv;
			Matrix                matN_ac_cc_inv_na; 
			Matrix                matN_ac_cc_inv_ca;
             //Matrix                mat_a; // 记录参数估计结果: 动力学参数、模糊度、天线偏移、系统偏差
			 //Matrix                mat_c; // 记录参数估计结果: 钟差
			Matrix                matda;
			Matrix                matdc;
			// 运动学矩阵 + 邵凯
			vector<Matrix>        Nxx_List; // 运动学 4 x 4，[x,y,z,clk] +
			vector<Matrix>    Nxx_inv_List;
			vector<Matrix>        Nxb_List;
			vector<Matrix>         nx_List;
			Matrix                      nb;
			Matrix                     Nbb;
		    vector<Matrix>         dx_List;      // 记录位置参数改进数
			Matrix                      db;      // 记录系统偏差+模糊度参数改进数   
			Matrix       matN_bx_xx_inv_xb;      // 
			Matrix       matN_bx_xx_inv_nx;
			//////////////////////////
			bool getEphemeris(TDT t, TimePosVel& interpOrbit, int nLagrange = 9);
			bool getEphemeris_ITRF(GPST t, TimePosVel& interpOrbit, int nLagrange = 9); // 地固系
			bool getInterpRtPartial(TDT t, Matrix& interpRtPartial);
			// ***************************
			bool on_ION_HO;       // 高阶电离层计算
			//*******************************
			// 根据每个区间的有效数据点个数clkList.validIndex统计, 进行dynDatum_Est大气阻力和经验加速度区间合并
			void mergeDynDatum(unsigned int min_SubSectionPoints);
			// 更新轨道改进结果
			void updateDynDatum();
			int updateDynDatum_Kinematic();
			// 保存轨道改进结果
			void writeDynDatum();
			void writeDynDatum_SD();
			// 残差编辑
			void ocResEdit(float factor = 3.0f, bool flag_UseSingleFrequency = false);
			void ocResEdit_Kine(float factor = 3.0f);
			void ocResEdit_Kine_New(float factor = 3.0f);
			void ocResEdit_NET(float factor = 3.0f);
			// 更新残差
			void ocResOutput();
			void ocResOutput_Kine();
			// 计算待估参数初始位置: n0_SolarPressure、n0_AtmosphereDrag、n0_EmpiricalForce、n0_ManeuverForce、n0_RecPCO、n0_SysBias
			void getEstParameters_n0();
			void getEstParameters_n0_Kinematic();
            // 根据L_IFArcList检索生成对应的非差模糊度列表
			void getAmbZDEqList(string sysFreqName, vector<LEOPOD_ZD_AmbEq> &ambZDEqList);
			//*******************************************************************************
			// 根据非差模糊度列表ambZDEqList检索单差模糊度区间ambSectionList，邵凯，2020.4.25 +
			void getAmbZDEqList_sd(string sysFreqName, vector<LEOPOD_ZD_AmbEq> &ambZDEqList);
			void getAmbZDEqList_zd(string sysFreqName, vector<LEOPOD_ZD_AmbEq> &ambZDEqList);
			vector<LEOPOD_SD_Section>   ambSectionList;
			int                         count_SDAmbiguity_All;
			int                         count_FixedSDAmbiguity_mw;
			int                         count_FixedSDAmbiguity_NA;
			double                      ratio_Delete_NA; // Delete所占比例
			float     max_var_MW_singel; // 非差mw方差阈值
			float     max_rms_res_singel; // 非差IF定轨残差阈值
			float     ratio_UpdateREFAmb_mw;
			float     ksb_LAMBDA_mw;
			float     max_AmbDecimal_mw;
			float     ksb_LAMBDA_NA;
			float     max_AmbDecimal_NA;
			double     max_Rms_mw_ZD;
			bool      on_SDAmbDebugInfo;
			double    minCommonViewTime; // 最小共视时间
			string    sysFreqName; // 兼容同一系统不同频率组合
			void getSDSection();
			void getSDSection_zd();
			// 利用lambda方法进行寻优, 搜索最佳的模糊度组合, 以满足模糊度固定的准则
			bool lambdaSelected(Matrix matAFloat, Matrix matQahat, Matrix& matSqnorm, Matrix& matAFixed, Matrix matSelectedFlag);
			// 根据ambSectionList进行单差模糊度固定
			void fixSDAmb(float ratio_UpdateREFAmb_mw, 
				          float max_AmbDecimal_mw, 
						  float max_AmbDecimal_NA, 
						  float ksb_LAMBDA_mw, 
						  float ksb_LAMBDA_NA, 
						  double COEFF_MW, 
						  double WAVELENGTH_N, 
						  bool on_SDAmbDebugInfo);
			void fixSDAmb_sd(float max_AmbDecimal_mw, 
						  float max_AmbDecimal_NA, 
						  double COEFF_MW, 
						  double WAVELENGTH_N, 
						  bool on_SDAmbDebugInfo);
			// 宽巷模糊度已固定，只固定窄巷
			void fixSDAmb_zd(float max_AmbDecimal_mw, 
						  float max_AmbDecimal_NA, 
						  double COEFF_MW, 
						  double WAVELENGTH_N, 
						  bool on_SDAmbDebugInfo);
			//*******************************************************************************
			// 输出钟差文件
			void getRecClkFile(CLKFile& recClkFile); 
			void getRecClkFile_Kine(CLKFile& recClkFile); 

			//vector<TimePosVel>             m_leoOrbitList;            // 动力学参考轨道
			//void setLeoOrbitList(vector<TimePosVel> leoOrbitList); // +
			//bool getLeoOrbitPosVel(GPST t, TimePosVel& orbit, unsigned int nLagrange = 8); // +

			LEOPOD_StaDatum()
			{
				on_ION_HO = false;
				count_MixedEpoch = 0;
				orbOutputInterval = 30.0;
				on_IsUsed_InitDynDatum = false;
				ArcLength_InitDynDatum = 3600.0 * 3.0;
				on_RecPCVFromIGS       = false;
				period_AtmosphereDrag = 3600 *  3.0;
				period_SolarPressure  = 3600 * 24.0; 
				period_RadialEmpAcc   = 3600 * 24.0;
				period_EarthIrradiance = 3600 * 24.0;
				period_TangentialEmpAcc = 3600.0 * 24.0;
				period_NormalEmpAcc   = 3600.0 * 24.0;
				period_EmpiricalAcc   = 3600 *  1.5;
				on_Cst_EmpAcc_R		  = false;
				on_Cst_EmpAcc_T		  = false;
				on_Cst_EmpAcc_N		  = false;
				on_Cst_Radial_EmpAcc  = false;
				on_Cst_Tangential_EmpAcc  = false;
				on_Cst_Normal_EmpAcc  = false;
				on_Cst_Maneuver_R     = false;
				on_Cst_Maneuver_T     = false;
				on_Cst_Maneuver_N     = false;
				on_EstSysBias         = false;
				on_UseWSB             = false;
				flag_amb              = false;
				k_amb                 = 0;
				//attModelType          = TYPE_ATT_Body2J2000;
				priorsigma_RecPCO         = 1.0f;
				priorsigma_EmpAcc_R       = 1E-9f; 
				priorsigma_EmpAcc_T       = 1E-6f;
				priorsigma_EmpAcc_N       = 1E-6f;
				priorsigma_RadialEmpAcc       = 1E-6f;
				priorsigma_TangentialEmpAcc   = 1E-6f;
				priorsigma_NormalEmpAcc       = 1E-6f;
				priorsigma_EmpAcc_Sum     = 1E-8f;
				priorsigma_ManeuverAcc    = 1E-4f;
				m_matAxisBody2RTN.MakeUnitMatrix(3);
				m_matAxisAnt2Body = TimeCoordConvert::rotate(PI, 1);
				// **************************************************************
				ratio_Delete_NA                  = 0.6;
				max_var_MW_singel                = 1.5f;
				max_rms_res_singel               = 0.01f;
				on_SDAmbDebugInfo         = false;
				ratio_UpdateREFAmb_mw     = 1.5f;
				max_AmbDecimal_mw         = 0.4f;    // 兼顾原有1m软件测试使用, 适当放宽宽巷浮点解检测, 避免20060107 04:49:20-05:23:00 4个宽巷模糊度浮点解小数部分在0.3-0.4之间被删除
			    max_AmbDecimal_NA         = 0.5f;
				ksb_LAMBDA_mw             = 3.0f; 
				ksb_LAMBDA_NA             = 5.0f;
				minCommonViewTime         = 480;     // 8分钟
				max_Rms_mw_ZD             = 0.1;
				// **************************************************************
				
			}
			~LEOPOD_StaDatum()
			{
				rcvRecFileList.clear();
			}
		};
		typedef map<string, LEOPOD_StaDatum> StaDatumMap;
		struct LEOPOD_DD_Section
		{
			GPST   t0;
			GPST   t1;
			bool   on_Fixed_mw;        // 记录模糊度是否固定
			float  max_Decimal_mw;     // 记录模糊度固定的最大小数部分
			Matrix matDDFixedFlag_mw;  // 记录模糊度固定成功标记
			Matrix matDDFixed_mw;      // 记录模糊度固定值
			int    count_Fixed_mw;     // mw固定个数
			bool   on_Fixed_NA;
			float  max_Decimal_NA;
			Matrix matDDFixedFlag_NA;
			Matrix matDDFixed_NA;
			int    count_Fixed_NA;     // NA固定个数 
			vector<LEOPOD_SD_AmbEq> ambSDEqList;
			vector<LEOPOD_DD_AmbEq> ambDDEqList;
		};
		struct LEOPOD_KBRArcElement
		{   
			GPST   t;             // 观测时间序号
			double obs;           // 原始KBR测距, 修正过概略值
			double range_0;       // 概略相对距离值
			double res;   
			double robustweight;  // 鲁棒估计调整权
			
			LEOPOD_KBRArcElement()
			{
				robustweight = 1.0;
				res = 0.0;
			}
		};

		struct LEOPOD_KBRArc 
		{
			double                       ambiguity;    
			vector<LEOPOD_KBRArcElement> obsList;
		};

		// 测站基站数据结构, 包括输入数据和输出轨道解部分
		struct LEOPOD_StaBaseline
		{
			int       id_priority; // 优先级，1,2,3
			float     w_function; // 观测权值
			float     ratio_UpdateREFAmb_mw;
			float     ksb_LAMBDA_mw;//默认3.0f
			float     max_AmbDecimal_mw;//最大小数部分
			float     ksb_LAMBDA_NA;
			float     max_AmbDecimal_NA;
			bool      on_DDAmbDebugInfo;
			double    minCommonViewTime; // 最小共视时间
			double    max_var_mw_sd; // 单差MW组合方差阈值
  			string    sysFreqName; // 兼容同一系统不同频率组合
			string    staName_A; 
			string    staName_B;
			vector<LEOPOD_ZD_AmbEq>   ambZDEqList_A;
			vector<LEOPOD_ZD_AmbEq>   ambZDEqList_B;

			// 下一步将重点考虑增加星间基线KBR测距数据约束
			// updateNet_NEQ_SQL 函数需要对应修改，增加KBR测距数据约束和模糊度的求解
			// 以基线为单位, 增加 updateBL_KBRObsArc（初始化弧段）、ocKBRResEdit（更新权值）、ocKBRResOutput（输出KBR残差）
			// 调用测站里插值函数分别获得对应指定时间点t的偏导数
			bool bOn_KBR_i;       // 基线i是否使用KBR数据(有可能没有)，邵凯，2020.7.28
			graceKBR1BFile            graceKBRFile;
			//gracefoKBR1BFile          gracefoKBRFile;
			vector<LEOPOD_KBRArc>     KBRArcList;
			
			// 单差文件
			LeoNetPODEditedSdObsFile editedSDObsFile;
			// 过程部分
			vector<LEOPOD_SD_AmbEq>   ambSDEqList;  // 过程数据, 调用getDDSection函数生成ambSectionList时, 会清空ambSDEqList

			// 输出部分
			int                       n0_KBRAmbiguity;
			vector<LEOPOD_DD_Section>   ambSectionList;
			int                       count_DDAmbiguity_All;
			int                       count_FixedDDAmbiguity_mw;
			int                       count_FixedDDAmbiguity_NA;

			// 根据非差模糊度列表ambZDEqList_A、ambZDEqList_B检索生成单差模糊度列表 ambSDEqList
			void getSDAmbEq();

			// 根据单差模糊度列表ambSDEqList检索双差模糊度区间ambSectionList
			void getDDSection();
			void getDDSection_new();

			// 利用lambda方法进行寻优, 搜索最佳的模糊度组合, 以满足模糊度固定的准则
			bool lambdaSelected(Matrix matAFloat, Matrix matQahat, Matrix& matSqnorm, Matrix& matAFixed, Matrix matSelectedFlag);
			// 根据ambSectionList进行双差模糊度固定
			void fixDDAmb(float ratio_UpdateREFAmb_mw, 
				          float max_AmbDecimal_mw, 
						  float max_AmbDecimal_NA, 
						  float ksb_LAMBDA_mw, 
						  float ksb_LAMBDA_NA, 
						  double COEFF_MW, 
						  double WAVELENGTH_N, 
						  bool on_DDAmbDebugInfo);
			LEOPOD_StaBaseline()
			{
				bOn_KBR_i   = false;
				id_priority = 1;
				w_function = 1.0E+2;
				on_DDAmbDebugInfo         = false;
				ratio_UpdateREFAmb_mw     = 1.5f;
				max_AmbDecimal_mw         = 0.4f;    // 兼顾原有1m软件测试使用, 适当放宽宽巷浮点解检测, 避免20060107 04:49:20-05:23:00 4个宽巷模糊度浮点解小数部分在0.3-0.4之间被删除
			    max_AmbDecimal_NA         = 0.5f;
				ksb_LAMBDA_mw             = 3.0f; 
				ksb_LAMBDA_NA             = 5.0f;
				minCommonViewTime         = 300; // 5分钟
				max_var_mw_sd             = 1.5;
			}
		};

		struct LEOPOD_DEF
		{
			bool          on_RecARP;  // 是否引入天线PCO在各个坐标轴上去投影 一般来说肯定引入啊
			bool          on_RecRelativity;
			bool          on_GraRelativity;
			bool          on_WeightElevation;
			bool          on_WeightGEO;
			bool          bOnNotUseGEO;
			bool          bOnUseOBX;//是否引入机构姿态
			float         weightGEO;
			float         max_ArcInterval;
			int           max_OrbIterationCount;     // 轨道改进次数阈值	
            int           max_AmbNum;                // 轨道改进中模糊度固定次数最大值 	
			unsigned int  min_ArcPointCount;         // 弧段观测个数阈值, 个数小于该阈值的弧段将被删除
            unsigned int  min_ArcPointCount_SD;       // 单差
            double        max_var_MW;
			double        max_var_MW_SD;         
			bool          flag_UseSingleFrequency;    // 是否使用单频观测数据，1/2(L1+C1)
			double        max_pdop;               // 几何精度因子阈值(加权后), 超过该值的观测点将不参与运算
			int           min_EyeableSatCount_kine;   // 最小可视卫星个数, 针对运动学定轨
			int           min_EyeableSatCount;       // 最小可视卫星个数, 个数小于该阈值的历元将不参与解算
			int           min_EyeableCommonnSatCount; // 最小可视共视卫星个数，+
			float         min_Elevation;
			float         priorsigma_LIF;            // 用于相位加权控制
			//float         priorsigma_RecPCO;
			//float         priorsigma_RadialEmpAcc;  // R方向常值经验力约束
			//float         priorsigma_EmpAcc_R;
			//float         priorsigma_EmpAcc_T;
			//float         priorsigma_EmpAcc_N;
			//float         priorsigma_EmpAcc_Sum;
			//float         priorsigma_ManeuverAcc;
			int           type_Cst_EmpAcc;           // 经验力约束类型, 0: 相对约束; 1: 绝对零约束; 2: 求和零均值约束; 3: 相对约束 + 求和零均值约束; 4: 绝对约束 + 求和零均值约束
			unsigned int  min_SubSectionPoints;      // 增加分段区间的最小点数
			bool          on_ocResEdit;              // 残差编辑
			bool          on_ocResEdit_NET;          // 残差编辑
			bool          on_ocResEdit_KBR;           // 残差编辑
			float         priorsigma_KBR;            // 用于KBR加权控制

			float         ratio_ocEdit;
			bool          on_KinematicPROD;         // 运动学相对定轨，邵凯，2020.12.24
			bool          on_FixDDAmbiguity;
			bool          on_FixSDAmbiguity;        // 单差模糊度固定，邵凯 +
			bool          on_FixZDAmbiguity;        // 非差模糊度固定，邵凯，2020.2.8 +
			//bool          on_DDAmbDebugInfo;
			//float         ratio_UpdateREFAmb_mw;     // 宽巷参考模糊度更换比例
			//float         max_AmbDecimal_mw;
			//float         max_AmbDecimal_NA;
			//float         ksb_LAMBDA_mw;
			//float         ksb_LAMBDA_NA;
			float         min_OrbPosIteration;     // 轨道改进的最小值
			float         min_OrbPosIteration_NET; // 整网解的轨道改进的最小值
			int           max_num_edit;           // 增加编辑次数
			double        threshold_initDynDatumEst; // 初轨确定间隔
			bool          on_UsedKBR;

			int           ambiguityFixedType_MW;       // 宽巷模糊度固定方法: 0-利用历元双差关系; 1-单差弧段直接构造双差[推荐方法] 

			LEOPOD_DEF()
			{
				on_KinematicPROD          = false;
				on_RecARP                 = true;
				on_RecRelativity          = true;
				on_GraRelativity          = true;
				on_WeightElevation        = true;
				on_WeightGEO              = false;
				bOnNotUseGEO              = false;
				bOnUseOBX                 = false;
				weightGEO                 = 1.0f / 3.0f;
				max_OrbIterationCount     = 10;
				max_AmbNum                = 8;
				min_ArcPointCount         = 30;
                min_ArcPointCount_SD      = 40;
                max_var_MW                = 0.5;  // 推荐值 GRACE 0.5 宽巷波长, 其他编队如果伪码噪声偏大, 需要适当放宽
                max_var_MW_SD             = 0.5;  // 推荐值 GRACE 0.5 宽巷波长, 其他编队如果伪码噪声偏大, 需要适当放宽
				max_ArcInterval           = 2000.0f;
				max_pdop                  = 4.5;
				min_EyeableSatCount_kine  = 5;
				min_EyeableSatCount       = 3;
				min_EyeableCommonnSatCount = 3;
				min_Elevation             = 5.0f;
				priorsigma_LIF            = 0.005f;
				priorsigma_KBR            = 0.0001f;
				type_Cst_EmpAcc           = 0;
				//priorsigma_RecPCO         = 1.0f;
				//priorsigma_EmpAcc_R       = 1E-9f; 
				//priorsigma_EmpAcc_T       = 1E-6f;
				//priorsigma_EmpAcc_N       = 1E-6f;
				//priorsigma_RadialEmpAcc   = 1E-6f;
				//priorsigma_EmpAcc_Sum     = 1E-8f;
				//priorsigma_ManeuverAcc    = 1E-4f;
				min_SubSectionPoints       = 30;      // 暂定为30个有效点, 如果10秒钟采样相当于5分钟
				on_ocResEdit               = true;
				on_FixZDAmbiguity          = false;
				on_FixDDAmbiguity          = false;
				on_FixSDAmbiguity          = false;
				//on_DDAmbDebugInfo         = false;
				ratio_ocEdit               = 3.0f;
				min_OrbPosIteration        = 5.0E-3f;  // 5 mm
				min_OrbPosIteration_NET    = 1.0E-3f;  // 1.0 mm
				//ratio_UpdateREFAmb_mw     = 1.5f;
				//max_AmbDecimal_mw         = 0.4f;    // 兼顾原有1m软件测试使用, 适当放宽宽巷浮点解检测, 避免20060107 04:49:20-05:23:00 4个宽巷模糊度浮点解小数部分在0.3-0.4之间被删除
			 //   max_AmbDecimal_NA         = 0.5f;
				//ksb_LAMBDA_mw             = 3.0f; 
				//ksb_LAMBDA_NA             = 5.0f;
				on_UsedKBR                  = false;
				on_ocResEdit_KBR            = false;
				on_ocResEdit_NET            = false;
				threshold_initDynDatumEst   = 300.0;
			    ambiguityFixedType_MW       = 1;
				max_num_edit                = 1;
				flag_UseSingleFrequency     = false;
			}
		};

		class GNSSLeoNetPOD : public SatdynBasic
		{
		public:
			GNSSLeoNetPOD(void);
		public:
			~GNSSLeoNetPOD(void);
		public:
			void setSP3File(SP3File sp3File); 
			void setCLKFile(CLKFile clkFile);
			void setIONFile(Ionex1_0_File ionFile); 
			bool loadSP3File(string strSp3FileName); 
			bool loadCLKFile(string strCLKFileName);
			bool loadCLKFile_rinex304(string strCLKFileName);
			bool loadIONFile(string strIONFileName); 
			bool adamsCowell_Interp_Leo(vector<TDT> interpTimelist, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist, vector<Matrix> &matRtPartiallist, double h = 10.0, int q = 11); 
            bool initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength = 3600 * 3); 
			
			// 单星动力学拟合，+
			bool dynamicGNSSPOD_pos(StaDatumMap::iterator it_Sta, double interval = 30.0, bool bInitDynDatumEst = false);

            void orbitExtrapolation(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0); 
			void orbitExtrapolation(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVelAcc> &forecastOrbList, double interval = 30.0); 
			void orbitExtrapolation_jerk(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVelAccJerk> &forecastOrbList, double interval = 30.0);
		public:
			bool adamsCowell_ac(TDT t0, TDT t1, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 10.0, int q = 11);
			bool mainNetPOD_MixedGNSS(GPST t0, GPST t1, double interval = 30.0); // 缩减动力学绝对定轨+附加双差模糊度的相对定轨
			bool mainNetPOD_MixedGNSS_DD(GPST t0, GPST t1, double interval = 30.0,char cSatSystem = 'G'); // 双差数据缩减动力学+运动学相对定轨
			bool mainNetPOD_MixedGNSS_DD_L1(GPST t0, GPST t1, double interval = 30.0, char cSatSystem = 'G'); // 双差数据缩减动力学+运动学相对定轨,单频L1固定解
			bool mainNetPOD_MixedGNSS_Kinematic(GPST t0, GPST t1, double interval = 30.0); // 运动学绝对定轨，暂不在运动学绝对定轨基础上进行相对定轨
			// 获得单点定位的几何精度因子pdop，+ 邵凯，2020.2.1
			bool pdopSPP(int index_P1, int index_P2, double freq_L1, double freq_L2, char cSys, Rinex2_1_LeoEditedObsEpoch obsEpoch, int& eyeableGPSCount, double& pdop);
			bool mainFuncSdPreproc(LEOPOD_StaBaseline staBL, LeoNetPODEditedSdObsFile& editedSdObsFile);
		private:
			double  m_stepAdamsCowell;
			SP3File m_sp3FileJ2000;	// + 记录J2000下Gnss卫星轨道, 方便updateSta_AdamsCowell积分后修正概略点使用
			map<string, AntCorrectionBlk> m_mapGnssPCVBlk; // + 记录Gnss卫星的PCV检索结果, 方便updateSta_AdamsCowell积分后修正概略点使用
			bool updateSta_ObsArc(StaDatumMap::iterator it_Sta); // 初始化观测数据弧段信息
			bool updateSta_ObsArc_Kinematic(StaDatumMap::iterator it_Sta); // 初始化观测数据弧段信息,运动学
			void updateSta_AdamsCowell(StaDatumMap::iterator it_Sta); // + 更新测站积分信息，动力学
			void updateSta_CorrectData(StaDatumMap::iterator it_Sta); // + 更新测站改正数据，运动学
			void updateSta_NEQ(StaDatumMap::iterator it_Sta);         // + 更新测站法方程，动力学
			void updateSta_NEQ_Kinematic(StaDatumMap::iterator it_Sta); // + 更新测站法方程,运动学
			void updateSta_SOL(StaDatumMap::iterator it_Sta); // + 更新测站解
			void updateSta_SOL_Kinematic(StaDatumMap::iterator it_Sta); // + 更新测站解,运动学
			void updateNET_KBRArc(LEOPOD_StaBaseline &staBL); // + 初始化KBR弧段信息
			void updateNet_NEQ_SQL(); // + 更新整网法方程和网解
			void updateNet_NEQ_SQL_Kinematic(); // + 更新整网法方程和网解，运动学
			void updateKBRRes(float factor = 3.0); // +更新KBR残差并编辑
		public:
			LEOPOD_DEF                     m_podDefine;
			CLKFile                        m_clkFile;			   // 精密钟差数据文件
			SP3File                        m_sp3File;			   // 精密星历数据文件
			wsbFile                        m_wsbFile;              // wsb文件
			Ionex1_0_File                  m_ionFile;              // IGS电离层网格产品
			igs05atxFile			       m_atxFile;		       // 天线修正文件
			svnavMixedFile                 m_svnavMixedFile;
			GNSSYawAttitudeModel           m_gymMixed;             // +
			GNSS_AttFile                   m_gnssAttFile;        // GNSS姿态数据文件
			//POS3D                        m_arpAnt;		       // 天线偏心量 (测站PCO)
			StaDatumMap                    m_mapStaDatum;          // 多星数据 + 
			vector<LEOPOD_MixedSys>        m_dataMixedSysList;     // 混合系统定义 +
			vector<LEOPOD_StaBaseline>     m_staBaselineList;      // 暂时先利用外部输入 + 
			vector<O_CResEpoch>            m_ocResP_IFEpochList;
			vector<O_CResEpoch>            m_ocResL_IFEpochList;
		};
	}
}
