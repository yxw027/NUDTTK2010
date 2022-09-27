#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Matrix.hpp"
#include <windows.h>
#include <vector>
#include <map>
#include "SatdynBasic.hpp"
#include "Rinex2_1_EditedObsFile.hpp"
#include "Rinex2_1_EditedSdObsFile.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	namespace Geodyn
	{
		// 观测方程历元元素
		struct ObsEqEpochElement 
		{ 
			BYTE   id_Sat;        // 卫星号
			int    id_Ambiguity;  // 模糊度序号
			double obs;           // 原始无电离层组合观测量
			double res;
			double robustweight;  // 鲁棒估计调整权

			ObsEqEpochElement()
			{
				robustweight = 1.0;
				res = 0.0;
			}
		};

		struct ObsEqEpoch
		{
			int                        nObsTime;    // 观测时间序号, 用于索引关键数据
			vector<ObsEqEpochElement>  obsSatList;  // 不同卫星观测列表
		};

		// 观测方程弧段元素
		struct ObsEqArcElement
		{   
			BYTE   id_Sat;       // GPS卫星号    
			int    nObsTime;     // 观测时间序号
			double obs;          // 原始相位无电离层组合观测量
			double obs_mw;       // 原始 MW 组合观测量
			double res;   
			double robustweight; // 鲁棒估计调整权
			
			ObsEqArcElement()
			{
				robustweight = 1.0;
				res = 0.0;
			}
		};

		typedef map<int, ObsEqArcElement> ObsEqArcElementMap;

		// 增加模糊度固定相关定义，邵凯，2019/07/06
		struct ObsEqArc 
		{
			GPST               t0;              // 开始时刻
			GPST               t1;              // 结束时刻
			GPST               t;               // 弧段中间时刻
			double             ambiguity;       // IF 模糊度概略解：初始模糊度通过伪码和相位之差平均获得，之后通过迭代改进
			double             ambiguity_MW;    // MW 模糊度概略解：浮点解通过弧段平均值的方式计算，单位：周
			double             ambFixedWL;      // 宽巷模糊度固定解：宽巷模糊度固定解通过 MW 组合方式计算
			bool               bOn_fixedWL;     // 该弧段宽巷模糊度固定成功与否
			double             ambiguity_NL;    // 窄巷 N1 模糊度概略解
			double             res_WL;          // 宽巷模糊度固定解残差
			double             res_NL;          // 窄巷模糊度固定解残差
			double             ambFixedNL;      // 窄巷模糊度固定解：窄巷模糊度固定解 IF 浮点解+ WL 固定解获得
			bool               bOn_fixedNL;     // 该弧段窄模糊度固定成功与否
			int                id_Ambiguity_GL; // 模糊度全局编号, 辨识分段后不同段内的模糊度是否属于同一模糊度
			ObsEqArcElementMap obsList;
			double             rms_oc;          // 2015/01/25,刘俊宏，用于计算弧段的残差的rms,编辑超差弧段(测试阶段)
			double             rms_mw;          // 弧段 MW 变化方差
			int                count_valid;     // 有效数据个数
			BYTE               id_sat;          // 卫星号
			bool updateRobustWeight(double threshold_slip, double threshold_rms, unsigned int threshold_count_np = 20, double robustfactor = 3.0);
			// 模糊度固定情况下，相位观测数据加权，弧段不删除 + ，邵凯，2020.1.7
			bool updateRobustWeight_fix(double threshold_slip, double threshold_rms, unsigned int threshold_count_np = 20, double robustfactor = 3.0);

			ObsEqArc()
			{
				ambiguity_NL = 0.0;
				ambFixedWL = 0.0;
				ambFixedNL = 0.0;
				res_WL     = 0.0;
				res_NL     = 0.0;
				rms_oc     = 0.0;
				rms_mw     = 0.0;
				bOn_fixedWL = false;
				bOn_fixedNL = false;
			}
		};

		struct SdObsEqEpoch 
		{
			int                          nObsTime;          // 观测时间序号, 用于索引关键数据
			int                          id_DDObs;          // 双差GPS参考卫星的观测数据序号
			int                          id_DDRefSat;       // 双差GPS参考卫星序号
			int                          id_DDRefAmbiguity; // 双差参考模糊度序号
			vector<ObsEqEpochElement>    obsSatList;        
			
			SdObsEqEpoch()
			{
				id_DDRefSat       =  NULL_PRN;
				id_DDRefAmbiguity = -1;
				id_DDObs          = -1;
			}
		};

		// 定轨残差相关数据结构
		struct O_CResEpochElement 
		{ 
			BYTE   id_Sat;       // 卫星号
			double res;
			double robustweight; // 鲁棒估计调整权
			double Elevation;    // 天线方向图显示需要, 2010/11/03
			double Azimuth;

			O_CResEpochElement()
			{
				robustweight = 1.0;
				res = 0;
			}
		};

		struct O_CResEpoch
		{
			GPST t;      
			vector<O_CResEpochElement> ocResSatList;
		};

		struct O_CResArcElement
		{   
			BYTE    id_Sat;  
			GPST    t;               
			double  res;   
			double  robustweight;     
			double  Elevation; 
			double  Azimuth;
			
			O_CResArcElement()
			{
				robustweight = 1.0;
				res = 0.0;
			}
		};

		struct O_CResArc 
		{
			double ambiguity; // 模糊度概略解
			vector<O_CResArcElement> ocResEpochList;
		};

		struct PODEpochElement
		{
			double  obscorrected_value;      // 观测数据改正量-相位
			double  obscorrected_value_code; // 观测数据改正量-伪码
			double  ionosphereDelay;         // 视线方向电离层延迟, 用于记录单频修正值
			POSCLK  vecLos_A;                // 视线矢量, 单星定轨时使用
			POSCLK  vecLos_B;                // 视线矢量, 相对定轨时使用			
			double  weightCode;              // 伪码观测权值
			double  weightPhase;             // 相位观测权值
			Matrix  interpRtPartial_A;       // 偏导数, GPS卫星定轨时使用
			Matrix  interpRtPartial_B;     
			double  wmf_A;                   // 对流层湿分量映射函数（偏导数），用于对流层湿分量估计
			double  wmf_B;
			BYTE    mark_GNSSSatShadow;      // 进入地影标记, 默认 0 — 未进入地影
			bool    bEphemeris;
			TimePosVel pvEphemeris;          // 记录卫星轨道位置检索结果
			double  reserved_value;          // 保留字段, 暂不使用, 为了编写算法使用保留
			double  nadir;                   // 当前GPS卫星对测站（LEO）天底角, 邵凯，2019.10.2
			PODEpochElement()
			{
				wmf_A = 0;
				wmf_B = 0;
				mark_GNSSSatShadow = 0;
				nadir = 0;
			}
		};

		typedef map<int, PODEpochElement> PODEpochSatMap;

		struct PODEpoch
		{
			int                validIndex;   // 有效时间标签  (用于相位观测方程设计矩阵索引有效位置参数号）
			int                eyeableGPSCount;
			PODEpochSatMap     mapDatum;
		};

		struct AMBIGUITY_SECTION
		{
			int                  id_t0;                        // 起始时刻
			int                  id_t1;                        // 结束时刻
			vector<ObsEqArc>     mw_ArcList;                   // 记录分段后弧段数据
			vector<ObsEqArc>     L_IF_ArcList;
			vector<ObsEqArc>     P_IF_ArcList;
			vector<SdObsEqEpoch> mw_EpochList;                 // 记录分段后时刻数据,  构造双差时需要根据时刻进行
			vector<SdObsEqEpoch> L_IF_EpochList;
			vector<SdObsEqEpoch> P_IF_EpochList;
			
			bool                 bDDAmFixed_MW;                // 标记宽巷模糊度整体固定是否成功
			Matrix               matDDFixedFlag_MW;            // 标记每个宽巷模糊度固定是否成功, 0: 未固定, 1: 已固定
			vector<double>       ambiguity_DD_MW_List;         // 每个宽巷模糊度固定结果, 未固定的采用浮点解

			bool                 bDDAmFixed_L1;                // 标记窄巷模糊度整体固定是否成功
			Matrix               matDDFixedFlag_L1;            // 标记每个窄巷模糊度固定是否成功
			int                  count_DD_L1_UnFixed;          // 记录窄巷模糊度未固定的个数, 初始化为宽巷模糊度个数, 经过窄巷模糊度固定后会更新
			vector<double>       ambiguity_DD_L1_list;         // 每个窄巷模糊度固定结果, 未固定的采用浮点解
			vector<int>          ambiguity_DD_L1_UnFixed_list; // 模糊度固定标记, 已固定: -1; 未固定: 0, 1, 2, ..., max = count_DD_L1_UnFixed - 1, 用于计算 O-C 残差
			Matrix               matDDAdjust_L1;               // 窄巷模糊度改进结果
			Matrix               N_bb;                         // 窄巷模糊度浮点解协方差矩阵
			Matrix               n_xb;   
			vector<Matrix>       n_xb_List;                // + 交叉项 for 运动学
			Matrix               nb;

			bool ArcList2EpochList(vector<ObsEqArc>& arcList, vector<SdObsEqEpoch>& epochList, int min_arcpointcount = 20, bool bOn_Loop = true);

			AMBIGUITY_SECTION()
			{
				bDDAmFixed_MW = false;
			}
		};

		//struct DDFIXEDAMBVALID // 增加宽巷模糊度校核程序, 2016-11-27, 谷德峰
		//{
		//	int         ambiguity;
		//	vector<int> sectionList;
		//  bool        flag;
		//	DDFIXEDAMBVALID()
		//	{
		//		ambiguity = 0;
		//		sectionList.clear();
		//		flag = true;
		//	}
		//};

		struct Sp3Fit_SatdynDatum
		{
			SatdynBasicDatum    dynamicDatum_Init;     // 初始轨道, 从sp3文件读取
			SatdynBasicDatum    dynamicDatum_Est;      // 估计的轨道结果
			//SatdynBasicDatum  dynamicDatum_Predict;  // 预报轨道参数, 拟合后输出, 供下一天使用
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

		typedef map<BYTE, Sp3Fit_SatdynDatum> Sp3Fit_SatdynDatumMap;

		struct Sp3FitParameter
		{
			Sp3Fit_SatdynDatumMap satParaList;
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

			Sp3FitParameter()
			{
				xp     = 0;
				xpDot  = 0;
				yp     = 0;
				ypDot  = 0;
				ut1    = 0;
				ut1Dot = 0;
			}
		};

		struct SatDatum
		{
			SatdynBasicDatum    dynamicDatum_Init;     // 初始轨道
			SatdynBasicDatum    dynamicDatum_Est;      // 估计的轨道结果
			vector<TimePosVel>  orbitList_ECEF;        // 轨道, 地固系
			vector<TimePosVel>  acOrbitList;           // 用于插值轨道序列
			vector<Matrix>      acRtPartialList;       // 用于插值偏导数序列
			int                 count_obs;             // 观测数据总数
			int                 index;                 // 待估参数块的位置
			int                 dynamicIndex_0;        // 卫星的首个力学参数在整个估计参数列表中的位置，2015/11/16，刘俊宏	

			bool getEphemeris(TDT t, TimePosVel& gpsOrb, int nLagrange = 9);
			bool getInterpRtPartial(TDT t, Matrix& interpRtPartial);
			bool getEphemeris_PathDelay(TDT t, POSCLK staPosClk, double& delay, TimePosVel& gpsOrb, Matrix& gpsRtPartial, double threshold = 1.0E-07);
		};

		typedef map<BYTE, SatDatum> SatDatumMap;
		
		struct SatOrbEstParameter
		{
			SatDatumMap satParaList;
			GPST   t0_xpyput1;
			double xp;           // 单位: 弧度
			double xpDot;        // 单位: 弧度/秒
			double yp;
			double ypDot;
			double ut1;          // 单位: 弧度
			double ut1Dot;       // 单位: 弧度/秒
			
			SatOrbEstParameter()
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

		struct TropZenithDelayEstPara
		{
			GPST    t;                  // 对流层估计的时间	
			double  zenithDelay_Est;    // 对流层估计结果
			double  zenithDelay_Init;   // 对流层湿分量估计初值
			int     nValid_Epoch;       // 有效历元个数
		};
		struct TropGradEstPara
		{
			GPST    t;                  // 对流层梯度估计的时间	
			double  troGrad_NS;         // 对流层南北方向梯度估计结果（初始值默认为0）
			double  troGrad_EW;         // 对流层东西方向梯度估计结果（初始值默认为0）
			int     nValid_Epoch;       // 有效历元个数
		};

		typedef map<int,    POS3D>    StaPosMap;

		struct StaDatum
		{
			GPST                           t0;               // 观测数据起始时间
            GPST                           t1;               // 观测数据起始时间
			POS6D                          posvel;
            ENU                            arpAnt;
			Rinex2_1_EditedObsFile         obsFile;
			string                         pathObsFile;      // 测站观测数据文件目录
			int                            index_P1;
			int                            index_P2;
			int                            index_L1;
			int                            index_L2;
			bool                           bUsed;
			int                            id;               // 序号, 用于标记在整个参数估计的序列位置, 从 0 开始
			// 位置估计块
			char		                   szAntType[20 + 1];// 天线类型
			bool                           bOnEst_StaPos;
			double                         sigma;            // 控制测站位置估计的约束条件，2012.12.09，刘俊宏
			int                            indexEst_StaPos;  // 测站位置估计序号, 根据bOnEst_StaPos开关进行排列
			POS3D                          pos_Est;          // 位置估计结果
			int                            count_obs_dd;     // 统计双差观测方程个数， 用于测站参数约束
            // 对流层估计块
			vector<TropZenithDelayEstPara> zenithDelayEstList;
			int                            zenithIndex_0;    // 测站的首个对流层估计参数在整个估计参数列表中的位置			
			// 钟差估计块
			vector<ObsEqArc>  L_IFArcList;      // 与观测数据obsFile相对应
			map<GPST, int>    mapEpochId;       // 时间索引
			vector<PODEpoch>  kineEpochList;    // 记录每个时刻的观测权值信息
			vector<POS3D>     corrTideList;     // 存储测站坐标潮汐改正量[地固系], 避免反复计算

			vector<ObsEqArc>  mw_ArcList;       // 与观测数据obsFile相对应

			// 解算过程测站坐标
			StaPosMap                 staPosList;     //测站A在惯性系下的坐标列表
			StaPosMap                 staECEFPosList; //测站A在地固系下的坐标列表

			// 地球旋转参数矩阵
			vector<Matrix>            matPR_NRList;
			vector<Matrix>            matERList_0;    // 概略地球旋转矩阵
			vector<Matrix>            matEPList_0;    // 概略极移旋转矩阵
		    vector<Matrix>            matEPList;      // 改进后的地球旋转矩阵
			vector<Matrix>            matERList;      // 改进后的地球旋转矩阵
			
			StaDatum()
			{
				bUsed          = true;
				id             = -1;
				count_obs_dd   = 0;
				bOnEst_StaPos  = true;
				sigma          = 0.01;
				index_P1       = -1;
				index_P2       = -1;
				index_L1       = -1;
				index_L2       = -1;
			}

			void init(double period_TropZenithDelay = 3600 * 2.0,double apriorityWet_TZD = 0);	
			int  getIndexZenithDelayEstList(DayTime t);      // 确定时刻t在对流层参数列表中的位置
			bool getIndexP1P2L1L2();
		};

		typedef map<string, StaDatum> StaDatumMap;
		
		struct StaBaselineDatum
		{
			string                    name_A;
			POS6D                     posvel_A;
            ENU                       arpAnt_A;
			string                    name_B;  
			POS6D                     posvel_B;
            ENU                       arpAnt_B;
			Rinex2_1_EditedSdObsFile  editedSdObsFile;
			int                       ddObs_count;
			double                    weight_baseline; // 测站观测数据
			double                    rms_oc_code;
            double                    rms_oc_phase;
			// 解算过程结构数据
			vector<PODEpoch>          dynEpochList;
			StaPosMap                 staPosList_A;     //测站A在惯性系下的坐标列表
			StaPosMap                 staPosList_B;     //测站A在惯性系下的坐标列表
			StaPosMap                 staECEFPosList_A; //测站A在地固系下的坐标列表
			StaPosMap                 staECEFPosList_B; //测站A在地固系下的坐标列表
			// 地球旋转参数矩阵
			vector<Matrix>            matPR_NRList;
			vector<Matrix>            matERList_A_0;    // 概略地球旋转矩阵, 测站 A
			vector<Matrix>            matERList_B_0;    // 概略地球旋转矩阵, 测站 B
			vector<Matrix>            matEPList_0;      // 概略极移旋转矩阵
		    vector<Matrix>            matEPList;        // 改进后的地球旋转矩阵, 测站 A
			vector<Matrix>            matERList_A;      // 改进后的地球旋转矩阵, 测站 B
			vector<Matrix>            matERList_B;		// 改进后的概略极移旋转矩阵
			//vector<Matrix>          matERDOTList_A;
			//vector<Matrix>          matERDOTList_B;
			// 基线模糊度分块结构
			bool                      bOn_AmbiguityFix;   // 用于标记基线模糊度固定选择策略, 例如基线长度超 5000 km, 可以选择浮点解
			vector<AMBIGUITY_SECTION> amSectionList;
			int                       count_DD_MW_Fixed;
			Matrix                    N_bb; 
			Matrix                    N_bb_inv;           // N_bb矩阵的逆，避免重复计算，2015/01/15 
			Matrix                    n_xb; 
			Matrix                    nb;   
			Matrix                    matQ_dd_L1;
			Matrix                    matdb;
			// 定轨残差
			vector<O_CResEpoch>       ocResP_IFEpochList;
			vector<O_CResEpoch>       ocResL_IFEpochList;
			// 对流层
			vector<int>               id_ZenithDelayList_A;// 每个时刻对流层估计参数在该测站对流层参数列表中的位置
			vector<int>               id_ZenithDelayList_B;// 每个时刻对流层估计参数在该测站对流层参数列表中的位置
			
			int getDDObsCount();

			double getStaDistance()
			{
				POS3D vec_A_B = posvel_B.getPos() - posvel_A.getPos();
				return sqrt(vec_A_B.x * vec_A_B.x + vec_A_B.y * vec_A_B.y + vec_A_B.z * vec_A_B.z);
			}
			
			StaBaselineDatum()
			{
				bOn_AmbiguityFix = true; 
				weight_baseline  = 1.0;
				ddObs_count  = 0;
				rms_oc_code  = 0;
				rms_oc_phase = 0;
			}
		};
	}
}
