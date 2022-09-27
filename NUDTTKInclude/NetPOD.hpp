#pragma once
#include "structDef.hpp"
#include "SatdynBasic.hpp"
#include "dynPODStructDef.hpp"
#include "MathAlgorithm.hpp"
#include "SP3File.hpp"
#include "CLKFile.hpp"

//  Copyright 2018, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
using namespace NUDTTK::Geodyn;
using namespace NUDTTK::Math;

// 基于星间链路的整网定轨
namespace NUDTTK
{
	namespace NETPOD
	{
		// 钟差解部分
		struct NETPOD_CLKSOL
		{
			GPST    t;
			double  clock;
		};
		// 卫星数据结构, 包括输入数据和输出轨道解部分
		struct NETPOD_StaDatum 
		{
			// 输入部分
			GPST      t0;                     // 起始时间
			GPST      t1;                     // 结束时间
            double    interval;               // 数据采样间隔
			string    staName;
			string    pathFolder;
			size_t    count_MixedEpoch;    
			double    orbOutputInterval;
			bool      on_IsUsed_InitDynDatum;
			double    ArcLength_InitDynDatum;
			SatdynBasicDatum dynDatum_Init;     // 初始值动力学轨道信息
			double    period_SolarPressure;    // 目前网解卫星编队轨道应当高度相近, 分段区间的定义大致统一

			vector<Matrix>              attMatrixList;
			vector<int>                 attFlagList;
			vector<POS3D>               attXYZAntList[3];
			vector<TimePosVel>          interpOrbitlist;     // 插值序列
			vector<Matrix>              interpRtPartiallist; // 插值偏导数序列
			vector<TDT>                 interpTimelist;
			vector<TimePosVel>          acOrbitList;         // 用于插值轨道序列 getEphemeris
			vector<Matrix>              acRtPartialList;     // 用于插值偏导数序列 getInterpRtPartial

			// 输出部分
			int                   n0_EstParameters;         // 记录合并后待估参数的分块起点, 便于分块检索
			int                   count_EstParameters;      // 不含钟差: count_EstDynParameters + count_EstAmbiguity + count_EstRecPCO + count_EstSysBias
			int                   count_EstDynParameters;   // 动力学参数个数
			int                   count_EstClockParameters; // 钟差参数个数
			int                   n0_SolarPressure;         // 分块内部位置, 记录太阳光压参数起始位置
			int                   n0_EmpiricalForce;        // 分块内部位置, 记录经验力参数在起始位置
			SatdynBasicDatum      dynDatum_Est;  // 估计值动力学轨道信息
			vector<TimePosVel>    orbList; // 轨道估计结果（包含预报部分）
			vector<NETPOD_CLKSOL> clkList; // 钟差估计结果（动力学）
			Matrix                matN_aa; // 动力学参数、模糊度、天线偏移、系统偏差
			Matrix                matN_a;
			Matrix                matda;
			//////////////////////////
			bool getEphemeris(TDT t, TimePosVel& interpOrbit, int nLagrange = 9);
			bool getEphemeris_ITRF(GPST t, TimePosVel& interpOrbit, int nLagrange = 9); // 地固系
			bool getInterpRtPartial(TDT t, Matrix& interpRtPartial);
			// 更新轨道改进结果
			void updateDynDatum();
			// 保存轨道改进结果
			void writeDynDatum();
			// 残差编辑
			void ocResEdit(float factor = 3.0f, bool flag_UseSingleFrequency = false);
			void ocResEdit_NET(float factor = 3.0f);
			// 更新残差
			void ocResOutput();
			// 计算待估参数初始位置: n0_SolarPressure、n0_AtmosphereDrag、n0_EmpiricalForce、n0_ManeuverForce、n0_RecPCO、n0_SysBias
			void getEstParameters_n0();
			//*******************************************************************************
			// 输出钟差文件
			void getRecClkFile(CLKFile& recClkFile); 

			NETPOD_StaDatum()
			{
				count_MixedEpoch = 0;
				orbOutputInterval = 30.0;
				on_IsUsed_InitDynDatum = false;
				ArcLength_InitDynDatum = 3600.0 * 3.0;
				period_SolarPressure  = 3600 * 24.0; 
			}
		};
		typedef map<string, NETPOD_StaDatum> StaDatumMap;

		// 北斗卫星数据结构, 输入数据
		struct NETBDS_StaDatum
		{
			GPST      t0;                     // 起始时间
			GPST      t1;                     // 结束时间
			string    staName;
			vector<TimePosVel>          pOrbitlist;     // 轨道序列
			//////////////////////////
			bool getEphemeris(TDT t, TimePosVel& interpOrbit, int nLagrange = 9);

		};
		typedef map<string, NETBDS_StaDatum> BDSDatumMap;

		struct NETPOD_KBRArcElement
		{   
			GPST   t;             // 观测时间序号
			double obs;           // 原始KBR测距, 修正过概略值
			double range_0;       // 概略相对距离值
			double res;   
			double robustweight;  // 鲁棒估计调整权
			
			NETPOD_KBRArcElement()
			{
				robustweight = 1.0;
				res = 0.0;
			}
		};

		struct NETPOD_KBRArc 
		{
			double                       ambiguity;    
			vector<NETPOD_KBRArcElement> obsList;
		};

		struct NETPOD_DEF
		{
			bool          on_RecRelativity;
			float         max_ArcInterval;
			int           max_OrbIterationCount;     // 轨道改进次数阈值	
            int           max_AmbNum;                // 轨道改进中模糊度固定次数最大值 	
			unsigned int  min_ArcPointCount;         // 弧段观测个数阈值, 个数小于该阈值的弧段将被删除    
			unsigned int  min_SubSectionPoints;      // 增加分段区间的最小点数
			float         min_OrbPosIteration;     // 轨道改进的最小值
			int           max_num_edit;           // 增加编辑次数
			double        threshold_initDynDatumEst; // 初轨确定间隔
			NETPOD_DEF()
			{
				on_RecRelativity          = true;
				max_OrbIterationCount     = 10;
				max_AmbNum                = 8;
				min_ArcPointCount         = 30;
				max_ArcInterval           = 2000.0f;
				min_SubSectionPoints      = 30;      // 暂定为30个有效点, 如果10秒钟采样相当于5分钟
				min_OrbPosIteration       = 5.0E-3f;  // 5 mm
				threshold_initDynDatumEst = 300.0;
				max_num_edit              = 1;
			}
		};
				
		struct NETISLObsEpoch
		{
			DayTime                  t;
			string                   sat_A;
			string                   sat_B;
			double                   obs_AB;  
			double                   obs_BA; 
			double                   d;
			double                   clk_A;
			double                   clk_B; 

			//double                   obs_d; // 距离量obs
			//double                   obs_c; // 钟差量obs
			NETISLObsEpoch()
			{
				obs_AB = 0.0;
				//obs_d  = 0.0;
				//obs_c  = 0.0;
			}
			double getobs_d()
			{
				double obs_d = (obs_AB+obs_BA)/2;
				return obs_d;
			}
			double getobs_c()
			{
				double obs_c = (obs_AB-obs_BA)/2;
				return obs_c;
			}
		};
		// 测站基站数据结构
		struct NETPOD_StaBaseline
		{
			string    staName_A; 
			string    staName_B;
			vector<NETISLObsEpoch>   m_data;
		};

		struct NETPOD_Sta_BDSBaseline
		{
			string    staName_A; 
			string    staName_BDS;
			vector<NETISLObsEpoch>   m_data;
		};

		class NetPOD: public SatdynBasic
		{
		public:
			NetPOD(void);
		public:
			~NetPOD(void);
		public:
			bool adamsCowell_Interp_Leo(vector<TDT> interpTimelist, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist, vector<Matrix> &matRtPartiallist, double h = 10.0, int q = 11); 
            bool initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength = 3600 * 3); 
            void orbitExtrapolation(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0); 
			void orbitExtrapolation(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVelAcc> &forecastOrbList, double interval = 30.0); 
			void orbitExtrapolation_jerk(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVelAccJerk> &forecastOrbList, double interval = 30.0);
		public:
			bool adamsCowell_ac(TDT t0, TDT t1, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 10.0, int q = 11);
			bool mainNetPOD_InterSatLink(); // 基于星间测量的整网定轨
		private:
			void updateNET_ISL();      // + 初始化ISL信息, 更新残差
			void updateSta_AdamsCowell(StaDatumMap::iterator it_Sta); // + 更新测站积分信息
			void updateNet_NEQ_SQL();
		public:
            double                         m_stepAdamsCowell;
			NETPOD_DEF                     m_podDefine;
			StaDatumMap                    m_mapStaDatum;          // 多星数据 + 
			BDSDatumMap                    m_mapBDSDatum;          // BDS数据 + 
			vector<NETPOD_StaBaseline>     m_staBaselineList;      // 利用外部输入 + 
			vector<NETPOD_Sta_BDSBaseline> m_sta_BDSBaselineList;  // 地月空间导航星座与BDS建链数据
		};
	}
}
