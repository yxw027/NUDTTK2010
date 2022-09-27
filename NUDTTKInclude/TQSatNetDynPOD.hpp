#pragma once
#include"structDef.hpp"
#include"SatdynBasic.hpp"
#include<map>
#include"TimePosVelFile.hpp"
#include"MathAlgorithm.hpp"
#include"TQUXBObsFile.hpp"
#include"TQISLObsFile.hpp"
#include"SLRSatDynPOD.hpp"
#include"RuningInfoFile.hpp"

// Copyright 2018, SUN YAT-SEN UNIVERSITY TianQin Research Center at ZhuHai
using namespace NUDTTK;
using namespace NUDTTK::Math;
using namespace NUDTTK::Geodyn;

// 深空网+激光测距，重新梳理，2022/03/15

namespace NUDTTK
{
	namespace TQPod
	{
		// 统一S/X波段（USB/UXB）测量数据, 单程or双程？ 增加弧段概念？
		struct TQNETPOD_UXBStaDatum
		{
			string                   nameSta;         // 测站名 "XXXX"
            string                   nameSat;         // 卫星名 "TQ01" +
			POS3D                    pos_ITRF;        // 测站在ITRF框架下的位置
			map<UTC, TQUXBObsLine>   obsList;         // 观测数据列表

			double                   zeroDelay_0;     // 测站距离零值延迟
			double                   zeroDelay_Est;   // 测站距离零值估计值
			int                      id_ZeroDelay;    // 下标从零开始
			bool                     on_EstStaZeroDelay;
			// 注: 多星定轨时，同一测站的零值延迟公用一个参数，此时增加相对约束
			TQNETPOD_UXBStaDatum()
			{
				zeroDelay_0        = 0.0;
				zeroDelay_Est      = 0.0;
				id_ZeroDelay       = -1;
				on_EstStaZeroDelay = true;
			}
		};
		typedef map<string, TQNETPOD_UXBStaDatum> TQNETPOD_UXBStaDatumMap;

		// SLR 测量数据格式，定义？统一到与UXB观测数据定义类似？




		struct TQNETPOD_SatDatum
		{
			// 输入部分
			string                      satName;
			TQNETPOD_UXBStaDatumMap     mapUXBStaDatum;
			UTC                         t0;
			UTC                         t1;
			TimePosVelFile              orbJ2000File_0;           // 概略先验轨道 +，J2000？
			SatdynBasicDatum            dynDatum_Init;            // 初始值动力学轨道信息
			TYPE_SOLARPRESSURE_MODEL    solarPressure_Model;      // 太阳光压模型
			double                      period_SolarPressure;     // 太阳光压周期
			double                      period_EmpiricalAcc;      // 经验力加速度周期
			bool                        on_SRP9_D0;               // 是否估计光压D0参数
			bool                        on_SRP9_DC1;              // 是否估计光压DC1参数
			bool                        on_SRP9_DS1;              // 是否估计光压DS1参数	
			bool                        on_SRP9_Y0;               // 是否估计光压Y0参数
			bool                        on_SRP9_YC1;              // 是否估计光压YC1参数
			bool                        on_SRP9_YS1;              // 是否估计光压YS1参数
			bool                        on_SRP9_B0;               // 是否估计光压B0参数
			bool                        on_SRP9_BC1;              // 是否估计光压BC1参数
			bool                        on_SRP9_BS1;              // 是否估计光压BS1参数
			bool                        on_EstSatZeroDelay;       // 是否估计星上零点延迟参数
			double                      satZeroDelay_0;           // 星上零点延迟
            
			// 积分信息
			vector<TimePosVel>          acOrbitList;              // 用于插值轨道序列 getEphemeris
			vector<Matrix>              acRtPartialList;          // 用于插值偏导数序列 getInterpRtPartial
			
			// 输出部分
			int                         n0_EstParameters;         // 记录合并后待估参数的分块起点, 便于分块检索
			int                         count_EstParameters; 
			int                         count_EstDynParameters;
			int                         count_EstSatZeroDelay;
			int                         count_EstStaZeroDelay;
			int                         n0_SolarPressure;         // 分块内部位置, 记录太阳光压参数起始位置
			int                         n0_EmpiricalForce;        // 分块内部位置, 记录经验力参数在起始位置
			int                         n0_ManeuverForce;         // 分块内部位置, 记录机动力参数在起始位置
			SatdynBasicDatum            dynDatum_Est;             // 估计值动力学轨道信息
			vector<TimePosVel>          orbList;                  // 轨道估计结果（包含预报部分）
			double                      satZeroDelay_Est;

            // 残差信息
			int                         count_obs;
			double                      ocResRMS_Range;
			double                      ocResRMS_Doppler;
            // 法方程信息
			Matrix                      n_xx; 
			Matrix                      nx;
			Matrix                      dx;

			bool getEphemeris(UTC t, TimePosVel& interpOrbit, int nLagrange = 9);
			bool getInterpRtPartial(UTC t, Matrix& interpRtPartial);
		    bool getEphemeris_PathDelay(UTC t, POS3D staPos, double& delay, TimePosVel& tqOrb, Matrix& tqRtPartial, double threshold = 1.0E-8);
			
			// 计算待估参数初始位置: n0_SolarPressure、n0_EmpiricalForce、n0_ManeuverForce
			void getEstParameters_n0();

			// 更新残差
			void ocResOutput();

			// 残差编辑
			void ocResEdit(double factor);

			// 更新轨道改进结果
			void updateDynDatum();

			// 保存轨道改进结果
			void writeDynDatum(string pathDynPodFitFile);

			TQNETPOD_SatDatum()
			{
				period_SolarPressure       = 3600.0 * 24.0;  // 保持与仿真时的光压周期一致
				period_EmpiricalAcc        = 3600.0 * 24.0;
				solarPressure_Model        = TYPE_SOLARPRESSURE_9PARA; 
				on_EstSatZeroDelay         = false;
				satZeroDelay_0             = 0.0;
				satZeroDelay_Est           = 0.0;
				count_obs                  = 0;
				ocResRMS_Range             = 0.0;
				ocResRMS_Doppler           = 0.0;

				on_SRP9_D0                 = true;
				on_SRP9_DC1                = true;
				on_SRP9_DS1                = true;
				on_SRP9_Y0                 = true;
				on_SRP9_YC1                = true;
				on_SRP9_YS1                = true;
				on_SRP9_B0                 = true;
				on_SRP9_BC1                = true;
				on_SRP9_BS1                = true;
			}
		};

		typedef map<string, TQNETPOD_SatDatum> TQNETPOD_SatDatumMap;

		// 星间链路数据
		struct TQNETPOD_ISLArcElement
		{   
			GPST   t;             // 观测时间序号
			double obs_code;      // 原始测距, 修正过概略值
			double obs_phase;     // 原始测距, 修正过概略值
			double range_0;       // 概略相对距离值
			double oc_code;   
			double oc_phase;  
			double rw_code;       // 鲁棒估计调整权
			double rw_phase;
			
			TQNETPOD_ISLArcElement()
			{
				rw_code  = 1.0;
				rw_phase = 1.0;
				oc_code  = 0.0;
				oc_phase = 0.0;
			}
		};

		typedef map<UTC, TQNETPOD_ISLArcElement> ISLArcElementMap;

		struct TQNETPOD_ISLArc
		{
			double                         ambiguity;    // ？模糊度
			vector<TQNETPOD_ISLArcElement> obsList;
		};

		// 星间链路数据结构
		struct TQNETPOD_InterSatLink
		{
			string    satName_A;
			string    satName_B;
			// 下一步将重点考虑增加星间基线激光测距数据约束 ?????????
			//ISLFile                   tqISLFile;
			vector<TQNETPOD_ISLArc>     ISLArcList;
			TQISLObsFile                tqISLFile;

			// 输出部分
			int                         n0_ISLAmbiguity;
		};


		struct TQNETPOD_DEF
		{
			// 数据的编辑信息统一放到预定义里
			bool                     on_TropDelay;         // 是否考虑对流层延迟效应
			bool                     on_SolidTides;        // 是否考虑固体潮效应
			bool                     on_OceanTides;        // 是否考虑海潮效应
			bool                     on_GraRelativity;     // 是否考虑广义相对论效应
			bool                     on_SatPCO;            // 星上相位中心误差
			bool                     on_ObsEditedInfo;  
			bool                     on_ResEdit;
			bool                     on_EstStaZeroDelay; 
			bool                     on_Used_Range;        // 是否使用测距数据定轨
			bool                     on_Used_Doppler;      // 是否使用测速数据定轨
			bool                     on_Used_ISL_Code;     // Inter satellite link, 星间链路伪距
			bool                     on_Used_ISL_Phase;    // Inter satellite link, 星间链路相位
			double                   min_Elevation;        // 最小高度角阈值
			double                   max_Edit_OcRange;     // 测距残差最大阈值，用于剔除一些残差超大的点
			double                   max_Edit_OcDoppler;   // 测速残差最大阈值 // 暂时添加 
			double                   ratio_ocEdit;         // 残差编辑比率  
			int                      min_ArcPointCount;    // 观测数据次数最小值
			int                      max_OrbIteration;     // 轨道改进次数最大值
			string                   nameDynPodFitFile;
			double                   min_OrbPosIteration;  // 轨道改进的最小值
			double                   weight_doppler;       // 对测速加权，法方程加权
			double                   weight_code;          // 对伪码加权
			double                   weight_phase;         // 对相位加权

			 TQNETPOD_DEF()
			{
				on_TropDelay               = true;
				on_SolidTides              = true;
				on_OceanTides              = true;
				on_GraRelativity           = true;
				on_SatPCO                  = true;
				on_ResEdit                 = true;
				on_EstStaZeroDelay         = false;
				min_Elevation              = 10;
				max_Edit_OcRange           = 1000.0;
				max_Edit_OcDoppler         = 1.0;           // 待更新 可以更小一些 测速代码
				on_ObsEditedInfo           = false;
				ratio_ocEdit               = 3.0;
				min_ArcPointCount          = 0;
				max_OrbIteration           = 10;
				on_Used_Range              = true;
				on_Used_Doppler            = true;
				nameDynPodFitFile          = "dynpod.fit";
				min_OrbPosIteration        = 1.0E-1;
				weight_doppler             = 1.0;
				weight_code                = 1.0;
				weight_phase               = 1.0;
				on_Used_ISL_Code           = false;
				on_Used_ISL_Phase          = false;
			}
		};

		class TQSatNetDynPOD : public SatdynBasic
		{
		public:
			TQSatNetDynPOD(void);
		public:
			~TQSatNetDynPOD(void);
		public:
			bool    adamsCowell_ac(UTC t0, UTC t1, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 60.0, int q = 11);
			bool    initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength = 3600 * 3, double h = 75.0);
			bool    dynamicPOD_pos(vector<TimePosVel> obsOrbitList, TQNETPOD_SatDatumMap::iterator it_Sat, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0, bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true);
		    void    orbitExtrapolation(SatdynBasicDatum dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel>    &forecastOrbList, double interval = 30.0);
			void    orbitExtrapolation(SatdynBasicDatum dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVelAcc> &forecastOrbList, double interval = 30.0);
			bool    mainNetPOD(UTC t0, UTC t1, double interval = 30.0);
		private:
			double  m_stepAdamsCowell;  // 轨道积分步长统一修改
			void    updateSat_Obs(TQNETPOD_SatDatumMap::iterator it_Sat); // + 初始化观测数据 
			void    updateSat_AdamsCowell(TQNETPOD_SatDatumMap::iterator it_Sat); // + 更新积分信息
			void    updateSat_NEQ(TQNETPOD_SatDatumMap::iterator it_Sat); // + 更新法方程
            void    updateSat_SOL(TQNETPOD_SatDatumMap::iterator it_Sat); // + 更新卫星解
			void    updateNET_ISLArc(TQNETPOD_InterSatLink &satISL);      // + 初始化ISL弧段信息, 更新残差
			void    updateNet_NEQ_SQL();                                  // + 更新整网法方程和网解
			void	updateISLResEdit(double factor = 3.0); // + 更新星间测距信息残差并编辑  测试代码：2019/11/22  李康康
		public:
			string                         m_tqSatNetDynPODPath;
			TQNETPOD_DEF                   m_tqNetPODDefine;
			TQNETPOD_SatDatumMap           m_mapSatDatum; // 多星数据 +
			vector<TQNETPOD_InterSatLink>  m_ISLList;     // 星间链路数据 + 
		};
	}
}
