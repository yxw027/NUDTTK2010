#pragma once
#include "structDef.hpp"
#include "SatdynBasic.hpp"
#include "MathAlgorithm.hpp"
#include "SLROrbitComparison.hpp"
#include "RuningInfoFile.hpp"

//  Copyright 2020, Sun Yat-sen University at GuangZhou
using namespace NUDTTK;
using namespace NUDTTK::Geodyn;
using namespace NUDTTK::Math;

namespace NUDTTK
{
	namespace SLR
	{
		struct SLRPOD_ObsElement
		{
			unsigned int id;                    // 测站在美国宇航局所编站址录中的编号
			POS3D        staPos_ECEF;           // 测站位置, 用于方向分析
			POS3D        staPos_J2000;          // 测站位置, Ts时刻
			BLH          staBLH;
			UTC          Ts;                    // 激光信号发射时间
			double       obs;                   // 单程激光测距值（原始观测数据）
			double       wavelength;
			double       temperature;
			double       pressure;
			double       humidity;
			ENU          ecc;

			// 从计算获得
			UTC          Tr;                    // 激光信号反射时间
			double       obscorrected_value;    // 单程激光修正值
			double       r_mean;                // 由卫星位置计算的上下行距离的平均值
			TimePosVel   satOrb;
			TimePosVel   satOrb_ECEF;
			Matrix       satRtPartial;
			double       Elevation;
			double       dR_correct_Trop;       // 对流层改正
			double       dR_correct_Relativity; // 相对论改正
			double       dR_correct_SatMco;     // 卫星质心改正 Mass Center Correct
			double       dR_correct_StaEcc;     // 测站偏心改正
			double       dR_correct_Tide;       // 潮汐改正

			// 精密定轨相关参数
			double       weight;
			double       oc; 
			double       rw;  

			SLRPOD_ObsElement()
			{
				dR_correct_Trop       = 0;
				dR_correct_Relativity = 0;
				dR_correct_Tide       = 0;
				dR_correct_StaEcc     = 0;
				dR_correct_SatMco     = 0;

				weight                = 1.0;
				rw                    = 1.0;
			}
		};

		struct SLRPOD_ObsArc
		{
			double rms_oc;
			int count_valid;
			vector<SLRPOD_ObsElement> obsList;

			SLRPOD_ObsArc()
			{
				count_valid = 0;
				rms_oc = 0.0;
			}
		};

		struct SLRPOD_DEF
		{
			bool                     on_TropDelay;               // 是否考虑对流层延迟效应
			bool                     on_Tides;                   // 是否考虑固体潮效应
			bool                     on_GraRelativity;           // 是否考虑广义相对论效应
			bool                     on_SatPCO;                  // 星上相位中心误差
			bool                     on_StaEcc;                  // 测站偏差
			bool                     on_YawAttitudeModel;        // 姿态偏航控制模式，默认关闭
			double                   min_Elevation;              // 最小高度角阈值
			double                   max_ocEdit;                 // 残差最大阈值，用于剔除一些残差超大的点
			double                   ratio_ocEdit;               // 残差编辑比率       
			int                      min_ArcPointCount;          // 观测数据次数最小值
			int                      max_OrbIteration;           // 轨道改进次数最大值
			double                   threshold_max_adjustpos;
			double                   period_SolarPressure;       // 太阳光压周期
			double                   period_AtmosphereDrag;      // 大气阻力周期
			double                   period_EmpiricalAcc;        // 经验力加速度周期
			bool                     on_SRP9_D0;                 // 是否估计光压D0参数
			bool                     on_SRP9_DC1;                // 是否估计光压DC1参数
			bool                     on_SRP9_DS1;                // 是否估计光压DS1参数	
			bool                     on_SRP9_Y0;                 // 是否估计光压Y0参数
			bool                     on_SRP9_YC1;                // 是否估计光压YC1参数
			bool                     on_SRP9_YS1;                // 是否估计光压YS1参数
			bool                     on_SRP9_B0;                 // 是否估计光压B0参数
			bool                     on_SRP9_BC1;                // 是否估计光压BC1参数
			bool                     on_SRP9_BS1;                // 是否估计光压BS1参数
			string                   nameDynPodFitFile;
			double                   span_InitDynDatumCoarsePos;

			SLRPOD_DEF()
			{
				on_TropDelay               = true;
				on_Tides                   = true;
				on_GraRelativity           = true;
				on_SatPCO                  = true;
				on_StaEcc                  = true;
				on_YawAttitudeModel        = false;
				min_Elevation              = 10;
				max_ocEdit                 = 10.0; 
				ratio_ocEdit               = 4.0;
				min_ArcPointCount          = 0; 
				max_OrbIteration           = 10;
				period_SolarPressure       = 3600.0 * 24.0;
				period_AtmosphereDrag      = 3600.0 * 24.0;
				period_EmpiricalAcc        = 3600.0 * 24.0;
				threshold_max_adjustpos    = 0.02;
				//solarPressure_Model        = TYPE_SOLARPRESSURE_1PARA; 
				on_SRP9_D0                 = true;
				on_SRP9_DC1                = true;
				on_SRP9_DS1                = true;
				on_SRP9_Y0                 = true;
				on_SRP9_YC1                = true;
				on_SRP9_YS1                = true;
				on_SRP9_B0                 = true;
				on_SRP9_BC1                = true;
				on_SRP9_BS1                = true;
				nameDynPodFitFile          = "dynpod.fit";
				span_InitDynDatumCoarsePos = 300;
			}
		};

		class SLRSatDynPOD : public SatdynBasic
		{
		public:
			SLRSatDynPOD(void);
		public:
			~SLRSatDynPOD(void);
		public:
			bool getStaPosvel(UTC t, int id, POS6D& posvel);
			bool getStaEcc(UTC t, int id, ENU& ecc);
		public:
			void    setStepAdamsCowell(double step);
			void    weighting_Elevation(double Elevation, double& weight);
			bool    adamsCowell_ac(UTC t0, UTC t1, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 10.0, int q = 11);
			bool    initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength = 3600 * 3, double h = 10.0);
			bool    dynamicPOD_pos(vector<TimePosVel> obsOrbitList,  SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0, bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true);
			bool    mainPOD(string strObsFileName, int nObsFileType, SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0, bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true);
		private:
			bool    getEphemeris(UTC t, TimePosVel& satOrb, Matrix& satRtPartial, int nLagrange = 9);
		private:
			double                           m_stepAdamsCowell; 
			vector<TimePosVel>               m_acOrbitList;            // 用于插值轨道序列
			vector<Matrix>                   m_acRtPartialList;        // 用于插值偏导数序列

		public:
			SLRPOD_DEF                       m_podDefine;
			string                           m_strPODPath;
			vector<SLRPOD_ObsArc>            m_obsArc;

			// 激光数据预处理所需数据
			vector<StaEccRecord>             m_staEccList;             // 激光测站的偏心信息
			vector<StaSscRecord>             m_staSscList;             // 激光测站的坐标信息
			POS3D                            m_mcoLaserRetroReflector; // 激光反射器质心偏移
			StaOceanLoadingDisplacementFile  m_staOldFile;             // 测站的海潮振幅和相位文件( http://www.oso.chalmers.se/~loading/ 网站提供 )
			double                           m_constRangeBias;         // 固定偏差校正, 依靠外部输入, 默认为 0
			bool                             m_bChecksum;
		};
	}
}
