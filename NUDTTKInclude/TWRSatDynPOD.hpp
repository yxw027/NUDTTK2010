#pragma once
#include"structDef.hpp"
#include"SatdynBasic.hpp"
#include<map>
#include"TWRObsFile.hpp"
#include"TWRObsErrFile.hpp"
#include"TWRAttFile.hpp"
#include"TimePosVelFile.hpp"
#include"MathAlgorithm.hpp"


//  Copyright 2017, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
using namespace NUDTTK::Math;
using namespace NUDTTK::Geodyn;

namespace NUDTTK
{
	namespace TwoWayRangingPod
	{
		struct TWR_StaDatum
		{
			int                  id_Station; // 测站号
			string               name;
			POS3D                pos_ITRF;
			map<UTC, TWRObsLine> obsList;
			TWRObsErrFile        obsErrFile;
			bool                 on_EstZeroDelay;
			double               zeroDelay_0;
			double               zeroDelay_Est;
			int                  id_ZeroDelay;

			TWR_StaDatum()
			{
				on_EstZeroDelay = false;
				zeroDelay_0     = 0.0;
				zeroDelay_Est   = 0.0;
				id_ZeroDelay    = -1;
			}
		};

		// 机动力
		struct ManeuverAttitude
		{
            UTC    t0;     
			UTC    t1;     
		};

		struct TWRPOD_DEF
		{
			bool                     on_TropDelay;
			bool                     on_SolidTides;
			bool                     on_OceanTides;
			bool                     on_GraRelativity;
			bool                     on_SatPCO;
			bool                     on_ObsEditedInfo;
			bool                     on_DeleteManeuver;    // 是否删除机动前后数据
			bool                     on_WeightManeuver;    // 是否使用机动期间加权
			double                   on_DeleteData;        // 是否删除给定时间段数据

			double                   min_Elevation;
			double                   max_ocEdit;
			double                   ratio_ocEdit;
			double                   weightManeuver;
			int                      min_ArcPointCount;
			int                      max_OrbIteration;     // 轨道改进次数最大值
			bool                     on_EstZeroDelay;
			double                   satZeroDelay_0;
			double                   satZeroDelay_Est;
			double                   span_InitDynDatumCoarsePos; // 初轨确定间隔
			double                   span_DeleteManeuver;        // 删除机动前后时间间隔

			TYPE_SOLARPRESSURE_MODEL solarPressure_Model;        // 太阳光压模型
			double                   period_SolarPressure;
			double                   period_EmpiricalAcc;

			bool                     on_SRP9_D0;               // 是否估计光压D0参数
			bool                     on_SRP9_DC1;              // 是否估计光压DC1参数
			bool                     on_SRP9_DS1;              // 是否估计光压DS1参数	
			bool                     on_SRP9_Y0;               // 是否估计光压Y0参数
			bool                     on_SRP9_YC1;              // 是否估计光压YC1参数
			bool                     on_SRP9_YS1;              // 是否估计光压YS1参数
			bool                     on_SRP9_B0;               // 是否估计光压B0参数
			bool                     on_SRP9_BC1;              // 是否估计光压BC1参数
			bool                     on_SRP9_BS1;              // 是否估计光压BS1参数

			string                   nameDynPodFitFile;

			TWRPOD_DEF()
			{
				on_TropDelay               = true;
				on_SolidTides              = true;
				on_OceanTides              = true;
				on_GraRelativity           = true;
				on_SatPCO                  = true;
				on_ObsEditedInfo           = false;
				on_DeleteManeuver          = false;
				on_WeightManeuver          = false;
				on_DeleteData              = false;
				min_Elevation              = 10;
				max_ocEdit                 = 70.0;
				ratio_ocEdit               = 3.0;
				weightManeuver             = 0.20;
				min_ArcPointCount          = 20;
				max_OrbIteration           = 10;
				span_InitDynDatumCoarsePos = 300.0; // 5分钟
				period_SolarPressure       = 3600.0 * 24.0;
				period_EmpiricalAcc        = 3600.0 * 24.0;
				solarPressure_Model        = TYPE_SOLARPRESSURE_9PARA; 
				on_EstZeroDelay            = false;
				satZeroDelay_0             = 0.0;
				satZeroDelay_Est           = 0.0;
				span_DeleteManeuver        = 0.5 * 3600.0; // 半小时

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
			}
		};

		class TWRSatDynPOD : public SatdynBasic
		{
		public:
			TWRSatDynPOD(void);
		public:
			~TWRSatDynPOD(void);
		public:
			bool    judgeAttManeuver(UTC t);
			bool    judgeDeleteData(UTC t);
            bool    judgeAttManeuverTime(UTC t);
			bool    adamsCowell_ac(UTC t0, UTC t1, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 75.0, int q = 11);			
			bool    initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength = 3600 * 3, double h = 75.0);
			bool    dynamicGEOPOD_pos(vector<TimePosVel> obsOrbitList, SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0, bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true);
			bool    mainGEOPOD(SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0, bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true);
		private:
			bool    getEphemeris(UTC t, TimePosVel& geoOrb, int nLagrange = 9);
			bool    getEphemeris_PathDelay(UTC t, POS3D staPos, double& delay, TimePosVel& geoOrb, Matrix& geoRtPartial, double threshold = 1.0E-07);
		private:
		    vector<TimePosVel>        m_acOrbitList;           // 用于插值轨道序列
			vector<Matrix>            m_acRtPartialList;       // 用于插值偏导数序列

		public:
			string                    m_strPODPath;
			TWRPOD_DEF                m_twrDefine;
			POS3D                     m_pcoAnt;
			TWRAttFile                m_twrAttFile;
			map<string, TWR_StaDatum> m_staDatumList;
			TimePosVelFile            m_orbJ2000File_0;
			TimePosVelFile            m_orbECEFFile_0;
			vector<ManeuverAttitude>  m_attManeuverList;
			vector<ManeuverAttitude>  m_attDeleteDataList;
		};
	}
}
