#pragma once
#include"structDef.hpp"
#include"SatdynBasic.hpp"
#include<map>
#include"TimePosVelFile.hpp"
#include"MathAlgorithm.hpp"
#include"TQUXBObsFile.hpp"
#include"RuningInfoFile.hpp"

// Copyright 2018, SUN YAT-SEN UNIVERSITY TianQin Research Center at ZhuHai
using namespace NUDTTK;
using namespace NUDTTK::Math;
using namespace NUDTTK::Geodyn;

namespace NUDTTK
{
	namespace TQPod
	{

        struct TQUXB_StaDatum
		{
			int                      id_Station;      // 测站号
			string                   name;            // 测站名
			POS3D                    pos_ITRF;        // 测站在ITRF框架下的位置
			map<UTC, TQUXBObsLine>   obsList;         // 轨道信息列表
			bool                     on_EstZeroDelay; // 
			double                   zeroDelay_0;     // 测站距离零点延迟
			double                   zeroDelay_Est;   // Est
			int                      id_ZeroDelay;    //

            //TWRObsErrFile          obsErrFile;

			TQUXB_StaDatum()
			{
				on_EstZeroDelay = false;
				zeroDelay_0     = 0.0;
				zeroDelay_Est   = 0.0;
				id_ZeroDelay    = -1;
			}
		};
        
		struct TQUXBPOD_DEF
		{
			bool                     on_TropDelay;         // 是否考虑对流层延迟效应
			bool                     on_SolidTides;        // 是否考虑固体潮效应
			bool                     on_OceanTides;        // 是否考虑海潮效应
			bool                     on_GraRelativity;     // 是否考虑广义相对论效应
			bool                     on_SatPCO;            // 星上相位中心误差
			bool                     on_ObsEditedInfo;     
			
			double                   on_DeleteData;        // 是否删除给定时间段数据
			double                   min_Elevation;        // 最小高度角阈值
			double                   max_ocEdit;           // 残差最大阈值
			double                   ratio_ocEdit;         // 残差编辑比率       
			int                      min_ArcPointCount;    // 观测数据次数最小值
			int                      max_OrbIteration;     // 轨道改进次数最大值
			bool                     on_EstZeroDelay;       
			double                   satZeroDelay_0;       // 星上零点延迟
			double                   satZeroDelay_Est;

			TYPE_SOLARPRESSURE_MODEL solarPressure_Model;        // 太阳光压模型
			double                   period_SolarPressure;       // 太阳光压周期
			double                   period_EmpiricalAcc;        // 经验力加速度周期

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

            //bool                     on_DeleteManeuver;    // 是否删除机动前后数据   
			//bool                     on_WeightManeuver;    // 是否使用机动期间加权
			//double                   weightManeuver;
			//double                   span_InitDynDatumCoarsePos; // 初轨确定间隔   
			//double                   span_DeleteManeuver;        // 删除机动前后时间间隔

			 TQUXBPOD_DEF()
			{
				on_TropDelay               = true;
				on_SolidTides              = true;
				on_OceanTides              = true;
				on_GraRelativity           = true;
				on_SatPCO                  = true;
				on_ObsEditedInfo           = false;
				on_DeleteData              = false;
				min_Elevation              = 10;
				max_ocEdit                 = 70.0;
				ratio_ocEdit               = 3.0;
				min_ArcPointCount          = 20;
				max_OrbIteration           = 10;
			
				period_SolarPressure       = 3600.0 * 24.0;
				period_EmpiricalAcc        = 3600.0 * 24.0;
				solarPressure_Model        = TYPE_SOLARPRESSURE_9PARA; 
				on_EstZeroDelay            = false;
				satZeroDelay_0             = 0.0;
				satZeroDelay_Est           = 0.0;
			
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

				//on_DeleteManeuver          = false;
				//on_WeightManeuver          = false;
				//span_InitDynDatumCoarsePos = 300.0; // 5分钟
				//span_DeleteManeuver        = 0.5 * 3600.0; // 半小时
				//span_DeleteManeuver        = 0.5 * 3600.0; // 半小时
			}
		};

		class TQSatDynPOD : public SatdynBasic
		{
		public:
			TQSatDynPOD(void);
		public:
			~TQSatDynPOD(void);
		public:
			bool    adamsCowell_ac(UTC t0, UTC t1, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 60.0, int q = 11);			
		    bool    dynamicTQPOD_pos(vector<TimePosVel> obsOrbitList, SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0, bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true);
			bool    mainTQPOD(SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval = 30.0, bool bInitDynDatumEst = false, bool bForecast = true, bool bResEdit = true);
		private:
			bool    getEphemeris(UTC t, TimePosVel& tqOrb, int nLagrange = 9);
			bool    getEphemeris_PathDelay(UTC t, POS3D staPos, double& delay, TimePosVel& tqOrb, Matrix& tqRtPartial, double threshold = 1.0E-07);
		private:
			vector<TimePosVel>           m_acOrbitList;           // 用于插值轨道序列
			vector<Matrix>               m_acRtPartialList;       // 用于插值偏导数序列
		public:
			string                       m_strTQPODPath;
			TQUXBPOD_DEF                 m_tqUXBDefine;
			POS3D                        m_pcoAnt;                // 天线偏心信息
			map<string, TQUXB_StaDatum>  m_staDatumList;          // 测站观测信息
			TimePosVelFile               m_orbJ2000File_0;
			TimePosVelFile               m_orbECEFFile_0;
		};
	}
}
