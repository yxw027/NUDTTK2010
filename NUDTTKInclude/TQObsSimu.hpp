#pragma once
#include "TQUXBObsFile.hpp"
#include "MathAlgorithm.hpp"
#include "TimeCoordConvert.hpp"
#include "RuningInfoFile.hpp"
#include <time.h>

using namespace NUDTTK;
using namespace NUDTTK::Math;

// Copyright 2018, SUN YAT-SEN UNIVERSITY TianQin Research Center at ZhuHai
namespace NUDTTK
{
	namespace TQPod
	{
		// 测站信息
		struct TQStaInfo
		{
			string       name;         // 名称
			BLH          pos_blh;       // 测站的经纬度
			POS3D        pos;          // 测站位置信息
			ENU          pcoAnt;       // 天线偏心信息
			double       StaZeroDelay; // 测站系统偏差

			double       mu;           // 暂时添加 测试代码 随机噪声
			double       sigma;


			TQStaInfo()
			{
				StaZeroDelay  = 0.0;
				mu            = 0.0;
				sigma         = 0.0;
			}
		};

		// 卫星信息
		struct TQSatInfo
		{
			string             satName;  // 卫星名称
			POS3D              pcoAnt;   // 天线偏心信息
			vector<TimePosVel> orbList;  // 卫星轨道列表

			double             SatZeroDelay;
			int                JMS; // JMS测站观测数据量
			int                KASH;
			int                NEUQ;

			TQSatInfo()
			{
				SatZeroDelay = 0.0;
				JMS          = 0;
				KASH         = 0;
				NEUQ         = 0;
			}

			bool getEphemeris(UTC t, TimePosVel& satOrb, int nLagrange = 9);
			bool getEphemeris_PathDelay(UTC t, POS3D staPos, double& delay, TimePosVel& satOrb,  double threshold = 1.0E-8); // 迭代计算星上信号反射的时刻
		};

		struct TQOBSSIMU_DEF
		{
			bool                     on_TropDelay;          // 是否修正对流层延迟
			bool                     on_SolidTides;         // 是否考虑固体潮效应
			bool                     on_OceanTides;         // 是否考虑海潮效应
			bool                     on_GraRelativity;      // 是否考虑广义相对论效应
			bool                     on_SatPCO;             // 是否考虑卫星相位中心偏差
			bool                     on_RecClock;           // 是否考虑接收机钟差
			bool                     on_ObsRandnNoise;      // 是否添加观测数据随机误差
			bool                     on_StaZeroDelayNoise;  // 是否添加测站系统偏差
			bool                     on_SatZeroDelayNoise;  // 是否添加卫星系统偏差
			bool                     on_Day;                // 白天是否可见，用于激光测距仿真
			double                   min_Elevation;         // 最小高角度
			double                   min_DopplerIntergTime; // 最小多普勒积分时间

			double                   r_Noise;         // 位置噪声
			double                   v_Noise;         // 速度噪声

			TQOBSSIMU_DEF()
			{
				on_TropDelay          = false;
				on_SolidTides         = false;
				on_OceanTides         = false;
				on_GraRelativity      = true;
				on_SatPCO             = true;	
				on_ObsRandnNoise      = false;
				on_StaZeroDelayNoise  = false;
				on_SatZeroDelayNoise  = false;
				on_Day                = false;
				min_Elevation         = 10;
                min_DopplerIntergTime = 5;
				r_Noise               = 0.01;
				v_Noise               = 0.0;
			}
		};

		class TQObsSimu
		{
		public:
			TQObsSimu(void);
		public:
			~TQObsSimu(void);
		public:
			bool   judgeUXBSignalCover(POS3D posSta, POS3D posSat, double cut_elevation = 10);
			bool   simuUXBObsFile(TQUXBObsFile &obsFile, UTC t0, UTC t1, string staname, string satname, double h = 10.0, double DopplerTime0 = 10.0);
			bool   judgeUXBElevation(UTC t_Receive, POS3D posSta, double& elevation1, double& elevation2);//计算高度角
		public:
			TQOBSSIMU_DEF       m_simuDefine;
			TimeCoordConvert    m_TimeCoordConvert; // 时间坐标系转换
			TQStaInfo           m_staInfo; 
			TQSatInfo           m_satInfo;
			vector<TQSatInfo>   m_satInfoList; // 对比三个探测器的高度角，若当前航天器与测站的高度角最大，则仿真对应时刻的数据，否则跳过。
		};
	}
}
