#pragma once
#include "TQISLObsFile.hpp"
#include "MathAlgorithm.hpp"
#include "TimeCoordConvert.hpp"
#include <time.h>

using namespace NUDTTK;
using namespace NUDTTK::Math;

namespace NUDTTK
{
	namespace TQPod
	{
		// 卫星信息
		struct TQSatISLInfo
		{
			string             satName;  // 卫星名称
			POS3D              pcoAnt;   // 天线偏心信息
			vector<TimePosVel> orbList;  // 卫星轨道列表

			bool getEphemeris(UTC t, TimePosVel& satOrb, int nLagrange = 9);
			//bool getEphemeris_PathDelay(UTC t, POS3D staPos, double& delay, TimePosVel& satOrb,  double threshold = 1.0E-07); // 迭代计算星上信号反射的时刻
		};

		struct TQISLOBSSIMU_DEF
		{
			bool                     on_SatPCO;             // 是否考虑卫星质心中心偏差
			bool                     on_LightTime;          // 是否考虑光行时

			bool                     on_GraRelativity;      // 是否考虑广义相对论

			bool                     on_ISLCodeSysBias;  // 伪码系统误差             
			bool                     on_ISLPhaseSysBias;
			bool                     on_ISLCodeRandnNoise;   //随机噪声
			bool                     on_ISLPhaseRandnNoise;

			double                   WAVELENGTH; // 激光波长


			TQISLOBSSIMU_DEF()
			{
				on_SatPCO             = false;
				on_LightTime          = false;
				on_ISLCodeSysBias     = false;
				on_ISLPhaseSysBias    = false;
				on_ISLCodeRandnNoise  = false;
				on_ISLPhaseRandnNoise = false;
				WAVELENGTH            = 1064 * 10E-9;  // 激光波长1024纳米
			}
		};

		class TQISLObsSimu
		{
		public:
			TQISLObsSimu(void);
		public:
			~TQISLObsSimu(void);
		public:
			bool   simuISLObsFile(TQISLObsFile &obsFile, UTC t0, UTC t1, double h = 60.0 * 30);// 默认采样间隔30分钟

		public:
			TQISLOBSSIMU_DEF    m_simuDefine;
			TimeCoordConvert    m_TimeCoordConvert; // 时间坐标系转换
			TQSatISLInfo        m_satInfoA;
			TQSatISLInfo        m_satInfoB;
			//vector<TQSatInfo>   m_satInfoList;
		};
	}
}
