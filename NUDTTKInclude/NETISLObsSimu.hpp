#pragma once
#include "jplEphFile.hpp"
#include "TimeCoordConvert.hpp"
#include <time.h>
#include "structDef.hpp"
#include "dynPODStructDef.hpp"
#include "MathAlgorithm.hpp"

using namespace NUDTTK;
namespace NUDTTK
{
	namespace NETPOD
	{
		struct NETISLObsEpoch_simu
		{
			DayTime                  t;
			string                   sat_A;
			string                   sat_B;
			double                   obs_AB;  
			double                   obs_BA; 
			double                   d;
			double                   clk_A;
			double                   clk_B; 
		};

		// 卫星数据结构, 包括卫星名、轨道等
		struct NETISL_StaDatum 
		{
			// 输入部分
			GPST      t0;
			GPST      t1;
			string    staName;
			vector<TimePosVel>        m_pvOrbList; //               轨道文件
			double    orbOutputInterval;
			double    clk_bias;       // 偏差
			double    clk_biasdraft; // 钟漂

			bool getEphemeris(GPST t, TimePosVel& interpOrbit, int nLagrange = 9);

			NETISL_StaDatum()
			{
				clk_bias = 1E-7;
				clk_biasdraft = 1E-10;
				orbOutputInterval = 30.0;
			}
		};
		typedef map<string, NETISL_StaDatum> StaDatumMap_simu; 

		// 测站基站数据结构
		struct NETISL_StaBaseline
		{
			// 输入
			string    staName_A; 
			string    staName_B;
			// 输出
			vector<NETISLObsEpoch_simu>   m_data; // 

			NETISL_StaBaseline()
			{
			}
		};

		// 公共参数
		struct NETISLObsSimu_DEF
		{
			double        system_err; // 测距系统误差
			double        noise_err; // 测距随机误差
			double        beta_1; // MEO地球遮挡角
			double        beta_2; // GEO、IGSO地球遮挡角
			double        alpha;  // ka波束扫描范围（俯仰角±60）
			double        intervel; // 采样间隔
			NETISLObsSimu_DEF()
			{
				system_err = 0.5;
				noise_err  = 0.5;
				beta_1 = 12.9;
				beta_2 = 8.7;
				alpha = 60.0;
				intervel = 600.0;
			}
		};

		class NETISLObsimu
		{
		public:
			NETISLObsimu(void);
		public:
			~NETISLObsimu(void);
		public: 
			bool  main_obssimu();
			bool  main_EMobssimu();
			bool  main_EM_BDSobssimu();
			bool  main_EM_GEOobssimu();  //  20220620
			bool  main_BDS_EMobssimu();  //  20220620
		public:
			NETISLObsSimu_DEF              m_simuParaDefine;
			JPLEphFile				       m_JPLEphFile;       // JPL DE405 星历数据文件
			TimeCoordConvert		       m_TimeCoordConvert;  // 时间坐标系转换
			StaDatumMap_simu               m_staDatumMap;
			vector<NETISL_StaBaseline>     m_staBaselineList;      // 利用外部输入 + 
		};
	}
}
