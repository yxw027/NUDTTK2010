#pragma once
#include <string>
#include <vector>
#include "structDef.hpp"
#include "Matrix.hpp"
#include <math.h>

namespace NUDTTK
{
	namespace TQPod
	{
		struct TQISLObsLine
		{
			UTC    t;                    // 接收时刻
			double range;
			double ISL_Code;            // 星间激光伪码观测值
			double ISL_Phase;            // 星间激光相位观测值
			

			// 卫星名称
			string satNameA;             // 卫星A
			string satNameB;             // 卫星B


			// 数据预处理相关参数
			TimePosVel tqOrb_A;            // 卫星A在t时刻的星历表
			TimePosVel tqOrb_B;            // 卫星B在t时刻的星历表
			Matrix     tqRtPartial_A;      // 卫星A在t时刻的偏导数
			Matrix     tqRtPartial_B;      // 卫星B在t时刻的偏导数

			double     dR_satpco;        // 反射器的质心偏移,几何位置修正
			double     dR_lightTime;     // 光行时修正
		    //double      dR_sun;          // 太阳风影响？ 需进一步确认
			double     corrected_value;  // 修正值, 用于距离修正:包括质心修正和光行时修正

			// 精密定轨相关参数
			double weight_code;        // 伪码测距权重
			double weight_phase;       // 相位测距权重
			double oc_code;            // 伪码测距残差
			double oc_phase;           // 相位测距残差
			double rw_code;            // 伪码测距鲁棒权值
			double rw_phase;           // 相位测距鲁棒权值

			double ambiguity;          // 相位模糊参数
			                           // 相位模糊参数是一个固定值，每个卫星的每个弧段都有一个，
			                           // 在仿真数据时，仿真连续的弧段即可，
			                           // 模糊参数可以是直接给定，也可以随机产生一个。
			
			TQISLObsLine()
			{
				ISL_Code          = 0.0; // 星间激光伪码测距真值
				ISL_Phase         = 0.0; // 星间激光相位测距真值

				dR_satpco         = 0.0; // 卫星天线偏移
				dR_lightTime      = 0.0; // 星间光行时

				weight_code       = 0.0; // 伪码测距权重
				weight_phase      = 0.0; // 相位测距权重
				oc_code           = 0.0; // 伪码测距残差
				oc_phase          = 0.0; // 相位测距残差
				rw_code           = 0.0; // 伪码测距鲁棒权值
				rw_phase          = 0.0; // 相位测距鲁棒权值
				ambiguity         = 0.0;
			}

			double getCode()
			{
				return ISL_Code + dR_satpco + dR_lightTime;
			}

			double getPhase()
			{
				return ISL_Phase + dR_satpco + dR_lightTime;
			}
		};

		class TQISLObsFile
		{
		public:
			TQISLObsFile(void);
		public:
			~TQISLObsFile(void);
		public:
			bool write(string strISLObsFileName);
			bool open(string strISLObsFileName); 
		public:
			vector<TQISLObsLine> m_data;
		};
	}
}
