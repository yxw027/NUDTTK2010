#pragma once
#include <string.h>
#include <vector>
#include "structDef.hpp"
#include "constDef.hpp"


// 读取CHAMP卫星加速度计数据
// 文件名:CH-OG-2-ACC+年_年积日_00.0.dat，非标准行结构文件
// 参考文档:CH-GFZ-FD-001.pdf
// 日期:2019.09.17

namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		// 加速度计标校参数
		struct CalibrationPara
		{
			POS3D CaliPara;        // Bias/Scale factor,Lorentz force acceleration(01),Corrections due to temperature models and so on(02)
			int Flag;              // Application flag:1 Calibration applied,0 Calibration not applied
			CalibrationPara()
			{
				CaliPara.x = 0.0;
				CaliPara.y = 0.0;
				CaliPara.z = 0.0;

				Flag = 0;
			}
		};
		// 卫星质量信息
		struct SatMass
		{
			double TotalMass;
			double GasMass;
			GPST Epoch_t;
		};
		// 加速度计数据
		/*struct Acceleration
		{
			POS3D acc;
			int Samples[3];
		}*/

		// 推进器信息
		struct Thruster
		{
			GPST t;
			int thrusterFlag[7];
			double duration;

			Thruster()
			{
				for(int i = 0; i < 7; i++)
				{
					thrusterFlag[i] = 0;
				}
				duration = 0.0;
			}
		};
		struct champACCFileLine
		{
			GPST t;  // 时间参考坐标系gps
			CalibrationPara acl_k0;     // lin_acc, Bias, mm/s^2
			CalibrationPara acl_k1;     // lin_acc, Scale factor
			CalibrationPara aca_k0;     // ang_acc, Bias,mrad/s^2
			CalibrationPara aca_k1;     // ang_acc, Scale factor
			CalibrationPara acc_01;     // Lorentz force acceleration(01)
			CalibrationPara acc_02;     // Corrections due to temperature models and so on(02)

			POS3D line_acl;
			POS3D line_aca;
			ATT_Q4 Q4;

			champACCFileLine()
			{
				line_acl.x = 0.0;
				line_acl.y = 0.0;
				line_acl.z = 0.0;
				line_aca.x = 0.0;
				line_aca.y = 0.0;
				line_aca.z = 0.0;
				Q4.q1 = 0.0;
				Q4.q2 = 0.0;
				Q4.q3 = 0.0;
				Q4.q4 = 0.0;
			}
		};

		class champACCFile
		{
		public:
			champACCFile(void);
		public:
			~champACCFile(void);
		public:
			int  isValidEpochLine(string strline, FILE *AccFile, int &nFlag, GPST &t);
			bool open(string strAccFileName);
			bool getAcc(GPST t, champACCFileLine &accline, int nlagrange = 2);
			bool calibrate(bool interFlag);
		public:
			vector<champACCFileLine> m_data;
			vector<SatMass> mm_data;            // 卫星质量信息
			vector<Thruster> tm_data;           // 推进器信息
		};
	}
}