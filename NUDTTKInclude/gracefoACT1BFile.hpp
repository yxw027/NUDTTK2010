#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>

// GRACE-FO卫星的加速度计数据中包含了线性加速度与角加速度，其值均在科学参考坐标系中给出；
// 科学参考坐标系->仪器坐标系->星体坐标系->惯性坐标系（参考GRACE-FO_L1_Data_Product_User_Handbook）；
// 线性加速度单位为m/s^2，角加速度的单位为rad/s^2；
// 官方给出的GRACE卫星加速度计数据的偏差和尺度因子参见PDF文档：Recommendation for a-priori Bias & Scale Parameters for Level-1B ACC Data
// 2020.4.8 韦春博

namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		#pragma pack(1)     // 使编译器将结构体数据强制连续排列
		struct ACT1BRecord
		{
			int           gps_time;
			char          gracefo_id;
			double        lin_accl_x;
			double        lin_accl_y;
			double        lin_accl_z;
			double        ang_accl_x;
			double        ang_accl_y;
			double        ang_accl_z;
			double        acl_x_res;   // ??
			double        acl_y_res;
			double        acl_z_res;
			unsigned char qualflg;

			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0.0);
				return t0 + gps_time;
			};
		};
		#pragma pack()
		

		class gracefoACT1BFile
		{
		public:
			gracefoACT1BFile(void);
		public:
			~gracefoACT1BFile(void);
		public:
			bool    isValidEpochLine(string strLine, ACT1BRecord& act1bRecord);
			bool    open(string strACT1BFileName);
			bool    isEmpty();
			bool    getAcc(GPST t, POS3D &lin_acc, POS3D &ang_acc, int nlagrange = 8);
			bool    exportTimeAccFile(string strTimeAccFileName);
		public:
			vector<ACT1BRecord> m_data;
		};
	}
}