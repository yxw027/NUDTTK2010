#pragma once
#include <string>
#include <vector>
#include "structDef.hpp"
#include "Matrix.hpp"
#include <math.h>

// Copyright 2018, SUN YAT-SEN UNIVERSITY TianQin Research Center at ZhuHai
namespace NUDTTK
{
	namespace TQPod
	{
		struct TQUXBObsLine
		{
			UTC    t;                  // 接收时刻, 北京时, 我国测站数据大部分采用北京时
			double R;                  // 距离真值
			double V;                  // 径向速度

			//double ISL_Code; // 星间激光伪码真值   测试代码
			//double ISL_Code1;// 星间激光伪码观测值 测试代码

			//double ISL_Phase; // 星间激光相位测距真值 测试代码
			//double ISL_Phase1;// 星间激光相位测距观测值 测试代码
			
			double R1;                 // 原始测距值
			double R0s;                // 星上距离零值
			double R0g;                // 地面距离零值
			double err_Trop;           // 对流层误差
			double err_Iono;           // 电离层误差

			double V1;                 // 原始测速值-平均距离变化率
            UTC    t0;                 // 积分时间起点 +
            UTC    t1;                 // 积分时间终点 + 

			double Azimuth;            // 观测方位角, 东北天坐标系
			double Elevation;          // 观测高度角，东北天坐标系

			// 测站和卫星名字 李康康 测试
			double time_Doppler;       // 多普勒积分时间长度
			string staName;            // 测站名字
			string satName;            // 卫星名字

			//string satName1;   // uxb无线电测距测速 测试代码
			//string satName2;   // satName1-satName2的星间激光测距 测试代码

			// 数据预处理相关参数
			POS3D      pos_J2000;      // 测站在t时刻J2000下的位置坐标
			TimePosVel tqOrb;          // 卫星星历表
			Matrix     tqRtPartial;    // 偏导数
			double     dR_up;          // 上行距离
			double     dR_down;        // 下行距离

			POS3D      pos_J2000_t0;    // 测站在t0时刻J2000下的位置坐标
			POS3D      pos_J2000_t1;    // 测站在t1时刻J2000下的位置坐标
			TimePosVel tqOrb_t0;        // 调用getEphemeris_PathDelay，用于存储多普勒距离积分偏导数 + 
			TimePosVel tqOrb_t1;        // 调用getEphemeris_PathDelay，用于存储多普勒距离积分偏导数 + 
			Matrix     tqRtPartial_t0;  // 调用getEphemeris_PathDelay，用于存储多普勒距离积分偏导数 + 
			Matrix     tqRtPartial_t1;  // 调用getEphemeris_PathDelay，用于存储多普勒距离积分偏导数 +
			double     dR_trop;         // 对流层修正
			double     dR_satpco;       // 反射器的质心偏移
			double     dR_GraRelativity;// 广义相对论修正
			double     dR_SolidTides;   // 固体潮修正
			double     dR_OceanTides;   // 海潮修正
			double     corrected_value; // 修正值, 用于距离修正

			// 精密定轨相关参数
			double weight_range;        // 测距权重
			double weight_doppler;      // 测速权重
			double oc_range;            // 测距残差
			double oc_doppler;          // 测速残差
			double rw_range;            // 测距鲁棒权值
			double rw_doppler;          // 测速鲁棒权值
			


			TQUXBObsLine()
			{
				R               = 0.0;
				V               = 0.0;

				Elevation       = 0.0;
				Azimuth         = 0.0;   

				time_Doppler    = 0.0;

				R0s             = 0.0;
				R1              = 0.0;
				R0g             = 0.0;
				err_Trop        = 0.0;
				err_Iono        = 0.0;

				oc_range        = 0.0;
				oc_doppler      = 0.0;
				rw_range        = 1.0;
				rw_doppler      = 1.0;
				weight_range    = 1.0;
				weight_doppler  = 1.0;

				V1              = 0.0;

				dR_trop         = 0.0;
				dR_satpco       = 0.0;
				dR_GraRelativity= 0.0;
				dR_SolidTides   = 0.0;
                dR_OceanTides   = 0.0;
				corrected_value = 0.0;

			}

			double getRange()
			{
				return R1 - R0s - R0g + err_Trop + err_Iono;
			}

			double getVel()
			{
				return V1;
			}
		};

		class TQUXBObsFile
		{
		public:
			TQUXBObsFile(void);
		public:
			~TQUXBObsFile(void);
		public:
			bool write(string  strUXBObsFileName);
			bool open(string strUXBObsFileName); // 考虑是否添加读取观测数据
		public:
			vector<TQUXBObsLine> m_data;
		};
	}
}
