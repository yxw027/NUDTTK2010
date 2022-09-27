#pragma once
#include <string>
#include <vector>
#include "structDef.hpp"
#include "Matrix.hpp"
#include <math.h>

//  Copyright 2017, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
namespace NUDTTK
{
	namespace TwoWayRangingPod
	{
		struct TWRObsLine
		{
			UTC    t;               // 接收时刻, 北京时, 我国测站数据大部分采用北京时
			double R;               // 扣除设备零值的测距值（不包含星上零值）
			double R0s;             // 星上距离自校零值
			double R1;              // 原始测距值
			double R0g;             // 地面距离零值
			double R01g;            // 地面距离零值附加修正量

			// 数据预处理相关参数
			POS3D      pos_J2000;
			TimePosVel geoOrb;
			Matrix     geoRtPartial;
			double     dR_trop;         // 对流层修正
			double     dR_satpco;       // 反射器的质心偏移
			double     dR_GraRelativity;
			double     dR_SolidTides;
			double     dR_OceanTides;
			double     dR_up;
			double     dR_down;
			double     Azimuth;         // 观测方位角, 东北天坐标系
			double     Elevation;       // 观测高度角，东北天坐标系
			double     corrected_value; // 修正值, 用于距离修正

			// 精密定轨相关参数
			double weight;
			double oc;
			double rw;              // 鲁棒权值

			TWRObsLine()
			{
				R               = 0.0;
				R0s             = 0.0;
				R1              = 0.0;
				R0g             = 0.0;
				R01g            = 0.0;
				dR_trop         = 0.0;
				dR_satpco       = 0.0;
				dR_GraRelativity= 0.0;
				dR_SolidTides   = 0.0;
                dR_OceanTides   = 0.0;
				corrected_value = 0.0;
				oc              = 0.0;
				weight          = 1.0;
				rw              = 1.0;
			}
			double getRange()
			{
				return R1 - R0s - R0g + R01g;
			}
		};

		class TWRObsFile
		{
			public:
				TWRObsFile(void);
			public:
				~TWRObsFile(void);
			public:
				bool isValidEpochLine(string strLine, TWRObsLine &obsLine);
				bool open(string strFileName);
			public:
				vector<TWRObsLine> m_data;
		};
	}
}
