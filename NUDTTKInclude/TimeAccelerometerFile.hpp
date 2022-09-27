#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
namespace NUDTTK
{
	struct TimeAccLine
	{
		GPST  t;
		POS3D acc;     // 线性加速度     加速度计仪器坐标系下
		POS3D ang;     // 角加速度
		POS3D res;     // 残差
		int   flag;
	};

	class TimeAccelerometerFile
	{
	public:
		TimeAccelerometerFile(void);
	public:
		~TimeAccelerometerFile(void);
	public:
		bool open(string strAccelerometerFileName);
		bool isValidEpochLine(string strLine, TimeAccLine& line);
		bool getAcc(GPST t, POS3D &lin_acc, POS3D &ang_acc, int nlagrange = 2);    // Flag = 1 线性插值， Flag = 2 拉格朗日插值
		bool write(string  strAccelerometerFileName);
	public:
		vector<TimeAccLine> m_data;
	};
}