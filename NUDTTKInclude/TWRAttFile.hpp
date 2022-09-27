#pragma once
#include <string>
#include <vector>
#include "structDef.hpp"

//  Copyright 2017, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
namespace NUDTTK
{
	namespace TwoWayRangingPod
	{
		struct TWRAttLine
		{
			UTC         t;          // 北京时, 我国卫星在轨数据大部分采用北京时
			EULERANGLE  eulerAngle;
		};

		class TWRAttFile
		{
			public:
				TWRAttFile(void);
			public:
				~TWRAttFile(void);
			public:
				bool isValidEpochLine(string strLine, TWRAttLine &attLine);
				bool open(string strFileName);
				bool getAngle(DayTime t, EULERANGLE &eulerAngle, int nlagrange = 4);
			public:
				vector<TWRAttLine> m_data;
			
		};
	}
}

