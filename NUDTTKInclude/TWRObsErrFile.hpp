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
		struct TWRObsErrLine
		{
			UTC    t;             // 时刻, 北京时, 我国测站数据大部分采用北京时
			double correct_R;     // 距离改正量, 包括大气对流层、电离层
		};

		class TWRObsErrFile
		{
		public:
			TWRObsErrFile(void);
		public:
			~TWRObsErrFile(void);
		public:
			bool isValidEpochLine(string strLine, TWRObsErrLine &obserrLine);
			bool open(string strTWRObsErrFileName);
			bool write(string strTWRObsErrFileName);
			bool getCorrect_R(UTC t, double &correct_R, int nlagrange = 2);
		public:
			vector<TWRObsErrLine> m_data;
		};
	}
}
