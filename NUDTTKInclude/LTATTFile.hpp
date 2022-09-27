#pragma once
#include "TimeAttitudeFile.hpp"
#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>
//
namespace NUDTTK
{
		struct TimeAttLine_LT
		{
			BJT            t;
			EULERANGLE  angle;
			int          flag;
		};
		//
		class TimeAttitudeFile_LT
		{
		public:
			TimeAttitudeFile_LT(void);
		public:
			~TimeAttitudeFile_LT(void);
		public:
			bool open(string  strAttFileName);
			bool write(string  strAttFileName);
			bool isValidEpochLine(string strLine, TimeAttLine_LT& line);
			bool LT2StdQ4(TimeAttLine& m_line,TimeAttLine_LT line);
		public:
			vector<TimeAttLine_LT>  m_data;
		};
}