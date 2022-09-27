#pragma once
#include <string>
#include <vector>
#include "structDef.hpp"

using namespace std;
//读取JB2008大气模型输入参数：F10.7等

namespace NUDTTK
{
	namespace Geodyn
	{
		struct SolfsmyLine
		{
			int year;
			int day;
			double jd;
			double F10;
			double F81c;
			double S10;
			double S81c;
			double M10;
			double M81c;
			double Y10;
			double Y81c;
			
			SolfsmyLine()
			{
				year = 0;
				day  = 0;
				jd   = 0.0;
			}
		};

		class solfsmyFile
		{
			public:
				solfsmyFile(void);
			public:
				~solfsmyFile(void);
			public:
				bool isValidEpochLine(string strLine, SolfsmyLine &solLine);
				bool open(string strSolfsmyFileName);
				bool getIndex(double jd,  SolfsmyLine &solLine);
				bool getIndex_list(double jd,  SolfsmyLine &solLine);
			public:
				vector<SolfsmyLine> m_data;
		};
	}
}