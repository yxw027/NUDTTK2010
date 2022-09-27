#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <vector>
#include "Matrix.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
namespace NUDTTK
{
	struct TimeAttLine
	{
		GPST    t;
		ATT_Q4  Q4;
		int     flag;
	};
	class TimeAttitudeFile
	{
	public:
		TimeAttitudeFile(void);
	public:
		~TimeAttitudeFile(void);
	public:
		bool open(string  strAttFileName);
		bool write(string  strAttFileName);
		bool isValidEpochLine(string strLine, TimeAttLine& line);
		bool getQ4(GPST t, ATT_Q4& Q4, int nlagrange = 4);
		bool getAttMatrix(GPST t, Matrix& matATT, int nlagrange = 4);
	public:
		vector<TimeAttLine>  m_data;
	};
}
