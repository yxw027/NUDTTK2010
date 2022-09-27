#pragma once
#include "structDef.hpp"
#include <string>
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{

	struct TAI_UTCLine
	{
		UTC     t;
		double  leapSeconds;
	};

	class TAI_UTCFile
	{
	public:
		TAI_UTCFile(void);
	public:
		~TAI_UTCFile(void);
	public:
		void     clear();
		bool     isEmpty();
		bool     isValidEpochLine(string strLine);
		bool     open(string  strTAI_UTCfileName);
		bool     getLeapSeconds(UTC t,double& leapSeconds);
		bool     TAI2UTC(TAI T_TAI,UTC& T_UTC);
		bool     UTC2TAI(UTC T_UTC,TAI& T_TAI);
    public:
		vector<TAI_UTCLine> m_data;
	};
}
