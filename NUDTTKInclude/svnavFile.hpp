#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct SvNavLine
	{
		int     id_PRN;
		int     id_SV;
		int     id_Block;
		int     mass;
		char    yawBiasFlag;
		float   yawRate;
		DayTime t;
		float   x;
		float   y;
		float   z;

		SvNavLine()
		{
			id_PRN = 0;
		    id_SV = 0;
		    id_Block = 0;
		}
	};

	class svnavFile
	{
	public:
		svnavFile(void);
	public:
		~svnavFile(void);
	public:
		void clear();
		bool isValidEpochLine(string strLine, SvNavLine& svnavLine);
		bool open(string strSvNavFileName);
		bool getPCO(DayTime t, int id_PRN, double& x,double& y,double& z, int& id_Block);
		bool getPCO(DayTime t, double ppPCO[MAX_PRN_GPS][3], int ppBlockID[MAX_PRN_GPS]);
	public:
		vector<SvNavLine> m_data;
	};
}
