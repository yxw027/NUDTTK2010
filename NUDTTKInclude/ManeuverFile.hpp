#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <vector>
//  Copyright 2019
using namespace NUDTTK;
namespace NUDTTK
{
	struct ManeuverLine
	{
		BJT     t0; 
		BJT     t1; 
		double  duration;
		int     direction_x;
		int     direction_y;
		int     direction_z;
		int     thruster_id;
		double  thrust_size;
		double  delta_v;
		ManeuverLine()
		{
			duration    = 0.0;
			direction_x = 0;
			direction_y = 0;
			direction_z = 0;
			thruster_id = 0;
			thrust_size = 0.0;
			delta_v     = 0.0;
		};
	};

	class ManeuverFile
	{
	public:
		ManeuverFile(void);
	public:
		~ManeuverFile(void);
	public:
		bool open(string  strFileName);
		bool write(string strFileName);
		bool isValidEpochLine(string strLine, ManeuverLine& line);
	public:
		vector<ManeuverLine>  m_data;
	};
}
