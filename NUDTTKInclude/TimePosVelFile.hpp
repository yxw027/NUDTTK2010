#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
namespace NUDTTK
{
	//namespace SpaceborneGPSPreproc
	//{
		class TimePosVelFile
		{
		public:
			TimePosVelFile(void);
		public:
			~TimePosVelFile(void);
		public:
			bool open(string  strFileName);
			bool write(string  strFileName);
			bool isEmpty();
			bool isValidEpochLine(string strLine, TimePosVel& timePosVel);
			bool getPosVel(DayTime t, POS6D& posvel, int nlagrange = 8, bool bVelFromPos = false);
			bool getPosVelAcc(DayTime t, POS6D& posvel, POS3D& acc, int nlagrange = 8);
			bool orbComparision_XYZ(GPST t, TimePosVelClock posvel, POS6D& error);
			bool orbComparision_RTN_ECF(GPST t,TimePosVelClock posvel,POS6D& error);
			bool orbComparision_RTN_ECI(GPST t,TimePosVelClock posvel,POS6D& error);
		public:
			vector<TimePosVel> m_data;
		};
	//}
}
