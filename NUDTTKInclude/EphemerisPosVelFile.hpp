#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <vector>

using namespace NUDTTK;
namespace NUDTTK
{
	struct EphemerisPosVel
	{
		double  sec;
		POS3D   pos;
		POS3D   vel;
		
		POS6D getPosVel()
		{
			POS6D pv;
			pv.x  = pos.x;
			pv.y  = pos.y;
			pv.z  = pos.z;
			pv.vx = vel.x;
			pv.vy = vel.y;
			pv.vz = vel.z;
			return pv;
		}
	};
	class EphemerisPosVelFile
	{
	public:
		EphemerisPosVelFile(void);
	public:
		~EphemerisPosVelFile(void);
	public:
		bool open(string  strFileName);
		bool write(string  strFileName);
		bool isEmpty();
		bool isValidEpochLine(string strLine, EphemerisPosVel& timePosVel);
	public:
		vector<EphemerisPosVel> m_data;
	};
}
