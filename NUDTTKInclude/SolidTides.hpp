#pragma once
#include "structDef.hpp"

//  Copyright 2013, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	class SolidTides
	{
	public:
		SolidTides(void);
	public:
		~SolidTides(void);
	public:
		static double  IERS2003_Table7_5a[11][9]; 
		static double  IERS2003_Table7_5b[5][9];
	public:
		static POS3D solidTideCorrect(GPST t, POS3D sunPos, POS3D moonPos, POS3D staPos, double xp, double yp);
	};
}
