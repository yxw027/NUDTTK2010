#pragma once
#include "structDef.hpp"
#include "constDef.hpp"
#include "StaOceanLoadingDisplacementFile.hpp"

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SLR
	{
		class SLRPreproc
		{
		public:
			SLRPreproc(void);
		public:
			~SLRPreproc(void);
		public:
			static double  tropCorrect_Marini_IERS2003(double Temperature, double Pressure, double Humidity, double Wavelength, double E, double fai, double h, double Alpha = 1);
			static double  tropCorrect_Marini_IERS2010(double Temperature, double Pressure, double Humidity, double Wavelength, double E, double fai, double h, double Alpha = 1);
			static double  relativityCorrect(POS3D sunPos, POS3D leoPos, POS3D staPos, double Gamma = 1);
			static double  staEccCorrect(POS3D staPos, POS3D leoPos, ENU offset);
			static double  satMassCenterCorrect_J2000(POS3D staPos, POS6D leoPV, POS3D offset);
			static double  satMassCenterCorrect_ECEF(UT1 t, POS3D staPos, POS6D leoPos, POS3D offset);
			static double  satMassCenterCorrect_YawAttitudeModel(POS3D staPos, POS3D leoPos, POS3D sunPos, POS3D offset);
			static double  tideCorrect(GPST t, POS3D sunPos, POS3D moonPos, POS3D staPos, POS3D  leoPos, StaOceanTide sotDatum, double xp, double yp);
		};
	}
}
