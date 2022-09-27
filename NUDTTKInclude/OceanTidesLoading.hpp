#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "StaOceanLoadingDisplacementFile.hpp"

namespace NUDTTK
{
	class OceanTidesLoading
	{
	public:
		OceanTidesLoading(void);
	public:
		~OceanTidesLoading(void);
	public:
		static bool  argle_Schwiderski(int nYear, double dDay, double angle[11]);
		static ENU   oceanTideLoadingENUCorrect(GPST t, StaOceanTide sotDatum);
		static POS3D oceanTideLoadingCorrect(GPST t, POS3D staPos, StaOceanTide sotDatum);
	};
}
