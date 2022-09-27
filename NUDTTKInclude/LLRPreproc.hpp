#pragma once
#include "structDef.hpp"
#include "constDef.hpp"
#include "StaOceanLoadingDisplacementFile.hpp"

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SLR
	{
		class LLRPreproc
		{
		public:
			LLRPreproc(void);
		public:
			~LLRPreproc(void);
		public:
			static double  relativityCorrect(POS3D sunPos, POS3D leoPos, POS3D staPos, double Gamma = 1);
		};
	}
}
