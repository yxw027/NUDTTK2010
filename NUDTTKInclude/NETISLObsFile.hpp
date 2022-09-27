#pragma once
#include <string>
#include <vector>
#include "structDef.hpp"
#include "Matrix.hpp"
#include <math.h>

namespace NUDTTK
{
	namespace NETPOD
	{
		struct NETISLObsLine
		{

			GPST                     t;
			string                   sat_A;
			string                   sat_B;
			double                   obs_AB;  
			double                   obs_BA; 
			double                   d;
			double                   clk_A;
			double                   clk_B; 
			
			NETISLObsLine()
			{
				obs_AB          = 0.0; // 
				obs_BA          = 0.0; // 
				obs_BA          = 0.0; // 
				clk_A           = 0.0; // 
				clk_B           = 0.0; //
			}
		};

		class NETISLObsFile
		{
		public:
			NETISLObsFile(void);
		public:
			~NETISLObsFile(void);
		public:
			bool write(string strNETISLObsFileName);
			bool open(string strNETISLObsFileName); 
		public:
			vector<NETISLObsLine> m_data;
		};
	}
}
