#pragma once
#include <string>
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	namespace Geodyn
	{
		struct OceanPoleTideLine
		{
			int    degree;
			int    order;
			double Anm_Re;
			double Bnm_Re;
			double Anm_Im;
			double Bnm_Im;

			OceanPoleTideLine()
			{
				degree     = 0;
				order      = 0;
				Anm_Re     = 0;
				Bnm_Re     = 0;
				Anm_Im     = 0;
				Bnm_Im     = 0;
			}
		};

		class OceanPoleTideFile
		{
		public:
			OceanPoleTideFile(int N = 360);
		public:
			~OceanPoleTideFile(void);
			bool open(string strfileName);
		public:
			int m_maxDegree;
			vector<OceanPoleTideLine> m_data;
		};
	}
}
