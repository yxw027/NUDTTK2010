#pragma once
#include <string>
#include <vector>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	namespace Geodyn
	{
		struct OceanTideLine
		{
			int    n1;
			int    n2;
			int    n3;
			int    n4;
			int    n5;
			int    n6;
			char   szName[5];
			int    degree;
			int    order;
			double c_positive;
			double s_positive;
			double c_negative;
			double s_negative;

			OceanTideLine()
			{
				n1         = 0;
				n2         = 0;
				n3         = 0;
				n4         = 0;
				n5         = 0;
				n6         = 0;
				degree     = 0;
				order      = 0;
				c_positive = 0;
				s_positive = 0;
				c_negative = 0;
				s_negative = 0;
			}
		};

		class OceanTideFile
		{
		public:
			OceanTideFile(int N = 20);
		public:
			~OceanTideFile(void);
			bool open(string strfileName);
			bool openFES2004ModelFile(string strfileName); //FES2004模型
			bool openFES2014ModelFile(string strfileName); //FES2014模型
		public:
			int m_maxDegree;
			vector<OceanTideLine> m_data;
		};
	}
}
