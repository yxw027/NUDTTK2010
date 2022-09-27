#pragma once
#include <windows.h>
#include <string>
//  Copyright 2012, The National University of Defense Technology at ChangSha

using namespace std;
namespace NUDTTK
{
	namespace Geodyn
	{
		struct GravityModelCoeff
		{
			double   re;         // 地球赤道半径
			double   gm;         // 地球引力常数
			int      n;          // 引力场球谐函数阶数
			double   epoch;      // 参考时刻
	        double   j2_dot;     // j2变化率
			double** ppCmn;      // 球谐函数系数
			double** ppSmn;      // 球谐函数系数

			GravityModelCoeff()
			{
				n      = 0;
				j2_dot = 0;
				ppCmn  = NULL;
				ppSmn  = NULL;
			};

			void init(int n0);

			GravityModelCoeff(int n0)
			{
				init(n0);
			};

			void clear();

			void operator = (const GravityModelCoeff& other);

			~GravityModelCoeff()
			{
				clear();
			};
		};

		class GravityModelFile
		{
		public:
			GravityModelFile(void);
		public:
			~GravityModelFile(void);
		public:
			static bool loadGEOfile(string strFileName, GravityModelCoeff* pCoef);
			static bool loadGCFfile(string strFileName, GravityModelCoeff* pCoef);
			static bool loadMOONGRAILfile(string strFileName, GravityModelCoeff* pCoef);

			
		};
	}
}
