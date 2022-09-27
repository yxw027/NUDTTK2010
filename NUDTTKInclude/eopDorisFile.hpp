#pragma once
#include "structDef.hpp"
#include <string>
#include <map>

namespace NUDTTK
{
	namespace DORIS
	{
		struct EopDorisLine
		{
			double  mjd;       // 修正的儒略日
			double  xp;        // 单位: E-6角秒
			double  yp;        // 单位: E-6角秒    
			double  ut1_utc;   // 单位: E-7秒
			double  lod;       // 单位: E-7秒/天
			double  s_xp;          
			double  s_yp;           
			double  s_ut1_utc;  
			double  s_lod;  

			EopDorisLine() 
			{
				mjd =  0;
			}
		};

		typedef map<double, EopDorisLine> EopDorisMap;

		class eopDorisFile
		{
		public:
			eopDorisFile(void);
		public:
			~eopDorisFile(void);
		public:
			bool isValidEpochLine(string strLine,EopDorisLine& eopLine);
			bool open(string strEopDorisFile);
			bool getEopLine(UTC t, EopDorisLine& eopLine);
			void toEopRapidFile(string strEopRapidFile);
		public:
			EopDorisMap m_data;
		};
	}
}
