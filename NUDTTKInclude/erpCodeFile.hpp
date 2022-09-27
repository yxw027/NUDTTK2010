#pragma once
#include "structDef.hpp"
#include <string>
#include <map>

namespace NUDTTK
{
	struct ErpCodeLine
	{
		double  mjd;       // 修正的儒略日
		double  xp;        // 单位: E-6角秒
		double  yp;        // 单位: E-6角秒    
		double  ut1_utc;   // 单位: E-7秒
		double  s_xp;          
		double  s_yp;           
		double  s_ut1_utc;  

		ErpCodeLine() 
		{
			mjd =  0;
		}
	};

	typedef map<double, ErpCodeLine> ErpCodeMap;

    // code 地球旋转参数周解文件
	class erpCodeFile
	{
	public:
		erpCodeFile(void);
	public:
		~erpCodeFile(void);
	public:
		bool isValidEpochLine(string strLine,ErpCodeLine& erpLine);
		bool add(string strErpCodeFile);
		bool open(string strErpCodeFile);
		bool getErpLine(UTC t, ErpCodeLine& erpLine);
		void toEopRapidFile(string strEopRapidFile);
	public:
		ErpCodeMap m_data;
	};
}
