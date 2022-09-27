#pragma once
#include "structDef.hpp"
#include <string>
#include <map>

namespace NUDTTK
{
	struct ErpGFZLine
	{
		double  mjd;       // 修正的儒略日
		double  xp;        // 单位: E-6角秒
		double  yp;        // 单位: E-6角秒    
		double  ut1_utc;   // 单位: E-7秒
		double  s_xp;          
		double  s_yp;           
		double  s_ut1_utc;  

		ErpGFZLine() 
		{
			mjd =  0;
		}
	};

	typedef map<double, ErpGFZLine> ErpGFZMap;

    // code 地球旋转参数周解文件
	class erpMgexGFZFile
	{
	public:
		erpMgexGFZFile(void);
	public:
		~erpMgexGFZFile(void);
	public:
		bool isValidEpochLine(string strLine,ErpGFZLine& erpLine);
		bool open(string strErpGFZFile);
		bool getErpLine(UTC t, ErpGFZLine& erpLine);
		void toEopRapidFile(string strEopRapidFile);
	public:
		ErpGFZMap m_data;
	};
}
