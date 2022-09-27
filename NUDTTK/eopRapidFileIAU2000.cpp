#include "eopRapidFileIAU2000.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	eopRapidFileIAU2000::eopRapidFileIAU2000(void)
	{
	}

	eopRapidFileIAU2000::~eopRapidFileIAU2000(void)
	{
	}

	void eopRapidFileIAU2000::clear()
	{
		m_data.clear();
	}

	// 子程序名称： isValidEpochLine   
	// 功能：判断当前文本行数据是否为有效时刻行 
	// 变量类型：strLine           : 行文本 
	//           eopLine           : 行数据结构
	// 输入：strLine
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/8/21
	// 版本时间：2012/8/21
	// 修改记录：
	// 备注： 
	bool eopRapidFileIAU2000::isValidEpochLine(string strLine,EopRapidFileIAU2000Line& eopLine)
	{
		bool nFlag = true;
		if(strLine.length() < 78)
			return false;
		char szYear[2+1];
		char szMonth[2+1];
		char szDay[2+1];
		char szMjd[9+1];
		char szPmx[10+1];
		char szPmy[10+1];
		char szUt1_utc[10+1];
		EopRapidFileIAU2000Line Line; // EopRapidFileIAU2000Line 的构造函数要有必要的初始化, 以保证不出现错误的判断, 因为在读到最后一行时, strLine的有效字符很短, line无法通过strLine来赋值
		sscanf(strLine.c_str(), "%2c%2c%2c%9c%*2c%10c%*9c%10c%*9c%*3c%10c",
			                    szYear,
								szMonth,
								szDay,
								szMjd,
                                szPmx,
								szPmy,
								szUt1_utc);

		szYear[2] = '\0';
		szMonth[2] = '\0';
		szDay[2] = '\0';
		szMjd[9] = '\0';
		szPmx[10] = '\0';
		szPmy[10] = '\0';
		szUt1_utc[10] = '\0';

		Line.year    = atoi(szYear);
		Line.month   = atoi(szMonth);
		Line.day     = atoi(szDay);
		Line.mjd     = atof(szMjd);
        Line.pm_x    = atof(szPmx);
		Line.pm_y    = atof(szPmy);
		Line.ut1_utc = atof(szUt1_utc);

		if(Line.year <= 50) // 自主设定 区分是19xx 还是 20xx
			Line.year += 2000;
		else
			Line.year += 1900;

		if(Line.month == MONTH_UNKNOWN)
		   nFlag = false;
		if(Line.day > 31 || Line.day < 0)
		   nFlag = false;
		if(nFlag)
		   eopLine = Line;
		return nFlag;
	}

	// 子程序名称： open   
	// 功能：读取文件
	// 变量类型：strEopRapidFileName : 文件完整路径 
	// 输入：strEopRapidFileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/8/21
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool eopRapidFileIAU2000::open(string  strEopRapidFileName)
	{
		FILE * pEopRapidFile = fopen(strEopRapidFileName.c_str(),"r+t");
		if(pEopRapidFile == NULL) 
			return false;
		m_data.clear();
		char line[300]; 
		while(!feof(pEopRapidFile))
		{
			if(fgets(line,300,pEopRapidFile)) 
			{
				EopRapidFileIAU2000Line eopLine;
				if(isValidEpochLine(line, eopLine))
					m_data.push_back(eopLine);
			}
		}
		fclose(pEopRapidFile);
		return true;
	}

	// 子程序名称： getPoleOffset   
	// 功能：利用插值(二阶线性) 获得时间T的地极偏移
	// 变量类型：t       : UTC时间
	//           x       : 地极偏移x (角秒)
	//           y       : 地极偏移y (角秒)
	// 输入：t
	// 输出：x, y
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/8/21
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool eopRapidFileIAU2000::getPoleOffset(UTC t, double& x, double& y)
	{
		if(m_data.size() <= 1)
			return false;
		double mjd = TimeCoordConvert::DayTime2MJD(t);  // 获得修改的儒略日
		double interval = m_data[1].mjd - m_data[0].mjd;
		int nN = int((mjd - m_data[0].mjd) / interval); // 获得大的区间
		if(nN < 0) 
			return false;
		else if(nN + 1 >= int(m_data.size()))   
		{// 直接赋成最后一个点的值
			x = m_data[m_data.size() - 1].pm_x;
			y = m_data[m_data.size() - 1].pm_y;
		}
		else
		{// 线性插值
			double u = (mjd - m_data[nN].mjd ) / interval;
			x = (1 - u) * m_data[nN].pm_x + u * m_data[nN + 1].pm_x;
			y = (1 - u) * m_data[nN].pm_y + u * m_data[nN + 1].pm_y;
		}
		return true;
	}

	// 子程序名称： getUT1_UTC   
	// 功能：利用插值(二阶线性)获得时间T的UT1_UTC 和线性插值系数
	// 变量类型：t             :UTC时间
	//           ut1_utc       :UT1_UTC(秒)
	//           ut1_utc_rate  :线性插值系数(秒/秒)
	// 输入：t
	// 输出：ut1_utc, ut1_utc_rate
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/08/21
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool eopRapidFileIAU2000::getUT1_UTC(UTC t, double& ut1_utc, double& ut1_utc_rate)
	{
		double mjd = TimeCoordConvert::DayTime2MJD(t); // 获得t对应的修改儒略日
		double interval = m_data[1].mjd - m_data[0].mjd;
		int nN = int((mjd - m_data[0].mjd) / interval); // 获得大的区间
		if(nN < 0) 
			return false;
		else if( nN + 1 >= int(m_data.size()))   
		{// 直接赋成最后一个点的值
			if(fabs(m_data[m_data.size() - 1].ut1_utc - m_data[m_data.size() - 2].ut1_utc) < 0.5)
			{// 克服跨年跳秒的情形
				ut1_utc_rate = (m_data[m_data.size() - 1].ut1_utc - m_data[m_data.size() - 2].ut1_utc) / (interval * 86400.0);
				ut1_utc = m_data[m_data.size() - 1].ut1_utc
						+ ut1_utc_rate * (mjd - m_data[m_data.size() - 1].mjd)  * 86400.0;
			}
			else
			{
				ut1_utc_rate = (m_data[m_data.size() - 2].ut1_utc - m_data[m_data.size() - 3].ut1_utc) / (interval * 86400.0);
				ut1_utc = m_data[m_data.size() - 1].ut1_utc
						+ ut1_utc_rate * (mjd - m_data[m_data.size() - 1].mjd)  * 86400.0;
			}
		}
		else
		{// 线性插值
			double u = (mjd - m_data[nN].mjd) / interval;
			if(fabs(m_data[nN + 1].ut1_utc - m_data[nN].ut1_utc) < 0.5)
			{
				ut1_utc_rate = (m_data[nN + 1].ut1_utc - m_data[nN].ut1_utc)/(interval * 86400.0);// interval 的单位为天
                ut1_utc = (1 - u) * m_data[nN].ut1_utc + u * m_data[nN + 1].ut1_utc;
			}
			else
			{// 克服跨年跳秒的情形
				ut1_utc_rate = (m_data[nN + 1].ut1_utc - 1.0 - m_data[nN].ut1_utc)/(interval * 86400.0);
				ut1_utc = (1 - u) * m_data[nN].ut1_utc + u * (m_data[nN + 1].ut1_utc - 1.0); // 扣除跳秒影响
			}
		}
		return true;
	}

	// 子程序名称： getUT1_UTC   
	// 功能：利用插值(二阶线性)获得时间T的UT1_UTC 和线性插值系数
	// 变量类型：t             :UTC时间
	//           ut1_utc       :UT1_UTC(秒)
	// 输入：t
	// 输出：ut1_utc
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/08/21
	// 版本时间：
	// 修改记录：
	// 备注：
	bool eopRapidFileIAU2000::getUT1_UTC(UTC t, double& ut1_utc)
	{
		double ut1_utc_rate;
		return getUT1_UTC(t, ut1_utc, ut1_utc_rate);
	}
}
