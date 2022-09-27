#include "eopc04File.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	Eopc04File::Eopc04File(void)
	{
	}

	Eopc04File::~Eopc04File(void)
	{
	}

	void Eopc04File::clear()
	{
		m_data.clear();
	}

	// 子程序名称： isValidEpochLine   
	// 功能：判断当前文本行数据是否为有效时刻行 
	// 变量类型：strLine           : 行文本 
	//           eopc04line        : 行数据结构
	// 输入：strLine
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/3/30
	// 版本时间：2007/3/30
	// 修改记录：
	// 备注： 
	bool Eopc04File::isValidEpochLine(string strLine, Eopc04Line& eopc04line)
	{
		bool nFlag = true;
		Eopc04Line Line; // Eopc04Line 的构造函数要有必要的初始化, 以保证不出现错误的判断, 因为在读到最后一行时, strLine的有效字符很短, line无法通过strLine来赋值
		sscanf(strLine.c_str(), "%*4c%4d%4d%7d%11lf%11lf%12lf%12lf%12lf%12lf",
			                    &Line.month,
								&Line.day,
								&Line.mjd,
								&Line.x,
								&Line.y,
								&Line.ut1_utc,
								&Line.lod,
								&Line.psi,
								&Line.eps);

		if(Line.month == MONTH_UNKNOWN)
		   nFlag = false;
		if(Line.day > 31 || Line.day < 0)
		   nFlag = false;
		if(nFlag)
		   eopc04line = Line;
		return nFlag;
	}

	// 子程序名称： open   
	// 功能：读取文件
	// 变量类型：strEopc04fileName : 文件完整路径 
	// 输入：strEopc04fileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/3/30
	// 版本时间：2008/07/24
	// 修改记录：1. 2008/07/24 由谷德峰修改, 文件格式发生变化
	// 备注： 
	bool Eopc04File::open(string  strEopc04fileName)
	{
		//if(!isWildcardMatch(strEopc04fileName.c_str(), "*eopc04*.*", true))
		//{
		//	printf(" %s 文件名不匹配!\n", strEopc04fileName.c_str());
		//	return false;
		//}
		FILE * pEopc04file = fopen(strEopc04fileName.c_str(),"r+t");
		if(pEopc04file == NULL) 
			return false;
		Eopc04YearRecord eopc04;     // 一年的数据记录
		char strYear[3];
		strEopc04fileName.copy(strYear, 2, strEopc04fileName.length() - 2);
		strYear[2]='\0';
		eopc04.year = atoi(strYear);
		if(eopc04.year <= 50)       // 自主设定 区分是19xx 还是 20xx
			eopc04.year += 2000;
		else
			eopc04.year += 1900;
		char line[300];             // 读取文件头信息 17 行文字; 2008/07/24 更改为 14
		for(int i = 0; i < 14; i++) 
		{
			fgets(line,300,pEopc04file);
			eopc04.strText[i] = line;
		}
		eopc04.eopc04Linelist.clear();
		while(!feof(pEopc04file))
		{
			if(fgets(line,300,pEopc04file)) 
			{
				Eopc04Line eopc04line;
				if(isValidEpochLine(line, eopc04line))
					eopc04.eopc04Linelist.push_back(eopc04line);
			}
		}
		eopc04.interval  = eopc04.eopc04Linelist[1].mjd - eopc04.eopc04Linelist[0].mjd; // 记录间隔
		eopc04.mjd_first = eopc04.eopc04Linelist[0].mjd;
		// 清除 m_data 中 eopc04.year 的数据 
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			if(m_data[s_i].year == eopc04.year)
			{
				m_data.erase(m_data.begin() + s_i);
				break;
			}
		}
		m_data.push_back(eopc04);  // 添加此数据
		fclose(pEopc04file);
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
	// 创建时间：2007/3/30
	// 版本时间：
	// 修改记录：1. 2007/11/28 由谷德峰修改, 数据越界
	//           2. 2012/03/04 由谷德峰修改, 数据跨年越界
	// 备注： 
	bool Eopc04File::getPoleOffset(UTC t, double& x, double& y)
	{
		if(m_data.size() <= 0)
			return false;
		
		bool bFind = false;
		size_t s_i = 0;
		int year_max = m_data[0].year;
		size_t year_max_i = 0;
		for(s_i = 0; s_i < m_data.size(); s_i++)
		{
			if(year_max < m_data[s_i].year)
			{
				year_max = m_data[s_i].year;
				year_max_i = s_i;
			}
			if(m_data[s_i].year == t.year)
			{
				bFind = true;
				break;
			}
		}
		if(!bFind) // 未发现本年的数据
		{
			if(t.year > year_max)
			{// 直接返回最末尾的数据, 2012/03/04
				x = m_data[year_max_i].eopc04Linelist[m_data[year_max_i].eopc04Linelist.size() - 1].x;
				y = m_data[year_max_i].eopc04Linelist[m_data[year_max_i].eopc04Linelist.size() - 1].y;
				return true;
			}
			else
			{
				return false;
			}
		}

		Eopc04YearRecord eopc04 = m_data[s_i];
		double mjd = TimeCoordConvert::DayTime2MJD(t);  // 获得修改的儒略日
		int nN = int((mjd - eopc04.mjd_first) / eopc04.interval); // 获得大的区间
		if(nN < 0) // 2007/11/28, 数据越界进行了修改
			return false;
		else if(nN + 1 >= int(eopc04.eopc04Linelist.size()))   
		{// 直接赋成最后一个点的值
			x = eopc04.eopc04Linelist[eopc04.eopc04Linelist.size() - 1].x;
			y = eopc04.eopc04Linelist[eopc04.eopc04Linelist.size() - 1].y;
		}
		else
		{// 线性插值
			double u = (mjd - eopc04.mjd_first - nN * eopc04.interval) / eopc04.interval;
			x = (1 - u) * eopc04.eopc04Linelist[nN].x + u * eopc04.eopc04Linelist[nN + 1].x;
			y = (1 - u) * eopc04.eopc04Linelist[nN].y + u * eopc04.eopc04Linelist[nN + 1].y;
		}
		return true;
	}

	// 子程序名称： getNutationCorrect   
	// 功能：利用插值(二阶线性)获得时间T的 IAU1980 模型的章动修正量
	// 变量类型：t            : UTC时间
	//           psi          : 黄经章动改进量(度)
	//           eps          : 交角章动改进量(度)
	// 输入：t
	// 输出：psi, eps
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/3/30
	// 版本时间：
	// 修改记录：1. 2007/11/28 由谷德峰修改, 数据越界
	//           2. 2012/03/04 由谷德峰修改, 数据跨年越界
	// 备注： 
	bool Eopc04File::getNutationCorrect(UTC t, double& psi, double& eps)
	{
		if(m_data.size() <= 0)
			return false;

		bool bFind = false;
		size_t s_i = 0;
		int year_max = m_data[0].year;
		size_t year_max_i = 0;
		for(s_i = 0; s_i < m_data.size(); s_i++)
		{
			if(year_max < m_data[s_i].year)
			{
				year_max = m_data[s_i].year;
				year_max_i = s_i;
			}
			if(m_data[s_i].year == t.year)
			{
				bFind=true;
				break;
			}
		}
		if(!bFind)                                                   // 未发现本年的数据
		{
			if(t.year > year_max)
			{// 直接返回最末尾的数据, 2012/03/04
				psi = m_data[year_max_i].eopc04Linelist[m_data[year_max_i].eopc04Linelist.size() - 1].psi;
				eps = m_data[year_max_i].eopc04Linelist[m_data[year_max_i].eopc04Linelist.size() - 1].eps;
				psi = psi / 3600.0;
		        eps = eps / 3600.0;
				return true;
			}
			else
			{
				return false;
			}
		}

		Eopc04YearRecord eopc04 = m_data[s_i];
		double mjd = TimeCoordConvert::DayTime2MJD(t);  // 获得修改的儒略日
		int nN = int((mjd - eopc04.mjd_first) / eopc04.interval); // 获得大的区间
		if(nN < 0)  // 2007/11/28, 数据越界进行了修改
			return false;
		else if(nN + 1 >= int(eopc04.eopc04Linelist.size()))   
		{// 直接赋成最后一个点的值
			psi = eopc04.eopc04Linelist[eopc04.eopc04Linelist.size() - 1].psi;
			eps = eopc04.eopc04Linelist[eopc04.eopc04Linelist.size() - 1].eps;
		}
		else
		{// 线性插值
			double u = (mjd - eopc04.mjd_first - nN * eopc04.interval) / eopc04.interval;
			psi = (1 - u) * eopc04.eopc04Linelist[nN].psi + u * eopc04.eopc04Linelist[nN+1].psi;
			eps = (1 - u) * eopc04.eopc04Linelist[nN].eps + u * eopc04.eopc04Linelist[nN+1].eps;
		}
		psi = psi / 3600.0;
		eps = eps / 3600.0;
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
	// 创建时间：2007/05/08
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool Eopc04File::getUT1_UTC(UTC t,double& ut1_utc)
	{
		double ut1_utc_rate;
		return getUT1_UTC(t, ut1_utc, ut1_utc_rate);
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
	// 创建时间：2007/05/08
	// 版本时间：
	// 修改记录：1. 2007/11/28 由谷德峰修改, 数据越界
	//           2. 2012/03/04 由谷德峰修改, 数据跨年越界采取预报的方式
	// 备注： 
	bool Eopc04File::getUT1_UTC(UTC t, double& ut1_utc, double& ut1_utc_rate)
	{
		double mjd = TimeCoordConvert::DayTime2MJD(t); // 获得t对应的修改儒略日
		bool bFind = false;
		size_t s_i = 0;
		int year_max = m_data[0].year;
		size_t year_max_i = 0;
		for(s_i = 0; s_i < m_data.size(); s_i++)
		{
			if(year_max < m_data[s_i].year)
			{
				year_max = m_data[s_i].year;
				year_max_i = s_i;
			}
			if(m_data[s_i].year == t.year)
			{
				bFind=true;
				break;
			}
		}

		if(!bFind) // 未发现本年的数据
		{// 没有数据记录, 此处以 0 进行填充, 防止溢出, 2007/11/25
			if(t.year > year_max)
			{// ut1_utc_rate 直接返回最末尾的数据, 2012/03/04
			 // ut1_utc = 最末尾的数据 + 预报增量
				ut1_utc_rate = (m_data[year_max_i].eopc04Linelist[m_data[year_max_i].eopc04Linelist.size() - 1].ut1_utc - m_data[year_max_i].eopc04Linelist[m_data[year_max_i].eopc04Linelist.size() - 2].ut1_utc) / (m_data[year_max_i].interval * 86400.0);
				ut1_utc = m_data[year_max_i].eopc04Linelist[m_data[year_max_i].eopc04Linelist.size() - 1].ut1_utc
					    + ut1_utc_rate * (mjd - m_data[year_max_i].eopc04Linelist[m_data[year_max_i].eopc04Linelist.size() - 1].mjd)  * 86400.0;
				return true;
			}
			else
			{
				ut1_utc_rate = 0;
				ut1_utc = 0;
				return false;
			}
		}

		Eopc04YearRecord eopc04 = m_data[s_i];
		int nN = int(floor((mjd - eopc04.mjd_first) / eopc04.interval));  // 获得大的区间
		if(nN < 0) // 2007/11/28, 数据越界进行了修改
			return false;
		else if( nN + 1 >= int(eopc04.eopc04Linelist.size()))   
		{// 直接赋成最后一个点的值
			ut1_utc_rate = (eopc04.eopc04Linelist[eopc04.eopc04Linelist.size() - 1].ut1_utc - eopc04.eopc04Linelist[eopc04.eopc04Linelist.size() - 2].ut1_utc) / (eopc04.interval * 86400.0);
			ut1_utc = eopc04.eopc04Linelist[eopc04.eopc04Linelist.size() - 1].ut1_utc
				    + ut1_utc_rate * (mjd - eopc04.eopc04Linelist[eopc04.eopc04Linelist.size() - 1].mjd)  * 86400.0;
		}
		else
		{// 线性插值
			double u = (mjd - eopc04.mjd_first - nN * eopc04.interval) / eopc04.interval;
			ut1_utc = (1 - u) * eopc04.eopc04Linelist[nN].ut1_utc + u * eopc04.eopc04Linelist[nN + 1].ut1_utc;
			ut1_utc_rate = (eopc04.eopc04Linelist[nN + 1].ut1_utc - eopc04.eopc04Linelist[nN].ut1_utc)/(eopc04.interval * 86400.0);// eopc04.interval 的单位为天
		}
		return true;
	}
}
