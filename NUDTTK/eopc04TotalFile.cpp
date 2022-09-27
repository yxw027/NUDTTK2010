#include "eopc04TotalFile.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	Eopc04TotalFile::Eopc04TotalFile(void)
	{
	}

	Eopc04TotalFile::~Eopc04TotalFile(void)
	{
	}

	void Eopc04TotalFile::clear()
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
	bool Eopc04TotalFile::isValidEpochLine(string strLine, Eopc04TotalYearLine& eopc04line)
	{
		bool nFlag = true;
		Eopc04TotalYearLine Line; // Eopc04Line 的构造函数要有必要的初始化, 以保证不出现错误的判断, 因为在读到最后一行时, strLine的有效字符很短, line无法通过strLine来赋值
		sscanf(strLine.c_str(), "%4d%4d%4d%7d%11lf%11lf%12lf%12lf%12lf%12lf",
								 &Line.year,
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
	bool Eopc04TotalFile::open(string  strEopc04TotalfileName)
	{
		FILE * pEopc04Totalfile = fopen(strEopc04TotalfileName.c_str(),"r+t");
		if(pEopc04Totalfile == NULL) 
			return false;
		// 略过前 14 行
		char line[300];             // 读取文件头信息 17 行文字; 2008/07/24 更改为 14
		for(int i = 0; i < 14; i++) 
		{
			fgets(line,300,pEopc04Totalfile);
		}
       // 15 行开始读取文件
		m_data.clear();
		while(!feof(pEopc04Totalfile))
		{
			if(fgets(line,300,pEopc04Totalfile)) 
			{
				Eopc04TotalYearLine eopLine;
				if(isValidEpochLine(line, eopLine))
					m_data.push_back(eopLine);
			}
		}
		fclose(pEopc04Totalfile);
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
	//           3. 2018/05/06 由邵凯修改，多年数据
	// 备注： 
	bool Eopc04TotalFile::getPoleOffset(UTC t, double& x, double& y)
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
			x = m_data[m_data.size() - 1].x;
			y = m_data[m_data.size() - 1].y;
		}
		else
		{// 线性插值
			double u = (mjd - m_data[nN].mjd ) / interval;
			x = (1 - u) * m_data[nN].x + u * m_data[nN + 1].x;
			y = (1 - u) * m_data[nN].y + u * m_data[nN + 1].y;
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
	//           3. 2018/05/06 由邵凯修改，多年数据
	// 备注： 
	bool Eopc04TotalFile::getNutationCorrect(UTC t, double& psi, double& eps)
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
			psi = m_data[m_data.size() - 1].psi;
			eps = m_data[m_data.size() - 1].eps;
		}
		else
		{// 线性插值
			double u = (mjd - m_data[nN].mjd ) / interval;
			psi = (1 - u) * m_data[nN].psi + u * m_data[nN + 1].psi;
			eps = (1 - u) * m_data[nN].eps + u * m_data[nN + 1].eps;
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
	bool Eopc04TotalFile::getUT1_UTC(UTC t,double& ut1_utc)
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
	bool Eopc04TotalFile::getUT1_UTC(UTC t, double& ut1_utc, double& ut1_utc_rate)
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
}
