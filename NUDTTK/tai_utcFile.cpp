#include "TAI_UTCFile.hpp"
#include <limits>
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	TAI_UTCFile::TAI_UTCFile(void)
	{
	}

	TAI_UTCFile::~TAI_UTCFile(void)
	{
	}

	void TAI_UTCFile::clear()
	{
		m_data.clear();
	}

	bool TAI_UTCFile::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}

	// 子程序名称： isValidEpochLine   
	// 功能：判断当前文本行数据是否为有效时刻行 
	// 变量类型：strLine           : 行文本 
	// 输入：strLine
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/3/02
	// 版本时间：2012/3/02
	// 修改记录：
	// 备注： 
	bool TAI_UTCFile::isValidEpochLine(string strLine)
	{
		bool nFlag = true;
		if(strLine.length() < 34)
			return false;
		int  nYear;
		char strMonth[6];
		int  nDay = 0;
		sscanf(strLine.c_str(), "%4d%*2c%5c%2d", &nYear, strMonth, &nDay);
		strMonth[5]='\0';
		int nMonth = string2MonthId(strMonth);
		if( strLine.c_str()[14] != '-' )
			nFlag = false;
		if( nMonth==0 )
			nFlag = false;
		if( nDay > 31 || nDay < 1 )
			nFlag = false;
		return nFlag;
	}


	// 子程序名称： open   
	// 功能：观测数据解析 
	// 变量类型：strTAI_UTCfileName : 观测数据文件路径
	// 输入：strTAI_UTCfileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/3/02
	// 版本时间：2012/3/02
	// 修改记录：
	// 备注： 
	bool TAI_UTCFile::open(string  strTAI_UTCfileName)
	{
		FILE * pTAI_UTCfile = fopen(strTAI_UTCfileName.c_str(),"r+t");
		if(pTAI_UTCfile == NULL) 
			return false;
		char line[200];
		m_data.clear();
		while(!feof(pTAI_UTCfile))
		{
			if(fgets(line,200, pTAI_UTCfile)) 
			{// 20070706
				if(isValidEpochLine(line))
				{
					int  nYear;
					char strMonth[6];
					int  nDay = 0;
					sscanf(line,"%4d%*2c%5c%2d",&nYear,strMonth,&nDay);
					strMonth[5]='\0';
					int nMonth = string2MonthId(strMonth);
					// 1972  Jan.  1 日以前的跳秒计算直接在程序中固定好，不需要通过文件读取上，花费太多时间
					if( nYear >= 1972 ) // 1972  Jan.  1 日以后的才有效
					{
						TAI_UTCLine Line;
						Line.t = UTC(nYear, nMonth, nDay, 0, 0, 0);
						sscanf(line,"%*29c%14lf",&Line.leapSeconds);
						m_data.push_back(Line);
					}
				}
			}
		}
		fclose(pTAI_UTCfile);
		return true;
	}


	// 子程序名称： getLeapSeconds   
	// 功能：根据UTC时间T获得 TAI-UTC 跳秒
	// 变量类型：t                 :UTC时间
	//           leapSeconds       :TAI-UTC跳秒
	// 输入：t
	// 输出：leapSeconds
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/05/08
	// 版本时间：
	// 修改记录：
	// 备注： 1972/07/01以前的跳秒尚未编写
	bool TAI_UTCFile::getLeapSeconds(UTC t,double& leapSeconds)
	{
		leapSeconds = DBL_MAX;
		double mjd_T = TimeCoordConvert::DayTime2MJD(t); // 获得修改的儒略日
		UTC T_19720101(1972, 1,1,0,0,0); 
		double mjd_19720101 = TimeCoordConvert::DayTime2MJD(T_19720101);
		if(mjd_T >= mjd_19720101) // 1972  Jan.  1 是临界点
		{
			if(isEmpty())
				return false;
			// 为提高效率，从后往前进行搜索
			for(int i = int(m_data.size() - 1); i >= 0; i--)
			{
				TAI_UTCLine Line = m_data[i];
				double dMJD = TimeCoordConvert::DayTime2MJD(Line.t);
				if(mjd_T >= dMJD) 
				{// 找到合适的区间
					leapSeconds = Line.leapSeconds;
					return true;
				}
			}
		}
		else
		{// 暂时不进行处理
			return false;  
			//// 时间必须大于 1961  Jan.  1
			//UTC T_19610101(1961, 1,1,0,0,0);// 1961  Jan.  1
			//UTC T_19610801(1961, 8,1,0,0,0);// 1961  Aug.  1
			//UTC T_19620101(1962, 1,1,0,0,0);// 1962  Jan.  1
			//UTC T_19631101(1963,11,1,0,0,0);// 1963  Nov.  1
			//UTC T_19640101(1964, 1,1,0,0,0);// 1964  Jan.  1
			//UTC T_19640401(1964, 4,1,0,0,0);// 1964  April 1
			//UTC T_19640901(1964, 9,1,0,0,0);// 1964  Sept. 1
			//UTC T_19650101(1965, 1,1,0,0,0);// 1965  Jan.  1
			//UTC T_19650301(1965, 3,1,0,0,0);// 1965  March 1
			//UTC T_19650701(1965, 7,1,0,0,0);// 1965  Jul.  1
			//UTC T_19650901(1965, 9,1,0,0,0);// 1965  Sept. 1
			//UTC T_19660101(1966, 1,1,0,0,0);// 1966  Jan.  1
			//UTC T_19680201(1968, 2,1,0,0,0);// 1968  Feb.  1 
		}
		return true;
	}

	// 子程序名称： TAI2UTC   
	// 功能：根据TAI时间获得 UTC 时间
	// 变量类型：T_TAI      :TAI时间
	//           T_UTC      :UTC时间
	// 输入：T_TAI
	// 输出：T_UTC 
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/05/08
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool TAI_UTCFile::TAI2UTC(TAI T_TAI,UTC& T_UTC)
	{
		/*
		   UTC跳秒节点一般发生在12月或6月的末尾，在每个跳秒区间区间内，跳秒数保持不变，时间尺度保持不变
		   UTC的跳秒区间可表示为：[UTC0，UTC1]
		   对应的TAI时间为：      [TAI0，TAI1]
		   其中：	TAI0 = UTC0 + LeapSeconds ；TAI1 = UTC1 + LeapSeconds
		   在[TAI0-1，TAI0)这1S的时间里，UTC是停表的，这段TAI时间成为“多余”的，此时跳秒操作正在进行中，可认为其尚未发生跳秒 
		   因此，将这段时间统一映射成UTC0
		*/
		double  mjd_TAI = TimeCoordConvert::DayTime2MJD(T_TAI);// 获得本时刻对应的修改儒略日
		UTC TAI_19720101(1972, 1,1,0,0,10);                             // 1972  Jan.  1 对应的跳秒为10秒
		double  mjd_19720101 = TimeCoordConvert::DayTime2MJD(TAI_19720101);
		if(mjd_TAI >= mjd_19720101) // 1972  Jan.  1 是临界点
		{
			if(isEmpty())
				return false;
			// 为提高效率，从后往前进行搜索
			for(int i = int(m_data.size() - 1); i >= 0; i--)
			{
				TAI_UTCLine Line = m_data[i];
				// 获得节点处的TAI时间 : TAI0 = UTC + TAI_UTC
				TAI T_TAI0   = Line.t + Line.leapSeconds;
				TAI T_TAI0_1 = Line.t + Line.leapSeconds - 1;
				double mjd0   = TimeCoordConvert::DayTime2MJD(T_TAI0);
				double mjd0_1 = TimeCoordConvert::DayTime2MJD(T_TAI0_1);
				if(mjd_TAI >= mjd0) 
				{// 找到合适的跳秒区间
					T_UTC = T_TAI - Line.leapSeconds;// UTC = TAI - TAI_UTC
					return true;
				}
				// [TAI0-1，TAI0) = [mjd0_1, mjd0)
				if(mjd_TAI >= mjd0_1 && mjd_TAI < mjd0) // 判断是否处于“多余”TAI时间区间[TAI0-1，TAI0)
				{
					T_UTC = Line.t;                     // 将这段时间统一映射成UTC0
					return true;
				}
			}
		}
		return false;// 1972  Jan.  1  以前的时间不作处理
	}

	// 子程序名称： UTC2TAI   
	// 功能：根据UTC时间获得 TAI 时间
	// 变量类型：T_UTC      :UTC时间
	//           T_TAI      :TAI时间
	// 输入：T_UTC
	// 输出：T_TAI 
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/12/30
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool TAI_UTCFile::UTC2TAI(UTC T_UTC,TAI& T_TAI)
	{// 直接在 T_UTC 上叠加跳秒参数即可
		double leapSeconds;
		if(!getLeapSeconds(T_UTC, leapSeconds))
			return false;
		T_TAI = T_UTC + leapSeconds;
		return true;
	}

		

}
