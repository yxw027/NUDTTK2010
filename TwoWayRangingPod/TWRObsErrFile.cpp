#include "TWRObsErrFile.hpp"
#include "MathAlgorithm.hpp"
#include <math.h>

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace TwoWayRangingPod
	{
		TWRObsErrFile::TWRObsErrFile(void)
		{
		}

		TWRObsErrFile::~TWRObsErrFile(void)
		{
		}

		bool TWRObsErrFile::isValidEpochLine(string strLine, TWRObsErrLine &obserrLine)
		{
			bool Flag = true;
			TWRObsErrLine Line;
			sscanf(strLine.c_str(), "%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2lf%*1c%8lf",
				                    &Line.t.year,
									&Line.t.month,
									&Line.t.day,
									&Line.t.hour,
									&Line.t.minute,
									&Line.t.second,
									&Line.correct_R);
			//判断行是否有效
			if(Line.t.year > 3000 || Line.t.year < 0)
				Flag = false;
			if(Line.t.month > 12 || Line.t.month < 0)
				Flag = false;
			if(Line.t.day > 31 || Line.t.day < 0)
				Flag = false;
			if(Line.t.hour > 24 || Line.t.hour < 0)
				Flag = false;
			if(Line.t.minute > 60 || Line.t.minute < 0)
				Flag = false;
			if(Line.t.second > 60.0 || Line.t.second < 0.0)
				Flag = false;
			if(Flag)
				obserrLine = Line;
			return Flag;
		}

		bool TWRObsErrFile::open(string strTWRObsErrFileName)
		{
			bool Flag = true;
			FILE *pFile = fopen(strTWRObsErrFileName.c_str(), "r+t");
			if(pFile == NULL)
				return false;
			char Line[100];
			m_data.clear();
			while(!feof(pFile))
			{
				if(fgets(Line, 100, pFile))
				{
					TWRObsErrLine ErrLine;
					if(isValidEpochLine(Line, ErrLine))
						m_data.push_back(ErrLine);
				}
			}
			fclose(pFile);
			return true;
		}

		bool TWRObsErrFile::getCorrect_R(UTC t, double &correct_R, int nlagrange)
		{
			// 姿态数据序列的个数
			int nLagrange_left  = int(floor(nlagrange/2.0));   
			int nLagrange_right = int(ceil (nlagrange/2.0));
			if(m_data.size() < size_t(nlagrange))
				return false;
			// 插值时考虑到钟差的影响, 以准确轨道采样时刻为参考
			DayTime t_Begin = m_data[0].t; // 姿态序列初始时间
			DayTime t_End   = m_data[m_data.size() - 1].t; // 姿态序列结束时间
			double  span_Total = t_End - t_Begin;
			double  span_T     = t - t_Begin; // 换算成相对时间
			if(span_T < 0 || span_T > span_Total)  // 确保 span_T 在有效范围之内
				return false;
			// 采用二分法, 2008/05/11
			int nLeftPos = -1;
			size_t left  =  1;
			size_t right = m_data.size() - 1;
			int n = 0;
			while(left < right)
			{
				n++;
				int middle  = int(left + right)/2;
				double time_L = (m_data[middle - 1].t - t_Begin);
				double time_R = (m_data[middle].t     - t_Begin);
				if(span_T >= time_L && span_T <= time_R) 
				{// 终止条件
					nLeftPos = middle - 1;
					break;
				}
				if(span_T < time_L) 
					right = middle - 1;
				else 
					left  = middle + 1;
			}
			if(right == left)
			{
				double time_L = (m_data[left - 1].t - t_Begin);
				double time_R = (m_data[left].t     - t_Begin);
				if(span_T >= time_L && span_T <= time_R) 
				{// 终止条件
					nLeftPos = int(left) - 1;
				}
			}
			if(nLeftPos == -1)
				return false;
			// 确定插值区间位置 [nBegin, nEnd], nEnd - nBegin + 1 = nLagrange
			int nBegin, nEnd; 
			if(nLeftPos - nLagrange_left + 1 < 0) 
			{
				nBegin = 0;
				nEnd   = nlagrange - 1;
			}
			else if(nLeftPos + nLagrange_right >= int(m_data.size()))
			{
				nBegin = int(m_data.size()) - nlagrange;
				nEnd = int(m_data.size()) - 1;
			}
			else
			{
				nBegin = nLeftPos - nLagrange_left + 1;
				nEnd   = nLeftPos + nLagrange_right;
			}
			// 整理插值参考点
			double *xa_t   = new double [nlagrange];
			double *ya_Err = new double [nlagrange];
			for(int i = nBegin; i <= nEnd; i++)
			{
				xa_t[i - nBegin]   = m_data[i].t - t_Begin;
				ya_Err[i - nBegin] = m_data[i].correct_R;
			}
			double max_spanSecond = 0;
			for(int i = nBegin + 1; i <= nEnd; i++)
			{
				if(max_spanSecond < (xa_t[i - nBegin] - xa_t[i - 1 - nBegin]))
					max_spanSecond = (xa_t[i - nBegin] - xa_t[i - 1 - nBegin]);
			}
			InterploationLagrange(xa_t, ya_Err, nlagrange, span_T, correct_R);
			delete xa_t;
			delete ya_Err;
			// 需要增加最大插值间隔的控制, 防止数据缺失, 1小时
			if(max_spanSecond <= 3600.0)
				return true;
			else
			{
				// printf("%s误差修正数据缺失%.1f秒\n", t.ToString().c_str(), max_span);
				return false;
			}
		}

		bool TWRObsErrFile::write(string strTWRObsErrFileName)
		{
			FILE *pFile = fopen(strTWRObsErrFileName.c_str(), "w+");
			if(pFile == NULL)
				return false;
			for(size_t s_i = 0; s_i < m_data.size(); s_i++)
			{
				fprintf(pFile, "%4d %02d %02d %02d %02d %.2f %f\n",
					            m_data[s_i].t.year,
								m_data[s_i].t.month,
								m_data[s_i].t.day,
								m_data[s_i].t.hour,
								m_data[s_i].t.minute,
								m_data[s_i].t.second,
								m_data[s_i].correct_R);
			}
			fclose(pFile);
			return true;
		}
	}
}
