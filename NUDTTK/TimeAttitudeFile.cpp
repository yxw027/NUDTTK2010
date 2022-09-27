#include "TimeAttitudeFile.hpp"
#include "MathAlgorithm.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	TimeAttitudeFile::TimeAttitudeFile(void)
	{
	}

	TimeAttitudeFile::~TimeAttitudeFile(void)
	{
	}
	bool TimeAttitudeFile::isValidEpochLine(string strLine, TimeAttLine& line)
	{
		bool bflag = true;
		char szTime_yyyy[100];
		char szTime_mm[100];
		char szTime_dd[100];
		char szTime_HH[100];
		char szTime_MM[100];
		char szTime_SS[100];
		char szQ1[100];
		char szQ2[100];
		char szQ3[100];
		char szQ4[100];
		char szflag[100];
		sscanf(strLine.c_str(),"%s%s%s%s%s%s%s%s%s%s%s",
							   szTime_yyyy,
							   szTime_mm,
							   szTime_dd,
							   szTime_HH,
							   szTime_MM,
							   szTime_SS,
							   szQ4,//w
							   szQ1,//x
							   szQ2,//y
							   szQ3,//z
							   szflag);
		line.t.year    = atoi(szTime_yyyy);
		line.t.month   = atoi(szTime_mm);
		line.t.day     = atoi(szTime_dd);
		line.t.hour    = atoi(szTime_HH);
		line.t.minute  = atoi(szTime_MM);
		line.t.second  = atof(szTime_SS);
		line.Q4.q1 = atof(szQ1);
		line.Q4.q2 = atof(szQ2);
		line.Q4.q3 = atof(szQ3);
		line.Q4.q4 = atof(szQ4);
		line.flag  = atoi(szflag);
		if(line.t.year == 0)
			bflag = false;
		if(line.t.month  > 12 || line.t.month  <= 0)
			bflag = false;
		if(line.t.day    > 31 || line.t.day    < 0)
			bflag = false;
		if(line.t.hour   > 24 || line.t.hour   < 0)
			bflag = false;
		if(line.t.minute > 60 || line.t.minute < 0)
			bflag = false;
		if(line.t.second > 60 || line.t.second < 0)
			bflag = false;
		if(line.flag != 0)
			bflag = false;
		return bflag;
	}

	bool TimeAttitudeFile::open(string strAttFileName)
	{
		FILE * pFile = fopen(strAttFileName.c_str(), "r+t");
		if(pFile == NULL) 
			return false;
		char line[400];
		m_data.clear();
		while(!feof(pFile))
		{
			if(fgets(line, 400, pFile))
			{
				TimeAttLine attLine;
				if(isValidEpochLine(line, attLine))
					m_data.push_back(attLine);
			}
		}
		fclose(pFile);
		return true;
	}

	bool TimeAttitudeFile::write(string strAttFileName)
	{
		FILE* pFile = fopen(strAttFileName.c_str(), "w+");
        for(size_t s_i = 0; s_i < m_data.size(); s_i = s_i+1)
		{
			m_data[s_i].flag = 0;
			//m_data[s_i].t.second = floor(m_data[s_i].t.second);
			fprintf(pFile, "%s %16.10f %16.10f %16.10f %16.10f \n",  m_data[s_i].t.toString().c_str(),
																	    m_data[s_i].Q4.q4,//尺度参数放前面w
																		m_data[s_i].Q4.q1,//x
																		m_data[s_i].Q4.q2,//y
																		m_data[s_i].Q4.q3);//z
																		//m_data[s_i].flag);
		}
		fclose(pFile);
		return true;
	}

	bool TimeAttitudeFile::getQ4(GPST t, ATT_Q4& Q4, int nlagrange)
	{
		/*FILE *fp;
        fp = fopen("D:\\Test\\test.txt", "a+"); */
		// 姿态数据序列的个数
		int nLagrange_left  = int(floor(nlagrange/2.0));   
		int nLagrange_right = int(ceil (nlagrange/2.0));
		//printf("%10d %10d %10d %10d\n",nlagrange,nLagrange_left,nLagrange_right,m_data.size());
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
		double *xa_t  = new double [nlagrange];
		double *ya_q1 = new double [nlagrange];
		double *ya_q2 = new double [nlagrange];
		double *ya_q3 = new double [nlagrange];
		double *ya_q4 = new double [nlagrange];
		for(int i = nBegin; i <= nEnd; i++)
		{
			 xa_t[i - nBegin] = m_data[i].t - t_Begin;
			ya_q1[i - nBegin] = m_data[i].Q4.q1;
			ya_q2[i - nBegin] = m_data[i].Q4.q2;
			ya_q3[i - nBegin] = m_data[i].Q4.q3;
			ya_q4[i - nBegin] = m_data[i].Q4.q4;
		}
		double max_spanSecond = 0;
		for(int i = nBegin + 1; i <= nEnd; i++)
		{
			if(max_spanSecond < (xa_t[i - nBegin] - xa_t[i - 1 - nBegin]))
				max_spanSecond = (xa_t[i - nBegin] - xa_t[i - 1 - nBegin]);
		}
		InterploationLagrange( xa_t, ya_q1, nlagrange, span_T, Q4.q1);
		InterploationLagrange( xa_t, ya_q2, nlagrange, span_T, Q4.q2);
		InterploationLagrange( xa_t, ya_q3, nlagrange, span_T, Q4.q3);
		InterploationLagrange( xa_t, ya_q4, nlagrange, span_T, Q4.q4);
		delete xa_t;
		delete ya_q1;
		delete ya_q2;
		delete ya_q3;
		delete ya_q4;
		// 需要增加最大插值间隔的控制, 防止姿态数据缺失, 2分钟(0.06 * 60 = 3.6度)
		if(max_spanSecond <= 60 * 4) //2
			return true;
		else
		{
			// printf("%s姿态缺失%.1f秒\n", t.ToString().c_str(), max_span);
			return false;
		}
	}

	bool TimeAttitudeFile::getAttMatrix(GPST t, Matrix& matATT, int nlagrange)
	{
		ATT_Q4 Q4;
		if(!getQ4(t, Q4, nlagrange))
			return false;

		/*
			  惯性坐标系到星体坐标系
			|x|                   |x|
			|y|      =[姿态矩阵] *|y| 
			|z|星体               |z|惯性
		*/
		matATT.Init(3, 3);
		double q1 = Q4.q1;
		double q2 = Q4.q2;
		double q3 = Q4.q3;
		double q4 = Q4.q4;
		matATT.SetElement(0, 0,  q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);
		matATT.SetElement(0, 1,  2 * (q1 * q2 + q3 * q4));
		matATT.SetElement(0, 2,  2 * (q1 * q3 - q2 * q4));
		matATT.SetElement(1, 0,  2 * (q1 * q2 - q3 * q4));
		matATT.SetElement(1, 1, -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4);
		matATT.SetElement(1, 2,  2 * (q2 * q3 + q1 * q4));
		matATT.SetElement(2, 0,  2 * (q1 * q3 + q2 * q4));
		matATT.SetElement(2, 1,  2 * (q2 * q3 - q1 * q4));
		matATT.SetElement(2, 2, -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4);
		return true;
	}
}
