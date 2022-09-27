#include "LTATTFile.hpp"
#include "MathAlgorithm.hpp"
#include "AttitudeTrans.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	TimeAttitudeFile_LT::TimeAttitudeFile_LT(void)
	{
	}

	TimeAttitudeFile_LT::~TimeAttitudeFile_LT(void)
	{
	}
	// 程序说明：LT一号卫星的姿态数据由欧拉角转换到四元素
    bool TimeAttitudeFile_LT::LT2StdQ4(TimeAttLine& m_line,TimeAttLine_LT line)
	{
		//北京时间转成GPS时间，LT一号的姿态数据给的是北京时间
		m_line.t = line.t;
		m_line.flag = line.flag;
		/*double c1 = acos(line.angle.xRoll / 2);
		double s1 = asin(line.angle.xRoll / 2);
		double c2 = acos(line.angle.yPitch / 2);
		double s2 = asin(line.angle.yPitch / 2);
		double c3 = acos(line.angle.zYaw / 2);
		double s3 = asin(line.angle.zYaw / 2);
		double c1c2 = c1 * c2;
		double s1s2 = s1 * s2;
		m_line.Q4.q4 = (c1c2 * c3 + s1s2 * s3);
		m_line.Q4.q1 = (c1c2 * s3 + s1s2 * c3);
		m_line.Q4.q2 = (s1 * c2 * c3 + c1 * s2 * s3);
		m_line.Q4.q3 = (c1 * s2 * c3 - s1 * c2 * s3);*/
		ATT_Q4 q1;
		q1.q1 = 0;
		q1.q2 = 0;
		q1.q3 = sin(line.angle.zYaw / 2);
		q1.q4 = cos(line.angle.zYaw / 2);
		ATT_Q4 q2;
		q2.q1 = sin(line.angle.xRoll / 2);
		q2.q2 = 0;
		q2.q3 = 0;
		q2.q4 = cos(line.angle.xRoll / 2);
		ATT_Q4 q3;
		q3.q1 = 0;
		q3.q2 = sin(line.angle.yPitch / 2);
		q3.q3 = 0;
		q3.q4 = cos(line.angle.yPitch / 2);
		m_line.Q4 = q1 * q2 * q3;
		if(m_line.Q4.q1 != 0.0||m_line.Q4.q2 != 0.0|| m_line.Q4.q2 != 0.0|| m_line.Q4.q4 != 0.0)
			return true;
		else
			return false;
	}
	// 程序说明：读取文件的每一行，并验证是否为有效行
	bool TimeAttitudeFile_LT::isValidEpochLine(string strLine, TimeAttLine_LT& line)
	{
		bool bflag = true;
		char BaoCount[100];
		char BeijingCount[100];
		char szTime_yyyy[100];
		char szTime_mm[100];
		char szTime_dd[100];
		char szTime_HH[100];
		char szTime_MM[100];
		char szTime_SS[100];
		char szEx[100];
		char szEy[100];
		char szEz[100];
		char szVEx[100];
		char szVEy[100];
		char szVEz[100];
		char szflag[100];
		sscanf(strLine.c_str(),"%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
								BaoCount,
								BeijingCount,
							   szTime_yyyy,
							   szTime_mm,
							   szTime_dd,
							   szTime_HH,
							   szTime_MM,
							   szTime_SS,
							   szEx,
							   szEy,
							   szEz,
							   szVEx,
							   szVEy,
							   szVEz,
							   szflag);
		line.t.year    = atoi(szTime_yyyy);
		line.t.month   = atoi(szTime_mm);
		line.t.day     = atoi(szTime_dd);
		line.t.hour    = atoi(szTime_HH);
		line.t.minute  = atoi(szTime_MM);
		line.t.second  = atof(szTime_SS);
		line.angle.xRoll  = atof(szEx);
		line.angle.yPitch = atof(szEy);
		line.angle.zYaw   = atof(szEz);
		line.angle.xRoll  = line.angle.xRoll * PI/180.0;//角度转弧度
		line.angle.yPitch = line.angle.yPitch* PI/180.0;
		line.angle.zYaw   = line.angle.zYaw  * PI/180.0;
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
	// 程序说明：
	bool TimeAttitudeFile_LT::open(string strAttFileName)
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
				TimeAttLine_LT attLine;
				if(isValidEpochLine(line, attLine))
					m_data.push_back(attLine);
			}
		}
		fclose(pFile);
		return true;
	}
	// 程序说明：
	bool TimeAttitudeFile_LT::write(string strAttFileName)
	{
		FILE* pFile = fopen(strAttFileName.c_str(), "w+");
        for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			fprintf(pFile, "%s %16.10f %16.10f %16.10f\n",  m_data[s_i].t.toString().c_str(),
																	    m_data[s_i].angle.xRoll,
																		m_data[s_i].angle.yPitch,
																		m_data[s_i].angle.zYaw);
																		//m_data[s_i].flag);
		}
		fclose(pFile);
		return true;
	}
	//
}