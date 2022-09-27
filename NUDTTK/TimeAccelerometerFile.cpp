#include "TimeAccelerometerFile.hpp"
#include "MathAlgorithm.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	TimeAccelerometerFile::TimeAccelerometerFile(void)
	{
	}

	TimeAccelerometerFile::~TimeAccelerometerFile(void)
	{
	}

	bool TimeAccelerometerFile::isValidEpochLine(string strLine, TimeAccLine& line)
	{
		bool bflag = true;
		char szTime_yyyy[100];
		char szTime_mm[100];
		char szTime_dd[100];
		char szTime_HH[100];
		char szTime_MM[100];
		char szTime_SS[100];
		char szLine_X[100];
		char szLine_Y[100];
		char szLine_Z[100];
		char szAng_X[100];
		char szAng_Y[100];
		char szAng_Z[100];
		char szRes_X[100];
		char szRes_Y[100];
		char szRes_Z[100];
		char szflag[100];
		sscanf(strLine.c_str(), "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s", 
			                     szTime_yyyy,
								 szTime_mm, 
								 szTime_dd, 
								 szTime_HH, 
								 szTime_MM, 
								 szTime_SS, 
								 szLine_X, 
								 szLine_Y, 
								 szLine_Z, 
								 szAng_X, 
								 szAng_Y, 
								 szAng_Z, 
								 szRes_X, 
								 szRes_Y, 
								 szRes_Z, 
								 szflag);
		line.t.year   = atoi(szTime_yyyy);
		line.t.month  = atoi(szTime_mm);
		line.t.day    = atoi(szTime_dd);
		line.t.hour   = atoi(szTime_HH);
		line.t.minute = atoi(szTime_MM);
		line.t.second = atof(szTime_SS);
		line.acc.x    = atof(szLine_X);
		line.acc.y    = atof(szLine_Y);
		line.acc.z    = atof(szLine_Z);
		line.ang.x    = atof(szAng_X);
		line.ang.y    = atof(szAng_Y);
		line.ang.z    = atof(szAng_Z);
		line.res.x    = atof(szRes_X);
		line.res.y    = atof(szRes_Y);
		line.res.z    = atof(szRes_Z);
		line.flag     = atoi(szflag);
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

	bool TimeAccelerometerFile::open(string strAccelerometerFileName)
	{
		FILE * pFile = fopen(strAccelerometerFileName.c_str(), "r+t");
		if(pFile == NULL) 
			return false;
		char line[400];
		m_data.clear();
		while(!feof(pFile))
		{
			if(fgets(line, 400, pFile))
			{
				TimeAccLine accLine;
				if(isValidEpochLine(line, accLine))
					m_data.push_back(accLine);
			}
		}
		fclose(pFile);
		return true;
	}

	bool TimeAccelerometerFile::getAcc(GPST t, POS3D &lin_acc, POS3D &ang_acc, int nlagrange)
	{
		int nDataNumber = int(m_data.size());
		GPST t0 = m_data[0].t;
		// 获得相邻时间间隔，默认间距
		double spanSecond   = m_data[1].t - t0;
		double spanSecond_t = t - t0;
		if(nDataNumber < nlagrange)
			return false;
		// 记录n个已知参考点
		int nLeftPos = int(spanSecond_t / spanSecond);
		if(nLeftPos > nDataNumber)
			return false;
		// nLeftOos左右两边参考点的个数满足nLeftNum + nRightNum = nLangrange
		int nLeftNum  = int(floor(nlagrange / 2.0));
		int nRightNum = int(ceil(nlagrange / 2.0));
		int nBegin, nEnd;
		if(nLeftPos - nLeftNum + 1 < 0)
		{
			nBegin = 0;
			nEnd   = nlagrange - 1;
		}
		else if(nLeftPos + nRightNum >= nDataNumber)
		{
			nBegin = nDataNumber - nlagrange;
			nEnd   = nDataNumber - 1;
		}
		else
		{
			nBegin = nLeftPos - nLeftNum + 1;
			nEnd   = nLeftPos + nRightNum;
		}
		double *xa_t    = new double [nlagrange];
		double *ya_linX = new double [nlagrange];
		double *ya_linY = new double [nlagrange];
		double *ya_linZ = new double [nlagrange];
		double *ya_angX = new double [nlagrange];
		double *ya_angY = new double [nlagrange];
		double *ya_angZ = new double [nlagrange];

		for(int i = nBegin; i <= nEnd; i++)
		{
			xa_t[i - nBegin] = m_data[i].t - t0;
			ya_linX[i - nBegin] = m_data[i].acc.x;
			ya_linY[i - nBegin] = m_data[i].acc.y;
			ya_linZ[i - nBegin] = m_data[i].acc.z;
			ya_angX[i - nBegin] = m_data[i].ang.x;
			ya_angY[i - nBegin] = m_data[i].ang.y;
			ya_angZ[i - nBegin] = m_data[i].ang.z;
		}
		// 插值获得t时刻的加速度
		InterploationLagrange(xa_t, ya_linX, nlagrange, spanSecond_t, lin_acc.x);
		InterploationLagrange(xa_t, ya_linY, nlagrange, spanSecond_t, lin_acc.y);
		InterploationLagrange(xa_t, ya_linZ, nlagrange, spanSecond_t, lin_acc.z);
		InterploationLagrange(xa_t, ya_angX, nlagrange, spanSecond_t, ang_acc.x);
		InterploationLagrange(xa_t, ya_angY, nlagrange, spanSecond_t, ang_acc.y);
		InterploationLagrange(xa_t, ya_angZ, nlagrange, spanSecond_t, ang_acc.z);
		delete xa_t;
		delete ya_linX;
		delete ya_linY;
		delete ya_linZ;
		delete ya_angX;
		delete ya_angY;
		delete ya_angZ;
		return true;
	}

	bool TimeAccelerometerFile::write(string strAccelerometerFileName)
	{
		size_t count = m_data.size();
		if(count <= 0)
			return false;
		FILE *pACT1BFile = fopen(strAccelerometerFileName.c_str(), "w+");
		// 数据部分
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			fprintf(pACT1BFile, "%s %25.15e %25.15e %25.15e %2.1f %2.1f %2.1f %25.15e %25.15e %25.15e %2d\n", 
				                    m_data[s_i].t.toString().c_str(), 
									m_data[s_i].acc.x, m_data[s_i].acc.y, m_data[s_i].acc.z, 
									m_data[s_i].ang.x, m_data[s_i].ang.y, m_data[s_i].ang.z, 
									m_data[s_i].res.x, m_data[s_i].res.y, m_data[s_i].res.z, 
									m_data[s_i].flag);
		}
		fclose(pACT1BFile);
		return true;
	}
}