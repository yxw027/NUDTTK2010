#include "erpCodeFile.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	erpCodeFile::erpCodeFile(void)
	{
	}

	erpCodeFile::~erpCodeFile(void)
	{
	}

	bool erpCodeFile::isValidEpochLine(string strLine,ErpCodeLine& erpLine)
	{
		bool nFlag = true;
		if(strLine.length() < 66)
			return false;
		char szMjd[8+1];
		char szXp[9+1];
		char szYp[9+1];
		char szUt1_utc[9+1];
		char szS_Xp[6+1];
		char szS_Yp[6+1];
		char szS_Ut1_utc[6+1];
		ErpCodeLine Line; // EopRapidFileIAU2000Line 的构造函数要有必要的初始化, 以保证不出现错误的判断, 因为在读到最后一行时, strLine的有效字符很短, line无法通过strLine来赋值
		sscanf(strLine.c_str(), "%8c%9c%9c%9c%*7c%6c%6c%6c",
								szMjd,
								szXp,
								szYp,
								szUt1_utc,
								szS_Xp,
								szS_Yp,
								szS_Ut1_utc);
		szMjd[8] = '\0';
		szXp[9] = '\0';
		szYp[9] = '\0';
		szUt1_utc[9] = '\0';
		szS_Xp[6] = '\0';
		szS_Yp[6] = '\0';
		szS_Ut1_utc[6] = '\0';
        Line.mjd = atof(szMjd);
		Line.xp = atof(szXp);
		Line.yp = atof(szYp);
		Line.ut1_utc = atof(szUt1_utc);
		Line.s_xp = atof(szS_Xp);
		Line.s_yp = atof(szS_Yp);
		Line.s_ut1_utc = atof(szS_Ut1_utc);
		if(Line.mjd <= 0.0)
		   nFlag = false;
		if(nFlag)
		   erpLine = Line;
		return nFlag;
	}

	bool erpCodeFile::open(string strErpCodeFile)
	{
		FILE * pErpCodeFile = fopen(strErpCodeFile.c_str(),"r+t");
		if(pErpCodeFile == NULL) 
			return false;
		m_data.clear();
		char line[300]; 
		for(int i = 0; i < 6; i++) // 跳过文件头
			fgets(line,300,pErpCodeFile);
		while(!feof(pErpCodeFile))
		{
			if(fgets(line,300,pErpCodeFile)) 
			{
				ErpCodeLine erpLine;
				if(isValidEpochLine(line, erpLine))
				{
					if(m_data.find(erpLine.mjd) == m_data.end())
						m_data.insert(ErpCodeMap::value_type(erpLine.mjd, erpLine));
				}
			}
		}
		fclose(pErpCodeFile);
		return true;
	}

	bool erpCodeFile::add(string strErpCodeFile)
	{
		FILE * pErpCodeFile = fopen(strErpCodeFile.c_str(),"r+t");
		if(pErpCodeFile == NULL) 
			return false;
		char line[300]; 
		for(int i = 0; i < 6; i++) // 跳过文件头
			fgets(line,300,pErpCodeFile);
		while(!feof(pErpCodeFile))
		{
			if(fgets(line,300,pErpCodeFile)) 
			{
				ErpCodeLine erpLine;
				if(isValidEpochLine(line, erpLine))
				{
					if(m_data.find(erpLine.mjd) == m_data.end())
						m_data.insert(ErpCodeMap::value_type(erpLine.mjd, erpLine));
				}
			}
		}
		fclose(pErpCodeFile);
		return true;
	}

	bool erpCodeFile::getErpLine(UTC t, ErpCodeLine& erpLine)
	{
		double mjd_t = TimeCoordConvert::DayTime2MJD(t);
        double mjd_L = floor(mjd_t + 0.5) - 0.5;
		double mjd_R = ceil(mjd_t + 0.5) - 0.5;
		ErpCodeMap::const_iterator it_L =  m_data.find(mjd_L);
		ErpCodeMap::const_iterator it_R =  m_data.find(mjd_R);
		if(it_L != m_data.end() && it_R != m_data.end())
		{
			erpLine.mjd = mjd_t;
            erpLine.xp =  (mjd_t - mjd_L) * it_R->second.xp + (mjd_R - mjd_t) * it_L->second.xp;
			erpLine.yp =  (mjd_t - mjd_L) * it_R->second.yp + (mjd_R - mjd_t) * it_L->second.yp;
			// 克服跨年跳秒的情形
			if(fabs(it_R->second.ut1_utc - it_L->second.ut1_utc) > 0.5E+7 && (mjd_t - mjd_L) < 0.5)
				erpLine.ut1_utc =  (mjd_t - mjd_L) * (it_R->second.ut1_utc - 1.0E+7) + (mjd_R - mjd_t) * it_L->second.ut1_utc;   // 扣除跳秒影响
			else if(fabs(it_R->second.ut1_utc - it_L->second.ut1_utc) > 0.5E+7 && (mjd_t - mjd_L) >= 0.5)
				erpLine.ut1_utc =  (mjd_t - mjd_L) * (it_R->second.ut1_utc) + (mjd_R - mjd_t) * (it_L->second.ut1_utc + 1.0E+7); // 扣除跳秒影响
			else 
				erpLine.ut1_utc =  (mjd_t - mjd_L) * it_R->second.ut1_utc + (mjd_R - mjd_t) * it_L->second.ut1_utc;
			erpLine.s_xp =  it_L->second.s_xp;
			erpLine.s_yp =  it_L->second.s_yp;
			erpLine.s_ut1_utc =  it_L->second.s_ut1_utc;
			return true;
		}
		else
			return false;
	}

	void erpCodeFile::toEopRapidFile(string strEopRapidFile)
	{
		FILE* pFile = fopen(strEopRapidFile.c_str(), "w+");
		for(ErpCodeMap::iterator it = m_data.begin(); it != m_data.end(); ++it)
		{
			UTC t =  TimeCoordConvert::JD2DayTime(it->first + 0.5 + 2400000.5);
			ErpCodeLine erpLine;
			if(getErpLine(t, erpLine))
			{
				fprintf(pFile, "%1d%1d%2d%2d%9.2lf I%10.6lf%9.6lf%10.6lf%9.6lf  I%10.7lf%10.7lf%-107s\n", 
					            getIntBit(t.year, 1),
								getIntBit(t.year, 0),
								t.month,
								t.day,
								TimeCoordConvert::DayTime2MJD(t),
								erpLine.xp * 1.0E-6,
								erpLine.s_xp * 1.0E-6,
								erpLine.yp * 1.0E-6,
								erpLine.s_yp * 1.0E-6,
								erpLine.ut1_utc * 1.0E-7,
								erpLine.s_ut1_utc * 1.0E-7,
								" "); 
			}
		}
		fclose(pFile);
	}
}
