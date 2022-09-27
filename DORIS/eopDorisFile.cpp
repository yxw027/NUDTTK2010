#include "eopDorisFile.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	namespace DORIS
	{
		eopDorisFile::eopDorisFile(void)
		{
		}

		eopDorisFile::~eopDorisFile(void)
		{
		}

		bool eopDorisFile::isValidEpochLine(string strLine,EopDorisLine& eopLine)
		{
			bool nFlag = true;
			if(strLine.length() < 50)
				return false;
			char szMjd[50];
			char szXp[50];
			char szYp[50];
			char szUt1_utc[50];
			char szLod[50];
			char szS_Xp[50];
			char szS_Yp[50];
			char szS_Ut1_utc[50];
			char szS_Lod[50];
			EopDorisLine Line; // EopDorisLine 的构造函数要有必要的初始化, 以保证不出现错误的判断, 因为在读到最后一行时, strLine的有效字符很短, line无法通过strLine来赋值
			sscanf(strLine.c_str(), "%s%s%s%s%s%s%s%s%s",
									szMjd,
									szXp,
									szYp,
									szUt1_utc,
                                    szLod,
									szS_Xp,
									szS_Yp,
									szS_Ut1_utc,
									szS_Lod);
            Line.mjd = atof(szMjd);
			Line.xp = atof(szXp);
			Line.yp = atof(szYp);
			Line.ut1_utc = atof(szUt1_utc);
			Line.lod =  atof(szLod);
			Line.s_xp = atof(szS_Xp);
			Line.s_yp = atof(szS_Yp);
			Line.s_ut1_utc = atof(szS_Ut1_utc);
			Line.s_lod =  atof(szS_Lod);
			if(Line.mjd <= 0.0)
			   nFlag = false;
			if(nFlag)
			   eopLine = Line;
			return nFlag;
		}

		bool eopDorisFile::open(string strEopDorisFile)
		{
			FILE * pEopDorisFile = fopen(strEopDorisFile.c_str(),"r+t");
			if(pEopDorisFile == NULL) 
				return false;
			m_data.clear();
			char line[300]; 
			for(int i = 0; i < 3; i++) // 跳过文件头
				fgets(line,300, pEopDorisFile);
			while(!feof(pEopDorisFile))
			{
				if(fgets(line,300,pEopDorisFile)) 
				{
					EopDorisLine eopLine;
					if(isValidEpochLine(line, eopLine))
					{
						if(m_data.find(eopLine.mjd) == m_data.end())
							m_data.insert(EopDorisMap::value_type(eopLine.mjd, eopLine));
					}
				}
			}
			fclose(pEopDorisFile);
			return true;
		}

		bool eopDorisFile::getEopLine(UTC t, EopDorisLine& eopLine)
		{
			double mjd_t = TimeCoordConvert::DayTime2MJD(t);
            double mjd_L = floor(mjd_t + 0.5) - 0.5;
			double mjd_R = ceil(mjd_t + 0.5) - 0.5;
			EopDorisMap::const_iterator it_L =  m_data.find(mjd_L);
			EopDorisMap::const_iterator it_R =  m_data.find(mjd_R);
			if(it_L != m_data.end() && it_R != m_data.end())
			{
				eopLine.mjd = mjd_t;
                eopLine.xp  =  (mjd_t - mjd_L) * it_R->second.xp  + (mjd_R - mjd_t) * it_L->second.xp;
				eopLine.yp  =  (mjd_t - mjd_L) * it_R->second.yp  + (mjd_R - mjd_t) * it_L->second.yp;
				eopLine.lod =  (mjd_t - mjd_L) * it_R->second.lod + (mjd_R - mjd_t) * it_L->second.lod;
				// 克服跨年跳秒的情形
				if(fabs(it_R->second.ut1_utc - it_L->second.ut1_utc) > 0.5E+7 && (mjd_t - mjd_L) < 0.5)
					eopLine.ut1_utc =  (mjd_t - mjd_L) * (it_R->second.ut1_utc - 1.0E+7) + (mjd_R - mjd_t) * it_L->second.ut1_utc;   // 扣除跳秒影响
				else if(fabs(it_R->second.ut1_utc - it_L->second.ut1_utc) > 0.5E+7 && (mjd_t - mjd_L) >= 0.5)
					eopLine.ut1_utc =  (mjd_t - mjd_L) * (it_R->second.ut1_utc) + (mjd_R - mjd_t) * (it_L->second.ut1_utc + 1.0E+7); // 扣除跳秒影响
				else 
					eopLine.ut1_utc =  (mjd_t - mjd_L) * it_R->second.ut1_utc + (mjd_R - mjd_t) * it_L->second.ut1_utc;
				eopLine.s_xp =  it_L->second.s_xp;
				eopLine.s_yp =  it_L->second.s_yp;
				eopLine.s_ut1_utc =  it_L->second.s_ut1_utc;
				eopLine.s_lod =  it_L->second.s_lod;
				return true;
			}
			else
				return false;
			return true;
		}

		void eopDorisFile::toEopRapidFile(string strEopRapidFile)
		{
			FILE* pFile = fopen(strEopRapidFile.c_str(), "w+");
			for(EopDorisMap::iterator it = m_data.begin(); it != m_data.end(); ++it)
			{
				UTC t =  TimeCoordConvert::JD2DayTime(it->first + 0.5 + 2400000.5);
				EopDorisLine eopLine;
				if(getEopLine(t, eopLine))
				{
					fprintf(pFile, "%1d%1d%2d%2d%9.2lf I%10.6lf%9.6lf%10.6lf%9.6lf  I%10.7lf%10.7lf%-107s\n", 
						            getIntBit(t.year, 1),
									getIntBit(t.year, 0),
									t.month,
									t.day,
									TimeCoordConvert::DayTime2MJD(t),
									eopLine.xp * 1.0E-6,
									eopLine.s_xp * 1.0E-6,
									eopLine.yp * 1.0E-6,
									eopLine.s_yp * 1.0E-6,
									eopLine.ut1_utc * 1.0E-7,
									eopLine.s_ut1_utc * 1.0E-7,
									" "); 
				}
			}
			fclose(pFile);
		}
	}
}
