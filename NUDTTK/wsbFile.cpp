#include "wsbFile.hpp"

namespace NUDTTK
{
	wsbFile::wsbFile(void)
	{
	}
	wsbFile::~wsbFile(void)
	{
	}

	bool wsbFile::isValidEpochLine(string strline, wsbFileLine &wsbline)
	{
		bool bFlag = true;
		wsbFileLine Line;
		//判断是否为注释行
		char szHead[2];
		char year[5];
		char month[4];
		char day[4];
		sscanf(strline.c_str(),"%1c",szHead);
		szHead[1] = '\0';          //添加结束符
		if(strcmp(szHead,"#") == 0)
			return false;
		sscanf(strline.c_str(),"%*1c%4c%3c%3c%*3c%*1c%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf%7lf",
							    year, month, day, &Line.wsb[0], &Line.wsb[1], &Line.wsb[2],&Line.wsb[3], &Line.wsb[4],
								&Line.wsb[5], &Line.wsb[6], &Line.wsb[7], &Line.wsb[8], &Line.wsb[9], &Line.wsb[10],
								&Line.wsb[11], &Line.wsb[12],&Line.wsb[13], &Line.wsb[14], &Line.wsb[15], &Line.wsb[16], 
								&Line.wsb[17], &Line.wsb[18],&Line.wsb[19], &Line.wsb[20], &Line.wsb[21], &Line.wsb[22],
								&Line.wsb[23], &Line.wsb[24],&Line.wsb[25], &Line.wsb[26], &Line.wsb[27], &Line.wsb[28], 
								&Line.wsb[29], &Line.wsb[30],&Line.wsb[31], &Line.wsb[32], &Line.wsb[33], &Line.wsb[34],
								&Line.wsb[35], &Line.wsb[36],&Line.wsb[37], &Line.wsb[38], &Line.wsb[39]);

		year[4] = '\0';
		month[3]  = '\0';
		day[3]  = '\0';
		Line.year = atoi(year);
		Line.month  = atoi(month);
		Line.day  = atoi(day);
		if(bFlag)
			wsbline = Line;
		return bFlag;
	}

	bool wsbFile::open(string strWSBFileName)
	{
		bool bFlag = true;
		FILE *pFile = fopen(strWSBFileName.c_str(),"r+t");
		if(pFile == NULL)
			return false;
		char Line[300];
		//跳过前5行
		for(int i = 0; i < 5; i++)
		  fgets(Line, 300, pFile);
		m_data.clear();        //清空容器
		while(!feof(pFile))
		{
		  if(fgets(Line, 300, pFile))
		  {
	  		wsbFileLine wsbLine;
	  		if(isValidEpochLine(Line, wsbLine))
			{
	  			m_data.push_back(wsbLine);
			}
		  }
		}
		fclose(pFile);
		return true;
	}

	bool wsbFile::getWSB(DayTime t, wsbFileLine &wsbline)
	{
		bool bFind = false;
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			if(t.year == m_data[s_i].year && t.month == m_data[s_i].month && t.day == m_data[s_i].day)
			{
				bFind = true;
				wsbline = m_data[s_i];
				break;
			}
		}
  		return bFind;
	}
}
