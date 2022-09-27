#include "svnavMixedFile.hpp"

namespace NUDTTK
{
	svnavMixedFile::svnavMixedFile(void)
	{
	}

	svnavMixedFile::~svnavMixedFile(void)
	{
	}

	void svnavMixedFile::clear()
	{
		m_data.clear();
	}

	DayTime SvNavMixedLine::doy2daytime(int year, int doy, int hour, int minute)
	{
		DayTime t(year, 1, 1, hour, minute, 0.0);
		t = t + (doy - 1) * 86400.0;
		return t;
	}

    //判断该行是否有效
	bool svnavMixedFile::isValidEpochLine(string strLine, SvNavMixedLine &mixedLine)
	{
		bool bFlag = true;
		SvNavMixedLine Line;
		char cHead;
		sscanf(strLine.c_str(), "%1c", &cHead);
		if (cHead == '#')
			return false;
		sscanf(strLine.c_str(), "%*1c%1c%*3c%3d%*2c%2d%*2c%2d%*2c%20c%10d%*6c%1c%*2c%10f%*2c%4d%*c%3d%*c%2d%*c%2d%*2c%4d%*c%3d%*c%2d%*c%2d",
			                   &Line.cSys,
							   &Line.id_SVN,
							   &Line.id_PRN,
							   &Line.id_CHAN,
							    Line.szBlock,
							   &Line.mass,
							   &Line.yawBiasFlag,
							   &Line.yawRate,
							   &Line.y0, &Line.d0, &Line.h0, &Line.m0,
							   &Line.y1, &Line.d1, &Line.h1, &Line.m1);
		Line.szBlock[20] = '\0';
		if(Line.cSys != 'G' && Line.cSys != 'C' && Line.cSys != 'R' && Line.cSys != 'E')
			bFlag = false;
		if(Line.id_PRN == 0)
			bFlag = false;
		if(Line.mass == 0)
			bFlag = false;
		if(Line.y0 == 0 || Line.y1 == 0)
			bFlag = false;

		if(bFlag)
		{
			Line.t0 = Line.doy2daytime(Line.y0, Line.d0, Line.h0, Line.m0);
			Line.t1 = Line.doy2daytime(Line.y1, Line.d1, Line.h1, Line.m1);
			mixedLine = Line;
		}
		return bFlag;
	}

    //打开文件
	bool svnavMixedFile::open(string strSvNavMixedFileName)
	{
		FILE * pFile = fopen(strSvNavMixedFileName.c_str(), "r+t");
		if(pFile == NULL) 
			return false;
		// 越过前两行
		char line[300];
		for(int i = 0; i < 2; i++)
			fgets(line, 300, pFile);
		m_data.clear();  //清空容器
		while(!feof(pFile))
		{
			if(fgets(line, 300, pFile))
			{
				SvNavMixedLine mixedLine;
				if(isValidEpochLine(line, mixedLine))
					m_data.push_back(mixedLine);
			}
		}
		fclose(pFile);
		return true;
	}
	//将文件写入文本中
	bool svnavMixedFile::write(string strSvNavMixedFileName)
	{
		//bool wFlag = true;
		FILE *pFile = fopen(strSvNavMixedFileName.c_str(), "w+");
		for (size_t i = 0; i < m_data.size(); i++)
		{
			fprintf(pFile, " %c   %3d  %2d  %2d  %20s%10d.     %c  %10.4f  %4d  %3d  %2d  %2d  %4d  %3d  %2d  %2d\n",
		                   m_data[i].cSys,
		                   m_data[i].id_SVN, 
						   m_data[i].id_PRN,
		                   m_data[i].id_CHAN,
						   m_data[i].szBlock,
						   m_data[i].mass,
						   m_data[i].yawBiasFlag,
						   m_data[i].yawRate,
						   m_data[i].y0, m_data[i].d0, m_data[i].h0, m_data[i].m0,
						   m_data[i].y1, m_data[i].d1, m_data[i].h1, m_data[i].m1);
		}
		fclose(pFile);
		return true;
	}

    //取值
	bool svnavMixedFile::getSvNavInfo(DayTime t, string satName, SvNavMixedLine& svNavMixedLine)
	{
		bool bFind = true;
		//定义一个容器来存放卫星名称符合的行
		vector<SvNavMixedLine> mixedLinelist_i;
		mixedLinelist_i.clear();
		char cSys;
		int  id_PRN;
		sscanf(satName.c_str(), "%c%02d", &cSys, &id_PRN);
		for (size_t i = 0; i < m_data.size(); i++)
		{
			if (cSys == m_data[i].cSys && id_PRN == m_data[i].id_PRN)
				mixedLinelist_i.push_back(m_data[i]);
		}
		if (mixedLinelist_i.size() == 0)
			return false;
		else
		{
			bFind = false;
			for (int i = int (mixedLinelist_i.size() - 1); i >= 0; i--)
			{

				if (t - mixedLinelist_i[i].t0 >= 0 && t - mixedLinelist_i[i].t1 < 0)
				{
					bFind = true;
					svNavMixedLine = mixedLinelist_i[i];
					break;
				}
			}
			return bFind;
		}
		return false;
	}
}
