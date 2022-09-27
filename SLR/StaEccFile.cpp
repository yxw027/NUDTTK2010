#include "StaEccFile.hpp"

namespace NUDTTK
{
	namespace SLR
	{
		StaEccFile::StaEccFile(void)
		{
		}

		StaEccFile::~StaEccFile(void)
		{
		}

		// 84:010:86399 -> 转换成标准时间格式
		UTC StaEccFile::getTime(int year, int day, int second)
		{
			int nB4Year = yearB2toB4(year);
			UTC t(nB4Year, 1, 1, 0, 0, 0);
			t = t + (day - 1) * 86400;    // 先算天的(nDay是否从 1 开始计数?)
			t = t + second;               // 再加上秒的部分
			return t;
		}

		bool StaEccFile::readEccLine(string strLine, EccLine& line)
		{
			if(strLine.length() < 88)
				return false;
			EccLine eccline;
			// 起始和结束时间
			int nYear_Start;
			int nDay_Start;
			int nSecond_Start;
			int nYear_End;
			int nDay_End;
			int nSecond_End;
			sscanf(strLine.c_str(),"%4d%*1c%2c%*1c%4c%*1c%1c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%3c%*1c%8lf%*1c%8lf%*1c%8lf%*1c%*7c%8c",
								   &eccline.id,
									eccline.szPT,
									eccline.szSoln,
									eccline.szT,
								   &nYear_Start,
								   &nDay_Start,
								   &nSecond_Start,
								   &nYear_End,
								   &nDay_End,
								   &nSecond_End,
									eccline.szUNE,
								   &eccline.ecc.U,
								   &eccline.ecc.N,
								   &eccline.ecc.E,
								   eccline.szCDP);
			eccline.szPT[2]   = '\0';
			eccline.szSoln[4] = '\0';
			eccline.szT[1]    = '\0';
			eccline.szUNE[3]  = '\0';
			eccline.szCDP[8]  = '\0';
			if(strcmp(eccline.szUNE,"UNE") != 0)
				return false;
			eccline.t0 = getTime(nYear_Start, nDay_Start, nSecond_Start);
			if(nYear_End == 0 && nDay_End == 0 && nSecond_End == 0)
			{// 人为的指定一个上界
				eccline.t1 = UTC(2030, 1, 1, 0, 0, 0);
			}
			else
				eccline.t1 = getTime(nYear_End, nDay_End, nSecond_End);
			line = eccline;
			return true;
		}

		bool StaEccFile::open(string strEccFileName)
		{
			FILE * pEccFile = fopen(strEccFileName.c_str(),"r+t");
			if(pEccFile == NULL) 
				return false;
			char szline[200];     
			m_data.clear();
			// 越过先前的 1 行
			fgets(szline, 200, pEccFile);
			StaEccEnsemble newEcc;
			newEcc.id = -1;
			newEcc.eccLineList.clear();
			while(!feof(pEccFile))
			{
				if(fgets(szline, 200, pEccFile))  
				{
					EccLine Line;
					if(readEccLine(szline, Line))
					{
						if(Line.id != newEcc.id)
						{// 找到新的测站
							if(newEcc.eccLineList.size() > 0)
								m_data.push_back(newEcc);
							newEcc.eccLineList.clear();
							newEcc.id = Line.id;
							newEcc.eccLineList.push_back(Line);
						}
						else
						{
							newEcc.eccLineList.push_back(Line);
						}
					}
				}
			}
			// 2008/11/08
			if(newEcc.eccLineList.size() > 0)
				m_data.push_back(newEcc);
			fclose(pEccFile);
			return true;
		}

		// 导出激光测站偏心数据列表, 用于外部使用, 2008/04/11
		bool StaEccFile::getStaEccRecordList(vector<StaEccRecord> &staEccList)
		{
			int count = int(m_data.size());
			if(count <= 0)
				return false;
			staEccList.clear();
			for(int s_i =  0; s_i < count; s_i++)
			{
				for(size_t s_j = 0; s_j < m_data[s_i].eccLineList.size(); s_j++)
				{
					StaEccRecord datum;
					datum.id  = m_data[s_i].id;
					datum.t0  = m_data[s_i].eccLineList[s_j].t0;
					datum.t1  = m_data[s_i].eccLineList[s_j].t1;
					datum.ecc = m_data[s_i].eccLineList[s_j].ecc;
					staEccList.push_back(datum);
				}
			}
			return true;
		}
	}
}
