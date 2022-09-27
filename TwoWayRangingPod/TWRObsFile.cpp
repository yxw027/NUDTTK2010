#include "TWRObsFile.hpp"

namespace NUDTTK
{
	namespace TwoWayRangingPod
	{
		TWRObsFile::TWRObsFile(void)
		{
		}

		TWRObsFile::~TWRObsFile(void)
		{
		}

		bool TWRObsFile::isValidEpochLine(string strLine, TWRObsLine &obsLine)
		{
			bool bflag = true;
			char szTime_yyyy[100];
			char szTime_mm[100];
			char szTime_dd[100];
			char szTime_sec[100]; // 一天内秒数
			char szR[100];
			char szR0s[100];
			char szR1[100];
			char szR0g[100];
			char szR01g[100];
			sscanf(strLine.c_str(),"%*s%*s%*s%s%s%s%s%*s%*s%*s%*s%s%s%s%s%s%*s%*s%*s%*s%*s",
								   szTime_yyyy,
								   szTime_mm,
								   szTime_dd,
								   szTime_sec,
								   szR,
								   szR0s,
								   szR1,
								   szR0g,
								   szR01g);
			obsLine.t.year    = atoi(szTime_yyyy);
			obsLine.t.month   = atoi(szTime_mm);
			obsLine.t.day     = atoi(szTime_dd);
			double t_sec      = atof(szTime_sec);
			obsLine.t.hour    = int(floor(t_sec/3600.0));
			obsLine.t.minute  = int(floor((t_sec - obsLine.t.hour*3600.0)/60.0));
			obsLine.t.second  = t_sec - obsLine.t.hour*3600.0 - obsLine.t.minute*60.0;
			obsLine.R         = atof(szR);
			obsLine.R0s       = atof(szR0s);
			obsLine.R1        = atof(szR1);
			obsLine.R0g       = atof(szR0g);
			obsLine.R01g      = atoi(szR01g);
			if(obsLine.t.year == 0)
				bflag = false;
			if(obsLine.t.month  > 12 || obsLine.t.month  <= 0)
				bflag = false;
			if(obsLine.t.day    > 31 || obsLine.t.day    < 0)
				bflag = false;
			if(obsLine.t.hour   > 24 || obsLine.t.hour   < 0)
				bflag = false;
			if(obsLine.t.minute > 60 || obsLine.t.minute < 0)
				bflag = false;
			if(obsLine.t.second > 60 || obsLine.t.second < 0)
				bflag = false;
			return bflag;		
		}

		bool TWRObsFile::open(string strFileName)
		{
			FILE * pFile = fopen(strFileName.c_str(), "r+t");
			if (pFile == NULL)
				return false;
			char line[500];
			//跳过一行
			fgets(line, 500, pFile);
			m_data.clear();           //清空容器
			while(!feof(pFile))
			{
				if(fgets(line, 500, pFile))
				{
					TWRObsLine obsLine;
					if (isValidEpochLine(line, obsLine))
						m_data.push_back(obsLine);
				}
			}
			fclose(pFile);
			return true;
		}	
	}
}
