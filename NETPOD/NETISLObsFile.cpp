#include "NETISLObsFile.hpp"

namespace NUDTTK
{
	namespace NETPOD
	{
		NETISLObsFile::NETISLObsFile(void)
		{
		}

		NETISLObsFile::~NETISLObsFile(void)
		{
		}

		bool NETISLObsFile::write(string strNETISLObsFileName)
		{
			FILE* pFile = fopen(strNETISLObsFileName.c_str(), "w+");
			for(size_t s_i = 0; s_i < m_data.size(); s_i++)
			{		
				fprintf(pFile, "%s %8s %8s %30.10f %30.10f %30.10f %30.10f %30.10f\n", 
					           m_data[s_i].t.toString().c_str(),
							   m_data[s_i].sat_A.c_str(),
							   m_data[s_i].sat_B.c_str(),
							   m_data[s_i].obs_AB,
							   m_data[s_i].obs_BA,
							   m_data[s_i].d,
							   m_data[s_i].clk_A,
							   m_data[s_i].clk_B);
			}
			fclose(pFile);
			return true;
		}
        
		// 读取文件数据 
		bool NETISLObsFile::open(string strNETISLObsFileName)
		{
			bool bflag = true;
			FILE* pFile = fopen(strNETISLObsFileName.c_str(),"r+t");
			if (pFile == NULL)
				return false;
			m_data.clear();
			char line[500];
			while(!feof(pFile))
			{
				NETISLObsLine obsLine; 
				if(fgets(line,500,pFile))
				{
					char satA[100];
					char satB[100];
					char year[100];
					char month[100];
					char day[100];
					char hour[100];
					char minute[100];
					char second[100];
					char obs_AB[100];
					char obs_BA[100];
					char d[100];
					char clk_A[100];
					char clk_B[100];

					sscanf(line,"%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
								 year,
								 month,
								 day,
								 hour,
								 minute,
								 second,
								 satA,
								 satB,
								 obs_AB,
								 obs_BA,
								 d,
								 clk_A,
								 clk_B);

					obsLine.sat_A     = satA;
					obsLine.sat_B     = satB;
					obsLine.t.year       = atoi(year);
					obsLine.t.month      = atoi(month);
					obsLine.t.day        = atoi(day);
					obsLine.t.hour       = atoi(hour);
					obsLine.t.minute     = atoi(minute);
					obsLine.t.second     = atof(second);
					obsLine.obs_AB     = atof(obs_AB);
					obsLine.obs_BA    = atof(obs_BA);
					obsLine.d    = atof(d);
					obsLine.clk_A = atof(clk_A);
					obsLine.clk_B = atof(clk_B);

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
					m_data.push_back(obsLine); 
				}
			}
			fclose(pFile);
			return bflag;
		}
	}
}
