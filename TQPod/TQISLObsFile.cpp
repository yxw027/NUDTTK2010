#include "TQISLObsFile.hpp"

namespace NUDTTK
{
	namespace TQPod
	{
		TQISLObsFile::TQISLObsFile(void)
		{
		}

		TQISLObsFile::~TQISLObsFile(void)
		{
		}

		bool TQISLObsFile::write(string strISLObsFileName)
		{
			FILE* pFile = fopen(strISLObsFileName.c_str(), "w+");
			for(size_t s_i = 0; s_i < m_data.size(); s_i++)
			{		
				fprintf(pFile, "%4s %4s  %s %30.10f%30.10f%30.10f%30.10f\n", 
							   m_data[s_i].satNameA.c_str(),
							   m_data[s_i].satNameB.c_str(),
					           m_data[s_i].t.toString().c_str(),
							   m_data[s_i].ISL_Code,
							   m_data[s_i].ISL_Phase,
							   m_data[s_i].dR_satpco,
							   m_data[s_i].dR_lightTime);
			}
			fclose(pFile);
			return true;
		}
        
		// 读取文件数据 
		bool TQISLObsFile::open(string strISLObsFileName)
		{
			bool bflag = true;
			FILE* pFile = fopen(strISLObsFileName.c_str(),"r+t");
			if (pFile == NULL)
				return false;
			m_data.clear();
			char line[500];
			while(!feof(pFile))
			{
				TQISLObsLine obsLine; 
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
					char ISL_Code[100];
					char ISL_Phase[100];
					char satpco[100];
					char lightTime[100];

					sscanf(line,"%s%s%s%s%s%s%s%s%s%s%s%s%s",
						         satA,
								 satB,
								 year,
								 month,
								 day,
								 hour,
								 minute,
								 second,
								 ISL_Code,
								 ISL_Phase,
								 satpco,
								 lightTime);

					obsLine.satNameA     = satA;
					obsLine.satNameB     = satB;
					obsLine.t.year       = atoi(year);
					obsLine.t.month      = atoi(month);
					obsLine.t.day        = atoi(day);
					obsLine.t.hour       = atoi(hour);
					obsLine.t.minute     = atoi(minute);
					obsLine.t.second     = atof(second);
					obsLine.ISL_Code     = atof(ISL_Code);
					obsLine.ISL_Phase    = atof(ISL_Phase);
					obsLine.dR_satpco    = atof(satpco);
					obsLine.dR_lightTime = atof(lightTime);

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
