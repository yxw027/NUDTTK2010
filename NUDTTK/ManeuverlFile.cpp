#include "ManeuverFile.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	ManeuverFile::ManeuverFile(void)
	{
	}

	ManeuverFile::~ManeuverFile(void)
	{
	}

	bool ManeuverFile::isValidEpochLine(string strLine, ManeuverLine& maneuverLine)
	{
		bool bFlag = true;
		char szTime_yyyy[5];
		char szTime_mm[5];
		char szTime_dd[5];
		char szTime_HH[5];
		char szTime_MM[5];
		char szTime_SS[11];
		char szDuration[11];
		char szDirection_xa[5];
		char szDirection_ya[5];
		char szDirection_xb[5];
		char szThruster_id[5];
		char szThrust_size[11];
		char szDelta_v[11];
		sscanf(strLine.c_str(),"%s%s%s%s%s%s%s%s%s%s%s%s%s",
							   szTime_yyyy,
							   szTime_mm,
							   szTime_dd,
							   szTime_HH,
							   szTime_MM,
							   szTime_SS,
							   szDuration,
							   szDirection_xa,
							   szDirection_ya,
							   szDirection_xb,
							   szThruster_id,
							   szThrust_size,
							   szDelta_v);
		maneuverLine.t0.year    = atoi(szTime_yyyy);//ÕÅºñ†´ÐÞ¸Ä 2019 10 24
		if(maneuverLine.t0.year > 2500 || maneuverLine.t0.year < 1900)
		{
			bFlag = false;
			return bFlag;
		}
		maneuverLine.t0.month   = atoi(szTime_mm);
		if(maneuverLine.t0.month > 12 || maneuverLine.t0.month < 1)
		{
			bFlag = false;
			return bFlag;
		}
		maneuverLine.t0.day     = atoi(szTime_dd);
		if(maneuverLine.t0.day > 31 || maneuverLine.t0.day < 0)
		{
			bFlag = false;
			return bFlag;
		}
		maneuverLine.t0.hour    = atoi(szTime_HH);
		if(maneuverLine.t0.hour > 24 || maneuverLine.t0.hour < 0)
		{
			bFlag = false;
			return bFlag;
		}
		maneuverLine.t0.minute  = atoi(szTime_MM);
		if(maneuverLine.t0.minute > 60 || maneuverLine.t0.minute < 0)
		{
			bFlag = false;
			return bFlag;
		}
		maneuverLine.t0.second  = atof(szTime_SS);
		if(maneuverLine.t0.second > 60 || maneuverLine.t0.second < 0)
		{
			bFlag = false;
			return bFlag;
		}
		maneuverLine.duration   = atof(szDuration);
		if(maneuverLine.duration >= 86400 || maneuverLine.duration < 0)
		{
			bFlag = false;
			return bFlag;
		}
		//maneuverLine.t0.year    = atoi(szTime_yyyy);
		//maneuverLine.t0.month   = atoi(szTime_mm);
		//maneuverLine.t0.day     = atoi(szTime_dd);
		//maneuverLine.t0.hour    = atoi(szTime_HH);
		//maneuverLine.t0.minute  = atoi(szTime_MM);
		//maneuverLine.t0.second  = atof(szTime_SS);
		//maneuverLine.duration   = atof(szDuration);
		maneuverLine.direction_x  = atoi(szDirection_xa);
		maneuverLine.direction_y  = atoi(szDirection_ya);
		maneuverLine.direction_z  = atoi(szDirection_xb);
		maneuverLine.thruster_id  = atoi(szThruster_id);
		maneuverLine.thrust_size  = atof(szThrust_size);
		maneuverLine.delta_v     = atof(szDelta_v);
		maneuverLine.t1 = maneuverLine.t0 + maneuverLine.duration;
		return bFlag;
	}

	bool ManeuverFile::open(string  strFileName)
	{
		FILE * pFile = fopen(strFileName.c_str(), "r+t");
		if(pFile == NULL) 
			return false;
		char line[200];
		m_data.clear();
		while(!feof(pFile))
		{
			if(fgets(line, 200, pFile))
			{
				ManeuverLine maneuverLine;
				if(isValidEpochLine(line, maneuverLine))
					m_data.push_back(maneuverLine);
			}
		}
		fclose(pFile);
		return true;
	}

	bool ManeuverFile::write(string strFileName)
	{
		FILE* pFile = fopen(strFileName.c_str(), "w+");
        for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			/*fprintf(pFile, "%s  %8.4f  %2d  %2d  %2d  %2d  %8.4f  %8.4f\n", m_data[s_i].t0.toString().c_str(), m_data[s_i].duration, 
				            m_data[s_i].direction_x, m_data[s_i].direction_y, m_data[s_i].direction_z,
							m_data[s_i].thruster_id, m_data[s_i].thrust_size, m_data[s_i].delta_v);*/
			fprintf(pFile, "%s  %8.4f  %2d  %2d  %2d  %2d \n", m_data[s_i].t0.toString().c_str(), m_data[s_i].duration, 
				            m_data[s_i].direction_x, m_data[s_i].direction_y, m_data[s_i].direction_z,
							m_data[s_i].thruster_id);
		}
		fclose(pFile);
		return true;
	}
}