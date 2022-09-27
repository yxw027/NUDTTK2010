#include "TimePosVelFile.hpp"
#include <math.h>
#include "MathAlgorithm.hpp"
#include "EphemerisPosVelFile.hpp"
#include "TimeCoordConvert.hpp"

using namespace NUDTTK::Math;

namespace NUDTTK
{
	EphemerisPosVelFile::EphemerisPosVelFile(void)
	{
	}

	EphemerisPosVelFile::~EphemerisPosVelFile(void)
	{
	}

	bool EphemerisPosVelFile::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}

	bool EphemerisPosVelFile::isValidEpochLine(string strLine, EphemerisPosVel &timePosVel)
	{
		bool bFlag = true;
		//char szTime_yyyy[100];
		//char szTime_mm[100];
		//char szTime_dd[100];
		//char szTime_HH[100];
		//char szTime_MM[100];
		char szTime_SS[100];
		char  szX[100];
		char  szY[100];
		char  szZ[100];
		char szVx[100];
		char szVy[100];
		char szVz[100];
		//sscanf(strLine.c_str(),"%s%s%s%s%s%s%s%s%s%s%s%s",
		sscanf(strLine.c_str(),"%s%s%s%s%s%s%s",
							   szTime_SS,
							   szX,
							   szY,
							   szZ,
							   szVx,
							   szVy,
							   szVz);
		timePosVel.sec  = atof(szTime_SS);
		timePosVel.pos.x = atof(szX);
		timePosVel.pos.y = atof(szY);
		timePosVel.pos.z = atof(szZ);
		timePosVel.vel.x = atof(szVx);
		timePosVel.vel.y = atof(szVy);
		timePosVel.vel.z = atof(szVz);
		double r = sqrt(timePosVel.pos.x * timePosVel.pos.x 
			          + timePosVel.pos.y * timePosVel.pos.y
					  + timePosVel.pos.z * timePosVel.pos.z);
		//if(r <= 6000000 || r >= 100000000)
		//	bFlag = false;
		return bFlag;
	}

	bool EphemerisPosVelFile::open(string  strFileName)
	{
		FILE * pFile = fopen(strFileName.c_str(), "r+t");
		if(pFile == NULL) 
			return false;
		char line[400];
		//Ìø¹ýÇ°25ÐÐ
		for (int i = 0;i < 25;i++)
			fgets(line, 300, pFile);
		m_data.clear();
		while(!feof(pFile))
		{
			if(fgets(line, 400, pFile))
			{
				EphemerisPosVel timePosVel;
				if(isValidEpochLine(line, timePosVel))
					m_data.push_back(timePosVel);
			}
		}
		fclose(pFile);
		return true;
	}

	bool EphemerisPosVelFile::write(string strFileName)
	{
		FILE* pFile = fopen(strFileName.c_str(), "w+");
        for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			fprintf(pFile, "%16.6f  %16.6f %16.6f %16.6f %16.6f %16.6f %16.6f\n", m_data[s_i].sec,
																			 m_data[s_i].pos.x,
																			 m_data[s_i].pos.y,
																			 m_data[s_i].pos.z,
																			 m_data[s_i].vel.x,
																			 m_data[s_i].vel.y,
																			 m_data[s_i].vel.z);
		}
		fclose(pFile);
		return true;
	}
}