#include "DorisSTCDFile.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	namespace DORIS
	{
		DorisSTCDFile::DorisSTCDFile(void)
		{
		}

		DorisSTCDFile::~DorisSTCDFile(void)
		{
		}

		bool DorisSTCDFile::readLine(string strLine, DORIS_COORDINATE_SERIES& line)
		{
			if(strLine.length() < 91)
				return false;
            // FIELDS - modified julian date, dX, dY, dZ, sX, sY, sZ, dEast, dNorth, dUp, sEast, sNorth, sUp
            // FORMAT - 2x,f7.1,2(2x,3(1x,f6.1),3(1x,f5.1))
			DORIS_COORDINATE_SERIES coordinateline;
			coordinateline.mjd = 0.0;
			sscanf(strLine.c_str(),"%*2c%7f%*2c%*1c%6f%*1c%6f%*1c%6f%*1c%5f%*1c%5f%*1c%5f%*2c%*1c%6f%*1c%6f%*1c%6f%*1c%5f%*1c%5f%*1c%5f",
								   &coordinateline.mjd,
								   &coordinateline.X,
								   &coordinateline.Y,
								   &coordinateline.Z,
								   &coordinateline.sigma_X,
								   &coordinateline.sigma_Y,
								   &coordinateline.sigma_Z,
								   &coordinateline.E,
								   &coordinateline.N,
								   &coordinateline.U,
								   &coordinateline.sigma_E,
								   &coordinateline.sigma_N,
								   &coordinateline.sigma_U);
			if(coordinateline.mjd <= 40000.0)
				return false;
			line = coordinateline;
			return true;
		}

		bool DorisSTCDFile::open(string  strDorisSTCDFileName)
		{
			FILE * pSTCDfile = fopen(strDorisSTCDFileName.c_str(), "r+t");
			if(pSTCDfile == NULL) 
				return false;
            char line[200];
			string strLine;
			// 第 1-19 行
			for(int i = 1; i <= 19; i++)
			{
				fgets(line, 200, pSTCDfile);
			}
			// 第20行 - 75个字符
			fgets(line, 200, pSTCDfile);
            sscanf(line, "%*1c%4c%*1c%2c%*1c%9c%*1c%1c%*1c%22c%*1c%3d%*1c%2d%*1c%4f%*1c%3d%*1c%2d%*1c%4f%*1c%7f",
				          m_siteID.site_code,
				          m_siteID.point_code,
				          m_siteID.DOMES_number,
				          m_siteID.observation_code,
				          m_siteID.station_description,
				         &m_siteID.longitude_degrees,
				         &m_siteID.longitude_minutes,
				         &m_siteID.longitude_seconds,
				         &m_siteID.latitude_degrees,
				         &m_siteID.latitude_minutes,
				         &m_siteID.latitude_seconds,
				         &m_siteID.height);
            // 第 21-24 行
			for(int i = 21; i <= 24; i++)
			{
				fgets(line, 200, pSTCDfile);
			}
			// 第 25 行 - 80个字符
			fgets(line, 200, pSTCDfile);
			sscanf(line, "%*1c%5d%*1c%6c%*1c%4c%*1c%2c%*1c%4c%*1c%2d%*1c%3d%*1c%5d%*1c%4c%*1c%1c%*1c%21le%*1c%11le",
			              &m_solutionApriori[0].parameter_index,
					       m_solutionApriori[0].parameter_type,
					       m_solutionApriori[0].site_code,
					       m_solutionApriori[0].point_code,
					       m_solutionApriori[0].solution_id,
					      &m_solutionApriori[0].time_year,
						  &m_solutionApriori[0].time_day,
						  &m_solutionApriori[0].time_second,
					       m_solutionApriori[0].parameter_units,
					       m_solutionApriori[0].constraint_code,
					      &m_solutionApriori[0].parameter_value,
					      &m_solutionApriori[0].standard_deviation);
			// 第 26 行 - 80个字符
			fgets(line, 200, pSTCDfile);
			sscanf(line, "%*1c%5d%*1c%6c%*1c%4c%*1c%2c%*1c%4c%*1c%2d%*1c%3d%*1c%5d%*1c%4c%*1c%1c%*1c%21le%*1c%11le",
			              &m_solutionApriori[1].parameter_index,
					       m_solutionApriori[1].parameter_type,
					       m_solutionApriori[1].site_code,
					       m_solutionApriori[1].point_code,
					       m_solutionApriori[1].solution_id,
					      &m_solutionApriori[1].time_year,
						  &m_solutionApriori[1].time_day,
						  &m_solutionApriori[1].time_second,
					       m_solutionApriori[1].parameter_units,
					       m_solutionApriori[1].constraint_code,
					      &m_solutionApriori[1].parameter_value,
					      &m_solutionApriori[1].standard_deviation);
			// 第 27 行 - 80个字符
			fgets(line, 200, pSTCDfile);
			sscanf(line, "%*1c%5d%*1c%6c%*1c%4c%*1c%2c%*1c%4c%*1c%2d%*1c%3d%*1c%5d%*1c%4c%*1c%1c%*1c%21le%*1c%11le",
			              &m_solutionApriori[2].parameter_index,
					       m_solutionApriori[2].parameter_type,
					       m_solutionApriori[2].site_code,
					       m_solutionApriori[2].point_code,
					       m_solutionApriori[2].solution_id,
					      &m_solutionApriori[2].time_year,
						  &m_solutionApriori[2].time_day,
						  &m_solutionApriori[2].time_second,
					       m_solutionApriori[2].parameter_units,
					       m_solutionApriori[2].constraint_code,
					      &m_solutionApriori[2].parameter_value,
					      &m_solutionApriori[2].standard_deviation);
			// 第 28 行 
			fgets(line, 200, pSTCDfile);
			// 第 29 行 
			fgets(line, 200, pSTCDfile);
            // 从第 30 行开始
			int k = 0;
			m_data.clear();
			while(!feof(pSTCDfile))
			{
				if(fgets(line, 200, pSTCDfile))  
				{
					k++;
					strLine = line;
					DORIS_COORDINATE_SERIES coordinate_series;
					if(readLine(strLine, coordinate_series))
					{
						m_data.push_back(coordinate_series);
					}
				}
			}
			fclose(pSTCDfile);
			return true;
		}

		bool DorisSTCDFile::getPos(TAI t, double &x, double &y, double &z)
		{
			x = m_solutionApriori[0].parameter_value;
			y = m_solutionApriori[1].parameter_value;
			z = m_solutionApriori[2].parameter_value;
			if(m_data.size() <= 0)
			{
				return false;
			}
			double mjd = TimeCoordConvert::DayTime2MJD(t);
			if(mjd < m_data[0].mjd)
			{
				x +=  m_data[0].X * 0.001;
				y +=  m_data[0].Y * 0.001;
				z +=  m_data[0].Z * 0.001;
				return true;
			}
			// 查找当前 mjd 所在的区间
			for(int i = 0; i < int(m_data.size()) - 1; i++)
			{
				if(mjd >= m_data[i].mjd && mjd < m_data[i + 1].mjd)
				{
					double u = (mjd - m_data[i].mjd) / (m_data[i + 1].mjd - m_data[i].mjd);
				    x += (1 - u) * m_data[i].X * 0.001 + u * m_data[i + 1].X * 0.001;
					y += (1 - u) * m_data[i].Y * 0.001 + u * m_data[i + 1].Y * 0.001;
					z += (1 - u) * m_data[i].Z * 0.001 + u * m_data[i + 1].Z * 0.001;
					return true;
				}
			}
			if(mjd > m_data[m_data.size() - 1].mjd)
			{
				x +=  m_data[m_data.size() - 1].X * 0.001;
				y +=  m_data[m_data.size() - 1].Y * 0.001;
				z +=  m_data[m_data.size() - 1].Z * 0.001;
                return true;
			}
			return true;
		}
	}
}
