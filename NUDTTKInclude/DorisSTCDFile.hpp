#pragma once
#include "structDef.hpp"
#include <vector>

namespace NUDTTK
{
	namespace DORIS
	{
		struct DORIS_SITE_ID
		{
			char  site_code[5];
			char  point_code[3];
			char  DOMES_number[12];
			char  observation_code[2];
			char  station_description[22];
			int   longitude_degrees;
			int   longitude_minutes;
			float longitude_seconds;
			int   latitude_degrees;
			int   latitude_minutes;
			float latitude_seconds;
			float height;
		};
		struct DORIS_SOLUTION_APRIORI
		{
			int    parameter_index;
			char   parameter_type[6 + 1];
			char   site_code[4 + 1];
			char   point_code[2 + 1];
			char   solution_id[4 + 1];
			int    time_year;
			int    time_day;
			int    time_second;
			char   parameter_units[4 + 1];
			char   constraint_code[1 + 1];
			double parameter_value;
			double standard_deviation;
		};

		struct DORIS_COORDINATE_SERIES
		{
			float mjd;
			float X;
			float Y;
			float Z;
			float sigma_X;
			float sigma_Y;
			float sigma_Z;
			float E;
			float N;
			float U;
			float sigma_E;
			float sigma_N;
			float sigma_U;
		};

		class DorisSTCDFile
		{
		public:
			DorisSTCDFile(void);
		public:
			~DorisSTCDFile(void);
		public:
			bool readLine(string strLine, DORIS_COORDINATE_SERIES& line);
			bool open(string  strDorisSTCDFileName);
			bool getPos(TAI t, double &x, double &y, double &z);
		public:
			DORIS_SITE_ID                   m_siteID;
			DORIS_SOLUTION_APRIORI          m_solutionApriori[3];
			vector<DORIS_COORDINATE_SERIES> m_data;
		};
	}
}
