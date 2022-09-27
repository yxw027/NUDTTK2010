#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>

namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		#pragma pack(1)
		struct SCA1BRecordfo
		{
			int           gps_time;
			char          grace_id;
            int           sca_id;
			double        quatangle;
			double        quaticoeff;
			double        quatjcoeff;
			double        quatkcoeff;
			double        qual_rss;
			unsigned char qualflg;

			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0);
				return t0 + gps_time;
			};
		};
		#pragma pack()

		class gracefoSCA1BFile
		{
		public:
			gracefoSCA1BFile(void);
		public:
			~gracefoSCA1BFile(void);
		public:
			bool open(string strSCA1BfileName);
			bool exportTimeAttitudeFile(string  strTimeAttitudeFileName);
		public:
			vector<SCA1BRecordfo> m_data;
		};
	}
}
