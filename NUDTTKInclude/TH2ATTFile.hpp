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
		struct ATTRecord
		{// 47¸ö×Ö½Ú
			int           gps_time;
			char          grace_id;
            char          sca_id;
			double        quatangle;
			double        quaticoeff;
			double        quatjcoeff;
			double        quatkcoeff;
			double        qual_rss;
			char          qualflg;
			GPST gettime()
			{
				GPST t0(2019, 1, 1, 12, 0, 0);
				return t0 + gps_time;
			};
		};
		#pragma pack()

		class TH2ATTFile
		{
		public:
			TH2ATTFile(void);
		public:
			~TH2ATTFile(void);
		public:
			bool open(string strATTfileName);
			bool exportTimeAttitudeFile(string  strTimeAttitudeFileName);
		public:
			vector<ATTRecord> m_data;
		};
	}
}
