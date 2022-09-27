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
		struct CLK1BRecord
		{// 47¸ö×Ö½Ú
			int           gps_time;
			char          grace_id;
            char          clock_id;
			double        eps_time;
			double        eps_err;
			double        eps_drift;
			double        drift_err;
			unsigned char qualflg;
			/*
				Quality flags that indicate data gaps filled, according to severity:
					qualflg bit 0=1 filled data at T
					qualflg bit 1=1 filled data at T +/- 1 second
					qualflg bit 2=1 filled data at T +/- 2 seconds
				Additional quality flags include:
					qualflg bit 3=1 only one star camera enabled
					qualflg bit 4=1 extrapolated clock correction used
					qualflg bit 6=1 low rate data from 2nd star camera
					qualflg bit 7=1 low rate data from 1st star camera
			*/

			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0);
				return t0 + gps_time;
			};
		};
		#pragma pack()

		class graceCLK1BFile
		{
		public:
			graceCLK1BFile(void);
		public:
			~graceCLK1BFile(void);
		public:
			bool open(string strCLK1BfileName);
			bool open_gracefo(string strCLK1BfileName);
			bool exportTimeCLKFile(string  strTimeCLKFileName);
		    bool exportCLKFile(string strCLKFileName);
		public:
			vector<CLK1BRecord> m_data;
		};
	}
}
