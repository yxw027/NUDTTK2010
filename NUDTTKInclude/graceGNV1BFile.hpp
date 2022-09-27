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
		struct GNV1BRecord
		{// 103¸ö×Ö½Ú
			int           gps_time;
			char          grace_id;
			char          coord_ref;
			double        xpos;
			double        ypos;
			double        zpos;
			double        xpos_err;
			double        ypos_err;
			double        zpos_err;
			double        xvel;
			double        yvel;
			double        zvel;
			double        xvel_err;
			double        yvel_err;
			double        zvel_err;
			unsigned char qualflg;

			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0);
				return t0 + gps_time;
			};
		};
		#pragma pack()


		class graceGNV1BFile
		{
		public:
			graceGNV1BFile(void);
		public:
			~graceGNV1BFile(void);
		public:
			bool    open(string strGNV1BfileName);
			bool    open_GRACEfo(string strGNV1BfileName);
			bool    isEmpty();
			bool    cutdata(GPST T1, GPST T2);
			bool    cutdata();
			bool    append(string strGNV1BfileName);
			bool    getPosVel(GPST t, POS6D &posvel, int nlagrange = 8);
			bool	exportTimePosVelFile(string  strTimePosVelfileName);
			bool    orbComparision_XYZ(GPST t, TimePosVelClock posvel, POS6D& err_itrf);
			bool    orbComparision_RTN_ECF(GPST t,TimePosVelClock posvel,POS6D& error);
			bool    orbComparision_RTN_ECI(GPST t,TimePosVelClock posvel,POS6D& error);
		public:
			vector<GNV1BRecord> m_data;
		};
	}
}
