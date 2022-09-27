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
		struct THR1BRecord
		{
			int           time_intg;		// 4
			int           time_frac;		// 4
			char		  time_ref;			// 1
			char          GRACE_id;			// 1
			unsigned int  thrust_count[14];	// 4 * 14
			unsigned int  on_time[14];	    // 4 * 14
			unsigned int  accumk_dur[14];	// 4 * 14
			unsigned char qualflg;			// 1
			
			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0);
				return t0 + (time_intg + time_frac * 1.0E-6);
			};
		};
		#pragma pack()

		class graceTHR1BFile
		{
		public:
			graceTHR1BFile(void);
		public:
			~graceTHR1BFile(void);
		public:
			bool open(string strTHR1BfileName);
			bool exportThrustFile(string strThrustfileName);
		public:
			vector<THR1BRecord> m_data;
		};
	}
}
