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
		struct VGN1BRecord
		{// 38¸ö×Ö½Ú
			int           gps_time;
			char          grace_id;
			double        mag;
			double        cosx;
			double        cosy;
			double        cosz;
			unsigned char qualflg;

			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0);
				return t0 + gps_time;
			};

			POS3D getAntOffset()
			{
				POS3D offset;
				offset.x = mag * cosx;
				offset.y = mag * cosy;
				offset.z = mag * cosz;
				return offset;
			}
		};
		#pragma pack()

		class graceVGN1BFile
		{
		public:
			graceVGN1BFile(void);
		public:
			~graceVGN1BFile(void);
		public:
			bool open(string strVGN1BfileName);
		public:
			vector<VGN1BRecord> m_data;
		};
	}
}
