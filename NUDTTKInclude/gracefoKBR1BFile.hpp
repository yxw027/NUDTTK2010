#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>

using namespace NUDTTK;
using namespace std;
namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		#pragma pack(1)
		struct KBR1BRecordfo
		{
			int            gps_time;
            double         biased_range;
			double         range_rate;
			double         range_accl; 
			double         iono_corr; 
			double         lighttime_corr;
			double         lighttime_rate;
			double         lighttime_accl;
			double         ant_centr_corr;
			double         ant_centr_rate;
			double         ant_centr_accl;
			double         K_A_SNR; 
			double         Ka_A_SNR;
			double         K_B_SNR; 
			double         Ka_B_SNR;
			unsigned char  qualflg;

			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0);
				return t0 + gps_time;
			};
			bool getCorrectedBiasedRange(double &range);
		};
		#pragma pack()

		struct KBRRes_fo
		{
			GPST   t;  
			double res;       // 扣掉模糊后的残差
			double ambiguity; // 模糊度
		};
		
		class gracefoKBR1BFile
		{
		public:
			gracefoKBR1BFile(void);
		public:
			~gracefoKBR1BFile(void);
		public:
			bool open(string  strKBR1BFileName);
			void cutdata(GPST t0, GPST t1);
			bool write(string  strKBR1BFileName);
			bool write_grace(string  strKBR1BFileName);
			static bool calculateKBRRes(vector<KBRRes_fo> &resList, double &rms);
		public:
			vector<string>      m_header;
			vector<KBR1BRecordfo> m_data;
		};
	}
}
