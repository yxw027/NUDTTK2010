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
		struct KBR1BRecord
		{// 103个字节
			int            gps_time;       // 4
            double         biased_range;   // 8
			double         range_rate;     // 8
			double         range_accl;     // 8
			double         iono_corr;      // 8
			double         lighttime_corr; // 8
			double         lighttime_rate; // 8
			double         lighttime_accl; // 8
			double         ant_centr_corr; // 8
			double         ant_centr_rate; // 8
			double         ant_centr_accl; // 8
			unsigned short K_A_SNR;        // 2
			unsigned short Ka_A_SNR;       // 2
			unsigned short K_B_SNR;        // 2
			unsigned short Ka_B_SNR;       // 2
			unsigned char  qualflg;

			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0);
				return t0 + gps_time;
			};
			bool getCorrectedBiasedRange(double &range);
		};
		#pragma pack()

		struct KBRRes
		{
			GPST   t;  
			double res;       // 扣掉模糊后的残差
			double ambiguity; // 模糊度
		};
		
		class graceKBR1BFile
		{
		public:
			graceKBR1BFile(void);
		public:
			~graceKBR1BFile(void);
		public:
			bool open(string  strKBR1BFileName);
			bool open_fo(string  strKBR1BFileName);
			void cutdata(GPST t0, GPST t1);
			void cutdata_fo(GPST t0, GPST t1);
			bool write(string  strKBR1BFileName);
			bool write_fo(string  strKBR1BFileName); // grace_fo格式文件
			static bool calculateKBRRes(vector<KBRRes> &resList, double &rms);
		public:
			vector<string>      m_header;
			vector<KBR1BRecord> m_data;
		};
	}
}
