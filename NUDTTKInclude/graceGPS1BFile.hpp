#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>
#include <windows.h>
#include <limits>

namespace NUDTTK
{
	namespace SpaceborneGPSPreproc
	{
		#pragma pack(1)
		struct GPS1BRecord
		{// 74个字节
			int            rcvtime_intg; // 4
			int            rcvtime_frac; // 4
			char           GRACE_id;     // 1
			char           prn_id;       // 1
			char           ant_id;       // 1
			unsigned short prod_flag;    // 2
			unsigned char  qualflg;      // 1
			double         CA_range;     // 8
			double         L1_range;     // 8
			double         L2_range;     // 8
			double         CA_phase;     // 8
			double         L1_phase;     // 8
			double         L2_phase;     // 8
			unsigned short CA_SNR;       // 2
			unsigned short L1_SNR;       // 2
			unsigned short L2_SNR;       // 2
			unsigned short CA_chan;      // 2
			unsigned short L1_chan;      // 2
			unsigned short L2_chan;      // 2
			GPST gettime()
			{
				GPST t0(2000, 1, 1, 12, 0, 0);
				return t0 + (rcvtime_intg + rcvtime_frac * 1.0E-6);
			};
		};

		struct GPS1BEpoch
		{
			GPST						t;
			vector<GPS1BRecord>   obslist;
		};
		#pragma pack()

		//// grace_fo
		//struct GPS1B_GRACEFO_Record
		//{
		//	double         rcvtime_intg;
		//	double         rcvtime_frac;
		//	char           GRACE_id;  // C or D 
		//	int            prn_id;       
		//	int            ant_id;       
		//	vector<int>    prodFlagList;       // prod flag列表         
		//	vector<int>    qualFlagList;       //qual flag列表  
		//	double         CA_range;     
		//	double         L1_range;     
		//	double         L2_range;     
		//	double         CA_phase;     
		//	double         L1_phase;     
		//	double         L2_phase;    
		//	int         CA_SNR;       
		//	int         L1_SNR;       
		//	int         L2_SNR;       
		//	int         CA_chan;      
		//	int         L1_chan;      
		//	int         L2_chan; 
		//	double         K_phase;
		//	double         Ka_phase;
		//	int         K_SNR;       
		//	int         Ka_SNR; 
		//	GPST gettime()
		//	{
		//		GPST t0(2000, 1, 1, 12, 0, 0);
		//		return t0 + (rcvtime_intg + rcvtime_frac * 1.0E-6);
		//	};
		//	GPS1B_GRACEFO_Record()
		//	{
		//		memset(this,0,sizeof(GPS1B_GRACEFO_Record));	
		//		CA_SNR            = INT_MAX;
		//		L1_SNR            = INT_MAX;
		//		L2_SNR            = INT_MAX;
		//		CA_chan           = INT_MAX;
		//		L1_chan           = INT_MAX;
		//		L2_chan           = INT_MAX;
		//		K_SNR             = INT_MAX;
		//		Ka_SNR            = INT_MAX;
		//		CA_range          = DBL_MAX;
		//		L1_range          = DBL_MAX;
		//		L2_range          = DBL_MAX;
		//		CA_phase          = DBL_MAX;
		//		L1_phase          = DBL_MAX;
		//		L2_phase          = DBL_MAX;
		//		K_phase           = DBL_MAX;
		//		Ka_phase          = DBL_MAX;
		//	}
		//};

		//struct GPS1B_GRACEFO_Epoch
		//{
		//	GPST						        t;
		//	vector<GPS1B_GRACEFO_Record>   obslist;
		//};


		class graceGPS1BFile
		{
		public:
			graceGPS1BFile(void);
		public:
			~graceGPS1BFile(void);
			bool open(string  strGPS1BfileName);
			bool open_gracefo(string  strGPS1BfileName); // 打开gracefo GPS数据
			bool exportRinexObsFile(string  strObsfileName);
			bool exportRinexObsFile(string  strObsfileName, string& strNewObsfileName);
		public:
			vector<GPS1BEpoch> m_data;
		};
	}
}
