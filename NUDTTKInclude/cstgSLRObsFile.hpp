#pragma once
#include "structDef.hpp"
#include <vector>

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SLR
	{
		struct cstgHeaderRecord
		{
			char     szSatCOSPARID[8];       // 1  - 7   '7603901'
			int      nYear;                  // 8  - 9   '89'
			int      nDay;                   // 10 -12   '079'
			int      nCDPPadID;              // 13 -16   '7105'      CDP -- Crustal Dynamics Project
			int      nCDPSysNum;             // 17 -18   '07'
			int      nCDPOccSeqNum;          // 19 -20   '02'
			// 读取后, 波长以纳米为单位
			double   Wavelength;             // 21 -24   '5321'      3000 - 9999: units are 0.1 nanometer, 1000 - 2999: units are 1.0 nanometer
			double   CalibrationSysDelay;    // 25 -32   '00095942'  two-way value in picoseconds 
			double   CalibrationDelayShift;  // 33 -38   '000033'    two-way value in picoseconds
			double   CalibrationDelayRMS;    // 39 -42   '0040'      Root Mean Square (RMS) of raw system delay values from the mean
			int      NormalWindowIndicator;  // 43       '7'
			int      EpochTimeScale;         // 44       '3'         Epoch time scale indicator
			int      SCMIndicator;           // 45       '0'         System calibration method and delay shift indicator
			int      SCHIndicator;           // 46       '0'         A flag to increment for every major change to the system (hardware or software). 
			int      SCIndicator;            // 47       '1'         System Configuration Indicator (SCI)
			double   PassRMS;                // 48 -51   '0065'      two-way value in picoseconds
			///////////////////////////////////
			// LLR 使用
			int      DQSIndicator;           // 52       '0'         Data quality assessment indicator
			///////////////////////////////////
			int      nCheckSum;              // 53 -54   '54'        an integer value equal to the sum of integers in columns 1-52, modulo 100
			int      nFormatIndicator;       // 55       '1'         
		};

		struct cstgDataRecord
		{
			double   TimeofDay;              //  1 -12   '214360786545'  .1 microseconds  Time of day of laser firing, Value is given module 86400 if pass crosses 24 hours UTC
			double   LaserRange;             // 13 -24   '052035998000'  in picoseconds,  Two-way time-of-flight corrected for system delay
			double   RMS;                    // 25 -31   '0000066'       in picoseconds
			double   SurfacePressure;        // 32 -36   '10135'         .1 millibar
			double   SurfaceTemperature;     // 37 -40   '2905'          .1 degree Kelvin
			double   SurfaceRelHumidity;     // 41 -43   ' 55'           percentage
			int      CompressedRangeNum;     // 44 -47   '0108'			 Number of raw ranges (after editing) compressed into normal point					 
			int      DataReleaseIndicate;    // 48       '0'
			///////////////////////////////////
			// For SLR data: not used
			int      IntegerSeconds;         // 49  	 '2'			 integer seconds of the two-way time of	flight 	 For SLR data: not used 		 
			int      NormalPointWindow;      // 50       '1'             normal point window indicator                   For SLR data: not used 
			double   SNR;                    //	51 -52	 '00'      		 signal to noise ratio, in units of 0.1,         For SLR data: not used	
			///////////////////////////////////
			int      nCheckSum;              // 53 -54	 '51'			 an integer value equal to the sum of integers in columns 1-52, modulo 100				
		};

		struct cstgRawDataRecord
		{
			double   TimeofDay;              //  1 -12   '214360786545'  .1 microseconds  Time of day of laser firing, Value is given module 86400 if pass crosses 24 hours UTC
			double   LaserRange;             // 13 -24   '052035998000'  in picoseconds,  Two-way time-of-flight with no corrections applied
			double   SurfacePressure;        // 25 -29   '10135'         .1 millibar
			double   SurfaceTemperature;     // 30 -33   '2905'          .1 degree Kelvin
			double   SurfaceRelHumidity;     // 34 -36   ' 55'           percentage
			double   InternalBurstCSD;       // 37 -44   '00003124'      Internal burst calibration system delay
			double   RelativeSignalStrength; // 45 -48   '0789'          unit of measure determined by individual stations                 
			int      AngleOriginIndicator;   // 49       '3'             Angle origin indicator
			double   Azimuth;                // 50 -56   '0981501'       0.0001 degree, north 0, east = 90
			double   Elevation;              // 57 -62   '292501'        0.0001 degree, zenith = 90
			int      Unused;                 // 63 -67   '00000'         Unused (zero-filled)
			int      nCheckSum;              // 68 -69	 '07'			 modulo 100		
		};

		// 记录单次过顶的激光数据
		struct cstgSinglePassArc
		{// 主要是前两项, 后面两项暂时未读取
			cstgHeaderRecord          normalHeaderRecord;     // 头记录
			vector<cstgDataRecord>    normalDataRecordList;   // 标准点数据列表
			cstgHeaderRecord          rawHeaderRecord;        // 头记录
			vector<cstgRawDataRecord> rawDataRecordList;      // 原始测量数据列表
			UTC getTime(cstgDataRecord Record);
		};

		class cstgSLRObsFile
		{
		public:
			cstgSLRObsFile(void);
		public:
			~cstgSLRObsFile(void);
		public:
			int  isValidNewPass(string strLine, FILE * pCSTGFile = NULL);
			bool readLine_HeaderRecord(string strLine, cstgHeaderRecord& HeaderRecord);
			bool readLine_DataRecord(string strLine, cstgDataRecord& DataRecord);
			bool open(string strCSTGFileName, bool bAdd = false);
		public:
			static void deleteStringZero(char* strSrc);
		public:
			vector<cstgSinglePassArc> m_data;
			bool                      m_bChecksum;
		};
	}
}
