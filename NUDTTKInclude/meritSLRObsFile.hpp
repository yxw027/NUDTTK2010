#pragma once
#include "structDef.hpp"
#include <vector>

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SLR
	{
		struct meritDataRecord
		{
			char     szSatCOSPARID[8];      // 1  - 7   '7603901'
			int      Year;                  // 8  - 9   '87'
			int      Day;                   // 10 -12   ' 76'
			double   TimeofDay;             // 13 -24   ' 36005000000'  .1 microsecond
			int      StationID;             // 25 -28   '7105'
			int      CDPSysNum;             // 29 -30   '07'
			int      CDPOSNum;              // 31 -32   '02'
			double   Azimuth;               // 33 -39   ' 987500'       .1 millidegree
			double   Elevation;             // 40 -45   '292500'        .1 millidegree
			double   LaserRange;            // 46 -57   ' 52035998000'   1 picosecond
			double   LaserRangeSD;          // 58 -64   '     66'        1 picosecond
			double   Wavelength;            // 65 -68   '5320'          .1 nanometer
			double   SurfacePressure;       // 69 -73   '10135'         .1 millibar
			double   SurfaceTemperature;    // 74 -77   '2905'          .1 degree Kelvin
			double   SurfaceRelHumidity;    // 78 -80   ' 55'              percentage
			double   TropCorrect;           // 81 -85   '33956'          1 picosecond 
			double   MassCenterCorrect;     // 86 -91   '  1601'         1 picosecond
			double   ReceiveAmplitude;      // 92 -96   '  700'          0 - 2000
			double   CalibrationSysDelay;   // 97 -104  '   95942'       1 picosecond
			double   CalibrationDelayShift; // 105-110  '    33'         1 picosecond
			double   CalibrationSD;         // 111-114  '  40'           1 picosecond
			int      NormalWindowIndicator; // 115      '0'
			int      CompressedRangeNum;    // 116-119  
			int      EpochEvent;            // 120
			int      EpochTimeScale;        // 121
			
			int      AngleOriginIndicator;  // 122
			int      TRCIndicator;          // 123
			int      MCCIndicator;          // 124
			int      RACIndicator;          // 125      '1'   Receive Amplitude Correction Indicator
			
			int      SCMIndicator;          // 126      '0'   System Calibration Method Indicator
			int      SCHIndicator;          // 127      '0'   系统改变标记指示(System CHange indicator, SCH), 2008-11-06
			int      SCFIndicator;          // 128      '1'   System Configuration Flag Indicator
			int      FRNIndicator;          // 129      '2'   Format Revision Number Indicator
			char     RFIndicator;           // 130      'A'   Release Flag Indicator
			
			meritDataRecord() // 2007-09-04
			{
				memset(this, 0, sizeof(meritDataRecord));
			}
			void screenPrintf();
			UTC  getTime();      
		};

		class meritSLRObsFile
		{
		public:
			meritSLRObsFile(void);
		public:
			~meritSLRObsFile(void);
		public:
			public:
			bool open(string strMeritFileName);
			bool append(string strMeritFileName);
			bool write(string strMeritFileName);
			bool isValidEpochLine(string strLine, meritDataRecord& meritLine);
		public:
			vector<meritDataRecord> m_data;
		};
	}
}
