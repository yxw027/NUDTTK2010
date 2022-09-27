#pragma once
#include "structDef.hpp"
#include <vector>

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SLR
	{
		struct CRDFRDHeaderRecord
		{ // 与 CRDHeaderRecord 相同
			////////////////////Format Header,H1或h1(下同)
			char     szFormat[3 + 1];        // A3
			int      nVersion;               // I2
			UTC      ProductTime;            // I4 I2 I2 I2 I2
			////////////////////Station Header,H2
			char     szStaName[10 + 1];      // A10 
			int      nCDPPadID;              // I4
			int      nCDPSysNum;             // I2
			int      nCDPOccSeqNum;          // I2
			int      nStaTimeRef;               // I2 3=UTC (USNO),4 = UTC (GPS),7 = UTC (BIH)
			////////////////////Target Header,H3
			char     szTargetName[10 + 1];   // A10
			int      nILRSSatID;        	 // I8,基于COSPAR ID
			int      nSIC;              	 // I4,Satellite Identification Code
			int      nNORADID;               // I8
			int      nTargetTimeRef;   	     // I1,0 = Not used,1 = UTC,2 = Spacecraft Time Scale
			int      nTargetType;       	 // I1,1=passive (retro-reflector) artificial satellite,2=passive (retro-reflector) lunar reflector
                                        	 //    3=synchronous transponder,4=asynchronous transponder
			////////////////////Session(Pass) Header,H4
			int      nDataType;         	 // I1,0 = full rate,1 = normal point,2 = sampled engineering(三种数据格式)
			UTC      StartTime;              // I4,I2,I2,I2,I2,I2
			UTC      EndTime;                // I4,I2,I2,I2,I2,I2
			int      nReleaseFlag;      	 // I1,0: first release of data，1: first replacement release of the data,2: second replacement release, etc
			int      nTroCorr;          	 // I1,是否进行了对流层修正,0-false,1-true
			int      nCenOfMassCorr;    	 // I1,是否进行了质心修正
			int      nRecAmpCorr;       	 // I1,是否进行了Receive amplitude修正
			int      nStaSysDelayCorr;  	 // I1,是否进行了测站系统延迟修正
			int      nCraftSysDelayCorr;	 // I1,是否进行了空间目标系统延迟修正
			int      nRangeType;         	 // I1,距离种类，0-无距离(仅时间)，1-单程距离，2-双程距离，3-仅接收时间，4-Mixed
			int      nDataQuality;      	 // I1,0 = 较好;1 = 可能有问题;2 = 质量较差或未知
			CRDFRDHeaderRecord()
			{
				memset(this,0,sizeof(CRDFRDHeaderRecord));				
			}
		};
		struct CRDFRDConfigRecord
		{ // 与 CRDConfigRecord 相同
			////////////////////System Configuration Record,C0或c0
			int      nC0DetailType;        	 // I1,"0"
			double   Wavelength;           	 // F10.3,波长，nm
			char     szSysID[4 + 1]; 	 	 // A4
			char     szComAID[4 + 1];	 	 // A4
			char     szComBID[4 + 1];	 	 // A4
			char     szComCID[4 + 1];	 	 // A4
			char     szComDID[4 + 1];	 	 // A4
			////////////////////Laser Configuration Record,C1或c1
			int      nC1DetailType;          // I1,"0"
			char     szLaserID[4 + 1];       // A4
			char     szLaserType[10 + 1];    // A10
			double   PriWavelength;          // F10.2,nm
			double   NormalFireRate;         // F10.2,Hz
			double   PulseEnergy;            // F10.2,mJ
			double   PulseWidth;             // F6.1,FWHM in ps
			double   BeamDiv;                // F5.2,arcsec
			int      nOSTCount;              // I4,Number of pulses in outgoing semi-train
			////////////////////Detector Configuration Record,C2或c2
			int      nC2DetailType;          // I1,"0"
			char     szDetectID[4 + 1];      // A4
			char     szDetectType[10 + 1];   // A10
			double   AppWavelength;          // F10.2,nm
			double   QuaEff;                 // F6.2,%
			double   AppVoltage;             // F5.1,V
			double   DarkCount;              // F5.1,kHz
			char     szOutPulType[10 + 1];   // A10
			double   OutPulWidth;            // F5.1,ps
			double   SpeFilter;              // F5.2,nm
			double   TraSpeFilter;           // F5.1,%
			double   SpaFilter;              // F5.1,arcsec
			char     szExtSigPro[10 + 1];    // A10
			////////////////////Timing System Configuration Record,C3或c3
			int      nC3DetailType;          // I1,"0"
			char     szTimeSysID[4 + 1];     // A4
			char     szTimeSource[20 + 1];   // A20
			char     szFreSource[20 + 1];    // A20
			char     szTimer[20 + 1];        // A20
			char     szTimerSerNum[20 + 1];  // A20
			double   EpochDelayCorr;         // us			
			////////////////////Transponder (Clock) Configuration Record,C4或c4
			int      nC4DetailType;          // I1,"0"
			char     szTraID[4 + 1];         // A4
			double   StaTimeOffset;          // F20.3,ns,UTC
			double   StaTimeDrift;           // F11.2,in parts in 10^15
			double   TraTimeOffset;          // F20.3,ns,UTC
			double   TraTimeDrift;           // F11.2,in parts in 10^15
			double   TraTimeRef;             // F20.12,s,
			int      nStaTimeApp;            // I1,0-没修正钟差和钟漂;1-仅修正了钟差;2-仅修正了钟漂;3-两项均已修正
			int      nCraftTimeApp;          // I1,同上
			int      nCraftTimeSimp;         // I1,0-false，1-true
			CRDFRDConfigRecord()
			{
				memset(this,0,sizeof(CRDFRDConfigRecord));				
			}
		};
		struct CRDFRDDataRecord                 
		{ // 与 CRDDataRecord 有差别
			////////////////////Range Record (Full rate),10
			double   TimeofDay;              // F18.12,s
			double   TimeofFlight;           // F18.12,s
			char     szSysID[4 + 1];         // A4
			int      nEpochEvent;            // I1-Spacecraft bounce time(2-way),2-Ground transmit time(2-way)
			int      nFilterFlag;            // I1-0: unknown, 1:noise, 2:data
			int      nDetectorChannel;       // I1-0:not applicable or "all"; 1-4:for quadrant; 1-n: for many channels
			int      nStopNumber;            // I1-0:not applicable or unknown; 1-n: stop number
			int      nReceiveAmplitude;      // I5
			////////////////////Meteorological Record,20
			double   MRTimeofDay;            // F18.12,(typically to 1 milllisec)
			double   SurfacePressure;        // F7.2,mbar
			double   SurfaceTemperature;     // F6.2,Kelvin
			double   SurfaceRelHumidity;     // F4.0,%
			int      nOriginofValue;         // 0-measured values,1-interpolated
			CRDFRDDataRecord()
			{
				//通常20和10 作为一组数据，但当气象数据变化不大时，20数据仅出现一次
				memset(this,0,sizeof(CRDFRDDataRecord));				
			}
		};
		struct CRDFRD405060Record
		{ // 与 CRD405060Record 相同
			////////////////////Calibration Record,40
			double   SecondofDay;            // F18.12,s
			int      nDataType;              // I1,0-测站发射与接收;1-测站发射;2-测站接收;3,4,5(目标)
			char     sz40SysID[4 + 1];       // A4
			int      nPointRecord;           // I8,(-1,no information)
			int      nPointUsed;             // I8,
			double   DistanceOneWay;         // F7.3,m
			double   SysDelay;               // F10.1,ps
			double   DelayShift;             // F8.1,ps
			double   RawSysDelayRMS;         // F6.1,ps
			double   RawSysDelaySkew;        // F7.3
			double   RawSysDelayKurtosis;    // F7.3
			double   SysDelayPeak_Mean;      // F6.1,ps
			int      nCalibrationType;       // I1,0 = Not used or undefined,1 = Nominal (from once off assessment),2 = External cals,3 = Internal cals,4 = Burst cals,5 = Other
			int      nShiftType;             // I1,0 = Not used or undefined,1 = Nominal (from once off assessment),2 = Pre – to Post- Shift,3 = Minimum to maximum,4 = Other
			int      nDetecorChannel;        // I1,0 = not applicable or “all”,1-4 for quadrant,1-n for many channels
			////////////////////Session (Pass) Statistics Record,50
			char     sz50SysID[4 + 1];       // A4
			double   SessionRMS;             // F6.1,ps
			double   SessionSkew;            // F7.3
			double   SessionKurtosis;        // F7.3
			double   SessionPeak_Mean;       // F6.1,ps
			int      nDataQuality;           // I1,0-未知;1-Clear;2-Clear with noise;3--Clear with significant noise;4-un-clear;5-No data apparent
			////////////////////Compatibility Record,60
			char     sz60SysID[4 + 1];       // A4
			int      nSCHIndicator;          // I1,
			int      nSCIIndicator;          // I1,
			CRDFRD405060Record()
			{				
				memset(this,0,sizeof(CRDFRD405060Record));				
			}
		};
		struct crdFrdSinglePassArc
		{
			CRDFRDHeaderRecord         crdHeader;
			CRDFRDConfigRecord         crdConfig;
			vector<CRDFRDDataRecord>   crdDataRecordList;
			CRDFRD405060Record         crd405060;
			UTC getTime(CRDFRDDataRecord Record);
		};

		class crdFrdSLRObsFile
		{
		public:
			crdFrdSLRObsFile(void);
		public:
			~crdFrdSLRObsFile(void);
		public:			
			bool open(string strCRDFRDFileName);		
		public:			
			vector<crdFrdSinglePassArc>     m_data;
		};
	}
}