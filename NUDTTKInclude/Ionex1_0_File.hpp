#pragma once
#include "structDef.hpp"
#include "constDef.hpp"
#include "Matrix.hpp"
#include <vector>

//  Copyright 2013, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	struct Ionex1_0_MaskString
	{
		static const char ioVerType[];
		static const char ioComment[];
		static const char ioPgmRunDate[];
		static const char ioDescription[];
		static const char ioEpochFirstMap[];
		static const char ioEpochLastMap[];
		static const char ioInterval[];
		static const char ioOfMapsinFile[];
		static const char ioMappingFun[];
		static const char ioElevationCutOff[];
		static const char ioObservablesUsed[];
		static const char ioOfStastions[];
		static const char ioOfSatellites[];
		static const char ioBaseRadius[];
		static const char ioMapDimension[];
		static const char ioHgt1Hgt2Dhgt[];
		static const char ioLat1Lat2Dlat[];
		static const char ioLon1Lon2Dlon[];
		static const char ioExponent[];
		static const char ioEndofHeader[];
	};

	struct Ionex1_0_Header
	{
		char    ioVersion[20+1];                
		char    ioFileType[20+1];                    
		char    ioSatelliteSystem[20+1];
		char    ioProgramName[20+1];
		char    ioProgramAgencyName[20 + 1];
		char    ioFileDate[20 + 1];
		char    ioObservablesUsed[60+1];
		char    ioMappingFunction[60+1];		
		char    ioCommentList[100];
		char    ioDescription[100];
		int     NumberOfMaps;		
		int     NumberOfStations;
		int     NumberOfSatellites;		
		int     MapDimension;
		int     Exponent;
		double  Height1;
		double  Height2;
		double  DeltaHeight;
		double  Lat1;
		double  Lat2;
		double  DeltaLat;
		double  Lon1;
		double  Lon2;
		double  DeltaLon;
		int     Interval;
		double  ElevationCutoff;
		double  BaseRadius;
		DayTime epochFirst;
		DayTime epochLast;

		Ionex1_0_Header()
		{
			memset(this,0,sizeof(Ionex1_0_Header));  //将Ionex1_0_Header初始化为零
		}
	};

	struct Ionex1_0_Map
	{
		DayTime         t;
		double          Lat1;
		double          Lat2;
		double          DeltaLat;
		double          Lon1;
		double          Lon2;
		double          DeltaLon;
		double          Height;
		Matrix          Grid;
	};

	class Ionex1_0_File
	{
	public:
		Ionex1_0_File(void);
	public:
		~Ionex1_0_File(void);
	public:
		void    clear();
		bool    isEmpty();
		bool    open(string  strIONFileName);
		bool    getVTEC(DayTime t, double Lat, double Lon, double& VTEC);
	public:
		Ionex1_0_Header      m_header;
		vector<Ionex1_0_Map> m_data; 
	};
}

