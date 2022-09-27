#pragma once
#include "constDef.hpp"
#include <string>
#include <map>
 
//  Copyright 2014, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct DCBFileHeader
	{
		char szAgencyObsTypeTime[80 + 1];	// DCB数据说明
		char szline[80 + 1];				// -----------
		char szComments[62 + 1];			// Comments on Data Type
		char szPrnValueRms[46 + 1];			// PRN + VALUE + RMS
		char szFormatMarker[47 + 1];		// ***  *****.*** *****.***
		DCBFileHeader()
		{
			 memset(szAgencyObsTypeTime, '1' ,sizeof(szAgencyObsTypeTime));
			 memset(szline, '1' ,sizeof(szline));
			 memset(szComments, '1' ,sizeof(szComments));
			 memset(szPrnValueRms, '1' ,sizeof(szPrnValueRms));
			 memset(szFormatMarker, '1' ,sizeof(szFormatMarker));
		}
	};

	struct DCBDatum
	{
		string  name;
		char szName[3+1];
		double  value;	// 单位：NS (1e-9s)
		double  rms;	// 单位：NS (1e-9s)
		DCBDatum()
		{
			name = "";
			value = 0.0;
			rms   = 0.0;
		}
	};

	typedef map<string, DCBDatum> SatDCBCorrectMap;

	// CODE DCB 文件解析类
	class DCBFile
	{
	public:
		DCBFile(void);
	public:
		~DCBFile(void);
	public:
		void clear();
		bool isValidEpochLine(string strLine,DCBDatum& datum);
		bool open(string  P1C1FileName, string  P1P2FileName);
		bool write(string  P1C1FileName, string  P1P2FileName);
		bool getDCBCorrectValue(string satName, double &dcb_P1, double &dcb_P2, char RecType);
	public:
		DCBFileHeader     m_headP1C1;	// P1C1文件头
		SatDCBCorrectMap  m_dataP1C1;   // P1C1的DCB修正数据
		DCBFileHeader     m_headP1P2;   // P1P2文件头
		SatDCBCorrectMap  m_dataP1P2;   // P1P2的DCB修正数据
	};
}