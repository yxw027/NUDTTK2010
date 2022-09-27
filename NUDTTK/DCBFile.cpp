#include "DCBFile.hpp"
#include <math.h>

namespace NUDTTK
{
	DCBFile::DCBFile(void)
	{
	}
	DCBFile::~DCBFile(void)
	{
	}
	void DCBFile::clear()
	{
		m_headP1C1 = DCBFileHeader::DCBFileHeader();
		m_dataP1C1.clear();
		m_headP1P2 = DCBFileHeader::DCBFileHeader();
		m_dataP1P2.clear();
	}

	bool DCBFile::isValidEpochLine(string strLine, DCBDatum& datum)
	{
		bool nFlag = true;
		char szName[4];
		char szStationName[17];
		double value = 0;
		double rms = 0;
		//***   ****************    *****.***   *****.***
		sscanf(strLine.c_str(), "%3c%*3c%16c%*4c%9lf%*3c%9lf",
			                    szName,
	                            szStationName,
							    &value,
							    &rms);
		szName[3] = '\0';
		szStationName[16] = '\0';

	    if(szName[0] == 'G'
		|| szName[0] == 'R'
		|| szName[0] == 'C'
		|| value != 0)
		{
			datum.name  = szName;
            datum.value = value;
            datum.rms = rms;
			nFlag = true;
		}
		else
			nFlag = false;
		return nFlag;
	}


	// 子程序名称： open   
	// 功能：读取 DCB 文件数据
	// 变量类型：P1C1FileName: DCB文件名称
	//           P1P2FileName: DCB文件名称
	// 输入：P1C1FileName, P1P2FileName
	// 输出：
	// 语言：C++
	// 创建者：鞠 冰, 谷德峰
	// 创建时间：2013/11/16
	// 版本时间：2013/11/15
	// 修改记录：
	// 备注： 
	bool DCBFile::open(string  P1C1FileName, string  P1P2FileName)
	{
		// P1C1
		clear();
		FILE * pDCBFile = fopen(P1C1FileName.c_str(), "r+t");
		if(pDCBFile == NULL) 
			return false;
		char line[200];
		// Line 1
		fgets(line, 200, pDCBFile); 
		sprintf(m_headP1C1.szAgencyObsTypeTime, "%80s", line);
		m_headP1C1.szAgencyObsTypeTime[80] = '\0';
		// Line 2
		fgets(line, 200, pDCBFile); 
		sprintf(m_headP1C1.szline, "%80s", line);
		m_headP1C1.szline[80] = '\0';
		// Line 4
		fgets(line, 200, pDCBFile); 
		fgets(line, 200, pDCBFile); 
		sprintf(m_headP1C1.szComments, "%62s", line);
		m_headP1C1.szComments[62] = '\0';
		// Line 6
		fgets(line, 200, pDCBFile); 
		fgets(line, 200, pDCBFile); 
		sprintf(m_headP1C1.szPrnValueRms, "%46s", line);
		m_headP1C1.szPrnValueRms[46] = '\0';
		// Line 7
		fgets(line, 200, pDCBFile); 
		sprintf(m_headP1C1.szFormatMarker, "%47s", line);
		m_headP1C1.szFormatMarker[47] = '\0';
        //m_dataP1C1.clear();
		while(!feof(pDCBFile))
		{
			if(fgets(line, 200, pDCBFile))
			{
				DCBDatum datum;
				if(isValidEpochLine(line, datum))
				{
					m_dataP1C1.insert(SatDCBCorrectMap::value_type(datum.name, datum));
				}
			}
		}
		fclose(pDCBFile);
        // P1P2
		FILE * pDCBFile2 = fopen(P1P2FileName.c_str(), "r+t");
		if(pDCBFile2 == NULL) 
			return false;
		// Line 1
		fgets(line, 200, pDCBFile2); 
		sprintf(m_headP1P2.szAgencyObsTypeTime, "%80s", line);
		m_headP1P2.szAgencyObsTypeTime[80] = '\0';
		// Line 2
		fgets(line, 200, pDCBFile2); 
		sprintf(m_headP1P2.szline, "%80s", line);
		m_headP1P2.szline[80] = '\0';
		// Line 4
		fgets(line, 200, pDCBFile2); 
		fgets(line, 200, pDCBFile2); 
		sprintf(m_headP1P2.szComments, "%62s", line);
		m_headP1P2.szComments[62] = '\0';
		// Line 6
		fgets(line, 200, pDCBFile2); 
		fgets(line, 200, pDCBFile2); 
		sprintf(m_headP1P2.szPrnValueRms, "%46s", line);
		m_headP1P2.szPrnValueRms[46] = '\0';
		// Line 7
		fgets(line, 200, pDCBFile2); 
		sprintf(m_headP1P2.szFormatMarker, "%47s", line);
		m_headP1P2.szFormatMarker[47] = '\0';
        //m_dataP1P2.clear();
		while(!feof(pDCBFile2))
		{
			if(fgets(line, 200, pDCBFile2))
			{
				DCBDatum datum;
				if(isValidEpochLine(line, datum))
				{
					m_dataP1P2.insert(SatDCBCorrectMap::value_type(datum.name, datum));
				}
			}
		}
		fclose(pDCBFile2);
		return true;
	}

	// 子程序名称： write   
	// 功能：读取 DCB 文件数据
	// 变量类型：P1C1FileName: DCB文件名称
	//           P1P2FileName: DCB文件名称
	// 输入：P1C1FileName, P1P2FileName
	// 输出：
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/11/16
	// 版本时间：2013/11/15
	// 修改记录：
	// 备注： 
	bool DCBFile::write(string  P1C1FileName, string  P1P2FileName)
	{
		return true;
	}

	// 子程序名称： getDCBCorrectValue   
	// 功能：获取天线修正数据结构
	// 变量类型：satName：  卫星名称
	//			 dcb_P1 ：  DCB修正值(单位：米)
	//			 dcb_P2 ：  DCB修正值(单位：米)
	//           RecType：  接收机类型, 'P'、'C'、'N'
	/*
				 'P':  receiver is cross-correlating and requires correction of P2' and C1 Rogue SNR, Trimble 4000, etc.
				 'C':  receiver is non-cross-correlating but reports C1 instead of P1 Trimble 4700, 5700, Leica RS500, CRS1000, SR9600, etc. unless AS is off
				 'N':  receiver is non-cross-correlating and reports true P1, P2 
	*/
	// 接收机类型对应解释
	// P类型：C1/X2=C1 + (P2 - P1)
	// C类型：C1/P2
	// N类型：P1/P2
	// 输入：satName, RecType
	// 输出：dcb_P1, dcb_P2
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/11/16
	// 版本时间：2013/11/15
	// 修改记录：
	// 备注： 
	bool DCBFile::getDCBCorrectValue(string satName, double &dcb_P1, double &dcb_P2, char RecType)
	{
		dcb_P1 = 0;
		dcb_P2 = 0;
		SatDCBCorrectMap::iterator it_P1C1 = m_dataP1C1.find(satName);
		SatDCBCorrectMap::iterator it_P1P2 = m_dataP1P2.find(satName);
		if(RecType == 'C')
		{
			if(it_P1C1 != m_dataP1C1.end() && it_P1P2 != m_dataP1P2.end())
			{
				dcb_P1 = it_P1C1->second.value * 1.0E-9 * SPEED_LIGHT + pow(GPS_FREQUENCE_L2, 2) / (pow(GPS_FREQUENCE_L1, 2) - pow(GPS_FREQUENCE_L2, 2)) * it_P1P2->second.value * 1.0E-9 * SPEED_LIGHT;
				dcb_P2 = pow(GPS_FREQUENCE_L1, 2) / (pow(GPS_FREQUENCE_L1, 2) - pow(GPS_FREQUENCE_L2, 2)) * it_P1P2->second.value * 1.0E-9 * SPEED_LIGHT;
			}
		}
		if(RecType == 'N')
		{
			if(it_P1P2 != m_dataP1P2.end())
			{
				dcb_P1 = pow(GPS_FREQUENCE_L2, 2) / (pow(GPS_FREQUENCE_L1, 2) - pow(GPS_FREQUENCE_L2, 2)) * it_P1P2->second.value * 1.0E-9 * SPEED_LIGHT;
				dcb_P2 = pow(GPS_FREQUENCE_L1, 2) / (pow(GPS_FREQUENCE_L1, 2) - pow(GPS_FREQUENCE_L2, 2)) * it_P1P2->second.value * 1.0E-9 * SPEED_LIGHT;
			}
		}
		if(RecType == 'P')
		{
			if(it_P1C1 != m_dataP1C1.end() && it_P1P2 != m_dataP1P2.end())
			{
				dcb_P1 = it_P1C1->second.value * 1.0E-9 * SPEED_LIGHT + pow(GPS_FREQUENCE_L2, 2) / (pow(GPS_FREQUENCE_L1, 2) - pow(GPS_FREQUENCE_L2, 2)) * it_P1P2->second.value * 1.0E-9 * SPEED_LIGHT;
				dcb_P2 = it_P1C1->second.value * 1.0E-9 * SPEED_LIGHT + pow(GPS_FREQUENCE_L1, 2) / (pow(GPS_FREQUENCE_L1, 2) - pow(GPS_FREQUENCE_L2, 2)) * it_P1P2->second.value * 1.0E-9 * SPEED_LIGHT;
			}
		}
		return true;
	}
}