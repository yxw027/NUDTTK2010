#include "cstgSLRObsFile.hpp"
#include <math.h>

namespace NUDTTK
{
	namespace SLR
	{
		cstgSLRObsFile::cstgSLRObsFile(void)
		{
			m_bChecksum = true;
		}

		cstgSLRObsFile::~cstgSLRObsFile(void)
		{
		}

		UTC cstgSinglePassArc::getTime(cstgDataRecord Record)
		{
			int nB4Year = yearB2toB4(normalHeaderRecord.nYear);
			UTC t(nB4Year, 1, 1, 0, 0, 0);
			t = t + (normalHeaderRecord.nDay - 1) * 86400 ;    // 先算天的(nDay是否从 1 开始计数 ?)
			t = t + Record.TimeofDay * 1.0E-7;                 // 再加上秒的部分
			return t;
		}

		void cstgSLRObsFile::deleteStringZero(char* strSrc)
		{
			string str = strSrc;
			for(size_t i = 0; i < str.length(); i++)
			{
				if(strSrc[i] == '0')
					strSrc[i] = ' ';
				else
					return;
			}
		}

		// 寻找新弧段的起点:  0 - EOF; 1 - 新弧段开始; 2 - 原始数据记录开始; 3 - 其它
		int  cstgSLRObsFile::isValidNewPass(string strLine, FILE * pCSTGFile)
		{
			if(pCSTGFile != NULL) // 判断是否为文件末尾
			{
				if(feof(pCSTGFile))
					return 0;
			}
			if(strLine.length() < 5)
				return 3;
			char szHead[6];
			strLine.copy(szHead, 5, 0);
			szHead[5] = '\0';
			if(strcmp(szHead,"99999") == 0)
				return 1;
			if(strcmp(szHead,"88888") == 0)
				return 2;
			return 3;
		}

		bool cstgSLRObsFile::readLine_HeaderRecord(string strLine, cstgHeaderRecord& HeaderRecord)
		{
			if(strLine.length() < 55)
				return false;
			cstgHeaderRecord Record;
			char szCalibrationSysDelay[9];   // 可能是负数, 2007/12/27
			char szCalibrationDelayShift[7];
			sscanf(strLine.c_str(),"%7c%2d%3d%4d%2d%2d%4lf%8c%6c%4lf%1d%1d%1d%1d%1d%4lf%1d%2d%1d",
									Record.szSatCOSPARID,
								   &Record.nYear,
								   &Record.nDay,
								   &Record.nCDPPadID,
								   &Record.nCDPSysNum,
								   &Record.nCDPOccSeqNum,
								   &Record.Wavelength,
								   szCalibrationSysDelay,
								   szCalibrationDelayShift,
								   &Record.CalibrationDelayRMS,
								   &Record.NormalWindowIndicator,
								   &Record.EpochTimeScale,
								   &Record.SCMIndicator,
								   &Record.SCHIndicator,
								   &Record.SCIndicator,
								   &Record.PassRMS,
								   &Record.DQSIndicator,
								   &Record.nCheckSum,
								   &Record.nFormatIndicator
								);
			Record.szSatCOSPARID[7]    = '\0';
			szCalibrationSysDelay[8]   = '\0';
			szCalibrationDelayShift[6] = '\0';
			deleteStringZero(szCalibrationSysDelay);
			deleteStringZero(szCalibrationDelayShift);
			sscanf(szCalibrationSysDelay,   "%8lf",  &Record.CalibrationSysDelay);
			sscanf(szCalibrationDelayShift, "%6lf",  &Record.CalibrationDelayShift);
			if(Record.Wavelength >= 3000) // 变换成 ns
				Record.Wavelength = Record.Wavelength * 0.1;
			// 进行校验和比对
			double dsum = 0;
			for(size_t i = 0; i < 52; i++)
			{
				char szA[2];
				szA[0] = strLine.at(i);
				szA[1] = '\0';
				double dA = atof(szA);
				dsum += dA;
			}
			dsum = fmod(dsum, 100);
			if( int(dsum) != Record.nCheckSum && m_bChecksum)
			{
				//printf("readLine_HeaderRecord 校验和未通过检验! %s", strLine.c_str());
				return false;
			}
			HeaderRecord = Record;
			return true;
		}

		bool cstgSLRObsFile::readLine_DataRecord(string strLine, cstgDataRecord& DataRecord)
		{
			if(strLine.length() < 55)
				return false;
			cstgDataRecord Record;
			char szTimeofDay[13];
			char szLaserRange[13];
			char szRMS[8];
			char szSurfacePressure[6];
			char szSurfaceTemperature[5];
			char szSurfaceRelHumidity[4];
			char szCompressedRangeNum[5];
			char szDataReleaseIndicate[2];
			char szIntegerSeconds[2];
			char szNormalPointWindow[2];
			char szSNR[3];
			char sznCheckSum[3];
			sscanf(strLine.c_str(),"%12c%12c%7c%5c%4c%3c%4c%1c%1c%1c%2c%2c",
								   szTimeofDay,
								   szLaserRange,
								   szRMS,
								   szSurfacePressure,
								   szSurfaceTemperature,
								   szSurfaceRelHumidity,
								   szCompressedRangeNum,
								   szDataReleaseIndicate,
								   szIntegerSeconds,
								   szNormalPointWindow,
								   szSNR,
								   sznCheckSum
								);
			szTimeofDay[12]          = '\0';
			szLaserRange[12]         = '\0';
			szRMS[7]                 = '\0';
			szSurfacePressure[5]     = '\0';
			szSurfaceTemperature[4]  = '\0';
			szSurfaceRelHumidity[3]  = '\0';
			szCompressedRangeNum[4]  = '\0';
			szDataReleaseIndicate[1] = '\0';
			szIntegerSeconds[1]      = '\0';
			szNormalPointWindow[1]   = '\0';
			szSNR[2]                 = '\0';
			sznCheckSum[2]           = '\0';
			sscanf(szTimeofDay,            "%12lf",  &Record.TimeofDay);
			sscanf(szLaserRange,           "%12lf",  &Record.LaserRange);
			sscanf(szRMS,                  "%7lf",   &Record.RMS);
			sscanf(szSurfacePressure,      "%5lf",   &Record.SurfacePressure);
			sscanf(szSurfaceTemperature,   "%4lf",   &Record.SurfaceTemperature);
			sscanf(szSurfaceRelHumidity,   "%3lf",   &Record.SurfaceRelHumidity);
			sscanf(szCompressedRangeNum,   "%4d",    &Record.CompressedRangeNum);
			sscanf(szDataReleaseIndicate,  "%1d",    &Record.DataReleaseIndicate);
			sscanf(szIntegerSeconds,       "%1d",    &Record.IntegerSeconds);
			sscanf(szNormalPointWindow,    "%1d",    &Record.NormalPointWindow);
			sscanf(szSNR,                  "%2lf",   &Record.SNR);
			sscanf(sznCheckSum,            "%2d",    &Record.nCheckSum);
			// 进行校验和比对
			double dsum = 0;
			for(size_t i = 0; i < 52; i++)
			{
				char szA[2];
				szA[0] = strLine.at(i);
				szA[1] = '\0';
				double dA = atof(szA);
				dsum += dA;
			}
			dsum = fmod(dsum, 100);
			if( int(dsum) != Record.nCheckSum  && m_bChecksum)
			{
				//printf("readLine_DataRecord 校验和未通过检验! %s", strLine.c_str());
				return false;
			}
			DataRecord = Record;
			return true;
		}

		bool cstgSLRObsFile::open(string strCSTGFileName, bool bAdd)
		{
			FILE * pCSTGFile = fopen(strCSTGFileName.c_str(),"r+t");
			if(pCSTGFile == NULL) 
				return false;
			char szline[200]; 
			if(!bAdd)
				m_data.clear();
			int bFlag = 1;
			// 新弧段记录
			int bNormal_Raw = 0; // 0 继续记录Normal点; 1 继续记录Raw点
			int bPassAdd = 0;    // 标记该弧段数据是否可以被添加
			cstgSinglePassArc newPass;
			fgets(szline, 200, pCSTGFile);
			int count_add = 0;
			while(bFlag)
			{
				int nLineFlag = isValidNewPass(szline, pCSTGFile);
				if(nLineFlag == 0)
				{
					bFlag = false;
					// 2008/04/04添加, 否则最后一段会漏掉一组数据
					// 上一个弧段记录完毕
					if(bPassAdd == 1 && newPass.normalDataRecordList.size() > 0)
					{
						// 重新整理观测时间，避免出现午夜间断的情况
						for(size_t s_k = 1; s_k < newPass.normalDataRecordList.size(); s_k++)
						{
							if(newPass.normalDataRecordList[s_k].TimeofDay
							 - newPass.normalDataRecordList[s_k-1].TimeofDay < 0)
							{// 对 s_k 处进行连续化, 一个弧段仅可能出现一次该情况
								for(size_t s_j = s_k; s_j < newPass.normalDataRecordList.size(); s_j++)
									newPass.normalDataRecordList[s_j].TimeofDay += 86400;
								break;
							}
							//cout<<newPass.getTime(newPass.normalDataRecordList[s_k]).ToString()<<endl;
						}
						m_data.push_back(newPass);
						count_add++;
						//cout<<newPass.normalDataRecordList.size()<<endl;
					}
				}
				else if(nLineFlag == 1)
				{// 新弧段
					bFlag = true;
					// 上一个弧段记录完毕
					if(bPassAdd == 1 && newPass.normalDataRecordList.size() > 0)
					{
						// 重新整理观测时间，避免出现午夜间断的情况
						for(size_t s_k = 1; s_k < newPass.normalDataRecordList.size(); s_k++)
						{
							if(newPass.normalDataRecordList[s_k].TimeofDay
							 - newPass.normalDataRecordList[s_k-1].TimeofDay < 0)
							{// 对 s_k 处进行连续化, 一个弧段仅可能出现一次该情况
								for(size_t s_j = s_k; s_j < newPass.normalDataRecordList.size(); s_j++)
									newPass.normalDataRecordList[s_j].TimeofDay += 86400;
								break;
							}
							//cout<<newPass.getTime(newPass.normalDataRecordList[s_k]).ToString()<<endl;
						}
						m_data.push_back(newPass);
						count_add++;
						//cout<<newPass.normalDataRecordList.size()<<endl;
					}
					bPassAdd = 1;
					// 根据 HeaderRecord 判断弧段数据是否有效
					bNormal_Raw = 0;
					// 更新 NormalHeaderRecord
					fgets(szline, 200, pCSTGFile);
					if(!readLine_HeaderRecord(szline, newPass.normalHeaderRecord))
					{// 不通过进行标记，整个弧段的数据不能进行添加
						bPassAdd = 0;
					}
					// 清空 normalDataRecordList 和 rawDataRecordList
					newPass.normalDataRecordList.clear();
					newPass.rawDataRecordList.clear();
				}
				else if(nLineFlag == 2)
				{// 原始数据记录开始
					bFlag = true;
					bNormal_Raw = 1;
					// 更新 RawHeaderRecord
					fgets(szline, 200, pCSTGFile);
					if(!readLine_HeaderRecord(szline, newPass.rawHeaderRecord))
					{// 不通过进行标记, 整个弧段的数据不能进行添加
						bPassAdd = 0;
					}
				}
				else
				{
					bFlag = true;
					// 根据 bNormal_Raw 指示, 记录数据
					if(bNormal_Raw == 0)
					{// 继续记录Normal点
						cstgDataRecord DataRecord;
						if(readLine_DataRecord(szline, DataRecord))
							newPass.normalDataRecordList.push_back(DataRecord);
					}
					else
					{
					}
				}
				// 更新下一行
				fgets(szline, 200, pCSTGFile);
			}
			fclose(pCSTGFile);
			if(m_data.size() > 0 && count_add > 0)
				return true;
			else
				return false;
		}
	}
}
