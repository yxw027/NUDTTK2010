#include "meritSLRObsFile.hpp"
#include "cstgSLRObsFile.hpp"
#include "constDef.hpp"

#include <math.h>

namespace NUDTTK
{
	namespace SLR
	{
		meritSLRObsFile::meritSLRObsFile(void)
		{
		}

		meritSLRObsFile::~meritSLRObsFile(void)
		{
		}

		bool meritSLRObsFile::isValidEpochLine(string strLine, meritDataRecord& meritLine)
		{
			bool nFlag = true;
			meritDataRecord Line;  // meritDataRecord 的构造函数要有必要的初始化, 以保证不出现错误的判断, 因为在读到最后一行时, strLine的有效字符很短, line无法通过strLine来赋值
			// 字符串的长度要 >= 130
			if(strLine.length() < 130)
				return false;
			// 可能为负数的应该单独特别对待
			char szYear[3];
			char szDay[4];
			char szTimeofDay[13];
			char szStationID[5];
			char szCDPSysNum[3];
			char szCDPOSNum[3];
			char szAzimuth[8];
			char szElevation[7];
			char szLaserRange[13];
			char szLaserRangeSD[8];
			char szWavelength[5];
			char szSurfacePressure[6];
			char szSurfaceTemperature[5];
			char szSurfaceRelHumidity[4];
			char szTropCorrect[6];
			char szMassCenterCorrect[7];
			char szReceiveAmplitude[6];
			char szCalibrationSD[5];
			char szNormalWindowIndicator[2];
			char szCompressedRangeNum[5];
			char szEpochEvent[2];
			char szEpochTimeScale[2];
			char szAngleOriginIndicator[2];
			char szTRCIndicator[2];
			char szMCCIndicator[2];
			char szRACIndicator[2];
			char szSCMIndicator[2];
			char szCDSIndicator[2];
			char szSCFIndicator[2];
			char szFRNIndicator[2];
			char szCalibrationSysDelay[9];   
			char szCalibrationDelayShift[7];
			sscanf(strLine.c_str(),"%7c%2c%3c%12c%4c%2c%2c%7c%6c%12c%7c%4c%5c%4c%3c%5c%6c%5c%8c%6c%4c%1c%4c%1c%1c%1c%1c%1c%1c%1c%1c%1c%1c%c",
									 Line.szSatCOSPARID,
									 szYear,
									 szDay,
									 szTimeofDay,
									 szStationID,
									 szCDPSysNum,
									 szCDPOSNum,
									 szAzimuth,
									 szElevation,
									 szLaserRange,
									 szLaserRangeSD,
									 szWavelength,
									 szSurfacePressure,
									 szSurfaceTemperature,
									 szSurfaceRelHumidity,
									 szTropCorrect,
									 szMassCenterCorrect,
									 szReceiveAmplitude,
									 szCalibrationSysDelay,
									 szCalibrationDelayShift,
									 szCalibrationSD,
									 szNormalWindowIndicator,
									 szCompressedRangeNum,
									 szEpochEvent,
									 szEpochTimeScale,
									 szAngleOriginIndicator,
									 szTRCIndicator,
									 szMCCIndicator,
									 szRACIndicator,
									 szSCMIndicator,
									 szCDSIndicator,
									 szSCFIndicator,
									 szFRNIndicator,
									&Line.RFIndicator);
			Line.szSatCOSPARID[7]   = '\0';
			szYear[2]               = '\0';
			szDay[3]                = '\0';
			szTimeofDay[12]         = '\0';
			szStationID[4]          = '\0';
			szCDPSysNum[2]          = '\0';
			szCDPOSNum[2]           = '\0';
			szAzimuth[7]            = '\0';
			szElevation[6]          = '\0';
			szLaserRange[12]        = '\0';
			szLaserRangeSD[7]       = '\0';
			szWavelength[4]         = '\0';
			szSurfacePressure[5]    = '\0';
			szSurfaceTemperature[4] = '\0';
			szSurfaceRelHumidity[3] = '\0';
			szTropCorrect[5]        = '\0';
			szMassCenterCorrect[6]  = '\0';
			szReceiveAmplitude[5]   = '\0';
			szCalibrationSD[4]      = '\0';
			szNormalWindowIndicator[1] = '\0';
			szCompressedRangeNum[4]    = '\0';
			szEpochEvent[1]            = '\0';
			szEpochTimeScale[1]        = '\0';
			szAngleOriginIndicator[1]  = '\0';
			szTRCIndicator[1]          = '\0';
			szMCCIndicator[1]          = '\0';
			szRACIndicator[1]          = '\0';
			szSCMIndicator[1]          = '\0';
			szCDSIndicator[1]          = '\0';
			szSCFIndicator[1]          = '\0';
			szFRNIndicator[1]          = '\0';
			szCalibrationSysDelay[8]   = '\0';
			szCalibrationDelayShift[6] = '\0';
			cstgSLRObsFile::deleteStringZero(szCalibrationSysDelay);
			cstgSLRObsFile::deleteStringZero(szCalibrationDelayShift);
			sscanf(szCalibrationSysDelay,   "%8lf",  &Line.CalibrationSysDelay);
			sscanf(szCalibrationDelayShift, "%6lf",  &Line.CalibrationDelayShift);
			sscanf(szYear, "%2d",   &Line.Year);
			sscanf(szDay,  "%3d",   &Line.Day);
			sscanf(szTimeofDay,     "%12lf",   &Line.TimeofDay);
			sscanf(szStationID,     "%4d",     &Line.StationID);
			sscanf(szCDPSysNum,     "%2d",     &Line.CDPSysNum);
			sscanf(szCDPOSNum,      "%2d",     &Line.CDPOSNum);
			sscanf(szAzimuth,       "%7lf",    &Line.Azimuth);
			sscanf(szElevation,     "%6lf",    &Line.Elevation);
			sscanf(szLaserRange,    "%12lf",   &Line.LaserRange);
			sscanf(szLaserRangeSD,  "%7lf",    &Line.LaserRangeSD);
			sscanf(szWavelength,    "%4lf",    &Line.Wavelength);
			sscanf(szSurfacePressure,    "%5lf",   &Line.SurfacePressure);
			sscanf(szSurfaceTemperature, "%4lf",   &Line.SurfaceTemperature);
			sscanf(szSurfaceRelHumidity, "%3lf",   &Line.SurfaceRelHumidity);
			sscanf(szTropCorrect,        "%5lf",   &Line.TropCorrect);
			sscanf(szMassCenterCorrect,  "%6lf",   &Line.MassCenterCorrect);
			sscanf(szReceiveAmplitude,   "%5lf",   &Line.ReceiveAmplitude);
			sscanf(szCalibrationSD,          "%4lf",   &Line.CalibrationSD);
			sscanf(szNormalWindowIndicator,  "%1d",    &Line.NormalWindowIndicator);
			sscanf(szCompressedRangeNum,     "%4d",    &Line.CompressedRangeNum);
			sscanf(szEpochEvent,           "%1d",    &Line.EpochEvent);
			sscanf(szEpochTimeScale,       "%1d",    &Line.EpochTimeScale);
			sscanf(szAngleOriginIndicator, "%1d",    &Line.AngleOriginIndicator);
			sscanf(szTRCIndicator,         "%1d",    &Line.TRCIndicator);
			sscanf(szMCCIndicator,         "%1d",    &Line.MCCIndicator);
			sscanf(szRACIndicator,         "%1d",    &Line.RACIndicator);
			sscanf(szSCMIndicator,         "%1d",    &Line.SCMIndicator);
			sscanf(szCDSIndicator,         "%1d",    &Line.SCHIndicator);
			sscanf(szSCFIndicator,         "%1d",    &Line.SCFIndicator);
			sscanf(szFRNIndicator,         "%1d",    &Line.FRNIndicator);
			
			//Line.screenPrintf();
			nFlag = true;
			if( Line.Day > 366 || Line.Day < 0 )
				nFlag = false;
			if( Line.Year < 0 )
				nFlag = false;
			if( Line.TimeofDay < 0 )
				nFlag = false;
			// 判断条件待补充?
			if(nFlag)
				meritLine = Line;
			return nFlag;
		}

		bool meritSLRObsFile::open(string strMeritFileName)
		{
			FILE * pMeritFile = fopen(strMeritFileName.c_str(),"r+t");
			if(pMeritFile == NULL) 
				return false;
			char szline[200];     
			m_data.clear();
			while(!feof(pMeritFile))
			{
				if(fgets(szline, 200, pMeritFile))  
				{
					meritDataRecord Line;
					if(isValidEpochLine(szline, Line))
					{
						m_data.push_back(Line);
					}
				}
			}
			fclose(pMeritFile);

			if(m_data.size() > 0)
				return true;
			else
				return false;
		}

		bool meritSLRObsFile::append(string strMeritFileName)
		{
			meritSLRObsFile meritFile;
			if(!meritFile.open(strMeritFileName))
				return false;
			for(size_t s_i = 0; s_i < meritFile.m_data.size(); s_i++)
				m_data.push_back(meritFile.m_data[s_i]);
			return true;
		}

		UTC meritDataRecord::getTime()
		{
			int nB4Year = yearB2toB4(Year);
			UTC t(nB4Year, 1, 1, 0, 0, 0);
			t = t + (Day - 1) * 86400 ; // 先算天的(nDay是否从1开始计数)
			t = t + TimeofDay * 1.0E-7; // 再加上秒的部分
			return t;
		}

		void meritDataRecord::screenPrintf()
		{
			printf("******************************************************\n");
			printf("卫星COSPAR号: %s\n",szSatCOSPARID);
			printf("世纪年: %2d\n",Year);
			printf("年积日: %3d\n",Day);
			printf("秒: %f\n",TimeofDay * 0.1E-6);
			printf("测站编号: %4d\n",StationID);
			printf("真方位角: %f\n", Azimuth * 1.0E-4);
			printf("真高度角: %f\n", Elevation* 1.0E-4);
			printf("往返距离: %f 千米\n", LaserRange * 1.0E-12 * SPEED_LIGHT * 1.0E-3);
			printf("距离标准差: %f 米\n", LaserRangeSD * 1.0E-12 * SPEED_LIGHT);
			printf("激光波长: %f 纳米\n", Wavelength  * 10);
			printf("地表面压力: %f 毫帕\n", SurfacePressure * 0.1);
			printf("地表面温度: %f K\n", SurfaceTemperature * 0.1);
			printf("地表面相对湿度: %%%f\n", SurfaceRelHumidity);
			printf("往返对流层折射改正: %f 米\n", TropCorrect * 1.0E-12 * SPEED_LIGHT);
			printf("往返一周的质心改正: %f 米\n", MassCenterCorrect * 1.0E-12 * SPEED_LIGHT);
			printf("系统延迟: %f 米\n", CalibrationSysDelay * 1.0E-12 * SPEED_LIGHT);
			printf("系统延迟校正漂移: %f 米\n", CalibrationDelayShift * 1.0E-12 * SPEED_LIGHT);
			printf("系统延迟校正标准差: %f 米\n", CalibrationSD * 1.0E-12 * SPEED_LIGHT);

			switch(NormalWindowIndicator)
			{
			case 0:
				printf("标准点指示窗口：非标准点\n");
				break;
			case 2:
				printf("标准点指示窗口：10秒\n");
				break;
			case 3:
				printf("标准点指示窗口：15秒\n");
				break;
			case 4:
				printf("标准点指示窗口：20秒\n");
				break;
			case 5:
				printf("标准点指示窗口：30秒\n");
				break;
			case 6:
				printf("标准点指示窗口：1分钟\n");
				break;
			case 7:
				printf("标准点指示窗口：2分钟\n");
				break;
			case 8:
				printf("标准点指示窗口：3分钟\n");
				break;
			case 9:
				printf("标准点指示窗口：5分钟\n");
				break;
			default:
				printf("标准点指示窗口：*** %d\n",NormalWindowIndicator);
				break;
			}

			printf("压缩成标准点的原始数据的数目: %d 个\n", CompressedRangeNum);
			switch(EpochEvent)
			{
			case 0:
				printf("历元参考点：地面激光接收回波时刻\n");
				break;
			case 1:
				printf("历元参考点：卫星反射时刻\n");
				break;
			case 2:
				printf("历元参考点：地面发射时刻\n");
				break;
			case 3:
				printf("历元参考点：击中卫星时刻\n");
				break;
			default:
				printf("历元参考点：***%d\n",EpochEvent);
				break;
			}
			switch(EpochTimeScale)
			{
			case 0:
				printf("历元时间基准：UT0\n");
				break;
			case 1:
				printf("历元时间基准：UT1\n");
				break;
			case 2:
				printf("历元时间基准：UT2\n");
				break;
			case 3:
				printf("历元时间基准：UTC(USNO) 美国海军天文台\n");
				break;
			case 4:
				printf("历元时间基准：A.1(USNO)\n");
				break;
			case 5:
				printf("历元时间基准：TAI\n");
				break;
			case 6:
				printf("历元时间基准：A-S\n");
				break;
			case 7:
				printf("历元时间基准：UTC(BIH) 国际时间局\n");
				break;
			default:
				printf("历元时间基准：***%d\n",EpochTimeScale);
				break;
			}

			switch(AngleOriginIndicator)
			{
			case 0:
				printf("角度零点指示：未知\n");
				break;
			case 1:
				printf("角度零点指示：由测距计算得到\n");
				break;
			case 2:
				printf("角度零点指示：赋予（预报或操作员输入）\n");
				break;
			case 3:
				printf("角度零点指示：测量得到（被校正仪器的读数）\n");
				break;
			default:
				printf("角度零点指示：***%d\n",AngleOriginIndicator);
				break;
			}

			if( TRCIndicator == 0 )
				printf("对流层折射改正指示：数据已用 Marini-Murray 公式修正 \n");
			else
				printf("对流层折射改正指示：数据未作相应改正 \n");

			if( MCCIndicator == 0 )
				printf("质心改正指示：已作改正 \n");
			else
				printf("质心改正指示：未作改正 \n");

			if( RACIndicator == 0 )
				printf("接收幅度改正指示：已作改正 \n");
			else
				printf("接收幅度改正指示：未作改正 \n");

			switch(SCMIndicator)
			{
			case 0:
				printf("系统校正方法指示：外部校正\n");
				break;
			case 1:
				printf("系统校正方法指示：内部校正\n");
				break;
			case 2:
				printf("系统校正方法指示：分段校正\n");
				break;
			case 3:
				printf("系统校正方法指示：未考虑校正\n");
				break;
			default:
				printf("系统校正方法指示：***%d\n", SCMIndicator);
				break;
			}

			switch(SCHIndicator)
			{
			case 0:
				printf("延迟校正漂移指示：测前到测后的漂移\n");
				break;
			case 1:
				printf("延迟校正漂移指示：峰值到峰值的漂移\n");
				break;
			default:
				printf("延迟校正漂移指示：***%d\n",SCHIndicator);
				break;
			}

			printf("系统结构标记指示：%d\n", SCFIndicator);
			printf("格式修正指示：%d\n", FRNIndicator);
			printf("释放信号指示：%c\n", RFIndicator);
			printf("******************************************************\n");
		}
	}
}
