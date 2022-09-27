#include "Rinex2_1_NavFile.hpp"
#include "Rinex2_1_ObsFile.hpp"
#include "TimeCoordConvert.hpp"
#include "SP3File.hpp"
#include "RuningInfoFile.hpp"

namespace NUDTTK
{
	Rinex2_1_NavFile::Rinex2_1_NavFile(void)
	{
		m_typeSatSystem = 0;         // 默认GPS系统
		if(m_typeSatSystem == 1)
		{
			m_EARTH_W  = CGCS2000_EARTH_W;  // CGCS2000
			m_GM_EARTH = CGCS2000_GM_EARTH; // CGCS2000
		}
		else
		{
			m_EARTH_W  = WGS84_EARTH_W;  // WGS84
			m_GM_EARTH = WGS84_GM_EARTH; // WGS84
		}
	}

	Rinex2_1_NavFile::~Rinex2_1_NavFile(void)
	{
	}

	void Rinex2_1_NavFile::clear()
	{
		m_header = Rinex2_1_NavHeader::Rinex2_1_NavHeader();
		m_data.clear();
	}

	bool Rinex2_1_NavFile::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}

	int Rinex2_1_NavFile::isValidEpochLine(string strLine, FILE * pNavfile)
	{
		GPST tmEpoch;
		// 下面几种数据用int型, 避免因为strLine的格式问题引起sscanf函数发生错误
		int nSatPRN = -1;                        
		if(pNavfile != NULL) // 判断是否为文件末尾
		{
			if(feof(pNavfile))
				return 0;
		}
		sscanf(strLine.c_str(),"%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%5lf",
			                   &nSatPRN,
							   &tmEpoch.year,
							   &tmEpoch.month,
							   &tmEpoch.day,
							   &tmEpoch.hour,
							   &tmEpoch.minute,
							   &tmEpoch.second);
		int nFlag = 1;
		if(tmEpoch.month > 12 || tmEpoch.month < 0)
			nFlag = 2;
		if(tmEpoch.day > 31 || tmEpoch.day < 0)
			nFlag = 2;
		if(tmEpoch.hour > 24 || tmEpoch.hour < 0)
			nFlag = 2;
		if(tmEpoch.minute > 60 || tmEpoch.minute < 0)
			nFlag = 2;
		if(tmEpoch.second > 60 || tmEpoch.second < 0)
			nFlag = 2;
		if(nSatPRN > MAX_PRN_GPS || nSatPRN < 0)
			nFlag = 2;
		return nFlag;
	}

	// 子程序名称： open   
	// 功能：观测数据解析 
	// 变量类型：strNavfileName : 观测数据文件路径
	// 输入：strNavfileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/10/05
	// 版本时间：2007/10/05
	// 修改记录：
	// 备注： 
	bool Rinex2_1_NavFile::open(string  strNavfileName)
	{// 通过后缀名来判断
		if(isWildcardMatch(strNavfileName.c_str(), "*.*N", true))
		{
			m_typeSatSystem = 0;         // GPS系统
			m_EARTH_W  = WGS84_EARTH_W;  // WGS84
			m_GM_EARTH = WGS84_GM_EARTH; // WGS84
		}
		if(isWildcardMatch(strNavfileName.c_str(), "*.*c", true)|| isWildcardMatch(strNavfileName.c_str(), "*.*r", true))//2014年后北斗导航文件名调整为r
		{
			m_typeSatSystem = 1;            // BD系统
			m_EARTH_W  = CGCS2000_EARTH_W;  // CGCS2000
			m_GM_EARTH = CGCS2000_GM_EARTH; // CGCS2000
		}

		if(!(isWildcardMatch(strNavfileName.c_str(), "*.*N", true) || isWildcardMatch(strNavfileName.c_str(), "*.*c", true) || isWildcardMatch(strNavfileName.c_str(), "*.*r", true)))
		{
			return false;
		}

		FILE * pNavfile = fopen(strNavfileName.c_str(), "r+t");
		if(pNavfile == NULL) 
			return false;
		m_header = Rinex2_1_NavHeader::Rinex2_1_NavHeader();
		// 读取文件头
		int bFlag = 1;
		while(bFlag)
		{
			char line[100];
			fgets(line, 100, pNavfile);
			string strLineMask = line;
			string strLine     = line;
			strLineMask.erase(0, 60);     // 从第0个元素开始，删除60个
			// 剔除\n
			size_t nPos_n = strLineMask.find('\n');
			if(nPos_n < strLineMask.length())
				strLineMask.erase(nPos_n, 1);
			// 超过20位，截取20位
			while(strLineMask.length() > 20)
				strLineMask.erase(strLineMask.length() - 1, 1);
			// 补齐20位
			if(strLineMask.length() < 20) // strLineMask.length 包含最后的'\0'
				strLineMask.append(20 - strLineMask.length(), ' ');
			if(strLineMask == Rinex2_1_MaskString::szVerType)
			{
				strLine.copy(m_header.szRinexVersion, 20, 0);
				strLine.copy(m_header.szFileType, 20, 20);
			}
			else if(strLineMask == Rinex2_1_MaskString::szPgmRunDate)
			{
				strLine.copy(m_header.szFileDate, 20, 40);
			}
			else if(strLineMask == Rinex2_1_MaskString::szIonAlpha)
			{
				stringRaplaceA2B(line,'D','E');
				sscanf(line,"%*2c%12lf%12lf%12lf%12lf",
					        &m_header.pdIonAlpha[0],
							&m_header.pdIonAlpha[1],
							&m_header.pdIonAlpha[2],
							&m_header.pdIonAlpha[3]);
			}
			else if(strLineMask == Rinex2_1_MaskString::szIonBeta)
			{
				stringRaplaceA2B(line, 'D', 'E');
				sscanf(line,"%*2c%12lf%12lf%12lf%12lf",
					        &m_header.pdIonBeta[0],
							&m_header.pdIonBeta[1],
							&m_header.pdIonBeta[2],
							&m_header.pdIonBeta[3]);
			}
			else if(strLineMask == Rinex2_1_MaskString::szDeltaUTC)
			{
				stringRaplaceA2B(line, 'D', 'E');
				sscanf(line,"%*3c%19lf%19lf%9d%9d",
					        &m_header.deltaUTC.dA0,
							&m_header.deltaUTC.dA1,
							&m_header.deltaUTC.lnT,
							&m_header.deltaUTC.lnW);
			}
			else if(strLineMask == Rinex2_1_MaskString::szLeapSec)
			{
				sscanf(line,"%6d", &m_header.lnLeapSeconds);
			}
			else if(strLineMask == Rinex2_1_MaskString::szEndOfHead)
			{
				bFlag = false;
			}
			else // Comment等不作处理
			{
			}
		}
		// 观测数据
		bFlag = TRUE;
		m_data.clear();
		int k = 0;
		char line[100];
		fgets(line, 100, pNavfile);
		NavDatumList navDatumList;
		while(bFlag)
		{
			string strLine = line;
			int nFlag = isValidEpochLine(strLine, pNavfile);
			if(nFlag == 0)      // 文件末尾
			{
				bFlag=false;
			}
			else if(nFlag == 1) // 找到新时刻的数据段
			{
				k++;
				Rinex2_1_NavDatum navDatum;
				// PRN / EPOCH / SV CLK
				stringRaplaceA2B(strLine, 'D', 'E');
				sscanf(strLine.c_str(),"%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%5lf%19lf%19lf%19lf",
					                   &navDatum.bySatPRN,
									   &navDatum.tmTimeOfClock.year,
					                   &navDatum.tmTimeOfClock.month,
									   &navDatum.tmTimeOfClock.day,
									   &navDatum.tmTimeOfClock.hour,
									   &navDatum.tmTimeOfClock.minute,
									   &navDatum.tmTimeOfClock.second,
					                   &navDatum.dSVClockBias,
									   &navDatum.dSVClockDrift,
									   &navDatum.dSVClockDriftRate);
				navDatum.tmTimeOfClock.year = yearB2toB4(navDatum.tmTimeOfClock.year);
				// BROADCAST ORBIT - 1
				fgets(line, 100, pNavfile);
				stringRaplaceA2B(line, 'D', 'E');
				strLine = line;
				sscanf(strLine.c_str(),"%*3c%19lf%19lf%19lf%19lf",
					                   &navDatum.dIODE,
									   &navDatum.dCrs,
									   &navDatum.dDetla_n,
									   &navDatum.dM0);
				// BROADCAST ORBIT - 2
				fgets(line, 100, pNavfile);
				stringRaplaceA2B(line, 'D', 'E');
				strLine = line;
				sscanf(strLine.c_str(),"%*3c%19lf%19lf%19lf%19lf",
					                   &navDatum.dCuc,
									   &navDatum.dEccentricity,
									   &navDatum.dCus,
									   &navDatum.dSqrt_A);
				// BROADCAST ORBIT - 3
				fgets(line, 100, pNavfile);
				stringRaplaceA2B(line, 'D', 'E');
				strLine = line;
				sscanf(strLine.c_str(),"%*3c%19lf%19lf%19lf%19lf",
					                   &navDatum.dTOE,
									   &navDatum.dCic,
									   &navDatum.dOMEGA,
									   &navDatum.dCis);
				// BROADCAST ORBIT - 4
				fgets(line, 100, pNavfile);
				stringRaplaceA2B(line, 'D', 'E');
				strLine = line;
				sscanf(strLine.c_str(),"%*3c%19lf%19lf%19lf%19lf",
					                   &navDatum.d_i0,
									   &navDatum.dCrc,
									   &navDatum.d_omega,
									   &navDatum.dOMEGADOT);
				// BROADCAST ORBIT - 5
				fgets(line, 100, pNavfile);
				stringRaplaceA2B(line, 'D', 'E');
				strLine = line;
				sscanf(strLine.c_str(),"%*3c%19lf%19lf%19lf%19lf",
					                   &navDatum.didot,
									   &navDatum.dCodesOnL2Channel,
									   &navDatum.dWeek,
									   &navDatum.dL2PDataFlag);
				// BROADCAST ORBIT - 6
				fgets(line, 100, pNavfile);
				stringRaplaceA2B(line, 'D', 'E');
				strLine     = line;
				sscanf(strLine.c_str(),"%*3c%19lf%19lf%19lf%19lf",
					                   &navDatum.dSVAccuracy,
									   &navDatum.dSVHealth,
									   &navDatum.dTGD,
									   &navDatum.dIODC);
				// BROADCAST ORBIT - 7
				fgets(line, 100, pNavfile);
				stringRaplaceA2B(line, 'D', 'E');
				strLine = line;
				sscanf(strLine.c_str(),"%*3c%19lf%19lf%19lf%19lf",
					                   &navDatum.dTransmissionTimeOfMessage,
									   &navDatum.dFitInterval,
									   &navDatum.dSpare1,
									   &navDatum.dSpare2);
				navDatumList.push_back(navDatum);
				fgets(line, 100, pNavfile);
			}
			else
			{
				fgets(line, 100, pNavfile);
			}
		}
		fclose(pNavfile);

		// 统计获得 m_datalist_sat 20071005
		NavDatumList navDatumList_sat[MAX_PRN_GPS];
		for(int i = 0; i < MAX_PRN_GPS; i++)
			navDatumList_sat[i].clear();
		for(size_t s_i = 0; s_i < navDatumList.size(); s_i++)
		{
			if(navDatumList[s_i].bySatPRN < MAX_PRN_GPS)
				navDatumList_sat[navDatumList[s_i].bySatPRN].push_back(navDatumList[s_i]);
		}
		for(int i = 0; i < MAX_PRN_GPS; i++)
		{
			if(navDatumList_sat[i].size() > 0)
			{
				m_data.insert(NavSatMap::value_type(i, navDatumList_sat[i]));
			}
		}
		return true;
	}

	void Rinex2_1_NavFile::writedouble(FILE* pNavfile, double value)
	{
		if(pNavfile)
		{
			if(value != DBL_MAX)
			{
				char szValue[21];
				string strValue;
				sprintf(szValue, "%20.12E", value);
				stringEraseFloatZero(szValue, strValue);
				stringRaplaceA2B(strValue, 'E', 'D'); // 2012/04/24
				fprintf(pNavfile, "%19s", strValue.c_str());
			}
			else
				fprintf(pNavfile, "%-19s", " ");
		}
	}

	bool Rinex2_1_NavFile::write(string strNavfileName)
	{
		if(isEmpty())
			return false;
		// 写文件头
		NavSatMap::const_iterator it_0 = m_data.begin();
		int n1  = getIntBit(it_0->second[0].tmTimeOfClock.year, 0);
		int n10 = getIntBit(it_0->second[0].tmTimeOfClock.year, 1);
		char strFileExp[5];
		if(m_typeSatSystem == 1)
			sprintf(strFileExp, ".%1d%1dC", n10, n1);
		else
            sprintf(strFileExp, ".%1d%1dN", n10, n1);

		strNavfileName = strNavfileName + strFileExp;
		FILE* pNavfile = fopen(strNavfileName.c_str(), "w+");
		fprintf(pNavfile,"%20s%20s%-20s%20s\n",
			             m_header.szRinexVersion,
						 m_header.szFileType,
						 " ",
						 Rinex2_1_MaskString::szVerType);
		fprintf(pNavfile,"%-40s%20s%20s\n"," ",
			             m_header.szFileDate,
						 Rinex2_1_MaskString::szPgmRunDate);

		char szValue[14];
		string strValue;
		

		/*fprintf(pNavfile,"%-2s%12.3E%12.3E%12.3E%12.3E%-10s%20s\n",
			             " ",
						 m_header.pdIonAlpha[0],
						 m_header.pdIonAlpha[1],
			             m_header.pdIonAlpha[2],
						 m_header.pdIonAlpha[3],
						 " ",
						 Rinex2_1_MaskString::szIonAlpha);*/

		fprintf(pNavfile,"%-2s", " ");

		sprintf(szValue, "%13.4E", m_header.pdIonAlpha[0]);
	    stringEraseFloatZero(szValue, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%12s", strValue.c_str());

		sprintf(szValue, "%13.4E", m_header.pdIonAlpha[1]);
	    stringEraseFloatZero(szValue, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%12s", strValue.c_str());

		sprintf(szValue, "%13.4E", m_header.pdIonAlpha[2]);
	    stringEraseFloatZero(szValue, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%12s", strValue.c_str());

		sprintf(szValue, "%13.4E", m_header.pdIonAlpha[3]);
	    stringEraseFloatZero(szValue, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%12s", strValue.c_str());

		fprintf(pNavfile,"%-10s%20s\n", " ", Rinex2_1_MaskString::szIonAlpha);


		/*fprintf(pNavfile,"%-2s%12.3E%12.3E%12.3E%12.3E%-10s%20s\n",
			             " ",
						 m_header.pdIonBeta[0],
						 m_header.pdIonBeta[1],
			             m_header.pdIonBeta[2],
						 m_header.pdIonBeta[3],
						 " ",
						 Rinex2_1_MaskString::szIonBeta);*/
		fprintf(pNavfile,"%-2s", " ");

		sprintf(szValue, "%13.4E", m_header.pdIonBeta[0]);
	    stringEraseFloatZero(szValue, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%12s", strValue.c_str());

		sprintf(szValue, "%13.4E", m_header.pdIonBeta[1]);
	    stringEraseFloatZero(szValue, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%12s", strValue.c_str());

		sprintf(szValue, "%13.4E", m_header.pdIonBeta[2]);
	    stringEraseFloatZero(szValue, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%12s", strValue.c_str());

		sprintf(szValue, "%13.4E", m_header.pdIonBeta[3]);
	    stringEraseFloatZero(szValue, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%12s", strValue.c_str());

		fprintf(pNavfile,"%-10s%20s\n", " ", Rinex2_1_MaskString::szIonBeta);


		/*fprintf(pNavfile,"%-3s%19.11E%19.11E%9d%9d%-1s%20s\n",
			             " ",
						 m_header.deltaUTC.dA0,
						 m_header.deltaUTC.dA1,
			             m_header.deltaUTC.lnT,
						 m_header.deltaUTC.lnW,
						 " ",
						 Rinex2_1_MaskString::szDeltaUTC);*/

		char szValue2[21];
		fprintf(pNavfile,"%-3s", " ");

		sprintf(szValue2, "%20.12E", m_header.deltaUTC.dA0);
	    stringEraseFloatZero(szValue2, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%19s", strValue.c_str());

		sprintf(szValue2, "%20.12E", m_header.deltaUTC.dA1);
	    stringEraseFloatZero(szValue2, strValue);
	    stringRaplaceA2B(strValue, 'E', 'D'); 
		fprintf(pNavfile, "%19s", strValue.c_str());

		fprintf(pNavfile,"%9d%9d%-1s%20s\n",
			             m_header.deltaUTC.lnT,
						 m_header.deltaUTC.lnW,
						 " ",
						 Rinex2_1_MaskString::szDeltaUTC);

		if(m_header.lnLeapSeconds != INT_MAX)
			fprintf(pNavfile,"%6d%-54s%20s\n",
			                 m_header.lnLeapSeconds,
							 " ",
							 Rinex2_1_MaskString::szLeapSec);
		fprintf(pNavfile,"%-60s%20s\n",
			             " ",
						 Rinex2_1_MaskString::szEndOfHead);
		
		// 写观测数据
		for(NavSatMap::iterator it = m_data.begin(); it != m_data.end(); ++it)
		{
			for(size_t s_i = 0; s_i < it->second.size(); s_i++)
			{
				Rinex2_1_NavDatum navDatum = it->second[s_i];
				// PRN / EPOCH / SV CLK
				fprintf(pNavfile,"%2d %1d%1d %2d %2d %2d %2d%5.1f",
					             navDatum.bySatPRN,
								 getIntBit(navDatum.tmTimeOfClock.year, 1),
								 getIntBit(navDatum.tmTimeOfClock.year, 0),
								 navDatum.tmTimeOfClock.month,
								 navDatum.tmTimeOfClock.day,
								 navDatum.tmTimeOfClock.hour,
								 navDatum.tmTimeOfClock.minute,
								 navDatum.tmTimeOfClock.second);
				writedouble(pNavfile, navDatum.dSVClockBias);
				writedouble(pNavfile, navDatum.dSVClockDrift);
				writedouble(pNavfile, navDatum.dSVClockDriftRate);
				fprintf(pNavfile, "\n");
				// BROADCAST ORBIT - 1
				fprintf(pNavfile, "%-3s", " ");
				writedouble(pNavfile, navDatum.dIODE);
				writedouble(pNavfile, navDatum.dCrs);
				writedouble(pNavfile, navDatum.dDetla_n);
				writedouble(pNavfile, navDatum.dM0);
				fprintf(pNavfile, "\n");
				// BROADCAST ORBIT - 2
				fprintf(pNavfile, "%-3s", " ");
				writedouble(pNavfile, navDatum.dCuc);
				writedouble(pNavfile, navDatum.dEccentricity);
				writedouble(pNavfile, navDatum.dCus);
				writedouble(pNavfile, navDatum.dSqrt_A);
				fprintf(pNavfile, "\n");
				// BROADCAST ORBIT - 3
				fprintf(pNavfile, "%-3s", " ");
				writedouble(pNavfile, navDatum.dTOE);
				writedouble(pNavfile, navDatum.dCic);
				writedouble(pNavfile, navDatum.dOMEGA);
				writedouble(pNavfile, navDatum.dCis);
				fprintf(pNavfile, "\n");
				// BROADCAST ORBIT - 4
				fprintf(pNavfile, "%-3s", " ");
				writedouble(pNavfile, navDatum.d_i0);
				writedouble(pNavfile, navDatum.dCrc);
				writedouble(pNavfile, navDatum.d_omega);
				writedouble(pNavfile, navDatum.dOMEGADOT);
				fprintf(pNavfile, "\n");
				// BROADCAST ORBIT - 5
				fprintf(pNavfile, "%-3s", " ");
				writedouble(pNavfile, navDatum.didot);
				writedouble(pNavfile, navDatum.dCodesOnL2Channel);
				writedouble(pNavfile, navDatum.dWeek);
				writedouble(pNavfile, navDatum.dL2PDataFlag);
				fprintf(pNavfile, "\n");
				// BROADCAST ORBIT - 6
				fprintf(pNavfile, "%-3s", " ");
				writedouble(pNavfile, navDatum.dSVAccuracy);
				writedouble(pNavfile, navDatum.dSVHealth);
				writedouble(pNavfile, navDatum.dTGD);
				writedouble(pNavfile, navDatum.dIODC);
				fprintf(pNavfile,"\n");
				// BROADCAST ORBIT - 7
				fprintf(pNavfile,"%-3s"," ");
				writedouble(pNavfile, navDatum.dTransmissionTimeOfMessage);
				writedouble(pNavfile, navDatum.dFitInterval);
				writedouble(pNavfile, navDatum.dSpare1);
				writedouble(pNavfile, navDatum.dSpare2);
				fprintf(pNavfile,"\n");
			}
		}
		fclose(pNavfile);
		return true;
	}

	// 子程序名称： getEphemeris   
	// 功能：根据T, 获得第 nPRN 颗 GPS 卫星的广播星历单元(最邻近规则)
	// 变量类型：T                   : GPS时间
	//           nPRN                : GPS卫星的PRN号
	//           navDatum            : 第nPRN颗GPS卫星的导航广播星历
	//           threshold_span_max  : 广播星历间隔的阈值, 如果超过该阈值, 则对应的卫星星历外推无效
	// 输入：t, nPRN, threshold_span_max
	// 输出：navDatum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/10/05
	// 版本时间：2007/10/05
	// 修改记录：1. 2012/04/24, 由谷德峰修改, 增加判断北斗/GPS系统判断, 认为北斗的广播星历沿用GPS时间系统, 坐标系选用CGCS2000
	// 备注： 
	bool Rinex2_1_NavFile::getEphemeris(DayTime T, int nPRN, Rinex2_1_NavDatum& navDatum, double threshold_span_max, bool onForwad)
	{
		// 获得 GPS 周秒
		WeekTime T_WT;
		// 判断北斗/GPS系统标记
		//if(m_typeSatSystem == 1) // 2012/04/24
		//	T_WT = TimeCoordConvert::BDT2WeekTime(T);
		//else
			T_WT = TimeCoordConvert::GPST2WeekTime(T);
		// 本颗 GPS 导航卫星的观测序列
		NavSatMap::const_iterator it;
		size_t nCount;
		if((it = m_data.find(nPRN)) != m_data.end())
		{
			nCount = it->second.size();
			if(nCount <= 0)
				return false;
		}
		else
			return false;

		int index = -1;
		// 时间最临近法则, 参考时间为轨道参考历元 TOE 
		double dSpanSeconds = 100000000;
		for(size_t s_i = 0; s_i < nCount; s_i++)
		{
			Rinex2_1_NavDatum navDatum_i   = it->second[s_i];
			WeekTime TOE       = navDatum_i.getWeekTime_toe();
			//double dSpanSeconds_i = T_WT - TOE;
			double dSpanSeconds_i = T - navDatum_i.tmTimeOfClock;
			if(fabs(dSpanSeconds_i) < dSpanSeconds && ((!onForwad && dSpanSeconds_i >= 0.0) || onForwad)) // 2016/08/07, 为防止广播星历无法输出起始时间以前的数据 
			{// 保证采用星历的时刻在观测时刻之前
				index = int(s_i);
				dSpanSeconds = fabs(dSpanSeconds_i);
			}
		}
		// 如果超过该阈值，则对应的卫星星历外推无效
		if(index != -1 && dSpanSeconds <= threshold_span_max)
		{
			navDatum = it->second[index];
			/*char info[200];
			sprintf(info, "G%02d  %2d[%8.1lf]  %s.", nPRN, index, dSpanSeconds, T.toString().c_str());
			RuningInfoFile::Add(info);*/
			return true;
		}
		else
		{
			return false;
		}
	}

	// 子程序名称： getEphemeris   
	// 功能：根据T, 获得第nPRN颗GPS卫星的广播星历(位置、钟差)
	// 变量类型：T           : GPS时间
	//           nPRN        : GPS卫星的PRN号
	//           pos         : GPS的位置和钟差
	// 输入：t, nPRN
	// 输出：NavElement
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/01/25
	// 版本时间：2008/01/25
	// 修改记录：
	// 备注：
	bool Rinex2_1_NavFile::getEphemeris(DayTime T, int nPRN, POSCLK &posclk, double threshold_span_max, bool onForwad)
	{
		Rinex2_1_NavDatum navDatum;
		if(getEphemeris(T, nPRN, navDatum, threshold_span_max, onForwad))
		{// 本颗 GPS 卫星存在有效星历, 计算其轨道位置和钟差
			posclk = getPosition(T, navDatum);
			return true;
		}
		else
			return false;
	}

	// 子程序名称： getClock   
	// 功能：根据GPS卫星广播星历, 获得某颗GPS卫星的钟差改正值
	// 变量类型：T           : 参考历元时间
	//           navDatum    : 某颗GPS卫星的导航广播星历
	//           dClock      : 钟差改正值(秒)
	// 输入：T, NavElement
	// 输出：dClock
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/01/25
	// 版本时间：2008/01/25
	// 修改记录：
	// 备注：
	double Rinex2_1_NavFile::getClock(DayTime T, Rinex2_1_NavDatum navDatum)
	{
		// 此处不进行一阶相对论效应的修正, 因为定位解算的时候已经进行考虑
		//WeekTime T_WT;
		// 判断北斗/GPS系统标记
		//if(m_typeSatSystem == 1) // 2012/04/24
		//	T_WT = TimeCoordConvert::BDT2WeekTime(T);
		//else
		//	T_WT = TimeCoordConvert::GPST2WeekTime(T);
		//double tk = T_WT - navDatum.getWeekTime_toe();
		double tk = T - navDatum.tmTimeOfClock; // 20160516, 鞠冰修改
		double dClock = navDatum.dSVClockBias + navDatum.dSVClockDrift * tk + navDatum.dSVClockDriftRate * pow(tk,2);
		return dClock;
	}

	// 子程序名称： getPosition   
	// 功能：根据 GPS 卫星广播星历，获得某颗 GPS 卫星的轨道位置和钟差
	// 变量类型：T           : 参考历元时间
	//           navDatum    : 某颗GPS卫星的导航广播星历
	//           pos         : GPS卫星的轨道位置(米)和钟差(秒)
	// 输入：t, navDatum
	// 输出：pos
	// 语言：C++
    // 创建者：谷德峰
	// 创建时间：2007/10/03
	// 版本时间：
	// 修改记录：1. 2008/01/22, 由谷德峰修改, 将位置和钟差计算合并
	//           2. 2009/11/03, 由谷德峰修改, 采用WGS84_GM_EARTH常量
	// 备注：
	POSCLK Rinex2_1_NavFile::getPosition(DayTime T, Rinex2_1_NavDatum navDatum)
	{
		// 第一步: 计算卫星运行的角速度
		//const double GM_EARTH_WGS84 = 3.986005E+14; // 2009/11/03 修改
		double n0 = sqrt(m_GM_EARTH) / (pow(navDatum.dSqrt_A, 3));
		double n  = n0 + navDatum.dDetla_n;
		//第二步: 计算归化时间
		// 获得GPS周秒
		WeekTime T_WT;
		// 判断北斗/GPS系统标记
		//if(m_typeSatSystem == 1) // 2012/04/24
		//	T_WT = TimeCoordConvert::BDT2WeekTime(T);
		//else
			T_WT = TimeCoordConvert::GPST2WeekTime(T);

		double tk = T_WT - navDatum.getWeekTime_toe(); // 20160516, 鞠冰修改
		/*if( tk > 302400)
			tk -= 604800;
		else if( tk <- 302400 )
			tk += 604800;*/
		//第三步: 计算观测瞬时的卫星平近点角
		double Mk = navDatum.dM0 + n * tk;
		//第四步: 计算偏近点角（弧度）
		double Ek   = Mk;
		double Ek_1 = Mk - 1;
		while( fabs(Ek - Ek_1) > 1.0E-10 )
		{
			Ek_1 = Ek;
			Ek   = Mk + navDatum.dEccentricity * sin(Ek);
		}
		//第五步: 计算真近点角（弧度）
		double fk = atan2( sqrt(1 - pow(navDatum.dEccentricity, 2)) * sin(Ek),
						   cos(Ek) - navDatum.dEccentricity );
		//第六步: 计算升交点角距
		double faik = fk + navDatum.d_omega;
		//第七步: 计算摄动改正项
		double cos2faik = cos(2 * faik);
		double sin2faik = sin(2 * faik);
		double detla_u = navDatum.dCuc * cos2faik + navDatum.dCus * sin2faik;
		double detla_r = navDatum.dCrc * cos2faik + navDatum.dCrs * sin2faik;
		double detla_i = navDatum.dCic * cos2faik + navDatum.dCis * sin2faik;
		//第八步: 计算经过摄动改正的升交距角uk，卫星径矢rk，和轨道倾角ik
		double uk = faik + detla_u;
		double rk = pow(navDatum.dSqrt_A, 2) * (1 - navDatum.dEccentricity * cos(Ek)) + detla_r;
		double ik = navDatum.d_i0 + detla_i + navDatum.didot * tk;
		//第九步: 计算卫星在轨道平面上的位置
		double xk = rk * cos(uk);
		double yk = rk * sin(uk);
		//第十步: 计算观测时刻的升交点经度
		//const double EARTH_W_WGS84 = 7.2921151467E-5;
		//double OMEGAk = navDatum.dOMEGA + (navDatum.dOMEGADOT - m_EARTH_W) * tk - m_EARTH_W * navDatum.dTOE;
		//第十一步: 计算卫星在地心固定坐标系中的位置
		POSCLK pos;
		if(m_typeSatSystem == 1 && navDatum.bySatPRN <= 5)
		{
			Matrix  POS,new_POS,RX,RZ;
			double  theta_E = m_EARTH_W * tk;
			double  theta_R = -5*PI/180;
			POS.Init(3,1);
			new_POS.Init(3,1);
			RX.Init(3,3);
			RZ.Init(3,3);
			double OMEGAk = navDatum.dOMEGA + navDatum.dOMEGADOT * tk - m_EARTH_W * navDatum.dTOE;
			//第十一步: 计算卫星在惯性系中的位置
			POS.SetElement(0,0,xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk));
			POS.SetElement(1,0,xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk));
			POS.SetElement(2,0,yk * sin(ik));
			RX.SetElement(0,0,1);
			RX.SetElement(1,1,cos(theta_R));
			RX.SetElement(1,2,sin(theta_R));
			RX.SetElement(2,1,-sin(theta_R));
			RX.SetElement(2,2,cos(theta_R));
			RZ.SetElement(2,2,1);
			RZ.SetElement(0,0,cos(theta_E));
			RZ.SetElement(0,1,sin(theta_E));
			RZ.SetElement(1,0,-sin(theta_E));
			RZ.SetElement(1,1,cos(theta_E));
			new_POS = RZ * RX * POS;
			pos.x = new_POS.GetElement(0,0);
			pos.y = new_POS.GetElement(1,0);
			pos.z = new_POS.GetElement(2,0);
		}
		else
		{
			double OMEGAk = navDatum.dOMEGA + (navDatum.dOMEGADOT - m_EARTH_W) * tk - m_EARTH_W * navDatum.dTOE; 
			//第十一步: 计算卫星在地心固定坐标系中的位置		
			pos.x = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
			pos.y = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
			pos.z = yk * sin(ik);
		}		
		pos.clk = getClock(T, navDatum);
		return pos;
	}

	// 子程序名称： exportSP3File   
	// 功能：导出 sp3 格式星历文件
	// 变量类型：strSP3fileName  : 文件名称
	//           T_Begin         : 星历开始时间
	//           T_End           : 星历结束时间
	//           spanSeconds     : 星历相邻时间点的时间间隔, 默认5分钟
	// 输入：T_Begin, T_End, spanSeconds
	// 输出：
	// 语言：C++
	// 创建时间：2008/01/22
	// 版本时间：
	// 修改记录：1. 2012/05/23, 由谷德峰修改, 将北斗的精密星历输出时间由GPS时间调整为BDT时间系统, 因为地面观测系统的时间为BDT
	// 备注：

	void Rinex2_1_NavFile::exportSP3File(string strSP3fileName, DayTime T_Begin, DayTime T_End, double spanSeconds, double threshold_span_max, bool onForwad)
	{
		SP3File sp3file;
		BYTE pbySatList[MAX_PRN_GPS]; // 卫星列表
		for(int i = 0; i < MAX_PRN_GPS; i++)
			pbySatList[i] = 0;

		DayTime T = T_Begin; // 调整到BDT时间
		int k = 0;
		while( T_End - T > 0 )
		{
			SP3Epoch sp3Epoch;
			sp3Epoch.t = T;
			sp3Epoch.sp3.clear();
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				POSCLK pos;
				if(getEphemeris(T, i, pos, threshold_span_max, onForwad))
				{
					pbySatList[i] = 1;
					SP3Datum sp3Datum;
					sp3Datum.pos.x  = pos.x * 0.001;     // 换算成米
					sp3Datum.pos.y  = pos.y * 0.001;     // 换算成米
					sp3Datum.pos.z  = pos.z * 0.001;     // 换算成米
					sp3Datum.clk    = pos.clk * 1.0E+06; // 换算微秒
					char szSatName[5]; // 2014/03/23, 由谷德峰修改, 兼容混合系统
					if(m_typeSatSystem == 1)
						sprintf(szSatName, "C%02d ", i);
					else
						sprintf(szSatName, "G%02d ", i);
					szSatName[4] = '\0';
					sp3Epoch.sp3.insert(SP3SatMap::value_type(szSatName, sp3Datum));
				}
			}
			if(sp3Epoch.sp3.size() > 0)
				sp3file.m_data.push_back(sp3Epoch);
			T = T + spanSeconds;
		}
		// 书写文件头
		// Line 1
		sprintf(sp3file.m_header.szSP3Version, "#a");
		sprintf(sp3file.m_header.szPosVelFlag, "P");
		sp3file.m_header.tmStart = T_Begin;
		sp3file.m_header.nNumberofEpochs = 96;
		sprintf(sp3file.m_header.szDataType, "ORBIT");
		sprintf(sp3file.m_header.szCoordinateSys, "IGS00");
		sprintf(sp3file.m_header.szOrbitType, "HLM");
		sprintf(sp3file.m_header.szAgency, "IGS");
		// Line 2
		sprintf(sp3file.m_header.szLine2Symbols, "##");
		// 判断北斗/GPS系统标记
		if(m_typeSatSystem == 1) // 2012/05/23
			sp3file.m_header.tmGPSWeek = TimeCoordConvert::GPST2WeekTime(TimeCoordConvert::BDT2GPST(T_Begin));
		else
			sp3file.m_header.tmGPSWeek = TimeCoordConvert::GPST2WeekTime(T_Begin);

		sp3file.m_header.dEpochInterval = spanSeconds;
		double dMJD = TimeCoordConvert::DayTime2MJD(T_Begin);
		sp3file.m_header.nModJulDaySt = long(floor(dMJD));    
		sp3file.m_header.dFractionalDay = dMJD - sp3file.m_header.nModJulDaySt;
		// Line 3
		sprintf(sp3file.m_header.szLine3Symbols, "+ ");
		// 综合统计可视卫星列表
		sp3file.m_header.pstrSatNameList.clear(); // 2014/03/23, 由谷德峰修改, 兼容混合系统
		sp3file.m_header.pbySatAccuracyList.clear();

		for(int i = 0; i < MAX_PRN_GPS; i++)
		{
			if(pbySatList[i] == 1)
			{
				char szSatName[4]; // 2014/03/23, 由谷德峰修改, 兼容混合系统
				if(m_typeSatSystem == 1)
					sprintf(szSatName, "C%02d", i);
				else
                    sprintf(szSatName, "G%02d", i);
				szSatName[3] = '\0';
				sp3file.m_header.pstrSatNameList.push_back(szSatName);
				sp3file.m_header.pbySatAccuracyList.push_back(3);
				//  2016/06/26, 由谷德峰修改, 补充无法检索的轨道数据, 以保证星历完整
				for(size_t s_i = 0; s_i < sp3file.m_data.size(); s_i++)
				{
					if(sp3file.m_data[s_i].sp3.size() > 0)
					{
						SP3SatMap::const_iterator it = sp3file.m_data[s_i].sp3.find(szSatName);
						if(it == sp3file.m_data[s_i].sp3.end())
						{
							SP3Datum sp3Datum;
							sp3Datum.pos.x  = 0.0; 
							sp3Datum.pos.y  = 0.0; 
							sp3Datum.pos.z  = 0.0; 
							sp3Datum.clk    = 999999.999999; 
							char szName[5];
							if(m_typeSatSystem == 1)
								sprintf(szName, "C%02d ", i);
							else
								sprintf(szName, "G%02d ", i);
							szName[4] = '\0';
							sp3file.m_data[s_i].sp3.insert(SP3SatMap::value_type(szName, sp3Datum));
						}
					}
				}
			}
		}
		sp3file.m_header.bNumberofSats = BYTE(sp3file.m_header.pstrSatNameList.size());
		// Line 8--12
		sprintf(sp3file.m_header.szLine8Symbols, "++");
		// Line 13-14
		sprintf(sp3file.m_header.szLine13Symbols, "%%c");
		if(m_typeSatSystem == 1)
		{
			sprintf(sp3file.m_header.szFileType, "C");
			sprintf(sp3file.m_header.szTimeSystem, "BDT");
		}
		else
		{
			sprintf(sp3file.m_header.szFileType, "G");
			sprintf(sp3file.m_header.szTimeSystem, "GPS");
		}
		// Line 15-16
		sprintf(sp3file.m_header.szLine15Symbols, "%%f");
		sp3file.m_header.dBaseforPosVel  = 0;
		sp3file.m_header.dBaseforClkRate = 0;
		// Line 17-18
		sprintf(sp3file.m_header.szLine17Symbols, "%%i");
		// Line 19-22
		sprintf(sp3file.m_header.szLine19Symbols, "/*");
		sp3file.write(strSP3fileName);
	}

	// 子程序名称： exportCLKFile   
	// 作用：导出clk格式钟差文件
	// 类型：strCLKfileName  : 文件名称
	//       T_Begin         : 星历开始时间
	//       T_End           : 星历结束时间
	//       spanSeconds    : 星历相邻时间点的时间间隔，默认2分钟
	// 输入：T_Begin, T_End, dSpanSeconds
	// 输出：
	// 语言：C++
	// 创建时间：2008/01/22
	// 版本时间：
	// 修改记录：
	// 备注：
	void Rinex2_1_NavFile::exportCLKFile(string strCLKfileName, DayTime T_Begin, DayTime T_End, double spanSeconds, double threshold_span_max, bool onForwad)
	{
		CLKFile clkfile;
		BYTE pbySatList[MAX_PRN_GPS];    // 卫星列表
		for(int i = 0; i < MAX_PRN_GPS; i++)
			pbySatList[i] = 0;
		DayTime T = T_Begin;
		int k = 0;
		while( T_End - T > 0 )
		{
			CLKEpoch clkEpoch;
			clkEpoch.t = T;
			clkEpoch.ARList.clear();
			clkEpoch.ASList.clear();
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				POSCLK posclk;
				if(getEphemeris(T, i, posclk, threshold_span_max, onForwad))
				{
					pbySatList[i] = 1;
					CLKDatum   ASDatum;
					ASDatum.count = 2;
					char  satname[4];
					if(m_typeSatSystem == 0)     //GPS
						sprintf(satname,"G%02d",i);
					else
						sprintf(satname,"C%02d",i);//北斗
					satname[3] = '\0';
					ASDatum.name = satname;						
					ASDatum.clkBias = posclk.clk;
					ASDatum.clkBiasSigma = 0;
					clkEpoch.ASList.insert(CLKMap::value_type(satname, ASDatum));					
				}
			}
			clkfile.m_data.push_back(clkEpoch);
			T = T + spanSeconds;
		}
		// 书写文件头
		sprintf(clkfile.m_header.szRinexVersion, "2.0");
		clkfile.m_header.cFileType = 'C';
		sprintf(clkfile.m_header.szProgramName,"NUDT 1.0");
		sprintf(clkfile.m_header.szAgencyName,"704");
		clkfile.m_header.LeapSecond = 0;
		clkfile.m_header.ClockDataTypeCount = 1;
		clkfile.m_header.pstrClockDataTypeList.clear();
		clkfile.m_header.pstrClockDataTypeList.push_back("AS");
		sprintf(clkfile.m_header.szACShortName,"NUDT");
		clkfile.m_header.nStaCount = 0;
		sprintf(clkfile.m_header.szStaCoordFrame,"IGS00 : IGS REALIZATION OF THE ITRF2000");
		// 综合统计可视卫星列表
		clkfile.m_header.pszSatList.clear();
		for(int i = 0; i < MAX_PRN_GPS; i++)
		{
			if(pbySatList[i] == 1)
			{
				char szPRN[4];
				if(m_typeSatSystem == 0) //GPS
					sprintf(szPRN, "G%02d", i);
				else
					sprintf(szPRN, "C%02d", i);;//北斗
				szPRN[3] = '\0';
				clkfile.m_header.pszSatList.push_back(szPRN);
			}
		}
		clkfile.m_header.bySatCount = BYTE(clkfile.m_header.pszSatList.size());
		clkfile.write(strCLKfileName);
	}
}
