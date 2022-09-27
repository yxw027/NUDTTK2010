#include "CLKFile.hpp"
#include "Rinex2_1_ObsFile.hpp"
#include "MathAlgorithm.hpp"
#include "RuningInfoFile.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	CLKFile::CLKFile(void)
	{
	}

	CLKFile::~CLKFile(void)
	{
	}

	void CLKFile::clear()
	{
		m_header = CLKHeader::CLKHeader();
		m_data.clear();
	}

	bool CLKFile::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}

	// 子程序名称： isValidEpochLine   
	// 功能：判断当前文本行数据是否为有效时刻行 
	//         返回0 -> 文件末尾
	//         返回1 -> 有效时刻
	//         返回2 -> 无效
	// 变量类型：strLine           : 行文本 
	//           pCLKFile      　　: 文件指针
	// 输入：strLine, pCLKFile
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/05/10
	// 版本时间：2008/05/10
	// 修改记录：
	// 备注： 
	int  CLKFile::isValidEpochLine(string strLine, FILE * pCLKFile)
	{
		GPST tmEpoch;
		char szType[2 + 1];  // 目前只处理 AR  AS 两种类型
		if(pCLKFile != NULL) // 判断是否为文件末尾
		{
			if(feof(pCLKFile))
				return 0;
		}
		sscanf(strLine.c_str(),"%2c%*6c%4d%3d%3d%3d%3d%10lf",
			                   szType,
							   &tmEpoch.year,
							   &tmEpoch.month,
							   &tmEpoch.day,
							   &tmEpoch.hour,
							   &tmEpoch.minute,
							   &tmEpoch.second);
		szType[2] = '\0'; // 2010/06/20 调整, 增加'\0', 否则 strcmp 函数无法匹配
		int nFlag = 1;
		if(strcmp(szType,"AR") != 0 && strcmp(szType,"AS") != 0)
			nFlag = 2;
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
		return nFlag;
	}

	int  CLKFile::isValidEpochLine_rinex304(string strLine, FILE * pCLKFile)
	{
		GPST tmEpoch;
		char szType[2 + 1];  // 目前只处理 AR  AS 两种类型
		if(pCLKFile != NULL) // 判断是否为文件末尾
		{
			if(feof(pCLKFile))
				return 0;
		}
		sscanf(strLine.c_str(),"%2c%*6c%*5c%4d%3d%3d%3d%3d%10lf",
			                   szType,
							   &tmEpoch.year,
							   &tmEpoch.month,
							   &tmEpoch.day,
							   &tmEpoch.hour,
							   &tmEpoch.minute,
							   &tmEpoch.second);
		szType[2] = '\0'; // 2010/06/20 调整, 增加'\0', 否则 strcmp 函数无法匹配
		int nFlag = 1;
		if(strcmp(szType,"AR") != 0 && strcmp(szType,"AS") != 0)
			nFlag = 2;
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
		return nFlag;
	}
	// 子程序名称： getEpochSpan   
	// 功能：获得数据间隔 
	// 变量类型：
	// 输入：
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/03/16
	// 版本时间：2012/03/16
	// 修改记录：
	// 备注： 
	double CLKFile::getEpochSpan()
	{
		if( m_data.size() > 1 )
		{
			return m_data[1].t - m_data[0].t;
		}
		else
			return DBL_MAX;
	}

	// 子程序名称： open   
	// 功能：观测数据解析 
	// 变量类型：strCLKFileName : 观测数据文件路径
	// 输入：strCLKFileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/05/10
	// 版本时间：2008/05/10
	// 修改记录：
	// 备注： 
	bool CLKFile::open(string  strCLKFileName)
	{
		if(!isWildcardMatch(strCLKFileName.c_str(), "*.clk", true) 
			&& !isWildcardMatch(strCLKFileName.c_str(), "*.clk_30s", true)
			&& !isWildcardMatch(strCLKFileName.c_str(), "*.clk_05s", true))
		{
			printf("钟差文件名不匹配!\n");
			return false;
		}
		FILE * pCLKFile = fopen(strCLKFileName.c_str(), "r+t");
		if(pCLKFile == NULL) 
			return false;
		m_header = CLKHeader::CLKHeader();
		// 开始循环读取每一行数据，直到 END OF HEADER
		int bFlag = 1;
		while(bFlag)
		{
			char line[100];
			fgets(line, 100, pCLKFile);
			string strLineMask = line;
			string strLine = line;
			strLineMask.erase(0,60); // 从第 0 个元素开始，删除 60 个
			// 剔除 \n
			size_t nPos_n=strLineMask.find('\n');
			if(nPos_n<strLineMask.length())
				strLineMask.erase(nPos_n,1);
			// 超过 20 位，截取 20 位
			while(strLineMask.length() > 20)
				strLineMask.erase(strLineMask.length() - 1, 1);
			// 补齐 20 位
			if(strLineMask.length() < 20) // strLineMask.length 包含最后的'\0'
				strLineMask.append(20 - strLineMask.length(), ' ');
			if(strLineMask == Rinex2_1_MaskString::szVerType)
			{
				strLine.copy(m_header.szRinexVersion, 20, 0);
				m_header.szRinexVersion[20] = '\0';
				sscanf(line, "%*20c%c", &m_header.cFileType);
			}
			else if(strLineMask == Rinex2_1_MaskString::szPgmRunDate)
			{
				strLine.copy(m_header.szProgramName, 20, 0);
				m_header.szProgramName[20] = '\0';
				strLine.copy(m_header.szAgencyName, 20, 20);
				m_header.szAgencyName[20] = '\0';
				strLine.copy(m_header.szFileDate, 20, 40);
				m_header.szFileDate[20] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szLeapSec)
			{  
				sscanf(line,"%6d", &m_header.LeapSecond);
			}
			else if(strLineMask == Rinex2_1_MaskString::szDataTypes)
			{
				sscanf(line, "%6d", &m_header.ClockDataTypeCount);
				for(int i = 1; i <= m_header.ClockDataTypeCount; i++)
				{
					char szCLKDataType[3];
					strLine.copy(szCLKDataType, 2, 6 + i * 6 - 2);
					szCLKDataType[2] = '\0';
					m_header.pstrClockDataTypeList.push_back(szCLKDataType);
				}
			}
			else if(strLineMask == Rinex2_1_MaskString::szAnalysisCenter)
			{
				strLine.copy(m_header.szACShortName, 3, 0);
				m_header.szACShortName[3] = '\0';
				strLine.copy(m_header.szACFullName, 55, 5);
				m_header.szACFullName[55] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szTRFofSolnSta)
			{
				sscanf(line,"%6d%*4c%50c", &m_header.nStaCount, m_header.szStaCoordFrame);
				m_header.szStaCoordFrame[50] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szSolnStaNameNum)
			{
				CLKStaNamePos staNamePos;
				char szX[12];
				char szY[12];
				char szZ[12];
				sscanf(line,"%4c%*1c%20c%11c%*1c%11c%*1c%11c",
					        staNamePos.szName,
							staNamePos.szID,
							szX,
							szY,
							szZ);
				staNamePos.szName[4] = '\0';
				staNamePos.szID[20]  = '\0';
				szX[11] = '\0';
				szY[11] = '\0';
				szZ[11] = '\0';
				staNamePos.lnX = _atoi64(szX);
				staNamePos.lnY = _atoi64(szY);
				staNamePos.lnZ = _atoi64(szZ);
				m_header.pStaPosList.push_back(staNamePos);
			}
			else if(strLineMask == Rinex2_1_MaskString::szSolnSatsNum)
			{  
				sscanf(line, "%6d", &m_header.bySatCount);
			}
			else if(strLineMask == Rinex2_1_MaskString::szPRNList)
			{  
				for(int i = 1; i <= 15; i++)
				{
					char szPRN[4];
					strLine.copy(szPRN,3,(i-1) * 4);
					szPRN[3] = '\0';
					if(szPRN[0] == 'G' || szPRN[0] == 'R' || szPRN[0] == 'C' || szPRN[0] == 'E') // 2012/04/06, 增加北斗考虑
					{
						m_header.pszSatList.push_back(szPRN);
					}
				}
			}
			else if(strLineMask == Rinex2_1_MaskString::szEndOfHead)
			{
				bFlag = false;
			}
			else // Comment 等不作处理
			{
			}
		}
		// 观测数据
		bFlag = TRUE;
		m_data.clear();
		int k = 0;
		char line[100];
		fgets(line, 100, pCLKFile);
		GPST  tmCLKRef;
		CLKEpoch clkEpoch;
		while(bFlag)
		{
			string strLine = line;
			int nFlag = isValidEpochLine(strLine, pCLKFile);
			if(nFlag == 0)
			{// 文件末尾
				if(clkEpoch.ARList.size() != 0 || clkEpoch.ASList.size() != 0)
				{// 初始点, 只更新时间
					clkEpoch.t = tmCLKRef;
					m_data.push_back(clkEpoch);
				}
				bFlag = false;
			}
			else if(nFlag == 1)
			{// 找到新一行有效数据段
				k++;
				GPST tmEpoch;
				//读取时间
				sscanf(line,"%*8c%4d%3d%3d%3d%3d%10lf",
					        &tmEpoch.year,
							&tmEpoch.month,
							&tmEpoch.day,
							&tmEpoch.hour,
							&tmEpoch.minute,
							&tmEpoch.second);
				if(tmEpoch == tmCLKRef)
				{
					char szType[2 + 1];
					sscanf(line, "%2c", szType);
					szType[2] = '\0';
					if(strcmp(szType,"AR") == 0)
					{
						char szARName[4 + 1];
						CLKDatum ARDatum; // 单位: 秒
						sscanf(line,"%*3c%4c%*27c%3d%*3c%19lf%*1c%19lf",
							        szARName,
									&ARDatum.count,
									&ARDatum.clkBias,
									&ARDatum.clkBiasSigma);
						szARName[4] = '\0';
						ARDatum.name = szARName;
						if(ARDatum.count > 2 && ARDatum.count <= 6)
						{
							fgets(line, 100, pCLKFile);
							sscanf(line,"%19lf%*1c%19lf%*1c%19lf%*1c%19lf",
								        &ARDatum.clkRate,
										&ARDatum.clkRateSigma,
										&ARDatum.clkAcc,
										&ARDatum.clkAccSigma);
						}
						clkEpoch.ARList.insert(CLKMap::value_type(ARDatum.name, ARDatum));
					}
					else if(strcmp(szType, "AS") == 0)
					{
						CLKDatum ASDatum;
						char szASName[4 + 1];
						sscanf(line,"%*3c%4c%*27c%3d%*3c%19lf%*1c%19lf",
							        szASName,
									&ASDatum.count,
									&ASDatum.clkBias,
									&ASDatum.clkBiasSigma);
						if(szASName[1] == ' ')// 2012/10/31, 将G 1->G01
							szASName[1] = '0';
						szASName[4] = '\0';
						ASDatum.name = szASName;
						if(ASDatum.count > 2 && ASDatum.count <= 6)
						{
							fgets(line, 100, pCLKFile);
							sscanf(line,"%19lf%*1c%19lf%*1c%19lf%*1c%19lf",
								        &ASDatum.clkRate,
										&ASDatum.clkRateSigma,
										&ASDatum.clkAcc,
										&ASDatum.clkAccSigma);
						}
						clkEpoch.ASList.insert(CLKMap::value_type(ASDatum.name, ASDatum));
					}
					fgets(line, 100, pCLKFile);
				}
				else // 发现新的时刻
				{
					// 将一个时刻的数据记录在 m_data 中
					if(clkEpoch.ARList.size() == 0 && clkEpoch.ASList.size() == 0)
					{// 初始点，只更新时间
						tmCLKRef = tmEpoch; 
					}
					else
					{
						clkEpoch.t = tmCLKRef;
						m_data.push_back(clkEpoch);
						tmCLKRef = tmEpoch; // 更新时刻
						clkEpoch = CLKEpoch::CLKEpoch(); // 清空 clkEpoch, 2008/04/27
					}
				}
			}
			else
			{
				fgets(line, 100, pCLKFile);
			}
		}
		fclose(pCLKFile);
		// 20150822, 进行连续化处理, 谷德峰
		if( m_data.size() > 0 )
		{
			double spanInterval = DBL_MAX;
			for(size_t s_i = 1; s_i < m_data.size(); s_i++)
			{
				double span_i = m_data[s_i].t - m_data[s_i - 1].t;
				spanInterval = spanInterval > span_i ? span_i : spanInterval;
			}
			vector<CLKEpoch> data;
			data.push_back(m_data[0]);
			size_t s_i = 1;
			while(s_i < m_data.size())
			{
				GPST t_last = data[data.size() - 1].t;
				double span_t = m_data[s_i].t - t_last;
				if(span_t == spanInterval)
				{// 数据刚好匹配
					data.push_back(m_data[s_i]);
					s_i++;
					continue;
				}
				else if(span_t > spanInterval)
				{// 添加新的空纪录
					CLKEpoch clkEpoch_t;
					clkEpoch_t.ARList.clear();
					clkEpoch_t.ASList.clear();
					clkEpoch_t.t = t_last + spanInterval;
					data.push_back(clkEpoch_t);
					//char info[100];
					//sprintf(info, "星历数据不连续, 历元%s缺失!", clkEpoch_t.t.toString().c_str());
					//RuningInfoFile::Add(info);
					continue;
				}
				else
				{// 跳过该条记录
					s_i++;
					continue;
				}
			}
			m_data = data; // 更新数据
		}
		return true;
	}
	
	// 子程序名称： open_rinex304
	bool CLKFile::open_rinex304(string  strCLKFileName)
	{
		if(!isWildcardMatch(strCLKFileName.c_str(), "*.clk", true) 
			&& !isWildcardMatch(strCLKFileName.c_str(), "*.clk_30s", true)
			&& !isWildcardMatch(strCLKFileName.c_str(), "*.clk_05s", true))
		{
			printf("钟差文件名不匹配!\n");
			return false;
		}
		FILE * pCLKFile = fopen(strCLKFileName.c_str(), "r+t");
		if(pCLKFile == NULL) 
			return false;
		m_header = CLKHeader::CLKHeader();
		// 开始循环读取每一行数据，直到 END OF HEADER
		int bFlag = 1;
		while(bFlag)
		{
			char line[100];
			fgets(line, 100, pCLKFile);
			string strLineMask = line;
			string strLine = line;
			strLineMask.erase(0,65); // 从第 0 个元素开始，删除 65 个
			// 剔除 \n
			size_t nPos_n=strLineMask.find('\n');
			if(nPos_n<strLineMask.length())
				strLineMask.erase(nPos_n,1);
			// 超过 20 位，截取 20 位
			while(strLineMask.length() > 20)
				strLineMask.erase(strLineMask.length() - 1, 1);
			// 补齐 20 位
			if(strLineMask.length() < 20) // strLineMask.length 包含最后的'\0'
				strLineMask.append(20 - strLineMask.length(), ' ');
			if(strLineMask == Rinex2_1_MaskString::szVerType)
			{
				strLine.copy(m_header.szRinexVersion, 20, 0);
				m_header.szRinexVersion[20] = '\0';
				sscanf(line, "%*21c%c", &m_header.cFileType);
			}
			else if(strLineMask == Rinex2_1_MaskString::szPgmRunDate)
			{
				strLine.copy(m_header.szProgramName, 20, 0);
				m_header.szProgramName[20] = '\0';
				strLine.copy(m_header.szAgencyName, 20, 21);
				m_header.szAgencyName[20] = '\0';
				strLine.copy(m_header.szFileDate, 20, 42);
				m_header.szFileDate[20] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szLeapSecGNSS)
			{  
				sscanf(line,"%6d", &m_header.LeapSecond);
			}
			else if(strLineMask == Rinex2_1_MaskString::szDataTypes)
			{
				sscanf(line, "%6d", &m_header.ClockDataTypeCount);
				for(int i = 1; i <= m_header.ClockDataTypeCount; i++)
				{
					char szCLKDataType[3];
					strLine.copy(szCLKDataType, 2, 6 + i * 6 - 2);
					szCLKDataType[2] = '\0';
					m_header.pstrClockDataTypeList.push_back(szCLKDataType);
				}
			}
			else if(strLineMask == Rinex2_1_MaskString::szAnalysisCenter)
			{
				strLine.copy(m_header.szACShortName, 3, 0);
				m_header.szACShortName[3] = '\0';
				strLine.copy(m_header.szACFullName, 55, 5);
				m_header.szACFullName[55] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szTRFofSolnSta)
			{
				sscanf(line,"%6d%*4c%50c", &m_header.nStaCount, m_header.szStaCoordFrame);
				m_header.szStaCoordFrame[50] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szSolnStaNameNum)
			{
				CLKStaNamePos staNamePos;
				char szX[12];
				char szY[12];
				char szZ[12];
				sscanf(line,"%4c%*5c%*1c%20c%11c%*1c%11c%*1c%11c",
					        staNamePos.szName,
							staNamePos.szID,
							szX,
							szY,
							szZ);
				staNamePos.szName[4] = '\0';
				staNamePos.szID[20]  = '\0';
				szX[11] = '\0';
				szY[11] = '\0';
				szZ[11] = '\0';
				staNamePos.lnX = _atoi64(szX);
				staNamePos.lnY = _atoi64(szY);
				staNamePos.lnZ = _atoi64(szZ);
				m_header.pStaPosList.push_back(staNamePos);
			}
			else if(strLineMask == Rinex2_1_MaskString::szSolnSatsNum)
			{  
				sscanf(line, "%6d", &m_header.bySatCount);
			}
			else if(strLineMask == Rinex2_1_MaskString::szPRNList)
			{  
				for(int i = 1; i <= 16; i++)
				{
					char szPRN[4];
					strLine.copy(szPRN,3,(i-1) * 4);
					szPRN[3] = '\0';
					if(szPRN[0] == 'G' || szPRN[0] == 'R' || szPRN[0] == 'C' || szPRN[0] == 'E') // 2012/04/06, 增加北斗考虑
					{
						m_header.pszSatList.push_back(szPRN);
					}
				}
			}
			else if(strLineMask == Rinex2_1_MaskString::szEndOfHead)
			{
				bFlag = false;
			}
			else // Comment 等不作处理
			{
			}
		}
		// 观测数据
		bFlag = TRUE;
		m_data.clear();
		int k = 0;
		char line[100];
		fgets(line, 100, pCLKFile);
		GPST  tmCLKRef;
		CLKEpoch clkEpoch;
		while(bFlag)
		{
			string strLine = line;
			int nFlag = isValidEpochLine_rinex304(strLine, pCLKFile);
			if(nFlag == 0)
			{// 文件末尾
				if(clkEpoch.ARList.size() != 0 || clkEpoch.ASList.size() != 0)
				{// 初始点, 只更新时间
					clkEpoch.t = tmCLKRef;
					m_data.push_back(clkEpoch);
				}
				bFlag = false;
			}
			else if(nFlag == 1)
			{// 找到新一行有效数据段
				k++;
				GPST tmEpoch;
				//读取时间
				sscanf(line,"%*13c%4d%3d%3d%3d%3d%10lf",
					        &tmEpoch.year,
							&tmEpoch.month,
							&tmEpoch.day,
							&tmEpoch.hour,
							&tmEpoch.minute,
							&tmEpoch.second);
				if(tmEpoch == tmCLKRef)
				{
					char szType[2 + 1];
					sscanf(line, "%2c", szType);
					szType[2] = '\0';
					if(strcmp(szType,"AR") == 0)
					{
						char szARName[4 + 1];
						CLKDatum ARDatum; // 单位: 秒
						sscanf(line,"%*3c%4c%*5c%*27c%3d%*3c%19lf%*1c%19lf",
							        szARName,
									&ARDatum.count,
									&ARDatum.clkBias,
									&ARDatum.clkBiasSigma);
						szARName[4] = '\0';
						ARDatum.name = szARName;
						//if(ARDatum.count > 2 && ARDatum.count <= 6)
						//{
						//	fgets(line, 100, pCLKFile);
						//	sscanf(line,"%19lf%*1c%19lf%*1c%19lf%*1c%19lf",
						//		        &ARDatum.clkRate,
						//				&ARDatum.clkRateSigma,
						//				&ARDatum.clkAcc,
						//				&ARDatum.clkAccSigma);
						//}
						clkEpoch.ARList.insert(CLKMap::value_type(ARDatum.name, ARDatum));
					}
					else if(strcmp(szType, "AS") == 0)
					{
						CLKDatum ASDatum;
						char szASName[4 + 1];
						sscanf(line,"%*3c%4c%*5c%*27c%3d%*3c%19lf%*1c%19lf",
							        szASName,
									&ASDatum.count,
									&ASDatum.clkBias,
									&ASDatum.clkBiasSigma);
						if(szASName[1] == ' ')// 2012/10/31, 将G 1->G01
							szASName[1] = '0';
						szASName[4] = '\0';
						ASDatum.name = szASName;
						//if(ASDatum.count > 2 && ASDatum.count <= 6)
						//{
						//	fgets(line, 100, pCLKFile);
						//	sscanf(line,"%19lf%*1c%19lf%*1c%19lf%*1c%19lf",
						//		        &ASDatum.clkRate,
						//				&ASDatum.clkRateSigma,
						//				&ASDatum.clkAcc,
						//				&ASDatum.clkAccSigma);
						//}
						clkEpoch.ASList.insert(CLKMap::value_type(ASDatum.name, ASDatum));
					}
					fgets(line, 100, pCLKFile);
				}
				else // 发现新的时刻
				{
					// 将一个时刻的数据记录在 m_data 中
					if(clkEpoch.ARList.size() == 0 && clkEpoch.ASList.size() == 0)
					{// 初始点，只更新时间
						tmCLKRef = tmEpoch; 
					}
					else
					{
						clkEpoch.t = tmCLKRef;
						m_data.push_back(clkEpoch);
						tmCLKRef = tmEpoch; // 更新时刻
						clkEpoch = CLKEpoch::CLKEpoch(); // 清空 clkEpoch, 2008/04/27
					}
				}
			}
			else
			{
				fgets(line, 100, pCLKFile);
			}
		}
		fclose(pCLKFile);
		// 20150822, 进行连续化处理, 谷德峰
		if( m_data.size() > 0 )
		{
			double spanInterval = DBL_MAX;
			for(size_t s_i = 1; s_i < m_data.size(); s_i++)
			{
				double span_i = m_data[s_i].t - m_data[s_i - 1].t;
				spanInterval = spanInterval > span_i ? span_i : spanInterval;
			}
			vector<CLKEpoch> data;
			data.push_back(m_data[0]);
			size_t s_i = 1;
			while(s_i < m_data.size())
			{
				GPST t_last = data[data.size() - 1].t;
				double span_t = m_data[s_i].t - t_last;
				if(span_t == spanInterval)
				{// 数据刚好匹配
					data.push_back(m_data[s_i]);
					s_i++;
					continue;
				}
				else if(span_t > spanInterval)
				{// 添加新的空纪录
					CLKEpoch clkEpoch_t;
					clkEpoch_t.ARList.clear();
					clkEpoch_t.ASList.clear();
					clkEpoch_t.t = t_last + spanInterval;
					data.push_back(clkEpoch_t);
					//char info[100];
					//sprintf(info, "星历数据不连续, 历元%s缺失!", clkEpoch_t.t.toString().c_str());
					//RuningInfoFile::Add(info);
					continue;
				}
				else
				{// 跳过该条记录
					s_i++;
					continue;
				}
			}
			m_data = data; // 更新数据
		}
		return true;
	}

		// 子程序名称： open_LeoClk   
	// 功能：观测数据解析 
	// 变量类型：strCLKFileName : 观测数据文件路径
	// 输入：strCLKFileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/05/10
	// 版本时间：2008/05/10
	// 修改记录：
	// 备注： 不进行空白数据补充
	bool CLKFile::open_LeoClk(string  strCLKFileName)
	{
		if(!isWildcardMatch(strCLKFileName.c_str(), "*.clk", true) && !isWildcardMatch(strCLKFileName.c_str(), "*.clk_30s", true))
			return false;
		FILE * pCLKFile = fopen(strCLKFileName.c_str(), "r+t");
		if(pCLKFile == NULL) 
			return false;
		m_header = CLKHeader::CLKHeader();
		// 开始循环读取每一行数据，直到 END OF HEADER
		int bFlag = 1;
		while(bFlag)
		{
			char line[100];
			fgets(line, 100, pCLKFile);
			string strLineMask = line;
			string strLine = line;
			strLineMask.erase(0,60); // 从第 0 个元素开始，删除 60 个
			// 剔除 \n
			size_t nPos_n=strLineMask.find('\n');
			if(nPos_n<strLineMask.length())
				strLineMask.erase(nPos_n,1);
			// 超过 20 位，截取 20 位
			while(strLineMask.length() > 20)
				strLineMask.erase(strLineMask.length() - 1, 1);
			// 补齐 20 位
			if(strLineMask.length() < 20) // strLineMask.length 包含最后的'\0'
				strLineMask.append(20 - strLineMask.length(), ' ');
			if(strLineMask == Rinex2_1_MaskString::szVerType)
			{
				strLine.copy(m_header.szRinexVersion, 20, 0);
				sscanf(line, "%*20c%c", &m_header.cFileType);
			}
			else if(strLineMask == Rinex2_1_MaskString::szPgmRunDate)
			{
				strLine.copy(m_header.szProgramName, 20, 0);
				strLine.copy(m_header.szAgencyName, 20, 20);
				strLine.copy(m_header.szFileDate, 20, 40);
			}
			else if(strLineMask == Rinex2_1_MaskString::szLeapSec)
			{  
				sscanf(line,"%6d", &m_header.LeapSecond);
			}
			else if(strLineMask == Rinex2_1_MaskString::szDataTypes)
			{
				sscanf(line, "%6d", &m_header.ClockDataTypeCount);
				for(int i = 1; i <= m_header.ClockDataTypeCount; i++)
				{
					char szCLKDataType[3];
					strLine.copy(szCLKDataType, 2, 6 + i * 6 - 2);
					szCLKDataType[2] = '\0';
					m_header.pstrClockDataTypeList.push_back(szCLKDataType);
				}
			}
			else if(strLineMask == Rinex2_1_MaskString::szAnalysisCenter)
			{
				strLine.copy(m_header.szACShortName, 3, 0);
				strLine.copy(m_header.szACFullName, 55, 5);
			}
			else if(strLineMask == Rinex2_1_MaskString::szTRFofSolnSta)
			{
				sscanf(line,"%6d%*4c%50c", &m_header.nStaCount, m_header.szStaCoordFrame);
			}
			else if(strLineMask == Rinex2_1_MaskString::szSolnStaNameNum)
			{
				CLKStaNamePos staNamePos;
				char szX[12];
				char szY[12];
				char szZ[12];
				sscanf(line,"%4c%*1c%20c%11c%*1c%11c%*1c%11c",
					        staNamePos.szName,
							staNamePos.szID,
							szX,
							szY,
							szZ);
				staNamePos.szName[4] = '\0';
				staNamePos.szID[20]  = '\0';
				szX[11] = '\0';
				szY[11] = '\0';
				szZ[11] = '\0';
				staNamePos.lnX = _atoi64(szX);
				staNamePos.lnY = _atoi64(szY);
				staNamePos.lnZ = _atoi64(szZ);
				m_header.pStaPosList.push_back(staNamePos);
			}
			else if(strLineMask == Rinex2_1_MaskString::szSolnSatsNum)
			{  
				sscanf(line, "%6d", &m_header.bySatCount);
			}
			else if(strLineMask == Rinex2_1_MaskString::szPRNList)
			{  
				for(int i = 1; i <= 15; i++)
				{
					char szPRN[4];
					strLine.copy(szPRN,3,(i-1) * 4);
					szPRN[3] = '\0';
					if(szPRN[0] == 'G' || szPRN[0] == 'R' || szPRN[0] == 'C' || szPRN[0] == 'E') // 2012/04/06, 增加北斗考虑
					{
						m_header.pszSatList.push_back(szPRN);
					}
				}
			}
			else if(strLineMask == Rinex2_1_MaskString::szEndOfHead)
			{
				bFlag = false;
			}
			else // Comment 等不作处理
			{
			}
		}
		// 观测数据
		bFlag = TRUE;
		m_data.clear();
		int k = 0;
		char line[100];
		fgets(line, 100, pCLKFile);
		GPST  tmCLKRef;
		CLKEpoch clkEpoch;
		while(bFlag)
		{
			string strLine = line;
			int nFlag = isValidEpochLine(strLine, pCLKFile);
			if(nFlag == 0)
			{// 文件末尾
				if(clkEpoch.ARList.size() != 0 || clkEpoch.ASList.size() != 0)
				{// 初始点, 只更新时间
					clkEpoch.t = tmCLKRef;
					m_data.push_back(clkEpoch);
				}
				bFlag = false;
			}
			else if(nFlag == 1)
			{// 找到新一行有效数据段
				k++;
				GPST tmEpoch;
				//读取时间
				sscanf(line,"%*8c%4d%3d%3d%3d%3d%10lf",
					        &tmEpoch.year,
							&tmEpoch.month,
							&tmEpoch.day,
							&tmEpoch.hour,
							&tmEpoch.minute,
							&tmEpoch.second);
				if(tmEpoch == tmCLKRef)
				{
					char szType[2 + 1];
					sscanf(line, "%2c", szType);
					szType[2] = '\0';
					if(strcmp(szType,"AR") == 0)
					{
						char szARName[4 + 1];
						CLKDatum ARDatum; // 单位: 秒
						sscanf(line,"%*3c%4c%*27c%3d%*3c%19lf%*1c%19lf",
							        szARName,
									&ARDatum.count,
									&ARDatum.clkBias,
									&ARDatum.clkBiasSigma);
						szARName[4] = '\0';
						ARDatum.name = szARName;
						if(ARDatum.count > 2 && ARDatum.count <= 6)
						{
							fgets(line, 100, pCLKFile);
							sscanf(line,"%19lf%*1c%19lf%*1c%19lf%*1c%19lf",
								        &ARDatum.clkRate,
										&ARDatum.clkRateSigma,
										&ARDatum.clkAcc,
										&ARDatum.clkAccSigma);
						}
						clkEpoch.ARList.insert(CLKMap::value_type(ARDatum.name, ARDatum));
					}
					else if(strcmp(szType, "AS") == 0)
					{
						CLKDatum ASDatum;
						char szASName[4 + 1];
						sscanf(line,"%*3c%4c%*27c%3d%*3c%19lf%*1c%19lf",
							        szASName,
									&ASDatum.count,
									&ASDatum.clkBias,
									&ASDatum.clkBiasSigma);
						if(szASName[1] == ' ')// 2012/10/31, 将G 1->G01
							szASName[1] = '0';
						szASName[4] = '\0';
						ASDatum.name = szASName;
						if(ASDatum.count > 2 && ASDatum.count <= 6)
						{
							fgets(line, 100, pCLKFile);
							sscanf(line,"%19lf%*1c%19lf%*1c%19lf%*1c%19lf",
								        &ASDatum.clkRate,
										&ASDatum.clkRateSigma,
										&ASDatum.clkAcc,
										&ASDatum.clkAccSigma);
						}
						clkEpoch.ASList.insert(CLKMap::value_type(ASDatum.name, ASDatum));
					}
					fgets(line, 100, pCLKFile);
				}
				else // 发现新的时刻
				{
					// 将一个时刻的数据记录在 m_data 中
					if(clkEpoch.ARList.size() == 0 && clkEpoch.ASList.size() == 0)
					{// 初始点，只更新时间
						tmCLKRef = tmEpoch; 
					}
					else
					{
						clkEpoch.t = tmCLKRef;
						m_data.push_back(clkEpoch);
						tmCLKRef = tmEpoch; // 更新时刻
						clkEpoch = CLKEpoch::CLKEpoch(); // 清空 clkEpoch, 2008/04/27
					}
				}
			}
			else
			{
				fgets(line, 100, pCLKFile);
			}
		}
		fclose(pCLKFile);
		return true;
	}
	bool CLKFile::write(string strCLKFileName)
	{
		//strCLKFileName = strCLKFileName + ".clk";
		FILE* pCLKFile = fopen(strCLKFileName.c_str(), "w+");
		fprintf(pCLKFile,"%-20s%1c%-39s%20s\n",
			             m_header.szRinexVersion,
						 m_header.cFileType,
						 " ",
						 Rinex2_1_MaskString::szVerType);
		fprintf(pCLKFile,"%-20s%-20s%-20s%-20s\n",
			             m_header.szProgramName,
						 m_header.szAgencyName,
						 m_header.szFileDate,
						 Rinex2_1_MaskString::szPgmRunDate);
		fprintf(pCLKFile,"%6d%-54s%20s\n",
			             m_header.LeapSecond,
						 " ",
						 Rinex2_1_MaskString::szLeapSec);
		fprintf(pCLKFile,"%6d",
			             m_header.ClockDataTypeCount);
		for(int i = 1; i <= 5; i++)
		{
			if(i <= m_header.ClockDataTypeCount)
			{
				string strCLKDataType;
				strCLKDataType = m_header.pstrClockDataTypeList[i - 1];
				fprintf(pCLKFile, "%-4s%2s", " ", strCLKDataType.c_str());
			}
			else
			{
				fprintf(pCLKFile, "%-6s", " ");
			}
		}
		fprintf(pCLKFile,"%-24s%20s\n",
			             " ",
						 Rinex2_1_MaskString::szDataTypes);
		fprintf(pCLKFile,"%3s%-2s%55s%20s\n",
			             m_header.szACShortName,
						 " ",
						 m_header.szACFullName,
						 Rinex2_1_MaskString::szAnalysisCenter);
		fprintf(pCLKFile,"%6d%-4s%-50s%20s\n",
			             m_header.nStaCount,
						 " ",
						 m_header.szStaCoordFrame,
						 Rinex2_1_MaskString::szTRFofSolnSta);
		for(size_t s_i = 1; s_i <= m_header.pStaPosList.size(); s_i++)
		{
			CLKStaNamePos staNamePos;
			staNamePos = m_header.pStaPosList[s_i - 1];
			char szX[12];
			char szY[12];
			char szZ[12];
			_i64toa(staNamePos.lnX, szX, 10);
			_i64toa(staNamePos.lnY, szY, 10);
			_i64toa(staNamePos.lnZ, szZ, 10);
			fprintf(pCLKFile,"%4s %20s%11s %11s %11s%20s\n",
				             staNamePos.szName,
							 staNamePos.szID,
							 szX,
							 szY,
							 szZ,
							 Rinex2_1_MaskString::szSolnStaNameNum);
		}
		fprintf(pCLKFile,"%6d%-54s%20s\n",
			             m_header.bySatCount,
						 " ",
						 Rinex2_1_MaskString::szSolnSatsNum);
		size_t nSatellite = m_header.pszSatList.size();
		size_t nLine    = nSatellite / 15;
		size_t nResidue = nSatellite % 15;
		if(nSatellite <= 15)
		{// 卫星个数小于等于15
			if(nSatellite > 0)
			{
				for(size_t s_j = 0; s_j < nSatellite; s_j++)
				{
					string strSat;
					strSat = m_header.pszSatList[s_j];
					fprintf(pCLKFile, "%3s ", strSat.c_str());
				}
				string strBlank;
				strBlank.append(60 - (4 * nSatellite), ' ');
				fprintf(pCLKFile,"%s%20s\n",
								 strBlank.c_str(),
								 Rinex2_1_MaskString::szPRNList);
			}
		}
		else// 卫星个数大于15 
		{
			// 前15个卫星
			for(size_t s_j = 0; s_j < 15; s_j++)
			{
				string strSat;
				strSat = m_header.pszSatList[s_j];
				fprintf(pCLKFile, "%3s ", strSat.c_str());
			}
			fprintf(pCLKFile, "%20s\n", Rinex2_1_MaskString::szPRNList);
			// 中间 nLine - 1 行数据
			for(size_t s_ii = 1; s_ii < nLine; s_ii++)
			{
				for(size_t s_j = 0; s_j < 15; s_j++)
				{
					string strSatellite;
					strSatellite = m_header.pszSatList[s_ii * 15 + s_j];
					fprintf(pCLKFile, "%3s ", strSatellite.c_str());
				}
				fprintf(pCLKFile, "%20s\n", Rinex2_1_MaskString::szPRNList);
			}
			// 最后 nResidue 个数据
			if(nResidue > 0)
			{
				for(size_t s_j = 0; s_j < nResidue; s_j++)
				{
					string strSat;
					strSat = m_header.pszSatList[nLine * 15 + s_j];
					fprintf(pCLKFile, "%3s ", strSat.c_str());
				}
				string strBlank;
				strBlank.append(60 - (4 * nResidue), ' ');
				fprintf(pCLKFile, "%s%20s\n", strBlank.c_str(), Rinex2_1_MaskString::szPRNList);
			}
		}
		fprintf(pCLKFile, "%-60s%20s\n", " ", Rinex2_1_MaskString::szEndOfHead);
		for(size_t s_i = 1; s_i <= m_data.size(); s_i++)
		{
			CLKEpoch clkEpoch = m_data[s_i - 1];
			for(CLKMap::iterator it = clkEpoch.ARList.begin(); it != clkEpoch.ARList.end(); ++it)
			{
				CLKDatum ARDatum = it->second;
				char szCLKBias[21];
				string strCLKBias;
				sprintf(szCLKBias, "%20.12e", ARDatum.clkBias);
				stringEraseFloatZero(szCLKBias, strCLKBias);
				if(ARDatum.count == 1)
				{
					fprintf(pCLKFile, "AR %-4s %4d%3d%3d%3d%3d%10.6f%3d%-3s%19s\n",
									  ARDatum.name.c_str(),
									  clkEpoch.t.year,
									  clkEpoch.t.month,
									  clkEpoch.t.day,
									  clkEpoch.t.hour,
									  clkEpoch.t.minute,
									  clkEpoch.t.second,
									  ARDatum.count,
									  " ",
									  strCLKBias.c_str());
					continue;
				}
				char szCLKBiasSigma[21];
				string strCLKBiasSigma;
				sprintf(szCLKBiasSigma, "%20.12e", ARDatum.clkBiasSigma);
				stringEraseFloatZero(szCLKBiasSigma, strCLKBiasSigma);
				if(ARDatum.count == 2)
				{
					fprintf(pCLKFile, "AR %-4s %4d%3d%3d%3d%3d%10.6f%3d%-3s%19s %19s\n",
									  ARDatum.name.c_str(),
									  clkEpoch.t.year,
									  clkEpoch.t.month,
									  clkEpoch.t.day,
									  clkEpoch.t.hour,
									  clkEpoch.t.minute,
									  clkEpoch.t.second,
									  ARDatum.count,
									  " ",
									  strCLKBias.c_str(),
									  strCLKBiasSigma.c_str());
				}
				char szCLKRate[21];
				string strCLKRate;
				sprintf(szCLKRate, "%20.12e", ARDatum.clkRate);
				stringEraseFloatZero(szCLKRate, strCLKRate);
				if(ARDatum.count == 3)
				{
					fprintf(pCLKFile," %19s\n",
						             strCLKRate.c_str());
				}
                char szCLKRateSigma[21];
				string strCLKRateSigma;
				sprintf(szCLKRateSigma, "%20.12e", ARDatum.clkRateSigma);
				stringEraseFloatZero(szCLKRateSigma, strCLKRateSigma);
				if(ARDatum.count == 4)
				{
					fprintf(pCLKFile," %19s %19s\n",
						             strCLKRate.c_str(),
									 strCLKRateSigma.c_str());
					continue;
				}
                char szCLKAcc[21];
				string strCLKAcc;
				sprintf(szCLKAcc, "%20.12e", ARDatum.clkAcc);
				stringEraseFloatZero(szCLKAcc, strCLKAcc);
				if(ARDatum.count == 5)
				{
					fprintf(pCLKFile," %19s %19s %19s\n",
						             strCLKRate.c_str(),
									 strCLKRateSigma.c_str(),
									 strCLKAcc.c_str());
					continue;
				}
                char szCLKAccSigma[21];
				string strCLKAccSigma;
				sprintf(szCLKAccSigma, "%20.12e", ARDatum.clkAccSigma);
				stringEraseFloatZero(szCLKAccSigma, strCLKAccSigma);
				if(ARDatum.count == 6)
				{
					fprintf(pCLKFile," %19s %19s %19s %19s\n",
						             strCLKRate.c_str(),
									 strCLKRateSigma.c_str(),
									 strCLKAcc.c_str(),
									 strCLKAccSigma.c_str());
					continue;
				}
			}
			for(CLKMap::iterator it = clkEpoch.ASList.begin(); it != clkEpoch.ASList.end(); ++it)
			{
				CLKDatum ASDatum = it->second;
				char szCLKBias[21];
				string strCLKBias;
				sprintf(szCLKBias, "%20.12e", ASDatum.clkBias);
				stringEraseFloatZero(szCLKBias, strCLKBias);
				if(ASDatum.count == 1)
				{
					fprintf(pCLKFile, "AS %-4s %4d%3d%3d%3d%3d%10.6f%3d%-3s%19s\n",
									  ASDatum.name.c_str(),
									  clkEpoch.t.year,
									  clkEpoch.t.month,
									  clkEpoch.t.day,
									  clkEpoch.t.hour,
									  clkEpoch.t.minute,
									  clkEpoch.t.second,
									  ASDatum.count,
									  " ",
									  strCLKBias.c_str());
					continue;
				}
				char szCLKBiasSigma[21];
				string strCLKBiasSigma;
				sprintf(szCLKBiasSigma, "%20.12e", ASDatum.clkBiasSigma);
				stringEraseFloatZero(szCLKBiasSigma, strCLKBiasSigma);
				if(ASDatum.count == 2)
				{
					fprintf(pCLKFile, "AS %-4s %4d%3d%3d%3d%3d%10.6f%3d%-3s%19s %19s\n",
									  ASDatum.name.c_str(),
									  clkEpoch.t.year,
									  clkEpoch.t.month,
									  clkEpoch.t.day,
									  clkEpoch.t.hour,
									  clkEpoch.t.minute,
									  clkEpoch.t.second,
									  ASDatum.count,
									  " ",
									  strCLKBias.c_str(),
									  strCLKBiasSigma.c_str());
				}
				char szCLKRate[21];
				string strCLKRate;
				sprintf(szCLKRate, "%20.12e", ASDatum.clkRate);
				stringEraseFloatZero(szCLKRate, strCLKRate);
				if(ASDatum.count == 3)
				{
					fprintf(pCLKFile," %19s\n",
						             strCLKRate.c_str());
				}
                char szCLKRateSigma[21];
				string strCLKRateSigma;
				sprintf(szCLKRateSigma, "%20.12e", ASDatum.clkRateSigma);
				stringEraseFloatZero(szCLKRateSigma, strCLKRateSigma);
				if(ASDatum.count == 4)
				{
					fprintf(pCLKFile," %19s %19s\n",
						             strCLKRate.c_str(),
									 strCLKRateSigma.c_str());
					continue;
				}
                char szCLKAcc[21];
				string strCLKAcc;
				sprintf(szCLKAcc, "%20.12e", ASDatum.clkAcc);
				stringEraseFloatZero(szCLKAcc, strCLKAcc);
				if(ASDatum.count == 5)
				{
					fprintf(pCLKFile," %19s %19s %19s\n",
						             strCLKRate.c_str(),
									 strCLKRateSigma.c_str(),
									 strCLKAcc.c_str());
					continue;
				}
                char szCLKAccSigma[21];
				string strCLKAccSigma;
				sprintf(szCLKAccSigma, "%20.12e", ASDatum.clkAccSigma);
				stringEraseFloatZero(szCLKAccSigma, strCLKAccSigma);
				if(ASDatum.count == 6)
				{
					fprintf(pCLKFile," %19s %19s %19s %19s\n",
						             strCLKRate.c_str(),
									 strCLKRateSigma.c_str(),
									 strCLKAcc.c_str(),
									 strCLKAccSigma.c_str());
					continue;
				}
			}
		}
		fclose(pCLKFile);
		return true;
	}

	// 子程序名称： getSatClock_0   
	// 功能：滑动 lagrange 插值获得任意时刻卫星钟差
	// 变量类型：t           : 时间
	//           name        : 卫星名称
	//           clkDatum    : 卫星钟数据
	//           nLagrange   : 插值阶数
	// 输入：t, name, nLagrange
	// 输出：clkDatum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/2/05
	// 版本时间：2007/2/05
	// 修改记录：
	// 备注： 
	bool CLKFile::getSatClock_0(GPST t, string name, CLKDatum& clkDatum, int nLagrange)
	{	
		clkDatum.name = name;
		int clkDataCount = int(m_data.size()); // 数据点个数
		if(t - m_data[0].t < -1.0 ||  t - m_data[clkDataCount - 1].t > 1.0) 
			return false;
		// 获得相邻时间间隔, 默认等距
		double spanSecond = getEpochSpan();
		double spanSecond_T = t - m_data[0].t;
		if(clkDataCount < nLagrange)              // 如果数据点个数小于n, 返回
			return false;
		// 记录n个参考已知点
		// 首先寻找最靠近时间T的左端点，从0开始计数，默认GPS钟差数据是等时间间隔的
		int nLeftPos  = int(floor(spanSecond_T / spanSecond));
		// 理论上nLeftPos左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
		int nLeftNum  = int(floor(nLagrange / 2.0));
		int nRightNum = int(ceil (nLagrange / 2.0));
		int nBegin,nEnd;                            // 位于区间[0, clkDataCount - 1]
		if(nLeftPos - nLeftNum + 1 < 0)             // nEnd - nBegin = nLagrange - 1 
		{
			nBegin = 0;
			nEnd   = nLagrange - 1;
		}
		else if(nLeftPos + nRightNum >= clkDataCount)
		{
			nBegin = clkDataCount - nLagrange;
			nEnd   = clkDataCount - 1;
		}
		else
		{
			nBegin = nLeftPos - nLeftNum + 1;
			nEnd   = nLeftPos + nRightNum;
		}
		// 取出 nBegin 到 nEnd 间 nPRN 号 GPS 卫星的钟差，构成列表 GPSASElementList
		double *xa_t       = new double [nLagrange];
		double *ya_clkBias = new double [nLagrange];
		int validcount = 0;
		for(int i = nBegin; i <= nEnd; i++)
		{
			// 计算当前时间到起始时间的相对秒间隔，记录在xa_t中，等待插值用
			xa_t[i - nBegin] = m_data[i].t - m_data[0].t;
			CLKMap::const_iterator it;
			if((it = m_data[i].ASList.find(clkDatum.name)) != m_data[i].ASList.end())
			{
				if(it->second.count >= 1)
				{// 确保至少观测数据个数大于 
					ya_clkBias[i - nBegin] = it->second.clkBias;
					validcount++;
				}
			}
		}
		if(validcount != nLagrange) 
		{// nPRN 号 GPS 卫星数据完整性较验
			delete xa_t;
			delete ya_clkBias;
			return false;
		}
		// 通过位置插值出钟差和钟差变化率
		InterploationLagrange(xa_t, ya_clkBias, nLagrange, spanSecond_T, clkDatum.clkBias, clkDatum.clkRate);
		delete xa_t;
		delete ya_clkBias;
		return true;
	}

	// 子程序名称： getSatClock   
	// 功能：滑动 lagrange 插值获得任意时刻卫星钟差
	// 变量类型：t           : 时间
	//           name        : 卫星名称
	//           clkDatum    : 卫星钟数据
	//           nLagrange   : 插值阶数
	// 输入：t, name, nLagrange
	// 输出：clkDatum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/2/05
	// 版本时间：2007/2/05
	// 修改记录：
	// 备注： 
	bool CLKFile::getSatClock(GPST t, string name, CLKDatum& clkDatum, int nLagrange)
	{	
		clkDatum.name = name;
		int clkDataCount = int(m_data.size()); // 数据点个数
		if(t - m_data[0].t < -1.0 ||  t - m_data[clkDataCount - 1].t > 1.0) 
			return false;
		// 获得相邻时间间隔, 默认等距
		double spanSecond = getEpochSpan();
		double spanSecond_T = t - m_data[0].t;
		if(clkDataCount < nLagrange)              // 如果数据点个数小于n, 返回
			return false;
		// 记录n个参考已知点
		// 首先寻找最靠近时间T的左端点，从0开始计数，默认GPS钟差数据是等时间间隔的
		int nLeftPos  = int(floor(spanSecond_T / spanSecond));
		// 理论上nLeftPos左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
		int nLeftNum  = int(floor(nLagrange / 2.0));
		int nRightNum = int(ceil (nLagrange / 2.0));
		int nBegin,nEnd;                            // 位于区间[0, clkDataCount - 1]
		if(nLeftPos - nLeftNum + 1 < 0)             // nEnd - nBegin = nLagrange - 1 
		{
			nBegin = 0;
			nEnd   = nLagrange - 1;
		}
		else if(nLeftPos + nRightNum >= clkDataCount)
		{
			nBegin = clkDataCount - nLagrange;
			nEnd   = clkDataCount - 1;
		}
		else
		{
			nBegin = nLeftPos - nLeftNum + 1;
			nEnd   = nLeftPos + nRightNum;
		}
		// 取出 nBegin 到 nEnd 间 nPRN 号 GPS 卫星的钟差，构成列表 GPSASElementList
		double *xa_t       = new double [nLagrange];
		double *ya_clkBias = new double [nLagrange];
		int validcount = 0;
		for(int i = nBegin; i <= nEnd; i++)
		{
			// 计算当前时间到起始时间的相对秒间隔，记录在xa_t中，等待插值用
			xa_t[i - nBegin] = m_data[i].t - m_data[0].t;
			CLKMap::const_iterator it;
			if((it = m_data[i].ASList.find(clkDatum.name)) != m_data[i].ASList.end())
			{
				if(it->second.count >= 1)
				{// 确保至少观测数据个数大于 
					ya_clkBias[i - nBegin] = it->second.clkBias;
					validcount++;
				}
			}
		}
		if(validcount != nLagrange) 
		{// nPRN 号 GPS 卫星数据完整性较验
			delete xa_t;
			delete ya_clkBias;
			return false;
		}
		// 通过位置插值出钟差和钟差变化率
		InterploationLagrange(xa_t, ya_clkBias, nLagrange, spanSecond_T, clkDatum.clkBias, clkDatum.clkRate);
		delete xa_t;
		delete ya_clkBias;
		return true;

		// 钟差的解算会导致卫星绝对钟差中包含参考站钟的变化, 如果参考站钟变化比较剧烈此时会对插值也带来一定影响
		//if(getSatClock_0(t, name, clkDatum, nLagrange)) 
		//	return true;
		//else
		//{// 针对 igc 实时星历数据存在部分小间隔缺失现象, 进行 5 分钟粗采样
		// // 钟差长时间中断后参考钟可能存在调整, 例如 AS G13  2014 04 18 21 03 20.000000 - AS G03  2014 04 18 21 04 40.000000
		// // 此时插值会带来问题
		//	clkDatum.name = name;
		//	int clkDataCount = int(m_data.size()); // 数据点个数
		//	if(t - m_data[0].t < -1.0 ||  t - m_data[clkDataCount - 1].t > 1.0) 
		//		return false;
		//	// 获得相邻时间间隔, 默认等距
		//	double spanSecond = getEpochSpan();
		//	if(spanSecond == 10.0) // 处理 igc 实时星历数据, 避免短时星历不连续情况
		//	{
		//		double span_fix = 60.0;  
		//		int span_num = int(span_fix / spanSecond);
		//		int count_fix = int(ceil((m_data[clkDataCount - 1].t - m_data[0].t) / span_fix)); 
		//		if(count_fix < nLagrange)
		//			return false;
		//		double spanSecond_T = t - m_data[0].t;
		//		int nLeftPos_fix  = int(spanSecond_T / span_fix);
		//		// 理论上nLeftPos左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
		//		int nLeftNum_fix  = int(floor(nLagrange / 2.0));
		//		int nRightNum_fix = int(ceil (nLagrange / 2.0));
		//		int nBegin_fix, nEnd_fix;               // 位于区间[0, nSP3DataNumber - 1]
		//		if(nLeftPos_fix - nLeftNum_fix + 1 < 0) // nEnd_fix - nBegin_fix = nLagrange - 1 
		//		{
		//			nBegin_fix = 0;
		//			nEnd_fix   = nLagrange - 1;
		//		}
		//		else if(nLeftPos_fix + nRightNum_fix >= count_fix)
		//		{
		//			nBegin_fix = count_fix - nLagrange;
		//			nEnd_fix   = count_fix - 1;
		//		}
		//		else
		//		{
		//			nBegin_fix = nLeftPos_fix - nLeftNum_fix + 1;
		//			nEnd_fix   = nLeftPos_fix + nRightNum_fix;
		//		} 
		//		// 取出 nBegin 到 nEnd 间 nPRN 号 GPS 卫星的钟差，构成列表 GPSASElementList
		//		double *xa_t       = new double [nLagrange];
		//		double *ya_clkBias = new double [nLagrange];
		//		int validcount = 0;
		//		for(int i = nBegin_fix; i <= nEnd_fix; i++)
		//		{
		//			bool bFind = false;
		//			for(int j = i * span_num; j < (i + 1) * span_num; j++)
		//			{
		//				
		//				if(j < clkDataCount) // 确保不溢出
		//				{
		//					CLKMap::const_iterator it = m_data[j].ASList.find(name);
		//					if(it != m_data[j].ASList.end())
		//					{
		//						if(it->second.count >= 1) 
		//						{
		//							bFind = true;
		//							xa_t[i - nBegin_fix] = m_data[j].t - m_data[0].t;
		//							ya_clkBias[i - nBegin_fix] = it->second.clkBias;
		//							validcount++;
		//							break; // 每隔区间只要保证 1 个点即可
		//						}
		//					}
		//				}
		//			}
		//			if(!bFind)
		//			{
		//				/*char info[100];
		//				sprintf(info, "%s %s 星历数据不完整!", t.toString().c_str(), name.c_str());
		//				RuningInfoFile::Add(info);*/
		//				break; // 只要缺失一个点及时跳出
		//			}
		//		}
		//		if(validcount != nLagrange) 
		//		{// nPRN 号 GPS 卫星数据完整性较验
		//			delete xa_t;
		//			delete ya_clkBias;
		//			return false;
		//		}
		//		// 通过位置插值出钟差和钟差变化率
		//		InterploationLagrange(xa_t, ya_clkBias, nLagrange, spanSecond_T, clkDatum.clkBias, clkDatum.clkRate);
		//		char info[100];
		//		sprintf(info, "%s %s 星历数据粗采样插值%20.12e!", t.toString().c_str(), name.c_str(), clkDatum.clkBias);
		//		RuningInfoFile::Add(info);
		//		delete xa_t;
		//		delete ya_clkBias;
		//		return true;
		//	}
		//	else
		//		return false;
		//}
	}

	// 子程序名称： getSatClock   
	// 功能：滑动 lagrange 插值获得任意时刻卫星钟差
	// 变量类型：t           : 时间
	//           nPRN        : 卫星号
	//           clkDatum    : 卫星钟数据
	//           nLagrange   : 插值阶数
	//           cSatSystem  : 导航系统标记
	// 输入：t, nPRN, nLagrange, cSatSystem
	// 输出：clkDatum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/2/05
	// 版本时间：2007/2/05
	// 修改记录：
	// 备注： 
	bool CLKFile::getSatClock(GPST t, int nPRN, CLKDatum& clkDatum, int nLagrange, char cSatSystem)
	{	
		char szASName[5];
		sprintf(szASName, "%c%02d ", cSatSystem, nPRN);
        szASName[4] = '\0';
		return getSatClock(t, szASName, clkDatum, nLagrange);
	}

	// 子程序名称： getStaClock   
	// 功能：滑动 lagrange 插值获得任意时刻测站钟差
	// 变量类型：t           : 时间
	//           name        : 测站名称
	//           clkDatum    : 卫星钟数据
	//           nLagrange   : 插值阶数
	// 输入：t, name, nLagrange
	// 输出：clkDatum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/10/15
	// 版本时间：2012/10/15
	// 修改记录：
	// 备注： 
	bool CLKFile::getStaClock(GPST t, string name, CLKDatum& clkDatum, int nLagrange)
	{
		clkDatum.name = name;
		int clkDataCount = int(m_data.size()); // 数据点个数
		if(t - m_data[0].t < -1.0 ||  t - m_data[clkDataCount - 1].t > 1.0)
			return false;
		// 获得相邻时间间隔, 默认等距
		double spanSecond = getEpochSpan();
		double spanSecond_T = t - m_data[0].t;
		if(clkDataCount < nLagrange)              // 如果数据点个数小于n, 返回
			return false;
		// 记录n个参考已知点
		// 首先寻找最靠近时间T的左端点，从0开始计数，默认GPS钟差数据是等时间间隔的
		int nLeftPos  = int(floor(spanSecond_T / spanSecond));
		// 理论上nLeftPos左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
		int nLeftNum  = int(floor(nLagrange / 2.0));
		int nRightNum = int(ceil (nLagrange / 2.0));
		int nBegin,nEnd;                            // 位于区间[0, clkDataCount - 1]
		if(nLeftPos - nLeftNum + 1 < 0)             // nEnd - nBegin = nLagrange - 1 
		{
			nBegin = 0;
			nEnd   = nLagrange - 1;
		}
		else if(nLeftPos + nRightNum >= clkDataCount)
		{
			nBegin = clkDataCount - nLagrange;
			nEnd   = clkDataCount - 1;
		}
		else
		{
			nBegin = nLeftPos - nLeftNum + 1;
			nEnd   = nLeftPos + nRightNum;
		}
		// 取出 nBegin 到 nEnd 间 nPRN 号 GPS 卫星的钟差，构成列表 GPSASElementList
		double *xa_t       = new double [nLagrange];
		double *ya_clkBias = new double [nLagrange];
		int validcount = 0;
		for(int i = nBegin; i <= nEnd; i++)
		{
			// 计算当前时间到起始时间的相对秒间隔，记录在xa_t中，等待插值用
			xa_t[i - nBegin] = m_data[i].t - m_data[0].t;
			CLKMap::const_iterator it;
			if((it = m_data[i].ARList.find(clkDatum.name)) != m_data[i].ARList.end())
			{
				if(it->second.count >= 1)
				{// 确保至少观测数据个数大于 1
					ya_clkBias[i - nBegin] = it->second.clkBias;
					validcount++;
				}
			}
		}
		if(validcount != nLagrange) 
		{// nPRN 号 GPS 卫星数据完整性较验
			delete xa_t;
			delete ya_clkBias;
			return false;
		}
		// 通过位置插值出钟差和钟差变化率
		InterploationLagrange(xa_t, ya_clkBias, nLagrange, spanSecond_T, clkDatum.clkBias, clkDatum.clkRate);
		delete xa_t;
		delete ya_clkBias;
		return true;
	}
	// 子程序名称： getLeoClock   
	// 功能：滑动 lagrange 插值获得任意时刻测站钟差
	// 变量类型：t           : 时间
	//           name        : 测站名称
	//           clkDatum    : 卫星钟数据
	//           nLagrange   : 插值阶数
	// 输入：t, name, nLagrange
	// 输出：clkDatum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/10/15
	// 版本时间：2012/10/15
	// 修改记录：基于getStaClock添加数据缺失处理，TH02存在部分历元无钟差情况，该部分采用线性插值，张厚喆，2020.11.1
	// 备注： 
	bool CLKFile::getLeoClock(GPST t, string name, CLKDatum& clkDatum, int nLagrange)
	{
		clkDatum.name = name;
		int nLagrange_left  = int(floor(nLagrange / 2.0));   
		int nLagrange_right = int(ceil (nLagrange / 2.0));
		if(int(m_data.size()) < nLagrange)           
			return false;
		// 插值时考虑到钟差的影响，以准确轨道采样时刻为参考 
		DayTime t_Begin = m_data[0].t;               
		DayTime t_End   = m_data[m_data.size() - 1].t; 
		double  span_total = t_End - t_Begin;
		double  span_t = t - t_Begin; 
		if(span_t < 0) 
		{
			CLKMap::const_iterator it;
			if((it = m_data[0].ARList.find(clkDatum.name)) != m_data[0].ARList.end())
			{
				clkDatum.clkBias = it->second.clkBias;
				return true;
			}
			else
				return false;
		}
		else if(span_t > span_total)
		{
			CLKMap::const_iterator it;
			if((it = m_data[m_data.size() - 1].ARList.find(clkDatum.name)) != m_data[m_data.size() - 1].ARList.end())
			{
				clkDatum.clkBias = it->second.clkBias;
				return true;
			}
			else
				return false;
		}

		int nLeftPos = -1;
		// 采用二分法, 2008/05/11
		size_t left  = 1;
		size_t right = m_data.size() - 1;
		int n = 0;
		while(left < right)
		{
			n++;
			int middle = int(left + right) / 2;
			double time_L = m_data[middle - 1].t - t_Begin;
			double time_R = m_data[middle].t - t_Begin;
			if(span_t >= time_L && span_t <= time_R) 
			{// 终止条件
				nLeftPos = middle - 1;
				break;
			}
			if(span_t < time_L) 
				right = middle - 1;
			else 
				left  = middle + 1;////???
				//left  = middle;
		}
		if(right == left)
		{
			double time_L = m_data[left - 1].t - t_Begin;
			double time_R = m_data[left].t - t_Begin;
			if(span_t >= time_L && span_t <= time_R) 
			{// 终止条件
				nLeftPos = int(left - 1);
			}
		}
		if(nLeftPos == -1 || (nLeftPos+1) == int(m_data.size()))
		{
			return false;
		}
		// 确定插值区间位置 [nBegin, nEnd]，nEnd - nBegin + 1 = nLagrange
		int nBegin, nEnd; 
		if(nLeftPos - nLagrange_left + 1 < 0) 
		{
			nBegin = 0;
			nEnd   = nLagrange - 1;
		}
		else if(nLeftPos + nLagrange_right >= int(m_data.size()))
		{
			nBegin = int(m_data.size()) - nLagrange;
			nEnd   = int(m_data.size()) - 1;
		}
		else
		{
			nBegin = nLeftPos - nLagrange_left + 1;
			nEnd   = nLeftPos + nLagrange_right;
		}
		double *xa_t =new double [nLagrange];
		double *ya_clkBias =new double [nLagrange];
		int validcount = 0;
		for(int i = nBegin; i <= nEnd; i++)
		{
			xa_t[i - nBegin] = m_data[i].t - t_Begin;
			CLKMap::const_iterator it;
			if((it = m_data[i].ARList.find(clkDatum.name)) != m_data[i].ARList.end())
			{
				if(it->second.count >= 1)
				{// 确保至少观测数据个数大于 1
					ya_clkBias[i - nBegin] = it->second.clkBias;
					validcount++;
				}
			}
		}
		if(validcount != nLagrange) //无法拉格朗日插值则线性插值
		{// 线性插值
			delete xa_t;
			delete ya_clkBias;
			double interval = m_data[nLeftPos + 1].t - m_data[nLeftPos].t;
			double u = (t - m_data[nLeftPos].t) / interval;
			double clk0 = 0.0;
			double clk1 = 0.0;
			CLKMap::const_iterator it;
			if((it = m_data[nLeftPos].ARList.find(clkDatum.name)) != m_data[nLeftPos].ARList.end())
			{
				//if(it->second.count >= 1)
				//{// 确保至少观测数据个数大于 1
					clk0 = it->second.clkBias;
				//}
			}
			if((it = m_data[nLeftPos + 1].ARList.find(clkDatum.name)) != m_data[nLeftPos + 1].ARList.end())
			{
				//if(it->second.count >= 1)
				//{// 确保至少观测数据个数大于 1
					clk1 = it->second.clkBias;
				//}
			}
			clkDatum.clkBias = (1 - u) * clk0 + u * clk1;
			return true;
		}
		else 
		{
			 //通过位置插值出钟差和钟差变化率
			InterploationLagrange(xa_t, ya_clkBias, nLagrange, span_t, clkDatum.clkBias);
			delete xa_t;
			delete ya_clkBias;
			return true;
		}
	}
}
