#include "igs05atxFile.hpp"

namespace NUDTTK
{
	const char AntexFile_HeaderLabel::szAntexVerSyst[]		= "ANTEX VERSION / SYST";
	const char AntexFile_HeaderLabel::szPCVTypeRefAnt[]     = "PCV TYPE / REFANT   ";
	const char AntexFile_HeaderLabel::szComment[]           = "COMMENT             ";
	const char AntexFile_HeaderLabel::szEndOfHead[]         = "END OF HEADER       ";
	const char AntexFile_HeaderLabel::szStartOfAnt[]		= "START OF ANTENNA    ";
	const char AntexFile_HeaderLabel::szTypeSerialNo[]		= "TYPE / SERIAL NO    ";
	const char AntexFile_HeaderLabel::szMethByDate[]		= "METH / BY / # / DATE";
	const char AntexFile_HeaderLabel::szDAZI[]				= "DAZI                ";
	const char AntexFile_HeaderLabel::szZen1Zen2DZen[]		= "ZEN1 / ZEN2 / DZEN  ";
	const char AntexFile_HeaderLabel::szOfFreqs[]			= "# OF FREQUENCIES    ";
	const char AntexFile_HeaderLabel::szValidFrom[]			= "VALID FROM          ";
	const char AntexFile_HeaderLabel::szValidUntil[]		= "VALID UNTIL         ";
	const char AntexFile_HeaderLabel::szSinexCode[]			= "SINEX CODE          ";
	const char AntexFile_HeaderLabel::szStartOfFreq[]		= "START OF FREQUENCY  ";
	const char AntexFile_HeaderLabel::szNEU[]				= "NORTH / EAST / UP   ";
	const char AntexFile_HeaderLabel::szEndOfFreq[]			= "END OF FREQUENCY    ";
	const char AntexFile_HeaderLabel::szStartOfFreqRms[]	= "START OF FREQ RMS   ";
	const char AntexFile_HeaderLabel::szEndOfFreqRms[]		= "END OF FREQ RMS     ";
	const char AntexFile_HeaderLabel::szEndOfAnt[]			= "END OF ANTENNA      ";
	const char AntexFile_HeaderLabel::szNOAZI[]				= "NOAZI";

	igs05atxFile::igs05atxFile(void)
	{
	}

	igs05atxFile::~igs05atxFile(void)
	{
	}
	// 子程序名称： open   
	// 功能：卫星及测站天线相位中心修正数据解析 
	// 变量类型：strAtxFileName : 数据文件路径
	// 输入：strAtxFileName
	// 输出：
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/4/10
	// 版本时间：2013/4/5
	// 修改记录：1、添加无效数据行的判断；2013-4-12
	//			 2、添加'G 1'-> 'G01'处理；2013-4-18
	// 备注：
	bool igs05atxFile::open(string  strAtxFileName)
	{
		// TODO:添加代码
		if(!isWildcardMatch(strAtxFileName.c_str(), "*.atx", true))
		{
			printf(" %s 文件名不匹配!\n", strAtxFileName.c_str());
			return false;
		}
		FILE *pAtxFile = fopen(strAtxFileName.c_str(),"r+t");
		if(pAtxFile == NULL)	// fopen不返回NULL？（2013-04-07）
		{
			return false;
		}
		// 读取ATX文件头，循环读取每一行数据，直到 END OF HEADER
		bool bFlag = true;
		while(bFlag)
		{
			char line[100];
			fgets(line, 100, pAtxFile);
			string strLine      = line;
			string strLineLabel = line;
			strLineLabel.erase(0, 60);				// 从第 0 个字符开始，删除 60 个字符
			size_t nPos_n = strLineLabel.find('\n');// 剔除换行符 '\n'
			strLineLabel.erase(nPos_n, 1);
			// 长度超过 20 位，截取前 20 位
			while(strLineLabel.length() > 20)
			{
				strLineLabel.erase(strLineLabel.length() - 1, 1);
			}
			// 长度不足 20 位，补齐 20 位
			if(strLineLabel.length() < 20)
			{
				strLineLabel.append(20 - strLineLabel.length(), ' ');
			}
			// 开始读取文件头数据
			if(strLineLabel == AntexFile_HeaderLabel::szAntexVerSyst)
			{	
				sscanf(strLine.c_str(), "%8lf%*12c%1c", &m_header.AntexVersion, &m_header.SatSysytem);	// %8f 错误！！！(2013-4-11)
			}
			else if(strLineLabel == AntexFile_HeaderLabel::szPCVTypeRefAnt)
			{
				sscanf(strLine.c_str(), "%1c%*19c%20c%20c", &m_header.PCVType, m_header.szRefAntType, m_header.szRefAntNumber);
			}
			else if(strLineLabel == AntexFile_HeaderLabel::szComment)		// optional
			{
				m_header.pstrCommentList.push_back(strLine.substr(0, 60));	//???
			}
			else if(strLineLabel == AntexFile_HeaderLabel::szEndOfHead)
			{
				bFlag = false;
			}
			else
			{
				printf(" ANTEX 文件头解析异常！\n");
				return false;
			}
		} // end for while
		// 读取ATX文件中天线相位中心修正数据， 从 START OF ANTENNA 开始，循环读取每一行数据，直到 END OF ANTENNA
		while(!feof(pAtxFile))
		{
			char line[1000];		// 数组长度要保证能够存储 PCV of NOAZI 300-500
			fgets(line, 1000, pAtxFile);
			string strLine      = line;
			// 判断是否为有效数据行(针对空白回车行，2013-04-12)
			if(strLine.length() < 61)
			{
				continue;
			}
			string strLineLabel = line;
			strLineLabel.erase(0, 60);				// 从第 0 个字符开始，删除 60 个字符
			size_t nPos_n = strLineLabel.find('\n');// 剔除换行符 '\n'
			strLineLabel.erase(nPos_n, 1);
			// 出现多个'	’,邵凯，2019/06/11
			// 长度超过 20 位，截取前 20 位
			while(strLineLabel.length() > 20)
			{
				strLineLabel.erase(strLineLabel.length() - 1, 1);
			}
			// 长度不足 20 位，补齐 20 位
			if(strLineLabel.length() < 20)
			{
				strLineLabel.append(20 - strLineLabel.length(), ' ');
			}
			// 略过 START OF ANTENNA 之前的内容
			if(strLineLabel != AntexFile_HeaderLabel::szStartOfAnt)
			{
				continue;
			}
			// 开始读取 START OF ANTENNA 与 END OF ANTENNA 之间的数据
			AntCorrectionBlk  datablk;		// 天线修正数据结构
			bFlag = true;
			while(bFlag)
			{
				fgets(line, 1000, pAtxFile);
				strLine		 = line;
				//strLineLabel = strLine.substr(60, 20);	// 实际读取文件过程中发现：某些 strLineLabel 不足 20 个字符！(2013-04-12)
				//*************************************************************************
				strLineLabel = line;
				strLineLabel.erase(0, 60);				// 从第 0 个字符开始，删除 60 个字符
				size_t nPos_n = strLineLabel.find('\n');// 剔除换行符 '\n'
				strLineLabel.erase(nPos_n, 1);
				// 长度超过 20 位，截取前 20 位
				while(strLineLabel.length() > 20)
				{
					strLineLabel.erase(strLineLabel.length() - 1, 1);
				}
				// 长度不足 20 位，补齐 20 位
				if(strLineLabel.length() < 20)
				{
					strLineLabel.append(20 - strLineLabel.length(), ' ');
				}
				//*************************************************************************
				if(strLineLabel == AntexFile_HeaderLabel::szTypeSerialNo)
				{
					sscanf(strLine.c_str(), "%20c%3c%*17c%4c%*6c%10c", datablk.AntType, datablk.sNN, datablk.sNNN, datablk.COSPARId);
					// 判断该数据块是卫星天线数据还是测站天线数据，2013-4-12
					if(strcmp(datablk.sNNN, "    ") == 0 && strcmp(datablk.COSPARId, "          ") == 0)	
					{
						datablk.flagAntType = 1;
					}
					else
					{
						datablk.flagAntType = 0;
						// 邵凯，2021.04.18
						datablk.AntType[20] = '\0';
						datablk.sNN[3] = '\0';
						datablk.sNNN[4] = '\0';
						datablk.COSPARId[10] = '\0';
					}
				}
				else if(strLineLabel == AntexFile_HeaderLabel::szMethByDate)
				{
					sscanf(strLine.c_str(), "%20c%20c%6d%*4c%10c", datablk.Method, datablk.Agency, &datablk.IdvAntNum, datablk.Date);
				}
				else if(strLineLabel == AntexFile_HeaderLabel::szDAZI)
				{
					sscanf(strLine.c_str(), "%*2c%6lf", &datablk.DAZI);
					if(datablk.DAZI != 0.0)
					{
						int Row = int(360 / datablk.DAZI) + 1; 
						for(int i = 0; i < Row; i++)
						{
							datablk.azimuthList.push_back(i * datablk.DAZI);
						}
					}
				}
				else if(strLineLabel == AntexFile_HeaderLabel::szZen1Zen2DZen)
				{
					sscanf(strLine.c_str(), "%*2c%6lf%6lf%6lf", &datablk.ZEN1, &datablk.ZEN2, &datablk.DZEN);
					int Col = int((datablk.ZEN2 - datablk.ZEN1) / datablk.DZEN) + 1;
					for(int j = 0; j < Col; j++)
					{
						datablk.zenithList.push_back(datablk.ZEN1 + j * datablk.DZEN);
					}
				}
				else if(strLineLabel == AntexFile_HeaderLabel::szOfFreqs)
				{
					sscanf(strLine.c_str(), "%6d", &datablk.FreqNum);
				}
				else if(strLineLabel == AntexFile_HeaderLabel::szValidFrom)		// optional
				{
					sscanf(strLine.c_str(), "%6d%6d%6d%6d%6d%13lf", &datablk.ValidFrom.year,
																	 &datablk.ValidFrom.month,
																	 &datablk.ValidFrom.day,
																	 &datablk.ValidFrom.hour,
																	 &datablk.ValidFrom.minute,
																	 &datablk.ValidFrom.second);
				}
				else if(strLineLabel == AntexFile_HeaderLabel::szValidUntil)	// optional
				{
					sscanf(strLine.c_str(), "%6d%6d%6d%6d%6d%13lf", &datablk.ValidUntil.year,
																	 &datablk.ValidUntil.month,
																	 &datablk.ValidUntil.day,
																	 &datablk.ValidUntil.hour,
																	 &datablk.ValidUntil.minute,
																	 &datablk.ValidUntil.second);
				}
				else if(strLineLabel == AntexFile_HeaderLabel::szSinexCode)		// optional
				{
					sscanf(strLine.c_str(), "%10c", &datablk.SinexCode);
				}
				else if(strLineLabel == AntexFile_HeaderLabel::szComment)		// optional
				{
					datablk.pstrCommentList.push_back(strLine.substr(0, 60));	//???
				}
				// 开始读取 START OF FREQUENCY 与 END OF FREQUENCY 之间的数据
				else if(strLineLabel == AntexFile_HeaderLabel::szStartOfFreq)
				{
					char freqIndex[3 + 1];
					freqIndex[3] = '\0';
					strLine.copy(freqIndex, 3, 3);		// 略过3X，读取 (A1 + I2) 个字符
					if(freqIndex[1] = ' ')				// 'G 1'->'G01'(2013-04-18)
					{
						freqIndex[1] = '0';
					}
					datablk.flagFreqList.push_back(freqIndex);
					// 读取 PCO 数据，"NORTH / EAST / UP   " 行
					fgets(line, 1000, pAtxFile);
					strLine = line;
					POS3D	PCO;
					sscanf(strLine.c_str(), "%10lf%10lf%10lf", &PCO.x, &PCO.y, &PCO.z);
					datablk.PCOList.push_back(PCO);
					// 读取 NOAZI_PCV 数据，"NOAZI" 行
					fgets(line, 1000, pAtxFile);
					strLine = line;
					strLine.erase(0, 8);	// 略过3X，A5 - '   NOAZI' 8 个字符
					double matElement = 0.0;
					char str_matElement[8 + 1];
					str_matElement[8] = '\0';
					int Col = int((datablk.ZEN2 - datablk.ZEN1) / datablk.DZEN) + 1;
					Matrix matNOAZI_PCV(1, Col);
					for(int j = 0; j < Col; j++)
					{
						strLine.copy(str_matElement, 8, 8*j);
						sscanf(str_matElement, "%8lf", &matElement);
						matNOAZI_PCV.SetElement(0, j, matElement);
					}
					datablk.NOAZI_PCVList.push_back(matNOAZI_PCV);
					// 读取 AZIDEPT_PCV 数据(optional)
					if(datablk.DAZI != 0.0)
					{
						int Row = int(360 / datablk.DAZI) + 1;
						int Col = int((datablk.ZEN2 - datablk.ZEN1) / datablk.DZEN) + 2;
						Matrix matAZIDEPT_PCV(Row, Col);
						for(int i = 0; i < Row; i++)
						{
							fgets(line, 1000, pAtxFile);
							strLine = line;
							sscanf(strLine.c_str(), "%8lf", &matElement);
							matAZIDEPT_PCV.SetElement(i, 0, matElement);
							for(int j = 1; j < Col; j++)
							{
								strLine.copy(str_matElement, 8, 8*j);
								sscanf(str_matElement, "%8lf", &matElement);
								matAZIDEPT_PCV.SetElement(i, j, matElement);
							}
						}
						datablk.AZIDEPT_PCVList.push_back(matAZIDEPT_PCV);
					}
					// 略过 END OF FREQUENCY 行
					fgets(line, 1000, pAtxFile);
				}
				// 开始读取 START OF FREQ RMS 与 END OF FREQ RMS 之间的数据(optional)
				else if(strLineLabel == AntexFile_HeaderLabel::szStartOfFreqRms)
				{
					// 读取 RmsPCO 数据，"NORTH / EAST / UP   " 行
					fgets(line, 1000, pAtxFile);
					strLine = line;
					POS3D	RmsPCO;
					sscanf(strLine.c_str(), "%10lf%10lf%10lf", &RmsPCO.x, &RmsPCO.y, &RmsPCO.z);
					datablk.RmsPCOList.push_back(RmsPCO);
					// 读取 RmsNOAZI_PCV 数据，"NOAZI" 行
					fgets(line, 1000, pAtxFile);
					strLine = line;
					strLine.erase(0, 8);	// 略过3X，A5 - '   NOAZI' 8 个字符 
					double matElement = 0.0;
					char str_matElement[8 + 1];
					str_matElement[8] = '\0';
					int Col = int((datablk.ZEN2 - datablk.ZEN1) / datablk.DZEN) + 3;
					Matrix matRmsNOAZI_PCV(1, Col);
					for(int j = 0; j < Col; j++)
					{
						strLine.copy(str_matElement, 8, 8*j);
						sscanf(str_matElement, "%8lf", &matElement);
						matRmsNOAZI_PCV.SetElement(0, j, matElement);
					}
					datablk.RmsNOAZI_PCVList.push_back(matRmsNOAZI_PCV);
					// 读取 RmsAZIDEPT_PCV 数据(optional)
					if(datablk.DAZI != 0.0)
					{
						int Row = int(360 / datablk.DAZI) + 1;
						int Col = int((datablk.ZEN2 - datablk.ZEN1) / datablk.DZEN) + 4;
						Matrix matRmsAZIDEPT_PCV(Row, Col);
						for(int i = 0; i < Row; i++)
						{
							fgets(line, 1000, pAtxFile);
							strLine = line;
							sscanf(strLine.c_str(), "%8lf", &matElement);
							matRmsAZIDEPT_PCV.SetElement(i, 0, matElement);
							for(int j = 1; j < Col; j++)
							{
								strLine.copy(str_matElement, 8, 8*j);
								sscanf(str_matElement, "%8lf", &matElement);
								matRmsAZIDEPT_PCV.SetElement(i, j, matElement);
							}
						}
						datablk.RmsAZIDEPT_PCVList.push_back(matRmsAZIDEPT_PCV);
					}
					// 略过 END OF FREQ RMS 行
					fgets(line, 1000, pAtxFile);	
				}
				else if(strLineLabel == AntexFile_HeaderLabel::szEndOfAnt)
				{
					bFlag = false;
				}
				else
				{
					printf(" ANTEX 文件解析异常！\n" );
					return false;
				}
			}// end for while(bFlag)		

			if(datablk.flagAntType == 0)	// 卫星天线数据成员
			{
				SatAntCorrectionMap::iterator it = m_satdata.find(datablk.sNN);			
				if(it == m_satdata.end())	// 若无该卫星的数据，则将 datablk 添加至 m_satdata 的数据列表中
				{
					SatAntCorrectBlkList	datablkList;	// 缓存同一编号卫星、不同时段的天线数据列表
			        datablkList.push_back(datablk);
					m_satdata.insert(SatAntCorrectionMap::value_type(datablk.sNN, datablkList));
				}
				else						// 若已有该卫星的数据，则将 datablk 添加至 it->second 的数据列表中
				{
					it->second.push_back(datablk);
				}
			}
			else							// 测站天线数据成员
			{
				m_recdata.insert(RecAntCorrectionMap::value_type(datablk.AntType, datablk));
			}
		}// end for while(!feof(pAtxFile))
		fclose(pAtxFile);
		return true;
	}

	// 子程序名称： write   
	// 功能：将天线修正数据写到文件 
	// 变量类型：strAtxFileName: 天线修正数据文件名
	// 输入：
	// 输出：天线文件
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/4/13
	// 版本时间：2013/4/5
	// 修改记录：
	// 备注： 
	bool  igs05atxFile::write(string strAtxFileName)
	{
		if(m_satdata.size() == 0 && m_recdata.size() == 0)
		{
			return false;
		}
		FILE *pAtxFile = fopen(strAtxFileName.c_str(),"w+");
		/* Step1：写入文件头数据 */
		// ANTEX VERSION / SYST
		fprintf(pAtxFile, "%8.1f%12s%1c%39s%20s\n",
						   m_header.AntexVersion,
						   " ",
						   m_header.SatSysytem,
						   " ",
						   AntexFile_HeaderLabel::szAntexVerSyst);
		// PCV TYPE / REFANT 
		fprintf(pAtxFile, "%1c%19s%20s%20s%20s\n",
							m_header.PCVType,
							" ",
							m_header.szRefAntType,
							m_header.szRefAntNumber,
							AntexFile_HeaderLabel::szPCVTypeRefAnt);
		// COMMENT (Optional)
		for(size_t i = 0; i < m_header.pstrCommentList.size(); i++)
		{
			fprintf(pAtxFile, "%60s%20s\n", 
								m_header.pstrCommentList[i].c_str(), 
								AntexFile_HeaderLabel::szComment);
		}
		// END OF HEADER
		fprintf(pAtxFile, "%60s%20s\n", " ", AntexFile_HeaderLabel::szEndOfHead);
		/* Step2：写入天线修正数据 */
		vector<AntCorrectionBlk>  datablklist;
		if(m_satdata.size() != 0)
		{
			for(SatAntCorrectionMap::iterator it = m_satdata.begin(); it != m_satdata.end(); it++)
			{
				for(size_t k = 0; k < it->second.size(); k++)
				{
					datablklist.push_back(it->second[k]);
				}
			}
		}
		if(m_recdata.size() != 0)
		{
			for(RecAntCorrectionMap::iterator it = m_recdata.begin(); it != m_recdata.end(); it++)
			{
				datablklist.push_back(it->second);
			}
		}
		for(size_t k = 0; k < datablklist.size(); k++)
		{
			// START OF ANTENNA
			fprintf(pAtxFile, "%60s%20s\n", " ", AntexFile_HeaderLabel::szStartOfAnt);
			// TYPE / SERIAL NO
			fprintf(pAtxFile, "%20s%-20s%-10s%10s%20s\n",
								datablklist[k].AntType,
								datablklist[k].sNN,
								datablklist[k].sNNN,
								datablklist[k].COSPARId,
								AntexFile_HeaderLabel::szTypeSerialNo);
			// METH / BY / # / DATE
			fprintf(pAtxFile, "%-20s%-20s%6d%4s%10s%20s\n",
							   datablklist[k].Method,
							   datablklist[k].Agency,
							   datablklist[k].IdvAntNum,
							   " ",
							   datablklist[k].Date,
							   AntexFile_HeaderLabel::szMethByDate);
			// DAZI
			fprintf(pAtxFile, "%2s%6.1f%52s%20s\n",
								" ",
								datablklist[k].DAZI,
								" ",
								AntexFile_HeaderLabel::szDAZI);
			// ZEN1 / ZEN2 / DZEN
			fprintf(pAtxFile, "%2s%6.1f%6.1f%6.1f%40s%20s\n",
								" ",
								datablklist[k].ZEN1,
								datablklist[k].ZEN2,
								datablklist[k].DZEN,
								" ",
								AntexFile_HeaderLabel::szZen1Zen2DZen);
			// # OF FREQUENCIES
			fprintf(pAtxFile, "%6d%54s%20s\n",
								datablklist[k].FreqNum,
								" ",
								AntexFile_HeaderLabel::szOfFreqs);
			// VALID FROM (Optional)
			if(!(datablklist[k].ValidFrom == DayTime(1980,1,1,0,0,0.0)))
			{
				fprintf(pAtxFile, "%6d%6d%6d%6d%6d%13.7f%17s%20s\n",
									datablklist[k].ValidFrom.year,
									datablklist[k].ValidFrom.month,
									datablklist[k].ValidFrom.day,
									datablklist[k].ValidFrom.hour,
									datablklist[k].ValidFrom.minute,
									datablklist[k].ValidFrom.second,
									" ",
									AntexFile_HeaderLabel::szValidFrom);
			}
			// VALID UNTIL (Optional)
			if(!(datablklist[k].ValidUntil == DayTime(2500,1,1,0,0,0.0)))
			{
				fprintf(pAtxFile, "%6d%6d%6d%6d%6d%13.7f%17s%20s\n",
									datablklist[k].ValidUntil.year,
									datablklist[k].ValidUntil.month,
									datablklist[k].ValidUntil.day,
									datablklist[k].ValidUntil.hour,
									datablklist[k].ValidUntil.minute,
									datablklist[k].ValidUntil.second,
									" ",
									AntexFile_HeaderLabel::szValidUntil);
			}
			// SINEX CODE (Optional)
			if(datablklist[k].SinexCode[1] != '\0')
			{
				fprintf(pAtxFile, "%10s%50s%20s\n",
									datablklist[k].SinexCode,
									" ",
									AntexFile_HeaderLabel::szSinexCode);
			}
			// COMMENT (Optional)
			for(size_t i = 0; i < datablklist[k].pstrCommentList.size(); i++)
			{
				fprintf(pAtxFile, "%60s%20s\n", datablklist[k].pstrCommentList[i].c_str(), AntexFile_HeaderLabel::szComment);
			}
			// START OF FREQUENCY - END OF FREQUENCY
			for(int Id = 0; Id < datablklist[k].FreqNum; Id++)
			{
				// START OF FREQUENCY
				fprintf(pAtxFile, "%3s%3s%54s%20s\n",
									" ",
									datablklist[k].flagFreqList[Id].c_str(),
									" ",
									AntexFile_HeaderLabel::szStartOfFreq);
				// NORTH / EAST / UP
				fprintf(pAtxFile, "%10.2f%10.2f%10.2f%30s%20s\n",
									datablklist[k].PCOList[Id].x,
									datablklist[k].PCOList[Id].y,
									datablklist[k].PCOList[Id].z,
									" ",
									AntexFile_HeaderLabel::szNEU);
				// NOAZI
				fprintf(pAtxFile, "%3s%5s", " ", AntexFile_HeaderLabel::szNOAZI);
				for(int j = 0; j < datablklist[k].NOAZI_PCVList[Id].GetNumColumns(); j++)
				{
					fprintf(pAtxFile, "%8.2f", datablklist[k].NOAZI_PCVList[Id].GetElement(0, j));
				}
				fprintf(pAtxFile, "\n");
				// AZIDEPT_PCV (Optional)
				if(datablklist[k].DAZI != 0.0 && datablklist[k].AZIDEPT_PCVList.size() != 0)
				{
					for(int i = 0; i < datablklist[k].AZIDEPT_PCVList[Id].GetNumRows(); i++)
					{
						fprintf(pAtxFile, "%8.1f", datablklist[k].AZIDEPT_PCVList[Id].GetElement(i, 0));
						for(int j = 1; j < datablklist[k].AZIDEPT_PCVList[Id].GetNumColumns(); j++)
						{
							fprintf(pAtxFile, "%8.2f", datablklist[k].AZIDEPT_PCVList[Id].GetElement(i, j));
						}
						fprintf(pAtxFile, "\n");
					}
				}
				// END OF FREQUENCY
				fprintf(pAtxFile, "%3s%3s%54s%20s\n",
									" ",
									datablklist[k].flagFreqList[Id].c_str(),
									" ",
									AntexFile_HeaderLabel::szEndOfFreq);
				// START OF FREQ RMS - END OF FREQ RMS (Optional)
				if(datablklist[k].RmsPCOList.size() != 0 || 
				   datablklist[k].RmsNOAZI_PCVList.size() != 0 ||
				   datablklist[k].RmsAZIDEPT_PCVList.size() != 0)
				{
					// START OF FREQ RMS
					fprintf(pAtxFile, "%3s%3s%54s%20s\n",
										" ",
										datablklist[k].flagFreqList[Id].c_str(),
										" ",
										AntexFile_HeaderLabel::szStartOfFreqRms);
					// NORTH / EAST / UP
					if(datablklist[k].RmsPCOList.size() != 0)
					{
						fprintf(pAtxFile, "%10.2f%10.2f%10.2f%30s%20s\n",
									datablklist[k].RmsPCOList[Id].x,
									datablklist[k].RmsPCOList[Id].y,
									datablklist[k].RmsPCOList[Id].z,
									" ",
									AntexFile_HeaderLabel::szNEU);
					}
					// NOAZI
					if(datablklist[k].RmsNOAZI_PCVList.size() != 0)
					{
						fprintf(pAtxFile, "%3s%5s", " ", AntexFile_HeaderLabel::szNOAZI);
						for(int j = 0; j < datablklist[k].RmsNOAZI_PCVList[Id].GetNumColumns(); j++)
						{
							fprintf(pAtxFile, "%8.2f", datablklist[k].RmsNOAZI_PCVList[Id].GetElement(0, j));
						}
						fprintf(pAtxFile, "\n");
					}
					// Rms of AZIDEPT_PCV
					if(datablklist[k].RmsAZIDEPT_PCVList.size() != 0)
					{
						for(int i = 0; i < datablklist[k].RmsAZIDEPT_PCVList[Id].GetNumRows(); i++)
						{
							fprintf(pAtxFile, "%8.1f", datablklist[k].RmsAZIDEPT_PCVList[Id].GetElement(i, 0));
							for(int j = 1; j < datablklist[k].RmsAZIDEPT_PCVList[Id].GetNumColumns(); j++)
							{
								fprintf(pAtxFile, "%8.2f", datablklist[k].RmsAZIDEPT_PCVList[Id].GetElement(i, j));
							}
							fprintf(pAtxFile, "\n");
						}
					}
					// END OF FREQ RMS
					fprintf(pAtxFile, "%3s%3s%54s%20s\n",
										" ",
										datablklist[k].flagFreqList[Id].c_str(),
										" ",
										AntexFile_HeaderLabel::szEndOfFreqRms);
				}
			}// end for Id
			// END OF ANTENNA
			fprintf(pAtxFile, "%60s%20s\n", " ", AntexFile_HeaderLabel::szEndOfAnt);
		}// end for k
		fclose(pAtxFile);
		return true;
	}

	// 子程序名称： write   
	// 功能：将天线修正数据写到文件 "SinexCode.atx"
	// 变量类型：
	// 输入：
	// 输出：
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/4/13
	// 版本时间：2013/4/5
	// 修改记录：
	// 备注： 
	bool  igs05atxFile::write()
	{
		string FileName;
		if(m_satdata.size() != 0)
		{
			SatAntCorrectionMap::iterator it = m_satdata.begin();
			FileName = it->second[1].SinexCode;
		}
		else if(m_recdata.size() != 0)
		{
			RecAntCorrectionMap::iterator it = m_recdata.begin();
			FileName = it->second.SinexCode;
		}
		else
		{
			return false;
		}
		FileName = FileName.append(".atx");
		return write(FileName);
	}

	// 子程序名称： getAntCorrectBlk   
	// 功能：获取天线修正数据结构
	// 变量类型：index_Name： 卫星或测站索引
	//			 t：          首次观测历元
	//           flag_Sat：   平台类型，true - 卫星，false - 测站
	//           datablk：    天线 PCO、PCV 修正数据结构
	// 输入：index_Name，t, flag_Sat
	// 输出：datablk
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/4/16
	// 版本时间：2013/4/5
	// 修改记录：
	// 备注： 
	bool igs05atxFile::getAntCorrectBlk(string index_Name, GPST t, AntCorrectionBlk &datablk, bool flag_Sat)
	{
		// 获取卫星天线修正数据
		if(flag_Sat)
		{
			SatAntCorrectionMap::iterator it = m_satdata.find(index_Name);
			if(it == m_satdata.end())
			{
				//printf("没有卫星 %s 的天线修正数据！\n", index_Name.c_str());
				return false;
			}
			bool T_Valid = false;
			for(size_t k = 0; k < it->second.size(); k++)
			{
				if((it->second[k].ValidFrom < t  || it->second[k].ValidFrom == t) &&
				   (t < it->second[k].ValidUntil || it->second[k].ValidUntil == t))
				{
					datablk = it->second[k];
					T_Valid = true;
				}
			}
			if(!T_Valid)
			{
				//printf("卫星 %s 无此天的天线修正数据！\n", index_Name.c_str());
				return false;
			}
		}
		// 获取测站天线修正数据
		else
		{
			RecAntCorrectionMap::iterator it = m_recdata.find(index_Name);
			if(it == m_recdata.end())
			{
				//printf("没有 %s 类型的天线修正数据！\n", index_Name.c_str());
				return false;
			}
			else
			{
				datablk = it->second;
			}
		}
		return true;
	}

	// 子程序名称： correctSatAntPCOPCV   
	// 功能：计算伪距的卫星天线修正量
	// 变量类型：datablk：	卫星天线数据结构
	//			 FreqId：	频点索引
	//           recPos：	测站位置坐标
	//           satPos：	卫星位置坐标
	//           sunPos：	太阳位置坐标
	//           bOn_PCV：	是否修正PCV
	// 输入：datablk，FreqId, recPos, satPos，sunPos，bOn_PCV
	// 输出：卫星天线相位中心修正量，单位：mm
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/4/17
	// 版本时间：2013/4/5
	// 修改记录：
	// 备注： recPos, satPos, sunPos 必须在统一坐标系下
	double igs05atxFile::correctSatAntPCOPCV(AntCorrectionBlk datablk, int FreqId, POS3D recPos, POS3D satPos, POS3D sunPos, bool bOn_PCV)
	{
		// 计算星固系三轴矢量
		double correctdistance = 0.0;
		POS3D es = vectorNormal(sunPos - satPos);		// 卫星到太阳的单位矢量
		POS3D ez = vectorNormal(satPos) * (-1.0);		// Z轴单位矢量，卫星指向地心
		POS3D ey;
		vectorCross(ey, ez, es);					
		ey = vectorNormal(ey);							// Y轴单位矢量，卫星太阳帆板转轴方向
		POS3D ex;
		vectorCross(ex, ey, ez);					
		ex = vectorNormal(ex);							// X轴单位矢量，右手系
		// 此段代码的作用？？？鞠 冰，2014-11-10
		if(vectorDot(es, ex) < 0)
		{
			ex = ex * (-1.0);
			ey = ey * (-1.0);	
		}
		// 此段代码的作用？？？鞠 冰，2014-11-10
		// PCO修正
		double correctdistance_PCO = 0.0;
		POS3D satPCO = datablk.PCOList[FreqId];
		POS3D vecPCO = ex * satPCO.x + ey * satPCO.y + ez * satPCO.z;
        POS3D vecLOS = vectorNormal(recPos - satPos);	// 视线单位矢量，卫星指向测站
		// 将卫星天线相位中心偏移矢量 vecPCO 向视线矢量 vecLOS 投影，单位：m
		correctdistance_PCO = vectorDot(vecPCO, vecLOS) / 1000.0;	
		// PCV修正
		if(bOn_PCV)
		{
			double correctdistance_PCV = 0.0;
			// 计算视线矢量在星固系中的天顶角，单位：度
			double zenith = 0.0;
			zenith = acos(vectorDot(vecLOS, ez))*180/PI;
			int N = int(datablk.zenithList.size());
			if(zenith < datablk.zenithList[0] || zenith > datablk.zenithList[N - 1])
			{
				//printf("卫星 %s 天顶角越界，zenith = %4.1f 取最邻近的PCV修正值！\n", datablk.sNN, zenith);
				Matrix matPCV = datablk.NOAZI_PCVList[FreqId];
				correctdistance_PCV = matPCV.GetElement(0, N - 1);
				correctdistance_PCV = correctdistance_PCV / 1000.0;
				correctdistance = correctdistance_PCO - correctdistance_PCV; //  谷德峰, 2014/11/08, correctdistance_PCO + correctdistance_PCV
			}
			else
			{
				if(datablk.DAZI != 0.0)		// 与方位角有关 PCV 修正
				{
					//printf("暂时不修正与方位角有关的卫星天线PCV!\n");
				}
				Matrix matPCV = datablk.NOAZI_PCVList[FreqId];
				// 搜索天顶角 zenith 所在的区间
				int nBegin_E  = -1;
				int nEnd_E    = -1;
				for(int j = 0; j < N - 1; j++)
				{
					if(datablk.zenithList[j] <= zenith && zenith <= datablk.zenithList[j + 1])
					{
						nBegin_E  = j;
						nEnd_E    = j + 1;
						break;
					}
				}
				// 线性插值求 zenith 对应的 PCV 修正值
				correctdistance_PCV = matPCV.GetElement(0, nBegin_E)*(datablk.zenithList[nEnd_E] - zenith)/datablk.DZEN
									+ matPCV.GetElement(0, nEnd_E)*(zenith - datablk.zenithList[nBegin_E])/datablk.DZEN;

				correctdistance_PCV = correctdistance_PCV / 1000.0;
				correctdistance = correctdistance_PCO - correctdistance_PCV; // 谷德峰, 2014/11/08, correctdistance_PCO + correctdistance_PCV
			}
		}
		else
		{
			correctdistance = correctdistance_PCO;
		}
		return correctdistance;
	}
	// 子程序名称： correctSatAntPCOPCV_YawFixed   
	// 功能：卫星在YawFixed姿态模式下的卫星天线修正量(用于BDS GEO卫星)
	// 变量类型：datablk：	卫星天线数据结构
	//			 FreqId：	频点索引
	//           vecLOS：	视线单位矢量，卫星指向测站
	//           ex：	    X轴单位矢量，右手系(卫星飞行方向)
	//           ey：	    Y轴单位矢量，卫星太阳帆板转轴方向
	//           ez：	    Z轴单位矢量，卫星指向地心
	//           bOn_PCV：	是否修正PCV
	// 输入：datablk，FreqId, vecLOS, ex，ey，ez,bOn_PCV
	// 输出：卫星天线相位中心修正量，单位：mm
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/8/24
	// 版本时间：2014/8/24
	// 修改记录：
	// 备注：
	double igs05atxFile::correctSatAntPCOPCV_YawFixed(AntCorrectionBlk datablk, int FreqId, POS3D vecLOS, POS3D ex, POS3D ey, POS3D ez, bool bOn_PCV)
	{
		//// 计算星固系三轴矢量
		double correctdistance = 0.0;
		//POS3D es = vectorNormal(sunPos - satPos);		// 卫星到太阳的单位矢量
		//POS3D ez = vectorNormal(satPos) * (-1.0);		// Z轴单位矢量，卫星指向地心
		//POS3D ey;
		//vectorCross(ey, ez, es);					
		//ey = vectorNormal(ey);							// Y轴单位矢量，卫星太阳帆板转轴方向
		//POS3D ex;
		//vectorCross(ex, ey, ez);					
		//ex = vectorNormal(ex);							// X轴单位矢量，右手系
		//if(vectorDot(es, ex) < 0)
		//{
		//	ex = ex * (-1.0);
		//	ey = ey * (-1.0);	
		//}
		// PCO修正
		double correctdistance_PCO = 0.0;
		POS3D satPCO = datablk.PCOList[FreqId];
		POS3D vecPCO = ex * satPCO.x + ey * satPCO.y + ez * satPCO.z;
        //POS3D vecLOS = vectorNormal(recPos - satPos);	// 视线单位矢量，卫星指向测站
		// 将卫星天线相位中心偏移矢量 vecPCO 向视线矢量 vecLOS 投影，单位：m
		correctdistance_PCO = vectorDot(vecPCO, vecLOS) / 1000.0;	
		// PCV修正
		if(bOn_PCV)
		{
			double correctdistance_PCV = 0.0;
			// 计算视线矢量在星固系中的天顶角，单位：度
			double zenith = 0.0;
			zenith = acos(vectorDot(vecLOS, ez))*180/PI;
			int N = int(datablk.zenithList.size());
			if(zenith < datablk.zenithList[0] || zenith > datablk.zenithList[N - 1])
			{
				//printf("卫星 %s 天顶角越界，zenith = %4.1f 取最邻近的PCV修正值！\n", datablk.sNN, zenith);
				Matrix matPCV = datablk.NOAZI_PCVList[FreqId];
				correctdistance_PCV = matPCV.GetElement(0, N - 1);
				correctdistance_PCV = correctdistance_PCV / 1000.0;
				correctdistance = correctdistance_PCO + correctdistance_PCV;
			}
			else
			{
				if(datablk.DAZI != 0.0)		// 与方位角有关 PCV 修正
				{
					//printf("暂时不修正与方位角有关的卫星天线PCV!\n");
				}
				Matrix matPCV = datablk.NOAZI_PCVList[FreqId];
				// 搜索天顶角 zenith 所在的区间
				int nBegin_E  = -1;
				int nEnd_E    = -1;
				for(int j = 0; j < N - 1; j++)
				{
					if(datablk.zenithList[j] <= zenith && zenith <= datablk.zenithList[j + 1])
					{
						nBegin_E  = j;
						nEnd_E    = j + 1;
						break;
					}
				}
				// 线性插值求 zenith 对应的 PCV 修正值
				correctdistance_PCV = matPCV.GetElement(0, nBegin_E)*(datablk.zenithList[nEnd_E] - zenith)/datablk.DZEN
									+ matPCV.GetElement(0, nEnd_E)*(zenith - datablk.zenithList[nBegin_E])/datablk.DZEN;

				correctdistance_PCV = correctdistance_PCV / 1000.0;
				correctdistance = correctdistance_PCO + correctdistance_PCV;
			}
		}
		else
		{
			correctdistance = correctdistance_PCO;
		}
		return correctdistance;
	}

	double igs05atxFile::correctSatAntPCOPCV_GYM95(AntCorrectionBlk datablk, string nameFreq, POS3D vecLOS, POS3D ex, POS3D ey, POS3D ez, bool bOn_PCV)
	{
		for(int FreqId = 0; FreqId < int(datablk.flagFreqList.size()); FreqId++)
		{
			if(nameFreq == datablk.flagFreqList[FreqId])
				return correctSatAntPCOPCV_GYM95(datablk, FreqId, vecLOS, ex, ey, ez, bOn_PCV);
		}
		return 0.0;
	}

	// 子程序名称： correctSatAntPCOPCV_GYM95   
	// 功能：卫星在姿态控制模式下的卫星天线修正量
	// 变量类型：datablk  ：卫星天线数据结构
	//			 FreqId   ：频点索引
	//           vecLOS   ：视线单位矢量，卫星指向测站
	//           ex       : 星固系X轴单位矢量
	//           ey       : 星固系Y轴单位矢量
	//           ez       : 星固系Z轴单位矢量
	//           bOn_PCV  ：是否修正PCV
	// 输入：datablk，FreqId, vecLOS, ex,ey,ez,bOn_PCV
	// 输出：卫星天线相位中心修正量，单位：mm
	// 语言：C++
	// 创建者：谷德峰、刘俊宏
	// 创建时间：2014/11/20
	// 版本时间：2014/11/20
	// 修改记录：
	// 备注：
	double igs05atxFile::correctSatAntPCOPCV_GYM95(AntCorrectionBlk datablk, int FreqId, POS3D vecLOS, POS3D ex, POS3D ey, POS3D ez, bool bOn_PCV)
	{
		// 计算轨道系三轴矢量
		double correctdistance = 0.0;
		vecLOS = vectorNormal(vecLOS);
		// PCO修正
		double correctdistance_PCO = 0.0;
		POS3D satPCO = datablk.PCOList[FreqId];
		POS3D vecPCO = ex * satPCO.x + ey * satPCO.y + ez * satPCO.z;
        //POS3D vecLOS = vectorNormal(recPos - satPos);	// 视线单位矢量，卫星指向测站
		// 将卫星天线相位中心偏移矢量 vecPCO 向视线矢量 vecLOS 投影，单位：m
		correctdistance_PCO = vectorDot(vecPCO, vecLOS) / 1000.0;	
		// PCV修正
		if(bOn_PCV)
		{
			double correctdistance_PCV = 0.0;
			// 计算视线矢量在星固系中的天顶角，单位：度
			double zenith = 0.0;
			zenith = acos(vectorDot(vecLOS, ez))*180/PI;
			int N = int(datablk.zenithList.size());
			if(zenith < datablk.zenithList[0] || zenith > datablk.zenithList[N - 1])
			{
				//printf("卫星 %s 天顶角越界，zenith = %4.1f 取最邻近的PCV修正值！\n", datablk.sNN, zenith);
				Matrix matPCV = datablk.NOAZI_PCVList[FreqId];
				correctdistance_PCV = matPCV.GetElement(0, N - 1);
				correctdistance_PCV = correctdistance_PCV / 1000.0;
				correctdistance = correctdistance_PCO - correctdistance_PCV; //  谷德峰, 2014/11/08, correctdistance_PCO + correctdistance_PCV
			}
			else
			{
				if(datablk.DAZI != 0.0)		// 与方位角有关 PCV 修正
				{
					//printf("暂时不修正与方位角有关的卫星天线PCV!\n");
				}
				Matrix matPCV = datablk.NOAZI_PCVList[FreqId];
				// 搜索天顶角 zenith 所在的区间
				int nBegin_E  = -1;
				int nEnd_E    = -1;
				for(int j = 0; j < N - 1; j++)
				{
					if(datablk.zenithList[j] <= zenith && zenith <= datablk.zenithList[j + 1])
					{
						nBegin_E  = j;
						nEnd_E    = j + 1;
						break;
					}
				}
				// 线性插值求 zenith 对应的 PCV 修正值
				correctdistance_PCV = matPCV.GetElement(0, nBegin_E)*(datablk.zenithList[nEnd_E] - zenith)/datablk.DZEN
									+ matPCV.GetElement(0, nEnd_E)*(zenith - datablk.zenithList[nBegin_E])/datablk.DZEN;

				correctdistance_PCV = correctdistance_PCV / 1000.0;
				correctdistance = correctdistance_PCO - correctdistance_PCV; // 谷德峰, 2014/11/08, correctdistance_PCO + correctdistance_PCV
			}
		}
		else
		{
			correctdistance = correctdistance_PCO;
		}
		return correctdistance;
	}

	// 子程序名称： correctRecAntPCOPCV   
	// 功能：计算伪距的测站天线修正量
	// 变量类型：datablk：	测站天线数据结构
	//			 FreqId：	频点索引
	//           vecLOS：   测站到卫星的视线单位矢量, ENU坐标系
	//           bOn_PCV：	是否修正PCV
	// 输入：datablk，FreqId, recPos, satPos，bOn_PCV
	// 输出：测站天线相位中心修正量，单位：mm
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/4/17
	// 版本时间：2013/4/5
	// 修改记录：
	// 备注： recPos, satPos 必须在统一坐标系下
	double igs05atxFile::correctRecAntPCOPCV(AntCorrectionBlk datablk, int FreqId, POS3D vecLOS, bool bOn_PCV)
	{
		double correctdistance = 0.0;
		// PCO修正
		double correctdistance_PCO = 0.0;
		POS3D recPCO = datablk.PCOList[FreqId];			// NEU 左手系
		POS3D vecPCO;
		vecPCO.x = recPCO.y;
		vecPCO.y = recPCO.x;
		vecPCO.z = recPCO.z;
		// 将测站天线相位中心偏移矢量 vecPCO 向视线矢量 vecLOS 投影, 单位：m
		correctdistance_PCO = vectorDot(vecPCO, vecLOS)/1000.0;
		// PCV修正
		if(bOn_PCV)
		{
			double correctdistance_PCV = 0.0;
			// 计算视线矢量在 ENU 坐标系中的天顶角, 单位：度
			double zenith = 0.0;
			zenith = acos(vecLOS.z)*180/PI;
			int N = int(datablk.zenithList.size());
			if(zenith < datablk.zenithList[0] || zenith > datablk.zenithList[N - 1])
			{
				//printf("天顶角越界, 不修正PCV!\n");
				correctdistance = correctdistance_PCO;
				return correctdistance;
			}
			else
			{
				if(datablk.DAZI != 0.0)		// 与方位角有关PCV修正
				{
					Matrix matPCV = datablk.AZIDEPT_PCVList[FreqId];
					// 搜索天顶角 zenith 所在的区间
					int nBegin_E = -1;
					int nEnd_E   = -1;
					for(int j = 0; j < N - 1; j++)
					{
						if(datablk.zenithList[j] <= zenith && zenith <= datablk.zenithList[j + 1])
						{
							nBegin_E  = j;
							nEnd_E    = j + 1;
							break;
						}
					}
					// 计算视线矢量在 ENU 坐标系中的方位角，单位：度
					double azimuth = 0.0;
					azimuth = atan2(vecLOS.x, vecLOS.y)*180/PI;
					if(azimuth < 0)
					{
						azimuth = azimuth + 360;
					}
					int M = int(datablk.azimuthList.size());
					// 搜索方位角 azimuth 所在的区间
					int nBegin_A = -1;
					int nEnd_A   = -1;
					for(int i = 0; i < M - 1; i++)
					{
						if(datablk.azimuthList[i] <= azimuth && azimuth <= datablk.azimuthList[i + 1])
						{
							nBegin_A  = i;
							nEnd_A    = i + 1;
							break;
						}
					}
					// 双线性插值求 (azimuth, zenith) 处的 PCV 修正值
					double PCVtemp1 = 0.0;
					double PCVtemp2 = 0.0;
					PCVtemp1 = matPCV.GetElement(nBegin_A, nBegin_E)*(datablk.zenithList[nEnd_E] - zenith)/datablk.DZEN
							 + matPCV.GetElement(nBegin_A, nEnd_E)*(zenith - datablk.zenithList[nBegin_E])/datablk.DZEN;

					PCVtemp2 = matPCV.GetElement(nEnd_A, nBegin_E)*(datablk.zenithList[nEnd_E] - zenith)/datablk.DZEN
							 + matPCV.GetElement(nEnd_A, nEnd_E)*(zenith - datablk.zenithList[nBegin_E])/datablk.DZEN;

					correctdistance_PCV = PCVtemp1*(datablk.azimuthList[nEnd_A] - azimuth)/datablk.DAZI
						                + PCVtemp2*(azimuth - datablk.azimuthList[nBegin_A])/datablk.DAZI;
					
					correctdistance_PCV = correctdistance_PCV / 1000.0;
				}
				else						// 与方位角无关的PCV修正
				{
					Matrix matPCV = datablk.NOAZI_PCVList[FreqId];
					// 搜索天顶角 zenith 所在的区间
					int nBegin_E  = -1;
					int nEnd_E    = -1;
					for(int j = 0; j < N - 1; j++)
					{
						if(datablk.zenithList[j] <= zenith && zenith <= datablk.zenithList[j + 1])
						{
							nBegin_E  = j;
							nEnd_E    = j + 1;
							break;
						}
					}
					// 线性插值求 zenith 对应的 PCV 修正值
					correctdistance_PCV = matPCV.GetElement(0, nBegin_E)*(datablk.zenithList[nEnd_E] - zenith)/datablk.DZEN
										+ matPCV.GetElement(0, nEnd_E)*(zenith - datablk.zenithList[nBegin_E])/datablk.DZEN;
					
					correctdistance_PCV = correctdistance_PCV / 1000.0;
				}
				correctdistance = correctdistance_PCO - correctdistance_PCV; // correctdistance_PCO + correctdistance_PCV
			}
		} // end for if(bOn_PCV)
		else
		{
			correctdistance = correctdistance_PCO;
		}
		return correctdistance;
	}

	double igs05atxFile::correctRecAntPCOPCV(AntCorrectionBlk datablk, string nameFreq, POS3D recPos, POS3D satPos, bool bOn_PCV)
	{
		for(int FreqId = 0; FreqId < int(datablk.flagFreqList.size()); FreqId++)
		{
			if(nameFreq == datablk.flagFreqList[FreqId])
				return correctRecAntPCOPCV(datablk, FreqId, recPos, satPos, bOn_PCV);
		}
		return 0.0;
	}

	// 子程序名称： correctRecAntPCOPCV   
	// 功能：计算伪距的测站天线修正量
	// 变量类型：datablk：	测站天线数据结构
	//			 FreqId：	频点索引
	//           recPos：   测站位置坐标
	//           satPos：   卫星位置坐标
	//           bOn_PCV：	是否修正PCV
	// 输入：datablk，FreqId, recPos, satPos，bOn_PCV
	// 输出：测站天线相位中心修正量，单位：mm
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/4/17
	// 版本时间：2013/4/5
	// 修改记录：
	// 备注： recPos, satPos 必须在统一坐标系下
	double igs05atxFile::correctRecAntPCOPCV(AntCorrectionBlk datablk, int FreqId, POS3D recPos, POS3D satPos, bool bOn_PCV)
	{
		double correctdistance = 0.0;
		// 计算卫星在测站 ENU 坐标系下的位置
		ENU satPos_ENU;
		TimeCoordConvert::ECF2ENU(recPos, satPos, satPos_ENU);
		// 计算测站到卫星的视线单位矢量
		POS3D vecLOS;
		vecLOS.x = satPos_ENU.E;
		vecLOS.y = satPos_ENU.N;
		vecLOS.z = satPos_ENU.U;
		vecLOS = vectorNormal(vecLOS);
		return correctRecAntPCOPCV(datablk, FreqId, vecLOS, bOn_PCV);
		//// PCO修正
		//double correctdistance_PCO = 0.0;
		//POS3D recPCO = datablk.PCOList[FreqId];			// NEU 左手系
		//POS3D vecPCO;
		//vecPCO.x = recPCO.y;
		//vecPCO.y = recPCO.x;
		//vecPCO.z = recPCO.z;
		//// 将测站天线相位中心偏移矢量 vecPCO 向视线矢量 vecLOS 投影, 单位：m
		//correctdistance_PCO = vectorDot(vecPCO, vecLOS)/1000.0;
		//// PCV修正
		//if(bOn_PCV)
		//{
		//	double correctdistance_PCV = 0.0;
		//	// 计算视线矢量在 ENU 坐标系中的天顶角, 单位：度
		//	double zenith = 0.0;
		//	zenith = acos(vecLOS.z)*180/PI;
		//	int N = int(datablk.zenithList.size());
		//	if(zenith < datablk.zenithList[0] || zenith > datablk.zenithList[N - 1])
		//	{
		//		//printf("天顶角越界, 不修正PCV!\n");
		//		correctdistance = correctdistance_PCO;
		//		return correctdistance;
		//	}
		//	else
		//	{
		//		if(datablk.DAZI != 0.0)		// 与方位角有关PCV修正
		//		{
		//			Matrix matPCV = datablk.AZIDEPT_PCVList[FreqId];
		//			// 搜索天顶角 zenith 所在的区间
		//			int nBegin_E = -1;
		//			int nEnd_E   = -1;
		//			for(int j = 0; j < N - 1; j++)
		//			{
		//				if(datablk.zenithList[j] <= zenith && zenith <= datablk.zenithList[j + 1])
		//				{
		//					nBegin_E  = j;
		//					nEnd_E    = j + 1;
		//					break;
		//				}
		//			}
		//			// 计算视线矢量在 ENU 坐标系中的方位角，单位：度
		//			double azimuth = 0.0;
		//			azimuth = atan2(vecLOS.x, vecLOS.y)*180/PI;
		//			if(azimuth < 0)
		//			{
		//				azimuth = azimuth + 360;
		//			}
		//			int M = int(datablk.azimuthList.size());
		//			// 搜索方位角 azimuth 所在的区间
		//			int nBegin_A = -1;
		//			int nEnd_A   = -1;
		//			for(int i = 0; i < M - 1; i++)
		//			{
		//				if(datablk.azimuthList[i] <= azimuth && azimuth <= datablk.azimuthList[i + 1])
		//				{
		//					nBegin_A  = i;
		//					nEnd_A    = i + 1;
		//					break;
		//				}
		//			}
		//			// 双线性插值求 (azimuth, zenith) 处的 PCV 修正值
		//			double PCVtemp1 = 0.0;
		//			double PCVtemp2 = 0.0;
		//			PCVtemp1 = matPCV.GetElement(nBegin_A, nBegin_E)*(datablk.zenithList[nEnd_E] - zenith)/datablk.DZEN
		//					 + matPCV.GetElement(nBegin_A, nEnd_E)*(zenith - datablk.zenithList[nBegin_E])/datablk.DZEN;

		//			PCVtemp2 = matPCV.GetElement(nEnd_A, nBegin_E)*(datablk.zenithList[nEnd_E] - zenith)/datablk.DZEN
		//					 + matPCV.GetElement(nEnd_A, nEnd_E)*(zenith - datablk.zenithList[nBegin_E])/datablk.DZEN;

		//			correctdistance_PCV = PCVtemp1*(datablk.azimuthList[nEnd_A] - azimuth)/datablk.DAZI
		//				                + PCVtemp2*(azimuth - datablk.azimuthList[nBegin_A])/datablk.DAZI;
		//			
		//			correctdistance_PCV = correctdistance_PCV / 1000.0;
		//		}
		//		else						// 与方位角无关的PCV修正
		//		{
		//			Matrix matPCV = datablk.NOAZI_PCVList[FreqId];
		//			// 搜索天顶角 zenith 所在的区间
		//			int nBegin_E  = -1;
		//			int nEnd_E    = -1;
		//			for(int j = 0; j < N - 1; j++)
		//			{
		//				if(datablk.zenithList[j] <= zenith && zenith <= datablk.zenithList[j + 1])
		//				{
		//					nBegin_E  = j;
		//					nEnd_E    = j + 1;
		//					break;
		//				}
		//			}
		//			// 线性插值求 zenith 对应的 PCV 修正值
		//			correctdistance_PCV = matPCV.GetElement(0, nBegin_E)*(datablk.zenithList[nEnd_E] - zenith)/datablk.DZEN
		//								+ matPCV.GetElement(0, nEnd_E)*(zenith - datablk.zenithList[nBegin_E])/datablk.DZEN;
		//			
		//			correctdistance_PCV = correctdistance_PCV / 1000.0;
		//		}
		//		correctdistance = correctdistance_PCO - correctdistance_PCV; // correctdistance_PCO + correctdistance_PCV
		//	}
		//} // end for if(bOn_PCV)
		//else
		//{
		//	correctdistance = correctdistance_PCO;
		//}
		//return correctdistance;
	}
}