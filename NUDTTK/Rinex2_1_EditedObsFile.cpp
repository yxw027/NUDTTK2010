#include "Rinex2_1_EditedObsFile.hpp"
#pragma warning(disable: 4996)

namespace NUDTTK
{
	Rinex2_1_EditedObsFile::Rinex2_1_EditedObsFile(void)
	{
	}

	Rinex2_1_EditedObsFile::~Rinex2_1_EditedObsFile(void)
	{
	}

	void Rinex2_1_EditedObsFile::clear()
	{
		m_header = Rinex2_1_ObsHeader::Rinex2_1_ObsHeader();
		m_data.clear();
	}

	bool Rinex2_1_EditedObsFile::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}

	// 子程序名称： cutdata   
	// 功能：剪切数据,数据区间为[t_Begin, t_End)
	// 变量类型：t_Begin     : 起始时间
	//           t_End       : 终止时间
	// 输入：t_Begin, t_End
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/08/13
	// 版本时间：2012/04/11
	// 修改记录：2012/04/11, 由刘俊宏进行修改, 使原程序可以应用于北斗 Rinex2.1 数据读取
	// 备注： 
	bool Rinex2_1_EditedObsFile::cutdata(DayTime t_Begin, DayTime t_End)
	{
		// 确保 t_Begin <= t_End
		if( t_Begin - t_End >= 0 || t_End - m_header.tmStart <= 0 || t_Begin - m_header.tmEnd > 0 )
			return false;
		vector<Rinex2_1_EditedObsEpoch> obsDataList;
		obsDataList.clear();
		BYTE pbySatList[MAX_PRN];
		for( int i = 0; i < MAX_PRN; i++ )
			pbySatList[i] = 0;
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			Rinex2_1_EditedObsEpoch ObsEpoch = m_data[s_i];
			if( ObsEpoch.t - t_Begin >= 0 && ObsEpoch.t - t_End < 0)
			{
				for(Rinex2_1_EditedObsSatMap::iterator it = ObsEpoch.editedObs.begin(); it != ObsEpoch.editedObs.end(); ++it)
				{
					it->second.nObsTime = int(obsDataList.size());
					pbySatList[ it->first ] = 1;
				}
				obsDataList.push_back(ObsEpoch);
			}
		}
		size_t nListNum = obsDataList.size();
		if( nListNum <= 0 )
			return false;

		m_data.clear();
		m_data = obsDataList;
		// 更新文件头信息, 更新初始观测时刻和最后观测时刻
		m_header.tmStart = m_data[0].t;
		m_header.tmEnd   = m_data[nListNum - 1].t;
		// 综合统计可视卫星列表
		m_header.pbySatList.clear();
		for(int i = 0; i < MAX_PRN; i++)
		{
			if(pbySatList[i] == 1)
			{
				m_header.pbySatList.push_back(BYTE(i));
			}
		}
		m_header.bySatCount = BYTE(m_header.pbySatList.size());
		return true;
	}

	// 子程序名称： isValidEpochLine   
	// 功能：判断当前文本行数据是否为有效时刻行 
	//         返回0 -> 文件末尾
	//         返回1 -> 有效时刻
	//         返回2 -> 无效
	// 变量类型：strLine           : 行文本 
	//           pObsfile      　　: 文件指针
	// 输入：strLine, pObsfile
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/3/27
	// 版本时间：2007/3/27
	// 修改记录：
	// 备注：
	int  Rinex2_1_EditedObsFile::isValidEpochLine(string strLine, FILE * pEditedObsfile)
	{
		// 下面几种数据用int型, 避免因为 strLine 的格式问题引起 sscanf 函数发生错误
		DayTime tmEpoch;
		int byEpochFlag   =  1;     // 0: ok,  1: power failure between previous and current epoch  > 1: Event flag (2-5) 
		int bySatCount = -1;     // 本时刻可视卫星(或测站)个数
		if(pEditedObsfile != NULL)  // 判断是否为文件末尾
		{
			if(feof(pEditedObsfile))
				return 0;
		}
		sscanf(strLine.c_str(), "%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%11lf%*2c%1d%3d",
			                    &tmEpoch.year,
								&tmEpoch.month,
								&tmEpoch.day,
								&tmEpoch.hour,
								&tmEpoch.minute,
								&tmEpoch.second,
								&byEpochFlag,
								&bySatCount);
		int nFlag = 1;
		if(tmEpoch.month > 12 || tmEpoch.month < 0)
			nFlag = 2;
		if(tmEpoch.day > 31 || tmEpoch.day < 0)
			nFlag = 2;
		if(tmEpoch.hour > 24||tmEpoch.hour < 0)
			nFlag = 2;
		if(tmEpoch.minute > 60 || tmEpoch.minute < 0)
			nFlag = 2;
		if(tmEpoch.second > 60||tmEpoch.second < 0)
			nFlag = 2;
		if(byEpochFlag > 5 || byEpochFlag < 0)
			nFlag = 2;
		if(bySatCount > MAX_PRN_GPS || bySatCount < 0)
			nFlag = 2;
		return nFlag;
	}

	// 子程序名称： open   
	// 功能：编辑后的观测数据解析 
	// 变量类型：strEditedObsfileName :编辑后的观测数据文件路径
	// 输入：strEditedObsfileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/8/27
	// 版本时间：2012/4/11
	// 修改记录：
	// 备注： 
	bool Rinex2_1_EditedObsFile::open(string strEditedObsfileName)
	{
		FILE * pEditedObsfile = fopen(strEditedObsfileName.c_str(), "r+t");
		if(pEditedObsfile == NULL) 
			return false;
		m_header = Rinex2_1_ObsHeader::Rinex2_1_ObsHeader();
		// 开始循环读取每一行数据, 直到 END OF HEADER
		int bFlag = 1;
		while(bFlag)
		{// 2008-08-08 进行了调整，文件头信息得到了补充
			char line[200];
			fgets(line, 100, pEditedObsfile);
			string strLineMask = line;
			string strLine     = line;
			strLineMask.erase(0, 60); // 从第 0 个元素开始，删除 60 个
			// 剔除 \n
			size_t nPos_n = strLineMask.find('\n');
			if(nPos_n < strLineMask.length())
				strLineMask.erase(nPos_n,1);
			// 超过20位，截取20位
			while(strLineMask.length() > 20)
				strLineMask.erase(strLineMask.length() - 1, 1);
			// 补齐20位
			if(strLineMask.length() < 20) // strLineMask.length 包含最后的 '\0'
				strLineMask.append(20 - strLineMask.length(), ' ');

			if(strLineMask == Rinex2_1_MaskString::szVerType)
			{
				strLine.copy(m_header.szRinexVersion, 20, 0);
				strLine.copy(m_header.szFileType, 20, 20);
				strLine.copy(m_header.szSatlliteSystem, 20, 40);
			}
			else if(strLineMask == Rinex2_1_MaskString::szPgmRunDate)
			{
				strLine.copy(m_header.szProgramName, 20, 0);
				strLine.copy(m_header.szProgramAgencyName, 20, 20);
				strLine.copy(m_header.szFileDate, 20, 40);
			}
			else if(strLineMask == Rinex2_1_MaskString::szObservAgency)
			{
				strLine.copy(m_header.szObserverName, 20, 0);
				strLine.copy(m_header.szObserverAgencyName, 40, 20);
			}
			else if(strLineMask == Rinex2_1_MaskString::szMarkerName)
			{
				strLine.copy(m_header.szMarkName, 60, 0);
			}
			else if(strLineMask == Rinex2_1_MaskString::szMarkerNum)
			{
				strLine.copy(m_header.szMarkNumber, 20, 0);
			}
			else if(strLineMask == Rinex2_1_MaskString::szRecTypeVers)
			{
				strLine.copy(m_header.szRecNumber, 20, 0);
				strLine.copy(m_header.szRecType, 20, 20);
				strLine.copy(m_header.szRecVersion, 20, 40);
			}
			else if(strLineMask == Rinex2_1_MaskString::szAntType)
			{
				strLine.copy(m_header.szAntNumber, 20, 0);
				strLine.copy(m_header.szAntType, 20, 20);
			}
			else if(strLineMask == Rinex2_1_MaskString::szApproxPosXYZ)
			{  
				sscanf(line,"%14lf%14lf%14lf",
					        &m_header.ApproxPos.x,
							&m_header.ApproxPos.y,
							&m_header.ApproxPos.z);
			}
			else if(strLineMask == Rinex2_1_MaskString::szAntDeltaHEN)
			{  
				sscanf(line,"%14lf%14lf%14lf",
					        &m_header.AntOffset.x,
							&m_header.AntOffset.y,
							&m_header.AntOffset.z);
			}
			else if(strLineMask == Rinex2_1_MaskString::szWaveLenFact)
			{  
				sscanf(line,"%6d%6d%6d",
					        &m_header.bL1WaveLengthFact,
							&m_header.bL2WaveLengthFact,
							&m_header.bL5WaveLengthFact);
			}
			else if(strLineMask == Rinex2_1_MaskString::szTmOfFirstObs)
			{  
				sscanf(line,"%6d%6d%6d%6d%6d%12lf",
					        &m_header.tmStart.year,  
                            &m_header.tmStart.month,
                            &m_header.tmStart.day,   
                            &m_header.tmStart.hour,
                            &m_header.tmStart.minute,
                            &m_header.tmStart.second);
				strLine.copy(m_header.szTimeType, 3, 48);
			}
			else if(strLineMask == Rinex2_1_MaskString::szTmOfLastObs)
			{  
				sscanf(line,"%6d%6d%6d%6d%6d%12lf",
					        &m_header.tmEnd.year,	 
							&m_header.tmEnd.month,
						    &m_header.tmEnd.day,	 
							&m_header.tmEnd.hour,
							&m_header.tmEnd.minute,  
							&m_header.tmEnd.second);
			}
			else if(strLineMask == Rinex2_1_MaskString::szInterval)
			{  
				sscanf(line, "%10lf", &m_header.Interval);
			}
			else if(strLineMask == Rinex2_1_MaskString::szLeapSec)
			{  
				sscanf(line, "%6d", &m_header.LeapSecond);
			}
			else if(strLineMask == Rinex2_1_MaskString::szNumsOfSv)
			{  
				sscanf(line, "%6d", &m_header.bySatCount);
			}
			else if(strLineMask == Rinex2_1_MaskString::szTypeOfObserv)
			{  
				sscanf(line,"%6d", &m_header.byObsTypes);				
				int               nline    = m_header.byObsTypes / 9;	// 整行数
				int               nResidue = m_header.byObsTypes % 9;
				if(m_header.byObsTypes <= 9)
				{
					for(BYTE i = 0; i < m_header.byObsTypes; i++)
					{
						char strObsType[7];
						strLine.copy(strObsType, 6, 6 + i * 6);
						strObsType[6] = '\0';
						m_header.pbyObsTypeList.push_back(string2ObsId(strObsType));						
					}					
				}
				else
				{
					for(BYTE i = 0; i < 9; i++)
					{
						char strObsType[7];
						strLine.copy(strObsType, 6, 6 + i * 6);
						strObsType[6] = '\0';
						m_header.pbyObsTypeList.push_back(string2ObsId(strObsType));					
					}
					for(int n = 1; n < nline;n++)
					{// 读取中间的整行
						fgets(line,100,pEditedObsfile);
						strLine = line;
						for(BYTE i = 0; i < 9; i++)
						{
							char strObsType[7];
							strLine.copy(strObsType, 6, 6 + i * 6);
							strObsType[6] = '\0';
							m_header.pbyObsTypeList.push_back(string2ObsId(strObsType));
						}						
					}
					if(nResidue > 0)
					{
						fgets(line,100,pEditedObsfile);
						strLine = line;
						for(BYTE i = 0; i < nResidue; i++)
						{
							char strObsType[7];
							strLine.copy(strObsType, 6, 6 + i * 6);
							strObsType[6] = '\0';
							m_header.pbyObsTypeList.push_back(string2ObsId(strObsType));
						}		
					}
				}
				//sscanf(line, "%6d", &m_header.byObsTypes);
				//for(BYTE i = 0; i < m_header.byObsTypes; i++)
				//{
				//	char strObsType[7];
				//	strLine.copy(strObsType, 6, 6 + i * 6);
				//	strObsType[6] = '\0';
				//	m_header.pbyObsTypeList[i] = string2ObsId(strObsType);
				//}//
			}
			else if(strLineMask == Rinex2_1_MaskString::szPRNNumOfObs)
			{
				// 可视卫星 PRN 列表在文件读取完后再填写，不在文件头处读取
				/*
				BYTE bPRN;
				sscanf(line, "%*4c%2d", &bPRN);
				m_header.pbySatList.push_back(bPRN);
				*/
			}
			else if(strLineMask == Rinex2_1_MaskString::szEndOfHead)
			{
				bFlag = false;
			}
			else 
			{
				// Comment等不作处理
			}
		}
		// 观测数据
		bFlag = true;
		m_data.clear();
		int k = 0;
		char line[400];//对于21种观测数据类型300个字符已经不能满足要求，2014/09/02,刘俊宏
		fgets(line, 400, pEditedObsfile);
		BYTE pbySatList[MAX_PRN]; // 卫星列表
		for(int i = 0; i < MAX_PRN; i++)
			pbySatList[i] = 0;

		while(bFlag)
		{
			string strLine = line;
			int nFlag = isValidEpochLine(strLine, pEditedObsfile);
			if(nFlag == 0)      // 文件末尾
			{
				bFlag = false;
			}
			else if(nFlag == 1) // 找到新时刻的数据段
			{
				k++;
				Rinex2_1_EditedObsEpoch editedObsEpoch;
				// 解析Epoch/SAT -- 1 -- 32
				sscanf(strLine.c_str(),"%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%11lf%*2c%1d%3d%14lf%14lf%14lf%14lf%14lf%14lf%14lf",
					                   &editedObsEpoch.t.year,
									   &editedObsEpoch.t.month,
								       &editedObsEpoch.t.day,
									   &editedObsEpoch.t.hour,
									   &editedObsEpoch.t.minute,
									   &editedObsEpoch.t.second,
								       &editedObsEpoch.byEpochFlag,
									   &editedObsEpoch.bySatCount,
									   &editedObsEpoch.clock,
									   &editedObsEpoch.tropZenithDelayPriori_H,
									   &editedObsEpoch.tropZenithDelayPriori_W, 
									   &editedObsEpoch.tropZenithDelayEstimate,
									   &editedObsEpoch.temperature,
								       &editedObsEpoch.humidity,
									   &editedObsEpoch.pressure);
				editedObsEpoch.t.year  = yearB2toB4(editedObsEpoch.t.year);
				editedObsEpoch.editedObs.clear();
				for(int i = 0; i < editedObsEpoch.bySatCount; i++)
				{
					fgets(line, 400, pEditedObsfile);
					string strLine = line;
					Rinex2_1_EditedObsLine editedObsLine;
					editedObsLine.nObsTime = k - 1; // 2007/07/22 添加
					sscanf(line,"%*1c%2d%8lf%8lf",
						        &editedObsLine.Id,
                                &editedObsLine.Elevation,
                                &editedObsLine.Azimuth);

					pbySatList[editedObsLine.Id] = 1; // 可见卫星 PRN 标号处标记 1
					editedObsLine.obsTypeList.clear();
					for(int j = 0; j < m_header.byObsTypes; j++)
					{
						Rinex2_1_EditedObsDatum editedObsDatum;
						char strEditedObsDatum[18];
						strLine.copy(strEditedObsDatum, 17, 19 + j * 17);
						strEditedObsDatum[17] = '\0';
						// 对 edited 文件的读取进行改进, 防止空格无法读取, 2008-01-04
						char szObsValue[15];
						char szEditedMark1[2];
						char szEditedMark2[2];
						sscanf(strEditedObsDatum, "%14c%*1c%1c%1c", szObsValue, szEditedMark1, szEditedMark2);
						szObsValue[14]  = '\0';
						szEditedMark1[1] = '\0';
						szEditedMark2[1] = '\0';
						if(strcmp(szObsValue,"              ") == 0)
							editedObsDatum.obs.data = DBL_MAX;
						else
							sscanf(szObsValue, "%14lf", &editedObsDatum.obs.data);
						editedObsDatum.byEditedMark1 = atoi(szEditedMark1);
						editedObsDatum.byEditedMark2 = atoi(szEditedMark2);
						editedObsLine.obsTypeList.push_back(editedObsDatum);
					}
					editedObsEpoch.editedObs.insert(Rinex2_1_EditedObsSatMap::value_type(editedObsLine.Id, editedObsLine));
				}
				m_data.push_back(editedObsEpoch);
				fgets(line, 400, pEditedObsfile);
			}
			else  // 无效数据行, 比如空白行, 掠过不作处理
			{   
				fgets(line, 400, pEditedObsfile);
			}
		}

		// 综合统计可视卫星列表
		m_header.pbySatList.clear();
		for(int i = 0; i < MAX_PRN; i++)
		{
			if(pbySatList[i] == 1)
			{
				m_header.pbySatList.push_back(BYTE(i));
			}
		}
		m_header.bySatCount = BYTE(m_header.pbySatList.size());
		// 更新初始观测时刻和最后观测时刻
		size_t nListNum = m_data.size();
		if(nListNum > 0)
		{
			m_header.tmStart = m_data[0].t;
			m_header.tmEnd   = m_data[nListNum - 1].t;
		}
		fclose(pEditedObsfile);
		return true;
	}

	// 子程序名称： write 
	// 功能：将编辑后的观测数据写到文件 
	// 变量类型：strEditedObsfileName    : 编辑后的观测数据文件路径()	
	// 输入：strEditedObsfileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/8/27
	// 版本时间：2012/4/11
	// 修改记录：
	// 备注：
	bool Rinex2_1_EditedObsFile::write(string strEditedObsfileName)
	{
		size_t nlistnum = m_data.size();
		if(nlistnum <= 0)
			return false;
		// 2008-08-08 进行了调整, 文件头信息得到了补充
		FILE* pEditedfile = fopen(strEditedObsfileName.c_str(), "w+");
		// 若 RINEX 文件类型为 M (Mixed)，目前仅读取 GPS 数据，写文件时将 'M'改写为'G'(2013/6/28)
		if(m_header.getSatSystemChar() == 'M')
		{
			m_header.szSatlliteSystem[0] = 'G';
		}
		fprintf(pEditedfile,"%20s%-20s%20s%20s\n",
			                m_header.szRinexVersion,
							"EDITED OBSERVATION",
							m_header.szSatlliteSystem,
							Rinex2_1_MaskString::szVerType);
		fprintf(pEditedfile,"%-20s%-20s%-20s%20s\n",
			                m_header.szProgramName, 
							m_header.szProgramAgencyName, 
							m_header.szFileDate,
							Rinex2_1_MaskString::szPgmRunDate);
		fprintf(pEditedfile,"%60s%20s\n",
			                m_header.szMarkName,
							Rinex2_1_MaskString::szMarkerName);
		// 输出OBSERVER / AGENCY，2008-07-27
		fprintf(pEditedfile,"%-20s%-40s%20s\n",
			                m_header.szObserverName, 
							m_header.szObserverAgencyName, 
							Rinex2_1_MaskString::szObservAgency);
		// 该信息不输出，2008-07-27
		fprintf(pEditedfile,"%20s%20s%20s%20s\n",
			                m_header.szRecNumber,
							m_header.szRecType,
							m_header.szRecVersion,
							Rinex2_1_MaskString::szRecTypeVers);
		fprintf(pEditedfile,"%20s%20s%-20s%20s\n",
			                m_header.szAntNumber,
							m_header.szAntType,
							" ",
							Rinex2_1_MaskString::szAntType);
		fprintf(pEditedfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
			                m_header.ApproxPos.x,
							m_header.ApproxPos.y,
							m_header.ApproxPos.z,
							" ",
							Rinex2_1_MaskString::szApproxPosXYZ);
		fprintf(pEditedfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
			                m_header.AntOffset.x,
							m_header.AntOffset.y,
							m_header.AntOffset.z,
							" ",
							Rinex2_1_MaskString::szAntDeltaHEN);
		if(m_header.bL1WaveLengthFact != 100)
			fprintf(pEditedfile,"%6d%6d%6d%-42s%20s\n",
								m_header.bL1WaveLengthFact,
								m_header.bL2WaveLengthFact,
								m_header.bL5WaveLengthFact,
								" ",
								Rinex2_1_MaskString::szWaveLenFact);
		fprintf(pEditedfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
			                m_header.tmStart.year,  
							m_header.tmStart.month,
							m_header.tmStart.day,   
							m_header.tmStart.hour,
							m_header.tmStart.minute,
							m_header.tmStart.second,
							" ",
							m_header.szTimeType,
							" ",
							Rinex2_1_MaskString::szTmOfFirstObs);
		fprintf(pEditedfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
			                m_header.tmEnd.year,  
							m_header.tmEnd.month,
							m_header.tmEnd.day,   
							m_header.tmEnd.hour,
							m_header.tmEnd.minute,
							m_header.tmEnd.second,
							" ",
							m_header.szTimeType,
							" ",
							Rinex2_1_MaskString::szTmOfLastObs);
		if(m_header.Interval != DBL_MAX)
			fprintf(pEditedfile,"%10.3f%-50s%20s\n",
			                    m_header.Interval, 
								" ",
								Rinex2_1_MaskString::szInterval);
		if(m_header.LeapSecond != INT_MAX)
			fprintf(pEditedfile,"%6d%-54s%20s\n",
			                    m_header.LeapSecond,
								" ",
								Rinex2_1_MaskString::szLeapSec);
		fprintf(pEditedfile,"%6d%-54s%20s\n",
			                m_header.bySatCount,
							" ",
							Rinex2_1_MaskString::szNumsOfSv);
		fprintf(pEditedfile,"%6d",m_header.byObsTypes);
		////由于不同接收机对观测数据类型的排列方式可能不相同，此处将观测数据的类型同一按照先P1、L1、S1，P2、L2、S2的方式排列
		////另外当观测数据多于9个时，程序读写将出现错误，此处暂时未调整，2013/10/3,刘俊宏	// 已修改，20140902
		//for(int i = 0; i < m_header.byObsTypes; i++)
		//{// 根据CHAMP格式要求此处由4X2A直接调整为6A,具体参见 glCodeIDToString
		//	fprintf(pEditedfile,"%6s",obsId2String(m_header.pbyObsTypeList[i]).c_str());
		//	//fprintf(pEditedfile,"%6s",obsId2String(m_header.pbyObsTypeList[ObsTypes_NewOrder[i]]).c_str());
		//}
		//int nBlank = 60 - (6 + 6 * m_header.byObsTypes);
		//string strBlank;
		//strBlank.append(nBlank,' ');
		//fprintf(pEditedfile,"%s%20s\n", 
		//	                strBlank.c_str(),
		//					Rinex2_1_MaskString::szTypeOfObserv);//
		int obsTypes = (int)m_header.byObsTypes;
		int nLine    = obsTypes / 9;  // 整行数
	    int nResidue = obsTypes % 9;  // 余数
		int nBlank   = 0;              // 空白位数
		string         strBlank; 			
		if(obsTypes <= 9)
		{
			nBlank = 60 - (6 + 6 * obsTypes);
			strBlank.append(nBlank,' ');				
			for(int i = 0;i < obsTypes;i ++)
				fprintf(pEditedfile,"%6s",obsId2String(m_header.pbyObsTypeList[i]).c_str());
			fprintf(pEditedfile,"%s%20s\n", strBlank.c_str(), Rinex2_1_MaskString::szTypeOfObserv);
		}
		else
		{
			nBlank = 60 - (6 + 6 * nResidue);
			strBlank.append(nBlank,' ');
			for(int i = 0; i < 9; i ++)
				fprintf(pEditedfile,"%6s",obsId2String(m_header.pbyObsTypeList[i]).c_str());
			fprintf(pEditedfile,"%20s\n", Rinex2_1_MaskString::szTypeOfObserv);
			for(int n = 1; n < nLine; n ++) 
			{
				fprintf(pEditedfile,"%6s"," ");
				for(int i = 0; i < 9; i ++)
					fprintf(pEditedfile,"%6s",obsId2String(m_header.pbyObsTypeList[9 * n + i]).c_str());
			fprintf(pEditedfile,"%20s\n", Rinex2_1_MaskString::szTypeOfObserv);
			}
			if(nResidue > 0)
			{
				fprintf(pEditedfile,"%-6s"," ");
				for(int i = 0; i < nResidue; i ++)
					fprintf(pEditedfile,"%6s",obsId2String(m_header.pbyObsTypeList[9 * nLine + i]).c_str());
			}
			fprintf(pEditedfile,"%s%20s\n", strBlank.c_str(), Rinex2_1_MaskString::szTypeOfObserv);
		}

		// 补充输出 Comment 信息,  2008-07-27
		for( size_t s_i = 0; s_i < m_header.pstrCommentList.size(); s_i++)
		{
			fprintf(pEditedfile,"%s",m_header.pstrCommentList[s_i].c_str());
		}
		fprintf(pEditedfile,"%-60s%20s\n",
			                " ",
							Rinex2_1_MaskString::szEndOfHead);
		
		// 编辑后观测数据
		for(size_t s_i = 0; s_i < nlistnum; s_i ++)
		{
			size_t nsatnum = m_data[s_i].editedObs.size();
			fprintf(pEditedfile," %1d%1d %2d %2d %2d %2d%11.7f  %1d%3d",
				                getIntBit(m_data[s_i].t.year, 1),
								getIntBit(m_data[s_i].t.year, 0),
					            m_data[s_i].t.month,
								m_data[s_i].t.day,
								m_data[s_i].t.hour,
								m_data[s_i].t.minute,
					            m_data[s_i].t.second,
								m_data[s_i].byEpochFlag,
								nsatnum);
			// 增加对数据越界的判断, 2008-09-26
			if( fabs(m_data[s_i].clock) >= 1.0E+9 
			 || fabs(m_data[s_i].tropZenithDelayPriori_H) >= 1.0E+9
			 || fabs(m_data[s_i].tropZenithDelayPriori_W) >= 1.0E+9
			 || fabs(m_data[s_i].tropZenithDelayEstimate) >= 1.0E+9
			 || fabs(m_data[s_i].temperature) >= 1.0E+9
			 || fabs(m_data[s_i].humidity) >= 1.0E+9
			 || fabs(m_data[s_i].pressure) >= 1.0E+9)
				fprintf(pEditedfile,"%14.3E%14.4E%14.4E%14.4E%14.3E%14.3E%14.3E\n",
				                    m_data[s_i].clock,
									m_data[s_i].tropZenithDelayPriori_H,
									m_data[s_i].tropZenithDelayPriori_W,
									m_data[s_i].tropZenithDelayEstimate,
									m_data[s_i].temperature,
									m_data[s_i].humidity,
									m_data[s_i].pressure);
			else
				fprintf(pEditedfile,"%14.3f%14.4f%14.4f%14.4f%14.3f%14.3f%14.3f\n",
				                    m_data[s_i].clock,
									m_data[s_i].tropZenithDelayPriori_H,
									m_data[s_i].tropZenithDelayPriori_W,
									m_data[s_i].tropZenithDelayEstimate,
									m_data[s_i].temperature,
									m_data[s_i].humidity,
									m_data[s_i].pressure);

			for(Rinex2_1_EditedObsSatMap::iterator it = m_data[s_i].editedObs.begin(); it != m_data[s_i].editedObs.end(); ++it)
			{//fprintf(pObsfile,"%c%2d", m_header.getSatSystemChar(),it->first);
				Rinex2_1_EditedObsLine editedObsLine = it->second;
				fprintf(pEditedfile,"%c%2d%8.2f%8.2f",
					                m_header.getSatSystemChar(),
					                editedObsLine.Id,
									editedObsLine.Elevation,
									editedObsLine.Azimuth); 

				for(size_t s_k = 0; s_k < editedObsLine.obsTypeList.size(); s_k ++)
				{
					if(editedObsLine.obsTypeList[s_k].obs.data != DBL_MAX)
						fprintf(pEditedfile,"%14.3f %1d%1d",
						                    editedObsLine.obsTypeList[s_k].obs.data,
											editedObsLine.obsTypeList[s_k].byEditedMark1,
											editedObsLine.obsTypeList[s_k].byEditedMark2);
					else
						fprintf(pEditedfile,"%-14s %1d%1d",
						                    " ",
											editedObsLine.obsTypeList[s_k].byEditedMark1,
											editedObsLine.obsTypeList[s_k].byEditedMark2);//					
				}
				fprintf(pEditedfile, "\n");
			}
		}
		fclose(pEditedfile);
		return true;
	}
	// 子程序名称： write_4Obs 
	// 功能：将编辑后的观测数据写到文件，仅写 L1、L2、P1(C1)、P2 四类观测数据
	// 变量类型：strEditedObsfileName    : 编辑后的观测数据文件路径()	
	// 输入：strEditedObsfileName
	//       Freq1(频点标识)：默认 Freq1 = 1，L1/B1 频点 (取值：1，2，3)
	//		 Freq2(频点标识)：默认 Freq2 = 2，L2/B2 频点 (取值：1，2，3)
	// 输出：
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2013/7/4
	// 版本时间：2013/7/4
	// 修改记录：
	// 备注：	1.增加输入参数Freq1，Freq2，用于控制双频预处理的频点选择(2016/3/30)
	bool Rinex2_1_EditedObsFile::write_4Obs(string strEditedObsfileName, int Freq1, int Freq2, char RecType)
	{
		size_t nlistnum = m_data.size();
		if(nlistnum <= 0)
			return false;
		// 2008-08-08 进行了调整, 文件头信息得到了补充
		FILE* pEditedfile = fopen(strEditedObsfileName.c_str(), "w+");
		// 获取观测数据索引
		int index_C1 = -1, index_P1 = -1, index_P2 = -1, index_L1 = -1, index_L2 = -1, index_P5 = -1, index_L5 = -1;;
		for(int i = 0; i < m_header.byObsTypes; i++)
		{
			if(m_header.pbyObsTypeList[i] == TYPE_OBS_C1)
				index_C1 = i;
			if(m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
				index_P1 = i;
			if(m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
				index_P2 = i;
			if(m_header.pbyObsTypeList[i] == TYPE_OBS_P5)
				index_P5 = i;
			if(m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
				index_L1 = i;
			if(m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
				index_L2 = i;
			if(m_header.pbyObsTypeList[i] == TYPE_OBS_L5)
				index_L5 = i;
		}
		// 防止文件头信息：# / TYPES OF OBSERV 与实际数据不符，鞠冰(2014/4/3)
		bool bValid_P1 = false;	// 判断是否真的有 P1 数据
		if(index_P1 != -1)
		{
			for(size_t s_i = 0; s_i != m_data.size(); s_i++)
			{
				for(Rinex2_1_EditedObsSatMap::iterator it = m_data[s_i].editedObs.begin(); it != m_data[s_i].editedObs.end(); it++)
				{
					if(it->second.obsTypeList[index_P1].obs.data != DBL_MAX)
					{
						bValid_P1 = true;	// 只要有一颗卫星的数据不为空，则认为该类型数据有效
						break;
					}
				}
				if(bValid_P1)
				{
					break;
				}
			}
			if(!bValid_P1)
			{
				index_P1 = -1;		// P1 数据无效
			}
		}

		if(RecType != 'N')
		{
			index_P1 = index_C1;	// 根据接收机类型将 C1 赋给 P1 (2015/9/14)
		}
		// 此处添加频点选择，默认选择 L1(B1),L2(B2) 频点，2016/3/30，鞠 冰
		if(Freq1 == 2 && Freq2 == 3)		// L2,L5
		{
			index_P1 = index_P2;
			index_L1 = index_L2;
			index_P2 = index_P5;
			index_L2 = index_L5;
		}
		else if(Freq1 == 1 && Freq2 == 3)	// L1,L5
		{
			index_P2 = index_P5;
			index_L2 = index_L5;
		}
		if(index_P1 == -1 || index_P2 == -1 || index_L1 == -1 || index_L2 == -1)
		{
			printf("观测数据类型不足！跳过预处理环节！\n");
			return false;
		}
		int index[4];
		index[0] = index_L1;
		index[1] = index_L2;
		index[2] = index_P1;
		index[3] = index_P2;
		// 若 RINEX 文件类型为 M (Mixed)，目前仅读取 GPS 数据，写文件时将 'M'改写为'G'(2013/6/28)
		if(m_header.getSatSystemChar() == 'M')
		{
			m_header.szSatlliteSystem[0] = 'G';
		}
		fprintf(pEditedfile,"%20s%-20s%20s%20s\n",
			                m_header.szRinexVersion,
							"EDITED OBSERVATION",
							m_header.szSatlliteSystem,
							Rinex2_1_MaskString::szVerType);
		fprintf(pEditedfile,"%-20s%-20s%-20s%20s\n",
			                m_header.szProgramName, 
							m_header.szProgramAgencyName, 
							m_header.szFileDate,
							Rinex2_1_MaskString::szPgmRunDate);
		fprintf(pEditedfile,"%60s%20s\n",
			                m_header.szMarkName,
							Rinex2_1_MaskString::szMarkerName);
		// 输出OBSERVER / AGENCY，2008-07-27
		fprintf(pEditedfile,"%-20s%-40s%20s\n",
			                m_header.szObserverName, 
							m_header.szObserverAgencyName, 
							Rinex2_1_MaskString::szObservAgency);
		// 该信息不输出，2008-07-27
		fprintf(pEditedfile,"%20s%20s%20s%20s\n",
			                m_header.szRecNumber,
							m_header.szRecType,
							m_header.szRecVersion,
							Rinex2_1_MaskString::szRecTypeVers);
		fprintf(pEditedfile,"%20s%20s%-20s%20s\n",
			                m_header.szAntNumber,
							m_header.szAntType,
							" ",
							Rinex2_1_MaskString::szAntType);
		fprintf(pEditedfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
			                m_header.ApproxPos.x,
							m_header.ApproxPos.y,
							m_header.ApproxPos.z,
							" ",
							Rinex2_1_MaskString::szApproxPosXYZ);
		fprintf(pEditedfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
			                m_header.AntOffset.x,
							m_header.AntOffset.y,
							m_header.AntOffset.z,
							" ",
							Rinex2_1_MaskString::szAntDeltaHEN);
		fprintf(pEditedfile,"%6d%6d%6d%-42s%20s\n",
			                m_header.bL1WaveLengthFact,
							m_header.bL2WaveLengthFact,
							m_header.bL5WaveLengthFact,
			                " ",
							Rinex2_1_MaskString::szWaveLenFact);
		fprintf(pEditedfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
			                m_header.tmStart.year,  
							m_header.tmStart.month,
							m_header.tmStart.day,   
							m_header.tmStart.hour,
							m_header.tmStart.minute,
							m_header.tmStart.second,
							" ",
							m_header.szTimeType,
							" ",
							Rinex2_1_MaskString::szTmOfFirstObs);
		fprintf(pEditedfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
			                m_header.tmEnd.year,  
							m_header.tmEnd.month,
							m_header.tmEnd.day,   
							m_header.tmEnd.hour,
							m_header.tmEnd.minute,
							m_header.tmEnd.second,
							" ",
							m_header.szTimeType,
							" ",
							Rinex2_1_MaskString::szTmOfLastObs);
		if(m_header.Interval != DBL_MAX)
			fprintf(pEditedfile,"%10.3f%-50s%20s\n",
			                    m_header.Interval, 
								" ",
								Rinex2_1_MaskString::szInterval);
		if(m_header.LeapSecond != INT_MAX)
			fprintf(pEditedfile,"%6d%-54s%20s\n",
			                    m_header.LeapSecond,
								" ",
								Rinex2_1_MaskString::szLeapSec);
		fprintf(pEditedfile,"%6d%-54s%20s\n",
			                m_header.bySatCount,
							" ",
							Rinex2_1_MaskString::szNumsOfSv);
		m_header.byObsTypes = 4;
		fprintf(pEditedfile,"%6d", m_header.byObsTypes);
		fprintf(pEditedfile,"%6s%6s%6s%6s",
			    obsId2String(m_header.pbyObsTypeList[index_L1]).c_str(),	// 输出真实频点标识，鞠 冰，2016/3/30
				obsId2String(m_header.pbyObsTypeList[index_L2]).c_str(),	// 输出真实频点标识，鞠 冰，2016/3/30
				obsId2String(m_header.pbyObsTypeList[index_P1]).c_str(),	// 输出真实频点标识，鞠 冰，2016/3/30
				obsId2String(m_header.pbyObsTypeList[index_P2]).c_str());	// 输出真实频点标识，鞠 冰，2016/3/30
		//for(int i = 0; i < 4; i++)
		//{// 根据CHAMP格式要求此处由4X2A直接调整为6A,具体参见 glCodeIDToString
		//	fprintf(pEditedfile,"%6s",obsId2String(m_header.pbyObsTypeList[index[i]]).c_str());
		//}
		int nBlank = 60 - (6 + 6 * m_header.byObsTypes);
		string strBlank;
		strBlank.append(nBlank,' ');
		fprintf(pEditedfile,"%s%20s\n", 
			                strBlank.c_str(),
							Rinex2_1_MaskString::szTypeOfObserv);

		// 补充输出 Comment 信息,  2008-07-27
		for( size_t s_i = 0; s_i < m_header.pstrCommentList.size(); s_i++)
		{
			fprintf(pEditedfile,"%s",m_header.pstrCommentList[s_i].c_str());
		}
		fprintf(pEditedfile,"%-60s%20s\n",
			                " ",
							Rinex2_1_MaskString::szEndOfHead);
		
		// 编辑后观测数据
		for(size_t s_i = 0; s_i < nlistnum; s_i ++)
		{
			size_t nsatnum = m_data[s_i].editedObs.size();
			fprintf(pEditedfile," %1d%1d %2d %2d %2d %2d%11.7f  %1d%3d",
				                getIntBit(m_data[s_i].t.year, 1),
								getIntBit(m_data[s_i].t.year, 0),
					            m_data[s_i].t.month,
								m_data[s_i].t.day,
								m_data[s_i].t.hour,
								m_data[s_i].t.minute,
					            m_data[s_i].t.second,
								m_data[s_i].byEpochFlag,
								nsatnum);
			// 增加对数据越界的判断, 2008-09-26
			if( fabs(m_data[s_i].clock) >= 1.0E+9 
			 || fabs(m_data[s_i].tropZenithDelayPriori_H) >= 1.0E+9
			 || fabs(m_data[s_i].tropZenithDelayPriori_W) >= 1.0E+9
			 || fabs(m_data[s_i].tropZenithDelayEstimate) >= 1.0E+9
			 || fabs(m_data[s_i].temperature) >= 1.0E+9
			 || fabs(m_data[s_i].humidity) >= 1.0E+9
			 || fabs(m_data[s_i].pressure) >= 1.0E+9)
				fprintf(pEditedfile,"%14.3E%14.4E%14.4E%14.4E%14.3E%14.3E%14.3E\n",
				                    m_data[s_i].clock,
									m_data[s_i].tropZenithDelayPriori_H,
									m_data[s_i].tropZenithDelayPriori_W,
									m_data[s_i].tropZenithDelayEstimate,
									m_data[s_i].temperature,
									m_data[s_i].humidity,
									m_data[s_i].pressure);
			else
				fprintf(pEditedfile,"%14.3f%14.4f%14.4f%14.4f%14.3f%14.3f%14.3f\n",
				                    m_data[s_i].clock,
									m_data[s_i].tropZenithDelayPriori_H,
									m_data[s_i].tropZenithDelayPriori_W,
									m_data[s_i].tropZenithDelayEstimate,
									m_data[s_i].temperature,
									m_data[s_i].humidity,
									m_data[s_i].pressure);

			for(Rinex2_1_EditedObsSatMap::iterator it = m_data[s_i].editedObs.begin(); it != m_data[s_i].editedObs.end(); ++it)
			{
				Rinex2_1_EditedObsLine editedObsLine = it->second;
				fprintf(pEditedfile,"%c%2d%8.3f%8.3f",	// 修改俯仰角、方位角的有效位数至小数点后3位，鞠 冰，2016/4/11
					                m_header.getSatSystemChar(),
					                editedObsLine.Id,
									editedObsLine.Elevation,
									editedObsLine.Azimuth); 
				for(int i = 0; i < 4; i++)
				{
					if(editedObsLine.obsTypeList[index[i]].obs.data != DBL_MAX)
						fprintf(pEditedfile,"%14.3f %1d%1d",
						                    editedObsLine.obsTypeList[index[i]].obs.data,
											editedObsLine.obsTypeList[index[i]].byEditedMark1,
											editedObsLine.obsTypeList[index[i]].byEditedMark2);
					else
						fprintf(pEditedfile,"%-14s %1d%1d",
						                    " ",
											editedObsLine.obsTypeList[index[i]].byEditedMark1,
											editedObsLine.obsTypeList[index[i]].byEditedMark2);
				}
				fprintf(pEditedfile, "\n");
			}
		}
		//for(int k = 0; k < 12; k++)
		//{
		//	m_header.pbyObsTypeList[k] = 0;
		//}//
		m_header.pbyObsTypeList.resize(m_header.byObsTypes);
		m_header.pbyObsTypeList[0] = TYPE_OBS_L1;
		m_header.pbyObsTypeList[1] = TYPE_OBS_L2;
		//if(index[2] == index_P1)
		//{
		//	m_header.pbyObsTypeList[2] = TYPE_OBS_P1;
		//}
		//else
		//{
		//	m_header.pbyObsTypeList[2] = TYPE_OBS_C1;
		//}
		m_header.pbyObsTypeList[2] = TYPE_OBS_P1;
		m_header.pbyObsTypeList[3] = TYPE_OBS_P2;
		fclose(pEditedfile);
		return true;
	}
	// 子程序名称： write 
	// 功能：按条件将编辑后的观测数据以列形式写到文件: conditon = 1, 全部输出;
	//                                                 conditon = 2, 只输出标记为正常(为1)的观测数据;以后可根据需要进行扩展
	// 变量类型：strEditedObsfileName    : 编辑后的观测数据文件路径()	
	//           condition               : 输出条件
	// 输入：strEditedObsfileName
	// 输出：
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2012/4/11
	// 版本时间：2012/4/11
	// 修改记录：
	// 备注：
	bool Rinex2_1_EditedObsFile::write(string strEditedObsfileName, int condition)
	{
		if(condition < 0 || condition > 10)
		{
			printf("输入的控制条件不合法!\n");
			return false;
		}
		FILE* pEditedfile = fopen(strEditedObsfileName.c_str(), "w+");
		vector<Rinex2_1_EditedObsSat> editedObsSatlist;
		getEditedObsSatList(editedObsSatlist);
		for(size_t  i = 0; i < editedObsSatlist.size();i++)
		{
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[i].editedObs.begin(); it != editedObsSatlist[i].editedObs.end(); ++it)
			{
				if(condition == 1)
				{
					fprintf(pEditedfile,"%2d %4d %2d %2d %2d %2d %11.7f",
						                    editedObsSatlist[i].Id,
											it->first.year,
											it->first.month,
											it->first.day,
											it->first.hour,
											it->first.minute,
											it->first.second);
						                      
					for(size_t j = 0; j < it->second.obsTypeList.size(); j ++)
					{							
						fprintf(pEditedfile,"  %14.3f %1d%1d",
						                    it->second.obsTypeList[j].obs.data,
											it->second.obsTypeList[j].byEditedMark1,
											it->second.obsTypeList[j].byEditedMark2);
					}
					fprintf(pEditedfile, "\n");
				}
				else if(condition == 2)
				{
					size_t j=0;
					for(j = 0; j < it->second.obsTypeList.size(); j ++)						
						if(it->second.obsTypeList[j].byEditedMark1 != 1)
							break;
					if(j == it->second.obsTypeList.size())
					{
						fprintf(pEditedfile,"%2d %4d %2d %2d %2d %2d %11.7f",
							                editedObsSatlist[i].Id,
											it->first.year,
											it->first.month,
											it->first.day,
											it->first.hour,
											it->first.minute,
											it->first.second);
						for(size_t k = 0; k < it->second.obsTypeList.size(); k ++)
						{							
							fprintf(pEditedfile,"  %14.3f",
								it->second.obsTypeList[k].obs.data);
						}
						fprintf(pEditedfile, "\n");
					}
				}
			}
		}
		return true;
	}


	// 子程序名称： getEditedObsEpochList   
	// 功能：根据编辑后文件内容，获得 editedObsEpochlist
	// 变量类型：editedObsEpochlist   : 预处理后数据
	// 输入：
	// 输出：editedObsEpochlist 
	// 其它： 
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/9/9
	// 版本时间：2012/4/11
	// 修改记录：
	// 备注：
	bool Rinex2_1_EditedObsFile::getEditedObsEpochList(vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist)
	{
		if(isEmpty())
			return false;
		editedObsEpochlist = m_data;
		return true;
	}

	// 子程序名称： getEditedObsEpochList   
	// 功能：根据编辑后文件内容、起始时间 t_Begin 和终止时间 t_End, 获得 editedObsEpochlist
	//         ( 注意nObsTime要重新标记 )
	// 变量类型：editedObsEpochlist   : 预处理后数据 [t_Begin, t_End)
	//           t_Begin              : 起始时间
	//           t_End                : 终止时间
	// 输入：
	// 输出：editedObsEpochlist 
	// 其它： 
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/9/9
	// 版本时间：2012/4/11
	// 修改记录：
	// 备注：
	bool Rinex2_1_EditedObsFile::getEditedObsEpochList(vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End)
	{
		if(isEmpty())
			return false;
		editedObsEpochlist.clear();
		int k = 0;
		for(size_t i = 0; i < m_data.size(); i++)
		{
			Rinex2_1_EditedObsEpoch editedObsEpoch = m_data[i];
			if(editedObsEpoch.t - t_Begin < 0 )      // 确保 editedObsEpoch.T >= T_Begin
				continue;
			else if(editedObsEpoch.t - t_End >= 0 )  // 确保 editedObsEpoch.T <  T_End
				break;
			else                                     // T_Begin =< editedObsEpoch.T < T_End
			{
				k++;
				// 更新有效时间标记, 20070910 添加
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpoch.editedObs.begin(); it != editedObsEpoch.editedObs.end(); ++it)
				{
					it->second.nObsTime = k - 1;
				}
				editedObsEpochlist.push_back(editedObsEpoch);
			}
		}
		if(editedObsEpochlist.size() <= 0)
			return false;
		return true;
	}

	// 子程序名称： getEditedObsSatList   
	// 功能：根据编辑后文件内容，获得 editedObsSatlist
	// 变量类型：editedObsSatlist   : 预处理后数据
	// 输入：
	// 输出：editedObsSatlist 
	// 其它： 
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/9/9
	// 版本时间：2012/4/11
	// 修改记录：
	// 备注：
	bool Rinex2_1_EditedObsFile::getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist)
	{
		Rinex2_1_EditedObsSat editedObsSatlist_max[MAX_PRN]; 
		for(int i = 0; i < MAX_PRN; i++)
		{
			editedObsSatlist_max[i].Id = i;
			editedObsSatlist_max[i].editedObs.clear();
		}
		// 遍历每个历元的观测数据   /* 耗时7秒左右 */
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			// 在每个历元，遍历每颗GPS卫星的数据
			for(Rinex2_1_EditedObsSatMap::iterator it = m_data[s_i].editedObs.begin(); it != m_data[s_i].editedObs.end(); ++it)
			{
				BYTE bySatPRN = it->first;
				editedObsSatlist_max[bySatPRN].editedObs.insert(Rinex2_1_EditedObsEpochMap::value_type(m_data[s_i].t, it->second));
			}
		}
		// 整理所有可视卫星数据
		editedObsSatlist.clear();
		for(int i = 0; i < MAX_PRN; i++)
		{
			if(editedObsSatlist_max[i].editedObs.size()>0)
			{
				editedObsSatlist.push_back(editedObsSatlist_max[i]);
			}
		}
		return true;
	}

	// 子程序名称： getEditedObsSatList   
	// 功能：根据 editedObsEpochlist , 获得 editedObsSatlist
	// 变量类型：editedObsEpochlist  : 
	//           editedObsSatlist    : 
	// 输入：editedObsEpochlist
	// 输出：editedObsSatlist 
	// 其它： 
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/9/9
	// 版本时间：2012/4/11
	// 修改记录：
	// 备注：
	bool Rinex2_1_EditedObsFile::getEditedObsSatList(vector<Rinex2_1_EditedObsEpoch> editedObsEpochlist, vector<Rinex2_1_EditedObsSat>& editedObsSatlist)
	{
		if(editedObsEpochlist.size() <= 0)
			return false;

		Rinex2_1_EditedObsSat editedObsSatlist_max[MAX_PRN]; // 卫星列表
		for(int i = 0; i < MAX_PRN; i++)
		{
			editedObsSatlist_max[i].Id = i;
			editedObsSatlist_max[i].editedObs.clear();
		}
		// 遍历每个历元的观测数据   /* 耗时7秒左右 */
		for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
		{
			// 在每个历元，遍历每颗GPS卫星的数据
			for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
			{
				BYTE byID = it->first;
				editedObsSatlist_max[byID].editedObs.insert(Rinex2_1_EditedObsEpochMap::value_type(editedObsEpochlist[s_i].t, it->second));
			}
		}
		// 整理所有可视卫星数据
		editedObsSatlist.clear();
		for(int i = 0; i < MAX_PRN; i++)
		{
			if(editedObsSatlist_max[i].editedObs.size() > 0)
			{
				editedObsSatlist.push_back(editedObsSatlist_max[i]);
			}
		}
		return true;
	}

	// 子程序名称： getEditedObsSatList   
	// 功能：根据编辑后文件内容、起始时间 T_Begin 和终止时间 T_End ，获得 editedObsSatlist
	// 变量类型：  editedObsSatlist   : 预处理后数据 [t_Begin, t_End)
	//             t_Begin            : 起始时间
	//             t_End              : 终止时间
	// 输入：
	// 输出：editedObsSatlist 
	// 其它： 
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/9/9
	// 版本时间：2012/4/11
	// 修改记录：
	// 备注：
	bool Rinex2_1_EditedObsFile::getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End)
	{
		vector<Rinex2_1_EditedObsEpoch> editedObsEpochlist;
		if(!getEditedObsEpochList(editedObsEpochlist, t_Begin, t_End))
			return false;
		return getEditedObsSatList(editedObsEpochlist, editedObsSatlist);
	}
	// 子程序名称：datalist_sat2epoch   
	// 作用：将 editedObsSatlist 转换到 editedObsEpochlist, 为了保持时间的完整性, 
	//         转换期间以 m_obsFile.m_data[s_i].T 为参考
	// 类型：editedObsSatlist   : 数据结构格式1: 根据不同卫星(或测站)进行分类 , 称为editedObsSatlist  , 主要用途方便时间序列处理
	//         editedObsEpochlist : 数据结构格式2: 根据不同历元时刻进行分类, 称为editedObsEpochlist, 主要用途方便单点定位
	// 输入：editedObsSatlist
	// 输出：editedObsEpochlist 		
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/5/8
	// 版本时间：2012/4/11
	// 修改记录：2013/12/15,将此函数增加到Rinex2_1_EditedObsFile中
	//           2014/07/01,补全历元信息。//刘俊宏
	// 其它：依赖 m_data
	bool Rinex2_1_EditedObsFile::datalist_sat2epoch(vector<Rinex2_1_EditedObsSat> &editedObsSatlist, vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist)
	{
		if(editedObsSatlist.size() <= 0)
			return false;
		editedObsEpochlist.clear();
		//editedObsEpochlist.resize(m_obsFile.m_data.size());
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			Rinex2_1_EditedObsEpoch editedObsEpoch;
            editedObsEpoch.byEpochFlag             = m_data[s_i].byEpochFlag;
			editedObsEpoch.bySatCount              = m_data[s_i].bySatCount;
			editedObsEpoch.clock                   = m_data[s_i].clock;
			editedObsEpoch.tropZenithDelayPriori_H = m_data[s_i].tropZenithDelayPriori_H;
			editedObsEpoch.tropZenithDelayPriori_W = m_data[s_i].tropZenithDelayPriori_W;
			editedObsEpoch.tropZenithDelayEstimate = m_data[s_i].tropZenithDelayEstimate;
			editedObsEpoch.temperature             = m_data[s_i].temperature;
			editedObsEpoch.humidity                = m_data[s_i].humidity;
			editedObsEpoch.pressure                = m_data[s_i].pressure;
			editedObsEpoch.t                       = m_data[s_i].t;
			editedObsEpoch.editedObs.clear();
			// 遍历每颗 BD 卫星的数据列表
			for(size_t s_j = 0; s_j < editedObsSatlist.size(); s_j++)
			{// 判断当前时刻的数据是否符合要求(!前提是预处理期间，时间标签未被改动!)
				Rinex2_1_EditedObsEpochMap::const_iterator it;
				if((it = editedObsSatlist[s_j].editedObs.find(editedObsEpoch.t)) != editedObsSatlist[s_j].editedObs.end())
				{
					editedObsEpoch.editedObs.insert(Rinex2_1_EditedObsSatMap::value_type(editedObsSatlist[s_j].Id, it->second));
				}
			}
		    //editedObsEpochlist[s_i] = editedObsEpoch;
			if(editedObsEpoch.editedObs.size() > 0)
				editedObsEpochlist.push_back(editedObsEpoch);
		}
		return true;
	}
}
