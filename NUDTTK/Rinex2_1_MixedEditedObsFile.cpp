#include "Rinex2_1_MixedEditedObsFile.hpp"
#pragma warning(disable: 4996)

namespace NUDTTK
{
	Rinex2_1_MixedEditedObsFile::Rinex2_1_MixedEditedObsFile(void)
	{
	}

	Rinex2_1_MixedEditedObsFile::~Rinex2_1_MixedEditedObsFile(void)
	{
	}

	void Rinex2_1_MixedEditedObsFile::clear()
	{
		m_header = Rinex2_1_MixedObsHeader::Rinex2_1_MixedObsHeader();
		m_data.clear();
	}

	bool Rinex2_1_MixedEditedObsFile::isEmpty()
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
	bool Rinex2_1_MixedEditedObsFile::cutdata(DayTime t_Begin, DayTime t_End)
	{
		// 确保 t_Begin <= t_End
		if( t_Begin - t_End >= 0 || t_End - m_header.tmStart <= 0 || t_Begin - m_header.tmEnd > 0 )
			return false;
		vector<Rinex2_1_MixedEditedObsEpoch> obsDataList;
		obsDataList.clear();
		map<string, int> mapSatNameList; // 2014/12/01, 针对Mixed格式进行调整 
		mapSatNameList.clear();
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			Rinex2_1_MixedEditedObsEpoch ObsEpoch = m_data[s_i];
			if(ObsEpoch.t - t_Begin >= 0 && ObsEpoch.t - t_End < 0)
			{
				for(Rinex2_1_MixedEditedObsSatMap::iterator it = ObsEpoch.editedObs.begin(); it != ObsEpoch.editedObs.end(); ++it)
				{
					it->second.nObsTime = int(obsDataList.size());
					if(mapSatNameList.find(it->first) != mapSatNameList.end())
						mapSatNameList[it->first]++;
					else
						mapSatNameList.insert(map<string, int>::value_type(it->first, 1));
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
		m_header.pstrSatList.clear();
		for(map<string, int>::iterator it = mapSatNameList.begin(); it != mapSatNameList.end(); ++it)
		{
			m_header.pstrSatList.push_back(it->first);
		}
		m_header.bySatCount = BYTE(m_header.pstrSatList.size());
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
	int  Rinex2_1_MixedEditedObsFile::isValidEpochLine(string strLine, FILE * pEditedObsfile)
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
		if(bySatCount < 0)
			return 2;
		return nFlag;
	}

	// 子程序名称： open   
	// 功能：编辑后的观测数据解析 
	// 变量类型：strEditedObsfileName : 编辑后的观测数据文件路径
	//           strSystem            : 卫星导航系统缩写字母集合, GPS = "G", BDS = "C", 可以多个系统联合, 如"GC"
	// 输入：strEditedObsfileName, strSystem
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/8/27
	// 版本时间：2012/4/11
	// 修改记录：
	// 备注： 
	bool Rinex2_1_MixedEditedObsFile::open(string strEditedObsfileName, string strSystem)
	{
		FILE * pEditedObsfile = fopen(strEditedObsfileName.c_str(), "r+t");
		if(pEditedObsfile == NULL) 
			return false;
		m_header = Rinex2_1_MixedObsHeader::Rinex2_1_MixedObsHeader();
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
		map<string, int> mapSatNameList; // 可视卫星列表, 2014/12/01, 针对Mixed格式进行调整
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
				Rinex2_1_MixedEditedObsEpoch editedObsEpoch;
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
				BYTE bySatCount_GNSS = 0; // 存储当前历元可视卫星中有效GNSS卫星(符合szSystem定义)个数
				bool *pbSatSysValidFlag  = new bool [editedObsEpoch.bySatCount];	// 记录当前可视卫星序列中的每颗卫星类型是否有效, 用于指导数据行读取时遴选有效数据
				for(int i = 0; i < editedObsEpoch.bySatCount; i++)
				{
					fgets(line, 400, pEditedObsfile);
					string strLine = line;
					Rinex2_1_MixedEditedObsLine editedObsLine;
					editedObsLine.nObsTime = k - 1; // 2007/07/22 添加
					char strSatName[4]; // 存储卫星系统 + PRN (例如'G01')
					sscanf(line,"%3c%8lf%8lf",//修改  2017/7/7  昌虓
						        strSatName,
                                &editedObsLine.Elevation,
                                &editedObsLine.Azimuth);
					strSatName[3] = '\0';
					if(strSatName[0] == ' ')
						strSatName[0] = 'G';
					if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
						strSatName[1] = '0';
					editedObsLine.satName = strSatName;
					if(strSystem.find(strSatName[0]) != -1)
					{
						// 可见卫星观测数据个数累计
						if(mapSatNameList.find(strSatName) != mapSatNameList.end())
							mapSatNameList[strSatName]++;
						else
							mapSatNameList.insert(map<string, int>::value_type(strSatName, 1));
						pbSatSysValidFlag[i] = true;
						bySatCount_GNSS++;
					}
					else
					{
						pbSatSysValidFlag[i] = false;
						continue;
					}
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
					editedObsEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(editedObsLine.satName, editedObsLine));
				}
				editedObsEpoch.bySatCount = bySatCount_GNSS; // 更改当前历元可视卫星数(仅统计 GPS 卫星个数)
				m_data.push_back(editedObsEpoch);
				fgets(line, 400, pEditedObsfile);
				// 释放动态内存空间
				delete[] pbSatSysValidFlag;
			}
			else  // 无效数据行, 比如空白行, 掠过不作处理
			{   
				fgets(line, 400, pEditedObsfile);
			}
		}
		// 综合统计可视卫星列表
		m_header.pstrSatList.clear();
		for(map<string, int>::iterator it = mapSatNameList.begin(); it != mapSatNameList.end(); ++it)
		{
			m_header.pstrSatList.push_back(it->first);
		}
		m_header.bySatCount = BYTE(m_header.pstrSatList.size());
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
	bool Rinex2_1_MixedEditedObsFile::write(string strEditedObsfileName)
	{
		if(m_data.size() <= 0)
			return false;
		// 2008-08-08 进行了调整, 文件头信息得到了补充
		FILE* pEditedfile = fopen(strEditedObsfileName.c_str(), "w+");
		//// 若 RINEX 文件类型为 M (Mixed)，目前仅读取 GPS 数据，写文件时将 'M'改写为'G'(2013/6/28)
		//if(m_header.getSatSystemChar() == 'M')
		//{
		//	m_header.szSatlliteSystem[0] = 'G';
		//}
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
		for(size_t s_i = 0; s_i < m_data.size(); s_i ++)
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

			for(Rinex2_1_MixedEditedObsSatMap::iterator it = m_data[s_i].editedObs.begin(); it != m_data[s_i].editedObs.end(); ++it)
			{//fprintf(pObsfile,"%c%2d", m_header.getSatSystemChar(),it->first);
				Rinex2_1_MixedEditedObsLine editedObsLine = it->second;
				fprintf(pEditedfile,"%3s%8.3f%8.3f",//此处原有%2d在%3s后面，去除  2016/12/26  ChangXiao
					                editedObsLine.satName.c_str(), // 2014/12/01, 针对Mixed格式进行调整
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
	bool Rinex2_1_MixedEditedObsFile::getEditedObsEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist)
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
	bool Rinex2_1_MixedEditedObsFile::getEditedObsEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End)
	{
		if(isEmpty())
			return false;
		editedObsEpochlist.clear();
		int k = 0;
		for(size_t i = 0; i < m_data.size(); i++)
		{
			Rinex2_1_MixedEditedObsEpoch editedObsEpoch = m_data[i];
			if(editedObsEpoch.t - t_Begin < 0 )      // 确保 editedObsEpoch.T >= T_Begin
				continue;
			else if(editedObsEpoch.t - t_End >= 0 )  // 确保 editedObsEpoch.T <  T_End
				break;
			else                                     // T_Begin =< editedObsEpoch.T < T_End
			{
				k++;
				// 更新有效时间标记, 20070910 添加
				for(Rinex2_1_MixedEditedObsSatMap::iterator it = editedObsEpoch.editedObs.begin(); it != editedObsEpoch.editedObs.end(); ++it)
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
	bool Rinex2_1_MixedEditedObsFile::getEditedObsSatList(vector<Rinex2_1_MixedEditedObsSat>& editedObsSatlist)
	{
		map<string, Rinex2_1_MixedEditedObsSat> mapEditedObsSatlist; // 2014/12/01, 针对Mixed格式进行调整 
		mapEditedObsSatlist.clear();
		// 遍历每个历元的观测数据   /* 耗时7秒左右 */
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			// 在每个历元，遍历每颗GPS卫星的数据
			for(Rinex2_1_MixedEditedObsSatMap::iterator it = m_data[s_i].editedObs.begin(); it != m_data[s_i].editedObs.end(); ++it)
			{
				if(mapEditedObsSatlist.find(it->first) != mapEditedObsSatlist.end())
					mapEditedObsSatlist[it->first].editedObs.insert(Rinex2_1_MixedEditedObsEpochMap::value_type(m_data[s_i].t, it->second));
				else
				{// 发现新的卫星的数据
					Rinex2_1_MixedEditedObsSat editedObsSat;
					editedObsSat.satName = it->first;
					editedObsSat.editedObs.clear();
					editedObsSat.editedObs.insert(Rinex2_1_MixedEditedObsEpochMap::value_type(m_data[s_i].t, it->second));
					mapEditedObsSatlist.insert(map<string, Rinex2_1_MixedEditedObsSat>::value_type(it->first, editedObsSat));
				}
			}
		}
		// 整理所有可视卫星数据
		editedObsSatlist.clear();
		for(map<string, Rinex2_1_MixedEditedObsSat>::iterator it = mapEditedObsSatlist.begin(); it != mapEditedObsSatlist.end(); ++it)
		{
			if(it->second.editedObs.size() > 0)
				editedObsSatlist.push_back(it->second);
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
	bool Rinex2_1_MixedEditedObsFile::getEditedObsSatList(vector<Rinex2_1_MixedEditedObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End)
	{
		vector<Rinex2_1_MixedEditedObsEpoch> editedObsEpochlist;
		if(!getEditedObsEpochList(editedObsEpochlist, t_Begin, t_End))
			return false;
		return getEditedObsSatList(editedObsEpochlist, editedObsSatlist);
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
	bool Rinex2_1_MixedEditedObsFile::getEditedObsSatList(vector<Rinex2_1_MixedEditedObsEpoch> editedObsEpochlist, vector<Rinex2_1_MixedEditedObsSat>& editedObsSatlist)
	{
		if(editedObsEpochlist.size() <= 0)
			return false;
		map<string, Rinex2_1_MixedEditedObsSat> mapEditedObsSatlist; // 2014/12/01, 针对Mixed格式进行调整 
		mapEditedObsSatlist.clear();
		// 遍历每个历元的观测数据   /* 耗时7秒左右 */
		for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
		{
			// 在每个历元，遍历每颗GPS卫星的数据
			for(Rinex2_1_MixedEditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
			{
				if(mapEditedObsSatlist.find(it->first) != mapEditedObsSatlist.end())
					mapEditedObsSatlist[it->first].editedObs.insert(Rinex2_1_MixedEditedObsEpochMap::value_type(editedObsEpochlist[s_i].t, it->second));
				else
				{// 发现新的卫星的数据
					Rinex2_1_MixedEditedObsSat editedObsSat;
					editedObsSat.satName = it->first;
					editedObsSat.editedObs.clear();
					editedObsSat.editedObs.insert(Rinex2_1_MixedEditedObsEpochMap::value_type(editedObsEpochlist[s_i].t, it->second));
					mapEditedObsSatlist.insert(map<string, Rinex2_1_MixedEditedObsSat>::value_type(it->first, editedObsSat));
				}
			}
		}
		// 整理所有可视卫星数据
		editedObsSatlist.clear();
		for(map<string, Rinex2_1_MixedEditedObsSat>::iterator it = mapEditedObsSatlist.begin(); it != mapEditedObsSatlist.end(); ++it)
		{
			if(it->second.editedObs.size() > 0)
				editedObsSatlist.push_back(it->second);
		}
		return true;
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
	// 修改记录：2013/12/15,将此函数增加到 Rinex2_1_EditedObsFile 中
	//           2014/07/01,补全历元信息,刘俊宏
	// 其它：依赖 m_data
	bool Rinex2_1_MixedEditedObsFile::datalist_sat2epoch(vector<Rinex2_1_MixedEditedObsSat> &editedObsSatlist, vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist)
	{
		if(editedObsSatlist.size() <= 0)
			return false;
		editedObsEpochlist.clear();
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			Rinex2_1_MixedEditedObsEpoch editedObsEpoch;
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
				Rinex2_1_MixedEditedObsEpochMap::const_iterator it = editedObsSatlist[s_j].editedObs.find(editedObsEpoch.t);
				if(it != editedObsSatlist[s_j].editedObs.end())
					editedObsEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(editedObsSatlist[s_j].satName, it->second));
			}
			if(editedObsEpoch.editedObs.size() > 0)
				editedObsEpochlist.push_back(editedObsEpoch);
		}
		return true;
	}
	
	// 子程序名称： mainFunc_editedMixObsFile   
	// 功能：根据编辑后各系统的观测文件结构，输出编辑后混合格式的观测文件（暂时采用2.11格式）
	// 变量类型：  strEditedMixObsFileName : 预处理后混合格式观测文件路径
	//             systemFlag              : 参与合成文件的系统标识集：G-GPS, C-BDS, R-GLONASS, E-Galileo
	//             editedObsFile_gps       : 编辑后的GPS观测数据文件结构
	//             editedObsFile_bds       : 编辑后的BDS观测数据文件结构
	// 输入：editedObsFile_gps， editedObsFile_bds
	// 输出：strEditedMixObsFileName 
	// 其它： 
	// 语言：C++
	// 创建者：昌虓
	// 创建时间：2016/12/16
	// 版本时间：
	// 修改记录：
	// 备注：
	bool Rinex2_1_MixedEditedObsFile::mainFunc_editedMixObsFile(string strEditedMixObsFileName, string systemFlag, Rinex2_1_EditedObsFile &editedObsFile_gps, Rinex2_1_EditedObsFile &editedObsFile_bds, char RecType_gps)
	{
		bool bOnGPS = false, bOnBDS = false;
		//0 判断待合成文件的系统是否有预处理后对应的文件
		if(systemFlag.find('G') != systemFlag.npos && systemFlag.find('C') != systemFlag.npos)
		{
			bOnGPS = true;
			bOnBDS = true;
			if(editedObsFile_gps.isEmpty() && editedObsFile_bds.isEmpty())
				return false;
		}
		else if(systemFlag.find('G') != systemFlag.npos && systemFlag.find('C') == systemFlag.npos)
		{
			bOnGPS = true;
			if(editedObsFile_gps.isEmpty())
				return false;
		}
		else if(systemFlag.find('C') != systemFlag.npos && systemFlag.find('G') == systemFlag.npos)
		{
			bOnBDS = true;
			if(editedObsFile_bds.isEmpty())
				return false;
		}
		else
		{
			printf("多系统融合观测文件合并标识有误，请重新输入！\n");
			return false;
		}
		Rinex2_1_MixedEditedObsFile  mixedEditedObsFile;
		GPST                         t_start,t_end;// 合并后的edit文件起止时间
		double                       interval = DBL_MAX;      // 采样间隔

		//1 确定各系统双频观测数据对应的频点类型
		int nObsTypes_P1_GPS = -1, nObsTypes_P2_GPS = -1,nObsTypes_P1_BDS = -1, nObsTypes_P2_BDS = -1;
		int type_obs_P1_GPS  = TYPE_OBS_P1;
		int type_obs_P2_GPS  = TYPE_OBS_P2;
		int type_obs_P1_BDS  = TYPE_OBS_P1;
		int type_obs_P2_BDS  = TYPE_OBS_P2;
		if(RecType_gps != 'N')//这里需要再考虑下，针对'P'类接收机，是否还要进行其他处理  ChangXiao
			type_obs_P1_GPS  = TYPE_OBS_C1;
		if(m_DualFreqPreDefine.typ_GPSobs_L1 == TYPE_OBS_L2)
			type_obs_P1_GPS  = TYPE_OBS_P2;
		if(m_DualFreqPreDefine.typ_GPSobs_L1 == TYPE_OBS_L5)
			type_obs_P1_GPS  = TYPE_OBS_P5;
		if(m_DualFreqPreDefine.typ_GPSobs_L2 == TYPE_OBS_L1)
			type_obs_P2_GPS  = TYPE_OBS_P1;
		if(m_DualFreqPreDefine.typ_GPSobs_L2 == TYPE_OBS_L5)
			type_obs_P2_GPS  = TYPE_OBS_P5;
		if(m_DualFreqPreDefine.typ_BDSobs_L1 == TYPE_OBS_L2)					
			type_obs_P1_BDS  = TYPE_OBS_P2;			
		if(m_DualFreqPreDefine.typ_BDSobs_L1 == TYPE_OBS_L5)						
			type_obs_P1_BDS  = TYPE_OBS_P5;			
		if(m_DualFreqPreDefine.typ_BDSobs_L2 == TYPE_OBS_L1)				
			type_obs_P2_BDS  = TYPE_OBS_P1;			
		if(m_DualFreqPreDefine.typ_BDSobs_L2 == TYPE_OBS_L5)			
			type_obs_P2_BDS  = TYPE_OBS_P5;
		//根据需处理系统的类别分别进行整理，使得混合格式可输出单系统文件以供单系统后续使用
		if(bOnGPS && bOnBDS)
		{
			bool onGPSEmpty = editedObsFile_gps.isEmpty();
			int  obsTypesListMark_GPS[7+1], obsTypesListMark_BDS[7+1];
			for(size_t si = 0; si < 7; si++)
			{
				obsTypesListMark_GPS[si] = -1;
				obsTypesListMark_BDS[si] = -1;
			}
			obsTypesListMark_GPS[7] = '\0';
			obsTypesListMark_BDS[7] = '\0';
			if(!onGPSEmpty)
			{
				t_start = editedObsFile_gps.m_data.front().t;
				t_end   = editedObsFile_gps.m_data.back().t;
				interval = editedObsFile_gps.m_header.Interval;				
				for(int i = 0; i < editedObsFile_gps.m_header.byObsTypes; i++)
				{					
					if(editedObsFile_gps.m_header.pbyObsTypeList[i] == type_obs_P1_GPS)  //第一个频点伪距
						nObsTypes_P1_GPS = i;
					if(editedObsFile_gps.m_header.pbyObsTypeList[i] == type_obs_P2_GPS)  //第二个频点伪距
						nObsTypes_P2_GPS = i;
					if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_C1)
						obsTypesListMark_GPS[0] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
						obsTypesListMark_GPS[1] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
						obsTypesListMark_GPS[2] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_P5)
						obsTypesListMark_GPS[3] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
						obsTypesListMark_GPS[4] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
						obsTypesListMark_GPS[5] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_L5)
						obsTypesListMark_GPS[6] = i;
					else
						continue;
				}
				if(nObsTypes_P1_GPS == -1 || nObsTypes_P2_GPS == -1)
				{
					printf("GPS所需频点预处理后的观测数据缺失，预处理失败！\n");
					return false;
				}			
			}				
			if(!editedObsFile_bds.isEmpty())
			{
				if(onGPSEmpty == true)//此处因调试更改  昌虓  2016/12/20
				{
					t_start = editedObsFile_bds.m_data.front().t;
					t_end   = editedObsFile_bds.m_data.back().t;
					interval = editedObsFile_bds.m_header.Interval;
				}
				else
				{
					if(t_start - editedObsFile_bds.m_data.front().t > 0)
						t_start = editedObsFile_bds.m_data.front().t;
					if(t_end  - editedObsFile_bds.m_data.back().t < 0)
						t_end = editedObsFile_bds.m_data.back().t;
				}	
				for(int i = 0; i < editedObsFile_bds.m_header.byObsTypes; i++)
				{					
					if(editedObsFile_bds.m_header.pbyObsTypeList[i] == type_obs_P1_BDS)  //第一个频点伪距
						nObsTypes_P1_BDS = i;
					if(editedObsFile_bds.m_header.pbyObsTypeList[i] == type_obs_P2_BDS)  //第二个频点伪距
						nObsTypes_P2_BDS = i;
					if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
						obsTypesListMark_BDS[1] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
						obsTypesListMark_BDS[2] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_P5)
						obsTypesListMark_BDS[3] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
						obsTypesListMark_BDS[4] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
						obsTypesListMark_BDS[5] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_L5)
						obsTypesListMark_BDS[6] = i;
					else
						continue;
				}
				if(nObsTypes_P1_BDS == -1 || nObsTypes_P2_BDS == -1) 
				{
					printf("BDS所需频点预处理后的观测数据缺失，预处理失败！\n");
					return false;
				}
			}

			//2 获取混合格式下单历元观测数据结构
			int        i = 0;
			size_t   s_j = 0;
			size_t   s_k = 0;
			int      nObsTime = 0;
			if(interval == DBL_MAX)
				interval = 30.00;//  测试用，后续需更改。针对某些测站（如areg 2015.04.01观测文件中不含有interval项，在下面会出错）昌虓  2016/12/20
			while(t_start + i * interval - t_end <= 0)
			{
				Rinex2_1_MixedEditedObsEpoch  mixedEpoch;
				Rinex2_1_EditedObsEpoch       gpsEpoch, bdsEpoch;
				gpsEpoch.editedObs.clear();
				bdsEpoch.editedObs.clear();
				GPST t_epoch = t_start + i * interval;
				//寻找当前时刻，gps历元
				for(size_t s_i = s_j; s_i < editedObsFile_gps.m_data.size(); s_i ++)
				{
					if(fabs(editedObsFile_gps.m_data[s_i].t - t_epoch) < 1.0e-5)
					{
						gpsEpoch = editedObsFile_gps.m_data[s_i];
						s_j = s_i;
						break;
					}
					if(editedObsFile_gps.m_data[s_i].t - t_epoch >= 1.0e-5)
					{
						s_j = s_i;
						break;
					}
				}
				//寻找当前时刻，bds历元
				for(size_t s_ii = s_k; s_ii < editedObsFile_bds.m_data.size(); s_ii ++)
				{
					if(fabs(editedObsFile_bds.m_data[s_ii].t - t_epoch) < 1.0e-5)
					{
						bdsEpoch = editedObsFile_bds.m_data[s_ii];
						s_k = s_ii;
						break;
					}
					if(editedObsFile_bds.m_data[s_ii].t - t_epoch >= 1.0e-5)
					{
						s_k = s_ii;
						break;
					}
				}
				if(gpsEpoch.editedObs.size() > 0)
				{
					mixedEpoch.t = t_epoch;
					mixedEpoch.byEpochFlag = gpsEpoch.byEpochFlag;
					mixedEpoch.bySatCount  = gpsEpoch.bySatCount;                                                       
					mixedEpoch.clock       = gpsEpoch.clock;
					for(Rinex2_1_EditedObsSatMap::iterator it = gpsEpoch.editedObs.begin();it != gpsEpoch.editedObs.end(); ++it)
					{
						char szSatName[4];
						sprintf(szSatName, "G%02d", it->first);
						szSatName[3] = '\0';
						Rinex2_1_MixedEditedObsLine  mixedLine;						
						mixedLine.satName       = szSatName;
						mixedLine.Azimuth       = it ->second.Azimuth;
						mixedLine.Elevation     = it->second.Elevation;
						mixedLine.ReservedField = it->second.ReservedField;
						Rinex2_1_EditedObsDatum obsDatum_GPS;
						for(size_t si = 0; si < 7; si++)
						{
							if(obsTypesListMark_GPS[si] != -1)
								obsDatum_GPS = it->second.obsTypeList[obsTypesListMark_GPS[si]];
							else
							{
								obsDatum_GPS.obs.data = 0.0;
								obsDatum_GPS.byEditedMark1 = 0;
								obsDatum_GPS.byEditedMark2 = 0;
							}
							mixedLine.obsTypeList.push_back(obsDatum_GPS);
						}
						mixedEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(szSatName,mixedLine));						
					}
				}
				if(bdsEpoch.editedObs.size() > 0)
				{
					if(gpsEpoch.editedObs.size() > 0)
						mixedEpoch.bySatCount += bdsEpoch.bySatCount;
					else
					{
						mixedEpoch.t           = t_epoch;
						mixedEpoch.byEpochFlag = bdsEpoch.byEpochFlag;
						mixedEpoch.bySatCount  = bdsEpoch.bySatCount;
						mixedEpoch.clock       = bdsEpoch.clock;
					}
					for(Rinex2_1_EditedObsSatMap::iterator it = bdsEpoch.editedObs.begin();it != bdsEpoch.editedObs.end(); ++it)
					{
						char szSatName[4];
						sprintf(szSatName, "C%02d", it->first);
						szSatName[3] = '\0';
						Rinex2_1_MixedEditedObsLine  mixedLine;						
						mixedLine.satName       = szSatName;
						mixedLine.Azimuth       = it ->second.Azimuth;
						mixedLine.Elevation     = it->second.Elevation;
						mixedLine.ReservedField = it->second.ReservedField;
						Rinex2_1_EditedObsDatum obsDatum_BDS;
						for(size_t si = 0; si < 7; si++)
						{
							if(obsTypesListMark_BDS[si] != -1)
								obsDatum_BDS = it->second.obsTypeList[obsTypesListMark_BDS[si]];
							else
							{
								obsDatum_BDS.obs.data = 0.0;
								obsDatum_BDS.byEditedMark1 = 0;
								obsDatum_BDS.byEditedMark2 = 0;
							}
							mixedLine.obsTypeList.push_back(obsDatum_BDS);
						}
						mixedEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(szSatName,mixedLine));						
					}
				}
				// 确定nObsTime
				if(mixedEpoch.editedObs.size() > 0)
				{
					for(Rinex2_1_MixedEditedObsSatMap::iterator it = mixedEpoch.editedObs.begin();it != mixedEpoch.editedObs.end(); ++it)
					{						
						it->second.nObsTime = nObsTime;
					}				
					mixedEditedObsFile.m_data.push_back(mixedEpoch);
					nObsTime ++;
				}				
				i ++;
			}

			//3 整理文件头信息
			if(!editedObsFile_gps.isEmpty())			
				mixedEditedObsFile.m_header.init(editedObsFile_gps.m_header);				
			if(!editedObsFile_bds.isEmpty())
			{
				if(editedObsFile_gps.isEmpty())				
					mixedEditedObsFile.m_header.init(editedObsFile_bds.m_header);
				else
				{
					sprintf(mixedEditedObsFile.m_header.szSatlliteSystem, "M (MIXED)           ");
					mixedEditedObsFile.m_header.bySatCount += editedObsFile_bds.m_header.bySatCount;
					for(size_t s_i = 0; s_i < editedObsFile_bds.m_header.pbySatList.size(); s_i ++)
					{
						char szSatName[4];
						sprintf(szSatName, "C%02d", editedObsFile_bds.m_header.pbySatList[s_i]);
						szSatName[3] = '\0';
						mixedEditedObsFile.m_header.pstrSatList.push_back(szSatName);
					}
				}
			}
			mixedEditedObsFile.m_header.tmStart = t_start;
			mixedEditedObsFile.m_header.tmEnd   = t_end;
			mixedEditedObsFile.m_header.byObsTypes = 7;
			mixedEditedObsFile.m_header.pbyObsTypeList.clear();
			mixedEditedObsFile.m_header.pbyObsTypeList.resize(mixedEditedObsFile.m_header.byObsTypes);
			for(int i_obs = 0; i_obs < 7; i_obs ++)
				mixedEditedObsFile.m_header.pbyObsTypeList[i_obs] = i_obs + 1;//仅保留三个频点的伪码和相位观测数据，且对GPS而言，区分C1和P1 昌虓
		}
		else if(bOnGPS && !bOnBDS)
		{
			int  obsTypesListMark_GPS[7+1];
			for(size_t si = 0; si < 7; si++)
			{
				obsTypesListMark_GPS[si] = -1;
			}
			obsTypesListMark_GPS[7] = '\0';
			if(!editedObsFile_gps.isEmpty())
			{
				t_start = editedObsFile_gps.m_data.front().t;
				t_end   = editedObsFile_gps.m_data.back().t;
				interval = editedObsFile_gps.m_header.Interval;				
				for(int i = 0; i < editedObsFile_gps.m_header.byObsTypes; i++)
				{					
					if(editedObsFile_gps.m_header.pbyObsTypeList[i] == type_obs_P1_GPS)  //第一个频点伪距
						nObsTypes_P1_GPS = i;
					if(editedObsFile_gps.m_header.pbyObsTypeList[i] == type_obs_P2_GPS)  //第二个频点伪距
						nObsTypes_P2_GPS = i;
					if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_C1)
						obsTypesListMark_GPS[0] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
						obsTypesListMark_GPS[1] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
						obsTypesListMark_GPS[2] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_P5)
						obsTypesListMark_GPS[3] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
						obsTypesListMark_GPS[4] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
						obsTypesListMark_GPS[5] = i;
					else if(editedObsFile_gps.m_header.pbyObsTypeList[i] == TYPE_OBS_L5)
						obsTypesListMark_GPS[6] = i;
					else
						continue;
				}
				if(nObsTypes_P1_GPS == -1 || nObsTypes_P2_GPS == -1) 
					return false;
				//2 获取混合格式下单历元观测数据结构
				int        i = 0;
				size_t   s_j = 0;
				size_t   s_k = 0;
				int      nObsTime = 0;
				if(interval == DBL_MAX)
					interval = 30.00;//  测试用，后续需更改。针对某些测站（如areg 2015.04.01观测文件中不含有interval项，在下面会出错）昌虓  2016/12/20
				while(t_start + i * interval - t_end <= 0)
				{
					Rinex2_1_MixedEditedObsEpoch  mixedEpoch;
					Rinex2_1_EditedObsEpoch       gpsEpoch;
					gpsEpoch.editedObs.clear();
					GPST t_epoch = t_start + i * interval;
					//寻找当前时刻，gps历元
					for(size_t s_i = s_j; s_i < editedObsFile_gps.m_data.size(); s_i ++)
					{
						if(fabs(editedObsFile_gps.m_data[s_i].t - t_epoch) < 1.0e-5)
						{
							gpsEpoch = editedObsFile_gps.m_data[s_i];
							s_j = s_i;
							break;
						}
						if(editedObsFile_gps.m_data[s_i].t - t_epoch >= 1.0e-5)
						{
							s_j = s_i;
							break;
						}
					}
					if(gpsEpoch.editedObs.size() > 0)
					{
						mixedEpoch.t = t_epoch;
						mixedEpoch.byEpochFlag = gpsEpoch.byEpochFlag;
						mixedEpoch.bySatCount  = gpsEpoch.bySatCount;                                                       
						mixedEpoch.clock       = gpsEpoch.clock;
						for(Rinex2_1_EditedObsSatMap::iterator it = gpsEpoch.editedObs.begin();it != gpsEpoch.editedObs.end(); ++it)
						{
							char szSatName[4];
							sprintf(szSatName, "G%02d", it->first);
							szSatName[3] = '\0';
							Rinex2_1_MixedEditedObsLine  mixedLine;						
							mixedLine.satName       = szSatName;
							mixedLine.Azimuth       = it ->second.Azimuth;
							mixedLine.Elevation     = it->second.Elevation;
							mixedLine.ReservedField = it->second.ReservedField;
							Rinex2_1_EditedObsDatum obsDatum_GPS;
							for(size_t si = 0; si < 7; si++)
							{
								if(obsTypesListMark_GPS[si] != -1)
									obsDatum_GPS = it->second.obsTypeList[obsTypesListMark_GPS[si]];
								else
								{
									obsDatum_GPS.obs.data = 0.0;
									obsDatum_GPS.byEditedMark1 = 0;
									obsDatum_GPS.byEditedMark2 = 0;
								}
								mixedLine.obsTypeList.push_back(obsDatum_GPS);
							}
							mixedEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(szSatName,mixedLine));						
						}
					}
					// 确定nObsTime
					if(mixedEpoch.editedObs.size() > 0)
					{
						for(Rinex2_1_MixedEditedObsSatMap::iterator it = mixedEpoch.editedObs.begin();it != mixedEpoch.editedObs.end(); ++it)
						{						
							it->second.nObsTime = nObsTime;
						}				
						mixedEditedObsFile.m_data.push_back(mixedEpoch);
						nObsTime ++;
					}				
					i ++;
				}

				//3 整理文件头信息			
				mixedEditedObsFile.m_header.init(editedObsFile_gps.m_header);				
				sprintf(mixedEditedObsFile.m_header.szSatlliteSystem, "M (MIXED)           ");
				mixedEditedObsFile.m_header.tmStart = t_start;
				mixedEditedObsFile.m_header.tmEnd   = t_end;
				mixedEditedObsFile.m_header.byObsTypes = 7;
				mixedEditedObsFile.m_header.pbyObsTypeList.clear();
				mixedEditedObsFile.m_header.pbyObsTypeList.resize(mixedEditedObsFile.m_header.byObsTypes);
				for(int i_obs = 0; i_obs < 7; i_obs ++)
					mixedEditedObsFile.m_header.pbyObsTypeList[i_obs] = i_obs + 1;//仅保留三个频点的伪码和相位观测数据，且对GPS而言，区分C1和P1 昌虓
			}
		}
		else if(bOnBDS && !bOnGPS)
		{
			int  obsTypesListMark_BDS[7+1];
			for(size_t si = 0; si < 7; si++)
			{
				obsTypesListMark_BDS[si] = -1;
			}
			obsTypesListMark_BDS[7] = '\0';
			if(!editedObsFile_bds.isEmpty())
			{
				t_start = editedObsFile_bds.m_data.front().t;
				t_end   = editedObsFile_bds.m_data.back().t;
				interval = editedObsFile_bds.m_header.Interval;				
				for(int i = 0; i < editedObsFile_bds.m_header.byObsTypes; i++)
				{					
					if(editedObsFile_bds.m_header.pbyObsTypeList[i] == type_obs_P1_BDS)  //第一个频点伪距
						nObsTypes_P1_BDS = i;
					if(editedObsFile_bds.m_header.pbyObsTypeList[i] == type_obs_P2_BDS)  //第二个频点伪距
						nObsTypes_P2_BDS = i;
					if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
						obsTypesListMark_BDS[1] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
						obsTypesListMark_BDS[2] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_P5)
						obsTypesListMark_BDS[3] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
						obsTypesListMark_BDS[4] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
						obsTypesListMark_BDS[5] = i;
					else if(editedObsFile_bds.m_header.pbyObsTypeList[i] == TYPE_OBS_L5)
						obsTypesListMark_BDS[6] = i;
					else
						continue;
				}
				if(nObsTypes_P1_BDS == -1 || nObsTypes_P2_BDS == -1) 
					return false;
				//2 获取混合格式下单历元观测数据结构
				int        i = 0;
				size_t   s_j = 0;
				size_t   s_k = 0;
				int      nObsTime = 0;
				if(interval == DBL_MAX)
					interval = 30.00;//  测试用，后续需更改。针对某些测站（如areg 2015.04.01观测文件中不含有interval项，在下面会出错）昌虓  2016/12/20
				while(t_start + i * interval - t_end <= 0)
				{
					Rinex2_1_MixedEditedObsEpoch  mixedEpoch;
					Rinex2_1_EditedObsEpoch       bdsEpoch;
					bdsEpoch.editedObs.clear();
					GPST t_epoch = t_start + i * interval;
					//寻找当前时刻，bds历元
					for(size_t s_i = s_j; s_i < editedObsFile_bds.m_data.size(); s_i ++)
					{
						if(fabs(editedObsFile_bds.m_data[s_i].t - t_epoch) < 1.0e-5)
						{
							bdsEpoch = editedObsFile_bds.m_data[s_i];
							s_j = s_i;
							break;
						}
						if(editedObsFile_bds.m_data[s_i].t - t_epoch >= 1.0e-5)
						{
							s_j = s_i;
							break;
						}
					}
					if(bdsEpoch.editedObs.size() > 0)
					{
						mixedEpoch.t = t_epoch;
						mixedEpoch.byEpochFlag = bdsEpoch.byEpochFlag;
						mixedEpoch.bySatCount  = bdsEpoch.bySatCount;                                                       
						mixedEpoch.clock       = bdsEpoch.clock;
						for(Rinex2_1_EditedObsSatMap::iterator it = bdsEpoch.editedObs.begin();it != bdsEpoch.editedObs.end(); ++it)
						{
							char szSatName[4];
							sprintf(szSatName, "C%02d", it->first);
							szSatName[3] = '\0';
							Rinex2_1_MixedEditedObsLine  mixedLine;						
							mixedLine.satName       = szSatName;
							mixedLine.Azimuth       = it ->second.Azimuth;
							mixedLine.Elevation     = it->second.Elevation;
							mixedLine.ReservedField = it->second.ReservedField;
							Rinex2_1_EditedObsDatum obsDatum_BDS;
							for(size_t si = 0; si < 7; si++)
							{
								if(obsTypesListMark_BDS[si] != -1)
									obsDatum_BDS = it->second.obsTypeList[obsTypesListMark_BDS[si]];
								else
								{
									obsDatum_BDS.obs.data = 0.0;
									obsDatum_BDS.byEditedMark1 = 0;
									obsDatum_BDS.byEditedMark2 = 0;
								}
								mixedLine.obsTypeList.push_back(obsDatum_BDS);
							}
							mixedEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(szSatName,mixedLine));						
						}
					}
					// 确定nObsTime
					if(mixedEpoch.editedObs.size() > 0)
					{
						for(Rinex2_1_MixedEditedObsSatMap::iterator it = mixedEpoch.editedObs.begin();it != mixedEpoch.editedObs.end(); ++it)
						{						
							it->second.nObsTime = nObsTime;
						}				
						mixedEditedObsFile.m_data.push_back(mixedEpoch);
						nObsTime ++;
					}				
					i ++;
				}

				//3 整理文件头信息			
				mixedEditedObsFile.m_header.init(editedObsFile_bds.m_header);				
				sprintf(mixedEditedObsFile.m_header.szSatlliteSystem, "M (MIXED)           ");
				mixedEditedObsFile.m_header.tmStart = t_start;
				mixedEditedObsFile.m_header.tmEnd   = t_end;
				mixedEditedObsFile.m_header.byObsTypes = 7;
				mixedEditedObsFile.m_header.pbyObsTypeList.clear();
				mixedEditedObsFile.m_header.pbyObsTypeList.resize(mixedEditedObsFile.m_header.byObsTypes);
				for(int i_obs = 0; i_obs < 7; i_obs ++)
					mixedEditedObsFile.m_header.pbyObsTypeList[i_obs] = i_obs + 1;//仅保留三个频点的伪码和相位观测数据，且对GPS而言，区分C1和P1 昌虓
			}
		}

		mixedEditedObsFile.write(strEditedMixObsFileName);
		return true;
	}

    // 子程序名称： multiGNSS2SingleSysEpochList   
	// 功能：根据混合格式的mixedEditedObsEpochlist , 获得单系统的editedObsEpochlist
	// 变量类型：mixedEditedObsEpochlist  : 混合系统格式数据
	//           editedObsEpochlist       : 单系统数据
	//           cSystem 		          : 导航系统标识(默认 'G'- GPS 系统)
	// 输入：mixedEditedObsEpochlist,cSystem 
	// 输出：editedObsEpochlist 
	// 其它： 
	// 语言：C++
	// 创建者：昌虓
	// 创建时间：2017/4/1
	// 版本时间：2017/4/1
	// 修改记录：
	// 备注：
	bool Rinex2_1_MixedEditedObsFile::multiGNSS2SingleSysEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& mixedEditedObsEpochlist, vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist, char cSystem)
	{
		if(mixedEditedObsEpochlist.size() <= 0)
			return false;
		editedObsEpochlist.clear();
		for(size_t s_i = 0; s_i < mixedEditedObsEpochlist.size(); s_i ++)
		{
			Rinex2_1_MixedEditedObsEpoch   obsEpoch;
			obsEpoch.t                       = mixedEditedObsEpochlist[s_i].t;
			obsEpoch.byEpochFlag             = mixedEditedObsEpochlist[s_i].byEpochFlag;
			obsEpoch.clock                   = mixedEditedObsEpochlist[s_i].clock;
			obsEpoch.tropZenithDelayPriori_H = mixedEditedObsEpochlist[s_i].tropZenithDelayPriori_H;
			obsEpoch.tropZenithDelayPriori_W = mixedEditedObsEpochlist[s_i].tropZenithDelayPriori_W;
			obsEpoch.tropZenithDelayEstimate = mixedEditedObsEpochlist[s_i].tropZenithDelayEstimate;
			obsEpoch.temperature             = mixedEditedObsEpochlist[s_i].temperature;
			obsEpoch.humidity                = mixedEditedObsEpochlist[s_i].humidity;
			obsEpoch.pressure                = mixedEditedObsEpochlist[s_i].pressure;
			for(Rinex2_1_MixedEditedObsSatMap::iterator it = mixedEditedObsEpochlist[s_i].editedObs.begin();it != mixedEditedObsEpochlist[s_i].editedObs.end(); ++it)
			{
				if(it->first.find(cSystem) != -1)
				{
					Rinex2_1_MixedEditedObsLine   obsLine;
					obsLine.satName       = it->first;//此处和星载融合程序不同，星载的卫星仍是以BYTE形式的Id存储
					obsLine.Azimuth       = it->second.Azimuth;
					obsLine.Elevation     = it->second.Elevation;
					obsLine.nObsTime      = it->second.nObsTime;
					obsLine.obsTypeList   = it->second.obsTypeList;
					obsLine.ReservedField = it->second.ReservedField;
					obsEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(it->first,obsLine));
				}
			}
			if(obsEpoch.editedObs.size() > 0)
			{
				obsEpoch.bySatCount = int(obsEpoch.editedObs.size());
				editedObsEpochlist.push_back(obsEpoch);
			}
		}
		if(editedObsEpochlist.size() <= 0)
			return false;
		else
			return true;
	}

	bool Rinex2_1_MixedEditedObsFile::getEditedObsEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist, char cSys)
	{
		if(m_data.size() <= 0)
			return false;
		editedObsEpochlist.clear();
		for(size_t s_i = 0; s_i < m_data.size(); s_i ++)
		{
			Rinex2_1_MixedEditedObsEpoch   obsEpoch;
			obsEpoch.t                       = m_data[s_i].t;
			obsEpoch.byEpochFlag             = m_data[s_i].byEpochFlag;
			obsEpoch.clock                   = m_data[s_i].clock;
			obsEpoch.tropZenithDelayPriori_H = m_data[s_i].tropZenithDelayPriori_H;
			obsEpoch.tropZenithDelayPriori_W = m_data[s_i].tropZenithDelayPriori_W;
			obsEpoch.tropZenithDelayEstimate = m_data[s_i].tropZenithDelayEstimate;
			obsEpoch.temperature             = m_data[s_i].temperature;
			obsEpoch.humidity                = m_data[s_i].humidity;
			obsEpoch.pressure                = m_data[s_i].pressure;
			for(Rinex2_1_MixedEditedObsSatMap::iterator it = m_data[s_i].editedObs.begin();it != m_data[s_i].editedObs.end(); ++it)
			{
				if(it->first.find(cSys) != -1)
				{
					Rinex2_1_MixedEditedObsLine   obsLine;
					obsLine.satName       = it->first;//此处和星载融合程序不同，星载的卫星仍是以BYTE形式的Id存储
					obsLine.Azimuth       = it->second.Azimuth;
					obsLine.Elevation     = it->second.Elevation;
					obsLine.nObsTime      = it->second.nObsTime;
					obsLine.obsTypeList   = it->second.obsTypeList;
					obsLine.ReservedField = it->second.ReservedField;
					obsEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(it->first,obsLine));
				}
			}
			if(obsEpoch.editedObs.size() > 0)
			{
				obsEpoch.bySatCount = int(obsEpoch.editedObs.size());
				editedObsEpochlist.push_back(obsEpoch);
			}
		}
		if(editedObsEpochlist.size() <= 0)
			return false;
		else
			return true;
	}
}
