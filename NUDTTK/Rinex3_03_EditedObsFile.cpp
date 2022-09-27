#include "Rinex3_03_EditedObsFile.hpp"

namespace NUDTTK
{
	Rinex3_03_EditedObsFile::Rinex3_03_EditedObsFile(void)
	{
	}

	Rinex3_03_EditedObsFile::~Rinex3_03_EditedObsFile(void)
	{
	}

	void Rinex3_03_EditedObsFile::clear()
	{
		m_header = Rinex3_03_ObsHeader::Rinex3_03_ObsHeader();
		m_data.clear();
	}

	bool Rinex3_03_EditedObsFile::isEmpty()
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
	// 创建者：邵凯
	// 创建时间：2018/5/31
	// 修改记录：
	// 其它：    参考 Rinex2_1_EditedObsFile.cpp 文件和 RINEX 3.03 文件
	bool Rinex3_03_EditedObsFile::cutdata(DayTime t_Begin, DayTime t_End)
	{
		// 确保 t_Begin <= t_End
		if( t_Begin - t_End >= 0 || t_End - m_header.tmStart <= 0 || t_Begin - m_header.tmEnd > 0 )
			return false;
		vector<Rinex3_03_EditedObsEpoch> obsDataList;
		obsDataList.clear();
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			vector<Rinex3_03_SysEditedObs> sysEditedObsList;
		    sysEditedObsList.clear();
			Rinex3_03_EditedObsEpoch ObsEpoch = m_data[s_i];
			if(ObsEpoch.t - t_Begin >= 0 && ObsEpoch.t - t_End < 0)
			{
				for(size_t s_t = 0; s_t < ObsEpoch.editedObs.size(); s_t++)
				{
					Rinex3_03_SysEditedObs SysEditedObs = ObsEpoch.editedObs[s_t];
					for(Rinex3_03_EditedObsSatMap::iterator it = SysEditedObs.obsList.begin(); it != SysEditedObs.obsList.end(); ++it)
					{
						it->second.nObsTime = int(sysEditedObsList.size());
					}
					sysEditedObsList.push_back(SysEditedObs);
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
	// 创建者：邵凯
	// 创建时间：2018/5/31
	// 修改记录：
	// 其它：    参考 Rinex2_1_MixedEditedObsFile.cpp 文件和 RINEX 3.03 文件
	int  Rinex3_03_EditedObsFile::isValidEpochLine(string strLine, FILE * pEditedObsfile)
	{
		// 下面几种数据用int型, 避免因为 strLine 的格式问题引起 sscanf 函数发生错误
		DayTime tmEpoch;
		int byEpochFlag   =  1;     // 0: ok,  1: power failure between previous and current epoch  > 1: Event flag (2-5) 
		int bySatCount = -1;        // 本时刻可视卫星(或测站)个数
		char szRecordIdentifier[1 +1 ];
		if(pEditedObsfile != NULL)  // 判断是否为文件末尾
		{
			if(feof(pEditedObsfile))
				return 0;
		}
		sscanf(strLine.c_str(), "%1c%*1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%11lf%*2c%1d%3d",
								&szRecordIdentifier,                    
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
	// 输入：strEditedObsfileName
	// 输出：
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2018/5/31
	// 修改记录：
	// 其它：    参考 Rinex2_1_MixedEditedObsFile.cpp 文件和 RINEX 3.03 文件
	bool Rinex3_03_EditedObsFile::open(string strEditedObsfileName)
	{
		FILE * pEditedObsfile = fopen(strEditedObsfileName.c_str(), "r+t");
		if(pEditedObsfile == NULL) 
			return false;
		m_header = Rinex3_03_ObsHeader::Rinex3_03_ObsHeader();
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

			if(strLineMask == Rinex3_03_MaskString::szVerType)
			{
				//sscanf(line,"%9lf%*11c%1c%*19c%1c",&m_header.rinexVersion, &m_header.szFileType, &m_header.cSatSys);
				sscanf(line,"%10c%*10c%1c%*19c%1c",&m_header.rinexVersion,&m_header.szFileType,&m_header.cSatSys);
				m_header.rinexVersion[10]='\0';
				m_header.szFileType[1]='\0';
			}	
			else if(strLineMask == Rinex3_03_MaskString::szPgmRunDate)
			{
				strLine.copy(m_header.szProgramName, 20, 0);
				strLine.copy(m_header.szProgramAgencyName, 20, 20);
				strLine.copy(m_header.szFileDate, 20, 40);
			}
			else if(strLineMask == Rinex3_03_MaskString::szObservAgency)
			{
				strLine.copy(m_header.szObserverName, 20, 0);
				strLine.copy(m_header.szObserverAgencyName, 40, 20);
			}
			else if(strLineMask == Rinex3_03_MaskString::szMarkerName)
			{
				strLine.copy(m_header.szMarkName, 60, 0);
			}
			else if(strLineMask == Rinex3_03_MaskString::szMarkerNum)
			{
				strLine.copy(m_header.szMarkNumber, 20, 0);
			}
			else if(strLineMask == Rinex3_03_MaskString::szRecTypeVers)
			{
				strLine.copy(m_header.szRecNumber, 20, 0);
				strLine.copy(m_header.szRecType, 20, 20);
				strLine.copy(m_header.szRecVersion, 20, 40);
			}
			else if(strLineMask == Rinex3_03_MaskString::szAntType)
			{
				strLine.copy(m_header.szAntNumber, 20, 0);
				strLine.copy(m_header.szAntType, 20, 20);
			}
			else if(strLineMask == Rinex3_03_MaskString::szApproxPosXYZ)
			{  
				sscanf(line,"%14lf%14lf%14lf",
					        &m_header.approxPosXYZ.x,
							&m_header.approxPosXYZ.y,
							&m_header.approxPosXYZ.z);
			}
			else if(strLineMask == Rinex3_03_MaskString::szAntDeltaHEN)
			{  
				sscanf(line,"%14lf%14lf%14lf",
					        &m_header.antDeltaHEN.x,
							&m_header.antDeltaHEN.y,
							&m_header.antDeltaHEN.z);
			}
			else if(strLineMask == Rinex3_03_MaskString::szTmOfFirstObs)
			{  
				sscanf(line,"%6d%6d%6d%6d%6d%12lf",
					        &m_header.tmStart.year,  
                            &m_header.tmStart.month,
                            &m_header.tmStart.day,   
                            &m_header.tmStart.hour,
                            &m_header.tmStart.minute,
                            &m_header.tmStart.second);
				strLine.copy(m_header.szTimeSystem, 3, 48);
			}
			else if(strLineMask == Rinex3_03_MaskString::szTmOfLastObs)
			{  
				sscanf(line,"%6d%6d%6d%6d%6d%12lf",
					        &m_header.tmEnd.year,	 
							&m_header.tmEnd.month,
						    &m_header.tmEnd.day,	 
							&m_header.tmEnd.hour,
							&m_header.tmEnd.minute,  
							&m_header.tmEnd.second);
			}
			else if(strLineMask == Rinex3_03_MaskString::szInterval)
			{  
				sscanf(line, "%10lf", &m_header.interval);
			}
			else if(strLineMask == Rinex3_03_MaskString::szLeapSec)
			{  
				sscanf(line, "%6d", &m_header.leapSecond);
			}
			else if(strLineMask == Rinex3_03_MaskString::szNumsOfSat)
			{  
				sscanf(line, "%6d", &m_header.satCount);
			}
			else if(strLineMask == Rinex3_03_MaskString::szSysObsTypes)
			{  

				Rinex3_03_SysObsType         sysObsTyp;									
				sscanf(line,"%1c%*2c%3d", &sysObsTyp.cSatSys, &sysObsTyp.obsTypeCount);
				int         nline    = sysObsTyp.obsTypeCount / 13;	// 整行数
				int         nResidue = sysObsTyp.obsTypeCount % 13;
				if(sysObsTyp.obsTypeCount <= 13)
				{
					for(BYTE i = 0; i < sysObsTyp.obsTypeCount; i++)
					{
						char strObsType[3 + 1];
						strLine.copy(strObsType, 3, 6 + i * 4 + 1);
						strObsType[3] = '\0';
						sysObsTyp.obsTypeList.push_back(strObsType);
					}					
				}
				else
				{
					for(BYTE i = 0; i < 13; i++)
					{
						char strObsType[3 + 1];
						strLine.copy(strObsType, 3, 6 + i * 4 + 1);
						strObsType[3] = '\0';
						sysObsTyp.obsTypeList.push_back(strObsType);
					}
					for(int n = 1; n < nline;n++)
					{// 读取中间的整行
						fgets(line,100,pEditedObsfile);
						strLine = line;
						for(BYTE i = 0; i < 13; i++)
						{
							char strObsType[3 + 1];
							strLine.copy(strObsType, 3, 6 + i * 4 + 1);
							strObsType[3] = '\0';
							sysObsTyp.obsTypeList.push_back(strObsType);
						}						
					}
					if(nResidue > 0)
					{
						fgets(line,100,pEditedObsfile);
						strLine = line;
						for(BYTE i = 0; i < nResidue; i++)
						{
							char strObsType[3 + 1];
							strLine.copy(strObsType, 3, 6 + i * 4 + 1);
							strObsType[3] = '\0';
							sysObsTyp.obsTypeList.push_back(strObsType);
						}		
					}
				}
				m_header.sysObsTypeList.push_back(sysObsTyp);
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
		char line[1000];
		fgets(line, 1000, pEditedObsfile);
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
				Rinex3_03_EditedObsEpoch editedObsEpoch;
				editedObsEpoch.editedObs.clear();
				editedObsEpoch.editedObs.resize(m_header.sysObsTypeList.size());          //观测数据向量初始化
				for(size_t s_i = 0; s_i < m_header.sysObsTypeList.size(); s_i ++)
				{
					editedObsEpoch.editedObs[s_i].cSatSys = m_header.sysObsTypeList[s_i].cSatSys;
				}
				// 解析Epoch/SAT
				sscanf(strLine.c_str(),"%1c%*1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%11lf%*2c%1d%3d%*6c%15lf%14lf%14lf%14lf%14lf%14lf%14lf%14lf%14lf",
									   &editedObsEpoch.cRecordId[0],
									   &editedObsEpoch.t.year,
									   &editedObsEpoch.t.month,
									   &editedObsEpoch.t.day,
									   &editedObsEpoch.t.hour,
									   &editedObsEpoch.t.minute,
									   &editedObsEpoch.t.second,
									   &editedObsEpoch.byEpochFlag,
									   &editedObsEpoch.satCount,
									   &editedObsEpoch.clock,
									   &editedObsEpoch.tropZenithDelayPriori_H,
									   &editedObsEpoch.tropZenithDelayPriori_W, 
									   &editedObsEpoch.tropZenithDelayEst,
									   &editedObsEpoch.tropGradNSEst,  // +
									   &editedObsEpoch.tropGradEWEst,  // +
									   &editedObsEpoch.temperature,
									   &editedObsEpoch.humidity,
									   &editedObsEpoch.pressure);
				editedObsEpoch.cRecordId[1] = '\0';
				// 根据Epoch/SAT，解析可见观测数据
				for(int i = 0; i < editedObsEpoch.satCount; i++)
				{
					char                  cSatSys;              // 卫星系统
					char                  szSatName[4];        // 存储卫星系统 + PRN (例如'G01')
					fgets(line, 1000, pEditedObsfile);
					string strLine = line;
                    // 行结构体定义
					Rinex3_03_EditedObsLine editedObsLine;
					editedObsLine.nObsTime = k - 1; // 2007/07/22 添加

					sscanf(line,"%1c",&cSatSys);
					sscanf(line,"%3c%10f%10f%10f%10f%*1c%1d%*1c",
						        &szSatName,
                                &editedObsLine.Elevation,
                                &editedObsLine.Azimuth,
								&editedObsLine.gmfh,    // +
								&editedObsLine.gmfw,    // +
								&editedObsLine.mark_GNSSSatShadow); // +
					szSatName[3] = '\0';
					if(szSatName[1] == ' ')// 2014/03/22, 将G 1->G01
						szSatName[1] = '0';
					editedObsLine.satName = szSatName;
					editedObsLine.obsTypeList.clear();
					size_t    i_sys;                       // 系统编号
					// 卫星系统对应的观测数据类型
					int obsTypCount;
					for(size_t s_i = 0; s_i < m_header.sysObsTypeList.size(); s_i ++)
					{
						if(m_header.sysObsTypeList[s_i].cSatSys == cSatSys)
						{
							i_sys = s_i;
							obsTypCount = m_header.sysObsTypeList[s_i].obsTypeCount;
							break;
						}
					}
					// 根据观测数据类型个数 obsTypCount
					for(int j = 0; j < obsTypCount; j++)
					{
						Rinex2_1_EditedObsDatum editedObsDatum;
						char strEditedObsDatum[18];
						strLine.copy(strEditedObsDatum, 17, 46 + j * 17);
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
					editedObsEpoch.editedObs[i_sys].obsList.insert(Rinex3_03_EditedObsSatMap::value_type(szSatName,editedObsLine));
				}
				m_data.push_back(editedObsEpoch);
				fgets(line, 1000, pEditedObsfile);
			}
			else  // 无效数据行, 比如空白行, 掠过不作处理
			{   
				fgets(line, 1000, pEditedObsfile);
			}
		}
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
	// 创建时间：2018/5/29
	// 版本时间：
	// 修改记录：
	// 备注：  参考 Rinex2_1_MixedEditedObsFile.cpp 文件和 RINEX 3.03 文件
	bool Rinex3_03_EditedObsFile::write(string strEditedObsfileName)
	{
		if(m_data.size() <= 0)
			return false;
		FILE* pEditedfile = fopen(strEditedObsfileName.c_str(), "w+");
		// 写头文件，参考 Rinex3.03文件
		//fprintf(pEditedfile,"%9.2lf%-11s%-20s%1c%-19s%20s\n", 
		//	                m_header.rinexVersion,
		//	                " ",
		//	                m_header.szFileType,
		//	                m_header.cSatSys,
		//	                " ",
		//	                Rinex3_03_MaskString::szVerType);
		fprintf(pEditedfile,"%10s%-10s%-20s%1c%-19s%20s\n", 
			                m_header.rinexVersion,
			                " ",
			                m_header.szFileType,
			                m_header.cSatSys,
			                " ",
			                Rinex3_03_MaskString::szVerType);
		fprintf(pEditedfile,"%-20s%-20s%-20s%20s\n",
			                m_header.szProgramName, 
							m_header.szProgramAgencyName, 
							m_header.szFileDate,
							Rinex3_03_MaskString::szPgmRunDate);
		fprintf(pEditedfile,"%60s%20s\n",
			                m_header.szMarkName,
							Rinex3_03_MaskString::szMarkerName);
		fprintf(pEditedfile,"%-20s%-40s%20s\n",
			                m_header.szObserverName, 
							m_header.szObserverAgencyName, 
							Rinex3_03_MaskString::szObservAgency);
		fprintf(pEditedfile,"%20s%20s%20s%20s\n",
			                m_header.szRecNumber,
							m_header.szRecType,
							m_header.szRecVersion,
							Rinex3_03_MaskString::szRecTypeVers);
		fprintf(pEditedfile,"%20s%20s%-20s%20s\n",
			                m_header.szAntNumber,
							m_header.szAntType,
							" ",
							Rinex3_03_MaskString::szAntType);
		fprintf(pEditedfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
			                m_header.approxPosXYZ.x,
							m_header.approxPosXYZ.y,
							m_header.approxPosXYZ.z,
							" ",
							Rinex3_03_MaskString::szApproxPosXYZ);
		fprintf(pEditedfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
			                m_header.antDeltaHEN.x,
							m_header.antDeltaHEN.y,
							m_header.antDeltaHEN.z,
							" ",
							Rinex3_03_MaskString::szAntDeltaHEN);
		for(size_t s_i = 0; s_i < m_header.sysObsTypeList.size(); s_i ++)
		{
			int obsTypes = (int)m_header.sysObsTypeList[s_i].obsTypeList.size();
			int nLine    = obsTypes / 13;  // 整行数
		    int nResidue = obsTypes % 13;  // 余数
			int nBlank   = 0;              // 空白位数
			string         strBlank; 
			fprintf(pEditedfile,"%1c%-2s%3d",                         
				    m_header.sysObsTypeList[s_i].cSatSys,                          
				    " ",                                                
				    obsTypes);
			if(obsTypes <= 13)
			{
				nBlank = 60 - (6 + 4 * obsTypes);
				strBlank.append(nBlank,' ');				
				for(int i = 0;i < obsTypes;i ++)
					fprintf(pEditedfile,"%-1s%3s"," ",m_header.sysObsTypeList[s_i].obsTypeList[i].c_str());
				fprintf(pEditedfile,"%s%20s\n", strBlank.c_str(), Rinex3_03_MaskString::szSysObsTypes);
			}
			else
			{
				nBlank = 60 - (6 + 4 * nResidue);
				strBlank.append(nBlank,' ');
				for(int i = 0; i < 13; i ++)
					fprintf(pEditedfile,"%-1s%3s"," ",m_header.sysObsTypeList[s_i].obsTypeList[i].c_str());
				fprintf(pEditedfile,"%-2s%20s\n"," ",Rinex3_03_MaskString::szSysObsTypes);
				for(int n = 1; n < nLine; n ++) 
				{
					fprintf(pEditedfile,"%6s"," ");
					for(int j = 0; j < 13; j ++)
						fprintf(pEditedfile,"%-1s%3s"," ",m_header.sysObsTypeList[s_i].obsTypeList[13 * n + j].c_str());
					fprintf(pEditedfile,"%20s\n",Rinex3_03_MaskString::szSysObsTypes);
				}
				if(nResidue > 0)
				{
					fprintf(pEditedfile,"%-6s"," ");
					for(int i = 0; i < nResidue; i ++)
						fprintf(pEditedfile,"%-1s%3s"," ",m_header.sysObsTypeList[s_i].obsTypeList[13 * nLine + i].c_str());
				}
				fprintf(pEditedfile,"%s%20s\n", strBlank.c_str(), Rinex3_03_MaskString::szSysObsTypes);
			}			
		}
		fprintf(pEditedfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
			                m_header.tmStart.year,  
							m_header.tmStart.month,
							m_header.tmStart.day,   
							m_header.tmStart.hour,
							m_header.tmStart.minute,
							m_header.tmStart.second,
							" ",
							m_header.szTimeSystem,
							" ",
							Rinex3_03_MaskString::szTmOfFirstObs);
		fprintf(pEditedfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
			                m_header.tmEnd.year,  
							m_header.tmEnd.month,
							m_header.tmEnd.day,   
							m_header.tmEnd.hour,
							m_header.tmEnd.minute,
							m_header.tmEnd.second,
							" ",
							m_header.szTimeSystem,
							" ",
							Rinex3_03_MaskString::szTmOfLastObs);
		if(m_header.interval != DBL_MAX)
			fprintf(pEditedfile,"%10.3f%-50s%20s\n",
			                    m_header.interval, 
								" ",
								Rinex3_03_MaskString::szInterval);
		if(m_header.leapSecond != INT_MAX)
			fprintf(pEditedfile,"%6d%-54s%20s\n",
			                    m_header.leapSecond,
								" ",
								Rinex3_03_MaskString::szLeapSec);
		if(m_header.satCount != INT_MAX)
			fprintf(pEditedfile,"%6d%54s%20s\n",
					m_header.satCount,
					" ",			
					Rinex3_03_MaskString::szNumsOfSat);
		fprintf(pEditedfile,"%-60s%20s\n",
			                " ",
							Rinex3_03_MaskString::szEndOfHead);
	
		// 编辑后观测数据
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			fprintf(pEditedfile, "%1c %4d %02d %02d %02d %02d%11.7lf  %1d%3d      ",
				                 m_data[s_i].cRecordId[0], // 3.03格式
								 m_data[s_i].t.year,
								 m_data[s_i].t.month,
								 m_data[s_i].t.day,
								 m_data[s_i].t.hour,
								 m_data[s_i].t.minute,
								 m_data[s_i].t.second,
								 m_data[s_i].byEpochFlag,
								 m_data[s_i].satCount);	
			if( fabs(m_data[s_i].clock) >= 1.0 
			 || fabs(m_data[s_i].tropZenithDelayPriori_H) >= 1.0E+9
			 || fabs(m_data[s_i].tropZenithDelayPriori_W) >= 1.0E+9
			 || fabs(m_data[s_i].tropZenithDelayEst) >= 1.0E+9
			 || fabs(m_data[s_i].tropGradNSEst) >= 1.0E+9
			 || fabs(m_data[s_i].tropGradEWEst) >= 1.0E+9
			 || fabs(m_data[s_i].temperature) >= 1.0E+9
			 || fabs(m_data[s_i].humidity) >= 1.0E+9
			 || fabs(m_data[s_i].pressure) >= 1.0E+9)
			{
				fprintf(pEditedfile, "%15.8E%14.6E%14.6E%14.6E%14.6E%14.6E%14.3E%14.3E%14.3E\n",
									 m_data[s_i].clock,
									 m_data[s_i].tropZenithDelayPriori_H,
									 m_data[s_i].tropZenithDelayPriori_W,
									 m_data[s_i].tropZenithDelayEst,
									 m_data[s_i].tropGradNSEst, // +
									 m_data[s_i].tropGradEWEst, // +
									 m_data[s_i].temperature,
									 m_data[s_i].humidity,
									 m_data[s_i].pressure);	
			}
			else
			{
				fprintf(pEditedfile, "%15.12lf%14.6lf%14.6lf%14.6lf%14.6lf%14.6lf%14.3lf%14.3lf%14.3lf\n",
									 m_data[s_i].clock,
									 m_data[s_i].tropZenithDelayPriori_H,
									 m_data[s_i].tropZenithDelayPriori_W,
									 m_data[s_i].tropZenithDelayEst,
									 m_data[s_i].tropGradNSEst, // +
									 m_data[s_i].tropGradEWEst, // +
									 m_data[s_i].temperature,
									 m_data[s_i].humidity,
									 m_data[s_i].pressure);
			}
			for(size_t s_j = 0; s_j < m_data[s_i].editedObs.size(); s_j ++)
			{				
				for(Rinex3_03_EditedObsSatMap::iterator it = m_data[s_i].editedObs[s_j].obsList.begin(); it != m_data[s_i].editedObs[s_j].obsList.end(); ++it)
				{
					Rinex3_03_EditedObsLine editedObsLine = it->second;
					if( fabs(editedObsLine.gmfh) >= 1.0E+3
					 || fabs(editedObsLine.gmfw) >= 1.0E+3)
					{
						fprintf(pEditedfile, "%3s%10.3f%10.3f%10.3E%10.3E %1d ",
											 editedObsLine.satName.c_str(),
											 editedObsLine.Elevation,
											 editedObsLine.Azimuth,
											 editedObsLine.gmfh, // +
											 editedObsLine.gmfw, // +
											 editedObsLine.mark_GNSSSatShadow); // +
					}
					else
					{
						fprintf(pEditedfile, "%3s%10.3f%10.3f%10.3f%10.3f %1d ",
											 editedObsLine.satName.c_str(),
											 editedObsLine.Elevation,
											 editedObsLine.Azimuth,
											 editedObsLine.gmfh, // +
											 editedObsLine.gmfw, // +
											 editedObsLine.mark_GNSSSatShadow); // +
					}
					for(size_t s_k = 0; s_k < editedObsLine.obsTypeList.size(); s_k++)
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
												editedObsLine.obsTypeList[s_k].byEditedMark2);
					}
					fprintf(pEditedfile, "\n");
				}
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
	// 修改记录：对应 Rinex2_1_EditedObsFile.cpp 进行修改，2018/5/31
	bool Rinex3_03_EditedObsFile::getEditedObsEpochList(vector<Rinex3_03_EditedObsEpoch>& editedObsEpochlist)
	{
		if(isEmpty())
			return false;
		editedObsEpochlist = m_data;
		return true;
	}

	// 子程序名称： getEditedObsEpochList   
	// 功能：根据编辑后文件内容，获得 editedObsEpochlist
	// 变量类型：editedObsEpochlist   : 预处理后数据
	// 输入：cSatSys : 系统名
	// 输出：editedObsEpochlist 
	// 其它：返回该系统在观测数据中的位置
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/9/9
	// 版本时间：2012/4/11
	// 修改记录：对应 Rinex2_1_EditedObsFile.cpp 进行修改，2018/5/31
	// 其它：依赖 m_data
	int Rinex3_03_EditedObsFile::getEditedObsEpochList(vector<Rinex3_03_EditedObsEpoch>& editedObsEpochlist, char cSatSys)
	{
		if(isEmpty())
			return false;
		editedObsEpochlist.clear();
		// 首先查找卫星系统在每个历元 editedObs 中对应的序号
		int index = -1;
		for(size_t s_i = 0; s_i < m_data[0].editedObs.size(); s_i ++)
		{
			if(m_data[0].editedObs[s_i].cSatSys == cSatSys)
			{
				index = int(s_i);
			}
		}
		// 其次获得该系统的 editedObsEpochlist
		for(size_t s_i = 0; s_i < m_data.size(); s_i ++)
		{// 历元
			Rinex3_03_EditedObsEpoch editedObsEpoch;
			editedObsEpoch.t            = m_data[s_i].t;
			editedObsEpoch.byEpochFlag  = m_data[s_i].byEpochFlag;
			editedObsEpoch.clock        = m_data[s_i].clock;
			editedObsEpoch.cRecordId[0] = m_data[s_i].cRecordId[0];
			editedObsEpoch.cRecordId[1] = '\0';
			editedObsEpoch.tropZenithDelayPriori_H = m_data[s_i].tropZenithDelayPriori_H;
			editedObsEpoch.tropZenithDelayPriori_W = m_data[s_i].tropZenithDelayPriori_W;
			editedObsEpoch.tropZenithDelayEst      = m_data[s_i].tropZenithDelayEst;
			editedObsEpoch.tropGradNSEst           = m_data[s_i].tropGradNSEst;
			editedObsEpoch.tropGradEWEst           = m_data[s_i].tropGradEWEst;
			editedObsEpoch.temperature             = m_data[s_i].temperature;
			editedObsEpoch.humidity                = m_data[s_i].humidity;
			editedObsEpoch.pressure                = m_data[s_i].pressure;
			editedObsEpoch.satCount                = 0;
			editedObsEpoch.editedObs.clear();
			for(size_t i_sys = 0; i_sys < m_data[s_i].editedObs.size(); i_sys ++)
			{// 系统
				if(m_data[s_i].editedObs[i_sys].cSatSys == cSatSys)
				{
					editedObsEpoch.editedObs.push_back(m_data[s_i].editedObs[i_sys]);
					editedObsEpoch.satCount = int(m_data[s_i].editedObs[i_sys].obsList.size());
				}
			}
			editedObsEpochlist.push_back(editedObsEpoch);	
		}
		if(editedObsEpochlist.size() <= 0)
			return -1;
		return index;
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
	// 修改记录：对应 Rinex2_1_MixedEditedObsFile.cpp 进行修改，2018/5/31
	bool Rinex3_03_EditedObsFile::getEditedObsEpochList(vector<Rinex3_03_EditedObsEpoch>& editedObsEpochlist, DayTime t_Begin, DayTime t_End)
	{
		if(isEmpty())
			return false;
		editedObsEpochlist.clear();
		int k = 0;
		for(size_t i = 0; i < m_data.size(); i++)
		{
			vector<Rinex3_03_SysEditedObs> sysEditedObsList;
		    sysEditedObsList.clear();
			Rinex3_03_EditedObsEpoch editedObsEpoch = m_data[i];
			if(editedObsEpoch.t - t_Begin < 0 )      // 确保 editedObsEpoch.T >= T_Begin
				continue;
			else if(editedObsEpoch.t - t_End >= 0 )  // 确保 editedObsEpoch.T <  T_End
				break;
			else                                     // T_Begin =< editedObsEpoch.T < T_End
			{				
				for(size_t s_t = 0; s_t < editedObsEpoch.editedObs.size(); s_t++)
				{
					Rinex3_03_SysEditedObs sysEditedObs = editedObsEpoch.editedObs[s_t];					
					for(Rinex3_03_EditedObsSatMap::iterator it = sysEditedObs.obsList.begin(); it != sysEditedObs.obsList.end(); ++it)
					{
						it->second.nObsTime = int(sysEditedObsList.size());
					}
				    sysEditedObsList.push_back(sysEditedObs);
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
	// 修改记录：对应 Rinex2_1_MixedEditedObsFile.cpp 进行修改，2018/5/31
	// 备注：
	bool Rinex3_03_EditedObsFile::getEditedObsSatList(vector<Rinex3_03_EditedObsSat>& editedObsSatlist)
	{
		if(isEmpty())
			return false;
		map<string, Rinex3_03_EditedObsSat> mapEditedObsSatlist;
		mapEditedObsSatlist.clear();
		// 遍历每个历元的观测数据
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			// 在每个历元，遍历每个卫星系统
			for(size_t s_t = 0; s_t < m_data[s_i].editedObs.size();s_t++)
			{
				// 在每个卫星系统，遍历每颗卫星的数据
				for(Rinex3_03_EditedObsSatMap::iterator it = m_data[s_i].editedObs[s_t].obsList.begin(); it != m_data[s_i].editedObs[s_t].obsList.end(); ++it)
				{
					if(mapEditedObsSatlist.find(it->first) != mapEditedObsSatlist.end())
						mapEditedObsSatlist[it->first].editedObs.insert(Rinex3_03_EditedObsEpochMap::value_type(m_data[s_i].t, it->second));
					else
					{// 发现新的卫星的数据
						Rinex3_03_EditedObsSat editedObsSat;
						editedObsSat.satName = it->first;
						editedObsSat.editedObs.clear();
						editedObsSat.editedObs.insert(Rinex3_03_EditedObsEpochMap::value_type(m_data[s_i].t, it->second));
						mapEditedObsSatlist.insert(map<string, Rinex3_03_EditedObsSat>::value_type(it->first, editedObsSat));
					}
				}
			}

		}
		// 整理所有可视卫星数据
		editedObsSatlist.clear();
		for(map<string, Rinex3_03_EditedObsSat>::iterator it = mapEditedObsSatlist.begin(); it != mapEditedObsSatlist.end(); ++it)
		{
			if(it->second.editedObs.size() > 0)
				editedObsSatlist.push_back(it->second);
		}
		return true;
	}
	bool Rinex3_03_EditedObsFile::getEditedObsSatList(vector<Rinex3_03_EditedObsSat>& editedObsSatlist,char cSatSys)
	{
		if(isEmpty())
			return false;
		map<string, Rinex3_03_EditedObsSat> mapEditedObsSatlist;
		mapEditedObsSatlist.clear();
		// 遍历每个历元的观测数据
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			// 在每个历元，遍历每个卫星系统
			for(size_t s_t = 0; s_t < m_data[s_i].editedObs.size();s_t++)
			{
				// 查找对应的卫星系统
				if(m_data[s_i].editedObs[s_t].cSatSys == cSatSys)
				{
					for(Rinex3_03_EditedObsSatMap::iterator it = m_data[s_i].editedObs[s_t].obsList.begin(); it != m_data[s_i].editedObs[s_t].obsList.end(); ++it)
					{
						if(mapEditedObsSatlist.find(it->first) != mapEditedObsSatlist.end())
							mapEditedObsSatlist[it->first].editedObs.insert(Rinex3_03_EditedObsEpochMap::value_type(m_data[s_i].t, it->second));
						else
						{// 发现新的卫星的数据
							Rinex3_03_EditedObsSat editedObsSat;
							editedObsSat.satName = it->first;
							editedObsSat.editedObs.clear();
							editedObsSat.editedObs.insert(Rinex3_03_EditedObsEpochMap::value_type(m_data[s_i].t, it->second));
							mapEditedObsSatlist.insert(map<string, Rinex3_03_EditedObsSat>::value_type(it->first, editedObsSat));
						}
					}
				}
			}

		}
		// 整理所有可视卫星数据
		editedObsSatlist.clear();
		for(map<string, Rinex3_03_EditedObsSat>::iterator it = mapEditedObsSatlist.begin(); it != mapEditedObsSatlist.end(); ++it)
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
	// 修改记录：对应 Rinex2_1_MixedEditedObsFile.cpp 进行修改，2018/5/31
	// 备注：
	bool Rinex3_03_EditedObsFile::getEditedObsSatList(vector<Rinex3_03_EditedObsSat>& editedObsSatlist,DayTime t_Begin, DayTime t_End)
	{
		vector<Rinex3_03_EditedObsEpoch> editedObsEpochlist;
		if(!getEditedObsEpochList(editedObsEpochlist, t_Begin, t_End))
			return false;
		return getEditedObsSatList(editedObsEpochlist, editedObsSatlist);
	}
	// 子程序名称： getEditedObsSatList   
	// 功能：根据 editedObsEpochlist , 获得 editedObsSatlist
	// 变量类型：editedObsEpochlist  : 预处理后数据
	//           editedObsSatlist    : 预处理后数据
	// 输入：editedObsEpochlist
	// 输出：editedObsSatlist 
	// 其它： 
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/9/9
	// 版本时间：2012/4/11
	// 修改记录：对应 Rinex2_1_MixedEditedObsFile.cpp 进行修改，2018/5/31
	// 备注：
	bool Rinex3_03_EditedObsFile::getEditedObsSatList(vector<Rinex3_03_EditedObsEpoch> editedObsEpochlist, vector<Rinex3_03_EditedObsSat>& editedObsSatlist)
	{
		if(editedObsEpochlist.size() <= 0)
			return false;
		map<string, Rinex3_03_EditedObsSat> mapEditedObsSatlist; 
		mapEditedObsSatlist.clear();
		// 遍历每个历元的观测数据 
		for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
		{
			// 在每个历元，遍历每个卫星系统
			for(size_t s_t = 0; s_t < editedObsEpochlist[s_i].editedObs.size();s_t++)
			{
				// 在每个卫星系统，遍历每颗卫星的数据
				for(Rinex3_03_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs[s_t].obsList.begin(); it != editedObsEpochlist[s_i].editedObs[s_t].obsList.end(); ++it)
				{
					if(mapEditedObsSatlist.find(it->first) != mapEditedObsSatlist.end())
						mapEditedObsSatlist[it->first].editedObs.insert(Rinex3_03_EditedObsEpochMap::value_type(editedObsEpochlist[s_i].t, it->second));
					else
					{// 发现新的卫星的数据
						Rinex3_03_EditedObsSat editedObsSat;
						editedObsSat.satName = it->first;
						editedObsSat.editedObs.clear();
						editedObsSat.editedObs.insert(Rinex3_03_EditedObsEpochMap::value_type(editedObsEpochlist[s_i].t, it->second));
						mapEditedObsSatlist.insert(map<string, Rinex3_03_EditedObsSat>::value_type(it->first, editedObsSat));
					}
				}
			}
		}
		// 整理所有可视卫星数据
		editedObsSatlist.clear();
		for(map<string, Rinex3_03_EditedObsSat>::iterator it = mapEditedObsSatlist.begin(); it != mapEditedObsSatlist.end(); ++it)
		{
			if(it->second.editedObs.size() > 0)
				editedObsSatlist.push_back(it->second);
		}
		return true;
	}

	// 子程序名称：datalist_sat2epoch   
	// 作用：将 editedObsSatlist 转换到 editedObsEpochlist, 为了保持时间的完整性, 
	//         转换期间以 m_obsFile.m_data[s_i].T 为参考
	// 类型：editedObsSatlist   : 数据结构格式1: 根据不同卫星(或测站)进行分类 , 称为editedObsSatlist, 主要用途方便时间序列处理
	//       editedObsEpochlist : 数据结构格式2: 根据不同历元时刻进行分类, 称为editedObsEpochlist, 主要用途方便单点定位
	// 输入：editedObsSatlist
	// 输出：editedObsEpochlist 		
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/5/8
	// 版本时间：2012/4/11
	// 修改记录：对应 Rinex2_1_MixedEditedObsFile.cpp 进行修改，2018/5/31，邵凯
	// 备注：
	// 其它：依赖 m_data
	bool Rinex3_03_EditedObsFile::datalist_sat2epoch(vector<Rinex3_03_EditedObsSat> editedObsSatlist, vector<Rinex3_03_EditedObsEpoch>& editedObsEpochlist)
	{
		if(editedObsSatlist.size() <= 0)
			return false;
		editedObsEpochlist.clear();
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			Rinex3_03_EditedObsEpoch editedObsEpoch;
			editedObsEpoch.cRecordId[0]            = m_data[s_i].cRecordId[0]; // +
			editedObsEpoch.cRecordId[1]            = '\0';                     // +
			editedObsEpoch.byEpochFlag             = m_data[s_i].byEpochFlag;  // +
            editedObsEpoch.byEpochFlag             = m_data[s_i].byEpochFlag;
			editedObsEpoch.satCount                = m_data[s_i].satCount;
			editedObsEpoch.clock                   = m_data[s_i].clock;
			editedObsEpoch.tropZenithDelayPriori_H = m_data[s_i].tropZenithDelayPriori_H;
			editedObsEpoch.tropZenithDelayPriori_W = m_data[s_i].tropZenithDelayPriori_W;
			editedObsEpoch.tropZenithDelayEst      = m_data[s_i].tropZenithDelayEst;
			editedObsEpoch.tropGradEWEst           = m_data[s_i].tropGradEWEst; // +
			editedObsEpoch.tropGradNSEst           = m_data[s_i].tropGradNSEst; // +
			editedObsEpoch.temperature             = m_data[s_i].temperature; 
			editedObsEpoch.humidity                = m_data[s_i].humidity;
			editedObsEpoch.pressure                = m_data[s_i].pressure;
			editedObsEpoch.t                       = m_data[s_i].t;
			editedObsEpoch.editedObs.clear();
			map<char, Rinex3_03_SysEditedObs> mapSysEditedObsList;    // 单历元的系统编辑数据map
			// 遍历每颗卫星的数据列表, 2019/08/10 邵凯
			for(size_t s_j = 0; s_j < editedObsSatlist.size(); s_j++)
			{	
				char      cSatSys; // 卫星系统
				cSatSys = editedObsSatlist[s_j].satName[0];// 卫星系统标识
				if(mapSysEditedObsList.find(cSatSys) != mapSysEditedObsList.end())
				{ // 卫星系统
					for(Rinex3_03_EditedObsEpochMap::const_iterator it = editedObsSatlist[s_j].editedObs.begin(); 
						                                       it != editedObsSatlist[s_j].editedObs.end(); ++ it)
					{ // 查找时间
						if(editedObsEpoch.t == it->first)
						{
							mapSysEditedObsList[cSatSys].cSatSys = cSatSys;
							mapSysEditedObsList[cSatSys].obsList.insert(map<string, Rinex3_03_EditedObsLine>::value_type(editedObsSatlist[s_j].satName, it->second));
	
						}
					}
				}
				else
				{// 发现新的卫星系统
					for(Rinex3_03_EditedObsEpochMap::const_iterator it = editedObsSatlist[s_j].editedObs.begin(); 
						                                       it != editedObsSatlist[s_j].editedObs.end(); ++ it)
					{ // 查找时间
						if(editedObsEpoch.t == it->first)
						{
							Rinex3_03_SysEditedObs sysEditedObsSat;
							sysEditedObsSat.cSatSys = cSatSys;
							sysEditedObsSat.obsList.insert(map<string, Rinex3_03_EditedObsLine>::value_type(editedObsSatlist[s_j].satName, it->second));	
							mapSysEditedObsList.insert(map<char, Rinex3_03_SysEditedObs>::value_type(cSatSys, sysEditedObsSat));

						}
					}
				}
			}
			for(map<char, Rinex3_03_SysEditedObs>::const_iterator it = mapSysEditedObsList.begin();
																	it != mapSysEditedObsList.end(); ++it)
			{
				Rinex3_03_SysEditedObs sysEditedObs;
				sysEditedObs.cSatSys = it->first;
				//for(map<string,  Rinex3_03_EditedObsLine>::const_iterator jt = it->second.obsList)
				sysEditedObs.obsList = it->second.obsList;
				if(sysEditedObs.obsList.size() > 0)
					editedObsEpoch.editedObs.push_back(sysEditedObs);
			}
			if(editedObsEpoch.editedObs.size() > 0)
				editedObsEpochlist.push_back(editedObsEpoch);
		}
		return true;
	}
}
