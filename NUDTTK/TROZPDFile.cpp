#include "TROZPDFile.hpp"

namespace NUDTTK
{
	TROZPDFile::TROZPDFile(void)
	{
	}

	TROZPDFile::~TROZPDFile(void)
	{
	}
	void  TROZPDFile::clear()
	{
		m_header = TROZPDHeader::TROZPDHeader();
		m_data   = TROZPDData::TROZPDData();
	}
	bool TROZPDFile::isEmpty()
	{
		if(m_data.m_Comment.size()				 > 0		 
		 ||m_data.m_InputAck.size()				 > 0		
		 ||m_data.m_SiteAnt.size()          	 > 0
		 ||m_data.m_SiteAntARP.size()       	 > 0		
		 ||m_data.m_SiteGPSPCO.size()       	 > 0
		 ||m_data.m_SiteID.size()           	 > 0
		 ||m_data.m_SiteRec.size()          	 > 0
		 ||m_data.m_StaPos.size()           	 > 0
		 ||m_data.m_TroDes.pstrSolutField.size() > 0
		 ||m_data.m_TroSolut.size()              > 0)
			return false;
		else
			return true;
	}
	// 子程序名称： open   
	// 功能：SINEX_TRO(ZPD)文件解析 
	// 变量类型：strTROZPDFileName : SINEX_TRO(ZPD)文件路径
	// 输入：strTROZPDFileName
	// 输出：
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/11
	// 版本时间：2014/09/11
	// 修改记录：
	// 备注： 
	bool TROZPDFile::open(string strTROZPDFileName)
	{
		FILE * pSNXfile = fopen(strTROZPDFileName.c_str(),"r+t");
		if(pSNXfile == NULL)
		{
			printf(" The %s file is empty !\n",strTROZPDFileName.c_str());
			return false;
		}
		m_header = TROZPDHeader::TROZPDHeader();
		// 开始读取第一行
		char line[200];
		fgets(line,100,pSNXfile);	
		string strFirstLine = line;				
		size_t nPos_n = strFirstLine.find('\n');
		strFirstLine.erase(nPos_n, 1);
		// 补齐80位
		if(strFirstLine.length() < 80) // strLine.length 包含最后的'\0'
			strFirstLine.append(80 - strFirstLine.length(),' ');
		sscanf(strFirstLine.c_str(),"%1c%1c%3c%*1c%4lf%*1c%3c%*1c%2d%*1c%3d%*1c%5d%*1c%3c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%1c%*1c%4c",
							   &m_header.szFirstchar,
							   &m_header.szSecondchar,
							   &m_header.szDocType,
							   &m_header.Version,
							   &m_header.szFileAgency,
							   &m_header.FileTime.year,
							   &m_header.FileTime.doy,
							   &m_header.FileTime.second,
							   &m_header.szDataAgency,
							   &m_header.StartTimeSolut.year,
							   &m_header.StartTimeSolut.doy,
							   &m_header.StartTimeSolut.second,
							   &m_header.EndTimeSolut.year,
							   &m_header.EndTimeSolut.doy,
							   &m_header.EndTimeSolut.second,
							   &m_header.szObsCode,
							   &m_header.szSolutCont);
		if(m_header.szFirstchar[0] != '%'|| m_header.szSecondchar[0] != '=')
		{
			printf(" The first line of %s file is invalid !\n",strTROZPDFileName.c_str());
			fclose(pSNXfile);
			return false;
		}
		// 开始循环读取每一行数据，直到 End of SINEX(%ENDSNX)
		int bFlag = true;
		m_data   = TROZPDData::TROZPDData();
		while(bFlag)
		{
			if(feof(pSNXfile))
			{
				bFlag = false;
				break;
			}
			fgets(line,100,pSNXfile);			
			if(line[0] == '+')
			{//寻找到新的数据模块
				string strLineLabel = line;
				strLineLabel.erase(0, 1); // 从第 0 个元素开始，删除 1 个
				// 剔除 \n
				size_t nPos_n = strLineLabel.find('\n');
				if(nPos_n < strLineLabel.length())
					strLineLabel.erase(nPos_n, 1);
				// 超过33位，截取33位
				while(strLineLabel.length() > 33)
					strLineLabel.erase(strLineLabel.length() - 1, 1);
				// 补齐33位
				if(strLineLabel.length() < 33) // strLineLabel.length 包含最后的'\0'
					strLineLabel.append(33 - strLineLabel.length(),' ');
				//循环读取每个数据模块
				if(strLineLabel == Sinex2_0_BlockLabel::szFileRef)
				{					
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							char     szInfoType[18 + 1];
							string   strInfoType;
							sscanf(strLine.c_str(),"%*1c%18c",szInfoType);
							szInfoType[18] = '\0';
							strInfoType = szInfoType;
							if(strInfoType == Sinex2_0_BlockLabel::szRefFrame)							
								sscanf(strLine.c_str(),"%*20c%60c",&m_data.m_FileRef.szRefFrameInfo);
							else if(strInfoType == Sinex2_0_BlockLabel::szDes)
								sscanf(strLine.c_str(),"%*20c%60c",&m_data.m_FileRef.szDesInfo);
							else if(strInfoType == Sinex2_0_BlockLabel::szInput)
								sscanf(strLine.c_str(),"%*20c%60c",&m_data.m_FileRef.szInputInfo);
							else if(strInfoType == Sinex2_0_BlockLabel::szOutput)
								sscanf(strLine.c_str(),"%*20c%60c",&m_data.m_FileRef.szOutputInfo);
							else if(strInfoType == Sinex2_0_BlockLabel::szContact)
								sscanf(strLine.c_str(),"%*20c%60c",&m_data.m_FileRef.szContactInfo);
							else if(strInfoType == Sinex2_0_BlockLabel::szSoftware)
								sscanf(strLine.c_str(),"%*20c%60c",&m_data.m_FileRef.szSoftwareInfo);
							else if(strInfoType == Sinex2_0_BlockLabel::szHardware)
								sscanf(strLine.c_str(),"%*20c%60c",&m_data.m_FileRef.szHardwareInfo);
							m_data.m_FileRef.bBlockUse = true;
						}
					}while(line[0] != '-');//
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szFileComment) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == '*')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							char     szComment[79 + 1];
							sscanf(strLine.c_str(),"%*1c%79c",&szComment);
							szComment[79] = '\0';
							m_data.m_Comment.push_back(szComment);
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szInputAck || strLineLabel == Sinex2_0_BlockLabel::szInputAck_NOE) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							InputAck     inputAck;
							sscanf(strLine.c_str(),"%*1c%3c%*1c%75c",
												  &inputAck.szAgencyCode,										  
												  &inputAck.szAgencyDes);
							m_data.m_InputAck.push_back(inputAck);
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSiteID) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							SiteID       siteID;
							sscanf(line,"%*1c%4c%*1c%2c%*1c%9c%*1c%1c%*1c%22c%*1c%3d%*1c%2d%*1c%4lf%*1c%3d%*1c%2d%*1c%4lf%*1c%7lf",
								          &siteID.szSiteCode,										  
										  &siteID.szPointCode,
										  &siteID.szMonumentID,
										  &siteID.szObsCode,
										  &siteID.szStaDes,
										  &siteID.LonDegree,
										  &siteID.LonMinute,
										  &siteID.LonSecond,
										  &siteID.LatDegree,
										  &siteID.LatMinute,
										  &siteID.LatSecond,
										  &siteID.Height);
							m_data.m_SiteID.push_back(siteID);
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSiteRec) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							SiteRecAntARPEpoch       siteRec;
							sscanf(strLine.c_str(),"%*1c%4c%*1c%2c%*1c%4c%*1c%1c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%20c%*1c%5c%*1c%11c",
												  &siteRec.szSiteCode,										  
												  &siteRec.szPointCode,
												  &siteRec.szSolutID,
												  &siteRec.szObsCode,
												  &siteRec.StartTimeSolut.year,
												  &siteRec.StartTimeSolut.doy,
												  &siteRec.StartTimeSolut.second,
												  &siteRec.EndTimeSolut.year,
												  &siteRec.EndTimeSolut.doy,
												  &siteRec.EndTimeSolut.second,
												  &siteRec.szType,
												  &siteRec.szSerial,
												  &siteRec.szFirmware);							
							m_data.m_SiteRec.insert(SiteRecAntARPEpochMap::value_type(siteRec.szSiteCode,siteRec));
							
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSiteAnt) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							SiteRecAntARPEpoch       siteAnt;
							sscanf(strLine.c_str(),"%*1c%4c%*1c%2c%*1c%4c%*1c%1c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%20c%*1c%5c",
												  &siteAnt.szSiteCode,										  
												  &siteAnt.szPointCode,
												  &siteAnt.szSolutID,
												  &siteAnt.szObsCode,
												  &siteAnt.StartTimeSolut.year,
												  &siteAnt.StartTimeSolut.doy,
												  &siteAnt.StartTimeSolut.second,
												  &siteAnt.EndTimeSolut.year,
												  &siteAnt.EndTimeSolut.doy,
												  &siteAnt.EndTimeSolut.second,
												  &siteAnt.szType,
												  &siteAnt.szSerial);

							m_data.m_SiteAnt.insert(SiteRecAntARPEpochMap::value_type(siteAnt.szSiteCode,siteAnt));
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSiteGPSPCO) 
				{
					do
					{
						fgets(line,100,pSNXfile);						
						if(line[0] == ' ')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							SitePCO       sitePCO;
							BYTE          L1Index = 1; //代表L1
							ENU           PCOL1;
						    BYTE          L2Index = 2; //代表L2
							ENU           PCOL2;							
							sscanf(strLine.c_str(),"%*1c%20c%*1c%5c%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%10c",
												  &sitePCO.szAntType,										  
												  &sitePCO.szAntSerial,
												  &PCOL1.U,
												  &PCOL1.N,
												  &PCOL1.E,
												  &PCOL2.U,
												  &PCOL2.N,
												  &PCOL2.E,										 
												  &sitePCO.PCVCorModel);
							sitePCO.PCO.insert(RecFreqPCOMap::value_type(L1Index,PCOL1));
							sitePCO.PCO.insert(RecFreqPCOMap::value_type(L2Index,PCOL2));
							//由于定义时没有初始化，此处必须增加字符串结束标识符
							sitePCO.szAntType[20]    = '\0';
							sitePCO.szAntSerial[5]   = '\0';
							sitePCO.PCVCorModel[10]  = '\0';

							m_data.m_SiteGPSPCO.insert(RecAntPCOMap::value_type(sitePCO.szAntType,sitePCO));
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSiteAntARP) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							SiteRecAntARPEpoch       siteAntARP;
							sscanf(strLine.c_str(),"%*1c%4c%*1c%2c%*1c%4c%*1c%1c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%3c%*1c%8lf%*1c%8lf%*1c%8lf",
												  &siteAntARP.szSiteCode,										  
												  &siteAntARP.szPointCode,
												  &siteAntARP.szSolutID,
												  &siteAntARP.szObsCode,
												  &siteAntARP.StartTimeSolut.year,
												  &siteAntARP.StartTimeSolut.doy,
												  &siteAntARP.StartTimeSolut.second,
												  &siteAntARP.EndTimeSolut.year,
												  &siteAntARP.EndTimeSolut.doy,
												  &siteAntARP.EndTimeSolut.second,
												  &siteAntARP.szRefSys,
												  &siteAntARP.AntARP.U,
												  &siteAntARP.AntARP.N,
												  &siteAntARP.AntARP.E);

							m_data.m_SiteAntARP.insert(SiteRecAntARPEpochMap::value_type(siteAntARP.szSiteCode,siteAntARP));
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szTroDescription) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							char     szInfoType[29 + 1];
							string   strInfoType;
							sscanf(strLine.c_str(),"%*1c%29c",szInfoType);
							szInfoType[29] = '\0';
							strInfoType = szInfoType;
							if(strInfoType == Sinex2_0_BlockLabel::szTROSampInterval)							
								sscanf(strLine.c_str(),"%*31c%22d",&m_data.m_TroDes.sampInterval);
							else if(strInfoType == Sinex2_0_BlockLabel::szTroInterval)
								sscanf(strLine.c_str(),"%*31c%22d",&m_data.m_TroDes.troInterval);
							else if(strInfoType == Sinex2_0_BlockLabel::szEleCutoff)
								sscanf(strLine.c_str(),"%*31c%22lf",&m_data.m_TroDes.eleCutoff);
							else if(strInfoType == Sinex2_0_BlockLabel::szTroMapFunc)
							{
								sscanf(strLine.c_str(),"%*31c%22c",&m_data.m_TroDes.szMapFunc);
								m_data.m_TroDes.szMapFunc[22] = '\0';
								m_data.m_TroDes.bMapFunc = true;
							}
							else if(strInfoType == Sinex2_0_BlockLabel::szSolutField1 || strInfoType == Sinex2_0_BlockLabel::szSolutField2)
							{	
								char  szInfoType1[6 + 1];
								char  szInfoType2[6 + 1];
								char  szInfoType3[6 + 1];
								char  szInfoType4[6 + 1];
								char  szInfoType5[6 + 1];
								char  szInfoType6[6 + 1];
								char  szInfoType7[6 + 1];
								sscanf(strLine.c_str(),"%*31c%s%s%s%s%s%s%s",
									                   &szInfoType1,
													   &szInfoType2,
													   &szInfoType3,
													   &szInfoType4,
													   &szInfoType5,
													   &szInfoType6,
													   &szInfoType7);	
								if(szInfoType1[0] != -52)
									m_data.m_TroDes.pstrSolutField.push_back(szInfoType1);
								if(szInfoType2[0] != -52)
									m_data.m_TroDes.pstrSolutField.push_back(szInfoType2);
								if(szInfoType3[0] != -52)
									m_data.m_TroDes.pstrSolutField.push_back(szInfoType3);
								if(szInfoType4[0] != -52)
									m_data.m_TroDes.pstrSolutField.push_back(szInfoType4);
								if(szInfoType5[0] != -52)
									m_data.m_TroDes.pstrSolutField.push_back(szInfoType5);
								if(szInfoType6[0] != -52)
									m_data.m_TroDes.pstrSolutField.push_back(szInfoType6);
								if(szInfoType7[0] != -52)
									m_data.m_TroDes.pstrSolutField.push_back(szInfoType7);								
							}							
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szTroStaCoordinate) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							TroStaPos       troStaPos;
							sscanf(strLine.c_str(),"%*1c%4c%*1c%2c%*1c%4c%*1c%1c%*1c%12lf%*1c%12lf%*1c%12lf%*1c%6c%*1c%5c%*1c%2d%*1c%2d%*1c%2d%*1c%2d",
												  &troStaPos.szSiteCode,										  
												  &troStaPos.szPointCode,
												  &troStaPos.szSolutID,
												  &troStaPos.szObsCode,
												  &troStaPos.pos.x,
												  &troStaPos.pos.y,
												  &troStaPos.pos.z,
												  &troStaPos.szRefSys,
												  &troStaPos.szRemark,
												  &troStaPos.posSTD.x,
												  &troStaPos.posSTD.y,
												  &troStaPos.posSTD.z,
												  &troStaPos.counterAC);
							m_data.m_StaPos.insert(TroStaPosMap::value_type(troStaPos.szSiteCode,troStaPos));
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szTroSolution) 
				{
					TroSolutList    troSolutList;
					do
					{						
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							string strLine = line;				
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							// 补齐80位
							if(strLine.length() < 80) // strLine.length 包含最后的'\0'
								strLine.append(80 - strLine.length(),' ');
							TroSolut       troSolut;
							sscanf(strLine.c_str(),"%*1c%4c%*1c%2d%*1c%3d%*1c%5d%6lf%6lf%6lf%6lf%6lf%6lf",
												  &troSolut.szMarker,										  
												  &troSolut.EpochTime.year,
												  &troSolut.EpochTime.doy,
												  &troSolut.EpochTime.second,
												  &troSolut.TROTOT,
												  &troSolut.TROTOTSTD,
												  &troSolut.TGNTOT,
												  &troSolut.TGNTOTSTD,
												  &troSolut.TGETOT,
												  &troSolut.TGETOTSTD);							
							TroSolutMap::iterator it;
							if((it = m_data.m_TroSolut.find(troSolut.szMarker)) != m_data.m_TroSolut.end())
								m_data.m_TroSolut[troSolut.szMarker].push_back(troSolut);
							else
							{
								troSolutList.clear();
								troSolutList.push_back(troSolut);
								m_data.m_TroSolut.insert(TroSolutMap::value_type(troSolut.szMarker,troSolutList));
							}
						}
					}while(line[0] != '-');
				}
			}			
			else if(line[0] == '%')
			{
				bFlag = false;
			}
			else
			{
				//Comment,不做处理
			}			
		}
		fclose(pSNXfile);
		if(m_data.m_TroDes.pstrSolutField.size() == 0)
		{
			printf("The block of TROP/DESCRIPTION isn't complete in %s file!\n",strTROZPDFileName.c_str());
			return false;
		}
		else
			return true;
	}
	// 子程序名称： write   
	// 功能：将数据写到TRO(ZPD)文件 
	// 变量类型：strTROZPDFileName    : TRO(ZPD)文件路径
	// 输入：strTROZPDFileName
	// 输出：
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/11
	// 版本时间：2014/09/11
	// 修改记录：
	// 备注： 
	bool TROZPDFile::write(string strTROZPDFileName)
	{
		if(isEmpty())
		{
			printf("%s file is empty!\n",strTROZPDFileName.c_str());
			return false;	
		}
		FILE *pSNXfile = fopen(strTROZPDFileName.c_str(), "w+");
		// 写第一行	
		fprintf(pSNXfile,"%c%c%3s %4.2lf %3s %02d:%03d:%05d %3s %02d:%03d:%05d %02d:%03d:%05d %c %4s\n",
	                    m_header.szFirstchar[0],
					    m_header.szSecondchar[0],
				        m_header.szDocType,
					    m_header.Version,
					    m_header.szFileAgency,
					    m_header.FileTime.year,
					    m_header.FileTime.doy,
					    m_header.FileTime.second,
				        m_header.szDataAgency,
					    m_header.StartTimeSolut.year,
					    m_header.StartTimeSolut.doy,
					    m_header.StartTimeSolut.second,
					    m_header.EndTimeSolut.year,
					    m_header.EndTimeSolut.doy,
					    m_header.EndTimeSolut.second,
					    m_header.szObsCode[0],						
						m_header.szSolutCont);		
		// 循环书写每个数据模块
		if(m_data.m_FileRef.bBlockUse)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szFileRef);
			fprintf(pSNXfile,"*INFO_TYPE_________ INFO________________________________________________________\n");
			if(m_data.m_FileRef.szRefFrameInfo[0] != 0)
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szRefFrame,m_data.m_FileRef.szRefFrameInfo);
			if(m_data.m_FileRef.szDesInfo[0] != 0)
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szDes,m_data.m_FileRef.szDesInfo);
			if(m_data.m_FileRef.szInputInfo[0] != 0)
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szInput,m_data.m_FileRef.szInputInfo);
			if(m_data.m_FileRef.szOutputInfo[0] != 0)
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szOutput,m_data.m_FileRef.szOutputInfo);
			if(m_data.m_FileRef.szContactInfo[0] != 0)
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szContact,m_data.m_FileRef.szContactInfo);
			if(m_data.m_FileRef.szSoftwareInfo[0] != 0)
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szSoftware,m_data.m_FileRef.szSoftwareInfo);
			if(m_data.m_FileRef.szHardwareInfo[0] != 0)
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szHardware,m_data.m_FileRef.szHardwareInfo);
			
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szFileRef);
		}
		if(m_data.m_Comment.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szFileComment);
			for(size_t s_i = 0; s_i < m_data.m_Comment.size(); s_i ++)
				fprintf(pSNXfile,"*%-79s\n",m_data.m_Comment[s_i]);
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szFileComment);
		}
		if(m_data.m_InputAck.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szInputAck);
			fprintf(pSNXfile,"*AGY ______________________________FULL_DESCRIPTION_____________________________\n");
			for(size_t s_i = 0; s_i < m_data.m_InputAck.size(); s_i ++)
				fprintf(pSNXfile," %-3s %-75s\n",m_data.m_InputAck[s_i].szAgencyCode,m_data.m_InputAck[s_i].szAgencyDes);
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szInputAck);
		}
		if(m_data.m_SiteID.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSiteID);
			fprintf(pSNXfile,"*CODE PT __DOMES__ T _STATION DESCRIPTION__ APPROX_LON_ APPROX_LAT_ _APP_H_\n");
			for(size_t s_i = 0; s_i < m_data.m_SiteID.size(); s_i ++)
				fprintf(pSNXfile," %-4s %-2s %-9s %c %-22s %3d %2d %4.1lf %3d %2d %4.1lf %7.1lf\n",
						          m_data.m_SiteID[s_i].szSiteCode,										  
								  m_data.m_SiteID[s_i].szPointCode,
								  m_data.m_SiteID[s_i].szMonumentID,
								  m_data.m_SiteID[s_i].szObsCode[0],
								  m_data.m_SiteID[s_i].szStaDes,
								  m_data.m_SiteID[s_i].LonDegree,
								  m_data.m_SiteID[s_i].LonMinute,
								  m_data.m_SiteID[s_i].LonSecond,
								  m_data.m_SiteID[s_i].LatDegree,
								  m_data.m_SiteID[s_i].LatMinute,
								  m_data.m_SiteID[s_i].LatSecond,
								  m_data.m_SiteID[s_i].Height);				
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSiteID);
		}
		if(m_data.m_SiteRec.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSiteRec);
			fprintf(pSNXfile,"*CODE PT SOLN T _DATA START_ __DATA_END__ ________TYPE________ SERIE _FIRMWARE__\n");
			for(SiteRecAntARPEpochMap::iterator it = m_data.m_SiteRec.begin(); it != m_data.m_SiteRec.end(); ++it)
				fprintf(pSNXfile," %-4s %-2s %-4s %c %02d:%03d:%05d %02d:%03d:%05d %-20s %-5s %-11s\n",
								  it->second.szSiteCode,										  
								  it->second.szPointCode,
								  it->second.szSolutID,
								  it->second.szObsCode[0],
								  it->second.StartTimeSolut.year,
								  it->second.StartTimeSolut.doy,
								  it->second.StartTimeSolut.second,
								  it->second.EndTimeSolut.year,
								  it->second.EndTimeSolut.doy,
								  it->second.EndTimeSolut.second,
								  it->second.szType,
								  it->second.szSerial,
								  it->second.szFirmware);								
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSiteRec);
		}
		if(m_data.m_SiteAnt.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSiteAnt);
			fprintf(pSNXfile,"*CODE PT SOLN T _DATA START_ __DATA_END__ ________TYPE________ SERIE\n");
			for(SiteRecAntARPEpochMap::iterator it = m_data.m_SiteAnt.begin(); it != m_data.m_SiteAnt.end(); ++it)
				fprintf(pSNXfile," %-4s %-2s %-4s %c %02d:%03d:%05d %02d:%03d:%05d %-20s %-5s\n",
								  it->second.szSiteCode,										  
								  it->second.szPointCode,
								  it->second.szSolutID,
								  it->second.szObsCode[0],
								  it->second.StartTimeSolut.year,
								  it->second.StartTimeSolut.doy,
								  it->second.StartTimeSolut.second,
								  it->second.EndTimeSolut.year,
								  it->second.EndTimeSolut.doy,
								  it->second.EndTimeSolut.second,
								  it->second.szType,
								  it->second.szSerial);								
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSiteAnt);
		}
		if(m_data.m_SiteGPSPCO.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSiteGPSPCO);
			fprintf(pSNXfile,"*________TYPE________ SERIE _L1_U_ _L1_N_ _L1_E_ _L2_U_ _L2_N_ _L2_E_ __MODEL___\n");
			for(RecAntPCOMap::iterator it = m_data.m_SiteGPSPCO.begin(); it != m_data.m_SiteGPSPCO.end(); ++it)
			{
				double  PCO[6];
				PCO[0] = it->second.PCO[1].U;
				PCO[1] = it->second.PCO[1].N;
				PCO[2] = it->second.PCO[1].E;
				PCO[3] = it->second.PCO[2].U;
				PCO[4] = it->second.PCO[2].N;
				PCO[5] = it->second.PCO[2].E;
				fprintf(pSNXfile," %-20s %-5s",
								  it->second.szAntType,										  
								  it->second.szAntSerial);
				for(int i = 0; i < 6; i ++)
				{
					char szPara[8];
					string strPara;
					sprintf(szPara, "%7.4f", PCO[i]);			
					Sinex2_0_File::stringEraseFirstZero(szPara, strPara);
					fprintf(pSNXfile," %6s", strPara.c_str());
				}
				fprintf(pSNXfile," %-10s\n",it->second.PCVCorModel);					
			}
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSiteGPSPCO);
		}
		if(m_data.m_SiteAntARP.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSiteAntARP);
			fprintf(pSNXfile,"*CODE PT SOLN T _DATA START_ __DATA_END__ REF __DX1___ __DX2___ __DX3___\n");
			for(SiteRecAntARPEpochMap::iterator it = m_data.m_SiteAntARP.begin(); it != m_data.m_SiteAntARP.end(); ++it)
				fprintf(pSNXfile," %-4s %-2s %-4s %c %02d:%03d:%05d %02d:%03d:%05d %-3s %8.4lf %8.4lf %8.4lf\n",
								  it->second.szSiteCode,										  
								  it->second.szPointCode,
								  it->second.szSolutID,
								  it->second.szObsCode[0],
								  it->second.StartTimeSolut.year,
								  it->second.StartTimeSolut.doy,
								  it->second.StartTimeSolut.second,
								  it->second.EndTimeSolut.year,
								  it->second.EndTimeSolut.doy,
								  it->second.EndTimeSolut.second,
								  it->second.szRefSys,
								  it->second.AntARP.U,
								  it->second.AntARP.N,
								  it->second.AntARP.E);								
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSiteAntARP);
		}
		if(m_data.m_TroDes.pstrSolutField.size() > 0)
		{
			if(m_data.m_TroDes.pstrSolutField.size() != 6)
				printf("Warning!The TRO production doesn't use the normal format!\n");
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szTroDescription);
			fprintf(pSNXfile,"*KEYWORD______________________ VALUE(S)______________\n");
			if(m_data.m_TroDes.sampInterval != INT_MAX)
				fprintf(pSNXfile," %29s %22d\n",Sinex2_0_BlockLabel::szTROSampInterval,m_data.m_TroDes.sampInterval);
			if(m_data.m_TroDes.troInterval != INT_MAX)
				fprintf(pSNXfile," %29s %22d\n",Sinex2_0_BlockLabel::szTroInterval,m_data.m_TroDes.troInterval);
			if(m_data.m_TroDes.eleCutoff != DBL_MAX)
				fprintf(pSNXfile," %29s %22.1lf\n",Sinex2_0_BlockLabel::szEleCutoff,m_data.m_TroDes.eleCutoff);
			if(m_data.m_TroDes.bMapFunc)
				fprintf(pSNXfile," %29s %-s\n",Sinex2_0_BlockLabel::szTroMapFunc,m_data.m_TroDes.szMapFunc);
			if(m_data.m_TroDes.pstrSolutField.size() > 0)
			{
				fprintf(pSNXfile," %29s",Sinex2_0_BlockLabel::szSolutField1);
				if(m_data.m_TroDes.pstrSolutField.size() <= 7)
				{
					for(size_t s_i = 0; s_i < m_data.m_TroDes.pstrSolutField.size(); s_i++)
						fprintf(pSNXfile," %-s",m_data.m_TroDes.pstrSolutField[s_i].c_str());
					fprintf(pSNXfile,"\n");
				}
				else
				{
					for(size_t s_i = 0; s_i < 7; s_i++)
						fprintf(pSNXfile," %s",m_data.m_TroDes.pstrSolutField[s_i]);
					fprintf(pSNXfile,"\n");
					// 写第二行
					fprintf(pSNXfile," %29s",Sinex2_0_BlockLabel::szSolutField2);
					for(size_t s_j = 0; s_j < m_data.m_TroDes.pstrSolutField.size() - 7; s_j++)
						fprintf(pSNXfile," %s",m_data.m_TroDes.pstrSolutField[s_j + 7]);
					fprintf(pSNXfile,"\n");
				}
			}
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szTroDescription);
		}
		if(m_data.m_StaPos.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szTroStaCoordinate);
			fprintf(pSNXfile,"*SITE PT SOLN T STA_X_______ STA_Y_______ STA_Z_______ SYSTEM REMARK\n");
			for(TroStaPosMap::iterator it = m_data.m_StaPos.begin(); it != m_data.m_StaPos.end(); ++it)
				fprintf(pSNXfile," %-4s %-2s %-4s %c %12.3lf %12.3lf %12.3lf %-6s %-5s %2d %2d %2d %2d\n",
								  it->second.szSiteCode,										  
								  it->second.szPointCode,
								  it->second.szSolutID,
								  it->second.szObsCode[0],
								  it->second.pos.x,
								  it->second.pos.y,
								  it->second.pos.z,
								  it->second.szRefSys,
								  it->second.szRemark,							
								  int(it->second.posSTD.x),
								  int(it->second.posSTD.y),
								  int(it->second.posSTD.z),
								  it->second.counterAC);								
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szTroStaCoordinate);
		}
		if(m_data.m_TroSolut.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szTroSolution);
			fprintf(pSNXfile,"*SITE EPOCH_______ TROTOT STDEV  TGNTOT  STDEV  TGETOT  STDEV\n");
			for(TroSolutMap::iterator it = m_data.m_TroSolut.begin(); it != m_data.m_TroSolut.end(); ++it)
			{
				for(size_t s_i = 0; s_i < it->second.size(); s_i ++)
					fprintf(pSNXfile," %-4s %02d:%03d:%05d %6.1lf %5.1lf %7.3lf %6.3lf %7.3lf %6.3lf\n",
									  it->second[s_i].szMarker,										  
									  it->second[s_i].EpochTime.year,
									  it->second[s_i].EpochTime.doy,
									  it->second[s_i].EpochTime.second,
									  it->second[s_i].TROTOT,
									  it->second[s_i].TROTOTSTD,
									  it->second[s_i].TGNTOT,
									  it->second[s_i].TGNTOTSTD,
									  it->second[s_i].TGETOT,							
									  it->second[s_i].TGETOTSTD);
			}
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szTroSolution);
		}
		fprintf(pSNXfile,"%c%c%-6s\n",m_header.szFirstchar[0],m_header.szSecondchar[0],"ENDTRO");
		fclose(pSNXfile);
		return true;
	}
	// 子程序名称： getTroZPDSolut   
	// 功能：线性差值获取天顶延迟 
	// 变量类型：staName    : 测站名
	//           t          : UTC 时间
	//           t_tro      : t 时刻的对流层延迟参数
	//           t_forecast : 允许外延的最长时间，单位s
	// 输入：strStaName，t, t_forecast
	// 输出：t_tro
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/12
	// 版本时间：2014/09/12
	// 修改记录：2015/09/16,刘俊宏修改，增加外延参数t_forecast
	// 备注：
	bool    TROZPDFile::getTroZPDSolut(string name, UTC t, TroSolut &t_tro,double t_forecast)
	{		
		// 测站名称转换成大写
        char staName[5];
		for(int i = 0; i < 4; i++)
		{			
			staName[i] = toCapital(name[i]);
		}		
		staName[4] = '\0';
		TroSolutMap::iterator it;
		if((it = m_data.m_TroSolut.find(staName)) != m_data.m_TroSolut.end())
		{
			if(it->second.front().EpochTime.toUTC() - t > t_forecast || t - it->second.back().EpochTime.toUTC() > t_forecast)
			{
				//printf("%s beyond the valid bounds!\n",t.toString().c_str());
				return false;
			}
			else 
			{
				TroSolut  tro1;   // t时刻前的首个对流层产品
				TroSolut  tro2;   // t时刻后的首个对流层产品
				for(size_t s_i = 0; s_i < it->second.size() - 1; s_i ++)
				{
					UTC t1 = it->second[s_i].EpochTime.toUTC();
					UTC t2 = it->second[s_i + 1].EpochTime.toUTC();
					if((t1 - t <= 0 && t2 - t >= 0) || it->second.front().EpochTime.toUTC() - t > 0 || t - it->second.back().EpochTime.toUTC() > 0)
					{
						tro1 = it->second[s_i];
						tro2 = it->second[s_i + 1];
						strxfrm(t_tro.szMarker, it->second[s_i].szMarker,sizeof(it->second[s_i].szMarker));
						t_tro.EpochTime.year   = t.year % 100;  
						t_tro.EpochTime.doy    = t.doy();
						t_tro.EpochTime.second = t.hour * 3600 + t.minute * 60 + int(t.second);
						t_tro.TROTOT = tro1.TROTOT + (t - t1) * (tro2.TROTOT - tro1.TROTOT)/(t2 - t1);
						t_tro.TROTOTSTD = tro1.TROTOTSTD + (t - t1) * (tro2.TROTOTSTD - tro1.TROTOTSTD)/(t2 - t1);
						t_tro.TGNTOT = tro1.TGNTOT + (t - t1) * (tro2.TGNTOT - tro1.TGNTOT)/(t2 - t1);
						t_tro.TGNTOTSTD = tro1.TGNTOTSTD + (t - t1) * (tro2.TGNTOTSTD - tro1.TGNTOTSTD)/(t2 - t1);
						t_tro.TGETOT = tro1.TGETOT + (t - t1) * (tro2.TGETOT - tro1.TGETOT)/(t2 - t1);
						t_tro.TGETOTSTD = tro1.TGETOTSTD + (t - t1) * (tro2.TGETOTSTD - tro1.TGETOTSTD)/(t2 - t1);
						break;
					}
				}
				return true;
			}
			
		}
		else		
		{
			printf("Haven't found this station's TRO product(%s)!\n",staName);
			return false;	
		}
	}
}