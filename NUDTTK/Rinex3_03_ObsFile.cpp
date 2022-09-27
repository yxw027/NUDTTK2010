#include "Rinex3_03_ObsFile.hpp"
#include "constDef.hpp"
#include "MathAlgorithm.hpp"
#include "TimeCoordConvert.hpp"

using namespace NUDTTK::Math;

namespace NUDTTK
{	
	const char Rinex3_03_MaskString::szVerType[]                = "RINEX VERSION / TYPE";
	const char Rinex3_03_MaskString::szPgmRunDate[]             = "PGM / RUN BY / DATE ";
	const char Rinex3_03_MaskString::szComment[]                = "COMMENT             ";	
	const char Rinex3_03_MaskString::szMarkerName[]             = "MARKER NAME         ";	
	const char Rinex3_03_MaskString::szMarkerNum[]              = "MARKER NUMBER       ";
	const char Rinex3_03_MaskString::szMarkerType[]             = "MARKER TYPE         "; // +
	const char Rinex3_03_MaskString::szObservAgency[]           = "OBSERVER / AGENCY   ";
	const char Rinex3_03_MaskString::szRecTypeVers[]            = "REC # / TYPE / VERS ";
	const char Rinex3_03_MaskString::szAntType[]                = "ANT # / TYPE        ";
	const char Rinex3_03_MaskString::szApproxPosXYZ[]           = "APPROX POSITION XYZ ";	
	const char Rinex3_03_MaskString::szAntDeltaHEN[]            = "ANTENNA: DELTA H/E/N";
	const char Rinex3_03_MaskString::szAntDeltaXYZ[]            = "ANTENNA: DELTA X/Y/Z"; // +
	const char Rinex3_03_MaskString::szAntPhaseCenter[]         = "ANTENNA: PHASECENTER";
	const char Rinex3_03_MaskString::szAntBSightXYZ[]           = "ANTENNA: B.SIGHT XYZ"; // +
	const char Rinex3_03_MaskString::szZeroDIRAzimuth[]		    = "ANTENNA: ZERODIR AZI"; // +
	const char Rinex3_03_MaskString::szZeroDIRXYZ[]			    = "ANTENNA: ZERODIR XYZ"; // +
	const char Rinex3_03_MaskString::szCOMXYZ[]                 = "CENTER OF MASS: XYZ "; // +
	const char Rinex3_03_MaskString::szSysObsTypes[]            = "SYS / # / OBS TYPES "; // +
	const char Rinex3_03_MaskString::szSignalStrengthUnit[]     = "SIGNAL STRENGTH UNIT"; // +
	const char Rinex3_03_MaskString::szInterval[]               = "INTERVAL            ";
	const char Rinex3_03_MaskString::szTmOfFirstObs[]           = "TIME OF FIRST OBS   ";	
	const char Rinex3_03_MaskString::szTmOfLastObs[]            = "TIME OF LAST OBS    ";
	const char Rinex3_03_MaskString::szRCVClockOffsetApplied[]  = "RCV CLOCK OFFS APPL "; // +
	const char Rinex3_03_MaskString::szSysDCBSApplied[]         = "SYS / DCBS APPLIED  "; // +
	const char Rinex3_03_MaskString::szSysPCVSApplied[]         = "SYS / PCVS APPLIED  "; // +
	const char Rinex3_03_MaskString::szSysScaleFactor[]         = "SYS / SCALE FACTOR  "; // +
	const char Rinex3_03_MaskString::szSysPhaseShift[]          = "SYS / PHASE SHIFT   "; // +
	const char Rinex3_03_MaskString::szGlonassSlotFrq[]         = "GLONASS SLOT / FRQ #"; // + 
	const char Rinex3_03_MaskString::szGlonassCodePhaseBias[]   = "GLONASS COD/PHS/BIS "; // +
	const char Rinex3_03_MaskString::szLeapSec[]                = "LEAP SECONDS        ";
	const char Rinex3_03_MaskString::szNumsOfSat[]              = "# OF SATELLITES     ";
	const char Rinex3_03_MaskString::szPRNNumOfObs[]            = "PRN / # OF OBS      ";	
	const char Rinex3_03_MaskString::szEndOfHead[]              = "END OF HEADER       ";

	Rinex3_03_ObsFile::Rinex3_03_ObsFile(void)
	{
	}

	Rinex3_03_ObsFile::~Rinex3_03_ObsFile(void)
	{
	}

	void Rinex3_03_ObsFile::clear()
    {
		m_header = Rinex3_03_ObsHeader::Rinex3_03_ObsHeader();
		m_data.clear();		
	}

	bool Rinex3_03_ObsFile::isEmpty()
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
	//           pObsfile      　　: 文件指针
	// 输入：strLine, pObsfile
	// 输出：
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2018/5/29
	// 修改记录：
	// 其它：    参考 Rinex3_0_ObsFile.cpp 文件和 RINEX 3.03 文件
	int  Rinex3_03_ObsFile::isValidEpochLine(string strLine, FILE * pObsfile)
	{
		DayTime tmEpoch;
		int byEpochFlag = -1; // 0: OK; 1: power failure between previous and current epoch  > 1: Event flag ( 2 - 6 ) 
		int bySatCount  = -1; // 本时刻卫星（或测站）个数
		char szRecordIdentifier[1 +1 ];
		if(pObsfile != NULL)  // 判断是否为文件末尾
		{
			if(feof(pObsfile))
				return 0;
		}		
		sscanf(strLine.c_str(),"%1c%*1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%11lf%*2c%1d%3d",
			&szRecordIdentifier,
			&tmEpoch.year,
			&tmEpoch.month,
			&tmEpoch.day,
			&tmEpoch.hour,
			&tmEpoch.minute,
			&tmEpoch.second,
			&byEpochFlag,
			&bySatCount);
		if(szRecordIdentifier[0] != '>')
			return 2;
		if(tmEpoch.month > 12 || tmEpoch.month<= 0)
			return 2;
		if(tmEpoch.day >31 || tmEpoch.day <= 0)
			return 2;
		if(byEpochFlag > 6 || byEpochFlag < 0)
			return 2;
		if(bySatCount > 999 || bySatCount < 0)
			return 2;
		return 1;
	}
	// 子程序名称： open   
	// 功能：观测数据文件解析 
    // 变量类型：strObsfileName : 观测数据文件路径
	//           bOn_BDT2GPST   : 是否将BDT转换为GPST, 默认GPS时间
    // 输入：strObsfileName
    // 输出：
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2018/5/29
	// 修改记录：
	// 其它：    参考 Rinex3_0_ObsFile.cpp 文件和 RINEX 3.03 文件
    bool Rinex3_03_ObsFile::open(string strObsfileName, bool bOn_BDT2GPST)
	{
		if(!isWildcardMatch(strObsfileName.c_str(), "*.*O", true) && !isWildcardMatch(strObsfileName.c_str(), "*.rnx", true))
		{
			printf(" %s 文件名不匹配!\n", strObsfileName.c_str());
			return false;
		}

		FILE * pObsfile = fopen(strObsfileName.c_str(),"r+t");
		if(pObsfile == NULL) 
		{
			printf(" %s 文件为空!\n", strObsfileName.c_str());
			return false;
		}
		m_header = Rinex3_03_ObsHeader::Rinex3_03_ObsHeader();		
		// 开始循环读取每一行数据，直到 END OF HEADER
		int bFlag = 1;
		while(bFlag)
		{
			char line[400];
			fgets(line,400,pObsfile);
			string strLineMask = line;
			string strLine     = line;			
			strLineMask.erase(0, 60); // 从第 0 个元素开始，删除 60 个
			// 剔除 \n
			size_t nPos_n = strLineMask.find('\n');
			if(nPos_n > 10000)//2013.7.16 刘俊宏
			{
				printf(" %s 文件有误!\n", strObsfileName.c_str());
				fclose(pObsfile);	
				return false;
			}
			if(nPos_n < strLineMask.length())
				strLineMask.erase(nPos_n, 1);
			// 超过20位，截取20位
			while(strLineMask.length() > 20)
				strLineMask.erase(strLineMask.length() - 1, 1);
			// 补齐20位
			if(strLineMask.length() < 20) // strLineMask.length 包含最后的'\0'
				strLineMask.append(20-strLineMask.length(),' ');

			if(strLineMask == Rinex3_03_MaskString::szVerType)
			{
				//sscanf(line,"%9lf%*11c%1c%*19c%1c",&m_header.rinexVersion,&m_header.szFileType,&m_header.cSatSys);
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
			else if(strLineMask == Rinex3_03_MaskString::szComment)
			{
				strLine.copy(m_header.szCommentLine, 60, 0);					
			}
			else if(strLineMask == Rinex3_03_MaskString::szMarkerName)
			{
				strLine.copy(m_header.szMarkName, 60, 0);					
			}
			else if(strLineMask == Rinex3_03_MaskString::szMarkerNum)
			{
				strLine.copy(m_header.szMarkNumber, 20, 0);					
			}
			else if(strLineMask == Rinex3_03_MaskString::szMarkerType)
			{
				strLine.copy(m_header.szMarkType, 20, 0);					
			}
			else if(strLineMask == Rinex3_03_MaskString::szObservAgency)
			{
				strLine.copy(m_header.szObserverName, 20, 0);
				strLine.copy(m_header.szObserverAgencyName, 40, 20);
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
				sscanf(line,"%14lf%14lf%14lf",&m_header.approxPosXYZ.x, &m_header.approxPosXYZ.y, &m_header.approxPosXYZ.z);
			}
			else if(strLineMask == Rinex3_03_MaskString::szAntDeltaHEN)
			{  
				sscanf(line,"%14lf%14lf%14lf",&m_header.antDeltaHEN.x, &m_header.antDeltaHEN.y, &m_header.antDeltaHEN.z);
			}
			else if(strLineMask == Rinex3_03_MaskString::szAntDeltaXYZ)
			{  
				sscanf(line,"%14lf%14lf%14lf",&m_header.antDeltaXYZ.x, &m_header.antDeltaXYZ.y, &m_header.antDeltaXYZ.z);
			}
			else if(strLineMask == Rinex3_03_MaskString::szAntPhaseCenter)
			{  
				Rinex3_03_AntPhaseCenter	antPhaCen;		
				sscanf(line,"%1c%*1c%3c%9lf%14lf%14lf",&antPhaCen.cSatSys,
					                                   &antPhaCen.szObsCode,
													   &antPhaCen.phaseCenter.x,
													   &antPhaCen.phaseCenter.y,
													   &antPhaCen.phaseCenter.z);
				antPhaCen.szObsCode[3] = '\0';
				m_header.phaseCenterList.push_back(antPhaCen);
								
			}
			else if(strLineMask == Rinex3_03_MaskString::szAntBSightXYZ)
			{  
				sscanf(line,"%14lf%14lf%14lf",&m_header.antBSightXYZ.x, &m_header.antBSightXYZ.y, &m_header.antBSightXYZ.z);
			}
			else if(strLineMask == Rinex3_03_MaskString::szZeroDIRAzimuth)
			{  
				sscanf(line,"%14lf",&m_header.antZeroDIRAzimuth);
			}
			else if(strLineMask == Rinex3_03_MaskString::szZeroDIRXYZ)
			{  
				sscanf(line,"%14lf%14lf%14lf",&m_header.antZeroDIRXYZ.x, &m_header.antZeroDIRXYZ.y, &m_header.antZeroDIRXYZ.z);
			}
			else if(strLineMask == Rinex3_03_MaskString::szCOMXYZ)
			{  
				sscanf(line,"%14lf%14lf%14lf",&m_header.antCOMXYZ.x, &m_header.antCOMXYZ.y, &m_header.antCOMXYZ.z);
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
						fgets(line,100,pObsfile);
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
						fgets(line,100,pObsfile);
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
			else if(strLineMask == Rinex3_03_MaskString::szSignalStrengthUnit)
			{
				strLine.copy(m_header.szSignalStrengthUnit, 20, 0);
			}
			else if(strLineMask == Rinex3_03_MaskString::szInterval)
			{  
				sscanf(line,"%10lf", &m_header.interval);
			}
			else if(strLineMask == Rinex3_03_MaskString::szTmOfFirstObs)
			{  
				sscanf(line,"%6d%6d%6d%6d%6d%13lf",&m_header.tmStart.year, 
					                               &m_header.tmStart.month,
												   &m_header.tmStart.day,   
												   &m_header.tmStart.hour,
												   &m_header.tmStart.minute,
												   &m_header.tmStart.second);
				strLine.copy(m_header.szTimeSystem, 3, 48);

				string strTimeSystem = m_header.szTimeSystem;
				if(strTimeSystem.find("BDT") == -1)
				{
					bOn_BDT2GPST = false;
				}
				if(bOn_BDT2GPST)
				{// 20160519, 将时间系统调整为GPS时间
					sprintf(m_header.szTimeSystem, "GPS");
					m_header.tmStart = TimeCoordConvert::BDT2GPST(m_header.tmStart);
				}
				// 考虑其他系统时？

			}
			else if(strLineMask == Rinex3_03_MaskString::szTmOfLastObs)
			{  
				sscanf(line,"%6d%6d%6d%6d%6d%13lf",&m_header.tmEnd.year, 
					                               &m_header.tmEnd.month,
												   &m_header.tmEnd.day,	 
												   &m_header.tmEnd.hour,
												   &m_header.tmEnd.minute,
												   &m_header.tmEnd.second);
				if(bOn_BDT2GPST)// 20160519, 将时间系统调整为GPS时间
					m_header.tmEnd = TimeCoordConvert::BDT2GPST(m_header.tmEnd);
				// 考虑其他系统时？

			}
			else if(strLineMask == Rinex3_03_MaskString::szRCVClockOffsetApplied)
			{  
				sscanf(line,"%6d",&m_header.rcvClockOffsetApplied);
			}
			else if(strLineMask == Rinex3_03_MaskString::szSysDCBSApplied)
			{  
				Rinex3_03_CorrApplied         DCBApp;						
				sscanf(line,"%1c",&DCBApp.cSatSys);
				strLine.copy(DCBApp.szNameProgram, 17, 2);
				strLine.copy(DCBApp.szNameURL, 40, 20);	
				m_header.sysDCBSAppliedList.push_back(DCBApp);
			}
			else if(strLineMask == Rinex3_03_MaskString::szSysPCVSApplied)
			{  
				Rinex3_03_CorrApplied         PCVApp;				
				sscanf(line,"%1c",&PCVApp.cSatSys);
				strLine.copy(PCVApp.szNameProgram, 17, 2);
				strLine.copy(PCVApp.szNameURL, 40, 20);	
				m_header.sysPCVSAppliedList.push_back(PCVApp);
			}
			else if(strLineMask == Rinex3_03_MaskString::szSysScaleFactor)
			{  
				Rinex3_03_SysObsType         sysObsTyp;							
				sscanf(line,"%1c%*1c%4d%*2c%2d", &sysObsTyp.cSatSys,&sysObsTyp.scaleFactor,&sysObsTyp.obsTypeCount);
				int               nline = sysObsTyp.obsTypeCount / 12;	// 整行数
				int               nResidue = sysObsTyp.obsTypeCount % 12;
				if(sysObsTyp.obsTypeCount <= 12)
				{
					for(BYTE i = 0; i < sysObsTyp.obsTypeCount; i++)
					{
						char strObsType[3 + 1];
						strLine.copy(strObsType, 3, 10 + i * 4 + 1);
						strObsType[3] = '\0';
						sysObsTyp.obsTypeList.push_back(strObsType);
					}				
				}
				else
				{
					for(BYTE i = 0; i < 12; i++)
					{
						char strObsType[3 + 1];
						strLine.copy(strObsType, 3, 10 + i * 4 + 1);
						strObsType[3] = '\0';
						sysObsTyp.obsTypeList.push_back(strObsType);
					}
					for(int n = 1;n < nline; n++)
					{// 读取中间的整行
						fgets(line,100,pObsfile);
						strLine = line;
						for(BYTE i = 0; i < 12; i++)
						{
							char strObsType[3 + 1];
							strLine.copy(strObsType, 3, 10 + i * 4 + 1);
							strObsType[3] = '\0';
							sysObsTyp.obsTypeList.push_back(strObsType);
						}
					}
					if(nResidue > 0)
					{
						fgets(line,100,pObsfile);
						strLine = line;
						for(BYTE i = 0; i < nResidue; i++)
						{
							char strObsType[3 + 1];
							strLine.copy(strObsType, 3, 10 + i * 4 + 1);
							strObsType[3] = '\0';
							sysObsTyp.obsTypeList.push_back(strObsType);
						}	
					}
				}
				m_header.sysScaleFactorList.push_back(sysObsTyp);
			}
			else if(strLineMask == Rinex3_03_MaskString::szSysPhaseShift)
			{  
				char numofsat[3];
				Rinex3_03_SysPhaseShift         sysPhaseShift;							
				sscanf(line,"%1c%*1c%3c%*1c%8lf%*2c%2c", &sysPhaseShift.cSatSys,&sysPhaseShift.obsType,&sysPhaseShift.phaseShift, &numofsat);
				numofsat[2] = '\0';
				if(numofsat[0] == '\0')
                    sysPhaseShift.satCount = 0;
				else 
					sysPhaseShift.satCount = atoi(numofsat);
				sysPhaseShift.obsType[3] = '\0';
				if(sysPhaseShift.cSatSys == ' ' || sysPhaseShift.obsType[0] == ' ' || sysPhaseShift.phaseShift == DBL_MAX) // 系统标识为空
					continue;
				int               nline = sysPhaseShift.satCount / 10;	// 整行数
				int               nResidue = sysPhaseShift.satCount % 10;	
				if(sysPhaseShift.satCount <= 10)
				{
					for(BYTE i = 0; i < sysPhaseShift.satCount; i++)
					{
						char strSatName[3 + 1];
						strLine.copy(strSatName, 3, 18 + i * 4 + 1);
						strSatName[3] = '\0';
						sysPhaseShift.satList.push_back(strSatName);
					}				
				}
				else
				{
					for(BYTE i = 0; i < 10; i++)
					{
						char strSatName[3 + 1];
						strLine.copy(strSatName, 3, 18 + i * 4 + 1);
						strSatName[3] = '\0';
						sysPhaseShift.satList.push_back(strSatName);
					}
					for(int n = 1;n < nline; n++)
					{// 读取中间的整行
						fgets(line,100,pObsfile);
						strLine = line;
						for(BYTE i = 0; i < 10; i++)
						{
							char strSatName[3 + 1];
							strLine.copy(strSatName, 3, 18 + i * 4 + 1);
							strSatName[3] = '\0';
							sysPhaseShift.satList.push_back(strSatName);
						}
					}
					if(nResidue > 0)
					{
						fgets(line,100,pObsfile);
						strLine = line;
						for(BYTE i = 0; i < nResidue; i++)
						{
							char strSatName[3 + 1];
							strLine.copy(strSatName, 3, 18 + i * 4 + 1);
							strSatName[3] = '\0';
							sysPhaseShift.satList.push_back(strSatName);
						}	
					}
				}
				m_header.sysPhaseShiftList.push_back(sysPhaseShift);
			}
			else if(strLineMask == Rinex3_03_MaskString::szLeapSec)
			{  
				sscanf(line,"%6d", &m_header.leapSecond);
			}
			else if(strLineMask == Rinex3_03_MaskString::szNumsOfSat)
			{  
				sscanf(line,"%6d", &m_header.satCount);
			}			
			else if(strLineMask == Rinex3_03_MaskString::szEndOfHead)
			{
				bFlag = false;
			}
			else 
			{// 其余类型暂不作处理
			}
		}

		// 观测数据
		bFlag = TRUE;
		m_data.clear();
		int k = 0;
		char line[1000];
		fgets(line, 1000, pObsfile);
		while(bFlag)
		{
			string strLine = line;
			int nFlag = isValidEpochLine(strLine, pObsfile);
			if(nFlag == 0)      // 文件末尾
			{
				bFlag = false;
			}
			else if(nFlag == 1) // 找到新时刻的数据段
			{
				k++;				
				Rinex3_03_ObsEpoch   obsEpoch;	
				obsEpoch.obs.resize(m_header.sysObsTypeList.size()); //观测数据向量初始化
				for(size_t s_i = 0; s_i < m_header.sysObsTypeList.size(); s_i ++)
				{
					obsEpoch.obs[s_i].cSatSys = m_header.sysObsTypeList[s_i].cSatSys;
				}
				// 解析Epoch
				sscanf(strLine.c_str(),"%1c%*1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%11lf%*2c%1d%3d%*6c%15lf",
					&obsEpoch.cRecordId,
					&obsEpoch.t.year,
					&obsEpoch.t.month,
					&obsEpoch.t.day,
					&obsEpoch.t.hour,
					&obsEpoch.t.minute,
					&obsEpoch.t.second,
					&obsEpoch.byEpochFlag,
					&obsEpoch.satCount,
					&obsEpoch.clock);
				obsEpoch.cRecordId[1]='\0';

				// 整理时间标记的格式，2015-11-27
				if(obsEpoch.t.second >= 60)
				{
					int minute = int(floor(obsEpoch.t.second / 60.0));
					obsEpoch.t.second = obsEpoch.t.second - minute * 60;
					obsEpoch.t.minute += minute;
					if(obsEpoch.t.minute >= 60)
					{
						int hour = int(floor(obsEpoch.t.minute / 60.0));
						obsEpoch.t.minute = obsEpoch.t.minute - hour * 60;
						obsEpoch.t.hour += hour;
						if(obsEpoch.t.hour >= 24)
						{
							int day = int(floor(obsEpoch.t.hour / 24.0));
							obsEpoch.t.hour = obsEpoch.t.hour - day * 24;
							obsEpoch.t.day += day;
						}
					}	
				}

				if(bOn_BDT2GPST)// 20160519, 将时间系统调整为GPS时间
					obsEpoch.t = TimeCoordConvert::BDT2GPST(obsEpoch.t);

				// 根据Epoch/SAT，解析可见观测数据			
				for(int i = 0; i < obsEpoch.satCount; i++)
				{				
					char                  cSatSys;              // 卫星系统
					int                   satPRN = 0;                // 卫星编号
					char                  szSatName[3+1];            // 卫星名, C01
					int                   obsTypCount;              // 观测数据类型个数
					size_t                s_sat;
					Rinex2_1_ObsTypeList  obsTypeList;
					fgets(line, 1000, pObsfile);
					strLine = line;
					sscanf(line, "%1c%2d", &cSatSys, &satPRN);
					sscanf(line, "%3c", &szSatName);
					szSatName[3] = '\0';
					if(szSatName[1] == ' ')// 2012/10/31, 将G 1->G01
						szSatName[1] = '0';

					for(size_t s_i = 0; s_i < m_header.sysObsTypeList.size(); s_i ++)
					{
						if(m_header.sysObsTypeList[s_i].cSatSys == cSatSys)
						{
							s_sat = s_i;
							obsTypCount = m_header.sysObsTypeList[s_i].obsTypeCount;
							break;
						}
					}
					for(int j = 0; j < obsTypCount;j ++)
					{
						Rinex2_1_ObsDatum     obsDatum;						
						char strObsSatElements[17];
						char strObsValue[15];
						if(size_t(16 * j + 14) <= strLine.length())
						{
							strLine.copy(strObsSatElements, 16, 16 * j + 3);
							string  strElements = strObsSatElements;
							size_t  nPos_n = strElements.find('\n');
							if(nPos_n < strElements.length())      // 擦除回车后面的字符
							{
								strElements.erase(nPos_n, 16 - nPos_n);
								if(strElements.length() < 16)		   // 补齐16位						
									strElements.append(16 - strElements.length(),' ');
								strElements.copy(strObsSatElements, 16, 0);								
							}
							strObsSatElements[16] = '\0';
							strncpy(strObsValue, strObsSatElements, 14);							
							strObsValue[14] = '\0';
							//添加对空格无效数据的校验，当数据无效时一般会以空格填写
							if(strcmp(strObsValue,"              ") == 0)							
								obsDatum.data = DBL_MAX;												
							else
								//sscanf(strObsSatElements, "%14lf", &obsDatum.data);
								sscanf(strObsValue, "%lf", &obsDatum.data);
								sscanf(strObsSatElements, "%*14c%1c%1c", &obsDatum.lli, &obsDatum.ssi);
						}
						else
							obsDatum.data = DBL_MAX;
						obsTypeList.push_back(obsDatum);
					}
					obsEpoch.obs[s_sat].obsList.insert(Rinex3_03_SatMap::value_type(szSatName, obsTypeList));					
				}
				// 剔除观测数据为零的卫星系统
				size_t s_j  = 0;
				//int    k = 0;
				while(s_j < obsEpoch.obs.size())
				{
					//k++;					
					if(obsEpoch.obs[s_j].obsList.size() == 0)
						obsEpoch.obs.erase(obsEpoch.obs.begin() + s_j);
					else
					{
						s_j++;
						continue;
					}
				}	
				m_data.push_back(obsEpoch);
				fgets(line,1000,pObsfile);
		    }
	        else  
	        {// 无效数据行，例如空白行，掠过不作处理
				fgets(line,1000,pObsfile);
	        }
		}
		//更新初始观测时刻和最后观测时刻
		size_t nListNum = m_data.size();
		if(nListNum > 0)
		{
			m_header.tmStart = m_data[0].t;
			m_header.tmEnd   = m_data[nListNum - 1].t;
		}
		fclose(pObsfile);		
		return true;
    }
	// 子程序名称： write   
	// 功能：将观测数据写到文件 
	// 变量类型：strObsfileName_noExp     : 观测数据文件路径(后缀名程序自动生成, 输入不包含)
	//           strObsfileName_all       : 完整的观测数据文件路径
	// 输入：strObsfileName_noExp
	// 输出：strObsfileName_all
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2018/5/29
	// 修改记录：
	// 其它：    参考 Rinex3_0_ObsFile.cpp 文件和 RINEX 3.03 文件
	bool Rinex3_03_ObsFile::write(string strObsfileName_noExp, string& strObsfileName_all)
	{
		if(isEmpty())
			return false;
		// 写文件头，按照 Rinex3.03 的标准文件头书写
		int n1  = getIntBit(m_header.tmStart.year, 0);
		int n10 = getIntBit(m_header.tmStart.year, 1);
		char strFileExp[5];
		sprintf(strFileExp,".%1d%1do",n10,n1);
		strObsfileName_all = strObsfileName_noExp + strFileExp;
		FILE* pObsfile = fopen(strObsfileName_all.c_str(), "w+");
		fprintf(pObsfile,"%9.2lf%-11s%-20s%1c%-19s%20s\n", 
			m_header.rinexVersion,
			" ",
			m_header.szFileType,
			m_header.cSatSys,
			" ",
			Rinex3_03_MaskString::szVerType);
		fprintf(pObsfile,"%-20s%-20s%-20s%20s\n",
			m_header.szProgramName, 
			m_header.szProgramAgencyName, 
			m_header.szFileDate,
			Rinex3_03_MaskString::szPgmRunDate);
		fprintf(pObsfile,"%-60s%20s\n",
			m_header.szMarkName,
			Rinex3_03_MaskString::szMarkerName);	
		fprintf(pObsfile,"%-20s%-40s%20s\n",
			m_header.szMarkNumber,
			" ",
			Rinex3_03_MaskString::szMarkerNum);	
		fprintf(pObsfile,"%-20s%-40s%20s\n",
			m_header.szMarkType,
			" ",
			Rinex3_03_MaskString::szMarkerType);
		fprintf(pObsfile,"%-20s%-40s%20s\n",
			m_header.szObserverName, 
			m_header.szObserverAgencyName, 
			Rinex3_03_MaskString::szObservAgency);		
		fprintf(pObsfile,"%-20s%-20s%-20s%20s\n",
			m_header.szRecNumber,
			m_header.szRecType,
			m_header.szRecVersion,
			Rinex3_03_MaskString::szRecTypeVers);
		fprintf(pObsfile,"%-20s%-20s%-20s%20s\n",
			m_header.szAntNumber,
			m_header.szAntType,
			" ",
			Rinex3_03_MaskString::szAntType);
		if(m_header.approxPosXYZ.x != DBL_MAX)
		{
			fprintf(pObsfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
				m_header.approxPosXYZ.x,
				m_header.approxPosXYZ.y,
				m_header.approxPosXYZ.z,					 
				" ",
				Rinex3_03_MaskString::szApproxPosXYZ);
		}
		if(m_header.antDeltaHEN.x != DBL_MAX)
		{
			fprintf(pObsfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
				m_header.antDeltaHEN.x,
				m_header.antDeltaHEN.y,
				m_header.antDeltaHEN.z,					 
				" ",
				Rinex3_03_MaskString::szAntDeltaHEN);
		}
		if(m_header.antDeltaXYZ.x != DBL_MAX)
		{
			fprintf(pObsfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
				m_header.antDeltaXYZ.x,
				m_header.antDeltaXYZ.y,
				m_header.antDeltaXYZ.z,					 
				" ",
				Rinex3_03_MaskString::szAntDeltaXYZ);
		}
		for(size_t s_i = 0; s_i < m_header.phaseCenterList.size(); s_i ++)
		{
			fprintf(pObsfile,"%1c%1s%3s%9.4f%14.4f%14.4f%-18s%20s\n",
			m_header.phaseCenterList[s_i].cSatSys,
			" ",
			m_header.phaseCenterList[s_i].szObsCode,
			m_header.phaseCenterList[s_i].phaseCenter.x,
			m_header.phaseCenterList[s_i].phaseCenter.y,
			m_header.phaseCenterList[s_i].phaseCenter.z,
			" ",
			Rinex3_03_MaskString::szAntPhaseCenter);
		}
		if(m_header.antBSightXYZ.x != DBL_MAX)
		{
			fprintf(pObsfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
				m_header.antBSightXYZ.x,
				m_header.antBSightXYZ.y,
				m_header.antBSightXYZ.z,					 
				" ",
				Rinex3_03_MaskString::szAntBSightXYZ);
		}
		if(m_header.antZeroDIRAzimuth != DBL_MAX)
		{
			fprintf(pObsfile,"%14.4f%-46s%20s\n",
				m_header.antZeroDIRAzimuth,				 
				" ",
				Rinex3_03_MaskString::szZeroDIRAzimuth);
		}
		if(m_header.antZeroDIRXYZ.x != DBL_MAX)
		{
			fprintf(pObsfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
				m_header.antZeroDIRXYZ.x,
				m_header.antZeroDIRXYZ.y,
				m_header.antZeroDIRXYZ.z,					 
				" ",
				Rinex3_03_MaskString::szZeroDIRXYZ);
		}
		if(m_header.antCOMXYZ.x != DBL_MAX)
		{
			fprintf(pObsfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
				m_header.antCOMXYZ.x,
				m_header.antCOMXYZ.y,
				m_header.antCOMXYZ.z,
				" ",
				Rinex3_03_MaskString::szCOMXYZ);
		}
		for(size_t s_i = 0; s_i < m_header.sysObsTypeList.size(); s_i ++)
		{
			int obsTypes = (int)m_header.sysObsTypeList[s_i].obsTypeList.size();
			int nLine    = obsTypes / 13;  // 整行数
		    int nResidue = obsTypes % 13;  // 余数
			int nBlank   = 0;              // 空白位数
			string         strBlank; 
			fprintf(pObsfile,"%1c%-2s%3d",                         
				    m_header.sysObsTypeList[s_i].cSatSys,                          
				    " ",                                                
				    obsTypes);
			if(obsTypes <= 13)
			{
				nBlank = 60 - (6 + 4 * obsTypes);
				strBlank.append(nBlank,' ');				
				for(int i = 0;i < obsTypes;i ++)
					fprintf(pObsfile,"%-1s%3s"," ",m_header.sysObsTypeList[s_i].obsTypeList[i].c_str());
				fprintf(pObsfile,"%s%20s\n", strBlank.c_str(), Rinex3_03_MaskString::szSysObsTypes);
			}
			else
			{
				nBlank = 60 - (6 + 4 * nResidue);
				strBlank.append(nBlank,' ');
				for(int i = 0; i < 13; i ++)
					fprintf(pObsfile,"%-1s%3s"," ",m_header.sysObsTypeList[s_i].obsTypeList[i].c_str());
				fprintf(pObsfile,"%-2s%20s\n"," ",Rinex3_03_MaskString::szSysObsTypes);
				for(int n = 1; n < nLine; n ++) 
				{
					fprintf(pObsfile,"%6s"," ");
					for(int j = 0; j < 13; j ++)
						fprintf(pObsfile,"%-1s%3s"," ",m_header.sysObsTypeList[s_i].obsTypeList[13 * n + j].c_str());
					fprintf(pObsfile,"%20s\n",Rinex3_03_MaskString::szSysObsTypes);
				}
				if(nResidue > 0)
				{
					fprintf(pObsfile,"%-6s"," ");
					for(int i = 0; i < nResidue; i ++)
						fprintf(pObsfile,"%-1s%3s"," ",m_header.sysObsTypeList[s_i].obsTypeList[13 * nLine + i].c_str());
				}
				fprintf(pObsfile,"%s%20s\n", strBlank.c_str(), Rinex3_03_MaskString::szSysObsTypes);
			}			
		}
		fprintf(pObsfile,"%-20s%40s%20s\n",
			m_header.szSignalStrengthUnit,
			" ",			
			Rinex3_03_MaskString::szSignalStrengthUnit);		
		if(m_header.interval != DBL_MAX)
		{
			fprintf(pObsfile,"%10.3f%50s%20s\n",
				m_header.interval,
				" ",			
				Rinex3_03_MaskString::szInterval);
		}
		fprintf(pObsfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
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
		fprintf(pObsfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
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
		if(m_header.rcvClockOffsetApplied != INT_MAX)
		{
			fprintf(pObsfile,"%6d%54s%20s\n",
				m_header.rcvClockOffsetApplied,
				" ",			
				Rinex3_03_MaskString::szRCVClockOffsetApplied);
		}
		for(size_t s_i = 0; s_i < m_header.sysDCBSAppliedList.size(); s_i ++)
		{
			fprintf(pObsfile,"%1c%1s%17s%1s%40s%20s\n",
				m_header.sysDCBSAppliedList[s_i].cSatSys,
				" ",
				m_header.sysDCBSAppliedList[s_i].szNameProgram,
				m_header.sysDCBSAppliedList[s_i].szNameURL,			
				Rinex3_03_MaskString::szSysDCBSApplied);
		}
		for(size_t s_i = 0; s_i < m_header.sysPCVSAppliedList.size(); s_i ++)
		{
			fprintf(pObsfile,"%1c%1s%17s%1s%40s%20s\n",
				m_header.sysPCVSAppliedList[s_i].cSatSys,
				" ",
				m_header.sysPCVSAppliedList[s_i].szNameProgram,
				m_header.sysPCVSAppliedList[s_i].szNameURL,			
				Rinex3_03_MaskString::szSysPCVSApplied);
		}
		for(size_t s_i = 0; s_i < m_header.sysScaleFactorList.size(); s_i ++)
		{
			int obsTypes = (int)m_header.sysScaleFactorList[s_i].obsTypeList.size();
			int nLine    = obsTypes / 13;  // 整行数
		    int nResidue = obsTypes % 13;  // 余数
			int nBlank   = 0;              // 空白位数
			string         strBlank; 
			fprintf(pObsfile,"%1c%-2s%3d",                         
				    m_header.sysScaleFactorList[s_i].cSatSys,                          
				    " ",                                                
				    obsTypes);
			if(obsTypes <= 13)
			{
				nBlank = 60 - (6 + 4 * obsTypes);
				strBlank.append(nBlank,' ');				
				for(int i = 0;i < obsTypes;i ++)
					fprintf(pObsfile,"%c%3s"," ",m_header.sysScaleFactorList[s_i].obsTypeList[i].c_str());
				fprintf(pObsfile,"%s%20s\n", strBlank.c_str(), Rinex3_03_MaskString::szSysScaleFactor);
			}
			else
			{
				nBlank = 60 - (6 + 4 * nResidue);
				strBlank.append(nBlank,' ');
				for(int i = 0; i < 13; i ++)
					fprintf(pObsfile,"%-1s%3s"," ",m_header.sysScaleFactorList[s_i].obsTypeList[i].c_str());
				fprintf(pObsfile,"%-2s%20s\n"," ",Rinex3_03_MaskString::szSysScaleFactor);
				for(int n = 1; n < nLine; n ++) 
				{
					fprintf(pObsfile,"%6s"," ");
					for(int j = 0; j < 13; j ++)
						fprintf(pObsfile,"%-1s%3s"," ",m_header.sysScaleFactorList[s_i].obsTypeList[13 * n + j].c_str());
					fprintf(pObsfile,"%20s\n",Rinex3_03_MaskString::szSysScaleFactor);
				}
				if(nResidue > 0)
				{
					fprintf(pObsfile,"%-10s"," ");
					for(int i = 0; i < nResidue; i ++)
						fprintf(pObsfile,"%-1s%3s"," ",m_header.sysScaleFactorList[s_i].obsTypeList[13 * nLine + i].c_str());
				}
				fprintf(pObsfile,"%s%20s\n", strBlank.c_str(), Rinex3_03_MaskString::szSysScaleFactor);
			}			
		}
		if(m_header.leapSecond != INT_MAX)
		{
			fprintf(pObsfile,"%6d%54s%20s\n",
				m_header.leapSecond,
				" ",			
				Rinex3_03_MaskString::szLeapSec);
		}
		if(m_header.satCount != INT_MAX)
		{
			fprintf(pObsfile,"%6d%54s%20s\n",
					m_header.satCount,
					" ",			
					Rinex3_03_MaskString::szNumsOfSat);
		}
		fprintf(pObsfile,"%-60s%20s\n"," ",Rinex3_03_MaskString::szEndOfHead);
		// 文件头书写完毕

		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			Rinex3_03_ObsEpoch obsEpoch = m_data[s_i];
			//Epoch/SAT--1--32
			fprintf(pObsfile,"%-2s%4d%1s%02d%1s%02d%1s%02d%1s%02d%11.7lf%-2s%1d%3d%-6s%15.12lf%4s\n",
				obsEpoch.cRecordId,
				obsEpoch.t.year,
				" ",
				obsEpoch.t.month,
				" ",
				obsEpoch.t.day,
				" ",
				obsEpoch.t.hour,
				" ",
				obsEpoch.t.minute,
				obsEpoch.t.second,
				" ",
				obsEpoch.byEpochFlag,
				obsEpoch.satCount,
				" ",
				obsEpoch.clock,				
				" ");		
		    // 根据Epoch/SAT，解析可见观测数据--------------------
			for(size_t s_j = 0; s_j < obsEpoch.obs.size(); s_j ++)
			{				
				for(Rinex3_03_SatMap::iterator it = obsEpoch.obs[s_j].obsList.begin(); it != obsEpoch.obs[s_j].obsList.end(); ++it)
				{
					//fprintf(pObsfile,"%1s%02d",obsEpoch.obs[s_j].cSatSys,it->first);
					fprintf(pObsfile,"%3s",it->first.c_str());
					int ObsTypes = 0;
					for(size_t s_k = 0; s_k < m_header.sysObsTypeList.size(); s_k ++)
					{
						if(m_header.sysObsTypeList[s_k].cSatSys == obsEpoch.obs[s_j].cSatSys)
						{							
							ObsTypes = m_header.sysObsTypeList[s_k].obsTypeCount;
							break;
						}
					}
					for(int i = 0; i < ObsTypes; i++)
					{
						Rinex2_1_ObsDatum obsDatum = it->second[i];						
						if(obsDatum.data != DBL_MAX)
							fprintf(pObsfile,"%14.3f%c%c",							                 
								obsDatum.data,
								obsDatum.lli,
								obsDatum.ssi);
						else
							fprintf(pObsfile,"%-16s"," ");
					}
					fprintf(pObsfile,"\n");
				}
			}			
        }
		fclose(pObsfile);
		return true;
	}
	bool Rinex3_03_ObsFile::write(string strObsfileName_noExp)
	{
		string strObsfileName_all;
		return write(strObsfileName_noExp, strObsfileName_all);
	}    
}
