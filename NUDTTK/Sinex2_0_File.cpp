#include "Sinex2_0_File.hpp"

namespace NUDTTK
{
	//const char Sinex2_0_BlockLabel::szFileID[]          = "%=SNX";
	const char Sinex2_0_BlockLabel::szFileRef[]              = "FILE/REFERENCE                   ";
	const char Sinex2_0_BlockLabel::szFileComment[]          = "FILE/COMMENT                     ";
	const char Sinex2_0_BlockLabel::szInputHistory[]         = "INPUT/HISTORY                    ";
	const char Sinex2_0_BlockLabel::szInputFiles[]           = "INPUT/FILES                      ";
	const char Sinex2_0_BlockLabel::szInputAck[]             = "INPUT/ACKNOWLEDGEMENTS           ";
	const char Sinex2_0_BlockLabel::szInputAck_NOE[]         = "INPUT/ACKNOWLEDGMENTS            ";
	const char Sinex2_0_BlockLabel::szNutationData[]         = "NUTATION/DATA                    ";
	const char Sinex2_0_BlockLabel::szPrecessionData[]       = "PRECESSION/DATA                  ";
	const char Sinex2_0_BlockLabel::szSourceID[]             = "SOURCE/ID                        ";
	const char Sinex2_0_BlockLabel::szSiteID[]               = "SITE/ID                          ";
	const char Sinex2_0_BlockLabel::szSiteData[]             = "SITE/DATA                        ";
	const char Sinex2_0_BlockLabel::szSiteRec[]              = "SITE/RECEIVER                    ";
	const char Sinex2_0_BlockLabel::szSiteAnt[]              = "SITE/ANTENNA                     ";
	const char Sinex2_0_BlockLabel::szSiteGPSPCO[]           = "SITE/GPS_PHASE_CENTER            ";
	const char Sinex2_0_BlockLabel::szSiteGALPCO[]           = "SITE/GAL_PHASE_CENTER            ";
	const char Sinex2_0_BlockLabel::szSiteAntARP[]           = "SITE/ECCENTRICITY                ";        
	const char Sinex2_0_BlockLabel::szSatID[]                = "SATELLITE/ID                     ";     
	const char Sinex2_0_BlockLabel::szSatPCO[]               = "SATELLITE/PHASE_CENTER           ";  
	const char Sinex2_0_BlockLabel::szSolutEpochs[]          = "SOLUTION/EPOCHS                  "; 
	const char Sinex2_0_BlockLabel::szBiasEpochs[]           = "BIAS/EPOCHS                      "; 
	const char Sinex2_0_BlockLabel::szSolutStatistics[]      = "SOLUTION/STATISTICS              "; 
	const char Sinex2_0_BlockLabel::szSolutEst[]             = "SOLUTION/ESTIMATE                "; 
	const char Sinex2_0_BlockLabel::szSolutApri[]            = "SOLUTION/APRIORI                 "; 
	const char Sinex2_0_BlockLabel::szSolutMatrixEstLCORR[]  = "SOLUTION/MATRIX_ESTIMATE L CORR  "; 
	const char Sinex2_0_BlockLabel::szSolutMatrixEstUCORR[]  = "SOLUTION/MATRIX_ESTIMATE U CORR  "; 
	const char Sinex2_0_BlockLabel::szSolutMatrixEstLCOVA[]  = "SOLUTION/MATRIX_ESTIMATE L COVA  "; 
	const char Sinex2_0_BlockLabel::szSolutMatrixEstUCOVA[]  = "SOLUTION/MATRIX_ESTIMATE U COVA  ";
	const char Sinex2_0_BlockLabel::szSolutMatrixEstLINFO[]  = "SOLUTION/MATRIX_ESTIMATE L INFO  "; 
	const char Sinex2_0_BlockLabel::szSolutMatrixEstUINFO[]  = "SOLUTION/MATRIX_ESTIMATE U INFO  ";
	const char Sinex2_0_BlockLabel::szSolutMatrixApriLCORR[] = "SOLUTION/MATRIX_APRIORI L CORR   "; 
	const char Sinex2_0_BlockLabel::szSolutMatrixApriUCORR[] = "SOLUTION/MATRIX_APRIORI U CORR   ";
	const char Sinex2_0_BlockLabel::szSolutMatrixApriLCOVA[] = "SOLUTION/MATRIX_APRIORI L COVA   ";
	const char Sinex2_0_BlockLabel::szSolutMatrixApriUCOVA[] = "SOLUTION/MATRIX_APRIORI U COVA   "; 
	const char Sinex2_0_BlockLabel::szSolutMatrixApriLINFO[] = "SOLUTION/MATRIX_APRIORI L INFO   "; 
	const char Sinex2_0_BlockLabel::szSolutMatrixApriUINFO[] = "SOLUTION/MATRIX_APRIORI U INFO   ";
	const char Sinex2_0_BlockLabel::szSolutNEQVector[]       = "SOLUTION/NORMAL_EQUATION_VECTOR  "; 
	const char Sinex2_0_BlockLabel::szSolutNEQMatrixL[]      = "SOLUTION/NORMAL_EQUATION_MATRIX L";
	const char Sinex2_0_BlockLabel::szSolutNEQMatrixU[]      = "SOLUTION/NORMAL_EQUATION_MATRIX U";
	const char Sinex2_0_BlockLabel::szBlockSeparator[]       = "*-------------------------------------------------------------------------------";
	const char Sinex2_0_BlockLabel::szTroStaCoordinate[]     = "TROP/STA_COORDINATES             ";
	const char Sinex2_0_BlockLabel::szTroDescription[]       = "TROP/DESCRIPTION                 ";
	const char Sinex2_0_BlockLabel::szTroSolution[]          = "TROP/SOLUTION                    ";
	const char Sinex2_0_BlockLabel::szTroCentersInfo[]       = "CENTERS/INFO_SOLUTION            ";
	const char Sinex2_0_BlockLabel::szTroCentersModel[]      = "CENTERS/INFO_MODEL               ";
	//const char Sinex2_0_BlockLabel::zFileFooter[];

	//File/Reference Block 的常量部分
	const char Sinex2_0_BlockLabel::szDes[]                  = "DESCRIPTION       ";
	const char Sinex2_0_BlockLabel::szOutput[]               = "OUTPUT            ";
	const char Sinex2_0_BlockLabel::szInput[]                = "INPUT             ";
	const char Sinex2_0_BlockLabel::szContact[]              = "CONTACT           ";
	const char Sinex2_0_BlockLabel::szSoftware[]             = "SOFTWARE          ";
	const char Sinex2_0_BlockLabel::szHardware[]             = "HARDWARE          ";
	const char Sinex2_0_BlockLabel::szRefFrame[]             = "REFERENCE_FRAME   ";
     
	//SolutStatistics Block 的常量部分
	const char Sinex2_0_BlockLabel::szObsCount[]              = "NUMBER OF OBSERVATIONS        ";
	const char Sinex2_0_BlockLabel::szObsUnknown[]            = "NUMBER OF UNKNOWNS            ";
	const char Sinex2_0_BlockLabel::szObsFreedom[]            = "NUMBER OF DEGREES OF FREEDOM  ";//自由度
	const char Sinex2_0_BlockLabel::szFreedom[]               = "DEGREES OF FREEDOM            ";//自由度
	const char Sinex2_0_BlockLabel::szSampInterval[]          = "SAMPLING INTERVAL             ";
	const char Sinex2_0_BlockLabel::szSampIntervalUnit[]      = "SAMPLING INTERVAL (SECONDS)   ";
	const char Sinex2_0_BlockLabel::szSquareRes[]             = "SQUARE SUM OF RESIDUALS       ";
	const char Sinex2_0_BlockLabel::szSquaredRes[]            = "SQUARED SUM OF RESIDUALS      ";
	const char Sinex2_0_BlockLabel::szPhaseSigma[]            = "PHASE MEASUREMENTS SIGMA      ";
	const char Sinex2_0_BlockLabel::szCodeSigma[]             = "CODE MEASUREMENTS SIGMA       ";
	const char Sinex2_0_BlockLabel::szVarFactor[]             = "VARIANCE FACTOR               ";
	const char Sinex2_0_BlockLabel::szWSquareRes[]            = "WEIGHTED SQUARE SUM OF O-C    ";

	const char Sinex2_0_BlockLabel::szTROSampInterval[]       = "SAMPLING INTERVAL            ";
	const char Sinex2_0_BlockLabel::szTroInterval[]      	  = "SAMPLING TROP                ";
	const char Sinex2_0_BlockLabel::szEleCutoff[]        	  = "ELEVATION CUTOFF ANGLE       ";
	const char Sinex2_0_BlockLabel::szTroMapFunc[]       	  = "TROP MAPPING FUNCTION        ";
	const char Sinex2_0_BlockLabel::szSolutField1[]      	  = "SOLUTION_FIELDS_1            ";
	const char Sinex2_0_BlockLabel::szSolutField2[]      	  = "SOLUTION_FIELDS_2            ";//

   
	// 子程序名称： SinexFileTime::toUTC  
	// 功能：将SNX文件时间转为UTC时间
	// 变量类型：
	// 输入：
	// 输出：
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/12
	// 版本时间：2014/09/12
	// 修改记录：
	// 备注： 
	UTC SinexFileTime::toUTC()
	{		
		int tyear;
		if(year <= 50)
			tyear = year + 2000;
		else
			tyear = year + 1900;
		UTC   t0(tyear,1,1,0,0,0);		
		return (t0 + (doy - 1) * 86400.0 + second);
	}


	Sinex2_0_File::Sinex2_0_File(void)
	{
	}

	Sinex2_0_File::~Sinex2_0_File(void)
	{
	}
	void  Sinex2_0_File::clear()
	{
		m_header = Sinex2_0_HeaderLine::Sinex2_0_HeaderLine();
		m_data   = Sinex2_0_Data::Sinex2_0_Data();
	}
	bool Sinex2_0_File::isEmpty()
	{
		if(m_data.m_BiasEpoch.size()		> 0
		 ||m_data.m_Comment.size()			> 0		
		 ||m_data.m_InputAck.size()			> 0
		 ||m_data.m_InputFile.size()		> 0
		 ||m_data.m_InputHistory.size()		> 0		
	     ||m_data.m_Nutaion.size()          > 0
		 ||m_data.m_Precession.size()       > 0
		 ||m_data.m_SatID.size()            > 0
		 ||m_data.m_SatPCO.size()           > 0
		 ||m_data.m_SiteAnt.size()          > 0
		 ||m_data.m_SiteAntARP.size()       > 0
		 ||m_data.m_SiteData.size()         > 0
		 ||m_data.m_SiteGALPCO.size()       > 0
		 ||m_data.m_SiteGPSPCO.size()       > 0
		 ||m_data.m_SiteID.size()           > 0
		 ||m_data.m_SiteRec.size()          > 0
		 ||m_data.MatrixApriType            > 0
		 ||m_data.MatrixEstType             > 0
		 ||m_data.MatrixNEQType             > 0)
			return false;
		else
			return true;
	}
	// 子程序名称： stringEraseFirstZero   
	// 作用： 擦除纯小数的第一个零, 例如 0.123 ----> .123; -0.123 ----> -.123
	// 变量类型：strFloat       : 字符串
	// 输入：strFloat
	// 输出：
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/09
	// 版本时间：
	// 修改记录：
	// 备注：
	void    Sinex2_0_File::stringEraseFirstZero(const char* szFloat, string& strFloat)
	{
		strFloat = szFloat;
		size_t nPos;
		for(size_t i = 0; i < 2; i ++)
		{
			if(szFloat[i] == '0')
				nPos = i;
		}
		strFloat.erase(nPos, 1);
	}

	// 子程序名称： open   
	// 功能：SINEX文件解析 
	// 变量类型：strSNXFileName : SINEX文件路径
	// 输入：strSNXFileName
	// 输出：
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/07
	// 版本时间：2014/09/07
	// 修改记录：
	// 备注： 
	bool Sinex2_0_File::open(string strSNXFileName)
	{
		if(!isWildcardMatch(strSNXFileName.c_str(), "*.snx", true) || !isWildcardMatch(strSNXFileName.c_str(), "*.SNX", true) )
		{
			printf(" %s 文件名不匹配!\n", strSNXFileName.c_str());
			return false;
		}

		FILE * pSNXfile = fopen(strSNXFileName.c_str(),"r+t");
		if(pSNXfile == NULL) 
			return false;
		m_header = Sinex2_0_HeaderLine::Sinex2_0_HeaderLine();
		// 开始读取第一行
		char line[200];
		fgets(line,100,pSNXfile);	
		string strFirstLine = line;				
		size_t nPos_n = strFirstLine.find('\n');
		strFirstLine.erase(nPos_n, 1);
		// 补齐80位
		if(strFirstLine.length() < 80) // strLine.length 包含最后的'\0'
			strFirstLine.append(80 - strFirstLine.length(),' ');
		sscanf(strFirstLine.c_str(),"%1c%1c%3c%*1c%4lf%*1c%3c%*1c%2d%*1c%3d%*1c%5d%*1c%3c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%1c%*1c%5d%*1c%1c%*1c%1c%*1c%1c%*1c%1c%*1c%1c%*1c%1c%*1c%1c",
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
							   &m_header.EstParaCount,
							   &m_header.szConstCode,
							   &m_header.pszSolutCont[0],
							   &m_header.pszSolutCont[1],
							   &m_header.pszSolutCont[2],
							   &m_header.pszSolutCont[3],
							   &m_header.pszSolutCont[4],
							   &m_header.pszSolutCont[5]);
		if(m_header.szFirstchar[0] != '%'|| m_header.szSecondchar[0] != '=')
		{
			printf(" %s SNX文件第一行有误！\n",strSNXFileName.c_str());
			fclose(pSNXfile);
			return false;
		}
		// 开始循环读取每一行数据，直到 End of SINEX(%ENDSNX)
		int bFlag = true;
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
				else if(strLineLabel == Sinex2_0_BlockLabel::szInputHistory) 
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
							Sinex2_0_HeaderLine     headline;
							sscanf(strLine.c_str(),"%*1c%1c%3c%*1c%4lf%*1c%3c%*1c%2d%*1c%3d%*1c%5d%*1c%3c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%1c%*1c%5d%*1c%1c%*1c%1c%*1c%1c%*1c%1c%*1c%1c%*1c%1c%*1c%1c",
												   &headline.szSecondchar,
												   &headline.szDocType,
												   &headline.Version,
												   &headline.szFileAgency,
												   &headline.FileTime.year,
												   &headline.FileTime.doy,
												   &headline.FileTime.second,
												   &headline.szDataAgency,
												   &headline.StartTimeSolut.year,
												   &headline.StartTimeSolut.doy,
												   &headline.StartTimeSolut.second,
												   &headline.EndTimeSolut.year,
												   &headline.EndTimeSolut.doy,
												   &headline.EndTimeSolut.second,
												   &headline.szObsCode,
												   &headline.EstParaCount,
												   &headline.szConstCode,
												   &headline.pszSolutCont[0],
												   &headline.pszSolutCont[1],
												   &headline.pszSolutCont[2],
												   &headline.pszSolutCont[3],
												   &headline.pszSolutCont[4],
												   &headline.pszSolutCont[5]);
							m_data.m_InputHistory.push_back(headline);
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szInputFiles) 
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
							InputFile     inputFile;
							sscanf(strLine.c_str(),"%*1c%3c%*1c%2d%*1c%3d%*1c%5d%*1c%29c%*1c%32c",
												  &inputFile.szFileAgency,
												  &inputFile.FileTime.year,
												  &inputFile.FileTime.doy,
												  &inputFile.FileTime.second,
												  &inputFile.szFileName,
												  &inputFile.szFileDes);
							m_data.m_InputFile.push_back(inputFile);
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
				else if(strLineLabel == Sinex2_0_BlockLabel::szNutationData) 
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
							Nut_Pre      nut_Pre;
							sscanf(strLine.c_str(),"%*1c%8c%*1c%70c",
												  &nut_Pre.szModelCode,										  
												  &nut_Pre.szModelDes);
							m_data.m_Nutaion.push_back(nut_Pre);
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szPrecessionData) 
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
							Nut_Pre      nut_Pre;
							sscanf(strLine.c_str(),"%*1c%8c%*1c%70c",
												  &nut_Pre.szModelCode,										  
												  &nut_Pre.szModelDes);
							m_data.m_Precession.push_back(nut_Pre);
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSourceID) 
				{//VLBI的数据模块
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
							SourceID      sourceID;
							sscanf(strLine.c_str(),"%*1c%4c%*1c%8c%*1c%16c%*1c%68c",
												  &sourceID.szSourceCode,										  
												  &sourceID.szIERSDes,
												  &sourceID.szICRFDes,
												  &sourceID.szComments);
							m_data.m_SourceID.push_back(sourceID);
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
				else if(strLineLabel == Sinex2_0_BlockLabel::szSiteData) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							SiteData       siteData;
							sscanf(line,"%*1c%4c%*1c%2c%*1c%4c%*1c%4c%*1c%2c%*1c%4c%*1c%1c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%3c%*1c%2d%*1c%3d%*1c%5d",
								          &siteData.szSiteCode,										  
										  &siteData.szPointCode,
										  &siteData.szSolutID,
										  &siteData.szSiteCodeIn,
										  &siteData.szPointCodeIn,
										  &siteData.szSolutIDIn,
										  &siteData.szObsCodeIn,
										  &siteData.StartTimeSolutIn.year,
										  &siteData.StartTimeSolutIn.doy,
										  &siteData.StartTimeSolutIn.second,
										  &siteData.EndTimeSolutIn.year,
										  &siteData.EndTimeSolutIn.doy,
										  &siteData.EndTimeSolutIn.second,
										  &siteData.szAgencyCodeIn,
										  &siteData.FileTimeIn.year,
										  &siteData.FileTimeIn.doy,
										  &siteData.FileTimeIn.second);
							m_data.m_SiteData.push_back(siteData);
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
				else if(strLineLabel == Sinex2_0_BlockLabel::szSiteGALPCO) 
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
						    BYTE          L5Index = 2; //代表L5
							ENU           PCOL5;
							sscanf(strLine.c_str(),"%*1c%20c%*1c%5c%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%10c",
												  &sitePCO.szAntType,										  
												  &sitePCO.szAntSerial,
												  &PCOL1.U,
												  &PCOL1.N,
												  &PCOL1.E,
												  &PCOL5.U,
												  &PCOL5.N,
												  &PCOL5.E,										 
												  &sitePCO.PCVCorModel);
							sitePCO.PCO.insert(RecFreqPCOMap::value_type(L1Index,PCOL1));
							sitePCO.PCO.insert(RecFreqPCOMap::value_type(L5Index,PCOL5));
							sitePCO.szAntType[20]    = '\0';
							sitePCO.szAntSerial[5]   = '\0';
							sitePCO.PCVCorModel[10]  = '\0';

							m_data.m_SiteGALPCO.insert(RecAntPCOMap::value_type(sitePCO.szAntType,sitePCO));
						}
						else if(line[0] == '-')
							break;
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
							BYTE          L6Index = 6; //代表L6
							ENU           PCOL6;
						    BYTE          L7Index = 7; //代表L7
							ENU           PCOL7;
							sscanf(strLine.c_str(),"%*1c%20c%*1c%5c%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%10c",
												  &sitePCO.szAntType,										  
												  &sitePCO.szAntSerial,
												  &PCOL6.U,
												  &PCOL6.N,
												  &PCOL6.E,
												  &PCOL7.U,
												  &PCOL7.N,
												  &PCOL7.E,										 
												  &sitePCO.PCVCorModel);
							sitePCO.PCO.insert(RecFreqPCOMap::value_type(L6Index,PCOL6));
							sitePCO.PCO.insert(RecFreqPCOMap::value_type(L7Index,PCOL7));
							sitePCO.szAntType[20]    = '\0';
							sitePCO.szAntSerial[5]   = '\0';
							sitePCO.PCVCorModel[10]  = '\0';

							m_data.m_SiteGALPCO.insert(RecAntPCOMap::value_type(sitePCO.szAntType,sitePCO));
						}
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
							BYTE          L8Index = 8; //代表L8
							ENU           PCOL8;
						    BYTE          LXIndex = 0; //暂时空缺
							ENU           PCOLX;
							sscanf(strLine.c_str(),"%*1c%20c%*1c%5c%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%10c",
												  &sitePCO.szAntType,										  
												  &sitePCO.szAntSerial,
												  &PCOL8.U,
												  &PCOL8.N,
												  &PCOL8.E,
												  &PCOLX.U,
												  &PCOLX.N,
												  &PCOLX.E,										 
												  &sitePCO.PCVCorModel);
							sitePCO.PCO.insert(RecFreqPCOMap::value_type(L8Index,PCOL8));
							//sitePCO.PCO.insert(RecFreqPCOMap::value_type(LXIndex,PCOLX));
							sitePCO.szAntType[20]    = '\0';
							sitePCO.szAntSerial[5]   = '\0';
							sitePCO.PCVCorModel[10]  = '\0';

							m_data.m_SiteGALPCO.insert(RecAntPCOMap::value_type(sitePCO.szAntType,sitePCO));
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
				else if(strLineLabel == Sinex2_0_BlockLabel::szSatID) 
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
							SatID       satID;
							sscanf(strLine.c_str(),"%*1c%4c%*1c%2c%*1c%9c%*1c%1c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%20c%",
												  &satID.szSiteCode,										  
												  &satID.szPRN,
												  &satID.szCorparID,
												  &satID.szObsCode,
												  &satID.StartTimeSolut.year,
												  &satID.StartTimeSolut.doy,
												  &satID.StartTimeSolut.second,
												  &satID.EndTimeSolut.year,
												  &satID.EndTimeSolut.doy,
												  &satID.EndTimeSolut.second,
												  &satID.szAntType);
							char szSysPRN[3 + 1];
							sprintf(szSysPRN,"%1c%2s",satID.szSiteCode[0],satID.szPRN);
							szSysPRN[3] = '\0';
							m_data.m_SatID.insert(SatIDMap::value_type(szSysPRN,satID));
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSatPCO) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							SatPCO        satPCO;
							int           F1Index; //第一个频点
							POS3D         PCOF1;
						    int           F2Index; //第二个频点
							POS3D         PCOF2;
							sscanf(line,"%*1c%4c%*1c%1d%*1c%6lf%*1c%6lf%*1c%6lf%*1c%1d%*1c%6lf%*1c%6lf%*1c%6lf%*1c%10c%*1c%1c%*1c%1c",
								          &satPCO.szSiteCode,										  
										  &F1Index,
										  &PCOF1.z,
										  &PCOF1.x,
										  &PCOF1.y,
										  &F2Index,
										  &PCOF2.z,
										  &PCOF2.x,
										  &PCOF2.y,										 
										  &satPCO.PCVCorModel,
										  &satPCO.PCVType,
										  &satPCO.PCVAppModel);
							satPCO.PCO.insert(SatFrePCOMap::value_type(F1Index,PCOF1));
							satPCO.PCO.insert(SatFrePCOMap::value_type(F2Index,PCOF2));
							satPCO.szSiteCode[4]     = '\0';
							satPCO.PCVCorModel[10]   = '\0';
							satPCO.PCVType[1]        = '\0';
							satPCO.PCVAppModel[1]    = '\0';

							m_data.m_SatPCO.insert(SatPCOMap::value_type(satPCO.szSiteCode,satPCO));
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSolutEpochs) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							SiteRecAntARPEpoch       solutEpoch;
							sscanf(line,"%*1c%4c%*1c%2c%*1c%4c%*1c%1c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d",
								          &solutEpoch.szSiteCode,										  
										  &solutEpoch.szPointCode,
										  &solutEpoch.szSolutID,
										  &solutEpoch.szObsCode,
										  &solutEpoch.StartTimeSolut.year,
										  &solutEpoch.StartTimeSolut.doy,
										  &solutEpoch.StartTimeSolut.second,
										  &solutEpoch.EndTimeSolut.year,
										  &solutEpoch.EndTimeSolut.doy,
										  &solutEpoch.EndTimeSolut.second,
										  &solutEpoch.MeanTime.year,
										  &solutEpoch.MeanTime.doy,
										  &solutEpoch.MeanTime.second);

							m_data.m_SolutEpoch.insert(SiteRecAntARPEpochMap::value_type(solutEpoch.szSiteCode,solutEpoch));
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szBiasEpochs) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							SiteRecAntARPEpoch       biasEpoch;
							sscanf(line,"%*1c%4c%*1c%2c%*1c%4c%*1c%1c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d",
								          &biasEpoch.szSiteCode,										  
										  &biasEpoch.szPointCode,
										  &biasEpoch.szSolutID,
										  &biasEpoch.szObsCode,
										  &biasEpoch.StartTimeSolut.year,
										  &biasEpoch.StartTimeSolut.doy,
										  &biasEpoch.StartTimeSolut.second,
										  &biasEpoch.EndTimeSolut.year,
										  &biasEpoch.EndTimeSolut.doy,
										  &biasEpoch.EndTimeSolut.second,
										  &biasEpoch.MeanTime.year,
										  &biasEpoch.MeanTime.doy,
										  &biasEpoch.MeanTime.second);

							m_data.m_BiasEpoch.insert(SiteRecAntARPEpochMap::value_type(biasEpoch.szSiteCode,biasEpoch));
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSolutStatistics) 
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
							char     szInfoType[30 + 1];
							string   strInfoType;
							sscanf(strLine.c_str(),"%*1c%30c",szInfoType);
							szInfoType[30] = '\0';
							strInfoType = szInfoType;
							if(strInfoType == Sinex2_0_BlockLabel::szObsCount)
								sscanf(strLine.c_str(),"%*32c%22d",&m_data.m_SolutSatistics.obsCount);
							else if(strInfoType == Sinex2_0_BlockLabel::szObsUnknown)							
								sscanf(strLine.c_str(),"%*32c%22d",&m_data.m_SolutSatistics.unknowns);
							else if(strInfoType == Sinex2_0_BlockLabel::szObsFreedom || strInfoType == Sinex2_0_BlockLabel::szFreedom)							
								sscanf(strLine.c_str(),"%*32c%22d",&m_data.m_SolutSatistics.freedoms);
							else if(strInfoType == Sinex2_0_BlockLabel::szSampInterval || strInfoType == Sinex2_0_BlockLabel::szSampIntervalUnit)
								sscanf(strLine.c_str(),"%*32c%22d",&m_data.m_SolutSatistics.sampInterval);
							else if(strInfoType == Sinex2_0_BlockLabel::szPhaseSigma)
								sscanf(strLine.c_str(),"%*32c%22lf",&m_data.m_SolutSatistics.phaseSigma);
							else if(strInfoType == Sinex2_0_BlockLabel::szCodeSigma)
								sscanf(strLine.c_str(),"%*32c%22lf",&m_data.m_SolutSatistics.codeSigma);
							else if(strInfoType == Sinex2_0_BlockLabel::szSquareRes || strInfoType == Sinex2_0_BlockLabel::szSquaredRes)
								sscanf(strLine.c_str(),"%*32c%22lf",&m_data.m_SolutSatistics.squareRes);
							else if(strInfoType == Sinex2_0_BlockLabel::szVarFactor)
								sscanf(strLine.c_str(),"%*32c%22lf",&m_data.m_SolutSatistics.varFactor);
							else if(strInfoType == Sinex2_0_BlockLabel::szWSquareRes)
								sscanf(strLine.c_str(),"%*32c%22lf",&m_data.m_SolutSatistics.wSquareRes);
							m_data.m_SolutSatistics.bBlockUse = true;														
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSolutEst) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							SolutVector       solutVector;
							sscanf(line,"%*1c%5d%*1c%6c%*1c%4c%*1c%2c%*1c%4c%*1c%2d%*1c%3d%*1c%5d%*1c%4c%*1c%1c%*1c%21lf%*1c%11lf",
								          &solutVector.Index,
										  &solutVector.ParaType,
								          &solutVector.szSiteCode,										  
										  &solutVector.szPointCode,
										  &solutVector.szSolutID,										 
										  &solutVector.EpochTime.year,
										  &solutVector.EpochTime.doy,
										  &solutVector.EpochTime.second,
										  &solutVector.Unit,
										  &solutVector.szConstCode,
										  &solutVector.ParaValue,
										  &solutVector.ParaSTD);									

							m_data.m_SolutEst.push_back(solutVector);
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSolutApri) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							SolutVector       solutVector;
							sscanf(line,"%*1c%5d%*1c%6c%*1c%4c%*1c%2c%*1c%4c%*1c%2d%*1c%3d%*1c%5d%*1c%4c%*1c%1c%*1c%21lf%*1c%11lf",
								          &solutVector.Index,
										  &solutVector.ParaType,
								          &solutVector.szSiteCode,										  
										  &solutVector.szPointCode,
										  &solutVector.szSolutID,										 
										  &solutVector.EpochTime.year,
										  &solutVector.EpochTime.doy,
										  &solutVector.EpochTime.second,
										  &solutVector.Unit,
										  &solutVector.szConstCode,
										  &solutVector.ParaValue,
										  &solutVector.ParaSTD);									

							m_data.m_SolutApri.push_back(solutVector);
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSolutNEQVector) 
				{
					do
					{
						fgets(line,100,pSNXfile);
						if(line[0] == ' ')
						{//有效数据行
							SolutVector       solutVector;
							sscanf(line,"%*1c%5d%*1c%6c%*1c%4c%*1c%2c%*1c%4c%*1c%2d%*1c%3d%*1c%5d%*1c%4c%*1c%1c%*1c%21lf",
								          &solutVector.Index,
										  &solutVector.ParaType,
								          &solutVector.szSiteCode,										  
										  &solutVector.szPointCode,
										  &solutVector.szSolutID,										 
										  &solutVector.EpochTime.year,
										  &solutVector.EpochTime.doy,
										  &solutVector.EpochTime.second,
										  &solutVector.Unit,
										  &solutVector.szConstCode,
										  &solutVector.ParaValue);									

							m_data.m_SolutNEQVector.push_back(solutVector);
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstLCORR
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstUCORR
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstLCOVA
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstUCOVA
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstLINFO
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstUINFO) 
				{
					if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstLCORR)
						m_data.MatrixEstType = 1;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstUCORR)
						m_data.MatrixEstType = 2;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstLCOVA)
						m_data.MatrixEstType = 3;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstUCOVA)
						m_data.MatrixEstType = 4;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstLINFO)
						m_data.MatrixEstType = 5;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixEstUINFO)
						m_data.MatrixEstType = 6;
					// 矩阵初始化
					int   nRow = m_header.EstParaCount;
					m_data.m_MatrixEst.Init(nRow,nRow);
					do
					{
						fgets(line,100,pSNXfile);						
						if(line[0] == ' ')
						{//有效数据行							
							string  strLine = line;
							int     row     = 0;
							int     column  = 0;
							sscanf(line,"%*1c%5d%*1c%5d",
								          &row,
										  &column);
							for(int i = 0; i < 3; i ++)
							{
								char    szPara[22 + 1];
								double  para  = 0;
								if(size_t(12 + 22 * (i + 1)) <  strLine.length())//处理文件中无数据的地方直接回车的情况
								{
									size_t n_copy = strLine.copy(szPara,22,12 + 22 * i);
									szPara[22] = '\0';
									if(strcmp(szPara,"                      ") == 0)
										para  = 0;
									else
										sscanf(szPara,"%*1c%21lf",&para);
									if(para != 0 && column - 1 + i < nRow)
										m_data.m_MatrixEst.SetElement(row - 1,column - 1 + i,para);

								}
							}											
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriLCORR
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriUCORR
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriLCOVA
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriUCOVA
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriLINFO
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriUINFO) 
				{
					if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriLCORR)
						m_data.MatrixApriType = 1;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriUCORR)
						m_data.MatrixApriType = 2;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriLCOVA)
						m_data.MatrixApriType = 3;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriUCOVA)
						m_data.MatrixApriType = 4;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriLINFO)
						m_data.MatrixApriType = 5;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutMatrixApriUINFO)
						m_data.MatrixApriType = 6;
					// 矩阵初始化
					int   nRow = int(m_data.m_SolutApri.size());
					m_data.m_MatrixApri.Init(nRow,nRow);
					do
					{
						fgets(line,100,pSNXfile);						
						if(line[0] == ' ')
						{//有效数据行							
							string  strLine = line;
							int     row     = 0;
							int     column  = 0;
							sscanf(line,"%*1c%5d%*1c%5d",
								          &row,
										  &column);
							for(int i = 0; i < 3; i ++)
							{
								char    szPara[22 + 1];
								double  para  = 0;
								if(size_t(12 + 22 * (i + 1)) <  strLine.length())//处理文件中无数据的地方直接回车的情况
								{
									size_t n_copy = strLine.copy(szPara,22,12 + 22 * i);
									szPara[22] = '\0';
									if(strcmp(szPara,"                      ") == 0)
										para  = 0;
									else
										sscanf(szPara,"%*1c%21lf",&para);
									if(para != 0 && column - 1 + i < nRow)
										m_data.m_MatrixApri.SetElement(row - 1,column - 1 + i,para);

								}
							}											
						}
					}while(line[0] != '-');
				}
				else if(strLineLabel == Sinex2_0_BlockLabel::szSolutNEQMatrixL
					  ||strLineLabel == Sinex2_0_BlockLabel::szSolutNEQMatrixU) 
				{
					if(strLineLabel == Sinex2_0_BlockLabel::szSolutNEQMatrixL)
						m_data.MatrixNEQType = 1;
					else if(strLineLabel == Sinex2_0_BlockLabel::szSolutNEQMatrixU)
						m_data.MatrixNEQType = 2;					
					// 矩阵初始化
					int   nRow = m_header.EstParaCount;
					m_data.m_MatrixNEQ.Init(nRow,nRow);
					do
					{
						fgets(line,100,pSNXfile);						
						if(line[0] == ' ')
						{//有效数据行	
							string  strLine = line;
							int     row     = 0;
							int     column  = 0;
							sscanf(line,"%*1c%5d%*1c%5d",
								          &row,
										  &column);
							for(int i = 0; i < 3; i ++)
							{
								char    szPara[22 + 1];
								double  para  = 0;
								if(size_t(12 + 22 * (i + 1)) <  strLine.length())//处理文件中无数据的地方直接回车的情况
								{
									size_t n_copy = strLine.copy(szPara,22,12 + 22 * i);
									szPara[22] = '\0';
									if(strcmp(szPara,"                      ") == 0)
										para  = 0;
									else
										sscanf(szPara,"%*1c%21lf",&para);
									if(para != 0 && column - 1 + i < nRow)
										m_data.m_MatrixNEQ.SetElement(row - 1,column - 1 + i,para);

								}
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
		return true;
	}
	// 子程序名称： write   
	// 功能：将数据写到SNX文件 
	// 变量类型：strSNXFileName    : SNX文件路径
	// 输入：strSNXFileName
	// 输出：
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/08
	// 版本时间：2014/09/08
	// 修改记录：
	// 备注： 
	bool Sinex2_0_File::write(string strSNXFileName)
	{
		if(isEmpty())
		{
			printf("%s file is empty!\n",strSNXFileName.c_str());
			return false;	
		}
		FILE *pSNXfile = fopen(strSNXFileName.c_str(), "w+");
		// 写第一行	
		fprintf(pSNXfile,"%c%c%3s %4.2lf %3s %02d:%03d:%05d %3s %02d:%03d:%05d %02d:%03d:%05d %c %05d %c %c %c %c %c %c %c\n",
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
					    m_header.EstParaCount,
					    m_header.szConstCode[0],
					    m_header.pszSolutCont[0],
					    m_header.pszSolutCont[1],
					    m_header.pszSolutCont[2],
					    m_header.pszSolutCont[3],
					    m_header.pszSolutCont[4],
					    m_header.pszSolutCont[5]);		
		// 循环书写每个数据模块
		if(m_data.m_FileRef.bBlockUse)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szFileRef);
			fprintf(pSNXfile,"*INFO_TYPE_________ INFO________________________________________________________\n");
			if(m_data.m_FileRef.szRefFrameInfo[0] != ' ')
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szRefFrame,m_data.m_FileRef.szRefFrameInfo);
			if(m_data.m_FileRef.szDesInfo[0] != ' ')
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szDes,m_data.m_FileRef.szDesInfo);
			if(m_data.m_FileRef.szInputInfo[0] != ' ')
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szInput,m_data.m_FileRef.szInputInfo);
			if(m_data.m_FileRef.szOutputInfo[0] != ' ')
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szOutput,m_data.m_FileRef.szOutputInfo);
			if(m_data.m_FileRef.szContactInfo[0] != ' ')
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szContact,m_data.m_FileRef.szContactInfo);
			if(m_data.m_FileRef.szSoftwareInfo[0] != ' ')
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szSoftware,m_data.m_FileRef.szSoftwareInfo);
			if(m_data.m_FileRef.szHardwareInfo[0] != ' ')
				fprintf(pSNXfile," %-18s %-60s\n",Sinex2_0_BlockLabel::szHardware,m_data.m_FileRef.szHardwareInfo);
			
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szFileRef);
		}
		if(m_data.m_Comment.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szFileComment);
			for(size_t s_i = 0; s_i < m_data.m_Comment.size(); s_i ++)
				fprintf(pSNXfile,"*%-79s\n",m_data.m_Comment[s_i].c_str());
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szFileComment);
		}
		if(m_data.m_InputHistory.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);			
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szInputHistory);
			fprintf(pSNXfile,"*_VERSION_ CRE __CREATION__ OWN _DATA_START_ __DATA_END__ T PARAM S ____TYPE____\n");
			for(size_t s_i = 0; s_i < m_data.m_InputHistory.size(); s_i ++)
				fprintf(pSNXfile," %c%3s %4.2lf %3s %02d:%03d:%05d %3s %02d:%03d:%05d %02d:%03d:%05d %c %5d %c %c %c %c %c %c %c\n",								
								m_data.m_InputHistory[s_i].szSecondchar[0],
								m_data.m_InputHistory[s_i].szDocType,
								m_data.m_InputHistory[s_i].Version,
								m_data.m_InputHistory[s_i].szFileAgency,
								m_data.m_InputHistory[s_i].FileTime.year,
								m_data.m_InputHistory[s_i].FileTime.doy,
								m_data.m_InputHistory[s_i].FileTime.second,
								m_data.m_InputHistory[s_i].szDataAgency,
								m_data.m_InputHistory[s_i].StartTimeSolut.year,
								m_data.m_InputHistory[s_i].StartTimeSolut.doy,
								m_data.m_InputHistory[s_i].StartTimeSolut.second,
								m_data.m_InputHistory[s_i].EndTimeSolut.year,
								m_data.m_InputHistory[s_i].EndTimeSolut.doy,
								m_data.m_InputHistory[s_i].EndTimeSolut.second,
								m_data.m_InputHistory[s_i].szObsCode[0],
								m_data.m_InputHistory[s_i].EstParaCount,
								m_data.m_InputHistory[s_i].szConstCode[0],
								m_data.m_InputHistory[s_i].pszSolutCont[0],
								m_data.m_InputHistory[s_i].pszSolutCont[1],
								m_data.m_InputHistory[s_i].pszSolutCont[2],
								m_data.m_InputHistory[s_i].pszSolutCont[3],
								m_data.m_InputHistory[s_i].pszSolutCont[4],
								m_data.m_InputHistory[s_i].pszSolutCont[5]);	
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szInputHistory);
		}
		if(m_data.m_InputFile.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);			
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szInputFiles);
			fprintf(pSNXfile,"*OWN __CREATION__ ___________FILENAME__________ ___________DESCRIPTION__________\n");
			for(size_t s_i = 0; s_i < m_data.m_InputFile.size(); s_i ++)
				fprintf(pSNXfile," %-3s %02d:%03d:%05d %-29s %-32s\n",								
								m_data.m_InputFile[s_i].szFileAgency,								
								m_data.m_InputFile[s_i].FileTime.year,
								m_data.m_InputFile[s_i].FileTime.doy,
								m_data.m_InputFile[s_i].FileTime.second,
								m_data.m_InputFile[s_i].szFileName,
								m_data.m_InputFile[s_i].szFileDes);	
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szInputFiles);
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
		if(m_data.m_SolutSatistics.bBlockUse)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutStatistics);
			fprintf(pSNXfile,"*_STATISTICAL PARAMETER________ __VALUE(S)____________\n");
			if(m_data.m_SolutSatistics.obsCount != INT_MAX)
				fprintf(pSNXfile," %30s %22d\n",Sinex2_0_BlockLabel::szObsCount,m_data.m_SolutSatistics.obsCount);
			if(m_data.m_SolutSatistics.unknowns != INT_MAX)
				fprintf(pSNXfile," %30s %22d\n",Sinex2_0_BlockLabel::szObsUnknown,m_data.m_SolutSatistics.unknowns);
			if(m_data.m_SolutSatistics.freedoms != INT_MAX)
				fprintf(pSNXfile," %30s %22d\n",Sinex2_0_BlockLabel::szObsFreedom,m_data.m_SolutSatistics.freedoms);
			if(m_data.m_SolutSatistics.sampInterval != INT_MAX)
				fprintf(pSNXfile," %30s %22d\n",Sinex2_0_BlockLabel::szSampInterval,m_data.m_SolutSatistics.sampInterval);
			if(m_data.m_SolutSatistics.phaseSigma != DBL_MAX)
				fprintf(pSNXfile," %30s %22.5lf\n",Sinex2_0_BlockLabel::szPhaseSigma,m_data.m_SolutSatistics.phaseSigma);
			if(m_data.m_SolutSatistics.codeSigma != DBL_MAX)
				fprintf(pSNXfile," %30s %22.5lf\n",Sinex2_0_BlockLabel::szCodeSigma,m_data.m_SolutSatistics.codeSigma);
			if(m_data.m_SolutSatistics.squareRes != DBL_MAX)
				fprintf(pSNXfile," %30s %22.15lf\n",Sinex2_0_BlockLabel::szSquareRes,m_data.m_SolutSatistics.squareRes);
			if(m_data.m_SolutSatistics.wSquareRes != DBL_MAX)
				fprintf(pSNXfile," %30s %22.15lf\n",Sinex2_0_BlockLabel::szWSquareRes,m_data.m_SolutSatistics.wSquareRes);
			if(m_data.m_SolutSatistics.varFactor != DBL_MAX)
				fprintf(pSNXfile," %30s %22.15lf\n",Sinex2_0_BlockLabel::szVarFactor,m_data.m_SolutSatistics.varFactor);			
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutStatistics);
		}//
		if(m_data.m_Nutaion.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szNutationData);
			for(size_t s_i = 0; s_i < m_data.m_Nutaion.size(); s_i ++)
				fprintf(pSNXfile," %-8s %-70s\n",m_data.m_Nutaion[s_i].szModelCode,m_data.m_Nutaion[s_i].szModelDes);
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szNutationData);
		}
		if(m_data.m_Precession.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szPrecessionData);
			for(size_t s_i = 0; s_i < m_data.m_Precession.size(); s_i ++)
				fprintf(pSNXfile," %-8s %-70s\n",m_data.m_Precession[s_i].szModelCode,m_data.m_Precession[s_i].szModelDes);
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szPrecessionData);
		}
		if(m_data.m_SourceID.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSourceID);
			for(size_t s_i = 0; s_i < m_data.m_SourceID.size(); s_i ++)
				fprintf(pSNXfile," %-4s %-8s %-16s %-68s\n",
				                  m_data.m_SourceID[s_i].szSourceCode,
								  m_data.m_SourceID[s_i].szIERSDes,
								  m_data.m_SourceID[s_i].szICRFDes,
								  m_data.m_SourceID[s_i].szComments);
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSourceID);
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
		if(m_data.m_SiteData.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSiteData);
			fprintf(pSNXfile,"*This file___ Source____________________________________________________________\n");
			fprintf(pSNXfile,"*Code PT Soln Code PT Soln T _Data Start_ _Data End___ Own _File Time__         \n");
			for(size_t s_i = 0; s_i < m_data.m_SiteData.size(); s_i ++)
				fprintf(pSNXfile," %-4s %-2s %-4s %-4s %-2s %-4s %c %02d:%03d:%05d %02d:%03d:%05d %-3s %02d:%03d:%05d\n",
						           m_data.m_SiteData[s_i].szSiteCode,										  
								   m_data.m_SiteData[s_i].szPointCode,
								   m_data.m_SiteData[s_i].szSolutID,
								   m_data.m_SiteData[s_i].szSiteCodeIn,
								   m_data.m_SiteData[s_i].szPointCodeIn,
								   m_data.m_SiteData[s_i].szSolutIDIn,
								   m_data.m_SiteData[s_i].szObsCodeIn[0],
								   m_data.m_SiteData[s_i].StartTimeSolutIn.year,
								   m_data.m_SiteData[s_i].StartTimeSolutIn.doy,
								   m_data.m_SiteData[s_i].StartTimeSolutIn.second,
								   m_data.m_SiteData[s_i].EndTimeSolutIn.year,
								   m_data.m_SiteData[s_i].EndTimeSolutIn.doy,
								   m_data.m_SiteData[s_i].EndTimeSolutIn.second,
								   m_data.m_SiteData[s_i].szAgencyCodeIn,
								   m_data.m_SiteData[s_i].FileTimeIn.year,
								   m_data.m_SiteData[s_i].FileTimeIn.doy,
								   m_data.m_SiteData[s_i].FileTimeIn.second);						
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSiteData);
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
				PCO[0] = it->second.PCO.begin()->second.U;
				PCO[1] = it->second.PCO.begin()->second.N;
				PCO[2] = it->second.PCO.begin()->second.E;
				PCO[3] = it->second.PCO.rbegin()->second.U;
				PCO[4] = it->second.PCO.rbegin()->second.N;
				PCO[5] = it->second.PCO.rbegin()->second.E;//

				fprintf(pSNXfile," %-20s %-5s",
								  it->second.szAntType,										  
								  it->second.szAntSerial);
				for(int i = 0; i < 6; i ++)
				{
					char szPara[8];
					string strPara;
					sprintf(szPara, "%7.4f", PCO[i]);			
					stringEraseFirstZero(szPara, strPara);
					fprintf(pSNXfile," %6s", strPara.c_str());
				}
				fprintf(pSNXfile," %-10s\n",it->second.PCVCorModel);					
			}
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSiteGPSPCO);
		}
		//if(m_data.m_SiteGALPCO.size() > 0)//不完善，2015/09/19，刘俊宏
		//{
		//	fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
		//	fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSiteGALPCO);
		//	fprintf(pSNXfile,"*________TYPE________ SERIE _L1_U_ _L1_N_ _L1_E_ _L2_U_ _L2_N_ _L2_E_ __MODEL___\n");
		//	for(RecAntPCOMap::iterator it = m_data.m_SiteGALPCO.begin(); it != m_data.m_SiteGALPCO.end(); ++it)
		//	{
		//		//写第一行
		//		double  PCO[6];
		//		PCO[0] = it->second.PCO[1].U;
		//		PCO[1] = it->second.PCO[1].N;
		//		PCO[2] = it->second.PCO[1].E;
		//		PCO[3] = it->second.PCO[5].U;
		//		PCO[4] = it->second.PCO[5].N;
		//		PCO[5] = it->second.PCO[5].E;
		//		fprintf(pSNXfile," %-20s %-5s",
		//						  it->second.szAntType,										  
		//						  it->second.szAntSerial);
		//		for(int i = 0; i < 6; i ++)
		//		{
		//			char szPara[8];
		//			string strPara;
		//			sprintf(szPara, "%7.4f", PCO[i]);			
		//			stringEraseFirstZero(szPara, strPara);
		//			fprintf(pSNXfile," %6s", strPara.c_str());
		//		}
		//		fprintf(pSNXfile," %-10s\n",it->second.PCVCorModel);
		//		//写第二行
		//		PCO[0] = it->second.PCO[6].U;
		//		PCO[1] = it->second.PCO[6].N;
		//		PCO[2] = it->second.PCO[6].E;
		//		PCO[3] = it->second.PCO[7].U;
		//		PCO[4] = it->second.PCO[7].N;
		//		PCO[5] = it->second.PCO[7].E;
		//		fprintf(pSNXfile," %-20s %-5s",
		//						  it->second.szAntType,										  
		//						  it->second.szAntSerial);
		//		for(int i = 0; i < 6; i ++)
		//		{
		//			char szPara[8];
		//			string strPara;
		//			sprintf(szPara, "%7.4f", PCO[i]);			
		//			stringEraseFirstZero(szPara, strPara);
		//			fprintf(pSNXfile," %6s", strPara.c_str());
		//		}
		//		fprintf(pSNXfile," %-10s\n",it->second.PCVCorModel);
		//		//写第三行
		//		PCO[0] = it->second.PCO[8].U;
		//		PCO[1] = it->second.PCO[8].N;
		//		PCO[2] = it->second.PCO[8].E;
		//		fprintf(pSNXfile," %-20s %-5s",
		//						  it->second.szAntType,										  
		//						  it->second.szAntSerial);
		//		for(int i = 0; i < 3; i ++)
		//		{
		//			char szPara[8];
		//			string strPara;
		//			sprintf(szPara, "%7.4f", PCO[i]);			
		//			stringEraseFirstZero(szPara, strPara);
		//			fprintf(pSNXfile," %6s", strPara.c_str());
		//		}
		//		fprintf(pSNXfile," %-6s %-6s %-6s %-10s\n",
		//			              "      ",
		//						  "      ",
		//						  "      ",	
		//			              it->second.PCVCorModel);				
		//	}
		//	fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSiteGALPCO);
		//}
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
		if(m_data.m_SatID.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSatID);
			fprintf(pSNXfile,"*SITE PR COSPAR___ T DATA_START__ DATA_END____ ANTENNA_____________\n");
			for(SatIDMap::iterator it = m_data.m_SatID.begin(); it != m_data.m_SatID.end(); ++it)
				fprintf(pSNXfile," %-4s %-2s %-9s %c %02d:%03d:%05d %02d:%03d:%05d %-20s\n",
						          it->second.szSiteCode,										  
								  it->second.szPRN,
								  it->second.szCorparID,
								  it->second.szObsCode[0],
								  it->second.StartTimeSolut.year,
								  it->second.StartTimeSolut.doy,
								  it->second.StartTimeSolut.second,
								  it->second.EndTimeSolut.year,
								  it->second.EndTimeSolut.doy,
								  it->second.EndTimeSolut.second,
								  it->second.szAntType);				
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSatID);
		}
		if(m_data.m_SatPCO.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSatPCO);
			fprintf(pSNXfile,"*SITE L SATA_Z SATA_X SATA_Y L SATA_Z SATA_X SATA_Y MODEL_____ T M\n");
			for(SatPCOMap::iterator it = m_data.m_SatPCO.begin(); it != m_data.m_SatPCO.end(); ++it)
			{
				int nLine    = int(it->second.PCO.size()) / 2;
				int nResidue = int(it->second.PCO.size()) % 2;
				SatFrePCOMap::iterator jt = it->second.PCO.begin();
				for(int n = 1; n <= nLine; n++)
				{
					SatFrePCOMap::iterator kt      = jt;
					SatFrePCOMap::iterator kt_next = ++jt;					
					fprintf(pSNXfile," %-4s %1d %6.4lf",
						          it->second.szSiteCode,										  
								  kt->first,
								  kt->second.z);
					if(kt->second.x >= 0)
						fprintf(pSNXfile, " %6.4lf",kt->second.x);
					else if(kt->second.x < 0 && kt->second.x > -1)
					{
						char szF1x[8];
						string strF1x;
						sprintf(szF1x, "%7.4f", kt->second.x);			
						stringEraseFirstZero(szF1x, strF1x);
						fprintf(pSNXfile, " %6s",strF1x.c_str());
					}
					else	
						fprintf(pSNXfile, " %6.3lf",kt->second.x);
					
					if(kt->second.y >= 0)
						fprintf(pSNXfile, " %6.4lf",kt->second.y);
					else if(kt->second.y < 0 && kt->second.y > -1)
					{
						char szF1y[8];
						string strF1y;
						sprintf(szF1y, "%7.4f", kt->second.y);			
						stringEraseFirstZero(szF1y, strF1y);
						fprintf(pSNXfile, " %6s",strF1y.c_str());
					}
					else	
						fprintf(pSNXfile, " %6.3lf",kt->second.y);
					fprintf(pSNXfile," %1d %6.4lf",kt_next->first,kt_next->second.z);
					if(kt_next->second.x >= 0)
						fprintf(pSNXfile, " %6.4lf",kt_next->second.x);
					else if(kt_next->second.x < 0 && kt_next->second.x > -1)
					{
						char szF1x[8];
						string strF1x;
						sprintf(szF1x, "%7.4f", kt_next->second.x);			
						stringEraseFirstZero(szF1x, strF1x);
						fprintf(pSNXfile, " %6s",strF1x.c_str());
					}
					else	
						fprintf(pSNXfile, " %6.3lf",kt_next->second.x);
					if(kt_next->second.y >= 0)
						fprintf(pSNXfile, " %6.4lf",kt_next->second.y);
					else if(kt_next->second.y < 0 && kt_next->second.y > -1)
					{
						char szF1y[8];
						string strF1y;
						sprintf(szF1y, "%7.4f", kt_next->second.y);			
						stringEraseFirstZero(szF1y, strF1y);
						fprintf(pSNXfile, " %6s",strF1y.c_str());
					}
					else	
						fprintf(pSNXfile, " %6.3lf",kt_next->second.y);
					fprintf(pSNXfile," %-10s %c %c\n",
								  it->second.PCVCorModel,
								  it->second.PCVType[0],
								  it->second.PCVAppModel[0]);				
				}		
				if(nResidue == 1)
				{
					fprintf(pSNXfile," %-4s %1d %6.4lf",
						          it->second.szSiteCode,										  
								  it->second.PCO.rbegin()->first,
								  it->second.PCO.rbegin()->second.z);
					if(it->second.PCO.rbegin()->second.x >= 0)
						fprintf(pSNXfile, " %6.4lf",it->second.PCO.rbegin()->second.x);
					else if(it->second.PCO.rbegin()->second.x < 0 && it->second.PCO.rbegin()->second.x > -1)
					{
						char szF1x[8];
						string strF1x;
						sprintf(szF1x, "%7.4f", it->second.PCO.rbegin()->second.x);			
						stringEraseFirstZero(szF1x, strF1x);
						fprintf(pSNXfile, " %6s",strF1x.c_str());
					}
					else	
						fprintf(pSNXfile, " %6.3lf",it->second.PCO.rbegin()->second.x);
					if(it->second.PCO.rbegin()->second.y >= 0)
						fprintf(pSNXfile, " %6.4lf",it->second.PCO.rbegin()->second.y);
					else if(it->second.PCO.rbegin()->second.y < 0 && it->second.PCO.rbegin()->second.y > -1)
					{
						char szF1y[8];
						string strF1y;
						sprintf(szF1y, "%7.4f", it->second.PCO.rbegin()->second.y);			
						stringEraseFirstZero(szF1y, strF1y);
						fprintf(pSNXfile, " %6s",strF1y.c_str());
					}
					else	
						fprintf(pSNXfile, " %6.3lf",it->second.PCO.rbegin()->second.y);
					fprintf(pSNXfile," %c %-6s %-6s %-6s %-10s %c %c\n",
						          " ",
								  "      ",
								  "      ",
								  "      ",	
								  it->second.PCVCorModel,
								  it->second.PCVType[0],
								  it->second.PCVAppModel[0]);					
				}
			}				
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSatPCO);
		}
		if(m_data.m_SolutEpoch.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutEpochs);
			fprintf(pSNXfile,"*CODE PT SOLN T _DATA_START_ __DATA_END__ _MEAN_EPOCH_\n");
			for(SiteRecAntARPEpochMap::iterator it = m_data.m_SolutEpoch.begin(); it != m_data.m_SolutEpoch.end(); ++it)
				fprintf(pSNXfile," %-4s %-2s %-4s %c %02d:%03d:%05d %02d:%03d:%05d %02d:%03d:%05d\n",
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
								  it->second.MeanTime.year,
								  it->second.MeanTime.doy,
								  it->second.MeanTime.second);								
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutEpochs);
		}
		if(m_data.m_BiasEpoch.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szBiasEpochs);
			fprintf(pSNXfile,"*CODE PT SOLN T _DATA_START_ __DATA_END__ _MEAN_EPOCH_\n");
			for(SiteRecAntARPEpochMap::iterator it = m_data.m_BiasEpoch.begin(); it != m_data.m_BiasEpoch.end(); ++it)
				fprintf(pSNXfile," %-4s %-2s %-4s %c %02d:%03d:%05d %02d:%03d:%05d %02d:%03d:%05d\n",
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
								  it->second.MeanTime.year,
								  it->second.MeanTime.doy,
								  it->second.MeanTime.second);								
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szBiasEpochs);
		}		
		if(m_data.m_SolutEst.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutEst);
			fprintf(pSNXfile,"*INDEX TYPE__ CODE PT SOLN _REF_EPOCH__ UNIT S __ESTIMATED VALUE____ _STD_DEV___\n");
			for(size_t s_i = 0; s_i < m_data.m_SolutEst.size(); s_i ++)
			{
				char szParaValue[22 + 1];
				string strParaValue;
				sprintf(szParaValue, "%22.14e", m_data.m_SolutEst[s_i].ParaValue);	// 由于位数不够，正常的科学计数不能表示			
				stringEraseFloatZero(szParaValue, strParaValue);                    // 这里由15位小数调整为14位。				
				char szParaSTD[12 + 1];                                             // 这种表示方法和首位是0,15位小数的表示精度一样
				string strParaSTD;
				sprintf(szParaSTD, "%12.5e", m_data.m_SolutEst[s_i].ParaSTD);		// 这里由6位小数调整为5位				
				stringEraseFloatZero(szParaSTD, strParaSTD);				
				fprintf(pSNXfile," %5d %-6s %-4s %-2s %-4s %02d:%03d:%05d %-4s %c %21s %11s\n",
						          m_data.m_SolutEst[s_i].Index,
								  m_data.m_SolutEst[s_i].ParaType,
						          m_data.m_SolutEst[s_i].szSiteCode,										  
								  m_data.m_SolutEst[s_i].szPointCode,
								  m_data.m_SolutEst[s_i].szSolutID,										 
								  m_data.m_SolutEst[s_i].EpochTime.year,
								  m_data.m_SolutEst[s_i].EpochTime.doy,
								  m_data.m_SolutEst[s_i].EpochTime.second,
								  m_data.m_SolutEst[s_i].Unit,
								  m_data.m_SolutEst[s_i].szConstCode[0],
								  strParaValue.c_str(),								 
								  strParaSTD.c_str());				
			}
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutEst);
		}
		if(m_data.m_SolutApri.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutApri);
			fprintf(pSNXfile,"*INDEX TYPE__ CODE PT SOLN _REF_EPOCH__ UNIT S __APRIORI VALUE______ _STD_DEV___\n");
			for(size_t s_i = 0; s_i < m_data.m_SolutApri.size(); s_i ++)
			{
				char szParaValue[22 + 1];
				string strParaValue;
				sprintf(szParaValue, "%22.14e", m_data.m_SolutApri[s_i].ParaValue);	// 由于位数不够，正常的科学计数不能表示			
				stringEraseFloatZero(szParaValue, strParaValue);                    // 这里由15位小数调整为14位。				
				char szParaSTD[12 + 1];                                             // 这种表示方法和首位是0,15位小数的表示精度一样
				string strParaSTD;
				sprintf(szParaSTD, "%12.5e", m_data.m_SolutApri[s_i].ParaSTD);		// 这里由6位小数调整为5位				
				stringEraseFloatZero(szParaSTD, strParaSTD);				
				fprintf(pSNXfile," %5d %-6s %-4s %-2s %-4s %02d:%03d:%05d %-4s %c %21s %11s\n",
						          m_data.m_SolutApri[s_i].Index,
								  m_data.m_SolutApri[s_i].ParaType,
						          m_data.m_SolutApri[s_i].szSiteCode,										  
								  m_data.m_SolutApri[s_i].szPointCode,
								  m_data.m_SolutApri[s_i].szSolutID,										 
								  m_data.m_SolutApri[s_i].EpochTime.year,
								  m_data.m_SolutApri[s_i].EpochTime.doy,
								  m_data.m_SolutApri[s_i].EpochTime.second,
								  m_data.m_SolutApri[s_i].Unit,
								  m_data.m_SolutApri[s_i].szConstCode[0],
								  strParaValue.c_str(),								 
								  strParaSTD.c_str());	
			}
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutApri);
		}
		if(m_data.m_SolutNEQVector.size() > 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutNEQVector);
			fprintf(pSNXfile,"*INDEX TYPE__ CODE PT SOLN _REF_EPOCH__ UNIT S __RIGHT_HAND_SIDE____\n");
			for(size_t s_i = 0; s_i < m_data.m_SolutNEQVector.size(); s_i ++)
			{
				char szParaValue[22 + 1];
				string strParaValue;
				sprintf(szParaValue, "%22.14e", m_data.m_SolutApri[s_i].ParaValue);	// 由于位数不够，正常的科学计数不能表示			
				stringEraseFloatZero(szParaValue, strParaValue);                    // 这里由15位小数调整为14位。	
				fprintf(pSNXfile," %5d %-6s %-4s %-2s %-4s %02d:%03d:%05d %-4s %c %21s\n",
						          m_data.m_SolutNEQVector[s_i].Index,
								  m_data.m_SolutNEQVector[s_i].ParaType,
						          m_data.m_SolutNEQVector[s_i].szSiteCode,										  
								  m_data.m_SolutNEQVector[s_i].szPointCode,
								  m_data.m_SolutNEQVector[s_i].szSolutID,										 
								  m_data.m_SolutNEQVector[s_i].EpochTime.year,
								  m_data.m_SolutNEQVector[s_i].EpochTime.doy,
								  m_data.m_SolutNEQVector[s_i].EpochTime.second,
								  m_data.m_SolutNEQVector[s_i].Unit,
								  m_data.m_SolutNEQVector[s_i].szConstCode[0],
								  strParaValue.c_str());	
			}
			fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutNEQVector);
		}
		if(m_data.MatrixEstType != 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			if(m_data.MatrixEstType == 1)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstLCORR);
			else if(m_data.MatrixEstType == 2)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstUCORR);
			else if(m_data.MatrixEstType == 3)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstLCOVA);
			else if(m_data.MatrixEstType == 4)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstUCOVA);
			else if(m_data.MatrixEstType == 5)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstLINFO);
			else if(m_data.MatrixEstType == 6)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstUINFO);
			fprintf(pSNXfile,"*PARA1 PARA2 _______PARA2+0_______ _______PARA2+1_______ _______PARA2+2_______\n");			
			for(int i = 0; i < m_data.m_MatrixEst.GetNumRows(); i ++)
			{//写下三角矩阵
				int nRow     = i + 1;
				int nLine    = nRow / 3;
				int nResidue = nRow % 3;
				if(m_data.MatrixApriType == 2 || m_data.MatrixApriType == 4 || m_data.MatrixApriType == 6)
				{//写上三角矩阵
					nLine    = (m_data.m_MatrixNEQ.GetNumRows() - i) / 3;
					nResidue = (m_data.m_MatrixNEQ.GetNumRows() - i) % 3;
				}
				for(int j = 1; j <= nLine; j ++)
				{			
					int nColumn = 1 + 3 * (j - 1);						
					fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
				    for(int k = 0; k < 3; k ++)
					{
						char szParak[22 + 1];
						string strParak;
						sprintf(szParak, "%22.14e", m_data.m_MatrixEst.GetElement(nRow - 1,nColumn - 1 + k));				
						stringEraseFloatZero(szParak, strParak);
						fprintf(pSNXfile," %21s",strParak.c_str());	
					}
					fprintf(pSNXfile,"\n");
				}
				if(nResidue > 0)
				{						
					int nColumn = 1 + 3 * nLine;
					fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
				    for(int k = 0; k < nResidue; k ++)
					{
						char szParak[22 + 1];
						string strParak;
						sprintf(szParak, "%22.14e", m_data.m_MatrixEst.GetElement(nRow - 1,nColumn - 1 + k));				
						stringEraseFloatZero(szParak, strParak);
						fprintf(pSNXfile," %21s",strParak.c_str());	
					}
					fprintf(pSNXfile,"\n");
				}			
			}			
			if(m_data.MatrixEstType == 1)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstLCORR);
			else if(m_data.MatrixEstType == 2)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstUCORR);
			else if(m_data.MatrixEstType == 3)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstLCOVA);
			else if(m_data.MatrixEstType == 4)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstUCOVA);
			else if(m_data.MatrixEstType == 5)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstLINFO);
			else if(m_data.MatrixEstType == 6)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixEstUINFO);
		}
		if(m_data.MatrixApriType != 0)
		{//m_MatrixApri中存在大量的0，如果全写SNX文件的大小会因此扩大约一倍
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			if(m_data.MatrixApriType == 1)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriLCORR);
			else if(m_data.MatrixApriType == 2)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriUCORR);
			else if(m_data.MatrixApriType == 3)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriLCOVA);
			else if(m_data.MatrixApriType == 4)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriUCOVA);
			else if(m_data.MatrixApriType == 5)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriLINFO);
			else if(m_data.MatrixApriType == 6)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriUINFO);
			fprintf(pSNXfile,"*PARA1 PARA2 ____PARA2+0__________ ____PARA2+1__________ ____PARA2+2__________\n");			
			for(int i = 0; i < m_data.m_MatrixApri.GetNumRows(); i ++)
			{//写下三角矩阵
				int nRow     = i + 1;
				int nLine    = nRow / 3;
				int nResidue = nRow % 3;
				if(m_data.MatrixApriType == 2 || m_data.MatrixApriType == 4 || m_data.MatrixApriType == 6)
				{//写上三角矩阵
					nLine    = (m_data.m_MatrixNEQ.GetNumRows() - i) / 3;
					nResidue = (m_data.m_MatrixNEQ.GetNumRows() - i) % 3;
				}
				for(int j = 1; j <= nLine; j ++)
				{			
					int nColumn = 1 + 3 * (j - 1);	
					double para0 = m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1);
					double para1 = m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1 + 1);
					double para2 = m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1 + 2);
					//3个数（0，!0）有8种情况
					if(para0 == 0 && para1 == 0 && para2 == 0)
					{// 全为0则不输出	
						continue;
					}
					else if(para0 == 0 && para1 == 0 && para2 != 0)
					{
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn + 2);
						char szParak[22 + 1];
						string strParak;
						sprintf(szParak, "%22.14e", para2);				
						stringEraseFloatZero(szParak, strParak);
						fprintf(pSNXfile," %21s\n",strParak.c_str());
					}
					else if(para0 == 0 && para1 != 0 && para2 == 0)
					{
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn + 1);
						char szParak[22 + 1];
						string strParak;
						sprintf(szParak, "%22.14e", para1);				
						stringEraseFloatZero(szParak, strParak);
						fprintf(pSNXfile," %21s\n",strParak.c_str());
					}
					else if(para0 == 0 && para1 != 0 && para2 != 0)
					{
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn + 1);
						for(int k = 1; k < 3; k ++)
						{
							char szParak[22 + 1];
							string strParak;
							sprintf(szParak, "%22.14e", m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1 + k));				
							stringEraseFloatZero(szParak, strParak);
							fprintf(pSNXfile," %21s",strParak.c_str());	
						}
						fprintf(pSNXfile,"\n");
					}
					else if(para0 != 0 && para1 == 0 && para2 == 0)
					{		
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
						char szParak[22 + 1];
						string strParak;
						sprintf(szParak, "%22.14e", para0);				
						stringEraseFloatZero(szParak, strParak);
						fprintf(pSNXfile," %21s\n",strParak.c_str());
					}
					else if(para0 != 0 && para1 == 0 && para2 != 0)
					{
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
						for(int k = 0; k < 3; k ++)
						{
							char szParak[22 + 1];
							string strParak;
							sprintf(szParak, "%22.14e", m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1 + k));				
							stringEraseFloatZero(szParak, strParak);
							fprintf(pSNXfile," %21s",strParak.c_str());	
						}
						fprintf(pSNXfile,"\n");
					}
					else if(para0 != 0 && para1 != 0 && para2 == 0)
					{
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
						for(int k = 0; k < 2; k ++)
						{
							char szParak[22 + 1];
							string strParak;
							sprintf(szParak, "%22.14e", m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1 + k));				
							stringEraseFloatZero(szParak, strParak);
							fprintf(pSNXfile," %21s",strParak.c_str());	
						}
						fprintf(pSNXfile,"\n");
					}
					else 
					{//全不为0
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
						for(int k = 0; k < 3; k ++)
						{
							char szParak[22 + 1];
							string strParak;
							sprintf(szParak, "%22.14e", m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1 + k));				
							stringEraseFloatZero(szParak, strParak);
							fprintf(pSNXfile," %21s",strParak.c_str());	
						}
						fprintf(pSNXfile,"\n");

					}
				}
				if(nResidue == 1)
				{						
					int nColumn = 1 + 3 * nLine;
					fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);					 
					char szParak[22 + 1];
					string strParak;
					sprintf(szParak, "%22.14e", m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1));				
					stringEraseFloatZero(szParak, strParak);
					fprintf(pSNXfile," %21s",strParak.c_str());	
					fprintf(pSNXfile,"\n");
				}	
				if(nResidue == 2)
				{						
					int nColumn = 1 + 3 * nLine;
					double para0 = m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1);
					double para1 = m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1 + 1);
				    //有四种情况
					if(para0 == 0 && para1 == 0)
					{
						continue;
					}
					else if(para0 == 0 && para1 != 0)
					{
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn + 1);
						char szParak[22 + 1];
						string strParak;
						sprintf(szParak, "%22.14e", para1);				
						stringEraseFloatZero(szParak, strParak);
						fprintf(pSNXfile," %21s\n",strParak.c_str());
					}
					else if(para0 != 0 && para1 == 0)
					{
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
						char szParak[22 + 1];
						string strParak;
						sprintf(szParak, "%22.14e", para0);				
						stringEraseFloatZero(szParak, strParak);
						fprintf(pSNXfile," %21s\n",strParak.c_str());
					}
					else
					{
						fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
					    for(int k = 0; k < 2; k ++)
						{
							char szParak[22 + 1];
							string strParak;
							sprintf(szParak, "%22.14e", m_data.m_MatrixApri.GetElement(nRow - 1,nColumn - 1 + k));				
							stringEraseFloatZero(szParak, strParak);
							fprintf(pSNXfile," %21s",strParak.c_str());	
						}//						
						fprintf(pSNXfile,"\n");
					}						
				}			
			}			
			if(m_data.MatrixApriType == 1)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriLCORR);
			else if(m_data.MatrixApriType == 2)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriUCORR);
			else if(m_data.MatrixApriType == 3)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriLCOVA);
			else if(m_data.MatrixApriType == 4)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriUCOVA);
			else if(m_data.MatrixApriType == 5)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriLINFO);
			else if(m_data.MatrixApriType == 6)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutMatrixApriUINFO);
		}
		if(m_data.MatrixNEQType != 0)
		{
			fprintf(pSNXfile,"%80s\n",Sinex2_0_BlockLabel::szBlockSeparator);
			if(m_data.MatrixNEQType == 1)
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutNEQMatrixL);
			else
				fprintf(pSNXfile,"+%33s\n",Sinex2_0_BlockLabel::szSolutNEQMatrixU);	
			fprintf(pSNXfile,"*PARA1 PARA2 ____PARA2+0__________ ____PARA2+1__________ ____PARA2+2__________\n");
			for(int i = 0; i < m_data.m_MatrixNEQ.GetNumRows(); i ++)
			{	//默认下三角矩阵				
				int nRow     = i + 1;
				int nLine    = nRow / 3;
				int nResidue = nRow % 3;
				if(m_data.MatrixNEQType != 1)
				{//写上三角矩阵
					nLine    = (m_data.m_MatrixNEQ.GetNumRows() - i) / 3;
					nResidue = (m_data.m_MatrixNEQ.GetNumRows() - i) % 3;
				}
				for(int j = 1; j <= nLine; j ++)
				{			
					int nColumn = 1 + 3 * (j - 1);						
					fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
				    for(int k = 0; k < 3; k ++)
					{
						char szParak[22 + 1];
						string strParak;
						sprintf(szParak, "%22.14e", m_data.m_MatrixNEQ.GetElement(nRow - 1,nColumn - 1 + k));				
						stringEraseFloatZero(szParak, strParak);
						fprintf(pSNXfile," %21s",strParak.c_str());	
					}
					fprintf(pSNXfile,"\n");
				}
				if(nResidue > 0)
				{						
					int nColumn = 1 + 3 * nLine;
					fprintf(pSNXfile," %5d %5d",
						             nRow,
									 nColumn);
				    for(int k = 0; k < nResidue; k ++)
					{
						char szParak[22 + 1];
						string strParak;
						sprintf(szParak, "%22.14e", m_data.m_MatrixNEQ.GetElement(nRow - 1,nColumn - 1 + k));				
						stringEraseFloatZero(szParak, strParak);
						fprintf(pSNXfile," %21s",strParak.c_str());	
					}
					fprintf(pSNXfile,"\n");
				}			
			}			
			if(m_data.MatrixNEQType == 1)
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutNEQMatrixL);
			else
				fprintf(pSNXfile,"-%33s\n",Sinex2_0_BlockLabel::szSolutNEQMatrixU);		
		}
		fprintf(pSNXfile,"%c%-6s\n",m_header.szFirstchar[0],"ENDSNX");
		fclose(pSNXfile);
		return true;
	}
	// 子程序名称： getStaPos   
	// 功能：获取SNX文件中测站坐标
	// 变量类型：staName    : 测站名
	//           pos        : 测站坐标
	//           posSTD     : 测站坐标的标准差
	//           bEst       : 是否获取测站坐标的估计值，false,获取先验值
	// 输入：staName
	// 输出：posEst,posSTD,bEst         
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/10
	// 版本时间：2014/09/12
	// 修改记录：2014/09/12,用bEst控制获取先验值还是估计值
	// 备注： 
	bool    Sinex2_0_File::getStaPos(string name, POS3D &pos, POS3D &posSTD, bool bEst)
	{
		string staX = "STAX  ";
		string staY = "STAY  ";
		string staZ = "STAZ  ";
		// 测站名称转换成大写
        char staName[5];
		for(int j = 0; j < 4; j++)
		{			
			staName[j] = toCapital(name[j]);
		}		
		staName[4] = '\0';	
		string strName = staName;

		int i = 0;
		vector<SolutVector>    staPosVector = m_data.m_SolutEst;
		if(!bEst)
		{
			staPosVector.clear();
			staPosVector = m_data.m_SolutApri;
		}
		for(size_t  s_i = 0; s_i < staPosVector.size(); s_i ++)
		{
			SolutVector   Epoch = staPosVector[s_i];
			if(Epoch.szSiteCode == strName)
			{				
				if(Epoch.ParaType == staX)
				{
					i ++;
					pos.x = Epoch.ParaValue;
					posSTD.x = Epoch.ParaSTD;
				}
				else if(Epoch.ParaType == staY)
				{
					i ++;
					pos.y = Epoch.ParaValue;
					posSTD.y = Epoch.ParaSTD;
				}
				else if(Epoch.ParaType == staZ)
				{
					i ++;
					pos.z = Epoch.ParaValue;
					posSTD.z = Epoch.ParaSTD;
				}				
			}
			if(i == 3)
				break;
		}
		if(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z < 6e6 || i != 3)
			return false;
		else
			return true;
	}	
	// 子程序名称： getRecARP   
	// 功能：获取SNX文件中测站接收机 
	// 变量类型：staName    : 测站名
	//           antARP     : 天线参考点位置
	// 输入：staName
	// 输出：antARP        
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/10
	// 版本时间：2014/09/10
	// 修改记录：
	// 备注： 
	bool    Sinex2_0_File::getRecAntARP(string name, ENU  &antARP)
	{
		// 测站名称转换成大写
        char staName[5];
		for(int i = 0; i < 4; i++)
		{			
			staName[i] = toCapital(name[i]);
		}		
		staName[4] = '\0';
		SiteRecAntARPEpochMap::iterator it;
		if((it = m_data.m_SiteAntARP.find(staName)) != m_data.m_SiteAntARP.end())
		{
			antARP = it->second.AntARP;
			return true;
		}
		else		
		{
			printf("Haven't found this station(%s)!\n",staName);
			return false;	
		}
	}
	// 子程序名称： getGPSRecAntPCO   
	// 功能：获取SNX文件中测站GPS接收机天线的PCO 
	// 变量类型：staName      : 测站名
	//           freIndex     : 频点，1--L1,2--L2
	//           antPCO       : PCO,ENU 
	// 输入：staName,freIndex
	// 输出：antPCO        
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/10
	// 版本时间：2014/09/10
	// 修改记录：
	// 备注：
	bool    Sinex2_0_File::getGPSRecAntPCO(string name, BYTE  freIndex, ENU &antPCO)
	{
		// 测站名称转换成大写
        char staName[5];
		for(int i = 0; i < 4; i++)
		{			
			staName[i] = toCapital(name[i]);
		}		
		staName[4] = '\0';
		string strAntType;
		SiteRecAntARPEpochMap::iterator it;
		if((it = m_data.m_SiteAnt.find(staName)) != m_data.m_SiteAnt.end())
		{
			strAntType = it->second.szType;			
		}
		else		
		{
			printf("Haven't found this station's antenna type(%s)!\n",staName);
			return false;	
		}
		RecAntPCOMap::iterator jt;
		if((jt = m_data.m_SiteGPSPCO.find(strAntType)) != m_data.m_SiteGPSPCO.end())
		{
			RecFreqPCOMap::iterator kt;
			if((kt = jt->second.PCO.find(freIndex)) != jt->second.PCO.end())
			{
				antPCO = kt->second;
				return true;
			}
			else
			{
				printf("Haven't found this frequency(%1d)!\n",freIndex);
				return false;
			}
		}
		else
		{
			printf("Haven't found this station (%s) antenna type's PCO (%s)!\n",staName,strAntType.c_str());
			return false;
		}
	}
	// 子程序名称： getSatPCOApri   
	// 功能：获取SNX文件中卫星天线的先验值 
	// 变量类型：satName      : 测站名，系统名+PRN，例如"G01"
	//           freIndex     : 频点，1--L1,2--L2(GPS)
	//           satPCOApri   : 卫星PCO先验值,POS3D 
	// 输入：satName,freIndex
	// 输出：antPCO        
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/10
	// 版本时间：2014/09/10
	// 修改记录：
	// 备注：
	bool    Sinex2_0_File::getSatPCOApri(string satName, BYTE freIndex, POS3D  &satPCOApri)
	{	
		string strSiteCode; // 四位的卫星编号
		SatIDMap::iterator it;
		if((it = m_data.m_SatID.find(satName)) != m_data.m_SatID.end())
		{
			strSiteCode = it->second.szSiteCode;			
		}
		else		
		{
			printf("Haven't found this satellite's SiteCode(%s)!\n",satName);
			return false;	
		}
		SatPCOMap::iterator jt;
		if((jt = m_data.m_SatPCO.find(strSiteCode)) != m_data.m_SatPCO.end())
		{
			SatFrePCOMap::iterator kt;
			if((kt = jt->second.PCO.find(freIndex)) != jt->second.PCO.end())
			{
				satPCOApri = kt->second;
				return true;
			}
			else
			{
				printf("Haven't found this frequency(%1d)!\n",freIndex);
				return false;
			}
		}
		else
		{
			printf("Haven't found this satellite (%s) PCO!\n",satName);
			return false;
		}

	}
	// 子程序名称： getStaPosEst   
	// 功能：获取SNX文件中测站坐标的估计值
	// 变量类型：satName    : 测站名
	//           satPCOEst  : PCO估计值
	//           satPCOSTD  : PCO估计值标准差
	//           pointCode  : L1,L2,LC---消电离层组合
	// 输入：satName，pointCode
	// 输出：satPCOEst,satPCOSTD         
	// 语言：C++
	// 创建者：刘俊宏
	// 创建时间：2014/09/10
	// 版本时间：2014/09/10
	// 修改记录：
	// 备注：
	bool    Sinex2_0_File::getSatPCOEst(string satName, POS3D  &satPCOEst, POS3D &satPCOSTD, string pointCode)
	{
		string satPCOX = "SATA_X";
		string satPCOY = "SATA_Y";
		string satPCOZ = "SATA_Z";

		string strSiteCode; // 四位的卫星编号
		SatIDMap::iterator it;
		if((it = m_data.m_SatID.find(satName)) != m_data.m_SatID.end())
		{
			strSiteCode = it->second.szSiteCode;			
		}
		else		
		{
			printf("Haven't found this satellite's SiteCode(%s)!\n",satName);
			return false;	
		}
		int i = 0;
		for(size_t  s_i = 0; s_i < m_data.m_SolutEst.size(); s_i ++)
		{
			SolutVector   Epoch = m_data.m_SolutEst[s_i];
			if(Epoch.szSiteCode == strSiteCode)
			{				
				if(Epoch.ParaType == satPCOX && Epoch.szPointCode == pointCode)
				{
					i ++;
					satPCOEst.x = Epoch.ParaValue;
					satPCOSTD.x = Epoch.ParaSTD;
				}
				else if(Epoch.ParaType == satPCOY && Epoch.szPointCode == pointCode)
				{
					i ++;
					satPCOEst.y = Epoch.ParaValue;
					satPCOSTD.y = Epoch.ParaSTD;
				}
				else if(Epoch.ParaType == satPCOZ && Epoch.szPointCode == pointCode)
				{
					i ++;
					satPCOEst.z = Epoch.ParaValue;
					satPCOSTD.z = Epoch.ParaSTD;
				}				
			}
			if(i == 3)
				break;
		}
		if(i != 3)
			return false;
		else
			return true;
	}
}
