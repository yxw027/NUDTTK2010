#include "Rinex2_1_MixedObsFile.hpp"
#include "MathAlgorithm.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	Rinex2_1_MixedObsFile::Rinex2_1_MixedObsFile(void)
	{
	}

	Rinex2_1_MixedObsFile::~Rinex2_1_MixedObsFile(void)
	{
	}
	void Rinex2_1_MixedObsHeader::init(Rinex2_1_ObsHeader m_header)
	{
		AntOffset            = m_header.AntOffset; 
		ApproxPos            = m_header.ApproxPos; 
		bL1WaveLengthFact    = m_header.bL1WaveLengthFact; 
		bL2WaveLengthFact    = m_header.bL2WaveLengthFact; 
		bL5WaveLengthFact    = m_header.bL5WaveLengthFact;
		byObsTypes           = m_header.byObsTypes; 
		bySatCount           = m_header.bySatCount; 
		Interval             = m_header.Interval; 
		LeapSecond           = m_header.LeapSecond;
		pbyObsTypeList       = m_header.pbyObsTypeList;		
		strcpy(szAntNumber          , m_header.szAntNumber);		
		strcpy(szAntType            , m_header.szAntType); 
		strcpy(szFileDate           , m_header.szFileDate);
		strcpy(szFileType           , m_header.szFileType);
		strcpy(szMarkName           , m_header.szMarkName);
		strcpy(szMarkNumber         , m_header.szMarkNumber);
		strcpy(szObserverAgencyName , m_header.szObserverAgencyName);
		strcpy(szObserverName       , m_header.szObserverName);
		strcpy(szProgramAgencyName  , m_header.szProgramAgencyName);
		strcpy(szProgramName        , m_header.szProgramName);
		strcpy(szRecNumber          , m_header.szRecNumber);
		strcpy(szRecType            , m_header.szRecType);
		strcpy(szRecVersion         , m_header.szRecVersion);
		strcpy(szRinexVersion       , m_header.szRinexVersion);	
		strcpy(szSatlliteSystem     , m_header.szSatlliteSystem);
		strcpy(szTimeType           , m_header.szTimeType);
		tmEnd                = m_header.tmEnd;
		tmStart              = m_header.tmStart;
		pstrSatList.clear();
		for(size_t s_i = 0; s_i < m_header.pbySatList.size(); s_i ++)
		{
			char szSatName[4];
			sprintf(szSatName, "%1c%02d", m_header.getSatSystemChar(),m_header.pbySatList[s_i]);
			szSatName[3] = '\0';
			pstrSatList.push_back(szSatName);
		}
	}

	void Rinex2_1_MixedObsFile::clear()
	{
		m_header = Rinex2_1_MixedObsHeader::Rinex2_1_MixedObsHeader();
		m_data.clear();
	}

	bool Rinex2_1_MixedObsFile::isEmpty()
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
	// 创建者：谷德峰
	// 创建时间：2007/3/27
	// 版本时间：2007/3/27
	// 修改记录：
	// 备注： 
	int  Rinex2_1_MixedObsFile::isValidEpochLine(string strLine, FILE * pObsfile)
	{
		DayTime tmEpoch;
		// 下面几种数据用int型, 避免因为strLine的格式问题引起sscanf函数发生错误
		int byEpochFlag = -1; // 0: OK; 1: power failure between previous and current epoch  > 1: Event flag ( 2 - 5 ) 
		int bySatCount  = -1; // 本时刻可视GPS卫星个数
		if(pObsfile != NULL)  // 判断是否为文件末尾
		{
			if(feof(pObsfile))
				return 0;
		}

		sscanf(strLine.c_str(),"%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%11lf%*2c%1d%3d",
			                   &tmEpoch.year,
							   &tmEpoch.month,
							   &tmEpoch.day,
							   &tmEpoch.hour,
							   &tmEpoch.minute,
							   &tmEpoch.second,
							   &byEpochFlag,
							   &bySatCount);
		
		if(tmEpoch.month > 12 || tmEpoch.month < 0)
			return 2;
		if(tmEpoch.day >31 || tmEpoch.day < 0)
			return 2;
		if(tmEpoch.hour > 24 || tmEpoch.hour < 0)
			return 2;
		if(tmEpoch.minute > 60 || tmEpoch.minute < 0)
			return 2;
		if(tmEpoch.second > 60 || tmEpoch.second < 0)
			return 2;
		if(byEpochFlag > 5 || byEpochFlag < 0)
			return 2;

		if(bySatCount < 0)
			return 2;

		return 1;
	}

	// 子程序名称： open   
	// 功能：观测数据解析 
	// 变量类型：strObsfileName : 观测数据文件路径
	//           strSystem      : 卫星导航系统缩写字母集合, GPS = "G", BDS = "C", 可以多个系统联合, 如"GC"
	// 输入：strObsfileName  : 
	// 输出：
	// 语言：C++
	// 创建者：谷德峰, 鞠冰
	// 创建时间：2014/3/22
	// 版本时间：2014/3/22
	// 修改记录：
	// 备注： 
	bool Rinex2_1_MixedObsFile::open(string strObsfileName, string strSystem)
	{
		//clock_t start, finish;
		//double  duration;
		//start = clock();
		if(!isWildcardMatch(strObsfileName.c_str(), "*.*O", true))
		{
			printf(" %s 文件名不匹配!\n", strObsfileName.c_str());
			return false;
		}

		FILE * pObsfile = fopen(strObsfileName.c_str(),"r+t");
		if(pObsfile == NULL) 
			return false;
		m_header = Rinex2_1_MixedObsHeader::Rinex2_1_MixedObsHeader();
		// 开始循环读取每一行数据，直到 END OF HEADER
		int bFlag = 1;
		while(bFlag)
		{
			char line[200];
			fgets(line,100,pObsfile);
			string strLineMask = line;
			string strLine     = line;
			strLineMask.erase(0, 60); // 从第 0 个元素开始，删除 60 个
			// 剔除 \n
			size_t nPos_n = strLineMask.find('\n');
			if(nPos_n < strLineMask.length())
				strLineMask.erase(nPos_n, 1);
			// 超过20位，截取20位
			while(strLineMask.length() > 20)
				strLineMask.erase(strLineMask.length() - 1, 1);
			// 补齐20位
			if(strLineMask.length() < 20) // strLineMask.length 包含最后的'\0'
				strLineMask.append(20-strLineMask.length(),' ');
			
			if(strLineMask == Rinex2_1_MaskString::szVerType)
			{
				strLine.copy(m_header.szRinexVersion, 20, 0);
				m_header.szRinexVersion[20] = '\0';
				strLine.copy(m_header.szFileType, 20, 20);
				m_header.szFileType[20] = '\0';
				strLine.copy(m_header.szSatlliteSystem, 20, 40);
				m_header.szSatlliteSystem[20] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szPgmRunDate)
			{
				strLine.copy(m_header.szProgramName, 20, 0);
				m_header.szProgramName[20] = '\0';
				strLine.copy(m_header.szProgramAgencyName, 20, 20);
				m_header.szProgramAgencyName[20] = '\0';
				strLine.copy(m_header.szFileDate, 20, 40);
				m_header.szFileDate[20] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szObservAgency)
			{
				strLine.copy(m_header.szObserverName, 20, 0);
				m_header.szObserverName[20] = '\0';
				strLine.copy(m_header.szObserverAgencyName, 40, 20);
				m_header.szObserverAgencyName[40] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szMarkerName)
			{
				strLine.copy(m_header.szMarkName, 60, 0);
				m_header.szMarkName[60] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szMarkerNum)
			{
				strLine.copy(m_header.szMarkNumber, 20, 0);
				m_header.szMarkNumber[20] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szRecTypeVers)
			{
				strLine.copy(m_header.szRecNumber, 20, 0);
				m_header.szRecNumber[20] = '\0';
				strLine.copy(m_header.szRecType, 20, 20);
				m_header.szRecType[20] = '\0';
				strLine.copy(m_header.szRecVersion, 20, 40);
				m_header.szRecVersion[20] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szAntType)
			{
				strLine.copy(m_header.szAntNumber, 20, 0);
				m_header.szAntNumber[20] = '\0';
				strLine.copy(m_header.szAntType, 20, 20);
				m_header.szAntType[20] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szApproxPosXYZ)
			{  
				sscanf(line,"%14lf%14lf%14lf",&m_header.ApproxPos.x, &m_header.ApproxPos.y, &m_header.ApproxPos.z);
			}
			else if(strLineMask == Rinex2_1_MaskString::szAntDeltaHEN)
			{  
				sscanf(line,"%14lf%14lf%14lf",&m_header.AntOffset.x, &m_header.AntOffset.y, &m_header.AntOffset.z);
			}
			else if(strLineMask == Rinex2_1_MaskString::szWaveLenFact)
			{  
				sscanf(line,"%6d%6d%6d",&m_header.bL1WaveLengthFact, &m_header.bL2WaveLengthFact, &m_header.bL5WaveLengthFact);
			}
			else if(strLineMask == Rinex2_1_MaskString::szTmOfFirstObs)
			{  
				sscanf(line,"%6d%6d%6d%6d%6d%12lf",&m_header.tmStart.year,  &m_header.tmStart.month,
												   &m_header.tmStart.day,   &m_header.tmStart.hour,
												   &m_header.tmStart.minute,&m_header.tmStart.second);
				strLine.copy(m_header.szTimeType, 3, 48);
				m_header.szTimeType[3] = '\0';
			}
			else if(strLineMask == Rinex2_1_MaskString::szTmOfLastObs)
			{  
				sscanf(line,"%6d%6d%6d%6d%6d%12lf",&m_header.tmEnd.year,  &m_header.tmEnd.month,
												   &m_header.tmEnd.day,	  &m_header.tmEnd.hour,
												   &m_header.tmEnd.minute,&m_header.tmEnd.second);
			}
			else if(strLineMask == Rinex2_1_MaskString::szInterval)
			{  
				sscanf(line,"%10lf", &m_header.Interval);
			}
			else if(strLineMask == Rinex2_1_MaskString::szLeapSec)
			{  
				sscanf(line,"%6d", &m_header.LeapSecond);
			}
			else if(strLineMask == Rinex2_1_MaskString::szNumsOfSv)
			{  
				sscanf(line,"%6d", &m_header.bySatCount);
			}
			else if(strLineMask == Rinex2_1_MaskString::szTypeOfObserv)
			{  
				sscanf(line,"%6d", &m_header.byObsTypes);
				for(BYTE i = 0; i < m_header.byObsTypes; i++)
				{
					char strObsType[7];
					strLine.copy(strObsType, 6, 6 + i * 6);
					strObsType[6] = '\0';
					m_header.pbyObsTypeList.push_back(string2ObsId(strObsType));
				}
			}
			else if(strLineMask == Rinex2_1_MaskString::szPRNNumOfObs)
			{// 可视卫星 PRN 列表在文件读取完后再填写，不在文件头处读取
			}
			else if(strLineMask == Rinex2_1_MaskString::szEndOfHead)
			{
				bFlag = false;
			}
			else 
			{// Comment等不作处理
			}
		}
		// 观测数据解析
		bFlag = TRUE;
		m_data.clear();
		char line[100];
		fgets(line, 100, pObsfile);
		map<string, int> mapSatNameList; // 可视卫星列表
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
				Rinex2_1_MixedObsEpoch obsEpoch;
				vector<string>  strSatNameList;		// 卫星列表
				strSatNameList.clear();
				// 解析Epoch/SAT 1--32
				sscanf(strLine.c_str(),"%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%11lf%*2c%1d%3d",
					&obsEpoch.t.year,
					&obsEpoch.t.month,
					&obsEpoch.t.day,
					&obsEpoch.t.hour,
					&obsEpoch.t.minute,
					&obsEpoch.t.second,
					&obsEpoch.byEpochFlag,
					&obsEpoch.bySatCount);	// bySatCount 需根据GNSS系统卫星个数修改(2013/6/25)
				// 暂不处理 EpochFlag > 1 的 Event (2013/6/26)
				if(obsEpoch.byEpochFlag > 1)
				{
					fgets(line, 100, pObsfile);
					continue;
				}
				obsEpoch.t.year = yearB2toB4(obsEpoch.t.year);
				// 32----68  32X, 12(A1,I2)  68----80  receiver clock offset F12.9	
				char strSatName[4] = "   "; // 存储卫星系统 + PRN (例如'G01')
				strSatName[3] = '\0';
				int bySatPRN = 0; // 卫星 PRN 号
				BYTE bySatCount_GNSS = 0; // 存储当前历元可视卫星中有效GNSS卫星(符合szSystem定义)个数
				bool *pbSatSysValidFlag  = new bool [obsEpoch.bySatCount];	// 记录当前可视卫星序列中的每颗卫星类型是否有效, 用于指导数据行读取时遴选有效数据
				int nLine    = obsEpoch.bySatCount / 12;
				int nResidue = obsEpoch.bySatCount % 12;
				// 可视卫星数小于等于 12 
				if(obsEpoch.bySatCount <= 12)
				{
					for(int i = 0; i < obsEpoch.bySatCount; i++)
					{
						// 读取卫星 PRN 列表
						strLine.copy(strSatName, 3, 32 + i * 3);
						if(strSatName[0] == ' ')
							strSatName[0] = 'G';
						if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
							strSatName[1] = '0';
						strSatNameList.push_back(strSatName);
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
					}
				}
				// 可视卫星数大于 12
				else
				{
					// 读取前 12 颗卫星
					for(int i = 0; i < 12; i++)
					{
						// 读取卫星 PRN 列表
						strLine.copy(strSatName, 3, 32 + i * 3);
						if(strSatName[0] == ' ')
							strSatName[0] = 'G';
						if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
							strSatName[1] = '0';
						strSatNameList.push_back(strSatName);
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
					}
					// 读取中间 nLine - 1 行数据
					for(int j = 1; j < nLine; j++)
					{
						fgets(line, 100, pObsfile);
						strLine = line;
						for(int i = 0; i < 12; i++)
						{
							// 读取卫星 PRN 列表
							strLine.copy(strSatName, 3, 32 + i * 3);
							if(strSatName[0] == ' ')
								strSatName[0] = 'G';
							if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
								strSatName[1] = '0';
							strSatNameList.push_back(strSatName);
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
						}
					}
					//读取最后 nResidue 个数据
					if(nResidue > 0)
					{
						fgets(line, 100, pObsfile);
						strLine = line;
						for(int i = 0; i < nResidue; i++)
						{
							// 读取卫星 PRN 列表
							strLine.copy(strSatName, 3, 32 + i * 3);
							if(strSatName[0] == ' ')
								strSatName[0] = 'G';
							if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
								strSatName[1] = '0';
							strSatNameList.push_back(strSatName);
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
						}
					}
				}// end of else(可观测卫星数大于 12)
				// 开始读取当前历元的观测数据
				obsEpoch.obs.clear();
				// 根据Epoch/SAT，解析可见观测数据
				for(int m = 0; m < obsEpoch.bySatCount; m++)
				{
					nLine    = m_header.byObsTypes / 5;
					nResidue = m_header.byObsTypes % 5;
					Rinex2_1_ObsTypeList obsTypeList;
					obsTypeList.clear();
					// 观测类型的个数小于等于 5
					if(m_header.byObsTypes <= 5) 
					{
						fgets(line, 100, pObsfile);
						if(!pbSatSysValidFlag[m])
							continue;	// 若当前卫星不是指定系统卫星, 略过此行
						strLine = line;
						// 剔除回车符 '/n'(2013/6/26)
						size_t nPos_n = strLine.find('\n');
						strLine.erase(nPos_n, 1);
						for(int i = 0; i < m_header.byObsTypes; i++)
						{
							//Rinex2_1_ObsDatum obsDatum;
							//char strObsSatElements[17];		// F14.3 + I1 + I1
							//char strObsValue[15];			// F14.3
							//strObsSatElements[14] = ' ';	// I1
							//strObsSatElements[15] = ' ';	// I1
							//if(size_t(16 * i + 14) <= strLine.length())	// 防止空白数据未补齐直接换行
							//{
							//	strLine.copy(strObsSatElements, 16, 16 * i);
							//	strObsSatElements[16] = '\0';
							//	strncpy(strObsValue, strObsSatElements, 14);
							//	strObsValue[14] = '\0';
							//	// 添加对空格无效数据的校验，当数据无效时一般会以空格填写
							//	if(strcmp(strObsValue,"              ") == 0)
							//	{
							//		obsDatum.data = DBL_MAX;
							//	}
							//	else // 2007-4-29 防止读取过多的有效数字I1I1  F14.3I1I1
							//	{
							//		//sscanf(strObsValue, "%14lf", &obsDatum.data);
							//		sscanf(strObsValue, "%16lf", &obsDatum.data);
							//	}
							//	obsDatum.lli = strObsSatElements[14]; // 由于champ数据在每行数据的末尾直接添回车, 读取后如果不加以处理，在写文件时会增加一行空白行
							//	obsDatum.ssi = strObsSatElements[15];
							//}
							//else
							//{
							//	obsDatum.data = DBL_MAX;
							//}
							Rinex2_1_ObsDatum obsDatum;
							char strObsSatElements[17+2];		// F16.6 + I1 + I1
							char strObsValue[15+2];			// F16.6
							strObsSatElements[14+2] = ' ';	// I1
							strObsSatElements[15+2] = ' ';	// I1
							if(size_t(18 * i + 16) <= strLine.length())	// 防止空白数据未补齐直接换行
							{
								strLine.copy(strObsSatElements, 18, 18 * i);
								strObsSatElements[18] = '\0';
								strncpy(strObsValue, strObsSatElements, 16);
								strObsValue[16] = '\0';
								// 添加对空格无效数据的校验，当数据无效时一般会以空格填写
								if(strcmp(strObsValue,"                ") == 0)
								{
									obsDatum.data = DBL_MAX;
								}
								else // 2007-4-29 防止读取过多的有效数字I1I1  F16.6I1I1
								{
									//sscanf(strObsValue, "%14lf", &obsDatum.data);
									sscanf(strObsValue, "%16lf", &obsDatum.data);
								}
								obsDatum.lli = strObsSatElements[14+2]; // 由于champ数据在每行数据的末尾直接添回车, 读取后如果不加以处理，在写文件时会增加一行空白行
								obsDatum.ssi = strObsSatElements[15+2];
							}
							else
							{
								obsDatum.data = DBL_MAX;
							}
							obsTypeList.push_back(obsDatum);
						}
					}// end of if(观测类型的个数小于等于 5)
					// 观测类型的个数大于 5
					else 
					{
						// 若当前卫星不是指定系统卫星，略过此数据块
						if(!pbSatSysValidFlag[m])
						{
							for(int l = 1; l <= nLine; l++)
							{
								fgets(line, 100, pObsfile);
							}
							if(nResidue > 0)
							{
								fgets(line, 100, pObsfile);
							}
							continue;	
						}
						// 读取前5个数据
						fgets(line, 100, pObsfile);
						strLine = line;
						// 剔除回车符 '/n'(2013/6/26)
						size_t nPos_n = strLine.find('\n');
						strLine.erase(nPos_n, 1);
						for(int i = 0; i < 5; i++)
						{
							//Rinex2_1_ObsDatum obsDatum;
							//char strObsSatElements[17];		// F14.3 + I1 + I1
							//char strObsValue[15];			// F14.3
							//strObsSatElements[14] = ' ';	// I1
							//strObsSatElements[15] = ' ';	// I1
							//if(size_t(16 * i + 14) <= strLine.length())
							//{
							//	strLine.copy(strObsSatElements, 16, 16 * i);
							//	strObsSatElements[16] = '\0';
							//	strncpy(strObsValue, strObsSatElements, 14);
							//	strObsValue[14] = '\0';
							//	// 添加对空格无效数据的校验，当数据无效时一般会以空格填写
							//	if(strcmp(strObsValue,"              ") == 0)
							//	{
							//		obsDatum.data = DBL_MAX;
							//	}
							//	else // 2007-4-29 防止读取过多的有效数字I1I1  F14.3I1I1
							//	{
							//		//sscanf(strObsValue, "%14lf", &obsDatum.data);
							//		sscanf(strObsValue, "%16lf", &obsDatum.data);
							//	}
							//	obsDatum.lli = strObsSatElements[14];
							//	obsDatum.ssi = strObsSatElements[15];
							//}
							//else
							//{
							//	obsDatum.data = DBL_MAX;
							//}
							Rinex2_1_ObsDatum obsDatum;
							char strObsSatElements[17+2];		// F14.3 + I1 + I1
							char strObsValue[15+2];			// F14.3
							strObsSatElements[14+2] = ' ';	// I1
							strObsSatElements[15+2] = ' ';	// I1
							if(size_t(18 * i + 16) <= strLine.length())
							{
								strLine.copy(strObsSatElements, 18, 18 * i);
								strObsSatElements[16+2] = '\0';
								strncpy(strObsValue, strObsSatElements, 16);
								strObsValue[14+2] = '\0';
								// 添加对空格无效数据的校验，当数据无效时一般会以空格填写
								if(strcmp(strObsValue,"                ") == 0)
								{
									obsDatum.data = DBL_MAX;
								}
								else // 2007-4-29 防止读取过多的有效数字I1I1  F16.6I1I1
								{
									//sscanf(strObsValue, "%14lf", &obsDatum.data);
									sscanf(strObsValue, "%16lf", &obsDatum.data);
								}
								obsDatum.lli = strObsSatElements[14+2];
								obsDatum.ssi = strObsSatElements[15+2];
							}
							else
							{
								obsDatum.data = DBL_MAX;
							}
							obsTypeList.push_back(obsDatum);
						}
						// 读取中间 nLine - 1 行数据
						for(int j = 1; j < nLine; j++)
						{
							fgets(line, 100, pObsfile);
							strLine = line;
							// 剔除回车符 '/n'(2013/6/26)
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							for(int i = 0; i < 5; i++)
							{
								//Rinex2_1_ObsDatum obsDatum;
								//char strObsSatElements[17];		// F14.3 + I1 + I1
								//char strObsValue[15];			// F14.3
								//strObsSatElements[14] = ' ';	// I1
								//strObsSatElements[15] = ' ';	// I1
								//if(size_t(16 * i + 14) <= strLine.length())
								//{
								//	strLine.copy(strObsSatElements, 16, 16 * i);
								//	strObsSatElements[16] = '\0';
								//	strncpy(strObsValue, strObsSatElements, 14);
								//	strObsValue[14] = '\0';
								//	// 添加对空格无效数据的校验，当数据无效时一般会以空格填写
								//	if(strcmp(strObsValue,"              ") == 0)
								//	{
								//		obsDatum.data = DBL_MAX;
								//	}
								//	else // 2007-4-29 防止读取过多的有效数字I1I1  F14.3I1I1
								//	{
								//		//sscanf(strObsValue, "%14lf", &obsDatum.data);
								//		sscanf(strObsValue, "%16lf", &obsDatum.data);
								//	}
								//	obsDatum.lli = strObsSatElements[14];
								//	obsDatum.ssi = strObsSatElements[15];
								//}
								//else
								//{
								//	obsDatum.data = DBL_MAX;
								//}
								Rinex2_1_ObsDatum obsDatum;
								char strObsSatElements[17+2];		// F14.3 + I1 + I1
								char strObsValue[15+2];			// F14.3
								strObsSatElements[14+2] = ' ';	// I1
								strObsSatElements[15+2] = ' ';	// I1
								if(size_t(18 * i + 16) <= strLine.length())
								{
									strLine.copy(strObsSatElements, 18, 18 * i);
									strObsSatElements[18] = '\0';
									strncpy(strObsValue, strObsSatElements, 16);
									strObsValue[16] = '\0';
									// 添加对空格无效数据的校验，当数据无效时一般会以空格填写
									if(strcmp(strObsValue,"                ") == 0)
									{
										obsDatum.data = DBL_MAX;
									}
									else // 2007-4-29 防止读取过多的有效数字I1I1  F14.3I1I1
									{
										//sscanf(strObsValue, "%14lf", &obsDatum.data);
										sscanf(strObsValue, "%16lf", &obsDatum.data);
									}
									obsDatum.lli = strObsSatElements[16];
									obsDatum.ssi = strObsSatElements[17];
								}
								else
								{
									obsDatum.data = DBL_MAX;
								}
								obsTypeList.push_back(obsDatum);
							}
						}
						// 读取最后 nResidue 个数据
						if(nResidue > 0)
						{
							fgets(line, 100, pObsfile);
							strLine = line;
							// 剔除回车符 '/n'(2013/6/26)
							size_t nPos_n = strLine.find('\n');
							strLine.erase(nPos_n, 1);
							for(int i = 0; i < nResidue; i++)
							{
								//Rinex2_1_ObsDatum obsDatum;
								//char strObsSatElements[17];		// F14.3 + I1 + I1
								//char strObsValue[15];			// F14.3
								//strObsSatElements[14] = ' ';	// I1
								//strObsSatElements[15] = ' ';	// I1
								//if(size_t(16 * i + 14) <= strLine.length())
								//{
								//	strLine.copy(strObsSatElements, 16, 16 * i);
								//	strObsSatElements[16] = '\0';
								//	strncpy(strObsValue, strObsSatElements, 14);
								//	strObsValue[14] = '\0';
								//	// 添加对空格无效数据的校验，当数据无效时一般会以空格填写
								//	if(strcmp(strObsValue,"              ") == 0)
								//	{
								//		obsDatum.data = DBL_MAX;
								//	}
								//	else // 2007-4-29 防止读取过多的有效数字I1I1  F14.3I1I1
								//	{
								//		//sscanf(strObsValue, "%14lf", &obsDatum.data);
								//		sscanf(strObsValue, "%16lf", &obsDatum.data);
								//	}
								//	obsDatum.lli = strObsSatElements[14];
								//	obsDatum.ssi = strObsSatElements[15];
								//}
								//else
								//{
								//	obsDatum.data = DBL_MAX;
								//}
								Rinex2_1_ObsDatum obsDatum;
								char strObsSatElements[19];		// F14.3 + I1 + I1
								char strObsValue[17];			// F14.3
								strObsSatElements[16] = ' ';	// I1
								strObsSatElements[17] = ' ';	// I1
								if(size_t(18 * i + 16) <= strLine.length())
								{
									strLine.copy(strObsSatElements, 18, 18 * i);
									strObsSatElements[18] = '\0';
									strncpy(strObsValue, strObsSatElements, 16);
									strObsValue[16] = '\0';
									// 添加对空格无效数据的校验，当数据无效时一般会以空格填写
									if(strcmp(strObsValue,"                ") == 0)
									{
										obsDatum.data = DBL_MAX;
									}
									else // 2007-4-29 防止读取过多的有效数字I1I1  F14.3I1I1
									{
										//sscanf(strObsValue, "%14lf", &obsDatum.data);
										sscanf(strObsValue, "%16lf", &obsDatum.data);
									}
									obsDatum.lli = strObsSatElements[16];
									obsDatum.ssi = strObsSatElements[17];
								}
								else
								{
									obsDatum.data = DBL_MAX;
								}
								obsTypeList.push_back(obsDatum);
							}
						}
					}// end of else(观测类型的个数大于 5)
					obsEpoch.obs.insert(Rinex2_1_MixedSatMap::value_type(strSatNameList[m], obsTypeList));
				}// end of for(遍历卫星)
				obsEpoch.bySatCount = bySatCount_GNSS;			// 更改当前历元可视卫星数(仅统计 GPS 卫星个数)
				m_data.push_back(obsEpoch);
				fgets(line, 100, pObsfile);
				// 释放动态内存空间
				delete[] pbSatSysValidFlag;
			}
			else  
			{// 无效数据行，例如空白行，掠过不作处理
				fgets(line, 100, pObsfile);
			}
		}// end of while(bFlag)
		// 综合统计可视卫星列表
		m_header.pstrSatList.clear();
		for(map<string, int>::iterator it = mapSatNameList.begin(); it != mapSatNameList.end(); ++it)
		{
			m_header.pstrSatList.push_back(it->first);
		}
		m_header.bySatCount = BYTE(m_header.pstrSatList.size());
		//更新初始观测时刻和最后观测时刻
		size_t nListNum = m_data.size();
		if(nListNum > 0)
		{
			m_header.tmStart = m_data[0].t;
			m_header.tmEnd   = m_data[nListNum-1].t;
		}
		fclose(pObsfile);
		//finish = clock();
		//duration = (double)(finish - start) / CLOCKS_PER_SEC;
		//cout<<"文件读取完毕，耗时"<<duration<<"秒。"<<endl;
		return true;
	}

	// 子程序名称： write   
	// 功能：将观测数据写到文件 
	// 变量类型：strObsfileName_noExp    : 观测数据文件路径(后缀名程序自动生成, 输入不包含)
	//           strObsfileName_all      : 完整的观测数据文件路径
	// 输入：strObsfileName
	// 输出：strObsfileName_all
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/3/23
	// 版本时间：2014/3/23
	// 修改记录：
	// 备注：只在少数标记处"2014/03/23"与 Rinex2_1_ObsFile::write 函数存在区别
	bool Rinex2_1_MixedObsFile::write(string strObsfileName_noExp,string& strObsfileName_all)
	{
		if(isEmpty())
			return false;
		// 写文件头，按照 champ 的标准文件头书写
		int n1  = getIntBit(m_header.tmStart.year, 0);
		int n10 = getIntBit(m_header.tmStart.year, 1);
		char strFileExp[5];
		sprintf(strFileExp,".%1d%1do",n10,n1);
		strObsfileName_all = strObsfileName_noExp + strFileExp;
		FILE* pObsfile=fopen(strObsfileName_all.c_str(), "w+");
		//// 若 RINEX 文件类型为 M (Mixed)，目前仅读取 GPS 数据，写文件时将 'M'改写为'G'(2013/6/28)
		//if(m_header.getSatSystemChar() == 'M')
		//{
		//	m_header.szSatlliteSystem[0] = 'G';
		//}
		fprintf(pObsfile,"%20s%-20s%-20s%20s\n", 
			              m_header.szRinexVersion,
						  m_header.szFileType,
						  m_header.szSatlliteSystem,
						  Rinex2_1_MaskString::szVerType);
		fprintf(pObsfile,"%-20s%-20s%-20s%20s\n",
			              m_header.szProgramName, 
						  m_header.szProgramAgencyName, 
						  m_header.szFileDate,
						  Rinex2_1_MaskString::szPgmRunDate);
		fprintf(pObsfile,"%-60s%20s\n",
			              m_header.szMarkName,
						  Rinex2_1_MaskString::szMarkerName);
		// 输出OBSERVER / AGENCY，2008-07-27
		fprintf(pObsfile,"%-20s%-40s%20s\n",
			              m_header.szObserverName, 
						  m_header.szObserverAgencyName, 
						  Rinex2_1_MaskString::szObservAgency);
		// 该信息不输出，2008-07-27
		// fprintf(pObsfile,"%20s%-40s%20s\n",m_header.szMarkNumber," ",Rinex2_1_MaskString::szMarkerNum);
		fprintf(pObsfile,"%20s%20s%20s%20s\n",
			              m_header.szRecNumber,
						  m_header.szRecType,
						  m_header.szRecVersion,
						  Rinex2_1_MaskString::szRecTypeVers);
		fprintf(pObsfile,"%20s%20s%-20s%20s\n",
			              m_header.szAntNumber,
						  m_header.szAntType,
						  " ",
						  Rinex2_1_MaskString::szAntType);
		fprintf(pObsfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
			              m_header.ApproxPos.x,
						  m_header.ApproxPos.y,
						  m_header.ApproxPos.z,					 
						  " ",
						  Rinex2_1_MaskString::szApproxPosXYZ);
		fprintf(pObsfile,"%14.4f%14.4f%14.4f%-18s%20s\n",
			              m_header.AntOffset.x,
						  m_header.AntOffset.y,
						  m_header.AntOffset.z,
						  " ",
						  Rinex2_1_MaskString::szAntDeltaHEN);
		if(m_header.bL1WaveLengthFact != 100)
			fprintf(pObsfile,"%6d%6d%6d%-42s%20s\n",
			              m_header.bL1WaveLengthFact,
						  m_header.bL2WaveLengthFact,
						  m_header.bL5WaveLengthFact,
			              " ",Rinex2_1_MaskString::szWaveLenFact);
		fprintf(pObsfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
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
		fprintf(pObsfile,"%6d%6d%6d%6d%6d%13.7f%-5s%3s%-9s%20s\n",
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
			fprintf(pObsfile,"%10.3f%-50s%20s\n",
			              m_header.Interval,
						  " ",
						  Rinex2_1_MaskString::szInterval);
		if(m_header.LeapSecond != INT_MAX)
			fprintf(pObsfile,"%6d%-54s%20s\n",
			              m_header.LeapSecond,
						  " ",
						  Rinex2_1_MaskString::szLeapSec);
		fprintf(pObsfile,"%6d%-54s%20s\n",
			              m_header.bySatCount,
						  " ",
						  Rinex2_1_MaskString::szNumsOfSv);
		fprintf(pObsfile,"%6d",m_header.byObsTypes);  
		for(int i = 1; i <= m_header.byObsTypes; i++)  // 如果观测数据类型个数多于最大值，写文件将出错，需要修改，2013.09.10，刘俊宏
		{// 根据 champ 格式要求此处由4X2A直接调整为6A, 具体参见 gpsObsId2String
			fprintf(pObsfile,"%6s", obsId2String(m_header.pbyObsTypeList[i-1]).c_str());
		}
		int nBlank = 60 - (6 + 6 * m_header.byObsTypes);
		string strBlank;
		strBlank.append(nBlank,' ');
		fprintf(pObsfile,"%s%20s\n", strBlank.c_str(), Rinex2_1_MaskString::szTypeOfObserv);

		// 该信息不输出, 2008/07/27
		//for(size_t s_i = 1; s_i <= m_header.pbySatList.size(); s_i++)
		//{
		//	fprintf(pObsfile,"%-3sG%2d%-54s%20s\n"," ",m_header.pbySatList[s_i-1]," ",Rinex2_1_MaskString::szPRNNumOfObs);
		//}	

		// 补充输出 Comment 信息,  2008/07/27
		for( size_t s_i = 0; s_i < m_header.pstrCommentList.size(); s_i++)
		{
			fprintf(pObsfile,"%s",m_header.pstrCommentList[s_i].c_str());
		}

		fprintf(pObsfile,"%-60s%20s\n"," ",Rinex2_1_MaskString::szEndOfHead);
		// 文件头书写完毕

		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			Rinex2_1_MixedObsEpoch obsEpoch = m_data[s_i]; //2014/03/23
			// 时间量化处理, 2007/11/06 
			// 在文件保存时,为了避免出现 60 秒, 分钟不进位的情况
			DayTime t = obsEpoch.t;
			t.second = 0;
			double sencond = Round(obsEpoch.t.second * 1.0E+7) * 1.0E-7;
			t = t + sencond;
			//Epoch/SAT--1--32
			fprintf(pObsfile," %1d%1d %2d %2d %2d %2d%11.7f  %1d%3d",
				              getIntBit(obsEpoch.t.year, 1),
							  getIntBit(obsEpoch.t.year, 0),
					          t.month,
							  t.day,
							  t.hour,
							  t.minute,
							  t.second,
							  obsEpoch.byEpochFlag,
							  obsEpoch.bySatCount);
			//32----68  32X, 12(A1,I2)  68----80  receiver clock offset F12.9
			int nLine    = obsEpoch.bySatCount / 12;
			int nResidue = obsEpoch.bySatCount % 12;
			if(obsEpoch.bySatCount <= 12) 
			{// 观测卫星个数小于等于12
				for(Rinex2_1_MixedSatMap::iterator it = obsEpoch.obs.begin(); it != obsEpoch.obs.end(); ++it)// 2014/03/23
				{
					fprintf(pObsfile,"%3s", it->first.c_str());// 2014/03/23
				}
				nBlank = 36 - (3 * obsEpoch.bySatCount); // 涂先秦 2011/06/13
				strBlank.erase(0, strBlank.length());
				strBlank.append(nBlank, ' ');
				fprintf(pObsfile, "%s\n", strBlank.c_str());
			}
			else // 观测卫星个数大于12
			{// 读取前12个卫星
				Rinex2_1_MixedSatMap::iterator it = obsEpoch.obs.begin();// 2014/03/23
				for(int j = 0; j < 12; j++)
				{
					fprintf(pObsfile,"%3s", it->first.c_str());// 2014/03/23
					++it;
				}
				fprintf(pObsfile,"\n");
				for(int j = 1; j < nLine; j++)
				{// 读取中间nLine-1行数据
					fprintf(pObsfile, "%-32s", " "); // 每一行前增加32X
					for(int ii = 0; ii < 12; ii++)
					{
						fprintf(pObsfile,"%3s", it->first.c_str());// 2014/03/23
						++it;
					}
					fprintf(pObsfile,"\n");
				}
				if(nResidue > 0)
				{// 读取最后nResidue个数据
					fprintf(pObsfile,"%-32s"," ");
					for(int ii = 0; ii < nResidue; ii++)
					{
						fprintf(pObsfile,"%3s", it->first.c_str());// 2014/03/23
						++it;
					}
					nBlank = 36 - 3 * nResidue; // 计算空白个数
					strBlank.erase(0, strBlank.length());
					strBlank.append(nBlank,' ');
					fprintf(pObsfile,"%s\n", strBlank.c_str());
				}
			}
			// 根据Epoch/SAT，解析可见观测数据--------------------
			for(Rinex2_1_MixedSatMap::iterator it = obsEpoch.obs.begin(); it != obsEpoch.obs.end(); ++it)// 2014/03/23
			{
				int nLine    = m_header.byObsTypes / 5;
				int nResidue = m_header.byObsTypes % 5;
				if(m_header.byObsTypes <= 5) // 观测类型的个数小于等于5
				{
					for(int j = 0; j < m_header.byObsTypes; j++)
					{
						Rinex2_1_ObsDatum obsDatum = it->second[j];
						if(obsDatum.data != DBL_MAX)
							//fprintf(pObsfile,"%14.3f%c%c",
							//                 obsDatum.data,
							//				 obsDatum.lli,
							//				 obsDatum.ssi);
							fprintf(pObsfile,"%16.6f%c%c",
							                 obsDatum.data,
											 obsDatum.lli,
											 obsDatum.ssi);
						else
							fprintf(pObsfile,"%-16s"," ");
					}
					fprintf(pObsfile,"\n");
				}
				else // 观测类型大于5
				{// 读取前5个数据
					for(int j = 0; j < 5; j++)
					{
						Rinex2_1_ObsDatum obsDatum = it->second[j];
						if(obsDatum.data != DBL_MAX)
							//fprintf(pObsfile,"%14.3f%c%c",
							//                 obsDatum.data,
							//				 obsDatum.lli,
							//				 obsDatum.ssi);
							fprintf(pObsfile,"%16.6f%c%c",
							                 obsDatum.data,
											 obsDatum.lli,
											 obsDatum.ssi);
						else
							fprintf(pObsfile, "%-16s", " ");
					}
					fprintf(pObsfile, "\n");
					for(int j = 1; j < nLine; j++)
					{// 读取中间nLine-1行数据
						for(int ii = 0; ii < 5; ii++)
						{
							Rinex2_1_ObsDatum obsDatum = it->second[j * 5 + ii];
							if(obsDatum.data != DBL_MAX)
								//fprintf(pObsfile,"%14.3f%c%c",
								//                 obsDatum.data,
								//				 obsDatum.lli,
								//				 obsDatum.ssi);
								fprintf(pObsfile,"%16.6f%c%c",
								                 obsDatum.data,
												 obsDatum.lli,
												 obsDatum.ssi);
							else
								fprintf(pObsfile,"%-16s"," ");
						}
						fprintf(pObsfile, "\n");
					}
					if(nResidue > 0)
					{// 读取最后nResidue个数据
						for(int j = 0; j < nResidue; j++)
						{
							Rinex2_1_ObsDatum obsDatum = it->second[nLine * 5 + j];
							if(obsDatum.data != DBL_MAX)
								//fprintf(pObsfile,"%14.3f%c%c",
								//                 obsDatum.data,
								//				 obsDatum.lli,
								//				 obsDatum.ssi);
								fprintf(pObsfile,"%16.6f%c%c",
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
		}
		fclose(pObsfile);
		return true;
	}

	bool Rinex2_1_MixedObsFile::write(string strObsfileName_noExp)
	{
		string strObsfileName_all;
		return write(strObsfileName_noExp, strObsfileName_all);
	}


}
