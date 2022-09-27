#include  "BDRawObsFile.hpp"

namespace NUDTTK
{
	
	BDRawObsFile::BDRawObsFile(void)
	{
	}

	BDRawObsFile::~BDRawObsFile(void)
	{
	}
	bool BDRawObsFile::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}
	// 子程序名称： BDRawObsFile::isValidEpochLine   
    // 功能：判断读入的字符串是否有效
    // 变量类型：strLine   : 读入的字符串 
    //           line　　  : 北斗实验数据行结构
    // 输入：strLine
	// 输出：line
	// 语言：C++
    // 创建者：刘俊宏
    // 创建时间：2012/3/4
    // 版本时间：2012/3/4
    // 修改记录：
    // 其它：
	bool BDRawObsFile::isValidEpochLine(string strLine, BDRawObsFileline& line)
	{
		bool bflag = true;
		char szBDT_SECOND[100];
		char szBDT_NAVWN[100];
		char szBDT_LOCALWN[100];
		char szSAT_ID[100];
		char szB1IW[100];
		char szB1IN[100];
		char szB1IA[100];
		char szB1QW[100];
		char szB1QN[100];
		char szB1QA[100];
		char szB2IW[100];
		char szB2IN[100];
		char szB2IA[100];
		char szB2QW[100];
		char szB2QN[100];
		char szB2QA[100];
		char szB3IW[100];
		char szB3IN[100];
		char szB3IA[100];
		char szB3QW[100];
		char szB3QN[100];
		char szB3QA[100];
		char szB1IC[100];
		char szB1QC[100];
		char szB2IC[100];
		char szB2QC[100];
		char szB3IC[100];
		char szB3QC[100];
		char szB1D[100];
		char szB2D[100];
		char szB3D[100];
		char szB1ICN[100];
		char szB1QCN[100];
		char szB2ICN[100];
		char szB2QCN[100];
		char szB3ICN[100];
		char szB3QCN[100];
		sscanf(strLine.c_str(),"%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
							   szBDT_SECOND,
							   szBDT_NAVWN,
							   szBDT_LOCALWN,
							   szSAT_ID,
							   szB1IW,
							   szB1IN,
							   szB1IA,
							   szB1QW,
							   szB1QN,
							   szB1QA,
							   szB2IW,
							   szB2IN,
							   szB2IA,
							   szB2QW,
							   szB2QN,
							   szB2QA,
							   szB3IW,
							   szB3IN,
							   szB3IA,
							   szB3QW,
							   szB3QN,
							   szB3QA,
							   szB1IC,
							   szB1QC,
							   szB2IC,
							   szB2QC,
							   szB3IC,
							   szB3QC,
							   szB1D,
							   szB2D,
							   szB3D,
							   szB1ICN,
							   szB1QCN,
							   szB2ICN,
							   szB2QCN,
							   szB3ICN,
							   szB3QCN);
		line.BDT_SECOND    = atof(szBDT_SECOND);
		line.BDT_NAVWN     = atoi(szBDT_NAVWN);
		line.BDT_LOCALWN   = atoi(szBDT_LOCALWN);
		line.SAT_ID  = atoi(szSAT_ID);
		line.B1IW    = atof(szB1IW);
		line.B1IN    = atof(szB1IN);
		line.B1IA    = atof(szB1IA);
		line.B1QW    = atof(szB1QW);
		line.B1QN    = atof(szB1QN);
		line.B1QA    = atof(szB1QA);
		line.B2IW    = atof(szB2IW);
		line.B2IN    = atof(szB2IN);
		line.B2IA    = atof(szB2IA);
		line.B2QW    = atof(szB2QW);
		line.B2QN    = atof(szB2QN);
		line.B2QA    = atof(szB2QA);
		line.B3IW    = atof(szB3IW);
		line.B3IN    = atof(szB3IN);
		line.B3IA    = atof(szB3IA);
		line.B3QW    = atof(szB3QW);
		line.B3QN    = atof(szB3QN);
		line.B3QA    = atof(szB3QA);
		line.B1IC    = atof(szB1IC);
		line.B1QC    = atof(szB1QC);
		line.B2IC    = atof(szB2IC);
		line.B2QC    = atof(szB2QC);
		line.B3IC    = atof(szB3IC);
		line.B3QC    = atof(szB3QC);
		line.B1D     = atof(szB1D);
		line.B2D     = atof(szB2D);
		line.B3D     = atof(szB3D);
		line.B1ICN   = atof(szB1ICN);
		line.B1QCN   = atof(szB1QCN);
		line.B2ICN   = atof(szB2ICN);
		line.B2QCN   = atof(szB2QCN);
		line.B3ICN   = atof(szB3ICN);
		line.B3QCN   = atof(szB3QCN);			
		if(line.BDT_SECOND < 0 || line.BDT_NAVWN <= 0 || line.BDT_LOCALWN <= 0||line.SAT_ID <= 0)
			bflag = false;
		return bflag;
	}
	// 子程序名称： BDRawObsFile::open   
    // 功能：北斗实验数据解析 
    // 变量类型：strBDRawObsFileName : 北斗实验数据文件路径
    // 输入：    strBDRawObsFileName
    // 输出：
    // 语言：C++
    // 创建者：刘俊宏
    // 创建时间：2012/3/4
    // 版本时间：2012/3/4
    // 修改记录：
	// 其它： 
	bool BDRawObsFile::open(string  strBDRawObsFileName)
	{
		FILE * pfile = fopen(strBDRawObsFileName.c_str(), "r+t");
		if(pfile == NULL) 
			return false;
		char line[700];
		m_data.clear();
		fgets(BDRawObsFileHeader, sizeof(BDRawObsFileHeader), pfile);
		BDRawObsFiledata    BDrawobsline;
		string  BDSation=bdStationId2String(string2BDStationId(strBDRawObsFileName));//从文件路径中获取测站名称
		BDrawobsline.STA_ID[0] = BDSation[0];
		BDrawobsline.STA_ID[1] = BDSation[1];
		BDrawobsline.STA_ID[2] = BDSation[2];
		BDrawobsline.STA_ID[3] = BDSation[3];	
		while(!feof(pfile))
		{
			if(fgets(line, sizeof(line), pfile))
			{
				if(isValidEpochLine(line, BDrawobsline.BDrawobsfileline))
				{					
					m_data.push_back(BDrawobsline);					
				}
					
			}
		}
		fclose(pfile);
		return true;
	}
	// 子程序名称： BDRawObsFile::write   
    // 功能：将北斗实验数据写入文件 
    // 变量类型：strBDRawObsFileName : 输出的文件路径
    // 输入：
    // 输出：
    // 语言：C++
    // 创建者：刘俊宏
    // 创建时间：2012/3/4
    // 版本时间：2012/3/4
    // 修改记录：
	// 其它：
	bool BDRawObsFile::write(string  strBDRawObsFileName)
	{
		FILE* pfile = fopen(strBDRawObsFileName.c_str(), "w+");
		fprintf(pfile, "%s",BDRawObsFileHeader);
        for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{			
			fprintf(pfile, "%9.2f %4d %4d  %02d %19.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %21.3f %16.4f %16.4f %16.4f %11.3f %11.3f %11.3f %11.3f %11.3f %11.3f\n",
				            m_data[s_i].BDrawobsfileline.BDT_SECOND,
				            m_data[s_i].BDrawobsfileline.BDT_NAVWN,
							m_data[s_i].BDrawobsfileline.BDT_LOCALWN,
							m_data[s_i].BDrawobsfileline.SAT_ID,
							m_data[s_i].BDrawobsfileline.B1IW,
							m_data[s_i].BDrawobsfileline.B1IN,
							m_data[s_i].BDrawobsfileline.B1IA,
							m_data[s_i].BDrawobsfileline.B1QW,
							m_data[s_i].BDrawobsfileline.B1QN,
							m_data[s_i].BDrawobsfileline.B1QA,
							m_data[s_i].BDrawobsfileline.B2IW,
							m_data[s_i].BDrawobsfileline.B2IN,
							m_data[s_i].BDrawobsfileline.B2IA,
							m_data[s_i].BDrawobsfileline.B2QW,
							m_data[s_i].BDrawobsfileline.B2QN,
							m_data[s_i].BDrawobsfileline.B2QA,
							m_data[s_i].BDrawobsfileline.B3IW,
							m_data[s_i].BDrawobsfileline.B3IN,
							m_data[s_i].BDrawobsfileline.B3IA,
							m_data[s_i].BDrawobsfileline.B3QW,
							m_data[s_i].BDrawobsfileline.B3QN,
							m_data[s_i].BDrawobsfileline.B3QA,
							m_data[s_i].BDrawobsfileline.B1IC,
							m_data[s_i].BDrawobsfileline.B1QC,
							m_data[s_i].BDrawobsfileline.B2IC,
							m_data[s_i].BDrawobsfileline.B2QC,
							m_data[s_i].BDrawobsfileline.B3IC,
							m_data[s_i].BDrawobsfileline.B3QC,
							m_data[s_i].BDrawobsfileline.B1D,
							m_data[s_i].BDrawobsfileline.B2D,
							m_data[s_i].BDrawobsfileline.B3D,
							m_data[s_i].BDrawobsfileline.B1ICN,
							m_data[s_i].BDrawobsfileline.B1QCN,
							m_data[s_i].BDrawobsfileline.B2ICN,
							m_data[s_i].BDrawobsfileline.B2QCN,
							m_data[s_i].BDrawobsfileline.B3ICN,
							m_data[s_i].BDrawobsfileline.B3QCN);
		}
		fclose(pfile);
		return true;
	}
	// 子程序名称： bdRaw_To_Rinex2_1   
	// 功能：北斗原始数据转化为Rinex2.1格式
	// 变量类型：rinexObsFile                    : Rinex2.1格式的观测数据
	//           interval                        : 采样间隔,原始观测数据位1s采样间隔，此处需要做降采样处理
	// 输入： interval
	// 输出： rinexObsFile
	// 语言： C++
	// 创建者：刘俊宏
	// 创建时间：2013/11/10
	// 版本时间：2013/11/10
	// 修改记录：
	// 其它： 直接转为GPST的观测数据，方便以后与Mgex观测数据联合定轨
	bool    BDRawObsFile::bdRaw_To_Rinex2_1(Rinex2_1_ObsFile &rinexObsFile, int interval)
	{
		if(m_data.size() <= 300)
		{
			printf("原始数据太少！不进行格式转换!\n");
			return  false;
		}
		else
		{			
			BYTE pbySatList[MAX_PRN_BD + 1]; // 卫星列表
		    for(int i = 0; i < MAX_PRN_BD + 1; i++)
				pbySatList[i] = 0;
			rinexObsFile.clear();		
			GPST           t_now;              //当前时间
			GPST Begin_t = TimeCoordConvert::BDT2GPST(m_data.front().BDrawobsfileline.gettime());			
			GPST End_t   = TimeCoordConvert::BDT2GPST(m_data.back().BDrawobsfileline.gettime());		
			GPST t0      = Begin_t;
			t0.hour    = 0;
			t0.minute  = 0;
			t0.second  = 0;
			char   RinexVersion[20+1]      = "     2.1            ";
			char   FileType[20+1]          = "OBSERVATION DATA    ";					
			char   ProgramName[20+1]       = "Experiment          ";
			char   ProgramAgencyName[20+1] = "NUDT                ";
			char   ObserverName[20+1]      = "CASC                ";
			char   RecNumber[20+1]         = "4922K35365          ";
			char   RecType[20+1]           = "ASHTECH Z-XII3      ";
			char   AntType[20+1]           = "ASH700936D_M    NONE";					
			rinexObsFile.m_header.szSatlliteSystem[0] = 'C';
			rinexObsFile.m_header.byObsTypes = 6;
			rinexObsFile.m_header.pbyObsTypeList[0] = string2ObsId("L1");
			rinexObsFile.m_header.pbyObsTypeList[1] = string2ObsId("L2");
			rinexObsFile.m_header.pbyObsTypeList[2] = string2ObsId("P1");
			rinexObsFile.m_header.pbyObsTypeList[3] = string2ObsId("P2");
			rinexObsFile.m_header.pbyObsTypeList[4] = string2ObsId("L5");
			rinexObsFile.m_header.pbyObsTypeList[5] = string2ObsId("P5");
			rinexObsFile.m_header.Interval = interval;	
			strcpy(rinexObsFile.m_header.szRinexVersion, RinexVersion);
			strcpy(rinexObsFile.m_header.szFileType, FileType);
			strcpy(rinexObsFile.m_header.szProgramName, ProgramName);
			strcpy(rinexObsFile.m_header.szProgramAgencyName, ProgramAgencyName);
			strcpy(rinexObsFile.m_header.szMarkName, m_data[0].STA_ID);
			strcpy(rinexObsFile.m_header.szObserverName, ObserverName);
			strcpy(rinexObsFile.m_header.szObserverAgencyName, ObserverName);
			strcpy(rinexObsFile.m_header.szRecNumber, RecNumber);
			strcpy(rinexObsFile.m_header.szRecType, RecType);
			strcpy(rinexObsFile.m_header.szAntType, AntType);
			size_t     j = 0;  //  
			Rinex2_1_ObsEpoch             obsEpoch;
			obsEpoch.byEpochFlag          = 0;
			for(size_t s_i = 0; s_i < m_data.size();s_i ++)
			{					
				//if(m_data[s_i].BDrawobsfileline.BDT_SECOND < 11252.0)
				//	continue;
				//if(m_data[s_i].BDrawobsfileline.BDT_SECOND >= 11252.0 && m_data[s_i].BDrawobsfileline.BDT_SECOND <= 118980.0)
				//	m_data[s_i].BDrawobsfileline.BDT_SECOND += 485820;
				// 给定一个时间 t，根据 interval 大小，量化成一个有效时间 ti，
				// 通过判断 t 与 ti 的接进程度，来决定 t 是否就是所找的输出点
				t_now = TimeCoordConvert::BDT2GPST(m_data[s_i].BDrawobsfileline.gettime());
				double t = t_now - t0;
				double integer  = floor(t / interval);      // 量化过程
				double fraction = t - integer * interval;   // 接近程度判断				
				if(fabs(fraction) < 0.05)
				{
					vector<Rinex2_1_ObsDatum>     BDObsTypeList;
					Rinex2_1_ObsDatum             BDObsDatum[6];  //记录6种观测数据，分别是3个频率的伪距和相位测量值
					// 添加记录点
					BDObsDatum[0].data = m_data[s_i].BDrawobsfileline.B1IC;
					BDObsDatum[1].data = m_data[s_i].BDrawobsfileline.B2IC;
					BDObsDatum[2].data = m_data[s_i].BDrawobsfileline.B1IW;
					BDObsDatum[3].data = m_data[s_i].BDrawobsfileline.B2IW;							
					BDObsDatum[4].data = m_data[s_i].BDrawobsfileline.B3IC;
					BDObsDatum[5].data = m_data[s_i].BDrawobsfileline.B3IW;								
					BYTE SAT_ID = m_data[s_i].BDrawobsfileline.SAT_ID;
					pbySatList[SAT_ID] = 1; // 可见卫星PRN标号处标记1
					BDObsTypeList.clear();
					for(int i = 0;i < 6;i ++)
						BDObsTypeList.push_back(BDObsDatum[i]);	
					if(obsEpoch.obs.size() == 0)
					{
						obsEpoch.t    = t_now;
						obsEpoch.obs.insert(Rinex2_1_SatMap::value_type(SAT_ID, BDObsTypeList));
					}
					else
					{
						if(obsEpoch.t == t_now)
							obsEpoch.obs.insert(Rinex2_1_SatMap::value_type(SAT_ID, BDObsTypeList));
						else
						{
							obsEpoch.bySatCount = (int)obsEpoch.obs.size();
							rinexObsFile.m_data.push_back(obsEpoch);
							obsEpoch.t = t_now;
							obsEpoch.obs.clear();							
							obsEpoch.obs.insert(Rinex2_1_SatMap::value_type(SAT_ID, BDObsTypeList));
						}
					}					
					
				}
			}
			if(rinexObsFile.m_data.size() > 0)
			{
				// 综合统计可视卫星列表
				rinexObsFile.m_header.pbySatList.clear();
				for(int i = 0; i <= MAX_PRN_BD; i++)
				{
					if(pbySatList[i] == 1)
					{
						rinexObsFile.m_header.pbySatList.push_back(BYTE(i));
					}
				}
				rinexObsFile.m_header.bySatCount = BYTE(rinexObsFile.m_header.pbySatList.size());
				rinexObsFile.m_header.tmStart    = rinexObsFile.m_data.front().t;
				rinexObsFile.m_header.tmEnd      = rinexObsFile.m_data.back().t;
				return true;
			}
			else
			{
				printf("没有符合要求的观测数据\n");
				return false;
			}

		}
	}
}