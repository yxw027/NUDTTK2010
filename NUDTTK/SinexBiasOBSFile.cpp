#include "SinexBiasOBSFile.hpp"
#include "structDef.hpp"
#include <string>

namespace NUDTTK
{
	SinexBiasOBSFile::SinexBiasOBSFile(void)
	{
	}
	SinexBiasOBSFile::~SinexBiasOBSFile(void)
	{
	}
	DayTime SinexBiasOBSFile::doy2daytime(int year, int doy, double second)
	{
		DayTime t(year, 1, 1, 0, 0, second);
		t = t + (doy - 1) * 86400.0;
		return t;
	}
    bool SinexBiasOBSFile::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}
	bool SinexBiasOBSFile::isValidEpochLine(string strLine, SinexBiasOBSLine& obsline)
	{
	  bool   nFlag = true;
	  char   szBiasName[4+1]; 	
	  char   szSVNName[4+1]; 		
	  char   szPRNName[3+1]; 		
	  char   szStationName[9+1]; 		
	  char   szObs1Name[4+1]; 		
	  char   szObs2Name[4+1]; 				
	  GPST   t0;		
	  GPST   t1;
	  char   szT0Year[4+1]; 		
	  char   szT0Doy[3+1];
	  char   szT0Sec[5+1];
	  char   szT1Year[4+1]; 		
	  char   szT1Doy[3+1]; 
	  char   szT1Sec[5+1];
	  char   szUnitName[4+1];
	  double obsvalue = 0.0;		
	  double obsrms = 0.0;
	  char   OBSvalue[10+1];
	  char   OBSrms[11+1];
	  sscanf(strLine.c_str(), "%*1c%4c%*1c%4c%*1c%3c%*1c%9c%*1c%4c%*1c%4c%*1c%4c%*1c%3c%*1c%5c%*1c%4c%*1c%3c%*5c%1c%4c%*12c%10c%*1c%11c",
								szBiasName,szSVNName,szPRNName,szStationName,szObs1Name,szObs2Name,
								szT0Year,szT0Doy,szT0Sec,
	                            szT1Year,szT1Doy,szT1Sec,
								szUnitName,
								OBSvalue,
								OBSrms);
		szBiasName[4]    = '\0'; 		
		szSVNName[4]     = '\0'; 		
		szPRNName[3]     = '\0'; 		
		szStationName[9] = '\0'; 		
		szObs1Name[4]    = '\0'; 		
		szObs2Name[4]    = '\0';
		szUnitName[4]    = '\0';
		szT0Year[4]      = '\0'; 		
	    szT0Doy[3]       = '\0';
		szT0Sec[5]       = '\0';
 	    szT1Year[4]      = '\0'; 		
	    szT1Doy[3]       = '\0';
		szT1Sec[5]       = '\0';
        OBSvalue[10]     = '\0';
	    OBSrms[11]       = '\0';
		int T0_year   = atoi(szT0Year);
		int T0_doy    = atoi(szT0Doy);
		double T0_Second   = atof(szT0Sec);
		if (T0_year == 0 && T0_doy == 0)
		{
		  nFlag = false;
		  return nFlag;
		}
		else
		{
          t0 = doy2daytime(T0_year,T0_doy,T0_Second);
		}
		int T1_year   = atoi(szT1Year);
		int T1_doy    = atoi(szT1Doy);
		double T1_Second   = atof(szT1Sec);
		if (T1_year == 0 && T1_doy == 0)
		{
		   nFlag = false;
		   return nFlag;
		}
		else
		{
           t1 = doy2daytime(T1_year,T1_doy,T1_Second);
		}
        obsvalue = atof(OBSvalue);
        obsrms   = atof(OBSrms);

		strcpy(obsline.szBiasName, szBiasName);
		strcpy(obsline.szSVNName, szSVNName);
		strcpy(obsline.szPRNName, szPRNName);
		strcpy(obsline.szStationName, szStationName);
		strcpy(obsline.szObs1Name, szObs1Name);
		strcpy(obsline.szObs2Name, szObs2Name);
		obsline.t0             = t0;
		obsline.t1             = t1;
		strcpy(obsline.szUnitName, szUnitName);
		obsline.obsvalue       = obsvalue;
		obsline.obsrms         = obsrms;
		nFlag = true;

		return nFlag;
	}

	// 子程序名称： open   
	// 功能：读取 SinexBiasOBS 文件数据
	// 变量类型：strSinexBiasOBSFileName DCB文件名称
	// 输入：strSinexBiasOBSFileName
	// 输出：
	// 语言：C++
	// 创建者：易彬
	// 创建时间：2018/5/15
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool SinexBiasOBSFile::open(string  strSinexBiasOBSFileName)
	{
		FILE *pSinexBiasOBSFile = fopen(strSinexBiasOBSFileName.c_str(), "r+t");
		if(pSinexBiasOBSFile == NULL) 
			return false;
		char line[200];
		m_data.clear();
		while(!feof(pSinexBiasOBSFile))
		{
			if(fgets(line, 200, pSinexBiasOBSFile))
			{
			   string strline = string(line).substr(0,4);
               if(strline != " DSB" && strline != " ISB" && strline != " OSB")
				{
				 continue;
				}
				else
				{
				   SinexBiasOBSLine datum;
				   if(isValidEpochLine(line, datum))
				   {
					  string PRNObs1 = string(datum.szPRNName).substr(0,3) + string(datum.szObs1Name).substr(0,3);
					  SinexBiasOBSMap::iterator it = m_data.find(PRNObs1);
					 if(it == m_data.end())
					 {
						vector<SinexBiasOBSLine> SinexBiasOBSLineList;
						SinexBiasOBSLineList.push_back(datum);
						  m_data[PRNObs1] = SinexBiasOBSLineList;						
					 }
					 else 
					 {
					   it->second.push_back(datum);
					 }
				   }
				}
		    }
		}
		fclose(pSinexBiasOBSFile);
		return true;
	}
	//
	// 子程序名称： getOBSCorrectValue_Day   
	// 功能：获取天线修正数据结构
	// 变量类型：t        : 时间
	//			 PRNName  ：卫星号
	//			 Obs1Name ：观测数据类型1
	//           Obs2Name ：观测数据类型2
	//           dcbvalue ：OBS值
	// 输入：t, PRNName, Obs1Name, Obs2Name
	// 输出：dcbvalue
	// 语言：C++
	// 创建者：易彬
	// 创建时间：2018/5/15
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool SinexBiasOBSFile::getOBSCorrectValue_Day(GPST t, string PRNName, string Obs1Name, double &obsvalue)
	{
		obsvalue = 0;
		string PRNObs1 = string(PRNName).substr(0,3) + string(Obs1Name).substr(0,3);            
		//判断所需类型DCB是否存在？
		SinexBiasOBSMap::iterator it = m_data.find(PRNObs1);
		if(it == m_data.end())
		{
		  printf("输入的%s在文件中不存在!\n", PRNObs1.c_str());
          return false;
		}
		//判断所需时间DCB是否存在，若不存在取拟合值？
        for(size_t s_i = 0; s_i < it->second.size(); s_i++)   
		{
		    GPST t0 = it->second[s_i].t0;
			int t_doy = t.doy();
			int t0_doy = t0.doy();
            if(t_doy == t0_doy)  //应判断doy相等？
		   {
		     obsvalue = it->second[s_i].obsvalue;
			 string strline = string(it->second[s_i].szBiasName).substr(0,3);
             if(strline == "OBS" )
			  {
				printf("%s，%s, OBS_Day = %10.4lf\n", t.toString().c_str(), PRNObs1.c_str(), obsvalue);
			  }
		     return true;
		     break;
		    }
		}
		if(obsvalue == 0)
		{
			printf("输入的%s在时间%s不存在!\n", PRNObs1.c_str(), t.toString().c_str());
			return false;
		}
		else
		{
		  return true;
		}
	}
}