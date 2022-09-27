#include "SinexBiasDCBFile.hpp"
#include "structDef.hpp"
#include <string>

namespace NUDTTK
{
	SinexBiasDCBFile::SinexBiasDCBFile(void)
	{
	}
	SinexBiasDCBFile::~SinexBiasDCBFile(void)
	{
	}
	DayTime SinexBiasDCBFile::doy2daytime(int year, int doy, double second)
	{
		DayTime t(year, 1, 1, 0, 0, second);
		t = t + (doy - 1) * 86400.0;
		return t;
	}
    bool SinexBiasDCBFile::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}
	bool SinexBiasDCBFile::isValidEpochLine(string strLine, SinexBiasDCBLine& dcbline)
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
	  //char   szT0Second[5+1];
	  char   szT1Year[4+1]; 		
	  char   szT1Doy[3+1]; 
	  //char   szT1Second[5+1];
	  char   szUnitName[4+1];
	  double dcbvalue = 0.0;		
	  double dcbrms = 0.0;
	  char   DCBvalue[10+1];
	  char   DCBrms[11+1];
	 //*BIAS SVN_ PRN STATION__ OBS1 OBS2 BIAS_START____ BIAS_END______ UNIT __ESTIMATED_VALUE____ _STD_DEV___
	 // DSB  G063 G01           C1C  C1W  2018:001:00000 2018:002:00000 ns                 -1.0580      0.0085
	 //%*1s%4s%*1s%4s%*1s%3s%*1s%9s%*1s%4s%*1s%4s%*1s%4d%*1s%3d%*1s%5d%*1s%4d%*1s%3d%*1s%5d %*1s%4s%*12s%10lf%*1s%11lf
	  sscanf(strLine.c_str(), "%*1c%4c%*1c%4c%*1c%3c%*1c%9c%*1c%4c%*1c%4c%*1c%4c%*1c%3c%*7c%4c%*1c%3c%*7c%4c%*12c%10c%*1c%11c",
								szBiasName,szSVNName,szPRNName,szStationName,szObs1Name,szObs2Name,
								szT0Year,szT0Doy,
	                         szT1Year,szT1Doy,
								szUnitName,
								DCBvalue,
								DCBrms);
		szBiasName[4]    = '\0'; 		
		szSVNName[4]     = '\0'; 		
		szPRNName[3]     = '\0'; 		
		szStationName[9] = '\0'; 		
		szObs1Name[4]    = '\0'; 		
		szObs2Name[4]    = '\0';
		szUnitName[4]    = '\0';
		szT0Year[4]      = '\0'; 		
	    szT0Doy[3]       = '\0';
	    szT1Year[4]      = '\0'; 		
	    szT1Doy[3]       = '\0';
        DCBvalue[10]     = '\0';
	    DCBrms[11]       = '\0';
		int T0_year   = atoi(szT0Year);
		int T0_doy    = atoi(szT0Doy);
		if (T0_year == 0 && T0_doy == 0)
		{
		  nFlag = false;
		  return nFlag;
		}
		else
		{
          t0 = doy2daytime(T0_year,T0_doy,0.0);
		}
		int T1_year   = atoi(szT1Year);
		int T1_doy    = atoi(szT1Doy);
		if (T1_year == 0 && T1_doy == 0)
		{
		   nFlag = false;
		   return nFlag;
		}
		else
		{
           t1 = doy2daytime(T1_year,T1_doy,0.0);
		}
        dcbvalue = atof(DCBvalue);
        dcbrms   = atof(DCBrms);

	    if(fabs(dcbrms) >= 1 || dcbrms == 0)
		{
		  nFlag = false;
		}
		else
		{
			strcpy(dcbline.szBiasName, szBiasName);
			strcpy(dcbline.szSVNName, szSVNName);
			strcpy(dcbline.szPRNName, szPRNName);
			strcpy(dcbline.szStationName, szStationName);
			strcpy(dcbline.szObs1Name, szObs1Name);
			strcpy(dcbline.szObs2Name, szObs2Name);
			dcbline.t0             = t0;
			dcbline.t1             = t1;
			strcpy(dcbline.szUnitName, szUnitName);
			dcbline.dcbvalue       = dcbvalue;
			dcbline.dcbrms         = dcbrms;
			nFlag = true;
	     }
		return nFlag;
	}

	// 子程序名称： open   
	// 功能：读取 SinexBiasDCB 文件数据
	// 变量类型：strSinexBiasDCBFileName DCB文件名称
	// 输入：strSinexBiasDCBFileName
	// 输出：
	// 语言：C++
	// 创建者：易彬
	// 创建时间：2018/5/15
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool SinexBiasDCBFile::open(string  strSinexBiasDCBFileName)
	{
		FILE *pSinexBiasDCBFile = fopen(strSinexBiasDCBFileName.c_str(), "r+t");
		if(pSinexBiasDCBFile == NULL) 
			return false;
		char line[200];
		m_data.clear();
		while(!feof(pSinexBiasDCBFile))
		{
			if(fgets(line, 200, pSinexBiasDCBFile))
			{
				string strline = string(line).substr(0,4);
               if(strline != " DSB" && strline != " ISB" && strline != " OSB")
				{
				 continue;
				}
				else
				{
				   SinexBiasDCBLine datum;
				   if(isValidEpochLine(line, datum))
				   {
					  string PRNObs1Obs2 = string(datum.szPRNName).substr(0,3) + string(datum.szObs1Name).substr(0,3) + string(datum.szObs2Name).substr(0,3);
					  SinexBiasDCBMap::iterator it = m_data.find(PRNObs1Obs2);
					 if(it == m_data.end())
					 {
						vector<SinexBiasDCBLine> SinexBiasDCBLineList;
						SinexBiasDCBLineList.push_back(datum);
						  m_data[PRNObs1Obs2] = SinexBiasDCBLineList;						
					 }
					 else 
					 {
					   it->second.push_back(datum);
					 }
				   }
				}
		    }
		}
		fclose(pSinexBiasDCBFile);
		return true;
	}
	//
	// 子程序名称： getDCBCorrectValue_Day   
	// 功能：获取天线修正数据结构
	// 变量类型：t        : 时间
	//			 PRNName  ：卫星号
	//			 Obs1Name ：观测数据类型1
	//           Obs2Name ：观测数据类型2
	//           dcbvalue ：DCB值
	// 输入：t, PRNName, Obs1Name, Obs2Name
	// 输出：dcbvalue
	// 语言：C++
	// 创建者：易彬
	// 创建时间：2018/5/15
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool SinexBiasDCBFile::getDCBCorrectValue_Day(GPST t, string PRNName, string Obs1Name, string Obs2Name, double &dcbvalue)
	{
		dcbvalue = 0;
		string PRNObs1Obs2 = string(PRNName).substr(0,3) + string(Obs1Name).substr(0,3) + string(Obs2Name).substr(0,3);
                     
		//判断所需类型DCB是否存在？
		SinexBiasDCBMap::iterator it = m_data.find(PRNObs1Obs2);
		if(it == m_data.end())
		{
		  //printf("输入的%s文件中不存在!\n", PRNObs1Obs2.c_str());
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
		     dcbvalue = it->second[s_i].dcbvalue;
			 string strline = string(it->second[s_i].szBiasName).substr(0,3);
              if(strline == "DSB" )
			   {
				printf("%s，%s, DSB_Day = %10.4lf\n", t.toString().c_str(), PRNObs1Obs2.c_str(), dcbvalue);
			   }
              else 
			  {
				 if(strline == "ISB" )
			     {
				   printf("%s，%s, ISB_Day = %10.4lf\n", t.toString().c_str(), PRNObs1Obs2.c_str(), dcbvalue);
			     }
			     else 
			     {
				   printf("%s，%s, OSB_Day = %10.4lf\n", t.toString().c_str(), PRNObs1Obs2.c_str(), dcbvalue);
			     }
			   }
		     return true;
		     break;
		    }
		}
		if(dcbvalue == 0)
		{
			//printf("输入的%s在时间%s不存在!\n", PRNObs1Obs2.c_str(), t.toString().c_str());
			return false;
		}
		else
		{
		  return true;
		}
	}

    // 子程序名称： getDCBCorrectValue_Month   
	// 功能：获取天线修正数据结构
	// 变量类型：t        : 时间 GPST
	//			 PRNName  ：卫星号，如：G01
	//			 Obs1Name ：观测数据类型1, 如：C1C
	//           Obs2Name ：观测数据类型2, 如：C1W
	//           dcbvalue ：DCB值，单位：ns
	// 输入：t, PRNName, Obs1Name, Obs2Name
	// 输出：dcbvalue
	// 语言：C++
	// 创建者：易彬
	// 创建时间：2018/5/15
	// 版本时间：
	// 修改记录：1、增加组合情况的DCB值，如：由 C2W-C2L 和 C2W-C2S 获得 C2L-C2S，邵凯，2018.08.13
	// 备注：
	bool SinexBiasDCBFile::getDCBCorrectValue_Month(GPST t, string PRNName, string Obs1Name, string Obs2Name, double &dcbvalue)
	{
		dcbvalue = 0;
		// 判断文件是否直接含有该卫星观测数据类型DCB数据
       string PRNObs1Obs2 = string(PRNName).substr(0,3) + string(Obs1Name).substr(0,3) + string(Obs2Name).substr(0,3);
	   string PRNObs2Obs1 = string(PRNName).substr(0,3) + string(Obs2Name).substr(0,3) + string(Obs1Name).substr(0,3);
	   SinexBiasDCBMap::iterator it = m_data.find(PRNObs1Obs2);
	   SinexBiasDCBMap::iterator jt = m_data.find(PRNObs2Obs1);
	   if(it != m_data.end() || jt != m_data.end())
	   {
  			vector<double> dcbvalueList; 
			vector<double> dcbrmsList; 
			double sum = 0;
			int t_doy = t.doy();
			//判断所需时间DCB是否存在，若不存在取拟合值？
			if(jt == m_data.end())
			{
				for(size_t s_i = 0; s_i < it->second.size(); s_i++)   
				{
					GPST t0 = it->second[s_i].t0;
					int t0_doy = t0.doy();
					if(abs(t_doy - t0_doy) <= 15)  //应判断doy相等？
				   {
					  dcbvalueList.push_back(it->second[s_i].dcbvalue);
					  dcbrmsList.push_back(it->second[s_i].dcbrms);
					  sum += 1.0 / it->second[s_i].dcbrms;
				   }
				}
			}
			else
			{
				for(size_t s_i = 0; s_i < jt->second.size(); s_i++)   
				{
					GPST t0 = jt->second[s_i].t0;
					int t0_doy = t0.doy();
					if(abs(t_doy - t0_doy) <= 15)  //应判断doy相等？
				   {
					  dcbvalueList.push_back(-jt->second[s_i].dcbvalue);  // 负值
					  dcbrmsList.push_back(jt->second[s_i].dcbrms);
					  sum += 1.0 / jt->second[s_i].dcbrms;
				   }
				}
			}
			for(size_t i = 0; i < dcbvalueList.size(); i++)
			{
			  dcbvalue += 1.0 / dcbrmsList[i] / sum * dcbvalueList[i];
			}
	   }
	   else
	   {
	  // 		// 找到卫星号(如 G01)对应的所有源文件观测数据组合列表和组合数
			//int n_PRNObs1Obs2;
			//vector<string> PRNObs1Obs2_List;   // PRNObs1Obs2 列表 
			//for(SinexBiasDCBMap::iterator it = m_data.begin(); it != m_data.end(); ++ it)
			//{
			//	if(it->first.find(string(PRNName).substr(0,3)))
			//		PRNObs1Obs2_List.push_back(it->first);
			//}
			//n_PRNObs1Obs2 = int(PRNObs1Obs2_List.size()) + 1;
			//vector<string> Obs_List;          // 观测数据类型列表
			//for(size_t s_i = 0; i < PRNObs1Obs2_List.size(); s_i ++)
			//{
			//	Obs_List.push_back(PRNObs1Obs2_List[s_i].substr(3,6));
			//}
	   }
		if(dcbvalue == 0)
		{
			printf("输入的%s在时间%s附近一个月内无DCB值!\n", PRNObs1Obs2.c_str(), t.toString().c_str());
			return false;
		}
		else
		{
			 string strline = string(it->second[0].szBiasName).substr(0,3);
			 if(strline == "DSB" )
			 {
				printf("%s，%s, DSB_Month = %10.4lf\n", t.toString().c_str(), PRNObs1Obs2.c_str(), dcbvalue);
			 }
			 else if(strline == "ISB" )
			 {
			   printf("%s，%s, ISB_Month = %10.4lf\n", t.toString().c_str(), PRNObs1Obs2.c_str(), dcbvalue);
			 }
			 else 
			 {
			   printf("%s，%s, OSB_Month = %10.4lf\n", t.toString().c_str(), PRNObs1Obs2.c_str(), dcbvalue);
			 }
			//printf("%s，%s,  DCB_Month = %10.4lf\n", t.toString().c_str(), PRNObs1Obs2.c_str(), dcbvalue);
			return true;
		}
	}
}