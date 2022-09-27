#include "crdSLRObsFile.hpp"

namespace NUDTTK
{
	namespace SLR
	{
		crdSLRObsFile::crdSLRObsFile(void)
		{
		}

		crdSLRObsFile::~crdSLRObsFile(void)
		{
		}
		UTC crdSinglePassArc::getTime(CRDDataRecord Record)
		{			
			UTC t(crdHeader.StartTime.year, crdHeader.StartTime.month, crdHeader.StartTime.day, 0, 0, 0);			
			t = t + Record.TimeofDay;                 // 再加上秒的部分
			return t;
		}
		// 子程序名称： open   
		// 功能：激光CRD格式数据文件解析 
		// 变量类型：strCRDFileName : CRD文件路径
		// 输入：strCRDFileName
		// 输出：
		// 语言：C++
		// 创建者：刘俊宏
		// 创建时间：2014/11/01
		// 版本时间：2014/11/01
		// 修改记录：
		// 备注：
		bool crdSLRObsFile::open(string strCRDFileName)
		{
			FILE * pCRDFile = fopen(strCRDFileName.c_str(),"r+t");
			if(pCRDFile == NULL)
			{
				printf("npt文件为空！\n");
				return false;
			}
			char szline[200];     
			m_data.clear();
			int bFlag = 1;			
			fgets(szline, 200, pCRDFile);
			while(bFlag)
			{
				if(feof(pCRDFile))
					break;
				else if((szline[0] == 'H' || szline[0] == 'h') && szline[1] == '1')
				{//寻找到新的数据弧段
					crdSinglePassArc newPass;
					// 读取H1					
					sscanf(szline,"%*3c%3c%*1c%2d%*1c%4d%*1c%2d%*1c%2d%*1c%2d",
				                   &newPass.crdHeader.szFormat,
								   &newPass.crdHeader.nVersion,
							       &newPass.crdHeader.ProductTime.year,
								   &newPass.crdHeader.ProductTime.month,
								   &newPass.crdHeader.ProductTime.day,
								   &newPass.crdHeader.ProductTime.hour);
					// 读取H2
					
				//sscanf(strLine.c_str(),"%*2c%4c", szName_4c);

					//fgets(szline, 200, pCRDFile);
					//if((szline[0] == 'H' || szline[0] == 'h') && szline[1] == '2')
					//	sscanf(szline,"%*3c%10c%*1c%4d%*1c%2d%*1c%2d%*1c%2d",
				 //                  &newPass.crdHeader.szStaName,
					//			   &newPass.crdHeader.nCDPPadID,
					//		       &newPass.crdHeader.nCDPSysNum,
					//			   &newPass.crdHeader.nCDPOccSeqNum,
					//			   &newPass.crdHeader.nStaTimeRef);
					//else
					//{
					//	printf("npt文件格式错误！\n");
					//	return false;
					//}
					fgets(szline, 200, pCRDFile);
					char szName_4c[4+1];
					if((szline[0] == 'H' || szline[0] == 'h') && szline[1] == '2')
					{
						sscanf(szline,"%*3c%10c%*1c%4c%*1c%2d%*1c%2d%*1c%2d",
				                   &newPass.crdHeader.szStaName,
								   szName_4c,
							       &newPass.crdHeader.nCDPSysNum,
								   &newPass.crdHeader.nCDPOccSeqNum,
								   &newPass.crdHeader.nStaTimeRef);
						szName_4c[4] = '\0';
						newPass.crdHeader.nCDPPadID = atoi(szName_4c);
						newPass.crdHeader.name_4c   = szName_4c;
					}
					else
					{
						printf("npt文件格式错误！\n");
						return false;
					}
					// 读取H3
					fgets(szline, 200, pCRDFile);
					if((szline[0] == 'H' || szline[0] == 'h') && szline[1] == '3')
						sscanf(szline,"%*3c%10c%*1c%8d%*1c%4d%*1c%8d%*1c%1d%*1c%1d",
				                   &newPass.crdHeader.szTargetName,
								   &newPass.crdHeader.nILRSSatID,
							       &newPass.crdHeader.nSIC,
								   &newPass.crdHeader.nNORADID,
								   &newPass.crdHeader.nTargetTimeRef,
								   &newPass.crdHeader.nTargetType);
					else
					{
						printf("npt文件格式错误！\n");
						return false;
					}
					// 读取H4
					fgets(szline, 200, pCRDFile);
					if((szline[0] == 'H' || szline[0] == 'h') && szline[1] == '4')					
						sscanf(szline,"%*3c%2d%*1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2lf%*1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%2lf%*1c%2d%*1c%1d%*1c%1d%*1c%1d%*1c%1d%*1c%1d%*1c%1d%*1c%1d",
				                   &newPass.crdHeader.nDataType,
								   &newPass.crdHeader.StartTime.year,
							       &newPass.crdHeader.StartTime.month,
								   &newPass.crdHeader.StartTime.day,
								   &newPass.crdHeader.StartTime.hour,
								   &newPass.crdHeader.StartTime.minute,
								   &newPass.crdHeader.StartTime.second,
								   &newPass.crdHeader.EndTime.year,
							       &newPass.crdHeader.EndTime.month,
								   &newPass.crdHeader.EndTime.day,
								   &newPass.crdHeader.EndTime.hour,
								   &newPass.crdHeader.EndTime.minute,
								   &newPass.crdHeader.EndTime.second,
								   &newPass.crdHeader.nReleaseFlag,
								   &newPass.crdHeader.nTroCorr,
								   &newPass.crdHeader.nCenOfMassCorr,
								   &newPass.crdHeader.nRecAmpCorr,
								   &newPass.crdHeader.nStaSysDelayCorr,
								   &newPass.crdHeader.nCraftSysDelayCorr,
								   &newPass.crdHeader.nRangeType,
								   &newPass.crdHeader.nDataQuality);
					else
					{
						printf("npt文件格式错误！\n");
						return false;
					}
					//读取Configuration section
					int bCFlag = 1;
					while(bCFlag)
					{
						fgets(szline, 200, pCRDFile);
						if((szline[0] == 'C' || szline[0] == 'c') && (szline[1] == '0'))
						{
							sscanf(szline,"%*3c%1d%*1c%10lf%*1c%4c%*1c%4c%*1c%4c%*1c%4c%*1c%4c",
				                   &newPass.crdConfig.nC0DetailType,
								   &newPass.crdConfig.Wavelength,
							       &newPass.crdConfig.szSysID,
								   &newPass.crdConfig.szComAID,
								   &newPass.crdConfig.szComBID,
								   &newPass.crdConfig.szComCID,
								   &newPass.crdConfig.szComDID);
						}
						else if((szline[0] == 'C' || szline[0] == 'c') && (szline[1] == '1'))
						{
							sscanf(szline,"%*3c%1d%*1c%4c%*1c%10c%*1c%10lf%*1c%10lf%*1c%10lf%*1c%6lf%*1c%5lf%*1c%4d",
				                   &newPass.crdConfig.nC1DetailType,
								   &newPass.crdConfig.szLaserID,
							       &newPass.crdConfig.szLaserType,
								   &newPass.crdConfig.PriWavelength,
								   &newPass.crdConfig.NormalFireRate,
								   &newPass.crdConfig.PulseEnergy,
								   &newPass.crdConfig.PulseWidth,
								   &newPass.crdConfig.BeamDiv,
								   &newPass.crdConfig.nOSTCount);
						}
						else if((szline[0] == 'C' || szline[0] == 'c') && (szline[1] == '2'))
						{
							sscanf(szline,"%*3c%1d%*1c%4c%*1c%10c%*1c%10lf%*1c%6lf%*1c%5lf%*1c%5lf%*1c%10c%*1c%5lf%*1c%5lf%*1c%5lf%*1c%5lf%*1c%10c",
				                   &newPass.crdConfig.nC2DetailType,
								   &newPass.crdConfig.szDetectID,
							       &newPass.crdConfig.szDetectType,
								   &newPass.crdConfig.AppWavelength,
								   &newPass.crdConfig.QuaEff,
								   &newPass.crdConfig.AppVoltage,
								   &newPass.crdConfig.DarkCount,
								   &newPass.crdConfig.szOutPulType,
								   &newPass.crdConfig.OutPulWidth,
								   &newPass.crdConfig.SpeFilter,
								   &newPass.crdConfig.TraSpeFilter,
								   &newPass.crdConfig.SpaFilter,
								   &newPass.crdConfig.szExtSigPro);
						}
						else if((szline[0] == 'C' || szline[0] == 'c') && (szline[1] == '3'))
						{
							sscanf(szline,"%*3c%1d%*1c%4c%*1c%20c%*1c%20c%*1c%20c%*1c%20c%*1c%6lf",
				                   &newPass.crdConfig.nC3DetailType,
								   &newPass.crdConfig.szTimeSysID,
							       &newPass.crdConfig.szTimeSource,
								   &newPass.crdConfig.szFreSource,
								   &newPass.crdConfig.szTimer,
								   &newPass.crdConfig.szTimerSerNum,
								   &newPass.crdConfig.EpochDelayCorr);
						}
						else if((szline[0] == 'C' || szline[0] == 'c') && (szline[1] == '4'))
						{
							sscanf(szline,"%*3c%1d%*1c%4c%*1c%20lf%*1c%11lf%*1c%20lf%*1c%11lf%*1c%20lf%*1c%1d%*1c%1d%*1c%1d",
				                   &newPass.crdConfig.nC4DetailType,
								   &newPass.crdConfig.StaTimeOffset,
							       &newPass.crdConfig.StaTimeDrift,
								   &newPass.crdConfig.TraTimeOffset,
								   &newPass.crdConfig.TraTimeDrift,
								   &newPass.crdConfig.TraTimeRef,
								   &newPass.crdConfig.nStaTimeApp,
								   &newPass.crdConfig.nCraftTimeApp,
								   &newPass.crdConfig.nCraftTimeSimp);
							bCFlag = 2; // 有C4或c4,读取configuration 结束
							break;
						}
						else
						{
							bCFlag = 3; // 没有C4或c4,读取configuration 结束
							break;
						}
					}
					// 读取Data Section
					if(bCFlag == 2)
						fgets(szline, 200, pCRDFile);
					int bDflag = 1;					
					while(bDflag)
					{
						if(szline[0] == '4' && szline[1] == '0')
						{
							sscanf(szline,"%*3c%18lf%*1c%1d%*1c%4c%*1c%8d%*1c%8d%*1c%7lf%*1c%10lf%*1c%8lf%*1c%6lf%*1c%7lf%*1c%7lf%*1c%6lf%*1c%1d%*1c%1d%*1c%1d",
				                   &newPass.crd405060.SecondofDay,
								   &newPass.crd405060.nDataType,
							       &newPass.crd405060.sz40SysID,
								   &newPass.crd405060.nPointRecord,
								   &newPass.crd405060.nPointUsed,
								   &newPass.crd405060.DistanceOneWay,
								   &newPass.crd405060.SysDelay,
								   &newPass.crd405060.DelayShift,
								   &newPass.crd405060.RawSysDelayRMS,
								   &newPass.crd405060.RawSysDelaySkew,
								   &newPass.crd405060.RawSysDelayKurtosis,
								   &newPass.crd405060.SysDelayPeak_Mean,
								   &newPass.crd405060.nCalibrationType,
								   &newPass.crd405060.nShiftType,
								   &newPass.crd405060.nDetecorChannel);
							fgets(szline, 200, pCRDFile);
						}
						else if(szline[0] == '5' && szline[1] == '0')
						{
							sscanf(szline,"%*3c%*1c%4c%*1c%6lf%*1c%7lf%*1c%7lf%*1c%6lf%*1c%1d",
				                   &newPass.crd405060.sz50SysID,
								   &newPass.crd405060.SessionRMS,
							       &newPass.crd405060.SessionSkew,
								   &newPass.crd405060.SessionKurtosis,
								   &newPass.crd405060.SessionPeak_Mean,
								   &newPass.crd405060.nDataQuality);
							fgets(szline, 200, pCRDFile);
						}
						else if(szline[0] == '6' && szline[1] == '0')
						{
							sscanf(szline,"%*3c%4c%*1c%1d%*1c%1d",
				                   &newPass.crd405060.sz60SysID,
								   &newPass.crd405060.nSCHIndicator,
							       &newPass.crd405060.nSCIIndicator);
							fgets(szline, 200, pCRDFile);
						}
						else if(szline[0] == '2' && szline[1] == '0')
						{//气象数据和标准点数据							
							while(1)
							{//循环读取气象数据和标准点数据
								if(szline[0] == '2' && szline[1] == '0')
								{//第一次读取气象数据，或当气象数据改变时
									CRDDataRecord       nptdata;
									sscanf(szline,"%*3c%18lf%*1c%7lf%*1c%6lf%*1c%4lf%*1c%1d",
										   &nptdata.MRTimeofDay,
										   &nptdata.SurfacePressure,
										   &nptdata.SurfaceTemperature,
										   &nptdata.SurfaceRelHumidity,
										   &nptdata.nOriginofValue);
									do
									{
										fgets(szline, 200, pCRDFile);
									}
									while(!(szline[0] == '1' && szline[1] == '1')); //跳过21,30等字段，直到读取11										
									sscanf(szline,"%*3c%18lf%*1c%18lf%*1c%4c%*1c%1d%*1c%6lf%*1c%6d%*1c%9lf%*1c%7lf%*1c%7lf%*1c%9lf%*1c%5lf%*1c%1d",
										   &nptdata.TimeofDay,
										   &nptdata.TimeofFlight,
										   &nptdata.szSysID,
										   &nptdata.nEpochEvent,
										   &nptdata.NPTWindowLength,
										   &nptdata.nRawRangesNum,
										   &nptdata.BinRMS,
										   &nptdata.BinSkew,
										   &nptdata.BinKurtosis,
										   &nptdata.BinPeak_Mean,
										   &nptdata.ReturnRate,
										   &nptdata.nDetectorChannel);
									newPass.crdDataRecordList.push_back(nptdata);
								}
								else if(szline[0] == '1' && szline[1] == '1')
								{//气象数据没改变，读取NPT数据	
									if(newPass.crdDataRecordList.size() == 0)
									{
										printf("气象数据缺失！\n");
										return false;
									}
									CRDDataRecord       nptdata;
									nptdata.MRTimeofDay = newPass.crdDataRecordList.back().MRTimeofDay;
									nptdata.SurfacePressure = newPass.crdDataRecordList.back().SurfacePressure;
									nptdata.SurfaceTemperature = newPass.crdDataRecordList.back().SurfaceTemperature;
									nptdata.SurfaceRelHumidity = newPass.crdDataRecordList.back().SurfaceRelHumidity;
									nptdata.nOriginofValue = newPass.crdDataRecordList.back().nOriginofValue;
									sscanf(szline,"%*3c%18lf%*1c%18lf%*1c%4c%*1c%1d%*1c%6lf%*1c%6d%*1c%9lf%*1c%7lf%*1c%7lf%*1c%9lf%*1c%5lf%*1c%1d",
										   &nptdata.TimeofDay,
										   &nptdata.TimeofFlight,
										   &nptdata.szSysID,
										   &nptdata.nEpochEvent,
										   &nptdata.NPTWindowLength,
										   &nptdata.nRawRangesNum,
										   &nptdata.BinRMS,
										   &nptdata.BinSkew,
										   &nptdata.BinKurtosis,
										   &nptdata.BinPeak_Mean,
										   &nptdata.ReturnRate,
										   &nptdata.nDetectorChannel);
									newPass.crdDataRecordList.push_back(nptdata);
								}
								else if(szline[0] != '1' && szline[0] != '2')//标准点数据读取完毕
								{									
									break;
								}
								fgets(szline, 200, pCRDFile);
							}
						}
						else if((szline[0] == 'H' || szline[0] == 'h')&& szline[1] == '8')
						{
							bDflag = 2;
							break;
						}
						else
						{//暂不作处理
							fgets(szline, 200, pCRDFile);
						}						
					}
					// 重新整理观测时间，避免出现午夜间断的情况
					for(size_t s_k = 1; s_k < newPass.crdDataRecordList.size(); s_k++)
					{
						if(newPass.crdDataRecordList[s_k].TimeofDay
						 - newPass.crdDataRecordList[s_k-1].TimeofDay < 0)
						{// 对 s_k 处进行连续化, 一个弧段仅可能出现一次该情况
							for(size_t s_j = s_k; s_j < newPass.crdDataRecordList.size(); s_j++)
								newPass.crdDataRecordList[s_j].TimeofDay += 86400;
							break;
						}						
					}
					m_data.push_back(newPass);
					fgets(szline, 200, pCRDFile);//读取下一行

				}
				else if((szline[0] == 'H' || szline[0] == 'h') && szline[1] == '9')//文件结尾
				{
					bFlag = false;
					break;
				}
				else if(szline[0] == '0' && szline[1] == '0') // 2020.03.03, 谷德峰修改，注释行
				{
					fgets(szline, 200, pCRDFile);//读取下一行
					continue;
				}
			}
			fclose(pCRDFile);
			return true;
		}
		// 子程序名称： write   
		// 功能：激光CRD格式数据写到文件 
		bool crdSLRObsFile::write(string strCRDFileName)
		{
			if(m_data.size() <= 0)
			return false;
			FILE * pCRDFile = fopen(strCRDFileName.c_str(),"w+");
			for(size_t s_i = 0;s_i < m_data.size();s_i++)
			{   
			  crdSinglePassArc newPass;
			  newPass = m_data[s_i];
			  //H1
			  fprintf(pCRDFile,"H1 %3s %2d %4d %2d %2d %2d\n",
				                   newPass.crdHeader.szFormat,
								   newPass.crdHeader.nVersion,
							       newPass.crdHeader.ProductTime.year,
								   newPass.crdHeader.ProductTime.month,
								   newPass.crdHeader.ProductTime.day,
								   newPass.crdHeader.ProductTime.hour);
			  //H2
			  fprintf(pCRDFile,"H2 %10s %4d %2d %2d %2d\n",
				                   newPass.crdHeader.szStaName,
								   newPass.crdHeader.nCDPPadID,
							       newPass.crdHeader.nCDPSysNum,
								   newPass.crdHeader.nCDPOccSeqNum,
								   newPass.crdHeader.nStaTimeRef);
			  //H3
			  fprintf(pCRDFile,"H3 %10s %7d %4d %5d %1d %1d\n",
				                   newPass.crdHeader.szTargetName,
								   newPass.crdHeader.nILRSSatID,
							       newPass.crdHeader.nSIC,
								   newPass.crdHeader.nNORADID,
								   newPass.crdHeader.nTargetTimeRef,
								   newPass.crdHeader.nTargetType);
			  //H4
			  fprintf(pCRDFile,"H4 %2d %4d %2d %2d %2d %2d %2d %4d %2d %2d %2d %2d %2d %2d %1d %1d %1d %1d %1d %1d %1d\n",
				                   newPass.crdHeader.nDataType,
								   newPass.crdHeader.StartTime.year,
							       newPass.crdHeader.StartTime.month,
								   newPass.crdHeader.StartTime.day,
								   newPass.crdHeader.StartTime.hour,
								   newPass.crdHeader.StartTime.minute,
								   newPass.crdHeader.StartTime.second,
								   newPass.crdHeader.EndTime.year,
							       newPass.crdHeader.EndTime.month,
								   newPass.crdHeader.EndTime.day,
								   newPass.crdHeader.EndTime.hour,
								   newPass.crdHeader.EndTime.minute,
								   newPass.crdHeader.EndTime.second,
								   newPass.crdHeader.nReleaseFlag,
								   newPass.crdHeader.nTroCorr,
								   newPass.crdHeader.nCenOfMassCorr,
								   newPass.crdHeader.nRecAmpCorr,
								   newPass.crdHeader.nStaSysDelayCorr,
								   newPass.crdHeader.nCraftSysDelayCorr,
								   newPass.crdHeader.nRangeType,
								   newPass.crdHeader.nDataQuality);
			  //c0
			  fprintf(pCRDFile,"C0 %1d %10.3f %4s %4s %4s %4s %4s\n",
				                   newPass.crdConfig.nC0DetailType,
								   newPass.crdConfig.Wavelength,
							       newPass.crdConfig.szSysID,
								   newPass.crdConfig.szComAID,
								   newPass.crdConfig.szComBID,
								   newPass.crdConfig.szComCID,
								   newPass.crdConfig.szComDID);
			  //c1
			  fprintf(pCRDFile,"C1 %1d %4s %10s %10.2f %10.2f %10.2f %6.1f %5.2f %4d\n",
				                   newPass.crdConfig.nC1DetailType,
								   newPass.crdConfig.szLaserID,
							       newPass.crdConfig.szLaserType,
								   newPass.crdConfig.PriWavelength,
								   newPass.crdConfig.NormalFireRate,
								   newPass.crdConfig.PulseEnergy,
								   newPass.crdConfig.PulseWidth,
								   newPass.crdConfig.BeamDiv,
								   newPass.crdConfig.nOSTCount);
			  //c2
			  fprintf(pCRDFile,"C2 %1d %4s %10s %10.3f %6.2f %5.1f %5.1f %10s %5.1f %5.2f %5.1f %5.1f %10s\n",
				                   newPass.crdConfig.nC2DetailType,
								   newPass.crdConfig.szDetectID,
							       newPass.crdConfig.szDetectType,
								   newPass.crdConfig.AppWavelength,
								   newPass.crdConfig.QuaEff,
								   newPass.crdConfig.AppVoltage,
								   newPass.crdConfig.DarkCount,
								   newPass.crdConfig.szOutPulType,
								   newPass.crdConfig.OutPulWidth,
								   newPass.crdConfig.SpeFilter,
								   newPass.crdConfig.TraSpeFilter,
								   newPass.crdConfig.SpaFilter,
								   newPass.crdConfig.szExtSigPro);
			  //c3
			  fprintf(pCRDFile,"C3 %1d %4s %20s %20s %20s %20s %6.1f\n",
				                   newPass.crdConfig.nC3DetailType,
								   newPass.crdConfig.szTimeSysID,
							       newPass.crdConfig.szTimeSource,
								   newPass.crdConfig.szFreSource,
								   newPass.crdConfig.szTimer,
								   newPass.crdConfig.szTimerSerNum,
								   newPass.crdConfig.EpochDelayCorr);

			  //c4
			  /*fprintf(pCRDFile,"C4 %1d %4s %20.3f %11.2f %20.3f %11.2f %20.12f %1d %1d %1d\n",
				                   newPass.crdConfig.nC4DetailType,
								   newPass.crdConfig.StaTimeOffset,
							       newPass.crdConfig.StaTimeDrift,
								   newPass.crdConfig.TraTimeOffset,
								   newPass.crdConfig.TraTimeDrift,
								   newPass.crdConfig.TraTimeRef,
								   newPass.crdConfig.nStaTimeApp,
								   newPass.crdConfig.nCraftTimeApp,
								   newPass.crdConfig.nCraftTimeSimp);*/
			  //60
			  fprintf(pCRDFile,"60 %4s %1d %1d\n",
			  	                   newPass.crd405060.sz60SysID,
			  					   newPass.crd405060.nSCHIndicator,
			  				       newPass.crd405060.nSCIIndicator);

			  //40
			  fprintf(pCRDFile,"40 %18.12f %1d %4s %8d %8d %7.3f %10.1f %8.1f %6.1f %7.3f %7.3f %6.1f %1d %1d %1d\n",
				                   newPass.crd405060.SecondofDay,
								   newPass.crd405060.nDataType,
							       newPass.crd405060.sz40SysID,
								   newPass.crd405060.nPointRecord,
								   newPass.crd405060.nPointUsed,
								   newPass.crd405060.DistanceOneWay,
								   newPass.crd405060.SysDelay,
								   newPass.crd405060.DelayShift,
								   newPass.crd405060.RawSysDelayRMS,
								   newPass.crd405060.RawSysDelaySkew,
								   newPass.crd405060.RawSysDelayKurtosis,
								   newPass.crd405060.SysDelayPeak_Mean,
								   newPass.crd405060.nCalibrationType,
								   newPass.crd405060.nShiftType,
								   newPass.crd405060.nDetecorChannel);

			  //data section
			  CRDDataRecord nptdata;
		      nptdata.MRTimeofDay = 0.0;
		      nptdata.SurfacePressure = 0.0;
		      nptdata.SurfaceTemperature = 0.0;
		      nptdata.SurfaceRelHumidity = 0.0;
		      nptdata.nOriginofValue = 0;
			  for(size_t s_j = 0 ;s_j < newPass.crdDataRecordList.size();s_j++)
			  {     
			        //20
				    // 如果s_j == 0，写一行20；如果s_j != 0， 判断头是否相等，如果相等，直接写11，否则写先写完头，再写11
		           if (s_j == 0 || (s_j != 0 && (nptdata.MRTimeofDay != newPass.crdDataRecordList[s_j].MRTimeofDay 
					    || nptdata.SurfacePressure != newPass.crdDataRecordList[s_j].SurfacePressure
					    || nptdata.SurfaceTemperature != newPass.crdDataRecordList[s_j].SurfaceTemperature
					    || nptdata.SurfaceRelHumidity != newPass.crdDataRecordList[s_j].SurfaceRelHumidity
					    || nptdata.nOriginofValue != newPass.crdDataRecordList[s_j].nOriginofValue))) 
				   {
					   fprintf(pCRDFile,"20 %18.12f %7.2f %6.2f %4.0f %1d\n", 
							   newPass.crdDataRecordList[s_j].MRTimeofDay,
							   newPass.crdDataRecordList[s_j].SurfacePressure,
							   newPass.crdDataRecordList[s_j].SurfaceTemperature,
							   newPass.crdDataRecordList[s_j].SurfaceRelHumidity,
							   newPass.crdDataRecordList[s_j].nOriginofValue);
					   nptdata.MRTimeofDay = newPass.crdDataRecordList[s_j].MRTimeofDay;
					   nptdata.SurfacePressure = newPass.crdDataRecordList[s_j].SurfacePressure;
					   nptdata.SurfaceTemperature = newPass.crdDataRecordList[s_j].SurfaceTemperature;
					   nptdata.SurfaceRelHumidity = newPass.crdDataRecordList[s_j].SurfaceRelHumidity;
					   nptdata.nOriginofValue = newPass.crdDataRecordList[s_j].nOriginofValue;
				   }
							fprintf(pCRDFile,"11 %18.12f %18.12f %4s %1d %6.1f %6d %9.1f %7.3f %7.3f %9.1f %5.1f %1d\n",
							   newPass.crdDataRecordList[s_j].TimeofDay,
							   newPass.crdDataRecordList[s_j].TimeofFlight,
							   newPass.crdDataRecordList[s_j].szSysID,
							   newPass.crdDataRecordList[s_j].nEpochEvent,
							   newPass.crdDataRecordList[s_j].NPTWindowLength,
							   newPass.crdDataRecordList[s_j].nRawRangesNum,
							   newPass.crdDataRecordList[s_j].BinRMS,
							   newPass.crdDataRecordList[s_j].BinSkew,
							   newPass.crdDataRecordList[s_j].BinKurtosis,
							   newPass.crdDataRecordList[s_j].BinPeak_Mean,
							   newPass.crdDataRecordList[s_j].ReturnRate,
							   newPass.crdDataRecordList[s_j].nDetectorChannel);//标准格式fprintf(pCRDFile,"11 %18.12f %18.12f

			  }			                       
			   			  
			  //50
			  fprintf(pCRDFile,"50 %4s %6.1f %7.3f %7.3f %6.1f %1d\n",
				                   newPass.crd405060.sz50SysID,
								   newPass.crd405060.SessionRMS,
							       newPass.crd405060.SessionSkew,
								   newPass.crd405060.SessionKurtosis,
								   newPass.crd405060.SessionPeak_Mean,
								   newPass.crd405060.nDataQuality);
			  ////60
			  //fprintf(pCRDFile,"60 %4s %1d %1d\n",
			  //	                   newPass.crd405060.sz60SysID,
			  //					   newPass.crd405060.nSCHIndicator,
			  //				       newPass.crd405060.nSCIIndicator);

			  //H8
			  fprintf(pCRDFile,"H8\n");
			}
			//H9
			fprintf(pCRDFile,"H9\n");

            fclose(pCRDFile);
			return true;
		}
	}
}