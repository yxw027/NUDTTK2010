#pragma once
#include "BDObsQualityEvaluate.hpp"
#include "MathAlgorithm.hpp"
#include <direct.h>

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace BDPreproc
	{	
		BDObsQualityEvaluate::BDObsQualityEvaluate(void)
		{
			//m_cut_max_pdop = 6.0;
		}

		BDObsQualityEvaluate::~BDObsQualityEvaluate(void)
		{
		}		
		// 子程序名称： init   
		// 功能：初始化观测质量与仰角关系的列表
		// 变量类型： elevation         :  间隔仰角
		// 输入：elevation
		// 输出：
		// 语言：C++
		// 创建者：刘俊宏
		// 创建时间：2013/09/26
		// 版本时间：
		// 修改记录：
		// 备注： 
		void QERESIDUAL_STATION::init(double elevation_Inter)
		{
			double max_ele = 90.0; // 仰角的变化范围为[0,90]
			double fraction_ele = fmod(max_ele, elevation_Inter);
			int count_Inter;       // 总的区间个数
			if(fraction_ele >= 0.5 * elevation_Inter)
				count_Inter = int (ceil(max_ele / elevation_Inter));
			else 
				count_Inter = int (floor(max_ele / elevation_Inter));
			if(count_Inter < 1)
				count_Inter = 1;
			satInfolist_ele.resize(count_Inter);
			for(int i = 0 ; i < count_Inter; i++)
			{
				satInfolist_ele[i].e0 = i * elevation_Inter;	
				satInfolist_ele[i].e1 = (i + 1) * elevation_Inter;				
			}			
			satInfolist_ele[count_Inter - 1].e1 = 90.0;
		}
		// 子程序名称： getInterval   
		// 功能：确定观测仰角在仰角列表中的位置
		// 变量类型： elevation  :  观测仰角
		// 输入：     elevation
		// 输出：
		// 语言：C++
		// 创建者：刘俊宏
		// 创建时间：2013/09/26
		// 版本时间：
		// 修改记录：
		// 备注： 
		int QERESIDUAL_STATION::getInterval(double elevation)
		{
			int interval = 0;
			for(size_t s_i = 0; s_i <  satInfolist_ele.size();s_i++)
			{
				if(elevation - satInfolist_ele[s_i].e0 >= 0 && elevation - satInfolist_ele[s_i].e1 < 0)
				{								
					interval = int(s_i);
					break;
				}
			}						
			return interval;
		}
		//   子程序名称： mainFunc   
		//   作用：双频观测数据质量评估
		//   类型：strEdtedObsfilePath: 预处理后的观测数据路径		

		//   输入：strEdtedObsfilePath
		//   输出：
		//   其它：
		//   语言： C++
		//   版本号：2012/9/29
		//   生成者：刘俊宏，谷德峰
		//   修改记录：
		//   备注：
		bool BDObsQualityEvaluate::mainFunc(string  strEdtedObsfilePath)
		{
			// 分析 strEdtedObsfilePath 路径, 提取根目录和文件名
			string edtedFileName = strEdtedObsfilePath.substr(strEdtedObsfilePath.find_last_of("\\") + 1);
			string folder = strEdtedObsfilePath.substr(0, strEdtedObsfilePath.find_last_of("\\"));
			string edtedFileName_noexp = edtedFileName.substr(0, edtedFileName.find_last_of("."));
			// 创建质量评估目录
			string strOQEFolder = folder + "\\OQE";
			_mkdir(strOQEFolder.c_str());
			Rinex2_1_EditedObsFile m_editedObsFile;
			if(!m_editedObsFile.open(strEdtedObsfilePath))
			{
				printf("%s 文件无法打开!\n", strEdtedObsfilePath.c_str());
				return false;
			}			
			// 寻找观测类型观测序列中的序号
			int nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_L1 = -1, nObsTypes_L2 = -1;
			for(int i = 0; i < m_editedObsFile.m_header.byObsTypes; i++)
			{				
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1 = i;
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2 = i;
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1 = i;
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2 = i;
			}
			if(nObsTypes_P1 == -1 || nObsTypes_P2 == -1 || nObsTypes_L1 == -1 || nObsTypes_L2 == -1) 
			{
				printf("观测数据不完整！");
				return false;
			}
			////统计同一时刻所有卫星都发生周跳的情况
			//FILE *pfile_s = fopen("C:\\cycleslip_time.cpp","a+");
			//for(size_t s_i = 0; s_i < m_editedObsFile.m_data.size();s_i++)
			//{
			//	int k = 0;
			//	int j = 0;				
			//	for(Rinex2_1_EditedObsSatMap::iterator it = m_editedObsFile.m_data[s_i].editedObs.begin();it != m_editedObsFile.m_data[s_i].editedObs.end();++it)
			//	{
			//		if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_NORMAL)
			//			k++;
			//		if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
			//			j++;
			//		if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 == 3 &&it->second.obsTypeList[nObsTypes_L1].byEditedMark2 == 0)
			//			j++;
			//	}
			//	if(k == m_editedObsFile.m_data[s_i].editedObs.size()&& k != j)
			//		fprintf(pfile_s,"%s  %s\n",strEdtedObsfilePath.c_str(),m_editedObsFile.m_data[s_i].t.toString().c_str());
			//}
			//fclose(pfile_s);

			vector<Rinex2_1_EditedObsSat> editedObsSatlist;			
			double rms_P1 = 0;
			double rms_P2 = 0;
			double rms_L = 0;
			if(!m_editedObsFile.getEditedObsSatList(editedObsSatlist))
			{
				printf("获取预处理后的观测数据失败！");
				return false;
			}
			//// 仅统计IGSO卫星的观测数据质量
			//vector<Rinex2_1_EditedObsSat> editedObsSatlistIGSO;
			//for(size_t s_i = 0; s_i < editedObsSatlist.size();s_i++)
			//	if(editedObsSatlist[s_i].Id > 5)
			//		editedObsSatlistIGSO.push_back(editedObsSatlist[s_i]);
			//editedObsSatlist.clear();
			//editedObsSatlist = editedObsSatlistIGSO;
			//FILE *pfile = fopen("C:\\residuls_L_fit.cpp","w+");
			//fclose(pfile);
			m_QEInfo.init();        // 初始化观测仰角列表，2013/9/26
			if(!evaluate_code_multipath(editedObsSatlist, nObsTypes_P1, nObsTypes_P2, nObsTypes_L1, nObsTypes_L2))				
			{
				printf("伪码数据质量评估失败！");
				return false;
			}
			else if(!evaluate_phase_vondrak(editedObsSatlist, nObsTypes_L1, nObsTypes_L2))
			{
				printf("相位数据质量评估失败！");
				return false;
			}
			else
			{
				//统计平均可视卫星数和数据总数			
				m_QEInfo.total_Epochs = 0;
				for(size_t s_i = 0; s_i < m_editedObsFile.m_data.size(); s_i++)				
					m_QEInfo.total_Epochs = m_QEInfo.total_Epochs + (int)m_editedObsFile.m_data[s_i].editedObs.size();
				m_QEInfo.mean_VisibleSat = m_QEInfo.total_Epochs/double(m_editedObsFile.m_data.size());
				char infoFilePath[100];
			    sprintf(infoFilePath,"%s\\%s.OQE", strOQEFolder.c_str(), edtedFileName_noexp.c_str());
			    FILE * pInfoFile = fopen(infoFilePath, "w+");
				string StationName = m_editedObsFile.m_header.szMarkName;
				StationName.erase(4, 56);
				fprintf(pInfoFile," 测站%s观测数据质量评估\n",StationName.c_str());
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile," MP1              (m)                   %12.4lf\n",m_QEInfo.rms_P1);
				fprintf(pInfoFile," MP2              (m)                   %12.4lf\n",m_QEInfo.rms_P2);
				fprintf(pInfoFile," MP5              (m)                   %12.4lf\n",m_QEInfo.rms_P5);
				fprintf(pInfoFile," ML               (cm)                  %12.4lf\n",m_QEInfo.rms_L*100);
				fprintf(pInfoFile," 观测历元总数     (个)                  %12d\n"   ,m_QEInfo.total_Epochs);
				fprintf(pInfoFile," 平均可视卫星颗数 (颗)                  %12.2lf\n",m_QEInfo.mean_VisibleSat);
				fprintf(pInfoFile," 伪码正常数据比率                       %12.4lf\n",m_QEInfo.ratio_P_normal);
				fprintf(pInfoFile," 相位正常数据比率                       %12.4lf\n",m_QEInfo.ratio_L_normal);
				fprintf(pInfoFile," 周跳比率                               %12.4lf\n",m_QEInfo.ratio_SLip);
				fprintf(pInfoFile, "======================================================\n");				
				fprintf(pInfoFile," 野值信息统计\n");
				fprintf(pInfoFile,"   标记为野值的原因                             个数\n");
				for(QEAbNormalObsCountMap::iterator it = m_QEInfo.AbnormalObsCount.begin();it != m_QEInfo.AbnormalObsCount.end();it++)
				{
					if(it->first == 20)
						fprintf(pInfoFile,"     无观测数据                           %10d\n",it->second);
					if(it->first == 21)
						fprintf(pInfoFile,"     观测数据为零                         %10d\n",it->second);
					if(it->first == 22)
						fprintf(pInfoFile,"     观测弧段过短                         %10d\n",it->second);
					if(it->first == 23)
						fprintf(pInfoFile,"     性噪比过低                           %10d\n",it->second);
					if(it->first == 24)
						fprintf(pInfoFile,"     观测仰角过低                         %10d\n",it->second);
					if(it->first == 25)
						fprintf(pInfoFile,"     电离层超差                           %10d\n",it->second);
					if(it->first == 26)
						fprintf(pInfoFile,"     三频伪距相位GIF组合                  %10d\n",it->second);
					if(it->first == 27)
						fprintf(pInfoFile,"     Vondrak滤波拟合超差                  %10d\n",it->second);
					if(it->first == 28)
						fprintf(pInfoFile,"     三频相位无几何距离组合超差           %10d\n",it->second);					
				}
				fprintf(pInfoFile," 周跳信息统计\n");
				fprintf(pInfoFile,"   标记为周跳的原因                              个数\n");
				for(QEAbNormalObsCountMap::iterator it = m_QEInfo.AbnormalObsCount.begin();it != m_QEInfo.AbnormalObsCount.end();it++)
				{					
					if(it->first == 30)
						fprintf(pInfoFile,"     新弧段起始点                         %10d\n",it->second);
					if(it->first == 31)
						fprintf(pInfoFile,"     MW组合                               %10d\n",it->second);
					if(it->first == 32)
						fprintf(pInfoFile,"     消电离层组合                         %10d\n",it->second);
					if(it->first == 33)
						fprintf(pInfoFile,"     L1-L2组合                            %10d\n",it->second);					
				}
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile, " 伪距观测数据质量与仰角的关系（不包括GEO卫星）\n");
				fprintf(pInfoFile, "  Ele(deg)     Epochs     MP1(m)     MP2(m)    MP5(m)\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfolist_ele.size(); s_i ++)
				{
					fprintf(pInfoFile," %4.1f-%4.1f     %6d%10.3lf %10.3lf%10.3lf\n",
						               m_QEInfo.satInfolist_ele[s_i].e0,
									   m_QEInfo.satInfolist_ele[s_i].e1,
									   m_QEInfo.satInfolist_ele[s_i].epochs_P,
									   m_QEInfo.satInfolist_ele[s_i].rms_P1,
									   m_QEInfo.satInfolist_ele[s_i].rms_P2,
									   m_QEInfo.satInfolist_ele[s_i].rms_P5);
				}
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile, " 相位观测数据质量与仰角的关系（不包括GEO卫星）\n");
				fprintf(pInfoFile, "  Ele(deg)     Epochs     ML(cm)     Slips\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfolist_ele.size(); s_i ++)
				{
					fprintf(pInfoFile," %4.1f-%4.1f     %6d%10.3lf     %6d\n",
						               m_QEInfo.satInfolist_ele[s_i].e0,
									   m_QEInfo.satInfolist_ele[s_i].e1,
									   m_QEInfo.satInfolist_ele[s_i].epochs_L,
									   m_QEInfo.satInfolist_ele[s_i].rms_L * 100,
									   m_QEInfo.satInfolist_ele[s_i].slips);
				}
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile, " 信噪比与仰角的关系（不包括GEO卫星）\n");
				fprintf(pInfoFile, "  Ele(deg)     SN1(dB)   SN2(dB)    SN3(dB)\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfolist_ele.size(); s_i ++)
				{
					fprintf(pInfoFile," %4.1f-%4.1f %10.3lf%10.3lf %10.3lf\n",
						               m_QEInfo.satInfolist_ele[s_i].e0,
									   m_QEInfo.satInfolist_ele[s_i].e1,
									   m_QEInfo.satInfolist_ele[s_i].CN_L1,
									   m_QEInfo.satInfolist_ele[s_i].CN_L2,
									   m_QEInfo.satInfolist_ele[s_i].CN_L5);
				}
				fprintf(pInfoFile, "======================================================\n");
				if(m_QEInfo.satInforlist_CycleSlip.size() > 0)
				{
					fprintf(pInfoFile, " 周跳信息\n");
					fprintf(pInfoFile, " PRN                Time                  Method\n");
					for(size_t s_i = 0; s_i < m_QEInfo.satInforlist_CycleSlip.size(); s_i ++)
					{
						if(m_QEInfo.satInforlist_CycleSlip[s_i].preproc_info == 31)
							fprintf(pInfoFile,"  %2d   %s  M-W组合\n",
										   m_QEInfo.satInforlist_CycleSlip[s_i].id_sat,
										   m_QEInfo.satInforlist_CycleSlip[s_i].t.toString().c_str());
						if(m_QEInfo.satInforlist_CycleSlip[s_i].preproc_info == 32)
							fprintf(pInfoFile,"  %2d   %s  消电离层组合\n",
										   m_QEInfo.satInforlist_CycleSlip[s_i].id_sat,
										   m_QEInfo.satInforlist_CycleSlip[s_i].t.toString().c_str());
						if(m_QEInfo.satInforlist_CycleSlip[s_i].preproc_info == 33)
							fprintf(pInfoFile,"  %2d   %s  L1-L2组合\n",
										   m_QEInfo.satInforlist_CycleSlip[s_i].id_sat,
										   m_QEInfo.satInforlist_CycleSlip[s_i].t.toString().c_str());						
					}
					fprintf(pInfoFile, "======================================================\n");
				}
				fprintf(pInfoFile," 每颗卫星的观测数据质量\n");				
				fprintf(pInfoFile," PRN Epochs Slips   MP1(m)  MP2(m)  MP5(m)  ML(cm)\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P1.size(); s_i++)
				{
					fprintf(pInfoFile,"  %2d  %5d   %3d%8.3lf%8.3lf%8.3lf%8.3lf\n",
						               m_QEInfo.satInfoList_P1[s_i].id_sat,									   
									   m_QEInfo.satInfoList_P1[s_i].epochs,
									   m_QEInfo.satInfoList_P1[s_i].slips,
						               m_QEInfo.satInfoList_P1[s_i].rms_sat,
									   m_QEInfo.satInfoList_P2[s_i].rms_sat,
									   m_QEInfo.rms_P5,
									   m_QEInfo.satInfoList_L[s_i].rms_sat * 100);

				}				
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile," 每个弧段的观测数据质量\n");
				fprintf(pInfoFile," PRN  Arc_N         MP1(m)  MP2(m)  MP5(m)  ML(cm)\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P1.size(); s_i++)
				{					
					for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_P1[s_i].arcList.size(); s_j++)
						fprintf(pInfoFile,"  %2d    %3d      %8.3lf%8.3lf%8.3lf%8.3lf\n",
						                   m_QEInfo.satInfoList_P1[s_i].id_sat,
						                   s_j + 1,
										   m_QEInfo.satInfoList_P1[s_i].arcList[s_j].rms_arc,
										   m_QEInfo.satInfoList_P2[s_i].arcList[s_j].rms_arc,
										   m_QEInfo.rms_P5,
										   m_QEInfo.satInfoList_L[s_i].arcList[s_j].rms_arc * 100);										
				}
				fclose(pInfoFile);
				////统计平均可视卫星数和数据总数
				//size_t ncount = 0;
				//double mean_sat = 0;
				//for(size_t s_i = 0; s_i < m_editedObsFile.m_data.size(); s_i++)				
				//	ncount = ncount + m_editedObsFile.m_data[s_i].editedObs.size();
				//mean_sat = ncount/double(m_editedObsFile.m_data.size());
				//char infoFilePath[100];
			 //   sprintf(infoFilePath,"%s\\%s.OQE", folder.c_str(), edtedFileName_noexp.c_str());
			 //   FILE * pInfoFile = fopen(infoFilePath, "w+");
				//fprintf(pInfoFile," 测站%s观测数据质量评估\n",bdStationId2String(string2BDStationId(edtedFileName_noexp)).c_str());
				//fprintf(pInfoFile, "======================================================\n");
				//fprintf(pInfoFile," MP1              (m)                   %12.4lf\n",m_QEInfo.rms_P1);
				//fprintf(pInfoFile," MP2              (m)                   %12.4lf\n",m_QEInfo.rms_P2);
				//fprintf(pInfoFile," L1_L2_RMS        (cm)                  %12.4lf\n",m_QEInfo.rms_L*100);
				//fprintf(pInfoFile," 观测历元总数     (个)                  %12d\n",ncount);
				//fprintf(pInfoFile," 平均可视卫星颗数 (颗)                  %12.2lf\n",mean_sat);
				//fprintf(pInfoFile," 伪码正常数据比率                       %12.4lf\n",m_QEInfo.ratio_P_normal);
				//fprintf(pInfoFile," 相位正常数据比率                       %12.4lf\n",m_QEInfo.ratio_L_normal);
				//fprintf(pInfoFile," 周跳比率                               %12.4lf\n",m_QEInfo.ratio_SLip);
				//fprintf(pInfoFile, "======================================================\n");				
				//fprintf(pInfoFile," 野值信息统计\n");
				//fprintf(pInfoFile,"   标记为野值的原因                             个数\n");
				//for(QEAbNormalObsCountMap::iterator it = m_QEInfo.AbnormalObsCount.begin();it != m_QEInfo.AbnormalObsCount.end();it++)
				//{
				//	if(it->first == 20)
				//		fprintf(pInfoFile,"     无观测数据                           %10d\n",it->second);
				//	if(it->first == 21)
				//		fprintf(pInfoFile,"     观测数据为零                         %10d\n",it->second);
				//	if(it->first == 22)
				//		fprintf(pInfoFile,"     观测弧段过短                         %10d\n",it->second);
				//	if(it->first == 23)
				//		fprintf(pInfoFile,"     性噪比过低                           %10d\n",it->second);
				//	if(it->first == 24)
				//		fprintf(pInfoFile,"     观测仰角过低                         %10d\n",it->second);
				//	if(it->first == 25)
				//		fprintf(pInfoFile,"     电离层超差                           %10d\n",it->second);
				//	if(it->first == 26)
				//		fprintf(pInfoFile,"     MW组合超差                           %10d\n",it->second);
				//	if(it->first == 27)
				//		fprintf(pInfoFile,"     Vondrak滤波拟合超差                  %10d\n",it->second);
				//	if(it->first == 28)
				//		fprintf(pInfoFile,"     L1-L2组合超差                        %10d\n",it->second);					
				//}
				//fprintf(pInfoFile," 周跳信息统计\n");
				//fprintf(pInfoFile,"   标记为周跳的原因                             个数\n");
				//for(QEAbNormalObsCountMap::iterator it = m_QEInfo.AbnormalObsCount.begin();it != m_QEInfo.AbnormalObsCount.end();it++)
				//{					
				//	if(it->first == 30)
				//		fprintf(pInfoFile,"     新弧段起始点                         %10d\n",it->second);
				//	if(it->first == 31)
				//		fprintf(pInfoFile,"     MW组合                               %10d\n",it->second);
				//	if(it->first == 32)
				//		fprintf(pInfoFile,"     消电离层组合                         %10d\n",it->second);
				//	if(it->first == 33)
				//		fprintf(pInfoFile,"     L1-L2组合                            %10d\n",it->second);
				//}
				//fprintf(pInfoFile, "======================================================\n");
				//fprintf(pInfoFile," 每颗卫星的观测数据质量\n");				
				//fprintf(pInfoFile," PRN          P1_RMS (m)  P2_RMS (m)  L1_L2_RMS (cm)\n");
				//for(size_t s_i=0; s_i < m_QEInfo.satInfoList_P1.size(); s_i++)
				//{
				//	fprintf(pInfoFile,"  %2d         %12.4lf%12.4lf   %12.4lf\n",
				//		               m_QEInfo.satInfoList_P1[s_i].id_sat,
				//		               m_QEInfo.satInfoList_P1[s_i].rms_sat,
				//					   m_QEInfo.satInfoList_P2[s_i].rms_sat,
				//					   m_QEInfo.satInfoList_L[s_i].rms_sat * 100);

				//}				
				//fprintf(pInfoFile, "======================================================\n");
				//fprintf(pInfoFile," 每个弧段的观测数据质量\n");
				//fprintf(pInfoFile," PRN  Arc_N   P1_RMS (m)  P2_RMS (m)  L1_L2_RMS (cm)\n");
				//for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P1.size(); s_i++)
				//{					
				//	for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_P1[s_i].arcList.size(); s_j++)
				//		fprintf(pInfoFile,"  %2d       %2d%12.4lf%12.4lf   %12.4lf\n",
				//		                   m_QEInfo.satInfoList_P1[s_i].id_sat,
				//		                   s_j + 1,
				//						   m_QEInfo.satInfoList_P1[s_i].arcList[s_j].rms_arc,
				//						   m_QEInfo.satInfoList_P2[s_i].arcList[s_j].rms_arc,
				//						   m_QEInfo.satInfoList_L[s_i].arcList[s_j].rms_arc * 100);										
				//}
				//fclose(pInfoFile);				
				//char P1ResFilePath[100];
				//char P2ResFilePath[100];
				//char LResFilePath[100];
			 //   sprintf(P1ResFilePath,"%s\\%s.RP1", folder.c_str(), edtedFileName_noexp.c_str());
				//sprintf(P2ResFilePath,"%s\\%s.RP2", folder.c_str(), edtedFileName_noexp.c_str());
				//sprintf(LResFilePath,"%s\\%s.RL", folder.c_str(), edtedFileName_noexp.c_str());
				//FILE * pP1File = fopen(P1ResFilePath, "w+");
				//FILE * pP2File = fopen(P2ResFilePath, "w+");
				//FILE * pLFile  = fopen(LResFilePath, "w+");
				//for(size_t s_i = 0;s_i < m_QEInfo.satInfoList_P1.size();s_i++)
				//{
				//	for(size_t s_j = 0;s_j < m_QEInfo.satInfoList_P1[s_i].arcList.size();s_j++)
				//	{
				//		for(size_t s_k = 0;s_k < m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList.size();s_k++)
				//		{
				//			fprintf(pP1File,"%2d  %2d  %10.1f  %14.2f  %14.2f  %14.4f\n",
				//				m_QEInfo.satInfoList_P1[s_i].id_sat,
				//				s_j + 1,
				//				m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].t - m_QEInfo.satInfoList_P1[s_i].arcList[0].resList[0].t,								
				//				m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].Elevation,
				//				m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].Azimuth,
				//				m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].res);
				//			
				//		}
				//	}
				//}
				//for(size_t s_i = 0;s_i < m_QEInfo.satInfoList_P2.size();s_i++)
				//{
				//	for(size_t s_j = 0;s_j < m_QEInfo.satInfoList_P2[s_i].arcList.size();s_j++)
				//	{
				//		for(size_t s_k = 0;s_k < m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList.size();s_k++)
				//		{
				//			fprintf(pP2File,"%2d  %2d  %10.1f  %14.2f  %14.2f  %14.4f\n",
				//				m_QEInfo.satInfoList_P1[s_i].id_sat,
				//				s_j + 1,
				//				m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].t - m_QEInfo.satInfoList_P2[s_i].arcList[0].resList[0].t,								
				//				m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].Elevation,
				//				m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].Azimuth,
				//				m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].res);
				//			
				//		}
				//	}
				//}
				//for(size_t s_i = 0;s_i < m_QEInfo.satInfoList_L.size();s_i++)
				//{
				//	for(size_t s_j = 0;s_j < m_QEInfo.satInfoList_L[s_i].arcList.size();s_j++)
				//	{
				//		for(size_t s_k = 0;s_k < m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList.size();s_k++)
				//		{
				//			fprintf(pLFile,"%2d  %2d  %10.1f  %14.2f  %14.2f  %14.4f\n",
				//				m_QEInfo.satInfoList_P1[s_i].id_sat,
				//				s_j + 1,
				//				m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].t - m_QEInfo.satInfoList_L[s_i].arcList[0].resList[0].t,								
				//				m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Elevation,
				//				m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Azimuth,
				//				m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].res);
				//			
				//		}
				//	}
				//}			
				//fclose(pP1File);
				//fclose(pP2File);
				//fclose(pLFile);//
				return true;
			}
		}	
		//   子程序名称： mainFunc_ThrFreObs   
		//   作用：三频观测数据质量评估
		//   类型：strEdtedObsfilePath: 预处理后的观测数据路径		

		//   输入：strEdtedObsfilePath
		//   输出：
		//   其它：
		//   语言： C++
		//   版本号：2013/4/1
		//   生成者：刘俊宏
		//   修改记录：1.创建OQE文件夹，方便数据管理//2014/06/25
		//   备注：
		bool BDObsQualityEvaluate::mainFunc_ThrFreObs(string  strEdtedObsfilePath)
		{
			// 分析 strEdtedObsfilePath 路径, 提取根目录和文件名
			string edtedFileName = strEdtedObsfilePath.substr(strEdtedObsfilePath.find_last_of("\\") + 1);
			string folder = strEdtedObsfilePath.substr(0, strEdtedObsfilePath.find_last_of("\\"));
			string edtedFileName_noexp = edtedFileName.substr(0, edtedFileName.find_last_of("."));
			// 创建质量评估目录
			string strOQEFolder = folder + "\\OQE";
			_mkdir(strOQEFolder.c_str());
			Rinex2_1_EditedObsFile m_editedObsFile;
			if(!m_editedObsFile.open(strEdtedObsfilePath))
			{
				printf("%s 文件无法打开!\n", strEdtedObsfilePath.c_str());
				return false;
			}			
			// 寻找观测类型观测序列中的序号
			int nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_P5 = -1,nObsTypes_L1 = -1, nObsTypes_L2 = -1,nObsTypes_L5 = -1;
			for(int i = 0; i < m_editedObsFile.m_header.byObsTypes; i++)
			{				
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1 = i;
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2 = i;
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1 = i;
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2 = i;
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P5)
					nObsTypes_P5 = i;
				if(m_editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L5)
					nObsTypes_L5 = i;
			}
			if(nObsTypes_P1 == -1 || nObsTypes_P2 == -1 || nObsTypes_L1 == -1 || nObsTypes_L2 == -1 || nObsTypes_P5 == -1 || nObsTypes_L5 == -1) 
			{
				printf("观测数据不完整！");
				return false;
			}
			////统计同一时刻所有卫星都发生周跳的情况
			//FILE *pfile_s = fopen("C:\\cycleslip_time.cpp","a+");
			//for(size_t s_i = 0; s_i < m_editedObsFile.m_data.size();s_i++)
			//{
			//	int k = 0;
			//	int j = 0;				
			//	for(Rinex2_1_EditedObsSatMap::iterator it = m_editedObsFile.m_data[s_i].editedObs.begin();it != m_editedObsFile.m_data[s_i].editedObs.end();++it)
			//	{
			//		if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_NORMAL)
			//			k++;
			//		if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
			//			j++;
			//		if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 == 3 &&it->second.obsTypeList[nObsTypes_L1].byEditedMark2 == 0)
			//			j++;
			//	}
			//	if(k == m_editedObsFile.m_data[s_i].editedObs.size()&& k != j)
			//		fprintf(pfile_s,"%s  %s\n",strEdtedObsfilePath.c_str(),m_editedObsFile.m_data[s_i].t.toString().c_str());
			//}
			//fclose(pfile_s);

			vector<Rinex2_1_EditedObsSat> editedObsSatlist;			
			double rms_P1 = 0;
			double rms_P2 = 0;
			double rms_L = 0;
			if(!m_editedObsFile.getEditedObsSatList(editedObsSatlist))
			{
				printf("获取预处理后的观测数据失败！");
				return false;
			}
			//// 仅统计IGSO卫星的观测数据质量
			//vector<Rinex2_1_EditedObsSat> editedObsSatlistIGSO;
			//for(size_t s_i = 0; s_i < editedObsSatlist.size();s_i++)
			//	if(editedObsSatlist[s_i].Id > 5)
			//		editedObsSatlistIGSO.push_back(editedObsSatlist[s_i]);
			//editedObsSatlist.clear();
			//editedObsSatlist = editedObsSatlistIGSO;
			
			if(!evaluate_thrfre_multipath(editedObsSatlist, nObsTypes_P1, nObsTypes_P2, nObsTypes_P5,nObsTypes_L1, nObsTypes_L2, nObsTypes_L5))				
			{
				printf("观测数据质量评估失败！");
				return false;
			}			
			else
			{
				//统计平均可视卫星数和数据总数			
				m_QEInfo.total_Epochs = 0;
				for(size_t s_i = 0; s_i < m_editedObsFile.m_data.size(); s_i++)				
					m_QEInfo.total_Epochs = m_QEInfo.total_Epochs + (int)m_editedObsFile.m_data[s_i].editedObs.size();
				m_QEInfo.mean_VisibleSat = m_QEInfo.total_Epochs/double(m_editedObsFile.m_data.size());
				char infoFilePath[100];
			    sprintf(infoFilePath,"%s\\%s.OQE", strOQEFolder.c_str(), edtedFileName_noexp.c_str());
			    FILE * pInfoFile = fopen(infoFilePath, "w+");
				string StationName = m_editedObsFile.m_header.szMarkName;
				StationName.erase(4, 56);
				fprintf(pInfoFile," 测站%s观测数据质量评估\n",StationName.c_str());
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile," MP1              (m)                   %12.4lf\n",m_QEInfo.rms_P1);
				fprintf(pInfoFile," MP2              (m)                   %12.4lf\n",m_QEInfo.rms_P2);
				fprintf(pInfoFile," MP5              (m)                   %12.4lf\n",m_QEInfo.rms_P5);
				fprintf(pInfoFile," ML               (cm)                  %12.4lf\n",m_QEInfo.rms_L*100);
				fprintf(pInfoFile," 观测历元总数     (个)                  %12d\n"   ,m_QEInfo.total_Epochs);
				fprintf(pInfoFile," 平均可视卫星颗数 (颗)                  %12.2lf\n",m_QEInfo.mean_VisibleSat);
				fprintf(pInfoFile," 伪码正常数据比率                       %12.4lf\n",m_QEInfo.ratio_P_normal);
				fprintf(pInfoFile," 相位正常数据比率                       %12.4lf\n",m_QEInfo.ratio_L_normal);
				fprintf(pInfoFile," 周跳比率                               %12.4lf\n",m_QEInfo.ratio_SLip);
				fprintf(pInfoFile, "======================================================\n");				
				fprintf(pInfoFile," 野值信息统计\n");
				fprintf(pInfoFile,"   标记为野值的原因                             个数\n");
				for(QEAbNormalObsCountMap::iterator it = m_QEInfo.AbnormalObsCount.begin();it != m_QEInfo.AbnormalObsCount.end();it++)
				{
					if(it->first == 20)
						fprintf(pInfoFile,"     无观测数据                           %10d\n",it->second);
					if(it->first == 21)
						fprintf(pInfoFile,"     观测数据为零                         %10d\n",it->second);
					if(it->first == 22)
						fprintf(pInfoFile,"     观测弧段过短                         %10d\n",it->second);
					if(it->first == 23)
						fprintf(pInfoFile,"     性噪比过低                           %10d\n",it->second);
					if(it->first == 24)
						fprintf(pInfoFile,"     观测仰角过低                         %10d\n",it->second);
					if(it->first == 25)
						fprintf(pInfoFile,"     电离层超差                           %10d\n",it->second);
					if(it->first == 26)
						fprintf(pInfoFile,"     三频伪距相位GIF组合                  %10d\n",it->second);
					if(it->first == 27)
						fprintf(pInfoFile,"     Vondrak滤波拟合超差                  %10d\n",it->second);
					if(it->first == 28)
						fprintf(pInfoFile,"     三频相位无几何距离组合超差           %10d\n",it->second);					
				}
				fprintf(pInfoFile," 周跳信息统计\n");
				fprintf(pInfoFile,"   标记为周跳的原因                              个数\n");
				for(QEAbNormalObsCountMap::iterator it = m_QEInfo.AbnormalObsCount.begin();it != m_QEInfo.AbnormalObsCount.end();it++)
				{					
					if(it->first == 30)
						fprintf(pInfoFile,"     新弧段起始点                         %10d\n",it->second);
					if(it->first == 31)
						fprintf(pInfoFile,"     MW组合                               %10d\n",it->second);
					if(it->first == 32)
						fprintf(pInfoFile,"     消电离层组合                         %10d\n",it->second);
					if(it->first == 33)
						fprintf(pInfoFile,"     L1-L2组合                            %10d\n",it->second);
					if(it->first == 34)
						fprintf(pInfoFile,"     三频伪距相位GIF组合                  %10d\n",it->second);
					if(it->first == 35)
						fprintf(pInfoFile,"     三频    相位GIF组合                  %10d\n",it->second);
					if(it->first == 36)
						fprintf(pInfoFile,"     三频    相位 GF组合                  %10d\n",it->second);
				}
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile, " 伪距观测数据质量与仰角的关系（不包括GEO卫星）\n");
				fprintf(pInfoFile, "  Ele(deg)     Epochs     MP1(m)     MP2(m)    MP5(m)\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfolist_ele.size(); s_i ++)
				{
					fprintf(pInfoFile," %4.1f-%4.1f     %6d%10.3lf %10.3lf%10.3lf\n",
						               m_QEInfo.satInfolist_ele[s_i].e0,
									   m_QEInfo.satInfolist_ele[s_i].e1,
									   m_QEInfo.satInfolist_ele[s_i].epochs_P,
									   m_QEInfo.satInfolist_ele[s_i].rms_P1,
									   m_QEInfo.satInfolist_ele[s_i].rms_P2,
									   m_QEInfo.satInfolist_ele[s_i].rms_P5);
				}
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile, " 相位观测数据质量与仰角的关系（不包括GEO卫星）\n");
				fprintf(pInfoFile, "  Ele(deg)     Epochs     ML(cm)     Slips\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfolist_ele.size(); s_i ++)
				{
					fprintf(pInfoFile," %4.1f-%4.1f     %6d%10.3lf     %6d\n",
						               m_QEInfo.satInfolist_ele[s_i].e0,
									   m_QEInfo.satInfolist_ele[s_i].e1,
									   m_QEInfo.satInfolist_ele[s_i].epochs_L,
									   m_QEInfo.satInfolist_ele[s_i].rms_L * 100,
									   m_QEInfo.satInfolist_ele[s_i].slips);
				}
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile, " 信噪比与仰角的关系（不包括GEO卫星）\n");
				fprintf(pInfoFile, "  Ele(deg)     SN1(dB)   SN2(dB)    SN3(dB)\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfolist_ele.size(); s_i ++)
				{
					fprintf(pInfoFile," %4.1f-%4.1f %10.3lf%10.3lf %10.3lf\n",
						               m_QEInfo.satInfolist_ele[s_i].e0,
									   m_QEInfo.satInfolist_ele[s_i].e1,
									   m_QEInfo.satInfolist_ele[s_i].CN_L1,
									   m_QEInfo.satInfolist_ele[s_i].CN_L2,
									   m_QEInfo.satInfolist_ele[s_i].CN_L5);
				}
				fprintf(pInfoFile, "======================================================\n");
				if(m_QEInfo.satInforlist_CycleSlip.size() > 0)
				{
					fprintf(pInfoFile, " 周跳信息\n");
					fprintf(pInfoFile, " PRN                Time                  Method\n");
					for(size_t s_i = 0; s_i < m_QEInfo.satInforlist_CycleSlip.size(); s_i ++)
					{
						if(m_QEInfo.satInforlist_CycleSlip[s_i].preproc_info == 31)
							fprintf(pInfoFile,"  %2d   %s  M-W组合\n",
										   m_QEInfo.satInforlist_CycleSlip[s_i].id_sat,
										   m_QEInfo.satInforlist_CycleSlip[s_i].t.toString().c_str());
						if(m_QEInfo.satInforlist_CycleSlip[s_i].preproc_info == 34)
							fprintf(pInfoFile,"  %2d   %s  三频伪距相位GIF组合\n",
										   m_QEInfo.satInforlist_CycleSlip[s_i].id_sat,
										   m_QEInfo.satInforlist_CycleSlip[s_i].t.toString().c_str());
						if(m_QEInfo.satInforlist_CycleSlip[s_i].preproc_info == 35)
							fprintf(pInfoFile,"  %2d   %s  三频    相位GIF组合\n",
										   m_QEInfo.satInforlist_CycleSlip[s_i].id_sat,
										   m_QEInfo.satInforlist_CycleSlip[s_i].t.toString().c_str());
						if(m_QEInfo.satInforlist_CycleSlip[s_i].preproc_info == 36)
							fprintf(pInfoFile,"  %2d   %s  三频    相位 GF组合\n",
										   m_QEInfo.satInforlist_CycleSlip[s_i].id_sat,
										   m_QEInfo.satInforlist_CycleSlip[s_i].t.toString().c_str());
					}
					fprintf(pInfoFile, "======================================================\n");
				}
				fprintf(pInfoFile," 每颗卫星的观测数据质量\n");				
				fprintf(pInfoFile," PRN Epochs Slips   MP1(m)  MP2(m)  MP5(m)  ML(cm)\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P1.size(); s_i++)
				{
					fprintf(pInfoFile,"  %2d  %5d   %3d%8.3lf%8.3lf%8.3lf%8.3lf\n",
						               m_QEInfo.satInfoList_P1[s_i].id_sat,									   
									   m_QEInfo.satInfoList_P1[s_i].epochs,
									   m_QEInfo.satInfoList_P1[s_i].slips,
						               m_QEInfo.satInfoList_P1[s_i].rms_sat,
									   m_QEInfo.satInfoList_P2[s_i].rms_sat,
									   m_QEInfo.satInfoList_P5[s_i].rms_sat,
									   m_QEInfo.satInfoList_L[s_i].rms_sat * 100);

				}				
				fprintf(pInfoFile, "======================================================\n");
				fprintf(pInfoFile," 每个弧段的观测数据质量\n");
				fprintf(pInfoFile," PRN  Arc_N         MP1(m)  MP2(m)  MP5(m)  ML(cm)\n");
				for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P1.size(); s_i++)
				{					
					for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_P1[s_i].arcList.size(); s_j++)
						fprintf(pInfoFile,"  %2d    %3d      %8.3lf%8.3lf%8.3lf%8.3lf\n",
						                   m_QEInfo.satInfoList_P1[s_i].id_sat,
						                   s_j + 1,
										   m_QEInfo.satInfoList_P1[s_i].arcList[s_j].rms_arc,
										   m_QEInfo.satInfoList_P2[s_i].arcList[s_j].rms_arc,
										   m_QEInfo.satInfoList_P5[s_i].arcList[s_j].rms_arc,
										   m_QEInfo.satInfoList_L[s_i].arcList[s_j].rms_arc * 100);										
				}
				fclose(pInfoFile);
				//char P1ResFilePath[100];
				//char P2ResFilePath[100];
				//char LResFilePath[100];
			 //   sprintf(P1ResFilePath,"%s\\%s.RP1", folder.c_str(), edtedFileName_noexp.c_str());
				//sprintf(P2ResFilePath,"%s\\%s.RP2", folder.c_str(), edtedFileName_noexp.c_str());
				//sprintf(LResFilePath,"%s\\%s.RL", folder.c_str(), edtedFileName_noexp.c_str());
				//FILE * pP1File = fopen(P1ResFilePath, "w+");
				//FILE * pP2File = fopen(P2ResFilePath, "w+");
				//FILE * pLFile  = fopen(LResFilePath, "w+");
				//for(size_t s_i = 0;s_i < m_QEInfo.satInfoList_P1.size();s_i++)
				//{
				//	for(size_t s_j = 0;s_j < m_QEInfo.satInfoList_P1[s_i].arcList.size();s_j++)
				//	{
				//		for(size_t s_k = 0;s_k < m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList.size();s_k++)
				//		{
				//			fprintf(pP1File,"%10.1f  %14.2f  %14.2f  %14.4f\n",
				//				m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].t - m_QEInfo.satInfoList_P1[s_i].arcList[0].resList[0].t,
				//				m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].Elevation,
				//				m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].Azimuth,
				//				m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].res);
				//			
				//		}
				//	}
				//}
				//for(size_t s_i = 0;s_i < m_QEInfo.satInfoList_P2.size();s_i++)
				//{
				//	for(size_t s_j = 0;s_j < m_QEInfo.satInfoList_P2[s_i].arcList.size();s_j++)
				//	{
				//		for(size_t s_k = 0;s_k < m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList.size();s_k++)
				//		{
				//			fprintf(pP2File,"%10.1f  %14.2f  %14.2f  %14.4f\n",
				//				m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].t - m_QEInfo.satInfoList_P2[s_i].arcList[0].resList[0].t,
				//				m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].Elevation,
				//				m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].Azimuth,
				//				m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].res);
				//			
				//		}
				//	}
				//}
				//for(size_t s_i = 0;s_i < m_QEInfo.satInfoList_L.size();s_i++)
				//{
				//	for(size_t s_j = 0;s_j < m_QEInfo.satInfoList_L[s_i].arcList.size();s_j++)
				//	{
				//		for(size_t s_k = 0;s_k < m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList.size();s_k++)
				//		{
				//			fprintf(pLFile,"%10.1f  %14.2f  %14.2f  %14.4f\n",
				//				m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].t - m_QEInfo.satInfoList_L[s_i].arcList[0].resList[0].t,
				//				m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Elevation,
				//				m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Azimuth,
				//				m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].res);
				//			
				//		}
				//	}
				//}			
				//fclose(pP1File);
				//fclose(pP2File);
				//fclose(pLFile);//
				return true;
			}
		}

		//   子程序名称： evaluate_code_multipath   
		//   作用：伪码观测数据质量评估
		//   类型：editedObsSatlist: 数据结构, 主要用途方便时间序列处理		
		//         index_P1          : P1码位置索引
		//         index_P2          : P2码位置索引
		//         index_L1          : L1位置索引
		//         index_L2          : L2位置索引
		//   输入：editedObsSatlist， index_P1，index_P2，index_L1，index_L2
		//   输出：
		//   其它：
		//   语言： C++
		//   版本号：2012/9/29
		//   生成者：刘俊宏，谷德峰
		//   修改记录：
		//   备注：
		bool BDObsQualityEvaluate::evaluate_code_multipath(vector<Rinex2_1_EditedObsSat> editedObsSatlist, int index_P1, int index_P2, int index_L1, int index_L2)
		{			
			int    statistic[40];                                     // 统计各种数据类型的个数   
			for(int i = 0; i < 40; i++)
				statistic[i] = 0;
			int m_nCycleSlip = 0;  // 统计周跳发生的个数    
			int m_nP1P2Count = 0;  // 统计正常的伪距观测数据个数   // 注： P1和P2均正常，则正常数据个数加2，相位统计方法类似
			int m_nL1L2Count = 0;  // 统计正常的相位观测数据个数		
			int m_nObsCount  = 0;  // 观测数据个数		           // 注： P1和P2或L1和L2的总个数
			const double alpha = pow(BD_FREQUENCE_L1,2) / pow(BD_FREQUENCE_L2,2);
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{
				QERESIDUAL_SAT   QEresidual_satP1;
				QERESIDUAL_SAT   QEresidual_satP2;  
				size_t nCount = editedObsSatlist[s_i].editedObs.size();   // 观测时间序列数据个数（某颗固定的卫星）	
				m_nObsCount = m_nObsCount + 2 * (int)nCount;
				double *pObsTime         = new double[nCount];            // 相对时间序列			
				double *pMultipath_P1    = new double[nCount];            // 伪距P1多路经组合观测数据序列
				double *pMultipath_P2    = new double[nCount];            // 伪距P2多路经组合观测数据序列
				//double *pCN_P1           = new double[nCount];            // 伪距P1信噪比序列
				//double *pCN_P2           = new double[nCount];            // 伪距P2信噪比序列
				double *pElevation       = new double[nCount];           
				double *pAzimuth         = new double[nCount];           
				int    *pEditedflag      = new int   [nCount];
				int    *pEditedflag_code = new int   [nCount]; 
                Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSatlist[s_i].editedObs.begin();
				DayTime t0 = it0->first; // 初始时间 - 用于计算相对时间(秒)
				int j = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{					
                    pObsTime[j] =  it->first - t0;	
					//if(it->second.Elevation < min_elevation)
					//	m_nlowECount = m_nlowECount + 4;
                    Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[index_P1];
					Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[index_P2];
					Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[index_L1];
					Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[index_L2];
					// 统计异常信息					
					BYTE   bynum = P1.byEditedMark1*10 + P1.byEditedMark2;
					statistic[bynum]++;
					bynum = P2.byEditedMark1*10 + P2.byEditedMark2;
					statistic[bynum]++;
					bynum = L1.byEditedMark1*10 + L1.byEditedMark2;
					statistic[bynum]++;
					bynum = L2.byEditedMark1*10 + L2.byEditedMark2;
					statistic[bynum]++;	
					if( P1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						m_nP1P2Count++;
					if( P2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
				    	m_nP1P2Count++;
					if( L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL || L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
						m_nL1L2Count++;
					if( L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL || L2.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
				    	m_nL1L2Count++;
					//if( L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP && L1.byEditedMark2 != 0)
					//	m_nCycleSlip++;
					if( L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP && L1.byEditedMark2 != 0)
					{
						m_nCycleSlip++;
						if(it->second.Id > 5)
						{
							int  ele_inter  = m_QEInfo.getInterval(it->second.Elevation);
							m_QEInfo.satInfolist_ele[ele_inter].slips ++;
						}
						CycleSlip_Info   slip_info;
						slip_info.t      = it->first;
						slip_info.id_sat = it->second.Id;
						slip_info.preproc_info = L1.byEditedMark1 * 10 + L1.byEditedMark2;
						m_QEInfo.satInforlist_CycleSlip.push_back(slip_info);
						QEresidual_satP1.slips = QEresidual_satP1.slips + 1;
					}
					double dP1 = P1.obs.data;
					double dP2 = P2.obs.data;
					double dL1 = BD_WAVELENGTH_L1 * L1.obs.data;
					double dL2 = BD_WAVELENGTH_L2 * L2.obs.data;
					double dIF = (1 / (alpha - 1)) * (dL1 - dL2);					
					pMultipath_P1[j] = dP1 - dL1 - 2 * dIF;
					pMultipath_P2[j] = dP2 - dL2 - 2 * alpha * dIF;
					pElevation[j]    = it->second.Elevation;
					pAzimuth[j]      = it->second.Azimuth;
					if(P1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && P2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						pEditedflag_code[j] = TYPE_EDITEDMARK_NORMAL; 
					else
					{
						pEditedflag_code[j] = TYPE_EDITEDMARK_OUTLIER;
					}
					if(L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						pEditedflag[j] = 0; // 正常
					else if(L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP || L2.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
					{
						pEditedflag[j] = 1; // 新的周跳标记
						//m_nCycleSlip++;
					}
					else
					{
						pEditedflag[j] = 2; // 异常点
					}
					j++;
				}
				size_t k   = 0;             // 记录新弧段起始点				
				size_t k_i = k;             // 记录新弧段终止点
				while(1)
				{
					if(k_i + 1 >= nCount)   // k_i 为时间序列终点
						goto newArc;
					else
					{
						// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
						if(pEditedflag[k_i + 1] != 1)
						{
							k_i++;
							continue;
						}						
						else // k_i + 1 为新弧段的起点
							goto newArc;
					}
					newArc:  // 本弧段[k，k_i]数据处理 
					{
						QERESIDUAL_ARC   QEresidual_arcP1;
						QERESIDUAL_ARC   QEresidual_arcP2; 
						vector<size_t>   listNormalPoint;                        // 记录正常数据
						listNormalPoint.clear();
						for( size_t s_k = k; s_k <= k_i; s_k++ )
						{
							if((pEditedflag[s_k] == 0 || pEditedflag[s_k] == 1)
							&&  pEditedflag_code[s_k] == TYPE_EDITEDMARK_NORMAL) // 正常数据标记 1
								listNormalPoint.push_back(s_k);     
						}
						size_t nArcPointsNumber = listNormalPoint.size();      // 正常数据点个数
						if(nArcPointsNumber > 20)
						{
							//// 计算均值和残差
							double* pX  = new double [nArcPointsNumber];
							double* pW  = new double [nArcPointsNumber];
							double mean = 0;
							double var  = 0;							
							// 第二步: P1码的多路经残差计算						
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)	
							{
								pX[s_k] = pMultipath_P1[listNormalPoint[s_k]];
								mean = mean + pX[s_k];							
							}
							mean = mean/nArcPointsNumber;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)	
							{							
								var = var + pow(pX[s_k] - mean, 2);							
							}
							var = sqrt(var/(nArcPointsNumber - 1));
							QEresidual_arcP1.rms_arc = var;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)
							{
								QERESIDUAL     P1;
								P1.t           = t0 + pObsTime[listNormalPoint[s_k]];
								P1.Elevation   = pElevation[listNormalPoint[s_k]];
								P1.Azimuth     = pAzimuth[listNormalPoint[s_k]];
								P1.res         = pX[s_k] - mean;								
								P1.id_elevation_Inter = m_QEInfo.getInterval(P1.Elevation);		
								QEresidual_arcP1.resList.push_back(P1);
								if(editedObsSatlist[s_i].Id > 5)
								{//不统计GEO卫星
									m_QEInfo.satInfolist_ele[P1.id_elevation_Inter].epochs_P ++;
									m_QEInfo.satInfolist_ele[P1.id_elevation_Inter].rms_P1 += P1.res * P1.res;
								}							
							}						
							// 第三步: P2码的多路经残差计算
							mean = 0;
							var  = 0;						
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)	
							{
								pX[s_k] = pMultipath_P2[listNormalPoint[s_k]];
								mean = mean + pX[s_k];
							}
							mean = mean/nArcPointsNumber;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)	
							{							
								var = var + pow(pX[s_k] - mean, 2);							
							}
							var = sqrt(var/(nArcPointsNumber - 1));
							QEresidual_arcP2.rms_arc = var;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)
							{
								QERESIDUAL     P2;
								P2.t           = t0 + pObsTime[listNormalPoint[s_k]];
								P2.Elevation   = pElevation[listNormalPoint[s_k]];
								P2.Azimuth     = pAzimuth[listNormalPoint[s_k]];
								P2.res         = pX[s_k] - mean;								
								P2.id_elevation_Inter = m_QEInfo.getInterval(P2.Elevation);	
								QEresidual_arcP2.resList.push_back(P2);
								if(editedObsSatlist[s_i].Id > 5)
								{//不统计GEO卫星									
									m_QEInfo.satInfolist_ele[P2.id_elevation_Inter].rms_P2 += P2.res * P2.res;
								}	
								
							}									
							QEresidual_satP1.arcList.push_back(QEresidual_arcP1);						
							QEresidual_satP2.arcList.push_back(QEresidual_arcP2);						
							delete pX;
							delete pW;
						}
						if(k_i + 1 >= nCount) // k_i为时间序列终点, 跳出
							break;
						else  
						{   
							k   = k_i + 1;    // 新弧段的起点设置
							k_i = k;
							continue;
						}
					}
				}
				if(QEresidual_satP1.arcList.size() > 0)
				{
					////// 统计每颗卫星的均方根
					QEresidual_satP1.rms_sat = 0;
					int satObsNum = 0; //每一颗卫星使用的观测数据个数
					for(size_t s_n = 0; s_n < QEresidual_satP1.arcList.size(); s_n++)
					{
						QEresidual_satP1.rms_sat += pow(QEresidual_satP1.arcList[s_n].rms_arc, 2) * (QEresidual_satP1.arcList[s_n].resList.size()-1);
						satObsNum = satObsNum + (int)QEresidual_satP1.arcList[s_n].resList.size();
					}
					QEresidual_satP1.epochs  = satObsNum;
					QEresidual_satP1.rms_sat = sqrt(QEresidual_satP1.rms_sat / (satObsNum - 1));				
					QEresidual_satP2.rms_sat = 0;
					satObsNum = 0;
					for(size_t s_n = 0; s_n < QEresidual_satP2.arcList.size(); s_n++)
					{
						QEresidual_satP2.rms_sat  += pow(QEresidual_satP2.arcList[s_n].rms_arc, 2) * (QEresidual_satP2.arcList[s_n].resList.size()-1);
						satObsNum = satObsNum + (int)QEresidual_satP2.arcList[s_n].resList.size();
					}
					QEresidual_satP2.rms_sat = sqrt(QEresidual_satP2.rms_sat / (satObsNum - 1));
					QEresidual_satP1.id_sat = editedObsSatlist[s_i].Id;
					QEresidual_satP2.id_sat = editedObsSatlist[s_i].Id;
					m_QEInfo.satInfoList_P1.push_back(QEresidual_satP1);
					m_QEInfo.satInfoList_P2.push_back(QEresidual_satP2);
				}
				delete pObsTime;				
				delete pMultipath_P1;
				delete pMultipath_P2;
				delete pElevation;
				delete pAzimuth;
				delete pEditedflag;
			}
			// 统计异常信息
			for(int i = 0 ; i < 40; i++)
			{
				if(statistic[i] > 0)
				{
					if(i >= 30)
						statistic[i] = statistic[i]/2;
					m_QEInfo.AbnormalObsCount.insert(QEAbNormalObsCountMap::value_type(i,statistic[i]));
				}
			}
			m_QEInfo.ratio_P_normal = m_nP1P2Count / (double)m_nObsCount;			
			m_QEInfo.ratio_L_normal = m_nL1L2Count / (double)m_nObsCount;
			m_QEInfo.ratio_SLip     = m_nCycleSlip * 2 / (double)m_nObsCount;
			//m_QEInfo.ratio_SLip     = statistic[21]  / ((double)m_nObsCount * 2 - m_nlowECount);
			//m_QEInfo.ratio_P_normal = kk1/(double)kk;			
			//m_QEInfo.ratio_L_normal = kk2/(double)kk;
			//m_QEInfo.ratio_SLip = kk3/(double)kk;
			//m_QEInfo.rms_P1 = kk4/(double)kk;
			//m_QEInfo.rms_P2 = kk5/(double)kk;
			////// 统计测站的均方根			
			m_QEInfo.rms_P1 = 0;
			int   staObsNum = 0; //每一个测站使用的观测数据个数
			for(size_t s_n = 0; s_n < m_QEInfo.satInfoList_P1.size(); s_n++)
			{
				int satObsNum = 0;
				for(size_t s_m = 0; s_m < m_QEInfo.satInfoList_P1[s_n].arcList.size(); s_m++)
					satObsNum = satObsNum + (int)m_QEInfo.satInfoList_P1[s_n].arcList[s_m].resList.size();
				m_QEInfo.rms_P1 += pow(m_QEInfo.satInfoList_P1[s_n].rms_sat, 2) * (satObsNum - 1);
				staObsNum = staObsNum + satObsNum;
			}
			m_QEInfo.rms_P1 = sqrt(m_QEInfo.rms_P1 / (staObsNum - 1));
			m_QEInfo.rms_P2 = 0;
			staObsNum = 0;
			for(size_t s_n = 0; s_n < m_QEInfo.satInfoList_P2.size(); s_n++)
			{
				int satObsNum = 0;
				for(size_t s_m = 0; s_m < m_QEInfo.satInfoList_P2[s_n].arcList.size(); s_m++)
					satObsNum = satObsNum + (int)m_QEInfo.satInfoList_P2[s_n].arcList[s_m].resList.size();
				m_QEInfo.rms_P2 += pow(m_QEInfo.satInfoList_P2[s_n].rms_sat, 2) * (satObsNum - 1);
				staObsNum = staObsNum + satObsNum;
			}
			m_QEInfo.rms_P2 = sqrt(m_QEInfo.rms_P2 / (staObsNum - 1));
			// 统计测站的观测数据质量与仰角的关系
			for(size_t s_i = 0; s_i < m_QEInfo.satInfolist_ele.size(); s_i ++)
			{
				int obs_P = m_QEInfo.satInfolist_ele[s_i].epochs_P;				
				if(obs_P> 1)
				{
					m_QEInfo.satInfolist_ele[s_i].rms_P1 = sqrt(m_QEInfo.satInfolist_ele[s_i].rms_P1 / (obs_P - 1));
					m_QEInfo.satInfolist_ele[s_i].rms_P2 = sqrt(m_QEInfo.satInfolist_ele[s_i].rms_P2 / (obs_P - 1));					
				}		
			}
			

			//int nArcCount = 0;
			//FILE *pfile = fopen("C:\\residuls_P1.cpp","w+");
			//for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P1.size(); s_i++)
			//{
			//	for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_P1[s_i].arcList.size(); s_j++)
			//	{
			//		if(m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList.size() != 0)
			//		{
			//			nArcCount++;
			//			for(size_t s_k = 0; s_k < m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList.size(); s_k++)										
			//				fprintf(pfile,"%s %2d %2d %8.4f %8.4f %10.6lf\n",m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].t.toString().c_str(),
			//											 m_QEInfo.satInfoList_P1[s_i].id_sat,
			//									 		 nArcCount,
			//											 m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].Elevation,
			//											 m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].Azimuth,
			//											 m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].res);
			//		}
			//	}

			//}
			//fclose(pfile);//
			//nArcCount = 0;
			//FILE *pfile1 = fopen("C:\\residuls_P2.cpp","w+");
			//for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P2.size(); s_i++)
			//{
			//	for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_P2[s_i].arcList.size(); s_j++)
			//	{
			//		if(m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList.size() != 0)
			//		{
			//			nArcCount++;
			//			for(size_t s_k = 0; s_k < m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList.size(); s_k++)										
			//				fprintf(pfile1,"%s %2d %2d %8.4f %8.4f %10.6lf\n",m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].t.toString().c_str(),
			//											 m_QEInfo.satInfoList_P2[s_i].id_sat,
			//									 		 nArcCount,
			//											 m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].Elevation,
			//											 m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].Azimuth,
			//											 m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].res);
			//		}
			//	}

			//}
			//fclose(pfile1);//
			return true;
		}
        //   子程序名称： evaluate_phase_poly   
		//   作用：评估相位观测数据的质量, [采用对相位电离层残差组合Vondrank滤波拟合的方法]
		//   类型：editedObsSatlist  : 数据结构, 根据不同GPS卫星进行分类, 主要用途方便时间序列处理
		//         index_L1          :  L1位置索引
		//         index_L2          :  L2位置索引
		//         vondrak_LIF_eps   :  vondrak 拟合参数
		//         vondrak_LIF_max   :  vondrak 拟合参数
        //         vondrak_LIF_min   :  vondrak 拟合参数
        //         vondrak_LIF_width :  vondrak 拟合参数
		//   输入：editedObsSatlist, index_L1, index_L2,vondrak_LIF_eps,vondrak_LIF_max,vondrak_LIF_min,vondrak_LIF_min,vondrak_LIF_width
		//   输出：
		//   其它：
		//   语言： C++
		//   版本号：2012.9.27
		//   生成者：刘俊宏，谷德峰
		//   修改者：
		bool BDObsQualityEvaluate::evaluate_phase_vondrak(vector<Rinex2_1_EditedObsSat> editedObsSatlist, int index_L1, int index_L2, double vondrak_L1_L2_eps, double vondrak_L1_L2_max,double vondrak_L1_L2_min,unsigned int vondrak_L1_L2_width)
		{			
			int nArcCount = 0;                       // 记录弧段			
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{
				QERESIDUAL_SAT QEresidual_sat;
				size_t nCount = editedObsSatlist[s_i].editedObs.size();   // 观测时间序列数据个数				
			    double *pObsTime         = new double[nCount];            // 相对时间序列
				double *ionoL1_L2        = new double[nCount];            // L1 - L2 残差
				double *vondrak_fit      = new double[nCount];
				double *pElevation       = new double[nCount];           
				double *pAzimuth         = new double[nCount]; 
				int    *pEditedflag      = new int   [nCount];            // 编辑标记序列 0--正常    1--新的周跳标记   2--异常点
				Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSatlist[s_i].editedObs.begin();
				DayTime t0 = it0->first; // 初始时间 - 用于计算相对时间(秒)
				int j = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{
					pObsTime[j] =  it->first - t0;
					Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[index_L1];
					Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[index_L2];
					double dL1 = BD_WAVELENGTH_L1 * L1.obs.data;
					double dL2 = BD_WAVELENGTH_L2 * L2.obs.data;
					ionoL1_L2[j] = dL1 - dL2;
					pElevation[j]  = it->second.Elevation;
					pAzimuth[j]    = it->second.Azimuth;
					if(L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						pEditedflag[j] = 0; // 正常
					else if(L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP || L2.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
					{
						pEditedflag[j] = 1; // 新的周跳标记
					}
					else
					{
						pEditedflag[j] = 2; // 异常点
					}
					j++;
				}

				size_t k   = 0; // 记录新弧段起始点				
				size_t k_i = k; // 记录新弧段终止点				
				while(1)
				{
					if(k_i + 1 >= nCount) // k_i 为时间序列终点
						goto newArc;
					else
					{
						// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
						if(pEditedflag[k_i + 1] != 1)
						{
							k_i++;
							continue;
						}						
						else // k_i + 1 为新弧段的起点
							goto newArc;
					}
					newArc:  // 本弧段[k，k_i]数据处理 
					{
						//FILE *pfile = fopen("C:\\residuls_L_fit.cpp","a+");						
						QERESIDUAL_ARC   QEresidual_arc; 
						int nArcPointsCount = int(k_i - k + 1);	
						int nNormPointsCount = 0;
						double *w  = new double [nArcPointsCount];	
						for (size_t s_j = k ; s_j <= k_i; s_j++)
						{
							if(pEditedflag[s_j] == 2)
							{
								w[s_j - k] = 0;								
							}
							else
							{
								w[s_j - k] = 1.0;
								nNormPointsCount++;
							}
						}
						if(nNormPointsCount > 20)
						{
							nArcCount++;
							KinematicRobustVandrakFilter(pObsTime + k , ionoL1_L2 + k, w, nArcPointsCount,
												   vondrak_L1_L2_eps,
												   vondrak_fit + k,
												   vondrak_L1_L2_max,
												   vondrak_L1_L2_min,
												   vondrak_L1_L2_width);//
							for(size_t s_k = k; s_k <= k_i; s_k++)
							{
								if(pEditedflag[s_k] != 2)
								{
									// 记录拟合残差
									QERESIDUAL             QEresidual;
									QEresidual.t           = t0 + pObsTime[s_k];								
									QEresidual.Elevation   = pElevation[s_k];
									QEresidual.Azimuth     = pAzimuth[s_k];
									QEresidual.res         = vondrak_fit[s_k] - ionoL1_L2[s_k];
									QEresidual.id_elevation_Inter = m_QEInfo.getInterval(QEresidual.Elevation);		
									QEresidual_arc.resList.push_back(QEresidual);
									if(editedObsSatlist[s_i].Id > 5)
									{//不统计GEO卫星
										m_QEInfo.satInfolist_ele[QEresidual.id_elevation_Inter].epochs_L ++;
										m_QEInfo.satInfolist_ele[QEresidual.id_elevation_Inter].rms_L += QEresidual.res * QEresidual.res;
									}	
								}
								//if(pEditedflag[s_k] != 2)
								//	fprintf(pfile,"%s %2d %2d %8.4f %8.4f %14.6lf %14.6lf %14.6lf %4.2f\n",
								//		   (t0 + pObsTime[s_k]).toString().c_str(),
								//			editedObsSatlist[s_i].Id,
								//			nArcCount,
								//			pElevation[s_k],
								//			pAzimuth[s_k],
								//			ionoL1_L2[s_k],
								//			vondrak_fit[s_k],
								//			vondrak_fit[s_k] - ionoL1_L2[s_k],
								//			w[s_k - k]);//
							}
							//fclose(pfile);
							//统计每一弧段的均方根误差
							QEresidual_arc.rms_arc = 0;
							for(size_t s_n = 0; s_n < QEresidual_arc.resList.size(); s_n++)
								QEresidual_arc.rms_arc += pow(QEresidual_arc.resList[s_n].res, 2);				
							QEresidual_arc.rms_arc = sqrt(QEresidual_arc.rms_arc / (QEresidual_arc.resList.size() - 1));
							QEresidual_sat.arcList.push_back(QEresidual_arc);						
							delete w;	
						}
						if(k_i + 1 >= nCount) // k_i为时间序列终点, 跳出
							break;
						else  
						{   
							k   = k_i + 1;    // 新弧段的起点设置
							k_i = k;
							continue;
						}
					}
				}
				if(QEresidual_sat.arcList.size() > 0)
				{
					//统计每一卫星的均方根误差
					QEresidual_sat.rms_sat = 0;
					int satObsNum = 0; //每一颗卫星使用的观测数据个数				
					for(size_t s_n = 0; s_n < QEresidual_sat.arcList.size(); s_n++)
					{
						QEresidual_sat.rms_sat += pow(QEresidual_sat.arcList[s_n].rms_arc, 2) * (QEresidual_sat.arcList[s_n].resList.size() - 1);
						satObsNum = satObsNum + (int)QEresidual_sat.arcList[s_n].resList.size();
					}
					QEresidual_sat.rms_sat = sqrt(QEresidual_sat.rms_sat / (satObsNum - 1));
					QEresidual_sat.id_sat = editedObsSatlist[s_i].Id;
					m_QEInfo.satInfoList_L.push_back(QEresidual_sat);
				}
				delete pObsTime;
				delete ionoL1_L2;
				delete pElevation;
				delete pAzimuth;
				delete pEditedflag;
				delete vondrak_fit;
			}
						
			// 统计测站的均方根
			m_QEInfo.rms_L = 0;			
			int   staObsNum = 0; //每一个测站使用的观测数据个数	
			for(size_t s_n = 0; s_n < m_QEInfo.satInfoList_L.size(); s_n++)
			{
				int satObsNum = 0;
				for(size_t s_m = 0; s_m < m_QEInfo.satInfoList_L[s_n].arcList.size(); s_m++)
					satObsNum = satObsNum + (int)m_QEInfo.satInfoList_L[s_n].arcList[s_m].resList.size();
				m_QEInfo.rms_L += pow(m_QEInfo.satInfoList_L[s_n].rms_sat, 2) * (satObsNum - 1);
				staObsNum = staObsNum + satObsNum;
			}
			m_QEInfo.rms_L = sqrt(m_QEInfo.rms_L / (staObsNum - 1));
			// 统计测站的观测数据质量与仰角的关系
			for(size_t s_i = 0; s_i < m_QEInfo.satInfolist_ele.size(); s_i ++)
			{				
				int obs_L = m_QEInfo.satInfolist_ele[s_i].epochs_L;			
				if(obs_L> 1)
					m_QEInfo.satInfolist_ele[s_i].rms_L = sqrt(m_QEInfo.satInfolist_ele[s_i].rms_L / (obs_L - 1));
			}
			   
			//nArcCount = 0;
			//FILE *pfile = fopen("C:\\phase_residuls_L.cpp","w+");
			//for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_L.size(); s_i++)
			//{
			//	for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_L[s_i].arcList.size(); s_j++)
			//	{
			//		if(m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList.size() != 0)
			//		{
			//			nArcCount++;
			//			for(size_t s_k = 0; s_k < m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList.size(); s_k++)										
			//				fprintf(pfile,"%s %2d %2d %8.4f %8.4f %10.6lf\n",m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].t.toString().c_str(),
			//											 m_QEInfo.satInfoList_L[s_i].id_sat,
			//									 		 nArcCount,
			//											 m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Azimuth,
			//											 m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Elevation,
			//											 m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].res);
			//		}
			//	}

			//}
			//fclose(pfile);//

			return true;
		}
		//   子程序名称： evaluate_thrfre_multipath   
		//   作用：三频伪码观测数据质量评估
		//   类型：editedObsSatlist: 数据结构, 主要用途方便时间序列处理		
		//         index_P1          : P1码位置索引
		//         index_P2          : P2码位置索引
		//         index_P5          : P5码位置索引
		//         index_L1          : L1位置索引
		//         index_L2          : L2位置索引
		//         index_L5          : L2位置索引
		//   输入：editedObsSatlist， index_P1，index_P2，index_P5,index_L1，index_L2,index_L5
		//   输出：
		//   其它：
		//   语言： C++
		//   版本号：2013/4/1
		//   生成者：刘俊宏
		//   修改记录：
		//   备注：
		bool BDObsQualityEvaluate::evaluate_thrfre_multipath(vector<Rinex2_1_EditedObsSat> editedObsSatlist, int index_P1, int index_P2, int index_P5, int index_L1, int index_L2, int index_L5)
		{
			int    statistic[40];                                     // 统计各种数据类型的个数   
			int    min_normalpoints = 2;
			for(int i = 0; i < 40; i++)
				statistic[i] = 0;
			int m_nCycleSlip = 0;   // 统计周跳发生的个数    
			int m_nCodeCount = 0;   // 统计正常的伪距观测数据个数   // 注： P1,P2,P5均正常，则正常数据个数加3，相位统计方法类似
			int m_nPhaseCount = 0;  // 统计正常的相位观测数据个数
			m_QEInfo.init();        // 初始化观测仰角列表，2013/9/26			
			int m_nObsCount  = 0;  // 观测数据个数		           // 注： P1,P2和P5或L1,L2和L5的总个数
			const double alpha = pow(BD_FREQUENCE_L1,2) / pow(BD_FREQUENCE_L2,2);
			const double beta  = pow(BD_FREQUENCE_L1,2) / pow(BD_FREQUENCE_L5,2);

			const double coefficient_L1 = 1 / (1 - pow( BD_FREQUENCE_L2 / BD_FREQUENCE_L1, 2 )) -  
				                    1 / (1 - pow( BD_FREQUENCE_L5 / BD_FREQUENCE_L1, 2 ));
			const double coefficient_L2 = - 1 / (pow( BD_FREQUENCE_L1 / BD_FREQUENCE_L2, 2 ) - 1);
			const double coefficient_L5 = 1 / (pow( BD_FREQUENCE_L1 / BD_FREQUENCE_L5, 2 ) - 1);
			
			//FILE *pfile_e = fopen("C:\\test_ele_compute.dat","w+");
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{
				QERESIDUAL_SAT   QEresidual_satP1;
				QERESIDUAL_SAT   QEresidual_satP2;  
				QERESIDUAL_SAT   QEresidual_satP5; 
				QERESIDUAL_SAT   QEresidual_satL;
				QEresidual_satL.slips = 0;                                // 统计每颗卫星的周跳个数
				size_t nCount = editedObsSatlist[s_i].editedObs.size();   // 观测时间序列数据个数（某颗固定的卫星）	
				m_nObsCount = m_nObsCount + 3 * (int)nCount;
				double *pObsTime         = new double[nCount];            // 相对时间序列			
				double *pMultipath_P1    = new double[nCount];            // 伪距P1多路经组合观测数据序列
				double *pMultipath_P2    = new double[nCount];            // 伪距P2多路经组合观测数据序列
				double *pMultipath_P5    = new double[nCount];            // 伪距P5多路经组合观测数据序列
				double *pMultipath_L     = new double[nCount];            // 相位多路经组合观测数据序列
				double *pElevation       = new double[nCount];           
				double *pAzimuth         = new double[nCount];           
				int    *pEditedflag      = new int   [nCount];
				int    *pEditedflag_code = new int   [nCount]; 				  
                Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSatlist[s_i].editedObs.begin();
				DayTime t0 = it0->first; // 初始时间 - 用于计算相对时间(秒)
				int j = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{					
                    pObsTime[j] =  it->first - t0;						
                    Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[index_P1];
					Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[index_P2];
					Rinex2_1_EditedObsDatum P5 = it->second.obsTypeList[index_P5];
					Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[index_L1];
					Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[index_L2];
					Rinex2_1_EditedObsDatum L5 = it->second.obsTypeList[index_L5];
					// 统计异常信息					
					BYTE   bynum = P1.byEditedMark1*10 + P1.byEditedMark2;
					statistic[bynum]++;
					bynum = P2.byEditedMark1*10 + P2.byEditedMark2;
					statistic[bynum]++;
					bynum = P5.byEditedMark1*10 + P5.byEditedMark2;
					statistic[bynum]++;
					bynum = L1.byEditedMark1*10 + L1.byEditedMark2;
					statistic[bynum]++;
					bynum = L2.byEditedMark1*10 + L2.byEditedMark2;
					statistic[bynum]++;	
					bynum = L5.byEditedMark1*10 + L5.byEditedMark2;
					statistic[bynum]++;	
					if( P1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						m_nCodeCount++;
					if( P2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
				    	m_nCodeCount++;
					if( P5.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
				    	m_nCodeCount++;
					if( L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL || L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
						m_nPhaseCount++;
					if( L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL || L2.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
				    	m_nPhaseCount++;
					if( L5.byEditedMark1 == TYPE_EDITEDMARK_NORMAL || L5.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
				    	m_nPhaseCount++;
					if( L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP && L1.byEditedMark2 != 0)
					{
						m_nCycleSlip++;
						if(it->second.Id > 5)
						{
							int  ele_inter  = m_QEInfo.getInterval(it->second.Elevation);
							m_QEInfo.satInfolist_ele[ele_inter].slips ++;
						}
						CycleSlip_Info   slip_info;
						slip_info.t      = it->first;
						slip_info.id_sat = it->second.Id;
						slip_info.preproc_info = L1.byEditedMark1 * 10 + L1.byEditedMark2;
						m_QEInfo.satInforlist_CycleSlip.push_back(slip_info);
						QEresidual_satP1.slips = QEresidual_satP1.slips + 1;
					}
					double dP1 = P1.obs.data;
					double dP2 = P2.obs.data;
					double dP5 = P5.obs.data;
					double dL1 = BD_WAVELENGTH_L1 * L1.obs.data;
					double dL2 = BD_WAVELENGTH_L2 * L2.obs.data;
					double dL5 = BD_WAVELENGTH_L5 * L5.obs.data;
					double dIF = (1 / (alpha - 1)) * (dL1 - dL2);					
					pMultipath_P1[j] = dP1 - dL1 - 2 * dIF;
					pMultipath_P2[j] = dP2 - dL2 - 2 * alpha * dIF;
					pMultipath_P5[j] = dP5 - dL5 - 2 * beta * dIF;
					pMultipath_L[j]  = coefficient_L1 * dL1 + coefficient_L2 *dL2 + coefficient_L5 * dL5;	
					pElevation[j]    = it->second.Elevation;
					pAzimuth[j]      = it->second.Azimuth;
					if(P1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && P2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && P5.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						pEditedflag_code[j] = TYPE_EDITEDMARK_NORMAL; 
					else
					{
						pEditedflag_code[j] = TYPE_EDITEDMARK_OUTLIER;
					}
					if(L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && L5.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						pEditedflag[j] = 0; // 正常
					else if(L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP || L2.byEditedMark1 == TYPE_EDITEDMARK_SLIP || L5.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
					{
						pEditedflag[j] = 1; // 新的周跳标记				
					}
					else
					{
						pEditedflag[j] = 2; // 异常点
					}
					j++;
				}
				size_t k   = 0;             // 记录新弧段起始点				
				size_t k_i = k;             // 记录新弧段终止点
				while(1)
				{
					if(k_i + 1 >= nCount)   // k_i 为时间序列终点
						goto newArc;
					else
					{
						// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
						if(pEditedflag[k_i + 1] != 1)
						{
							k_i++;
							continue;
						}						
						else // k_i + 1 为新弧段的起点
							goto newArc;
					}
					newArc:  // 本弧段[k，k_i]数据处理 
					{
						QERESIDUAL_ARC   QEresidual_arcP1;
						QERESIDUAL_ARC   QEresidual_arcP2; 
						QERESIDUAL_ARC   QEresidual_arcP5; 
						QERESIDUAL_ARC   QEresidual_arcL; 
						vector<size_t>   listNormalPoint;                        // 记录正常数据
						listNormalPoint.clear();
						for( size_t s_k = k; s_k <= k_i; s_k++ )
						{
							if((pEditedflag[s_k] == 0 || pEditedflag[s_k] == 1)
							&&  pEditedflag_code[s_k] == TYPE_EDITEDMARK_NORMAL) // 正常数据标记 1
								listNormalPoint.push_back(s_k);     
						}
						size_t nArcPointsNumber = listNormalPoint.size();      // 正常数据点个数
						if((int)nArcPointsNumber > min_normalpoints)
						{
							//// 计算均值和残差
							double* pX  = new double [nArcPointsNumber];
							double* pW  = new double [nArcPointsNumber];
							double mean = 0;
							double var  = 0;							
							// 计算P1码的多路径						
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)	
							{
								pX[s_k] = pMultipath_P1[listNormalPoint[s_k]];
								mean = mean + pX[s_k];							
							}
							mean = mean/nArcPointsNumber;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)								
								var = var + pow(pX[s_k] - mean, 2);								
							var = sqrt(var/(nArcPointsNumber - 1));
							QEresidual_arcP1.rms_arc = var;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)
							{
								QERESIDUAL     P1;
								P1.t           = t0 + pObsTime[listNormalPoint[s_k]];
								P1.Elevation   = pElevation[listNormalPoint[s_k]];
								P1.Azimuth     = pAzimuth[listNormalPoint[s_k]];							
								P1.res         = pX[s_k] - mean;	
								P1.id_elevation_Inter = m_QEInfo.getInterval(P1.Elevation);
								QEresidual_arcP1.resList.push_back(P1);
								if(editedObsSatlist[s_i].Id > 5)
								{//不统计GEO卫星
									m_QEInfo.satInfolist_ele[P1.id_elevation_Inter].epochs_P ++;
									m_QEInfo.satInfolist_ele[P1.id_elevation_Inter].rms_P1 += P1.res * P1.res;
								}
								//if(P1.id_elevation_Inter == 0)
								//	fprintf(pfile_e,"%2d  %s\n",editedObsSatlist[s_i].Id,P1.t.toString().c_str());
								
							}						
							// 计算P2码的多路径
							mean = 0;
							var  = 0;						
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)	
							{
								pX[s_k] = pMultipath_P2[listNormalPoint[s_k]];
								mean = mean + pX[s_k];
							}
							mean = mean/nArcPointsNumber;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)														
								var = var + pow(pX[s_k] - mean, 2);								
							var = sqrt(var/(nArcPointsNumber - 1));
							QEresidual_arcP2.rms_arc = var;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)
							{
								QERESIDUAL     P2;
								P2.t           = t0 + pObsTime[listNormalPoint[s_k]];
								P2.Elevation   = pElevation[listNormalPoint[s_k]];
								P2.Azimuth     = pAzimuth[listNormalPoint[s_k]];
								P2.res         = pX[s_k] - mean;		
								P2.id_elevation_Inter = m_QEInfo.getInterval(P2.Elevation);
								QEresidual_arcP2.resList.push_back(P2);
								if(editedObsSatlist[s_i].Id > 5)
								{//不统计GEO卫星
									m_QEInfo.satInfolist_ele[P2.id_elevation_Inter].rms_P2 += P2.res * P2.res;
								}
								
							}	
							// 计算P5码的多路径
							mean = 0;
							var  = 0;						
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)	
							{
								pX[s_k] = pMultipath_P5[listNormalPoint[s_k]];
								mean = mean + pX[s_k];
							}
							mean = mean/nArcPointsNumber;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)														
								var = var + pow(pX[s_k] - mean, 2);								
							var = sqrt(var/(nArcPointsNumber - 1));
							QEresidual_arcP5.rms_arc = var;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)
							{
								QERESIDUAL     P5;
								P5.t           = t0 + pObsTime[listNormalPoint[s_k]];
								P5.Elevation   = pElevation[listNormalPoint[s_k]];
								P5.Azimuth     = pAzimuth[listNormalPoint[s_k]];
								P5.res         = pX[s_k] - mean;			
								P5.id_elevation_Inter = m_QEInfo.getInterval(P5.Elevation);
								QEresidual_arcP5.resList.push_back(P5);		
								if(editedObsSatlist[s_i].Id > 5)
								{//不统计GEO卫星
									m_QEInfo.satInfolist_ele[P5.id_elevation_Inter].rms_P5 += P5.res * P5.res;
								}
							}
							QEresidual_satP1.arcList.push_back(QEresidual_arcP1);						
							QEresidual_satP2.arcList.push_back(QEresidual_arcP2);	
							QEresidual_satP5.arcList.push_back(QEresidual_arcP5);
							delete pX;
							delete pW;
						}
						// 计算相位的多路径							
						listNormalPoint.clear();
						for( size_t s_k = k; s_k <= k_i; s_k++ )							
							if(pEditedflag[s_k] == 0 || pEditedflag[s_k] == 1) // 正常数据标记 1
								listNormalPoint.push_back(s_k); 
						nArcPointsNumber = listNormalPoint.size();      // 正常数据点个数
						if((int)nArcPointsNumber > min_normalpoints)
						{
							double* pX  = new double [nArcPointsNumber];
							double* pW  = new double [nArcPointsNumber];
							double mean = 0;
							double var  = 0;						
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)	
							{
								pX[s_k] = pMultipath_L[listNormalPoint[s_k]];
								mean = mean + pX[s_k];
							}
							mean = mean/nArcPointsNumber;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)															
								var = var + pow(pX[s_k] - mean, 2);							
							var = sqrt(var/(nArcPointsNumber - 1));
							QEresidual_arcL.rms_arc = var;
							for(size_t s_k = 0; s_k < nArcPointsNumber; s_k++)
							{
								QERESIDUAL     L;
								L.t           = t0 + pObsTime[listNormalPoint[s_k]];
								L.Elevation   = pElevation[listNormalPoint[s_k]];
								L.Azimuth     = pAzimuth[listNormalPoint[s_k]];
								L.res         = pX[s_k] - mean;					
								L.id_elevation_Inter = m_QEInfo.getInterval(L.Elevation);
								QEresidual_arcL.resList.push_back(L);	
								if(editedObsSatlist[s_i].Id > 5)
								{//不统计GEO卫星
									m_QEInfo.satInfolist_ele[L.id_elevation_Inter].epochs_L ++;
									m_QEInfo.satInfolist_ele[L.id_elevation_Inter].rms_L += L.res * L.res;
								}
							}
							QEresidual_satL.arcList.push_back(QEresidual_arcL);	
							delete pX;
							delete pW;

						}
						if(k_i + 1 >= nCount) // k_i为时间序列终点, 跳出
							break;
						else  
						{   
							k   = k_i + 1;    // 新弧段的起点设置
							k_i = k;
							continue;
						}
					}
				}
				//统计每颗卫星伪码多径的均方根
				if(QEresidual_satP1.arcList.size() > 0)
				{					
					// 统计第一个频点的伪码多径误差
					QEresidual_satP1.rms_sat = 0;
					int satObsNum = 0; //每一颗卫星使用的观测数据个数
					for(size_t s_n = 0; s_n < QEresidual_satP1.arcList.size(); s_n++)
					{
						QEresidual_satP1.rms_sat += pow(QEresidual_satP1.arcList[s_n].rms_arc, 2) * (QEresidual_satP1.arcList[s_n].resList.size()-1);
						satObsNum = satObsNum + (int)QEresidual_satP1.arcList[s_n].resList.size();
					}
					QEresidual_satP1.rms_sat = sqrt(QEresidual_satP1.rms_sat / (satObsNum - 1));
					QEresidual_satP1.epochs  = satObsNum;
					// 统计第二个频点的伪码多径误差
					QEresidual_satP2.rms_sat = 0;
					satObsNum = 0;
					for(size_t s_n = 0; s_n < QEresidual_satP2.arcList.size(); s_n++)
					{
						QEresidual_satP2.rms_sat  += pow(QEresidual_satP2.arcList[s_n].rms_arc, 2) * (QEresidual_satP2.arcList[s_n].resList.size()-1);
						satObsNum = satObsNum + (int)QEresidual_satP2.arcList[s_n].resList.size();
					}
					QEresidual_satP2.rms_sat = sqrt(QEresidual_satP2.rms_sat / (satObsNum - 1));

					// 统计第三个频点的伪码多径误差
					QEresidual_satP5.rms_sat = 0;
					satObsNum = 0;
					for(size_t s_n = 0; s_n < QEresidual_satP5.arcList.size(); s_n++)
					{
						QEresidual_satP5.rms_sat  += pow(QEresidual_satP5.arcList[s_n].rms_arc, 2) * (QEresidual_satP5.arcList[s_n].resList.size()-1);
						satObsNum = satObsNum + (int)QEresidual_satP5.arcList[s_n].resList.size();
					}
					QEresidual_satP5.rms_sat = sqrt(QEresidual_satP5.rms_sat / (satObsNum - 1));
					//统计每颗卫星相位多径的均方根				
					QEresidual_satL.rms_sat = 0;
					satObsNum = 0; //每一颗卫星使用的观测数据个数
					for(size_t s_n = 0; s_n < QEresidual_satL.arcList.size(); s_n++)
					{
						QEresidual_satL.rms_sat += pow(QEresidual_satL.arcList[s_n].rms_arc, 2) * (QEresidual_satL.arcList[s_n].resList.size()-1);
						satObsNum = satObsNum + (int)QEresidual_satL.arcList[s_n].resList.size();
					}
					QEresidual_satL.rms_sat = sqrt(QEresidual_satL.rms_sat / (satObsNum - 1));

					QEresidual_satP1.id_sat = editedObsSatlist[s_i].Id;
					QEresidual_satP2.id_sat = editedObsSatlist[s_i].Id;
					QEresidual_satP5.id_sat = editedObsSatlist[s_i].Id;
					QEresidual_satL.id_sat  = editedObsSatlist[s_i].Id;
					m_QEInfo.satInfoList_P1.push_back(QEresidual_satP1);
					m_QEInfo.satInfoList_P2.push_back(QEresidual_satP2);
					m_QEInfo.satInfoList_P5.push_back(QEresidual_satP5);
					m_QEInfo.satInfoList_L.push_back(QEresidual_satL);					
				}			

				delete pObsTime;				
				delete pMultipath_P1;
				delete pMultipath_P2;
				delete pMultipath_P5;
				delete pElevation;
				delete pAzimuth;
				delete pEditedflag;
			}
			//fclose(pfile_e);
			// 统计异常信息
			for(int i = 0 ; i < 40; i++)
			{
				if(statistic[i] > 0)
				{
					if(i >= 30)
						statistic[i] = statistic[i]/3;
					m_QEInfo.AbnormalObsCount.insert(QEAbNormalObsCountMap::value_type(i,statistic[i]));
				}
			}
			m_QEInfo.ratio_P_normal = m_nCodeCount / (double)m_nObsCount;			
			m_QEInfo.ratio_L_normal = m_nPhaseCount / (double)m_nObsCount;
			m_QEInfo.ratio_SLip     = m_nCycleSlip * 3 / (double)m_nObsCount;
			////// 统计第一个频点的测站伪距多径均方根
			m_QEInfo.rms_P1 = 0;
			int   staObsNum = 0; //每一个测站使用的观测数据个数
			for(size_t s_n = 0; s_n < m_QEInfo.satInfoList_P1.size(); s_n++)
			{
				int satObsNum = 0;
				for(size_t s_m = 0; s_m < m_QEInfo.satInfoList_P1[s_n].arcList.size(); s_m++)
					satObsNum = satObsNum + (int)m_QEInfo.satInfoList_P1[s_n].arcList[s_m].resList.size();
				m_QEInfo.rms_P1 += pow(m_QEInfo.satInfoList_P1[s_n].rms_sat, 2) * (satObsNum - 1);
				staObsNum = staObsNum + satObsNum;
			}
			m_QEInfo.rms_P1 = sqrt(m_QEInfo.rms_P1 / (staObsNum - 1));
			// 统计第二个频点的测站伪距多径均方根
			m_QEInfo.rms_P2 = 0;
			staObsNum = 0;
			for(size_t s_n = 0; s_n < m_QEInfo.satInfoList_P2.size(); s_n++)
			{
				int satObsNum = 0;
				for(size_t s_m = 0; s_m < m_QEInfo.satInfoList_P2[s_n].arcList.size(); s_m++)
					satObsNum = satObsNum + (int)m_QEInfo.satInfoList_P2[s_n].arcList[s_m].resList.size();
				m_QEInfo.rms_P2 += pow(m_QEInfo.satInfoList_P2[s_n].rms_sat, 2) * (satObsNum - 1);
				staObsNum = staObsNum + satObsNum;
			}
			m_QEInfo.rms_P2 = sqrt(m_QEInfo.rms_P2 / (staObsNum - 1));
			// 统计第三个频点的测站伪距多径均方根
			m_QEInfo.rms_P5 = 0;
			staObsNum = 0;
			for(size_t s_n = 0; s_n < m_QEInfo.satInfoList_P5.size(); s_n++)
			{
				int satObsNum = 0;
				for(size_t s_m = 0; s_m < m_QEInfo.satInfoList_P5[s_n].arcList.size(); s_m++)
					satObsNum = satObsNum + (int)m_QEInfo.satInfoList_P5[s_n].arcList[s_m].resList.size();
				m_QEInfo.rms_P5 += pow(m_QEInfo.satInfoList_P5[s_n].rms_sat, 2) * (satObsNum - 1);
				staObsNum = staObsNum + satObsNum;
			}
			m_QEInfo.rms_P5 = sqrt(m_QEInfo.rms_P5 / (staObsNum - 1));
			// 统计测站的相位多径均方根
			m_QEInfo.rms_L = 0;
			staObsNum = 0;
			for(size_t s_n = 0; s_n < m_QEInfo.satInfoList_L.size(); s_n++)
			{
				int satObsNum = 0;
				for(size_t s_m = 0; s_m < m_QEInfo.satInfoList_L[s_n].arcList.size(); s_m++)
					satObsNum = satObsNum + (int)m_QEInfo.satInfoList_L[s_n].arcList[s_m].resList.size();
				m_QEInfo.rms_L += pow(m_QEInfo.satInfoList_L[s_n].rms_sat, 2) * (satObsNum - 1);
				staObsNum = staObsNum + satObsNum;
			}
			m_QEInfo.rms_L = sqrt(m_QEInfo.rms_L / (staObsNum - 1));
			// 统计测站的观测数据质量与仰角的关系
			for(size_t s_i = 0; s_i < m_QEInfo.satInfolist_ele.size(); s_i ++)
			{
				int obs_P = m_QEInfo.satInfolist_ele[s_i].epochs_P;
				int obs_L = m_QEInfo.satInfolist_ele[s_i].epochs_L;
				if(obs_P> 1)
				{
					m_QEInfo.satInfolist_ele[s_i].rms_P1 = sqrt(m_QEInfo.satInfolist_ele[s_i].rms_P1 / (obs_P - 1));
					m_QEInfo.satInfolist_ele[s_i].rms_P2 = sqrt(m_QEInfo.satInfolist_ele[s_i].rms_P2 / (obs_P - 1));
					m_QEInfo.satInfolist_ele[s_i].rms_P5 = sqrt(m_QEInfo.satInfolist_ele[s_i].rms_P5 / (obs_P - 1));
				}
				if(obs_L> 1)
					m_QEInfo.satInfolist_ele[s_i].rms_L = sqrt(m_QEInfo.satInfolist_ele[s_i].rms_L / (obs_L - 1));
			}
			//int nArcCount = 0;
			//FILE *pfile = fopen("C:\\residuls_P1.cpp","w+");
			//for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P1.size(); s_i++)
			//{
			//	for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_P1[s_i].arcList.size(); s_j++)
			//	{
			//		if(m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList.size() != 0)
			//		{
			//			nArcCount++;
			//			for(size_t s_k = 0; s_k < m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList.size(); s_k++)										
			//				fprintf(pfile,"%s %2d %2d %8.4f %8.4f %10.6lf\n",m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].t.toString().c_str(),
			//											 m_QEInfo.satInfoList_P1[s_i].id_sat,
			//									 		 nArcCount,
			//											 m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].Elevation,
			//											 m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].Azimuth,
			//											 m_QEInfo.satInfoList_P1[s_i].arcList[s_j].resList[s_k].res);
			//		}
			//	}

			//}
			//fclose(pfile);//
			//nArcCount = 0;
			//FILE *pfile1 = fopen("C:\\residuls_P2.cpp","w+");
			//for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P2.size(); s_i++)
			//{
			//	for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_P2[s_i].arcList.size(); s_j++)
			//	{
			//		if(m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList.size() != 0)
			//		{
			//			nArcCount++;
			//			for(size_t s_k = 0; s_k < m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList.size(); s_k++)										
			//				fprintf(pfile1,"%s %2d %2d %8.4f %8.4f %10.6lf\n",m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].t.toString().c_str(),
			//											 m_QEInfo.satInfoList_P2[s_i].id_sat,
			//									 		 nArcCount,
			//											 m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].Elevation,
			//											 m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].Azimuth,
			//											 m_QEInfo.satInfoList_P2[s_i].arcList[s_j].resList[s_k].res);
			//		}
			//	}

			//}
			//fclose(pfile1);//
			//nArcCount = 0;
			//FILE *pfile2 = fopen("C:\\residuls_P5.cpp","w+");
			//for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_P5.size(); s_i++)
			//{
			//	for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_P5[s_i].arcList.size(); s_j++)
			//	{
			//		if(m_QEInfo.satInfoList_P5[s_i].arcList[s_j].resList.size() != 0)
			//		{
			//			nArcCount++;
			//			for(size_t s_k = 0; s_k < m_QEInfo.satInfoList_P5[s_i].arcList[s_j].resList.size(); s_k++)										
			//				fprintf(pfile2,"%s %2d %2d %8.4f %8.4f %10.6lf\n",m_QEInfo.satInfoList_P5[s_i].arcList[s_j].resList[s_k].t.toString().c_str(),
			//											 m_QEInfo.satInfoList_P5[s_i].id_sat,
			//									 		 nArcCount,
			//											 m_QEInfo.satInfoList_P5[s_i].arcList[s_j].resList[s_k].Elevation,
			//											 m_QEInfo.satInfoList_P5[s_i].arcList[s_j].resList[s_k].Azimuth,
			//											 m_QEInfo.satInfoList_P5[s_i].arcList[s_j].resList[s_k].res);
			//		}
			//	}

			//}
			//fclose(pfile2);//
			//nArcCount = 0;
			//FILE *pfile3 = fopen("C:\\residuls_L.cpp","w+");
			//for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_L.size(); s_i++)
			//{
			//	for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_L[s_i].arcList.size(); s_j++)
			//	{
			//		if(m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList.size() != 0)
			//		{
			//			nArcCount++;
			//			for(size_t s_k = 0; s_k < m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList.size(); s_k++)										
			//				fprintf(pfile3,"%s %2d %2d %8.4f %8.4f %10.6lf\n",m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].t.toString().c_str(),
			//											 m_QEInfo.satInfoList_L[s_i].id_sat,
			//									 		 nArcCount,
			//											 m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Elevation,
			//											 m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Azimuth,
			//											 m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].res);
			//		}
			//	}

			//}
			//fclose(pfile3);//
			return true;
		}
		 //   子程序名称： evaluate_thrfrephase_multipath   
		//   作用：评估相位观测数据的质量, [利用三频相位数据组合]
		//   类型：editedObsSatlist  : 数据结构, 根据不同GPS卫星进行分类, 主要用途方便时间序列处理
		//         index_L1          :  L1位置索引
		//         index_L2          :  L2位置索引
		//         index_L5          :  L5位置索引
		//   输入：editedObsSatlist, index_L1, index_L2, index_L5
		//   输出：
		//   其它：
		//   语言： C++
		//   版本号：2013/4/1
		//   生成者：刘俊宏
		//   修改者：
		//bool BDObsQualityEvaluate::evaluate_thrfrephase_multipath(vector<Rinex2_1_EditedObsSat> editedObsSatlist, int index_L1, int index_L2, int index_L5)
		//{
		//	int nArcCount = 0;                       // 记录弧段	
		//	double coefficient_L1 = 1 / (1 - pow( BD_FREQUENCE_L2 / BD_FREQUENCE_L1, 2 )) -  
		//		                    1 / (1 - pow( BD_FREQUENCE_L5 / BD_FREQUENCE_L1, 2 ));
		//	double coefficient_L2 = - 1 / (pow( BD_FREQUENCE_L1 / BD_FREQUENCE_L2, 2 ) - 1);
		//	double coefficient_L5 = 1 / (pow( BD_FREQUENCE_L1 / BD_FREQUENCE_L5, 2 ) - 1);
		//	for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
		//	{
		//		QERESIDUAL_SAT QEresidual_sat;
		//		size_t nCount = editedObsSatlist[s_i].editedObs.size();   // 观测时间序列数据个数				
		//	    double *pObsTime         = new double[nCount];            // 相对时间序列
		//		double *phase_multipath  = new double[nCount];            // 相位多径		
		//		double *pElevation       = new double[nCount];           
		//		double *pAzimuth         = new double[nCount]; 
		//		int    *pEditedflag      = new int   [nCount];            // 编辑标记序列 0--正常    1--新的周跳标记   2--异常点
		//		Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSatlist[s_i].editedObs.begin();
		//		DayTime t0 = it0->first; // 初始时间 - 用于计算相对时间(秒)
		//		int j = 0;
		//		for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
		//		{
		//			pObsTime[j] =  it->first - t0;
		//			Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[index_L1];
		//			Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[index_L2];
		//			Rinex2_1_EditedObsDatum L5 = it->second.obsTypeList[index_L5];					
		//			phase_multipath[j] =  coefficient_L1 * L1.obs.data * BD_WAVELENGTH_L1
		//			                + coefficient_L2 * L2.obs.data * BD_WAVELENGTH_L2
		//							+ coefficient_L5 * L5.obs.data * BD_WAVELENGTH_L5;						
		//			pElevation[j]  = it->second.Elevation;
		//			pAzimuth[j]    = it->second.Azimuth;
		//			if(L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL  && L5.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
		//				pEditedflag[j] = 0; // 正常
		//			else if(L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP || L2.byEditedMark1 == TYPE_EDITEDMARK_SLIP || L5.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
		//			{
		//				pEditedflag[j] = 1; // 新的周跳标记
		//			}
		//			else
		//			{
		//				pEditedflag[j] = 2; // 异常点
		//			}
		//			j++;
		//		}

		//		size_t k   = 0; // 记录新弧段起始点				
		//		size_t k_i = k; // 记录新弧段终止点				
		//		while(1)
		//		{
		//			if(k_i + 1 >= nCount) // k_i 为时间序列终点
		//				goto newArc;
		//			else
		//			{
		//				// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
		//				if(pEditedflag[k_i + 1] != 1)
		//				{
		//					k_i++;
		//					continue;
		//				}						
		//				else // k_i + 1 为新弧段的起点
		//					goto newArc;
		//			}
		//			newArc:  // 本弧段[k，k_i]数据处理 
		//			{
		//				FILE *pfile = fopen("C:\\residuls_L_multipath.cpp","a+");						
		//				QERESIDUAL_ARC   QEresidual_arc; 
		//				vector<size_t>    unknownPointlist;		
		//				// 第一步：根据电离层残差的阈值大小，直接进行野值判断，剔除一些大的野值
		//				for(size_t s_i = 0; s_i <= nCount; s_i++)													
		//					if(fabs(pThrFreCodeCom[s_i]) > m_PreprocessorDefine.max_thrfrecodecom) 																						
		//						pOutlier[s_i] = OBSPREPROC_OUTLIER_IONOMAXMIN; //电离层超差，直接标记为野值
		//					else
		//					{
		//						if(pOutlier[s_i] == OBSPREPROC_NORMAL)
		//							unknownPointlist.push_back(s_i);
		//					}
		//				size_t nCount_points = unknownPointlist.size(); 

		//				int nArcPointsCount = int(k_i - k + 1);	
		//				int nNormPointsCount = 0;

		//				for (size_t s_j = k ; s_j <= k_i; s_j++)
		//				{
		//					if(pEditedflag[s_j] == 2)
		//					{
		//						w[s_j - k] = 0;								
		//					}
		//					else
		//					{
		//						w[s_j - k] = 1.0;
		//						nNormPointsCount++;
		//					}
		//				}
		//				if(nNormPointsCount > 20)
		//				{
		//					nArcCount++;
		//					KinematicRobustVandrakFilter(pObsTime + k , ionoL1_L2 + k, w, nArcPointsCount,
		//										   vondrak_L1_L2_eps,
		//										   vondrak_fit + k,
		//										   vondrak_L1_L2_max,
		//										   vondrak_L1_L2_min,
		//										   vondrak_L1_L2_width);//
		//					for(size_t s_k = k; s_k <= k_i; s_k++)
		//					{
		//						if(pEditedflag[s_k] != 2)
		//						{
		//							// 记录拟合残差
		//							QERESIDUAL             QEresidual;
		//							QEresidual.t           = t0 + pObsTime[s_k];								
		//							QEresidual.Elevation   = pElevation[s_k];
		//							QEresidual.Azimuth     = pAzimuth[s_k];
		//							QEresidual.res         = vondrak_fit[s_k] - ionoL1_L2[s_k];							
		//							QEresidual_arc.resList.push_back(QEresidual);
		//						}
		//						if(pEditedflag[s_k] != 2)
		//							fprintf(pfile,"%s %2d %2d %8.4f %8.4f %14.6lf %14.6lf %14.6lf %4.2f\n",
		//								   (t0 + pObsTime[s_k]).toString().c_str(),
		//									editedObsSatlist[s_i].Id,
		//									nArcCount,
		//									pElevation[s_k],
		//									pAzimuth[s_k],
		//									ionoL1_L2[s_k],
		//									vondrak_fit[s_k],
		//									vondrak_fit[s_k] - ionoL1_L2[s_k],
		//									w[s_k - k]);//
		//					}
		//					fclose(pfile);
		//					//统计每一弧段的均方根误差
		//					QEresidual_arc.rms_arc = 0;
		//					for(size_t s_n = 0; s_n < QEresidual_arc.resList.size(); s_n++)
		//						QEresidual_arc.rms_arc += pow(QEresidual_arc.resList[s_n].res, 2);				
		//					QEresidual_arc.rms_arc = sqrt(QEresidual_arc.rms_arc / (QEresidual_arc.resList.size() - 1));
		//					QEresidual_sat.arcList.push_back(QEresidual_arc);						
		//					delete w;	
		//				}
		//				if(k_i + 1 >= nCount) // k_i为时间序列终点, 跳出
		//					break;
		//				else  
		//				{   
		//					k   = k_i + 1;    // 新弧段的起点设置
		//					k_i = k;
		//					continue;
		//				}
		//			}
		//		}
		//		if(QEresidual_sat.arcList.size() > 0)
		//		{
		//			//统计每一卫星的均方根误差
		//			QEresidual_sat.rms_sat = 0;
		//			int satObsNum = 0; //每一颗卫星使用的观测数据个数				
		//			for(size_t s_n = 0; s_n < QEresidual_sat.arcList.size(); s_n++)
		//			{
		//				QEresidual_sat.rms_sat += pow(QEresidual_sat.arcList[s_n].rms_arc, 2) * (QEresidual_sat.arcList[s_n].resList.size() - 1);
		//				satObsNum = satObsNum + (int)QEresidual_sat.arcList[s_n].resList.size();
		//			}
		//			QEresidual_sat.rms_sat = sqrt(QEresidual_sat.rms_sat / (satObsNum - 1));
		//			QEresidual_sat.id_sat = editedObsSatlist[s_i].Id;
		//			m_QEInfo.satInfoList_L.push_back(QEresidual_sat);
		//		}
		//		delete pObsTime;
		//		delete ionoL1_L2;
		//		delete pElevation;
		//		delete pAzimuth;
		//		delete pEditedflag;
		//		delete vondrak_fit;
		//	}
		//				
		//	// 统计测站的均方根
		//	m_QEInfo.rms_L = 0;			
		//	int   staObsNum = 0; //每一个测站使用的观测数据个数	
		//	for(size_t s_n = 0; s_n < m_QEInfo.satInfoList_L.size(); s_n++)
		//	{
		//		int satObsNum = 0;
		//		for(size_t s_m = 0; s_m < m_QEInfo.satInfoList_L[s_n].arcList.size(); s_m++)
		//			satObsNum = satObsNum + (int)m_QEInfo.satInfoList_L[s_n].arcList[s_m].resList.size();
		//		m_QEInfo.rms_L += pow(m_QEInfo.satInfoList_L[s_n].rms_sat, 2) * (satObsNum - 1);
		//		staObsNum = staObsNum + satObsNum;
		//	}
		//	m_QEInfo.rms_L = sqrt(m_QEInfo.rms_L / (staObsNum - 1));
		//	   
		//	//nArcCount = 0;
		//	//FILE *pfile = fopen("C:\\phase_residuls_L.cpp","w+");
		//	//for(size_t s_i = 0; s_i < m_QEInfo.satInfoList_L.size(); s_i++)
		//	//{
		//	//	for(size_t s_j = 0; s_j < m_QEInfo.satInfoList_L[s_i].arcList.size(); s_j++)
		//	//	{
		//	//		if(m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList.size() != 0)
		//	//		{
		//	//			nArcCount++;
		//	//			for(size_t s_k = 0; s_k < m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList.size(); s_k++)										
		//	//				fprintf(pfile,"%s %2d %2d %8.4f %8.4f %10.6lf\n",m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].t.toString().c_str(),
		//	//											 m_QEInfo.satInfoList_L[s_i].id_sat,
		//	//									 		 nArcCount,
		//	//											 m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Azimuth,
		//	//											 m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].Elevation,
		//	//											 m_QEInfo.satInfoList_L[s_i].arcList[s_j].resList[s_k].res);
		//	//		}
		//	//	}

		//	//}
		//	//fclose(pfile);//

		//	return true;
		//}

		// 子程序名称： statBDSatCount   
		// 功能：统计最大可卫星个数和平均可视卫星个数
		// 变量类型：max_count    最大可视卫星个数
		//           mean_count   平均可视卫星个数
		// 输入：
		// 输出：max_count，mean_count
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/10/05
		// 版本时间：2007/10/05
		// 修改记录：
		// 备注：
		bool BDObsQualityEvaluate::statBDSatCount(Rinex2_1_EditedObsFile m_editedObsFile, int &max_count, double &mean_count)
		{
			max_count = 0; mean_count = 0;
			int sum_k = 0;
			int k_max = 0;
			for(size_t s_i = 0; s_i < m_editedObsFile.m_data.size(); s_i++)
			{
				if(k_max < int(m_editedObsFile.m_data[s_i].editedObs.size()))
					k_max = int(m_editedObsFile.m_data[s_i].editedObs.size());
				sum_k += int(m_editedObsFile.m_data[s_i].editedObs.size());
			}
			max_count  = k_max;
			mean_count = sum_k * 1.0 / m_editedObsFile.m_data.size();
			return true;
		}

	}
}