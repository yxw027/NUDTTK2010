#include "BDObsPreproc.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace BDPreproc
	{
		BDObsPreproc::BDObsPreproc(void)
		{
		}

		BDObsPreproc::~BDObsPreproc(void)
		{
		}

		void BDObsPreproc::setObsFile(Rinex2_1_ObsFile obsFile)
		{
			m_obsFile = obsFile;
		}

		void BDObsPreproc::setStationPosition(POS3D pos)
		{
			m_posStation = pos;
		}		
		bool BDObsPreproc::loadObsFile(string  strObsfileName)
		{
			return m_obsFile.open(strObsfileName);
		}
		bool BDObsPreproc::loadNavFile(string  strNavfileName)
		{
			return m_navFile.open(strNavfileName);
		}
		bool BDObsPreproc::loadSp3File(string  strSp3fileName)
		{
			return m_sp3File.open(strSp3fileName);
		}
		bool BDObsPreproc::loadClkFile(string  strClkfileName)
		{
			return m_clkFile.open(strClkfileName);
		}

		BYTE BDObsPreproc::obsPreprocInfo2EditedMark1(int obsPreprocInfo)
		{
			return BYTE(getIntBit(obsPreprocInfo, 1));
		}

		BYTE BDObsPreproc::obsPreprocInfo2EditedMark2(int obsPreprocInfo)
		{
			return BYTE(getIntBit(obsPreprocInfo, 0));
		}

		// 子程序名称： datalist_epoch2sat   
		// 作用：将 editedObsEpochlist 转换到 editedObsSatlist
		// 类型：editedObsEpochlist  : 数据结构格式2, 根据不同历元时刻进行分类, 称为editedObsEpochlist, 主要用途方便单点定位
		//         editedObsSatlist    : 数据结构格式1, 根据不同卫星(或测站)进行分类 , 称为editedObsSatlist, 主要用途方便时间序列处理
		// 输入：editedObsEpochlist
		// 输出：editedObsSatlist 
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/5/8
		// 版本时间：2012/4/11
		// 修改记录： 
		// 其它：
		bool BDObsPreproc::datalist_epoch2sat(vector<Rinex2_1_EditedObsEpoch> &editedObsEpochlist, vector<Rinex2_1_EditedObsSat>& editedObsSatlist)
		{
			Rinex2_1_EditedObsSat editedObsSatlist_max[MAX_PRN]; 
			for(int i = 0; i < MAX_PRN; i++)
			{
				editedObsSatlist_max[i].Id = i;
				editedObsSatlist_max[i].editedObs.clear();
			}
			// 遍历每个历元的观测数据   /* 耗时7秒左右 */
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				// 在每个历元，遍历每颗GPS卫星的数据
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					BYTE Id = it->first;
					editedObsSatlist_max[Id].editedObs.insert(Rinex2_1_EditedObsEpochMap::value_type(editedObsEpochlist[s_i].t, it->second));
				}
			}
			// 整理所有可视卫星数据
			int validcount = 0;
			for(int i = 0; i < MAX_PRN; i++)
			{
				if(editedObsSatlist_max[i].editedObs.size() > 0)
				{
					validcount++;
				}
			}
			// 利用 resize 提高效率
			editedObsSatlist.clear();
			editedObsSatlist.resize(validcount);
			validcount = 0;
			for(int i = 0; i < MAX_PRN; i++)
			{
				if(editedObsSatlist_max[i].editedObs.size() > 0)
				{
					editedObsSatlist[validcount] = editedObsSatlist_max[i];
					validcount++;
				}
			}
			//// 获取观测数据在测站列表中的时间序号
			//for(size_t s_i = 0;s_i < editedObsSatlist.size();s_i++)
			//{
			//	int obs_i = 0;
			//	for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
			//	{
			//		it->second.nObsTime = obs_i;
			//		obs_i++;
			//	}
			//}
			return true;
		}

		// 子程序名称：datalist_sat2epoch   
		// 作用：将 editedObsSatlist 转换到 editedObsEpochlist, 为了保持时间的完整性, 
		//         转换期间以 m_obsFile.m_data[s_i].T 为参考
		// 类型：editedObsSatlist   : 数据结构格式1: 根据不同卫星(或测站)进行分类 , 称为editedObsSatlist  , 主要用途方便时间序列处理
		//         editedObsEpochlist : 数据结构格式2: 根据不同历元时刻进行分类, 称为editedObsEpochlist, 主要用途方便单点定位
		// 输入：editedObsSatlist
		// 输出：editedObsEpochlist 		
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/5/8
		// 版本时间：2012/4/11
		// 修改记录：
		// 其它：依赖 m_obsFile
		bool BDObsPreproc::datalist_sat2epoch(vector<Rinex2_1_EditedObsSat> &editedObsSatlist, vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist)
		{
			if(editedObsSatlist.size() <= 0)
				return false;
			editedObsEpochlist.clear();
			//editedObsEpochlist.resize(m_obsFile.m_data.size());
			for(size_t s_i = 0; s_i < m_obsFile.m_data.size(); s_i++)
			{
				Rinex2_1_EditedObsEpoch editedObsEpoch;
                editedObsEpoch.byEpochFlag = m_obsFile.m_data[s_i].byEpochFlag;
				editedObsEpoch.bySatCount  = m_obsFile.m_data[s_i].bySatCount;
				editedObsEpoch.t           = m_obsFile.m_data[s_i].t;
				//editedObsEpoch.clock       = m_obsFile.m_data[s_i].
				editedObsEpoch.editedObs.clear();
				// 遍历每颗 BD 卫星的数据列表
				for(size_t s_j = 0; s_j < editedObsSatlist.size(); s_j++)
				{// 判断当前时刻的数据是否符合要求(!前提是预处理期间，时间标签未被改动!)
					Rinex2_1_EditedObsEpochMap::const_iterator it;
					if((it = editedObsSatlist[s_j].editedObs.find(editedObsEpoch.t)) != editedObsSatlist[s_j].editedObs.end())
					{
						editedObsEpoch.editedObs.insert(Rinex2_1_EditedObsSatMap::value_type(editedObsSatlist[s_j].Id, it->second));
					}
				}
			    //editedObsEpochlist[s_i] = editedObsEpoch;
				if(editedObsEpoch.editedObs.size() > 0)
					editedObsEpochlist.push_back(editedObsEpoch);
			}
			return true;
		}
		// 子程序名称： getEditedObsEpochList   
		// 作用：从观测数据文件 m_obsFile 导出预处理数据结构 editedObsEpochlist
		// 类型：editedObsEpochlist : 数据结构格式2: 根据不同历元时刻进行分类，称为editedObsEpochlist, 主要用途方便单点定位
		// 输入：
		// 输出：editedObsEpochlist 
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/5/8
		// 版本时间：2012/4/11
		// 修改记录：
		// 其它：
		bool BDObsPreproc::getEditedObsEpochList(vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist)
		{
			if(m_obsFile.isEmpty())
				return false;
			editedObsEpochlist.clear();
			// resize函数可提高效率
			editedObsEpochlist.resize(m_obsFile.m_data.size());
			/* 转换 CHAMP 一天数据, 耗时 11.70 秒左右 */
			for(size_t s_i = 0; s_i < m_obsFile.m_data.size(); s_i++)
			{
				Rinex2_1_EditedObsEpoch editedObsEpoch;
				editedObsEpoch.load(m_obsFile.m_data[s_i]);
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpoch.editedObs.begin(); it != editedObsEpoch.editedObs.end(); ++it)
				{
					it->second.nObsTime = int(s_i);
				}
				editedObsEpochlist[s_i] = editedObsEpoch;
			}
			return true;
		}
		// 子程序名称： getEditedObsSatList   
		// 作用：从观测数据文件 m_obsFile 载入预处理数据 editedObsSatlist
		// 类型：editedObsSatlist : 数据结构格式1: 根据不同卫星(或测站)进行分类, 称为editedObsSatlist, 主要用途方便时间序列处理
		// 输入：
		// 输出：editedObsSatlist 
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2012/5/8
		// 版本时间：2012/4/11
		// 修改记录：
		// 其它：
		bool BDObsPreproc::getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist)
		{
			vector<Rinex2_1_EditedObsEpoch> editedObsEpochlist;
			// 首先根据观测数据文件 m_obsFile 初始化每个时刻的预处理数据
			if(!getEditedObsEpochList(editedObsEpochlist))
				return	false;
			return datalist_epoch2sat(editedObsEpochlist, editedObsSatlist);
		}
		// 子程序名称： detectCodeOutlier_ionosphere   
		// 功能：电离层组合探测伪码野值
		// 变量类型：    index_P1                                  : 观测类型P1索引
		//               index_P2                                  : 观测类型P2索引
		//               frequence_L1                              : L1的频率
		//               frequence_L2                              : L2的频率
		//               editedObsSat                              : 输出的弧段结构数据
		//               bOutTempFile                              : 是否输出预处理信息
		// 输入：index_P1，index_P2，frequence_L1，frequence_L2,editedObsSat,bOutTempFile 
		// 输出：editedObsSat
		// 语言：C++
		// 创建者：刘俊宏,谷德峰
		// 创建时间：2012/4/13
		// 版本时间：2012/4/13
		// 修改记录：
		// 其它： 
		bool BDObsPreproc::detectCodeOutlier_ionosphere(int index_P1, int index_P2, double frequence_L1, double frequence_L2, Rinex2_1_EditedObsSat& editedObsSat,bool bOutTempFile)
		{		
			FILE *pfile;
			if(bOutTempFile == true)
			{
				// 测试程序，临时写文件				
				char szStationName[4 + 1];
				for(int k = 0; k < 4; k++)
				{
					szStationName[k] = m_obsFile.m_header.szMarkName[k];
				}
				szStationName[4] = '\0';			
				char  IonospherePFileName[200];
				sprintf(IonospherePFileName, "%s\\%s_Ionosphere_P.dat", m_strPreprocFilePath.c_str(), szStationName);
				pfile = fopen(IonospherePFileName,"a+");				
			}

			size_t nCount = editedObsSat.editedObs.size();
			if(nCount <= m_PreprocessorDefine.min_arcpointcount)  // 观测个数太少, 直接丢弃
			{				
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
				{
					//防止重复标记
					if(it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
				}
				return true;
			}
			double *pIonosphere = new double[nCount];
            double *pIonosphere_fit = new double[nCount];			
			int *pOutlier = new int    [nCount];		
			double *pEpochTime = new double [nCount];
			Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSat.editedObs.begin();
			DayTime t0 = it0->first;  
			// 构造伪码电离层组合序列 ionosphere = coefficient_ionosphere * (P1 - P2)
			double coefficient_ionosphere = 1 / (1 - pow( frequence_L1 / frequence_L2, 2 ));
			int i = 0;			
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
			{				
				pEpochTime[i] = it->first - t0;
				Rinex2_1_EditedObsDatum  P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum  P2 = it->second.obsTypeList[index_P2];
				pIonosphere[i] = coefficient_ionosphere * (P1.obs.data - P2.obs.data);				
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
					pOutlier[i] = TYPE_EDITEDMARK_OUTLIER; // 接受先前野值判断结果, 以和其他野值判断方法相配合
				else
					pOutlier[i] = OBSPREPROC_NORMAL;
				i++;
			}
			size_t k   = 0;
			size_t k_i = k;
			 //获得每个连续跟踪弧段的数据
			while(1)
			{
				if(k_i + 1 >= nCount)
					goto newArc;
				else
				{
					// 判断 k_i+1 与 k_i 是否位于同一跟踪弧段
					if(pEpochTime[k_i + 1] - pEpochTime[k_i] <= m_PreprocessorDefine.max_arclengh)
					{
						k_i++;
						continue;
					}
					else // k_i + 1 为新弧段的起点
						goto newArc;
				}
				newArc:  // 本弧段[k，k_i]数据处理 
				{// 弧段内数据个数太少
					int nArcPointsCount = int(k_i - k + 1);
					if(nArcPointsCount <= int(m_PreprocessorDefine.min_arcpointcount))
					{
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{							
							if(pOutlier[s_i] != TYPE_EDITEDMARK_OUTLIER)
								pOutlier[s_i] = OBSPREPROC_OUTLIER_COUNT; // 弧段内数据个数太少标记为野值
						}
					}
					else
					{
						static int nArcCount = 0;
						nArcCount++;
						double *w = new double [nArcPointsCount];
						// 首先根据电离层残差的阈值大小，直接进行野值判断，剔除一些大的野值
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{							
							if(pIonosphere[s_i] <= m_PreprocessorDefine.min_ionosphere 
							|| pIonosphere[s_i] >= m_PreprocessorDefine.max_ionosphere) 
							{
								w[s_i - k] = 0;								
								pOutlier[s_i] = OBSPREPROC_OUTLIER_IONOMAXMIN; //电离层超差，直接标记为野值
							}
							else if(pOutlier[s_i] == TYPE_EDITEDMARK_OUTLIER)
							{
								w[s_i - k] = 0;
							}
							else
							{
								w[s_i - k] = 1.0;
							}
						}						
						KinematicRobustVandrakFilter(pEpochTime + k , pIonosphere + k, w, nArcPointsCount,
							                       m_PreprocessorDefine.vondrak_PIF_eps,
												   pIonosphere_fit + k,
												   m_PreprocessorDefine.vondrak_PIF_max,
												   m_PreprocessorDefine.vondrak_PIF_min,
												   m_PreprocessorDefine.vondrak_PIF_width);
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{							
							if(w[s_i - k] == 0 && pOutlier[s_i] == OBSPREPROC_NORMAL)
							{
								pOutlier[s_i] = OBSPREPROC_OUTLIER_VONDRAK;
							}
						}
						delete w;
					}
					if(k_i + 1 >= nCount)
						break;
					else  
					{
						// 新弧段的起点设置
						k   = k_i + 1;
						k_i = k;
						continue;
					}
				}
			}
			i = 0;
			//FILE *pfile = fopen("C:\\Ionosphere_P1.dat","a+");
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
			{	
				if(it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(pOutlier[i]);
					it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(pOutlier[i]);					
				}
				if(it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(pOutlier[i]);
					it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(pOutlier[i]);					
				}
				if(bOutTempFile == true)
				{
					// 写临时文件
					if(it->second.obsTypeList[index_P1].obs.data != DBL_MAX 
					&& it->second.obsTypeList[index_P2].obs.data != DBL_MAX)
					{
						fprintf(pfile,"%2d %8.2f %8.2f %8.4f %8.4f\n",
							            editedObsSat.Id,
										pEpochTime[i],
										it->second.Elevation,
										pIonosphere[i],
										pIonosphere_fit[i]);
					}
				}
				i++;
			}
			if(bOutTempFile == true)
				fclose(pfile);
			delete pIonosphere;
			delete pIonosphere_fit;
			delete pOutlier;
			delete pEpochTime;			
			return true;
			
		}
		//   子程序名称： detectPhaseSlip   
		//   作用： 构造 Melbourne-Wuebbena 组合量，首先利用M-W组合的历元差完成大周跳的准确探测       
		//   类型： index_P1       : 观测类型P1索引
		//          index_P2       : 观测类型P2索引
		//          index_L1       : 观测类型L1索引
		//          index_L2       : 观测类型L2索引
		//          frequence_L1   : L1的频率
		//          frequence_L2   : L2的频率
		//          editedObsSat   : 输出的弧段结构数据
		//          bOutTempFile   : 是否输出预处理信息
		//   输入： index_P1, index_P2, index_L1, index_L2,frequence_L1, frequence_L2,editedObsSat,bOutTempFile
		//   输出： editedObsSat		
		//   语言： C++
		//   创建者：谷德峰、刘俊宏
		//   创建时间：2012/6/4
		//   版本时间：2012/6/4
		//   修改记录：
		//   其它： 
		bool    BDObsPreproc::detectPhaseSlip(int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1, double frequence_L2, Rinex2_1_EditedObsSat& editedObsSat,bool bOutTempFile)
		{
			//// 创建预处理目录
			//string folder = m_strObsFileName.substr(0, m_strObsFileName.find_last_of("\\"));
			//string obsFileName = m_strObsFileName.substr(m_strObsFileName.find_last_of("\\") + 1);
			//string obsFileName_noexp = obsFileName.substr(0, obsFileName.find_last_of("."));
			//// 创建预处理目录
			//string strPreprocFolder = folder + "\\Preproc";		
			//char  M_W_L1_L2FileName[200];
			//sprintf(M_W_L1_L2FileName, "%s\\%s_M_W_L1L2_IF.dat", strPreprocFolder.c_str(), obsFileName_noexp.c_str());
			FILE *pfile;
			if(bOutTempFile == true)
			{
				// 测试程序，临时写文件				
				char szStationName[4 + 1];
				for(int k = 0; k < 4; k++)
				{
					szStationName[k] = m_obsFile.m_header.szMarkName[k];
				}
				szStationName[4] = '\0';			
				char  M_W_L1_L2FileName[200];
				sprintf(M_W_L1_L2FileName, "%s\\%s_M_W_L1L2_IF.dat", m_strPreprocFilePath.c_str(), szStationName);
				pfile = fopen(M_W_L1_L2FileName,"a+");				
			}

			size_t nCount = editedObsSat.editedObs.size();
			if(nCount <= m_PreprocessorDefine.min_arcpointcount)  // 观测个数太少, 直接丢弃
			{
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
				{	
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}					
				}
				return true;
			}
			// 宽巷载波相位 - 窄巷伪距
			double  *pWL_NP = new double[nCount];
			double  *pEpochTime = new double[nCount];			
			double  *pL1_L2   = new double[nCount];//相位电离层组合(相位之差)	
			double  *pL1_L2_mean   = new double[nCount];//相位电离层组合(相位之差)
			int *pSlip = new int [nCount];
			double *pIonosphere_phase_code = new double[nCount];
			Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSat.editedObs.begin();
			BDT t0 = it0->first;  
			double coefficient_ionosphere = 1 / (1 - pow( frequence_L1 / frequence_L2, 2 ));
			int i = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
			{
				pEpochTime[i] = it->first - t0;					
				Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[index_P2];
				Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[index_L1];
				Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[index_L2];
				double dP1 = P1.obs.data;
				double dP2 = P2.obs.data;
				double dL1 = L1.obs.data*SPEED_LIGHT/frequence_L1;
				double dL2 = L2.obs.data*SPEED_LIGHT/frequence_L2;
				// 构造无电离层组合
				double code_ionofree  = dP1  -(dP1 - dP2) * coefficient_ionosphere;
				double phase_ionofree = dL1 - (dL1 - dL2) * coefficient_ionosphere;
				// 构造宽巷载波相位 widelane_L 和窄巷伪距 narrowlane_P
				double widelane_L   = (frequence_L1 * dL1 - frequence_L2 * dL2) / (frequence_L1 - frequence_L2);
				double narrowlane_P = (frequence_L1 * dP1 + frequence_L2 * dP2) / (frequence_L1 + frequence_L2);
				double WAVELENGTH_W = SPEED_LIGHT/(frequence_L1 - frequence_L2);			  				
				pWL_NP[i] = (widelane_L - narrowlane_P)/WAVELENGTH_W; // melbourne-wuebbena 组合量
				pIonosphere_phase_code[i] = phase_ionofree - code_ionofree; //消电离层组合
				// 构造相位电离层组合(L1-L2)
				pL1_L2[i] = dL1 - dL2;
				//DayTime tt = it->first;
				//fprintf(pfile,"%3d %s %14.4lf %14.4lf %14.4lf\n",editedObsSat.Id,tt.toString().c_str(),pWL_NP[i],pL1_L2[i],pIonosphere_phase_code[i]);
				// 如果伪码已经标记为野值，相位为正常点，则将相应的相位也标记为野值。需要修改，因为L1-L2时，只用到了相位的数据。12.11.14
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)					
				{
					if(L1.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_MW);						
					}
					if(L2.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{						
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_MW);
					}
				}
				// 接受先前伪码观测数据的野值判断结果,  补充相位野值
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER
				|| L1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || L2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
					 pSlip[i] = TYPE_EDITEDMARK_OUTLIER; 
				else
					pSlip[i] = OBSPREPROC_NORMAL;				
				i++;

			}
			//fclose(pfile);
			size_t k   = 0;
			size_t k_i = k;
			 //获得每个连续跟踪弧段的数据			
			while(1)
			{
				if(k_i + 1 >= nCount)
					goto newArc;
				else
				{   
					// 判断 k_i+1 与 k_i 是否位于同一跟踪弧段
					if(pEpochTime[k_i + 1] - pEpochTime[k_i] <= min(m_PreprocessorDefine.threshold_gap,m_PreprocessorDefine.max_arclengh))
					{
						k_i++;
						continue;
					}
					else // k_i + 1 为新弧段的起点
						goto newArc;
				}
				newArc:  // 本弧段[k，k_i]数据处理 
				{
					vector<size_t>   unknownPointlist;
					unknownPointlist.clear();
					for(size_t s_i = k; s_i <= k_i; s_i++)
					{
						// 未知数据标记
						if(pSlip[s_i] == OBSPREPROC_NORMAL)
							unknownPointlist.push_back(s_i); 
					}
					size_t nCount_points = unknownPointlist.size(); 
					// 弧段内数据个数太少					
					if(nCount_points <= int(m_PreprocessorDefine.min_arcpointcount))
					{
						for(size_t s_i = 0; s_i < nCount_points; s_i++)
						{
							if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
								pSlip[unknownPointlist[s_i]] = OBSPREPROC_OUTLIER_COUNT; // 弧段内数据个数太少标记为野值							
						}
					}
					else
					{   						
						// 第一步: 计算MW组合量历元差数据的方差
						// 构造MW组合量历元差数据
						double *pDWL_NP = new double[nCount_points - 1];
						for(size_t s_i = 1; s_i < nCount_points; s_i++)
							pDWL_NP[s_i - 1] = pWL_NP[unknownPointlist[s_i]] - pWL_NP[unknownPointlist[s_i - 1]] ;
						double var = RobustStatRms(pDWL_NP, int(nCount_points - 1));
						delete pDWL_NP;
						// 第二步：进行相位野值剔除
						double threshold_outlier = 5 * var;						
						// 20071012 添加, 利用 threshold_slipsize_wm 对 threshold_outlier 的上界进行控制
						// 因为当周跳发生在序列中间附近时, var 可能会超界, 影响野值探测
						threshold_outlier = min(threshold_outlier, m_PreprocessorDefine.threshold_slipsize_mw);
						// [1, nCount_points - 2]
						for(size_t s_i = 1; s_i < nCount_points - 1; s_i++)
						{
							if(fabs(pWL_NP[unknownPointlist[s_i]]     - pWL_NP[unknownPointlist[s_i-1]]) > threshold_outlier
							&& fabs(pWL_NP[unknownPointlist[s_i + 1]] - pWL_NP[unknownPointlist[s_i] ])  > threshold_outlier)
							{								
								if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
									pSlip[unknownPointlist[s_i]] = OBSPREPROC_OUTLIER_MW;								
							}							
						}
						// 首尾两点 0 和 nCount_points - 1
						if(pSlip[unknownPointlist[1]] != OBSPREPROC_NORMAL)
							pSlip[unknownPointlist[0]] = OBSPREPROC_OUTLIER_MW;
						else
						{
							if(fabs(pWL_NP[unknownPointlist[0]] - pWL_NP[unknownPointlist[1]])  > threshold_outlier)
								pSlip[unknownPointlist[0]] = OBSPREPROC_OUTLIER_MW;
						}
						if(pSlip[unknownPointlist[nCount_points - 2]] != OBSPREPROC_NORMAL)
							pSlip[unknownPointlist[nCount_points - 1]] = OBSPREPROC_OUTLIER_MW;						
						else
						{
							if(fabs(pWL_NP[unknownPointlist[nCount_points - 1]] - pWL_NP[unknownPointlist[nCount_points - 2] ])  > threshold_outlier)
								pSlip[unknownPointlist[nCount_points - 1]] = OBSPREPROC_OUTLIER_MW;
						}
						size_t s_i = 0;
						while(s_i < unknownPointlist.size())
						{
							if(pSlip[unknownPointlist[s_i]] == OBSPREPROC_NORMAL)
								s_i++;
							else
							{
								// 在进行周跳探测时, 先将野值 erase
								unknownPointlist.erase(unknownPointlist.begin() + s_i);
							}
						}
						nCount_points = unknownPointlist.size();
						// 第三步：进行大周跳探测
						if(nCount_points <= 3)
						{
							// 个数太少则直接丢弃
							for(size_t s_i = 0; s_i < nCount_points; s_i++)
							{								
								if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
									pSlip[unknownPointlist[s_i]] = OBSPREPROC_OUTLIER_COUNT;								
							}
						}
						else
						{
							vector<size_t> slipindexlist;
							slipindexlist.clear();
							// [1, nCount_points - 2]
							double threshold_largeslip = max(threshold_outlier, m_PreprocessorDefine.threshold_slipsize_mw);
							for(size_t s_i = 1; s_i < nCount_points - 1; s_i++)
							{
								// 大周跳发生, 每个大周跳在探测后, 其信息都被保存下来了
								if(fabs(pWL_NP[unknownPointlist[s_i]]     - pWL_NP[unknownPointlist[s_i - 1]]) >  threshold_largeslip
								&& fabs(pWL_NP[unknownPointlist[s_i + 1]] - pWL_NP[unknownPointlist[s_i] ])    <= threshold_largeslip) 
								{									
									size_t index = unknownPointlist[s_i];
									pSlip[index] = OBSPREPROC_SLIP_MW;
								}
								else
								{
									/* 
									    消电离层组合检验, 
										M-W组合量只能识别 L1 - L2 的周跳, 因此无法识别两个频率发生的等大小的周跳,
										而等大小的周跳同样会对相位无电离层组合带来影响, 因此在这里要补充关于无电 
										离层组合的大周跳探测, 以确保在精密定轨中迭代收殓      
									*/
									// 消电离层组合要放大观测噪声 3 倍左右, 大约是 mw 组合的 4 倍
									if(m_PreprocessorDefine.bOn_IonosphereFree)
									{
										if(fabs(pIonosphere_phase_code[unknownPointlist[s_i]]     - pIonosphere_phase_code[unknownPointlist[s_i - 1]]) > 8
										&& fabs(pIonosphere_phase_code[unknownPointlist[s_i + 1]] - pIonosphere_phase_code[unknownPointlist[s_i] ])   <= 8)
										{											
											size_t index = unknownPointlist[s_i];
											pSlip[index] = OBSPREPROC_SLIP_IF;
										}
									}
									// 计算历元间隔，因为L1-L2探测野值也周跳受电离层影响较大，所以需要考虑历元间隔的影响，2012.10.24
									int interval = 30;// 采样间隔
									//double threshold_iono_diff = 0.08;  // 采样间隔的电离层历元差阈值
									int before = int(pEpochTime[unknownPointlist[s_i]] - pEpochTime[unknownPointlist[s_i - 1]])/interval;
									int after  = int(pEpochTime[unknownPointlist[s_i + 1]]  - pEpochTime[unknownPointlist[s_i]])/interval;
									// L1 - L2探测野值
									if(fabs(pL1_L2[unknownPointlist[s_i]]     - pL1_L2[unknownPointlist[s_i - 1]]) > m_PreprocessorDefine.threshold_outliersize_L1_L2 * before
									&& fabs(pL1_L2[unknownPointlist[s_i + 1]] - pL1_L2[unknownPointlist[s_i]])     > m_PreprocessorDefine.threshold_outliersize_L1_L2 * after
									&& pEpochTime[unknownPointlist[s_i]]      - pEpochTime[unknownPointlist[s_i - 1]] <= m_PreprocessorDefine.threshold_gap_L1_L2
									&& pEpochTime[unknownPointlist[s_i + 1]]  - pEpochTime[unknownPointlist[s_i]]     <= m_PreprocessorDefine.threshold_gap_L1_L2)
									{										
										size_t index = unknownPointlist[s_i];
										pSlip[index] = OBSPREPROC_OUTLIER_L1_L2;
									}
									// L1 - L2探测周跳
									else if(fabs(pL1_L2[unknownPointlist[s_i]]     - pL1_L2[unknownPointlist[s_i - 1]]) > m_PreprocessorDefine.threshold_slipsize_L1_L2
									&& fabs(pL1_L2[unknownPointlist[s_i + 1]] - pL1_L2[unknownPointlist[s_i]])   <= m_PreprocessorDefine.threshold_outliersize_L1_L2
									&& pEpochTime[unknownPointlist[s_i]]      - pEpochTime[unknownPointlist[s_i - 1]] <= m_PreprocessorDefine.threshold_gap_L1_L2
									&& pEpochTime[unknownPointlist[s_i + 1]]  - pEpochTime[unknownPointlist[s_i]]     <= m_PreprocessorDefine.threshold_gap_L1_L2
									&& pSlip[unknownPointlist[s_i - 1]] != OBSPREPROC_OUTLIER_L1_L2)
									{
										size_t index = unknownPointlist[s_i];
										pSlip[index] = OBSPREPROC_SLIP_L1_L2;
									}									
								}
							}
							// 首尾两点 0 和 nCount_points - 1
							if(pSlip[unknownPointlist[1]] != OBSPREPROC_NORMAL)
								pSlip[unknownPointlist[0]] = OBSPREPROC_OUTLIER_L1_L2;
							else
							{
								if(fabs(pL1_L2[unknownPointlist[0]] - pL1_L2[unknownPointlist[1]])  > m_PreprocessorDefine.threshold_outliersize_L1_L2)
									pSlip[unknownPointlist[0]] = OBSPREPROC_OUTLIER_L1_L2;
							}
							if(pSlip[unknownPointlist[nCount_points - 2]] != OBSPREPROC_NORMAL)
								pSlip[unknownPointlist[nCount_points - 1]] = OBSPREPROC_OUTLIER_L1_L2;							
							else
							{
								if(fabs(pL1_L2[unknownPointlist[nCount_points - 1]] - pL1_L2[unknownPointlist[nCount_points - 2]])  >  m_PreprocessorDefine.threshold_outliersize_L1_L2)
									pSlip[unknownPointlist[nCount_points - 1]] = OBSPREPROC_OUTLIER_L1_L2;
							}
							////////////////////////剔除周跳频繁的段弧段，并进一步剔除小野值点
							for(size_t s_i = 1; s_i < nCount_points; s_i++)
							{
								size_t index = unknownPointlist[s_i];
								if(pSlip[index] == OBSPREPROC_SLIP_MW || pSlip[index] == OBSPREPROC_SLIP_IF
								|| pSlip[index] == OBSPREPROC_SLIP_L1_L2)
									slipindexlist.push_back(index); 
							}
							size_t count_slips = slipindexlist.size();
							size_t *pSubsection_left  = new size_t [count_slips + 1];
							size_t *pSubsection_right = new size_t [count_slips + 1];
							if(count_slips > 0)
							{ 
								// 记录周跳的左右端点值
								pSubsection_left[0] = unknownPointlist[0];
								for(size_t s_i = 0; s_i < count_slips; s_i++)
								{
									pSubsection_right[s_i]    = slipindexlist[s_i] -  1;
									pSubsection_left[s_i + 1] = slipindexlist[s_i];
								}
								pSubsection_right[count_slips] = unknownPointlist[nCount_points - 1]; 
							}
							else
							{
								pSubsection_left[0]  = unknownPointlist[0];
								pSubsection_right[0] = unknownPointlist[nCount_points - 1];
							} 
							for(size_t s_i = 0; s_i < count_slips + 1; s_i++)
							{
								// 整理 [pSubsection_left[s_i], pSubsection_right[s_i]]
								vector<size_t> subsectionNormalPointlist;
								subsectionNormalPointlist.clear();
								for(size_t s_j = pSubsection_left[s_i]; s_j <= pSubsection_right[s_i]; s_j++)
								{
									if(pSlip[s_j] != TYPE_EDITEDMARK_OUTLIER && pSlip[s_j] != OBSPREPROC_OUTLIER_MW && pSlip[s_j] != OBSPREPROC_OUTLIER_L1_L2)
										subsectionNormalPointlist.push_back(s_j); 
								}
								size_t count_subsection = subsectionNormalPointlist.size(); 
								if(count_subsection > m_PreprocessorDefine.min_arcpointcount)
								{   
									//用M-W剔除野值(暂时不做)
									////count_normal_arc++;
									//double *pX = new double [count_subsection];
									//double *pW = new double [count_subsection];
									//double mean = 0;
									//double var  = 0;
									//for(size_t s_j = 0; s_j < count_subsection; s_j++)
									//	pX[s_j] = pL1_L2[subsectionNormalPointlist[s_j]];  
									//robustcalculate_mean(pX, pW, int(count_subsection), mean, var, 4);//
									//for(size_t s_j = 0; s_j < count_subsection; s_j++)
									//	pL1_L2_mean[subsectionNormalPointlist[s_j]] = pL1_L2[subsectionNormalPointlist[s_j]] - mean;
									//FILE *pfile = fopen("C:\\L1_L2.cpp","a+");
									//for(size_t s_j = 0; s_j < count_subsection-1; s_j++)
									//{
									//	int interval = 30;// 采样间隔										
									//	int before = int(pEpochTime[subsectionNormalPointlist[s_j + 1]] - pEpochTime[subsectionNormalPointlist[s_j]])/interval;
									//	//int after  = int(pEpochTime[unknownPointlist[s_i + 1]]  - pEpochTime[unknownPointlist[s_i]])/interval;
									//	fprintf(pfile,"%16.4f %16.4f\n",pEpochTime[subsectionNormalPointlist[s_j]],
									//	        (pL1_L2[subsectionNormalPointlist[s_j+1]] - pL1_L2[subsectionNormalPointlist[s_j]])/before);
									//}
									//fclose(pfile);
								}
								else
								{
									//MW 组合序列弧段的正常点个数过少!直接标为野值
									for(size_t s_j = 0; s_j < count_subsection; s_j++)									
										pSlip[subsectionNormalPointlist[s_j]] = OBSPREPROC_OUTLIER_COUNT; 									
								}
							}
							delete pSubsection_left;
							delete pSubsection_right;
							for(size_t s_i = k; s_i <= k_i; s_i++)
							{
								// 将第一个非野值点, 更新标记为周跳
								if(pSlip[s_i] == OBSPREPROC_NORMAL || pSlip[s_i] == OBSPREPROC_SLIP_MW
									||pSlip[s_i] == OBSPREPROC_SLIP_IF || pSlip[s_i] == OBSPREPROC_SLIP_L1_L2)
								{
									pSlip[s_i] = OBSPREPROC_NEWARCBEGIN;
									break;
								}
							}
						}						
					}
					if(k_i + 1 >= nCount)
						break;
					else  
					{
						// 新弧段的起点设置
						k   = k_i + 1;
						k_i = k;
						continue;
					}
				}
			}	

			// 保存周跳标志
			i = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
			{
				DayTime tt = it->first;
				if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[i]);
					it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[i]);
				}
				if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[i]);	
					it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[i]);
				}		
				if(bOutTempFile == true)
				{
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
					&& it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
					&& it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
					&& it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						fprintf(pfile,"%3d %s %14.4lf %14.4lf %14.4lf %2d\n",editedObsSat.Id,
																			 tt.toString().c_str(),
																			 pWL_NP[i],
																			 pL1_L2[i],
																			 pIonosphere_phase_code[i],
																			 it->second.obsTypeList[index_L1].byEditedMark1 * 10 + it->second.obsTypeList[index_L1].byEditedMark2);
				}
				i++;
			}
			if(bOutTempFile == true)
				fclose(pfile);
			// 保留
			delete pWL_NP;
			delete pEpochTime;
			delete pL1_L2;
			delete pL1_L2_mean;
			delete pSlip;			
			delete pIonosphere_phase_code;					
			return true;
		
		}
		// 子程序名称： mainFuncObsPreproc   
		// 功能：北斗双频观测数据预处理
		// 变量类型：    editedobsfile      　　              : 预处理后的观测数据
		//               bOutTempFile                         : 是否输出预处理信息
		// 输入：m_obsFile，bOutTempFile
		// 输出：
		// 语言：C++
		// 创建者：谷德峰，刘俊宏
		// 创建时间：2012/3/14
		// 版本时间：2012/4/9
		// 修改记录：
		// 其它： 
		bool BDObsPreproc::mainFuncObsPreproc(Rinex2_1_EditedObsFile  &editedobsfile,bool bOutTempFile)
		{
			//// 创建预处理目录
			//string folder = m_strObsFileName.substr(0, m_strObsFileName.find_last_of("\\"));
			//string obsFileName = m_strObsFileName.substr(m_strObsFileName.find_last_of("\\") + 1);
			//string obsFileName_noexp = obsFileName.substr(0, obsFileName.find_last_of("."));
			// 创建预处理目录
			//string strPreprocFolder = folder + "\\Preproc";
			
			//FILE *pfile;
			if(bOutTempFile == true)
			{
				_mkdir(m_strPreprocFilePath.c_str());
				// 测试程序，临时写文件				
				char szStationName[4 + 1];
				for(int k = 0; k < 4; k++)
				{
					szStationName[k] = m_obsFile.m_header.szMarkName[k];
				}
				szStationName[4] = '\0';			
				char  M_W_L1_L2FileName[200];
				char  Ionosphere_P[200];
				sprintf(M_W_L1_L2FileName, "%s\\%s_M_W_L1L2_IF.dat", m_strPreprocFilePath.c_str(), szStationName);
				sprintf(Ionosphere_P, "%s\\%s_Ionosphere_P.dat", m_strPreprocFilePath.c_str(), szStationName);
				FILE *pfile1 = fopen(M_W_L1_L2FileName,"w+");
				FILE *pfile2 = fopen(Ionosphere_P,"w+");
				fclose(pfile1);			
				fclose(pfile2);
			}


			bool  nav_flag = false;
			if(m_navFile.isEmpty() && m_sp3File.isEmpty())
			{		
				printf("无星历数据!，请确认!\n");				
				return  false;				
			}
			else
				if(m_navFile.isEmpty())
					nav_flag = true;//
			if(m_obsFile.isEmpty())
			{
				if(m_obsFile.isEmpty())
					printf("无观测数据!，请确认!\n");
				return  false;
			}
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_L2 = -1, nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_L5 = -1, nObsTypes_P5 = -1;
			for(int i = 0; i < m_obsFile.m_header.byObsTypes; i++)
			{
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2 = i;					
			}			
			if(nObsTypes_L1 == -1 || nObsTypes_L2 == -1 || nObsTypes_P1 == -1 || nObsTypes_P2 == -1) 
			{
				printf("观测数据类型不完整！\n");
				return false;		
			}
			int index[4];
			index[0] = nObsTypes_L1;
			index[1] = nObsTypes_L2;
			index[2] = nObsTypes_P1;
			index[3] = nObsTypes_P2;
			vector<Rinex2_1_EditedObsEpoch> editedObsEpochlist;
			vector<Rinex2_1_EditedObsSat>   editedObsSatlist1,editedObsSatlist;
			getEditedObsSatList(editedObsSatlist1);
			for(size_t s_j = 0; s_j < editedObsSatlist1.size(); s_j++)
			{
				if(editedObsSatlist1[s_j].Id <= 14)          //30号卫星不可用				   
					editedObsSatlist.push_back(editedObsSatlist1[s_j]);
			}					
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{	// 根据数据的 DBL_MAX (或为0)标记直接判断原始观测数据为野值添加
			    // 因为原始数据某些通道可能空缺, 此时该数据通常被赋值为 DBL_MAX(或为0), 在这里要将标记其恢复为野值	
				int   nPRN = editedObsSatlist[s_i].Id;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{
					for(int k = 0; k < 4; k++)
					{
						if(it->second.obsTypeList[index[k]].obs.data == 0.0)	
						{
							it->second.obsTypeList[index[k]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_ZERO);
							it->second.obsTypeList[index[k]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_ZERO);
						}
						if(it->second.obsTypeList[index[k]].obs.data == DBL_MAX)
						{
							it->second.obsTypeList[index[k]].obs.data = 0.0;
							it->second.obsTypeList[index[k]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_BLANK);
							it->second.obsTypeList[index[k]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_BLANK);
						}
					}				
					//计算观测仰角,将仰角过低的数据标记为野值点
					POSCLK    posclk;          // 从导航文件获取的卫星位置
					SP3Datum  sp3Datum;        // 从sp3文件获取的卫星位置
					POS3D     ECFposSat;       // 卫星在地固系下的坐标
					ENU       ENUposSat;       // 卫星在测站东北天坐标系下的位置
					bool      posflag = false;
					if(nav_flag)//
					{
						if(m_sp3File.getEphemeris(it->first,nPRN,sp3Datum,9,'C'))
						{
							ECFposSat = sp3Datum.pos;
							posflag = true;
						}
					}
					else
					{
						if(m_navFile.getEphemeris(it->first,nPRN,posclk))
						{
							ECFposSat = posclk.getPos();
							posflag = true;
						}
					}
					if(posflag)
					{
						TimeCoordConvert::ECF2ENU(m_posStation,ECFposSat,ENUposSat);		
						// 计算仰角(单位：度)
						it->second.Elevation = atan(ENUposSat.U/sqrt(ENUposSat.E*ENUposSat.E + ENUposSat.N*ENUposSat.N))*180/PI;
						// 计算方位角(单位：度)
						it->second.Azimuth   = atan2(ENUposSat.E, ENUposSat.N) * 180 / PI;

						POS3D p_station = vectorNormal(m_posStation);
						POS3D p_sat = vectorNormal(ECFposSat - m_posStation);					
						p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); // 20150608, 考虑到地球扁率的影响, 卫星仰角的计算进行了修正, 谷德峰
						p_station = vectorNormal(p_station);					
						it->second.Elevation = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;
					}
					else
					{
						it->second.Elevation = 0;
						it->second.Azimuth   = 0;
					}
					if(it->second.Azimuth < 0)
					{// 变换到[0, 360]
						it->second.Azimuth += 360.0;
					}
					double min_elevation = m_PreprocessorDefine.min_elevation;
					if(nPRN <= 5) //GEO 卫星10度以下仰角的观测数据较差，例如brst测站的C05卫星
						min_elevation = 10;         //对于定轨，强制使用大于10度的观测数据，20140816，刘俊宏
					// 将低仰角数据标记为野值：OBSPREPROC_OUTLIER_ELEVATION
					if(it->second.Elevation <= min_elevation)
					{						
						for(int k = 0; k < 4; k++)
						{
							if(it->second.obsTypeList[index[k]].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[index[k]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_ELEVATION);
								it->second.obsTypeList[index[k]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_ELEVATION);
							}
						}
					}								
				}
			}			
			// 伪码野值探测		
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
				detectCodeOutlier_ionosphere(nObsTypes_P1, nObsTypes_P2, BD_FREQUENCE_L1,BD_FREQUENCE_L2,editedObsSatlist[s_i],bOutTempFile);			
			//相位野值周跳探测			
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
				detectPhaseSlip(nObsTypes_P1, nObsTypes_P2, nObsTypes_L1, nObsTypes_L2, BD_FREQUENCE_L1, BD_FREQUENCE_L2, editedObsSatlist[s_i],bOutTempFile);

			datalist_sat2epoch(editedObsSatlist,editedObsEpochlist);
			//更新头文件
			editedobsfile.m_header = m_obsFile.m_header;
			editedobsfile.m_header.bySatCount = (int)editedObsSatlist.size();
			editedobsfile.m_header.tmStart = editedObsEpochlist.front().t;
			editedobsfile.m_header.tmEnd = editedObsEpochlist.back().t;
			editedobsfile.m_data   = editedObsEpochlist;					

			return  true;
		}
		// 子程序名称： detectCodeOutlier_ionosphere_GPS   
		// 功能：电离层组合探测伪码野值
		// 变量类型：index_P1，index_P2 :  P1和P2伪码数据的索引
		//           frequence_P1       :  P1对应的载波频点
		//           frequence_P2       :  P2对应的载波频点
		//           editedObsSat       :  输出的弧段结构数据
		//           bOutTempFile       :  输出文件开关
		// 输入：index_P1, index_P2, frequence_P1, frequence_P2, bOutTempFile
		// 输出：editedObsSat
		// 语言：C++
		// 创建者：谷德峰，鞠 冰
		// 创建时间：2012/9/14
		// 版本时间：2012/9/12
		// 修改记录：1、添加数据是否已被标记的判断（2012-10-12）
		//			 2、每颗卫星的观测数据分弧段写文件,分析 VONDARK 滤波性能(2012-10-26)
		//			 3、调整 VONDARK 滤波器参数，降低误判概率(2013-05-09)
		//           4、添加输出文件开关(2014-03-26)
		//           5、用于BDS数据处理，2014/7/1,刘俊宏
		// 备注： 原程序名detectCodeOutlier_ionosphere
		bool BDObsPreproc::detectCodeOutlier_ionosphere_GPS(int index_P1, int index_P2, double frequence_P1, double frequence_P2, DayTime T0, Rinex2_1_EditedObsSat& editedObsSat, bool bOutTempFile)
		{
			size_t num = editedObsSat.editedObs.size();
			/* Step1：若该颗卫星的正常观测弧段(剔除低仰角数据后的弧段)过短，直接将P1、P2数据判为野值（2012-10-16）*/
			if(num <= m_PreprocessorDefine.min_arcpointcount)
			{
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); it++)
				{	// 判断观测数据是否已被标记为野值，若已被标记则不再重复标记
					if(it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
				}
				return true;
			}
			/* Step2：计算电离层延迟及观测时间序列 */
			double alpha   = pow(frequence_P1,2)/pow(frequence_P2,2);	// alpha = f1^2/f2^2
			double coef_ionosphere1 = 1/(1 - alpha);					// P1电离层延迟系数				
			//double coef_ionosphere2 = alpha/(alpha - 1);				// (预留)P2电离层延迟系数
			Rinex2_1_EditedObsDatum  P1,P2;								// 伪码数据P1、P2
			double *pdeltaP1	= new double[num];						// 伪码电离层组合序列1:deltaP1[i] = coef_ionosphere1*(P1-P2)
			//double *pdeltaP2	= new double[num];						// (预留)伪码电离层组合序列2:deltaP2[i] = coef_ionosphere2*(P2-P1)
			double *pfitdeltaP1 = new double[num];						// 电离层Vondrak滤波拟合值
			double *pEpochTime  = new double[num];						// 观测时间序列（相对于起始观测时刻，单位：秒）
			int    *pOutlier    = new    int[num];						// 存储每个历元伪码数据的编辑信息
			int k = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); it++)
			{
				pEpochTime[k] = it->first - T0;
				P1 = it->second.obsTypeList[index_P1];
				P2 = it->second.obsTypeList[index_P2];
				pdeltaP1[k] = coef_ionosphere1*(P1.obs.data - P2.obs.data);
				//deltaP2[k] = coef_ionosphere2*(P2.obs.data - P1.obs.data);				//(预留)
				// 接受先前野值判决信息：OBSPREPROC_OUTLIER_BLANK,若有一个伪码数据为OBSPREPROC_OUTLIER_BLANK，另一个标为OBSPREPROC_OUTLIER_IONOMAXMIN
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
				{
					if(P1.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_IONOMAXMIN);
						it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_IONOMAXMIN);
					}
					if(P2.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_IONOMAXMIN);
						it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_IONOMAXMIN);
					}
					pOutlier[k] = TYPE_EDITEDMARK_OUTLIER;	// 含有空白数据或“0”处pOutlier值始终保持TYPE_EDITEDMARK_OUTLIER不变
				}
				else
				{
					pOutlier[k] = OBSPREPROC_NORMAL;
				}
				k = k + 1;
			}
			/* Step3：判断电离层延迟是否超差 */
			int   ArcCounts  = 0;				// 记录弧段标号
			size_t arc_begin = 0;				// 弧段起始位置
			size_t arc_end   = arc_begin;		// 弧段终止位置
			FILE *pfile;
			if(bOutTempFile == true)
			{
				// 将不含空白数据的所有历元预处理结果写入文件(2012-10-28)
				char szCodePreprocFileName[100];
				char szStationName[4 + 1];
				for(int k = 0; k < 4; k++)
				{
					szStationName[k] = m_obsFile.m_header.szMarkName[k];
				}
				szStationName[4] = '\0';
				sprintf(szCodePreprocFileName,"%s\\%s_CodePreproc.dat",m_strPreprocFilePath.c_str(), szStationName);
				pfile = fopen(szCodePreprocFileName,"a+");
			}
			// 获取每个连续弧段的观测数据
			while(1)
			{
				if(arc_end + 1 >= num)
				{
					goto newArc;
				}
				else
				{
					if(pEpochTime[arc_end + 1] - pEpochTime[arc_end] < m_PreprocessorDefine.max_arclengh)
					{
						arc_end++;
						continue;
					}
					else
					{
						goto newArc;
					}
				}
				newArc:	// 在弧段[arc_begin, arc_end]内数据处理
				{
					ArcCounts++;
					//若观测弧段过短，直接判为野值
					size_t nArcPointsCount = arc_end - arc_begin + 1;
					if(nArcPointsCount <= m_PreprocessorDefine.min_arcpointcount)
					{
						for(size_t k = arc_begin; k <= arc_end; k++)
						{
							pfitdeltaP1[k] = 0.0;					// 写文件时，若弧段过短填零(2012-10-20)
							if(pOutlier[k] == OBSPREPROC_NORMAL)	// 不修改数据空白处的pOutlier值(2012-10-28)
							{
								pOutlier[k] = OBSPREPROC_OUTLIER_COUNT;
							}
						}
					}
					else
					{
						double *w = new double [nArcPointsCount];
						// 根据阈值判断电离层延迟是否超差
						for(size_t k = arc_begin; k <= arc_end; k++)
						{
							if(pdeltaP1[k] > m_PreprocessorDefine.max_ionosphere || pdeltaP1[k] < m_PreprocessorDefine.min_ionosphere)
							{
								w[k - arc_begin] = 0;
								if(pOutlier[k] == OBSPREPROC_NORMAL) // 不修改已经判为野值点的pOutlier[k]值(2012-10-28)
								{
									pOutlier[k] = OBSPREPROC_OUTLIER_IONOMAXMIN;
								}
							}
							else
							{
								if(pOutlier[k] == TYPE_EDITEDMARK_OUTLIER)// 防止P1、P2均为0时将权重w赋为1
								{
									w[k - arc_begin] = 0;
								}
								else
								{
									w[k - arc_begin] = 1;
								}
							}	
						}
						// Vondrak滤波检测野值
						KinematicRobustVandrakFilter(pEpochTime + arc_begin , pdeltaP1 + arc_begin, w, int(nArcPointsCount),
							m_PreprocessorDefine.vondrak_PIF_eps,
							pfitdeltaP1 + arc_begin,
							m_PreprocessorDefine.vondrak_PIF_max,
							m_PreprocessorDefine.vondrak_PIF_min,
							m_PreprocessorDefine.vondrak_PIF_width);
						for(size_t k = arc_begin; k <= arc_end; k++)
						{
							if(w[k - arc_begin] == 0 && pOutlier[k] == OBSPREPROC_NORMAL)
							{
								pOutlier[k] = OBSPREPROC_OUTLIER_VONDRAK;
							}
						}
						delete[] w;
					}
					if(bOutTempFile == true)
					{
						// 写文件
						fprintf(pfile, "PRN %2d -> Arc: %2d\n", editedObsSat.Id, ArcCounts);
						fprintf(pfile, "%8s %8s %10s %10s %8s\n",
							"T",
							"E",
							"dP1",
							"fit_dP1",
							"Marks");
						for(size_t k = arc_begin; k <= arc_end; k++)
						{
							// 输出所有P1、P2均为非空白数据的野值拣择结果(2012-10-28)
							if(pOutlier[k] != TYPE_EDITEDMARK_OUTLIER)
							{
								Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.find(T0 + pEpochTime[k]);
								fprintf(pfile,"%8.2f %8.2f %10.4f %10.4f %8d\n",
									pEpochTime[k],
									it->second.Elevation,
									pdeltaP1[k],
									pfitdeltaP1[k],
									pOutlier[k]);
							}
						}
					}
					if(arc_end + 1 >= num)
						break;
					else
					{
						arc_begin = arc_end + 1;
						arc_end   = arc_begin;
						continue;
					}
				}//end of newArc
			}// end of while()
			if(bOutTempFile == true)
			{
				fclose(pfile);
			}

			/* Step4：标记伪码数据野值检测信息 */
			k = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); it++)
			{
				if(it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(pOutlier[k]);
					it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(pOutlier[k]);
				}
				if(it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(pOutlier[k]);
					it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(pOutlier[k]);
				}
				k = k + 1;
			}
			// 释放内存空间
			delete[] pdeltaP1;
			delete[] pfitdeltaP1;
			delete[] pEpochTime;
			delete[] pOutlier;
			//delete []deltaP2;			//(预留)
			return true;
		}
		//   子程序名称： detectPhaseSlip_GPS   
		//   作用： 1、构造 Melbourne-Wuebbena 组合量，利用M-W组合的历元差完成相位数据的野值及大周跳探测
		//          2、构造 L1 - L2 组合量，利用多项式拟合消除电离层影响，从而通过拟合残差进一步探测野值及小周跳
		//   类型： index_P1       : 观测类型P1索引
		//          index_P2       : 观测类型P2索引
		//          index_L1       : 观测类型L1索引
		//          index_L2       : 观测类型L2索引
		//          frequence_L1   : L1的频率
		//          frequence_L2   : L2的频率
		//          T0             : 测站首次观测历元
		//          editedObsSat   : 某颗卫星的观测数据时间序列
		//   输入： index_P1, index_P2, index_L1, index_L2, frequence_L1, frequence_L2，T0
		//   输出： editedObsSat		
		//   语言： C++
		//   创建者：鞠 冰
		//   创建时间：2013/11/30
		//   版本时间：2012/9/12
		//   修改记录：1、添加输出文件开关，2014/3/26,鞠 冰
		//             2、修改 M-W 野值拣择阈值，取min(5*sigma, 5cycles)，2014/4/5，鞠 冰
		//			   3、对历元间隔超过 L1-L2 拟合区间长度的点添加处理，2014/4/5，鞠 冰
		//			   4、对M-W弧段第一点的周跳信息进行保留，2014/4/5，鞠 冰
		//			   5、对L1-L2拟合点数不足的小弧段进行剔除，2014/4/5，鞠冰
		//             6、用于BDS数据处理，2014/7/1,刘俊宏
		//   其它：原程序名detectPhaseSlip_Juice
		bool BDObsPreproc::detectPhaseSlip_GPS(int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1, double frequence_L2, DayTime T0, Rinex2_1_EditedObsSat& editedObsSat, bool bOutTempFile)
		{
			double WAVELENGH_W   = SPEED_LIGHT/(frequence_L1 - frequence_L2);	// 宽巷波长
			double WAVELENGTH_L1 = SPEED_LIGHT/frequence_L1;
			double WAVELENGTH_L2 = SPEED_LIGHT/frequence_L2;
			size_t num = editedObsSat.editedObs.size();	
			int ArcN = 0;	// M-W弧段计数
			/* Step1：若该颗卫星的正常观测弧段(剔除低仰角数据后的弧段)过短，直接将L1、L2数据判为野值（2012/10/16）*/
			if(num <= m_PreprocessorDefine.min_arcpointcount)
			{
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); it++)
				{	// 判断观测数据是否已被标记为野值，若已被标记则不再重复标记
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
				}
				return true;
			}
			/* Step2：计算 M-W 组合量、L1-L2 组合量及观测时间序列 */		
			double *L_W = new double[num];			// 宽巷相位值
			double *P_N = new double[num];			// 窄巷伪码值
			double *M_W	  = new double[num];		// 构造M-W组合量，宽巷相位与窄巷伪码作差
			double *L1_L2 = new double[num];		// 构造L1-L2组合量
			double *L1_L2_fit = new double[num];	// L1-L2的分段多项式拟合值(2013-12-25)
			int    *pSlip = new int[num];			// 存储每个历元相位数据的编辑信息
			double *pEpochTime = new double[num];	// 观测时间序列（相对于起始观测时刻，单位：秒）
			int k = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); it++)
			{
				pEpochTime[k] = it->first - T0;
				double P1 = it->second.obsTypeList[index_P1].obs.data;
				double P2 = it->second.obsTypeList[index_P2].obs.data;
				double L1 = it->second.obsTypeList[index_L1].obs.data;
				double L2 = it->second.obsTypeList[index_L2].obs.data;
				// 计算宽巷相位
				L_W[k] =  SPEED_LIGHT * (L1 - L2)/(frequence_L1 - frequence_L2);
				// 计算窄巷伪码
				P_N[k] = (frequence_L1 * P1 + frequence_L2 * P2)/(frequence_L1 + frequence_L2);
				// 计算M-W组合量
				M_W[k] = (L_W[k] - P_N[k]) / WAVELENGH_W;
				// 计算L1-L2组合量
				L1_L2[k] = WAVELENGTH_L1 * L1 - WAVELENGTH_L2 * L2;
				// 初始化L1-L2_fit
				L1_L2_fit[k] = 0.0;
				// 若P1或P2已被判为野值，则将L1、L2均判为野值(类型为：OBSPREPROC_OUTLIER_MW)；并考虑了载波相位数据缺失的情况(2012-10-27)
				if(it->second.obsTypeList[index_P1].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || it->second.obsTypeList[index_P2].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER ||
					it->second.obsTypeList[index_L1].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || it->second.obsTypeList[index_L2].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
				{// 接受先前码和相位数据的野值判决信息，剔除相位空白数据和码的野值数据
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_MW);
					}
					if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_MW);
					}
					pSlip[k] = TYPE_EDITEDMARK_OUTLIER;		// 保持TYPE_EDITEDMARK_OUTLIER始终不变
				}
				else
				{
					pSlip[k] = OBSPREPROC_NORMAL;
				}
				k = k + 1;
			}// end of for(it)
			FILE *pfile, *pfile4;
			if(bOutTempFile == true)
			{
				char szPhasePreprocFileName[100];
				char szL1_L2FileName[100];
				string szStationName = m_obsFile.m_header.szMarkName;
				szStationName = szStationName.substr(0, 4);
				sprintf(szPhasePreprocFileName,"%s\\%s_PhasePreproc.dat", m_strPreprocFilePath.c_str(), szStationName.c_str());
				sprintf(szL1_L2FileName, "%s\\%s_L1_L2.dat", m_strPreprocFilePath.c_str(), szStationName.c_str());
				pfile  = fopen(szPhasePreprocFileName,"a+");		// 输出相位数据预处理结果
				//FILE *pfile1 = fopen("E:\\stdDM_W.dat", "a+");		// 输出DM-W序列的标准差
				//FILE *pfile3 = fopen("E:\\maxD_L1_L2.dat", "a+");		// 输出各卫星各 M-W 弧段的 maxD_L1_L2 数据
				pfile4 = fopen(szL1_L2FileName,"a+");				// 输出 maxD_L1_L2 超差弧段的 L1_L2 数据（2013-11-29）
				//FILE *pfile5 = fopen("E:\\D_arc.dat","a+");			// 输出 Sigma 较大的 M-W 组合序列
			}
			/* Step3：判断 M-W 组合量及 L1-L2 组合量是否超差 */
			int   ArcCounts  = 0;				// 记录弧段标号
			size_t arc_begin = 0;				// 弧段起始位置
			size_t arc_end   = arc_begin;		// 弧段终止位置
			double  max_arclength = m_PreprocessorDefine.max_arclengh; 
			if(editedObsSat.Id <= 5)//2015/01/12,刘俊宏，GEO卫星始终处于一个观测弧段
				max_arclength = 3 * 24 * 3600.0;
			while(1)
			{
				if(arc_end + 1 >= num)
				{
					goto newArc;
				}
				else
				{
					if(pEpochTime[arc_end + 1] - pEpochTime[arc_end] < max_arclength)
					{
						arc_end++;
						continue;
					}
					else
					{
						goto newArc;
					}
				}
				newArc:
				{
					ArcCounts++;
					// 弧段[arc_begin, arc_end]内正常点链表
					vector<size_t> pNormalPoints;	
					for(size_t k = arc_begin; k <= arc_end; k++)
					{
						if(pSlip[k] == OBSPREPROC_NORMAL)
							pNormalPoints.push_back(k);
					}
					size_t nCounts = pNormalPoints.size();
					//若观测弧段过短，直接判为野值
					if(nCounts <= m_PreprocessorDefine.min_arcpointcount)
					{
						for(size_t k = 0; k < nCounts; k++)
						{
							pSlip[pNormalPoints[k]] = OBSPREPROC_OUTLIER_COUNT;  // 不修改已经判为野值点的值 (2012/10/28)
						}
					}
					else
					{
						// 计算M-W组合量差分数据的标准差: sigma = sqrt(2)*var(M_W)，反映了 M-W 组合量的噪声水平
						double *DM_W  = new double [nCounts - 1];
						for(size_t k = 0; k < nCounts - 1; k++)
						{
							DM_W[k] = M_W[pNormalPoints[k + 1]] - M_W[pNormalPoints[k]];
						}
						double sigma = RobustStatRms(DM_W, int(nCounts - 1), 3.0);	// 抗差方差估计的权值改为 3 * sigma (2012/12/24)
						delete DM_W;
						// 输出各弧段 stdDM_W 数据至文件：E:\\stdDM_W.dat
						//fprintf(pfile1, "\nPRN% 2d -> Arc: %2d\n", editedObsSat.Id, ArcCounts);
						//fprintf(pfile1, "%10.4f\n", 5 * sigma);
						// 利用 M-W 组合量对相位数据进行野值拣择，sigma 的因子应稍大一些，因为低仰角处的噪声较大，取的过小会有很多虚警
						double threshold_outlier = min(5 * sigma, m_PreprocessorDefine.threshold_slipsize_mw);
						for(size_t k = 1; k < nCounts - 1; k++)
						{
							if(fabs(M_W[pNormalPoints[k]] - M_W[pNormalPoints[k - 1]]) >= threshold_outlier &&
								fabs(M_W[pNormalPoints[k]] - M_W[pNormalPoints[k + 1]]) >= threshold_outlier)
							{
								pSlip[pNormalPoints[k]] = OBSPREPROC_OUTLIER_MW;
							}
						}
						// 处理首尾两端点: pNormalPoints[0] 及 pNormalPoints[nCounts - 1]
						if(pSlip[pNormalPoints[1]] == OBSPREPROC_OUTLIER_MW 
						|| fabs(M_W[pNormalPoints[1]] - M_W[pNormalPoints[0]]) > threshold_outlier)
						{
							pSlip[pNormalPoints[0]] = OBSPREPROC_OUTLIER_MW;
						}
						if(pSlip[pNormalPoints[nCounts - 2]] == OBSPREPROC_OUTLIER_MW
						|| fabs(M_W[pNormalPoints[nCounts - 1]] - M_W[pNormalPoints[nCounts - 2]]) > threshold_outlier)
						{
							pSlip[pNormalPoints[nCounts - 1]] = OBSPREPROC_OUTLIER_MW;
						}
						// 周跳探测前首先剔除野值点, 防止周跳探测虚警（如果不剔除野值，野值后的第一个点就会被误判成是周跳点）
						size_t k = 0;
						while(k < pNormalPoints.size())
						{
							if(pSlip[pNormalPoints[k]] == OBSPREPROC_NORMAL)
								k = k + 1;
							else
								pNormalPoints.erase(pNormalPoints.begin() + k);
						}
						nCounts = pNormalPoints.size();
						// 剔除 OBSPREPROC_OUTLIER_MW 后，剩下的数据量不足标为 OBSPREPROC_OUTLIER_COUNT， 2013-05-08修改
						if(nCounts <= m_PreprocessorDefine.min_arcpointcount)
						{	
							for(size_t k = 0; k < nCounts; k++)
							{
								pSlip[pNormalPoints[k]] = OBSPREPROC_OUTLIER_COUNT;
							}
						}
						else
						{
							vector<size_t> slipindexlist;	// 弧段[arc_begin, arc_end]内周跳点链表
							double threshold_slipsize_mw = m_PreprocessorDefine.threshold_slipsize_mw;
							for(size_t k = 1; k < nCounts - 1; k++)
							{
								// M-W 探测大周跳
								if(fabs(M_W[pNormalPoints[k]] - M_W[pNormalPoints[k - 1]]) >=  threshold_slipsize_mw
									&&fabs(M_W[pNormalPoints[k]] - M_W[pNormalPoints[k + 1]]) <   threshold_slipsize_mw)
								{
									pSlip[pNormalPoints[k]] = OBSPREPROC_SLIP_MW;
									slipindexlist.push_back(pNormalPoints[k]); 
								}
							}// end of for(k)
							// 记录周跳弧段的左右端点
							size_t slipCounts = slipindexlist.size();
							size_t *pSubsection_left  = new size_t [slipCounts + 1];
							size_t *pSubsection_right = new size_t [slipCounts + 1];
							if(slipCounts > 0)
							{
								pSubsection_left[0] = pNormalPoints[0];	
								for(size_t k = 0; k < slipCounts; k++)
								{
									pSubsection_right[k]    = slipindexlist[k] - 1;	// 周跳前一个点未必是 NormalPoints ,后续处理需重新挑出 NormalPoints 点列
									pSubsection_left[k + 1] = slipindexlist[k];
								}
								pSubsection_right[slipCounts] = pNormalPoints[nCounts - 1];
							}
							else
							{
								pSubsection_left[0]  = pNormalPoints[0];
								pSubsection_right[0] = pNormalPoints[nCounts - 1];
							}
							//size_t slipCounts = 0;
							//size_t *pSubsection_left  = new size_t [slipCounts + 1];
							//size_t *pSubsection_right = new size_t [slipCounts + 1];
							//pSubsection_left[0]  = pNormalPoints[0];
							//pSubsection_right[0] = pNormalPoints[nCounts - 1];
							int ArcMW = 0;
							// 在 M-W 探测无周跳的弧段[pSubsection_left, pSubsection_right]内，利用 L1_L2 历元差进行野值及小周跳探测
							for(size_t k = 0; k < slipCounts + 1; k++)
							{
								// 进入 L1_L2 历元差判断环节,2013/04/30 修改
								vector<size_t> pSubNormalPoints;
								pSubNormalPoints.clear();
								// 提取正常数据点链表
								for(size_t j = pSubsection_left[k]; j <= pSubsection_right[k]; j++)
								{
									if(pSlip[j] == OBSPREPROC_NORMAL || pSlip[j] == OBSPREPROC_SLIP_MW)
										pSubNormalPoints.push_back(j);
								}
								// 若 pSubNormalPoints 数据量不足则标为 OBSPREPROC_OUTLIER_COUNT， 2013-05-21修改
								// 为准确统计周跳信息，M-W 短弧段暂不标记为 OBSPREPROC_OUTLIER_COUNT，2013-09-08修改
								size_t SubnCounts = pSubNormalPoints.size();
								if(SubnCounts <= m_PreprocessorDefine.min_arcpointcount)
								{
									for(size_t ks = 0; ks < SubnCounts; ks++)
									{
										pSlip[pSubNormalPoints[ks]] = OBSPREPROC_OUTLIER_COUNT;
									}
									continue;
								}
								ArcMW++;
								// 利用滑动多项式拟合消除电离层变化趋势项的影响，计算 L1-L2 历元差 pD_L1_L2 (2013/12/5)
								double *pSub_EpochTime = new double[SubnCounts];		// 子弧段观测时刻序列
								double *pL1_L2         = new double[SubnCounts];		// 子弧段L1-L2序列
								double *pL1_L2_fit     = new double[SubnCounts];
								double *pD_L1_L2       = new double[SubnCounts - 1];		// 子弧段DL1-L2序列
								double *pD_L1_L2_fit   = new double[SubnCounts - 1];		// 子弧段DL1-L2的fit序列
								double *Res_pD_L1_L2   = new double[SubnCounts - 1];		// 子弧段DL1-L2拟合残差序列
								for(int i = 0; i < int(SubnCounts); i++)
								{
									pSub_EpochTime[i] = pEpochTime[pSubNormalPoints[i]];
									pL1_L2[i] = L1_L2[pSubNormalPoints[i]];
								}
								// 初始化pL1_L2_fit
								pL1_L2_fit[0] = pL1_L2[0];
								for(int i = 0; i < int(SubnCounts) - 1; i++)
								{
									pD_L1_L2[i] = pL1_L2[i + 1] - pL1_L2[i];
								}
								size_t nBegin = 0;		// 加窗fit输出的左端点
								size_t nEnd   = 0;		// 加窗fit输出的右端点
								size_t nBegin_fit = 0;  // 区间fit的左端点
								size_t nEnd_fit   = 0;	// 区间fit的右端点
								//bool bOnFit = true;		// 是否进行多项式拟合
								//int nSubIntvel = 0;		// 暂时记录小区间数，便于调试
								//bool bOndebug = true;
								// 滑窗拟合 pL1_L2 序列
								while(1)
								{
									//// debug...2014/4/20，鞠 冰
									//if(editedObsSat.Id == 14 && pSub_EpochTime[nBegin] == 82230 && bOndebug)
									//{
									//	bOndebug = false;
									//	getchar();
									//}
									if(nEnd == SubnCounts - 1)
									{
										goto PolyFit;
									}
									else
									{
										if(pSub_EpochTime[nEnd + 1] - pSub_EpochTime[nBegin] <= m_PreprocessorDefine.nwidth_L1_L2)
										{
											nEnd++;
											continue;
										}
										else
										{
											// 处理弧段末尾处的点，若剩余点数少于 m_PreprocessorDefine.nwidth_L1_L2 / 2 则并入倒数第二个区间一起拟合（2013-12-16）
											if(pSub_EpochTime[SubnCounts - 1] - pSub_EpochTime[nEnd] <= m_PreprocessorDefine.nwidth_L1_L2 / 2)
											{
												nEnd = SubnCounts - 1;
											}
											// 处理 T(nBegin + 1) - T(nBegin) > m_PreprocessorDefine.nwidth_L1_L2 的情况，否则进入死循环（2014-03-26）
											if(nEnd == nBegin)
											{
												nBegin = nBegin + 1;
												nEnd   = nBegin;
												pL1_L2_fit[nBegin] = pL1_L2[nBegin];
												// 直接将 ppSubNormalPoints[nBegin] 点标为L1-L2周跳
												pSlip[pSubNormalPoints[nBegin]] = OBSPREPROC_SLIP_L1_L2;
												continue;
											}
											goto PolyFit;
										}
									}
									PolyFit:
									{
										// 确定拟合多项式的区间左右端点
										nBegin_fit = nBegin;
										nEnd_fit   = nEnd;
										// 多项式fit的左端点
										while(1)
										{
											if(nBegin_fit == 0)
											{
												break;
											}
											else
											{
												if(pSub_EpochTime[nBegin] - pSub_EpochTime[nBegin_fit - 1] <= m_PreprocessorDefine.extendwidth_L1_L2)
												{
													nBegin_fit--;
													continue;
												}
												else
												{
													break;
												}
											}
										}
										// 多项式fit的右端点
										while(1)
										{
											if(nEnd_fit == SubnCounts - 1)
											{
												break;
											}
											else
											{
												if(pSub_EpochTime[nEnd_fit + 1] - pSub_EpochTime[nEnd] <= m_PreprocessorDefine.extendwidth_L1_L2)
												{
													nEnd_fit++;
													continue;
												}
												else
												{
													break;
												}
											}
										}
										// 窗口内对L1-L2历元差数据进行多项式拟合(消除电离层趋势项)
										//RobustPolyFitL1_L2(pSub_EpochTime + nBegin_fit, pL1_L2 + nBegin_fit, pD_L1_L2_fit + nBegin, Res_pD_L1_L2 + nBegin, nEnd_fit - nBegin_fit + 1, nBegin - nBegin_fit, nEnd - nBegin, m_PreprocessorDefine.order_L1_L2);
										if(!RobustPolyFitL1_L2(pSub_EpochTime + nBegin_fit, pL1_L2 + nBegin_fit, pL1_L2_fit + nBegin + 1, int(nEnd_fit - nBegin_fit + 1), int(nBegin - nBegin_fit + 1), int(nEnd - nBegin), pL1_L2_fit[nBegin], m_PreprocessorDefine.order_L1_L2))
										{
											// (a)如果参与拟合点数过少，则有理由认为该弧段有较大的概率存在周跳，
											// 策略：整段数据剔除，标记为 OBSPREPROC_OUTLIER_L1_L2，下一点标记为周跳 OBSPREPROC_SLIP_L1_L2；
											// (b)如果拟合残差过大，逐点剔除后剩余拟合点数不足，也认为该弧段有较大的概率存在周跳，
											// 策略：整段数据剔除，标记为 OBSPREPROC_OUTLIER_L1_L2，下一点标记为周跳 OBSPREPROC_SLIP_L1_L2。
											for(size_t n_excute = nBegin; n_excute < nEnd; n_excute++)
											{
												pSlip[pSubNormalPoints[n_excute]] = OBSPREPROC_OUTLIER_L1_L2;
											}
											pSlip[pSubNormalPoints[nEnd]] = OBSPREPROC_SLIP_L1_L2;
											if(nEnd == int(SubnCounts) - 1)
											{
												pSlip[pSubNormalPoints[nEnd]] = OBSPREPROC_OUTLIER_L1_L2;
											}
											//getchar(); // debug

										}
										// to do...
										if(nEnd == int(SubnCounts) - 1)
											break;
										else
										{
											nBegin = nEnd;
											nEnd = nBegin;
											continue;
										}
									}// end of PolyFit
								}// end of while
								if(bOutTempFile == true)
								{
									for(int i = 0; i < int(SubnCounts); i++)
									{
										// 输出 M-W 弧段
										fprintf(pfile4, "%02d %02d %s %12.2f %16.4f %16.4f %16.4f %16.4f\n",
											editedObsSat.Id,
											ArcMW + ArcN,
											(T0 + pSub_EpochTime[i]).toString().c_str(),
											pSub_EpochTime[i],
											editedObsSat.editedObs.find(T0 + pSub_EpochTime[i])->second.Elevation,
											pL1_L2[i],
											pL1_L2_fit[i], 
											pL1_L2[i] - pL1_L2_fit[i]);
									}
								}
								// 计算 L1-L2 历元差序列的拟合残差
								for(int i = 0; i < int(SubnCounts) - 1; i++)
								{
									L1_L2_fit[pSubNormalPoints[i]] = pL1_L2_fit[i];
									Res_pD_L1_L2[i] = (pL1_L2[i + 1] - pL1_L2_fit[i + 1]) - (pL1_L2[i] - pL1_L2_fit[i]);
								}
								L1_L2_fit[pSubNormalPoints[SubnCounts - 1]] = pL1_L2_fit[SubnCounts - 1];
								// 根据 Res_pD_L1_L2 拟合残差进行相位数据野值拣择(2013/12/26)
								size_t L1_L2_outlerCounts = 0;		// 弧段[pSubsection_left, pSubsection_right]内野值点个数
								for(size_t ks = 0; ks < SubnCounts - 2; ks++)
								{
									if((fabs(Res_pD_L1_L2[ks]) >= m_PreprocessorDefine.threshold_slipsize_L1_L2 && fabs(Res_pD_L1_L2[ks + 1]) >= m_PreprocessorDefine.threshold_slipsize_L1_L2)
									 &&(pSlip[pSubNormalPoints[ks + 1]] == OBSPREPROC_NORMAL))
									{
										pSlip[pSubNormalPoints[ks + 1]] = OBSPREPROC_OUTLIER_L1_L2;
										L1_L2_outlerCounts++;
									}
								}
								// 处理首尾两端点: pSubNormalPoints[0] 及 pSubNormalPoints[SubnCounts - 1]
								// 需特别注意：起始点在 L1 - L2 判决环节可能已经标记过，此时要谨慎移动标记信息！2014/4/6，鞠 冰
								if(pSlip[pSubNormalPoints[0]] != OBSPREPROC_OUTLIER_L1_L2)
								{
									if(pSlip[pSubNormalPoints[1]] == OBSPREPROC_OUTLIER_L1_L2
									  || fabs(Res_pD_L1_L2[0]) > m_PreprocessorDefine.threshold_slipsize_L1_L2)
									{
										for(size_t s_i = 1; s_i < pSubNormalPoints.size(); s_i++)
										{
											if(pSlip[pSubNormalPoints[s_i]] != OBSPREPROC_OUTLIER_L1_L2)
											{
												pSlip[pSubNormalPoints[s_i]] = pSlip[pSubNormalPoints[0]];	// 第一个正常点的标记信息需要后移，2014/4/5，鞠 冰
												break;
											}
										}
										pSlip[pSubNormalPoints[0]] = OBSPREPROC_OUTLIER_L1_L2;
										L1_L2_outlerCounts++;
									}
								}
								if(pSlip[pSubNormalPoints[SubnCounts - 2]] == OBSPREPROC_OUTLIER_L1_L2
								 ||fabs(Res_pD_L1_L2[SubnCounts - 2]) > m_PreprocessorDefine.threshold_slipsize_L1_L2)
								{
									pSlip[pSubNormalPoints[SubnCounts - 1]] = OBSPREPROC_OUTLIER_L1_L2;
									L1_L2_outlerCounts++;
								}
								// 若有 OBSPREPROC_OUTLIER_L1_L2 类型的野值，则进行连续弧段的长度判断，2013/6/1
								if(L1_L2_outlerCounts > 0)
								{
									size_t kk = 0;
									while(kk < pSubNormalPoints.size())
									{
										if(pSlip[pSubNormalPoints[kk]] == OBSPREPROC_NORMAL  ||
										   pSlip[pSubNormalPoints[kk]] == OBSPREPROC_SLIP_MW ||
										   pSlip[pSubNormalPoints[kk]] == OBSPREPROC_SLIP_L1_L2)	// 避免删掉前面已经判过的 OBSPREPROC_SLIP_L1_L2 (2014/4/4，鞠 冰)
											kk = kk + 1;
										else
											pSubNormalPoints.erase(pSubNormalPoints.begin() + kk);
									}		
									SubnCounts = pSubNormalPoints.size();
									if(SubnCounts <= m_PreprocessorDefine.min_arcpointcount)
									{	
										for(size_t ks = 0; ks < SubnCounts; ks++)
										{
											pSlip[pSubNormalPoints[ks]] = OBSPREPROC_OUTLIER_COUNT;
										}
										delete []pL1_L2;
										delete []pL1_L2_fit;
										delete []pSub_EpochTime;
										delete []pD_L1_L2;
										delete []pD_L1_L2_fit;
										delete []Res_pD_L1_L2;
										continue;
									}
								}
								// 根据 Res_pD_L1_L2 拟合残差进行相位数据周跳探测(2013/12/26)
								vector<size_t> slipindexlistL1_L2;	// 弧段[pSubsection_left, pSubsection_right]内周跳点链表
								double tempLeft  = 0.0;
								double tempRight = 0.0;
								for(size_t ks = 0; ks < SubnCounts - 2; ks++)
								{
									tempLeft  = (L1_L2[pSubNormalPoints[ks + 1]] - L1_L2_fit[pSubNormalPoints[ks + 1]]) - (L1_L2[pSubNormalPoints[ks]] - L1_L2_fit[pSubNormalPoints[ks]]);
									tempRight = (L1_L2[pSubNormalPoints[ks + 2]] - L1_L2_fit[pSubNormalPoints[ks + 2]]) - (L1_L2[pSubNormalPoints[ks + 1]] - L1_L2_fit[pSubNormalPoints[ks + 1]]);
									if((fabs(tempLeft) >= m_PreprocessorDefine.threshold_slipsize_L1_L2 && fabs(tempRight) < m_PreprocessorDefine.threshold_slipsize_L1_L2)
									|| pSlip[pSubNormalPoints[ks + 1]] == OBSPREPROC_SLIP_L1_L2)
									{
										pSlip[pSubNormalPoints[ks + 1]] = OBSPREPROC_SLIP_L1_L2;
										slipindexlistL1_L2.push_back(pSubNormalPoints[ks + 1]);
									}
									
								}
								// 若有 OBSPREPROC_SLIP_L1_L2 类型的周跳，则进行连续弧段的长度判断，2013/5/28
								size_t L1_L2slipCounts = slipindexlistL1_L2.size();
								if(L1_L2slipCounts > 0)
								{
									// 记录 L1-L2 周跳弧段的左右端点
									size_t *pL1_L2_left  = new size_t [L1_L2slipCounts + 1];
									size_t *pL1_L2_right = new size_t [L1_L2slipCounts + 1];
									pL1_L2_left[0] = pSubNormalPoints[0];	
									for(size_t ks = 0; ks < L1_L2slipCounts; ks++)
									{
										pL1_L2_right[ks]    = slipindexlistL1_L2[ks] - 1;	// 周跳前一个点未必是 pSubNormalPoints ,后续处理需重新挑出 pSubNormalPoints 点列
										pL1_L2_left[ks + 1] = slipindexlistL1_L2[ks];
									}
									pL1_L2_right[L1_L2slipCounts] = pSubNormalPoints[SubnCounts - 1];
									for(size_t i = 0; i < L1_L2slipCounts + 1; i++)
									{
										vector<size_t> pL1_L2NormalPoints;
										pL1_L2NormalPoints.clear();
										for(size_t kk = pL1_L2_left[i]; kk <= pL1_L2_right[i]; kk++)
										{
											if(pSlip[kk] == OBSPREPROC_NORMAL || pSlip[kk] == OBSPREPROC_SLIP_MW || pSlip[kk] == OBSPREPROC_SLIP_L1_L2)
											{
												pL1_L2NormalPoints.push_back(kk);
											}
										}
										size_t L1_L2nCounts = pL1_L2NormalPoints.size();
										if(L1_L2nCounts <= m_PreprocessorDefine.min_arcpointcount)
										{
											for(size_t kk = 0; kk < L1_L2nCounts; kk++)
											{
												pSlip[pL1_L2NormalPoints[kk]] = OBSPREPROC_OUTLIER_COUNT; 
											}
										}
										else
										{
											// 保持原有标记
										}
									}
									delete[] pL1_L2_left;
									delete[] pL1_L2_right;
								}
								// 释放内存空间
								delete []pL1_L2;
								delete []pL1_L2_fit;
								delete []pSub_EpochTime;
								delete []pD_L1_L2;
								delete []pD_L1_L2_fit;
								delete []Res_pD_L1_L2;
							}// end of for(所有 M-W 连续弧段的循环)
							delete[] pSubsection_left;
							delete[] pSubsection_right;
							ArcN = ArcN + ArcMW;
						}// end of else(剔除 OBSPREPROC_OUTLIER_MW 后，M-W弧段数据个数足够)
						// 将第一个非野值点, 更新标记为 OBSPREPROC_NEWARCBEGIN
						for(size_t k = arc_begin; k <= arc_end; k++)
						{
							if(pSlip[k] == OBSPREPROC_NORMAL || pSlip[k] == OBSPREPROC_SLIP_MW || pSlip[k] == OBSPREPROC_SLIP_L1_L2)
							{
								pSlip[k] = OBSPREPROC_NEWARCBEGIN;
								break;
							}
						}
					}// end of else(newArc)
					// 写文件
					if(bOutTempFile == true)
					{
						fprintf(pfile, "PRN %2d -> Arc: %2d\n", editedObsSat.Id, ArcCounts);
						fprintf(pfile, "%8s %8s %18s %18s %8s\n",
							"T",
							"E",
							"M-W",
							"L1-L2",
							"Marks");
						for(size_t k = arc_begin; k <= arc_end; k++)
						{
							// 仅输出正常弧段数据（2013-05-10）
							/*if(pSlip[k] == OBSPREPROC_NORMAL || pSlip[k] == OBSPREPROC_NEWARCBEGIN ||
							pSlip[k] == OBSPREPROC_SLIP_MW || pSlip[k] == OBSPREPROC_SLIP_L1_L2)*/
							// 输出所有不含空白数据和伪码野值的历元
							if(pSlip[k] != TYPE_EDITEDMARK_OUTLIER)
							{
								Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.find(T0 + pEpochTime[k]);
								fprintf(pfile,"%8.2f %8.2f %18.3f %18.3f %8d\n",
									pEpochTime[k],
									it->second.Elevation,
									M_W[k],
									L1_L2[k],
									pSlip[k]);
							}
						}
					}
					if(arc_end + 1 >= num)
						break;
					else
					{
						arc_begin = arc_end + 1;
						arc_end   = arc_begin;
						continue;
					}
				}// end of newArc;
			}// end of while()
			if(bOutTempFile == true)
			{
				fclose(pfile);
				//fclose(pfile1);
				//fclose(pfile3);
				fclose(pfile4);
				//fclose(pfile5);
				//fclose(pfile6);
			}
			/* Step4：标记相位数据野值及周跳检测信息 */
			k = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); it++)
			{
				if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[k]);
					it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[k]);
				}
				if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[k]);
					it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[k]);
				}
				k = k + 1;
			}
			// 释放内存空间
			delete[] L_W;
			delete[] P_N;
			delete[] M_W;
			delete[] pEpochTime;
			delete[] L1_L2;
			delete[] L1_L2_fit;
			delete[] pSlip;
			return true;
		}
		// 子程序名称： mainFuncDualFreObsPreproc_GPS   
		// 功能：GPS观测数据预处理主程序
		// 变量类型：EditedObsFile :  预处理后的观测数据结构
		// 输入：editedObsFile(数据成员为空)
		//       bOutTempFile(临时文件输出开关)
		// 输出：editedObsFile
		// 语言：C++
		// 创建者：鞠 冰
		// 创建时间：2012/9/17
		// 版本时间：2012/9/12
		// 修改记录：1、只对 P1(C1)、P2、L1、L2 四类观测数据进行预处理，去掉其它数据的标记信息(2012/10/19)
		//			 2、低仰角数据不参与码的野值检测和相位数据的野值检测和周跳探测(2012/10/20)
		//			 3、添加预处理数据目录创建(2013/9/13)
		//			 4、添加临时文件输出开关(2014/3/26)
		//           5、用于BDS数据处理，2014/7/1,刘俊宏
		// 备注： 原程序名mainFuncObsPreproc
		bool BDObsPreproc::mainFuncDualFreObsPreproc_GPS(Rinex2_1_EditedObsFile &editedObsFile, bool bOutTempFile)
		{
			// 判断观测数据结构和广播星历数据结构是否为空(测站坐标？？？2012-09-18)
			if(m_navFile.isEmpty() && m_sp3File.isEmpty())
			{
				printf("无星历数据!，请确认!\n");
				return false;
			}
			if(m_obsFile.isEmpty())
			{
				if(m_obsFile.isEmpty())
					printf("无观测数据!，请确认!\n");
				return false;
			}
			// 获取观测数据索引
			int index_C1 = -1, index_P1 = -1, index_P2 = -1, index_L1 = -1, index_L2 = -1;
			for(int i = 0; i < m_obsFile.m_header.byObsTypes; i++)
			{
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_C1)
					index_C1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					index_P1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					index_P2 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					index_L1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					index_L2 = i;
			}
			// 防止文件头信息：# / TYPES OF OBSERV 与实际数据不符，鞠冰(2014/4/3)
			bool bValid_P1 = false;
			if(index_P1 != -1)
			{
				for(size_t s_i = 0; s_i != m_obsFile.m_data.size(); s_i++)
				{
					for(Rinex2_1_SatMap::iterator it = m_obsFile.m_data[s_i].obs.begin(); it != m_obsFile.m_data[s_i].obs.end(); it++)
					{
						if(it->second[index_P1].data != DBL_MAX)
						{
							bValid_P1 = true;	// 只要有一颗卫星的数据不为空，则认为该类型数据有效
							break;
						}
					}
					if(bValid_P1)
					{
						break;
					}
				}
				if(!bValid_P1)
				{
					index_P1 = -1;		// P1 数据无效
				}
			}
			// 判断数据类型是否充足，2014/4/6，鞠 冰
			if(index_P1 == -1 && index_C1 != -1)
			{
				index_P1 = index_C1;	// 暂时将 C1 直接赋给 P1 (2013/6/26)
			}
			if(index_P1 == -1 || index_P2 == -1 || index_L1 == -1 || index_L2 == -1)
			{
				printf("观测数据类型不足！跳过预处理环节！\n");
				return false;
			}
			// 仅对观测数据中L1、L2、P1、P2数据进行预处理(2012-10-19)
			int index[4];
			index[0] = index_L1;
			index[1] = index_L2;
			index[2] = index_P1;
			index[3] = index_P2;
			// 调整数据结构(按卫星列表排列)，方便预处理
			DayTime T0 = m_obsFile.m_data[0].t;		// 首次观测历元(2013/09/07)
			//vector<Rinex2_1_EditedObsSat>   editedObsSatList;
			//getEditedObsSatList(editedObsSatList);//
			
			vector<Rinex2_1_EditedObsSat>   editedObsSatlist_raw,editedObsSatList; //暂时去除某些卫星,20140929			
			getEditedObsSatList(editedObsSatlist_raw);
			for(size_t s_j = 0; s_j < editedObsSatlist_raw.size(); s_j++)
			{
				//if(editedObsSatlist_raw[s_j].Id <= 14 && editedObsSatlist_raw[s_j].Id !=4)	//30号卫星不可用	
				if(editedObsSatlist_raw[s_j].Id <= 14)	//30号卫星不可用
					editedObsSatList.push_back(editedObsSatlist_raw[s_j]);
			}
			//****** Simulation ********* 在某颗卫星的某个频点上添加周跳 ***********
			//**********************************************************************
			FILE *pfile;
			if(bOutTempFile == true)
			{
				_mkdir(m_strPreprocFilePath.c_str());
				// 测试程序，临时写文件
				char szEditedObsSatListFileName[100];
				char szCodePreprocFileName[100];
				char szPhasePreprocFileName[100];
				char szL1_L2FileName[100];
				char szStationName[4 + 1];
				for(int k = 0; k < 4; k++)
				{
					szStationName[k] = m_obsFile.m_header.szMarkName[k];
				}
				szStationName[4] = '\0';
				sprintf(szEditedObsSatListFileName, "%s\\%s_editedObsSatList.dat", m_strPreprocFilePath.c_str(), szStationName);
				sprintf(szCodePreprocFileName, "%s\\%s_CodePreproc.dat",   m_strPreprocFilePath.c_str(), szStationName);
				sprintf(szPhasePreprocFileName, "%s\\%s_PhasePreproc.dat", m_strPreprocFilePath.c_str(), szStationName);
				sprintf(szL1_L2FileName, "%s\\%s_L1_L2.dat", m_strPreprocFilePath.c_str(), szStationName);
				pfile  = fopen(szEditedObsSatListFileName,"w+");
				FILE *pfile1 = fopen(szCodePreprocFileName,"w+");
				FILE *pfile2 = fopen(szPhasePreprocFileName,"w+");
				FILE *pfile3 = fopen(szL1_L2FileName,"w+");
				fclose(pfile1);
				fclose(pfile2);
				fclose(pfile3);
			}
			//**********************************************************************
			/* 逐颗卫星遍历处理 */
			for(size_t s = 0; s < editedObsSatList.size(); s++)
			{
				//printf("PRN %2d ......", editedObsSatList[s].Id);
				vector<DayTime> erase_EpochList;					// 低仰角历元列表
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatList[s].editedObs.begin(); it != editedObsSatList[s].editedObs.end(); it++)
				{
					// Step1: 将L1、L2、P1、P2数据中缺失的数据(DBL_MAX)标记为野值：OBSPREPROC_OUTLIER_BLANK（2012-09-21）
					for(int k = 0; k < 4; k++)
					{
						if(it->second.obsTypeList[index[k]].obs.data == 0.0)	// 处理实际观测数据时发现有“.000”数据(2012-12-24)
						{
							it->second.obsTypeList[index[k]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_ZERO);
							it->second.obsTypeList[index[k]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_ZERO);
						}
						if(it->second.obsTypeList[index[k]].obs.data == DBL_MAX)
						{
							it->second.obsTypeList[index[k]].obs.data = 0.0;//暂时在此处重新赋值，是否更改Rinex文件解析时DBL_MAX的赋值待商榷(2012-10-18)
							it->second.obsTypeList[index[k]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_BLANK);
							it->second.obsTypeList[index[k]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_BLANK);
						}
					}
					// Step2：计算每颗可视卫星的仰角及方位角，将低仰角数据标记为野值（2012-10-08）
					//获取星历时需要考虑星历缺失的情况，此时不用计算仰角，直接赋值为0（2014-07-08，刘俊宏）
					bool      posflag = false;
					POS3D ECFposSat;	// 卫星在地固系下的坐标
					if(!m_navFile.isEmpty())
					{
						POSCLK posclk;
						if(m_navFile.getEphemeris(it->first, editedObsSatList[s].Id, posclk))
							posflag = true;
						ECFposSat = posclk.getPos();
					}
					else
					{
						SP3Datum sp3Datum;
						if(m_sp3File.getEphemeris(it->first, editedObsSatList[s].Id, sp3Datum,9 ,'C'))
							posflag = true;						
						ECFposSat = sp3Datum.pos;
					}
					double Azimuth   = 0.0;						// 方位角(单位：度)
					double Elevation = 0.0;						// 俯仰角(单位：度)
					if(posflag)
					{
						ENU ENUposSat;								// 卫星在测站坐标系下的坐标
						TimeCoordConvert::ECF2ENU(m_posStation, ECFposSat, ENUposSat);					
						// 计算仰角(单位：度)
						Elevation = atan(ENUposSat.U/sqrt(ENUposSat.E*ENUposSat.E + ENUposSat.N*ENUposSat.N))*180/PI;		
						// 计算方位角(单位：度)
						Azimuth   = atan2(ENUposSat.E, ENUposSat.N)*180/PI;
						if(Azimuth < 0)
						{
							Azimuth = Azimuth + 360;				// 保证方位角在[0, 360)之间
						}
						POS3D p_station = vectorNormal(m_posStation);
						POS3D p_sat = vectorNormal(ECFposSat - m_posStation);					
						p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); // 20150608, 考虑到地球扁率的影响, 卫星仰角的计算进行了修正, 谷德峰
						p_station = vectorNormal(p_station);					
						Elevation = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;
						it->second.Elevation = Elevation;
						it->second.Azimuth   = Azimuth;
					}
					else
					{
						it->second.Elevation = 0.0;
						it->second.Azimuth   = 0.0;
					}

					// 将低仰角数据标记为野值：OBSPREPROC_OUTLIER_ELEVATION
					double min_elevation = m_PreprocessorDefine.min_elevation;
					if(editedObsSatList[s].Id <= 5) //GEO 卫星10度以下仰角的观测数据较差，例如brst测站的C05卫星
						min_elevation = 10;         //对于定轨，强制使用大于10度的观测数据，20140816，刘俊宏
					if(Elevation <= min_elevation)
					{
						erase_EpochList.push_back(it->first);
						for(int k = 0; k < 4; k++)
						{// 判断观测数据是否已被标记为野值，若已被标记则不再重复标记（2012-10-12）
							if(it->second.obsTypeList[index[k]].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[index[k]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_ELEVATION);
								it->second.obsTypeList[index[k]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_ELEVATION);
							}
						}
					}
				}//end of for(iterator it)
				// 剔除低仰角数据,提高计算效率
				Rinex2_1_EditedObsSat temp_editedObsSat = editedObsSatList[s];
				// 该颗卫星的首次观测历元
				//DayTime T0 = editedObsSatList[s].editedObs.begin()->first;
				if(!erase_EpochList.empty())
				{
					for(size_t k = 0; k < erase_EpochList.size(); k++)
					{
						temp_editedObsSat.editedObs.erase(erase_EpochList[k]);
					}
				}
				// Step3: 利用电离层组合及 Vondark 滤波探测伪码野值
				detectCodeOutlier_ionosphere_GPS(index_P1, index_P2, BD_FREQUENCE_L1, BD_FREQUENCE_L2, T0, temp_editedObsSat, bOutTempFile);
				// Step4: 利用 M-W 组合及 L1-L2 组合探测相位周跳及野值
				detectPhaseSlip_GPS(index_P1, index_P2, index_L1, index_L2, BD_FREQUENCE_L1, BD_FREQUENCE_L2, T0, temp_editedObsSat, bOutTempFile);
				// 将低仰角数据添加至预处理后的数据结构
				if(!erase_EpochList.empty())
				{
					for(size_t k = 0; k < erase_EpochList.size(); k++)
					{
						Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatList[s].editedObs.find(erase_EpochList[k]);
						temp_editedObsSat.editedObs.insert(Rinex2_1_EditedObsEpochMap::value_type(it->first,it->second));
					}
				}
				editedObsSatList[s] = temp_editedObsSat;
				//**********************************************************************
				if(bOutTempFile == true)
				{
					// 写临时文件
					fprintf(pfile, "PRN %2d\n", editedObsSatList[s].Id);
					fprintf(pfile, "%8s %8s %14s   %14s   %14s   %14s\n",
						"T",
						"E",
						obsId2String(TYPE_OBS_L1).c_str(),
						obsId2String(TYPE_OBS_L2).c_str(),
						obsId2String(TYPE_OBS_P1).c_str(),
						obsId2String(TYPE_OBS_P2).c_str());
					for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatList[s].editedObs.begin(); it != editedObsSatList[s].editedObs.end(); it++)
					{
						fprintf(pfile, "%8.2f %8.2f ",it->first - T0, it->second.Elevation);
						for(int k = 0;k < 4; k++)
						{
							fprintf(pfile, "%14.3f %1d%1d",
								it->second.obsTypeList[index[k]].obs.data, 
								it->second.obsTypeList[index[k]].byEditedMark1, 
								it->second.obsTypeList[index[k]].byEditedMark2);
						}
						fprintf(pfile, "\n");
					}
				}
				//**********************************************************************
			}// end of for(size_t s)
			if(bOutTempFile == true)
			{
				fclose(pfile);
			}
			// 调整数据结构(按历元排列)
			vector<Rinex2_1_EditedObsEpoch>   editedObsEpochList;
			datalist_sat2epoch(editedObsSatList, editedObsEpochList);
			editedObsFile.m_header = m_obsFile.m_header;
			editedObsFile.m_data   = editedObsEpochList;
			return true;
		}
		//   子程序名称： RobustPolyFitL1_L2   
		//   作用：差分数据拟合相位电离层组合的趋势项(不含常数项)
		//   类型：x		 :  历元时间序列（n 维）
		//         y    	 :  相位电离层组合序列（n 维）
		//		   y_fit     :  相位电离层组合拟合序列（n_out 维）
		//         n         :  用于拟合的数据个数
		//         offset    :  输出数据的起始位置
		//         n_out     :  输出数据的个数
		//         N0        :  常数项
		//         m		 :  多项式阶数, 2 =< m <= n, 默认取 4
		//   输入：x, y, n, offset, n_out, m
		//   输出：y_fit
		//   语言：C++
		//   创建者：鞠冰
		//   创建时间：2013/12/25
		//   版本时间：2013/12/15
		//   修改记录：1、抗差算法采用逐点剔除野值，2014/4/4，鞠 冰
		//             2、综合考虑 OC 残差及 L1-L2 历元差进行野值拣择，2014/4/5，鞠 冰
		//   其它：
		bool BDObsPreproc::RobustPolyFitL1_L2(double x[], double y[], double y_fit[], int n, int offset, int n_out, double N0, int m)
		{
			// n < m_PreprocessorDefine.nwidth_L1_L2 / 2, n < 2 * (m - 1) 即认为数据个数过少，2014/4/19，鞠 冰
			int minPnts =  int(m_PreprocessorDefine.nwidth_L1_L2 / 30.0) / 2;
			if(m < 1 || n < 2*(m - 1) || n < minPnts)
			{
				for(int i = 0; i < n_out; i++)
				{
					y_fit[i] = y[offset + i];
				}
				//printf("[RobustPolyFitL1_L2]用于拟合的数据个数过少！\n");
				return false;
			}
			Matrix matDY0(n - 1, 1);		// L1-L2历元差序列
			for(int i = 0; i < n - 1; i++)
			{
				matDY0.SetElement(i, 0, (y[i + 1] - y[i]));
			}
			// 鲁棒权值
			double *w     = new double [n - 1];
			double *w_new = new double [n - 1];
			for(int i = 0; i < n - 1; i++)
			{
				w[i]     = 1.0;
				w_new[i] = 1.0;
			}
			// 计算相对时间序列
			double *xx = new double [n];
			for(int i = 0; i < n; i++)
			{
				xx[i] = x[i] - x[0];
			}
			Matrix matC0(n - 1, m - 1);		// 拟合用设计矩阵(常数项差分后被消除，故只有 n - 1 行，m - 1 列)
			// 计算设计矩阵及L1-L2差分数据序列
			for(int i = 0; i < n - 1; i++)
			{
				for(int j = 0; j < m - 1; j++)
				{
					matC0.SetElement(i, j, (pow(xx[i + 1], j + 1) - pow(xx[i], j + 1)));
				}
			}
			int nLoop = 0;
			int nLoop_max = 6;			// 设置迭代次数阈值，防止无法收敛陷入死循环
			int nExclude = 0;			// 剔除的点数
			vector<int> fitPoints;		// 参与拟合的点集
			vector<double> fitDY0;		// 参与拟合的历元差
			while(1)
			{
				nLoop++;
				fitPoints.clear();
				fitDY0.clear();
				// 计算设计矩阵及L1-L2差分数据序列
				Matrix matC(n - 1, m - 1);
				Matrix matDY(n - 1, 1);
				for(int i = 0; i < n - 1; i++)
				{
					matDY.SetElement(i, 0, w[i] * (y[i + 1] - y[i]));
					for(int j = 0; j < m - 1; j++)
					{
						matC.SetElement(i, j, w[i] * (pow(xx[i + 1], j + 1) - pow(xx[i], j + 1)));
					}
				}
				Matrix matS = (matC.Transpose() * matC).Inv() * matC.Transpose() * matDY;
				Matrix matDY_Fit = matC0 * matS;
				// 根据 OC 残差及 L1-L2 历元差进行逐点野值剔除，否则由于较大的离差点会影响所有的点，导致将所有点剔除（2014/4/3，鞠 冰）
				double rms = 0.0;
				int kk = 0;
				double tempoc = 0.0;
				for(int i = 0; i < n - 1; i++)
				{
					//printf("%10.4f %10.4f\n", matDY0.GetElement(i,0), matDY_Fit.GetElement(i,0));
					if(w[i] == 1)
					{
						kk++;
						tempoc = matDY0.GetElement(i,0) - matDY_Fit.GetElement(i,0);
						rms += pow(tempoc, 2);
						fitPoints.push_back(i);
						int nEpoch = int((xx[i + 1] - xx[i]) / 30.0);
						fitDY0.push_back(fabs(matDY0.GetElement(i,0)) / nEpoch);
					}
				}
				// 首先判断参与拟合点的个数是否充足,至少2个点(2014/4/4，鞠 冰)
				if(kk < m || kk < minPnts)
				{
					for(int i = 0; i < n_out; i++)
					{
						y_fit[i] = y[offset + i];
					}
					//printf("[RobustPolyFitL1_L2]剩余数据个数过少！\n");
					delete []xx;
					delete []w;
					delete []w_new;
					return false;
				}
				rms = sqrt(rms / kk);
				bool bEqual = true;
				// 搜索最大历元差点
				int max_index  = -1;
				double max_DY0 = 0.0;
				for(int i = 0; i < int(fitPoints.size()); i++)
				{
					if(fabs(matDY0.GetElement(fitPoints[i],0) - matDY_Fit.GetElement(fitPoints[i],0)) >= min(3.0 * rms, m_PreprocessorDefine.threshold_slipsize_L1_L2))
					{
						if(fitDY0[i] > max_DY0)
						{
							max_DY0    = fitDY0[i];
							max_index  = fitPoints[i];
						}
					}
				}
				if(max_index != -1)
				{
					w_new[max_index] = 0.0;
					bEqual = false;
				}
				if(bEqual || nLoop > nLoop_max)
				{
					Matrix matT(n, m - 1);			// 计算fit值用设计矩阵
					for(int i = 0; i < n; i++)
					{
						for(int j = 0; j < m - 1; j++)
						{
							matT.SetElement(i, j, pow(xx[i], j + 1));
						}
					}
					Matrix matY_fit = matT * matS;
					//printf("%14.4f\n",N0);
					for(int i = 0; i < n_out; i++)
					{
						//y_fit[i] = matY_fit.GetElement(offset + i,0) + y[offset + 0];
						y_fit[i] = matY_fit.GetElement(offset + i,0) + N0;
						//printf("%14.4f\n",y_fit[i]);
					}
					//printf("\n");
					break;
				}
				else
				{
					memcpy(w, w_new, sizeof(double) * (n - 1));
				}
			}
			delete []xx;
			delete []w;
			delete []w_new;
			return true;
		}

		// 子程序名称： detectThrFreCodeOutlier   
		// 功能：三频消电离层组合探测伪码野值
		// 变量类型：    index_P1                                  : 观测类型P1索引
		//               index_P2                                  : 观测类型P2索引
		//               index_P5                                  : 观测类型P5索引
        //               editedObsSat                              : 输出的弧段结构数据
		//               bOutTempFile                              : 是否输出预处理信息
		// 输入：index_P1,index_P2,index_P5,editedObsSat,bOutTempFile
		// 输出：editedObsSat
		// 语言：C++
		// 创建者：刘俊宏
		// 创建时间：2013/3/23
		// 版本时间：2013/3/23
		// 修改记录：
		// 其它： 
		bool BDObsPreproc::detectThrFreCodeOutlier(int index_P1, int index_P2,int index_P5,Rinex2_1_EditedObsSat &editedObsSat,bool bOutTempFile)
		{
			//// 创建预处理目录
			//string folder = m_strObsFileName.substr(0, m_strObsFileName.find_last_of("\\"));
			//string obsFileName = m_strObsFileName.substr(m_strObsFileName.find_last_of("\\") + 1);
			//string obsFileName_noexp = obsFileName.substr(0, obsFileName.find_last_of("."));
			//// 创建预处理目录
			//string strPreprocFolder = folder + "\\Preproc";
			//char  TreFreCode_FileName[200];			
			//sprintf(TreFreCode_FileName, "%s\\%s_ThreFreCode.dat", strPreprocFolder.c_str(), obsFileName_noexp.c_str());			
			//FILE *pfile = fopen(TreFreCode_FileName,"a+");	
			FILE *pfile;
			if(bOutTempFile == true)
			{
				// 测试程序，临时写文件				
				char szStationName[4 + 1];
				for(int k = 0; k < 4; k++)
				{
					szStationName[k] = m_obsFile.m_header.szMarkName[k];
				}
				szStationName[4] = '\0';			
				char  TreFreCode_FileName[200];
				sprintf(TreFreCode_FileName, "%s\\%s_ThreFreCode.dat", m_strPreprocFilePath.c_str(), szStationName);
				pfile = fopen(TreFreCode_FileName,"a+");				
			}

			size_t nCount = editedObsSat.editedObs.size();
			if(nCount <= m_PreprocessorDefine.min_arcpointcount)  // 观测个数太少, 直接丢弃
			{				
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
				{
					//防止重复标记
					if(it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_P5].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P5].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_P5].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
				}
				return true;
			}
			double *pThrFreCodeCom = new double[nCount];            			
			int    *pOutlier       = new int   [nCount];		
			double *pEpochTime     = new double[nCount];
			Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSat.editedObs.begin();
			DayTime t0 = it0->first;  
			// 构造三频伪距组合 pThrFreCodeCom = coefficient_P1 * P1 - coefficient_P2 * P2 + coefficient_P5 * P5
			double coefficient_P1 = 1 / (1 - pow( BD_FREQUENCE_L2 / BD_FREQUENCE_L1, 2 )) -  
				                    1 / (1 - pow( BD_FREQUENCE_L5 / BD_FREQUENCE_L1, 2 ));
			double coefficient_P2 = 1 / (pow( BD_FREQUENCE_L1 / BD_FREQUENCE_L2, 2 ) - 1);
			double coefficient_P5 = 1 / (pow( BD_FREQUENCE_L1 / BD_FREQUENCE_L5, 2 ) - 1);
			int i = 0;			
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
			{				
				pEpochTime[i] = it->first - t0;
				Rinex2_1_EditedObsDatum  P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum  P2 = it->second.obsTypeList[index_P2];
				Rinex2_1_EditedObsDatum  P5 = it->second.obsTypeList[index_P5];
				pThrFreCodeCom[i] = coefficient_P1 * P1.obs.data - coefficient_P2 * P2.obs.data + coefficient_P5 * P5.obs.data;				
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P5.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
					pOutlier[i] = TYPE_EDITEDMARK_OUTLIER; // 接受先前野值判断结果, 以和其他野值判断方法相配合
				else
					pOutlier[i] = OBSPREPROC_NORMAL;
				i++;
			}
			// 暂时不分弧段，因为频间差基本恒定，短时间不会改变
			vector<size_t>    unknownPointlist;		
			// 第一步：根据电离层残差的阈值大小，直接进行野值判断，剔除一些大的野值
			for(size_t s_i = 0; s_i < nCount; s_i++)	
			{
				if(fabs(pThrFreCodeCom[s_i]) > m_PreprocessorDefine.max_thrfrecodecom) 																						
					pOutlier[s_i] = OBSPREPROC_OUTLIER_IONOMAXMIN; //电离层超差，直接标记为野值
				else
				{
					if(pOutlier[s_i] == OBSPREPROC_NORMAL)
						unknownPointlist.push_back(s_i);
				}
			}
			size_t nCount_points = unknownPointlist.size(); 
			// 第二步：判断正常点数据个数，正常点数据个数太少标记为野值						
			if(nCount_points <= int(m_PreprocessorDefine.min_arcpointcount))
			{				
				for(size_t s_i = 0; s_i < nCount_points; s_i++)					
					pOutlier[unknownPointlist[s_i]] = OBSPREPROC_OUTLIER_COUNT; // 正常点数据个数太少标记为野值					
			}
			// 第三步：用n*sigma准则剔除野值	
			else
			{				
				// 计算三频伪码组合的均值的方差
				double *pThrCode = new double[nCount_points];
				double *w        = new double[nCount_points];
				double mean = 0;
				double var  = 0;
				for(size_t s_j = 0; s_j < nCount_points; s_j ++)
					pThrCode[s_j] = pThrFreCodeCom[unknownPointlist[s_j]];
				double factor = 5;
				//if(editedObsSat.editedObs.begin()->second.Id <= 5) //GEO卫星野值处理时要适当放大sigma的倍数
				//	factor = 4;  
				RobustStatMean(pThrCode,w,(int)nCount_points,mean,var,factor);
				//printf("mean = %8.4f var = %8.4f\n",mean,var);
				for(size_t s_k = 0; s_k < nCount_points; s_k ++)
					if(w[s_k] == 1)
						pOutlier[unknownPointlist[s_k]] = OBSPREPROC_OUTLIER_IONOMAXMIN;
				delete pThrCode;
				delete w;

			}				
			i = 0;			
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
			{	
				if(it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(pOutlier[i]);
					it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(pOutlier[i]);					
				}
				if(it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(pOutlier[i]);
					it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(pOutlier[i]);					
				}
				if(it->second.obsTypeList[index_P5].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_P5].byEditedMark1 = obsPreprocInfo2EditedMark1(pOutlier[i]);
					it->second.obsTypeList[index_P5].byEditedMark2 = obsPreprocInfo2EditedMark2(pOutlier[i]);					
				}
				if(bOutTempFile == true)
				{
					GPST t = it->first;	
					GPST t0 = m_obsFile.m_data.front().t;
					t0.hour   = 0;
					t0.minute = 0;
					t0.second = 0;
					//// 写临时文件			
					if(it->second.obsTypeList[index_P1].obs.data != DBL_MAX
						&& it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
						&& it->second.obsTypeList[index_P5].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)				
					{
						//GPST t_epoch = it->first;
						fprintf(pfile,"%2d %s %8.2f %8.2f %14.4f\n",						 			            
										editedObsSat.Id,									
										t.toString().c_str(),
										(t - t0)/3600,	
										it->second.Elevation,
										pThrFreCodeCom[i]);
					}
				}
				i++;
			}
			if(bOutTempFile == true)
				fclose(pfile);			
			delete pOutlier;
			delete pEpochTime;
			delete pThrFreCodeCom;

			return  true;
		}
		//   子程序名称： detectThrFrePhaseSlip  
		//   作用： 构造三频相位组合量，利用三频相位组合量周跳探测       
		//   类型： index_L1                                       : 观测类型L1索引
		//          index_L2                                       : 观测类型L2索引
		//          index_L5                                       : 观测类型L5索引
		//          index_P1                                       : 观测类型P1索引
		//          index_P2                                       : 观测类型P2索引
		//          index_P5                                       : 观测类型P5索引
		//          editedObsSat                                   : 输出的弧段结构数据
		//          bOutTempFile                                   : 是否输出预处理信息
		//   输入： index_L1, index_L2, index_L5,editedObsSat
		//   输出： editedObsSat		
		//   语言： C++
		//   创建者：刘俊宏
		//   创建时间：2013/4/1
		//   版本时间：2013/4/1
		//   修改记录：2013/5/7 增加伪码相位一致性检验，北斗观测数据有钟跳现象导致伪码和相位数据不匹配
		//   其它： 1、2013/10/17 增加三频伪距相位GIF组合量和电离层组合，并修改原来的三频相位GIF组合观测量，使观测噪声进一步降低。
		//             使用三频伪距相位GIF组合，同样可以检验伪距和相位的一致性，因此不再使用M-W组合。
		//             当前使用的三个组合观测量使得周跳探测的理论更加完善，策略更加优化
		//          2、由于实际数据中经常出现漏警，因此将threshold_slipsize_LGIF = max(m_PreprocessorDefine.threshold_slipsize_LGIF,4 * var_DLGIF)
		//             改为threshold_slipsize_LGIF = max(m_PreprocessorDefine.threshold_slipsize_LGIF,3 * var_DLGIF)
		bool BDObsPreproc::detectThrFrePhaseSlip(int index_L1, int index_L2,int index_L5, int index_P1, int index_P2, int index_P5, Rinex2_1_EditedObsSat& editedObsSat,bool bOutTempFile)
		{
			//// 创建预处理目录
			//string folder = m_strObsFileName.substr(0, m_strObsFileName.find_last_of("\\"));
			//string obsFileName = m_strObsFileName.substr(m_strObsFileName.find_last_of("\\") + 1);
			//string obsFileName_noexp = obsFileName.substr(0, obsFileName.find_last_of("."));
			//// 创建预处理目录
			//string strPreprocFolder = folder + "\\Preproc";
			//char  TreFrePhase_FileName[200];			
			//sprintf(TreFrePhase_FileName, "%s\\%s_ThreFrePhase.dat", strPreprocFolder.c_str(), obsFileName_noexp.c_str());			
			//FILE *pfile = fopen(TreFrePhase_FileName,"a+");
			FILE *pfile;
			if(bOutTempFile == true)
			{
				// 测试程序，临时写文件				
				char szStationName[4 + 1];
				for(int k = 0; k < 4; k++)
				{
					szStationName[k] = m_obsFile.m_header.szMarkName[k];
				}
				szStationName[4] = '\0';		
				char  TreFrePhase_FileName[200];			
				sprintf(TreFrePhase_FileName, "%s\\%s_ThreFrePhase.dat", m_strPreprocFilePath.c_str(), szStationName);			
				pfile = fopen(TreFrePhase_FileName,"a+");							
			}

			size_t nCount           = editedObsSat.editedObs.size();
			//step 1: 整个卫星的观测数据太少，直接丢弃
			if(nCount <= m_PreprocessorDefine.min_arcpointcount)  
			{
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
				{	
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_L5].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L5].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_L5].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
					}	
				}
				return true;
			}
			//step 2: 构造周跳探测组合观测量，并记录先前伪码相位野值标记结果
			double *pEpochTime      = new double[nCount];					
			double *pThrFreCodePhaseGIFCom = new double[nCount]; // 三频伪距相位无几何距离、无电离层组合观测量(地位等同M-W组合，但比M-W组合的周跳探测能力更强，用于探测大周跳和伪码相位一致性检验)
			double *pThrFrePhaseGIFCom = new double[nCount];     // 三频相位无几何距离、无电离层组合观测量(仅使用相位观测量，观测噪声低，小周跳的探测能力较强)	
			double *pThrFrePhaseGFCom  = new double[nCount]; 	 // 三频相位无几何距离组合观测量，作为前两种方法的有益补充，能够探测出很多特殊的周跳组合，尤其是三个频点同时发生的周跳。		
			int    *pSlip           = new int   [nCount];
			int    *pOutlierCode    = new int   [nCount]; // 伪距野值,在运用三频伪距相位GIF组合时需要考虑			
			double alpha = pow(BD_FREQUENCE_L1 / BD_FREQUENCE_L2, 2);
			double belta = pow(BD_FREQUENCE_L1 / BD_FREQUENCE_L5, 2);
			double coefficient_L1_basic = (belta - alpha) / (alpha - 1);
			double coefficient_L2_basic = (1 - belta) / (alpha - 1);
			Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSat.editedObs.begin();
			GPST t0 = it0->first; 		
			// 构造三频伪距相位GIF组合pThrFreCodePhaseGIFCom = coefficient_L1_PLGIF * L1 + coefficient_L2_PLGIF * L2 + coefficient_L5_PLGIF * L5 + (P1 + P2 + P3)/3
			double coefficient_L5_PLGIF = 25;
			double coefficient_L1_PLGIF = - (4 * alpha + belta + 1)/(3 * (alpha - 1)) + coefficient_L1_basic * coefficient_L5_PLGIF;
			double coefficient_L2_PLGIF = (alpha + belta + 4)/(3 * (alpha - 1)) + coefficient_L2_basic * coefficient_L5_PLGIF;			
			// 构造三频相位GIF组合 pThrFrePhaseGIFCom = coefficient_L1_LGIF * L1 + coefficient_L2_LGIF * L2 + coefficient_L5_LGIF *L5
			double coefficient_L5_LGIF = 1;
			double coefficient_L1_LGIF = coefficient_L1_basic * coefficient_L5_LGIF;
			double coefficient_L2_LGIF = coefficient_L2_basic * coefficient_L5_LGIF;				
			int i = 0;		
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
			{				
				pEpochTime[i] = it->first - t0;		
				Rinex2_1_EditedObsDatum  L1 = it->second.obsTypeList[index_L1];
				Rinex2_1_EditedObsDatum  L2 = it->second.obsTypeList[index_L2];
				Rinex2_1_EditedObsDatum  L5 = it->second.obsTypeList[index_L5];
				Rinex2_1_EditedObsDatum  P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum  P2 = it->second.obsTypeList[index_P2];
				Rinex2_1_EditedObsDatum  P5 = it->second.obsTypeList[index_P5];	
				// 构造三频伪距相位无几何距离无电离层组合
				pThrFreCodePhaseGIFCom[i] =  coefficient_L1_PLGIF * L1.obs.data * BD_WAVELENGTH_L1
					                + coefficient_L2_PLGIF * L2.obs.data * BD_WAVELENGTH_L2
									+ coefficient_L5_PLGIF * L5.obs.data * BD_WAVELENGTH_L5
									+ (P1.obs.data + P2.obs.data + P5.obs.data)/3 ;
				// 构造新的三频相位无几何距离电离层组合
				pThrFrePhaseGIFCom[i] =  coefficient_L1_LGIF * L1.obs.data * BD_WAVELENGTH_L1
					                + coefficient_L2_LGIF * L2.obs.data * BD_WAVELENGTH_L2
									+ coefficient_L5_LGIF * L5.obs.data * BD_WAVELENGTH_L5;						
				// 构造三频相位无几何距离组合
		    	pThrFrePhaseGFCom[i] = 1.5 * L2.obs.data * BD_WAVELENGTH_L2 - L1.obs.data * BD_WAVELENGTH_L1 - 0.5 * L5.obs.data * BD_WAVELENGTH_L5;				
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER|| P5.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)				
					pOutlierCode[i] = TYPE_EDITEDMARK_OUTLIER;
				else
					pOutlierCode[i] = OBSPREPROC_NORMAL;

				if(L1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || L2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || L5.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
				{
					// 将不匹配的观测数据标记为野值
					if(L1.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_MW);						
					}
					if(L2.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{						
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_MW);
					}
					if(L5.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{						
						it->second.obsTypeList[index_L5].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L5].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_MW);
					}
					pSlip[i] = OBSPREPROC_OUTLIER_MW; // 接受先前野值判断结果, 以和其他野值判断方法相配合
				}
				else
				{
					if(pOutlierCode[i] == TYPE_EDITEDMARK_OUTLIER)//需要细致考虑以便充分使用相位数据,这里仅标记为未知，以便后续处理使用
						pSlip[i] = OBSPREPROC_UNKNOWN; 
					else
						pSlip[i] = OBSPREPROC_NORMAL;					
				}
				i++;
			}
			//step 3: 分弧段探测周跳		
			size_t k   = 0;
			size_t k_i = k;
			//获得每个连续跟踪弧段的数据			
			while(1)
			{
				if(k_i + 1 >= nCount)
					goto newArc;
				else
				{   
					// 判断 k_i+1 与 k_i 是否位于同一跟踪弧段
					if(pEpochTime[k_i + 1] - pEpochTime[k_i] <= min(m_PreprocessorDefine.threshold_gap,m_PreprocessorDefine.max_arclengh))
					{
						k_i++;
						continue;
					}
					else // k_i + 1 为新弧段的起点
						goto newArc;
				}
				newArc:  // 本弧段[k，k_i]数据处理 
				{
					vector<size_t>    unknownPointlist;							
					for(size_t s_i = k; s_i <= k_i; s_i++)
						if(pSlip[s_i] == OBSPREPROC_NORMAL)
							unknownPointlist.push_back(s_i);						
					size_t nCount_points = unknownPointlist.size(); 
					//if(editedObsSat.Id == 10)
					//	printf("%6d\n",nCount_points);
					// 第一步：判断正常点数据个数，正常点数据个数太少标记为野值						
					if(nCount_points <= int(m_PreprocessorDefine.min_arcpointcount))
					{
						for(size_t s_i = 0; s_i < nCount_points; s_i++)	
						{
							//if(pSlip[unknownPointlist[s_i]] != OBSPREPROC_OUTLIER_MW)
								pSlip[unknownPointlist[s_i]] = OBSPREPROC_OUTLIER_COUNT; // 正常点数据个数太少标记为野值								
						}
					}
					else
					{
						// 第二步：根据三频伪距相位GIF组合探测野值,进行相位野值剔除
						double *pDPLGIF = new double[nCount_points - 1];
						double var_DPLGIF = 0;						
						for(size_t s_i = 1; s_i < nCount_points; s_i++)
						{//此处的历元差，野值判断时可用，后续处理不必重复计算
							pDPLGIF[s_i - 1] = fabs(pThrFreCodePhaseGIFCom[unknownPointlist[s_i]] - pThrFreCodePhaseGIFCom[unknownPointlist[s_i - 1]]);
							if(pDPLGIF[s_i - 1] < m_PreprocessorDefine.threshold_slipsize_PLGIF)							
								var_DPLGIF += pDPLGIF[s_i - 1] * pDPLGIF[s_i - 1];	
						}						
						// 为了提高方差的计算效率，此处不采用RobustStatRms计算方差。
						var_DPLGIF = sqrt(var_DPLGIF/nCount_points);						
						//double var = RobustStatRms(pDPLGIF, int(nCount_points - 1));										
						double threshold_outlier_DPLGIF = 5 * var_DPLGIF;						
						// 利用 threshold_slipsize_PLGIF 对 threshold_outlier 的上界进行控制
						// 因为当周跳发生在序列中间附近时, var 可能会超界, 影响野值探测
						// 为了充分利用数据，将min,改为max,2014/5/3,刘俊宏
						threshold_outlier_DPLGIF = max(threshold_outlier_DPLGIF, m_PreprocessorDefine.threshold_slipsize_PLGIF);
						// [1, nCount_points - 2]
						for(size_t s_i = 1; s_i < nCount_points - 1; s_i++)
						{
							if(pDPLGIF[s_i - 1] > threshold_outlier_DPLGIF && pDPLGIF[s_i]  > threshold_outlier_DPLGIF)
							{			
								// 此时标记的野值未必是相位出现了野值，很可能是码出现了野值
								// 为了后续处理使用连续的相位数据，暂时将码标记为野值
								// if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
									//pSlip[unknownPointlist[s_i]] = OBSPREPROC_OUTLIER_MW;	
									pOutlierCode[unknownPointlist[s_i]] = OBSPREPROC_OUTLIER_MW;

							}							
						}
						delete pDPLGIF;	
						// 首尾两点 0 和 nCount_points - 1
						if(pOutlierCode[unknownPointlist[1]] != OBSPREPROC_NORMAL)
							//pSlip[unknownPointlist[0]] = OBSPREPROC_OUTLIER_MW;
						   pOutlierCode[unknownPointlist[0]] = OBSPREPROC_OUTLIER_MW;
						else
						{
							if(fabs(pThrFreCodePhaseGIFCom[unknownPointlist[0]] - pThrFreCodePhaseGIFCom[unknownPointlist[1]])  > threshold_outlier_DPLGIF)
								//pSlip[unknownPointlist[0]] = OBSPREPROC_OUTLIER_MW;
								pOutlierCode[unknownPointlist[0]] = OBSPREPROC_OUTLIER_MW;
						}
						if(pOutlierCode[unknownPointlist[nCount_points - 2]] != OBSPREPROC_NORMAL)
							//pSlip[unknownPointlist[nCount_points - 1]] = OBSPREPROC_OUTLIER_MW;	
							pOutlierCode[unknownPointlist[nCount_points - 1]] = OBSPREPROC_OUTLIER_MW;	
						else
						{
							if(fabs(pThrFreCodePhaseGIFCom[unknownPointlist[nCount_points - 1]] - pThrFreCodePhaseGIFCom[unknownPointlist[nCount_points - 2] ])  > threshold_outlier_DPLGIF)
								//pSlip[unknownPointlist[nCount_points - 1]] = OBSPREPROC_OUTLIER_MW;
								pOutlierCode[unknownPointlist[nCount_points - 1]] = OBSPREPROC_OUTLIER_MW;
						}					
						size_t s_i = 0;
						while(s_i < unknownPointlist.size())
						{
							//if(pSlip[unknownPointlist[s_i]] == OBSPREPROC_NORMAL)
							if(pOutlierCode[unknownPointlist[s_i]] == OBSPREPROC_NORMAL)
								s_i++;
							else
							{
								// 在进行周跳探测时, 先将野值 erase
								unknownPointlist.erase(unknownPointlist.begin() + s_i);
							}
						}
						//if(editedObsSat.Id == 10)
						//{
						//	FILE *pflie_t = fopen("C:\\thrfre_cycleslip_detection.dat","w+");
						//	for(size_t s_i = 1; s_i < unknownPointlist.size(); s_i ++)
						//	{
						//		GPST t = t0 + pEpochTime[s_i];
						//		fprintf(pflie_t,"%2d %s %8.4lf %8.4lf %8.4lf\n",editedObsSat.Id,t.toString().c_str(),
						//			                           pThrFreCodePhaseGIFCom[unknownPointlist[s_i]],
						//			                           fabs(pThrFreCodePhaseGIFCom[unknownPointlist[s_i]]     - pThrFreCodePhaseGIFCom[unknownPointlist[s_i-1]]),
						//									   pThrFrePhaseGIFCom[unknownPointlist[s_i]]);
						//	}
						//	fclose(pflie_t);
						//}//
						nCount_points = unknownPointlist.size();																
						// 第三步：根据三频伪距相位GIF组合探测大周跳
						if(nCount_points <= 3)
						{
							// 个数太少则直接丢弃
							for(size_t s_i = 0; s_i < nCount_points; s_i++)
							{								
								//if(pSlip[unknownPointlist[s_i]] != OBSPREPROC_OUTLIER_MW)
									pSlip[unknownPointlist[s_i]] = OBSPREPROC_OUTLIER_COUNT;								
							}
						}
						else
						{
							// 重新计算历元差
							double *pDPLGIF_new = new double[nCount_points - 1];										
							for(size_t s_i = 1; s_i < nCount_points; s_i++)//此处的历元差，周跳探测时可用，后续处理不必重复计算						
							pDPLGIF_new[s_i - 1] = fabs(pThrFreCodePhaseGIFCom[unknownPointlist[s_i]] - pThrFreCodePhaseGIFCom[unknownPointlist[s_i - 1]]);	
							vector<size_t> slipindexlist;
							slipindexlist.clear();
							// [1, nCount_points - 2]						
							for(size_t s_i = 1; s_i < nCount_points - 1; s_i++)
							{
								// 周跳在探测后, 其信息都被保存下来了
								if(pDPLGIF_new[s_i - 1] >  m_PreprocessorDefine.threshold_slipsize_PLGIF && pDPLGIF_new[s_i]    <= threshold_outlier_DPLGIF) 
								{									
									size_t index = unknownPointlist[s_i];
									pSlip[index] = OBSPREPROC_SLIP_ThrFrePLGIFCom;
								}
							}		
							delete pDPLGIF_new;
							////////////////////////剔除周跳频繁的段弧段，并进一步剔除小野值点
							for(size_t s_i = 1; s_i < nCount_points; s_i++)
							{
								size_t index = unknownPointlist[s_i];
								if(pSlip[index] == OBSPREPROC_SLIP_ThrFrePLGIFCom)
									slipindexlist.push_back(index); 
							}
							size_t count_slips = slipindexlist.size();
							size_t *pSubsection_left  = new size_t [count_slips + 1];
							size_t *pSubsection_right = new size_t [count_slips + 1];
							if(count_slips > 0)
							{ 
								// 记录周跳的左右端点值
								pSubsection_left[0] = unknownPointlist[0];
								for(size_t s_i = 0; s_i < count_slips; s_i++)
								{
									pSubsection_right[s_i]    = slipindexlist[s_i] -  1;
									pSubsection_left[s_i + 1] = slipindexlist[s_i];
								}
								pSubsection_right[count_slips] = unknownPointlist[nCount_points - 1]; 
							}
							else
							{
								pSubsection_left[0]  = unknownPointlist[0];
								pSubsection_right[0] = unknownPointlist[nCount_points - 1];
							} 
							for(size_t s_i = 0; s_i < count_slips + 1; s_i++)
							{
								// 整理 [pSubsection_left[s_i], pSubsection_right[s_i]]
								vector<size_t> subsectionNormalPointlist;
								subsectionNormalPointlist.clear();								
								for(size_t s_j = pSubsection_left[s_i]; s_j <= pSubsection_right[s_i]; s_j++)
								{
									if(pOutlierCode[s_j] != OBSPREPROC_OUTLIER_MW && pSlip[s_j] != OBSPREPROC_OUTLIER_MW && pSlip[s_j] != OBSPREPROC_UNKNOWN)									
										subsectionNormalPointlist.push_back(s_j);									
								}
								size_t count_subsection = subsectionNormalPointlist.size(); 
								if(subsectionNormalPointlist.size() <= m_PreprocessorDefine.min_arcpointcount)
								{ 
									//组合序列弧段的正常点个数过少!直接标为野值
									for(size_t s_j = 0; s_j < count_subsection; s_j++)									
										pSlip[subsectionNormalPointlist[s_j]] = OBSPREPROC_OUTLIER_COUNT; 									
								}
								else
								{   // 第四步：用相位GIF组合观测量探测周跳	
									vector<size_t> slipindexlist_LGIF;
									double *pDLGIF = new double[count_subsection - 1];
									double max_DLGIF = 0;
									double var_DLGIF = 0;
									for(size_t s_i = 1; s_i < count_subsection; s_i++)
									{
										pDLGIF[s_i - 1] = fabs(pThrFrePhaseGIFCom[subsectionNormalPointlist[s_i]] - pThrFrePhaseGIFCom[subsectionNormalPointlist[s_i - 1]]);
										max_DLGIF = max(max_DLGIF,pDLGIF[s_i - 1]);//差分数据在后续过程可以直接使用，不用重复计算
										if(pDLGIF[s_i - 1] < 3 * m_PreprocessorDefine.threshold_slipsize_LGIF)
											var_DLGIF += pDLGIF[s_i - 1] * pDLGIF[s_i - 1];
									}
									var_DLGIF = sqrt(var_DLGIF/count_subsection);
									// 绝大多数情况下，4*var_DLGIF < 3cm,但BJF2和WHU2测站的相位噪声太大，4*var_DLGIF 通常超过6cm，导致周跳探测出现虚警
									// 为了让周跳探测方法自适应数据的噪声，特地增加历元差方差的计算
									double threshold_slipsize_LGIF = max(m_PreprocessorDefine.threshold_slipsize_LGIF,3 * var_DLGIF);	
									//double threshold_slipsize_LGIF = 4 * var_DLGIF;
									if(max_DLGIF > threshold_slipsize_LGIF)
									{
										for(size_t s_j = 1; s_j < subsectionNormalPointlist.size() - 1; s_j++)
										{
											if(pDLGIF[s_j - 1] >  threshold_slipsize_LGIF && pDLGIF[s_j]   <= threshold_slipsize_LGIF) 
											{			
												// 用弧段之间的均值检验是否真的发生了周跳，由于受噪声的影响，很可能出现虚警，检验必不可少
									            // step1 计算均值(不可以计算整个弧段的均值，由于多径误差的影响，整个弧段的均值可能受到影响)
												// 因此仅计算周跳附近的几个点，而且但此处跳跃较大时不用检验
												if(pDLGIF[s_j - 1] < m_PreprocessorDefine.threshold_LGIF_slipcheck)
												{
													size_t nobs = 5;// 暂时取前后5个点
													if(s_j > nobs + 1 && subsectionNormalPointlist.size() - 1 > nobs + s_j)
													{
														double mean_slip_before = 0;
														double mean_slip_after  = 0;
														for(size_t s_c = 0; s_c < nobs; s_c ++)
														{															
															mean_slip_after  += pThrFrePhaseGIFCom[subsectionNormalPointlist[s_c + s_j]];															
															mean_slip_before += pThrFrePhaseGIFCom[subsectionNormalPointlist[s_j - s_c - 1]];																																										
														}
														mean_slip_after  = mean_slip_after / int(nobs);
														mean_slip_before = mean_slip_before / int(nobs);
														if(fabs(mean_slip_after - mean_slip_before) > threshold_slipsize_LGIF)
														{
															size_t index = subsectionNormalPointlist[s_j];
															pSlip[index] = OBSPREPROC_SLIP_ThrFreLGIFCom;
															slipindexlist_LGIF.push_back(index);													
														}
													}
													else
													{
														size_t index = subsectionNormalPointlist[s_j];
														pSlip[index] = OBSPREPROC_SLIP_ThrFreLGIFCom;
														slipindexlist_LGIF.push_back(index);
													}
												}
												else
												{
													size_t index = subsectionNormalPointlist[s_j];
													pSlip[index] = OBSPREPROC_SLIP_ThrFreLGIFCom;
													slipindexlist_LGIF.push_back(index);
												}
											}
											else if(pDLGIF[s_j - 1] >  threshold_slipsize_LGIF && pDLGIF[s_j]   > threshold_slipsize_LGIF) 
											{
												// 此处周跳和野值同时判断，暂时用OBSPREPROC_OUTLIER_MW标记为野值
												pSlip[subsectionNormalPointlist[s_j]]  = OBSPREPROC_OUTLIER_MW;												
											}
										}//	
										delete pDLGIF;
										// 首尾两点 0 和 count_subsection - 1
										if(fabs(pThrFrePhaseGIFCom[subsectionNormalPointlist[0]] - pThrFrePhaseGIFCom[subsectionNormalPointlist[1]])  > m_PreprocessorDefine.threshold_slipsize_LGIF)
											pSlip[subsectionNormalPointlist[0]] = OBSPREPROC_OUTLIER_MW;	
										    //pOutlierCode[subsectionNormalPointlist[0]] = OBSPREPROC_OUTLIER_MW;
										if(fabs(pThrFrePhaseGIFCom[subsectionNormalPointlist[count_subsection - 1]] - pThrFrePhaseGIFCom[subsectionNormalPointlist[count_subsection - 2] ])  > m_PreprocessorDefine.threshold_slipsize_LGIF)
											pSlip[subsectionNormalPointlist[count_subsection - 1]] = OBSPREPROC_OUTLIER_MW;
										    //pOutlierCode[subsectionNormalPointlist[count_subsection - 1]] = OBSPREPROC_OUTLIER_MW;										
									}
									else
										delete pDLGIF;

									size_t count_slips_LGIF = slipindexlist_LGIF.size();  // 记录相位消电离层组合探测的周跳个数
									size_t *pSubsection_left_LGIF  = new size_t [count_slips_LGIF + 1];
									size_t *pSubsection_right_LGIF = new size_t [count_slips_LGIF + 1];
									if(count_slips_LGIF > 0)
									{ 
										// 记录周跳的左右端点值
										pSubsection_left_LGIF[0] = subsectionNormalPointlist[0];
										for(size_t s_i = 0; s_i < count_slips_LGIF; s_i++)
										{
											pSubsection_right_LGIF[s_i]    = slipindexlist_LGIF[s_i] -  1;
											pSubsection_left_LGIF[s_i + 1] = slipindexlist_LGIF[s_i];
										}
										pSubsection_right_LGIF[count_slips_LGIF] = subsectionNormalPointlist[count_subsection - 1]; 
									}
									else
									{
										 pSubsection_left_LGIF[0]  = subsectionNormalPointlist[0];
										 pSubsection_right_LGIF[0] = subsectionNormalPointlist[count_subsection - 1];
									} 
									// 用弧段之间的均值检验是否真的发生了周跳，由于受噪声的影响，很可能出现虚警，检验必不可少
									////step1 计算均值(不可以计算整个弧段的均值，由于多径误差的影响，整个弧段的均值可能受到影响)
									//double *mean_LGIF_arc = new double [count_slips_LGIF + 1];
									//for(int i = 0; i < count_slips_LGIF + 1; i ++)
									//	mean_LGIF_arc[i] = 0;									
									//for(size_t s_i = 0; s_i < count_slips_LGIF + 1; s_i++)
									//{
									//	// 计算 [pSubsection_left[s_i], pSubsection_right[s_i]]的均值
									//	int arc_num = 0;//弧段内数据个数										
									//	for(size_t s_j = pSubsection_left_LGIF[s_i]; s_j <= pSubsection_right_LGIF[s_i]; s_j++)	
									//	{
									//		if(pSlip[s_j] != OBSPREPROC_OUTLIER_MW)
									//		{
									//			mean_LGIF_arc[s_i] + = pThrFrePhaseGIFCom[s_j];
									//			arc_num ++;
									//		}														
									//	}
									//	mean_LGIF_arc[s_i] = mean_LGIF_arc[s_i] / arc_num;
									//}
									////step 2 用均值检验是否发生了周跳
									//for(int i = 0; i < count_slips_LGIF; i ++)
									//{
									//	if(fabs(mean_LGIF_arc[i + 1] - mean_LGIF_arc[i]) < threshold_slipsize_LGIF)
									//}




									for(size_t s_i = 0; s_i < count_slips_LGIF + 1; s_i++)
									{
										// 整理 [pSubsection_left[s_i], pSubsection_right[s_i]]
										vector<size_t> unknownPhaselist;
										unknownPhaselist.clear();								
										for(size_t s_j = pSubsection_left_LGIF[s_i]; s_j <= pSubsection_right_LGIF[s_i]; s_j++)	
										{
											if(pSlip[s_j] != OBSPREPROC_OUTLIER_MW)
												unknownPhaselist.push_back(s_j);		
										}
										size_t count_subsection_phase = unknownPhaselist.size(); 
										if(count_subsection_phase <= m_PreprocessorDefine.min_arcpointcount)
										{ 
											//组合序列弧段的正常点个数过少!直接标为野值
											for(size_t s_j = 0; s_j < count_subsection_phase; s_j++)									
												pSlip[unknownPhaselist[s_j]] = OBSPREPROC_OUTLIER_COUNT; 									
										}
										else
										{
											//  第五步：用相位GF组合观测量探测周跳											
											// 计算历元差		
											vector<size_t> slipindexlist_LGF;
											double *pDLGF = new double[count_subsection_phase - 1];
											double max_DLGF = 0;
											for(size_t s_j = 1; s_j < count_subsection_phase; s_j++)
											{											
												double  count_interval = (pEpochTime[unknownPhaselist[s_j]] - pEpochTime[unknownPhaselist[s_j - 1]]) / m_PreprocessorDefine.interval;
												pDLGF[s_j - 1] = fabs(pThrFrePhaseGFCom[unknownPhaselist[s_j]] - pThrFrePhaseGFCom[unknownPhaselist[s_j - 1]]) / count_interval;
												max_DLGF = max(max_DLGF,pDLGF[s_j - 1]);											 
											}
											//printf("%8.4lf\n",max_DLGF);
											
											// 仅对历元差超差的弧段进行野值和周跳判断
											if(max_DLGF > m_PreprocessorDefine.threshold_slipsize_LGF)
											{												
												// 首先进行野值剔除	,由于电离层变化不规则，此处的野值判别值应稍大一些。
												int    count_outlier_LGF = 0;
												double threshold_outlier_LGF = 4 * m_PreprocessorDefine.threshold_slipsize_LGF;
												for(size_t s_k = 1; s_k < count_subsection_phase - 1; s_k ++)
												{
													if(pDLGF[s_k - 1] > threshold_outlier_LGF && pDLGF[s_k] > threshold_outlier_LGF)
													{
														pSlip[unknownPhaselist[s_k]] = OBSPREPROC_OUTLIER_L1_L2;
														count_outlier_LGF ++;
													}
												}											
												// 剔除野值进行周跳判断
												// 多数情况下，相位数据没有野值，因此为了减少计算量，此处做一个判断
												if(count_outlier_LGF > 0)
												{
													size_t s_l = 0;
													while(s_l < unknownPhaselist.size())
													{
														//if(pSlip[unknownPointlist[s_i]] == OBSPREPROC_NORMAL)
														if(pSlip[unknownPhaselist[s_l]] != OBSPREPROC_OUTLIER_L1_L2)
															s_l++;
														else
														{
															// 在进行周跳探测时, 先将野值 erase
															unknownPhaselist.erase(unknownPhaselist.begin() + s_l);
														}
													}
													count_subsection_phase = unknownPhaselist.size();
												}													
												// 重新计算历元差
												double *pDLGF_new = new double[count_subsection_phase - 1];
												if(count_outlier_LGF > 0)
												{
													for(size_t s_m = 1; s_m < count_subsection_phase; s_m ++)
													{											
														double  count_interval = (pEpochTime[unknownPhaselist[s_m]] - pEpochTime[unknownPhaselist[s_m - 1]]) / m_PreprocessorDefine.interval;
														pDLGF_new[s_m - 1] = fabs(pThrFrePhaseGFCom[unknownPhaselist[s_m]] - pThrFrePhaseGFCom[unknownPhaselist[s_m - 1]]) / count_interval;																						 
													}
												}
												else
												{
													for(size_t s_m = 1; s_m < count_subsection_phase; s_m ++)																								
														pDLGF_new[s_m - 1] = pDLGF[s_m - 1];
												}
												
												// 进行周跳探测,需要进行高度角加权
												for(size_t s_n = 1; s_n < count_subsection_phase - 1; s_n ++)
												{													
													DayTime  t_epoch = t0 + pEpochTime[unknownPhaselist[s_n]];
													double   elevation = editedObsSat.editedObs[t_epoch].Elevation;
													double   weight = 1.0;
													if(editedObsSat.Id > 5)
													{// GEO卫星相对于地面基本不动，因此由于距离变化引起的电离层变化很小。
														if(elevation > 5)
															weight  = 1.0 / sin(elevation * PI / 180);		
														else
															weight = 4.0;
													}
													if(pDLGF_new[s_n - 1] > m_PreprocessorDefine.threshold_slipsize_LGF *  weight
													&& pDLGF_new[s_n] <= m_PreprocessorDefine.threshold_slipsize_LGF * weight)
													{
														size_t index = unknownPhaselist[s_n];
														pSlip[index] = OBSPREPROC_SLIP_ThrFreLGFCom;
														slipindexlist_LGF.push_back(index);
													}
												}
												delete pDLGF;
												delete pDLGF_new;
												// 整理弧段信息，剔除周跳频繁的弧段
												size_t count_slips_LGF = slipindexlist_LGF.size();
												size_t *pSubsection_left_LGF  = new size_t [count_slips_LGF + 1];
												size_t *pSubsection_right_LGF = new size_t [count_slips_LGF + 1];
												if(count_slips_LGF > 0)
												{ 
													// 记录周跳的左右端点值
													pSubsection_left_LGF[0] = unknownPhaselist[0];
													for(size_t s_i = 0; s_i < count_slips_LGF; s_i++)
													{
														pSubsection_right_LGF[s_i]    = slipindexlist_LGF[s_i] -  1;
														pSubsection_left_LGF[s_i + 1] = slipindexlist_LGF[s_i];
													}
													pSubsection_right_LGF[count_slips_LGF] = unknownPhaselist[count_subsection_phase - 1]; 
												}
												else
												{
													pSubsection_left_LGF[0]  = unknownPhaselist[0];
													pSubsection_right_LGF[0] = unknownPhaselist[count_subsection_phase - 1];
												} 
												for(size_t s_i = 0; s_i < count_slips_LGF + 1; s_i++)
												{
													// 整理 [pSubsection_left[s_i], pSubsection_right[s_i]]
													vector<size_t> subsectionNormalPhaselist;
													subsectionNormalPhaselist.clear();								
													for(size_t s_j = pSubsection_left_LGF[s_i]; s_j <= pSubsection_right_LGF[s_i]; s_j++)
													{
														if(pSlip[s_j] != OBSPREPROC_OUTLIER_MW && pSlip[s_j] != OBSPREPROC_OUTLIER_L1_L2)									
															subsectionNormalPhaselist.push_back(s_j);									
													}
													size_t count_subsection = subsectionNormalPhaselist.size(); 
													if(count_subsection <= m_PreprocessorDefine.min_arcpointcount)
													{ 
														//组合序列弧段的正常点个数过少!直接标为野值
														for(size_t s_j = 0; s_j < count_subsection; s_j++)									
															pSlip[subsectionNormalPhaselist[s_j]] = OBSPREPROC_OUTLIER_COUNT; 									
													}
												}
												delete pSubsection_left_LGF;
												delete pSubsection_right_LGF;
											}
											else
												delete pDLGF;
										}
									}
									delete pSubsection_left_LGIF;
									delete pSubsection_right_LGIF;
									
								}
							}
							delete pSubsection_left;
							delete pSubsection_right;
							for(size_t s_i = k; s_i <= k_i; s_i++)
							{
								// 将第一个非野值点, 更新标记为周跳
								if((pSlip[s_i] == OBSPREPROC_NORMAL && pOutlierCode[s_i] == OBSPREPROC_NORMAL)
								|| pSlip[s_i] == OBSPREPROC_SLIP_ThrFrePLGIFCom 
								|| pSlip[s_i] == OBSPREPROC_SLIP_ThrFreLGIFCom
								|| pSlip[s_i] == OBSPREPROC_SLIP_ThrFreLGFCom)
								{
									pSlip[s_i] = OBSPREPROC_NEWARCBEGIN;
									break;
								}
							}
						}
					}
					if(k_i + 1 >= nCount)
						break;
					else  
					{
						// 新弧段的起点设置
						k   = k_i + 1;
						k_i = k;
						continue;
					}
				}				
			}
			i = 0;
			//FILE *pfile = fopen("C:\\TreFrePhase.cpp","a+");
			for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSat.editedObs.begin(); it != editedObsSat.editedObs.end(); ++it)
			{	
				if(pSlip[i] == OBSPREPROC_UNKNOWN) // 将未知的点恢复为野值点，因为码为野值点
					pSlip[i] = OBSPREPROC_OUTLIER_MW;
				if(pOutlierCode[i] == OBSPREPROC_OUTLIER_MW && pSlip[i] == OBSPREPROC_NORMAL)
					pSlip[i] = OBSPREPROC_OUTLIER_MW;
				if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[i]);
					it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[i]);					
				}
				if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[i]);
					it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[i]);					
				}
				if(it->second.obsTypeList[index_L5].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_L5].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[i]);
					it->second.obsTypeList[index_L5].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[i]);					
				}
				if(bOutTempFile == true)
				{	
					//// 写临时文件
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
						&& it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
						&& it->second.obsTypeList[index_L5].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						GPST t = it->first;	
						GPST t0 = m_obsFile.m_data.front().t;
						t0.hour   = 0;
						t0.minute = 0;
						t0.second = 0;
						fprintf(pfile,"%2d %s %8.2f %8.2f %14.4f %14.4f %14.4f %2d\n",
										editedObsSat.Id,
										t.toString().c_str(),						            
										(t - t0)/3600,
										it->second.Elevation,									
										pThrFreCodePhaseGIFCom[i],
										pThrFrePhaseGIFCom[i],
										pThrFrePhaseGFCom[i],
										it->second.obsTypeList[index_L1].byEditedMark1 * 10 + it->second.obsTypeList[index_L1].byEditedMark2
										//it ->second.obsTypeList[index_L1].obs.data * BD_WAVELENGTH_L1 - it ->second.obsTypeList[index_L2].obs.data  * BD_WAVELENGTH_L2 ,
										//it ->second.obsTypeList[index_L5].obs.data  * BD_WAVELENGTH_L5 - it ->second.obsTypeList[index_L2].obs.data  * BD_WAVELENGTH_L2,
										//it ->second.obsTypeList[index_L1].obs.data * BD_WAVELENGTH_L1 + it ->second.obsTypeList[index_L2].obs.data  * BD_WAVELENGTH_L2 * 4 + it ->second.obsTypeList[index_L5].obs.data  * BD_WAVELENGTH_L5 * (-5),
										//it ->second.obsTypeList[index_L1].obs.data  * BD_WAVELENGTH_L1 + it ->second.obsTypeList[index_L2].obs.data  * BD_WAVELENGTH_L2 * 2 - it ->second.obsTypeList[index_L5].obs.data  * BD_WAVELENGTH_L5 * 3);
										//it ->second.obsTypeList[index_L1 + 2].obs.data- it ->second.obsTypeList[index_L2 + 2].obs.data,
										//it ->second.obsTypeList[index_L5 + 1].obs.data - it ->second.obsTypeList[index_L2 + 2].obs.data);	
						                
										//it ->second.obsTypeList[index_L1].obs.data * BD_WAVELENGTH_L1
										);	
					
					}
				}
				i++;
			}
			if(bOutTempFile == true)
				fclose(pfile);			
			delete  pSlip;
			delete  pEpochTime;
			delete  pOutlierCode;
			delete  pThrFreCodePhaseGIFCom;
			delete  pThrFrePhaseGIFCom;
			delete  pThrFrePhaseGFCom;
			return  true;//
		}
		// 子程序名称： mainFuncThrFreObsPreproc   
		// 功能：北斗三频观测数据预处理
		// 变量类型：    editedobsfile      　　              : 预处理后的观测数据
		//               bOutTempFile                         : 是否输出预处理信息
		// 输入：m_obsFile，bOutTempFile
		// 输出：
		// 语言：C++
		// 创建者：刘俊宏
		// 创建时间：2013/3/23
		// 版本时间：2013/3/23
		// 修改记录：1、将数据过短的弧段直接剔除，以免增加后续数据的计算负担。刘俊宏2014/06/25
		// 其它：    1、三频数据预处理优缺点：优点--使得周跳探测更加容易
		//                                    缺点--引入了第三个频点的周跳，如果定轨不用第三个频点的数据，则认为中断了观测数据
		bool BDObsPreproc::mainFuncThrFreObsPreproc(Rinex2_1_EditedObsFile &editedObsFile,bool bOutTempFile)
		{
			//// 创建预处理目录
			//string folder = m_strObsFileName.substr(0, m_strObsFileName.find_last_of("\\"));
			//string obsFileName = m_strObsFileName.substr(m_strObsFileName.find_last_of("\\") + 1);
			//string obsFileName_noexp = obsFileName.substr(0, obsFileName.find_last_of("."));
			//// 创建预处理目录
			//string strPreprocFolder = folder + "\\Preproc";
			//_mkdir(strPreprocFolder.c_str());
			//char  TreFreCode_FileName[200];
			//char  TreFrePhase_FileName[200];
			//sprintf(TreFreCode_FileName, "%s\\%s_ThreFreCode.dat", strPreprocFolder.c_str(), obsFileName_noexp.c_str());
			//sprintf(TreFrePhase_FileName, "%s\\%s_ThreFrePhase.dat", strPreprocFolder.c_str(), obsFileName_noexp.c_str());
			//FILE *pfile_C = fopen(TreFreCode_FileName,"w+");
			//FILE *pfile_P = fopen(TreFrePhase_FileName,"w+");
			//fclose(pfile_C);
			//fclose(pfile_P);		
			if(bOutTempFile == true)
			{
				_mkdir(m_strPreprocFilePath.c_str());
				// 测试程序，临时写文件				
				char szStationName[4 + 1];
				for(int k = 0; k < 4; k++)
				{
					szStationName[k] = m_obsFile.m_header.szMarkName[k];
				}
				szStationName[4] = '\0';	
				char  TreFreCode_FileName[200];
				char  TreFrePhase_FileName[200];
				sprintf(TreFreCode_FileName, "%s\\%s_ThreFreCode.dat", m_strPreprocFilePath.c_str(), szStationName);
				sprintf(TreFrePhase_FileName, "%s\\%s_ThreFrePhase.dat", m_strPreprocFilePath.c_str(), szStationName);
				FILE *pfile_C = fopen(TreFreCode_FileName,"w+");
				FILE *pfile_P = fopen(TreFrePhase_FileName,"w+");
				fclose(pfile_C);
				fclose(pfile_P);				
			}

			bool  nav_flag = false;
			if(m_navFile.isEmpty() && m_sp3File.isEmpty())
			{		
				printf("无星历数据!，请确认!\n");				
				return  false;				
			}
			else
				if(m_navFile.isEmpty())
					nav_flag = true;//
			if(m_obsFile.isEmpty())
			{
				if(m_obsFile.isEmpty())
					printf("无观测数据!，请确认!\n");
				return  false;
			}
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_L2 = -1, nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_L5 = -1, nObsTypes_P5 = -1;
			for(int i = 0; i < m_obsFile.m_header.byObsTypes; i++)
			{
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L5)
					nObsTypes_L5 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P5)
					nObsTypes_P5 = i;	//		
			}
			if(nObsTypes_L1 == -1 || nObsTypes_L2 == -1 || nObsTypes_P1 == -1 || nObsTypes_P2 == -1 || nObsTypes_L5 == -1|| nObsTypes_P5 == -1) 
			{
				printf("观测数据类型不完整！\n");
				return false;		
			}
			vector<Rinex2_1_EditedObsEpoch> editedObsEpochlist;
			vector<Rinex2_1_EditedObsSat>   editedObsSatlist_raw,editedObsSatlist;
			getEditedObsSatList(editedObsSatlist_raw);
			for(size_t s_j = 0; s_j < editedObsSatlist_raw.size(); s_j++)
			{
				if(editedObsSatlist_raw[s_j].Id <= 14)		
					editedObsSatlist.push_back(editedObsSatlist_raw[s_j]);
			}
			int nObsTypes[6];
			nObsTypes[0] = nObsTypes_L1;
			nObsTypes[1] = nObsTypes_L2;
			nObsTypes[2] = nObsTypes_P1;
			nObsTypes[3] = nObsTypes_P2;
			nObsTypes[4] = nObsTypes_L5;
			nObsTypes[5] = nObsTypes_P5;	
			size_t s_i = 0;
			while(s_i < editedObsSatlist.size())
			//for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{
				if(editedObsSatlist[s_i].editedObs.size() <= m_PreprocessorDefine.min_arcpointcount)  // 观测个数太少, 直接丢弃
				{				
					for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
					{
						for(int i = 0; i < 6; i++)
						{
							it->second.obsTypeList[nObsTypes[i]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_COUNT);
							it->second.obsTypeList[nObsTypes[i]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_COUNT);
						}
					}	
					editedObsSatlist.erase(editedObsSatlist.begin() + s_i);
				}
				else
				{
					// 根据数据的 DBL_MAX (或为0)标记直接判断原始观测数据为野值添加
					// 因为原始数据某些通道可能空缺, 此时该数据通常被赋值为 DBL_MAX(或为0), 在这里要将标记其恢复为野值	
					int   nPRN = editedObsSatlist[s_i].Id;
					for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
					{
						//if(it->first.hour == 13 && it->first.minute == 43)
						//{
						//	printf("Debug!\n");
						//}//
						for(int i = 0; i < 6; i++)
						{
							if(it->second.obsTypeList[nObsTypes[i]].obs.data == DBL_MAX)
							{
								it->second.obsTypeList[nObsTypes[i]].obs.data  = 0;
								it->second.obsTypeList[nObsTypes[i]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_BLANK);
								it->second.obsTypeList[nObsTypes[i]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_BLANK);
							}
							else if(it->second.obsTypeList[nObsTypes[i]].obs.data == 0)
							{
								it->second.obsTypeList[nObsTypes[i]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_ZERO);
								it->second.obsTypeList[nObsTypes[i]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_ZERO);
							}
							//if(nPRN == 11)
							//{
							//	GPST t0(2012,12,10,1,7,0);
							//	GPST t1(2012,12,10,2,18,0);
							//	if(it->first - t0 >= 0 && it->first - t1 <= 0)
							//	{
							//		it->second.obsTypeList[nObsTypes[i]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_BLANK);
							//		it->second.obsTypeList[nObsTypes[i]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_BLANK);
							//	}
							//}//2012.12.10(345)CSHA测站11号卫星的观测数据在这段时间有问题，预处理正常，但双差的残差较大，影响解算
						}
						//计算观测仰角,将仰角过低的数据标记为野值点
						POSCLK    posclk;          // 从导航文件获取的卫星位置
						SP3Datum  sp3Datum;        // 从sp3文件获取的卫星位置
						POS3D     ECFposSat;       // 卫星在地固系下的坐标
						ENU       ENUposSat;       // 卫星在测站东北天坐标系下的位置
						bool      posflag = false; // 是否成功获取卫星位置标记
						if(nav_flag)
						{
							if(m_sp3File.getEphemeris(it->first,nPRN,sp3Datum,9 ,'C'))
							{
								ECFposSat = sp3Datum.pos;
								posflag = true;
							}
						}
						else
						{
							if(m_navFile.getEphemeris(it->first,nPRN,posclk))
							{
								ECFposSat = posclk.getPos();
								posflag = true;
							}
						}//
						if(posflag)
						{
							TimeCoordConvert::ECF2ENU(m_posStation,ECFposSat,ENUposSat);	
							// 计算仰角(单位：度)
							it->second.Elevation = atan(ENUposSat.U/sqrt(ENUposSat.E*ENUposSat.E + ENUposSat.N*ENUposSat.N))*180/PI; 
							//it->second.Elevation = (PI/2 - acos(ENUposSat.U/sqrt(ENUposSat.E * ENUposSat.E + ENUposSat.N * ENUposSat.N + ENUposSat.U * ENUposSat.U)))*180/PI; // 计算仰角(单位：度)GAMIT
							// 计算方位角(单位：度)
							it->second.Azimuth   = atan2(ENUposSat.E, ENUposSat.N) * 180 / PI;

							POS3D p_station = vectorNormal(m_posStation);
							POS3D p_sat = vectorNormal(ECFposSat - m_posStation);					
							p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); // 20150608, 考虑到地球扁率的影响, 卫星仰角的计算进行了修正, 谷德峰
							p_station = vectorNormal(p_station);					
							it->second.Elevation = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;
						}
						else
						{
							it->second.Elevation = 0;
							it->second.Azimuth   = 0;
						}
						if(it->second.Azimuth < 0)					
							it->second.Azimuth += 360.0; // 变换到[0, 360]	
						double min_elevation = m_PreprocessorDefine.min_elevation;
						if(nPRN <= 5) //GEO 卫星10度以下仰角的观测数据较差，例如brst测站的C05卫星
							min_elevation = 10;         //对于定轨，强制使用大于10度的观测数据，20140816，刘俊宏

						// 根据最小仰角阈值，将低仰角观测数据标记为野值
						if(it->second.Elevation <= min_elevation)
						{
							for(int i = 0; i < 6; i++)
								if( it->second.obsTypeList[nObsTypes[i]].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
								{
									it->second.obsTypeList[nObsTypes[i]].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_ELEVATION);
									it->second.obsTypeList[nObsTypes[i]].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_ELEVATION);
								}
						}

					}
					// 伪码野值探测
					detectThrFreCodeOutlier(nObsTypes_P1,nObsTypes_P2,nObsTypes_P5,editedObsSatlist[s_i],bOutTempFile);					
					// 相位周跳探测					
					detectThrFrePhaseSlip(nObsTypes_L1,nObsTypes_L2,nObsTypes_L5, nObsTypes_P1,nObsTypes_P2, nObsTypes_P5, editedObsSatlist[s_i],bOutTempFile);
					s_i ++;
				}
			}
			datalist_sat2epoch(editedObsSatlist,editedObsEpochlist);
			//更新头文件
			editedObsFile.m_header = m_obsFile.m_header;
			editedObsFile.m_header.szSatlliteSystem[0] = 'C';
			editedObsFile.m_header.bySatCount = (int)editedObsSatlist.size();
			editedObsFile.m_header.tmStart = editedObsEpochlist.front().t;
			editedObsFile.m_header.tmEnd = editedObsEpochlist.back().t;
			editedObsFile.m_data   = editedObsEpochlist;							
			return  true;
		}
		//   子程序名称： receiverClkEst   
		//   作用：估计接收机的钟差
		//   类型：
		//         editedObsFile       : 观测数据
		//         editedObsFile_clk   : 包含接收机钟差的观测数据
		//   输入：editedObsFile
		//   输出：editedObsFile_clk
		//   语言：C++
		//   创建者：刘俊宏
		//   创建时间：2014/04/24
		//   版本时间：2014/04/24
		//   修改记录：
		//   其它：如果有卫星的Sp3和Clk文件，优先使用Sp3和Clk文件，否则考虑使用导航星历中的卫星位置和钟差 
		//         通道之间存在不一致现象，增加检验通道之间的一致性的算法，2014/05/04
		bool BDObsPreproc::receiverClkEst(Rinex2_1_EditedObsFile &editedObsFile,Rinex2_1_EditedObsFile &editedObsFile_clk)
		{
			if(m_sp3File.isEmpty())
			{
				printf("无星历数据！\n");
				return false;
			}
			if(m_clkFile.isEmpty())
			{
				printf("无钟差数据！\n");
				return false;
			}
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_L2 = -1, nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_L5 = -1, nObsTypes_P5 = -1;
			for(int i = 0; i < editedObsFile.m_header.byObsTypes; i++)
			{
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2 = i;				
			}			
			if(nObsTypes_L1 == -1 || nObsTypes_L2 == -1 || nObsTypes_P1 == -1 || nObsTypes_P2 == -1) 
			{
				printf("观测数据类型不完整！\n");
				return false;		
			}
			char  cSatSystem = editedObsFile.m_header.getSatSystemChar();
			double frequence1,frequence2;
			if( cSatSystem == 'C')
			{
				frequence1 = BD_FREQUENCE_L1;
				frequence2 = BD_FREQUENCE_L2;
			}
			else
			{
				frequence1 = GPS_FREQUENCE_L1;
				frequence2 = GPS_FREQUENCE_L2;
			}
			editedObsFile_clk = editedObsFile;
			int      k_max = 3;//允许迭代的最大次数
			double   threshold = 1.0;//接收机钟差收敛条件，单位：米
			double   coefficient_ionosphere = 1 / (1 - pow( frequence1 / frequence2, 2 ));
			for(size_t s_i = 0; s_i < editedObsFile_clk.m_data.size(); s_i ++)
			{				
				double rec_clk_sum = 0;// 接收机钟差
				int    valid_clk   = 0;
				vector<CLKDatum>     recClk_sat;  // 记录每颗卫星的解算的测站钟差
				vector<int>          problem_obs; // 记录有问题的卫星数据
				recClk_sat.clear();
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsFile_clk.m_data[s_i].editedObs.begin();it != editedObsFile_clk.m_data[s_i].editedObs.end(); ++ it)
				{
					Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[nObsTypes_P1];
					Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[nObsTypes_P2];
					Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[nObsTypes_L1];
					Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[nObsTypes_L2];
					double dP1 = P1.obs.data;
					double dP2 = P2.obs.data;					
					CLKDatum     ASDatum,ARDatum;
					SP3Datum     sp3Datum;					
					char szSatName[4];//根据PRN编号和系统名获取卫星名
					sprintf(szSatName, "%c%02d", cSatSystem, it->first);
					szSatName[3] = '\0';
					bool  clk_est_flag = true;// 钟差解算是否成功
					if(P1.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER && P2.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
					 &&L1.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER && L2.byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{						
						// 构造无电离层组合
						double PIF  = dP1  -(dP1 - dP2) * coefficient_ionosphere;						
						double receiver_clk_1 = 0;
						double delay = 0;// 信号传播延迟
						POSCLK receiverPosClock;
						receiverPosClock.x = m_posStation.x;
						receiverPosClock.y = m_posStation.y;
						receiverPosClock.z = m_posStation.z;
						receiverPosClock.clk = receiver_clk_1;
						GPST t_Receive = editedObsFile_clk.m_data[s_i].t - receiverPosClock.clk/SPEED_LIGHT; //信号接收时刻						
						GPST t_Transmit = t_Receive - delay;						
						if(!m_sp3File.getEphemeris_PathDelay(t_Transmit,receiverPosClock,szSatName,delay,sp3Datum))
						{
							printf("%s获取%02d卫星星历失败！\n",t_Transmit.toString().c_str(),it->first);
							clk_est_flag = false;
							continue;
						}
						//  卫星相对论改正
						double correct_relativity = (sp3Datum.pos.x * sp3Datum.vel.x 
												   + sp3Datum.pos.y * sp3Datum.vel.y
												   + sp3Datum.pos.z * sp3Datum.vel.z) * (-2.0) / SPEED_LIGHT;
						PIF = PIF + correct_relativity;
						// 计算概略距离
						double  distance =  pow(receiverPosClock.x - sp3Datum.pos.x, 2)
										  + pow(receiverPosClock.y - sp3Datum.pos.y, 2)
								          + pow(receiverPosClock.z - sp3Datum.pos.z, 2);
						distance = sqrt(distance);											
					    if(!m_clkFile.getSatClock(t_Transmit, it->first, ASDatum, 3, cSatSystem)) // 获得信号发射时间的卫星钟差改正
						{							
							printf("%s获取%02d卫星钟差失败！\n",t_Transmit.toString().c_str(),it->first);
							clk_est_flag = false;
							continue;
						}							
						double receiver_clk = PIF - distance + ASDatum.clkBias * SPEED_LIGHT;
						int k = 0;
						while(fabs(receiver_clk - receiver_clk_1) > threshold)
						{
							k++;
							receiverPosClock.clk = receiver_clk;
							if(k > k_max) 
							{
								printf("%s求解%02d迭代发散!\n", t_Transmit.toString().c_str(),it->first);
								clk_est_flag = false;
								break;
							}																				
							t_Transmit = editedObsFile_clk.m_data[s_i].t - receiverPosClock.clk/SPEED_LIGHT;							
							if(!m_sp3File.getEphemeris_PathDelay(t_Transmit,receiverPosClock,szSatName,delay,sp3Datum))
							{
								printf("%s获取%02d卫星星历失败！\n",t_Transmit.toString().c_str(),it->first);
								clk_est_flag = false;
								break;
							}
							// 计算概略距离
							distance =  pow(receiverPosClock.x - sp3Datum.pos.x, 2)
									  + pow(receiverPosClock.y - sp3Datum.pos.y, 2)
									  + pow(receiverPosClock.z - sp3Datum.pos.z, 2);
							distance = sqrt(distance);						
							t_Transmit = t_Transmit - delay;    //信号发射时刻							
							if(!m_clkFile.getSatClock(t_Transmit, it->first, ASDatum, 3, cSatSystem)) // 获得 GPS 信号发射时间的卫星钟差改正
							{							
								printf("%s获取%02d卫星钟差失败！\n",t_Transmit.toString().c_str(),it->first);
								clk_est_flag = false;
								break;
							}		
							receiver_clk_1 = receiver_clk;
							receiver_clk = PIF - distance + ASDatum.clkBias * SPEED_LIGHT;
						}
						if(clk_est_flag)
						{
							ARDatum.count   = it->first;
							ARDatum.clkBias = receiver_clk;
							recClk_sat.push_back(ARDatum);							
							rec_clk_sum += receiver_clk;
							valid_clk ++;
						}
					}					
				}				
				if(valid_clk == 2)
				{
					if(fabs(recClk_sat[0].clkBias - recClk_sat[1].clkBias) < m_PreprocessorDefine.threshold_recClk)
						editedObsFile_clk.m_data[s_i].clock = (recClk_sat[0].clkBias + recClk_sat[1].clkBias)/2;
					else
					{
						editedObsFile_clk.m_data[s_i].byEpochFlag = 10;//如果钟差解算有误，则认为这个历元不可用，后续处理应不使用这样的历元，2014/5/3,刘俊宏
				        printf("%s 两颗卫星解算的接收机钟差不一致,相差 % 14.4lf！\n",editedObsFile_clk.m_data[s_i].t.toString().c_str(),fabs(recClk_sat[0].clkBias - recClk_sat[1].clkBias));
					}
				}
				else if (valid_clk <= 1)
				{
					editedObsFile_clk.m_data[s_i].byEpochFlag = 10;//如果钟差解算有误，则认为这个历元不可用，后续处理应不使用这样的历元，2014/5/3,刘俊宏
					printf("%s 有效接收机钟差解的个数小于2！\n",editedObsFile_clk.m_data[s_i].t.toString().c_str());
				}
				else
				{
					//第一步:计算各卫星数据的接收机钟差解的标准差
					double  sigma = 0;
					double  mean  = rec_clk_sum / valid_clk;
					for(size_t s_ii = 0; s_ii < recClk_sat.size(); s_ii ++)					
						sigma = sigma + (recClk_sat[s_ii].clkBias - mean) * (recClk_sat[s_ii].clkBias - mean);					
					sigma = sqrt(sigma / valid_clk);
					//第二步:根据标准差判断钟差解是否有误
					if(sigma < m_PreprocessorDefine.threshold_recClk)					
						editedObsFile_clk.m_data[s_i].clock = mean;
					else
					{//第三步:剔除钟差解不一致的观测数据
				     //寻找离均值最远的点，将其剔除，然后再计算标准差，并重复这一步骤，直到剩余的钟差解个数为2.
					 //	unknownPointlist.erase(unknownPointlist.begin() + s_i);
						bool  flag = false;
						while((int)recClk_sat.size() > 2)
						{
							size_t max_index = 0;
							double max_clk = fabs(recClk_sat[0].clkBias - mean);
							for(size_t s_j = 1; s_j < recClk_sat.size(); s_j ++)
							{
								if(max_clk < fabs(recClk_sat[s_j].clkBias - mean))
								{
									max_clk = fabs(recClk_sat[s_j].clkBias - mean);
									max_index = s_j;
								}
							}
							problem_obs.push_back(recClk_sat[max_index].count);
							recClk_sat.erase(recClk_sat.begin() + max_index);
							//重新计算均值和方差
							mean  = 0;
							sigma = 0;
							for(size_t s_k = 0; s_k < recClk_sat.size(); s_k ++)
								mean = mean + recClk_sat[s_k].clkBias;
							mean = mean/int(recClk_sat.size());
							for(size_t s_l = 0; s_l < recClk_sat.size(); s_l ++)					
								sigma = sigma + (recClk_sat[s_l].clkBias - mean) * (recClk_sat[s_l].clkBias - mean);
							sigma = sqrt(sigma / int(recClk_sat.size()));
							if(sigma < m_PreprocessorDefine.threshold_recClk)
							{
								editedObsFile_clk.m_data[s_i].clock = mean;
								flag = true;
								break;
							}
						}
						if(!flag)
						{
							editedObsFile_clk.m_data[s_i].byEpochFlag = 10;//如果钟差解算有误，则认为这个历元不可用，后续处理应不使用这样的历元，2014/5/3,刘俊宏
							printf("%s 卫星解算的接收机钟差不一致,相差 % 14.4lf！\n",editedObsFile_clk.m_data[s_i].t.toString().c_str(),fabs(recClk_sat[0].clkBias - recClk_sat[1].clkBias));
						}
						else
						{
							//将通道间不一致的观测数据打上野值标记
							for(size_t s_s = 0; s_s < problem_obs.size(); s_s ++)
							{
								if(editedObsFile_clk.m_data[s_i].editedObs[problem_obs[s_s]].obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
								{
									editedObsFile_clk.m_data[s_i].editedObs[problem_obs[s_s]].obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_RAIM);
									editedObsFile_clk.m_data[s_i].editedObs[problem_obs[s_s]].obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_RAIM);
								}
								if(editedObsFile_clk.m_data[s_i].editedObs[problem_obs[s_s]].obsTypeList[nObsTypes_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
								{
									editedObsFile_clk.m_data[s_i].editedObs[problem_obs[s_s]].obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(OBSPREPROC_OUTLIER_RAIM);
									editedObsFile_clk.m_data[s_i].editedObs[problem_obs[s_s]].obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(OBSPREPROC_OUTLIER_RAIM);
								}
							}
						}
					}			
				}
				

			}
			return true;
		}
		//   子程序名称： desampling_unsmoothed   
		//   作用：预处理后的观测数据的降采样处理，不进行任何平滑处理
		//   类型：posStaion      : 测站的位置
        //         nSampleSpan    : 降采样后的采样间隔, nSampleSpan 应当是原始采样率的倍数
		//         editedObsFile  : 未降采样的观测数据
		//         desampleFile   : 降采样后的观测数据
		//         btroCor        : 是否进行先验对流层修正
		//   输入：editedObsFile,nSampleSpan
		//   输出：desampleFile
		//   语言：C++
		//   创建者：刘俊宏, 谷德峰
		//   创建时间：2012/11/12
		//   版本时间：2012/12/13
		//   修改记录：2012/12/13, 将其改为静态成员函数
		//   其它： 分弧段对原始数据进行降采样, 方便记录周跳标记        
		bool BDObsPreproc::desampling_unsmoothed(POS3D posStaion,Rinex2_1_EditedObsFile &editedObsFile,Rinex2_1_EditedObsFile &desampleFile,bool btroCor, int nSampleSpan, int Freq1, int Freq2)		
		{
			if(editedObsFile.isEmpty())
			{
				printf("预处理后的观测数据为空！\n");
				return false;
			}
			// 初始化频率信息
			double WAVELENGTH_L1 = BD_WAVELENGTH_L1;
			double WAVELENGTH_L2 = BD_WAVELENGTH_L2;
			char  cSatSystem = editedObsFile.m_header.getSatSystemChar();
			if(cSatSystem != 'C')//2014/10/08,增加GPS系统考虑
			{
				WAVELENGTH_L1 = GPS_WAVELENGTH_L1;
				WAVELENGTH_L2 = GPS_WAVELENGTH_L2;
			}
			// 获取观测数据索引
			int nObsTypes_C1 = -1, nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_L1 = -1, nObsTypes_L2 = -1, nObsTypes_P5 = -1, nObsTypes_L5 = -1;
			for(int i = 0; i < editedObsFile.m_header.byObsTypes; i++)
			{
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_C1)
					nObsTypes_C1 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P5)
					nObsTypes_P5 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L5)
					nObsTypes_L5 = i;
			}

			// 此处添加频点选择，默认选择 L1(B1),L2(B2) 频点，2016/3/30，鞠 冰
			if(Freq1 == 2 && Freq2 == 3)		// L2,L5
			{
				nObsTypes_P1 = nObsTypes_P2;
				nObsTypes_L1 = nObsTypes_L2;
				WAVELENGTH_L1 = BD_WAVELENGTH_L2;
				nObsTypes_P2 = nObsTypes_P5;
				nObsTypes_L2 = nObsTypes_L5;
				WAVELENGTH_L2 = BD_WAVELENGTH_L5;
			}
			else if(Freq1 == 1 && Freq2 == 3)	// L1,L5
			{
				nObsTypes_P2 = nObsTypes_P5;
				nObsTypes_L2 = nObsTypes_L5;
				WAVELENGTH_L2 = BD_WAVELENGTH_L5;
			}

			if(nObsTypes_P1 == -1 || nObsTypes_P2 == -1 || nObsTypes_L1 == -1 || nObsTypes_L2 == -1)
			{
				if(nObsTypes_P1 == -1 && nObsTypes_C1 != -1 && nObsTypes_P2 != -1 && nObsTypes_L1 != -1 && nObsTypes_L2 != -1)
				{
					// TODO: 添加C1/P2型接收机的数据预处理策略
					//printf("暂无C1/P2型接收机处理策略！ \n");
					//return false;
					nObsTypes_P1 = nObsTypes_C1;	// 暂时将 C1 直接赋给 P1 (2013/6/26)
				}
				else
				{
					printf("观测数据类型不足！\n");
					return false;
				}
			}
			BYTE pbySatList[MAX_PRN + 1];                  // 统计降采样后的观测卫星
			for( int i = 0; i < MAX_PRN + 1; i++ )
				pbySatList[i] = 0;
			DayTime t0 = editedObsFile.m_header.tmStart;   // 文件的起始时间
			t0.hour   = 0;                                 // 2013.5.30增加，不同测站观测数据的起始时间可能不一致,保证采样起始时间相同
			t0.minute = 0;                                 // 
			t0.second = 0;
			DayTime te = editedObsFile.m_header.tmEnd;     // 文件的结束时间
			vector<Rinex2_1_EditedObsEpoch> dsampleEpochlist;
			vector<Rinex2_1_EditedObsSat>   editedObsSatlist,dsampleSatlist;
			editedObsFile.getEditedObsSatList(editedObsSatlist);			
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)		
			{	// 分弧段降采样，方便记录周跳信息
				Rinex2_1_EditedObsSat  dsampleSat;                     // 某颗卫星降采样后的观测数据
				dsampleSat.Id = editedObsSatlist[s_i].Id;			
				Rinex2_1_EditedObsEpochMap::iterator k = editedObsSatlist[s_i].editedObs.begin();   // 记录新弧段起始点
				Rinex2_1_EditedObsEpochMap::iterator k_i = editedObsSatlist[s_i].editedObs.begin(); // 记录新弧段终止点
				while(1)
				{					
					if(!(++k_i != editedObsSatlist[s_i].editedObs.end()))   // k_i 为时间序列终点
						goto newArc;
					else
					{
						// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
						if(k_i->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_SLIP)
						{							
							continue;
						}							
						else // k_i 为新弧段的起点
							goto newArc;
					}
					newArc:  // 本弧段[k，k_i]数据处理 
					{					
						Rinex2_1_EditedObsSat  dsampleSatarc;                 // 某个弧段降采样后的观测数据						
						for(Rinex2_1_EditedObsEpochMap::iterator it = k; it != k_i; ++it)
						{							
							// 给定一个时间 t，根据 nSampleSpan 大小，量化成一个有效时间 ti，
							// 通过判断 t 与 ti 的接近程度，来决定 t 是否就是所找的输出点
							double t = it->first - t0;
							double integer  = floor(t / nSampleSpan);      // 量化过程
							double fraction = t - integer * nSampleSpan;   // 接近程度判断
							if(fabs(fraction) < 0.05)
							{// 寻找到输出点							 
								dsampleSatarc.editedObs.insert(Rinex2_1_EditedObsEpochMap::value_type(it->first,it->second));									
							}
						}
						//保留周跳标记
						for(Rinex2_1_EditedObsEpochMap::iterator dit = dsampleSatarc.editedObs.begin(); dit != dsampleSatarc.editedObs.end(); ++dit)
						{
							if(dit->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{   //将第一个非野值点的标记更新为周跳
								if(dit->second.obsTypeList[nObsTypes_L1].byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
								{
									dit->second.obsTypeList[nObsTypes_L1].byEditedMark1 = k->second.obsTypeList[nObsTypes_L1].byEditedMark1;
									dit->second.obsTypeList[nObsTypes_L1].byEditedMark2 = k->second.obsTypeList[nObsTypes_L1].byEditedMark2;
									dit->second.obsTypeList[nObsTypes_L2].byEditedMark1 = k->second.obsTypeList[nObsTypes_L2].byEditedMark1;
									dit->second.obsTypeList[nObsTypes_L2].byEditedMark2 = k->second.obsTypeList[nObsTypes_L2].byEditedMark2;
									break;
								}
								else if(dit->second.obsTypeList[nObsTypes_L1].byEditedMark1 == TYPE_EDITEDMARK_SLIP)
									break;
							}
						}
						//将包含完整的预处理信息的观测数据保留下来
						for(Rinex2_1_EditedObsEpochMap::iterator dit = dsampleSatarc.editedObs.begin(); dit != dsampleSatarc.editedObs.end(); ++dit)
							dsampleSat.editedObs.insert(Rinex2_1_EditedObsEpochMap::value_type(dit->first,dit->second));
						if(!(k_i != editedObsSatlist[s_i].editedObs.end())) // k_i为时间序列终点, 跳出
							break;
						else  
						{   
							k   = k_i;    // 新弧段的起点设置
							k_i = k;
							continue;
						}
					}					
				}	
				if(dsampleSat.editedObs.size() > 0)
				{
					dsampleSatlist.push_back(dsampleSat);
					pbySatList[dsampleSat.Id] = 1;
				}				
			}	
            // 将测站结构的数据转换为历元结构
			dsampleEpochlist.clear();
			for(DayTime t_i = t0; t_i - te <= 0; t_i = t_i + nSampleSpan)
			{
				Rinex2_1_EditedObsEpoch editedObsEpoch;
                editedObsEpoch.byEpochFlag = 0;      // 历元标记暂时没有使用，先标记为0
				editedObsEpoch.t           = t_i;
				// 计算对流层延迟先验值//2012.1.15增加
				double dmjd = TimeCoordConvert::DayTime2MJD(editedObsEpoch.t);
				BLH    blh;
				double undu;
				editedObsEpoch.humidity = 50;
				TimeCoordConvert::XYZ2BLH(posStaion,blh);
				blh.B = blh.B * PI/180;//转化为弧度
				blh.L = blh.L * PI/180;
				GlobalPT(dmjd,blh.B,blh.L,blh.H,editedObsEpoch.pressure,editedObsEpoch.temperature,undu);
				Saastamoinen_model(editedObsEpoch.temperature,50,editedObsEpoch.pressure,blh.B,blh.H - undu,editedObsEpoch.tropZenithDelayPriori_H,editedObsEpoch.tropZenithDelayPriori_W);
				
				editedObsEpoch.editedObs.clear();
				// 遍历每颗 星的数据列表
				for(size_t s_j = 0; s_j < dsampleSatlist.size(); s_j++)
				{// 判断当前时刻的数据是否符合要求(!前提是预处理期间，时间标签未被改动!)
					Rinex2_1_EditedObsEpochMap::iterator it;
					if((it = dsampleSatlist[s_j].editedObs.find(editedObsEpoch.t)) != dsampleSatlist[s_j].editedObs.end())
					{
						if(btroCor && it->second.Elevation > 0)
						{
							double gmfh,gmfw;   // 2013.5.9增加，直接修正对流层延迟
							GlobalMF(dmjd,blh.B,blh.L,blh.H - undu,it->second.Elevation * PI/180,gmfh,gmfw);
							double  trocor = editedObsEpoch.tropZenithDelayPriori_H * gmfh + editedObsEpoch.tropZenithDelayPriori_W * gmfw;
							it->second.obsTypeList[nObsTypes_L1].obs.data -=  (trocor/WAVELENGTH_L1); 
							it->second.obsTypeList[nObsTypes_L2].obs.data -=  (trocor/WAVELENGTH_L2); 
							it->second.obsTypeList[nObsTypes_P1].obs.data -=  trocor; 
							it->second.obsTypeList[nObsTypes_P2].obs.data -=  trocor; //
						}
						it->second.nObsTime = (int)dsampleEpochlist.size();//2012.12.29增加						
						editedObsEpoch.editedObs.insert(Rinex2_1_EditedObsSatMap::value_type(dsampleSatlist[s_j].Id, it->second));						
					}
				}		
				if(editedObsEpoch.editedObs.size() > 0)//2013.7.8修改
				{
					editedObsEpoch.bySatCount = (BYTE)editedObsEpoch.editedObs.size();
					dsampleEpochlist.push_back(editedObsEpoch);
				}			    
			}
			if(dsampleEpochlist.size() > 0 )
			{
				desampleFile.m_data.clear();
				desampleFile.m_data = dsampleEpochlist;
				// 更新文件头信息, 更新初始观测时刻和最后观测时刻
				desampleFile.m_header = editedObsFile.m_header;
				desampleFile.m_header.tmStart = dsampleEpochlist.front().t;
				desampleFile.m_header.tmEnd   = dsampleEpochlist.back().t;
				// 综合统计可视卫星列表
				desampleFile.m_header.pbySatList.clear();
				for(int i = 0; i < MAX_PRN + 1; i++)
				{
					if(pbySatList[i] == 1)
					{
						desampleFile.m_header.pbySatList.push_back(BYTE(i));
					}
				}
				desampleFile.m_header.bySatCount = BYTE(desampleFile.m_header.pbySatList.size());
				desampleFile.m_header.Interval   = double(nSampleSpan);
				

				// 文件创建日期
				DayTime T_Now;
				T_Now.Now();
				sprintf(desampleFile.m_header.szFileDate,"%04d-%02d-%02d %02d:%02d:%02d",T_Now.year,T_Now.month,T_Now.day,
																			T_Now.hour,T_Now.minute,int(T_Now.second));
				sprintf(desampleFile.m_header.szProgramName,"%-20s","desampling_edt");
				sprintf(desampleFile.m_header.szProgramAgencyName,"%-20s","NUDT");
				
				// 注释行
				desampleFile.m_header.pstrCommentList.clear();
				char szComment[100];
				sprintf(szComment,"%3d%-57s%20s\n", nSampleSpan,"s desampling, with unsmoothed method",Rinex2_1_MaskString::szComment);
				desampleFile.m_header.pstrCommentList.push_back(szComment);
				return true;
			}
			else
			{
				printf("没有满足降采样要求的观测数据！\n");
				return false;
			}
		}
		bool BDObsPreproc::desampling_unsmoothed_GPS(POS3D posStaion,Rinex2_1_EditedObsFile &editedObsFile,Rinex2_1_EditedObsFile &desampleFile,bool btroCor, int nSampleSpan)		
		{
			if(editedObsFile.isEmpty())
			{
				printf("预处理后的观测数据为空！\n");
				return false;
			}
			double WAVELENGTH_L1 = BD_WAVELENGTH_L1;
			double WAVELENGTH_L2 = BD_WAVELENGTH_L2;
			char  cSatSystem = editedObsFile.m_header.getSatSystemChar();
			if(cSatSystem != 'C')//2014/10/08,增加GPS系统考虑
			{
				WAVELENGTH_L1 = GPS_WAVELENGTH_L1;
				WAVELENGTH_L2 = GPS_WAVELENGTH_L2;
			}
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_L2 = -1, nObsTypes_C1 = -1, nObsTypes_P1 = -1, nObsTypes_P2 = -1;
			for(int i = 0; i < editedObsFile.m_header.byObsTypes; i++)
			{
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_C1)
					nObsTypes_C1 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1 = i;
				if(editedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2 = i;								
			}
			if(nObsTypes_P1 == -1 || nObsTypes_P2 == -1 || nObsTypes_L1 == -1 || nObsTypes_L2 == -1)
			{
				if(nObsTypes_P1 == -1 && nObsTypes_C1 != -1 && nObsTypes_P2 != -1 && nObsTypes_L1 != -1 && nObsTypes_L2 != -1)
				{
					// TODO: 添加C1/P2型接收机的数据预处理策略
					//printf("暂无C1/P2型接收机处理策略！ \n");
					//return false;
					nObsTypes_P1 = nObsTypes_C1;	// 暂时将 C1 直接赋给 P1 (2013/6/26)
				}
				else
				{
					printf("观测数据类型不足！\n");
					return false;
				}
			}
			BYTE pbySatList[MAX_PRN + 1];                  // 统计降采样后的观测卫星
			for( int i = 0; i < MAX_PRN + 1; i++ )
				pbySatList[i] = 0;
			DayTime t0 = editedObsFile.m_header.tmStart;   // 文件的起始时间
			t0.hour   = 0;                                 // 2013.5.30增加，不同测站观测数据的起始时间可能不一致,保证采样起始时间相同
			t0.minute = 0;                                 // 
			t0.second = 0;
			DayTime te = editedObsFile.m_header.tmEnd;     // 文件的结束时间
			vector<Rinex2_1_EditedObsEpoch> dsampleEpochlist;
			vector<Rinex2_1_EditedObsSat>   editedObsSatlist,dsampleSatlist;
			editedObsFile.getEditedObsSatList(editedObsSatlist);			
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)		
			{	// 分弧段降采样，方便记录周跳信息
				Rinex2_1_EditedObsSat  dsampleSat;                     // 某颗卫星降采样后的观测数据
				dsampleSat.Id = editedObsSatlist[s_i].Id;			
				Rinex2_1_EditedObsEpochMap::iterator k = editedObsSatlist[s_i].editedObs.begin();   // 记录新弧段起始点
				Rinex2_1_EditedObsEpochMap::iterator k_i = editedObsSatlist[s_i].editedObs.begin(); // 记录新弧段终止点
				while(1)
				{					
					if(!(++k_i != editedObsSatlist[s_i].editedObs.end()))   // k_i 为时间序列终点
						goto newArc;
					else
					{
						// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
						if(k_i->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_SLIP)
						{							
							continue;
						}							
						else // k_i 为新弧段的起点
							goto newArc;
					}
					newArc:  // 本弧段[k，k_i]数据处理 
					{					
						Rinex2_1_EditedObsSat  dsampleSatarc;                 // 某个弧段降采样后的观测数据						
						for(Rinex2_1_EditedObsEpochMap::iterator it = k; it != k_i; ++it)
						{							
							// 给定一个时间 t，根据 nSampleSpan 大小，量化成一个有效时间 ti，
							// 通过判断 t 与 ti 的接近程度，来决定 t 是否就是所找的输出点
							double t = it->first - t0;
							double integer  = floor(t / nSampleSpan);      // 量化过程
							double fraction = t - integer * nSampleSpan;   // 接近程度判断
							if(fabs(fraction) < 0.05)
							{// 寻找到输出点							 
								dsampleSatarc.editedObs.insert(Rinex2_1_EditedObsEpochMap::value_type(it->first,it->second));									
							}
						}
						//保留周跳标记
						for(Rinex2_1_EditedObsEpochMap::iterator dit = dsampleSatarc.editedObs.begin(); dit != dsampleSatarc.editedObs.end(); ++dit)
						{
							if(dit->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{   //将第一个非野值点的标记更新为周跳
								if(dit->second.obsTypeList[nObsTypes_L1].byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
								{
									dit->second.obsTypeList[nObsTypes_L1].byEditedMark1 = k->second.obsTypeList[nObsTypes_L1].byEditedMark1;
									dit->second.obsTypeList[nObsTypes_L1].byEditedMark2 = k->second.obsTypeList[nObsTypes_L1].byEditedMark2;
									dit->second.obsTypeList[nObsTypes_L2].byEditedMark1 = k->second.obsTypeList[nObsTypes_L2].byEditedMark1;
									dit->second.obsTypeList[nObsTypes_L2].byEditedMark2 = k->second.obsTypeList[nObsTypes_L2].byEditedMark2;
									break;
								}
								else if(dit->second.obsTypeList[nObsTypes_L1].byEditedMark1 == TYPE_EDITEDMARK_SLIP)
									break;
							}
						}
						//将包含完整的预处理信息的观测数据保留下来
						for(Rinex2_1_EditedObsEpochMap::iterator dit = dsampleSatarc.editedObs.begin(); dit != dsampleSatarc.editedObs.end(); ++dit)
							dsampleSat.editedObs.insert(Rinex2_1_EditedObsEpochMap::value_type(dit->first,dit->second));
						if(!(k_i != editedObsSatlist[s_i].editedObs.end())) // k_i为时间序列终点, 跳出
							break;
						else  
						{   
							k   = k_i;    // 新弧段的起点设置
							k_i = k;
							continue;
						}
					}					
				}	
				if(dsampleSat.editedObs.size() > 0)
				{
					dsampleSatlist.push_back(dsampleSat);
					pbySatList[dsampleSat.Id] = 1;
				}				
			}	
            // 将测站结构的数据转换为历元结构
			dsampleEpochlist.clear();
			for(DayTime t_i = t0; t_i - te <= 0; t_i = t_i + nSampleSpan)
			{
				Rinex2_1_EditedObsEpoch editedObsEpoch;
                editedObsEpoch.byEpochFlag = 0;      // 历元标记暂时没有使用，先标记为0
				editedObsEpoch.t           = t_i;
				// 计算对流层延迟先验值//2012.1.15增加
				double dmjd = TimeCoordConvert::DayTime2MJD(editedObsEpoch.t);
				BLH    blh;
				double undu;
				editedObsEpoch.humidity = 50;
				TimeCoordConvert::XYZ2BLH(posStaion,blh);
				blh.B = blh.B * PI/180;//转化为弧度
				blh.L = blh.L * PI/180;
				GlobalPT(dmjd,blh.B,blh.L,blh.H,editedObsEpoch.pressure,editedObsEpoch.temperature,undu);
				Saastamoinen_model(editedObsEpoch.temperature,50,editedObsEpoch.pressure,blh.B,blh.H - undu,editedObsEpoch.tropZenithDelayPriori_H,editedObsEpoch.tropZenithDelayPriori_W);
				
				editedObsEpoch.editedObs.clear();
				// 遍历每颗 星的数据列表
				for(size_t s_j = 0; s_j < dsampleSatlist.size(); s_j++)
				{// 判断当前时刻的数据是否符合要求(!前提是预处理期间，时间标签未被改动!)
					Rinex2_1_EditedObsEpochMap::iterator it;
					if((it = dsampleSatlist[s_j].editedObs.find(editedObsEpoch.t)) != dsampleSatlist[s_j].editedObs.end())
					{
						if(btroCor && it->second.Elevation > 0)
						{
							double gmfh,gmfw;   // 2013.5.9增加，直接修正对流层延迟
							GlobalMF(dmjd,blh.B,blh.L,blh.H - undu,it->second.Elevation * PI/180,gmfh,gmfw);
							double  trocor = editedObsEpoch.tropZenithDelayPriori_H * gmfh + editedObsEpoch.tropZenithDelayPriori_W * gmfw;
							it->second.obsTypeList[nObsTypes_L1].obs.data -=  (trocor/WAVELENGTH_L1); 
							it->second.obsTypeList[nObsTypes_L2].obs.data -=  (trocor/WAVELENGTH_L2); 
							it->second.obsTypeList[nObsTypes_P1].obs.data -=  trocor; 
							it->second.obsTypeList[nObsTypes_P2].obs.data -=  trocor; //
						}
						it->second.nObsTime = (int)dsampleEpochlist.size();//2012.12.29增加						
						editedObsEpoch.editedObs.insert(Rinex2_1_EditedObsSatMap::value_type(dsampleSatlist[s_j].Id, it->second));						
					}
				}		
				if(editedObsEpoch.editedObs.size() > 0)//2013.7.8修改
				{
					editedObsEpoch.bySatCount = (BYTE)editedObsEpoch.editedObs.size();
					dsampleEpochlist.push_back(editedObsEpoch);
				}			    
			}
			if(dsampleEpochlist.size() > 0 )
			{
				desampleFile.m_data.clear();
				desampleFile.m_data = dsampleEpochlist;
				// 更新文件头信息, 更新初始观测时刻和最后观测时刻
				desampleFile.m_header = editedObsFile.m_header;
				desampleFile.m_header.tmStart = dsampleEpochlist.front().t;
				desampleFile.m_header.tmEnd   = dsampleEpochlist.back().t;
				// 综合统计可视卫星列表
				desampleFile.m_header.pbySatList.clear();
				for(int i = 0; i < MAX_PRN + 1; i++)
				{
					if(pbySatList[i] == 1)
					{
						desampleFile.m_header.pbySatList.push_back(BYTE(i));
					}
				}
				desampleFile.m_header.bySatCount = BYTE(desampleFile.m_header.pbySatList.size());
				desampleFile.m_header.Interval   = double(nSampleSpan);
				

				// 文件创建日期
				DayTime T_Now;
				T_Now.Now();
				sprintf(desampleFile.m_header.szFileDate,"%04d-%02d-%02d %02d:%02d:%02d",T_Now.year,T_Now.month,T_Now.day,
																			T_Now.hour,T_Now.minute,int(T_Now.second));
				sprintf(desampleFile.m_header.szProgramName,"%-20s","desampling_edt");
				sprintf(desampleFile.m_header.szProgramAgencyName,"%-20s","NUDT");
				
				// 注释行
				desampleFile.m_header.pstrCommentList.clear();
				char szComment[100];
				sprintf(szComment,"%3d%-57s%20s\n", nSampleSpan,"s desampling, with unsmoothed method",Rinex2_1_MaskString::szComment);
				desampleFile.m_header.pstrCommentList.push_back(szComment);
				return true;
			}
			else
			{
				printf("没有满足降采样要求的观测数据！\n");
				return false;
			}
		}
		// 子程序名称： exportSP3File_GPST   
		// 功能：导出GPS时间系统的北斗广播星历(sp3格式)
		// 变量类型：strnavFilePath      : 导航文件路径
		//           t0               : sp3文件起始时间
		//           t1               : sp3文件结束时间
        //           interval         : sp3文件采样间隔
		// 输入：strnavFilePath，t0，t1
		// 输出：
		// 语言：C++
		// 创建者：刘俊宏
		// 创建时间：2013/03/05
		// 版本时间：
		// 修改记录：
		// 备注： 由于目前获得的观测数据是GPST时，而广播星历是BDT，为了统一时间系统，将广播星历统一到GPST
		bool BDObsPreproc::exportSP3File_GPST(string strnavFilePath,GPST t0, GPST t1,double interval)
		{
			// 分析 navFilePath 路径, 提取根目录和文件名
			string navFileName = strnavFilePath.substr(strnavFilePath.find_last_of("\\") + 1);
			string folder = strnavFilePath.substr(0, strnavFilePath.find_last_of("\\"));
			string navFileName_noexp = navFileName.substr(0, navFileName.find_last_of("."));
			char sp3FilePath[100];
			sprintf(sp3FilePath,"%s\\%s.sp3", folder.c_str(), navFileName_noexp.c_str());
			Rinex2_1_NavFile	navFile;	// 导航数据文件
			SP3File             sp3FileBDT,sp3FileGPST;    // sp3星历文件
			if(!navFile.open(strnavFilePath))
			{
				printf("导航文件无法打开！");
				return false;
			}
			
			//GPST  t0(2012,1,18,0,0,0);
			//GPST  t1(2012,1,19,0,0,0);
			//double interval = 5 * 60;
			navFile.exportSP3File(sp3FilePath,t0 - 3600,t1 + 3600); // 导出北斗时间的sp3文件
			sp3FileBDT.open(sp3FilePath);
			for(int i = 0; t0 + interval * i - t1 <= 0; i++)
			{
				SP3Epoch   sp3epoch;
				GPST gps_t  = t0 + interval * i;
				BDT  bds_t = TimeCoordConvert::GPST2BDT(gps_t);
				for(size_t s_i = 0; s_i < sp3FileBDT.m_header.pstrSatNameList.size(); s_i ++)
				{
					SP3Datum  sp3Datum;
					if(sp3FileBDT.getEphemeris(bds_t,sp3FileBDT.m_header.pstrSatNameList[s_i],sp3Datum))
					{
						sp3epoch.t = gps_t;
						sp3Datum.pos.x = sp3Datum.pos.x/1000;
						sp3Datum.pos.y = sp3Datum.pos.y/1000;
						sp3Datum.pos.z = sp3Datum.pos.z/1000;
						sp3Datum.clk = 0;
						sp3epoch.sp3.insert(SP3SatMap::value_type(sp3FileBDT.m_header.pstrSatNameList[s_i],sp3Datum));
					}			
				}
				sp3FileGPST.m_data.push_back(sp3epoch);
			}//
			sp3FileGPST.m_header = sp3FileBDT.m_header;	
			sp3FileGPST.m_header.tmStart = sp3FileGPST.m_data.front().t;
			sprintf(sp3FileGPST.m_header.szTimeSystem, "GPS");
			sp3FileGPST.write(sp3FilePath);
			return true;
		}
		// 子程序名称： exportCLKFile_GPST   
		// 作用：导出clk格式钟差文件
		// 类型：strCLKfileName  : 文件名称
		//       T_Begin         : 星历开始时间
		//       T_End           : 星历结束时间
		//       spanSeconds    : 星历相邻时间点的时间间隔，默认2分钟
		// 输入：T_Begin, T_End, dSpanSeconds
		// 输出：
		// 语言：C++
		// 创建时间：2008/01/22
		// 版本时间：
		// 修改记录：
		// 备注：
		bool BDObsPreproc::exportCLKFile_GPST(string strnavFilePath,GPST t0, GPST t1,double interval)
		{
			// 分析 navFilePath 路径, 提取根目录和文件名
			string navFileName = strnavFilePath.substr(strnavFilePath.find_last_of("\\") + 1);
			string folder = strnavFilePath.substr(0, strnavFilePath.find_last_of("\\"));
			string navFileName_noexp = navFileName.substr(0, navFileName.find_last_of("."));
			char clkFilePath[100];
			sprintf(clkFilePath,"%s\\%s.clk", folder.c_str(), navFileName_noexp.c_str());
			Rinex2_1_NavFile	    navFile;	// 导航数据文件
			CLKFile                 clkfile;    // 卫星钟差文件
			if(!navFile.open(strnavFilePath))
			{
				printf("导航文件无法打开！");
				return false;
			}			
			BYTE pbySatList[MAX_PRN_GPS];       // 卫星列表
			for(int i = 0; i < MAX_PRN_GPS; i++)
				pbySatList[i] = 0;
			DayTime T =  TimeCoordConvert::GPST2BDT(t0);//导出时间系统为GPS时间系统			
			int k = 0;
			while( t1 - T >= 0 )
			{
				CLKEpoch clkEpoch;				
				clkEpoch.t = TimeCoordConvert::BDT2GPST(T);
				clkEpoch.ARList.clear();
				clkEpoch.ASList.clear();
				for(int i = 0; i < MAX_PRN_GPS; i++)
				{
					POSCLK posclk;
					if(navFile.getEphemeris(T, i, posclk))
					{
						pbySatList[i] = 1;
						CLKDatum   ASDatum;
						ASDatum.count = 2;
						char  satname[4];
						if(navFile.m_typeSatSystem == 0)     //GPS
							sprintf(satname,"G%2d",i);
						else
							sprintf(satname,"C%2d",i);//北斗
						satname[3] = '\0';
						ASDatum.name = satname;						
						ASDatum.clkBias = posclk.clk;
						ASDatum.clkBiasSigma = 0;
						clkEpoch.ASList.insert(CLKMap::value_type(satname, ASDatum));
					}
				}
				clkfile.m_data.push_back(clkEpoch);
				T = T + interval;
			}
			// 书写文件头
			sprintf(clkfile.m_header.szRinexVersion, "2.0");
			clkfile.m_header.cFileType = 'C';
			sprintf(clkfile.m_header.szProgramName,"NUDTTK");
			sprintf(clkfile.m_header.szAgencyName,"NUDT");
			clkfile.m_header.LeapSecond = 0;
			clkfile.m_header.ClockDataTypeCount = 1;
			clkfile.m_header.pstrClockDataTypeList.clear();
			clkfile.m_header.pstrClockDataTypeList.push_back("AS");
			sprintf(clkfile.m_header.szACShortName,"NUDT");
			clkfile.m_header.nStaCount = 0;
			sprintf(clkfile.m_header.szStaCoordFrame,"IGS00 : IGS REALIZATION OF THE ITRF2000");
			// 综合统计可视卫星列表
			clkfile.m_header.pszSatList.clear();
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				if(pbySatList[i] == 1)
				{
					char szPRN[4];
					if(navFile.m_typeSatSystem == 0) //GPS
						sprintf(szPRN, "G%2d", i);
					else
						sprintf(szPRN, "C%2d", i);;//北斗
					szPRN[3] = '\0';
					clkfile.m_header.pszSatList.push_back(szPRN);
				}
			}
			clkfile.m_header.bySatCount = BYTE(clkfile.m_header.pszSatList.size());
			clkfile.write(clkFilePath);
			return true;
		}

	}
}