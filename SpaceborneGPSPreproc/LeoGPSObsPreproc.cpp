#include "LeoGPSObsPreproc.hpp"
#include "GNSSBasicCorrectFunc.hpp"
#include "RuningInfoFile.hpp"
#include "TimeCoordConvert.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace SpaceborneGPSPreproc
	{
		LeoGPSObsPreproc::LeoGPSObsPreproc(void)
		{
			m_strPreprocPath = "";
			m_matAxisAnt2Body = TimeCoordConvert::rotate(PI, 1);
		}

		LeoGPSObsPreproc::~LeoGPSObsPreproc(void)
		{
		}

		void LeoGPSObsPreproc::setPreprocPath(string strPreprocPath)
		{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
			m_strPreprocPath = strPreprocPath;
		}

		void LeoGPSObsPreproc::setSP3File(SP3File sp3File)
		{
			m_sp3File = sp3File;
		}

		void LeoGPSObsPreproc::setCLKFile(CLKFile clkFile)
		{
			m_clkFile = clkFile;
		}

		bool LeoGPSObsPreproc::loadSP3File(string  strSp3FileName)
		{
			return m_sp3File.open(strSp3FileName);
		}

		bool LeoGPSObsPreproc::loadCLKFile(string  strCLKFileName)
		{
			return m_clkFile.open(strCLKFileName);
		}

		bool LeoGPSObsPreproc::loadObsFile(string  strObsFileName)
		{
			return m_obsFile.open(strObsFileName);
		}

		void LeoGPSObsPreproc::setAntPhaseCenterOffset(POS3D posRTN)
		{
			m_pcoAnt = posRTN;
		}

		// 子程序名称： setAntPhaseCenterOffset   
		// 作用：设置天线偏移量
		// 类型：posBody         : 天线偏移量, 星固系
		//       matAxisBody2RTN : 星固系到轨道系的固定准换矩阵, 用于存在固定偏差角度的三轴稳定控制
		// 输入：posBody, matAxisBody2RTN
		// 输出：m_pcoAnt 
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2015/3/11
		// 版本时间：
		// 修改记录：
		// 其它：
		void LeoGPSObsPreproc::setAntPhaseCenterOffset(POS3D posBody, Matrix matAxisBody2RTN)
		{
			Matrix matPCO(3, 1);
			matPCO.SetElement(0, 0, posBody.x);
			matPCO.SetElement(1, 0, posBody.y);
			matPCO.SetElement(2, 0, posBody.z);
			matPCO = matAxisBody2RTN * matPCO;
			m_pcoAnt.x = matPCO.GetElement(0, 0);
			m_pcoAnt.y = matPCO.GetElement(1, 0);
			m_pcoAnt.z = matPCO.GetElement(2, 0);
		}

		void LeoGPSObsPreproc::setLeoOrbitList(vector<TimePosVel> leoOrbitList)
		{
			m_leoOrbitList = leoOrbitList;
		}

		BYTE LeoGPSObsPreproc::obsPreprocInfo2EditedMark1(int obsPreprocInfo)
		{
			return BYTE(getIntBit(obsPreprocInfo, 1));
		}

		BYTE LeoGPSObsPreproc::obsPreprocInfo2EditedMark2(int obsPreprocInfo)
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
		bool LeoGPSObsPreproc::datalist_epoch2sat(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist, vector<Rinex2_1_EditedObsSat>& editedObsSatlist)
		{
			Rinex2_1_EditedObsSat editedObsSatlist_max[MAX_PRN_GPS]; 
			for(int i = 0; i < MAX_PRN_GPS; i++)
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
			for(int i = 0; i < MAX_PRN_GPS; i++)
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
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				if(editedObsSatlist_max[i].editedObs.size() > 0)
				{
					editedObsSatlist[validcount] = editedObsSatlist_max[i];
					validcount++;
				}
			}
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
		bool LeoGPSObsPreproc::datalist_sat2epoch(vector<Rinex2_1_EditedObsSat>& editedObsSatlist, vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist)
		{
			if(editedObsSatlist.size() <= 0)
				return false;
			editedObsEpochlist.clear();
			editedObsEpochlist.resize(m_obsFile.m_data.size());
			for(size_t s_i = 0; s_i < m_obsFile.m_data.size(); s_i++)
			{
				Rinex2_1_LeoEditedObsEpoch editedObsEpoch;
                editedObsEpoch.byEpochFlag = m_obsFile.m_data[s_i].byEpochFlag;
				editedObsEpoch.t           = m_obsFile.m_data[s_i].t;
				editedObsEpoch.editedObs.clear();
				// 遍历每颗 GPS 卫星的数据列表
				for(size_t s_j = 0; s_j < editedObsSatlist.size(); s_j++)
				{// 判断当前时刻的数据是否符合要求(!前提是预处理期间，时间标签未被改动!)
					Rinex2_1_EditedObsEpochMap::const_iterator it;
					if((it = editedObsSatlist[s_j].editedObs.find(editedObsEpoch.t)) != editedObsSatlist[s_j].editedObs.end())
					{
						editedObsEpoch.editedObs.insert(Rinex2_1_EditedObsSatMap::value_type(editedObsSatlist[s_j].Id, it->second));
					}
				}
			    editedObsEpochlist[s_i] = editedObsEpoch;
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
		bool LeoGPSObsPreproc::getEditedObsEpochList(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist)
		{
			if(m_obsFile.isEmpty())
				return false;
			editedObsEpochlist.clear();
			// resize函数可提高效率
			editedObsEpochlist.resize(m_obsFile.m_data.size());
			/* 转换 CHAMP 一天数据, 耗时 11.70 秒左右 */
			for(size_t s_i = 0; s_i < m_obsFile.m_data.size(); s_i++)
			{
				Rinex2_1_LeoEditedObsEpoch editedObsEpoch;
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
		bool LeoGPSObsPreproc::getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist)
		{
			vector<Rinex2_1_LeoEditedObsEpoch> editedObsEpochlist;
			// 首先根据观测数据文件 m_obsFile 初始化每个时刻的预处理数据
			if(!getEditedObsEpochList(editedObsEpochlist))
				return	false;
			return datalist_epoch2sat(editedObsEpochlist, editedObsSatlist);
		}

		// 子程序名称： SinglePointPositioning_PIF   
		// 功能：构造伪码无电离层组合量PIF, 实现概略轨道确定, 同时输出统计量 rms_res, 供 RAIM 检验使用
		// 变量类型：index_P1             : 观测类型 P1 索引
		//           index_P2             : 观测类型 P2 索引
		//           frequence_L1         : 观测类型 P1 的频率
		//           frequence_L2         : 观测类型 P2 的频率
		//           obsEpoch             : 某时刻的预处理后观测数据
		//           posclk               : 概略轨道位置、钟差, 返回值存储
		//           eyeableGPSCount      : 可见有效 GPS 卫星个数
		//           pdop                 : 定位的几何精度因子
		//           rms_res              : 定位残差的 Q 统计量, 主要用于后续 RAIM 检验
		//           threshold            : 高斯牛顿迭代收敛阈值
		// 输入：index_P1, index_P2, obsEpoch, posclk, threshold
		// 输出：posclk, pdop, eyeableGPSCount, rms_res
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/5/10
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool LeoGPSObsPreproc::SinglePointPositioning_PIF(int index_P1, int index_P2, double frequence_L1,double frequence_L2, Rinex2_1_LeoEditedObsEpoch obsEpoch, POSCLK& posclk, int& eyeableGPSCount, double& pdop, double& rms_res, double threshold)
		{
			char cSatSystem = m_obsFile.m_header.getSatSystemChar(); // 2012/01/03, 增加北斗数据的处理
			pdop = 0;
			eyeableGPSCount = 0;
            Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); 
			while(it != obsEpoch.editedObs.end())
			{
				Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[index_P2];
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
				{
					Rinex2_1_EditedObsSatMap::iterator jt = it;
					++it;
					obsEpoch.editedObs.erase(jt);
					continue;
				}
				else
				{
					eyeableGPSCount++;
					++it;
					continue;
				}
			}
			if(eyeableGPSCount < 4)  // 可见星要大于或等于4颗
				return false;
			Matrix matObs(eyeableGPSCount, 1); // 伪码无电离层组合
			Matrix matDy(eyeableGPSCount, 1);  // 高斯牛顿迭代的观测值改进
			Matrix matH(eyeableGPSCount,  4);  // 高斯牛顿迭代的线性化展开矩阵
			Matrix matAppPosClk(4, 1);         // 高斯牛顿迭代的概略点, 初始化可取地心
			Matrix matY(eyeableGPSCount,  1);  // 高斯牛顿迭代的观测值
			Matrix matGPSSp3Clk(eyeableGPSCount, 4);
			matAppPosClk.SetElement(0, 0, posclk.x);
			matAppPosClk.SetElement(1, 0, posclk.y);
			matAppPosClk.SetElement(2, 0, posclk.z);
			matAppPosClk.SetElement(3, 0, posclk.clk);
			// 双频 P 码消除电离层组合系数
			double coefficient_IF = 1 / (1 - pow(frequence_L1 / frequence_L2, 2));
			// 初始化信号真实接收时间(t_Receive) = 观测时间 - 接收机钟差
			GPST t_Receive;
			if(m_PreprocessorDefine.bOn_ClockEliminate)
				t_Receive = obsEpoch.t; // 认为sy1软校正过的采样时刻是较为准确的, 2010/06/12
			else
				t_Receive = obsEpoch.t - posclk.clk / SPEED_LIGHT;
			int j = 0;
			for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{// 双频 P 码消除电离层组合 R = R1- (R1 - R2) / (1 - (f1^2 / f2^2))
				double y = it->second.obsTypeList[index_P1].obs.data - (it->second.obsTypeList[index_P1].obs.data - it->second.obsTypeList[index_P2].obs.data) * coefficient_IF;
				matObs.SetElement(j, 0, y);
				j++;
			}
			j = 0;
			for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{
				double y = matObs.GetElement(j, 0);
				// 迭代计算卫星信号传播时间
				double delay = 0;
				int nPRN = it->first; // 第 j 颗可见GPS卫星的卫星号
				char szSatName[4];
				sprintf(szSatName, "%1c%02d", cSatSystem, nPRN);
				szSatName[3] = '\0';
				SP3Datum sp3Datum;
				m_sp3File.getEphemeris_PathDelay(obsEpoch.t, posclk, szSatName, delay, sp3Datum);
				// 对 GPS 卫星星历进行地球自转改正
				GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
				// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
				GPST t_Transmit = t_Receive - delay;
				// 对观测值 y 进行误差修正,包括 GPS 卫星钟差改正, GPS卫星相对论修正等, 相位中心修正在数据预处理中暂时不考虑
				// 1. GPS卫星钟差改正
				CLKDatum ASDatum;
				m_clkFile.getSatClock(t_Transmit, nPRN, ASDatum, 3, cSatSystem); // 获得 GPS 信号发射时间的卫星钟差改正
				double correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT;  
				y = y + correct_gpsclk;
				// 2. GPS卫星相对论改正
				double correct_relativity = (sp3Datum.pos.x * sp3Datum.vel.x 
                                           + sp3Datum.pos.y * sp3Datum.vel.y
                                           + sp3Datum.pos.z * sp3Datum.vel.z) * (-2.0) / SPEED_LIGHT;
				y = y + correct_relativity;
				matGPSSp3Clk.SetElement(j, 0, sp3Datum.pos.x);
				matGPSSp3Clk.SetElement(j, 1, sp3Datum.pos.y);
				matGPSSp3Clk.SetElement(j, 2, sp3Datum.pos.z);
				matGPSSp3Clk.SetElement(j, 3, ASDatum.clkBias);
				// 根据 sp3Datum 和接收机概略位置, 计算概略距离
				double distance;
				distance = pow(posclk.x - sp3Datum.pos.x, 2)
                         + pow(posclk.y - sp3Datum.pos.y, 2)
                         + pow(posclk.z - sp3Datum.pos.z, 2);
				distance = sqrt(distance);
				matY.SetElement(j, 0, y); 
				matDy.SetElement(j, 0, y - (distance + posclk.clk)); // 计算观测值与概略距离之差, 包含钟差
				// 利用概略点计算视线方向
				matH.SetElement(j, 0, (posclk.x - sp3Datum.pos.x) / distance);
				matH.SetElement(j, 1, (posclk.y - sp3Datum.pos.y) / distance);
				matH.SetElement(j, 2, (posclk.z - sp3Datum.pos.z) / distance);
				matH.SetElement(j, 3, 1.0);
				j++;
			}
			const int max_GaussNewtonCount = 10; // 非线性迭代次数阈值
			int k_GaussNewton = 0; // 非线性迭代次数
			double delta = 100; // 非线性迭代收敛判断变量
			double delta_max = 1.0E+8; // 防止迭代发散, 2008/03/03
			Matrix matDx;
			while(1)
			{
				Matrix matHt = matH.Transpose();
				Matrix matHH_inv = (matHt * matH).Inv_Ssgj();
				matDx = matHH_inv * matHt * matDy;
				delta = matDx.Abs().Max();
				k_GaussNewton = k_GaussNewton + 1;
				if(delta <= threshold) 
				{
					// 空间三维位置精度因子 pdop = sqrt(q11 + q22 + q33), 2007/07/10
					pdop = sqrt(matHH_inv.GetElement(0,0) + matHH_inv.GetElement(1,1) + matHH_inv.GetElement(2,2));
					if(pdop > m_PreprocessorDefine.max_pdop)
					{// 2008/07/01添加
						rms_res = DBL_MAX;
						pdop = 0;
						return false;
					}
					rms_res = 0;
					if(eyeableGPSCount >= 5)
					{// 只有卫星数大于5, 才能计算 RAIM 检验统计量
						double sse = 0;
						for(int i = 0; i < eyeableGPSCount; i++)
							sse += pow(matDy.GetElement(i, 0), 2);
						rms_res = sqrt(sse / (eyeableGPSCount - 4));
					} 
					break;
				}
				// 控制迭代次数, 控制改进增量
				if(k_GaussNewton >= max_GaussNewtonCount || fabs(delta) >= delta_max)
				{
					rms_res = DBL_MAX;
					pdop = 0;
					//printf("%s 迭代次数 %d 溢出, delta = %f !\n", obsEpoch.t.toString().c_str(), k_GaussNewton, delta);
					return false;
				}
				// 更新概略点
				matAppPosClk = matAppPosClk + matDx;
				posclk.x   = matAppPosClk.GetElement(0,0);
				posclk.y   = matAppPosClk.GetElement(1,0);
				posclk.z   = matAppPosClk.GetElement(2,0);
				posclk.clk = matAppPosClk.GetElement(3,0);
				// 更新信号接收参考时间
				GPST t_Receive;
				if(m_PreprocessorDefine.bOn_ClockEliminate)
					t_Receive = obsEpoch.t; // 认为sy1软校正过的采样时刻是较为准确的, 2010/06/12
				else
					t_Receive = obsEpoch.t - posclk.clk / SPEED_LIGHT;
				j = 0;
				for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
				{
					if(delta > 100.0) // 如果改进量小于100m, 为减少计算量, 可不进行视线矢量 matH 的更新
					{
						double y = matObs.GetElement(j, 0);
						// 迭代计算卫星信号传播时间
						double delay = 0;
						int nPRN = it->first; // 第 j 颗可见GPS卫星的卫星号
						char szSatName[4];
						sprintf(szSatName, "%1c%02d", cSatSystem, nPRN);
						szSatName[3] = '\0';
						SP3Datum sp3Datum;
						m_sp3File.getEphemeris_PathDelay(obsEpoch.t, posclk, szSatName, delay, sp3Datum);
						// 对 GPS 卫星星历进行地球自转改正
						GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
						// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
						GPST t_Transmit = t_Receive - delay;
						// 对观测值 y 进行误差修正,包括 GPS 卫星钟差改正, GPS卫星相对论修正等, 相位中心修正在数据预处理中暂时不考虑
						// 1. GPS卫星钟差改正
						CLKDatum ASDatum;
						m_clkFile.getSatClock(t_Transmit, nPRN, ASDatum, 3, cSatSystem); // 获得 GPS 信号发射时间的卫星钟差改正
						double correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT;  
						y = y + correct_gpsclk;
						// 2. GPS卫星相对论改正
						double correct_relativity = (sp3Datum.pos.x * sp3Datum.vel.x 
												   + sp3Datum.pos.y * sp3Datum.vel.y
												   + sp3Datum.pos.z * sp3Datum.vel.z) * (-2.0) / SPEED_LIGHT;
						y = y + correct_relativity;
						matGPSSp3Clk.SetElement(j, 0, sp3Datum.pos.x);
						matGPSSp3Clk.SetElement(j, 1, sp3Datum.pos.y);
						matGPSSp3Clk.SetElement(j, 2, sp3Datum.pos.z);
						matGPSSp3Clk.SetElement(j, 3, ASDatum.clkBias);
						// 根据 sp3Datum 和接收机概略位置, 计算概略距离
						double distance;
						distance = pow(posclk.x - sp3Datum.pos.x, 2)
								 + pow(posclk.y - sp3Datum.pos.y, 2)
								 + pow(posclk.z - sp3Datum.pos.z, 2);
						distance = sqrt(distance);
						matY.SetElement(j, 0, y); 
						matDy.SetElement(j, 0, y - (distance + posclk.clk)); // 计算观测值与概略距离之差, 包含钟差
						// 利用概略点计算视线方向
						matH.SetElement(j, 0, (posclk.x - sp3Datum.pos.x) / distance);
						matH.SetElement(j, 1, (posclk.y - sp3Datum.pos.y) / distance);
						matH.SetElement(j, 2, (posclk.z - sp3Datum.pos.z) / distance);
						matH.SetElement(j, 3, 1.0);
					}
					else
					{
						double distance = pow(posclk.x - matGPSSp3Clk.GetElement(j, 0), 2)
							            + pow(posclk.y - matGPSSp3Clk.GetElement(j, 1), 2)
							            + pow(posclk.z - matGPSSp3Clk.GetElement(j, 2), 2);
						distance = sqrt(distance);
						matDy.SetElement(j, 0, matY.GetElement(j, 0) - (distance + posclk.clk)); // 计算观测值与概略距离之差, 包含钟差
					}
					j++;
				}
			}
			matAppPosClk = matAppPosClk + matDx;
			posclk.x   = matAppPosClk.GetElement(0,0);
			posclk.y   = matAppPosClk.GetElement(1,0);
			posclk.z   = matAppPosClk.GetElement(2,0);
			posclk.clk = matAppPosClk.GetElement(3,0);
			return true;
		}
		// 子程序名称： RaimEstChannelBias_PIF   
		// 功能： 在 RAIM 检验中, 当卫星数大于6, 估算故障通道的码偏差
		// 变量类型：index_P1             : 观测类型 P1 索引
		//           index_P2             : 观测类型 P2 索引
		//           frequence_L1         : 观测类型 P1 的频率
		//           frequence_L2         : 观测类型 P2 的频率
		//           obsEpoch             : 某时刻的预处理后观测数据
        //           nPRN                 : 待估通道对应的GPS卫星编号
		//           posclk               : 概略轨道位置、钟差, 返回值存储
		//           channelBias          : 故障通道的码偏差估计结果
		//           threshold            : 高斯牛顿迭代收敛阈值
		// 输入：index_P1, index_P2, obsEpoch, nPRN, posclk
		// 输出：channelBias
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/5/10
		// 版本时间：
		// 修改记录：
		// 备注：
		bool LeoGPSObsPreproc::RaimEstChannelBias_PIF(int index_P1,int index_P2,double frequence_L1,double frequence_L2, Rinex2_1_LeoEditedObsEpoch obsEpoch, int nPRN, POSCLK& posclk, double& channelBias, double threshold)
		{
			char cSatSystem = m_obsFile.m_header.getSatSystemChar();
			double pdop = 0;
			channelBias = 0;
			int eyeableGPSCount = 0;
            Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); 
			while(it != obsEpoch.editedObs.end())
			{
				Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[index_P2];
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
				{
					Rinex2_1_EditedObsSatMap::iterator jt = it;
					++it;
					obsEpoch.editedObs.erase(jt);
					continue;
				}
				else
				{
					eyeableGPSCount++;
					++it;
					continue;
				}
			}
			if(eyeableGPSCount < 6)  // 可见星要大于或等于 6 颗
				return false;
			// 寻找 nPRN 颗 GPS 卫星的位置
			int npos_PRN = -1;
			int j = 0;
			for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{
				if(it->first == nPRN)
				{
					npos_PRN = j;
					break;
				}
				j++;
			}
			if(npos_PRN == -1)
				return false;
			Matrix matObs(eyeableGPSCount, 1); // 伪码无电离层组合
			Matrix matY(eyeableGPSCount,  1);  // 高斯牛顿迭代的观测值
			Matrix matDy(eyeableGPSCount, 1);  // 高斯牛顿迭代的观测值改进
			Matrix matH(eyeableGPSCount,  5);  // 高斯牛顿迭代的线性化展开矩阵
			Matrix matAppPosClkBias(5, 1);              // 高斯牛顿迭代的概略点--初始化为0
			matAppPosClkBias.SetElement(0, 0, posclk.x);
			matAppPosClkBias.SetElement(1, 0, posclk.y);
			matAppPosClkBias.SetElement(2, 0, posclk.z);
			matAppPosClkBias.SetElement(3, 0, posclk.clk);
			matAppPosClkBias.SetElement(4, 0, channelBias);
			// 双频 P 码消除电离层组合系数
			double coefficient_IF = 1 / (1 - pow(GPS_FREQUENCE_L1 / GPS_FREQUENCE_L2, 2));
			// 初始化信号真实接收时间(t_Receive) = 观测时间 - 接收机钟差
			GPST t_Receive;
			if(m_PreprocessorDefine.bOn_ClockEliminate)
				t_Receive = obsEpoch.t; // 认为sy1软校正过的采样时刻是较为准确的, 2010/06/12
			else
				t_Receive = obsEpoch.t - posclk.clk / SPEED_LIGHT;
			j = 0;
			for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{// 双频 P 码消除电离层组合 R = R1- (R1 - R2) / (1 - (f1^2 / f2^2))
				double y = it->second.obsTypeList[index_P1].obs.data - (it->second.obsTypeList[index_P1].obs.data - it->second.obsTypeList[index_P2].obs.data) * coefficient_IF;
				matObs.SetElement(j, 0, y);
				j++;
			}
			Matrix matGPSSp3Clk(eyeableGPSCount, 4);
			j = 0;
			for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{
				double y = matObs.GetElement(j, 0);
				// 迭代计算卫星信号传播时间
				double delay = 0;
				int nPRN = it->first; // 第 j 颗可见GPS卫星的卫星号
				char szSatName[4];
				sprintf(szSatName, "%1c%02d", cSatSystem, nPRN);
				szSatName[3] = '\0';
				SP3Datum sp3Datum;
				m_sp3File.getEphemeris_PathDelay(obsEpoch.t, posclk, szSatName, delay, sp3Datum);
				// 对 GPS 卫星星历进行地球自转改正
				GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
				// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
				GPST t_Transmit = t_Receive - delay;
				// 对观测值 y 进行误差修正,包括 GPS 卫星钟差改正, GPS卫星相对论修正等, 相位中心修正在数据预处理中暂时不考虑
				// 1. GPS卫星钟差改正
				CLKDatum ASDatum;
				m_clkFile.getSatClock(t_Transmit, nPRN, ASDatum, 3, cSatSystem); // 获得 GPS 信号发射时间的卫星钟差改正
				double correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT;  
				y = y + correct_gpsclk;
				// 2. GPS卫星相对论改正
				double correct_relativity = (sp3Datum.pos.x * sp3Datum.vel.x 
                                           + sp3Datum.pos.y * sp3Datum.vel.y
                                           + sp3Datum.pos.z * sp3Datum.vel.z) * (-2.0) / SPEED_LIGHT;
				y = y + correct_relativity;
				matGPSSp3Clk.SetElement(j, 0, sp3Datum.pos.x);
				matGPSSp3Clk.SetElement(j, 1, sp3Datum.pos.y);
				matGPSSp3Clk.SetElement(j, 2, sp3Datum.pos.z);
				matGPSSp3Clk.SetElement(j, 3, ASDatum.clkBias);
				// 根据 sp3Datum 和接收机概略位置, 计算概略距离
				double distance;
				distance = pow(posclk.x - sp3Datum.pos.x, 2)
                         + pow(posclk.y - sp3Datum.pos.y, 2)
                         + pow(posclk.z - sp3Datum.pos.z, 2);
				distance = sqrt(distance);
				matY.SetElement(j, 0, y); 
				// 利用概略点计算视线方向
				matH.SetElement(j, 0, (posclk.x - sp3Datum.pos.x) / distance);
				matH.SetElement(j, 1, (posclk.y - sp3Datum.pos.y) / distance);
				matH.SetElement(j, 2, (posclk.z - sp3Datum.pos.z) / distance);
				matH.SetElement(j, 3, 1.0);
				if(j == npos_PRN)
				{
					matDy.SetElement(j, 0, y - (distance + posclk.clk + channelBias) );
					matH.SetElement(j, 4, 1.0);
				}
				else
				{
					matDy.SetElement(j, 0, y - (distance + posclk.clk));
					matH.SetElement(j, 4, 0.0);
				}
				j++;
			}
			const int max_GaussNewtonCount = 10; // 非线性迭代次数阈值
			int k_GaussNewton = 0; // 非线性迭代次数
			double delta = 100; // 非线性迭代收敛判断变量
			double delta_max = 1.0E+8; // 防止迭代发散, 2008/03/03
			Matrix matDx;
			while(1)
			{
				Matrix matHt = matH.Transpose();
				Matrix matHH_inv = (matHt * matH).Inv_Ssgj();
				matDx = matHH_inv * matHt * matDy;
				delta = matDx.Abs().Max();
				k_GaussNewton = k_GaussNewton + 1;
				if(delta <= threshold) 
				{// 增加自我检测
					// pdop = sqrt(q11 + q22 + q33) 空间三维位置精度因子, 20070710
					pdop = sqrt(matHH_inv.GetElement(0,0) + matHH_inv.GetElement(1,1) + matHH_inv.GetElement(2,2));
					if(pdop > m_PreprocessorDefine.max_pdop)
					{// 2008-07-01添加
						pdop = 0;
						return false;
					}
					break;
				}
				// 控制迭代次数, 控制改进增量
				if(k_GaussNewton >= max_GaussNewtonCount || fabs(delta) >= delta_max)
				{
					pdop = 0;
					// printf("%s 迭代次数 %d 溢出, delta = %f !\n", obsEpoch.t.toString().c_str(), k_GaussNewton, delta);
					return false;
				}
				if(delta <= threshold) 
				{
					// 空间三维位置精度因子 pdop = sqrt(q11 + q22 + q33), 2007/07/10
					pdop = sqrt(matHH_inv.GetElement(0,0) + matHH_inv.GetElement(1,1) + matHH_inv.GetElement(2,2));
					if(pdop > m_PreprocessorDefine.max_pdop)
					{// 2008/07/01添加
						pdop = 0;
						return false;
					}
					break;
				}
				// 控制迭代次数, 控制改进增量
				if(k_GaussNewton >= max_GaussNewtonCount || fabs(delta) >= delta_max)
				{
					pdop = 0;
					printf("%s 迭代次数 %d 溢出, delta = %f !\n", obsEpoch.t.toString().c_str(), k_GaussNewton, delta);
					return false;
				}
				// 更新概略点
				matAppPosClkBias = matAppPosClkBias + matDx;
				posclk.x    = matAppPosClkBias.GetElement(0,0);
				posclk.y    = matAppPosClkBias.GetElement(1,0);
				posclk.z    = matAppPosClkBias.GetElement(2,0);
				posclk.clk  = matAppPosClkBias.GetElement(3,0);
				channelBias = matAppPosClkBias.GetElement(4,0);
				// 更新信号接收参考时间
				GPST t_Receive;
				if(m_PreprocessorDefine.bOn_ClockEliminate)
					t_Receive = obsEpoch.t; // 认为sy1软校正过的采样时刻是较为准确的, 2010/06/12
				else
					t_Receive = obsEpoch.t - posclk.clk / SPEED_LIGHT;
				j = 0;
				for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
				{
					if(delta > 100.0) // 如果改进量小于100m, 为减少计算量, 可不进行视线矢量 matH 的更新
					{
						double y = matObs.GetElement(j, 0);
						// 迭代计算卫星信号传播时间
						double delay = 0;
						int nPRN = it->first; // 第 j 颗可见GPS卫星的卫星号
						char szSatName[4];
						sprintf(szSatName, "%1c%02d", cSatSystem, nPRN);
						szSatName[3] = '\0';
						SP3Datum sp3Datum;
						m_sp3File.getEphemeris_PathDelay(obsEpoch.t, posclk, szSatName, delay, sp3Datum);
						// 对 GPS 卫星星历进行地球自转改正
						GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
						// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
						GPST t_Transmit = t_Receive - delay;
						// 对观测值 y 进行误差修正,包括 GPS 卫星钟差改正, GPS卫星相对论修正等, 相位中心修正在数据预处理中暂时不考虑
						// 1. GPS卫星钟差改正
						CLKDatum ASDatum;
						m_clkFile.getSatClock(t_Transmit, nPRN, ASDatum, 3, cSatSystem); // 获得 GPS 信号发射时间的卫星钟差改正
						double correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT;  
						y = y + correct_gpsclk;
						// 2. GPS卫星相对论改正
						double correct_relativity = (sp3Datum.pos.x * sp3Datum.vel.x 
												   + sp3Datum.pos.y * sp3Datum.vel.y
												   + sp3Datum.pos.z * sp3Datum.vel.z) * (-2.0) / SPEED_LIGHT;
						y = y + correct_relativity;
						matGPSSp3Clk.SetElement(j, 0, sp3Datum.pos.x);
						matGPSSp3Clk.SetElement(j, 1, sp3Datum.pos.y);
						matGPSSp3Clk.SetElement(j, 2, sp3Datum.pos.z);
						matGPSSp3Clk.SetElement(j, 3, ASDatum.clkBias);
						// 根据 sp3Datum 和接收机概略位置, 计算概略距离
						double distance;
						distance = pow(posclk.x - sp3Datum.pos.x, 2)
								 + pow(posclk.y - sp3Datum.pos.y, 2)
								 + pow(posclk.z - sp3Datum.pos.z, 2);
						distance = sqrt(distance);
						matY.SetElement(j, 0, y); 
						matDy.SetElement(j, 0, y - (distance + posclk.clk)); // 计算观测值与概略距离之差, 包含钟差
						if(j == npos_PRN) 
							matDy.SetElement(j, 0, matY.GetElement(j, 0) - (distance + posclk.clk + channelBias)); 
						else           
							matDy.SetElement(j, 0, matY.GetElement(j, 0) - (distance + posclk.clk)); 
						// 利用概略点计算视线方向
						matH.SetElement(j, 0, (posclk.x - sp3Datum.pos.x) / distance);
						matH.SetElement(j, 1, (posclk.y - sp3Datum.pos.y) / distance);
						matH.SetElement(j, 2, (posclk.z - sp3Datum.pos.z) / distance);
						matH.SetElement(j, 3, 1.0);
					}
					else
					{
						double distance = pow(posclk.x - matGPSSp3Clk.GetElement(j, 0), 2)
							            + pow(posclk.y - matGPSSp3Clk.GetElement(j, 1), 2)
							            + pow(posclk.z - matGPSSp3Clk.GetElement(j, 2), 2);
						distance = sqrt(distance);
						if(j == npos_PRN) 
							matDy.SetElement(j, 0, matY.GetElement(j, 0) - (distance + posclk.clk + channelBias)); 
						else           
							matDy.SetElement(j, 0, matY.GetElement(j, 0) - (distance + posclk.clk)); 
					}
					j++;
				}
			}
			matAppPosClkBias = matAppPosClkBias + matDx;
			posclk.x    = matAppPosClkBias.GetElement(0,0);
			posclk.y    = matAppPosClkBias.GetElement(1,0);
			posclk.z    = matAppPosClkBias.GetElement(2,0);
			posclk.clk  = matAppPosClkBias.GetElement(3,0);
			channelBias = matAppPosClkBias.GetElement(4,0);
			return true;
		}

		// 子程序名称： RaimSPP_PIF   
		// 功能： RAIM 检验算法, 
		//        返回值: 0 表示超差, 无法输出有效结果
		//                1 表示成功但不可靠
		//                2 表示可靠
		// 变量类型：index_P1             : 观测类型 P1 索引
		//           index_P2             : 观测类型 P2 索引
		//           frequence_L1         : 观测类型 P1 的频率
		//           frequence_L2         : 观测类型 P2 的频率
		//           obsEpoch             : 某时刻的预处理后观测数据
		//           posclk               : 概略轨道位置、钟差, 返回值存储
		//           pdop                 : 定位的几何精度因子
		//           rms_res              : 定位残差的 Q 统计量, 主要用于后续 RAIM 检验
		// 输入：index_P1, index_P2, obsEpoch, posclk
		// 输出：posclk, pdop, rms_res
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/5/10
		// 版本时间：
		// 修改记录：
		// 备注： 调用SinglePointPositioning_PIF, RaimEstChannelBias_PIF
		int LeoGPSObsPreproc::RaimSPP_PIF(int index_P1, int index_P2,double frequence_L1,double frequence_L2, Rinex2_1_LeoEditedObsEpoch& obsEpoch, POSCLK& posclk, double& pdop, double& rms_res)
		{
			int eyeableGPSCount = 0;
			POSCLK raimPosClk = posclk;
			// 2008/03/03, 不直接返回, 因为首次定位可能包含坏的通道, 导致迭代不收敛
			bool bResult = SinglePointPositioning_PIF(index_P1, index_P2, frequence_L1, frequence_L2, obsEpoch, posclk, eyeableGPSCount, pdop, rms_res);
			// 卫星数大于5, 才能计算 RAIM 检验统计量
			// 卫星数大于6, 才能诊断是哪个通道发生故障
			// 卫星数大于5, 才能估算故障通道的偏差
			if((rms_res >= m_PreprocessorDefine.threshold_res_raim || bResult == false) // 迭代发散标记
			 && eyeableGPSCount >= 6)
			{
				bool bFind = false;
				Rinex2_1_LeoEditedObsEpoch obsEpoch_i;
				POSCLK  posclk_i;
				double  pdop_i;
				double  rms_res_i;
				int eyeableGPSCount_i;
				Rinex2_1_EditedObsSatMap::iterator it_FailureSat; 
				int i = 0;
				for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
				{
					obsEpoch_i = obsEpoch;
					Rinex2_1_EditedObsSatMap::iterator jt = obsEpoch_i.editedObs.find(it->first);
					obsEpoch_i.editedObs.erase(jt);
					posclk_i = posclk;
					if(SinglePointPositioning_PIF(index_P1, index_P2, frequence_L1, frequence_L2, obsEpoch_i, posclk_i, eyeableGPSCount_i, pdop_i, rms_res_i))
					{
						if(rms_res_i <= rms_res)
						{// 遍历整个通道, 寻找最优的故障通道识别结果
							it_FailureSat = it;
							rms_res = rms_res_i;
							raimPosClk = posclk_i; // 更新概略位置
							pdop = pdop_i; // 更新几何精度因子
						}
					}
					i++;
				}
				posclk = raimPosClk;
				if(rms_res < m_PreprocessorDefine.threshold_res_raim)
				{
					int nPRN = it_FailureSat->first;
					double channelBias = 0;
					// 可以进一步识别故障偏差大小, eyeableGPSCount >= 6
					posclk_i = raimPosClk;
					if(RaimEstChannelBias_PIF(index_P1, index_P2, frequence_L1, frequence_L2, obsEpoch, nPRN, posclk_i, channelBias))
					{
						// 打上 TYPE_EDITEDMARK_RAIM 的标签, 配合进一步多时刻检验
						//if(m_PreprocessorDefine.bOn_RaimSPP)
						{
							it_FailureSat->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it_FailureSat->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it_FailureSat->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it_FailureSat->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);
						}
						return 2; // 识别故障通道, 结果可靠
					}
					else
					{
						if(m_PreprocessorDefine.bOn_RaimSPP)
						{
							for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
							{
								it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
								it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
								it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);
								it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);	
							}
						}
						pdop = 0;
						return 0;
					}
					
				}
				else
				{// 未找到, 说明可能有两个以上通道都有故障,则将所有通道均标记野值, 宁肯错判, 也不遗漏
					if(m_PreprocessorDefine.bOn_RaimSPP)
					{
						for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
						{
							it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);	
							
						}
					}
					pdop = 0;
					return 0;
				}
			}
			else 
			{// 2008/08/08 防止 4 颗星时的返回结果被漏掉
			 // 对于未能满足参与 RAIM 检验前提条件的情形, 不强行返回 false, 如果返回 false, 对应的观测量就将被打上 TYPE_EDITEDMARK_RAIM 的标签
				if(!bResult)
				{
					if(m_PreprocessorDefine.bOn_RaimSPP)
					{
						for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
						{
							it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);	
						}
					}
					pdop = 0;
					return 0;
				}
				else
				{
					if(rms_res < m_PreprocessorDefine.threshold_res_raim)
					{
						if(eyeableGPSCount >= 5)
							return 2; // bResult = true, 且可见卫星大于 5, 结果可靠
						else
							return 1;
					}
					else
					{
						pdop = 0;
						return 0;
					}
				}
			}
		}

		// 子程序名称： detectRaimArcChannelBias_PIF   
		// 功能： 利用 RAIM 检验的结果, 在一次连续跟踪弧段中统计超差的个数, 检查该弧段是否存在系统误差, 决定是否删除本段连续跟踪观测数据
		// 变量类型：index_P1             : 观测类型 P1 索引
		//           index_P2             : 观测类型 P2 索引
		//           obsEpochList         : 经过 RAIM 检验后的数据
		//           threshold            : RAIM 超差点的比率阈值
		// 输入：index_P1, index_P2, obsEpochList, threshold
		// 输出：obsEpochList
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2008/04/04
		// 版本时间：
		// 修改记录：
		// 备注： 调用 RaimSPP_PIF
		bool LeoGPSObsPreproc::detectRaimArcChannelBias_PIF(int index_P1,int index_P2, vector<Rinex2_1_LeoEditedObsEpoch> &obsEpochList, double threshold)
		{
			vector<Rinex2_1_EditedObsSat> obsSatList;
			datalist_epoch2sat(obsEpochList, obsSatList);
			for(size_t s_i = 0; s_i < obsSatList.size(); s_i++)
			{
				Rinex2_1_EditedObsEpochMap::iterator it_0 = obsSatList[s_i].editedObs.begin();
				GPST t0 = it_0->first; 
				double *pObsTime   = new double[obsSatList[s_i].editedObs.size()];
				int    *pRaimFlag  = new int   [obsSatList[s_i].editedObs.size()];
				int    *pTrackFlag = new int   [obsSatList[s_i].editedObs.size()];
				vector<GPST> obsTimeList;
				obsTimeList.resize(obsSatList[s_i].editedObs.size());
				int j = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = obsSatList[s_i].editedObs.begin(); it != obsSatList[s_i].editedObs.end(); ++it)
				{
					pObsTime[j] = it->first - t0;
					obsTimeList[j] = it->first;
					pTrackFlag[j] = 0;
					if((it->second.obsTypeList[index_P1].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER && it->second.obsTypeList[index_P1].byEditedMark2 == obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM))
					 ||(it->second.obsTypeList[index_P2].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER && it->second.obsTypeList[index_P2].byEditedMark2 == obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM)))
						pRaimFlag[j] = LEOGPSOBSPREPROC_OUTLIER_RAIM;
					else
						pRaimFlag[j] = LEOGPSOBSPREPROC_NORMAL;

					j++;
				}
				size_t k   = 0;
				size_t k_i = k;
				// 获得每个连续跟踪弧段的数据
				while(1)
				{
					if(k_i + 1 >= obsSatList[s_i].editedObs.size())
						goto newArc;
					else
					{// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段
						if(pObsTime[k_i + 1] - pObsTime[k_i] <= m_PreprocessorDefine.max_arclengh)
						{
							k_i++;
							continue;
						}
						else // k_i+1为新弧段的起点
							goto newArc;
					}
					newArc:  // 本弧段[k, k_i]数据处理 
					{// 统计本弧段内的 raim 检验的异常点个数
						int count_raimpoints = 0;
						int count_arcpoints = int(k_i - k + 1);
						for(size_t s_k = k; s_k <= k_i; s_k++)
						{
							if(pRaimFlag[s_k] == LEOGPSOBSPREPROC_OUTLIER_RAIM)
								count_raimpoints++;
						}
						if(double(count_raimpoints) / double(count_arcpoints) >= threshold)
						{
							for(size_t s_k = k; s_k <= k_i; s_k++)
								pTrackFlag[s_k] = 1;
							printf("%s -- %02d:%02d:%02d PRN%02d 弧段 RAIM 比率 = %6.2f 超差\n",obsTimeList[k].toString().c_str(),
								                                                  obsTimeList[k_i].hour,
																				  obsTimeList[k_i].minute,
																				  int(obsTimeList[k_i].second),
								                                                  obsSatList[s_i].Id, 
														                          double(count_raimpoints) / double(count_arcpoints));
						}
						if(k_i + 1 >= obsSatList[s_i].editedObs.size())
							break;
						else  
						{// 新弧段的起点设置
							k   = k_i+1;
							k_i = k;
							continue;
						}
					}
				}
				j = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = obsSatList[s_i].editedObs.begin(); it != obsSatList[s_i].editedObs.end(); ++it)
				{
					if(pTrackFlag[j] == 1)
					{
						// 保留原有野值标记, 不更改
						if(it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);
						}
						if(it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_RAIM);
							it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_RAIM);
						}
					}
					j++;
				}
				delete pObsTime;
				delete pTrackFlag;
				delete pRaimFlag;
			}
			datalist_sat2epoch(obsSatList, obsEpochList);
			return true;
		}

		// 子程序名称： detectL2SNRLost   
		// 功能：诊断 L2 失锁现象, L2 失锁现象在数据格式转换的时候, 已经利用阈值条件进行相应的处理, 但仍存在一些边界的残余
		// 变量类型：   index_P1       : 观测类型P1索引
		//              index_P2       : 观测类型P2索引
		//              index_L1       : 观测类型L1索引
		//              index_L2       : 观测类型L2索引
		//              obsSat         : 输出的弧段结构数据
		// 输入：index_P1, index_P2, index_L1, index_L2
		// 输出：obsSat
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2009/11/30
		// 版本时间：2009/11/30
		// 修改记录：
		// 其它： 为了保证能够被 mainFuncDFreqGPSObsPreproc 和 mainFuncDFreqGPSObsEdit 同时调用
		//        要求 LEOGPSOBSPREPROC_OUTLIER_COUNT = LEOGPSOBSEDIT_OUTLIER_COUNT
		//             LEOGPSOBSPREPROC_OUTLIER_SNR   = LEOGPSOBSEDIT_OUTLIER_SNRELEVATION
		bool LeoGPSObsPreproc::detectL2SNRLost(int index_S2, int index_P1, int index_P2, int index_L1, int index_L2, Rinex2_1_EditedObsSat& obsSat)
		{
			// 观测个数太少, 直接丢弃
			if(obsSat.editedObs.size() <= m_PreprocessorDefine.min_arcpointcount)  
			{
				for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
				{// 防止重复标记
					if(it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_COUNT);
					}
				}
				return true;
			}
			double *pS2 = new double [obsSat.editedObs.size()];
			int    *pOutlier = new int [obsSat.editedObs.size()];
			double *pEpochTime = new double [obsSat.editedObs.size()];
			Rinex2_1_EditedObsEpochMap::iterator it0 = obsSat.editedObs.begin();
			GPST t0 = it0->first;  
			int i = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
			{
				pEpochTime[i] = it->first - t0;
				Rinex2_1_EditedObsDatum S2 = it->second.obsTypeList[index_S2];
				pS2[i] = 20.0 * log10( S2.obs.data / sqrt(2.0));
				pOutlier[i] = TYPE_EDITEDMARK_UNKNOWN;
				i++;
			}
			size_t k   = 0;
			size_t k_i = k;
			// 获得每个连续跟踪弧段的数据
			while(1)
			{
				if(k_i + 1 >= obsSat.editedObs.size())
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
					int count_arcpoints = int(k_i - k + 1);
					if(count_arcpoints <= int(m_PreprocessorDefine.min_arcpointcount))
					{
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{
							pOutlier[s_i] = LEOGPSOBSPREPROC_OUTLIER_COUNT;
						}
					}
					else
					{   
						// 首先根据信噪比进行数据的双频野值剔除, 2008/01/07
						double *pX_T = new double [count_arcpoints];
						double *pX_S = new double [count_arcpoints];
						double *pX_F = new double [count_arcpoints];
						double *pW_S = new double [count_arcpoints];
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{
							pX_T[s_i - k] = pEpochTime[s_i];
							pX_S[s_i - k] = pS2[s_i];
							pW_S[s_i - k] = 1;
						}
					    KinematicRobustVandrakFilter(pX_T, 
						                             pX_S, 
												     pW_S, 
												     count_arcpoints,
												     5.0E-14, 
												     pX_F, 
												     1.5,
												     0,
												     count_arcpoints);
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{
							if(pW_S[s_i - k] == 0)
							{
								pOutlier[s_i] = LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION;
							}
						}
						delete pX_T;
						delete pX_S;
						delete pW_S;
						delete pX_F;
					}
					if(k_i + 1 >= size_t(count_arcpoints))
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
			for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
			{	
				// 恢复 TYPE_EDITEDMARK_UNKNOWN 为 TYPE_EDITEDMARK_NORMAL
				if(pOutlier[i] != TYPE_EDITEDMARK_UNKNOWN)
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
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(pOutlier[i]);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(pOutlier[i]);
					}
					if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(pOutlier[i]);	
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(pOutlier[i]);
					}
				}
				i++;
			}
			delete pS2;
			delete pOutlier;
			delete pEpochTime;
			return true;
		}

		// 子程序名称： detectCodeOutlier_ionosphere   
		// 功能：电离层组合探测伪码野值
		// 变量类型：    index_P1, index_P2      　　              : 第一个和第二个数据的位置
		//               editedObsSat                              : 输出的弧段结构数据
		//               frequence_L1,frequence_L2                 : 第一个频点和第二个频点的频率
		// 输入：index_P1, index_P2, frequence_L1, frequence_L2
		// 输出：editedObsSat
		// 语言：C++
		// 创建者：谷德峰, 刘俊宏
		// 创建时间：2007/05/10
		// 版本时间：2012/10/17
		// 修改记录：
		// 其它： 
		bool LeoGPSObsPreproc::detectCodeOutlier_ionosphere(int index_P1, int index_P2, Rinex2_1_EditedObsSat& obsSat,double frequence_L1,double frequence_L2)
		{
			size_t nCount = obsSat.editedObs.size();
			if(nCount <= m_PreprocessorDefine.min_arcpointcount)  // 观测个数太少, 直接丢弃
			{				
				for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
				{
					//防止重复标记
					if(it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_COUNT);
					}
				}
				return true;
			}
			double *pIonosphere = new double[nCount];
            double *pIonosphere_fit = new double[nCount];			
			int *pOutlier = new int    [nCount];		
			double *pEpochTime = new double [nCount];
			Rinex2_1_EditedObsEpochMap::iterator it0 = obsSat.editedObs.begin();
			DayTime t0 = it0->first;  
			// 构造伪码电离层组合序列 ionosphere = coefficient_ionosphere * (P1 - P2)
			double coefficient_ionosphere = 1 / (1 - pow( frequence_L1 / frequence_L2, 2 ));
			int i = 0;			
			for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
			{				
				pEpochTime[i] = it->first - t0;
				Rinex2_1_EditedObsDatum  P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum  P2 = it->second.obsTypeList[index_P2];
				pIonosphere[i] = coefficient_ionosphere * (P1.obs.data - P2.obs.data);				
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
					pOutlier[i] = TYPE_EDITEDMARK_OUTLIER; // 接受先前野值判断结果, 以和其他野值判断方法相配合
				else
					pOutlier[i] = LEOGPSOBSPREPROC_NORMAL;
				i++;
			}
			FILE *pFile; 
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				char szCodePreprocFileName[300];
				sprintf(szCodePreprocFileName,"%s\\preproc_P1P2.dat",m_strPreprocPath.c_str());
				pFile = fopen(szCodePreprocFileName,"a+");
			}
			size_t k   = 0;
			size_t k_i = k;
			static int nArcCount = 0;
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
							pOutlier[s_i] = LEOGPSOBSPREPROC_OUTLIER_COUNT; // 弧段内数据个数太少标记为野值
						}
					}
					else
					{   
						nArcCount++;
						double *w = new double [nArcPointsCount];
						// 首先根据电离层残差的阈值大小，直接进行野值判断，剔除一些大的野值
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{							
							if(pIonosphere[s_i] <= m_PreprocessorDefine.min_ionosphere 
							|| pIonosphere[s_i] >= m_PreprocessorDefine.max_ionosphere) 
							{
								w[s_i - k] = 0;								
								pOutlier[s_i] = LEOGPSOBSPREPROC_OUTLIER_IONOMAXMIN; //电离层超差，直接标记为野值
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
												     m_PreprocessorDefine.vondrak_PIF_width,
													 4);
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{							
							if(w[s_i - k] == 0 && pOutlier[s_i] == LEOGPSOBSPREPROC_NORMAL)
							{
								pOutlier[s_i] = LEOGPSOBSPREPROC_OUTLIER_VONDRAK;
							}
						}
						delete w;
					}
					if(!m_strPreprocPath.empty())
					{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
						// 写文件
						//fprintf(pFile, "PRN %2d -> Arc: %2d\n", obsSat.Id, nArcCount);
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{
							if(pOutlier[s_i] != TYPE_EDITEDMARK_OUTLIER)
							{// 只输出改判的点, 包含正常点和部分异常点
								fprintf(pFile,"%-30s %8.2f %8d %8d %18.4f %18.4f %8d\n",
									(t0 + pEpochTime[s_i]).toString().c_str(),
									pEpochTime[s_i],
									obsSat.Id,
									nArcCount,
									pIonosphere[s_i],
									pIonosphere_fit[s_i],
									pOutlier[s_i]);
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
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				fclose(pFile);
			}
			i = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
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
				i++;
			}
			delete pIonosphere;
			delete pIonosphere_fit;
			delete pOutlier;
			delete pEpochTime;			
			return true;
		}

		// 子程序名称： detectPhaseSlip   
		// 功能：完成周跳探测   
		// 变量类型： index_P1       : 观测类型P1索引
		//            index_P2       : 观测类型P2索引
		//            index_L1       : 观测类型L1索引
		//            index_L2       : 观测类型L2索引
		//           frequence_L1         : 观测类型 P1 的频率
		//           frequence_L2         : 观测类型 P2 的频率
		//            obsSat         : 某个BD测站的观测数据时间序列
		// 输入：index_P1, index_P2, index_L1, index_L2, obsSat
		// 输出：obsSat
		// 语言：C++
		// 创建者：谷德峰, 刘俊宏
		// 创建时间：2007/05/10
		// 版本时间：2012/10/17
		// 修改记录：
		// 其它： 
		bool LeoGPSObsPreproc::detectPhaseSlip(int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1,double frequence_L2, Rinex2_1_EditedObsSat& obsSat)
		{
			double  FREQUENCE_L1  = frequence_L1;
			double  FREQUENCE_L2  = frequence_L2;
			double  WAVELENGTH_L1 = SPEED_LIGHT / FREQUENCE_L1;
            double  WAVELENGTH_L2 = SPEED_LIGHT / FREQUENCE_L2;
			//FILE* pFileTest = fopen("c:\\wm.txt", "a+");
			size_t nCount = obsSat.editedObs.size();
			if(nCount <= m_PreprocessorDefine.min_arcpointcount)  // 观测个数太少, 直接丢弃
			{
				for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
				{	
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_COUNT);
					}
					if(it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_COUNT);	
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_COUNT);
					}					
				}
				return true;
			}
			// 宽巷载波相位 - 窄巷伪距
			double  *pWL_NP = new double[nCount];
			double  *pL1_L2 = new double[nCount]; // 相位电离层组合
			double  *pEpochTime = new double[nCount];			
			int *pSlip = new int [nCount];
			double *pIonosphere_phase_code = new double[nCount];
			Rinex2_1_EditedObsEpochMap::iterator it0 = obsSat.editedObs.begin();
			BDT t0 = it0->first;  
			double coefficient_ionosphere = 1 / (1 - pow( FREQUENCE_L1 / FREQUENCE_L2, 2 ));
			int i = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
			{
				pEpochTime[i] = it->first - t0;				
				Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[index_P2];
				Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[index_L1];
				Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[index_L2];
				double dP1 = P1.obs.data;
				double dP2 = P2.obs.data;
				double dL1 = L1.obs.data * SPEED_LIGHT/FREQUENCE_L1;
				double dL2 = L2.obs.data * SPEED_LIGHT/FREQUENCE_L2;
				// 构造无电离层组合
				double code_ionofree  = dP1  -(dP1 - dP2) * coefficient_ionosphere;
				double phase_ionofree = dL1 - (dL1 - dL2) * coefficient_ionosphere;
				// 构造宽巷载波相位 widelane_L 和窄巷伪距 narrowlane_P
				double widelane_L   = (FREQUENCE_L1 * dL1 - FREQUENCE_L2 * dL2) / (FREQUENCE_L1 - FREQUENCE_L2);
				double narrowlane_P = (FREQUENCE_L1 * dP1 + FREQUENCE_L2 * dP2) / (FREQUENCE_L1 + FREQUENCE_L2);
				double WAVELENGTH_W = SPEED_LIGHT/(FREQUENCE_L1 - FREQUENCE_L2);			  				
				pWL_NP[i] = (widelane_L - narrowlane_P) / WAVELENGTH_W; // melbourne-wuebbena 组合量
				pIonosphere_phase_code[i] = phase_ionofree - code_ionofree;
				pL1_L2[i] = dL1 - dL2; // 构造相位电离层组合(L1-L2)
				// 如果伪码已经标记为野值, 相位为正常点, 则将相应的相位也标记为野值
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)					
				{
					if(L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_MW);						
					}
					if(L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
					{						
						it->second.obsTypeList[index_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_MW);
					}
				}					
				// 接受先前伪码观测数据的野值判断结果,  补充相位野值
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER
				|| L1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || L2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
					pSlip[i] = TYPE_EDITEDMARK_OUTLIER; 
				else
					pSlip[i] = LEOGPSOBSPREPROC_NORMAL;					

				i++;
			}
			FILE *pFile;
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				char szPhasePreprocFileName[300];
				sprintf(szPhasePreprocFileName,"%s\\preproc_L1L2.dat",m_strPreprocPath.c_str());
				pFile = fopen(szPhasePreprocFileName,"a+");
			}
			size_t k   = 0;
			size_t k_i = k;
			int arc_k  = 0;
			static int ArcCounts = 0;
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
				newArc:  // 本弧段[k, k_i]数据处理 
				{
					vector<size_t>   unknownPointlist;
					unknownPointlist.clear();
					for(size_t s_i = k; s_i <= k_i; s_i++)
					{
						// 未知数据标记
						if(pSlip[s_i] == LEOGPSOBSPREPROC_NORMAL)
							unknownPointlist.push_back(s_i); 
					}
					size_t nCount_points = unknownPointlist.size(); 
					// 弧段内数据个数太少					
					if(nCount_points <= int(m_PreprocessorDefine.min_arcpointcount))
					{
						for(size_t s_i = 0; s_i < nCount_points; s_i++)
						{
							if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
								pSlip[unknownPointlist[s_i]] = LEOGPSOBSPREPROC_OUTLIER_COUNT; // 弧段内数据个数太少标记为野值							
						}
					}
					else
					{   						
						// 第一步: 计算MW组合量历元差数据的方差
						// 构造MW组合量历元差数据
						ArcCounts++;
						double *pDWL_NP = new double[nCount_points - 1];
						for(size_t s_i = 1; s_i < nCount_points; s_i++)
							pDWL_NP[s_i - 1] = pWL_NP[unknownPointlist[s_i]] - pWL_NP[unknownPointlist[s_i - 1]] ;
						double var = RobustStatRms(pDWL_NP, int(nCount_points - 1));
						delete pDWL_NP;
						// 第二步: 进行相位野值剔除					
						// 20071012 添加, 利用 threshold_slipsize_wm 对 threshold_outlier 的上界进行控制
						// 因为当周跳发生在序列中间附近时, var 可能会超界, 影响野值探测
						double threshold_outlier = min(5 * var, m_PreprocessorDefine.threshold_slipsize_mw);
						// [1, nCount_points - 2]
						for(size_t s_i = 1; s_i < nCount_points - 1; s_i++)
						{
							if(fabs(pWL_NP[unknownPointlist[s_i]]     - pWL_NP[unknownPointlist[s_i-1]]) > threshold_outlier
							&& fabs(pWL_NP[unknownPointlist[s_i + 1]] - pWL_NP[unknownPointlist[s_i] ])  > threshold_outlier)
							{								
								if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
									pSlip[unknownPointlist[s_i]] = LEOGPSOBSPREPROC_OUTLIER_MW;								
							}							
						}
						// 首尾两点 0 和 nCount_points - 1
						if(pSlip[unknownPointlist[1]] != LEOGPSOBSPREPROC_NORMAL && pSlip[unknownPointlist[0]] != TYPE_EDITEDMARK_OUTLIER)
							pSlip[unknownPointlist[0]] = LEOGPSOBSPREPROC_OUTLIER_MW;
						if(pSlip[unknownPointlist[nCount_points - 2]] != LEOGPSOBSPREPROC_NORMAL)
						{
							if(pSlip[unknownPointlist[nCount_points - 1]] != TYPE_EDITEDMARK_OUTLIER)
								pSlip[unknownPointlist[nCount_points - 1]] = LEOGPSOBSPREPROC_OUTLIER_MW;
						}
						else
						{
							if((fabs(pWL_NP[unknownPointlist[nCount_points - 1]] - pWL_NP[unknownPointlist[nCount_points - 2] ])  > threshold_outlier)
								&& (pSlip[unknownPointlist[nCount_points - 1]] != TYPE_EDITEDMARK_OUTLIER))
								pSlip[unknownPointlist[nCount_points - 1]] = LEOGPSOBSPREPROC_OUTLIER_MW;
						}
						size_t s_i = 0;
						while(s_i < unknownPointlist.size())
						{
							if(pSlip[unknownPointlist[s_i]] == LEOGPSOBSPREPROC_NORMAL)
								s_i++;
							else
							{
								// 在进行周跳探测时, 先将野值 erase
								unknownPointlist.erase(unknownPointlist.begin() + s_i);
							}
						}
						nCount_points = unknownPointlist.size();
						// 第三步: 进行大周跳探测
						if(nCount_points <= 3)
						{
							// 个数太少则直接丢弃
							for(size_t s_i = 0; s_i < nCount_points; s_i++)
							{								
								if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
									pSlip[unknownPointlist[s_i]] = LEOGPSOBSPREPROC_OUTLIER_COUNT;								
							}
						}
						else
						{
							vector<size_t> slipindexlist;
							slipindexlist.clear();
							// [1, nCount_points - 2]
							double threshold_largeslip = m_PreprocessorDefine.threshold_slipsize_mw;
							int countSlip_k = 0;
							for(size_t s_i = 1; s_i < nCount_points - 1; s_i++)
							{
								// 大周跳发生, 每个大周跳在探测后, 其信息都被保存下来了
								if(fabs(pWL_NP[unknownPointlist[s_i]]     - pWL_NP[unknownPointlist[s_i - 1]]) >  threshold_largeslip
								&& fabs(pWL_NP[unknownPointlist[s_i + 1]] - pWL_NP[unknownPointlist[s_i] ])    <= threshold_largeslip) 
								{									
									size_t index = unknownPointlist[s_i];
									pSlip[index] = LEOGPSOBSPREPROC_SLIP_MW;

									//if(fabs(pWL_NP[unknownPointlist[s_i]] - pWL_NP[unknownPointlist[s_i - 1]]) > 10)
									countSlip_k++; // 20170617, 谷德峰添加, 统计周跳信息, 只统计MW大周跳

									//char info[200];
									//sprintf(info, "MW发现大周跳发生 %10.2f", fabs(pWL_NP[unknownPointlist[s_i]]     - pWL_NP[unknownPointlist[s_i - 1]]));
									//RuningInfoFile::Add(info);
									//printf("MW发现大周跳发生 %10.2f\n", fabs(pWL_NP[unknownPointlist[s_i]]     - pWL_NP[unknownPointlist[s_i - 1]]));
								}
								else
								{
									/* 
									    消电离层组合检验, 2008/07/11,
										M-W组合量只能识别 L1 - L2 的周跳, 因此无法识别两个频率发生的等大小的周跳,
										而等大小的周跳同样会对相位无电离层组合带来影响, 因此在这里要补充关于无电 
										离层组合的大周跳探测, 以确保在精密定轨中迭代收殓      
									*/
									// 消电离层组合要放大观测噪声 3 倍左右, 大约是 mw 组合的 4 倍
									if(m_PreprocessorDefine.bOn_IonosphereFree)
									{
										if(fabs(pIonosphere_phase_code[unknownPointlist[s_i]]     - pIonosphere_phase_code[unknownPointlist[s_i - 1]]) > threshold_largeslip * 4
										&& fabs(pIonosphere_phase_code[unknownPointlist[s_i + 1]] - pIonosphere_phase_code[unknownPointlist[s_i] ])   <= threshold_outlier * 4)
										{											
											size_t index = unknownPointlist[s_i];
											pSlip[index] = LEOGPSOBSPREPROC_SLIP_IFAMB;
										}
									}
									// sy1卫星的轨道高度比较较高, 可以考虑增加 L1-L2 判断小周跳
									// 计算历元间隔, 因为L1-L2探测野值也周跳受电离层影响较大, 所以需要考虑历元间隔的影响, 2012/10/24
									if(m_PreprocessorDefine.bOn_Slip_L1_L2)
									{
										double threshold_iono_diff = 0.10;  // 采样间隔的电离层历元差阈值
										// L1 - L2探测野值
										if(fabs(pL1_L2[unknownPointlist[s_i]]     - pL1_L2[unknownPointlist[s_i - 1]]) >  threshold_iono_diff
										&& fabs(pL1_L2[unknownPointlist[s_i + 1]] - pL1_L2[unknownPointlist[s_i]])     >  threshold_iono_diff
										&& pEpochTime[unknownPointlist[s_i]]      - pEpochTime[unknownPointlist[s_i - 1]] <= 30.0
										&& pEpochTime[unknownPointlist[s_i + 1]]  - pEpochTime[unknownPointlist[s_i]]     <= 30.0)
										{										
											size_t index = unknownPointlist[s_i];
											pSlip[index] = LEOGPSOBSPREPROC_OUTLIER_L1_L2;
										}
										// L1 - L2探测周跳
										else if(fabs(pL1_L2[unknownPointlist[s_i]]     - pL1_L2[unknownPointlist[s_i - 1]]) > threshold_iono_diff
										&& fabs(pL1_L2[unknownPointlist[s_i + 1]] - pL1_L2[unknownPointlist[s_i]])   <= threshold_iono_diff
										&& pEpochTime[unknownPointlist[s_i]]      - pEpochTime[unknownPointlist[s_i - 1]] <= 30.0
										&& pEpochTime[unknownPointlist[s_i + 1]]  - pEpochTime[unknownPointlist[s_i]]     <= 30.0)
										{										
											size_t index = unknownPointlist[s_i];
											pSlip[index] = LEOGPSOBSPREPROC_SLIP_L1_L2;
											//printf("发现小周跳发生 L1-L2历元差 = %10.2f\n",fabs(pL1_L2[unknownPointlist[s_i]] - pL1_L2[unknownPointlist[s_i - 1]]));
										}
									}
								}
							}
						    // 无周跳弧段的内符合诊断, 主要针对 CHAMP 卫星, 其它卫星这种现象很少, 2008/11/11
							slipindexlist.clear();
							for(size_t s_i = 1; s_i < nCount_points; s_i++)
							{
								size_t index = unknownPointlist[s_i];
								if(pSlip[index] == LEOGPSOBSPREPROC_SLIP_MW
								|| pSlip[index] == LEOGPSOBSPREPROC_SLIP_IFAMB
								|| pSlip[index] == LEOGPSOBSPREPROC_SLIP_L1_L2)
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
									pSubsection_right[s_i]    = slipindexlist[s_i] -  1 ;
									pSubsection_left[s_i + 1] = slipindexlist[s_i] ;
								}
								pSubsection_right[count_slips] = unknownPointlist[nCount_points - 1]; 
							}
							else
							{
								pSubsection_left[0]  = unknownPointlist[0];
								pSubsection_right[0] = unknownPointlist[nCount_points - 1];
							} 
							int count_restslip = 0;
							for(size_t s_i = 0; s_i < count_slips + 1; s_i++)
							{
								// 整理 [pSubsection_left[s_i], pSubsection_right[s_i]]
								vector<size_t> subsectionNormalPointlist;
								subsectionNormalPointlist.clear();
								for(size_t s_j = pSubsection_left[s_i]; s_j <= pSubsection_right[s_i]; s_j++)
								{
									if(pSlip[s_j] != TYPE_EDITEDMARK_OUTLIER
									&& pSlip[s_j] != LEOGPSOBSPREPROC_OUTLIER_MW
									&& pSlip[s_j] != LEOGPSOBSPREPROC_OUTLIER_L1_L2)
										subsectionNormalPointlist.push_back(s_j); 
								}
								size_t count_subsection = subsectionNormalPointlist.size(); 
								if(count_subsection > m_PreprocessorDefine.min_arcpointcount)
								{   
									count_restslip++;
									double *pX = new double [count_subsection];
									double *pW = new double [count_subsection];
									double mean = 0;
									double var  = 0;
									for(size_t s_j = 0; s_j < count_subsection; s_j++)
										pX[s_j] = pWL_NP[subsectionNormalPointlist[s_j]];  
									RobustStatMean(pX, pW, int(count_subsection), mean, var, 5); 
									//for(size_t s_j = 1; s_j < count_subsection; s_j++)
									//{
									//	//fprintf(pFileTest, "%10.2f\n", pX[s_j] - mean);
									//	if(pEpochTime[subsectionNormalPointlist[s_j]] - pEpochTime[subsectionNormalPointlist[s_j-1]] <= 30.0)
									//	{
									//		fprintf(pFileTest, "PRN%02d %3d %10.4f\n", obsSat.Id, arc_k, pL1_L2[subsectionNormalPointlist[s_j]] - pL1_L2[subsectionNormalPointlist[s_j-1]]);
									//	}
									//}
									arc_k++;
									// 为了增加可靠性, 在每个无周跳的区间内增加该环节
									if(var > m_PreprocessorDefine.threshold_rms_mw)
									{
										printf("MW 无周跳区间标准差超差 var = %.2f/%.2f!(PRN%02d)\n", var, m_PreprocessorDefine.threshold_rms_mw, obsSat.Id);
										for(size_t s_j = 0; s_j < count_subsection; s_j++)
										{
											pSlip[subsectionNormalPointlist[s_j]] = LEOGPSOBSPREPROC_OUTLIER_MWRMS;
										}
										/*fprintf(pFileTest, "PRN%02d %3d MW组合序列弧段无周跳区间标准差 = %.2f\n", obsSat.Id, arc_k, var);
										for(size_t s_j = 1; s_j < count_subsection; s_j++)
										{
											fprintf(pFileTest, "%10.2f\n", pX[s_j] - mean);
										}*/
									}
									delete pX;
									delete pW;
								}
								else
								{
									//MW 组合序列弧段的正常点个数过少!直接标为野值
									for(size_t s_j = 0; s_j < count_subsection; s_j++)									
										pSlip[subsectionNormalPointlist[s_j]] = LEOGPSOBSPREPROC_OUTLIER_COUNT; 
								}
							}
							for(size_t s_i = k; s_i <= k_i; s_i++)
							{
								// 将第一个非野值点, 更新标记为周跳
								if(pSlip[s_i] == LEOGPSOBSPREPROC_NORMAL || pSlip[s_i] == LEOGPSOBSPREPROC_SLIP_MW || pSlip[s_i] == LEOGPSOBSPREPROC_SLIP_IFAMB || pSlip[s_i] == LEOGPSOBSPREPROC_SLIP_L1_L2)
								{
									pSlip[s_i] = LEOGPSOBSPREPROC_NEWARCBEGIN;
									break;
								}
							}
							delete pSubsection_left;
							delete pSubsection_right;

							if(count_restslip > 1)
								m_countRestSlip += count_restslip - 1; // 20170617, 谷德峰添加, 统计周跳信息, 由于被删除的弧段大部分周跳过于频繁，这里只统计保留弧段
							
							m_countSlip += countSlip_k;
							/*if(countSlip_k > 5)
							{
								char info[200];
								sprintf(info, "%-30s %8d %8d 发现周跳发生%2d次, 保留弧段%2d.", 
									          (t0 + pEpochTime[k]).toString().c_str(),
									          obsSat.Id,
											  ArcCounts, 
											  countSlip_k, 
											  count_restslip);
								RuningInfoFile::Add(info);
							}*/
						}						
					}
					if(!m_strPreprocPath.empty())
					{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
						//fprintf(pFile, "PRN %2d -> Arc: %2d\n", obsSat.Id, ArcCounts);
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{
							//if(pSlip[s_i] != TYPE_EDITEDMARK_OUTLIER)
							{
								fprintf(pFile,"%-30s %8.2f %8d %8d %18.3f %18.3f %8d\n",
									(t0 + pEpochTime[s_i]).toString().c_str(),	
									pEpochTime[s_i],
									obsSat.Id,
									ArcCounts,
									pWL_NP[s_i],
									pL1_L2[s_i],
									pSlip[s_i]);
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
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				fclose(pFile);
			}
			// 保存周跳标志
			i = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
			{
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
				i++;
			}
			// 保留
			delete pWL_NP;
			delete pL1_L2;
			delete pEpochTime;
			delete pSlip;			
			delete pIonosphere_phase_code;	
			//fclose(pFileTest);
			return true;
		}

		bool LeoGPSObsPreproc::mainFuncDFreqGPSObsPreproc(Rinex2_1_LeoEditedObsFile  &editedObsFile, double FREQUENCE_L1, double FREQUENCE_L2)
		{
			if(m_obsFile.isEmpty())
			{
				printf("无观测数据, 请确认!\n");
				return  false;				
			}
			char cSatSystem = m_obsFile.m_header.getSatSystemChar(); // 2012/01/03, 增加北斗数据的处理			
			// 根据系统标记和频点信息，获取频率和观测数据类型
			//double FREQUENCE_L1 = GPS_FREQUENCE_L1;
			//double FREQUENCE_L2 = GPS_FREQUENCE_L2;	
			int type_obs_P1  = TYPE_OBS_P1;
			int type_obs_P2  = TYPE_OBS_P2;
			int type_obs_L1  = TYPE_OBS_L1;
			int	type_obs_L2  = TYPE_OBS_L2;
			//if(cSatSystem == 'C') 
			//{//
			//	if(type_obs_L1 == TYPE_OBS_L1)				
			//		FREQUENCE_L1 = BD_FREQUENCE_L1;					
			//	if(type_obs_L1 == TYPE_OBS_L2)
			//	{
			//		FREQUENCE_L1 = BD_FREQUENCE_L2;	
			//		type_obs_P1  = TYPE_OBS_P2;
			//	}
			//	if(type_obs_L1 == TYPE_OBS_L5)
			//	{
			//		FREQUENCE_L1 = BD_FREQUENCE_L5;	
			//		type_obs_P1  = TYPE_OBS_P5;
			//	}
			//	if(type_obs_L2 == TYPE_OBS_L1)
			//	{
			//		FREQUENCE_L2 = BD_FREQUENCE_L1;
			//		type_obs_P2  = TYPE_OBS_P1;
			//	}
			//	if(type_obs_L2 == TYPE_OBS_L2)
			//		FREQUENCE_L2 = BD_FREQUENCE_L2;
			//	if(type_obs_L2 == TYPE_OBS_L5)
			//	{
			//		FREQUENCE_L2 = BD_FREQUENCE_L5;
			//		type_obs_P2  = TYPE_OBS_P5;
			//	}				
			//}
			//// 邵凯，根据TH2修改，2019.10.19
			//if(cSatSystem == 'C')   // BDS2
			//{//
			//	FREQUENCE_L1 = BD_FREQUENCE_L1;
			//	FREQUENCE_L2 = BD_FREQUENCE_L5;	
			//	type_obs_P1  = TYPE_OBS_P1;
			//	type_obs_P2  = TYPE_OBS_P2;	
			//	type_obs_L1  = TYPE_OBS_L1;
			//	type_obs_L2  = TYPE_OBS_L2;
			//}
			//if(cSatSystem == 'C')    // BDS3
			//{//
			//	FREQUENCE_L1 = 1575.42E+6;
			//	FREQUENCE_L2 = 1176.45E+6;	
			//	type_obs_P1  = TYPE_OBS_P1;
			//	type_obs_P2  = TYPE_OBS_P2;	
			//	type_obs_L1  = TYPE_OBS_L1;
			//	type_obs_L2  = TYPE_OBS_L2;
			//}
			double WAVELENGTH_L1 = SPEED_LIGHT / FREQUENCE_L1;
            double WAVELENGTH_L2 = SPEED_LIGHT / FREQUENCE_L2;	
			
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_L2 = -1, nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_S1 = -1, nObsTypes_S2 = -1;
			for(int i = 0; i < m_obsFile.m_header.byObsTypes; i++)
			{
				if(m_obsFile.m_header.pbyObsTypeList[i] == type_obs_L1)  //第一个频点相位
					nObsTypes_L1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == type_obs_L2)  //第二个频点相位
					nObsTypes_L2 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == type_obs_P1)  //第一个频点伪距
					nObsTypes_P1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == type_obs_P2)  //第二个频点伪距
					nObsTypes_P2 = i;
				//if(cSatSystem == 'G')
				//{
					if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_S1)
						nObsTypes_S1 = i;
					if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_S2)
						nObsTypes_S2 = i;
				//}
			}
			if(nObsTypes_L1 == -1 || nObsTypes_L2 == -1 || nObsTypes_P1 == -1 || nObsTypes_P2 == -1) 
				return false;
			vector<Rinex2_1_LeoEditedObsEpoch> editedObsEpochlist;
			editedObsEpochlist.clear();
			vector<Rinex2_1_EditedObsSat> editedObsSatlist;
			getEditedObsSatList(editedObsSatlist);
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{
					// L1
					if(it->second.obsTypeList[nObsTypes_L1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_L1].obs.data == 0)
					{
						it->second.obsTypeList[nObsTypes_L1].obs.data = 0; // 为防止 DBL_MAX 参与运算, 暂时赋值为 0
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
					}
					else
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					// L2
					if(it->second.obsTypeList[nObsTypes_L2].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_L2].obs.data == 0)
					{
						it->second.obsTypeList[nObsTypes_L2].obs.data = 0; // 为防止 DBL_MAX 参与运算, 暂时赋值为 0
						it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
					}
					else
						it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					// P1
					if(it->second.obsTypeList[nObsTypes_P1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_P1].obs.data == 0)
					{
						it->second.obsTypeList[nObsTypes_P1].obs.data = 0; // 为防止 DBL_MAX 参与运算, 暂时赋值为 0
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
					}
					else
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					// P2
					if(it->second.obsTypeList[nObsTypes_P2].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_P2].obs.data == 0)
					{
						it->second.obsTypeList[nObsTypes_P2].obs.data = 0; // 为防止 DBL_MAX 参与运算, 暂时赋值为 0
						it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
					}
					else
						it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
				}
			}
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				char szCodePreprocFileName[300];
				sprintf(szCodePreprocFileName,"%s\\preproc_P1P2.dat",m_strPreprocPath.c_str());
				FILE *pFile = fopen(szCodePreprocFileName,"w+");
				fprintf(pFile, "%-30s %8s %8s %8s %18s %18s %8s\n",
					        "Epoch",
							"T",
							"PRN",
							"Arc",
							"PIF",
							"Fit_PIF",
							"Marks");
				fclose(pFile);
			}
			// 伪码野值探测
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{
				if(nObsTypes_S2 != -1 && m_PreprocessorDefine.bOn_L2SNRLostDiagnose)
				{
					detectL2SNRLost(nObsTypes_S2, nObsTypes_P1, nObsTypes_P2, nObsTypes_L1, nObsTypes_L2, editedObsSatlist[s_i]);
				}
				detectCodeOutlier_ionosphere(nObsTypes_P1, nObsTypes_P2, editedObsSatlist[s_i],FREQUENCE_L1,FREQUENCE_L2);
			}
			datalist_sat2epoch(editedObsSatlist, editedObsEpochlist);
			// 计算概略轨道点
			double  threshold_coarse_orbitinterval = 180;               // 计算概略点的参考时间间隔, 默认180s
			double  cumulate_time = threshold_coarse_orbitinterval * 2; // 累积时间,从第1点开始计算
			vector<int> validindexlist;                                 // 有效索引号列表
			vector<TimePosVel>  coarseorbitlist;                        // 形成概略轨道列表 
			vector<int> locationPointlist;                              // 停靠点列表
			locationPointlist.resize(editedObsEpochlist.size());
			double *pTime = new double [editedObsEpochlist.size()];
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{// 星历检查, 先校验一下每颗 GPS 卫星的星历数据是否完整, 如果此时刻星历数据不完整, 则清除此时刻的该颗 GPS 数据, 2007/08/17
				pTime[s_i] = editedObsEpochlist[s_i].t - editedObsEpochlist[0].t;
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					int nPRN = it->first; 
					SP3Datum sp3Datum;
					CLKDatum ASDatum;
					// 左右各延伸一个点, 判断该点的星历数据是否完整
					if(!m_clkFile.getSatClock(editedObsEpochlist[s_i].t, nPRN, ASDatum, 3 + 2,cSatSystem) || !m_sp3File.getEphemeris(editedObsEpochlist[s_i].t, nPRN, sp3Datum, 9 + 2,cSatSystem))
					{
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
					}
				}
				if(s_i > 0)
					cumulate_time += editedObsEpochlist[s_i].t - editedObsEpochlist[s_i - 1].t;
				// 当累积时间满足条件，才开始计算概略点，以达到降低运算量的目的
				// 同时为保证插值精度，起始点和结束点的概略轨道要计算
				if(cumulate_time >= threshold_coarse_orbitinterval || s_i == editedObsEpochlist.size() - 1)
				{
					double raim_pdop = 0;
					double raim_rms_res = 0;
					POSCLK raim_posclk;
					int raim_flag = RaimSPP_PIF(nObsTypes_P1, nObsTypes_P2, FREQUENCE_L1, FREQUENCE_L2, editedObsEpochlist[s_i], raim_posclk, raim_pdop, raim_rms_res);
					if(raim_flag == 2)
					{// 结果可靠
						cumulate_time = 0;
						editedObsEpochlist[s_i].pos = raim_posclk.getPos();
					    editedObsEpochlist[s_i].clock = raim_posclk.clk;
					    editedObsEpochlist[s_i].byRAIMFlag = raim_flag;
					    editedObsEpochlist[s_i].pdop = raim_pdop;
						validindexlist.push_back(int(s_i));
						TimePosVel coarseorbit;
						coarseorbit.t = editedObsEpochlist[s_i].t;
						coarseorbit.pos = editedObsEpochlist[s_i].pos;
						coarseorbitlist.push_back(coarseorbit);
					}
				}
				// 每个单点将归属于一个有效点, 原则是左归属
				if(validindexlist.size() > 0)
					locationPointlist[s_i] = int(validindexlist.size() - 1);
				else
					locationPointlist[s_i] = 0;
			}
			// 微分平滑求速(10 阶 Lagrange 插值)
			const int nlagrangePoint_left  = 5;   
			const int nlagrangePoint_right = 5;
			int   nlagrangePoint = nlagrangePoint_left + nlagrangePoint_right; 
			size_t validindexcount = coarseorbitlist.size();
			if(validindexcount < size_t(nlagrangePoint)) // 插值点个数太少条件
				return false;
			else
			{   
				double *xa_t = new double [nlagrangePoint];
				double *ya_X = new double [nlagrangePoint];
				double *ya_Y = new double [nlagrangePoint];
				double *ya_Z = new double [nlagrangePoint];
				for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
				{
					// 通过位置插值匹配输出速度
					int k_now = locationPointlist[s_i];
					if(k_now <= nlagrangePoint_left - 1)
						k_now = 0;
					else if(k_now >= int(validindexcount - nlagrangePoint_right))
						k_now = int(validindexcount - nlagrangePoint_right - nlagrangePoint_left);
					else
						k_now = k_now - nlagrangePoint_left + 1;
					// 插值区间[k_now, k_now + nlagrangePoint_left + nlagrangePoint_right - 1]
					for(size_t s_j = k_now; s_j <= size_t(k_now + nlagrangePoint_left + nlagrangePoint_right - 1); s_j++)
					{
						xa_t[s_j - k_now] = pTime[validindexlist[s_j]];
						ya_X[s_j - k_now] = coarseorbitlist[s_j].pos.x;
						ya_Y[s_j - k_now] = coarseorbitlist[s_j].pos.y;
						ya_Z[s_j - k_now] = coarseorbitlist[s_j].pos.z;
					}
					InterploationLagrange(xa_t, ya_X, nlagrangePoint, pTime[s_i], editedObsEpochlist[s_i].pos.x, editedObsEpochlist[s_i].vel.x);
					InterploationLagrange(xa_t, ya_Y, nlagrangePoint, pTime[s_i], editedObsEpochlist[s_i].pos.y, editedObsEpochlist[s_i].vel.y);
					InterploationLagrange(xa_t, ya_Z, nlagrangePoint, pTime[s_i], editedObsEpochlist[s_i].pos.z, editedObsEpochlist[s_i].vel.z);
			        // 钟差采用线性插值
					double x_t[2];
					double y_t[2];
					k_now = locationPointlist[s_i];
					if( k_now <= -1)
						k_now = 0;
					else if(k_now >= int(validindexcount - 1))
						k_now = int(validindexcount - 2);
					else
						k_now = k_now;
					// 插值区间 [ k_now, k_now + 1 ]
					x_t[0] = pTime[validindexlist[k_now]];
					x_t[1] = pTime[validindexlist[k_now + 1]];
					y_t[0] = editedObsEpochlist[validindexlist[k_now]].clock;
					y_t[1] = editedObsEpochlist[validindexlist[k_now + 1]].clock;
					double u = (pTime[s_i] - x_t[0])/(x_t[1] - x_t[0]);
					editedObsEpochlist[s_i].clock = u * y_t[1] +(1 - u) * y_t[0];   // 钟差插值 u * y_t[0] +(1 - u) * y_t[1] 反了, 已修改 (20070917)
					if(editedObsEpochlist[s_i].byRAIMFlag != 1)
						editedObsEpochlist[s_i].byRAIMFlag = 0;
					// 计算GPS卫星的天空视图
					// 根据位置、速度计算天线坐标系
					POS3D S_Z; // Z轴指向卫星
					POS3D S_X; // X轴沿速度方向
					POS3D S_Y; // 右手系
					// 在地固系下计算轨道坐标系坐标轴矢量
					POS3D S_R, S_T, S_N;
					POS6D posvel_i;
					posvel_i.setPos(editedObsEpochlist[s_i].pos);
					posvel_i.setVel(editedObsEpochlist[s_i].vel);
					TimeCoordConvert::getCoordinateRTNAxisVector(editedObsEpochlist[s_i].t, posvel_i, S_R, S_T, S_N);
					//S_X = S_T * (1.0);
				 //   S_Y = S_N * (1.0);
					//S_Z = S_R * (1.0);
					// 考虑天线系到星固系的转换矩阵，用于天线可能安装在非天顶方向，2021.04.30，邵凯
					// 1、先计算星体系，与RTN存在对应关系
					POS3D exBody = S_T;        // 飞行方向
					POS3D eyBody = S_N * (-1); // 天底方向 x 飞行方向
					POS3D ezBody = S_R * (-1); // 天底方向
					Matrix matBody2ECEF(3, 3); // Body到ECEF转换矩阵
					matBody2ECEF.SetElement(0, 0, exBody.x);  // 第一列
					matBody2ECEF.SetElement(1, 0, exBody.y);
					matBody2ECEF.SetElement(2, 0, exBody.z);
					matBody2ECEF.SetElement(0, 1, eyBody.x);  // 第二列
					matBody2ECEF.SetElement(1, 1, eyBody.y);
					matBody2ECEF.SetElement(2, 1, eyBody.z);
					matBody2ECEF.SetElement(0, 2, ezBody.x);  // 第三列
					matBody2ECEF.SetElement(1, 2, ezBody.y);
					matBody2ECEF.SetElement(2, 2, ezBody.z);
					// 2、再计算天线系，与星体系存在已知固定转换关系
					Matrix matAnt = matBody2ECEF * m_matAxisAnt2Body;  // 天线系到地固系的旋转矩阵
					S_X.x = matAnt.GetElement(0, 0);
					S_X.y = matAnt.GetElement(1, 0);
					S_X.z = matAnt.GetElement(2, 0);
					S_X = vectorNormal(S_X);
					S_Y.x = matAnt.GetElement(0, 1);
					S_Y.y = matAnt.GetElement(1, 1);
					S_Y.z = matAnt.GetElement(2, 1);
					S_Y = vectorNormal(S_Y);
					S_Z.x = matAnt.GetElement(0, 2);
					S_Z.y = matAnt.GetElement(1, 2);
					S_Z.z = matAnt.GetElement(2, 2);
					S_Z = vectorNormal(S_Z);
					for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
					{
						SP3Datum sp3Datum;
						if(m_sp3File.getEphemeris(editedObsEpochlist[s_i].t, it->first, sp3Datum, 9, cSatSystem))
						{
							POS3D vecLosECEF = sp3Datum.pos - editedObsEpochlist[s_i].pos; // 视线矢量: 接收机位置指向GPS卫星
							POS3D vecLosXYZ;
							vecLosXYZ.x = vectorDot(vecLosECEF, S_X);
							vecLosXYZ.y = vectorDot(vecLosECEF, S_Y);
							vecLosXYZ.z = vectorDot(vecLosECEF, S_Z);
							vecLosXYZ = vectorNormal(vecLosXYZ);
							it->second.Elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;
							it->second.Azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
							if(it->second.Azimuth < 0)
							{// 变换到[0, 360]
								it->second.Azimuth += 360.0;
							}
						}
						else
						{
							it->second.Elevation = 0.0;
							it->second.Azimuth = 0.0;
						}
					}
				}
				delete xa_t;
				delete ya_X;
				delete ya_Y;
				delete ya_Z;
			}
			delete pTime;

			// 剔除仰角过低弧段数据
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					if(it->second.Elevation <= m_PreprocessorDefine.min_elevation || it->second.Elevation == DBL_MAX)
					{
						if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
						}
						if(it->second.obsTypeList[nObsTypes_P2].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
						}
						if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
						}
						if(it->second.obsTypeList[nObsTypes_L2].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);						
							it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
						}
					}	
				}
			}

			// 多通道 RAIM 检验, 识别采样系统误差
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{	
				if(editedObsEpochlist[s_i].byRAIMFlag == 1) // 跳过已经RAIM检验过的点
					continue;
				double raim_pdop = 0;
				double raim_rms_res = 0;
				POSCLK raim_posclk;
				raim_posclk.x = editedObsEpochlist[s_i].pos.x;
				raim_posclk.y = editedObsEpochlist[s_i].pos.y;
				raim_posclk.z = editedObsEpochlist[s_i].pos.z;
				raim_posclk.clk = editedObsEpochlist[s_i].clock;
				int raim_flag = RaimSPP_PIF(nObsTypes_P1, nObsTypes_P2, FREQUENCE_L1, FREQUENCE_L2, editedObsEpochlist[s_i], raim_posclk, raim_pdop, raim_rms_res);
				if(raim_flag)
				{// 更新概略位置
					editedObsEpochlist[s_i].pos = raim_posclk.getPos();
					editedObsEpochlist[s_i].clock = raim_posclk.clk;
					editedObsEpochlist[s_i].byRAIMFlag = raim_flag;
					editedObsEpochlist[s_i].pdop = raim_pdop;
				}
				else
				{
					editedObsEpochlist[s_i].byRAIMFlag = 0;
					editedObsEpochlist[s_i].pdop = 0;
				}
			}
			// 保存轨道钟差解算结果
			editedObsFile.m_data.resize(editedObsEpochlist.size());
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				editedObsFile.m_data[s_i].t = editedObsEpochlist[s_i].t;
				editedObsFile.m_data[s_i].byEpochFlag = editedObsEpochlist[s_i].byEpochFlag;
				editedObsFile.m_data[s_i].byRAIMFlag = editedObsEpochlist[s_i].byRAIMFlag;
				editedObsFile.m_data[s_i].pdop = editedObsEpochlist[s_i].pdop;
				editedObsFile.m_data[s_i].pos= editedObsEpochlist[s_i].pos;
				editedObsFile.m_data[s_i].vel = editedObsEpochlist[s_i].vel;
				editedObsFile.m_data[s_i].clock = editedObsEpochlist[s_i].clock;
			}
			// 弧段 RAIM 整体检验
			if(m_PreprocessorDefine.bOn_RaimArcChannelBias)
				detectRaimArcChannelBias_PIF(nObsTypes_P1, nObsTypes_P2, editedObsEpochlist);
          
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				char szPhasePreprocFileName[300];
				sprintf(szPhasePreprocFileName,"%s\\preproc_L1L2.dat",m_strPreprocPath.c_str());
				FILE *pFile = fopen(szPhasePreprocFileName,"w+");
				fprintf(pFile, "%-30s %8s %8s %8s %18s %18s %8s\n",
					        "Epoch",
							"T",
							"PRN",
							"Arc",
							"M-W",
							"L1-L2",
							"Marks");
				fclose(pFile);
			}
			m_countSlip = 0; // 20170617, 谷德峰添加, 统计周跳信息
			m_countRestSlip = 0;
			datalist_epoch2sat(editedObsEpochlist, editedObsSatlist);
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
				detectPhaseSlip(nObsTypes_P1, nObsTypes_P2, nObsTypes_L1, nObsTypes_L2,FREQUENCE_L1, FREQUENCE_L2, editedObsSatlist[s_i]);
            datalist_sat2epoch(editedObsSatlist, editedObsEpochlist);
			// 钟差消除
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				// 钟差消除, 在每个历元, 遍历每颗GPS卫星的数据
				if(m_PreprocessorDefine.bOn_ClockEliminate)
				{
					for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
					{
						it->second.obsTypeList[nObsTypes_P1].obs.data -= editedObsFile.m_data[s_i].clock;
						it->second.obsTypeList[nObsTypes_P2].obs.data -= editedObsFile.m_data[s_i].clock;
						it->second.obsTypeList[nObsTypes_L1].obs.data -= editedObsFile.m_data[s_i].clock / WAVELENGTH_L1;
						it->second.obsTypeList[nObsTypes_L2].obs.data -= editedObsFile.m_data[s_i].clock / WAVELENGTH_L2;
					}
					editedObsFile.m_data[s_i].clock = 0;
				}
				editedObsFile.m_data[s_i].bySatCount = editedObsEpochlist[s_i].bySatCount;
				editedObsFile.m_data[s_i].editedObs  = editedObsEpochlist[s_i].editedObs;
			}
			// 进行接收机的天线偏移改正
			for(size_t s_i = 0; s_i < editedObsFile.m_data.size(); s_i++)
			{
				POS3D posLeo = editedObsFile.m_data[s_i].pos;
				POS3D velLeo = editedObsFile.m_data[s_i].vel;
				POS3D correctOffset = GNSSBasicCorrectFunc::correctLeoAntPCO_ECEF(editedObsEpochlist[s_i].t, m_pcoAnt, posLeo, velLeo);
                editedObsFile.m_data[s_i].pos = editedObsFile.m_data[s_i].pos - correctOffset;
			}
			// 输出到 editedObsFile 文件
			editedObsFile.m_header = m_obsFile.m_header;
			editedObsFile.m_header.tmStart = editedObsEpochlist[0].t;           
			editedObsFile.m_header.tmEnd = editedObsEpochlist[editedObsEpochlist.size() - 1].t;
			DayTime T_Now;
			T_Now.Now();
			sprintf(editedObsFile.m_header.szFileDate, "%04d-%02d-%02d %02d:%02d:%02d",
				                                       T_Now.year,
													   T_Now.month,
													   T_Now.day,
											           T_Now.hour,
													   T_Now.minute,
													   int(T_Now.second));
			sprintf(editedObsFile.m_header.szProgramName, "%-20s", "NUDT Toolkit 1.0");
			sprintf(editedObsFile.m_header.szProgramAgencyName, "%-20s", "NUDT");
			editedObsFile.m_header.pstrCommentList.clear();
			char szComment[100];
			sprintf(szComment, "%-60s%20s\n", 
				               "created by LEO dual-frequence GPS preprocess program.", 
							   Rinex2_1_MaskString::szComment);
			editedObsFile.m_header.pstrCommentList.push_back(szComment);
			sprintf(editedObsFile.m_header.szFileType, "%-20s", "PREPROC OBS");
			return true;
		}

		// 子程序名称： pdopSPP   
		// 功能： 获得单点定位的几何精度因子pdop, 用于联合定轨时, 根据几何精度因子剔除个别点
		// 变量类型：index_P1            : 观测类型P1索引
		//           index_P2            : 观测类型P2索引
		//           obsEpoch            : 某时刻的预处理后观测数据, 包含轨道位置
		//           eyeableGPSCount     : 可见有效 GPS 卫星个数
		//           pdop                : 几何精度因子
		// 输入：index_P1, index_P2, obsEpoch
		// 输出：eyeableGPSCount, pdop
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/8/14
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool LeoGPSObsPreproc::pdopSPP(int index_P1, int index_P2, Rinex2_1_LeoEditedObsEpoch obsEpoch, int& eyeableGPSCount, double& pdop)
		{
			char cSatSystem = m_obsFile.m_header.getSatSystemChar(); // 2012/01/03, 增加北斗数据的处理		
			pdop = 0;
			eyeableGPSCount = 0;
            Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); 
			while(it != obsEpoch.editedObs.end())
			{
				Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[index_P2];
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
				{
					Rinex2_1_EditedObsSatMap::iterator jt = it;
					++it;
					obsEpoch.editedObs.erase(jt);
					continue;
				}
				else
				{
					eyeableGPSCount++;
					++it;
					continue;
				}
			}
			if(eyeableGPSCount < 4)  // 可见星要大于或等于4颗
				return false;
			POSCLK posclk;
			posclk.x = obsEpoch.pos.x;
			posclk.y = obsEpoch.pos.y;
			posclk.z = obsEpoch.pos.z;
			posclk.clk = obsEpoch.clock;
			Matrix matA(eyeableGPSCount, 4); // 高斯牛顿迭代的线性化展开矩阵
			Matrix matG_inv(eyeableGPSCount, eyeableGPSCount); // 观测权矩阵
			GPST t_Receive = obsEpoch.t - posclk.clk / SPEED_LIGHT;
			// 双频 P 码消除电离层组合系数
			//double coefficient_ionospherefree = 1 / (1 - pow(FREQUENCE_L1 / FREQUENCE_L2, 2));
			int j = 0;
			for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{// 可以考虑高度角加权处理的影响
				double weight_P_IF = 1.0;
				matG_inv.SetElement(j, j, weight_P_IF);
				j++;
			}
			j = 0;
			for(Rinex2_1_EditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{
				double delay = 0;
				SP3Datum sp3Datum;
				int nPRN = it->first; // 第 j 颗可见 GPS 卫星的卫星号
				char szSatName[4];
				sprintf(szSatName, "%1c%02d", cSatSystem, nPRN);
				szSatName[3] = '\0';
				m_sp3File.getEphemeris_PathDelay(obsEpoch.t, posclk, szSatName, delay, sp3Datum);
				GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum); // 对GPS卫星星历进行地球自转改正
				GPST t_Transmit = t_Receive - delay;
				double distance;
				distance = pow(posclk.x - sp3Datum.pos.x, 2)
						 + pow(posclk.y - sp3Datum.pos.y, 2)
						 + pow(posclk.z - sp3Datum.pos.z, 2);
				distance = sqrt(distance);
				// 计算视线方向, 利用概略点
				matA.SetElement(j, 0, (posclk.x - sp3Datum.pos.x) / distance);
				matA.SetElement(j, 1, (posclk.y - sp3Datum.pos.y) / distance);
				matA.SetElement(j, 2, (posclk.z - sp3Datum.pos.z) / distance);
				matA.SetElement(j, 3,  1.0);
				j++;
			}
			Matrix matAA_inv = (matA.Transpose() * matG_inv * matA).Inv_Ssgj();
			pdop = sqrt(matAA_inv.GetElement(0,0) + matAA_inv.GetElement(1,1) + matAA_inv.GetElement(2,2));
			return true;
		}

		// 子程序名称： obsEdited_LIF   
		// 功能： 进行相位数据的周跳编辑, 如果无法诊断则返回 false
		// 变量类型： index_L1        : 观测类型L1索引
		//            index_L2        : 观测类型L2索引
		//            frequence_L1    : 观测类型L1频率
		//            frequence_L2    : 观测类型L2频率
        //            nPRN            : 卫星号
		//            epoch_j_1       : j_1 时刻的卫星观测数据
		//            epoch_j         : j   时刻的卫星观测数据
		//            res             : 编辑残差
		//            slipFlag        : 周跳标记
		// 输入： index_L1, index_L2, nPRN, epoch_j_1, epoch_j  
		// 输出： res, slipFlag
		// 语言： C++
		// 创建者：谷德峰
		// 创建时间：2009/06/17
		// 版本时间：
		// 修改记录：
		// 备注：
		bool LeoGPSObsPreproc::obsEdited_LIF(int index_L1, int index_L2, double frequence_L1, double frequence_L2, int nPRN, Rinex2_1_LeoEditedObsEpoch epoch_j_1, Rinex2_1_LeoEditedObsEpoch epoch_j, double &res, bool &slipFlag)
		{
			double  FREQUENCE_L1  = frequence_L1;
			double  FREQUENCE_L2  = frequence_L2;
			double  WAVELENGTH_L1 = SPEED_LIGHT / FREQUENCE_L1;
            double  WAVELENGTH_L2 = SPEED_LIGHT / FREQUENCE_L2;
			res = 0;
			slipFlag = false; // 周跳标记
			Rinex2_1_EditedObsSatMap::iterator it = epoch_j.editedObs.begin();
			while(it != epoch_j.editedObs.end())
			{
				int nPRN_i = it->second.Id;
				if(epoch_j_1.editedObs.find(nPRN_i) == epoch_j_1.editedObs.end())
				{// 剔除非公共卫星
					Rinex2_1_EditedObsSatMap::iterator jt = it;
					++it;
					epoch_j.editedObs.erase(jt);
					continue;
				}
				else
				{
					++it;
					continue;
				}
			}
			// 判断卫星 nPRN 的数据是否存在
			size_t count_gpssat = epoch_j.editedObs.size();
            if(epoch_j.editedObs.find(nPRN) == epoch_j.editedObs.end() || count_gpssat < size_t(m_PreprocessorDefine.threshold_gpssatcount))
				return false;
			Matrix matLIF_R(int(count_gpssat), 1);
			Matrix matW(int(count_gpssat), 1);
			Matrix matPRN(int(count_gpssat), 1);
			int count_normalpoints = 0;
			int j = 0;
			double coefficient_ionospherefree = 1 / (1 - pow(FREQUENCE_L1 / FREQUENCE_L2, 2));
			for(Rinex2_1_EditedObsSatMap::iterator it = epoch_j.editedObs.begin(); it != epoch_j.editedObs.end(); ++it)
			{
				matPRN.SetElement(j, 0, it->first); // 卫星标号
				Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[index_L1];
				Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[index_L2];
				double dL1_L2_j_1 = WAVELENGTH_L1 * L1.obs.data - WAVELENGTH_L2 * L2.obs.data;
				double y = WAVELENGTH_L1 * L1.obs.data - (WAVELENGTH_L1 * L1.obs.data - WAVELENGTH_L2 * L2.obs.data) * coefficient_ionospherefree;
				double r = it->second.ReservedField;
				Rinex2_1_EditedObsSatMap::iterator it_1 = epoch_j_1.editedObs.find(it->first);
				L1 = it_1->second.obsTypeList[index_L1];
				L2 = it_1->second.obsTypeList[index_L2];
				double dL1_L2_j = WAVELENGTH_L1 * L1.obs.data - WAVELENGTH_L2 * L2.obs.data;
				y -= WAVELENGTH_L1 * L1.obs.data - (WAVELENGTH_L1 * L1.obs.data - WAVELENGTH_L2 * L2.obs.data) * coefficient_ionospherefree;
				r -= it_1->second.ReservedField;
				matLIF_R.SetElement(j, 0, y + r);
				// 2009/10/31, 继承已有的标记符号
				if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
				&& it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
				&& it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_SLIP
				&& it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_SLIP
				&& it->second.ReservedField != DBL_MAX
				&& it_1->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
				&& it_1->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
				&& it_1->second.ReservedField != DBL_MAX)
				{
					matW.SetElement(j, 0, 1.0);
					count_normalpoints++;
				}
				else
					matW.SetElement(j, 0, 0.0);
				j++;
			}

			double max_res =  0;
			int max_index = -1;
			double recerver_clk =  0;
			while(count_normalpoints >= m_PreprocessorDefine.threshold_gpssatcount)
			{// 计算接收机钟差及残差
				recerver_clk = 0;
				for(int i = 0; i < int(count_gpssat); i++)
				{
					if(matW.GetElement(i, 0) != 0)
					{
						recerver_clk += matLIF_R.GetElement(i, 0);
					}
				}
				recerver_clk = recerver_clk / count_normalpoints;
				// 寻找残差最大的点
				max_res = 0;
			    max_index = -1;
				for(int i = 0; i < int(count_gpssat); i++)
				{
					if(int(matPRN.GetElement(i, 0)) == nPRN)
						res = matLIF_R.GetElement(i, 0) - recerver_clk;
					if(matW.GetElement(i, 0) != 0)
					{
						double res_i = matLIF_R.GetElement(i, 0) - recerver_clk;
						if(fabs(res_i) > max_res)
						{
							max_index = i;
							max_res = fabs(res_i);
						}
					}
				}
				if(max_res <= m_PreprocessorDefine.threshold_editrms_phase)
					break;
				else
				{// 调整权矩阵 matW 
					matW.SetElement(max_index, 0, 0.0);
				    count_normalpoints = 0;
					for(int i = 0; i < int(count_gpssat); i++)
					{
						if(matW.GetElement(i, 0) != 0)
							count_normalpoints++;
					}
				}
			}
			if(count_normalpoints < m_PreprocessorDefine.threshold_gpssatcount)
				return false;
			else
			{
				j = 0;
				for(Rinex2_1_EditedObsSatMap::iterator it = epoch_j.editedObs.begin(); it != epoch_j.editedObs.end(); ++it)
				{// 查看本颗 GPS 卫星的权值是否为 0
					if(it->first == nPRN)
					{
						if(matW.GetElement(j, 0) == 0.0)
							slipFlag = true;
						else
							slipFlag = false;
						break;
					}
					j++;
				}
			}
			return true;
		}

		// 子程序名称： obsEdited_GRAPHIC   
		// 功能： 进行GRAPHIC(半和)组合数据的周跳编辑, 如果无法诊断则返回 false
		// 变量类型： index_P1        : 观测类型P1索引
		//            index_L1        : 观测类型L1索引
        //            nPRN            : 卫星号
		//            epoch_j_1       : j_1 时刻的卫星观测数据
		//            epoch_j         : j   时刻的卫星观测数据
		//            res             : 编辑残差
		//            slipFlag        : 周跳标记
		// 输入： index_P1, index_L1, nPRN, epoch_j_1, epoch_j  
		// 输出： res, slipFlag
		// 语言： C++
		// 创建者：谷德峰
		// 创建时间：2016/09/18
		// 版本时间：
		// 修改记录：
		// 备注：
		bool LeoGPSObsPreproc::obsEdited_GRAPHIC(int index_P1, int index_L1, int nPRN, Rinex2_1_LeoEditedObsEpoch epoch_j_1, Rinex2_1_LeoEditedObsEpoch epoch_j, double &res, bool &slipFlag)
		{
			res = 0;
			slipFlag = false; // 周跳标记
			Rinex2_1_EditedObsSatMap::iterator it = epoch_j.editedObs.begin();
			while(it != epoch_j.editedObs.end())
			{
				int nPRN_i = it->second.Id;
				if(epoch_j_1.editedObs.find(nPRN_i) == epoch_j_1.editedObs.end())
				{// 剔除非公共卫星
					Rinex2_1_EditedObsSatMap::iterator jt = it;
					++it;
					epoch_j.editedObs.erase(jt);
					continue;
				}
				else
				{
					++it;
					continue;
				}
			}
			// 判断卫星 nPRN 的数据是否存在
			size_t count_gpssat = epoch_j.editedObs.size();
            if(epoch_j.editedObs.find(nPRN) == epoch_j.editedObs.end() || count_gpssat < size_t(m_PreprocessorDefine.threshold_gpssatcount))
				return false;
			Matrix matGRAPHIC_R(int(count_gpssat), 1);
			Matrix matW(int(count_gpssat), 1);
			Matrix matPRN(int(count_gpssat), 1);
			int count_normalpoints = 0;
			int j = 0;
			for(Rinex2_1_EditedObsSatMap::iterator it = epoch_j.editedObs.begin(); it != epoch_j.editedObs.end(); ++it)
			{
				matPRN.SetElement(j, 0, it->first); // 卫星标号
				Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[index_L1];
				double y = 0.5 * (GPS_WAVELENGTH_L1 * L1.obs.data + P1.obs.data); // 基于半和改正，利用0.5*(P1+L1)，考虑码偏差
				double r = it->second.ReservedField;
				Rinex2_1_EditedObsSatMap::iterator it_1 = epoch_j_1.editedObs.find(it->first);
				P1 = it_1->second.obsTypeList[index_P1];
				L1 = it_1->second.obsTypeList[index_L1];
				y -= 0.5 * (GPS_WAVELENGTH_L1 * L1.obs.data + P1.obs.data);
				r -= it_1->second.ReservedField;
				matGRAPHIC_R.SetElement(j, 0, y + r);
				// 2009/10/31, 继承已有的标记符号
				if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
				&& it->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
				&& it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_SLIP
				&& it->second.ReservedField != DBL_MAX
				&& it_1->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
				&& it_1->second.obsTypeList[index_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER
				&& it_1->second.ReservedField != DBL_MAX)
				{
					matW.SetElement(j, 0, 1.0);
					count_normalpoints++;
				}
				else
					matW.SetElement(j, 0, 0.0);
				j++;
			}

			double max_res =  0;
			int max_index = -1;
			double recerver_clk =  0;
			while(count_normalpoints >= m_PreprocessorDefine.threshold_gpssatcount)
			{// 计算接收机钟差及残差
				recerver_clk = 0;
				for(int i = 0; i < int(count_gpssat); i++)
				{
					if(matW.GetElement(i, 0) != 0)
					{
						recerver_clk += matGRAPHIC_R.GetElement(i, 0);
					}
				}
				recerver_clk = recerver_clk / count_normalpoints;
				// 寻找残差最大的点
				max_res = 0;
			    max_index = -1;
				for(int i = 0; i < int(count_gpssat); i++)
				{
					if(int(matPRN.GetElement(i, 0)) == nPRN)
						res = matGRAPHIC_R.GetElement(i, 0) - recerver_clk;
					if(matW.GetElement(i, 0) != 0)
					{
						double res_i = matGRAPHIC_R.GetElement(i, 0) - recerver_clk;
						if(fabs(res_i) > max_res)
						{
							max_index = i;
							max_res = fabs(res_i);
						}
					}
				}
				if(max_res <= m_PreprocessorDefine.threshold_editrms_phase)
					break;
				else
				{// 调整权矩阵 matW 
					matW.SetElement(max_index, 0, 0.0);
				    count_normalpoints = 0;
					for(int i = 0; i < int(count_gpssat); i++)
					{
						if(matW.GetElement(i, 0) != 0)
							count_normalpoints++;
					}
				}
			}
			if(count_normalpoints < m_PreprocessorDefine.threshold_gpssatcount)
				return false;
			else
			{
				j = 0;
				for(Rinex2_1_EditedObsSatMap::iterator it = epoch_j.editedObs.begin(); it != epoch_j.editedObs.end(); ++it)
				{// 查看本颗 GPS 卫星的权值是否为 0
					if(it->first == nPRN)
					{
						if(matW.GetElement(j, 0) == 0.0)
							slipFlag = true;
						else
							slipFlag = false;
						break;
					}
					j++;
				}
			}
			return true;
		}

		// 子程序名称： getLeoOrbitPosVel   
		// 功能：通过滑动 lagrange 插值 m_leoOrbitList 获得时刻 t 对应的轨道  
		// 变量类型： t                 : 插值时刻
		//            orbit             : 时刻 t 对应的轨道
		//            nLagrange         : 阶数为 nLagrange - 1 
		// 输入：t, nLagrange
		// 输出：orbit
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2009/06/05
		// 版本时间：
		// 修改记录：
		// 其它： 
		bool LeoGPSObsPreproc::getLeoOrbitPosVel(GPST t, TimePosVel& orbit, unsigned int nLagrange)
		{
			size_t count = m_leoOrbitList.size();
			int nLagrange_left  = int(floor(nLagrange/2.0));   
			int nLagrange_right = int(ceil(nLagrange/2.0));
			if(count < nLagrange)
				return false;
			GPST t_Begin = m_leoOrbitList[0].t;
			GPST t_End   = m_leoOrbitList[count - 1].t;
			double span_Total = t_End - t_Begin;  
			double span_T = t - t_Begin; // 换算成相对时间
			if(span_T < 0 || span_T > span_Total) // 确保 span_T 在有效范围之内
				return false;
			// 搜索法寻找 t 时刻对应的轨道区间
	        int nLeftPos = -1;
			size_t left = 1;
			size_t right = count - 1;
			int n = 0;
			while(left < right) // 采用二分法, 2008/05/11
			{
				n++;
				int    middle = int(left + right)/2;
				double time_L = (m_leoOrbitList[middle - 1].t - t_Begin);
				double time_R = (m_leoOrbitList[middle].t - t_Begin);
				if(span_T >= time_L && span_T <= time_R ) 
				{// 终止条件
					nLeftPos = middle - 1;
					break;
				}
				if(span_T < time_L) 
					right = middle - 1;
				else 
					left  = middle + 1;
			}
			if(right == left)
			{
				double time_L = (m_leoOrbitList[left - 1].t - t_Begin);
				double time_R = (m_leoOrbitList[left].t - t_Begin);
				if(span_T >= time_L && span_T <= time_R) 
				{// 终止条件
					nLeftPos = int(left - 1);
				}
			}
			if(nLeftPos == -1)
				return false;
			// 确定插值区间位置 [nBegin, nEnd]，nEnd - nBegin + 1 = nLagrange
			int nBegin, nEnd; 
			if(nLeftPos - nLagrange_left + 1 < 0) 
			{
				nBegin = 0;
				nEnd   = nLagrange - 1;
			}
			else if(nLeftPos + nLagrange_right >= int(count))
			{
				nBegin = int(count) - nLagrange;
				nEnd   = int(count) - 1;
			}
			else
			{
				nBegin = nLeftPos - nLagrange_left + 1;
				nEnd   = nLeftPos + nLagrange_right;
			}
			double *xa_t  = new double [nLagrange];
			double *ya_x  = new double [nLagrange];
			double *ya_y  = new double [nLagrange];
			double *ya_z  = new double [nLagrange];
			double *ya_vx = new double [nLagrange];
			double *ya_vy = new double [nLagrange];
			double *ya_vz = new double [nLagrange];
			for(int i = nBegin; i <= nEnd; i++)
			{
				 xa_t[i - nBegin] = m_leoOrbitList[i].t - t_Begin;
				 ya_x[i - nBegin] = m_leoOrbitList[i].pos.x;
				 ya_y[i - nBegin] = m_leoOrbitList[i].pos.y;
				 ya_z[i - nBegin] = m_leoOrbitList[i].pos.z;
				ya_vx[i - nBegin] = m_leoOrbitList[i].vel.x;
				ya_vy[i - nBegin] = m_leoOrbitList[i].vel.y;
				ya_vz[i - nBegin] = m_leoOrbitList[i].vel.z;
			}
			orbit.t = t;
			InterploationLagrange(xa_t,  ya_x, nLagrange, span_T, orbit.pos.x);
			InterploationLagrange(xa_t,  ya_y, nLagrange, span_T, orbit.pos.y);
			InterploationLagrange(xa_t,  ya_z, nLagrange, span_T, orbit.pos.z);
			InterploationLagrange(xa_t, ya_vx, nLagrange, span_T, orbit.vel.x);
			InterploationLagrange(xa_t, ya_vy, nLagrange, span_T, orbit.vel.y);
			InterploationLagrange(xa_t, ya_vz, nLagrange, span_T, orbit.vel.z);
			delete  xa_t;
			delete  ya_x;
			delete  ya_y;
			delete  ya_z;
			delete ya_vx;
			delete ya_vy;
			delete ya_vz;
			return true;
		}

		// 子程序名称： mainFuncMixedObsEdit   
		// 功能：GNSS 混合原始观测数据文件编辑
		// 变量类型：editedObsFile   : 编辑后的单系统观测数据文件	
		//			 type_obs_L1     : 第一个频点相位观测类型
		//			 type_obs_L2     : 第二个频点相位观测类型
		// 输入：editedObsFile，type_obs_P1，type_obs_P2，type_obs_L1，type_obs_L2
		// 输出：editedObsFile
		// 其它：
		// 语言：C++
		// 版本号：2014/12/02
		// 生成者：谷德峰、刘俊宏
		// 修改者：
		// 备注：  
		bool LeoGPSObsPreproc::mainFuncDFreqGPSObsEdit(Rinex2_1_LeoEditedObsFile &editedObsFile, double FREQUENCE_L1, double FREQUENCE_L2)
		{
			//FILE* pFileTest = fopen("c:\\wm2.txt", "w+");
			if(m_obsFile.isEmpty())
			{
				printf("无观测数据, 请确认!\n");
				return  false;				
			}
			char cSatSystem = m_obsFile.m_header.getSatSystemChar(); // 2012/01/03, 增加北斗数据的处理			
			// 根据系统标记和频点信息，获取频率和观测数据类型
			//double FREQUENCE_L1 = GPS_FREQUENCE_L1;
			//double FREQUENCE_L2 = GPS_FREQUENCE_L2;	
			int type_obs_P1  = TYPE_OBS_P1;
			int type_obs_P2  = TYPE_OBS_P2;
			int type_obs_L1  = TYPE_OBS_L1;
			int	type_obs_L2  = TYPE_OBS_L2;
			//if(cSatSystem == 'C') 
			//{//
			//	if(type_obs_L1 == TYPE_OBS_L1)				
			//		FREQUENCE_L1 = BD_FREQUENCE_L1;					
			//	if(type_obs_L1 == TYPE_OBS_L2)
			//	{
			//		FREQUENCE_L1 = BD_FREQUENCE_L2;	
			//		type_obs_P1  = TYPE_OBS_P2;
			//	}
			//	if(type_obs_L1 == TYPE_OBS_L5)
			//	{
			//		FREQUENCE_L1 = BD_FREQUENCE_L5;	
			//		type_obs_P1  = TYPE_OBS_P5;
			//	}
			//	if(type_obs_L2 == TYPE_OBS_L1)
			//	{
			//		FREQUENCE_L2 = BD_FREQUENCE_L1;
			//		type_obs_P2  = TYPE_OBS_P1;
			//	}
			//	if(type_obs_L2 == TYPE_OBS_L2)
			//		FREQUENCE_L2 = BD_FREQUENCE_L2;
			//	if(type_obs_L2 == TYPE_OBS_L5)
			//	{
			//		FREQUENCE_L2 = BD_FREQUENCE_L5;
			//		type_obs_P2  = TYPE_OBS_P5;
			//	}				
			//}
			//// 邵凯，根据TH2修改，2019.10.19
			//if(cSatSystem == 'C')   // BDS2
			//{//
			//	FREQUENCE_L1 = BD_FREQUENCE_L1;
			//	FREQUENCE_L2 = BD_FREQUENCE_L5;	
			//	type_obs_P1  = TYPE_OBS_P1;
			//	type_obs_P2  = TYPE_OBS_P2;	
			//	type_obs_L1  = TYPE_OBS_L1;
			//	type_obs_L2  = TYPE_OBS_L2;
			//}
			//if(cSatSystem == 'C')    // BDS3
			//{//
			//	FREQUENCE_L1 = 1575.42E+6;
			//	FREQUENCE_L2 = 1176.45E+6;	
			//	type_obs_P1  = TYPE_OBS_P1;
			//	type_obs_P2  = TYPE_OBS_P2;	
			//	type_obs_L1  = TYPE_OBS_L1;
			//	type_obs_L2  = TYPE_OBS_L2;
			//}
			double WAVELENGTH_L1 = SPEED_LIGHT / FREQUENCE_L1;
            double WAVELENGTH_L2 = SPEED_LIGHT / FREQUENCE_L2;	
			
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_L2 = -1, nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_S1 = -1, nObsTypes_S2 = -1;
			for(int i = 0; i < m_obsFile.m_header.byObsTypes; i++)
			{
				if(m_obsFile.m_header.pbyObsTypeList[i] == type_obs_L1)  //第一个频点相位
					nObsTypes_L1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == type_obs_L2)  //第二个频点相位
					nObsTypes_L2 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == type_obs_P1)  //第一个频点伪距
					nObsTypes_P1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == type_obs_P2)  //第二个频点伪距
					nObsTypes_P2 = i;
				//if(cSatSystem == 'G')
				//{
					if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_S1)
						nObsTypes_S1 = i;
					if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_S2)
						nObsTypes_S2 = i;
				//}
			}
			if(nObsTypes_L1 == -1 || nObsTypes_L2 == -1 || nObsTypes_P1 == -1 || nObsTypes_P2 == -1) 
				return false;			
			vector<Rinex2_1_LeoEditedObsEpoch> editedObsEpochlist;
			getEditedObsEpochList(editedObsEpochlist);               // 
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					//// L1
					//if(it->second.obsTypeList[nObsTypes_L1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_L1].obs.data == 0.0)
					//{
					//	it->second.obsTypeList[nObsTypes_L1].obs.data = 0.0; // 为防止 DBL_MAX 参与运算, 暂时赋值为 0
					//	it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					//	it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					//}
					//else
					//	it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					//// L2
					//if(it->second.obsTypeList[nObsTypes_L2].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_L2].obs.data == 0.0)
					//{
					//	it->second.obsTypeList[nObsTypes_L2].obs.data == 0.0;
					//	it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					//	it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					//}
					//else
					//	it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					//// P1
					//if(it->second.obsTypeList[nObsTypes_P1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_P1].obs.data == 0.0)
					//{
					//	it->second.obsTypeList[nObsTypes_P1].obs.data == 0.0;
					//	it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					//	it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					//}
					//else
					//	it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					//// P2
					//if(it->second.obsTypeList[nObsTypes_P2].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_P2].obs.data == 0.0)
					//{
					//	it->second.obsTypeList[nObsTypes_P2].obs.data == 0.0;
					//	it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					//	it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					//}
					//else
					//	it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					// 修改：如果一个为0，则所有标记为不可用
					if(
						(it->second.obsTypeList[nObsTypes_L1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_L1].obs.data == 0.0)
						|| (it->second.obsTypeList[nObsTypes_L2].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_L2].obs.data == 0.0)
						|| (it->second.obsTypeList[nObsTypes_P1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_P1].obs.data == 0.0)
						|| (it->second.obsTypeList[nObsTypes_P2].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_P2].obs.data == 0.0)
						)
					{
						it->second.obsTypeList[nObsTypes_L1].obs.data = 0.0; // 为防止 DBL_MAX 参与运算, 暂时赋值为 0
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_L2].obs.data = 0.0;
						it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_P1].obs.data = 0.0;
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_P2].obs.data = 0.0;
						it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);

					}
					else
					{
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
						it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
						it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					}			
					// 判断星历是否完整
					int nPRN = it->first; 
					SP3Datum sp3Datum;
					CLKDatum ASDatum;
					// 左右各延伸一个点, 判断该点的星历数据是否完整
					if(!m_clkFile.getSatClock(editedObsEpochlist[s_i].t, nPRN, ASDatum, 3 + 2, cSatSystem) || !m_sp3File.getEphemeris(editedObsEpochlist[s_i].t, nPRN, sp3Datum, 9 + 2, cSatSystem))
					{
						if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
						}
						if( it->second.obsTypeList[nObsTypes_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
						}
						if( it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
						}
						if( it->second.obsTypeList[nObsTypes_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
						}
					}
				}
			}
			// 针对 sy1 卫星进行 L2 失锁诊断
			vector<Rinex2_1_EditedObsSat> editedObsSatlist;
			if(nObsTypes_S2 != -1 && m_PreprocessorDefine.bOn_L2SNRLostDiagnose)
			{
				datalist_epoch2sat(editedObsEpochlist, editedObsSatlist);
				for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
				{
					detectL2SNRLost(nObsTypes_S2, nObsTypes_P1, nObsTypes_P2, nObsTypes_L1, nObsTypes_L2, editedObsSatlist[s_i]);
				}
				datalist_sat2epoch(editedObsSatlist, editedObsEpochlist);
			}
			double rms_residual_code = 0; // 伪码编辑残差均方根   
			size_t count_normalpoints_all = 0; // 正常伪码观测点的个数
			double coefficient_ionospherefree = 1 / (1 - pow(FREQUENCE_L1 / FREQUENCE_L2, 2)); // 双频 P 码消除电离层组合系数
            vector<int> validindexlist;
			validindexlist.resize(editedObsEpochlist.size());
			editedObsFile.m_data.resize(editedObsEpochlist.size());
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				validindexlist[s_i] = -1;
				editedObsFile.m_data[s_i].t = editedObsEpochlist[s_i].t;
				editedObsFile.m_data[s_i].clock = 0.0; // 钟差初始化为 0
				editedObsFile.m_data[s_i].byRAIMFlag = 2;
				// 钟差要迭代进行计算
				int k = 0;
				while(k <= 1)
				{
					// 更新真实观测时刻
					GPST t_Receive;
					if(m_PreprocessorDefine.bOn_ClockEliminate)
						t_Receive = editedObsEpochlist[s_i].t; // 可认为经704软校正的采样时刻是较为准确的, 2010/06/12
					else
						t_Receive = editedObsEpochlist[s_i].t - editedObsEpochlist[s_i].clock / SPEED_LIGHT;
					TimePosVel orbit_t;
                    if(!getLeoOrbitPosVel(t_Receive, orbit_t))
					{
						editedObsFile.m_data[s_i].clock = 0;
						editedObsFile.m_data[s_i].byRAIMFlag = 0;
						// 该时刻的数据为无效时刻, 丢弃该时刻的数据
						for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
						{
							if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
								it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							}
							if( it->second.obsTypeList[nObsTypes_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
								it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							}
							if( it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
								it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							}
							if( it->second.obsTypeList[nObsTypes_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
								it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							}
						}
						//printf("%6d时刻 %s 数据无效(getLeoOrbitPosVel)!\n", s_i, editedObsFile.m_data[s_i].t.toString().c_str());
					}
					editedObsFile.m_data[s_i].pos  = orbit_t.pos;
					editedObsFile.m_data[s_i].vel  = orbit_t.vel;
					POSCLK posclk;
					posclk.x = orbit_t.pos.x;
					posclk.y = orbit_t.pos.y;
					posclk.z = orbit_t.pos.z;
					posclk.clk = editedObsFile.m_data[s_i].clock;
					if(m_PreprocessorDefine.bOn_ClockEliminate)
						posclk.clk = 0.0; // 可认为经704软校正的采样时刻是较为准确的, 2015/05/05
					// 根据观测时刻, 更新 GPS 卫星轨道位置和速度, 以及 GPS 卫星钟差
					// 记录y, 几何距离, gps卫星钟差
					size_t count_gpssat = editedObsEpochlist[s_i].editedObs.size();
					Matrix matP_IF(int(count_gpssat), 1);
					Matrix matR(int(count_gpssat), 1);
					Matrix matW(int(count_gpssat), 1);
					int count_normalpoints = 0;
					int j = 0;
					for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
					{
						int nPRN = it->first;
						Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[nObsTypes_P1];
						Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[nObsTypes_P2];
						// 双频 P 码消除电离层组合 R = R1- (R1 - R2) / (1 - (f1^2 / f2^2))
						double y = P1.obs.data - (P1.obs.data - P2.obs.data) * coefficient_ionospherefree;
                        // 迭代计算卫星信号传播时间
				        double delay = 0;
						SP3Datum sp3Datum;
						char szSatName[4];
						sprintf(szSatName, "%1c%02d", cSatSystem, nPRN);
						szSatName[3] = '\0';
						m_sp3File.getEphemeris_PathDelay(editedObsEpochlist[s_i].t, posclk, szSatName, delay, sp3Datum);
						// 对GPS卫星星历进行地球自转改正
						GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
						// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
						GPST t_Transmit = t_Receive - delay;
						double distance;
						distance = pow(posclk.x - sp3Datum.pos.x, 2)
								 + pow(posclk.y - sp3Datum.pos.y, 2)
								 + pow(posclk.z - sp3Datum.pos.z, 2);
						distance = sqrt(distance);
						POS3D E; // 记录概略展开系数
						E.x = (posclk.x - sp3Datum.pos.x) / distance;
						E.y = (posclk.y - sp3Datum.pos.y) / distance;
						E.z = (posclk.z - sp3Datum.pos.z) / distance;
						// 对观测值 y 进行误差修正
						// 包括: GPS卫星钟差改正, GPS卫星相对论修正等
						// 1.GPS卫星钟差改正
						CLKDatum ASDatum;
						m_clkFile.getSatClock(t_Transmit, nPRN, ASDatum, 3, cSatSystem); // 获得 GPS 信号发射时间的卫星钟差改正
						double correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT;  // 等效距离
						y = y + correct_gpsclk;
						// 2.GPS卫星相对论改正
						double correct_relativity = ( sp3Datum.pos.x * sp3Datum.vel.x 
													+ sp3Datum.pos.y * sp3Datum.vel.y
													+ sp3Datum.pos.z * sp3Datum.vel.z ) * (-2.0) / SPEED_LIGHT;
						y = y + correct_relativity;
						// 3. 接收机位置偏心改正
						POS3D posLeo = editedObsFile.m_data[s_i].pos;
						POS3D velLeo = editedObsFile.m_data[s_i].vel;
						POS3D correctOffset = GNSSBasicCorrectFunc::correctLeoAntPCO_ECEF(editedObsEpochlist[s_i].t, m_pcoAnt, posLeo, velLeo);
						double correct_LeoAntOffset = -(correctOffset.x * E.x + correctOffset.y * E.y + correctOffset.z * E.z);
						y = y + correct_LeoAntOffset;						
						matP_IF.SetElement(j, 0, y);
						matR.SetElement(j, 0, distance);
						if(it->second.obsTypeList[nObsTypes_P1].byEditedMark1 == TYPE_EDITEDMARK_NORMAL
						&& it->second.obsTypeList[nObsTypes_P2].byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						{
							matW.SetElement(j, 0, 1.0);
							count_normalpoints++;
						}
						else
							matW.SetElement(j, 0, 0.0);
						j++;
					}
					Matrix matRes(int(count_gpssat), 1); // 计算残差
					double max_res      =  0;
					int    max_index    = -1;
					double recerver_clk =  0;
					// 迭代剔除伪码野值
					while(count_normalpoints >= m_PreprocessorDefine.threshold_gpssatcount)
					{
						// 计算接收机钟差及残差
						recerver_clk = 0;
						for(int i = 0; i < int(count_gpssat); i++)
						{
							if(matW.GetElement(i, 0) != 0)
								recerver_clk += matP_IF.GetElement(i, 0) - matR.GetElement(i, 0);
						}
						recerver_clk = recerver_clk / count_normalpoints;
						// 寻找残差最大的点
						max_res =  0;
					    max_index = -1;
						for(int i = 0; i < int(count_gpssat); i++)
						{
							if(matW.GetElement(i, 0) != 0)
							{
								double res_i = matP_IF.GetElement(i, 0) - matR.GetElement(i, 0) - recerver_clk;
								matRes.SetElement(i, 0, res_i);
								if(fabs(res_i) > max_res)
								{
									max_index = i;
									max_res = fabs(res_i);
								}
							}
						}
						if(max_res <= m_PreprocessorDefine.threshold_editrms_code)
						{
							break;
						}
						else
						{// 调整权矩阵 matW 
							//char info[200];
							//sprintf(info, "%s %02d %14.2lf %10.4lf", editedObsEpochlist[s_i].t.toString().c_str(), max_index + 1, recerver_clk, max_res);
							//RuningInfoFile::Add(info);
							matW.SetElement(max_index, 0, 0.0);
							count_normalpoints = 0;
							for(int i = 0; i < int(count_gpssat); i++)
							{
								if(matW.GetElement(i, 0) != 0)
									count_normalpoints++;
							}
							continue;
						}
					}
					if(k == 0 && count_normalpoints >= m_PreprocessorDefine.threshold_gpssatcount)
					{// 第1次迭代主要目的为了更新钟差
						editedObsFile.m_data[s_i].clock = recerver_clk;
						k++;
						continue;
					}
					else if(k == 1 && count_normalpoints >= m_PreprocessorDefine.threshold_gpssatcount)
					{// 第1次迭代主要目的为了编辑伪码野值
						validindexlist[s_i] = 1;
						editedObsFile.m_data[s_i].clock = recerver_clk;
                        // 标记伪码的野值
						int j = 0;
						for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
						{
							if(matW.GetElement(j, 0) == 0)
							{
								if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
								{
									it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_PIF);
									it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_PIF);
								}
								if( it->second.obsTypeList[nObsTypes_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
								{
									it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_PIF);
									it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_PIF);
								}
							}
							else
							{// 记录所有正常点的残差
								count_normalpoints_all++;
								rms_residual_code += pow(matRes.GetElement(j, 0), 2);
							}
							j++;
						}
						break;
					}
					else
					{// 情形1: k == 0 && count_normalpoints < m_PreprocessorDefine.threshold_gpssatcount, 比如缺LEO星历或可视卫星个数偏少
					 // 情形2: k == 1 && count_normalpoints < m_PreprocessorDefine.threshold_gpssatcount, 比如可视卫星个数偏少 
						validindexlist[s_i] = -1;
						editedObsFile.m_data[s_i].clock = 0;
						for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
						{// 该时刻的数据为无效时刻, 丢弃该时刻的数据
							if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
								it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
							}
							if( it->second.obsTypeList[nObsTypes_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
								it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
							}
							if( it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
								it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
							}
							if( it->second.obsTypeList[nObsTypes_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
								it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
							}
						}
                        break;
					}
				}
			}
			rms_residual_code = sqrt(rms_residual_code / count_normalpoints_all);
			// 利用保留位记录每颗卫星的保留值: 观测数据的修正值 - R(概略距离)
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				if(validindexlist[s_i] == -1)
					continue;
				// 更新真实观测时刻
				GPST t_Receive;
				if(m_PreprocessorDefine.bOn_ClockEliminate)
					t_Receive = editedObsEpochlist[s_i].t; // 可认为经704软校正的采样时刻是较为准确的, 2010/06/12
				else
					t_Receive = editedObsEpochlist[s_i].t - editedObsFile.m_data[s_i].clock / SPEED_LIGHT;
				TimePosVel orbit_t;
                if(!getLeoOrbitPosVel(t_Receive, orbit_t))
					continue;
				editedObsFile.m_data[s_i].pos  = orbit_t.pos;
				editedObsFile.m_data[s_i].vel  = orbit_t.vel;
				POSCLK posclk;
				posclk.x = orbit_t.pos.x;
				posclk.y = orbit_t.pos.y;
				posclk.z = orbit_t.pos.z;
				posclk.clk = editedObsFile.m_data[s_i].clock;
				if(m_PreprocessorDefine.bOn_ClockEliminate)
					posclk.clk = 0.0; // 可认为经704软校正的采样时刻是较为准确的, 2015/05/05
				// 计算GPS卫星的天空视图
				POS3D S_Z; // Z轴指向卫星
				POS3D S_X; // X轴沿速度方向
				POS3D S_Y; // 右手系
				// 在地固系下计算轨道坐标系坐标轴矢量
				POS3D S_R, S_T, S_N;
				POS6D posvel_i;
				posvel_i.setPos(editedObsFile.m_data[s_i].pos);
				posvel_i.setVel(editedObsFile.m_data[s_i].vel);
				TimeCoordConvert::getCoordinateRTNAxisVector(editedObsEpochlist[s_i].t, posvel_i, S_R, S_T, S_N);
				//S_X = S_T * (1.0);
				//   S_Y = S_N * (1.0);
				//S_Z = S_R * (1.0);
				// 考虑天线系到星固系的转换矩阵，用于天线可能安装在非天顶方向，2021.04.30，邵凯
				// 1、先计算星体系，与RTN存在对应关系
				POS3D exBody = S_T;        // 飞行方向
				POS3D eyBody = S_N * (-1); // 天底方向 x 飞行方向
				POS3D ezBody = S_R * (-1); // 天底方向
				Matrix matBody2ECEF(3, 3); // Body到ECEF转换矩阵
				matBody2ECEF.SetElement(0, 0, exBody.x);  // 第一列
				matBody2ECEF.SetElement(1, 0, exBody.y);
				matBody2ECEF.SetElement(2, 0, exBody.z);
				matBody2ECEF.SetElement(0, 1, eyBody.x);  // 第二列
				matBody2ECEF.SetElement(1, 1, eyBody.y);
				matBody2ECEF.SetElement(2, 1, eyBody.z);
				matBody2ECEF.SetElement(0, 2, ezBody.x);  // 第三列
				matBody2ECEF.SetElement(1, 2, ezBody.y);
				matBody2ECEF.SetElement(2, 2, ezBody.z);
				// 2、再计算天线系，与星体系存在已知固定转换关系
				Matrix matAnt = matBody2ECEF * m_matAxisAnt2Body;  // 天线系到地固系的旋转矩阵
				S_X.x = matAnt.GetElement(0, 0);
				S_X.y = matAnt.GetElement(1, 0);
				S_X.z = matAnt.GetElement(2, 0);
				S_X = vectorNormal(S_X);
				S_Y.x = matAnt.GetElement(0, 1);
				S_Y.y = matAnt.GetElement(1, 1);
				S_Y.z = matAnt.GetElement(2, 1);
				S_Y = vectorNormal(S_Y);
				S_Z.x = matAnt.GetElement(0, 2);
				S_Z.y = matAnt.GetElement(1, 2);
				S_Z.z = matAnt.GetElement(2, 2);
				S_Z = vectorNormal(S_Z);
				// 根据观测时刻, 更新 GPS 卫星轨道位置和速度, 以及 GPS 卫星钟差
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					int nPRN = it->first;
			        double delay = 0;
					SP3Datum sp3Datum;
					char szSatName[4];
					sprintf(szSatName, "%1c%02d", cSatSystem, nPRN);
					szSatName[3] = '\0';
					if(!m_sp3File.getEphemeris_PathDelay(editedObsEpochlist[s_i].t, posclk, szSatName, delay, sp3Datum))
						continue;
					// 对 GPS 卫星星历进行地球自转改正
					GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
					// 计算天空视图
					POS3D vecLosECEF = sp3Datum.pos - posclk.getPos(); // 视线矢量: 接收机位置指向GPS卫星
					POS3D vecLosXYZ;
					vecLosXYZ.x = vectorDot(vecLosECEF, S_X);
					vecLosXYZ.y = vectorDot(vecLosECEF, S_Y);
					vecLosXYZ.z = vectorDot(vecLosECEF, S_Z);
					vecLosXYZ = vectorNormal(vecLosXYZ);
					it->second.Elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;
					it->second.Azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
					if(it->second.Azimuth < 0)
					{// 变换到[0, 360]
						it->second.Azimuth += 360.0;
					}
					// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
					GPST t_Transmit = t_Receive - delay;
					double distance;
					distance = pow(posclk.x - sp3Datum.pos.x, 2)
							 + pow(posclk.y - sp3Datum.pos.y, 2)
							 + pow(posclk.z - sp3Datum.pos.z, 2);
					distance = sqrt(distance);
					POS3D E; // 记录概略展开系数
					E.x = (posclk.x - sp3Datum.pos.x) / distance;
					E.y = (posclk.y - sp3Datum.pos.y) / distance;
					E.z = (posclk.z - sp3Datum.pos.z) / distance;
					// 对观测值 y 进行误差修正
					// 包括: GPS卫星钟差改正, GPS卫星相对论修正等
					// 1.GPS卫星钟差改正
					CLKDatum ASDatum;
					if(!m_clkFile.getSatClock(t_Transmit, nPRN, ASDatum, 3, cSatSystem))
						continue;
					double correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT;  // 等效距离
					// 2.GPS卫星相对论改正
					double correct_relativity = ( sp3Datum.pos.x * sp3Datum.vel.x 
												+ sp3Datum.pos.y * sp3Datum.vel.y
												+ sp3Datum.pos.z * sp3Datum.vel.z ) * (-2.0) / SPEED_LIGHT;
					// 3.接收机位置偏心改正
					POS3D posLeo = editedObsFile.m_data[s_i].pos;
					POS3D velLeo = editedObsFile.m_data[s_i].vel;
					POS3D correctOffset = GNSSBasicCorrectFunc::correctLeoAntPCO_ECEF(editedObsEpochlist[s_i].t, m_pcoAnt, posLeo, velLeo);
					double correct_LeoAntOffset = -(correctOffset.x * E.x + correctOffset.y * E.y + correctOffset.z * E.z);
					it->second.ReservedField = correct_gpsclk + correct_relativity + correct_LeoAntOffset - distance;
				}
			}
			// 根据观测仰角剔除野值
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					if(it->second.Elevation <= m_PreprocessorDefine.min_elevation || it->second.Elevation == DBL_MAX)
					{
						if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
						}
						if(it->second.obsTypeList[nObsTypes_P2].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
						}
						if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
						}
						if(it->second.obsTypeList[nObsTypes_L2].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);						
							it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
						}
					}	
				}
			}
			//// 根据载噪比CN0剔除野值，邵凯，2020.7.19
			//if(nObsTypes_S1 != -1)
			//{
			//	for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			//	{
			//		for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
			//		{
			//			if( it->second.obsTypeList[nObsTypes_S1].obs.data <= m_PreprocessorDefine.min_CN0)
			//			{
			//				if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
			//				{
			//					it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
			//					it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
			//				}
			//				if(it->second.obsTypeList[nObsTypes_P2].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
			//				{
			//					it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
			//					it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
			//				}
			//				if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
			//				{
			//					it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
			//					it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
			//				}
			//				if(it->second.obsTypeList[nObsTypes_L2].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
			//				{
			//					it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);						
			//					it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
			//				}
			//			}
			//		}
			//	}
			//}
			// 计算 pdop, 为了提高效率也可以提前, 可以直接利用前面的 E 进行计算
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				int eyeableGPSCount = 0;
				editedObsFile.m_data[s_i].pdop = 0;
				editedObsEpochlist[s_i].pos = editedObsFile.m_data[s_i].pos;
				pdopSPP(nObsTypes_P1, nObsTypes_P2,editedObsEpochlist[s_i], eyeableGPSCount, editedObsFile.m_data[s_i].pdop);
			}
			// 进行钟差归属计算
			double clock_first =  0;
			int    i_first   = -1;
			double clock_now   =  0;
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				if(validindexlist[s_i] == 1)
				{// 更新当前的钟差数据
					clock_now = editedObsFile.m_data[s_i].clock;
					if(i_first < 0)
					{
						i_first = int(s_i);
						clock_first = editedObsFile.m_data[s_i].clock;
						continue;
					}
				}
				else
				{// 将无效钟差更新为左侧的有效的钟差点
					if(i_first >= 0)
						editedObsFile.m_data[s_i].clock = clock_now;
				}
			}
			if(i_first > 0)
			{
				for(size_t s_i = 0; s_i < size_t(i_first); s_i++)
				{
					editedObsFile.m_data[s_i].clock = clock_first;
				}
			}

			FILE *pFile;
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				char szPhasePreprocFileName[300];
				sprintf(szPhasePreprocFileName,"%s\\preproc_L1L2.dat",m_strPreprocPath.c_str());
				pFile = fopen(szPhasePreprocFileName,"w+");
				fprintf(pFile, "%-30s %8s %8s %8s %18s %18s %8s\n",
				            "Epoch",
							"T",
							"PRN",
							"Arc",
							"M-W",
							"Res_Edited",
							"Marks");
			}
			m_countSlip = 0; // 20170617, 谷德峰添加, 统计周跳信息
			m_countRestSlip = 0;
			// 相位数据周跳探测, 继承先前的野值和周跳探测标记
			datalist_epoch2sat(editedObsEpochlist, editedObsSatlist);			
			int arc_k  = 0;
			int ArcCounts = 0;
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{
				Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSatlist[s_i].editedObs.begin();
				GPST t0 = it0->first; 
				int nPRN = it0->second.Id;
				double *pEpochTime = new double[editedObsSatlist[s_i].editedObs.size()];
				int *pSlip = new int [editedObsSatlist[s_i].editedObs.size()];
				int *pEpochId = new int [editedObsSatlist[s_i].editedObs.size()];
				double *pLIF = new double [editedObsSatlist[s_i].editedObs.size()];
				double *pPhaseRes = new double [editedObsSatlist[s_i].editedObs.size()];
				double *pWL_NP = new double[editedObsSatlist[s_i].editedObs.size()];
				double *pAmb = new double[editedObsSatlist[s_i].editedObs.size()];
				int *pCodeEditedFlag = new int [editedObsSatlist[s_i].editedObs.size()]; 
				int i = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{
					pPhaseRes[i] = 0.0;
					pEpochTime[i] = it->first - t0;
					pEpochId[i]  = it->second.nObsTime;
					Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[nObsTypes_P1];
					Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[nObsTypes_P2];
					Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[nObsTypes_L1];
					Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[nObsTypes_L2];
					pLIF[i] = WAVELENGTH_L1 * L1.obs.data - (WAVELENGTH_L1 * L1.obs.data - WAVELENGTH_L2 * L2.obs.data) * coefficient_ionospherefree;
					// 构造宽巷载波相位 widelane_L 和窄巷伪距 narrowlane_P
				    double widelane_L   = (FREQUENCE_L1 * L1.obs.data * WAVELENGTH_L1 - FREQUENCE_L2 * L2.obs.data * WAVELENGTH_L2) / (FREQUENCE_L1 - FREQUENCE_L2);
				    double narrowlane_P = (FREQUENCE_L1 * P1.obs.data + FREQUENCE_L2 * P2.obs.data) / (FREQUENCE_L1 + FREQUENCE_L2);
				    pWL_NP[i] = (widelane_L - narrowlane_P) / GPS_WAVELENGTH_W; // melbourne-wuebbena 组合量
					// 构造无电离层组合
					double PIF = P1.obs.data  - (P1.obs.data - P2.obs.data) * coefficient_ionospherefree;
					double LIF = L1.obs.data * WAVELENGTH_L1 - (L1.obs.data * WAVELENGTH_L1 - L2.obs.data * WAVELENGTH_L2) * coefficient_ionospherefree;
					pAmb[i] = LIF - PIF;
					// 继承前面的伪码观测数据编辑结果, 补充相位野值
					// 增加有效时刻判断, 以保证钟差数据是有效的, 2009/12/25
					// pSlip 的初始化仅包含两部分信息, 未知点TYPE_EDITEDMARK_UNKNOWN和野值TYPE_EDITEDMARK_OUTLIER
					if(L1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || L2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || validindexlist[pEpochId[i]] == -1)
						pSlip[i] = LEOGPSOBSEDIT_OUTLIER_LIF; // 20150529, TYPE_EDITEDMARK_OUTLIER = 2, 后面obsPreprocInfo2EditedMark1(TYPE_EDITEDMARK_OUTLIER) = 0, 标记会存在问题
					else
						pSlip[i] = LEOGPSOBSEDIT_UNKNOWN;
					if(P1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && P2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						pCodeEditedFlag[i] = LEOGPSOBSEDIT_NORMAL; 
					else
						pCodeEditedFlag[i] = TYPE_EDITEDMARK_OUTLIER;
					i++;
				}
				 // 对每颗 GPS 卫星的每个跟踪弧段进行周跳探测
				size_t k = 0;
				size_t k_i = k;
				// 获得每个连续跟踪弧段的数据, 并进行周跳探测
				while(1)
				{
					if(k_i + 1 >= editedObsSatlist[s_i].editedObs.size())
						goto newArc;
					else
					{// 判断 k_i+1 与 k_i 是否位于同一跟踪弧段					
						double clock_k = editedObsFile.m_data[pEpochId[k_i + 1]].clock;
			            double clock_k_1 = editedObsFile.m_data[pEpochId[k_i]].clock;
						if(pEpochTime[k_i + 1] - pEpochTime[k_i] <= min(m_PreprocessorDefine.threshold_gap, m_PreprocessorDefine.max_arclengh)
						&&(m_PreprocessorDefine.bOn_ClockJumpDiagnose && fabs(clock_k - clock_k_1) < m_PreprocessorDefine.threshold_ClockJumpSize))	
						{
							k_i++;
							continue;
						}
						else if(pEpochTime[k_i + 1] - pEpochTime[k_i] <= min(m_PreprocessorDefine.threshold_gap, m_PreprocessorDefine.max_arclengh)
						&& !m_PreprocessorDefine.bOn_ClockJumpDiagnose)
						{
							k_i++;
							continue;
						}
						else // k_i + 1 为新弧段的起点
						{
							GPST t_now = t0 + pEpochTime[k_i + 1];//测试代码
							bool bfind_gap = false;
							if((pEpochTime[k_i + 1] - pEpochTime[k_i] > m_PreprocessorDefine.threshold_gap) && (pEpochTime[k_i + 1] - pEpochTime[k_i] < m_PreprocessorDefine.max_arclengh))
							{
								bfind_gap = true;
							}
							//// 发现钟跳
							//if(m_PreprocessorDefine.bOn_ClockJumpDiagnose && fabs(clock_k - clock_k_1) >= m_PreprocessorDefine.threshold_ClockJumpSize)
							//{
							//}
							goto newArc;
						}
					}
					newArc: // 本弧段 [k, k_i] 数据处理 
					{
						// 整理未曾标记数据
						vector<size_t> unknownPointlist;
						unknownPointlist.clear();
						for(size_t s_ii = k; s_ii <= k_i; s_ii++)
						{// 未知数据标记
							if(pSlip[s_ii] == LEOGPSOBSEDIT_UNKNOWN)
								unknownPointlist.push_back(s_ii); 
						}
						size_t count_unknownpoints = unknownPointlist.size(); 
						if(count_unknownpoints <= m_PreprocessorDefine.min_arcpointcount)
						{// 新弧段内数据正常点个数太少, 直接丢弃
							for(size_t s_ii = 0; s_ii < count_unknownpoints; s_ii++)
							{
								pSlip[unknownPointlist[s_ii]] = LEOGPSOBSEDIT_OUTLIER_COUNT;
							}
						}
						else
						{// 进行相位野值剔除
							ArcCounts++;
							vector<int> slipMarklist; // 1 - 周跳; 0 - 正常点
							slipMarklist.resize(count_unknownpoints);
							for(size_t s_ii = 1; s_ii < count_unknownpoints; s_ii++)
							{
								bool slipFlag;
								int nObsTime_j_1 = pEpochId[unknownPointlist[s_ii - 1]];
								int nObsTime_j = pEpochId[unknownPointlist[s_ii]];
								double res;
								if(obsEdited_LIF(nObsTypes_L1, nObsTypes_L2, FREQUENCE_L1,FREQUENCE_L2, nPRN, editedObsEpochlist[nObsTime_j_1], editedObsEpochlist[nObsTime_j], res, slipFlag))
								{
									if(slipFlag)
										slipMarklist[s_ii] = 1;
									else
										slipMarklist[s_ii] = 0;
								}
								else
								{// 如果 s_ii 时刻的周跳无法诊断(如 n≤2, 或 s_ii 时刻为无效历元), 为了保守起见, 则直接判定 s_ii 时刻为野值
									slipMarklist[s_ii] = 1;
								}
							}
							// 进行野值诊断, 诊断 s_ii 为野值条件: s_ii 为周跳且s_ii + 1为周跳。
							for(size_t s_ii = 1; s_ii < count_unknownpoints - 1; s_ii++)
							{
								if(slipMarklist[s_ii] == 1 && slipMarklist[s_ii + 1] == 1)
								{
									pSlip[unknownPointlist[s_ii]] = LEOGPSOBSEDIT_OUTLIER_LIF;
								}
							}
							// 第一点的判断
							if(slipMarklist[1] == 1)
								pSlip[unknownPointlist[0]] = LEOGPSOBSEDIT_OUTLIER_LIF;
							// 最后一点直接判断
							if(slipMarklist[count_unknownpoints - 1] == 1)
								pSlip[unknownPointlist[count_unknownpoints - 1]] = LEOGPSOBSEDIT_OUTLIER_LIF;
							// 剔除野值 LEOGPSOBSEDIT_OUTLIER_LIF 的影响
							size_t s_ii = 0;
							while(s_ii < unknownPointlist.size())
							{
								if(pSlip[unknownPointlist[s_ii]] == LEOGPSOBSEDIT_UNKNOWN)
									s_ii++;
								else// 在进行周跳探测时, 先将野值 erase
									unknownPointlist.erase(unknownPointlist.begin() + s_ii);
							}
							count_unknownpoints = unknownPointlist.size();
							// 到此处, pSlip[unknownPointlist[i]] 仅包含 LEOGPSOBSEDIT_UNKNOWN
							// 进行周跳探测
							size_t count_slip = 0;
							if(count_unknownpoints > 0)
								pPhaseRes[unknownPointlist[0]] = 0;
							for(size_t s_ii = 1; s_ii < count_unknownpoints; s_ii++)
							{
								bool slipFlag;
								int nObsTime_j_1 = pEpochId[unknownPointlist[s_ii - 1]];
								int nObsTime_j   = pEpochId[unknownPointlist[s_ii]];
								double res;
								//if(nPRN == 30 && editedObsEpochlist[nObsTime_j].t - GPST(2006,1, 2, 21, 39, 10.0000000) > 0.0)
								//{// 测试代码
								//	GPST t0 = editedObsEpochlist[pEpochId[k]].t;
								//	GPST t1 = editedObsEpochlist[pEpochId[k_i]].t;
								//	int kkkk = 0;
								//	kkkk += 1;
								//}
								/*if(fabs(pEpochTime[unknownPointlist[s_ii]] - 3.0 * 3600) < 20)
								{
									double aaa=0;
								}*/
								if(obsEdited_LIF(nObsTypes_L1, nObsTypes_L2, FREQUENCE_L1,FREQUENCE_L2, nPRN, editedObsEpochlist[nObsTime_j_1], editedObsEpochlist[nObsTime_j], res, slipFlag))
								{
									pPhaseRes[unknownPointlist[s_ii]] = res;
									if(slipFlag)
									{
										pSlip[unknownPointlist[s_ii]] = LEOGPSOBSEDIT_SLIP_LIF;
										count_slip++;

										/*char info[200];
										sprintf(info, "obsEdited_LIF发现周跳发生.");
										RuningInfoFile::Add(info);*/
									}
								}
								else
								{
									pSlip[unknownPointlist[s_ii]] = LEOGPSOBSEDIT_SLIP_LIF;
									count_slip++;
								}
								//// 测试代码,刘俊宏
								//if((nPRN == 2 || nPRN == 6 || nPRN == 10 || nPRN == 15 || nPRN == 16 || nPRN == 21 || nPRN == 29 || nPRN == 30)
								//	&& (pEpochTime[unknownPointlist[s_ii]] >= 2.5 * 3600.0 && pEpochTime[unknownPointlist[s_ii]] <= 3.5 * 3600.0))
								//{
								//	fprintf(peditfile,"%s %02d %10.6lf %2d  %1d\n",
								//		editedObsEpochlist[nObsTime_j].t.toString().c_str(),
								//		nPRN,
								//		pPhaseRes[unknownPointlist[s_ii]],
								//		pSlip[unknownPointlist[s_ii]],
								//		slipFlag);
								//}
							}
							// 第一点的判断
							if(count_unknownpoints > 1)
							{
								if(pSlip[unknownPointlist[1]] == LEOGPSOBSEDIT_SLIP_LIF)
									pSlip[unknownPointlist[0]] = LEOGPSOBSEDIT_OUTLIER_LIF;
							}

							if(!m_strPreprocPath.empty())
							{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
								for(size_t s_i = k; s_i <= k_i; s_i++)
								{
									fprintf(pFile,"%-30s %8.2f %8d %8d %18.3f %18.3f %8d\n",
									    (t0 + pEpochTime[s_i]).toString().c_str(),
										pEpochTime[s_i],
										nPRN,
										ArcCounts,
										pWL_NP[s_i],
										pPhaseRes[s_i],
										pSlip[s_i]);
								}
							}
                            // 到此处, pSlip[unknownPointlist[i]] 中包含 LEOGPSOBSEDIT_UNKNOWN 、LEOGPSOBSEDIT_SLIP_LIF 和 LEOGPSOBSEDIT_OUTLIER_LIF (仅在第一点处) 三种标记
							// 无周跳弧段的内符合诊断
							vector<size_t> slipindexlist;
							slipindexlist.clear();
							for(size_t s_ii = k + 1; s_ii <= k_i; s_ii++)
							{
								if(pSlip[s_ii] == LEOGPSOBSEDIT_SLIP_LIF)
									slipindexlist.push_back(s_ii); 
							}
							size_t count_slips = slipindexlist.size();
							size_t *pSubsection_left  = new size_t [count_slips + 1];
							size_t *pSubsection_right = new size_t [count_slips + 1];
							// 记录周跳的左右端点值
							if(count_slips > 0)
							{ 
								pSubsection_left[0] = k;
								for(size_t s_ii = 0; s_ii < count_slips; s_ii++)
								{
									pSubsection_right[s_ii]    = slipindexlist[s_ii] -  1 ;
									pSubsection_left[s_ii + 1] = slipindexlist[s_ii] ;
								}
								pSubsection_right[count_slips] = k_i; 
							}
							else
							{
								pSubsection_left[0]  = k;
								pSubsection_right[0] = k_i;
							} 

							m_countSlip += int(slipindexlist.size()); // 20170617, 谷德峰添加, 统计周跳信息

							int count_restslip = 0;
							for(size_t s_ii = 0; s_ii < count_slips + 1; s_ii++)
							{// pSlip[unknownPointlist[i]] 中包含LEOGPSOBSEDIT_OUTLIER_LIF和LEOGPSOBSEDIT_UNKNOWN
								// 整理 [pSubsection_left[s_ii], pSubsection_right[s_ii]]
								{
									int count_normalpoints_i = 0;
									vector<size_t> normalPointlist;
									normalPointlist.clear();
									for(size_t s_jj = pSubsection_left[s_ii]; s_jj <= pSubsection_right[s_ii]; s_jj++)
									{// pSlip[i] 中包含LEOGPSOBSEDIT_OUTLIER_LIF、LEOGPSOBSEDIT_SLIP_LIF和LEOGPSOBSEDIT_UNKNOWN
										if(pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_LIF || pSlip[s_jj] == LEOGPSOBSEDIT_UNKNOWN)
										{// 要求相位和伪码数据均正常
											normalPointlist.push_back(s_jj);
											count_normalpoints_i++;  
										}
									}
									if(count_normalpoints_i > int(m_PreprocessorDefine.min_arcpointcount))
									{
										count_restslip++;
										//bool mark_slip = false;//后面程序会将第一个非野值点标记为周跳，此处可省，2014.3.19，刘俊宏
										//for(size_t s_jj = pSubsection_left[s_ii]; s_jj <= pSubsection_right[s_ii]; s_jj++)
										//{ 
										//	// 先将每个小弧段第一个非野值点, 更新标记为周跳
										//	if(!mark_slip && (pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_LIF || pSlip[s_jj] == LEOGPSOBSEDIT_UNKNOWN))
										//	{
										//		if(s_ii == 0)// 首个弧段的第一个非野值点更新标记为 LEOGPSOBSPREPROC_NEWARCBEGIN
										//			pSlip[s_jj] = LEOGPSOBSEDIT_NEWARCBEGIN;
										//		else
										//			pSlip[s_jj] = LEOGPSOBSEDIT_SLIP_LIF;
										//		mark_slip = true;
										//		break;
										//	}
										//}
									}
									else
									{// 删除正常点过少的弧段
										for(size_t s_jj = pSubsection_left[s_ii]; s_jj <= pSubsection_right[s_ii]; s_jj++)
										{
											//if(pCodeEditedFlag[s_jj] == LEOGPSOBSEDIT_NORMAL) // 2013/06/21, 确保伪码和相位均删除
											//	pCodeEditedFlag[s_jj] = LEOGPSOBSEDIT_OUTLIER_COUNT;
											if(pSlip[s_jj] != LEOGPSOBSEDIT_OUTLIER_LIF)
												pSlip[s_jj] = LEOGPSOBSEDIT_OUTLIER_COUNT;
											
										}
									}
								}

								if(m_PreprocessorDefine.bOn_SlipEditedMWDiagnose) // 2009/12/01 添加, 针对试验卫星伪码与相位不匹配现象
								{
									/*
										不匹配的原因是伪码的原因, 主要来sy1的钟的调整
										1. 如果相位发生野值, 无法判断相位是否匹配, 直接判定伪码不匹配
										2. 如果相位正常, 只对伪码正常点进行相位匹配判断, 而且只进行大周跳诊断
									*/
									int count_normalpoints_i = 0;
									vector<size_t> normalPointlist;
									normalPointlist.clear();
									for(size_t s_jj = pSubsection_left[s_ii]; s_jj <= pSubsection_right[s_ii]; s_jj++)
									{// pSlip[i] 中包含LEOGPSOBSEDIT_OUTLIER_COUNT、LEOGPSOBSEDIT_OUTLIER_LIF、LEOGPSOBSEDIT_SLIP_LIF和LEOGPSOBSEDIT_UNKNOWN
										// 如果相位发生野值, 无法判断相位是否匹配, 直接判定伪码不匹配
										if(m_PreprocessorDefine.bOn_POutlierAccordingL)
										{
											if(pSlip[s_jj] == LEOGPSOBSEDIT_OUTLIER_COUNT || pSlip[s_jj] == LEOGPSOBSEDIT_OUTLIER_LIF)
											{
												if(pCodeEditedFlag[s_jj] == LEOGPSOBSEDIT_NORMAL)
													pCodeEditedFlag[s_jj] = LEOGPSOBSEDIT_OUTLIER_LIF; // 由于相位野值, 防止相位与伪码不匹配
											}
										}
										if(pSlip[s_jj] == LEOGPSOBSEDIT_UNKNOWN || pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_LIF
										&& pCodeEditedFlag[s_jj] == LEOGPSOBSEDIT_NORMAL)
										{// 要求相位和伪码数据均正常 
										 // 注: 20150604, 此处导致正常弧段的初始时刻周跳标记点因为伪码不正常可能被排除掉
											normalPointlist.push_back(s_jj);
											count_normalpoints_i++;  
										}
									}
									if(count_normalpoints_i >= int(m_PreprocessorDefine.min_arcpointcount))
									{
										// 第一步: 计算MW组合量历元差数据的方差
										// 构造MW组合量历元差数据
										double *pDWL_NP = new double[count_normalpoints_i - 1];
										for(int s_kk = 1; s_kk < count_normalpoints_i; s_kk++)
											pDWL_NP[s_kk - 1] = pWL_NP[normalPointlist[s_kk]] - pWL_NP[normalPointlist[s_kk - 1]] ;
										double var = RobustStatRms(pDWL_NP, int(count_normalpoints_i - 1));
										delete pDWL_NP;
										// 第二步：进行野值剔除
										// 20071012 添加, 利用 threshold_slipsize_wm 对 threshold_outlier 的上界进行控制
										// 因为当周跳发生在序列中间附近时, var 可能会超界, 影响野值探测
										double threshold_outlier = min(5 * var, m_PreprocessorDefine.threshold_slipsize_mw);
										// [1, nCount_points - 2]
										for(int s_kk = 1; s_kk < count_normalpoints_i - 1; s_kk++)
										{
											if(fabs(pWL_NP[normalPointlist[s_kk]]     - pWL_NP[normalPointlist[s_kk-1]]) > threshold_outlier
											&& fabs(pWL_NP[normalPointlist[s_kk + 1]] - pWL_NP[normalPointlist[s_kk] ])  > threshold_outlier)
											{
												if(pCodeEditedFlag[normalPointlist[s_kk]] == LEOGPSOBSEDIT_NORMAL)
													pCodeEditedFlag[normalPointlist[s_kk]] = LEOGPSOBSEDIT_OUTLIER_MW;
											}
										}
										// 首尾两点 0 和 nCount_points - 1
										if(pCodeEditedFlag[normalPointlist[1]] != LEOGPSOBSEDIT_NORMAL)
											pCodeEditedFlag[normalPointlist[0]] = LEOGPSOBSEDIT_OUTLIER_MW;
										else
										{
											if(fabs(pWL_NP[normalPointlist[0]] - pWL_NP[normalPointlist[1]])  > threshold_outlier)
												pCodeEditedFlag[normalPointlist[0]] = LEOGPSOBSEDIT_OUTLIER_MW;
										}
										if(pCodeEditedFlag[normalPointlist[count_normalpoints_i - 2]] != LEOGPSOBSEDIT_NORMAL)
											pCodeEditedFlag[normalPointlist[count_normalpoints_i - 1]] = LEOGPSOBSEDIT_OUTLIER_MW;
										else
										{
											if(fabs(pWL_NP[normalPointlist[count_normalpoints_i - 1]] - pWL_NP[normalPointlist[count_normalpoints_i - 2]])  > threshold_outlier)
												pCodeEditedFlag[normalPointlist[count_normalpoints_i - 1]] = LEOGPSOBSEDIT_OUTLIER_MW;
										}									
										// 清除 normalPointlist 中 pCodeEditedFlag 为野值数据, 确保 normalPointlist 中相位伪码均正常
										size_t s_iii = 0;
										while(s_iii < normalPointlist.size())
										{// pCodeEditedFlag中包含LEOGPSOBSEDIT_OUTLIER_LIF、LEOGPSOBSEDIT_OUTLIER_MW和LEOGPSOBSEDIT_NORMAL
											if((pSlip[normalPointlist[s_iii]] == LEOGPSOBSEDIT_UNKNOWN || pSlip[normalPointlist[s_iii]] == LEOGPSOBSEDIT_SLIP_LIF)
											&&  pCodeEditedFlag[normalPointlist[s_iii]] == LEOGPSOBSEDIT_NORMAL)
												s_iii++;
											else
											{// 在进行周跳探测时, 先将野值 erase
												int nObsTime = pEpochId[normalPointlist[s_iii]];
												normalPointlist.erase(normalPointlist.begin() + s_iii);
											}
										}
										count_normalpoints_i = int(normalPointlist.size());
										// 第三步：进行大周跳探测
										if(count_normalpoints_i <= 3 )
										{// 个数太少则将伪码、相位观测数据直接丢弃
											for(size_t s_jj = pSubsection_left[s_ii]; s_jj <= pSubsection_right[s_ii]; s_jj++)
											{
												if(pCodeEditedFlag[s_jj] == LEOGPSOBSEDIT_NORMAL)
													pCodeEditedFlag[s_jj] = LEOGPSOBSEDIT_OUTLIER_COUNT;
												if(pSlip[s_jj] == LEOGPSOBSEDIT_UNKNOWN || pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_LIF)
													pSlip[s_jj] = LEOGPSOBSEDIT_OUTLIER_COUNT;
											}
										}
										else
										{
											// [1, nCount_points - 2]
											double threshold_largeslip = m_PreprocessorDefine.threshold_slipsize_mw;
											for(int s_kk = 1; s_kk < count_normalpoints_i - 1; s_kk++)
											{
												// 大周跳发生, 每个大周跳在探测后, 其信息都被保存下来了
												if(fabs(pWL_NP[normalPointlist[s_kk]]     - pWL_NP[normalPointlist[s_kk - 1]]) >  threshold_largeslip
												&& fabs(pWL_NP[normalPointlist[s_kk + 1]] - pWL_NP[normalPointlist[s_kk] ])    <= threshold_largeslip) 
												{
													pSlip[normalPointlist[s_kk]] = LEOGPSOBSEDIT_SLIP_MW;
													pCodeEditedFlag[normalPointlist[s_kk]] = LEOGPSOBSEDIT_SLIP_MW; // 邵凯，+
													//printf("MW 组合发现大周跳历元差 = %.2f !(threshold = %.2f)\n", pWL_NP[normalPointlist[s_kk]]     - pWL_NP[normalPointlist[s_kk - 1]], threshold_largeslip);
												}
												else
												{
													/* 
														消电离层组合检验, 2008-07-11,
														M-W组合量只能识别 L1 - L2 的周跳, 因此无法识别两个频率发生的等大小的周跳,
														而等大小的周跳同样会对相位无电离层组合带来影响, 因此在这里要补充关于无电 
														离层组合的大周跳探测, 以确保在精密定轨中迭代收殓      
													*/
													if(m_PreprocessorDefine.bOn_IonosphereFree)
													{
														if(fabs(pAmb[normalPointlist[s_kk]]     - pAmb[normalPointlist[s_kk - 1]]) > threshold_largeslip * 4
														&& fabs(pAmb[normalPointlist[s_kk + 1]] - pAmb[normalPointlist[s_kk] ])   <= threshold_outlier * 4)
														{// 消电离层组合要放大观测噪声 3 倍左右, 大约是 mw 组合的 4 倍
															pSlip[normalPointlist[s_kk]] = LEOGPSOBSEDIT_SLIP_IFAMB;
														}
													}
												}
											}
											// 因为当今最后一点发生周跳时, 前面的相位野值判断结果是最后一点与前一点保持一致, 为非野值点
											// 而且周跳判断也将最后一点漏掉了, 因此增加对最后一点的判断, 此时s_i = count_normalpoints_i - 1.
											if(fabs(pWL_NP[normalPointlist[count_normalpoints_i - 1]] - pWL_NP[normalPointlist[count_normalpoints_i - 2]]) > threshold_largeslip) 
												pSlip[normalPointlist[count_normalpoints_i - 1]] = LEOGPSOBSEDIT_OUTLIER_MW;
											else
											{
												if(m_PreprocessorDefine.bOn_IonosphereFree)
												{
													if(fabs(pAmb[normalPointlist[count_normalpoints_i - 1]]   - pAmb[normalPointlist[count_normalpoints_i - 2]])   > threshold_largeslip * 4)
														pSlip[normalPointlist[count_normalpoints_i - 1]] = LEOGPSOBSEDIT_OUTLIER_IFAMB;
												}
											}
											/*
												无周跳弧段的内符合诊断, 2008/11/11
												该项检查主要针对sy1卫星, var超标
											*/
											// 更新周跳区间, 因为周跳区间内[pSubsection_left[s_ii], pSubsection_right[s_ii]]已经产生了新的周跳
											vector<size_t> slipindexlist_ii;
											vector<int>    slipMarkList_ii;
											slipindexlist_ii.clear();
											slipMarkList_ii.clear();
											for(size_t s_jj = pSubsection_left[s_ii] + 1; s_jj <= pSubsection_right[s_ii]; s_jj++)
											{
												if(pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_LIF
												|| pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_MW
												|| pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_IFAMB)
												{
													slipindexlist_ii.push_back(s_jj);
													slipMarkList_ii.push_back(pSlip[s_jj]); 
												}
											}
											// 记录周跳的左右端点值
											size_t count_slips_ii = slipindexlist_ii.size();
											size_t *pSubsection_left_ii  = new size_t [count_slips_ii + 1];
											size_t *pSubsection_right_ii = new size_t [count_slips_ii + 1];
											if(count_slips_ii > 0)
											{ 
												pSubsection_left_ii[0] = pSubsection_left[s_ii];
												for(size_t s_jj = 0; s_jj < count_slips_ii; s_jj++)
												{
													pSubsection_right_ii[s_jj]    = slipindexlist_ii[s_jj] -  1 ;
													pSubsection_left_ii[s_jj + 1] = slipindexlist_ii[s_jj] ;
												}
												pSubsection_right_ii[count_slips_ii] = pSubsection_right[s_ii]; 
											}
											else
											{
												pSubsection_left_ii[0]  = pSubsection_left[s_ii];
												pSubsection_right_ii[0] = pSubsection_right[s_ii];
											} 
											for(size_t s_jj = 0; s_jj < count_slips_ii + 1; s_jj++)
											{
												// 整理 [pSubsection_left_ii[s_jj], pSubsection_right_ii[s_jj]]
												vector<size_t> subsectionNormalPointlist;
												subsectionNormalPointlist.clear();
												for(size_t s_kk = pSubsection_left_ii[s_jj]; s_kk <= pSubsection_right_ii[s_jj]; s_kk++)
												{
													if((pSlip[s_kk] == LEOGPSOBSEDIT_UNKNOWN
													 || pSlip[s_kk] == LEOGPSOBSEDIT_SLIP_LIF
													 || pSlip[s_kk] == LEOGPSOBSEDIT_SLIP_MW 
													 || pSlip[s_kk] == LEOGPSOBSEDIT_SLIP_IFAMB))
													 //&& pCodeEditedFlag[s_kk] == LEOGPSOBSEDIT_NORMAL) // 20150430, 避免如果伪码异常对应的相位数据被subsectionNormalPointlist漏掉，无法准确恢复周跳
														subsectionNormalPointlist.push_back(s_kk);    
												}
												size_t count_subsection = subsectionNormalPointlist.size(); 
												if(count_subsection > m_PreprocessorDefine.min_arcpointcount)
												{   
													double *pX = new double [count_subsection];
													double *pW = new double [count_subsection];
													double mean = 0;
													double var  = 0;
													for(size_t s_kk = 0; s_kk < count_subsection; s_kk++)
														pX[s_kk] = pWL_NP[subsectionNormalPointlist[s_kk]];  
													RobustStatMean(pX, pW, int(count_subsection), mean, var, 5);
													arc_k++;
													if(var > m_PreprocessorDefine.threshold_rms_mw)
													{
														printf("MW 无周跳区间标准差超差 var = %.2f/%.2f!(PRN%02d)\n", var, m_PreprocessorDefine.threshold_rms_mw, nPRN);
														for(size_t s_kk = 0; s_kk < count_subsection; s_kk++)
														{
															if(pCodeEditedFlag[subsectionNormalPointlist[s_kk]] == LEOGPSOBSEDIT_NORMAL) // 20150430, 避免如果伪码异常对应的相位数据被subsectionNormalPointlist漏掉，无法准确恢复周跳
															{
																pSlip[subsectionNormalPointlist[s_kk]] = LEOGPSOBSEDIT_OUTLIER_MWRMS;
																pCodeEditedFlag[subsectionNormalPointlist[s_kk]] = LEOGPSOBSEDIT_OUTLIER_MWRMS; 
															}
														}
														/*fprintf(pFileTest, "PRN%02d %3d MW组合序列弧段无周跳区间标准差 = %10.2lf\n", editedObsSatlist[s_i].Id, arc_k, var);
														for(size_t s_kk = 1; s_kk < count_subsection; s_kk++)
														{
															fprintf(pFileTest, "%10.2lf\n", pX[s_kk] - mean);
														}*/
													}
													else
													{ 
														for(size_t s_kk = 0; s_kk < count_subsection; s_kk++)
														{
															if(pW[s_kk] == 1 && pCodeEditedFlag[subsectionNormalPointlist[s_kk]] == LEOGPSOBSEDIT_NORMAL) // 20150430, 避免如果伪码异常对应的相位数据被subsectionNormalPointlist漏掉，无法准确恢复周跳
															{// 在无周跳区间内进行野值探测
																pSlip[subsectionNormalPointlist[s_kk]] = LEOGPSOBSEDIT_OUTLIER_MWRMS;
																pCodeEditedFlag[subsectionNormalPointlist[s_kk]] = LEOGPSOBSEDIT_OUTLIER_MWRMS; 
															}
														}
													}
													// 20150604, 调整到该位置, 无论var超差与否, 均需要更新第一个正常点
													for(size_t s_kk = 0; s_kk < count_subsection; s_kk++)
													{// 将第一个非野值点, 更新标记为周跳
														if(pSlip[subsectionNormalPointlist[s_kk]] == LEOGPSOBSEDIT_UNKNOWN
														|| pSlip[subsectionNormalPointlist[s_kk]] == LEOGPSOBSEDIT_SLIP_LIF
														|| pSlip[subsectionNormalPointlist[s_kk]] == LEOGPSOBSEDIT_SLIP_MW
														|| pSlip[subsectionNormalPointlist[s_kk]] == LEOGPSOBSEDIT_SLIP_IFAMB)
														{
															if(s_jj == 0) // 首个弧段的第一个非野值点更新标记为 LEOGPSOBSPREPROC_NEWARCBEGIN
															{
																pSlip[subsectionNormalPointlist[s_kk]] = LEOGPSOBSPREPROC_NEWARCBEGIN;
																//printf("%s %d %d\n", 
																//	   (t0 + pEpochTime[subsectionNormalPointlist[s_kk]]).toString().c_str(),
																//	   nPRN, LEOGPSOBSPREPROC_NEWARCBEGIN);
															}
															else          // 其余弧段的第一个非野值点更新标记为周跳
															{
																pSlip[subsectionNormalPointlist[s_kk]] = slipMarkList_ii[s_jj - 1];
																//printf("%s %d %d\n", 
																//	   (t0 + pEpochTime[subsectionNormalPointlist[s_kk]]).toString().c_str(),
																//	   nPRN, slipMarkList_ii[s_jj - 1]);
															}
															break;
														}
													}
                                                    delete pX;
										            delete pW;
												}
												else
												{
													/*for(size_t s_kk = 0; s_kk < count_subsection; s_kk++)
													{
														pSlip[subsectionNormalPointlist[s_kk]] = LEOGPSOBSEDIT_OUTLIER_COUNT; 
														pCodeEditedFlag[subsectionNormalPointlist[s_kk]] = LEOGPSOBSEDIT_OUTLIER_COUNT;
													}*/
													for(size_t s_kk = pSubsection_left_ii[s_jj]; s_kk <= pSubsection_right_ii[s_jj]; s_kk++)
													{// 2013/06/21, 替换上面代码, subsectionNormalPointlist 中漏掉了哪些相位正常但伪码不正常的点, 需要分开及进行标记
													 // 对于那些本来正常, 经过MW探测后, MW正常点较少, 需要整个清除的弧段, 上面代码漏掉了部分相位正常但伪码不正常的点
														if(pSlip[s_kk] == LEOGPSOBSEDIT_UNKNOWN
														|| pSlip[s_kk] == LEOGPSOBSEDIT_SLIP_LIF
														|| pSlip[s_kk] == LEOGPSOBSEDIT_SLIP_MW 
														|| pSlip[s_kk] == LEOGPSOBSEDIT_SLIP_IFAMB)
															pSlip[s_kk] = LEOGPSOBSEDIT_OUTLIER_COUNT;     
														if(pCodeEditedFlag[s_kk] == LEOGPSOBSEDIT_NORMAL)
															pCodeEditedFlag[s_kk] = LEOGPSOBSEDIT_OUTLIER_COUNT; 
													}
												}
											}
											delete pSubsection_left_ii;
											delete pSubsection_right_ii;
										}
									}
									else
									{
										for(size_t s_jj = pSubsection_left[s_ii]; s_jj <= pSubsection_right[s_ii]; s_jj++)
										{// pSlip[i] 中包含LEOGPSOBSEDIT_OUTLIER_LIF、LEOGPSOBSEDIT_SLIP_LIF和LEOGPSOBSEDIT_UNKNOWN
											if(pSlip[s_jj] == LEOGPSOBSEDIT_UNKNOWN || pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_LIF)
												pSlip[s_jj] = LEOGPSOBSEDIT_OUTLIER_COUNT;
											if(pCodeEditedFlag[s_jj] == LEOGPSOBSEDIT_NORMAL)
												pCodeEditedFlag[s_jj] = LEOGPSOBSEDIT_OUTLIER_COUNT;
										}
									}
								}
							}

							if(slipindexlist.size() > 0)
							{
								/*char info[200];
								sprintf(info, "弧段%3d   obsEdited_LIF 发现周跳发生%2d次, 保留弧段%2d.", ArcCounts, slipindexlist.size(), count_left);
								RuningInfoFile::Add(info);*/

								/*if(slipindexlist.size() > 5)
								{
									char info[200];
									sprintf(info, "%-30s %8d %8d 发现周跳发生%2d次, 保留弧段%2d.", 
												   (t0 + pEpochTime[k]).toString().c_str(),
												  nPRN,
												  ArcCounts, 
												  slipindexlist.size(), 
												  count_restslip);
									RuningInfoFile::Add(info);
								}*/

								if(count_restslip > 1)
									m_countRestSlip += count_restslip - 1; // 20170617, 谷德峰添加, 统计周跳信息, 由于被删除的弧段大部分周跳过于频繁，这里只统计保留弧段
							}

                            // 20150623, 谷德峰
							for(size_t s_ii = k; s_ii <= k_i; s_ii++)
							{
								// 将第一个非野值点, 更新标记为新弧段起点
								if(pSlip[s_ii] == LEOGPSOBSEDIT_UNKNOWN
								|| pSlip[s_ii] == LEOGPSOBSEDIT_SLIP_LIF
								|| pSlip[s_ii] == LEOGPSOBSEDIT_SLIP_MW
								|| pSlip[s_ii] == LEOGPSOBSEDIT_SLIP_IFAMB
								|| pSlip[s_ii] == LEOGPSOBSPREPROC_NEWARCBEGIN)
								{
									pSlip[s_ii] = LEOGPSOBSPREPROC_NEWARCBEGIN;
									break;
								}
							}
							delete pSubsection_left;
							delete pSubsection_right;
							// 将未知点 TYPE_EDITEDMARK_UNKNOWN 恢复为正常点 LEOGPSOBSEDIT_NORMAL
							for(size_t s_ii = k; s_ii <= k_i; s_ii++)
							{
								if(pSlip[s_ii] == LEOGPSOBSEDIT_UNKNOWN)
									pSlip[s_ii] = LEOGPSOBSEDIT_NORMAL;  
							}
						}
						if(k_i + 1 >= editedObsSatlist[s_i].editedObs.size())
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
				// 保存野值和周跳标记
				i = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{
					if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[i]);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[i]);
					}
					if(it->second.obsTypeList[nObsTypes_L2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[i]);	
						it->second.obsTypeList[nObsTypes_L2].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[i]);
					}		
					if(it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						if(pCodeEditedFlag[i]!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(pCodeEditedFlag[i]);
							it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(pCodeEditedFlag[i]);
						}
					}
					if(it->second.obsTypeList[nObsTypes_P2].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						if(pCodeEditedFlag[i]!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = obsPreprocInfo2EditedMark1(pCodeEditedFlag[i]);	
							it->second.obsTypeList[nObsTypes_P2].byEditedMark2 = obsPreprocInfo2EditedMark2(pCodeEditedFlag[i]);
						}
					}	
					it->second.ReservedField = 0.0;
					i++;
				}
				delete pEpochTime;
				delete pSlip;
				delete pEpochId;
				delete pLIF;
				delete pPhaseRes;
				delete pWL_NP;
				delete pAmb;
				delete pCodeEditedFlag;
			}
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				fclose(pFile);
			}
			datalist_sat2epoch(editedObsSatlist, editedObsEpochlist);
			// 输出到 editedObsFile 文件
			editedObsFile.m_header = m_obsFile.m_header;
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				// 钟差消除, 在每个历元, 遍历每颗GPS卫星的数据
				if(m_PreprocessorDefine.bOn_ClockEliminate)
				{
					for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
					{
						it->second.obsTypeList[nObsTypes_P1].obs.data -= editedObsFile.m_data[s_i].clock;
						it->second.obsTypeList[nObsTypes_P2].obs.data -= editedObsFile.m_data[s_i].clock;
						it->second.obsTypeList[nObsTypes_L1].obs.data -= editedObsFile.m_data[s_i].clock / WAVELENGTH_L1;
						it->second.obsTypeList[nObsTypes_L2].obs.data -= editedObsFile.m_data[s_i].clock / WAVELENGTH_L2;
					}
					editedObsFile.m_data[s_i].clock = 0;
				}
				editedObsFile.m_data[s_i].byEpochFlag = editedObsEpochlist[s_i].byEpochFlag;
				editedObsFile.m_data[s_i].editedObs   = editedObsEpochlist[s_i].editedObs;
				editedObsFile.m_data[s_i].bySatCount  = int(editedObsEpochlist[s_i].editedObs.size());	
			}
			editedObsFile.m_header.tmStart = editedObsEpochlist[0].t;           
			editedObsFile.m_header.tmEnd = editedObsEpochlist[editedObsEpochlist.size() - 1].t;
			DayTime T_Now;
			T_Now.Now();
			sprintf(editedObsFile.m_header.szFileDate, "%04d-%02d-%02d %02d:%02d:%02d",
				                                       T_Now.year,
													   T_Now.month,
													   T_Now.day,
											           T_Now.hour,
													   T_Now.minute,
													   int(T_Now.second));
			sprintf(editedObsFile.m_header.szProgramName, "%-20s", "NUDT Toolkit 1.0");
			sprintf(editedObsFile.m_header.szProgramAgencyName, "%-20s", "NUDT");
			editedObsFile.m_header.pstrCommentList.clear();
			char szComment[100];
			sprintf(szComment, "%-60s%20s\n", 
				               "created by LEO dual-frequence GPS edit program.", 
							   Rinex2_1_MaskString::szComment);
			editedObsFile.m_header.pstrCommentList.push_back(szComment);
			sprintf(editedObsFile.m_header.szFileType, "%-20s", "EDITED OBS");
			return true;
		}

		// 子程序名称： pdopMixedObsSPP   
		// 功能：计算混合原始观测数据的pdop
		// 变量类型：index_P1_GPS   :  GPS第一个频点伪距索引
		//			 index_P2_GPS   :  GPS第二个频点伪距索引
        //           index_P1_BDS   :  BDS第一个频点伪距索引
		//			 index_P2_BDS   :  BDS第二个频点伪距索引	
		//           obsEpoch       :  混合原始观测数据历元
		//           eyeableSatCount:  卫星总数
		//           pdop           :  pdop 值
		// 输入：index_P1_GPS,index_P2_GPS,index_P1_BDS, index_P2_BDS, obsEpoch 
		// 输出：eyeableSatCount,pdop
		// 其它：
		// 语言：C++
		// 版本号：2014/12/04
		// 生成者：谷德峰、刘俊宏
		// 修改者：
		// 备注：
		bool LeoGPSObsPreproc::pdopMixedObsSPP(int index_P1_GPS, int index_P2_GPS,int index_P1_BDS, int index_P2_BDS, Rinex2_1_LeoMixedEditedObsEpoch obsEpoch, int& eyeableSatCount, double& pdop)
		{
			pdop = 0;
			eyeableSatCount = 0;
            Rinex2_1_MixedEditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); 
			while(it != obsEpoch.editedObs.end())
			{
				Rinex2_1_EditedObsDatum P1,P2;
				if(it->first.find('G') != -1)
				{
					P1 = it->second.obsTypeList[index_P1_GPS];
					P2 = it->second.obsTypeList[index_P2_GPS];
				}
				else if(it->first.find('C') != -1)
				{
					P1 = it->second.obsTypeList[index_P1_BDS];
					P2 = it->second.obsTypeList[index_P2_BDS];
				}
				else
					continue;

				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || P2.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
				{
					Rinex2_1_MixedEditedObsSatMap::iterator jt = it;
					++it;
					obsEpoch.editedObs.erase(jt);
					continue;
				}
				else
				{
					eyeableSatCount++;
					++it;
					continue;
				}
			}
			if(eyeableSatCount < 4)  // 可见星要大于或等于4颗
				return false;
			POSCLK posclk;
			posclk.x = obsEpoch.pos.x;
			posclk.y = obsEpoch.pos.y;
			posclk.z = obsEpoch.pos.z;
			posclk.clk = obsEpoch.clock;
			Matrix matA(eyeableSatCount, 4); // 高斯牛顿迭代的线性化展开矩阵
			Matrix matG_inv(eyeableSatCount, eyeableSatCount); // 观测权矩阵
			GPST t_Receive = obsEpoch.t - posclk.clk / SPEED_LIGHT;
			// 双频 P 码消除电离层组合系数		
			int j = 0;
			for(Rinex2_1_MixedEditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{// 可以考虑高度角加权处理的影响
				double weight_P_IF = 1.0;
				matG_inv.SetElement(j, j, weight_P_IF);
				j++;
			}
			j = 0;
			for(Rinex2_1_MixedEditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{
				double delay = 0;
				SP3Datum sp3Datum;		
				m_sp3File.getEphemeris_PathDelay(obsEpoch.t, posclk, it->first, delay, sp3Datum);
				GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum); // 对GPS卫星星历进行地球自转改正
				GPST t_Transmit = t_Receive - delay;
				double distance;
				distance = pow(posclk.x - sp3Datum.pos.x, 2)
						 + pow(posclk.y - sp3Datum.pos.y, 2)
						 + pow(posclk.z - sp3Datum.pos.z, 2);
				distance = sqrt(distance);
				// 计算视线方向, 利用概略点
				matA.SetElement(j, 0, (posclk.x - sp3Datum.pos.x) / distance);
				matA.SetElement(j, 1, (posclk.y - sp3Datum.pos.y) / distance);
				matA.SetElement(j, 2, (posclk.z - sp3Datum.pos.z) / distance);
				matA.SetElement(j, 3,  1.0);
				j++;
			}
			Matrix matAA_inv = (matA.Transpose() * matG_inv * matA).Inv_Ssgj();
			pdop = sqrt(matAA_inv.GetElement(0,0) + matAA_inv.GetElement(1,1) + matAA_inv.GetElement(2,2));
			return true;
		}
		// 子程序名称： mainFuncMixedObsPreproc   
		// 功能：GNSS 混合原始观测数据文件预处理
		// 变量类型：strMixedObsFileName : 混合原始观测数据文件路径
		//			 mixedEditedObsFile  : 编辑后的观测数据文件	
		//           bOn_edit            : 是否采用数据编辑的方法对观测数据预处理，true:编辑结果,false:直接预处理
		// 输入：strMixedObsFileName   
		// 输出：mixedEditedObsFile
		// 其它：
		// 语言：C++
		// 版本号：2014/12/02
		// 生成者：谷德峰、刘俊宏
		// 修改者：
		// 备注：  
		bool LeoGPSObsPreproc::mainFuncMixedObsPreproc(string  strMixedObsFileName, Rinex2_1_LeoMixedEditedObsFile  &mixedEditedObsFile,bool bOn_edit)
		{
			// 提取根目录
			string folder = strMixedObsFileName.substr(0, strMixedObsFileName.find_last_of("\\"));
			string obsFileName = strMixedObsFileName.substr(strMixedObsFileName.find_last_of("\\") + 1);
			string obsFileName_noexp = obsFileName.substr(0, obsFileName.find_last_of("."));
			//char  gpsEditFileName[300],bdsEditFileName[300];//,mixedEditFileName[300];
			//sprintf(gpsEditFileName,"%s\\%s_GPS.edt",folder.c_str(),obsFileName_noexp.c_str());
			//sprintf(bdsEditFileName,"%s\\%s_BDS.edt",folder.c_str(),obsFileName_noexp.c_str());
			//sprintf(mixedEditFileName,"%s\\%s.edt",folder.c_str(),obsFileName_noexp.c_str());
			//step1:从混合系统的观测数据文件中提取单个系统的观测数据
			Rinex2_1_ObsFile gpsObsFile,bdsObsFile;
			if(!gpsObsFile.openMixedFile(strMixedObsFileName,'G'))
				printf("GPS观测数据打开失败!\n");
			if(!bdsObsFile.openMixedFile(strMixedObsFileName,'C'))
				printf("BDS观测数据打开失败!\n");
			if(gpsObsFile.m_data.size() < m_PreprocessorDefine.min_arcpointcount && bdsObsFile.m_data.size() < m_PreprocessorDefine.min_arcpointcount)
			{
				printf("可用观测数据不足!\n");
				return false;
			}
			//step2:单系统观测数据编辑/或预处理
			Rinex2_1_LeoEditedObsFile  gpsEditedObsFile,bdsEditedObsFile;
			m_obsFile = gpsObsFile;
			if(bOn_edit)
			{
				if(!mainFuncDFreqGPSObsEdit(gpsEditedObsFile))
					printf("GPS观测数据编辑失败!\n");
				//else	
				//	gpsEditedObsFile.write(gpsEditFileName);
				m_obsFile = bdsObsFile;
				if(!mainFuncDFreqGPSObsEdit(bdsEditedObsFile,m_PreprocessorDefine.typ_BDSobs_L1,m_PreprocessorDefine.typ_BDSobs_L2))
					printf("BDS观测数据编辑失败!\n");
				//else
				//	bdsEditedObsFile.write(bdsEditFileName);
			}
			else
			{
				if(!mainFuncDFreqGPSObsPreproc(gpsEditedObsFile))
					printf("GPS观测数据预处理失败!\n");	
				//else
				//	gpsEditedObsFile.write(gpsEditFileName);	
				m_PreprocessorDefine.bOn_RaimSPP = false;
				m_PreprocessorDefine.bOn_RaimArcChannelBias = false;
				m_obsFile = bdsObsFile;
				if(!mainFuncDFreqGPSObsPreproc(bdsEditedObsFile,m_PreprocessorDefine.typ_BDSobs_L1,m_PreprocessorDefine.typ_BDSobs_L2))
					printf("BDS观测数据预处理失败!\n");
				//else
				//	bdsEditedObsFile.write(bdsEditFileName);
			}
			//step3:将编辑后的单系统文件合并为mixedEditedObsFile
			if(gpsEditedObsFile.isEmpty() && bdsEditedObsFile.isEmpty())
				return false;
			GPST t_start,t_end;// 合并后的edit文件起止时间
			double interval = DBL_MAX;      // 采样间隔
			int nObsTypes_P1_GPS = -1, nObsTypes_P2_GPS = -1,nObsTypes_P1_BDS = -1, nObsTypes_P2_BDS = -1;
			int type_obs_P1_BDS  = TYPE_OBS_P1;
			int type_obs_P2_BDS  = TYPE_OBS_P2;						
			//if(m_PreprocessorDefine.typ_BDSobs_L1 == TYPE_OBS_L2)					
			//	type_obs_P1_BDS  = TYPE_OBS_P2;			
			//if(m_PreprocessorDefine.typ_BDSobs_L1 == TYPE_OBS_L5)						
			//	type_obs_P1_BDS  = TYPE_OBS_P5;			
			//if(m_PreprocessorDefine.typ_BDSobs_L2 == TYPE_OBS_L1)				
			//	type_obs_P2_BDS  = TYPE_OBS_P1;			
			//if(m_PreprocessorDefine.typ_BDSobs_L2 == TYPE_OBS_L5)			
			//	type_obs_P2_BDS  = TYPE_OBS_P5;	
			if(!gpsEditedObsFile.isEmpty())
			{
				t_start = gpsEditedObsFile.m_data.front().t;
				t_end   = gpsEditedObsFile.m_data.back().t;
				interval = gpsEditedObsFile.m_header.Interval;				
				for(int i = 0; i < gpsEditedObsFile.m_header.byObsTypes; i++)
				{					
					if(gpsEditedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)  //第一个频点伪距
						nObsTypes_P1_GPS = i;
					if(gpsEditedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)  //第二个频点伪距
						nObsTypes_P2_GPS = i;				
				}
				if(nObsTypes_P1_GPS == -1 || nObsTypes_P2_GPS == -1) 
					return false;			
			}				
			if(!bdsEditedObsFile.isEmpty())
			{
				if(interval == DBL_MAX)
				{
					t_start = bdsEditedObsFile.m_data.front().t;
					t_end   = bdsEditedObsFile.m_data.back().t;
					interval = bdsEditedObsFile.m_header.Interval;
				}
				else
				{
					if(t_start - bdsEditedObsFile.m_data.front().t > 0)
						t_start = bdsEditedObsFile.m_data.front().t;
					if(t_end  - bdsEditedObsFile.m_data.back().t < 0)
						t_end = bdsEditedObsFile.m_data.back().t;
				}	
				for(int i = 0; i < bdsEditedObsFile.m_header.byObsTypes; i++)
				{					
					if(bdsEditedObsFile.m_header.pbyObsTypeList[i] == type_obs_P1_BDS)  //第一个频点伪距
						nObsTypes_P1_BDS = i;
					if(bdsEditedObsFile.m_header.pbyObsTypeList[i] == type_obs_P2_BDS)  //第二个频点伪距
						nObsTypes_P2_BDS = i;				
				}
				if(nObsTypes_P1_BDS == -1 || nObsTypes_P2_BDS == -1) 
					return false;
			}
			int        i = 0;
			size_t   s_j = 0;
		    size_t   s_k = 0;
			int      nObsTime = 0;
			while(t_start + i * interval - t_end <= 0)
			{
				Rinex2_1_LeoMixedEditedObsEpoch  mixedEpoch;
				Rinex2_1_LeoEditedObsEpoch       gpsEpoch,bdsEpoch;
				gpsEpoch.editedObs.clear();
				bdsEpoch.editedObs.clear();
				GPST t_epoch = t_start + i * interval;
				//寻找当前时刻，gps历元
				for(size_t s_i = s_j; s_i < gpsEditedObsFile.m_data.size(); s_i ++)
				{
					if(fabs(gpsEditedObsFile.m_data[s_i].t - t_epoch) < 1.0e-5)
					{
						gpsEpoch = gpsEditedObsFile.m_data[s_i];
						s_j = s_i;
						break;
					}
					if(gpsEditedObsFile.m_data[s_i].t - t_epoch >= 1.0e-5)
					{
						s_j = s_i;
						break;
					}
				}
				//寻找当前时刻，bds历元
				for(size_t s_ii = s_k; s_ii < bdsEditedObsFile.m_data.size(); s_ii ++)
				{
					if(fabs(bdsEditedObsFile.m_data[s_ii].t - t_epoch) < 1.0e-5)
					{
						bdsEpoch = bdsEditedObsFile.m_data[s_ii];
						s_k = s_ii;
						break;
					}
					if(bdsEditedObsFile.m_data[s_ii].t - t_epoch >= 1.0e-5)
					{
						s_k = s_ii;
						break;
					}
				}
				if(gpsEpoch.editedObs.size() > 0)
				{
					mixedEpoch.t = t_epoch;
					mixedEpoch.byEpochFlag = gpsEpoch.byEpochFlag;
					mixedEpoch.bySatCount  = gpsEpoch.bySatCount;
					mixedEpoch.byRAIMFlag  = gpsEpoch.byRAIMFlag;
					mixedEpoch.pdop        = gpsEpoch.pdop;
					mixedEpoch.pos         = gpsEpoch.pos;
					mixedEpoch.vel         = gpsEpoch.vel;
					mixedEpoch.clock       = gpsEpoch.clock;
					for(Rinex2_1_EditedObsSatMap::iterator it = gpsEpoch.editedObs.begin();it != gpsEpoch.editedObs.end(); ++it)
					{
						char szSatName[4];
						sprintf(szSatName, "G%02d", it->first);
						szSatName[3] = '\0';
						Rinex2_1_MixedEditedObsLine  mixedLine;						
						mixedLine.satName       = szSatName;
						mixedLine.Azimuth       = it ->second.Azimuth;
						mixedLine.Elevation     = it->second.Elevation;
						mixedLine.ReservedField = it->second.ReservedField;
						mixedLine.obsTypeList   = it->second.obsTypeList;
						mixedEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(szSatName,mixedLine));						
					}
				}
				if(bdsEpoch.editedObs.size() > 0)
				{
					if(gpsEpoch.editedObs.size() > 0)
					{
						mixedEpoch.bySatCount += bdsEpoch.bySatCount;
						if(mixedEpoch.byRAIMFlag == 0)
							mixedEpoch.byRAIMFlag = bdsEpoch.byRAIMFlag;
					}
					else
					{
						mixedEpoch.t           = t_epoch;
						mixedEpoch.byEpochFlag = bdsEpoch.byEpochFlag;
						mixedEpoch.bySatCount  = bdsEpoch.bySatCount;
						mixedEpoch.byRAIMFlag  = bdsEpoch.byRAIMFlag;
						mixedEpoch.pdop        = bdsEpoch.pdop;
						mixedEpoch.pos         = bdsEpoch.pos;
						mixedEpoch.vel         = bdsEpoch.vel;
						mixedEpoch.clock       = bdsEpoch.clock;
					}
					for(Rinex2_1_EditedObsSatMap::iterator it = bdsEpoch.editedObs.begin();it != bdsEpoch.editedObs.end(); ++it)
					{
						char szSatName[4];
						sprintf(szSatName, "C%02d", it->first);
						szSatName[3] = '\0';
						Rinex2_1_MixedEditedObsLine  mixedLine;						
						mixedLine.satName       = szSatName;
						mixedLine.Azimuth       = it ->second.Azimuth;
						mixedLine.Elevation     = it->second.Elevation;
						mixedLine.ReservedField = it->second.ReservedField;
						mixedLine.obsTypeList   = it->second.obsTypeList;
						mixedEpoch.editedObs.insert(Rinex2_1_MixedEditedObsSatMap::value_type(szSatName,mixedLine));						
					}
				}
				// 确定nObsTime,pdop
				if(mixedEpoch.editedObs.size() > 0)
				{
					for(Rinex2_1_MixedEditedObsSatMap::iterator it = mixedEpoch.editedObs.begin();it != mixedEpoch.editedObs.end(); ++it)
					{						
						it->second.nObsTime = nObsTime;
					}	
					int eyeableSatCount = 0;
					mixedEpoch.pdop = 0;
					pdopMixedObsSPP(nObsTypes_P1_GPS, nObsTypes_P2_GPS, nObsTypes_P1_BDS, nObsTypes_P2_BDS, mixedEpoch, eyeableSatCount, mixedEpoch.pdop);					
					mixedEditedObsFile.m_data.push_back(mixedEpoch);
					nObsTime ++;
				}				
				i ++;
			}
			//step4:重新计算多系统观测数据的pdop值

			//step5:整理文件头信息
			if(!gpsEditedObsFile.isEmpty())			
				mixedEditedObsFile.m_header.init(gpsEditedObsFile.m_header);				
			if(!bdsEditedObsFile.isEmpty())
			{
				if(gpsEditedObsFile.isEmpty())				
					mixedEditedObsFile.m_header.init(bdsEditedObsFile.m_header);
				else
				{
					sprintf(mixedEditedObsFile.m_header.szSatlliteSystem, "M (MIXED)           ");
					mixedEditedObsFile.m_header.bySatCount += bdsEditedObsFile.m_header.bySatCount;
					for(size_t s_i = 0; s_i < bdsEditedObsFile.m_header.pbySatList.size(); s_i ++)
					{
						char szSatName[4];
						sprintf(szSatName, "C%02d", bdsEditedObsFile.m_header.pbySatList[s_i]);
						szSatName[3] = '\0';
						mixedEditedObsFile.m_header.pstrSatList.push_back(szSatName);
					}
				}
			}
			mixedEditedObsFile.m_header.tmStart = t_start;
			mixedEditedObsFile.m_header.tmEnd   = t_end;

			//mixedEditedObsFile.write(mixedEditFileName);
			return true;
		}

		// 子程序名称： detectPhaseSlip_L1   
		// 功能：针对单频数据, 完成周跳探测   
		// 变量类型： index_P1       : 观测类型P1索引
		//            index_L1       : 观测类型L1索引
		//            obsSat         : 某个卫星的观测数据时间序列
		// 输入：index_P1, index_L1, obsSat
		// 输出：obsSat
		// 语言：C++
		// 创建者：谷德峰, 刘俊宏
		// 创建时间：2016/03/11
		// 版本时间：2016/03/11
		// 修改记录：
		// 其它： 
		bool LeoGPSObsPreproc::detectPhaseSlip_L1(int index_P1, int index_L1, Rinex2_1_EditedObsSat& obsSat)
		{
			char info[200];
			size_t nCount = obsSat.editedObs.size();
			if(nCount <= m_PreprocessorDefine.min_arcpointcount)  // 观测个数太少, 直接丢弃
			{
				for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
				{	
					if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_COUNT);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_COUNT);
					}
				}
				return true;
			}
			// 伪码相位无几何距离组合
			double  *pP1_L1 = new double[nCount]; // 相位电离层组合
			double  *pEpochTime = new double[nCount];			
			int *pSlip = new int [nCount];
			Rinex2_1_EditedObsEpochMap::iterator it0 = obsSat.editedObs.begin();
			BDT t0 = it0->first;  
			int i = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
			{
				pEpochTime[i] = it->first - t0;				
				Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[index_P1];
				Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[index_L1];
				double dP1 = P1.obs.data;
				double dL1 = L1.obs.data * SPEED_LIGHT / GPS_FREQUENCE_L1;
				// 构造伪码相位无几何距离组合
				pP1_L1[i] = dP1 - dL1; 
				// 如果伪码已经标记为野值, 相位为正常点, 则将相应的相位也标记为野值
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)					
				{
					if(L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
					{
						it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_MW);
						it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_MW);						
					}
				}					
				// 接受先前伪码观测数据的野值判断结果,  补充相位野值
				if(P1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || L1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
					pSlip[i] = TYPE_EDITEDMARK_OUTLIER; 
				else
					pSlip[i] = LEOGPSOBSPREPROC_NORMAL;					
				i++;
			}
			FILE *pFile;
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				char szPhasePreprocFileName[300];
				sprintf(szPhasePreprocFileName,"%s\\preproc_P1L1.dat",m_strPreprocPath.c_str());
				pFile = fopen(szPhasePreprocFileName,"a+");
			}
			size_t k   = 0;
			size_t k_i = k;
			int arc_k  = 0;
			static int ArcCounts = 0;
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
				newArc:  // 本弧段[k, k_i]数据处理 
				{
					vector<size_t>   unknownPointlist;
					unknownPointlist.clear();
					for(size_t s_i = k; s_i <= k_i; s_i++)
					{
						// 未知数据标记
						if(pSlip[s_i] == LEOGPSOBSPREPROC_NORMAL)
							unknownPointlist.push_back(s_i); 
					}
					size_t nCount_points = unknownPointlist.size(); 
					// 弧段内数据个数太少					
					if(nCount_points <= int(m_PreprocessorDefine.min_arcpointcount))
					{
						for(size_t s_i = 0; s_i < nCount_points; s_i++)
						{
							if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
								pSlip[unknownPointlist[s_i]] = LEOGPSOBSPREPROC_OUTLIER_COUNT; // 弧段内数据个数太少标记为野值							
						}
					}
					else
					{   						
						// 第一步: 计算MW组合量历元差数据的方差
						// 构造MW组合量历元差数据
						ArcCounts++;
						double *pDP1_L1 = new double[nCount_points - 1];
						for(size_t s_i = 1; s_i < nCount_points; s_i++)
							pDP1_L1[s_i - 1] = pP1_L1[unknownPointlist[s_i]] - pP1_L1[unknownPointlist[s_i - 1]] ;
						double var = RobustStatRms(pDP1_L1, int(nCount_points - 1));
						delete pDP1_L1;
						// 第二步: 进行相位野值剔除					
						// 20071012 添加, 利用 threshold_slipsize_wm 对 threshold_outlier 的上界进行控制
						// 因为当周跳发生在序列中间附近时, var 可能会超界, 影响野值探测
						double threshold_outlier = min(5 * var, m_PreprocessorDefine.threshold_slipsize_P1_L1);
						// [1, nCount_points - 2]
						for(size_t s_i = 1; s_i < nCount_points - 1; s_i++)
						{
							if(fabs(pP1_L1[unknownPointlist[s_i]]     - pP1_L1[unknownPointlist[s_i-1]]) > threshold_outlier
							&& fabs(pP1_L1[unknownPointlist[s_i + 1]] - pP1_L1[unknownPointlist[s_i] ])  > threshold_outlier)
							{								
								if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
									pSlip[unknownPointlist[s_i]] = LEOGPSOBSPREPROC_OUTLIER_MW;								
							}							
						}
						// 首尾两点 0 和 nCount_points - 1
						if(pSlip[unknownPointlist[1]] != LEOGPSOBSPREPROC_NORMAL && pSlip[unknownPointlist[0]] != TYPE_EDITEDMARK_OUTLIER)
							pSlip[unknownPointlist[0]] = LEOGPSOBSPREPROC_OUTLIER_MW;
						if(pSlip[unknownPointlist[nCount_points - 2]] != LEOGPSOBSPREPROC_NORMAL)
						{
							if(pSlip[unknownPointlist[nCount_points - 1]] != TYPE_EDITEDMARK_OUTLIER)
								pSlip[unknownPointlist[nCount_points - 1]] = LEOGPSOBSPREPROC_OUTLIER_MW;
						}
						else
						{
							if((fabs(pP1_L1[unknownPointlist[nCount_points - 1]] - pP1_L1[unknownPointlist[nCount_points - 2] ])  > threshold_outlier)
								&& (pSlip[unknownPointlist[nCount_points - 1]] != TYPE_EDITEDMARK_OUTLIER))
								pSlip[unknownPointlist[nCount_points - 1]] = LEOGPSOBSPREPROC_OUTLIER_MW;
						}
						size_t s_i = 0;
						while(s_i < unknownPointlist.size())
						{
							if(pSlip[unknownPointlist[s_i]] == LEOGPSOBSPREPROC_NORMAL)
								s_i++;
							else
							{
								// 在进行周跳探测时, 先将野值 erase
								unknownPointlist.erase(unknownPointlist.begin() + s_i);
							}
						}
						nCount_points = unknownPointlist.size();
						// 第三步: 进行大周跳探测
						if(nCount_points <= 3)
						{
							// 个数太少则直接丢弃
							for(size_t s_i = 0; s_i < nCount_points; s_i++)
							{								
								if(pSlip[unknownPointlist[s_i]] != TYPE_EDITEDMARK_OUTLIER)
									pSlip[unknownPointlist[s_i]] = LEOGPSOBSPREPROC_OUTLIER_COUNT;								
							}
						}
						else
						{
							vector<size_t> slipindexlist;
							slipindexlist.clear();
							// [1, nCount_points - 2]
							double threshold_largeslip = m_PreprocessorDefine.threshold_slipsize_P1_L1;
							for(size_t s_i = 1; s_i < nCount_points - 1; s_i++)
							{
								// 大周跳发生, 每个大周跳在探测后, 其信息都被保存下来了
								if(fabs(pP1_L1[unknownPointlist[s_i]]     - pP1_L1[unknownPointlist[s_i - 1]]) >  threshold_largeslip
								&& fabs(pP1_L1[unknownPointlist[s_i + 1]] - pP1_L1[unknownPointlist[s_i] ])    <= threshold_largeslip) 
								{									
									size_t index = unknownPointlist[s_i];
									pSlip[index] = LEOGPSOBSPREPROC_SLIP_MW;
									//sprintf(info, "P1 - L1 发现大周跳发生 %10.2f", fabs(pP1_L1[unknownPointlist[s_i]]     - pP1_L1[unknownPointlist[s_i - 1]]));
									//printf("%s\n", info);
									//RuningInfoFile::Add(info);
								}
							}
						    // 无周跳弧段的内符合诊断, 主要针对 CHAMP 卫星, 其它卫星这种现象很少, 2008/11/11
							slipindexlist.clear();
							for(size_t s_i = 1; s_i < nCount_points; s_i++)
							{
								size_t index = unknownPointlist[s_i];
								if(pSlip[index] == LEOGPSOBSPREPROC_SLIP_MW)
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
									pSubsection_right[s_i]    = slipindexlist[s_i] -  1 ;
									pSubsection_left[s_i + 1] = slipindexlist[s_i] ;
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
									if(pSlip[s_j] != TYPE_EDITEDMARK_OUTLIER
									&& pSlip[s_j] != LEOGPSOBSPREPROC_OUTLIER_MW)
										subsectionNormalPointlist.push_back(s_j); 
								}
								size_t count_subsection = subsectionNormalPointlist.size(); 
								if(count_subsection > m_PreprocessorDefine.min_arcpointcount)
								{   
									double *pX = new double [count_subsection];
									double *pW = new double [count_subsection];
									double mean = 0;
									double var  = 0;
									for(size_t s_j = 0; s_j < count_subsection; s_j++)
										pX[s_j] = pP1_L1[subsectionNormalPointlist[s_j]];  
									RobustStatMean(pX, pW, int(count_subsection), mean, var, 5); 
									arc_k++;
									// 为了增加可靠性, 在每个无周跳的区间内增加该环节
									if(var > m_PreprocessorDefine.threshold_rms_P1_L1)
									{
										sprintf(info, "P1 - L1 无周跳区间标准差超差 var = %.2f/%.2f!(PRN%02d)", var, m_PreprocessorDefine.threshold_rms_P1_L1, obsSat.Id);
										printf("%s\n", info);
										//RuningInfoFile::Add(info);
										for(size_t s_j = 0; s_j < count_subsection; s_j++)
										{
											pSlip[subsectionNormalPointlist[s_j]] = LEOGPSOBSPREPROC_OUTLIER_MWRMS;
										}
									}
									/*else
									{
										sprintf(info,"P1 - L1 无周跳区间标准差正常 var = %.2f/%.2f!(PRN%02d)\n", var, m_PreprocessorDefine.threshold_rms_P1_L1, obsSat.Id);
									    RuningInfoFile::Add(info);
									}*/
									delete pX;
									delete pW;
								}
								else
								{
									//MW 组合序列弧段的正常点个数过少!直接标为野值
									for(size_t s_j = 0; s_j < count_subsection; s_j++)									
										pSlip[subsectionNormalPointlist[s_j]] = LEOGPSOBSPREPROC_OUTLIER_COUNT; 
								}
							}
							for(size_t s_i = k; s_i <= k_i; s_i++)
							{
								// 将第一个非野值点, 更新标记为周跳
								if(pSlip[s_i] == LEOGPSOBSPREPROC_NORMAL || pSlip[s_i] == LEOGPSOBSPREPROC_SLIP_MW)
								{
									pSlip[s_i] = LEOGPSOBSPREPROC_NEWARCBEGIN;
									break;
								}
							}
							delete pSubsection_left;
							delete pSubsection_right;
						}						
					}
					if(!m_strPreprocPath.empty())
					{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{
							if(pSlip[s_i] != TYPE_EDITEDMARK_OUTLIER)
							{
								fprintf(pFile,"%-30s %8.2f %8d %8d %18.3f %8d\n",
									(t0 + pEpochTime[s_i]).toString().c_str(),	
									pEpochTime[s_i],
									obsSat.Id,
									ArcCounts,
									pP1_L1[s_i],
									pSlip[s_i]);
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
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				fclose(pFile);
			}
			// 保存周跳标志
			i = 0;
			for(Rinex2_1_EditedObsEpochMap::iterator it = obsSat.editedObs.begin(); it != obsSat.editedObs.end(); ++it)
			{
				if(it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
				{
					it->second.obsTypeList[index_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[i]);
					it->second.obsTypeList[index_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[i]);
				}
				i++;
			}
			// 保留
			delete pP1_L1;
			delete pEpochTime;
			delete pSlip;		
			return true;
		}

		// 子程序名称： mainFuncSFreqGPSObsPreproc   
		// 功能：GPS 单频原始观测数据预处理
		// 变量类型：
		//			 editedObsFile  : 编辑后的观测数据文件		
		// 输入：  
		// 输出：editedObsFile
		// 其它：
		// 语言：C++
		// 版本号：2016/03/11
		// 生成者：谷德峰
		// 修改者：
		// 备注： 单频处理 threshold_res_raim 的阈值要调大 3.5 -> 15
		bool LeoGPSObsPreproc::mainFuncSFreqGPSObsPreproc(Rinex2_1_LeoEditedObsFile  &editedObsFile)
		{
			//char cSatSystem = m_obsFile.m_header.getSatSystemChar(); 
			char cSatSystem = 'G'; // 目前只处理GPS单频数据
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_P1 = -1;
			for(int i = 0; i < m_obsFile.m_header.byObsTypes; i++)
			{
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)  //第一个频点相位
					nObsTypes_L1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)  //第二个频点相位
					nObsTypes_P1 = i;
			}
			if(nObsTypes_L1 == -1 || nObsTypes_P1 == -1) 
				return false;

			vector<Rinex2_1_LeoEditedObsEpoch> editedObsEpochlist;
			editedObsEpochlist.clear();
			vector<Rinex2_1_EditedObsSat> editedObsSatlist;
			getEditedObsSatList(editedObsSatlist);
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{
					// L1
					if(it->second.obsTypeList[nObsTypes_L1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_L1].obs.data == 0)
					{
						it->second.obsTypeList[nObsTypes_L1].obs.data = 0; // 为防止 DBL_MAX 参与运算, 暂时赋值为 0
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
					}
					else
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					// P1
					if(it->second.obsTypeList[nObsTypes_P1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_P1].obs.data == 0)
					{
						it->second.obsTypeList[nObsTypes_P1].obs.data = 0; // 为防止 DBL_MAX 参与运算, 暂时赋值为 0
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_BLANKZERO);
					}
					else
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
				}
			}

			datalist_sat2epoch(editedObsSatlist, editedObsEpochlist);

			// 计算概略轨道点
			double  threshold_coarse_orbitinterval = 180;               // 计算概略点的参考时间间隔, 默认180s
			double  cumulate_time = threshold_coarse_orbitinterval * 2; // 累积时间,从第1点开始计算
			vector<int> validindexlist;                                 // 有效索引号列表
			vector<TimePosVel>  coarseorbitlist;                        // 形成概略轨道列表 
			vector<int> locationPointlist;                              // 停靠点列表
			locationPointlist.resize(editedObsEpochlist.size());
			double *pTime = new double [editedObsEpochlist.size()];
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{// 星历检查, 先校验一下每颗 GPS 卫星的星历数据是否完整, 如果此时刻星历数据不完整, 则清除此时刻的该颗 GPS 数据, 2007/08/17
				pTime[s_i] = editedObsEpochlist[s_i].t - editedObsEpochlist[0].t;
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					int nPRN = it->first; 
					SP3Datum sp3Datum;
					CLKDatum ASDatum;
					// 左右各延伸一个点, 判断该点的星历数据是否完整
					if(!m_clkFile.getSatClock(editedObsEpochlist[s_i].t, nPRN, ASDatum, 3 + 2,cSatSystem) || !m_sp3File.getEphemeris(editedObsEpochlist[s_i].t, nPRN, sp3Datum, 9 + 2,cSatSystem))
					{
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_EPHEMERIS);
					}
				}
				if(s_i > 0)
					cumulate_time += editedObsEpochlist[s_i].t - editedObsEpochlist[s_i - 1].t;
				// 当累积时间满足条件，才开始计算概略点，以达到降低运算量的目的
				// 同时为保证插值精度，起始点和结束点的概略轨道要计算
				
				if(cumulate_time >= threshold_coarse_orbitinterval || s_i == editedObsEpochlist.size() - 1)
				{
					double raim_pdop = 0;
					double raim_rms_res = 0;
					POSCLK raim_posclk;
					// 此处 P2 用 P1 代替
					int raim_flag = RaimSPP_PIF(nObsTypes_P1, nObsTypes_P1, GPS_FREQUENCE_L1, GPS_FREQUENCE_L2, editedObsEpochlist[s_i], raim_posclk, raim_pdop, raim_rms_res);
					if(raim_flag == 2)
					{// 结果可靠
						cumulate_time = 0;
						editedObsEpochlist[s_i].pos = raim_posclk.getPos();
					    editedObsEpochlist[s_i].clock = raim_posclk.clk;
					    editedObsEpochlist[s_i].byRAIMFlag = raim_flag;
					    editedObsEpochlist[s_i].pdop = raim_pdop;
						validindexlist.push_back(int(s_i));
						TimePosVel coarseorbit;
						coarseorbit.t = editedObsEpochlist[s_i].t;
						coarseorbit.pos = editedObsEpochlist[s_i].pos;
						coarseorbitlist.push_back(coarseorbit);
					}
				}

				// 每个单点将归属于一个有效点, 原则是左归属
				if(validindexlist.size() > 0)
					locationPointlist[s_i] = int(validindexlist.size() - 1);
				else
					locationPointlist[s_i] = 0;
			}

			// 微分平滑求速(10 阶 Lagrange 插值)
			const int nlagrangePoint_left  = 5;   
			const int nlagrangePoint_right = 5;
			int   nlagrangePoint = nlagrangePoint_left + nlagrangePoint_right; 
			size_t validindexcount = coarseorbitlist.size();
			if(validindexcount < size_t(nlagrangePoint)) // 插值点个数太少条件
				return false;
			else
			{   
				double *xa_t = new double [nlagrangePoint];
				double *ya_X = new double [nlagrangePoint];
				double *ya_Y = new double [nlagrangePoint];
				double *ya_Z = new double [nlagrangePoint];
				for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
				{
					// 通过位置插值匹配输出速度
					int k_now = locationPointlist[s_i];
					if(k_now <= nlagrangePoint_left - 1)
						k_now = 0;
					else if(k_now >= int(validindexcount - nlagrangePoint_right))
						k_now = int(validindexcount - nlagrangePoint_right - nlagrangePoint_left);
					else
						k_now = k_now - nlagrangePoint_left + 1;
					// 插值区间[k_now, k_now + nlagrangePoint_left + nlagrangePoint_right - 1]
					for(size_t s_j = k_now; s_j <= size_t(k_now + nlagrangePoint_left + nlagrangePoint_right - 1); s_j++)
					{
						xa_t[s_j - k_now] = pTime[validindexlist[s_j]];
						ya_X[s_j - k_now] = coarseorbitlist[s_j].pos.x;
						ya_Y[s_j - k_now] = coarseorbitlist[s_j].pos.y;
						ya_Z[s_j - k_now] = coarseorbitlist[s_j].pos.z;
					}
					InterploationLagrange(xa_t, ya_X, nlagrangePoint, pTime[s_i], editedObsEpochlist[s_i].pos.x, editedObsEpochlist[s_i].vel.x);
					InterploationLagrange(xa_t, ya_Y, nlagrangePoint, pTime[s_i], editedObsEpochlist[s_i].pos.y, editedObsEpochlist[s_i].vel.y);
					InterploationLagrange(xa_t, ya_Z, nlagrangePoint, pTime[s_i], editedObsEpochlist[s_i].pos.z, editedObsEpochlist[s_i].vel.z);
			        // 钟差采用线性插值
					double x_t[2];
					double y_t[2];
					k_now = locationPointlist[s_i];
					if( k_now <= -1)
						k_now = 0;
					else if(k_now >= int(validindexcount - 1))
						k_now = int(validindexcount - 2);
					else
						k_now = k_now;
					// 插值区间 [ k_now, k_now + 1 ]
					x_t[0] = pTime[validindexlist[k_now]];
					x_t[1] = pTime[validindexlist[k_now + 1]];
					y_t[0] = editedObsEpochlist[validindexlist[k_now]].clock;
					y_t[1] = editedObsEpochlist[validindexlist[k_now + 1]].clock;
					double u = (pTime[s_i] - x_t[0])/(x_t[1] - x_t[0]);
					editedObsEpochlist[s_i].clock = u * y_t[1] +(1 - u) * y_t[0];   // 钟差插值 u * y_t[0] +(1 - u) * y_t[1] 反了, 已修改 (20070917)
					if(editedObsEpochlist[s_i].byRAIMFlag != 1)
						editedObsEpochlist[s_i].byRAIMFlag = 0;
					// 计算GPS卫星的天空视图
					// 根据位置、速度计算天线坐标系
					POS3D S_Z; // Z轴指向卫星
					POS3D S_X; // X轴沿速度方向
					POS3D S_Y; // 右手系
					POS3D S_R, S_T, S_N;
					POS6D posvel_i;
					posvel_i.setPos(editedObsEpochlist[s_i].pos);
					posvel_i.setVel(editedObsEpochlist[s_i].vel);
					TimeCoordConvert::getCoordinateRTNAxisVector(editedObsEpochlist[s_i].t, posvel_i, S_R, S_T, S_N);
					S_X = S_T * (1.0);
				    S_Y = S_N * (1.0);
					S_Z = S_R * (1.0);
					for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
					{
						SP3Datum sp3Datum;
						if(m_sp3File.getEphemeris(editedObsEpochlist[s_i].t, it->first, sp3Datum, 9, cSatSystem))
						{
							POS3D vecLosECEF = sp3Datum.pos - editedObsEpochlist[s_i].pos; // 视线矢量: 接收机位置指向GPS卫星
							POS3D vecLosXYZ;
							vecLosXYZ.x = vectorDot(vecLosECEF, S_X);
							vecLosXYZ.y = vectorDot(vecLosECEF, S_Y);
							vecLosXYZ.z = vectorDot(vecLosECEF, S_Z);
							vecLosXYZ = vectorNormal(vecLosXYZ);
							it->second.Elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;
							it->second.Azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
							if(it->second.Azimuth < 0)
							{// 变换到[0, 360]
								it->second.Azimuth += 360.0;
							}
						}
						else
						{
							it->second.Elevation = 0.0;
							it->second.Azimuth = 0.0;
						}
					}
				}
				delete xa_t;
				delete ya_X;
				delete ya_Y;
				delete ya_Z;
			}
			delete pTime;

			// 剔除仰角过低弧段数据
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					if(it->second.Elevation <= m_PreprocessorDefine.min_elevation || it->second.Elevation == DBL_MAX)
					{
						if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
						}
						if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSPREPROC_OUTLIER_SNRELEVATION);
						}
					}	
				}
			}

			// 多通道 RAIM 检验, 识别采样系统误差
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{	
				if(editedObsEpochlist[s_i].byRAIMFlag == 1) // 跳过已经RAIM检验过的点
					continue;
				double raim_pdop = 0;
				double raim_rms_res = 0;
				POSCLK raim_posclk;
				raim_posclk.x = editedObsEpochlist[s_i].pos.x;
				raim_posclk.y = editedObsEpochlist[s_i].pos.y;
				raim_posclk.z = editedObsEpochlist[s_i].pos.z;
				raim_posclk.clk = editedObsEpochlist[s_i].clock;
				int raim_flag = RaimSPP_PIF(nObsTypes_P1, nObsTypes_P1, GPS_FREQUENCE_L1, GPS_FREQUENCE_L2, editedObsEpochlist[s_i], raim_posclk, raim_pdop, raim_rms_res);
				if(raim_flag)
				{// 更新概略位置
					editedObsEpochlist[s_i].pos = raim_posclk.getPos();
					editedObsEpochlist[s_i].clock = raim_posclk.clk;
					editedObsEpochlist[s_i].byRAIMFlag = raim_flag;
					editedObsEpochlist[s_i].pdop = raim_pdop;
				}
				else
				{
					editedObsEpochlist[s_i].byRAIMFlag = 0;
					editedObsEpochlist[s_i].pdop = 0;
				}
			}
			// 保存轨道钟差解算结果
			editedObsFile.m_data.resize(editedObsEpochlist.size());
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				editedObsFile.m_data[s_i].t = editedObsEpochlist[s_i].t;
				editedObsFile.m_data[s_i].byEpochFlag = editedObsEpochlist[s_i].byEpochFlag;
				editedObsFile.m_data[s_i].byRAIMFlag = editedObsEpochlist[s_i].byRAIMFlag;
				editedObsFile.m_data[s_i].pdop = editedObsEpochlist[s_i].pdop;
				editedObsFile.m_data[s_i].pos= editedObsEpochlist[s_i].pos;
				editedObsFile.m_data[s_i].vel = editedObsEpochlist[s_i].vel;
				editedObsFile.m_data[s_i].clock = editedObsEpochlist[s_i].clock;
			}

			// 弧段 RAIM 整体检验
			if(m_PreprocessorDefine.bOn_RaimArcChannelBias)
				detectRaimArcChannelBias_PIF(nObsTypes_P1, nObsTypes_P1, editedObsEpochlist);

			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				char szPhasePreprocFileName[300];
				sprintf(szPhasePreprocFileName,"%s\\preproc_P1L1.dat",m_strPreprocPath.c_str());
				FILE *pFile = fopen(szPhasePreprocFileName,"w+");
				fprintf(pFile, "%-30s %8s %8s %8s %18s %8s\n",
					        "Epoch",
							"T",
							"PRN",
							"Arc",
							"P1-L1",
							"Marks");
				fclose(pFile);
			}
			// 周跳探测
			datalist_epoch2sat(editedObsEpochlist, editedObsSatlist);
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
				detectPhaseSlip_L1(nObsTypes_P1, nObsTypes_L1, editedObsSatlist[s_i]);
            datalist_sat2epoch(editedObsSatlist, editedObsEpochlist);

			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				editedObsFile.m_data[s_i].bySatCount = editedObsEpochlist[s_i].bySatCount;
				editedObsFile.m_data[s_i].editedObs  = editedObsEpochlist[s_i].editedObs;
			}

			// 进行接收机的天线偏移改正
			for(size_t s_i = 0; s_i < editedObsFile.m_data.size(); s_i++)
			{
				POS3D posLeo = editedObsFile.m_data[s_i].pos;
				POS3D velLeo = editedObsFile.m_data[s_i].vel;
				POS3D correctOffset = GNSSBasicCorrectFunc::correctLeoAntPCO_ECEF(editedObsEpochlist[s_i].t, m_pcoAnt, posLeo, velLeo);
                editedObsFile.m_data[s_i].pos = editedObsFile.m_data[s_i].pos - correctOffset;
			}
			// 输出到 editedObsFile 文件
			editedObsFile.m_header = m_obsFile.m_header;
			editedObsFile.m_header.tmStart = editedObsEpochlist[0].t;           
			editedObsFile.m_header.tmEnd = editedObsEpochlist[editedObsEpochlist.size() - 1].t;
			DayTime T_Now;
			T_Now.Now();
			sprintf(editedObsFile.m_header.szFileDate, "%04d-%02d-%02d %02d:%02d:%02d",
				                                       T_Now.year,
													   T_Now.month,
													   T_Now.day,
											           T_Now.hour,
													   T_Now.minute,
													   int(T_Now.second));
			sprintf(editedObsFile.m_header.szProgramName, "%-20s", "NUDT Toolkit 2.0");
			sprintf(editedObsFile.m_header.szProgramAgencyName, "%-20s", "NUDT");
			editedObsFile.m_header.pstrCommentList.clear();
			char szComment[100];
			sprintf(szComment, "%-60s%20s\n", 
				               "created by LEO single-frequence GPS preprocess program.", 
							   Rinex2_1_MaskString::szComment);
			editedObsFile.m_header.pstrCommentList.push_back(szComment);
			sprintf(editedObsFile.m_header.szFileType, "%-20s", "PREPROC OBS");
			return true;
		}

		// 子程序名称： mainFuncSFreqGPSObsEdit   
		// 功能：GPS 单频原始观测数据文件编辑
		// 变量类型：editedObsFile   : 编辑后的单系统观测数据文件
		// 输入：editedObsFile
		// 输出：editedObsFile
		// 其它：
		// 语言：C++
		// 版本号：2016/9/18
		// 生成者：谷德峰、刘俊宏
		// 修改者：
		// 备注：threshold_editrms_code(电离层影响) 和 threshold_editrms_phase(伪码噪声) 需要对应调大
		bool LeoGPSObsPreproc::mainFuncSFreqGPSObsEdit(Rinex2_1_LeoEditedObsFile  &editedObsFile)
		{
			if(m_obsFile.isEmpty())
			{
				printf("无观测数据, 请确认!\n");
				return  false;				
			}
			char cSatSystem = 'G'; // 目前只处理GPS单频数据
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_P1 = -1;
			for(int i = 0; i < m_obsFile.m_header.byObsTypes; i++)
			{
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)  //第一个频点相位
					nObsTypes_L1 = i;
				if(m_obsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)  //第二个频点相位
					nObsTypes_P1 = i;
			}
			if(nObsTypes_L1 == -1 || nObsTypes_P1 == -1) 
				return false;
			vector<Rinex2_1_LeoEditedObsEpoch> editedObsEpochlist;
			getEditedObsEpochList(editedObsEpochlist);              
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					// L1
					if(it->second.obsTypeList[nObsTypes_L1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_L1].obs.data == 0)
					{
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					}
					else
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					// P1
					if(it->second.obsTypeList[nObsTypes_P1].obs.data == DBL_MAX || it->second.obsTypeList[nObsTypes_P1].obs.data == 0)
					{
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
						it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_BLANKZERO);
					}
					else
						it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					// 判断星历是否完整
					int nPRN = it->first; 
					SP3Datum sp3Datum;
					CLKDatum ASDatum;
					// 左右各延伸一个点, 判断该点的星历数据是否完整
					if(!m_clkFile.getSatClock(editedObsEpochlist[s_i].t, nPRN, ASDatum, 3 + 2, cSatSystem) || !m_sp3File.getEphemeris(editedObsEpochlist[s_i].t, nPRN, sp3Datum, 9 + 2, cSatSystem))
					{
						if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
						}
						if( it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
						}
					}
				}
			}
			double rms_residual_code = 0; // 伪码编辑残差均方根   
			size_t count_normalpoints_all = 0; // 正常伪码观测点的个数
            vector<int> validindexlist;
			validindexlist.resize(editedObsEpochlist.size());
			editedObsFile.m_data.resize(editedObsEpochlist.size());
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				validindexlist[s_i] = -1;
				editedObsFile.m_data[s_i].t = editedObsEpochlist[s_i].t;
				editedObsFile.m_data[s_i].clock = 0.0; // 钟差初始化为 0
				editedObsFile.m_data[s_i].byRAIMFlag = 2;
				// 钟差要迭代进行计算
				int k = 0;
				while(k <= 1)
				{
					// 更新真实观测时刻
					GPST t_Receive;
					if(m_PreprocessorDefine.bOn_ClockEliminate)
						t_Receive = editedObsEpochlist[s_i].t; // 可认为经704软校正的采样时刻是较为准确的, 2010/06/12
					else
						t_Receive = editedObsEpochlist[s_i].t - editedObsEpochlist[s_i].clock / SPEED_LIGHT;
					TimePosVel orbit_t;
                    if(!getLeoOrbitPosVel(t_Receive, orbit_t))
					{
						editedObsFile.m_data[s_i].clock = 0;
						editedObsFile.m_data[s_i].byRAIMFlag = 0;
						// 该时刻的数据为无效时刻, 丢弃该时刻的数据
						for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
						{
							if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
								it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							}
							if( it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
								it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_EPHEMERIS);
							}
						}
						//printf("%6d时刻 %s 数据无效(getLeoOrbitPosVel)!\n", s_i, editedObsFile.m_data[s_i].t.toString().c_str());
					}
					editedObsFile.m_data[s_i].pos  = orbit_t.pos;
					editedObsFile.m_data[s_i].vel  = orbit_t.vel;
					POSCLK posclk;
					posclk.x = orbit_t.pos.x;
					posclk.y = orbit_t.pos.y;
					posclk.z = orbit_t.pos.z;
					posclk.clk = editedObsFile.m_data[s_i].clock;
					if(m_PreprocessorDefine.bOn_ClockEliminate)
						posclk.clk = 0.0; // 可认为经704软校正的采样时刻是较为准确的, 2015/05/05
					// 根据观测时刻, 更新 GPS 卫星轨道位置和速度, 以及 GPS 卫星钟差
					// 记录y, 几何距离, gps卫星钟差
					size_t count_gpssat = editedObsEpochlist[s_i].editedObs.size();
					Matrix matP1(int(count_gpssat), 1);
					Matrix matR(int(count_gpssat), 1);
					Matrix matW(int(count_gpssat), 1);
					int count_normalpoints = 0;
					int j = 0;
					for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
					{
						int nPRN = it->first;
						Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[nObsTypes_P1];
						// 单频 P 码
						double y = P1.obs.data;
                        // 迭代计算卫星信号传播时间
				        double delay = 0;
						SP3Datum sp3Datum;
						char szSatName[4];
						sprintf(szSatName, "%1c%02d", cSatSystem, nPRN);
						szSatName[3] = '\0';
						m_sp3File.getEphemeris_PathDelay(editedObsEpochlist[s_i].t, posclk, szSatName, delay, sp3Datum);
						// 对GPS卫星星历进行地球自转改正
						GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
						// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
						GPST t_Transmit = t_Receive - delay;
						double distance;
						distance = pow(posclk.x - sp3Datum.pos.x, 2)
								 + pow(posclk.y - sp3Datum.pos.y, 2)
								 + pow(posclk.z - sp3Datum.pos.z, 2);
						distance = sqrt(distance);
						POS3D E; // 记录概略展开系数
						E.x = (posclk.x - sp3Datum.pos.x) / distance;
						E.y = (posclk.y - sp3Datum.pos.y) / distance;
						E.z = (posclk.z - sp3Datum.pos.z) / distance;
						// 对观测值 y 进行误差修正
						// 包括: GPS卫星钟差改正, GPS卫星相对论修正等
						// 1.GPS卫星钟差改正
						CLKDatum ASDatum;
						m_clkFile.getSatClock(t_Transmit, nPRN, ASDatum, 3, cSatSystem); // 获得 GPS 信号发射时间的卫星钟差改正
						double correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT;  // 等效距离
						y = y + correct_gpsclk;
						// 2.GPS卫星相对论改正
						double correct_relativity = ( sp3Datum.pos.x * sp3Datum.vel.x 
													+ sp3Datum.pos.y * sp3Datum.vel.y
													+ sp3Datum.pos.z * sp3Datum.vel.z ) * (-2.0) / SPEED_LIGHT;
						y = y + correct_relativity;
						// 3. 接收机位置偏心改正
						POS3D posLeo = editedObsFile.m_data[s_i].pos;
						POS3D velLeo = editedObsFile.m_data[s_i].vel;
						POS3D correctOffset = GNSSBasicCorrectFunc::correctLeoAntPCO_ECEF(editedObsEpochlist[s_i].t, m_pcoAnt, posLeo, velLeo);
						double correct_LeoAntOffset = -(correctOffset.x * E.x + correctOffset.y * E.y + correctOffset.z * E.z);
						y = y + correct_LeoAntOffset;						
						matP1.SetElement(j, 0, y);
						matR.SetElement(j, 0, distance);
						if(it->second.obsTypeList[nObsTypes_P1].byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						{
							matW.SetElement(j, 0, 1.0);
							count_normalpoints++;
						}
						else
							matW.SetElement(j, 0, 0.0);
						j++;
					}
					Matrix matRes(int(count_gpssat), 1); // 计算残差
					double max_res      =  0;
					int    max_index    = -1;
					double recerver_clk =  0;
					// 迭代剔除伪码野值
					while(count_normalpoints >= m_PreprocessorDefine.threshold_gpssatcount)
					{
						// 计算接收机钟差及残差
						recerver_clk = 0;
						for(int i = 0; i < int(count_gpssat); i++)
						{
							if(matW.GetElement(i, 0) != 0)
								recerver_clk += matP1.GetElement(i, 0) - matR.GetElement(i, 0);
						}
						recerver_clk = recerver_clk / count_normalpoints;
						// 寻找残差最大的点
						max_res =  0;
					    max_index = -1;
						for(int i = 0; i < int(count_gpssat); i++)
						{
							if(matW.GetElement(i, 0) != 0)
							{
								double res_i = matP1.GetElement(i, 0) - matR.GetElement(i, 0) - recerver_clk;
								matRes.SetElement(i, 0, res_i);
								if(fabs(res_i) > max_res)
								{
									max_index = i;
									max_res = fabs(res_i);
								}
							}
						}
						if(max_res <= m_PreprocessorDefine.threshold_editrms_code)
						{
							break;
						}
						else
						{// 调整权矩阵 matW 
							//char info[200];
							//sprintf(info, "%s %02d %14.2lf %10.4lf", editedObsEpochlist[s_i].t.toString().c_str(), max_index + 1, recerver_clk, max_res);
							//RuningInfoFile::Add(info);
							matW.SetElement(max_index, 0, 0.0);
							count_normalpoints = 0;
							for(int i = 0; i < int(count_gpssat); i++)
							{
								if(matW.GetElement(i, 0) != 0)
									count_normalpoints++;
							}
							continue;
						}
					}
					if(k == 0 && count_normalpoints >= m_PreprocessorDefine.threshold_gpssatcount)
					{// 第0次迭代主要目的为了更新钟差
						editedObsFile.m_data[s_i].clock = recerver_clk;
						k++;
						continue;
					}
					else if(k == 1 && count_normalpoints >= m_PreprocessorDefine.threshold_gpssatcount)
					{// 第1次迭代主要目的为了编辑伪码野值
						validindexlist[s_i] = 1;
						editedObsFile.m_data[s_i].clock = recerver_clk;
                        // 标记伪码的野值
						int j = 0;
						for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
						{
							if(matW.GetElement(j, 0) == 0)
							{
								if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
								{
									it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_PIF);
									it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_PIF);
								}
							}
							else
							{// 记录所有正常点的残差
								count_normalpoints_all++;
								rms_residual_code += pow(matRes.GetElement(j, 0), 2);
							}
							j++;
						}
						break;
					}
					else
					{// 情形1: k == 0 && count_normalpoints < m_PreprocessorDefine.threshold_gpssatcount, 比如缺LEO星历或可视卫星个数偏少
					 // 情形2: k == 1 && count_normalpoints < m_PreprocessorDefine.threshold_gpssatcount, 比如可视卫星个数偏少 
						validindexlist[s_i] = -1;
						editedObsFile.m_data[s_i].clock = 0;
						for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
						{// 该时刻的数据为无效时刻, 丢弃该时刻的数据
							if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
								it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
							}
							if( it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
							{
								it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
								it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_GPSSATCOUNT);
							}
						}
                        break;
					}
				}
			}
			rms_residual_code = sqrt(rms_residual_code / count_normalpoints_all);
			/*char info[200];
			sprintf(info, "伪码P1编辑后残差: %10.4lf.", rms_residual_code);
			RuningInfoFile::Add(info);*/

			// 利用保留位记录每颗卫星的保留值: 观测数据的修正值 - R(概略距离)
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				if(validindexlist[s_i] == -1)
					continue;
				// 更新真实观测时刻
				GPST t_Receive;
				if(m_PreprocessorDefine.bOn_ClockEliminate)
					t_Receive = editedObsEpochlist[s_i].t; // 可认为经704软校正的采样时刻是较为准确的, 2010/06/12
				else
					t_Receive = editedObsEpochlist[s_i].t - editedObsFile.m_data[s_i].clock / SPEED_LIGHT;
				TimePosVel orbit_t;
                if(!getLeoOrbitPosVel(t_Receive, orbit_t))
					continue;
				editedObsFile.m_data[s_i].pos  = orbit_t.pos;
				editedObsFile.m_data[s_i].vel  = orbit_t.vel;
				POSCLK posclk;
				posclk.x = orbit_t.pos.x;
				posclk.y = orbit_t.pos.y;
				posclk.z = orbit_t.pos.z;
				posclk.clk = editedObsFile.m_data[s_i].clock;
				if(m_PreprocessorDefine.bOn_ClockEliminate)
					posclk.clk = 0.0; // 可认为经704软校正的采样时刻是较为准确的, 2015/05/05
				// 计算GPS卫星的天空视图
				POS3D S_Z; // Z轴指向卫星
				POS3D S_X; // X轴沿速度方向
				POS3D S_Y; // 右手系
				POS3D S_R, S_T, S_N;
				POS6D posvel_i;
				posvel_i.setPos(editedObsFile.m_data[s_i].pos);
				posvel_i.setVel(editedObsFile.m_data[s_i].vel);
				TimeCoordConvert::getCoordinateRTNAxisVector(editedObsEpochlist[s_i].t, posvel_i, S_R, S_T, S_N);
                S_X = S_T * (1.0);
			    S_Y = S_N * (1.0);
				S_Z = S_R * (1.0);
				// 根据观测时刻, 更新 GPS 卫星轨道位置和速度, 以及 GPS 卫星钟差
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					int nPRN = it->first;
			        double delay = 0;
					SP3Datum sp3Datum;
					char szSatName[4];
					sprintf(szSatName, "%1c%02d", cSatSystem, nPRN);
					szSatName[3] = '\0';
					if(!m_sp3File.getEphemeris_PathDelay(editedObsEpochlist[s_i].t, posclk, szSatName, delay, sp3Datum))
						continue;
					// 对 GPS 卫星星历进行地球自转改正
					GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
					// 计算天空视图
					POS3D vecLosECEF = sp3Datum.pos - posclk.getPos(); // 视线矢量: 接收机位置指向GPS卫星
					POS3D vecLosXYZ;
					vecLosXYZ.x = vectorDot(vecLosECEF, S_X);
					vecLosXYZ.y = vectorDot(vecLosECEF, S_Y);
					vecLosXYZ.z = vectorDot(vecLosECEF, S_Z);
					vecLosXYZ = vectorNormal(vecLosXYZ);
					it->second.Elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;
					it->second.Azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
					if(it->second.Azimuth < 0)
					{// 变换到[0, 360]
						it->second.Azimuth += 360.0;
					}
					// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
					GPST t_Transmit = t_Receive - delay;
					double distance;
					distance = pow(posclk.x - sp3Datum.pos.x, 2)
							 + pow(posclk.y - sp3Datum.pos.y, 2)
							 + pow(posclk.z - sp3Datum.pos.z, 2);
					distance = sqrt(distance);
					POS3D E; // 记录概略展开系数
					E.x = (posclk.x - sp3Datum.pos.x) / distance;
					E.y = (posclk.y - sp3Datum.pos.y) / distance;
					E.z = (posclk.z - sp3Datum.pos.z) / distance;
					// 对观测值 y 进行误差修正
					// 包括: GPS卫星钟差改正, GPS卫星相对论修正等
					// 1.GPS卫星钟差改正
					CLKDatum ASDatum;
					if(!m_clkFile.getSatClock(t_Transmit, nPRN, ASDatum, 3, cSatSystem))
						continue;
					double correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT;  // 等效距离
					// 2.GPS卫星相对论改正
					double correct_relativity = ( sp3Datum.pos.x * sp3Datum.vel.x 
												+ sp3Datum.pos.y * sp3Datum.vel.y
												+ sp3Datum.pos.z * sp3Datum.vel.z ) * (-2.0) / SPEED_LIGHT;
					// 3.接收机位置偏心改正
					POS3D posLeo = editedObsFile.m_data[s_i].pos;
					POS3D velLeo = editedObsFile.m_data[s_i].vel;
					POS3D correctOffset = GNSSBasicCorrectFunc::correctLeoAntPCO_ECEF(editedObsEpochlist[s_i].t, m_pcoAnt, posLeo, velLeo);
					double correct_LeoAntOffset = -(correctOffset.x * E.x + correctOffset.y * E.y + correctOffset.z * E.z);
					it->second.ReservedField = correct_gpsclk + correct_relativity + correct_LeoAntOffset - distance;
				}
			}
			// 根据观测仰角剔除野值
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					if(it->second.Elevation <= m_PreprocessorDefine.min_elevation || it->second.Elevation == DBL_MAX)
					{
						if( it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
						}
						if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
							it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(LEOGPSOBSEDIT_OUTLIER_SNRELEVATION);
						}
					}	
				}
			}
			// 计算 pdop, 为了提高效率也可以提前, 可以直接利用前面的 E 进行计算
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				int eyeableGPSCount = 0;
				editedObsFile.m_data[s_i].pdop = 0;
				editedObsEpochlist[s_i].pos = editedObsFile.m_data[s_i].pos;
				pdopSPP(nObsTypes_P1, nObsTypes_P1, editedObsEpochlist[s_i], eyeableGPSCount, editedObsFile.m_data[s_i].pdop); // 单频处理将nObsTypes_P2赋值成nObsTypes_P1
			}
			// 进行钟差归属计算
			double clock_first =  0;
			int    i_first = -1;
			double clock_now =  0;
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				if(validindexlist[s_i] == 1)
				{// 更新当前的钟差数据
					clock_now = editedObsFile.m_data[s_i].clock;
					if(i_first < 0)
					{
						i_first = int(s_i);
						clock_first = editedObsFile.m_data[s_i].clock;
						continue;
					}
				}
				else
				{// 将无效钟差更新为左侧的有效的钟差点
					if(i_first >= 0)
						editedObsFile.m_data[s_i].clock = clock_now;
				}
			}
			if(i_first > 0)
			{
				for(size_t s_i = 0; s_i < size_t(i_first); s_i++)
				{
					editedObsFile.m_data[s_i].clock = clock_first;
				}
			}

			FILE *pFile;
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				char szPhasePreprocFileName[300];
				sprintf(szPhasePreprocFileName,"%s\\preproc_GRAPHIC.dat",m_strPreprocPath.c_str());
				pFile = fopen(szPhasePreprocFileName,"w+");
				fprintf(pFile, "%-30s %8s %8s %8s %18s %18s %8s\n",
				            "Epoch",
							"T",
							"PRN",
							"Arc",
							"0.5(P1+L1)",
							"Res_Edited",
							"Marks");
			}
			// 相位数据周跳探测, 继承先前的野值和周跳探测标记
			vector<Rinex2_1_EditedObsSat> editedObsSatlist;
			datalist_epoch2sat(editedObsEpochlist, editedObsSatlist);			
			int arc_k  = 0;
			int ArcCounts = 0;
			for(size_t s_i = 0; s_i < editedObsSatlist.size(); s_i++)
			{
				Rinex2_1_EditedObsEpochMap::iterator it0 = editedObsSatlist[s_i].editedObs.begin();
				GPST t0 = it0->first; 
				int nPRN = it0->second.Id;
				double *pEpochTime = new double[editedObsSatlist[s_i].editedObs.size()];
				int *pSlip = new int [editedObsSatlist[s_i].editedObs.size()];
				int *pEpochId = new int [editedObsSatlist[s_i].editedObs.size()];
				double *pGRAPHIC = new double [editedObsSatlist[s_i].editedObs.size()];
				double *pGRAPHICRes = new double [editedObsSatlist[s_i].editedObs.size()];
				int *pCodeEditedFlag = new int [editedObsSatlist[s_i].editedObs.size()]; 
				int i = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{
					pGRAPHICRes[i] = 0.0;
					pEpochTime[i] = it->first - t0;
					pEpochId[i]  = it->second.nObsTime;
					Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[nObsTypes_P1];
					Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[nObsTypes_L1];
					pGRAPHIC[i] = 0.5 * (GPS_WAVELENGTH_L1 * L1.obs.data + P1.obs.data); // 基于半和改正，利用0.5*(P1+L1)，考虑码偏差
					// 继承前面的伪码观测数据编辑结果, 补充相位野值
					// 增加有效时刻判断, 以保证钟差数据是有效的, 2009/12/25
					// pSlip 的初始化仅包含两部分信息, 未知点TYPE_EDITEDMARK_UNKNOWN和野值TYPE_EDITEDMARK_OUTLIER
					if(L1.byEditedMark1 == TYPE_EDITEDMARK_OUTLIER || validindexlist[pEpochId[i]] == -1)
						pSlip[i] = LEOGPSOBSEDIT_OUTLIER_LIF; // 20150529, TYPE_EDITEDMARK_OUTLIER = 2, 后面obsPreprocInfo2EditedMark1(TYPE_EDITEDMARK_OUTLIER) = 0, 标记会存在问题
					else
						pSlip[i] = LEOGPSOBSEDIT_UNKNOWN;
					if(P1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						pCodeEditedFlag[i] = LEOGPSOBSEDIT_NORMAL; 
					else
						pCodeEditedFlag[i] = TYPE_EDITEDMARK_OUTLIER;
					i++;
				}
				// 对每颗 GPS 卫星的每个跟踪弧段进行周跳探测
				size_t k = 0;
				size_t k_i = k;
				// 获得每个连续跟踪弧段的数据, 并进行周跳探测
				while(1)
				{
					if(k_i + 1 >= editedObsSatlist[s_i].editedObs.size())
						goto newArc;
					else
					{// 判断 k_i+1 与 k_i 是否位于同一跟踪弧段					
						double clock_k = editedObsFile.m_data[pEpochId[k_i + 1]].clock;
			            double clock_k_1 = editedObsFile.m_data[pEpochId[k_i]].clock;
						if(pEpochTime[k_i + 1] - pEpochTime[k_i] <= min(m_PreprocessorDefine.threshold_gap, m_PreprocessorDefine.max_arclengh)
						&&(m_PreprocessorDefine.bOn_ClockJumpDiagnose && fabs(clock_k - clock_k_1) < m_PreprocessorDefine.threshold_ClockJumpSize))	
						{
							k_i++;
							continue;
						}
						else if(pEpochTime[k_i + 1] - pEpochTime[k_i] <= min(m_PreprocessorDefine.threshold_gap, m_PreprocessorDefine.max_arclengh)
						&& !m_PreprocessorDefine.bOn_ClockJumpDiagnose)
						{
							k_i++;
							continue;
						}
						else // k_i + 1 为新弧段的起点
						{
							GPST t_now = t0 + pEpochTime[k_i + 1];//测试代码
							bool bfind_gap = false;
							if((pEpochTime[k_i + 1] - pEpochTime[k_i] > m_PreprocessorDefine.threshold_gap) && (pEpochTime[k_i + 1] - pEpochTime[k_i] < m_PreprocessorDefine.max_arclengh))
							{
								bfind_gap = true;
							}
							goto newArc;
						}
					}
					newArc: // 本弧段 [k, k_i] 数据处理 
					{
						// 整理未曾标记数据
						vector<size_t> unknownPointlist;
						unknownPointlist.clear();
						for(size_t s_ii = k; s_ii <= k_i; s_ii++)
						{// 未知数据标记
							if(pSlip[s_ii] == LEOGPSOBSEDIT_UNKNOWN)
								unknownPointlist.push_back(s_ii); 
						}
						size_t count_unknownpoints = unknownPointlist.size(); 
						if(count_unknownpoints <= m_PreprocessorDefine.min_arcpointcount)
						{// 新弧段内数据正常点个数太少, 直接丢弃
							for(size_t s_ii = 0; s_ii < count_unknownpoints; s_ii++)
							{
								pSlip[unknownPointlist[s_ii]] = LEOGPSOBSEDIT_OUTLIER_COUNT;
							}
						}
						else
						{// 进行相位野值剔除
							ArcCounts++;
							vector<int> slipMarklist; // 1 - 周跳; 0 - 正常点
							slipMarklist.resize(count_unknownpoints);
							for(size_t s_ii = 1; s_ii < count_unknownpoints; s_ii++)
							{
								bool slipFlag;
								int nObsTime_j_1 = pEpochId[unknownPointlist[s_ii - 1]];
								int nObsTime_j = pEpochId[unknownPointlist[s_ii]];
								double res;
								if(obsEdited_GRAPHIC(nObsTypes_P1, nObsTypes_L1, nPRN, editedObsEpochlist[nObsTime_j_1], editedObsEpochlist[nObsTime_j], res, slipFlag))
								{
									if(slipFlag)
										slipMarklist[s_ii] = 1;
									else
										slipMarklist[s_ii] = 0;
								}
								else
								{// 如果 s_ii 时刻的周跳无法诊断(如 n≤2, 或 s_ii 时刻为无效历元), 为了保守起见, 则直接判定 s_ii 时刻为野值
									slipMarklist[s_ii] = 1;
								}
							}
							// 进行野值诊断, 诊断 s_ii 为野值条件: s_ii 为周跳且s_ii + 1为周跳。
							for(size_t s_ii = 1; s_ii < count_unknownpoints - 1; s_ii++)
							{
								if(slipMarklist[s_ii] == 1 && slipMarklist[s_ii + 1] == 1)
								{
									pSlip[unknownPointlist[s_ii]] = LEOGPSOBSEDIT_OUTLIER_LIF;
								}
							}
							// 第一点的判断
							if(slipMarklist[1] == 1)
								pSlip[unknownPointlist[0]] = LEOGPSOBSEDIT_OUTLIER_LIF;
							// 最后一点直接判断
							if(slipMarklist[count_unknownpoints - 1] == 1)
								pSlip[unknownPointlist[count_unknownpoints - 1]] = LEOGPSOBSEDIT_OUTLIER_LIF;
							// 剔除野值 LEOGPSOBSEDIT_OUTLIER_LIF 的影响
							size_t s_ii = 0;
							while(s_ii < unknownPointlist.size())
							{
								if(pSlip[unknownPointlist[s_ii]] == LEOGPSOBSEDIT_UNKNOWN)
									s_ii++;
								else// 在进行周跳探测时, 先将野值 erase
									unknownPointlist.erase(unknownPointlist.begin() + s_ii);
							}
							count_unknownpoints = unknownPointlist.size();
							// 到此处, pSlip[unknownPointlist[i]] 仅包含 LEOGPSOBSEDIT_UNKNOWN
							// 进行周跳探测
							size_t count_slip = 0;
							if(count_unknownpoints > 0)
								pGRAPHICRes[unknownPointlist[0]] = 0;
							for(size_t s_ii = 1; s_ii < count_unknownpoints; s_ii++)
							{
								bool slipFlag;
								int nObsTime_j_1 = pEpochId[unknownPointlist[s_ii - 1]];
								int nObsTime_j   = pEpochId[unknownPointlist[s_ii]];
								double res;
								if(obsEdited_GRAPHIC(nObsTypes_P1, nObsTypes_L1, nPRN, editedObsEpochlist[nObsTime_j_1], editedObsEpochlist[nObsTime_j], res, slipFlag))
								{
									pGRAPHICRes[unknownPointlist[s_ii]] = res;
									if(slipFlag)
									{
										pSlip[unknownPointlist[s_ii]] = LEOGPSOBSEDIT_SLIP_LIF;
										count_slip++;
									}
								}
								else
								{
									pSlip[unknownPointlist[s_ii]] = LEOGPSOBSEDIT_SLIP_LIF;
									count_slip++;
								}
							}
							// 第一点的判断
							if(count_unknownpoints > 1)
							{
								if(pSlip[unknownPointlist[1]] == LEOGPSOBSEDIT_SLIP_LIF)
									pSlip[unknownPointlist[0]] = LEOGPSOBSEDIT_OUTLIER_LIF;
							}

							if(!m_strPreprocPath.empty())
							{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
								for(size_t s_i = k; s_i <= k_i; s_i++)
								{
									fprintf(pFile,"%-30s %8.2f %8d %8d %18.3f %18.3f %8d\n",
									    (t0 + pEpochTime[s_i]).toString().c_str(),
										pEpochTime[s_i],
										nPRN,
										ArcCounts,
										pGRAPHIC[s_i],
										pGRAPHICRes[s_i],
										pSlip[s_i]);
								}
							}
                            // 到此处, pSlip[unknownPointlist[i]] 中包含 LEOGPSOBSEDIT_UNKNOWN 、LEOGPSOBSEDIT_SLIP_LIF 和 LEOGPSOBSEDIT_OUTLIER_LIF (仅在第一点处) 三种标记
							// 无周跳弧段的内符合诊断
							vector<size_t> slipindexlist;
							slipindexlist.clear();
							for(size_t s_ii = k + 1; s_ii <= k_i; s_ii++)
							{
								if(pSlip[s_ii] == LEOGPSOBSEDIT_SLIP_LIF)
									slipindexlist.push_back(s_ii); 
							}
							size_t count_slips = slipindexlist.size();
							size_t *pSubsection_left  = new size_t [count_slips + 1];
							size_t *pSubsection_right = new size_t [count_slips + 1];
							// 记录周跳的左右端点值
							if(count_slips > 0)
							{ 
								pSubsection_left[0] = k;
								for(size_t s_ii = 0; s_ii < count_slips; s_ii++)
								{
									pSubsection_right[s_ii]    = slipindexlist[s_ii] -  1 ;
									pSubsection_left[s_ii + 1] = slipindexlist[s_ii] ;
								}
								pSubsection_right[count_slips] = k_i; 
							}
							else
							{
								pSubsection_left[0]  = k;
								pSubsection_right[0] = k_i;
							} 
							for(size_t s_ii = 0; s_ii < count_slips + 1; s_ii++)
							{// pSlip[unknownPointlist[i]] 中包含LEOGPSOBSEDIT_OUTLIER_LIF和LEOGPSOBSEDIT_UNKNOWN
								// 整理 [pSubsection_left[s_ii], pSubsection_right[s_ii]]
								{
									int count_normalpoints_i = 0;
									vector<size_t> normalPointlist;
									normalPointlist.clear();
									for(size_t s_jj = pSubsection_left[s_ii]; s_jj <= pSubsection_right[s_ii]; s_jj++)
									{// pSlip[i] 中包含LEOGPSOBSEDIT_OUTLIER_LIF、LEOGPSOBSEDIT_SLIP_LIF和LEOGPSOBSEDIT_UNKNOWN
										if(pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_LIF || pSlip[s_jj] == LEOGPSOBSEDIT_UNKNOWN)
										{// 要求相位和伪码数据均正常
											normalPointlist.push_back(s_jj);
											count_normalpoints_i++;  
										}
									}
									if(count_normalpoints_i > int(m_PreprocessorDefine.min_arcpointcount))
									{
										//bool mark_slip = false;//后面程序会将第一个非野值点标记为周跳，此处可省，2014.3.19，刘俊宏
										//for(size_t s_jj = pSubsection_left[s_ii]; s_jj <= pSubsection_right[s_ii]; s_jj++)
										//{ 
										//	// 先将每个小弧段第一个非野值点, 更新标记为周跳
										//	if(!mark_slip && (pSlip[s_jj] == LEOGPSOBSEDIT_SLIP_LIF || pSlip[s_jj] == LEOGPSOBSEDIT_UNKNOWN))
										//	{
										//		if(s_ii == 0)// 首个弧段的第一个非野值点更新标记为 LEOGPSOBSPREPROC_NEWARCBEGIN
										//			pSlip[s_jj] = LEOGPSOBSEDIT_NEWARCBEGIN;
										//		else
										//			pSlip[s_jj] = LEOGPSOBSEDIT_SLIP_LIF;
										//		mark_slip = true;
										//		break;
										//	}
										//}
									}
									else
									{// 删除正常点过少的弧段
										for(size_t s_jj = pSubsection_left[s_ii]; s_jj <= pSubsection_right[s_ii]; s_jj++)
										{
											if(pSlip[s_jj] != LEOGPSOBSEDIT_OUTLIER_LIF)
												pSlip[s_jj] = LEOGPSOBSEDIT_OUTLIER_COUNT;
											
										}
									}
								}
							}
                            // 20150623, 谷德峰
							for(size_t s_ii = k; s_ii <= k_i; s_ii++)
							{
								// 将第一个非野值点, 更新标记为新弧段起点
								if(pSlip[s_ii] == LEOGPSOBSEDIT_UNKNOWN
								|| pSlip[s_ii] == LEOGPSOBSEDIT_SLIP_LIF
								//|| pSlip[s_ii] == LEOGPSOBSEDIT_SLIP_MW       // 单频方法无此项标记
								//|| pSlip[s_ii] == LEOGPSOBSEDIT_SLIP_IFAMB    // 单频方法无此项标记
								|| pSlip[s_ii] == LEOGPSOBSPREPROC_NEWARCBEGIN)
								{
									pSlip[s_ii] = LEOGPSOBSPREPROC_NEWARCBEGIN;
									break;
								}
							}
							delete pSubsection_left;
							delete pSubsection_right;
							// 将未知点 TYPE_EDITEDMARK_UNKNOWN 恢复为正常点 LEOGPSOBSEDIT_NORMAL
							for(size_t s_ii = k; s_ii <= k_i; s_ii++)
							{
								if(pSlip[s_ii] == LEOGPSOBSEDIT_UNKNOWN)
									pSlip[s_ii] = LEOGPSOBSEDIT_NORMAL;  
							}
						}
						if(k_i + 1 >= editedObsSatlist[s_i].editedObs.size())
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
				// 保存野值和周跳标记
				i = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatlist[s_i].editedObs.begin(); it != editedObsSatlist[s_i].editedObs.end(); ++it)
				{
					if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = obsPreprocInfo2EditedMark1(pSlip[i]);
						it->second.obsTypeList[nObsTypes_L1].byEditedMark2 = obsPreprocInfo2EditedMark2(pSlip[i]);
					}
					if(it->second.obsTypeList[nObsTypes_P1].byEditedMark1 != TYPE_EDITEDMARK_OUTLIER)
					{
						if(pCodeEditedFlag[i]!=TYPE_EDITEDMARK_OUTLIER)
						{
							it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = obsPreprocInfo2EditedMark1(pCodeEditedFlag[i]);
							it->second.obsTypeList[nObsTypes_P1].byEditedMark2 = obsPreprocInfo2EditedMark2(pCodeEditedFlag[i]);
						}
					}
					it->second.ReservedField = 0.0;
					i++;
				}
				delete pEpochTime;
				delete pSlip;
				delete pEpochId;
				delete pGRAPHIC;
				delete pGRAPHICRes;
				delete pCodeEditedFlag;
			}
			if(!m_strPreprocPath.empty())
			{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
				fclose(pFile);
			}
			datalist_sat2epoch(editedObsSatlist, editedObsEpochlist);
			// 输出到 editedObsFile 文件
			editedObsFile.m_header = m_obsFile.m_header;
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				// 钟差消除, 在每个历元, 遍历每颗GPS卫星的数据
				if(m_PreprocessorDefine.bOn_ClockEliminate)
				{
					for(Rinex2_1_EditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
					{
						it->second.obsTypeList[nObsTypes_P1].obs.data -= editedObsFile.m_data[s_i].clock;
						it->second.obsTypeList[nObsTypes_L1].obs.data -= editedObsFile.m_data[s_i].clock / GPS_WAVELENGTH_L1;
					}
					editedObsFile.m_data[s_i].clock = 0;
				}
				editedObsFile.m_data[s_i].byEpochFlag = editedObsEpochlist[s_i].byEpochFlag;
				editedObsFile.m_data[s_i].editedObs   = editedObsEpochlist[s_i].editedObs;
				editedObsFile.m_data[s_i].bySatCount  = int(editedObsEpochlist[s_i].editedObs.size());	
			}
			editedObsFile.m_header.tmStart = editedObsEpochlist[0].t;           
			editedObsFile.m_header.tmEnd = editedObsEpochlist[editedObsEpochlist.size() - 1].t;
			DayTime T_Now;
			T_Now.Now();
			sprintf(editedObsFile.m_header.szFileDate, "%04d-%02d-%02d %02d:%02d:%02d",
				                                       T_Now.year,
													   T_Now.month,
													   T_Now.day,
											           T_Now.hour,
													   T_Now.minute,
													   int(T_Now.second));
			sprintf(editedObsFile.m_header.szProgramName, "%-20s", "NUDT Toolkit 1.0");
			sprintf(editedObsFile.m_header.szProgramAgencyName, "%-20s", "NUDT");
			editedObsFile.m_header.pstrCommentList.clear();
			char szComment[100];
			sprintf(szComment, "%-60s%20s\n", 
				               "created by LEO single-frequence GPS edit program.", 
							   Rinex2_1_MaskString::szComment);
			editedObsFile.m_header.pstrCommentList.push_back(szComment);
			sprintf(editedObsFile.m_header.szFileType, "%-20s", "EDITED OBS");
			return true;
		}
	}
}
