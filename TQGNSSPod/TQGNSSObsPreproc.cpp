#include "TQGNSSObsPreproc.hpp"
#include "GNSSBasicCorrectFunc.hpp"
#include "RuningInfoFile.hpp"
#include "TimeCoordConvert.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace TQGNSSPod
	{
		TQGNSSObsPreproc::TQGNSSObsPreproc(void)
		{
			m_strPreprocPath = "";
			m_matAxisAnt2Body = TimeCoordConvert::rotate(PI, 1);
		}

		TQGNSSObsPreproc::~TQGNSSObsPreproc(void)
		{
		}

		void TQGNSSObsPreproc::setPreprocPath(string strPreprocPath)
		{// 20150423, 谷德峰添加, 便于704半物理模拟数据问题分析
			m_strPreprocPath = strPreprocPath;
		}

		void TQGNSSObsPreproc::setSP3File(SP3File sp3File)
		{
			m_sp3File = sp3File;
		}

		void TQGNSSObsPreproc::setCLKFile(CLKFile clkFile)
		{
			m_clkFile = clkFile;
		}

		bool TQGNSSObsPreproc::loadSP3File(string  strSp3FileName)
		{
			return m_sp3File.open(strSp3FileName);
		}

		bool TQGNSSObsPreproc::loadCLKFile(string  strCLKFileName)
		{
			return m_clkFile.open(strCLKFileName);
		}

		bool TQGNSSObsPreproc::loadCLKFile_rinex304(string  strCLKFileName)
		{
			return m_clkFile.open_rinex304(strCLKFileName);
		}

		bool TQGNSSObsPreproc::loadMixedObsFile(string  strObsFileName)
		{
			return m_mixedObsFile.open(strObsFileName);
		}

		bool TQGNSSObsPreproc::loadMixedObsFile_5Obs(string  strObsFileName)
		{
			return m_mixedObsFile.open_5Obs(strObsFileName);
		}

		void TQGNSSObsPreproc::setHeoOrbitList(vector<TimePosVel> heoOrbitList)
		{
			m_heoOrbitList = heoOrbitList;
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
		bool TQGNSSObsPreproc::datalist_epoch2sat(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist, vector<Rinex2_1_EditedObsSat>& editedObsSatlist)
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
		bool TQGNSSObsPreproc::getHeoOrbitPosVel(GPST t, TimePosVel& orbit, unsigned int nLagrange)
		{
			size_t count = m_heoOrbitList.size();
			int nLagrange_left  = int(floor(nLagrange/2.0));   
			int nLagrange_right = int(ceil(nLagrange/2.0));
			if(count < nLagrange)
				return false;
			GPST t_Begin = m_heoOrbitList[0].t;
			GPST t_End   = m_heoOrbitList[count - 1].t;
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
				double time_L = (m_heoOrbitList[middle - 1].t - t_Begin);
				double time_R = (m_heoOrbitList[middle].t - t_Begin);
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
				double time_L = (m_heoOrbitList[left - 1].t - t_Begin);
				double time_R = (m_heoOrbitList[left].t - t_Begin);
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
				 xa_t[i - nBegin] = m_heoOrbitList[i].t - t_Begin;
				 ya_x[i - nBegin] = m_heoOrbitList[i].pos.x;
				 ya_y[i - nBegin] = m_heoOrbitList[i].pos.y;
				 ya_z[i - nBegin] = m_heoOrbitList[i].pos.z;
				ya_vx[i - nBegin] = m_heoOrbitList[i].vel.x;
				ya_vy[i - nBegin] = m_heoOrbitList[i].vel.y;
				ya_vz[i - nBegin] = m_heoOrbitList[i].vel.z;
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
		bool TQGNSSObsPreproc::datalist_sat2epoch(vector<Rinex2_1_EditedObsSat>& editedObsSatlist, vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist)
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
		bool TQGNSSObsPreproc::getEditedObsEpochList(vector<Rinex2_1_LeoEditedObsEpoch>& editedObsEpochlist)
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

		bool TQGNSSObsPreproc::getEditedObsEpochList(vector<Rinex2_1_MixedEditedObsEpoch>& editedObsEpochlist)
		{
			if(m_mixedObsFile.isEmpty())
				return false;
			editedObsEpochlist.clear();
			// resize函数可提高效率
			editedObsEpochlist.resize(m_mixedObsFile.m_data.size());
			/* 转换 CHAMP 一天数据, 耗时 11.70 秒左右 */
			for(size_t s_i = 0; s_i < m_mixedObsFile.m_data.size(); s_i++)
			{
				Rinex2_1_MixedEditedObsEpoch editedObsEpoch;
				editedObsEpoch.load(m_mixedObsFile.m_data[s_i]);
				for(Rinex2_1_MixedEditedObsSatMap::iterator it = editedObsEpoch.editedObs.begin(); it != editedObsEpoch.editedObs.end(); ++it)
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
		bool TQGNSSObsPreproc::getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist)
		{
			vector<Rinex2_1_LeoEditedObsEpoch> editedObsEpochlist;
			// 首先根据观测数据文件 m_obsFile 初始化每个时刻的预处理数据
			if(!getEditedObsEpochList(editedObsEpochlist))
				return	false;
			return datalist_epoch2sat(editedObsEpochlist, editedObsSatlist);
		}
        
		// 修改 。。。2022年3月18日，童礼胜
        // 添加，多系统SPP功能
		bool TQGNSSObsPreproc::mixedObsSPP(Rinex2_1_LeoMixedEditedObsEpoch obsEpoch, POSCLK& posclk, int& eyeableSatCount, double& pdop, double& rms_res, double threshold)
		{
			if(eyeableSatCount < 4)  // 可见星要大于或等于4颗
				return false;
			Matrix matObs(eyeableSatCount, 1); // 伪码无电离层组合
			Matrix matDy(eyeableSatCount, 1);  // 高斯牛顿迭代的观测值改进
			Matrix matH(eyeableSatCount,  4);  // 高斯牛顿迭代的线性化展开矩阵
			Matrix matAppPosClk(4, 1);         // 高斯牛顿迭代的概略点, 初始化可取地心
			Matrix matY(eyeableSatCount,  1);  // 高斯牛顿迭代的观测值
			Matrix matGPSSp3Clk(eyeableSatCount, 4);
			matAppPosClk.SetElement(0, 0, posclk.x);
			matAppPosClk.SetElement(1, 0, posclk.y);
			matAppPosClk.SetElement(2, 0, posclk.z);
			matAppPosClk.SetElement(3, 0, posclk.clk);
			// 双频 P 码消除电离层组合系数		
			int j = 0;
			GPST t_Receive;
			t_Receive = obsEpoch.t - posclk.clk / SPEED_LIGHT;
			for(Rinex2_1_MixedEditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{
				double y = it->second.obsTypeList[0].obs.data;		
				matObs.SetElement(j, 0, y);
				j++;
			}
			j = 0;
			for(Rinex2_1_MixedEditedObsSatMap::iterator it =obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
			{
				double y = matObs.GetElement(j, 0);
				// 迭代计算卫星信号传播时间
				double delay = 0;
				string szSatName= it->first;
				SP3Datum sp3Datum;
				m_sp3File.getEphemeris_PathDelay(obsEpoch.t, posclk, szSatName, delay, sp3Datum);
				// 对 GPS 卫星星历进行地球自转改正
				GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
				// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
				GPST t_Transmit = t_Receive - delay;
				// 对观测值 y 进行误差修正,包括 GPS 卫星钟差改正, GPS卫星相对论修正等, 相位中心修正在数据预处理中暂时不考虑
				// 1. GPS卫星钟差改正
				double correct_gpsclk = 0.0;	
				CLKDatum ASDatum;
				m_clkFile.getSatClock(t_Transmit, szSatName, ASDatum, 3); // 获得 GPS 信号发射时间的卫星钟差改正
				if(m_PreprocessorDefine.bOn_GNSSSAT_Clock)
				{		
					correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT; 
				}
				y = y + correct_gpsclk; 
				// 2. GPS卫星相对论改正
				double correct_relativity = 0.0;
				if(m_PreprocessorDefine.bOn_GNSSSAT_Relativity)
				{
					correct_relativity = (sp3Datum.pos.x * sp3Datum.vel.x 
												  + sp3Datum.pos.y * sp3Datum.vel.y
												  + sp3Datum.pos.z * sp3Datum.vel.z) * (-2.0) / SPEED_LIGHT;
				}
				y = y + correct_relativity; 
				matGPSSp3Clk.SetElement(j, 0, sp3Datum.pos.x);
				matGPSSp3Clk.SetElement(j, 1, sp3Datum.pos.y);
				matGPSSp3Clk.SetElement(j, 2, sp3Datum.pos.z);
				if(m_PreprocessorDefine.bOn_GNSSSAT_Clock)
					matGPSSp3Clk.SetElement(j, 3, ASDatum.clkBias);
				else
					matGPSSp3Clk.SetElement(j, 3, 0.0);
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
					rms_res = 0.0;
					////if(eyeableGPSCount >= 5)
					//{// 只有卫星数大于5, 才能计算 RAIM 检验统计量
					//	double sse = 0;
					//	for(int i = 0; i < eyeableSatCount; i++)
					//		sse += pow(matDy.GetElement(i, 0), 2);
					//	rms_res = sqrt(sse / (eyeableSatCount - 4));
					//} 
					break;
			     }
				// 控制迭代次数, 控制改进增量
				if(k_GaussNewton >= max_GaussNewtonCount || fabs(delta) >= delta_max)
				{
					rms_res = DBL_MAX;
					pdop = 0.0;
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
				t_Receive = obsEpoch.t - posclk.clk / SPEED_LIGHT;
				j = 0;
				for(Rinex2_1_MixedEditedObsSatMap::iterator it = obsEpoch.editedObs.begin(); it != obsEpoch.editedObs.end(); ++it)
				{
					if(delta > 100.0) // 如果改进量小于100m, 为减少计算量, 可不进行视线矢量 matH 的更新
					{
						double y = matObs.GetElement(j, 0);
						// 迭代计算卫星信号传播时间
						double delay = 0;
						string szSatName= it->first;
						SP3Datum sp3Datum;
						m_sp3File.getEphemeris_PathDelay(obsEpoch.t,  posclk,  szSatName, delay, sp3Datum);
						// 对 GPS 卫星星历进行地球自转改正
						GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);
						// 计算信号发射时间 t_Transmit( 参考信号真实接收时间(t_Receive))
						GPST t_Transmit = t_Receive - delay;
						// 对观测值 y 进行误差修正,包括 GPS 卫星钟差改正, GPS卫星相对论修正等, 相位中心修正在数据预处理中暂时不考虑
						// 1. GPS卫星钟差改正	
						CLKDatum ASDatum;
						m_clkFile.getSatClock(t_Transmit, szSatName, ASDatum, 3); // 获得 GPS 信号发射时间的卫星钟差改正
						double correct_gpsclk = 0.0;					
						if(m_PreprocessorDefine.bOn_GNSSSAT_Clock)
						{
							correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT; 
						}	
						y = y + correct_gpsclk; 
						// 2. GPS卫星相对论改正
						double correct_relativity = 0;
						if(m_PreprocessorDefine.bOn_GNSSSAT_Relativity)
						{
							correct_relativity = (sp3Datum.pos.x * sp3Datum.vel.x 
														  + sp3Datum.pos.y * sp3Datum.vel.y
														  + sp3Datum.pos.z * sp3Datum.vel.z) * (-2.0) / SPEED_LIGHT;
						}
						y = y + correct_relativity; 
						matGPSSp3Clk.SetElement(j, 0, sp3Datum.pos.x);
						matGPSSp3Clk.SetElement(j, 1, sp3Datum.pos.y);
						matGPSSp3Clk.SetElement(j, 2, sp3Datum.pos.z);
						if(m_PreprocessorDefine.bOn_GNSSSAT_Clock)
							matGPSSp3Clk.SetElement(j, 3, ASDatum.clkBias);
						else
							matGPSSp3Clk.SetElement(j, 3, 0.0);
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
		// 子程序名称： mainFuncHeoGNSSObsEdit   
		// 作用：高轨Heo卫星观测数据编辑
		// 类型：mainFuncHeoGNSSObsEdit主函数，主要包含编辑数据文件生成及SPP定位功能
		// 输入：mixedEditedObsFile
		// 输出：mixedEditedObsFile 
		// 语言：C++
		// 创建者：杨诚鋆，童礼胜
		// 创建时间：2022/3/23
		// 版本时间：2022/3/23
		// 修改记录：
		// 其它：
		bool  TQGNSSObsPreproc::mainFuncHeoGNSSObsEdit(Rinex2_1_LeoMixedEditedObsFile  &mixedEditedObsFile)
		{
			// 判断观测数据结构和星历数据结构是否为空
			if(m_sp3File.isEmpty())
			{
				printf("警告：星历数据缺失, 跳过预处理环节.\n");
				return false;
			}
			if(int(m_preMixedSysList.size()) == 0)
				return false;
			// 步骤1 将obs格式转换为edt格式，使用先验轨道卫星
			if(m_mixedObsFile.isEmpty())
			{
				printf("无观测数据, 请确认!\n");
				return  false;				
			}
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_L2 = -1, nObsTypes_P1 = -1, nObsTypes_P2 = -1;
			for(int i = 0; i < m_mixedObsFile.m_header.byObsTypes; i++)
			{
				if(m_mixedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)  //第一个频点相位
					nObsTypes_L1 = i;
				if(m_mixedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)  //第二个频点相位
					nObsTypes_L2 = i;
				if(m_mixedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)  //第一个频点伪距
					nObsTypes_P1 = i;
				if(m_mixedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)  //第二个频点伪距
					nObsTypes_P2 = i;
			}
			if(nObsTypes_L1 == -1 || nObsTypes_L2 == -1 || nObsTypes_P1 == -1 || nObsTypes_P2 == -1) 
				return false;			
			vector<Rinex2_1_MixedEditedObsEpoch> editedObsEpochlist; // 混合历元
			getEditedObsEpochList(editedObsEpochlist);               // 
			vector<int> validindexlist;
			validindexlist.resize(editedObsEpochlist.size());
			mixedEditedObsFile.m_data.clear();
			mixedEditedObsFile.m_data.resize(editedObsEpochlist.size());
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
             // 更新真实观测时刻
				GPST t_Receive;
				if(m_PreprocessorDefine.bOn_ClockEliminate)
					t_Receive = editedObsEpochlist[s_i].t; // 可认为经704软校正的采样时刻是较为准确的, 2010/06/12
				else
					t_Receive = editedObsEpochlist[s_i].t - editedObsEpochlist[s_i].clock / SPEED_LIGHT;
				TimePosVel orbit_t;
                if(!getHeoOrbitPosVel(t_Receive, orbit_t))
					continue;
				mixedEditedObsFile.m_data[s_i].pos  = orbit_t.pos;
				mixedEditedObsFile.m_data[s_i].vel  = orbit_t.vel;
				POSCLK posclk;
				posclk.x = orbit_t.pos.x;
				posclk.y = orbit_t.pos.y;
				posclk.z = orbit_t.pos.z;
				posclk.clk = editedObsEpochlist[s_i].clock;
				if(m_PreprocessorDefine.bOn_ClockEliminate)
					posclk.clk = 0.0; // 可认为经704软校正的采样时刻是较为准确的, 2015/05/05
				// 计算GPS卫星的天空视图
				POS3D S_Z; // Z轴指向卫星
				POS3D S_X; // X轴沿速度方向
				POS3D S_Y; // 右手系
				POS3D S_R, S_T, S_N;
				POS6D posvel_i;
				posvel_i.setPos(mixedEditedObsFile.m_data[s_i].pos);
				posvel_i.setVel(mixedEditedObsFile.m_data[s_i].vel);
				if(!TimeCoordConvert::getCoordinateRTNAxisVector(editedObsEpochlist[s_i].t, posvel_i, S_R, S_T, S_N))
					printf("%s 轨道坐标系获取失败！\n");
                S_X = S_T * (1.0);
			    S_Y = S_N * (1.0);
				S_Z = S_R * (1.0);
				// 根据观测时刻, 更新 GPS 卫星轨道位置和速度, 以及 GPS 卫星钟差
				for(Rinex2_1_MixedEditedObsSatMap::iterator it = editedObsEpochlist[s_i].editedObs.begin(); it != editedObsEpochlist[s_i].editedObs.end(); ++it)
				{
					double delay = 0;
					SP3Datum sp3Datum;
					char szSatName[5];
					sprintf(szSatName, "%s", it->first.c_str());
					szSatName[4] = '\0';
					if(!m_sp3File.getEphemeris_PathDelay(editedObsEpochlist[s_i].t, posclk, szSatName, delay, sp3Datum))
					{
						printf("星历获取失败！\n");
						continue;
					}
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
					it->second.obsTypeList[nObsTypes_L1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					it->second.obsTypeList[nObsTypes_L2].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					it->second.obsTypeList[nObsTypes_P1].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
					it->second.obsTypeList[nObsTypes_P2].byEditedMark1 = TYPE_EDITEDMARK_NORMAL;			
				}
			}
			mixedEditedObsFile.m_data.resize(editedObsEpochlist.size());
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				mixedEditedObsFile.m_data[s_i].t = editedObsEpochlist[s_i].t;
				mixedEditedObsFile.m_data[s_i].clock = 0.0; // 钟差初始化为 0
				mixedEditedObsFile.m_data[s_i].byRAIMFlag = 2;
				{
					// 更新真实观测时刻
					GPST t_Receive;
					t_Receive = editedObsEpochlist[s_i].t - editedObsEpochlist[s_i].clock / SPEED_LIGHT;
					TimePosVel orbit_t;
                    if(!getHeoOrbitPosVel(t_Receive, orbit_t))
					{
						// 该时刻的数据为无效时刻, 丢弃该时刻的数据
						printf("%6d时刻 %s 数据无效(getLeoOrbitPosVel)!\n", s_i, mixedEditedObsFile.m_data[s_i].t.toString().c_str());
					}
					mixedEditedObsFile.m_data[s_i].pos  = orbit_t.pos;
					mixedEditedObsFile.m_data[s_i].vel  = orbit_t.vel;
				}
			}
			// 输出到 editedObsFile 文件
			mixedEditedObsFile.m_header = m_mixedObsFile.m_header;
			for(size_t s_i = 0; s_i < editedObsEpochlist.size(); s_i++)
			{
				mixedEditedObsFile.m_data[s_i].byEpochFlag = editedObsEpochlist[s_i].byEpochFlag;
				mixedEditedObsFile.m_data[s_i].editedObs   = editedObsEpochlist[s_i].editedObs;
				mixedEditedObsFile.m_data[s_i].bySatCount  = int(editedObsEpochlist[s_i].editedObs.size());	
			}
			// 步骤2 进行SPP解算，多系统统一求解位置+钟差，暂时忽略系统间偏差问题
			//vector<spp_results>  SPP_resultsList;
			for(size_t s_i = 0; s_i < mixedEditedObsFile.m_data.size(); s_i ++)
			{
				int id_Epoch = int(s_i);
				POSCLK      posclk;
				posclk.x   = mixedEditedObsFile.m_data[s_i].pos.x;
				posclk.y   = mixedEditedObsFile.m_data[s_i].pos.y;
				posclk.z   = mixedEditedObsFile.m_data[s_i].pos.z;
				posclk.clk = mixedEditedObsFile.m_data[s_i].clock;
				double pdop = 0.0;
				double hdop = 0.0;//杨诚鋆 2022.03.16新增
				double vdop = 0.0;//杨诚鋆 2022.03.16新增
				double rms_res = 0.0;
				int satCount = int(mixedEditedObsFile.m_data[s_i].editedObs.size());
				bool result_spp = false;	
				result_spp = mixedObsSPP(mixedEditedObsFile.m_data[s_i], posclk, satCount, pdop, rms_res);
				if(result_spp && pdop != 0.0)
				{
					spp_results spp_results_i;
					spp_results_i.eyeableGPSCount = satCount;
					spp_results_i.pdop    = pdop;
					spp_results_i.posclk = posclk;
					spp_results_i.rms_res = rms_res;
					spp_results_i.t = mixedEditedObsFile.m_data[s_i].t;
					SPP_resultsList.push_back(spp_results_i);
									 
					if(pdop<=500)//杨诚鋆 2022.03.14新增 将pdop<500的SPP结果写入edt文件 //2022.04.05杨诚鋆注释 为检查是否因为初值导致的定轨发散
					{ // 500阈值有待确定？
						mixedEditedObsFile.m_data[s_i].pos.x = spp_results_i.posclk.x;
						mixedEditedObsFile.m_data[s_i].pos.y = spp_results_i.posclk.y;
						mixedEditedObsFile.m_data[s_i].pos.z = spp_results_i.posclk.z;
						mixedEditedObsFile.m_data[s_i].clock = spp_results_i.posclk.clk;
						mixedEditedObsFile.m_data[s_i].pdop = pdop;	
					}
					else
					{
						mixedEditedObsFile.m_data[s_i].pos.x = spp_results_i.posclk.x;
						mixedEditedObsFile.m_data[s_i].pos.y = spp_results_i.posclk.y;
						mixedEditedObsFile.m_data[s_i].pos.z = spp_results_i.posclk.z;
						mixedEditedObsFile.m_data[s_i].clock = spp_results_i.posclk.clk;
						mixedEditedObsFile.m_data[s_i].pdop = pdop;	
					}
				}
			}
			mixedEditedObsFile.m_header.tmStart = editedObsEpochlist[0].t;           
			mixedEditedObsFile.m_header.tmEnd = editedObsEpochlist[editedObsEpochlist.size() - 1].t;
			DayTime T_Now;
			T_Now.Now();
			sprintf(mixedEditedObsFile.m_header.szFileDate, "%04d-%02d-%02d %02d:%02d:%02d",
				                                       T_Now.year,
													   T_Now.month,
													   T_Now.day,
											           T_Now.hour,
													   T_Now.minute,
													   int(T_Now.second));
			sprintf(mixedEditedObsFile.m_header.szProgramName, "%-20s", "NUDT Toolkit 1.0");
			mixedEditedObsFile.m_header.szProgramName[20]    = '\0';
			sprintf(mixedEditedObsFile.m_header.szProgramAgencyName, "%-20s", "NUDT");
			mixedEditedObsFile.m_header.szProgramAgencyName[20]    = '\0';
			mixedEditedObsFile.m_header.pstrCommentList.clear();
			char szComment[100];
			sprintf(szComment, "%-60s%20s\n", 
				               "created by TQS dual-frequence GPS edit program.", 
							   Rinex2_1_MaskString::szComment);
			mixedEditedObsFile.m_header.pstrCommentList.push_back(szComment);
			sprintf(mixedEditedObsFile.m_header.szFileType, "%-20s", "EDITED OBS");
			mixedEditedObsFile.m_header.szRinexVersion[20] = '\0';
			mixedEditedObsFile.m_header.szAntNumber[20]  = '\0';
			mixedEditedObsFile.m_header.szAntType[20]    = '\0';
			mixedEditedObsFile.m_header.szObserverAgencyName[40] = '\0';
			mixedEditedObsFile.m_header.szRecNumber[20] = '\0';
			mixedEditedObsFile.m_header.szRecType[20] = '\0';
			mixedEditedObsFile.m_header.szRinexVersion[20]  = '\0';	
			mixedEditedObsFile.m_header.szFileType[20]  = '\0';
			mixedEditedObsFile.m_header.szRinexVersion[20] = '\0';
			mixedEditedObsFile.m_header.szSatlliteSystem[20] = '\0';
			mixedEditedObsFile.m_header.szProgramName[20] = '\0';
			mixedEditedObsFile.m_header.szProgramAgencyName[20] = '\0';
			mixedEditedObsFile.m_header.szFileDate[20] = '\0';
			mixedEditedObsFile.m_header.szObserverName[20] = '\0';
			return true;
		}
	}
}