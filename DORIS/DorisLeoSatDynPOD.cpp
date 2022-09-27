#include "DorisLeoSatDynPOD.hpp"
#include "MathAlgorithm.hpp"
#include "LeoSP3File.hpp"
#include "SolidTides.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace DORIS
	{
		DorisLeoSatDynPOD::DorisLeoSatDynPOD(void)
		{
		}

		DorisLeoSatDynPOD::~DorisLeoSatDynPOD(void)
		{
		}

		bool DorisLeoSatDynPOD::loadStdcFile(string strStdcFileFolder)
		{
			m_listStcdFile.clear();
			WIN32_FIND_DATA FindFileData;
		    char szSearchPath[MAX_PATH];
			sprintf(szSearchPath,"%s\\ign09wd01.stcd.*", strStdcFileFolder.c_str());
			HANDLE FileHandle = FindFirstFile(szSearchPath, &FindFileData);
			while(FileHandle != INVALID_HANDLE_VALUE)
			{
				char szStcdFilePath[MAX_PATH];
				sprintf(szStcdFilePath,"%s\\%s", strStdcFileFolder.c_str(), FindFileData.cFileName);
				DorisSTCDFile stcdfile;
				if(stcdfile.open(szStcdFilePath))
					m_listStcdFile.push_back(stcdfile);
				FindNextFile(FileHandle, &FindFileData);
				if(GetLastError() == ERROR_NO_MORE_FILES)
					break;
			}
			return true;
		}

		bool DorisLeoSatDynPOD::getStationPos(string site_code, TAI t, double &x, double &y, double &z)
		{
			for(int i = 0; i < int(m_listStcdFile.size()); i++)
			{
				if(site_code[0] == m_listStcdFile[i].m_siteID.site_code[0]
				&& site_code[1] == m_listStcdFile[i].m_siteID.site_code[1]
				&& site_code[2] == m_listStcdFile[i].m_siteID.site_code[2]
				&& site_code[3] == m_listStcdFile[i].m_siteID.site_code[3])
				{
					if(m_listStcdFile[i].getPos(t, x, y, z))
						return true;
				}
			}
			return false;
		}

		bool DorisLeoSatDynPOD::getObsArcList(vector<Doris2_2_EditedObsEpoch> obsEpochList, vector<DorisObsEqArc> &obsArcList)
		{
			if(obsEpochList.size() <= 0)
				return false;
			obsArcList.clear();
            // 首先根据obsEpochList整理出每个测站的所有数据 stationList, 并打上时间标记 nObsTime
			DorisObsEqArc stationList[MAX_ID_DORISSTATION + 1];
			for(int i = 0; i < MAX_ID_DORISSTATION + 1; i++)
			{
				stationList[i].offsetFrequence = 0.0;
				stationList[i].id_Station = i;
				stationList[i].obsList.clear();
			}
            for(size_t s_i = 0; s_i < obsEpochList.size(); s_i++)
			{
				for(Doris2_2_EditedObsStationMap::iterator it = obsEpochList[s_i].obs.begin(); it != obsEpochList[s_i].obs.end(); ++it)
				{
					if(it->first >= 0 && it->first <= MAX_ID_DORISSTATION)
					{
						ObsEqArcElement arcRaw;
						arcRaw.nObsTime = int(s_i);
						arcRaw.obs = it->second.Range_rate + it->second.mass_correction;
						if(it->second.Iono_apply != 0)
							arcRaw.obs += it->second.Iono_correction;
						if(it->second.Trop_apply != 0)
							arcRaw.obs += it->second.Trop_correction;
                        arcRaw.obs = arcRaw.obs * 1.0E-6;
                        arcRaw.robustweight = 1.0;
						arcRaw.res = 0.0;
						stationList[it->first].obsList.push_back(arcRaw);
					}
				}
			}
            // 然后根据时间间隔将每个子弧段的观测数据区分开, 存储在obsArcList
			// id_Station = 0 的测站未知数据将直接被丢弃
			for(int i = 1; i < MAX_ID_DORISSTATION + 1; i++)
			{
				if(stationList[i].obsList.size() <= 0)
					continue;
				// 每个有效连续跟踪弧段数据, 对应一个新的模糊度参数, 进行处理
				size_t k   = 0; // 记录新弧段起始点
				size_t k_i = k; // 记录新弧段终止点
				while(1)
				{
					if(k_i + 1 >= stationList[i].obsList.size()) // k_i 为时间序列终点
						goto newArc;
					else
					{
						// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段
						if(obsEpochList[stationList[i].obsList[k_i + 1].nObsTime].t - obsEpochList[stationList[i].obsList[k_i].nObsTime].t <= m_podParaDefine.max_arclengh)
						{
							k_i++;
							continue;
						}
						// 如果相邻两点的间隔时间 > max_arclengh
						else // k_i + 1 为新弧段的起点
							goto newArc;
					}
					newArc:  // 本弧段[k，k_i]数据处理 
					{
						{// 记录新弧段数据
							DorisObsEqArc newArc;
							newArc.id_Station = stationList[i].id_Station;
							newArc.offsetFrequence = stationList[i].offsetFrequence;
							newArc.obsList.clear();
							for(size_t s_k = k; s_k <= k_i; s_k++)
							{
								newArc.obsList.push_back(stationList[i].obsList[s_k]);
							}
							if(newArc.obsList.size() >= m_podParaDefine.min_arcpointcount) // 需要进一步剔除观测数据较少或测站标号未知的测站
								obsArcList.push_back(newArc);
						}

						if(k_i + 1 >= stationList[i].obsList.size()) // k_i为时间序列终点, 跳出
							break;
						else  
						{   
							k   = k_i + 1;    // 新弧段的起点设置
							k_i = k;
							continue;
						}
					}
				}
			}
			return true;
		}

		// 子程序名称： adamsCowell_ac  
		// 功能：首先利用线性多步数值积分方法获得长弧段卫星轨道数据和偏导数数据
		//       卫星轨道数据和偏导数数据的时间点与积分步长严格对齐,并向两段延长4个步长, 提高边缘部分插值精度
		//       为后续插值提供基准点数据, 其中轨道插值采用 8 阶 lagrange方法, 偏导数插值采用线性方法
		// 变量类型：t0_Interp           : 参考时间历元, 插值输出点的参考时间
		//           t1_Interp
		//           dynamicDatum        : 初始点的长弧段动力学参数, (dynamicDatum.t0 的时间可以与 interpTimelist[0] 不对应, 此时进行倒向积分)
		//           orbitlist_ac        : 每一点插值参考轨道
		//           matRtPartiallist_ac : 每一点轨道位置对动力学参数的偏导数
		//           h                   : 积分步长
		//           q                   : Adams_Cowell 的阶数
		// 输入： t0_Interp, t1_Interp, dynamicDatum, h, q
		// 输出： interpTimelist, matRtPartiallist
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2011/07/13
		// 版本时间：
		// 修改记录：
		// 备注： 默认步长选取30s, 11阶
		bool DorisLeoSatDynPOD::adamsCowell_ac(TDT t0_Interp, TDT t1_Interp, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h, int q)
		{
			orbitlist_ac.clear();
			matRtPartiallist_ac.clear();
			TDT  t_Begin = t0_Interp; // 起始时间
			TDT  t_End   = t1_Interp; // 终止时间
			//const int countDynParameter = dynamicDatum.getAllEstParaCount(); 
			// 倒向积分，积分区间 [dynamicDatum.t0, t_End   + h * 4]，为保证插值精度向两端进行扩展
			vector<TimePosVel> backwardOrbitlist_ac; 
			vector<TimePosVel> forwardOrbitlist_ac; 
			vector<Matrix> backwardRtPartiallist_ac;  
			vector<Matrix> forwardRtPartiallist_ac;  
			if(t_Begin - dynamicDatum.T0 < 0)
			{// t_Begin位于dynamicDatum.t0之前, 此时需要考虑倒向积分
				AdamsCowell(dynamicDatum, t_Begin - h * 4.0, backwardOrbitlist_ac, backwardRtPartiallist_ac, -h, q);
				for(size_t s_i = backwardOrbitlist_ac.size() - 1; s_i > 0; s_i--)
				{// 注: dynamicDatum.t0 点在下面的正向积分中添加
					orbitlist_ac.push_back(backwardOrbitlist_ac[s_i]);
					matRtPartiallist_ac.push_back(backwardRtPartiallist_ac[s_i]);
				}
			}
			AdamsCowell(dynamicDatum, t_End  + h * 4.0, forwardOrbitlist_ac, forwardRtPartiallist_ac, h, q);
			for(size_t s_i = 0; s_i < forwardOrbitlist_ac.size(); s_i++)
			{
				orbitlist_ac.push_back(forwardOrbitlist_ac[s_i]);
				matRtPartiallist_ac.push_back(forwardRtPartiallist_ac[s_i]);
			}
			return true;
		}

		// 子程序名称： getOrbPartial_interp   
		// 功能： 然后该算法考虑到了积分点的采样时刻与观测历元不一致的情况
		//        进行插值求解, 其中轨道插值采用 8 阶 lagrange方法, 偏导数插值采用线性方法
		// 变量类型： t                   : 插值时刻(TDT)
		//            orbitlist_ac        : 轨道插值参考序列
		//            matRtPartiallist_ac : 偏导数插值参考序列
		//            interpOrbit         : 轨道插值结果
		//            interpRtPartial     : 偏导数插值结果
		// 输入： t, orbitlist_ac, matRtPartiallist_ac
		// 输出： interpOrbit, interpRtPartial
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2011/07/13
		// 版本时间：
		// 修改记录：
		// 备注：
		bool DorisLeoSatDynPOD::getOrbPartial_interp(TDT t, vector<TimePosVel>& orbitlist_ac, vector<Matrix>& matRtPartiallist_ac, TimePosVel &interpOrbit, Matrix &interpRtPartial)
		{
			// 将 adamsCowell 轨道和偏导数, 插值 (8 阶 lagrange) 到观测时刻
			size_t count_ac = orbitlist_ac.size();
			const int nlagrange = 8; 
			if(count_ac < nlagrange) // 如果数据点个数小于nlagrange返回, 要求弧段长度 > h * nlagrange = 4分钟
				return false;
			const int countDynParameter = matRtPartiallist_ac[0].GetNumColumns(); 
			double h = orbitlist_ac[1].t - orbitlist_ac[0].t;
			{// 将 adamsCowell 积分轨道通过插值折算到 interpTimelist[s_i].T 处
				double spanSecond_t = t - orbitlist_ac[0].t;                   // 相对观测时间, 初始时间为 orbitlist_ac[0].t
				int nLeftPos  = int(spanSecond_t / h);                         // 首先寻找最靠近时间 T 的左端点，从 0 开始计数
				int nLeftNum  = int(floor(nlagrange / 2.0));                   // 理论上 nLeftPos 左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
				int nRightNum = int(ceil(nlagrange / 2.0));
				int nBegin, nEnd;                                              // 位于区间[0, count_ac - 1]
				if(nLeftPos - nLeftNum + 1 < 0)                                // nEnd - nBegin = nLagrange - 1 
				{
					nBegin = 0;
					nEnd   = nlagrange - 1;
				}
				else if(nLeftPos + nRightNum >= int(count_ac))
				{
					nBegin = int(count_ac) - nlagrange;
					nEnd   = int(count_ac) - 1;
				}
				else
				{
					nBegin = nLeftPos - nLeftNum + 1;
					nEnd   = nLeftPos + nRightNum;
				}
				// 轨道点
				interpOrbit.t = t;
				double *x = new double [nlagrange];
				double *y = new double [nlagrange];
				for(int i = nBegin; i <= nEnd; i++)
					x[i - nBegin] = orbitlist_ac[i].t - orbitlist_ac[0].t; // 参考相对时间点
				// X
				for(int i = nBegin; i <= nEnd; i++)
					y[i - nBegin] = orbitlist_ac[i].pos.x;
				InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.pos.x);
				// Y
				for(int i = nBegin; i <= nEnd; i++)
					y[i - nBegin] = orbitlist_ac[i].pos.y;
				InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.pos.y);
				// Z
				for(int i = nBegin; i <= nEnd; i++)
					y[i - nBegin] = orbitlist_ac[i].pos.z;
				InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.pos.z);
				// Vx
				for(int i = nBegin; i <= nEnd; i++)
					y[i - nBegin] = orbitlist_ac[i].vel.x;
				InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.x);
				// Vy
				for(int i = nBegin; i <= nEnd; i++)
					y[i - nBegin] = orbitlist_ac[i].vel.y;
				InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.y);
				// Vz
				for(int i = nBegin; i <= nEnd; i++)
					y[i - nBegin] = orbitlist_ac[i].vel.z;
				InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.z);
				// 偏导数，未必与轨道保持相同的插值方法，采用线性插值即可
				interpRtPartial.Init(3, countDynParameter);
				// 偏导数插值, 阶数2, 线性插值, 2008/06/27
				if(nLeftPos < 0) // nEnd - nBegin = nLagrange - 1 
				{
					nBegin = 0;
					nEnd   = 1;
				}
				else if(nLeftPos + 1 >= int(count_ac))
				{
					nBegin = int(count_ac) - 2;
					nEnd   = int(count_ac) - 1;
				}
				else
				{
					nBegin = nLeftPos;
					nEnd   = nLeftPos + 1;
				}
				double x_t[2];
				double y_t[2];
				x_t[0] = orbitlist_ac[nBegin].t - orbitlist_ac[0].t;
				x_t[1] = orbitlist_ac[nEnd].t - orbitlist_ac[0].t;
				double u = (spanSecond_t - x_t[0])/(x_t[1] - x_t[0]);
				for(int ii = 0; ii < 3; ii++)
				{
					for(int jj = 0; jj < int(countDynParameter); jj++)
					{// 对矩阵的每个元素[ii, jj]进行插值
						y_t[0] = matRtPartiallist_ac[nBegin].GetElement(ii, jj);
						y_t[1] = matRtPartiallist_ac[nEnd].GetElement(ii, jj);
						double element = u * y_t[1] + (1 - u) * y_t[0];
						interpRtPartial.SetElement(ii, jj, element);
					}
				}
				delete x;
				delete y;
			}
			return true;
		}

		// 子程序名称： getTransmitPathDelay   
		// 功能：根据测站的位置、信号接收时间和卫星位置,
		//       计算信号传播延迟时间(即测站“准确的”信号发射时间)
		// 变量类型： t                  : 信号接收时间
		//            satPos_j2000       : 接收机概略位置(J2000), 单位：米
		//            staPos_ECEF        : 测站的概略位置(地固系)
		//            delay              : 信号传播延迟时间, 单位：秒
		//            staPosVel_j2000    : 确定了正确的信号发射时间后，顺便返回测站的位置(J2000)
		//            threshold          : 迭代阈值，默认 1.0E-007
		// 输入：t, satPos_j2000, staPos_ECEF,threshold
		// 输出：delay, staPosVel_j2000
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2013/03/05
		// 版本时间：
		// 修改记录：1. 2013/04/23 由谷德峰修改, 为了进行地球旋转参数估计
		// 备注： 
		bool DorisLeoSatDynPOD::getTransmitPathDelay(TAI t, POS3D satPos_j2000, POS3D staPos_ECEF, DorisEopEstParameter eopEstPara, double& delay, POS6D& staPosVel_j2000, double threshold)
		{
			// 信号真实接收时间 = 观测时间(T)
			TAI t_Receive  = t;
			TAI t_Transmit = t_Receive; // 初始化Doris信号发射时间
			//double x_ecf[6];
			//double x_j2000[6];
			//x_ecf[0] = staPos_ECEF.x; 
			//x_ecf[1] = staPos_ECEF.y;
			//x_ecf[2] = staPos_ECEF.z;
			//x_ecf[3] = 0.0;
			//x_ecf[4] = 0.0;
			//x_ecf[5] = 0.0;
			//m_TimeCoordConvert.ECEF_J2000(TimeCoordConvert::TAI2GPST(t_Transmit), x_j2000, x_ecf, true);
			//staPosVel_j2000.x  = x_j2000[0];
			//staPosVel_j2000.y  = x_j2000[1];
			//staPosVel_j2000.z  = x_j2000[2];
			//staPosVel_j2000.vx = x_j2000[3];
			//staPosVel_j2000.vy = x_j2000[4];
			//staPosVel_j2000.vz = x_j2000[5];
			Matrix matPR_NR, matER, matEP, matER_DOT;
			m_TimeCoordConvert.Matrix_J2000_ECEF(TimeCoordConvert::TAI2GPST(t_Transmit), matPR_NR, matER, matEP, matER_DOT);
			Matrix matEst_EP, matEst_ER;
			eopEstPara.getEst_EOP(t_Transmit, matEst_EP, matEst_ER);
			matEP = matEst_EP * matEP;
			matER = matEst_ER * matER;
			Matrix matH = matEP * matER * matPR_NR; // J2000->ECEF 矩阵
			Matrix matJ2000Pos(3, 1);
			matJ2000Pos.SetElement(0, 0, staPos_ECEF.x);
			matJ2000Pos.SetElement(1, 0, staPos_ECEF.y);
			matJ2000Pos.SetElement(2, 0, staPos_ECEF.z);
			matJ2000Pos = matH.Transpose() * matJ2000Pos; // ECEF->J2000
			staPosVel_j2000.x  = matJ2000Pos.GetElement(0, 0);
			staPosVel_j2000.y  = matJ2000Pos.GetElement(1, 0);
			staPosVel_j2000.z  = matJ2000Pos.GetElement(2, 0);

			double distance = pow(satPos_j2000.x - staPosVel_j2000.x, 2)
                            + pow(satPos_j2000.y - staPosVel_j2000.y, 2)
						    + pow(satPos_j2000.z - staPosVel_j2000.z, 2);
			distance = sqrt(distance);       // 获得Doris信号发射传播距离
			double delay_k_1 = 0;
			delay = distance / SPEED_LIGHT;  // 获得GPS信号发射传播延迟
			const double delay_max  = 1.0;   // 为了防止迭代dDelay溢出，这里设置一个阈值
			const int    k_max      = 5;     // 迭代次数阈值，一般1次迭代就会收敛？ 
			int          k          = 0;
			while(fabs(delay - delay_k_1) > threshold)   // 迭代阈值控制, abs-->fabs, 2007-07-15
			{
				k++;
				if(fabs(delay) > delay_max || k > k_max) // 为防止 delay 溢出, 2007-04-06
				{
					printf("%d%d%f delay 迭代发散!\n", t.hour, t.minute, t.second);
					return false;
				}
				// 更新 Doris 信号发射时间
				t_Transmit = t_Receive - delay;
				/*m_TimeCoordConvert.ECEF_J2000(TimeCoordConvert::TAI2GPST(t_Transmit), x_j2000, x_ecf, true);
				staPosVel_j2000.x  = x_j2000[0];
				staPosVel_j2000.y  = x_j2000[1];
				staPosVel_j2000.z  = x_j2000[2];
				staPosVel_j2000.vx = x_j2000[3];
				staPosVel_j2000.vy = x_j2000[4];
				staPosVel_j2000.vz = x_j2000[5];*/
				m_TimeCoordConvert.Matrix_J2000_ECEF(TimeCoordConvert::TAI2GPST(t_Transmit), matPR_NR, matER, matEP, matER_DOT);
				eopEstPara.getEst_EOP(t_Transmit, matEst_EP, matEst_ER);
				matEP = matEst_EP * matEP;
				matER = matEst_ER * matER;
				Matrix matH = matEP * matER * matPR_NR; // J2000->ECEF 矩阵
				matJ2000Pos.SetElement(0, 0, staPos_ECEF.x);
				matJ2000Pos.SetElement(1, 0, staPos_ECEF.y);
				matJ2000Pos.SetElement(2, 0, staPos_ECEF.z);
				matJ2000Pos = matH.Transpose() * matJ2000Pos; // ECEF->J2000
				staPosVel_j2000.x  = matJ2000Pos.GetElement(0, 0);
				staPosVel_j2000.y  = matJ2000Pos.GetElement(1, 0);
				staPosVel_j2000.z  = matJ2000Pos.GetElement(2, 0);
				// 更新概略距离
				distance = pow(satPos_j2000.x - staPosVel_j2000.x, 2)
                         + pow(satPos_j2000.y - staPosVel_j2000.y, 2)
						 + pow(satPos_j2000.z - staPosVel_j2000.z, 2);
			    distance = sqrt(distance); 
				// 更新延迟数据
				delay_k_1 = delay;
				delay = distance / SPEED_LIGHT;
			}
			return true;
		}

		// 子程序名称： dynamicPOD_2_2   
		// 功能：利用doris 2.2 格式数据进行动力学轨道确定
		// 变量类型：obsFilePath          : 观测数据文件路径
        //           dynamicDatum         : 拟合后的初始动力学轨道参数
		//           t0_forecast          : 预报轨道初始时间, TAI
		//           t1_forecast          : 预报轨道终止时间, TAI
		//           forecastOrbList      : 预报轨道列表, 采用TAI, ITRF坐标系
		//           interval             : 预报轨道间隔
		//           bInitDynDatumEst     : 初始动力学轨道求解标记
		//           bForecast            : 预报标记, 默认true, 否则不进行预报, 用于初轨确定
		//           bResEdit             : 定轨 O-C 残差编辑标记开关, 默认 true, 否则不进行残差编辑 
		// 输入：dynamicDatum, t0_forecast, t1_forecast, interval, bInitDynDatumEst, bForecast, bResEdit
		// 输出：dynamicDatum, forecastOrbList
		// 语言：C++
		// 创建者：谷德峰, 刘俊宏
		// 创建时间：2013/03/05
		// 版本时间：
		// 修改记录：1. 2013/04/23 由谷德峰修改, 增加每个弧段一个天顶对流层延迟估计参数
		// 备注： 
		bool DorisLeoSatDynPOD::dynamicPOD_2_2(string obsFilePath, SatdynBasicDatum &dynamicDatum, TAI t0_forecast, TAI t1_forecast, vector<TimePosVel> &forecastOrbList, double interval, bool bForecast, bool bResEdit)
		{
			string obsFileName = obsFilePath.substr(obsFilePath.find_last_of("\\") + 1);
			string folder = obsFilePath.substr(0, obsFilePath.find_last_of("\\"));
			string obsFileName_noexp = obsFileName.substr(0, obsFileName.find_last_of("."));
			if(!m_obsFile.open(obsFilePath))
				return false;
			// 获得测站列表的地固系位置, 测站位置在地固系下是相对固定的
			for(int i = 0; i <= MAX_ID_DORISSTATION; i++)
			{
				if(!getStationPos(dorisStationId2String(i), TimeCoordConvert::TDT2TAI(dynamicDatum.T0), m_ppDorisStationPos[i][0], m_ppDorisStationPos[i][1], m_ppDorisStationPos[i][2]))
				{
					m_ppDorisStationPos[i][0] = 0;
					m_ppDorisStationPos[i][1] = 0;
					m_ppDorisStationPos[i][2] = 0;
				}
			}
			 // 整理观测数据, 得到每个时刻的观测数据 obsEpochList
			m_obsFile.cutdata(TimeCoordConvert::TDT2TAI(dynamicDatum.T0), TimeCoordConvert::TDT2TAI(dynamicDatum.T0) + dynamicDatum.ArcLength);
			vector<Doris2_2_EditedObsStation> obsStationList; // 测站列表
			m_obsFile.getObsStationList(obsStationList);
			vector<Doris2_2_EditedObsEpoch> obsEpochList;     // 时刻列表
			m_obsFile.getObsEpochList(obsStationList, obsEpochList);
			size_t count_epoch = obsEpochList.size(); // 观测数据个数
			if(count_epoch <= 0)
				return false;
			// 根据观测数据的分布情况, 调整大气阻力和经验加速度
			// 大气阻力
			size_t s_i = 0;
			size_t s_j = 0;
			int count_validindex_combined = 0; // 记录合并前一个区间的有效数据个数
			while(s_i < dynamicDatum.atmosphereDragParaList.size())
			{
				int count_validindex = count_validindex_combined;
				TAI t0 = TimeCoordConvert::TDT2TAI(dynamicDatum.atmosphereDragParaList[s_i].t0);
				TAI t1 = TimeCoordConvert::TDT2TAI(dynamicDatum.atmosphereDragParaList[s_i].t1);
				for(size_t s_k = s_j; s_k < count_epoch; s_k++)
				{
					if(obsEpochList[s_k].t- t1 <= 0 && obsEpochList[s_k].t - t0 >= 0)
					{// 统计有效点个数 [t0, t1]
					    count_validindex++;
					}
					else
					{
						s_j = s_k;
						break;
					}
				}
				count_validindex_combined = 0; //合并之前, 初始化为零
				if(count_validindex > 90) // 确定合并条件, 目前暂定为 90 个有效点, 相当于 15 分钟有效数据
				{
					s_i++;
				}
				else
				{
					if(s_i == dynamicDatum.atmosphereDragParaList.size() - 1 && dynamicDatum.atmosphereDragParaList.size() > 1)
					{// 最后一个区间, 且区间个数大于 1, 此时向前合并
						printf("区间%s—%s观测数据过少(%d), 大气阻力估计区间向前合并!\n", dynamicDatum.atmosphereDragParaList[s_i].t0.toString().c_str(),
							                                                              dynamicDatum.atmosphereDragParaList[s_i].t1.toString().c_str(),
																				          count_validindex);
						dynamicDatum.atmosphereDragParaList[s_i - 1].t1 = dynamicDatum.atmosphereDragParaList[s_i].t1;
						dynamicDatum.atmosphereDragParaList.erase(dynamicDatum.atmosphereDragParaList.begin() + s_i);
					}
					else if(s_i == dynamicDatum.atmosphereDragParaList.size() - 1 && dynamicDatum.atmosphereDragParaList.size() == 1)
						;// 只有一个区间且无效时,不进行任何处理, 一般不应该出现这种情况
					else
					{// 后向合并
						printf("区间%s—%s观测数据过少(%d), 大气阻力估计区间向后合并!\n", dynamicDatum.atmosphereDragParaList[s_i].t0.toString().c_str(),
							                                                              dynamicDatum.atmosphereDragParaList[s_i].t1.toString().c_str(),
																				          count_validindex);
						dynamicDatum.atmosphereDragParaList[s_i + 1].t0 = dynamicDatum.atmosphereDragParaList[s_i].t0;
                        dynamicDatum.atmosphereDragParaList.erase(dynamicDatum.atmosphereDragParaList.begin() + s_i);
						count_validindex_combined = count_validindex; // 合并事件发生后, 标记合并数据个数
						
					}
				}
			}
			// 经验加速度区间调整
			s_i = 0;
			s_j = 0;
			count_validindex_combined = 0; // 记录合并前一个区间的有效数据个数
			while(s_i < dynamicDatum.empiricalForceParaList.size())
			{
				int count_validindex = count_validindex_combined;
				TAI t0 = TimeCoordConvert::TDT2TAI(dynamicDatum.empiricalForceParaList[s_i].t0);
				TAI t1 = TimeCoordConvert::TDT2TAI(dynamicDatum.empiricalForceParaList[s_i].t1);
				for(size_t s_k = s_j; s_k < count_epoch; s_k++)
				{
					if(obsEpochList[s_k].t- t1 <= 0 && obsEpochList[s_k].t - t0 >= 0)
					{// 统计有效点个数 [t0, t1]
						count_validindex++;
					}
					else
					{
						s_j = s_k;
						break;
					}
				}
				count_validindex_combined = 0; //合并之前, 初始化为零
				if(count_validindex > 90) // 确定合并条件, 目前暂定为 90 个有效点, 相当于 15 分钟有效数据
				{
					s_i++;
				}
				else
				{
					if(s_i == dynamicDatum.empiricalForceParaList.size() - 1 && dynamicDatum.empiricalForceParaList.size() > 1)
					{// 最后一个区间, 且区间个数大于 1, 此时向前合并
						printf("区间%s—%s观测数据过少(%d), 经验力估计区间向前合并!\n", dynamicDatum.empiricalForceParaList[s_i].t0.toString().c_str(),
							                                                            dynamicDatum.empiricalForceParaList[s_i].t1.toString().c_str(),
																						count_validindex);
						dynamicDatum.empiricalForceParaList[s_i - 1].t1 = dynamicDatum.empiricalForceParaList[s_i].t1;
						dynamicDatum.empiricalForceParaList.erase(dynamicDatum.empiricalForceParaList.begin() + s_i);
					}
					else if(s_i == dynamicDatum.empiricalForceParaList.size() - 1 && dynamicDatum.empiricalForceParaList.size() == 1)
						;// 只有一个区间且无效时,不进行任何处理, 一般不应该出现这种情况
					else
					{// 后向合并
						printf("区间%s—%s观测数据过少(%d), 经验力估计区间向后合并!\n", dynamicDatum.empiricalForceParaList[s_i].t0.toString().c_str(),
							                                                            dynamicDatum.empiricalForceParaList[s_i].t1.toString().c_str(),
																						count_validindex);
						dynamicDatum.empiricalForceParaList[s_i + 1].t0 = dynamicDatum.empiricalForceParaList[s_i].t0;
                        dynamicDatum.empiricalForceParaList.erase(dynamicDatum.empiricalForceParaList.begin() + s_i);
						count_validindex_combined = count_validindex;
					}
				}
			}
			// 根据 obsEpochList, 初始化每个时刻的动力学定轨相关信息 dynEpochList
            vector<DorisPODEpoch> dynEpochList; 
			dynEpochList.resize(count_epoch);
            for(size_t s_i = 0; s_i < count_epoch; s_i++)
			{
				dynEpochList[s_i].eyeableStaCount = int(obsEpochList[s_i].obs.size()); // 每个时刻观测数据个数
				dynEpochList[s_i].t = obsEpochList[s_i].t;                                    
				for(Doris2_2_EditedObsStationMap::iterator it = obsEpochList[s_i].obs.begin(); it != obsEpochList[s_i].obs.end(); ++it)
				{
					DorisObsEqEpochElement datum_j;
					datum_j.weight = 1.0; // 观测权值
					//if(it->second.Point_infor >= 4 || it->second.Point_infor == 1)
					//{
					//	datum_j.weight = 0.0; // 无用的数据, 权值标记为 0
					//}
					if(it->second.Point_infor != 0 ||(string2DorisStationId(it->second.Station_ID) == 46)) // || it->second.Channel_ID == 7
					{
						datum_j.weight = 0.0; // 无用的数据, 权值标记为0
					}
					datum_j.duration = it->second.Cout_interval * 1.0E-7; // 用于距离变化率的差分时间间隔, 单位: 秒
					datum_j.obscorrected_value = 0.0;                     // 用于记录修正值, 如概略轨道改进值
                    dynEpochList[s_i].mapDatum.insert(DorisPODEpochStationMap::value_type(it->first, datum_j));
				}
			}
			// 根据历元观测数据信息obsEpochList, 重新整理获得弧段观测信息-dorisArcList(包含了每个历元的时序信息) 
			vector<DorisObsEqArc> dorisArcList;
			getObsArcList(obsEpochList, dorisArcList); // 剔除了部分弧段观测数据个数较少的数据和未知测站的数据
			// 保存初轨确定结果
			SatdynBasicDatum dynamicDatum_Init = dynamicDatum; // 记录初始轨道根数
			DorisEopEstParameter eopEstPara;
			eopEstPara.t0_xpyput1 = dynEpochList[0].t + 0.5 * (dynEpochList[count_epoch - 1].t - dynEpochList[0].t);
			char dynamicDatumFilePath[300];
			sprintf(dynamicDatumFilePath,"%s\\dynpod_%s.fit", folder.c_str(), obsFileName_noexp.c_str());
			FILE * pFitFile = fopen(dynamicDatumFilePath, "w+");
			fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
			fprintf(pFitFile, "%3d. EOP  XP   (mas)    %20.4f\n",  1,0.0);
			fprintf(pFitFile, "%3d. EOP  XPDOT(mas/d)  %20.4f\n",  2,0.0);
			fprintf(pFitFile, "%3d. EOP  YP   (mas)    %20.4f\n",  3,0.0);
			fprintf(pFitFile, "%3d. EOP  YPDOT(mas/d)  %20.4f\n",  4,0.0);
			fprintf(pFitFile, "%3d. EOP  UT   (ms)     %20.4f\n",  5,0.0);
			fprintf(pFitFile, "%3d. EOP  UTDOT(ms/d)   %20.4f\n",  6,0.0);
			fprintf(pFitFile, "%3d.      X    (m)      %20.4f\n",  7,dynamicDatum_Init.X0.x);
			fprintf(pFitFile, "%3d.      Y    (m)      %20.4f\n",  8,dynamicDatum_Init.X0.y);
			fprintf(pFitFile, "%3d.      Z    (m)      %20.4f\n",  9,dynamicDatum_Init.X0.z);
			fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f\n", 10,dynamicDatum_Init.X0.vx);
			fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f\n", 11,dynamicDatum_Init.X0.vy);
			fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f\n", 12,dynamicDatum_Init.X0.vz);
			int k_Parameter = 12;
			if(dynamicDatum_Init.bOn_SolarPressureAcc && dynamicDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
			{
				for(size_t s_i = 0; s_i < dynamicDatum_Init.solarPressureParaList.size(); s_i++)
				{
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].Cr = 0.0;
					fprintf(pFitFile, "%3d. %2d   CR            %20.4f\n", k_Parameter ,
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].Cr);
				}
			}
			if(dynamicDatum_Init.bOn_SolarPressureAcc && dynamicDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
			{
				for(size_t s_i = 0; s_i < dynamicDatum_Init.solarPressureParaList.size(); s_i++)
				{
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].D0 = 0;
					fprintf(pFitFile, "%3d. %2d   D0   (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].DC1 = 0;
					fprintf(pFitFile, "%3d. %2d   DCOS (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].DC1 * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].DS1 = 0;
					fprintf(pFitFile, "%3d. %2d   DSIN (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].DS1 * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].Y0 = 0;
					fprintf(pFitFile, "%3d. %2d   Y0   (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].YC1 = 0;
					fprintf(pFitFile, "%3d. %2d   YCOS (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].YC1 * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].YS1 = 0;
					fprintf(pFitFile, "%3d. %2d   YSIN (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].YS1 * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].B0 = 0;
					fprintf(pFitFile, "%3d. %2d   B0   (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].B0 * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].BC1 = 0;
					fprintf(pFitFile, "%3d. %2d   BCOS (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].BC1 * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].BS1 = 0;
					fprintf(pFitFile, "%3d. %2d   BSIN (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].BS1 * 1.0E+7);
				}
			}
			if(dynamicDatum_Init.bOn_AtmosphereDragAcc) // dynamicDatum_Init.atmosphereDragType == TYPE_ATMOSPHEREDRAG_JACCHIA71
			{
				for(size_t s_i = 0; s_i < dynamicDatum_Init.atmosphereDragParaList.size(); s_i++)
				{
					k_Parameter++;
					dynamicDatum_Init.atmosphereDragParaList[s_i].Cd = 0.0;
					fprintf(pFitFile, "%3d. %2d   CD            %20.4f\n",  k_Parameter,
																		    s_i+1,
																		    dynamicDatum_Init.atmosphereDragParaList[s_i].Cd);
				}
			}
			if(dynamicDatum_Init.bOn_EmpiricalForceAcc && dynamicDatum_Init.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
			{
				for(size_t s_i = 0; s_i < dynamicDatum_Init.empiricalForceParaList.size(); s_i++)
				{
					k_Parameter++;
					dynamicDatum_Init.empiricalForceParaList[s_i].cos_T = 0.0;
					fprintf(pFitFile, "%3d. %2d   TCOS (1.0E-7) %20.4f\n",    k_Parameter,
																			  s_i+1,
																			  dynamicDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.empiricalForceParaList[s_i].sin_T = 0.0;
					fprintf(pFitFile, "%3d. %2d   TSIN (1.0E-7) %20.4f\n",    k_Parameter,
																			  s_i+1,
																			  dynamicDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.empiricalForceParaList[s_i].cos_N = 0.0;
					fprintf(pFitFile, "%3d. %2d   NCOS (1.0E-7) %20.4f\n",    k_Parameter,
																			  s_i+1,
																			  dynamicDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.empiricalForceParaList[s_i].sin_N = 0.0;
					fprintf(pFitFile, "%3d. %2d   NSIN (1.0E-7) %20.4f\n",    k_Parameter,
																			  s_i+1,
																			  dynamicDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7);
				}
			}

			if(dynamicDatum_Init.bOn_EmpiricalForceAcc && dynamicDatum_Init.empiricalForceType == TYPE_EMPIRICALFORCE_SPLINE)
			{
				size_t s_i;
				for(s_i = 0; s_i < dynamicDatum_Init.empiricalForceParaList.size(); s_i++)
				{
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   A0_T (1.0E-7) %20.4f\n", k_Parameter,
																			           s_i+1,
																			           dynamicDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7);
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   A0_N (1.0E-7) %20.4f\n", k_Parameter,
																			           s_i+1,
																			           dynamicDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7);
				}
				s_i = dynamicDatum_Init.empiricalForceParaList.size() - 1;
				k_Parameter++;
				fprintf(pFitFile, "%3d. %2d   A0_T (1.0E-7) %20.4f\n", k_Parameter,
																		           s_i+2,
																		           dynamicDatum_Init.empiricalForceParaList[s_i].a1_T * 1.0E+7);
				k_Parameter++;
				fprintf(pFitFile, "%3d. %2d   A0_N (1.0E-7) %20.4f\n", k_Parameter,
																		           s_i+2,
																		           dynamicDatum_Init.empiricalForceParaList[s_i].a1_N * 1.0E+7);
			}
			fclose(pFitFile);
			// 迭代开始
			bool flag_robust = false;
			int  num_after_residual_edit = 0;
			bool flag_break = false;
			bool result = true;
			int  k = 0; // 记录迭代的次数
		    vector<TimePosVel> interpOrbitlist;     // 插值序列
			vector<Matrix>     interpRtPartiallist; // 插值偏导数序列
			// 动力学参数个数统计
			int count_DynParameter = dynamicDatum.getAllEstParaCount(); 
			int count_SolarPressureParaList = 0;
			if(dynamicDatum.bOn_SolarPressureAcc)
				count_SolarPressureParaList = int(dynamicDatum.solarPressureParaList.size());
			int count_AtmosphereDragParaList = 0;
			if(dynamicDatum.bOn_AtmosphereDragAcc)
				count_AtmosphereDragParaList = int(dynamicDatum.atmosphereDragParaList.size());
			int count_EmpiricalForceParaList = 0;
			if(dynamicDatum.bOn_EmpiricalForceAcc)
				count_EmpiricalForceParaList = int(dynamicDatum.empiricalForceParaList.size());
			double factor_eop = 1.0;
			while(1)
			{
				k++;
				if(k >= m_podParaDefine.max_OrbitIterativeNum)
				{
					result = false;
					printf("迭代超过%d次, 发散!", k);
					break;
				}
				// 利用初始动力学轨道, 获得 adamsCowell 积分轨道点序列 orbitlist_ac 和偏导数序列 matRtPartiallist_ac
				TDT t_Begin = TimeCoordConvert::TAI2TDT(obsEpochList[0].t);
				TDT t_End   = TimeCoordConvert::TAI2TDT(obsEpochList[count_epoch - 1].t); 
				vector<TimePosVel> orbitlist_ac;
				vector<Matrix> matRtPartiallist_ac;
				adamsCowell_ac(t_Begin, t_End, dynamicDatum, orbitlist_ac, matRtPartiallist_ac);
				printf("第%d次 adamsCowell_ac is ok!\n", k);

				//// 检核积分后的轨道数据
				//FILE* pFile_adamsCowell_ac = fopen("c:\\adamsCowell_ac.txt", "w+");
				//LeoSP3File leoSp3File;
				//leoSp3File.open("spot5\\lcasp501.b09105.e09109.sp3.001");
				//for(size_t s_i = 0; s_i < orbitlist_ac.size(); s_i++)
				//{
				//	double x_ecf[6];
				//	double x_j2000[6];
				//	POS6D posvel_ECEF;
				//	leoSp3File.getEphemeris(TimeCoordConvert::TDT2TAI(orbitlist_ac[s_i].t), posvel_ECEF);
				//	x_ecf[0] = posvel_ECEF.x;
				//	x_ecf[1] = posvel_ECEF.y;
				//	x_ecf[2] = posvel_ECEF.z;
				//	x_ecf[3] = posvel_ECEF.vx;
				//	x_ecf[4] = posvel_ECEF.vy;
				//	x_ecf[5] = posvel_ECEF.vz;
				//	m_TimeCoordConvert.ECEF_J2000(TimeCoordConvert::TDT2GPST(orbitlist_ac[s_i].t), x_j2000, x_ecf, true);
				//	fprintf(pFile_adamsCowell_ac,"%s %20.10f%20.10f%20.10f%20.10f%20.10f%20.10f\n", orbitlist_ac[s_i].t.toString().c_str(),
				//		                                                             orbitlist_ac[s_i].pos.x - x_j2000[0],
				//																	 orbitlist_ac[s_i].pos.y - x_j2000[1],
				//																	 orbitlist_ac[s_i].pos.z - x_j2000[2],
				//																	 orbitlist_ac[s_i].vel.x - x_j2000[3],
				//																	 orbitlist_ac[s_i].vel.y - x_j2000[4],
				//																	 orbitlist_ac[s_i].vel.z - x_j2000[5]);
				//}
				//fclose(pFile_adamsCowell_ac);
				//return false;

				// 根据概略点, 更新 dynEpochList
				for(size_t s_i = 0; s_i < count_epoch; s_i++)
				{// 积分获得 t0 时刻的卫星轨道以及偏导数序列(在每个历元, 测站的t0时刻相同)
					TDT t_TDT_i =  TimeCoordConvert::TAI2TDT(dynEpochList[s_i].t);
					TimePosVel interpOrbit;
					Matrix interpRtPartial;
					getOrbPartial_interp(t_TDT_i, orbitlist_ac, matRtPartiallist_ac, interpOrbit, interpRtPartial);
					for(DorisPODEpochStationMap::iterator it = dynEpochList[s_i].mapDatum.begin(); it != dynEpochList[s_i].mapDatum.end(); ++it)
					{// 积分获得 t1 时刻的卫星轨道以及偏导数序列(在每个历元, 测站的duration不同)
						TimePosVel interpOrbit_j;
				        Matrix interpRtPartial_j;
						getOrbPartial_interp(t_TDT_i + it->second.duration, orbitlist_ac, matRtPartiallist_ac, interpOrbit_j, interpRtPartial_j);
						// 记录偏导数插值结果
						it->second.interpRtPartial_t0 = interpRtPartial;
						it->second.interpRtPartial_t1 = interpRtPartial_j;
						// 获得太阳位置 
						double P_J2000[3]; // 惯性坐标, 用于坐标系转换
						double P_ITRF[3];  // 地固坐标
						TDB  t_TDB = TimeCoordConvert::TDT2TDB(t_TDT_i); // 获得TDB时间--提供太阳历参考时间
						GPST t_GPS = TimeCoordConvert::TDT2GPST(t_TDT_i);
					    double jd_TDB = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日
						POS3D sunPos_ITRF;
						POS3D sunPos_J2000;
						m_JPLEphFile.getSunPos_Delay_EarthCenter(jd_TDB, P_J2000); 
						for(int i = 0; i < 3; i ++)
							P_J2000[i] = P_J2000[i] * 1000; // 换算成米
						sunPos_J2000.x = P_J2000[0];
						sunPos_J2000.y = P_J2000[1];
						sunPos_J2000.z = P_J2000[2];
						m_TimeCoordConvert.J2000_ECEF(t_GPS, P_J2000, P_ITRF, false); // 坐标系转换
						sunPos_ITRF.x = P_ITRF[0];
						sunPos_ITRF.y = P_ITRF[1];
						sunPos_ITRF.z = P_ITRF[2];
						// 获得月球的位置
						POS3D moonPos_ITRF;
						m_JPLEphFile.getPlanetPos(JPLEph_Moon, jd_TDB, P_J2000);  // 获得J2000系下的太阳相对地心的位置（千米）
						for(int i = 0; i < 3; i ++)
							P_J2000[i] = P_J2000[i] * 1000;                       // 换算成米
						m_TimeCoordConvert.J2000_ECEF(t_GPS, P_J2000, P_ITRF, false); // 坐标系转换
						moonPos_ITRF.x  = P_ITRF[0];
						moonPos_ITRF.y  = P_ITRF[1];
						moonPos_ITRF.z  = P_ITRF[2];
						// 获得测站的 j2000 位置
					    POS6D staPosVel_j2000_t0; 
						POS6D staPosVel_j2000_t1;
						POS3D staPos_ECEF;
						staPos_ECEF.x = m_ppDorisStationPos[it->first][0];
						staPos_ECEF.y = m_ppDorisStationPos[it->first][1];
						staPos_ECEF.z = m_ppDorisStationPos[it->first][2];
						double xp = 0;
						double yp = 0;
						if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_2003)
							m_TimeCoordConvert.m_eopRapidFileIAU2000.getPoleOffset(m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::TDT2TAI(t_TDT_i)), xp, yp);
						POS3D posSolidTide_ECEF = SolidTides::solidTideCorrect(t_GPS, sunPos_ITRF, moonPos_ITRF, staPos_ECEF, xp, yp);
						staPos_ECEF = staPos_ECEF + posSolidTide_ECEF; // 进行固体潮修正, 2013/06/13
						// 增加地球旋转参数估计后, 由于发射时间是迭代计算, 对应时间点的地球旋转矩阵需改进
						double delay_t0;
						if(!getTransmitPathDelay(dynEpochList[s_i].t, interpOrbit.pos, staPos_ECEF, eopEstPara, delay_t0, staPosVel_j2000_t0))
							it->second.weight = 0.0;
						it->second.t0_sta_transmit = dynEpochList[s_i].t - delay_t0;
						double delay_t1;
						if(!getTransmitPathDelay(dynEpochList[s_i].t + it->second.duration, interpOrbit_j.pos, staPos_ECEF, eopEstPara, delay_t1, staPosVel_j2000_t1))
							it->second.weight = 0.0;
						it->second.t1_sta_transmit = dynEpochList[s_i].t  + it->second.duration - delay_t1;
                        // 获得相对矢量
						it->second.vecLos_t0 = interpOrbit.getPosVel() - staPosVel_j2000_t0;
						it->second.vecLos_t1 = interpOrbit_j.getPosVel() - staPosVel_j2000_t1;
						// 更新概略点的距离差分值
						double distance_t0 = sqrt(pow(it->second.vecLos_t0.x, 2)
							                    + pow(it->second.vecLos_t0.y, 2)
												+ pow(it->second.vecLos_t0.z, 2));
						double distance_t1 = sqrt(pow(it->second.vecLos_t1.x, 2)
							                    + pow(it->second.vecLos_t1.y, 2)
												+ pow(it->second.vecLos_t1.z, 2));
						/*double r_t0 = sqrt(pow(staPosVel_j2000_t0.x, 2)
							             + pow(staPosVel_j2000_t0.y, 2)
									     + pow(staPosVel_j2000_t0.z, 2));
						double r_t1 = sqrt(pow(staPosVel_j2000_t1.x, 2)
							             + pow(staPosVel_j2000_t1.y, 2)
									     + pow(staPosVel_j2000_t1.z, 2));*/
						it->second.elevation_sta_t0 = 90 - acos(vectorDot(vectorNormal(it->second.vecLos_t0.getPos()),vectorNormal(staPosVel_j2000_t0.getPos()))) * 180 / PI;
                        it->second.elevation_sta_t1 = 90 - acos(vectorDot(vectorNormal(it->second.vecLos_t1.getPos()),vectorNormal(staPosVel_j2000_t1.getPos()))) * 180 / PI;
						it->second.obscorrected_value = -(distance_t1 - distance_t0) / it->second.duration; 
					}
				}
				if(k == 1)
				{// 根据概略轨道及其修正值 obscorrected_value, 获得初始频偏大小
					for(size_t s_i = 0; s_i < dorisArcList.size(); s_i++)
					{
						int count_obs_i = int(dorisArcList[s_i].obsList.size());
						int id_Station = dorisArcList[s_i].id_Station;
						int count_normal = 0;
						dorisArcList[s_i].offsetFrequence = 0;
						dorisArcList[s_i].zenithDelay = 0;
						for(int s_j = 0; s_j < count_obs_i; s_j++)
						{
							int nObsTime = dorisArcList[s_i].obsList[s_j].nObsTime;
							double w = dynEpochList[nObsTime].mapDatum[id_Station].weight * dorisArcList[s_i].obsList[s_j].robustweight;
							if(w != 0)
							{
								count_normal++;
								dorisArcList[s_i].offsetFrequence += dorisArcList[s_i].obsList[s_j].obs + dynEpochList[nObsTime].mapDatum[id_Station].obscorrected_value;
							}
						}
                        if(count_normal > 0)
							dorisArcList[s_i].offsetFrequence = dorisArcList[s_i].offsetFrequence / count_normal;
					}
				}
				// 确定 dorisArcList 每个弧段数据是否参与解算
				size_t s_i = 0;
				while(s_i < dorisArcList.size())
				{
					size_t count_obs_normal_i = 0;
					int id_sation = dorisArcList[s_i].id_Station;
					for(size_t s_j = 0; s_j < dorisArcList[s_i].obsList.size(); s_j++)
					{
						int nObsTime = dorisArcList[s_i].obsList[s_j].nObsTime;
						double w = dynEpochList[nObsTime].mapDatum[id_sation].weight * dorisArcList[s_i].obsList[s_j].robustweight; // 观测权值
						if(w != 0.0)
							count_obs_normal_i++;
					}
					if(count_obs_normal_i < m_podParaDefine.min_arcpointcount)
						dorisArcList.erase(dorisArcList.begin() + s_i);
					else
						s_i++;
				}
				// 残差编辑
				if(flag_robust && num_after_residual_edit == 0 && bResEdit)
				{// 计算定轨进行, 并进行残差编辑, 更新观测权矩阵
					double rms = 0;
					int count_valid = 0;
					for(size_t s_i = 0; s_i < dorisArcList.size(); s_i++)
					{
						int count_obs_i = int(dorisArcList[s_i].obsList.size());
						int id_Station = dorisArcList[s_i].id_Station;
						for(int s_j = 0; s_j < count_obs_i; s_j++)
						{
							int nObsTime = dorisArcList[s_i].obsList[s_j].nObsTime;
							double tzdDelay = 0;
							if(m_podParaDefine.bOnEst_StaTropZenithDelay)
							{
								double h1 = 1 / sin(dynEpochList[nObsTime].mapDatum[id_Station].elevation_sta_t1 * PI / 180.0);
								double h0 = 1 / sin(dynEpochList[nObsTime].mapDatum[id_Station].elevation_sta_t0 * PI / 180.0);
								tzdDelay = dorisArcList[s_i].zenithDelay * (h1 - h0);
							}
							dorisArcList[s_i].obsList[s_j].res = dorisArcList[s_i].obsList[s_j].obs  // 距离变化率
									                           + dynEpochList[nObsTime].mapDatum[id_Station].obscorrected_value
									                           - dorisArcList[s_i].offsetFrequence
															   - tzdDelay;
							double w = dynEpochList[nObsTime].mapDatum[id_Station].weight * dorisArcList[s_i].obsList[s_j].robustweight;
							if(dorisArcList[s_i].obsList[s_j].robustweight == 1.0 && dynEpochList[nObsTime].mapDatum[id_Station].weight != 0)
							{
								count_valid++;
								rms += pow(dorisArcList[s_i].obsList[s_j].res, 2);
							}
						}
					}
					rms = sqrt(rms / count_valid);
				    // 更新 robustweight
					for(size_t s_i = 0; s_i < dorisArcList.size(); s_i++)
					{
						int id_Station = dorisArcList[s_i].id_Station;
						for(size_t s_j = 0; s_j < dorisArcList[s_i].obsList.size(); s_j++)
						{
							if(fabs(dorisArcList[s_i].obsList[s_j].res) > rms * 3.0)
							{
								dorisArcList[s_i].obsList[s_j].robustweight = rms / fabs(dorisArcList[s_i].obsList[s_j].res);
								//int nObsTime = dorisArcList[s_i].obsList[s_j].nObsTime;
								//if(dynEpochList[nObsTime].mapDatum[id_Station].weight != 0)// 超差残差
								//	printf("%20.10f %20.10f\n", rms, dorisArcList[s_i].obsList[s_j].res);
							}
							else
								dorisArcList[s_i].obsList[s_j].robustweight = 1.0;
						}
					}
					num_after_residual_edit++;
					flag_robust = false; // 关闭编辑, 防止下次又重新进行编辑
				}
				if(flag_break)
				{// 计算最终残差
					//FILE* pFile_res = fopen("c:\\res.txt", "w+");
					// 观测残差
					double rms = 0;
					int count_valid = 0;
					for(size_t s_i = 0; s_i < dorisArcList.size(); s_i++)
					{
						int count_obs_i = int(dorisArcList[s_i].obsList.size());
						int id_Station = dorisArcList[s_i].id_Station;
						for(int s_j = 0; s_j < count_obs_i; s_j++)
						{
							int nObsTime = dorisArcList[s_i].obsList[s_j].nObsTime;
							double tzdDelay = 0;
							if(m_podParaDefine.bOnEst_StaTropZenithDelay)
							{
								double h1 = 1 / sin(dynEpochList[nObsTime].mapDatum[id_Station].elevation_sta_t1 * PI / 180.0);
								double h0 = 1 / sin(dynEpochList[nObsTime].mapDatum[id_Station].elevation_sta_t0 * PI / 180.0);
								tzdDelay = dorisArcList[s_i].zenithDelay * (h1 - h0);
							}
							dorisArcList[s_i].obsList[s_j].res = dorisArcList[s_i].obsList[s_j].obs  // 距离变化率
									                           + dynEpochList[nObsTime].mapDatum[id_Station].obscorrected_value
									                           - dorisArcList[s_i].offsetFrequence
															   - tzdDelay;
							double w = dynEpochList[nObsTime].mapDatum[id_Station].weight * dorisArcList[s_i].obsList[s_j].robustweight;
							if(w != 0)
							{
								count_valid++;
								rms += pow(dorisArcList[s_i].obsList[s_j].res, 2);
								//fprintf(pFile_res, "%5d%5d%20.10f\n", s_i, id_Station, dorisArcList[s_i].obsList[s_j].res * w);
							}
						}
					}
					rms = sqrt(rms / count_valid);
					printf("相位定轨残差 rms_oc_phase = %.5f\n", rms);
				    //fclose(pFile_res);
					break;
				}
				// 设计矩阵
				int count_arc = int(dorisArcList.size());
				int count_ff_Parameter = count_arc;
				if(m_podParaDefine.bOnEst_StaTropZenithDelay)
					count_ff_Parameter += count_arc;
				int count_xx_Parameter = count_DynParameter;
				if(m_podParaDefine.bOnEst_ERP)
					count_xx_Parameter += 5;
				Matrix n_xx(count_xx_Parameter, count_xx_Parameter);
				Matrix n_ff(count_ff_Parameter, count_ff_Parameter);
				Matrix n_xf(count_xx_Parameter, count_ff_Parameter);
				Matrix nx(count_xx_Parameter, 1);
				Matrix nf(count_ff_Parameter, 1);
				/*
					| n_xx   n_xf|     |nx|
					|            |   = |  |
					| n_fx   n_ff|     |nf| 
					
					n_xx = H_x' * H_x
					n_ff = H_f' * H_f

					n_xf = H_x' * H_f

					nx   = H_x' * y
					nf   = H_f' * y
				*/
				int count_obs  = 0;      // 观测数据个数
				for(size_t s_i = 0; s_i < dorisArcList.size(); s_i++)
				{
					int count_obs_i = int(dorisArcList[s_i].obsList.size());
					count_obs += count_obs_i;
					int id_Station = dorisArcList[s_i].id_Station;
					for(int s_j = 0; s_j < count_obs_i; s_j++)
					{
						int nObsTime = dorisArcList[s_i].obsList[s_j].nObsTime;
						double w = dynEpochList[nObsTime].mapDatum[id_Station].weight * dorisArcList[s_i].obsList[s_j].robustweight; // 观测权值
						Matrix Matrix_H_xt_t0(1, 3); // t0时刻部分测量方程对位置偏导数
						Matrix Matrix_H_xt_t1(1, 3); // t1时刻部分测量方程对位置偏导数
						Matrix Matrix_H_x(1, count_xx_Parameter); // 测量方程对初始动力学轨道参数偏导数
						Matrix Matrix_H_f(1, count_ff_Parameter); // 测量方程对频率偏差参数和对流层参数的偏导数
						Matrix_H_f.SetElement(0, int(s_i), 1.0 * w);
						double tzdDelay = 0;
                        if(m_podParaDefine.bOnEst_StaTropZenithDelay)
						{
							double h1 = 1 / sin(dynEpochList[nObsTime].mapDatum[id_Station].elevation_sta_t1 * PI / 180.0);
							double h0 = 1 / sin(dynEpochList[nObsTime].mapDatum[id_Station].elevation_sta_t0 * PI / 180.0);
							Matrix_H_f.SetElement(0, count_arc + int(s_i), (h1 - h0) * w);
							tzdDelay = dorisArcList[s_i].zenithDelay * (h1 - h0);
						}
						double  y = dorisArcList[s_i].obsList[s_j].obs  // 距离变化率
							      + dynEpochList[nObsTime].mapDatum[id_Station].obscorrected_value
								  - dorisArcList[s_i].offsetFrequence
								  - tzdDelay;
                        dorisArcList[s_i].obsList[s_j].res = y;
						y = y * w;
						double distance_t0 = sqrt(pow(dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t0.x, 2)
							                    + pow(dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t0.y, 2)
												+ pow(dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t0.z, 2));
						double distance_t1 = sqrt(pow(dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t1.x, 2)
							                    + pow(dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t1.y, 2)
												+ pow(dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t1.z, 2));
						double duration = dynEpochList[nObsTime].mapDatum[id_Station].duration; // 测量方程设计矩阵系数中有一项 1/duration
						Matrix_H_xt_t0.SetElement(0, 0, -w * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t0.x / (distance_t0 * duration));
						Matrix_H_xt_t0.SetElement(0, 1, -w * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t0.y / (distance_t0 * duration));
						Matrix_H_xt_t0.SetElement(0, 2, -w * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t0.z / (distance_t0 * duration));
						Matrix_H_xt_t1.SetElement(0, 0,  w * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t1.x / (distance_t1 * duration));
						Matrix_H_xt_t1.SetElement(0, 1,  w * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t1.y / (distance_t1 * duration));
						Matrix_H_xt_t1.SetElement(0, 2,  w * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t1.z / (distance_t1 * duration));
						for(int s_k = 0; s_k < 6; s_k++)
						{// x y z vx vy vz
							double sum_posvel  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, s_k) * Matrix_H_xt_t0.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, s_k) * Matrix_H_xt_t0.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, s_k) * Matrix_H_xt_t0.GetElement(0, 2);
								   sum_posvel += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, s_k) * Matrix_H_xt_t1.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, s_k) * Matrix_H_xt_t1.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, s_k) * Matrix_H_xt_t1.GetElement(0, 2);
							Matrix_H_x.SetElement(0, s_k, sum_posvel);
						}
						int beginPara = 6;
						if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
						{// 太阳光压
							for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
							{// c_r
								double sum_solar  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k ) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k ) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k ) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_solar += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k ) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k ) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k ) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k, sum_solar);


							}
							beginPara += count_SolarPressureParaList;
						}
						else if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
						{
							for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
							{
								// A_D0
								double sum_A_D0   = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara +  s_k * 9 + 0) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara +  s_k * 9 + 0) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara +  s_k * 9 + 0) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_A_D0  += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara +  s_k * 9 + 0) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara +  s_k * 9 + 0) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara +  s_k * 9 + 0) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k * 9 + 0, sum_A_D0);
								// A_DC
								double sum_A_DC   = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara +  s_k * 9 + 1) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara +  s_k * 9 + 1) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara +  s_k * 9 + 1) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_A_DC  += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara +  s_k * 9 + 1) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara +  s_k * 9 + 1) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara +  s_k * 9 + 1) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k * 9 + 1, sum_A_DC);
                                // A_DS
								double sum_A_DS   = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara +  s_k * 9 + 2) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara +  s_k * 9 + 2) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara +  s_k * 9 + 2) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_A_DS  += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara +  s_k * 9 + 2) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara +  s_k * 9 + 2) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara +  s_k * 9 + 2) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k * 9 + 2, sum_A_DS);
								// A_Y0
								double sum_A_Y0   = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara +  s_k * 9 + 3) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara +  s_k * 9 + 3) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara +  s_k * 9 + 3) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_A_Y0  += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara +  s_k * 9 + 3) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara +  s_k * 9 + 3) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara +  s_k * 9 + 3) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k * 9 + 3, sum_A_Y0);
                                // A_YC
								double sum_A_YC   = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara +  s_k * 9 + 4) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara +  s_k * 9 + 4) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara +  s_k * 9 + 4) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_A_YC  += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara +  s_k * 9 + 4) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara +  s_k * 9 + 4) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara +  s_k * 9 + 4) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k * 9 + 4, sum_A_YC);
                                // A_YS
								double sum_A_YS   = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara +  s_k * 9 + 5) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara +  s_k * 9 + 5) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara +  s_k * 9 + 5) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_A_YS  += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara +  s_k * 9 + 5) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara +  s_k * 9 + 5) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara +  s_k * 9 + 5) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k * 9 + 5, sum_A_YS);
                                // A_X0
								double sum_A_X0   = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara +  s_k * 9 + 6) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara +  s_k * 9 + 6) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara +  s_k * 9 + 6) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_A_X0  += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara +  s_k * 9 + 6) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara +  s_k * 9 + 6) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara +  s_k * 9 + 6) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k * 9 + 6, sum_A_X0);
								// A_XC
								double sum_A_XC   = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara +  s_k * 9 + 7) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara +  s_k * 9 + 7) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara +  s_k * 9 + 7) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_A_XC  += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara +  s_k * 9 + 7) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara +  s_k * 9 + 7) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara +  s_k * 9 + 7) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k * 9 + 7, sum_A_XC);
								// A_XS
								double sum_A_XS   = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara +  s_k * 9 + 8) * Matrix_H_xt_t0.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara +  s_k * 9 + 8) * Matrix_H_xt_t0.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara +  s_k * 9 + 8) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_A_XS  += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara +  s_k * 9 + 8) * Matrix_H_xt_t1.GetElement(0, 0) 
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara +  s_k * 9 + 8) * Matrix_H_xt_t1.GetElement(0, 1)
												  + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara +  s_k * 9 + 8) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara +  s_k * 9 + 8, sum_A_XS);
							}
							beginPara += 9 * count_SolarPressureParaList;
						}
						if(dynamicDatum.bOn_AtmosphereDragAcc) // dynamicDatum.atmosphereDragType == TYPE_ATMOSPHEREDRAG_JACCHIA71
						{// 大气阻力
							for(int s_k = 0; s_k < int(dynamicDatum.atmosphereDragParaList.size()); s_k++)
							{
								double sum_cd  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k) * Matrix_H_xt_t0.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k) * Matrix_H_xt_t0.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_cd += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k) * Matrix_H_xt_t1.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k) * Matrix_H_xt_t1.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara + s_k, sum_cd);
							}
							beginPara += count_AtmosphereDragParaList;
						}
						// 判断在哪一个区间s_k, 不必对s_k后面的区间进行计算, 可以进一步优化1倍的运算效率 ????????
						if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_SPLINE)
						{// 经验力
							for(int s_k = 0; s_k < int(dynamicDatum.empiricalForceParaList.size()); s_k++)
							{
								// 偶数项
								double sum_a0_t  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k * 2 + 0) * Matrix_H_xt_t0.GetElement(0, 0) 
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k * 2 + 0) * Matrix_H_xt_t0.GetElement(0, 1)
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k * 2 + 0) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_a0_t += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k * 2 + 0) * Matrix_H_xt_t1.GetElement(0, 0) 
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k * 2 + 0) * Matrix_H_xt_t1.GetElement(0, 1)
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k * 2 + 0) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara + s_k * 2 + 0, sum_a0_t);
								double sum_a1_t  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k * 2 + 2) * Matrix_H_xt_t0.GetElement(0, 0) 
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k * 2 + 2) * Matrix_H_xt_t0.GetElement(0, 1)
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k * 2 + 2) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_a1_t += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k * 2 + 2) * Matrix_H_xt_t1.GetElement(0, 0) 
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k * 2 + 2) * Matrix_H_xt_t1.GetElement(0, 1)
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k * 2 + 2) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara + s_k * 2 + 2, sum_a1_t);
								// 奇数项
								double sum_a0_n  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k * 2 + 1) * Matrix_H_xt_t0.GetElement(0, 0) 
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k * 2 + 1) * Matrix_H_xt_t0.GetElement(0, 1)
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k * 2 + 1) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_a0_n += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k * 2 + 1) * Matrix_H_xt_t1.GetElement(0, 0) 
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k * 2 + 1) * Matrix_H_xt_t1.GetElement(0, 1)
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k * 2 + 1) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara + s_k * 2 + 1, sum_a0_n);
								double sum_a1_n  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k * 2 + 3) * Matrix_H_xt_t0.GetElement(0, 0) 
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k * 2 + 3) * Matrix_H_xt_t0.GetElement(0, 1)
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k * 2 + 3) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_a1_n += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k * 2 + 3) * Matrix_H_xt_t1.GetElement(0, 0) 
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k * 2 + 3) * Matrix_H_xt_t1.GetElement(0, 1)
											     + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k * 2 + 3) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara + s_k * 2 + 3, sum_a1_n);
							}
							beginPara += 2 * (count_EmpiricalForceParaList + 1);
						}
						else if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
						{// 经验力
							for(int s_k = 0; s_k < int(dynamicDatum.empiricalForceParaList.size()); s_k++)
							{
								double sum_ct  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k * 4 + 0) * Matrix_H_xt_t0.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k * 4 + 0) * Matrix_H_xt_t0.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k * 4 + 0) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_ct += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k * 4 + 0) * Matrix_H_xt_t1.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k * 4 + 0) * Matrix_H_xt_t1.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k * 4 + 0) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara + s_k * 4 + 0, sum_ct);
								double sum_st  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k * 4 + 1) * Matrix_H_xt_t0.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k * 4 + 1) * Matrix_H_xt_t0.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k * 4 + 1) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_st += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k * 4 + 1) * Matrix_H_xt_t1.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k * 4 + 1) * Matrix_H_xt_t1.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k * 4 + 1) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara + s_k * 4 + 1, sum_st);
								double sum_cn  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k * 4 + 2) * Matrix_H_xt_t0.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k * 4 + 2) * Matrix_H_xt_t0.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k * 4 + 2) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_cn += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k * 4 + 2) * Matrix_H_xt_t1.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k * 4 + 2) * Matrix_H_xt_t1.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k * 4 + 2) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara + s_k * 4 + 2, sum_cn);
								double sum_sn  = dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(0, beginPara + s_k * 4 + 3) * Matrix_H_xt_t0.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(1, beginPara + s_k * 4 + 3) * Matrix_H_xt_t0.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t0.GetElement(2, beginPara + s_k * 4 + 3) * Matrix_H_xt_t0.GetElement(0, 2);
									   sum_sn += dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(0, beginPara + s_k * 4 + 3) * Matrix_H_xt_t1.GetElement(0, 0) 
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(1, beginPara + s_k * 4 + 3) * Matrix_H_xt_t1.GetElement(0, 1)
											   + dynEpochList[nObsTime].mapDatum[id_Station].interpRtPartial_t1.GetElement(2, beginPara + s_k * 4 + 3) * Matrix_H_xt_t1.GetElement(0, 2);
								Matrix_H_x.SetElement(0, beginPara + s_k * 4 + 3, sum_sn);
							}
							beginPara += 4 * count_EmpiricalForceParaList;
						}
						if(m_podParaDefine.bOnEst_ERP)
						{
							// t0
							Matrix matECEFPos(3, 1);
							matECEFPos.SetElement(0, 0, m_ppDorisStationPos[id_Station][0]);
							matECEFPos.SetElement(1, 0, m_ppDorisStationPos[id_Station][1]);
							matECEFPos.SetElement(2, 0, m_ppDorisStationPos[id_Station][2]);
							Matrix matEPECEFPos(3, 1);
							Matrix matPR_NR, matER, matEP, matER_DOT;
							m_TimeCoordConvert.Matrix_J2000_ECEF(TimeCoordConvert::TAI2GPST(dynEpochList[nObsTime].mapDatum[id_Station].t0_sta_transmit),
								                                 matPR_NR, matER, matEP, matER_DOT);
							Matrix matEst_EP, matEst_ER;
							eopEstPara.getEst_EOP(dynEpochList[nObsTime].mapDatum[id_Station].t0_sta_transmit, matEst_EP, matEst_ER);
							matEP = matEst_EP * matEP;
                            matER = matEst_ER * matER;
							matEPECEFPos = matEP * matECEFPos;
							Matrix matM_ut1 = matER * matPR_NR;
							Matrix matM_xpyp= matEP * matM_ut1;
							matM_ut1 = matM_ut1.Transpose();
							matM_xpyp = matM_xpyp.Transpose();
							double spanSeconds_t0 = dynEpochList[nObsTime].mapDatum[id_Station].t0_sta_transmit - eopEstPara.t0_xpyput1;
							Matrix matHP_t0_xp(3, 1);
							Matrix matHP_t0_xpdot(3, 1);
							Matrix matHP_t0_yp(3, 1);
							Matrix matHP_t0_ypdot(3, 1);
							Matrix matHP_t0_ut1(3, 1);
							matHP_t0_xp.SetElement(0, 0, matM_xpyp.GetElement(0,2) * matECEFPos.GetElement(0, 0) - matM_xpyp.GetElement(0,0) * matECEFPos.GetElement(2, 0));
							matHP_t0_xp.SetElement(1, 0, matM_xpyp.GetElement(1,2) * matECEFPos.GetElement(0, 0) - matM_xpyp.GetElement(1,0) * matECEFPos.GetElement(2, 0));
							matHP_t0_xp.SetElement(2, 0, matM_xpyp.GetElement(2,2) * matECEFPos.GetElement(0, 0) - matM_xpyp.GetElement(2,0) * matECEFPos.GetElement(2, 0));
							matHP_t0_xpdot = matHP_t0_xp * spanSeconds_t0;
							matHP_t0_yp.SetElement(0, 0, matM_xpyp.GetElement(0,1) * matECEFPos.GetElement(2, 0) - matM_xpyp.GetElement(0,2) * matECEFPos.GetElement(1, 0));
							matHP_t0_yp.SetElement(1, 0, matM_xpyp.GetElement(1,1) * matECEFPos.GetElement(2, 0) - matM_xpyp.GetElement(1,2) * matECEFPos.GetElement(1, 0));
							matHP_t0_yp.SetElement(2, 0, matM_xpyp.GetElement(2,1) * matECEFPos.GetElement(2, 0) - matM_xpyp.GetElement(2,2) * matECEFPos.GetElement(1, 0));
							matHP_t0_ypdot = matHP_t0_yp * spanSeconds_t0;
							matHP_t0_ut1.SetElement(0, 0, matM_ut1.GetElement(0,1) * matEPECEFPos.GetElement(0, 0) - matM_ut1.GetElement(0,0) * matEPECEFPos.GetElement(1, 0));
							matHP_t0_ut1.SetElement(1, 0, matM_ut1.GetElement(1,1) * matEPECEFPos.GetElement(0, 0) - matM_ut1.GetElement(1,0) * matEPECEFPos.GetElement(1, 0));
							matHP_t0_ut1.SetElement(2, 0, matM_ut1.GetElement(2,1) * matEPECEFPos.GetElement(0, 0) - matM_ut1.GetElement(2,0) * matEPECEFPos.GetElement(1, 0));
							matHP_t0_ut1 = matHP_t0_ut1 * spanSeconds_t0;
                            // t1
							m_TimeCoordConvert.Matrix_J2000_ECEF(TimeCoordConvert::TAI2GPST(dynEpochList[nObsTime].mapDatum[id_Station].t1_sta_transmit),
								                                 matPR_NR, matER, matEP, matER_DOT);
							eopEstPara.getEst_EOP(dynEpochList[nObsTime].mapDatum[id_Station].t1_sta_transmit, matEst_EP, matEst_ER);
							matEP = matEst_EP * matEP;
                            matER = matEst_ER * matER;
							matEPECEFPos = matEP * matECEFPos;
							matM_ut1 = matER * matPR_NR;
							matM_xpyp= matEP * matM_ut1;
							matM_ut1 = matM_ut1.Transpose();
							matM_xpyp = matM_xpyp.Transpose();
							double spanSeconds_t1 = dynEpochList[nObsTime].mapDatum[id_Station].t1_sta_transmit - eopEstPara.t0_xpyput1;
							Matrix matHP_t1_xp(3, 1);
							Matrix matHP_t1_xpdot(3, 1);
							Matrix matHP_t1_yp(3, 1);
							Matrix matHP_t1_ypdot(3, 1);
							Matrix matHP_t1_ut1(3, 1);
							matHP_t1_xp.SetElement(0, 0, matM_xpyp.GetElement(0,2) * matECEFPos.GetElement(0, 0) - matM_xpyp.GetElement(0,0) * matECEFPos.GetElement(2, 0));
							matHP_t1_xp.SetElement(1, 0, matM_xpyp.GetElement(1,2) * matECEFPos.GetElement(0, 0) - matM_xpyp.GetElement(1,0) * matECEFPos.GetElement(2, 0));
							matHP_t1_xp.SetElement(2, 0, matM_xpyp.GetElement(2,2) * matECEFPos.GetElement(0, 0) - matM_xpyp.GetElement(2,0) * matECEFPos.GetElement(2, 0));
							matHP_t1_xpdot = matHP_t1_xp * spanSeconds_t1;
							matHP_t1_yp.SetElement(0, 0, matM_xpyp.GetElement(0,1) * matECEFPos.GetElement(2, 0) - matM_xpyp.GetElement(0,2) * matECEFPos.GetElement(1, 0));
							matHP_t1_yp.SetElement(1, 0, matM_xpyp.GetElement(1,1) * matECEFPos.GetElement(2, 0) - matM_xpyp.GetElement(1,2) * matECEFPos.GetElement(1, 0));
							matHP_t1_yp.SetElement(2, 0, matM_xpyp.GetElement(2,1) * matECEFPos.GetElement(2, 0) - matM_xpyp.GetElement(2,2) * matECEFPos.GetElement(1, 0));
							matHP_t1_ypdot = matHP_t1_yp * spanSeconds_t1;
							matHP_t1_ut1.SetElement(0, 0, matM_ut1.GetElement(0,1) * matEPECEFPos.GetElement(0, 0) - matM_ut1.GetElement(0,0) * matEPECEFPos.GetElement(1, 0));
							matHP_t1_ut1.SetElement(1, 0, matM_ut1.GetElement(1,1) * matEPECEFPos.GetElement(0, 0) - matM_ut1.GetElement(1,0) * matEPECEFPos.GetElement(1, 0));
							matHP_t1_ut1.SetElement(2, 0, matM_ut1.GetElement(2,1) * matEPECEFPos.GetElement(0, 0) - matM_ut1.GetElement(2,0) * matEPECEFPos.GetElement(1, 0));
							matHP_t1_ut1 = matHP_t1_ut1 * spanSeconds_t1;

							Matrix matLos_t0(1, 3), matLos_t1(1, 3);
							matLos_t0.SetElement(0, 0, -1.0 * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t0.x / distance_t0);
							matLos_t0.SetElement(0, 1, -1.0 * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t0.y / distance_t0);
							matLos_t0.SetElement(0, 2, -1.0 * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t0.z / distance_t0);
							matLos_t1.SetElement(0, 0, -1.0 * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t1.x / distance_t1);
							matLos_t1.SetElement(0, 1, -1.0 * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t1.y / distance_t1);
							matLos_t1.SetElement(0, 2, -1.0 * dynEpochList[nObsTime].mapDatum[id_Station].vecLos_t1.z / distance_t1);
							Matrix_H_x.SetElement(0, count_DynParameter + 0, ((matLos_t1 *    matHP_t1_xp).GetElement(0, 0) - (matLos_t0 *    matHP_t0_xp).GetElement(0, 0)) * factor_eop);
							Matrix_H_x.SetElement(0, count_DynParameter + 1, ((matLos_t1 * matHP_t1_xpdot).GetElement(0, 0) - (matLos_t0 * matHP_t0_xpdot).GetElement(0, 0)) * factor_eop);
							Matrix_H_x.SetElement(0, count_DynParameter + 2, ((matLos_t1 *    matHP_t1_yp).GetElement(0, 0) - (matLos_t0 *    matHP_t0_yp).GetElement(0, 0)) * factor_eop);
							Matrix_H_x.SetElement(0, count_DynParameter + 3, ((matLos_t1 * matHP_t1_ypdot).GetElement(0, 0) - (matLos_t0 * matHP_t0_ypdot).GetElement(0, 0)) * factor_eop);
							Matrix_H_x.SetElement(0, count_DynParameter + 4, ((matLos_t1 *   matHP_t1_ut1).GetElement(0, 0) - (matLos_t0 *   matHP_t0_ut1).GetElement(0, 0)) * factor_eop);
						}
						n_xx = n_xx +  Matrix_H_x.Transpose() * Matrix_H_x;
						//n_ff = n_ff + Matrix_H_f.Transpose() * Matrix_H_f;
						n_ff.SetElement(int(s_i), int(s_i), n_ff.GetElement(int(s_i), int(s_i)) + Matrix_H_f.GetElement(0, int(s_i)) * Matrix_H_f.GetElement(0, int(s_i)));
						nx = nx + Matrix_H_x.Transpose() * y;
						//nf = nf + Matrix_H_f.Transpose() * y;
						nf.SetElement(int(s_i), 0, nf.GetElement(int(s_i), 0) + Matrix_H_f.GetElement(0, int(s_i)) * y);
						for(int s_k = 0; s_k < count_xx_Parameter; s_k++)
						{// Matrix_H_f 只有第s_i个元素非零
							n_xf.SetElement(s_k, int(s_i), n_xf.GetElement(s_k, int(s_i)) + Matrix_H_x.GetElement(0, s_k) * Matrix_H_f.GetElement(0, int(s_i)));
						}
						if(m_podParaDefine.bOnEst_StaTropZenithDelay)
						{
							n_ff.SetElement(count_arc + int(s_i), count_arc + int(s_i), n_ff.GetElement(count_arc + int(s_i), count_arc + int(s_i)) + Matrix_H_f.GetElement(0, count_arc + int(s_i)) * Matrix_H_f.GetElement(0, count_arc + int(s_i)));
							n_ff.SetElement(count_arc + int(s_i), int(s_i), n_ff.GetElement(count_arc + int(s_i), int(s_i)) + Matrix_H_f.GetElement(0, count_arc + int(s_i)) * Matrix_H_f.GetElement(0, int(s_i)));
							n_ff.SetElement(int(s_i), count_arc + int(s_i), n_ff.GetElement(int(s_i), count_arc + int(s_i)) + Matrix_H_f.GetElement(0, count_arc + int(s_i)) * Matrix_H_f.GetElement(0, int(s_i)));
							nf.SetElement(count_arc + int(s_i), 0, nf.GetElement(count_arc + int(s_i), 0) + Matrix_H_f.GetElement(0, count_arc + int(s_i)) * y);
							for(int s_k = 0; s_k < count_xx_Parameter; s_k++)
							{// Matrix_H_f 只有第s_i个元素非零
								n_xf.SetElement(s_k, count_arc + int(s_i), n_xf.GetElement(s_k, count_arc + int(s_i)) + Matrix_H_x.GetElement(0, s_k) * Matrix_H_f.GetElement(0, count_arc + int(s_i)));
							}
						}
					}
					// 增加一个伪方程: dorisArcList[s_i].zenithDelay = 0 + e, 其对法方程的贡献仅在对角线元素上
					if(m_podParaDefine.bOnEst_StaTropZenithDelay)
					{
						double weight_TRO = m_podParaDefine.apriorityRms_obs / m_podParaDefine.apriorityRms_TZD;
						n_ff.SetElement(count_arc + int(s_i), count_arc + int(s_i), n_ff.GetElement(count_arc + int(s_i), count_arc + int(s_i)) + weight_TRO * weight_TRO);
					}
				}
				// 频率估计结果
				/*FILE* pFile_xx = fopen("c:\\n_xx.txt", "w+");
				fprintf(pFile_xx, "%s \n\n\n", n_xx.ToString().c_str());
				fclose(pFile_xx);*/
				if(m_podParaDefine.bOnEst_ERP)
				{// 先验约束, 伪方程
					double weight_xp = m_podParaDefine.apriorityRms_obs / m_podParaDefine.apriorityRms_xp;
					n_xx.SetElement(count_DynParameter + 0, count_DynParameter + 0, n_xx.GetElement(count_DynParameter + 0, count_DynParameter + 0) + weight_xp * weight_xp * factor_eop * factor_eop);
					double weight_xpDot = m_podParaDefine.apriorityRms_obs / m_podParaDefine.apriorityRms_xpDot;
					n_xx.SetElement(count_DynParameter + 1, count_DynParameter + 1, n_xx.GetElement(count_DynParameter + 1, count_DynParameter + 1) + weight_xpDot * weight_xpDot * factor_eop * factor_eop);
					double weight_yp = m_podParaDefine.apriorityRms_obs / m_podParaDefine.apriorityRms_yp;
					n_xx.SetElement(count_DynParameter + 2, count_DynParameter + 2, n_xx.GetElement(count_DynParameter + 2, count_DynParameter + 2) + weight_yp * weight_yp * factor_eop * factor_eop);
					double weight_ypDot = m_podParaDefine.apriorityRms_obs / m_podParaDefine.apriorityRms_ypDot;
					n_xx.SetElement(count_DynParameter + 3, count_DynParameter + 3, n_xx.GetElement(count_DynParameter + 3, count_DynParameter + 3) + weight_ypDot * weight_ypDot * factor_eop * factor_eop);
					double weight_ut1Dot = m_podParaDefine.apriorityRms_obs / m_podParaDefine.apriorityRms_ut1Dot;
					n_xx.SetElement(count_DynParameter + 4, count_DynParameter + 4, n_xx.GetElement(count_DynParameter + 4, count_DynParameter + 4) + weight_ut1Dot * weight_ut1Dot * factor_eop * factor_eop);
				}
				Matrix n_xx_inv = n_xx.Inv_Ssgj();
				Matrix n_fx_xx_inv = n_xf.Transpose() * n_xx_inv;
                Matrix n_fx_xx_inv_xf = n_fx_xx_inv * n_xf;
				Matrix matdf  = (n_ff - n_fx_xx_inv_xf).Inv_Ssgj() * (nf - n_fx_xx_inv * nx);
				// 动力学轨道改进结果
				Matrix matdx = n_xx_inv * (nx - n_xf * matdf);
				// 计算轨道改进量
				dynamicDatum.X0.x  += matdx.GetElement(0,0);
				dynamicDatum.X0.y  += matdx.GetElement(1,0);
				dynamicDatum.X0.z  += matdx.GetElement(2,0);
				dynamicDatum.X0.vx += matdx.GetElement(3,0);
				dynamicDatum.X0.vy += matdx.GetElement(4,0);
				dynamicDatum.X0.vz += matdx.GetElement(5,0);
				int beginPara = 6;
				if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
				{// 太阳光压
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
						dynamicDatum.solarPressureParaList[s_k].Cr +=  matdx.GetElement(beginPara + s_k, 0);
					beginPara += count_SolarPressureParaList;
				}
				else if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
				{
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
					{
						dynamicDatum.solarPressureParaList[s_k].D0  += matdx.GetElement(beginPara + s_k * 9 + 0, 0);
						dynamicDatum.solarPressureParaList[s_k].DC1 += matdx.GetElement(beginPara + s_k * 9 + 1, 0);
						dynamicDatum.solarPressureParaList[s_k].DS1 += matdx.GetElement(beginPara + s_k * 9 + 2, 0);
						dynamicDatum.solarPressureParaList[s_k].Y0  += matdx.GetElement(beginPara + s_k * 9 + 3, 0);
						dynamicDatum.solarPressureParaList[s_k].YC1 += matdx.GetElement(beginPara + s_k * 9 + 4, 0);
						dynamicDatum.solarPressureParaList[s_k].YS1 += matdx.GetElement(beginPara + s_k * 9 + 5, 0);
						dynamicDatum.solarPressureParaList[s_k].B0  += matdx.GetElement(beginPara + s_k * 9 + 6, 0);
						dynamicDatum.solarPressureParaList[s_k].BC1 += matdx.GetElement(beginPara + s_k * 9 + 7, 0);
						dynamicDatum.solarPressureParaList[s_k].BS1 += matdx.GetElement(beginPara + s_k * 9 + 8, 0);
						/*
						dynamicDatum.solarPressureParaList[s_k].A_D0 +=  matdx.GetElement(beginPara + s_k * 9 + 0, 0);
						dynamicDatum.solarPressureParaList[s_k].A_DC +=  matdx.GetElement(beginPara + s_k * 9 + 1, 0);
						dynamicDatum.solarPressureParaList[s_k].A_DS +=  matdx.GetElement(beginPara + s_k * 9 + 2, 0);
						dynamicDatum.solarPressureParaList[s_k].A_Y0 +=  matdx.GetElement(beginPara + s_k * 9 + 3, 0);
						dynamicDatum.solarPressureParaList[s_k].A_YC +=  matdx.GetElement(beginPara + s_k * 9 + 4, 0);
						dynamicDatum.solarPressureParaList[s_k].A_YS +=  matdx.GetElement(beginPara + s_k * 9 + 5, 0);
						dynamicDatum.solarPressureParaList[s_k].A_X0 +=  matdx.GetElement(beginPara + s_k * 9 + 6, 0);
						dynamicDatum.solarPressureParaList[s_k].A_XC +=  matdx.GetElement(beginPara + s_k * 9 + 7, 0);
						dynamicDatum.solarPressureParaList[s_k].A_XS +=  matdx.GetElement(beginPara + s_k * 9 + 8, 0);*/
					}
					beginPara += 9 * count_SolarPressureParaList;
				}
				if(dynamicDatum.bOn_AtmosphereDragAcc) // dynamicDatum.atmosphereDragType == TYPE_ATMOSPHEREDRAG_JACCHIA71
				{// 大气阻力
					for(int s_k = 0; s_k < int(dynamicDatum.atmosphereDragParaList.size()); s_k++)
						dynamicDatum.atmosphereDragParaList[s_k].Cd +=  matdx.GetElement(beginPara + s_k, 0);
					beginPara += count_AtmosphereDragParaList;
				}
				if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
				{// 经验力
					for(int s_k = 0; s_k < int(dynamicDatum.empiricalForceParaList.size()); s_k++)
					{
						dynamicDatum.empiricalForceParaList[s_k].cos_T += matdx.GetElement(beginPara + s_k * 4 + 0, 0);
						dynamicDatum.empiricalForceParaList[s_k].sin_T += matdx.GetElement(beginPara + s_k * 4 + 1, 0);
						dynamicDatum.empiricalForceParaList[s_k].cos_N += matdx.GetElement(beginPara + s_k * 4 + 2, 0);
						dynamicDatum.empiricalForceParaList[s_k].sin_N += matdx.GetElement(beginPara + s_k * 4 + 3, 0);
					}
					beginPara += 4 * count_EmpiricalForceParaList;
				}
				if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_SPLINE)
				{
					for(int s_k = 0; s_k < int(dynamicDatum.empiricalForceParaList.size()); s_k++)
					{
						dynamicDatum.empiricalForceParaList[s_k].a0_T += matdx.GetElement(beginPara + s_k * 2 + 0, 0);
						dynamicDatum.empiricalForceParaList[s_k].a1_T += matdx.GetElement(beginPara + s_k * 2 + 2, 0);
						dynamicDatum.empiricalForceParaList[s_k].a0_N += matdx.GetElement(beginPara + s_k * 2 + 1, 0);
						dynamicDatum.empiricalForceParaList[s_k].a1_N += matdx.GetElement(beginPara + s_k * 2 + 3, 0);
					}
					beginPara += 2 * (count_EmpiricalForceParaList + 1);
				}
				if(m_podParaDefine.bOnEst_ERP)
				{
					eopEstPara.xp     += matdx.GetElement(count_DynParameter + 0, 0) * factor_eop;
					eopEstPara.xpDot  += matdx.GetElement(count_DynParameter + 1, 0) * factor_eop;
					eopEstPara.yp     += matdx.GetElement(count_DynParameter + 2, 0) * factor_eop;
					eopEstPara.ypDot  += matdx.GetElement(count_DynParameter + 3, 0) * factor_eop;
					eopEstPara.ut1Dot += matdx.GetElement(count_DynParameter + 4, 0) * factor_eop;
				}
				// 计算频率偏差改进量
				for(int s_i = 0; s_i < int(dorisArcList.size()); s_i++)
				{
					dorisArcList[s_i].offsetFrequence += matdf.GetElement(s_i, 0);
				}
				// 计算对流层改进量
				if(m_podParaDefine.bOnEst_StaTropZenithDelay)
				{
					for(int s_i = 0; s_i < int(dorisArcList.size()); s_i++)
						dorisArcList[s_i].zenithDelay += matdf.GetElement(count_arc + s_i, 0);
				}
				// 记录轨道改进结果
				pFitFile = fopen(dynamicDatumFilePath, "w+");
				fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
				fprintf(pFitFile, "%3d. EOP  XP   (mas)    %20.4f%10.4f%20.4f\n",  1,0.0,                     eopEstPara.xp * 180 / PI * 3600000,                      eopEstPara.xp * 180 / PI * 3600000);
				fprintf(pFitFile, "%3d. EOP  XPDOT(mas/d)  %20.4f%10.4f%20.4f\n",  2,0.0,                     eopEstPara.xpDot * 86400.0  * 180 / PI * 3600000,        eopEstPara.xpDot * 86400.0  * 180 / PI * 3600000);
				fprintf(pFitFile, "%3d. EOP  YP   (mas)    %20.4f%10.4f%20.4f\n",  3,0.0,                     eopEstPara.yp * 180 / PI * 3600000,                      eopEstPara.yp * 180 / PI * 3600000);
				fprintf(pFitFile, "%3d. EOP  YPDOT(mas/d)  %20.4f%10.4f%20.4f\n",  4,0.0,                     eopEstPara.ypDot * 86400.0  * 180 / PI * 3600000,        eopEstPara.ypDot * 86400.0  * 180 / PI * 3600000);
				fprintf(pFitFile, "%3d. EOP  UT   (ms)     %20.4f%10.4f%20.4f\n",  5,0.0,                     eopEstPara.ut1 * 86400.0 / (2 * PI) * 1.0E+3,            eopEstPara.ut1 * 86400.0 / (2 * PI) * 1.0E+3);
				fprintf(pFitFile, "%3d. EOP  UTDOT(ms/d)   %20.4f%10.4f%20.4f\n",  6,0.0,                     eopEstPara.ut1Dot * 86400.0 / (2 * PI) * 1.0E+3 * 86400, eopEstPara.ut1Dot * 86400.0 / (2 * PI) * 1.0E+3 * 86400);
				fprintf(pFitFile, "%3d.      X    (m)      %20.4f%10.4f%20.4f\n",  7,dynamicDatum_Init.X0.x,  dynamicDatum.X0.x  - dynamicDatum_Init.X0.x,             dynamicDatum.X0.x);
				fprintf(pFitFile, "%3d.      Y    (m)      %20.4f%10.4f%20.4f\n",  8,dynamicDatum_Init.X0.y,  dynamicDatum.X0.y  - dynamicDatum_Init.X0.y,             dynamicDatum.X0.y);
				fprintf(pFitFile, "%3d.      Z    (m)      %20.4f%10.4f%20.4f\n",  9,dynamicDatum_Init.X0.z,  dynamicDatum.X0.z  - dynamicDatum_Init.X0.z,             dynamicDatum.X0.z);
				fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f%10.4f%20.4f\n", 10,dynamicDatum_Init.X0.vx, dynamicDatum.X0.vx - dynamicDatum_Init.X0.vx,            dynamicDatum.X0.vx);
				fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f%10.4f%20.4f\n", 11,dynamicDatum_Init.X0.vy, dynamicDatum.X0.vy - dynamicDatum_Init.X0.vy,            dynamicDatum.X0.vy);
				fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f%10.4f%20.4f\n", 12,dynamicDatum_Init.X0.vz, dynamicDatum.X0.vz - dynamicDatum_Init.X0.vz,            dynamicDatum.X0.vz);
				k_Parameter = 12;
				if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
				{
					for(size_t s_i = 0; s_i < dynamicDatum.solarPressureParaList.size(); s_i++)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   CR            %20.4f%10.4f%20.4f\n", k_Parameter ,
																				           s_i+1,
																				           dynamicDatum_Init.solarPressureParaList[s_i].Cr,
																						   dynamicDatum.solarPressureParaList[s_i].Cr - dynamicDatum_Init.solarPressureParaList[s_i].Cr,
																						   dynamicDatum.solarPressureParaList[s_i].Cr);
					}
				}
				if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
				{
					for(size_t s_i = 0; s_i < dynamicDatum.solarPressureParaList.size(); s_i++)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   D0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].D0 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].D0 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   DCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].DC1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].DC1 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].DC1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].DC1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   DSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].DS1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].DS1 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].DS1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].DS1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   Y0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].Y0 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].Y0 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   YCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].YC1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].YC1 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].YC1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].YC1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   YSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].YS1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].YS1 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].YS1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].YS1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   B0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].B0 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].B0 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].B0 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].B0 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   BCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].BC1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].BC1 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].BC1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].BC1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   BSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].BS1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].BS1 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].BS1 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].BS1 * 1.0E+7);
					}
				}
				if(dynamicDatum.bOn_AtmosphereDragAcc) //  dynamicDatum.atmosphereDragType == TYPE_ATMOSPHEREDRAG_JACCHIA71
				{
					for(size_t s_i = 0; s_i < dynamicDatum.atmosphereDragParaList.size(); s_i++)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   CD            %20.4f%10.4f%20.4f\n",  k_Parameter,
																				            s_i+1,
																							dynamicDatum_Init.atmosphereDragParaList[s_i].Cd,
																				            dynamicDatum.atmosphereDragParaList[s_i].Cd - dynamicDatum_Init.atmosphereDragParaList[s_i].Cd,
																							dynamicDatum.atmosphereDragParaList[s_i].Cd);
					}
				}
				if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
				{
					for(size_t s_i = 0; s_i < dynamicDatum.empiricalForceParaList.size(); s_i++)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   TCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																				           s_i+1,
																				           dynamicDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].cos_T * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].cos_T * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   TSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																				           s_i+1,
																				           dynamicDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].sin_T * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].sin_T * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   NCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																				           s_i+1,
																				           dynamicDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].cos_N * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].cos_N * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   NSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																				           s_i+1,
																				           dynamicDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].sin_N * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].sin_N * 1.0E+7);
					}
				}
				if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_SPLINE)
				{
					size_t s_i;
					for(s_i = 0; s_i < dynamicDatum.empiricalForceParaList.size(); s_i++)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   A0_T (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																				           s_i+1,
																				           dynamicDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].a0_T * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].a0_T * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   A0_N (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																				           s_i+1,
																				           dynamicDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].a0_N * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																						   dynamicDatum.empiricalForceParaList[s_i].a0_N * 1.0E+7);
					}
					s_i = dynamicDatum.empiricalForceParaList.size() - 1;
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   A0_T (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																			           s_i+2,
																			           dynamicDatum_Init.empiricalForceParaList[s_i].a1_T * 1.0E+7,
																					   dynamicDatum.empiricalForceParaList[s_i].a1_T * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a1_T * 1.0E+7,
																					   dynamicDatum.empiricalForceParaList[s_i].a1_T * 1.0E+7);
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   A0_N (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																			           s_i+2,
																			           dynamicDatum_Init.empiricalForceParaList[s_i].a1_N * 1.0E+7,
																					   dynamicDatum.empiricalForceParaList[s_i].a1_N * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a1_N * 1.0E+7,
																					   dynamicDatum.empiricalForceParaList[s_i].a1_N * 1.0E+7);
				}
				for(int s_i = 0; s_i < int(dorisArcList.size()); s_i++)
				{
					k_Parameter++;
					fprintf(pFitFile, "%3d. %4s FREQ (m)      %20.4f%10.4f%20.4f\n", k_Parameter,
																				     dorisStationId2String(dorisArcList[s_i].id_Station).c_str(),
																					 0.0,
																				     dorisArcList[s_i].offsetFrequence,
																				     dorisArcList[s_i].offsetFrequence);
				}
				if(m_podParaDefine.bOnEst_StaTropZenithDelay)
				{
					for(int s_i = 0; s_i < int(dorisArcList.size()); s_i++)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %4s TR0  (m)      %20.4f%10.4f%20.4f\n", k_Parameter,
																						 dorisStationId2String(dorisArcList[s_i].id_Station).c_str(),
																						 0.0,
																						 dorisArcList[s_i].zenithDelay,
																						 dorisArcList[s_i].zenithDelay);
					}
				}
				fclose(pFitFile);
				// 判断收敛条件
				double max_adjust_pos = 0;
				for(int i = 0; i < 3; i++)
					max_adjust_pos = max(max_adjust_pos, fabs(matdx.GetElement(i, 0)));
				printf("max_adjust_pos =  %10.4f \n", max_adjust_pos);
				if((max_adjust_pos <= 2.5E-3 || k >= 3)  || num_after_residual_edit > 0) 
				{
					if(flag_robust == false && num_after_residual_edit == 0 && bResEdit)
					{
						flag_robust = true; 
						continue;
					}
					else
					{
	                    flag_break = true;
					}
				}
			}
			if(!bForecast) // 不进行轨道预报
				return result;
			// 进行轨道预报
			TDT t0_tdt = TimeCoordConvert::TAI2TDT(t0_forecast);
			TDT t1_tdt = TimeCoordConvert::TAI2TDT(t1_forecast);
			if(result)
			{
				vector<TimePosVel> orbitlist_ac;
				vector<Matrix> matRtPartiallist_ac;
				// 倒向积分, 积分区间 [para.T0, T_End   + h * 4], 为保证插值精度向两端进行扩展
				vector<TimePosVel> backwardOrbitlist_ac; 
			    vector<TimePosVel> forwardOrbitlist_ac; 
                double h = 30.0;
				if(t0_tdt - dynamicDatum.T0 < h * 8.0)
				{
					AdamsCowell(dynamicDatum, t0_tdt - h * 8.0, backwardOrbitlist_ac, matRtPartiallist_ac, -h, 11);
					for(size_t s_i = backwardOrbitlist_ac.size() - 1; s_i > 0; s_i--)
						orbitlist_ac.push_back(backwardOrbitlist_ac[s_i]);
				}
				if(t1_tdt - dynamicDatum.T0 > h * 8.0)
				{
					AdamsCowell(dynamicDatum, t1_tdt + h * 8.0, forwardOrbitlist_ac, matRtPartiallist_ac, h, 11);
					for(size_t s_i = 0; s_i < forwardOrbitlist_ac.size(); s_i++)
						orbitlist_ac.push_back(forwardOrbitlist_ac[s_i]);
				}
				forecastOrbList.clear();
				int k = 0;
				double span = t1_tdt - t0_tdt;
				while(k * interval < span)             
				{
					TimePosVel point;
					point.t = t0_tdt + k * interval;
					forecastOrbList.push_back(point);
					k++;
				}
				size_t count_ac = orbitlist_ac.size();
				const int nlagrange = 8; 
				if(count_ac < nlagrange) // 如果数据点个数小于nlagrange返回，要求弧段长度 > h * nlagrange = 4分钟
					return false;
				for(size_t s_i = 0; s_i < forecastOrbList.size(); s_i++)
				{
					double spanSecond_t = forecastOrbList[s_i].t - orbitlist_ac[0].t; 
					int nLeftPos  = int(spanSecond_t / h);      
					int nLeftNum  = int(floor(nlagrange / 2.0));    
					int nRightNum = int(ceil(nlagrange / 2.0));
					int nBegin, nEnd;                                                    // 位于区间[0, nCount_AC-1]
					if(nLeftPos - nLeftNum + 1 < 0)                                      // nEnd - nBegin = nLagrange - 1 
					{
						nBegin = 0;
						nEnd   = nlagrange - 1;
					}
					else if(nLeftPos + nRightNum >= int(count_ac))
					{
						nBegin = int(count_ac) - nlagrange;
						nEnd   = int(count_ac) - 1;
					}
					else
					{
						nBegin = nLeftPos - nLeftNum + 1;
						nEnd   = nLeftPos + nRightNum;
					}
					// 轨道点
					TimePosVel interpOrbit; // 所有元素的参考时刻均相同
					interpOrbit.t = forecastOrbList[s_i].t;
					double *x = new double [nlagrange];
					double *y = new double [nlagrange];
					for(int i = nBegin; i <= nEnd; i++)
						x[i - nBegin] = orbitlist_ac[i].t - orbitlist_ac[0].t; // 参考相对时间点
					// X
					for(int i = nBegin; i <= nEnd; i++)
						y[i - nBegin] = orbitlist_ac[i].pos.x;
					InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.pos.x);
					// Y
					for(int i = nBegin; i <= nEnd; i++)
						y[i - nBegin] = orbitlist_ac[i].pos.y;
					InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.pos.y);
					// Z
					for(int i = nBegin; i <= nEnd; i++)
						y[i - nBegin] = orbitlist_ac[i].pos.z;
					InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.pos.z);
					// vx
					for(int i = nBegin; i <= nEnd; i++)
						y[i - nBegin] = orbitlist_ac[i].vel.x;
					InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.x);
					// vy
					for(int i = nBegin; i <= nEnd; i++)
						y[i - nBegin] = orbitlist_ac[i].vel.y;
					InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.y);
					// vz
					for(int i = nBegin; i <= nEnd; i++)
						y[i - nBegin] = orbitlist_ac[i].vel.z;
					InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.z);
					forecastOrbList[s_i] = interpOrbit;
					delete x;
				    delete y;
				}
				// 转换到地球固定坐标系, 坐标系: ITRF 系, 时间: GPS
				for(size_t s_i = 0; s_i < forecastOrbList.size(); s_i++)
				{
					/*double x_ecf[6];
					double x_j2000[6];
					x_j2000[0] = forecastOrbList[s_i].pos.x;  
					x_j2000[1] = forecastOrbList[s_i].pos.y;  
					x_j2000[2] = forecastOrbList[s_i].pos.z;
					x_j2000[3] = forecastOrbList[s_i].vel.x; 
					x_j2000[4] = forecastOrbList[s_i].vel.y; 
					x_j2000[5] = forecastOrbList[s_i].vel.z;
					forecastOrbList[s_i].t = TimeCoordConvert::TDT2TAI(forecastOrbList[s_i].t);
					m_TimeCoordConvert.J2000_ECEF(TimeCoordConvert::TAI2GPST(forecastOrbList[s_i].t), x_j2000, x_ecf);
					forecastOrbList[s_i].pos.x = x_ecf[0]; 
					forecastOrbList[s_i].pos.y = x_ecf[1]; 
					forecastOrbList[s_i].pos.z = x_ecf[2];
					forecastOrbList[s_i].vel.x = x_ecf[3]; 
					forecastOrbList[s_i].vel.y = x_ecf[4]; 
					forecastOrbList[s_i].vel.z = x_ecf[5];*/
					Matrix matJ2000Pos, matJ2000Vel, matECFPos,matECFVel;
					matJ2000Pos.Init(3,1);
					matJ2000Vel.Init(3,1);
					matECFPos.Init(3,1);
					matECFVel.Init(3,1);
					matJ2000Pos.SetElement(0,0,forecastOrbList[s_i].pos.x);
					matJ2000Pos.SetElement(1,0,forecastOrbList[s_i].pos.y);
					matJ2000Pos.SetElement(2,0,forecastOrbList[s_i].pos.z);
					matJ2000Vel.SetElement(0,0,forecastOrbList[s_i].vel.x);
					matJ2000Vel.SetElement(1,0,forecastOrbList[s_i].vel.y);
					matJ2000Vel.SetElement(2,0,forecastOrbList[s_i].vel.z);
					forecastOrbList[s_i].t = TimeCoordConvert::TDT2TAI(forecastOrbList[s_i].t);
					Matrix matPR_NR, matER, matEP, matER_DOT;
					m_TimeCoordConvert.Matrix_J2000_ECEF(TimeCoordConvert::TAI2GPST(forecastOrbList[s_i].t), matPR_NR, matER, matEP, matER_DOT);
					Matrix matEst_EP, matEst_ER;
					eopEstPara.getEst_EOP(forecastOrbList[s_i].t, matEst_EP, matEst_ER);// 更新 matEP, matER
					matEP = matEst_EP * matEP;
					matER = matEst_ER * matER; // 2013/04/24, 原程序漏乘 matEst_ER
					matECFPos = matPR_NR * matJ2000Pos;
                    matECFVel = matPR_NR * matJ2000Vel;
					matECFVel = matER *  matECFVel + matER_DOT * matECFPos;
					matECFPos = matER *  matECFPos;
					matECFPos = matEP *  matECFPos;
					matECFVel = matEP *  matECFVel;
					forecastOrbList[s_i].pos.x = matECFPos.GetElement(0, 0); 
					forecastOrbList[s_i].pos.y = matECFPos.GetElement(1, 0); 
					forecastOrbList[s_i].pos.z = matECFPos.GetElement(2, 0);
					forecastOrbList[s_i].vel.x = matECFVel.GetElement(0, 0); 
					forecastOrbList[s_i].vel.y = matECFVel.GetElement(1, 0); 
					forecastOrbList[s_i].vel.z = matECFVel.GetElement(2, 0);
				}
			}
			return result;
		}
	}
}
