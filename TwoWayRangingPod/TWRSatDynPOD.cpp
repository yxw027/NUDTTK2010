#include "TWRSatDynPOD.hpp"
#include "RuningInfoFile.hpp"
#include "AttitudeTrans.hpp"
#include "GNSSBasicCorrectFunc.hpp"
#include "SolidTides.hpp"
#include "OceanTidesLoading.hpp"

namespace NUDTTK
{
	namespace TwoWayRangingPod
	{
		TWRSatDynPOD::TWRSatDynPOD(void)
		{
			m_strPODPath = "";
		}

		TWRSatDynPOD::~TWRSatDynPOD(void)
		{
		}

		bool TWRSatDynPOD::judgeDeleteData(UTC t)
		{
			bool bDeleteData = false;
			for(size_t s_i = 0; s_i < m_attDeleteDataList.size(); s_i++)
			{
				if(t - m_attDeleteDataList[s_i].t0 >= 0.0 && t - m_attDeleteDataList[s_i].t1 <= 0.0)
				{
					bDeleteData = true;
					break;
				}
			}
			return bDeleteData;
		}

		bool TWRSatDynPOD::judgeAttManeuver(UTC t)
		{
			bool bManeuver = false;
			for(size_t s_i = 0; s_i < m_attManeuverList.size(); s_i++)
			{
				if(t - m_attManeuverList[s_i].t0 >= 0.0 && t - m_attManeuverList[s_i].t1 <= 0.0)
				{
					bManeuver = true;
					break;
				}
			}
			return bManeuver;
		}

		bool TWRSatDynPOD::judgeAttManeuverTime(UTC t)
		{
			bool bManeuverTime = false;
			for(size_t s_i = 0; s_i < m_attManeuverList.size(); s_i++)
			{
				if(t - m_attManeuverList[s_i].t0 >= -m_twrDefine.span_DeleteManeuver && t- m_attManeuverList[s_i].t0 <= m_twrDefine.span_DeleteManeuver)
				{
					bManeuverTime = true;
					break;
				}
				if(t - m_attManeuverList[s_i].t1 >= -m_twrDefine.span_DeleteManeuver && t- m_attManeuverList[s_i].t1 <= m_twrDefine.span_DeleteManeuver)
				{
					bManeuverTime = true;
					break;
				}
			}
			return bManeuverTime;
		}

		// 子程序名称： adamsCowell_ac  
		// 功能：首先利用线性多步数值积分方法获得长弧段卫星轨道数据和偏导数数据
		//       卫星轨道数据和偏导数数据的时间点与积分步长严格对齐,并向两段延长4个步长, 提高边缘部分插值精度
		//       为后续插值提供基准点数据, 其中轨道插值采用 8 阶 lagrange方法, 偏导数插值采用线性方法
		// 变量类型：t0                  : 参考时间历元, 插值输出点的参考时间, UTC北京时
		//           t1 
		//           dynamicDatum        : 初始点的长弧段动力学参数, (dynamicDatum.t0 的时间可以与 interpTimelist[0] 不对应, 此时进行倒向积分)
		//           orbitlist_ac        : 每一点插值参考轨道
		//           matRtPartiallist_ac : 每一点轨道位置对动力学参数的偏导数
		//           h                   : 积分步长
		//           q                   : Adams_Cowell 的阶数
		// 输入： t0_Interp, t1_Interp, dynamicDatum, h, q
		// 输出： interpTimelist, matRtPartiallist
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2012/12/31
		// 版本时间：2012/12/31
		// 修改记录：
		// 备注： 默认步长选取75s, 11阶, 参考MIT
		bool TWRSatDynPOD::adamsCowell_ac(UTC t0, UTC t1, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h, int q)
		{
			orbitlist_ac.clear();
			matRtPartiallist_ac.clear();
			TDT t_Begin = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t0 - 3600.0 * 8));
			TDT t_End   = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t1 - 3600.0 * 8));
			const int countDynParameter = dynamicDatum.getAllEstParaCount(); 
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
			// TDT -> UTC北京时
			for(size_t s_i = 0; s_i < orbitlist_ac.size(); s_i++)
				orbitlist_ac[s_i].t = m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::TDT2TAI(orbitlist_ac[s_i].t)) + 3600.0 * 8; // 转换到北京时
			return true;
		}

		// 子程序名称： getEphemeris   
		// 功能：滑动lagrange插值获得任意时刻BD卫星星历
		// 变量类型： t                     :  UTC北京时
		//            geoOrb                :  星历数值, 坐标单位: 米
		//            nLagrange             :  Lagrange 插值已知点个数, 默认为 9, 对应 8 阶 Lagrange 插值
		// 输入：t,  nLagrange
		// 输出：interpOrbit
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2012/12/31
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool TWRSatDynPOD::getEphemeris(UTC t, TimePosVel& geoOrb, int nLagrange)
		{
			size_t count_ac = m_acOrbitList.size();
			const int nlagrange = 8; 
			if(count_ac < nlagrange) // 如果数据点个数小于nlagrange返回, 要求弧段长度 > h * nlagrange = 4分钟
				return false;
			double h = m_acOrbitList[1].t - m_acOrbitList[0].t;
			double spanSecond_t = t - m_acOrbitList[0].t;  // 相对观测时间, 初始时间为 orbitlist_ac[0].t
			int nLeftPos  = int(spanSecond_t / h);       // 首先寻找最靠近时间 T 的左端点，从 0 开始计数
			int nLeftNum  = int(floor(nlagrange / 2.0)); // 理论上 nLeftPos 左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
			int nRightNum = int(ceil(nlagrange / 2.0));
			int nBegin, nEnd;                            // 位于区间[0, count_ac - 1]
			if(nLeftPos - nLeftNum + 1 < 0)              // nEnd - nBegin = nLagrange - 1 
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
			geoOrb.t = t;
			double *x = new double [nlagrange];
			double *y = new double [nlagrange];
			for(int i = nBegin; i <= nEnd; i++)
				x[i - nBegin] = m_acOrbitList[i].t - m_acOrbitList[0].t; // 参考相对时间点
			// X
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].pos.x;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, geoOrb.pos.x);
			// Y
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].pos.y;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, geoOrb.pos.y);
			// Z
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].pos.z;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, geoOrb.pos.z);
			// Vx
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].vel.x;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, geoOrb.vel.x);
			// Vy
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].vel.y;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, geoOrb.vel.y);
			// Vz
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].vel.z;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, geoOrb.vel.z);
			delete x;
			delete y;
			return true;
		}

		// 子程序名称： getEphemeris_PathDelay   
		// 功能：根据接收机的概略位置、信号接收时间和卫星的先验轨道,
		//       计算卫星信号传播延迟时间(即卫星“准确的”信号反射时间)
		// 变量类型： t                  : 信号接收时间
		//            staPos             : 接收机概略位置, 单位：米
		//            delay              : 信号传播延迟时间, 单位：秒
		//            geoOrb             : 确定了正确的信号发射时间后, 顺便返回本颗BD卫星星历
		//            geoRtPartial       : 卫星的偏导数
		//            threshold          : 迭代阈值，默认 1.0E-007
		// 输入：t, staPos, m_acOrbitList, m_acRtPartialList, threshold
		// 输出：geoOrb, geoRtPartial
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2012/12/31
		// 版本时间：2017/11/02
		// 修改记录：
		// 备注： 
		bool TWRSatDynPOD::getEphemeris_PathDelay(UTC t, POS3D staPos, double& delay, TimePosVel& geoOrb, Matrix& geoRtPartial, double threshold)
		{
			// 信号真实接收时间 = 观测时间(T) - 接收机钟差(receiverPos.dClock)
			UTC t_Receive  = t;
			UTC t_Transmit = t_Receive; // 初始化信号转发时间
			if(!getEphemeris(t_Transmit, geoOrb))
				return false;
			double distance = pow(staPos.x - geoOrb.pos.x, 2)
							+ pow(staPos.y - geoOrb.pos.y, 2)
							+ pow(staPos.z - geoOrb.pos.z, 2);
			distance = sqrt(distance); // 获得信号发射传播距离
			double delay_k_1 = 0;
			delay = distance / SPEED_LIGHT;  // 获得信号反射传播延迟
			const double delay_max  = 1.0;   // 为了防止迭代dDelay溢出，这里设置一个阈值
			const int    k_max      = 5;     // 迭代次数阈值，一般1次迭代就会收敛 
			int          k          = 0;
			while(fabs(delay - delay_k_1) > threshold)   // 迭代阈值控制, abs-->fabs, 2007/07/15
			{
				k++;
				if(fabs(delay) > delay_max || k > k_max) // 为防止 delay 溢出, 2007/04/06
				{
					printf("%d%d%f delay 迭代发散!\n", t.hour, t.minute, t.second);
					return false;
				}
				// 更新信号反射时间
				t_Transmit = t_Receive - delay;
				if(!getEphemeris(t_Transmit, geoOrb))
					return false;
				// 更新概略距离
				distance =  pow(staPos.x - geoOrb.pos.x, 2)
						  + pow(staPos.y - geoOrb.pos.y, 2)
						  + pow(staPos.z - geoOrb.pos.z, 2);
				distance = sqrt(distance);
				// 更新延迟数据
				delay_k_1 = delay;
				delay = distance / SPEED_LIGHT;
			}
			if(int(m_acRtPartialList.size()) < 2) 
				return false;
			
			const int countDynParameter = m_acRtPartialList[0].GetNumColumns(); 
			geoRtPartial.Init(3, countDynParameter);

			double spanSecond_t = t_Transmit - m_acOrbitList[0].t; 
			double h = m_acOrbitList[1].t - m_acOrbitList[0].t;
			int nLeftPos  = int(spanSecond_t / h); // 首先寻找最靠近时间 T 的左端点, 从 0 开始计数
			int nBegin, nEnd; 
			// 偏导数插值, 阶数2, 线性插值, 2008/06/27
			if(nLeftPos < 0) // nEnd - nBegin = nLagrange - 1 
			{
				nBegin = 0;
				nEnd   = 1;
			}
			else if(nLeftPos + 1 >= int(m_acRtPartialList.size()))
			{
				nBegin = int(m_acRtPartialList.size()) - 2;
				nEnd   = int(m_acRtPartialList.size()) - 1;
			}
			else
			{
				nBegin = nLeftPos;
				nEnd   = nLeftPos + 1;
			}
			double x_t[2];
			double y_t[2];
			x_t[0] = m_acOrbitList[nBegin].t - m_acOrbitList[0].t;
			x_t[1] = m_acOrbitList[nEnd].t   - m_acOrbitList[0].t;
			double u = (spanSecond_t - x_t[0])/(x_t[1] - x_t[0]);
			for(int ii = 0; ii < 3; ii++)
			{
				for(int jj = 0; jj < int(countDynParameter); jj++)
				{// 对矩阵的每个元素[ii, jj]进行插值
					y_t[0] = m_acRtPartialList[nBegin].GetElement(ii, jj);
					y_t[1] = m_acRtPartialList[nEnd].GetElement(ii, jj);
					double element = u * y_t[1] + (1 - u) * y_t[0];
					geoRtPartial.SetElement(ii, jj, element);
				}
			}
			return true;
		}

		// 子程序名称： initDynDatumEst   
		// 功能：对运动学轨道点位信息进行动力学平滑, 输出初始轨道参数
		// 变量类型：orbitlist        : 几何轨道
		//           dynamicDatum     : 动力学轨道根数 
		//           arclength        : 弧段的长度
		//           h                : 积分步长, GEO-75s; LEO-10s
		// 输入：orbitlist, arclength
		// 输出：dynamicDatum
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2008/11/14
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool TWRSatDynPOD::initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength, double h)
		{
			char info[200];
			// 提取粗间隔的插值点
			double  threshold_coarseorbit_interval = m_twrDefine.span_InitDynDatumCoarsePos;
			double  cumulate_time = threshold_coarseorbit_interval * 2; // 保证第一个点被加入
			vector<int> coarseindexList;                    
			coarseindexList.clear();
			vector<TimePosVel>  coarseorbitList;
			coarseorbitList.clear();
			for(size_t s_i = 0; s_i < orbitlist.size(); s_i++)
			{	
				if(s_i > 0)
				{
					cumulate_time += orbitlist[s_i].t - orbitlist[s_i - 1].t;
				}
				if(cumulate_time >= threshold_coarseorbit_interval || s_i == orbitlist.size() - 1)
				{
					cumulate_time = 0;
					coarseindexList.push_back(int(s_i));
					coarseorbitList.push_back(orbitlist[s_i]);
				}
			}
			// 为了保证初轨的插值速度精度, 需要前面连续M点间隔不超过 threshold_coarseorbit_interval * 2
			const int nLagrangePoint = 9; // 9 点 Lagrange 插值
			int M = 4;
			int k0 = 0;
			while(1)
			{
				if(k0 + nLagrangePoint > int(coarseorbitList.size()))
				{
					printf("无法找到足够的插值点来获取初始轨道速度, 初始轨道解算失败!\n");
					sprintf(info, "无法找到足够的插值点来获取初始轨道速度, 初始轨道解算失败!(%d)", coarseorbitList.size());
					RuningInfoFile::Add(info);
					return false;
				}
                double max_interval = 0;
				for(int s_i = k0 + 1; s_i < k0 + M; s_i++)
				{
					double innterval_i = coarseorbitList[s_i].t - coarseorbitList[s_i - 1].t;
					if(max_interval < innterval_i)
					{
						max_interval = innterval_i;
					}
				}
				if(max_interval <= threshold_coarseorbit_interval * 2)
				{
					break;
				}
				else
				{
					k0++;
				}
			}
			int i_begin = coarseindexList[k0];
			for(int s_i = 0; s_i < i_begin; s_i++)
			{// 清除前面的几个无效的初值点
				orbitlist.erase(orbitlist.begin());
			}
			while(1)
			{// 清除超过弧长的点
				int npos = int(orbitlist.size()) - 1;
				if(orbitlist[npos].t - orbitlist[0].t > arclength)
					orbitlist.erase(orbitlist.begin() + npos);
				else
					break;
			}
			// 利用微分匹配平滑求速, 更新 k0 点的速度信息 
			double *xa_t = new double [nLagrangePoint];
			double *ya_X = new double [nLagrangePoint];
			double *ya_Y = new double [nLagrangePoint];
			double *ya_Z = new double [nLagrangePoint];
			for(int s_i = k0; s_i < k0 + nLagrangePoint; s_i++)
			{
				xa_t[s_i - k0] = coarseorbitList[s_i].t - coarseorbitList[k0].t;
				ya_X[s_i - k0] = coarseorbitList[s_i].pos.x;
				ya_Y[s_i - k0] = coarseorbitList[s_i].pos.y;
				ya_Z[s_i - k0] = coarseorbitList[s_i].pos.z;
			}
			InterploationLagrange(xa_t, ya_X, nLagrangePoint, 0.0, orbitlist[0].pos.x, orbitlist[0].vel.x);
			InterploationLagrange(xa_t, ya_Y, nLagrangePoint, 0.0, orbitlist[0].pos.y, orbitlist[0].vel.y);
			InterploationLagrange(xa_t, ya_Z, nLagrangePoint, 0.0, orbitlist[0].pos.z, orbitlist[0].vel.z);
			delete xa_t;
			delete ya_X;
			delete ya_Y;
			delete ya_Z;
			// 以上计算是为了得到可靠的初始轨道速度

			//FILE* pfile = fopen("c:\\初轨确定.txt", "w+");
			//  时间坐标系统一, 坐标-J2000, 时间-TDT
			vector<TDT> interpTimelist;
			interpTimelist.resize(orbitlist.size());
			for(size_t s_i = 0; s_i < orbitlist.size(); s_i ++)
			{
				double x_ecf[6];
				double x_j2000[6];
				x_ecf[0] = orbitlist[s_i].pos.x;  
				x_ecf[1] = orbitlist[s_i].pos.y;  
				x_ecf[2] = orbitlist[s_i].pos.z;
				x_ecf[3] = orbitlist[s_i].vel.x; 
				x_ecf[4] = orbitlist[s_i].vel.y; 
				x_ecf[5] = orbitlist[s_i].vel.z;
				GPST t_GPS = m_TimeCoordConvert.UTC2GPST(orbitlist[s_i].t - 3600.0 * 8);
				m_TimeCoordConvert.ECEF_J2000(t_GPS, x_j2000, x_ecf);
				orbitlist[s_i].t = TimeCoordConvert::GPST2TDT(t_GPS);
				orbitlist[s_i].pos.x = x_j2000[0]; 
				orbitlist[s_i].pos.y = x_j2000[1]; 
				orbitlist[s_i].pos.z = x_j2000[2];
				orbitlist[s_i].vel.x = x_j2000[3]; 
				orbitlist[s_i].vel.y = x_j2000[4]; 
				orbitlist[s_i].vel.z = x_j2000[5];
				interpTimelist[s_i] = orbitlist[s_i].t;
			}
			dynamicDatum.T0 = orbitlist[0].t;
			dynamicDatum.X0 = orbitlist[0].getPosVel();
			dynamicDatum.ArcLength = orbitlist[orbitlist.size() - 1].t - dynamicDatum.T0; 
			dynamicDatum.bOn_SolarPressureAcc  = false;
			dynamicDatum.bOn_AtmosphereDragAcc = false;
			dynamicDatum.bOn_EmpiricalForceAcc = false;
			dynamicDatum.init(dynamicDatum.ArcLength, dynamicDatum.ArcLength, dynamicDatum.ArcLength);
			// 2008/11/15
			TDT t_End = orbitlist[orbitlist.size() - 1].t;
			int  k = 0; // 记录迭代的次数
			bool flag_robust = false;
			bool flag_done   = false;
			double factor = 4.0;
			Matrix matW(int(orbitlist.size()), 3); // 观测权矩阵
			for(size_t s_i = 0; s_i < orbitlist.size(); s_i++)
			{
				for(size_t s_j = 0; s_j < 3; s_j++)
				{
					matW.SetElement(int(s_i), int(s_j), 1.0); 
				}
			}
			bool result = true;
			size_t count_measureorbit_control;
			vector<TDT> interpTimelist_control;
			int num_control = 0;
			double rms_fitresidual = 0.0;
			while(1)
			{
				k++;
				if(k >= m_twrDefine.max_OrbIteration)
				{
					result = false;
					printf("初轨确定程序迭代次数溢出(initDynDatumEst)!\n");
					sprintf(info, "初轨确定程序迭代次数溢出!(%d)", k);
					RuningInfoFile::Add(info);
					break;
				}
				vector<TimePosVel> interpOrbitlist; // 插值序列
				vector<Matrix> interpRtPartiallist; // 插值偏导数序列
				if(k == 1)
				{
					adamsCowell_Interp(interpTimelist, dynamicDatum, interpOrbitlist, interpRtPartiallist, h); 
					// 由于初轨的速度是通过曲线拟合的方式计算得到, 精度不高, 难以支撑较长弧段, 因此需要控制初轨拟合的弧段长度, 首先进行短弧段定轨
					count_measureorbit_control = orbitlist.size();
					interpTimelist_control = interpTimelist;
					for(int i = 0; i < int(orbitlist.size()); i++)
					{
						double error_interp = sqrt(pow(orbitlist[i].pos.x - interpOrbitlist[i].pos.x * matW.GetElement(i, 0), 2)
												 + pow(orbitlist[i].pos.y - interpOrbitlist[i].pos.y * matW.GetElement(i, 1), 2)
												 + pow(orbitlist[i].pos.z - interpOrbitlist[i].pos.z * matW.GetElement(i, 2), 2));
						if(error_interp >= 10000.0) // 当前点的预报残差阈值超过10km的点
						{
                            // 统计当前点以后的预报残差阈值超过 10km 的点, 如果“大部分”超过 10km, 则认为是该残差是由于预报精度较差造成的, 并非野值点造成
							int count_threshold_points = 0;
							for(int j = i; j < int(orbitlist.size()); j++)
							{
								double error_interp_j = sqrt(pow(orbitlist[j].pos.x - interpOrbitlist[j].pos.x, 2)
														   + pow(orbitlist[j].pos.y - interpOrbitlist[j].pos.y, 2)
														   + pow(orbitlist[j].pos.z - interpOrbitlist[j].pos.z, 2));
								if(error_interp_j >= 10000.0)
									count_threshold_points++;
							}
							if((count_threshold_points * 1.0 / (orbitlist.size() - i)) > 0.30)
							{ 
								count_measureorbit_control = i + 1;
								interpTimelist_control.resize(count_measureorbit_control);
								for(int j = 0; j < int(count_measureorbit_control); j++)
								{
									interpTimelist_control[j] = interpTimelist[j];
								}
								num_control++;
								//fprintf(pfile, "正在第%2d次分布迭代, 解算点数 = %6d/%6d.\n", num_control, count_measureorbit_control, orbitlist.size());
								break;
							}
						}
					}
				}
				else
				{
					adamsCowell_Interp(interpTimelist_control, dynamicDatum, interpOrbitlist, interpRtPartiallist, h);
				}

				int NUM_M = 6; // 待估参数的个数: 6个初始轨道根数
				Matrix matH(int(count_measureorbit_control) * 3, NUM_M);
				Matrix matY(int(count_measureorbit_control) * 3, 1);  
				for(int i = 0; i < int(count_measureorbit_control); i++)
				{
					matY.SetElement(i * 3 + 0, 0, matW.GetElement(i, 0) * (orbitlist[i].pos.x - interpOrbitlist[i].pos.x));
					matY.SetElement(i * 3 + 1, 0, matW.GetElement(i, 1) * (orbitlist[i].pos.y - interpOrbitlist[i].pos.y));
					matY.SetElement(i * 3 + 2, 0, matW.GetElement(i, 2) * (orbitlist[i].pos.z - interpOrbitlist[i].pos.z));
					for(int j = 0; j < 6; j++)
					{
						matH.SetElement(i * 3 + 0, j, matW.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, j));
						matH.SetElement(i * 3 + 1, j, matW.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, j));
						matH.SetElement(i * 3 + 2, j, matW.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, j));
					}
				}
				Matrix matX = (matH.Transpose() * matH).Inv_Ssgj() * matH.Transpose() * matY; 
				dynamicDatum.X0.x  += matX.GetElement(0,0);
				dynamicDatum.X0.y  += matX.GetElement(1,0);
				dynamicDatum.X0.z  += matX.GetElement(2,0);
				dynamicDatum.X0.vx += matX.GetElement(3,0);
				dynamicDatum.X0.vy += matX.GetElement(4,0);
				dynamicDatum.X0.vz += matX.GetElement(5,0);

				// 判断收敛条件
				double max_pos = 0;
				double max_vel = 0;
				for(int i = 0; i < 3; i++)
				{
					max_pos = max(max_pos, fabs(matX.GetElement(i,     0)));
					max_vel = max(max_vel, fabs(matX.GetElement(i + 3, 0)));
				}

				if(max_pos <= 1.0E-3 || flag_done)
				{
					if(flag_robust == false && flag_done == false)
					{
						flag_robust = true;
						// 计算方差
						rms_fitresidual = 0; 
						size_t count_normalpoint = 0;
						for(int i = 0; i < int(count_measureorbit_control); i++)
						{
							if(matW.GetElement(i, 0) == 1.0)
							{
								count_normalpoint++;
								rms_fitresidual += matY.GetElement(i * 3 + 0, 0) * matY.GetElement(i * 3 + 0, 0);
							}
							if(matW.GetElement(i, 1) == 1.0)
							{
								count_normalpoint++;
								rms_fitresidual += matY.GetElement(i * 3 + 1, 0) * matY.GetElement(i * 3 + 1, 0);
							}
							if(matW.GetElement(i, 2) == 1.0)
							{
								count_normalpoint++;
								rms_fitresidual += matY.GetElement(i * 3 + 2, 0) * matY.GetElement(i * 3 + 2, 0);
							}
						}
						rms_fitresidual = sqrt(rms_fitresidual / count_normalpoint);
						//fprintf(pfile, "定轨残差 = %10.4f\n", rms_fitresidual);
                        int count_outliers = 0;
						for(int i = 0; i < int(count_measureorbit_control); i++)
						{
							if(fabs(matY.GetElement(i * 3 + 0, 0)) >= factor * rms_fitresidual)
							{
								matW.SetElement(i, 0, rms_fitresidual / fabs(matY.GetElement(i * 3 + 0, 0)));
								//fprintf(pfile, "i = %5d, X = %14.4f\n", i, matY.GetElement(i * 3 + 0, 0));
								count_outliers++;
							}
							if(fabs(matY.GetElement(i * 3 + 1, 0)) >= factor * rms_fitresidual)
							{
								matW.SetElement(i, 1, rms_fitresidual / fabs(matY.GetElement(i * 3 + 1, 0)));
								//fprintf(pfile, "i = %5d, Y = %14.4f\n", i, matY.GetElement(i * 3 + 1, 0));
								count_outliers++;
							}
							if(fabs(matY.GetElement(i * 3 + 2, 0)) >= factor * rms_fitresidual)
							{
								matW.SetElement(i, 2, rms_fitresidual / fabs(matY.GetElement(i * 3 + 2, 0)));
								//fprintf(pfile, "i = %5d, Z = %14.4f\n", i, matY.GetElement(i * 3 + 2, 0));
								count_outliers++;
							}
						}
						flag_done = true;
						if(count_outliers > 0)
						{
							continue;
						}
					}
					if(count_measureorbit_control <= orbitlist.size() / 2 && num_control <= 3)
					{// 分布解算完毕，重新开始新的迭代
						flag_robust = false;
						flag_done = false;
						for(size_t s_i = 0; s_i < orbitlist.size(); s_i++)
						{
							for(size_t s_j = 0; s_j < 3; s_j++)
							{
								matW.SetElement(int(s_i), int(s_j), 1.0); 
							}
						}
						k = 0;
						continue;
					}
					//fprintf(pfile, "经过%d次迭代收敛:\n%14.4f\n%14.4f\n%14.4f\n%14.4f\n%14.4f\n%14.4f\n", k, 
					//																					  dynamicDatum.X0.x,
					//																					  dynamicDatum.X0.y,
					//																					  dynamicDatum.X0.z,
					//																					  dynamicDatum.X0.vx,
					//																					  dynamicDatum.X0.vy,
					//																					  dynamicDatum.X0.vz);
					break;
				}
			}
			//fclose(pfile);
			if(result)
			{
				sprintf(info, "初始轨道解算成功!(%d/%d), oc = %.4f.", count_measureorbit_control, orbitlist.size(), rms_fitresidual);
				RuningInfoFile::Add(info);
				return true;
			}
			else
			{
				printf("初始轨道解算失败(initDynDatumEst)!\n");
				sprintf(info, "初始轨道解算失败(initDynDatumEst)!");
				RuningInfoFile::Add(info);
				return false;
			}
		}


		// 子程序名称： dynamicGEOPOD_pos
		// 功能：动力学轨道拟合GEO卫星的轨道位置点序列
		// 变量类型：obsOrbitList         : 测量轨道列表, 采用UTC, ITRF坐标系
        //           dynamicDatum         : 拟合后的初始动力学轨道参数
		//           t0_forecast          : 预报轨道初始时间, UTC, 北京时
		//           t1_forecast          : 预报轨道终止时间, UTC, 北京时
		//           forecastOrbList      : 预报轨道列表, 采用UTC, ITRF坐标系
		//           interval             : 预报轨道间隔
		//           bInitDynDatumEst     : 初始动力学轨道求解标记
		//           bForecast            : 预报标记, 默认true, 否则不进行预报, 用于初轨确定
		//           bResEdit             : 定轨O-C残差编辑标记开关, 默认true, 否则不进行残差编辑 
		// 输入：orbitlist, dynamicDatum, t0_forecast, t1_forecast, interval, bInitDynDatumEst, bForecast, bResEdit
		// 输出：dynamicDatum, forecastOrbList
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2009/06/21
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool TWRSatDynPOD::dynamicGEOPOD_pos(vector<TimePosVel> obsOrbitList, SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval, bool bInitDynDatumEst, bool bForecast, bool bResEdit)
		{
			char info[200];
			if(obsOrbitList.size() <= 0)
				return false;
			// 进行初轨确定
			if(!bInitDynDatumEst)
			{
				vector<TimePosVel> orbitlist;
				double arclength_initDynDatumEst = 3600.0 * 3; // 3小时
				for(size_t s_i = 0; s_i < obsOrbitList.size(); s_i++)
				{
					if(orbitlist.size() == 0)
						orbitlist.push_back(obsOrbitList[s_i]);
					else
					{
						if(obsOrbitList[s_i].t - orbitlist[0].t <= arclength_initDynDatumEst)
							orbitlist.push_back(obsOrbitList[s_i]);
						else
							break;
					}
				}
				SatdynBasicDatum dynamicDatum_0;
				dynamicDatum_0.bOn_SolidTide    = dynamicDatum.bOn_SolidTide; // 20161208, 便于外部仿真控制
				dynamicDatum_0.bOn_OceanTide    = dynamicDatum.bOn_OceanTide ;
				dynamicDatum_0.bOn_ThirdBodyAcc = dynamicDatum.bOn_ThirdBodyAcc;
				if(!initDynDatumEst(orbitlist, dynamicDatum_0,  arclength_initDynDatumEst))
					return false;
				GPST t_GPS = m_TimeCoordConvert.UTC2GPST(obsOrbitList[obsOrbitList.size() - 1].t - 3600.0 * 8);
				TDT t_End = TimeCoordConvert::GPST2TDT(t_GPS);
				dynamicDatum.T0 = dynamicDatum_0.T0;
				dynamicDatum.ArcLength = t_End - dynamicDatum.T0;
				dynamicDatum.X0 = dynamicDatum_0.X0;
				dynamicDatum.init(m_twrDefine.period_SolarPressure, dynamicDatum.ArcLength, m_twrDefine.period_EmpiricalAcc);
			}
			SatdynBasicDatum dynamicDatum_Init = dynamicDatum; // 记录初始轨道根数
			char dynamicDatumFilePath[300];
			sprintf(dynamicDatumFilePath,"%s\\%s", m_strPODPath.c_str(), m_twrDefine.nameDynPodFitFile.c_str());
			if(!m_strPODPath.empty())
			{
				FILE * pFitFile = fopen(dynamicDatumFilePath, "w+");
				fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
				fprintf(pFitFile, "%3d.      X    (m)      %20.4f\n", 1,dynamicDatum_Init.X0.x);
				fprintf(pFitFile, "%3d.      Y    (m)      %20.4f\n", 2,dynamicDatum_Init.X0.y);
				fprintf(pFitFile, "%3d.      Z    (m)      %20.4f\n", 3,dynamicDatum_Init.X0.z);
				fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f\n", 4,dynamicDatum_Init.X0.vx);
				fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f\n", 5,dynamicDatum_Init.X0.vy);
				fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f\n", 6,dynamicDatum_Init.X0.vz);
				int k_Parameter = 6;
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
				if(dynamicDatum_Init.bOn_SolarPressureAcc && (dynamicDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_9PARA || dynamicDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_5PARA))
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
				if(dynamicDatum_Init.bOn_EmpiricalForceAcc && dynamicDatum_Init.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
				{
					for(size_t s_i = 0; s_i < dynamicDatum_Init.empiricalForceParaList.size(); s_i++)
					{// 20140320, 谷德峰修改, 使得三个方向经验力可选
						if(dynamicDatum_Init.bOn_EmpiricalForceAcc_R)
						{
							k_Parameter++;
							dynamicDatum_Init.empiricalForceParaList[s_i].cos_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7);
							k_Parameter++;
							dynamicDatum_Init.empiricalForceParaList[s_i].sin_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7);
						}
						if(dynamicDatum_Init.bOn_EmpiricalForceAcc_T)
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
						}
						if(dynamicDatum_Init.bOn_EmpiricalForceAcc_N)
						{
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
				}
				// 20170427, 谷德峰修改, 增加常值项经验力
				if(dynamicDatum_Init.bOn_EmpiricalForceAcc && dynamicDatum_Init.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
				{
					for(size_t s_i = 0; s_i < dynamicDatum_Init.empiricalForceParaList.size(); s_i++)
					{// 20140320, 谷德峰修改, 使得三个方向经验力可选
						if(dynamicDatum_Init.bOn_EmpiricalForceAcc_R)
						{
							k_Parameter++;
							dynamicDatum_Init.empiricalForceParaList[s_i].a0_R = 0.0;
							fprintf(pFitFile, "%3d. %2d   R_A0 (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  dynamicDatum_Init.empiricalForceParaList[s_i].a0_R  * 1.0E+7);
							k_Parameter++;
							dynamicDatum_Init.empiricalForceParaList[s_i].cos_R = 0.0;
							fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7);
							k_Parameter++;
							dynamicDatum_Init.empiricalForceParaList[s_i].sin_R = 0.0;
							fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7);
						}
						if(dynamicDatum_Init.bOn_EmpiricalForceAcc_T)
						{
							k_Parameter++;
							dynamicDatum_Init.empiricalForceParaList[s_i].a0_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   T_A0 (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  dynamicDatum_Init.empiricalForceParaList[s_i].a0_T  * 1.0E+7);
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
						}
						if(dynamicDatum_Init.bOn_EmpiricalForceAcc_N)
						{
							k_Parameter++;
							dynamicDatum_Init.empiricalForceParaList[s_i].a0_N = 0.0;
							fprintf(pFitFile, "%3d. %2d   N_A0 (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  dynamicDatum_Init.empiricalForceParaList[s_i].a0_N  * 1.0E+7);
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
				}
				fclose(pFitFile);
			}
			// 经验加速度区间暂不调整, 增加绝对约束
			
			// 时间坐标系统一, 坐标->j2000; 时间->TDT
			vector<TDT> interpTimelist;
			interpTimelist.resize(obsOrbitList.size());
			for(size_t s_i = 0; s_i < obsOrbitList.size(); s_i ++)
			{
				double x_ecf[6];
				double x_j2000[6];
				x_ecf[0] = obsOrbitList[s_i].pos.x;  
				x_ecf[1] = obsOrbitList[s_i].pos.y;  
				x_ecf[2] = obsOrbitList[s_i].pos.z;
				x_ecf[3] = obsOrbitList[s_i].vel.x; 
				x_ecf[4] = obsOrbitList[s_i].vel.y; 
				x_ecf[5] = obsOrbitList[s_i].vel.z;
				GPST t_GPS = m_TimeCoordConvert.UTC2GPST(obsOrbitList[s_i].t - 3600.0 * 8);
				m_TimeCoordConvert.ECEF_J2000(t_GPS, x_j2000, x_ecf);
				obsOrbitList[s_i].pos.x = x_j2000[0]; 
				obsOrbitList[s_i].pos.y = x_j2000[1]; 
				obsOrbitList[s_i].pos.z = x_j2000[2];
				obsOrbitList[s_i].vel.x = x_j2000[3]; 
				obsOrbitList[s_i].vel.y = x_j2000[4]; 
				obsOrbitList[s_i].vel.z = x_j2000[5];
				obsOrbitList[s_i].t = TimeCoordConvert::GPST2TDT(t_GPS);
				interpTimelist[s_i] = obsOrbitList[s_i].t;
			}

			int count_DynParameter = dynamicDatum.getAllEstParaCount(); 
			int count_SolarPressureParaList = 0;
			if(dynamicDatum.bOn_SolarPressureAcc)
				count_SolarPressureParaList = int(dynamicDatum.solarPressureParaList.size());

			int count_EmpiricalForceParaList = 0;
			if(dynamicDatum.bOn_EmpiricalForceAcc)
				count_EmpiricalForceParaList = int(dynamicDatum.empiricalForceParaList.size());
			
			int count_SolarPressurePara = dynamicDatum.getSolarPressureParaCount();
			
			int index_EmpiricalParaBegin = 0; // 记录经验力参数在整个待估动力学参数列表中的位置，2014/10/07，谷德峰
			int k = 0; // 记录迭代的次数
			bool flag_robust = false;
			bool flag_done = false;
			double factor = 3.0;
			Matrix matWeight(int(obsOrbitList.size()), 3); // 观测权矩阵
			for(size_t s_i = 0; s_i < obsOrbitList.size(); s_i++)
			{
				for(size_t s_j = 0; s_j < 3; s_j++)
					matWeight.SetElement(int(s_i), int(s_j), 1.0); 
			}
			bool result = true;
			double rms_oc = 0.0; 
			while(1)
			{
				k++;
				if(k >= m_twrDefine.max_OrbIteration)
				{
					result = false;
					printf("迭代超过%d次, 发散!", k);
					break;
				}
				vector<TimePosVel> interpOrbitlist; // 插值序列
				vector<Matrix> interpRtPartiallist; // 插值偏导数序列
				adamsCowell_Interp(interpTimelist, dynamicDatum, interpOrbitlist, interpRtPartiallist, 75.0); // GEO h = 75.0
				printf("第%d次 adamsCowell_Interp is ok!\n", k);
				int RowNum = int(obsOrbitList.size()) * 3;
				Matrix matH(RowNum, count_DynParameter);
				Matrix matY(RowNum, 1);
				for(int i = 0; i < int(obsOrbitList.size()); i++)
				{
					matY.SetElement(i * 3 + 0, 0, matWeight.GetElement(i, 0) * (obsOrbitList[i].pos.x  - interpOrbitlist[i].pos.x));
					matY.SetElement(i * 3 + 1, 0, matWeight.GetElement(i, 1) * (obsOrbitList[i].pos.y  - interpOrbitlist[i].pos.y));
					matY.SetElement(i * 3 + 2, 0, matWeight.GetElement(i, 2) * (obsOrbitList[i].pos.z  - interpOrbitlist[i].pos.z));
					for(int j = 0; j < 6; j++)
					{
						matH.SetElement(i * 3 + 0, j, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, j));
						matH.SetElement(i * 3 + 1, j, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, j));
						matH.SetElement(i * 3 + 2, j, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, j));
					}
					int beginPara = 6;
					// 2015/10/18, 兼容多光压模型计算, 谷德峰
					if(dynamicDatum.bOn_SolarPressureAcc)
					{
						for(int j = 0; j < int(dynamicDatum.solarPressureParaList.size() * count_SolarPressurePara); j++)
						{
							matH.SetElement(i * 3 + 0, beginPara + j, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j));
							matH.SetElement(i * 3 + 1, beginPara + j, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j));
							matH.SetElement(i * 3 + 2, beginPara + j, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j));
						}
						beginPara += count_SolarPressurePara * count_SolarPressureParaList;
					}
					//if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
					//{// 太阳光压
					//	for(int j = 0; j < int(dynamicDatum.solarPressureParaList.size()); j++)
					//	{
					//		matH.SetElement(i * 3 + 0, beginPara + j, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j));
					//		matH.SetElement(i * 3 + 1, beginPara + j, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j));
					//		matH.SetElement(i * 3 + 2, beginPara + j, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j));
					//	}
					//	beginPara += count_SolarPressureParaList;
					//}
					//else if(dynamicDatum.bOn_SolarPressureAcc &&
					//	   (dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA || dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA_EX || dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_15PARA || dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_21PARA))
					//{
					//	// 20140320, 谷德峰添加
					//	for(int j = 0; j < int(dynamicDatum.solarPressureParaList.size()); j++)
					//	{
					//		for(int jj = 0; jj < count_SolarPressurePara; jj++)
					//		{
					//			matH.SetElement(i * 3 + 0, beginPara + j * count_SolarPressurePara + jj, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * 9 + jj));
					//			matH.SetElement(i * 3 + 1, beginPara + j * count_SolarPressurePara + jj, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * 9 + jj));
					//			matH.SetElement(i * 3 + 2, beginPara + j * count_SolarPressurePara + jj, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * 9 + jj));
					//		}
					//	}
					//	beginPara += count_SolarPressurePara * count_SolarPressureParaList;
					//}
					if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
					{// 经验力
						index_EmpiricalParaBegin = beginPara;
						int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
										 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2  
										 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 2; 
						for(int j = 0; j < int(dynamicDatum.empiricalForceParaList.size()); j++)
						{
							int i_sub = 0;
							if(dynamicDatum.bOn_EmpiricalForceAcc_R)
							{
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 1));
							}
							i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2;
							if(dynamicDatum.bOn_EmpiricalForceAcc_T)
							{
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 1));
							}
							i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2;
							if(dynamicDatum.bOn_EmpiricalForceAcc_N)
							{
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 1));
							}
						}
						beginPara += count_sub * count_EmpiricalForceParaList;
					}
					// 20170427, 谷德峰修改, 增加常值项经验力
					if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
					{// 经验力
						index_EmpiricalParaBegin = beginPara;
						int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
										 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 3  
										 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 3; 
						for(int j = 0; j < int(dynamicDatum.empiricalForceParaList.size()); j++)
						{
							int i_sub = 0;
							if(dynamicDatum.bOn_EmpiricalForceAcc_R)
							{
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 2, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 2));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 2, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 2));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 2, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 2));
							}
							i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3;
							if(dynamicDatum.bOn_EmpiricalForceAcc_T)
							{
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 2, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 2));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 2, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 2));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 2, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 2));
							}
							i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 3;
							if(dynamicDatum.bOn_EmpiricalForceAcc_N)
							{
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 2, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 2));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 2, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 2));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 2, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 2));
							}
						}
						beginPara += count_sub * count_EmpiricalForceParaList;
					}
				}				
				// 计算轨道改进量
				Matrix matdx = (matH.Transpose() * matH).Inv_Ssgj() * matH.Transpose() * matY; 
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
				// 2015/10/18, 兼容多光压模型计算, 谷德峰
				else if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
				{
					// 20140320, 谷德峰添加
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
					}
					beginPara += 9 * count_SolarPressureParaList;
				}
				else if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_5PARA)	
				{
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
					{
						dynamicDatum.solarPressureParaList[s_k].D0  += matdx.GetElement(beginPara + 5 * s_k,     0);
						dynamicDatum.solarPressureParaList[s_k].Y0  += matdx.GetElement(beginPara + 5 * s_k + 1, 0);
						dynamicDatum.solarPressureParaList[s_k].B0  += matdx.GetElement(beginPara + 5 * s_k + 2, 0);
						dynamicDatum.solarPressureParaList[s_k].BC1 += matdx.GetElement(beginPara + 5 * s_k + 3, 0);
						dynamicDatum.solarPressureParaList[s_k].BS1 += matdx.GetElement(beginPara + 5 * s_k + 4, 0);
					}
				}
				if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
				{// 经验力
					int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
									 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2  
									 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 2; 
					for(int s_k = 0; s_k < int(dynamicDatum.empiricalForceParaList.size()); s_k++)
					{
						int i_sub = 0;
						if(dynamicDatum.bOn_EmpiricalForceAcc_R)
						{
							dynamicDatum.empiricalForceParaList[s_k].cos_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2;
						if(dynamicDatum.bOn_EmpiricalForceAcc_T)
						{
							dynamicDatum.empiricalForceParaList[s_k].cos_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2;
                        if(dynamicDatum.bOn_EmpiricalForceAcc_N)
						{
							dynamicDatum.empiricalForceParaList[s_k].cos_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
					}
					beginPara += count_sub * count_EmpiricalForceParaList;
				}
				// 20170427, 谷德峰修改, 增加常值项经验力
				if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
				{// 经验力
					int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3  
									 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 3  
									 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 3; 
					for(int s_k = 0; s_k < int(dynamicDatum.empiricalForceParaList.size()); s_k++)
					{
						int i_sub = 0;
						if(dynamicDatum.bOn_EmpiricalForceAcc_R)
						{
							dynamicDatum.empiricalForceParaList[s_k].a0_R  += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].cos_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3;
						if(dynamicDatum.bOn_EmpiricalForceAcc_T)
						{
							dynamicDatum.empiricalForceParaList[s_k].a0_T  += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].cos_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 3;
                        if(dynamicDatum.bOn_EmpiricalForceAcc_N)
						{
							dynamicDatum.empiricalForceParaList[s_k].a0_N  += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].cos_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
					}
					beginPara += count_sub * count_EmpiricalForceParaList;
				}

				if(!m_strPODPath.empty())
				{
					// 记录轨道改进结果
					FILE * pFitFile = fopen(dynamicDatumFilePath, "w+");
					fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
					fprintf(pFitFile, "%3d.      X    (m)      %20.4f%10.4f%20.4f\n", 1,dynamicDatum_Init.X0.x,  dynamicDatum.X0.x  - dynamicDatum_Init.X0.x,  dynamicDatum.X0.x);
					fprintf(pFitFile, "%3d.      Y    (m)      %20.4f%10.4f%20.4f\n", 2,dynamicDatum_Init.X0.y,  dynamicDatum.X0.y  - dynamicDatum_Init.X0.y,  dynamicDatum.X0.y);
					fprintf(pFitFile, "%3d.      Z    (m)      %20.4f%10.4f%20.4f\n", 3,dynamicDatum_Init.X0.z,  dynamicDatum.X0.z  - dynamicDatum_Init.X0.z,  dynamicDatum.X0.z);
					fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f%10.4f%20.4f\n", 4,dynamicDatum_Init.X0.vx, dynamicDatum.X0.vx - dynamicDatum_Init.X0.vx, dynamicDatum.X0.vx);
					fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f%10.4f%20.4f\n", 5,dynamicDatum_Init.X0.vy, dynamicDatum.X0.vy - dynamicDatum_Init.X0.vy, dynamicDatum.X0.vy);
					fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f%10.4f%20.4f\n", 6,dynamicDatum_Init.X0.vz, dynamicDatum.X0.vz - dynamicDatum_Init.X0.vz, dynamicDatum.X0.vz);
					int k_Parameter = 7;
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
					if(dynamicDatum.bOn_SolarPressureAcc && (dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA || dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_5PARA))
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
					if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
					{
						for(size_t s_i = 0; s_i < dynamicDatum.empiricalForceParaList.size(); s_i++)
						{// 20140320, 谷德峰修改, 使得三个方向经验力可选
							if(dynamicDatum.bOn_EmpiricalForceAcc_R)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].cos_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].cos_R * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].sin_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].sin_R * 1.0E+7);
							}
							if(dynamicDatum.bOn_EmpiricalForceAcc_T)
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
							}
							if(dynamicDatum.bOn_EmpiricalForceAcc_N)
							{
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
					}
					// 20170427, 谷德峰修改, 增加常值项经验力
					if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
					{
						for(size_t s_i = 0; s_i < dynamicDatum.empiricalForceParaList.size(); s_i++)
						{// 20140320, 谷德峰修改, 使得三个方向经验力可选
							if(dynamicDatum.bOn_EmpiricalForceAcc_R)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   R_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].a0_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a0_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_R * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].cos_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].cos_R * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].sin_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].sin_R * 1.0E+7);
							}
							if(dynamicDatum.bOn_EmpiricalForceAcc_T)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   T_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_T * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_T * 1.0E+7);
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
							}
							if(dynamicDatum.bOn_EmpiricalForceAcc_N)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   N_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_N * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_N * 1.0E+7);
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
					}
					fclose(pFitFile);
				}
				// 判断收敛条件
				double max_adjust_pos = 0;
				for(int i = 0; i < 3; i++)
					max_adjust_pos = max(max_adjust_pos, fabs(matdx.GetElement(i, 0)));
				rms_oc = 0; 
				for(int i = 0; i < int(obsOrbitList.size()); i++)
					for(int j = 0; j < 3; j++)
						rms_oc += matY.GetElement(i * 3 + j, 0) * matY.GetElement(i * 3 + j, 0);
				rms_oc = sqrt(rms_oc / (obsOrbitList.size() * 3));
				//sprintf(info, "rms_oc / max_adjust_pos = %10.4lf / %10.4lf", rms_oc, max_adjust_pos);
				//RuningInfoFile::Add(info);
				if(max_adjust_pos <= 2.0E-1 || k >= 6 || flag_done) // 阈值调整到0.2m, 初轨精度较差
				{
					if(flag_robust == false && flag_done == false && bResEdit)
					{
						flag_robust = true;
						rms_oc = 0.0; 
						size_t count_normalpoint = 0;
						for(int i = 0; i < int(obsOrbitList.size()); i++)
						{
							for(int j = 0; j < 3; j++)
							{
								count_normalpoint++;
								rms_oc += matY.GetElement(i * 3 + j, 0) * matY.GetElement(i * 3 + j, 0);
							}
						}
						rms_oc = sqrt(rms_oc / count_normalpoint);
						size_t count_outliers = 0;
						for(int i = 0; i < int(obsOrbitList.size()); i++)
						{
							for(int j = 0; j < 3; j++)
							{
								if(fabs(matY.GetElement(i * 3 + j, 0)) >= factor * rms_oc)
								{
									matWeight.SetElement(i, j, rms_oc / fabs(matY.GetElement(i * 3 + j, 0)));
									count_outliers++;
								}	
							}
						}
						flag_done = true;
						if(count_outliers > 0)
							continue;
					}
					break;
				}
			}
			if(!bForecast) // 不进行轨道预报
				return result;
			// 进行轨道预报
			TDT t0_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t0_forecast - 3600.0 * 8));  
			TDT t1_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t1_forecast - 3600.0 * 8));
			if(result)
			{
				vector<TimePosVel> orbitlist_ac;
				vector<Matrix> matRtPartiallist_ac;
				// 倒向积分, 积分区间 [para.T0, T_End   + h * 4], 为保证插值精度向两端进行扩展
				vector<TimePosVel> backwardOrbitlist_ac; 
			    vector<TimePosVel> forwardOrbitlist_ac; 
                double h = 75.0; // 20150308, 谷德峰
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
					double x_ecf[6];
					double x_j2000[6];
					x_j2000[0] = forecastOrbList[s_i].pos.x;  
					x_j2000[1] = forecastOrbList[s_i].pos.y;  
					x_j2000[2] = forecastOrbList[s_i].pos.z;
					x_j2000[3] = forecastOrbList[s_i].vel.x; 
					x_j2000[4] = forecastOrbList[s_i].vel.y; 
					x_j2000[5] = forecastOrbList[s_i].vel.z;
					GPST t_GPS = TimeCoordConvert::TDT2GPST(forecastOrbList[s_i].t);
					m_TimeCoordConvert.J2000_ECEF(t_GPS, x_j2000, x_ecf);
					forecastOrbList[s_i].t = m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(t_GPS)) + 3600.0 * 8; // 转换到北京时
					forecastOrbList[s_i].pos.x = x_ecf[0]; 
					forecastOrbList[s_i].pos.y = x_ecf[1]; 
					forecastOrbList[s_i].pos.z = x_ecf[2];
					forecastOrbList[s_i].vel.x = x_ecf[3]; 
					forecastOrbList[s_i].vel.y = x_ecf[4]; 
					forecastOrbList[s_i].vel.z = x_ecf[5];
				}
			}
			if(result)
			{
				sprintf(info, "dynamicGEOPOD_pos解算成功!(oc = %.4f)", rms_oc);
				RuningInfoFile::Add(info);
				printf("%s\n", info);
				return true;
			}
			else
			{
				sprintf(info, "dynamicGEOPOD_pos解算失败!");
				RuningInfoFile::Add(info);
				printf("%s\n", info);
				return false;
			}
		}

		bool TWRSatDynPOD::mainGEOPOD(SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval, bool bInitDynDatumEst, bool bForecast, bool bResEdit)
		{
			char info[200];
			// 1. 根据先验卫星轨道、测站坐标数据, 逐个测站进行观测数据编辑
			for(map<string, TWR_StaDatum>::iterator it_Sta = m_staDatumList.begin(); it_Sta != m_staDatumList.end(); ++it_Sta)
			{
				map<UTC, TWRObsLine>::iterator jt_obs = it_Sta->second.obsList.begin();
				while(jt_obs != it_Sta->second.obsList.end())
				{
					double P_J2000[6]; // 惯性坐标, 用于坐标系转换
					double P_ITRF[6];  // 地固坐标
					// 获得测站在 J2000 惯性系中的位置(Tr_GPS 时刻)
					POS3D staPos_J2000;
					POS6D geoPV_J2000, geoPV_ITRF;;
					P_ITRF[0] = it_Sta->second.pos_ITRF.x;
					P_ITRF[1] = it_Sta->second.pos_ITRF.y;
					P_ITRF[2] = it_Sta->second.pos_ITRF.z;
					GPST tr_gps = m_TimeCoordConvert.UTC2GPST(jt_obs->second.t - 3600.0 * 8);
					m_TimeCoordConvert.ECEF_J2000(tr_gps, P_J2000, P_ITRF, false);
					staPos_J2000.x = P_J2000[0];
					staPos_J2000.y = P_J2000[1];
					staPos_J2000.z = P_J2000[2];
					jt_obs->second.pos_J2000 = staPos_J2000;
					double delay = jt_obs->second.getRange() / SPEED_LIGHT; // 初始化延迟数据
					UTC tr_utc = jt_obs->second.t - delay;
					if(!m_orbJ2000File_0.getPosVel(tr_utc, geoPV_J2000))
					{
						if(m_twrDefine.on_ObsEditedInfo)
						{
							sprintf(info, "剔除数据 %s %s 轨道缺失.", it_Sta->first.c_str(), jt_obs->second.t.toString().c_str());
							RuningInfoFile::Add(info);
						}

						map<UTC, TWRObsLine>::iterator jt_erase = jt_obs;
						++jt_obs;
						it_Sta->second.obsList.erase(jt_erase); // 剔除数据
						continue;
					}
					// 获得卫星在地固系下位置
					P_J2000[0] = geoPV_J2000.x;
					P_J2000[1] = geoPV_J2000.y;
					P_J2000[2] = geoPV_J2000.z;
					P_J2000[3] = geoPV_J2000.vx;
					P_J2000[4] = geoPV_J2000.vy;
					P_J2000[5] = geoPV_J2000.vz;
					m_TimeCoordConvert.J2000_ECEF(m_TimeCoordConvert.UTC2GPST(tr_utc - 3600.0 * 8), P_J2000, P_ITRF, true);
					geoPV_ITRF.x  = P_ITRF[0];
					geoPV_ITRF.y  = P_ITRF[1];
					geoPV_ITRF.z  = P_ITRF[2];
					geoPV_ITRF.vx = P_ITRF[3];
					geoPV_ITRF.vy = P_ITRF[4];
					geoPV_ITRF.vz = P_ITRF[5];

					/*double r = sqrt(pow(it_Sta->second.pos_ITRF.x - geoPV_ITRF.x, 2) +
									  pow(it_Sta->second.pos_ITRF.y - geoPV_ITRF.y, 2) +
									  pow(it_Sta->second.pos_ITRF.z - geoPV_ITRF.z, 2));*/

					TDB t_TDB = m_TimeCoordConvert.GPST2TDB(tr_gps); // 获得TDB时间--提供太阳历参考时间
					double jd_TDB = m_TimeCoordConvert.DayTime2JD(t_TDB); // 获得儒略日
					//1. 对流层延迟
					jt_obs->second.dR_trop = 0.0;
					if(m_twrDefine.on_TropDelay)
					{
						if(!it_Sta->second.obsErrFile.getCorrect_R(jt_obs->second.t, jt_obs->second.dR_trop))
						{
							if(m_twrDefine.on_ObsEditedInfo)
							{
								sprintf(info, "剔除数据 %s %s 对流层缺失.", it_Sta->first.c_str(), jt_obs->second.t.toString().c_str());
								RuningInfoFile::Add(info);
							}

							map<UTC, TWRObsLine>::iterator jt_erase = jt_obs;
							++jt_obs;
							it_Sta->second.obsList.erase(jt_erase); // 剔除数据
							continue;
						}
					}
					// 是否机动删除数据
					if(m_twrDefine.on_DeleteManeuver)
					{
						if(judgeAttManeuverTime(jt_obs->second.t)) // 姿态机动
						{
							//sprintf(info, "剔除数据 %s %s 机动时刻.", it_Sta->first.c_str(), jt_obs->second.t.toString().c_str());
							//RuningInfoFile::Add(info);
							map<UTC, TWRObsLine>::iterator jt_erase = jt_obs;
							++jt_obs;
							it_Sta->second.obsList.erase(jt_erase); // 剔除数据
							continue;
						}
					}
					// 是否删除给定时间段数据
					if(m_twrDefine.on_DeleteData)
					{
						if(judgeDeleteData(jt_obs->second.t)) // 姿态机动
						{
							map<UTC, TWRObsLine>::iterator jt_erase = jt_obs;
							++jt_obs;
							it_Sta->second.obsList.erase(jt_erase); // 剔除数据
							continue;
						}
					}
					//2. 卫星反射器偏移改正
					jt_obs->second.dR_satpco = 0.0;
					if(m_twrDefine.on_SatPCO)
					{
						EULERANGLE eulerAngle;
						bool on_AttFile = m_twrAttFile.getAngle(jt_obs->second.t, eulerAngle);
						if(!on_AttFile)
						{
							eulerAngle.xRoll  = 0.0;
							eulerAngle.yPitch = 0.0;
							eulerAngle.zYaw   = 0.0;
						}
						// 是否进行加权处理
						if(m_twrDefine.on_WeightManeuver)
						{
							if(judgeAttManeuver(jt_obs->second.t)) // 姿态机动 && !on_AttFile
								jt_obs->second.weight = m_twrDefine.weightManeuver;
						}
						// 转换为弧度
						eulerAngle.xRoll  = eulerAngle.xRoll  * PI / 180.0;
						eulerAngle.yPitch = eulerAngle.yPitch * PI / 180.0;
						eulerAngle.zYaw   = eulerAngle.zYaw   * PI / 180.0;
						Matrix matATT = AttitudeTrans::getAttMatrix(eulerAngle); // 轨道系->星体系
						matATT = matATT.Transpose(); 
						Matrix mat_pco(3, 1);
						mat_pco.SetElement(0, 0, m_pcoAnt.x);
						mat_pco.SetElement(1, 0, m_pcoAnt.y);
						mat_pco.SetElement(2, 0, m_pcoAnt.z);
						mat_pco = matATT * mat_pco;
						POS3D S_Z; // Z 轴指向地心
						S_Z.x = -geoPV_J2000.x;
						S_Z.y = -geoPV_J2000.y;
						S_Z.z = -geoPV_J2000.z;
						POS3D S_X; // X 轴沿速度方向
						S_X.x = geoPV_J2000.vx;
						S_X.y = geoPV_J2000.vy;
						S_X.z = geoPV_J2000.vz;
						POS3D S_Y; // 右手系
						vectorCross(S_Y, S_Z, S_X);
						vectorCross(S_X, S_Y, S_Z);
						S_X = vectorNormal(S_X);
						S_Y = vectorNormal(S_Y);
						S_Z = vectorNormal(S_Z);
						POS3D vecLos = vectorNormal(geoPV_J2000.getPos() - staPos_J2000);
						POS3D vecPco = S_X * mat_pco.GetElement(0, 0) + S_Y * mat_pco.GetElement(1, 0) + S_Z * mat_pco.GetElement(2, 0);
						jt_obs->second.dR_satpco = vectorDot(vecPco, vecLos);

						//// 在天线系下更新计算 Azimuth 和 Elevation, 天线系坐标轴 XYZ 分别对应着 exBody -eyBody -ezBody
						//POS3D vecLosJ2000 = jt_obs->second.pos_J2000 - geoPV_J2000.getPos();
						//POS3D vecLosXYZ;
						//vecLosXYZ.x = vectorDot(vecLosJ2000, S_X);
						//vecLosXYZ.y = vectorDot(vecLosJ2000, S_Y); 
						//vecLosXYZ.z = vectorDot(vecLosJ2000, S_Z); // 星固系 Z 轴指向地心, 天线系 Z 轴指向天顶, X 轴重合
						//vecLosXYZ   = vectorNormal(vecLosXYZ); // 视线矢量在天线系XYZ下的投影
						////double elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;  // 高度角
						//double elevation = acos(vecLosXYZ.z) * 180 / PI;  // 高度角
						//if(elevation < 0)
						//{// 变换到[0, 360]
						//	elevation += 90.0;
						//}
						//double azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
						//if(azimuth < 0)
						//{// 变换到[0, 360]
						//	azimuth += 360.0;
						//}
		    // 			// 保存高度角和方位角文件
						//char szFileName[300];
						//sprintf(szFileName, "D:\\test.txt");
						//FILE* pFile = fopen(szFileName, "a+");
						//fprintf(pFile, "%s %s %20.4f %20.4f\n", it_Sta->first.c_str(), jt_obs->second.t.toString().c_str(), azimuth, elevation);
						//fclose(pFile);
					}
					//3. 相对论修正
					jt_obs->second.dR_GraRelativity = 0.0;
					if(m_twrDefine.on_GraRelativity)
					{
						jt_obs->second.dR_GraRelativity = GNSSBasicCorrectFunc::graRelativityCorrect(geoPV_J2000.getPos(), staPos_J2000);
					}
					//4. 固体潮修正
					jt_obs->second.dR_SolidTides = 0.0;
					if(m_twrDefine.on_OceanTides)
					{
						// 获得太阳位置 
						POS3D sunPos_ITRF;
						POS3D sunPos_J2000;
						m_JPLEphFile.getSunPos_Delay_EarthCenter(jd_TDB, P_J2000); 
						for(int i = 0; i < 3; i ++)
							P_J2000[i] = P_J2000[i] * 1000; // 换算成米
						sunPos_J2000.x = P_J2000[0];
						sunPos_J2000.y = P_J2000[1];
						sunPos_J2000.z = P_J2000[2];
						m_TimeCoordConvert.J2000_ECEF(tr_gps, P_J2000, P_ITRF, false); // 坐标系转换
						sunPos_ITRF.x = P_ITRF[0];
						sunPos_ITRF.y = P_ITRF[1];
						sunPos_ITRF.z = P_ITRF[2];
						// 获得月球的位置
						POS3D moonPos_ITRF;
						m_JPLEphFile.getPlanetPos(JPLEph_Moon, jd_TDB, P_J2000);  // 获得J2000系下的月球相对地心的位置（千米）
						for(int i = 0; i < 3; i ++)
							P_J2000[i] = P_J2000[i] * 1000;                       // 换算成米
						m_TimeCoordConvert.J2000_ECEF(tr_gps, P_J2000, P_ITRF, false); // 坐标系转换
						moonPos_ITRF.x  = P_ITRF[0];
						moonPos_ITRF.y  = P_ITRF[1];
						moonPos_ITRF.z  = P_ITRF[2];	
						double xp = 0;
						double yp = 0;
						//if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_2003)
						//	m_TimeCoordConvert.m_eopRapidFileIAU2000.getPoleOffset(m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(tr_gps)), xp, yp);
						// 根据文件类型进行修改，邵凯，2018/05/09
						if(m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04_1980 || m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04_2000A) 
							m_TimeCoordConvert.m_eopc04File.getPoleOffset(m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(tr_gps)),xp,yp); // 获得极移数据
						else if(m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04Total_1980 || m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04Total_2000A) 
							m_TimeCoordConvert.m_eopc04TotalFile.getPoleOffset(m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(tr_gps)),xp,yp); // 获得极移数据
						else
							m_TimeCoordConvert.m_eopRapidFileIAU2000.getPoleOffset(m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(tr_gps)), xp, yp); // 获得极移数据

						POS3D posSolidTide_ECEF = SolidTides::solidTideCorrect(tr_gps, sunPos_ITRF, moonPos_ITRF, it_Sta->second.pos_ITRF, xp, yp);
						POS3D vecLos = vectorNormal(geoPV_ITRF.getPos() - it_Sta->second.pos_ITRF);
						jt_obs->second.dR_SolidTides = - vectorDot(posSolidTide_ECEF, vecLos);							
					}
					// 测站的海潮波暂时是缺失的
					/*if(m_twrDefine.on_OceanTides)
					{
						StaOceanTide sotDatum;
						if(m_staOldFile.getStaOceanTide(name_A, sotDatum))
						{
							POS3D posOceanTide_ECEF = OceanTidesLoading::oceanTideLoadingCorrect(tr_gps, it_Sta->second.pos_ITRF, sotDatum);
							POS3D vecLos = vectorNormal(leoPV_ITRF.getPos() - it_Sta->second.pos_ITRF);
							jt_obs->second.dR_OceanTides = - vectorDot(posOceanTide_ECEF, vecLos);	
						}
					}*/
					// 总的延迟量
					jt_obs->second.corrected_value = jt_obs->second.dR_trop
						                           + jt_obs->second.dR_satpco
					                               + jt_obs->second.dR_GraRelativity
												   + jt_obs->second.dR_SolidTides;

					// 迭代计算下行信号 reflect 时间 tr_utc, 获得反射时刻卫星轨道位置
					double dDelay_k_1 = 0;
					double dR_down = jt_obs->second.getRange();
					TimePosVel orbit;
					while(fabs(delay - dDelay_k_1) > 1.0E-8)
					{
						// 更新延迟时间
						dDelay_k_1 = delay;
						// 根据 dDelay 计算下行信号 reflect 时间
						tr_utc  = jt_obs->second.t - delay; 
						orbit.t = tr_utc;
						// 获得 J2000 惯性系下的卫星轨道 
						m_orbJ2000File_0.getPosVel(tr_utc, geoPV_J2000);
						// 计算下行几何距离
						dR_down = sqrt(pow(staPos_J2000.x - geoPV_J2000.x, 2) +
									   pow(staPos_J2000.y - geoPV_J2000.y, 2) +
									   pow(staPos_J2000.z - geoPV_J2000.z, 2));
						delay = (dR_down + jt_obs->second.corrected_value) / SPEED_LIGHT;
					}
					// 反射时刻 tr_utc, 卫星的轨道位置 orbit
					// 迭代计算上行信号延迟时间
					dDelay_k_1 = 0;
					double dR_up = jt_obs->second.getRange();
					delay = dR_up / SPEED_LIGHT;
					UTC tt_utc = tr_utc - delay;
					while(fabs(delay - dDelay_k_1) > 1.0E-8)
					{// 更新延迟时间
						dDelay_k_1 = delay;
						// 根据 dDelay 计算地面信号时间
						tt_utc = tr_utc - delay;
						// 获得 J2000 惯性系下的观测站位置
						P_ITRF[0] = it_Sta->second.pos_ITRF.x;
						P_ITRF[1] = it_Sta->second.pos_ITRF.y;
						P_ITRF[2] = it_Sta->second.pos_ITRF.z;
						GPST tt_gps = m_TimeCoordConvert.UTC2GPST(tt_utc - 3600.0 * 8);
						m_TimeCoordConvert.ECEF_J2000(tt_gps, P_J2000, P_ITRF, false);
						POS3D staPos_J2000_tt;
						staPos_J2000_tt.x = P_J2000[0];
						staPos_J2000_tt.y = P_J2000[1];
						staPos_J2000_tt.z = P_J2000[2];
						// 计算上行几何距离
						dR_up = sqrt(pow(staPos_J2000_tt.x - geoPV_J2000.x, 2) +
									 pow(staPos_J2000_tt.y - geoPV_J2000.y, 2) +
									 pow(staPos_J2000_tt.z - geoPV_J2000.z, 2));
						delay = (dR_up + jt_obs->second.corrected_value) / SPEED_LIGHT;
					}
					jt_obs->second.dR_up = dR_up;
					jt_obs->second.dR_down = dR_down;
					//double dR = jt_obs->second.getRange();
                    jt_obs->second.oc = jt_obs->second.getRange()
						              - m_twrDefine.satZeroDelay_0
						              - it_Sta->second.zeroDelay_0
						              - jt_obs->second.corrected_value
									  - 0.5 * (dR_up + dR_down);

					//sprintf(info, "%s %10.1f %20.4f %20.4f %20.4f %20.4f.", it_Sta->first.c_str(), jt_obs->second.t - t0_forecast, oc, jt_obs->second.getRange(), jt_obs->second.corrected_value, 0.5 * (dR_up + dR_down));
					//RuningInfoFile::Add(info);

					// 计算高度角和方位角
					ENU geoENU;
					TimeCoordConvert::ECF2ENU(it_Sta->second.pos_ITRF, geoPV_ITRF.getPos(), geoENU);
					//jt_obs->second.Elevation = atan(geoENU.U / sqrt(geoENU.E * geoENU.E + geoENU.N * geoENU.N)) * 180 / PI;
					jt_obs->second.Azimuth = atan2(geoENU.E, geoENU.N) * 180 / PI;
					if(jt_obs->second.Azimuth < 0)
						jt_obs->second.Azimuth = jt_obs->second.Azimuth  + 360; // 保证方位角在 [0, 360) 之间
					// 20150608, 卫星仰角计算考虑到地球扁率影响, 谷德峰
					POS3D p_station = vectorNormal(it_Sta->second.pos_ITRF);
					POS3D p_sat = vectorNormal(geoPV_ITRF.getPos() - it_Sta->second.pos_ITRF);					
					p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); 
					p_station = vectorNormal(p_station);					
					jt_obs->second.Elevation = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;

					if(jt_obs->second.Elevation < m_twrDefine.min_Elevation || fabs(jt_obs->second.oc) > m_twrDefine.max_ocEdit)
					{
						if(m_twrDefine.on_ObsEditedInfo)
						{
							sprintf(info, "剔除数据 %s %s %10.1f %20.4f.", it_Sta->first.c_str(), jt_obs->second.t.toString().c_str(), jt_obs->second.Elevation, jt_obs->second.oc);
							RuningInfoFile::Add(info);
						}
						map<UTC, TWRObsLine>::iterator jt_erase = jt_obs;
						++jt_obs;
						it_Sta->second.obsList.erase(jt_erase); // 剔除数据
						continue;
					}

					++jt_obs;
					continue;
				}
			}
			// 剔除观测数据较少的测站
			int id_ZeroDelay = -1; 
			map<string, TWR_StaDatum>::iterator it_Sta = m_staDatumList.begin();
			while(it_Sta != m_staDatumList.end())
			{
				if(int(it_Sta->second.obsList.size()) <= m_twrDefine.min_ArcPointCount)
				{
					sprintf(info, "剔除测站 %s %d.", it_Sta->first.c_str(), it_Sta->second.obsList.size());
					RuningInfoFile::Add(info);
					map<string, TWR_StaDatum>::iterator jt_erase = it_Sta;
					++it_Sta;
					m_staDatumList.erase(jt_erase); // 剔除数据
					continue;
				}
				id_ZeroDelay++;
				it_Sta->second.id_ZeroDelay = id_ZeroDelay;
				++it_Sta;
				continue;
			}

			const int max_iterator = 3; // 迭代次数阈值
			int num_iterator = 0;       // 记录迭代次数
			int num_after_ocEdit = 0;
			int count_DynParameter = dynamicDatum.getAllEstParaCount(); 
			int count_SolarPressureParaList = 0;
			if(dynamicDatum.bOn_SolarPressureAcc)
				count_SolarPressureParaList = int(dynamicDatum.solarPressureParaList.size());
			int count_EmpiricalForceParaList = 0;
			if(dynamicDatum.bOn_EmpiricalForceAcc)
				count_EmpiricalForceParaList = int(dynamicDatum.empiricalForceParaList.size());
			int count_SolarPressurePara = dynamicDatum.getSolarPressureParaCount();
			int count_zeroDelay_Est = int(m_staDatumList.size()); // 每个测站增加一个参数, 部分测站不估计, 增加零值伪方程约束
			bool result = true;
			bool flag_break = false;
			bool flag_robust = false;
			SatdynBasicDatum dynamicDatum_Init = dynamicDatum; // 记录初始轨道根数
			char dynamicDatumFilePath[300];
			sprintf(dynamicDatumFilePath,"%s\\%s", m_strPODPath.c_str(), m_twrDefine.nameDynPodFitFile.c_str());
			double rms_oc = 0.0;
			while(1)
			{
				num_iterator++;
				if(num_iterator >= m_twrDefine.max_OrbIteration)
				{
					result = false;	// 2014/06/18,发散轨道比较用，鞠 冰
					sprintf(info, "迭代超过%d次, 发散!", num_iterator);
					printf("%s\n");
					RuningInfoFile::Add(info);
					break;
				}
				// 根据初轨进行轨道积分, 用于后续的概略距离修正
				adamsCowell_ac(t0_forecast, t1_forecast, dynamicDatum, m_acOrbitList, m_acRtPartialList);
				// 更新残差
				rms_oc = 0.0;
				int count_obs = 0;
				for(map<string, TWR_StaDatum>::iterator it_Sta = m_staDatumList.begin(); it_Sta != m_staDatumList.end(); ++it_Sta)
				{
					for(map<UTC, TWRObsLine>::iterator jt_obs = it_Sta->second.obsList.begin(); jt_obs != it_Sta->second.obsList.end(); ++jt_obs)
					{
						// 更新 dR_up 和 dR_down
						double P_J2000[6]; // 惯性坐标, 用于坐标系转换
						double P_ITRF[6];  // 地固坐标
						TimePosVel  geoOrb;
						Matrix geoRtPartial;
						double delay_down;
						if(!getEphemeris_PathDelay(jt_obs->second.t, jt_obs->second.pos_J2000, delay_down, geoOrb, geoRtPartial))
							continue;
						jt_obs->second.geoOrb = geoOrb;
						jt_obs->second.geoRtPartial = geoRtPartial;
						jt_obs->second.dR_down = sqrt(pow(jt_obs->second.pos_J2000.x - geoOrb.pos.x, 2) +
									                  pow(jt_obs->second.pos_J2000.y - geoOrb.pos.y, 2) +
									                  pow(jt_obs->second.pos_J2000.z - geoOrb.pos.z, 2));
						UTC tr_utc = jt_obs->second.t - delay_down; // 反射时刻 tr_utc
						// 迭代计算上行信号延迟时间
						double dDelay_k_1 = 0;
						double dR_up = jt_obs->second.getRange();
						double delay_up = dR_up / SPEED_LIGHT;
						UTC tt_utc = tr_utc - delay_up;
						while(fabs(delay_up - dDelay_k_1) > 1.0E-8)
						{// 更新延迟时间
							dDelay_k_1 = delay_up;
							// 根据 dDelay 计算地面信号时间
							tt_utc = tr_utc - delay_up;
							// 获得 J2000 惯性系下的观测站位置
							P_ITRF[0] = it_Sta->second.pos_ITRF.x;
							P_ITRF[1] = it_Sta->second.pos_ITRF.y;
							P_ITRF[2] = it_Sta->second.pos_ITRF.z;
							GPST tt_gps = m_TimeCoordConvert.UTC2GPST(tt_utc - 3600.0 * 8);
							m_TimeCoordConvert.ECEF_J2000(tt_gps, P_J2000, P_ITRF, false);
							POS3D staPos_J2000_tt;
							staPos_J2000_tt.x = P_J2000[0];
							staPos_J2000_tt.y = P_J2000[1];
							staPos_J2000_tt.z = P_J2000[2];
							// 计算上行几何距离
							dR_up = sqrt(pow(staPos_J2000_tt.x - geoOrb.pos.x, 2) +
										 pow(staPos_J2000_tt.y - geoOrb.pos.y, 2) +
										 pow(staPos_J2000_tt.z - geoOrb.pos.z, 2));
							delay_up = (dR_up + jt_obs->second.corrected_value) / SPEED_LIGHT;
						}
					    jt_obs->second.dR_up = dR_up;
						jt_obs->second.oc = jt_obs->second.getRange()
							              - m_twrDefine.satZeroDelay_0
										  - m_twrDefine.satZeroDelay_Est
						                  - it_Sta->second.zeroDelay_0
										  - it_Sta->second.zeroDelay_Est
						                  - jt_obs->second.corrected_value
										  - 0.5 * (jt_obs->second.dR_up + jt_obs->second.dR_down);

						if(jt_obs->second.rw == 1.0)
						{
							rms_oc += pow(jt_obs->second.oc * jt_obs->second.weight, 2);
							count_obs++;
						}
					}
				}
				rms_oc = sqrt(rms_oc / count_obs);
				sprintf(info, "第%d次 adamsCowell_Interp is ok!(rms_oc = %8.4f)", num_iterator, rms_oc);
				RuningInfoFile::Add(info);
				printf("%s\n", info);

				if(flag_break)
				{
					/*for(map<string, TWR_StaDatum>::iterator it_Sta = m_staDatumList.begin(); it_Sta != m_staDatumList.end(); ++it_Sta)
					{
						for(map<UTC, TWRObsLine>::iterator jt_obs = it_Sta->second.obsList.begin(); jt_obs != it_Sta->second.obsList.end(); ++jt_obs)
						{
							if(jt_obs->second.rw == 1.0)
							{
								sprintf(info, "%s %10.1f %20.4f %20.4f %20.4f.", it_Sta->first.c_str(), jt_obs->second.t - t0_forecast, jt_obs->second.oc, jt_obs->second.dR_trop, jt_obs->second.dR_satpco);
								RuningInfoFile::Add(info);	
							}
						}
					}*/
					break;
				}

				// 残差编辑
				if(flag_robust && bResEdit)
				{
					for(map<string, TWR_StaDatum>::iterator it_Sta = m_staDatumList.begin(); it_Sta != m_staDatumList.end(); ++it_Sta)
					{
						for(map<UTC, TWRObsLine>::iterator jt_obs = it_Sta->second.obsList.begin(); jt_obs != it_Sta->second.obsList.end(); ++jt_obs)
						{
							if(fabs(jt_obs->second.oc * jt_obs->second.weight) > rms_oc * m_twrDefine.ratio_ocEdit)
							{
								jt_obs->second.rw = rms_oc / fabs(jt_obs->second.oc * jt_obs->second.weight);

								/*if(fabs(jt_obs->second.oc) >= m_twrDefine.max_ocEdit)
								{
									sprintf(info, "残差编辑 oc %s %s %8.2lf.", it_Sta->first.c_str(), jt_obs->second.t.toString().c_str(), jt_obs->second.oc);
									RuningInfoFile::Add(info);
								}*/

							}
							else
								jt_obs->second.rw = 1.0;
						}
					}
					num_after_ocEdit++;					
				}

				// 设计矩阵
				Matrix n_xx(count_DynParameter + count_zeroDelay_Est + int(m_twrDefine.on_EstZeroDelay), count_DynParameter + count_zeroDelay_Est + int(m_twrDefine.on_EstZeroDelay));
				Matrix nx(count_DynParameter + count_zeroDelay_Est + int(m_twrDefine.on_EstZeroDelay), 1);
				for(map<string, TWR_StaDatum>::iterator it_Sta = m_staDatumList.begin(); it_Sta != m_staDatumList.end(); ++it_Sta)
				{
					for(map<UTC, TWRObsLine>::iterator jt_obs = it_Sta->second.obsList.begin(); jt_obs != it_Sta->second.obsList.end(); ++jt_obs)
					{
						POS3D vecLos = vectorNormal(jt_obs->second.geoOrb.pos - jt_obs->second.pos_J2000);
						double w = jt_obs->second.rw * jt_obs->second.weight;
						Matrix matH_pos_j(1, 3);
						matH_pos_j.SetElement(0, 0, w * vecLos.x);
						matH_pos_j.SetElement(0, 1, w * vecLos.y);
						matH_pos_j.SetElement(0, 2, w * vecLos.z);
						Matrix matHx_j(1, count_DynParameter + count_zeroDelay_Est + int(m_twrDefine.on_EstZeroDelay));
						for(int s_k = 0; s_k < 6; s_k++)
						{// 初始位置速度
							double sum_posvel = jt_obs->second.geoRtPartial.GetElement(0, s_k) * matH_pos_j.GetElement(0, 0) 
											  + jt_obs->second.geoRtPartial.GetElement(1, s_k) * matH_pos_j.GetElement(0, 1)
											  + jt_obs->second.geoRtPartial.GetElement(2, s_k) * matH_pos_j.GetElement(0, 2);
							matHx_j.SetElement(0, s_k, sum_posvel);
						}
						int beginPara = 6;
						// 兼容多光压模型计算, 谷德峰
						if(dynamicDatum.bOn_SolarPressureAcc)
						{
							for(int j = 0; j < int(dynamicDatum.solarPressureParaList.size() * count_SolarPressurePara); j++)
							{
								double sum_solar = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j) * matH_pos_j.GetElement(0, 0) 
												 + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j) * matH_pos_j.GetElement(0, 1)
												 + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j) * matH_pos_j.GetElement(0, 2);
								matHx_j.SetElement(0, beginPara + j, sum_solar);
							}
							beginPara += count_SolarPressurePara * count_SolarPressureParaList;
						}
						if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
						{// 经验力
							int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
											 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2  
											 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 2; 
							for(int j = 0; j < int(dynamicDatum.empiricalForceParaList.size()); j++)
							{
								int i_sub = 0;
								if(dynamicDatum.bOn_EmpiricalForceAcc_R)
								{
									double sum_cr = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_cr);

									double sum_sr = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_sr);
								}
								i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2;
								if(dynamicDatum.bOn_EmpiricalForceAcc_T)
								{
									double sum_ct = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_ct);

									double sum_st = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_st);
								}
								i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2;
								if(dynamicDatum.bOn_EmpiricalForceAcc_N)
								{
									double sum_cn = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_cn);

									double sum_sn = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_sn);
								}
							}
							beginPara += count_sub * count_EmpiricalForceParaList;
						}if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
						{// 经验力
							int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
											 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 3  
											 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 3; 
							for(int j = 0; j < int(dynamicDatum.empiricalForceParaList.size()); j++)
							{
								int i_sub = 0;
								if(dynamicDatum.bOn_EmpiricalForceAcc_R)
								{
									double sum_a0 = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
								i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3;
								if(dynamicDatum.bOn_EmpiricalForceAcc_T)
								{
									double sum_a0 = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
								i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 3;
								if(dynamicDatum.bOn_EmpiricalForceAcc_N)
								{
									double sum_a0 = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr = jt_obs->second.geoRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.geoRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.geoRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
							}
							beginPara += count_sub * count_EmpiricalForceParaList;
						}
						// 测站延迟参数
						matHx_j.SetElement(0, count_DynParameter + it_Sta->second.id_ZeroDelay, w);
						// 卫星延迟参数
						if(m_twrDefine.on_EstZeroDelay)
							matHx_j.SetElement(0, count_DynParameter + count_zeroDelay_Est, w);
						
						n_xx = n_xx + matHx_j.Transpose() * matHx_j;
                        nx = nx + matHx_j.Transpose() * (w * jt_obs->second.oc);
					}
				}

				// 添加伪方程-光压参数
				for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
				{
					double weight_solar = 1.0E+12;	
					if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
					{
						if(!m_twrDefine.on_SRP9_D0)
						{// △D0[0]  = -(D0*) + ε									
							int index_D0 = 6 + 9 * (int)s_k + 0;
							n_xx.SetElement(index_D0, index_D0,  n_xx.GetElement(index_D0, index_D0) + weight_solar * weight_solar);
							nx.SetElement(index_D0, 0,           nx.GetElement(index_D0, 0)          - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].D0);
						}
						if(!m_twrDefine.on_SRP9_DC1)
						{// △DCOS[1]  = -(DCOS*) + ε								
							int index_DCOS = 6 + 9 * (int)s_k + 1;
							n_xx.SetElement(index_DCOS, index_DCOS,  n_xx.GetElement(index_DCOS, index_DCOS) + weight_solar * weight_solar);
							nx.SetElement(index_DCOS, 0,             nx.GetElement(index_DCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].DC1);
						}
						if(!m_twrDefine.on_SRP9_DS1)
						{// △DSIN[2]  = -(DSIN*) + ε								
							int index_DSIN = 6 + 9 * (int)s_k + 2;
							n_xx.SetElement(index_DSIN, index_DSIN,  n_xx.GetElement(index_DSIN, index_DSIN) + weight_solar * weight_solar);
							nx.SetElement(index_DSIN, 0,             nx.GetElement(index_DSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].DS1);
						}
						if(!m_twrDefine.on_SRP9_Y0)
						{// △Y0[3]  = -(Y0*) + ε								
							int index_Y0 = 6 + 9 * (int)s_k + 3;
							n_xx.SetElement(index_Y0, index_Y0,  n_xx.GetElement(index_Y0, index_Y0) + weight_solar * weight_solar);
							nx.SetElement(index_Y0, 0,           nx.GetElement(index_Y0, 0)          - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].Y0);
						}
						if(!m_twrDefine.on_SRP9_YC1)
						{// △YCOS[4]  = -(YCOS*) + ε								
							int index_YCOS = 6 + 9 * (int)s_k + 4;
							n_xx.SetElement(index_YCOS, index_YCOS,  n_xx.GetElement(index_YCOS, index_YCOS) + weight_solar * weight_solar);
							nx.SetElement(index_YCOS, 0,             nx.GetElement(index_YCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].YC1);
						}
						if(!m_twrDefine.on_SRP9_YS1)
						{// △YSIN[5]  = -(YSIN*) + ε								
							int index_YSIN = 6 + 9 * (int)s_k + 5;
							n_xx.SetElement(index_YSIN, index_YSIN,  n_xx.GetElement(index_YSIN, index_YSIN) + weight_solar * weight_solar);
							nx.SetElement(index_YSIN, 0,             nx.GetElement(index_YSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].YS1);
						}
						if(!m_twrDefine.on_SRP9_B0)
						{// △B0[6]  = -(B0*) + ε								
							int index_B0 = 6 + 9 * (int)s_k + 6;
							n_xx.SetElement(index_B0, index_B0,  n_xx.GetElement(index_B0, index_B0) + weight_solar * weight_solar);
							nx.SetElement(index_B0, 0,           nx.GetElement(index_B0, 0)          - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].B0);
						}
						if(!m_twrDefine.on_SRP9_BC1)
						{// △BCOS[7]  = -(BCOS*) + ε								
							int index_BCOS = 6 + 9 * (int)s_k + 7;
							n_xx.SetElement(index_BCOS, index_BCOS,  n_xx.GetElement(index_BCOS, index_BCOS) + weight_solar * weight_solar);
							nx.SetElement(index_BCOS, 0,             nx.GetElement(index_BCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].BC1);
						}
						if(!m_twrDefine.on_SRP9_BS1)
						{// △BSIN[8]  = -(BSIN*) + ε								
							int index_BSIN = 6 + 9 * (int)s_k + 8;
							n_xx.SetElement(index_BSIN, index_BSIN,  n_xx.GetElement(index_BSIN, index_BSIN) + weight_solar * weight_solar);
							nx.SetElement(index_BSIN, 0,             nx.GetElement(index_BSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].BS1);
						}
					}
				}

				// 添加伪方程-测站延迟参数
				for(map<string, TWR_StaDatum>::iterator it_Sta = m_staDatumList.begin(); it_Sta != m_staDatumList.end(); ++it_Sta)
				{
					double weight_ZeroDelay = 0.0;
					if(!it_Sta->second.on_EstZeroDelay) // 对于不估计偏差的测站增加强约束
						weight_ZeroDelay = 1.0E+4; // 伪方程的观测权值不应过大, 防止大数吃小数			 
					int index_ZeroDelay = count_DynParameter + it_Sta->second.id_ZeroDelay;
					nx.SetElement(index_ZeroDelay, 0, nx.GetElement(index_ZeroDelay, 0) - weight_ZeroDelay * weight_ZeroDelay * it_Sta->second.zeroDelay_Est);
					n_xx.SetElement(index_ZeroDelay, index_ZeroDelay, n_xx.GetElement(index_ZeroDelay, index_ZeroDelay) + weight_ZeroDelay * weight_ZeroDelay);
				}

				// 轨道改进
				Matrix matdx = n_xx.Inv_Ssgj() * nx;

				//RuningInfoFile::Add(matdx.ToString().c_str());

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
				// 2015/10/18, 兼容多光压模型计算, 谷德峰
				else if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
				{
					// 20140320, 谷德峰添加
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
					}
					beginPara += 9 * count_SolarPressureParaList;
				}
				else if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_5PARA)	
				{
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
					{
						dynamicDatum.solarPressureParaList[s_k].D0  += matdx.GetElement(beginPara + 5 * s_k,     0);
						dynamicDatum.solarPressureParaList[s_k].Y0  += matdx.GetElement(beginPara + 5 * s_k + 1, 0);
						dynamicDatum.solarPressureParaList[s_k].B0  += matdx.GetElement(beginPara + 5 * s_k + 2, 0);
						dynamicDatum.solarPressureParaList[s_k].BC1 += matdx.GetElement(beginPara + 5 * s_k + 3, 0);
						dynamicDatum.solarPressureParaList[s_k].BS1 += matdx.GetElement(beginPara + 5 * s_k + 4, 0);
					}
				}
				if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
				{// 经验力
					int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
									 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2  
									 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 2; 
					for(int s_k = 0; s_k < int(dynamicDatum.empiricalForceParaList.size()); s_k++)
					{
						int i_sub = 0;
						if(dynamicDatum.bOn_EmpiricalForceAcc_R)
						{
							dynamicDatum.empiricalForceParaList[s_k].cos_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2;
						if(dynamicDatum.bOn_EmpiricalForceAcc_T)
						{
							dynamicDatum.empiricalForceParaList[s_k].cos_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2;
                        if(dynamicDatum.bOn_EmpiricalForceAcc_N)
						{
							dynamicDatum.empiricalForceParaList[s_k].cos_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
					}
					beginPara += count_sub * count_EmpiricalForceParaList;
				}
				else if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
				{// 经验力
					int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
									 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 3  
									 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 3; 
					for(int s_k = 0; s_k < int(dynamicDatum.empiricalForceParaList.size()); s_k++)
					{
						int i_sub = 0;
						if(dynamicDatum.bOn_EmpiricalForceAcc_R)
						{
							dynamicDatum.empiricalForceParaList[s_k].a0_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].cos_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 1;
						if(dynamicDatum.bOn_EmpiricalForceAcc_T)
						{
							dynamicDatum.empiricalForceParaList[s_k].a0_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].cos_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 1 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 1;
                        if(dynamicDatum.bOn_EmpiricalForceAcc_N)
						{
							dynamicDatum.empiricalForceParaList[s_k].a0_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].cos_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
					}
					beginPara += count_sub * count_EmpiricalForceParaList;
				}

				for(map<string, TWR_StaDatum>::iterator it_Sta = m_staDatumList.begin(); it_Sta != m_staDatumList.end(); ++it_Sta)
				{
					it_Sta->second.zeroDelay_Est += matdx.GetElement(count_DynParameter + it_Sta->second.id_ZeroDelay, 0);
				}

				if(m_twrDefine.on_EstZeroDelay)
					m_twrDefine.satZeroDelay_Est += matdx.GetElement(count_DynParameter + count_zeroDelay_Est, 0);

				if(!m_strPODPath.empty())
				{
					// 记录轨道改进结果
					FILE * pFitFile = fopen(dynamicDatumFilePath, "w+");
					fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
					int k_Parameter = 0;
					fprintf(pFitFile, "%3d. %-4s Delay(m)      %20.4f%10.4f%20.4f\n", k_Parameter, "sat", m_twrDefine.satZeroDelay_0,  m_twrDefine.satZeroDelay_Est,  m_twrDefine.satZeroDelay_0 + m_twrDefine.satZeroDelay_Est);
					k_Parameter++;
					for(map<string, TWR_StaDatum>::iterator it_Sta = m_staDatumList.begin(); it_Sta != m_staDatumList.end(); ++it_Sta)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %-4s Delay(m)      %20.4f%10.4f%20.4f\n", k_Parameter, it_Sta->first.c_str(),it_Sta->second.zeroDelay_0,  it_Sta->second.zeroDelay_Est,  it_Sta->second.zeroDelay_0 + it_Sta->second.zeroDelay_Est);
					}
					fprintf(pFitFile, "%3d.      X    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 1,dynamicDatum_Init.X0.x,  dynamicDatum.X0.x  - dynamicDatum_Init.X0.x,  dynamicDatum.X0.x);
					fprintf(pFitFile, "%3d.      Y    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 2,dynamicDatum_Init.X0.y,  dynamicDatum.X0.y  - dynamicDatum_Init.X0.y,  dynamicDatum.X0.y);
					fprintf(pFitFile, "%3d.      Z    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 3,dynamicDatum_Init.X0.z,  dynamicDatum.X0.z  - dynamicDatum_Init.X0.z,  dynamicDatum.X0.z);
					fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 4,dynamicDatum_Init.X0.vx, dynamicDatum.X0.vx - dynamicDatum_Init.X0.vx, dynamicDatum.X0.vx);
					fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 5,dynamicDatum_Init.X0.vy, dynamicDatum.X0.vy - dynamicDatum_Init.X0.vy, dynamicDatum.X0.vy);
					fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 6,dynamicDatum_Init.X0.vz, dynamicDatum.X0.vz - dynamicDatum_Init.X0.vz, dynamicDatum.X0.vz);
					k_Parameter += 6;
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
					if(dynamicDatum.bOn_SolarPressureAcc && (dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA || dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_5PARA))
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
					if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
					{
						for(size_t s_i = 0; s_i < dynamicDatum.empiricalForceParaList.size(); s_i++)
						{// 20140320, 谷德峰修改, 使得三个方向经验力可选
							if(dynamicDatum.bOn_EmpiricalForceAcc_R)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].cos_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].cos_R * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].sin_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].sin_R * 1.0E+7);
							}
							if(dynamicDatum.bOn_EmpiricalForceAcc_T)
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
							}
							if(dynamicDatum.bOn_EmpiricalForceAcc_N)
							{
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
					}
					// 20170427, 谷德峰修改, 增加常值项经验力
					if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
					{
						for(size_t s_i = 0; s_i < dynamicDatum.empiricalForceParaList.size(); s_i++)
						{// 20140320, 谷德峰修改, 使得三个方向经验力可选
							if(dynamicDatum.bOn_EmpiricalForceAcc_R)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   R_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].a0_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a0_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_R * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].cos_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].cos_R * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].sin_R * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].sin_R * 1.0E+7);
							}
							if(dynamicDatum.bOn_EmpiricalForceAcc_T)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   T_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_T * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_T * 1.0E+7);
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
							}
							if(dynamicDatum.bOn_EmpiricalForceAcc_N)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   N_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   dynamicDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_N * 1.0E+7 - dynamicDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																								   dynamicDatum.empiricalForceParaList[s_i].a0_N * 1.0E+7);
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
					}
					fclose(pFitFile);
				}

				// 判断收敛条件
				double max_adjust_pos = 0;
				for(int i = 0; i < 3; i++)
					max_adjust_pos = max(max_adjust_pos, fabs(matdx.GetElement(i, 0)));

				//sprintf(info, "max_adjust_pos = %10.4lf.", max_adjust_pos);
				//RuningInfoFile::Add(info);

				if(max_adjust_pos <= 1.0E-1 || num_iterator >= max_iterator || num_after_ocEdit > 0) // 阈值调整到0.2m, 初轨精度较差
				{
					// 首次进行残差编辑，2014/5/18，鞠 冰
					if(flag_robust == false && bResEdit)
					{
						flag_robust = true; 
						continue;
					}
					else
					{
						if(bResEdit && num_after_ocEdit <= 1) // 编辑后迭代2次
							flag_break = false;
						else
							flag_break = true;
					}
				}
			}
			if(!bForecast) // 不进行轨道预报
				return result;
			// 进行轨道预报
			TDT t0_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t0_forecast - 3600.0 * 8));  
			TDT t1_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t1_forecast - 3600.0 * 8));
			if(result)
			{
				vector<TimePosVel> orbitlist_ac;
				vector<Matrix> matRtPartiallist_ac;
				// 倒向积分, 积分区间 [para.T0, T_End   + h * 4], 为保证插值精度向两端进行扩展
				vector<TimePosVel> backwardOrbitlist_ac; 
			    vector<TimePosVel> forwardOrbitlist_ac; 
                double h = 75.0; // 20150308, 谷德峰
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
					double x_ecf[6];
					double x_j2000[6];
					x_j2000[0] = forecastOrbList[s_i].pos.x;  
					x_j2000[1] = forecastOrbList[s_i].pos.y;  
					x_j2000[2] = forecastOrbList[s_i].pos.z;
					x_j2000[3] = forecastOrbList[s_i].vel.x; 
					x_j2000[4] = forecastOrbList[s_i].vel.y; 
					x_j2000[5] = forecastOrbList[s_i].vel.z;
					GPST t_GPS = TimeCoordConvert::TDT2GPST(forecastOrbList[s_i].t);
					m_TimeCoordConvert.J2000_ECEF(t_GPS, x_j2000, x_ecf);
					forecastOrbList[s_i].t = m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(t_GPS)) + 3600.0 * 8; // 转换到北京时
					forecastOrbList[s_i].pos.x = x_ecf[0]; 
					forecastOrbList[s_i].pos.y = x_ecf[1]; 
					forecastOrbList[s_i].pos.z = x_ecf[2];
					forecastOrbList[s_i].vel.x = x_ecf[3]; 
					forecastOrbList[s_i].vel.y = x_ecf[4]; 
					forecastOrbList[s_i].vel.z = x_ecf[5];
				}
			}
			return result;
		}
	}
}
