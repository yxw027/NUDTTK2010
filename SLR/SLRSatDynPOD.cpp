#include "SLRSatDynPOD.hpp"
#include "cstgSLRObsFile.hpp"
#include "meritSLRObsFile.hpp"
#include "crdSLRObsFile.hpp"
#include "SLRPreproc.hpp"

namespace NUDTTK
{
	namespace SLR
	{
		SLRSatDynPOD::SLRSatDynPOD(void)
		{
			m_constRangeBias = 0.0;
			m_stepAdamsCowell = 10.0;
			m_bChecksum = true;
		}

		SLRSatDynPOD::~SLRSatDynPOD(void)
		{
		}

		void SLRSatDynPOD::setStepAdamsCowell(double step)
		{
			m_stepAdamsCowell = step;
		}

		// 获得激光测站的坐标
		bool SLRSatDynPOD::getStaPosvel(UTC t, int id, POS6D& posvel)
		{
			bool bFind = false;
			int count = int(m_staSscList.size());
			if(count <= 0)
				return false;
			int i;
			for(i =  count - 1; i >= 0; i--)
			{// 倒序查找最近的测站信息, 20080121
				if(m_staSscList[i].id == id)
				{
					bFind = true;
					break;
				}
			}
			if(!bFind)
			{
				//cout<<StationID<<endl;
				return false;
			}
			posvel.x  = m_staSscList[i].x;
			posvel.y  = m_staSscList[i].y;
			posvel.z  = m_staSscList[i].z;
			posvel.vx = m_staSscList[i].vx;
			posvel.vy = m_staSscList[i].vy;
			posvel.vz = m_staSscList[i].vz;
			double year = (t - m_staSscList[i].t0) / (86400 * 365.25);
			posvel.x += posvel.vx * year;
			posvel.y += posvel.vy * year;
			posvel.z += posvel.vz * year;
			return true;
		}

		// 获得激光测站的偏心数据
		bool SLRSatDynPOD::getStaEcc(UTC t, int id, ENU& ecc)
		{
			for(size_t s_i = 0; s_i < m_staEccList.size(); s_i++)
			{
				if( m_staEccList[s_i].id == id
				 && t - m_staEccList[s_i].t0 >= 0 
				 && t - m_staEccList[s_i].t1 <= 0)
				{
					ecc = m_staEccList[s_i].ecc;
					return true;
				}
			}
			return false;
		}

		// 子程序名称： weighting_Elevation   
		// 功能：根据高度角获得观测权重
		// 变量类型：Elevation        : 观测高度角
		//           weight           : 观测权值
		// 输入：Elevation
		// 输出：weight
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/07/10
		// 版本时间：
		// 修改记录：
		// 备注： 
		void SLRSatDynPOD::weighting_Elevation(double Elevation, double& weight)
		{
			if(Elevation <= m_podDefine.min_Elevation)
			{
				weight = 0.0;
			}
			else
			{
				weight = 1.0;
			}
		}

		// 子程序名称： getEphemeris   
		// 功能：滑动lagrange插值获得任意时刻TQ卫星星历
		// 变量类型： t                     :  UTC北京时
		//            satOrb                :  星历数值, 坐标单位: 米
		//            satRtPartial          : 卫星的偏导数
		//            nLagrange             :  Lagrange 插值已知点个数, 默认为 9, 对应 8 阶 Lagrange 插值
		// 输入：t,  nLagrange
		// 输出：satOrb,satRtPartial
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2012/12/31
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool SLRSatDynPOD::getEphemeris(UTC t, TimePosVel& satOrb, Matrix& satRtPartial, int nLagrange)
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
			satOrb.t = t;
			double *x = new double [nlagrange];
			double *y = new double [nlagrange];
			for(int i = nBegin; i <= nEnd; i++)
				x[i - nBegin] = m_acOrbitList[i].t - m_acOrbitList[0].t; // 参考相对时间点
			// X
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].pos.x;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.pos.x);
			// Y
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].pos.y;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.pos.y);
			// Z
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].pos.z;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.pos.z);
			// Vx
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].vel.x;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.vel.x);
			// Vy
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].vel.y;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.vel.y);
			// Vz
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = m_acOrbitList[i].vel.z;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.vel.z);
			delete x;
			delete y;

			// 偏导数部分
			if(int(m_acRtPartialList.size()) < 2) 
				return false;
			const int countDynParameter = m_acRtPartialList[0].GetNumColumns(); 
			satRtPartial.Init(3, countDynParameter);
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
					satRtPartial.SetElement(ii, jj, element);
				}
			}
			return true;
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
		// 备注： 默认步长选取60s, 11阶, 参考MIT
		bool SLRSatDynPOD::adamsCowell_ac(UTC t0, UTC t1, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h, int q)
		{
			orbitlist_ac.clear();
			matRtPartiallist_ac.clear();
			TDT t_Begin = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t0));
			TDT t_End   = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t1));
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
			// TDT -> UTC
			for(size_t s_i = 0; s_i < orbitlist_ac.size(); s_i++)
				orbitlist_ac[s_i].t = m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::TDT2TAI(orbitlist_ac[s_i].t)); 
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
		bool SLRSatDynPOD::initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength, double h)
		{
			char info[200];
			// 提取粗间隔的插值点
			double  threshold_coarseorbit_interval = m_podDefine.span_InitDynDatumCoarsePos;
			//double  threshold_coarseorbit_interval = 300.0;
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
				GPST t_GPS = m_TimeCoordConvert.UTC2GPST(orbitlist[s_i].t);
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
				if(k >= m_podDefine.max_OrbIteration)
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

		// 子程序名称： dynamicPOD_pos
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
		// 版本时间：2019/01/11
		// 修改记录：
		// 备注： 
		bool SLRSatDynPOD::dynamicPOD_pos(vector<TimePosVel> obsOrbitList, SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval, bool bInitDynDatumEst, bool bForecast, bool bResEdit)
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
				GPST t_GPS = m_TimeCoordConvert.UTC2GPST(obsOrbitList[obsOrbitList.size() - 1].t);
				TDT t_End = TimeCoordConvert::GPST2TDT(t_GPS);
				dynamicDatum.T0 = dynamicDatum_0.T0;
				dynamicDatum.ArcLength = t_End - dynamicDatum.T0;
				dynamicDatum.X0 = dynamicDatum_0.X0;
				dynamicDatum.init(m_podDefine.period_SolarPressure, m_podDefine.period_AtmosphereDrag, m_podDefine.period_EmpiricalAcc);
			}
			SatdynBasicDatum dynamicDatum_Init = dynamicDatum; // 记录初始轨道根数
			char dynamicDatumFilePath[300];
			sprintf(dynamicDatumFilePath,"%s\\%s", m_strPODPath.c_str(), m_podDefine.nameDynPodFitFile.c_str());
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
				GPST t_GPS = m_TimeCoordConvert.UTC2GPST(obsOrbitList[s_i].t);
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

			int count_AtmosphereDragParaList = 0;
			if(dynamicDatum.bOn_AtmosphereDragAcc)
				count_AtmosphereDragParaList = int(dynamicDatum.atmosphereDragParaList.size());

			int count_EmpiricalForceParaList = 0;
			if(dynamicDatum.bOn_EmpiricalForceAcc)
				count_EmpiricalForceParaList = int(dynamicDatum.empiricalForceParaList.size());
			
			int count_SolarPressurePara = dynamicDatum.getSolarPressureParaCount();
			
			int index_EmpiricalParaBegin = 0; // 记录经验力参数在整个待估动力学参数列表中的位置，2014/10/07，谷德峰
			int k = 0; // 记录迭代的次数
			bool flag_robust = false;
			bool flag_done = false;
			double factor = 3.0;
			Matrix matWeight(int(obsOrbitList.size()), 3); // 观测权矩阵            //这里的权重矩阵是3列，分别表示什么？
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
				if(k >= m_podDefine.max_OrbIteration)
				{
					result = false;
					printf("迭代超过%d次, 发散!", k);
					break;
				}
				vector<TimePosVel> interpOrbitlist; // 插值序列
				vector<Matrix> interpRtPartiallist; // 插值偏导数序列
				adamsCowell_Interp(interpTimelist, dynamicDatum, interpOrbitlist, interpRtPartiallist, m_stepAdamsCowell);
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
					if(dynamicDatum.bOn_AtmosphereDragAcc) 
					{// 大气阻力
						for(int j = 0; j < int(dynamicDatum.atmosphereDragParaList.size()); j++)
						{
							matH.SetElement(i * 3 + 0, beginPara + j, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j));
							matH.SetElement(i * 3 + 1, beginPara + j, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j));
							matH.SetElement(i * 3 + 2, beginPara + j, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j));
						}
						beginPara += count_AtmosphereDragParaList;
					}
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

				Matrix n_xx(count_DynParameter, count_DynParameter);
				Matrix nx(count_DynParameter, 1);
				n_xx = matH.Transpose() * matH;
				nx = matH.Transpose() * matY;
				
				// 添加伪方程-光压参数
				for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
				{
					double weight_solar = 1.0E+12;	
					if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
					{
						if(!m_podDefine.on_SRP9_D0)
						{// △D0[0]  = -(D0*) + ε									
							int index_D0 = 6 + 9 * (int)s_k + 0;
							n_xx.SetElement(index_D0, index_D0,      n_xx.GetElement(index_D0, index_D0)     + weight_solar * weight_solar);
							nx.SetElement(index_D0, 0,               nx.GetElement(index_D0, 0)              - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].D0);
						}
						if(!m_podDefine.on_SRP9_DC1)
						{// △DCOS[1]  = -(DCOS*) + ε								
							int index_DCOS = 6 + 9 * (int)s_k + 1;
							n_xx.SetElement(index_DCOS, index_DCOS,  n_xx.GetElement(index_DCOS, index_DCOS) + weight_solar * weight_solar);
							nx.SetElement(index_DCOS, 0,             nx.GetElement(index_DCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].DC1);
						}
						if(!m_podDefine.on_SRP9_DS1)
						{// △DSIN[2]  = -(DSIN*) + ε								
							int index_DSIN = 6 + 9 * (int)s_k + 2;
							n_xx.SetElement(index_DSIN, index_DSIN,  n_xx.GetElement(index_DSIN, index_DSIN) + weight_solar * weight_solar);
							nx.SetElement(index_DSIN, 0,             nx.GetElement(index_DSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].DS1);
						}
						if(!m_podDefine.on_SRP9_Y0)
						{// △Y0[3]  = -(Y0*) + ε								
							int index_Y0 = 6 + 9 * (int)s_k + 3;
							n_xx.SetElement(index_Y0, index_Y0,      n_xx.GetElement(index_Y0, index_Y0)     + weight_solar * weight_solar);
							nx.SetElement(index_Y0, 0,               nx.GetElement(index_Y0, 0)              - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].Y0);
						}
						if(!m_podDefine.on_SRP9_YC1)
						{// △YCOS[4]  = -(YCOS*) + ε								
							int index_YCOS = 6 + 9 * (int)s_k + 4;
							n_xx.SetElement(index_YCOS, index_YCOS,  n_xx.GetElement(index_YCOS, index_YCOS) + weight_solar * weight_solar);
							nx.SetElement(index_YCOS, 0,             nx.GetElement(index_YCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].YC1);
						}
						if(!m_podDefine.on_SRP9_YS1)
						{// △YSIN[5]  = -(YSIN*) + ε								
							int index_YSIN = 6 + 9 * (int)s_k + 5;
							n_xx.SetElement(index_YSIN, index_YSIN,  n_xx.GetElement(index_YSIN, index_YSIN) + weight_solar * weight_solar);
							nx.SetElement(index_YSIN, 0,             nx.GetElement(index_YSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].YS1);
						}
						if(!m_podDefine.on_SRP9_B0)
						{// △B0[6]  = -(B0*) + ε								
							int index_B0 = 6 + 9 * (int)s_k + 6;
							n_xx.SetElement(index_B0, index_B0,      n_xx.GetElement(index_B0, index_B0)     + weight_solar * weight_solar);
							nx.SetElement(index_B0, 0,               nx.GetElement(index_B0, 0)              - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].B0);
						}
						if(!m_podDefine.on_SRP9_BC1)
						{// △BCOS[7]  = -(BCOS*) + ε								
							int index_BCOS = 6 + 9 * (int)s_k + 7;
							n_xx.SetElement(index_BCOS, index_BCOS,  n_xx.GetElement(index_BCOS, index_BCOS) + weight_solar * weight_solar);
							nx.SetElement(index_BCOS, 0,             nx.GetElement(index_BCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].BC1);
						}
						if(!m_podDefine.on_SRP9_BS1)
						{// △BSIN[8]  = -(BSIN*) + ε								
							int index_BSIN = 6 + 9 * (int)s_k + 8;
							n_xx.SetElement(index_BSIN, index_BSIN,  n_xx.GetElement(index_BSIN, index_BSIN) + weight_solar * weight_solar);
							nx.SetElement(index_BSIN, 0,             nx.GetElement(index_BSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].BS1);
						}
					}
				}

				// 计算轨道改进量
				Matrix matdx = n_xx.Inv_Ssgj() * nx; 
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
				if(dynamicDatum.bOn_AtmosphereDragAcc) 
				{// 大气阻力
					for(int s_k = 0; s_k < int(dynamicDatum.atmosphereDragParaList.size()); s_k++)
						dynamicDatum.atmosphereDragParaList[s_k].Cd +=  matdx.GetElement(beginPara + s_k, 0);
					beginPara += count_AtmosphereDragParaList;
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
					if(dynamicDatum.bOn_AtmosphereDragAcc)
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

				if(max_adjust_pos <= m_podDefine.threshold_max_adjustpos || k >= 6 || flag_done)
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
			TDT t0_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t0_forecast));  
			TDT t1_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t1_forecast));
			if(result)
			{
				vector<TimePosVel> orbitlist_ac;
				vector<Matrix> matRtPartiallist_ac;
				// 倒向积分, 积分区间 [para.T0, T_End   + h * 4], 为保证插值精度向两端进行扩展
				vector<TimePosVel> backwardOrbitlist_ac; 
			    vector<TimePosVel> forwardOrbitlist_ac; 
                double h = m_stepAdamsCowell; // 20200229, 谷德峰
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
					forecastOrbList[s_i].t = m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(t_GPS)); // 转换到UTC
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
				sprintf(info, "dynamicPOD_pos解算成功!(oc = %.4f)", rms_oc);
				RuningInfoFile::Add(info);
				printf("%s\n", info);
				return true;
			}
			else
			{
				sprintf(info, "dynamicPOD_pos解算失败!");
				RuningInfoFile::Add(info);
				printf("%s\n", info);
				return false;
			}
		}

		bool SLRSatDynPOD::mainPOD(string strObsFileName, int nObsFileType, SatdynBasicDatum &dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval, bool bInitDynDatumEst, bool bForecast, bool bResEdit)
		{
			char   info[200];
			double P_J2000[6]; // 惯性坐标, 用于坐标系转换
			double P_ITRF[6];  // 地固坐标
			// 更新观测数据
			if(nObsFileType!=0 && nObsFileType != 1 && nObsFileType != 2)
			{
				printf("激光数据文件类型未知!\n");
				return false;
			}
			size_t count_pass = 0;
			cstgSLRObsFile obsFile_cstg;
			obsFile_cstg.m_bChecksum = m_bChecksum;
			if(nObsFileType == 0)
			{
				obsFile_cstg.open(strObsFileName);
				count_pass = obsFile_cstg.m_data.size();
				if(count_pass <= 0)
				{
					printf("cstg 激光数据为空!\n");
					return false;
				}
			}
			meritSLRObsFile obsFile_merit;
			typedef vector<meritDataRecord> meritSinglePassArc;
			vector<meritSinglePassArc> meritSinglePassArcList;
			if(nObsFileType == 1)
			{
				meritSinglePassArcList.clear();
				obsFile_merit.open(strObsFileName);
				size_t count_obs = obsFile_merit.m_data.size();
				if(count_obs <= 0)
				{
					printf("merit 激光数据为空!\n");
					return false;
				}
				// 根据 merit 数据列表, 检索每个弧段
				// 前提是merit数据是依据测站分类, 且按照时间顺序进行排列的
				GPST t0 = obsFile_merit.m_data[0].getTime(); 
				double *pObsTime  = new double [count_obs]; // 相对时间
				for(size_t s_i = 0; s_i < count_obs; s_i++)
					pObsTime[s_i] =  obsFile_merit.m_data[s_i].getTime() - t0;
				// 获得每个连续跟踪弧段的数据
				size_t k   = 0;
				size_t k_i = k;
                count_pass = 0;
				while(1)
				{
					if(k_i + 1 >= count_obs)
						goto NewArc;
					else
					{// 判断 k_i+1 与 k_i 是否位于同一跟踪弧段
						if( fabs(pObsTime[k_i+1] - pObsTime[k_i]) < 2000.0 
						 && obsFile_merit.m_data[k_i + 1].StationID == obsFile_merit.m_data[k_i].StationID)
						{
							k_i++;
							continue;
						}
						else // k_i+1为新弧段的起点
							goto NewArc;
					}
					NewArc:  // 本弧段[k, k_i]数据处理 
					{
						meritSinglePassArc newPassArc;
						for(size_t s_i = k; s_i <= k_i; s_i++)
						{
							newPassArc.push_back(obsFile_merit.m_data[s_i]);
						}
						if(newPassArc.size() > 0)
							meritSinglePassArcList.push_back(newPassArc);
						if(k_i+1 >= count_obs)
							break;
						else  
						{// 新弧段的起点设置
							k   = k_i+1;
							k_i = k;
							continue;
						}
					}
					delete pObsTime;
				}
				count_pass = meritSinglePassArcList.size();
				if(count_pass <= 0)
				{
					printf("merit 激光数据为空!\n");
					return false;
				}
			}
			crdSLRObsFile obsFile_crd;
			if(nObsFileType == 2)
			{
				if(!obsFile_crd.open(strObsFileName))
				{
					printf("crd 激光数据文件打开失败!\n");
					return false;
				}
				count_pass = obsFile_crd.m_data.size();
				if(count_pass <= 0)
				{
					printf("crd 激光数据为空!\n");
					return false;
				}
			}
			m_obsArc.clear();
			for(size_t s_i = 0; s_i < count_pass; s_i++)
			{
				SLRPOD_ObsArc obsArc_i;
				size_t count = 0;
				int nCDPPadID;
				UTC t0, t1;
				double Wavelength;
				if(nObsFileType == 0)
				{
					count = obsFile_cstg.m_data[s_i].normalDataRecordList.size();
					// 激光波长换算成微米
					Wavelength = obsFile_cstg.m_data[s_i].normalHeaderRecord.Wavelength * 0.001;
					// 确定本弧段的起止时间
					t0 = obsFile_cstg.m_data[s_i].getTime(obsFile_cstg.m_data[s_i].normalDataRecordList[0]);
					t1 = obsFile_cstg.m_data[s_i].getTime(obsFile_cstg.m_data[s_i].normalDataRecordList[count - 1]);
					nCDPPadID = obsFile_cstg.m_data[s_i].normalHeaderRecord.nCDPPadID;
				}
				if(nObsFileType == 1)
				{
					count = meritSinglePassArcList[s_i].size();
					// 激光波长换算成微米
					Wavelength = meritSinglePassArcList[s_i][0].Wavelength * 0.0001; // 换算成微米
					// 确定本弧段的起止时间
					t0 = meritSinglePassArcList[s_i][0].getTime();
					t1 = meritSinglePassArcList[s_i][count - 1].getTime();
					nCDPPadID = meritSinglePassArcList[s_i][0].StationID;
				}
				if(nObsFileType == 2)
				{
					count = obsFile_crd.m_data[s_i].crdDataRecordList.size();
					// 激光波长换算成微米
					Wavelength = obsFile_crd.m_data[s_i].crdConfig.Wavelength * 0.001;
					// 确定本弧段的起止时间
					t0 = obsFile_crd.m_data[s_i].getTime(obsFile_crd.m_data[s_i].crdDataRecordList.front());
					t1 = obsFile_crd.m_data[s_i].getTime(obsFile_crd.m_data[s_i].crdDataRecordList.back());
					nCDPPadID = obsFile_crd.m_data[s_i].crdHeader.nCDPPadID;
				}

				// 获得激光发射时刻的测站位置(由于测站漂移的速度较小,一次跟踪弧端内的时间差异不做区分)
				POS6D staPV;
				if(!getStaPosvel(t0, nCDPPadID, staPV))
					continue;
				// 计算测站的大地经纬度
				BLH blh;
				m_TimeCoordConvert.XYZ2BLH(staPV.getPos(), blh);
				// 获得测站的偏心数据(ilrs)
				ENU ecc;
				if(!getStaEcc(t0, nCDPPadID, ecc))
					continue;
				for(size_t s_j = 0; s_j < count; s_j++)
				{
					double Temperature, Pressure, Humidity;
					SLRPOD_ObsElement obsLine;
					obsLine.staPos_ECEF = staPV.getPos();
					obsLine.staBLH = blh;
					obsLine.ecc = ecc;
					if(nObsFileType == 0)
					{
						Temperature = obsFile_cstg.m_data[s_i].normalDataRecordList[s_j].SurfaceTemperature * 0.1;
						Pressure    = obsFile_cstg.m_data[s_i].normalDataRecordList[s_j].SurfacePressure * 0.1;
						Humidity    = obsFile_cstg.m_data[s_i].normalDataRecordList[s_j].SurfaceRelHumidity;
						// 换算成单程激光距离
						obsLine.obs = obsFile_cstg.m_data[s_i].normalDataRecordList[s_j].LaserRange * 1.0E-12 * SPEED_LIGHT / 2.0;
						// 计算地面激光 fire 时刻 editedLine.Ts
						obsLine.Ts = obsFile_cstg.m_data[s_i].getTime(obsFile_cstg.m_data[s_i].normalDataRecordList[s_j]);
					}
					if(nObsFileType == 1)
					{
						Temperature = meritSinglePassArcList[s_i][s_j].SurfaceTemperature * 0.1;
						Pressure    = meritSinglePassArcList[s_i][s_j].SurfacePressure * 0.1;
						Humidity    = meritSinglePassArcList[s_i][s_j].SurfaceRelHumidity;
						// 换算成单程激光距离
						obsLine.obs = meritSinglePassArcList[s_i][s_j].LaserRange * 1.0E-12 * SPEED_LIGHT / 2.0;
						// 计算地面激光 fire 时刻 editedLine.Ts
						obsLine.Ts = meritSinglePassArcList[s_i][s_j].getTime();
					}
					if(nObsFileType == 2)
					{
						Temperature = obsFile_crd.m_data[s_i].crdDataRecordList[s_j].SurfaceTemperature;
						Pressure    = obsFile_crd.m_data[s_i].crdDataRecordList[s_j].SurfacePressure;
						Humidity    = obsFile_crd.m_data[s_i].crdDataRecordList[s_j].SurfaceRelHumidity;
						// 换算成单程激光距离
						if(obsFile_crd.m_data[s_i].crdHeader.nRangeType == 2)
							obsLine.obs = obsFile_crd.m_data[s_i].crdDataRecordList[s_j].TimeofFlight * SPEED_LIGHT / 2.0;
						else
							continue;//暂不做处理
						// 计算地面激光 fire 时刻 editedLine.Ts
						obsLine.Ts = obsFile_crd.m_data[s_i].getTime(obsFile_crd.m_data[s_i].crdDataRecordList[s_j]); // UTC时刻
					}

					// 根据定轨时间窗口来剪切数据
					if(obsLine.Ts - t0_forecast > 0 && obsLine.Ts - t1_forecast < 0)
					{
						obsLine.wavelength  = Wavelength;
						obsLine.temperature = Temperature;
						obsLine.pressure    = Pressure;
						obsLine.humidity    = Humidity;
						obsLine.id          = nCDPPadID;
						P_ITRF[0] = obsLine.staPos_ECEF.x;
						P_ITRF[1] = obsLine.staPos_ECEF.y;
						P_ITRF[2] = obsLine.staPos_ECEF.z;
						m_TimeCoordConvert.ECEF_J2000(m_TimeCoordConvert.UTC2GPST(obsLine.Ts), P_J2000, P_ITRF, false);
						obsLine.staPos_J2000.x = P_J2000[0];
						obsLine.staPos_J2000.y = P_J2000[1];
						obsLine.staPos_J2000.z = P_J2000[2];
						obsArc_i.obsList.push_back(obsLine);
					}
				}
				if(int(obsArc_i.obsList.size()) > m_podDefine.min_ArcPointCount)
					m_obsArc.push_back(obsArc_i);
			}

			// 迭代参数估计
			const int max_iterator = 3; // 迭代次数阈值
			int num_iterator = 0;       // 记录迭代次数
			int num_after_ocEdit = 0;
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
			int count_SolarPressurePara = dynamicDatum.getSolarPressureParaCount();
			bool result = true;
			bool flag_break = false;
			bool flag_robust = false;
			SatdynBasicDatum dynamicDatum_Init = dynamicDatum; // 记录初始轨道根数
			char dynamicDatumFilePath[300];
			sprintf(dynamicDatumFilePath,"%s\\%s", m_strPODPath.c_str(),  m_podDefine.nameDynPodFitFile.c_str());
			double rms_oc = 0.0;
			double rms_oc_arc = 0.0;
			double countpass_valid = 0;
			while(1)
			{
				num_iterator++;
				if(num_iterator >=  m_podDefine.max_OrbIteration)
				{
					result = false;	// 2014/06/18,发散轨道比较用，鞠 冰
					sprintf(info, "迭代超过%d次, 发散!", num_iterator);
					printf("%s\n");
					RuningInfoFile::Add(info);
					break;
				}
				// 根据初轨进行轨道积分, 用于后续的概略距离修正
				adamsCowell_ac(t0_forecast, t1_forecast, dynamicDatum, m_acOrbitList, m_acRtPartialList, m_stepAdamsCowell);
				// 更新残差
				rms_oc = 0;
				rms_oc_arc = 0.0;
				countpass_valid = 0;
				int count_obs = 0;
				for(size_t s_i = 0; s_i < m_obsArc.size(); s_i++)
				{
					m_obsArc[s_i].rms_oc = 0.0;
					m_obsArc[s_i].count_valid = 0;
					for(size_t s_j = 0; s_j < m_obsArc[s_i].obsList.size(); s_j++)
					{
						// 更新 dR_up 和 dR_down
						// 获得测站在 J2000 惯性系中的位置(Ts 时刻)
						double delay = m_obsArc[s_i].obsList[s_j].obs / SPEED_LIGHT; // 初始化延迟数据
						m_obsArc[s_i].obsList[s_j].Tr = m_obsArc[s_i].obsList[s_j].Ts + delay;
						GPST t_GPS = m_TimeCoordConvert.UTC2GPST(m_obsArc[s_i].obsList[s_j].Tr); 
						TDB t_TDB  = m_TimeCoordConvert.GPST2TDB(t_GPS); // 获得TDB时间--提供太阳历参考时间
						double jd_TDB = m_TimeCoordConvert.DayTime2JD(t_TDB); // 获得儒略日
						TimePosVel  satOrb;
						Matrix satRtPartial;
						if(!getEphemeris(m_obsArc[s_i].obsList[s_j].Tr, satOrb, satRtPartial))
							continue;
						m_obsArc[s_i].obsList[s_j].satOrb        = satOrb;
						m_obsArc[s_i].obsList[s_j].satRtPartial  = satRtPartial;
						m_obsArc[s_i].obsList[s_j].satOrb_ECEF.t = satOrb.t; // 转换为地固系
						P_J2000[0] = satOrb.pos.x;
						P_J2000[1] = satOrb.pos.y;
						P_J2000[2] = satOrb.pos.z;
						P_J2000[3] = satOrb.vel.x;
						P_J2000[4] = satOrb.vel.y;
						P_J2000[5] = satOrb.vel.z;
						m_TimeCoordConvert.J2000_ECEF(m_TimeCoordConvert.UTC2GPST(m_obsArc[s_i].obsList[s_j].Tr), P_J2000, P_ITRF, true); 
						m_obsArc[s_i].obsList[s_j].satOrb_ECEF.pos.x = P_ITRF[0];
						m_obsArc[s_i].obsList[s_j].satOrb_ECEF.pos.y = P_ITRF[1];
						m_obsArc[s_i].obsList[s_j].satOrb_ECEF.pos.z = P_ITRF[2];
						m_obsArc[s_i].obsList[s_j].satOrb_ECEF.vel.x = P_ITRF[3];
						m_obsArc[s_i].obsList[s_j].satOrb_ECEF.vel.y = P_ITRF[4];
						m_obsArc[s_i].obsList[s_j].satOrb_ECEF.vel.z = P_ITRF[5];

						// 计算卫星的仰角
						POS3D p_station = vectorNormal(m_obsArc[s_i].obsList[s_j].staPos_ECEF);
						POS3D p_sat = vectorNormal(m_obsArc[s_i].obsList[s_j].satOrb_ECEF.pos - m_obsArc[s_i].obsList[s_j].staPos_ECEF);
						//double E = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;
						p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); // 20150608, 考虑到地球扁率的影响, 卫星仰角的计算进行了修正, 谷德峰
						p_station = vectorNormal(p_station);
						double E = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;
						m_obsArc[s_i].obsList[s_j].Elevation = E;

						double weight = 1.0;
						weighting_Elevation(m_obsArc[s_i].obsList[s_j].Elevation, weight); // 根据仰角更新权值
						m_obsArc[s_i].obsList[s_j].weight = weight;
						m_obsArc[s_i].obsList[s_j].weight = weight / sqrt(m_obsArc[s_i].obsList.size() * 1.0); // 每个弧段根据观测数量定权值，避免部分测站因数据少作用被减弱

						// 获得太阳位置 
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
						// 获得极移数据()
						double xp = 0;
						double yp = 0;
						if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_2003)
							m_TimeCoordConvert.m_eopRapidFileIAU2000.getPoleOffset(m_obsArc[s_i].obsList[s_j].Tr, xp, yp);
						if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_1996)
							m_TimeCoordConvert.m_eopc04File.getPoleOffset(m_obsArc[s_i].obsList[s_j].Tr, xp, yp);
						// 计算各项修正
						/* 第一步：进行对流层改正 */
						m_obsArc[s_i].obsList[s_j].dR_correct_Trop = 0.0;
						if(m_podDefine.on_TropDelay)
						{
							double fai = m_obsArc[s_i].obsList[s_j].staBLH.B; 
							double h   = m_obsArc[s_i].obsList[s_j].staBLH.H;
							m_obsArc[s_i].obsList[s_j].dR_correct_Trop = SLRPreproc::tropCorrect_Marini_IERS2010(m_obsArc[s_i].obsList[s_j].temperature, 
							                                                                                     m_obsArc[s_i].obsList[s_j].pressure, 
																												 m_obsArc[s_i].obsList[s_j].humidity, 
																												 m_obsArc[s_i].obsList[s_j].wavelength, 
																												 E, 
																												 fai, 
																												 h);
						}
						/* 第二步：相对论改正     */
						m_obsArc[s_i].obsList[s_j].dR_correct_Relativity = 0.0;
						if(m_podDefine.on_TropDelay)
						{
							m_obsArc[s_i].obsList[s_j].dR_correct_Relativity = SLRPreproc::relativityCorrect(sunPos_ITRF, 
								                                                                             m_obsArc[s_i].obsList[s_j].satOrb_ECEF.pos, 
																											 m_obsArc[s_i].obsList[s_j].staPos_ECEF);
						}
						/* 第三步：测站偏心改正   */
						m_obsArc[s_i].obsList[s_j].dR_correct_StaEcc = 0.0;
						if(m_podDefine.on_StaEcc)
						{
							m_obsArc[s_i].obsList[s_j].dR_correct_StaEcc = SLRPreproc::staEccCorrect(m_obsArc[s_i].obsList[s_j].staPos_ECEF, 
								                                                                     m_obsArc[s_i].obsList[s_j].satOrb_ECEF.pos, 
																									 m_obsArc[s_i].obsList[s_j].ecc);
						}
						/* 第四步：卫星质心改正   */
						m_obsArc[s_i].obsList[s_j].dR_correct_SatMco = 0.0;
						if(m_podDefine.on_SatPCO)
						{
							Matrix matATT; // 利用姿态数据进行部位修正
							if(m_attFile.getAttMatrix(t_GPS, matATT))
							{
								matATT = matATT.Transpose(); 
								Matrix matPCO(3, 1);
								matPCO.SetElement(0, 0, m_mcoLaserRetroReflector.x);
								matPCO.SetElement(1, 0, m_mcoLaserRetroReflector.y);
								matPCO.SetElement(2, 0, m_mcoLaserRetroReflector.z);
								matPCO = matATT * matPCO;
								POS3D vecLos = vectorNormal(m_obsArc[s_i].obsList[s_j].satOrb.pos - m_obsArc[s_i].obsList[s_j].staPos_J2000);
								m_obsArc[s_i].obsList[s_j].dR_correct_SatMco = matPCO.GetElement(0, 0) * vecLos.x
															                 + matPCO.GetElement(1, 0) * vecLos.y
															                 + matPCO.GetElement(2, 0) * vecLos.z;
							}
							else
							{
								if(m_podDefine.on_YawAttitudeModel)
								{	
									m_obsArc[s_i].obsList[s_j].dR_correct_SatMco = SLRPreproc::satMassCenterCorrect_YawAttitudeModel(m_obsArc[s_i].obsList[s_j].staPos_J2000, 
										                                                                                             m_obsArc[s_i].obsList[s_j].satOrb.pos, 
																																	 sunPos_J2000, 
																																	 m_mcoLaserRetroReflector); // 20151125, 调整到惯性系下修正, 谷德峰							
								}
								else // 可用于BDS - GEO卫星
								{
									m_obsArc[s_i].obsList[s_j].dR_correct_SatMco = SLRPreproc::satMassCenterCorrect_J2000(m_obsArc[s_i].obsList[s_j].staPos_J2000, 
										                                                                                  m_obsArc[s_i].obsList[s_j].satOrb.getPosVel(), 
																														  m_mcoLaserRetroReflector); // 20151125, 调整到惯性系下修正, 谷德峰
								}
							}
							m_obsArc[s_i].obsList[s_j].dR_correct_SatMco += m_constRangeBias; // 20170424， 谷德峰针对HY2A的固定偏差修正添加
						}
						/* 第五步：潮汐改正       */
						m_obsArc[s_i].obsList[s_j].dR_correct_Tide = 0.0;
						if(m_podDefine.on_Tides)
						{
							StaOceanTide sotDatum;
							m_staOldFile.getStaOceanTide(m_obsArc[s_i].obsList[s_j].id, sotDatum);
							m_obsArc[s_i].obsList[s_j].dR_correct_Tide = SLRPreproc::tideCorrect(t_GPS, 
								                                                                 sunPos_ITRF, 
																								 moonPos_ITRF, 
																								 m_obsArc[s_i].obsList[s_j].staPos_ECEF, 
																								 m_obsArc[s_i].obsList[s_j].satOrb_ECEF.pos, 
																								 sotDatum, 
																								 xp, 
																								 yp);
						}
						// 总的延迟量
						m_obsArc[s_i].obsList[s_j].obscorrected_value =  m_obsArc[s_i].obsList[s_j].dR_correct_Trop
													                   + m_obsArc[s_i].obsList[s_j].dR_correct_Relativity
													                   + m_obsArc[s_i].obsList[s_j].dR_correct_StaEcc
													                   + m_obsArc[s_i].obsList[s_j].dR_correct_SatMco
													                   + m_obsArc[s_i].obsList[s_j].dR_correct_Tide;
						double dDelay_k_1 = 0;
						double dR_up = m_obsArc[s_i].obsList[s_j].obs;
						while(fabs(delay - dDelay_k_1) > 1.0E-8)
						{
							// 更新延迟时间
							dDelay_k_1 = delay;
							// 根据 dDelay 计算上行激光 reflect 时间
							m_obsArc[s_i].obsList[s_j].Tr = m_obsArc[s_i].obsList[s_j].Ts + delay;
							// 获得 J2000 惯性系下的卫星轨道 
							getEphemeris(m_obsArc[s_i].obsList[s_j].Tr, satOrb, satRtPartial);
							m_obsArc[s_i].obsList[s_j].satOrb = satOrb;
							m_obsArc[s_i].obsList[s_j].satRtPartial = satRtPartial;
							// 修正地球运动对卫星状态矢量的影响
							// 获得地球运动的速度
							double PV[6];
							POS3D  EarthVel;
							m_JPLEphFile.getEarthPosVel(jd_TDB, PV);
							EarthVel.x = PV[3] * 1000;
							EarthVel.y = PV[4] * 1000;
							EarthVel.z = PV[5] * 1000;
							POS3D leoPos_J2000_xz = satOrb.pos;// + EarthVel * delay;
							// 计算上行几何距离
							dR_up = sqrt(pow(m_obsArc[s_i].obsList[s_j].staPos_J2000.x - leoPos_J2000_xz.x, 2) +
										 pow(m_obsArc[s_i].obsList[s_j].staPos_J2000.y - leoPos_J2000_xz.y, 2) +
										 pow(m_obsArc[s_i].obsList[s_j].staPos_J2000.z - leoPos_J2000_xz.z, 2));
							delay = (dR_up + m_obsArc[s_i].obsList[s_j].obscorrected_value) / SPEED_LIGHT;
						}
						// 激光反射时刻 editedLine.Tr, 卫星的轨道位置 leoPos_J2000
						// 迭代计算下行激光延迟时间
						dDelay_k_1 = 0;
						double dR_down = m_obsArc[s_i].obsList[s_j].obs;
						while(fabs(delay - dDelay_k_1) > 1.0E-8)
						{// 更新延迟时间
							dDelay_k_1 = delay;
							// 根据 dDelay 计算地面激光接收时间
							GPST TR_GPS = m_TimeCoordConvert.UTC2GPST(m_obsArc[s_i].obsList[s_j].Tr + delay);
							// 获得 J2000 惯性系下的观测站位置
							POS3D staPos_J2000_TR;
							P_ITRF[0] = m_obsArc[s_i].obsList[s_j].staPos_ECEF.x;
							P_ITRF[1] = m_obsArc[s_i].obsList[s_j].staPos_ECEF.y;
							P_ITRF[2] = m_obsArc[s_i].obsList[s_j].staPos_ECEF.z;
							m_TimeCoordConvert.ECEF_J2000(TR_GPS, P_J2000, P_ITRF, false);
							staPos_J2000_TR.x = P_J2000[0];
							staPos_J2000_TR.y = P_J2000[1];
							staPos_J2000_TR.z = P_J2000[2];
							// 修正地球运动对卫星状态矢量的影响
							// 获得地球运动的速度
							double PV[6];
							POS3D  EarthVel;
							m_JPLEphFile.getEarthPosVel(jd_TDB, PV);
							EarthVel.x = PV[3] * 1000;
							EarthVel.y = PV[4] * 1000;
							EarthVel.z = PV[5] * 1000;
							POS3D staPos_J2000_xz = staPos_J2000_TR;// + EarthVel * delay;
							// 计算下行几何距离
							dR_down = sqrt(pow(staPos_J2000_xz.x - m_obsArc[s_i].obsList[s_j].satOrb.pos.x, 2) +
										   pow(staPos_J2000_xz.y - m_obsArc[s_i].obsList[s_j].satOrb.pos.y, 2) +
										   pow(staPos_J2000_xz.z - m_obsArc[s_i].obsList[s_j].satOrb.pos.z, 2));
							delay = (dR_down + m_obsArc[s_i].obsList[s_j].obscorrected_value) / SPEED_LIGHT;
						}
						m_obsArc[s_i].obsList[s_j].r_mean = 0.5 * (dR_down + dR_up);
						m_obsArc[s_i].obsList[s_j].oc = m_obsArc[s_i].obsList[s_j].obs
													  - m_obsArc[s_i].obsList[s_j].r_mean
													  - m_obsArc[s_i].obsList[s_j].obscorrected_value;

						// 剔除残差较大的点
						if(fabs(m_obsArc[s_i].obsList[s_j].oc) > m_podDefine.max_ocEdit)
						{
							m_obsArc[s_i].obsList[s_j].rw = 0.0;
							/*sprintf(info, "oc = %10.2lf, StaEcc = %10.2lf, Tide = %10.2lf.", m_obsArc[s_i].obsList[s_j].oc,
								                                                             m_obsArc[s_i].obsList[s_j].dR_correct_StaEcc,
																						  	 m_obsArc[s_i].obsList[s_j].dR_correct_Tide);
							RuningInfoFile::Add(info);*/
						}

						if (m_obsArc[s_i].obsList[s_j].rw == 1.0)
						{
							m_obsArc[s_i].rms_oc += pow(m_obsArc[s_i].obsList[s_j].oc, 2);
							m_obsArc[s_i].count_valid++;
							rms_oc += pow(m_obsArc[s_i].obsList[s_j].oc * m_obsArc[s_i].obsList[s_j].weight, 2);
							count_obs++;
						}
					}
					if(m_obsArc[s_i].count_valid > 0)
					{
						m_obsArc[s_i].rms_oc = sqrt(m_obsArc[s_i].rms_oc / m_obsArc[s_i].count_valid);
						rms_oc_arc += m_obsArc[s_i].rms_oc;
						countpass_valid++;
					}
				}
				rms_oc = sqrt(rms_oc / count_obs);
				if(countpass_valid > 0)
					rms_oc_arc = rms_oc_arc / countpass_valid; 
				//sprintf(info, "第%d次 adamsCowell_Interp is ok!(rms_oc = %8.4f)(rms_oc_arc = %8.4f)", num_iterator, rms_oc, rms_oc_arc);
				sprintf(info, "第%d次 adamsCowell_Interp is ok (rms_oc_arc = %8.4f)!", num_iterator, rms_oc_arc); // rms_oc_arc 每个弧段检核结果的平均值
				RuningInfoFile::Add(info);
				printf("%s\n", info);

				if(countpass_valid == 0)
				{
					sprintf(info, "警告：无有效的SLR跟踪数据可用!" ); 
					RuningInfoFile::Add(info);
					printf("%s\n", info);
					return false;
				}
			
				if(flag_break)
				{
					break;
				}

				// 残差编辑
				if(flag_robust && bResEdit)
				{
					for(size_t s_i = 0; s_i < m_obsArc.size(); s_i++)
					{
						for(size_t s_j = 0; s_j < m_obsArc[s_i].obsList.size(); s_j++)
						{
							if (fabs(m_obsArc[s_i].obsList[s_j].oc) > rms_oc_arc * m_podDefine.ratio_ocEdit) // 采用 rms_oc_arc 进行编辑
							{
								m_obsArc[s_i].obsList[s_j].rw = rms_oc_arc / fabs(m_obsArc[s_i].obsList[s_j].oc);
							}
						}
					}
					num_after_ocEdit++;		
				}

				// 设计矩阵
				Matrix n_xx(count_DynParameter, count_DynParameter);
				Matrix nx(count_DynParameter, 1);
				for(size_t s_i = 0; s_i < m_obsArc.size(); s_i++)
				{
					for(size_t s_j = 0; s_j < m_obsArc[s_i].obsList.size(); s_j++)
					{
						POS3D vecLos = vectorNormal(m_obsArc[s_i].obsList[s_j].satOrb.pos - m_obsArc[s_i].obsList[s_j].staPos_J2000);
						double w = m_obsArc[s_i].obsList[s_j].rw * m_obsArc[s_i].obsList[s_j].weight;
						Matrix matH_pos_j(1, 3);
						matH_pos_j.SetElement(0, 0, w * vecLos.x);
						matH_pos_j.SetElement(0, 1, w * vecLos.y);
						matH_pos_j.SetElement(0, 2, w * vecLos.z);
						Matrix matHx_j(1, count_DynParameter);
						for(int s_k = 0; s_k < 6; s_k++)
						{// 初始位置速度
							double sum_posvel = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, s_k) * matH_pos_j.GetElement(0, 0) 
											  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, s_k) * matH_pos_j.GetElement(0, 1)
											  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, s_k) * matH_pos_j.GetElement(0, 2);
							matHx_j.SetElement(0, s_k, sum_posvel);
						}
						int beginPara = 6;
						// 兼容多光压模型计算, 谷德峰
						if(dynamicDatum.bOn_SolarPressureAcc)
						{
							for(int j = 0; j < int(dynamicDatum.solarPressureParaList.size() * count_SolarPressurePara); j++)
							{
								double sum_solar = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j) * matH_pos_j.GetElement(0, 0) 
												 + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j) * matH_pos_j.GetElement(0, 1)
												 + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j) * matH_pos_j.GetElement(0, 2);
								matHx_j.SetElement(0, beginPara + j, sum_solar);
							}
							beginPara += count_SolarPressurePara * count_SolarPressureParaList;
						}
						if(dynamicDatum.bOn_AtmosphereDragAcc) 
						{// 大气阻力
							for(int j = 0; j < int(dynamicDatum.atmosphereDragParaList.size()); j++)
							{
								double sum_cd = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j) * matH_pos_j.GetElement(0, 0) 
											  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j) * matH_pos_j.GetElement(0, 1)
											  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j) * matH_pos_j.GetElement(0, 2);
								matHx_j.SetElement(0, beginPara + j, sum_cd);
							}
							beginPara += count_AtmosphereDragParaList;
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
									double sum_cr = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_cr);

									double sum_sr = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_sr);
								}
								i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2;
								if(dynamicDatum.bOn_EmpiricalForceAcc_T)
								{
									double sum_ct = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_ct);

									double sum_st = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_st);
								}
								i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2;
								if(dynamicDatum.bOn_EmpiricalForceAcc_N)
								{
									double sum_cn = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_cn);

									double sum_sn = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_sn);
								}
							}
							beginPara += count_sub * count_EmpiricalForceParaList;
						}
						if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
						{// 经验力
							int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
											 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 3  
											 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 3; 
							for(int j = 0; j < int(dynamicDatum.empiricalForceParaList.size()); j++)
							{
								int i_sub = 0;
								if(dynamicDatum.bOn_EmpiricalForceAcc_R)
								{
									double sum_a0 = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
								i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3;
								if(dynamicDatum.bOn_EmpiricalForceAcc_T)
								{
									double sum_a0 = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
								i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 3 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 3;
								if(dynamicDatum.bOn_EmpiricalForceAcc_N)
								{
									double sum_a0 = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr = m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 0) 
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 1)
												  + m_obsArc[s_i].obsList[s_j].satRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
							}
							beginPara += count_sub * count_EmpiricalForceParaList;
						}
						n_xx = n_xx + matHx_j.Transpose() * matHx_j;
                        nx = nx + matHx_j.Transpose() * (w * m_obsArc[s_i].obsList[s_j].oc);
					}
				}

				// 添加伪方程-光压参数
				for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
				{
					double weight_solar = 1.0E+12;	
					if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
					{
						if(!m_podDefine.on_SRP9_D0)
						{// △D0[0]  = -(D0*) + ε									
							int index_D0 = 6 + 9 * (int)s_k + 0;
							n_xx.SetElement(index_D0, index_D0,      n_xx.GetElement(index_D0, index_D0)     + weight_solar * weight_solar);
							nx.SetElement(index_D0, 0,               nx.GetElement(index_D0, 0)              - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].D0);
						}
						if(!m_podDefine.on_SRP9_DC1)
						{// △DCOS[1]  = -(DCOS*) + ε								
							int index_DCOS = 6 + 9 * (int)s_k + 1;
							n_xx.SetElement(index_DCOS, index_DCOS,  n_xx.GetElement(index_DCOS, index_DCOS) + weight_solar * weight_solar);
							nx.SetElement(index_DCOS, 0,             nx.GetElement(index_DCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].DC1);
						}
						if(!m_podDefine.on_SRP9_DS1)
						{// △DSIN[2]  = -(DSIN*) + ε								
							int index_DSIN = 6 + 9 * (int)s_k + 2;
							n_xx.SetElement(index_DSIN, index_DSIN,  n_xx.GetElement(index_DSIN, index_DSIN) + weight_solar * weight_solar);
							nx.SetElement(index_DSIN, 0,             nx.GetElement(index_DSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].DS1);
						}
						if(!m_podDefine.on_SRP9_Y0)
						{// △Y0[3]  = -(Y0*) + ε								
							int index_Y0 = 6 + 9 * (int)s_k + 3;
							n_xx.SetElement(index_Y0, index_Y0,      n_xx.GetElement(index_Y0, index_Y0)     + weight_solar * weight_solar);
							nx.SetElement(index_Y0, 0,               nx.GetElement(index_Y0, 0)              - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].Y0);
						}
						if(!m_podDefine.on_SRP9_YC1)
						{// △YCOS[4]  = -(YCOS*) + ε								
							int index_YCOS = 6 + 9 * (int)s_k + 4;
							n_xx.SetElement(index_YCOS, index_YCOS,  n_xx.GetElement(index_YCOS, index_YCOS) + weight_solar * weight_solar);
							nx.SetElement(index_YCOS, 0,             nx.GetElement(index_YCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].YC1);
						}
						if(!m_podDefine.on_SRP9_YS1)
						{// △YSIN[5]  = -(YSIN*) + ε								
							int index_YSIN = 6 + 9 * (int)s_k + 5;
							n_xx.SetElement(index_YSIN, index_YSIN,  n_xx.GetElement(index_YSIN, index_YSIN) + weight_solar * weight_solar);
							nx.SetElement(index_YSIN, 0,             nx.GetElement(index_YSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].YS1);
						}
						if(!m_podDefine.on_SRP9_B0)
						{// △B0[6]  = -(B0*) + ε								
							int index_B0 = 6 + 9 * (int)s_k + 6;
							n_xx.SetElement(index_B0, index_B0,      n_xx.GetElement(index_B0, index_B0)     + weight_solar * weight_solar);
							nx.SetElement(index_B0, 0,               nx.GetElement(index_B0, 0)              - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].B0);
						}
						if(!m_podDefine.on_SRP9_BC1)
						{// △BCOS[7]  = -(BCOS*) + ε								
							int index_BCOS = 6 + 9 * (int)s_k + 7;
							n_xx.SetElement(index_BCOS, index_BCOS,  n_xx.GetElement(index_BCOS, index_BCOS) + weight_solar * weight_solar);
							nx.SetElement(index_BCOS, 0,             nx.GetElement(index_BCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].BC1);
						}
						if(!m_podDefine.on_SRP9_BS1)
						{// △BSIN[8]  = -(BSIN*) + ε								
							int index_BSIN = 6 + 9 * (int)s_k + 8;
							n_xx.SetElement(index_BSIN, index_BSIN,  n_xx.GetElement(index_BSIN, index_BSIN) + weight_solar * weight_solar);
							nx.SetElement(index_BSIN, 0,             nx.GetElement(index_BSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].BS1);
						}
					}
				}
				
				// 轨道改进
				Matrix dx = n_xx.Inv_Ssgj() * nx;
				dynamicDatum.X0.x  += dx.GetElement(0,0);
				dynamicDatum.X0.y  += dx.GetElement(1,0);
				dynamicDatum.X0.z  += dx.GetElement(2,0);
				dynamicDatum.X0.vx += dx.GetElement(3,0);
				dynamicDatum.X0.vy += dx.GetElement(4,0);
				dynamicDatum.X0.vz += dx.GetElement(5,0);
				int beginPara = 6;
				if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
				{// 太阳光压
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
						dynamicDatum.solarPressureParaList[s_k].Cr +=  dx.GetElement(beginPara + s_k, 0);
					beginPara += count_SolarPressureParaList;
				}
				// 2015/10/18, 兼容多光压模型计算, 谷德峰
				else if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
				{
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
					{
						dynamicDatum.solarPressureParaList[s_k].D0  += dx.GetElement(beginPara + s_k * 9 + 0, 0);
						dynamicDatum.solarPressureParaList[s_k].DC1 += dx.GetElement(beginPara + s_k * 9 + 1, 0);
						dynamicDatum.solarPressureParaList[s_k].DS1 += dx.GetElement(beginPara + s_k * 9 + 2, 0);
						dynamicDatum.solarPressureParaList[s_k].Y0  += dx.GetElement(beginPara + s_k * 9 + 3, 0);
						dynamicDatum.solarPressureParaList[s_k].YC1 += dx.GetElement(beginPara + s_k * 9 + 4, 0);
						dynamicDatum.solarPressureParaList[s_k].YS1 += dx.GetElement(beginPara + s_k * 9 + 5, 0);
						dynamicDatum.solarPressureParaList[s_k].B0  += dx.GetElement(beginPara + s_k * 9 + 6, 0);
						dynamicDatum.solarPressureParaList[s_k].BC1 += dx.GetElement(beginPara + s_k * 9 + 7, 0);
						dynamicDatum.solarPressureParaList[s_k].BS1 += dx.GetElement(beginPara + s_k * 9 + 8, 0);
					}
					beginPara += 9 * count_SolarPressureParaList;
				}
				else if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_5PARA)	
				{
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
					{
						dynamicDatum.solarPressureParaList[s_k].D0  += dx.GetElement(beginPara + 5 * s_k,     0);
						dynamicDatum.solarPressureParaList[s_k].Y0  += dx.GetElement(beginPara + 5 * s_k + 1, 0);
						dynamicDatum.solarPressureParaList[s_k].B0  += dx.GetElement(beginPara + 5 * s_k + 2, 0);
						dynamicDatum.solarPressureParaList[s_k].BC1 += dx.GetElement(beginPara + 5 * s_k + 3, 0);
						dynamicDatum.solarPressureParaList[s_k].BS1 += dx.GetElement(beginPara + 5 * s_k + 4, 0);
					}
					beginPara += 5 * count_SolarPressureParaList;
				}
				if(dynamicDatum.bOn_AtmosphereDragAcc) 
				{// 大气阻力
					for(int s_k = 0; s_k < int(dynamicDatum.atmosphereDragParaList.size()); s_k++)
						dynamicDatum.atmosphereDragParaList[s_k].Cd +=  dx.GetElement(beginPara + s_k, 0);
					beginPara += count_AtmosphereDragParaList;
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
							dynamicDatum.empiricalForceParaList[s_k].cos_R += dx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_R += dx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2;
						if(dynamicDatum.bOn_EmpiricalForceAcc_T)
						{
							dynamicDatum.empiricalForceParaList[s_k].cos_T += dx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_T += dx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2;
						if(dynamicDatum.bOn_EmpiricalForceAcc_N)
						{
							dynamicDatum.empiricalForceParaList[s_k].cos_N += dx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_N += dx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
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
							dynamicDatum.empiricalForceParaList[s_k].a0_R  += dx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].cos_R += dx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_R += dx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 1;
						if(dynamicDatum.bOn_EmpiricalForceAcc_T)
						{
							dynamicDatum.empiricalForceParaList[s_k].a0_T  += dx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].cos_T += dx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_T += dx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
						i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 1 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 1;
						if(dynamicDatum.bOn_EmpiricalForceAcc_N)
						{
							dynamicDatum.empiricalForceParaList[s_k].a0_N  += dx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							dynamicDatum.empiricalForceParaList[s_k].cos_N += dx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							dynamicDatum.empiricalForceParaList[s_k].sin_N += dx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
					}
					beginPara += count_sub * count_EmpiricalForceParaList;
				}

				if(!m_strPODPath.empty())
				{
					// 记录轨道改进结果
					FILE * pFitFile = fopen(dynamicDatumFilePath, "w+");
					fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
					int k_Parameter = 0;
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
					if(dynamicDatum.bOn_AtmosphereDragAcc)
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
					max_adjust_pos = max(max_adjust_pos, fabs(dx.GetElement(i, 0)));

				//sprintf(info, "max_adjust_pos = %10.4lf.", max_adjust_pos);
				//RuningInfoFile::Add(info);

				if(max_adjust_pos <= m_podDefine.threshold_max_adjustpos || num_iterator >= max_iterator || num_after_ocEdit > 0)
				{
					// 首次进行残差编辑
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
			TDT t0_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t0_forecast));  
			TDT t1_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t1_forecast));
			if(result)
			{
				vector<TimePosVel> orbitlist_ac;
				vector<Matrix> matRtPartiallist_ac;
				// 倒向积分, 积分区间 [para.T0, T_End   + h * 4], 为保证插值精度向两端进行扩展
				vector<TimePosVel> backwardOrbitlist_ac; 
			    vector<TimePosVel> forwardOrbitlist_ac; 
                double h = m_stepAdamsCowell; // 20200229, 谷德峰
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
					forecastOrbList[s_i].t = m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(t_GPS)); // 转换到UTC
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
