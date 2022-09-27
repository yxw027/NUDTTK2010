#include "TQObsSimu.hpp"
namespace NUDTTK
{
	namespace TQPod
	{

		// 子程序名称： TQSatInfo::getEphemeris   
		// 功能：滑动lagrange插值获得任意时刻TQ卫星星历
		// 变量类型： t         :  UTC北京时
		//            satOrb    :  星历数值, 坐标单位: 米
		//            nLagrange :  Lagrange 插值已知点个数, 默认为 9, 对应 8 阶 Lagrange 插值
		// 输入：t,  nLagrange
		// 输出：satOrb
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2018/9/28
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool TQSatInfo::getEphemeris(UTC t, TimePosVel& satOrb, int nLagrange)
		{
			size_t count_ac = orbList.size();
			const int nlagrange = 8; 
			if(count_ac < nlagrange) // 如果数据点个数小于nlagrange返回, 要求弧段长度 > h * nlagrange = 4分钟
				return false;
			double h = orbList[1].t - orbList[0].t;
			double spanSecond_t = t - orbList[0].t;      // 相对观测时间, 初始时间为 orbitlist_ac[0].t
			int nLeftPos  = int(spanSecond_t / h);       // 首先寻找最靠近时间 T 的左端点，从 0 开始计数
			int nLeftNum  = int(floor(nlagrange / 2.0)); // 理论上 nLeftPos 左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
			int nRightNum = int(ceil(nlagrange / 2.0));
			int nBegin, nEnd;                            // 位于区间[0, count_ac - 1]
			
			//在插值星历表时，一般要求想要插值的节点在插值节点的最中间，这时会出现三种情况
			//1.当插值节点处于星历表的前端，不能使得插值节点位于中间
			//2.当插值节点处于星历表的后端，不能使得插值节点位于中间
			//3.当插值节点处于星历表的中间，可以使得插值节点位于中间
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
				x[i - nBegin] = orbList[i].t - orbList[0].t; // 参考相对时间点
			// X
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = orbList[i].pos.x;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.pos.x);
			// Y
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = orbList[i].pos.y;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.pos.y);
			// Z
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = orbList[i].pos.z;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.pos.z);
			// Vx
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = orbList[i].vel.x;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.vel.x);
			// Vy
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = orbList[i].vel.y;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.vel.y);
			// Vz
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = orbList[i].vel.z;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, satOrb.vel.z);
			delete x;
			delete y;
			return true;
		}

		// 子程序名称： TQSatInfo::getEphemeris_PathDelay   
		// 功能：根据接收机的概略位置、信号接收时间和卫星轨道,
		//       计算卫星信号传播延迟时间(即卫星“准确的”信号反射时间)
		// 变量类型： t                  : 信号接收时间
		//            staPos             : 接收机概略位置, 单位：米
		//            delay              : 信号传播延迟时间, 单位：秒
		//            satOrb             : 确定了正确的信号发射时间后, 顺便返回本颗卫星星历
		//            threshold          : 迭代阈值，默认 1.0E-007
		// 输入：t, staPos, orbList
		// 输出：satOrb
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2018/9/28
		// 版本时间：2018/9/28
		// 修改记录：
		// 备注： 
		bool TQSatInfo::getEphemeris_PathDelay(UTC t, POS3D staPos, double& delay, TimePosVel& satOrb, double threshold)
		{
			// 信号真实接收时间 = 观测时间(T) - 接收机钟差(receiverPos.dClock)
			UTC t_Receive  = t;
			//printf("%d",t_Receive.year);
			UTC t_Transmit = t_Receive; // 初始化信号转发时间
			if(!getEphemeris(t_Transmit, satOrb))
				return false;
			double distance = pow(staPos.x - satOrb.pos.x, 2)
							+ pow(staPos.y - satOrb.pos.y, 2)
							+ pow(staPos.z - satOrb.pos.z, 2);
			distance = sqrt(distance); // 获得信号发射传播距离
			double delay_k_1 = 0;
			delay = distance / SPEED_LIGHT;  // 获得信号反射传播延迟
			//const double delay_max  = 1.0;   // 为了防止迭代dDelay溢出，这里设置一个阈值
			//const int    k_max      = 5;     // 迭代次数阈值，一般1次迭代就会收敛
			const double delay_max  = 350.0;   // 为了防止迭代dDelay溢出，这里设置一个阈值
			const int    k_max      = 10;     // 迭代次数阈值，一般1次迭代就会收敛 
			int          k          = 0;
			while(fabs(delay - delay_k_1) > threshold)   // 迭代阈值控制, abs-->fabs, 2007/07/15
			{
				k++;
				if(fabs(delay) > delay_max || k > k_max) // 为防止 delay 溢出, 2007/04/06
				{
					printf("%s delay 迭代发散!\n",t.toString().c_str());
					return false;
				}
				// 更新信号反射时间
				t_Transmit = t_Receive - delay;
				if(!getEphemeris(t_Transmit, satOrb))
					return false;
				// 更新概略距离
				distance =  pow(staPos.x - satOrb.pos.x, 2)
						  + pow(staPos.y - satOrb.pos.y, 2)
						  + pow(staPos.z - satOrb.pos.z, 2);
				distance = sqrt(distance);
				// 更新延迟数据
				delay_k_1 = delay;
				delay = distance / SPEED_LIGHT;
			}
			return true;
		}

		TQObsSimu::TQObsSimu(void)
		{
		}

		TQObsSimu::~TQObsSimu(void)
		{
		}


		bool TQObsSimu::judgeUXBElevation(UTC t_Receive, POS3D posSta, double& elevation1, double& elevation2)
		{
			for(int s_i = 0; s_i < 2; ++s_i)//111，原来是2跑不通？
			{
				double delay = 0.0;
				TimePosVel satOrb;
				 //迭代计算下行信号 reflect 时间 tr_utc, 获得反射时刻卫星轨道位置
				if(!m_satInfoList[s_i].getEphemeris_PathDelay(t_Receive, posSta, delay, satOrb))	
					continue;

				UTC tr_utc = t_Receive - delay;
				 //将卫星坐标转换到地固系
				 //计算高度角和方位角
				double sat_ecf[6];
				double sat_j2000[6];
				sat_j2000[0] = satOrb.pos.x;  
				sat_j2000[1] = satOrb.pos.y;  
				sat_j2000[2] = satOrb.pos.z;
				m_TimeCoordConvert.J2000_ECEF(m_TimeCoordConvert.UTC2GPST(tr_utc), sat_j2000, sat_ecf, false);
				POS3D satPos_ITRF; 
				satPos_ITRF.x = sat_ecf[0];
				satPos_ITRF.y = sat_ecf[1];
				satPos_ITRF.z = sat_ecf[2];
				ENU satENU;// 测站东北坐标系
				TimeCoordConvert::ECF2ENU(m_staInfo.pos, satPos_ITRF, satENU);// 测站在地固坐标系到测站东北坐标系的转换 
				double Elevation = atan(satENU.U / sqrt(satENU.E * satENU.E + satENU.N * satENU.N)) * 180 / PI;//高度角
				double Azimuth = atan2(satENU.E, satENU.N) * 180 / PI;//方位角
				if(Azimuth < 0)
					Azimuth = Azimuth + 360; // 保证方位角在 [0, 360) 之间

				 //20150608, 卫星仰角计算考虑到地球扁率影响, 谷德峰
				POS3D p_station = vectorNormal(m_staInfo.pos);
				POS3D p_sat = vectorNormal(satPos_ITRF - m_staInfo.pos);					
				p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); 
				p_station = vectorNormal(p_station);					
				Elevation = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;

				if(s_i == 0)
					elevation1 = Elevation;
				else
					elevation2 = Elevation;
			}
			return true;
		}

		bool TQObsSimu::simuUXBObsFile(TQUXBObsFile &obsFile, UTC t0, UTC t1, string staname, string satname, double h,  double DopplerTime0)
		{
			// 判断多普勒积分时间是否小于最小积分时间
			if(DopplerTime0 < m_simuDefine.min_DopplerIntergTime)
				return false;

			// 统计观测数据量
			int tq1 = 0; //当前观测卫星的数据量
			int tq2 = 0;
			int tq3 = 0;
			
			bool flag1 = false; // 用以标记当前探测器是否正在跟踪中
			bool flag2 = false;
			bool flag3 = false;

			srand((unsigned)time(NULL));
			UTC t = t0;
			obsFile.m_data.clear();
			while(t - t1 <= 0.0)
			{
				// 1. 接收机钟差改正
				double recClock = 0;
				if(m_simuDefine.on_RecClock)
				{
					// 待补充
				}

				bool bflag = false;
				double dR[3];
				TQUXBObsLine obsLine;
				for(int k = 0; k < 3; k++)
				{
					// 积分开始时刻t0
					UTC t0_doppler = t - DopplerTime0;
					// 积分终止时刻
					UTC t1_doppler = t;

					UTC u_k = t;
					UTC t_Receive = u_k - recClock / SPEED_LIGHT;// 真实接收时刻

					// 将测站转换到J2000系
					double x_ecf[6];
					double x_j2000[6];
					x_ecf[0] = m_staInfo.pos.x;  
					x_ecf[1] = m_staInfo.pos.y;  
					x_ecf[2] = m_staInfo.pos.z;
					x_ecf[3] = 0.0;
					x_ecf[4] = 0.0;
					x_ecf[5] = 0.0;

					POS6D staPV; 
					m_TimeCoordConvert.ECEF_J2000( m_TimeCoordConvert.UTC2GPST(t_Receive), x_j2000, x_ecf, true);//地固坐标系转换到J2000系 
					staPV.x  = x_j2000[0];
					staPV.y  = x_j2000[1];
					staPV.z  = x_j2000[2];
					staPV.vx = x_j2000[3];
					staPV.vy = x_j2000[4];
					staPV.vz = x_j2000[5];

					if(k == 1)
					{
						u_k = t0_doppler;
						t_Receive = u_k - recClock / SPEED_LIGHT;// 真实接收时刻

						m_TimeCoordConvert.ECEF_J2000( m_TimeCoordConvert.UTC2GPST(t_Receive), x_j2000, x_ecf, true);//地固坐标系转换到J2000系 
						staPV.x  = x_j2000[0];
						staPV.y  = x_j2000[1];
						staPV.z  = x_j2000[2];
						staPV.vx = x_j2000[3];
						staPV.vy = x_j2000[4];
						staPV.vz = x_j2000[5];

					}

					if(k == 2)
					{
						u_k = t1_doppler;
						t_Receive = u_k - recClock / SPEED_LIGHT;// 真实接收时刻

						m_TimeCoordConvert.ECEF_J2000( m_TimeCoordConvert.UTC2GPST(t_Receive), x_j2000, x_ecf, true);//地固坐标系转换到J2000系 
						staPV.x  = x_j2000[0];
						staPV.y  = x_j2000[1];
						staPV.z  = x_j2000[2];
						staPV.vx = x_j2000[3];
						staPV.vy = x_j2000[4];
						staPV.vz = x_j2000[5];

					}

					double delay = 0.0;

					TimePosVel satOrb;

					// 迭代计算下行信号 reflect 时间 tr_utc, 获得反射时刻卫星轨道位置
					if(!m_satInfo.getEphemeris_PathDelay(t_Receive, staPV.getPos(), delay, satOrb))
					{	
						bflag = true;
						break;
					}

					UTC tr_utc = t_Receive - delay;
					// 将卫星坐标转换到地固系
					// 计算高度角和方位角
					double sat_ecf[6];
					double sat_j2000[6];
					sat_j2000[0] = satOrb.pos.x;  
					sat_j2000[1] = satOrb.pos.y;  
					sat_j2000[2] = satOrb.pos.z;
					//printf("%d",t_Receive.day);
					m_TimeCoordConvert.J2000_ECEF(m_TimeCoordConvert.UTC2GPST(tr_utc), sat_j2000, sat_ecf, false);
					POS3D satPos_ITRF; 
					satPos_ITRF.x = sat_ecf[0];
					satPos_ITRF.y = sat_ecf[1];
					satPos_ITRF.z = sat_ecf[2];
					ENU satENU;// 测站东北坐标系
					TimeCoordConvert::ECF2ENU(m_staInfo.pos, satPos_ITRF, satENU);// 测站在地固坐标系到测站东北坐标系的转换 
					double Elevation = atan(satENU.U / sqrt(satENU.E * satENU.E + satENU.N * satENU.N)) * 180 / PI;//高度角
					double Azimuth = atan2(satENU.E, satENU.N) * 180 / PI;//方位角
					if(Azimuth < 0)
						Azimuth = Azimuth + 360; // 保证方位角在 [0, 360) 之间

					// 20150608, 卫星仰角计算考虑到地球扁率影响, 谷德峰
					POS3D p_station = vectorNormal(m_staInfo.pos);
					POS3D p_sat = vectorNormal(satPos_ITRF - m_staInfo.pos);					
					p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); 
					p_station = vectorNormal(p_station);					
					Elevation = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;

					double elevation2,elevation3;
					elevation2 = 0.0;
					elevation3 = 0.0;
					//judgeUXBElevation(t_Receive, m_staInfo.pos, elevation2, elevation3);
					/*char info[300];
					sprintf(info, "%s\nm_simuDefine.min_Elevation = %10.5f\nElevation = %10.5f；elevation2 = %10.5f；elevation3 = %10.5f", 
						           t_Receive.toString().c_str(), 
					               m_simuDefine.min_Elevation, 
								   Elevation, elevation2, elevation3);
					RuningInfoFile::Add(info);
					printf("%s\n", info);*/

					// 白天不可见
					if(m_simuDefine.on_Day)
					{
						double Longitude = m_staInfo.pos_blh.L;
						int aa = 0;
						double  bb = 0.0;
						UTC utc_sta;

						if(Longitude > 0.0)
						{
							aa = Longitude / 15;
							bb = fmod(Longitude, 15);
							if(bb > 7.5)
								++aa;
							utc_sta = t + aa * 3600;
						}
						else
						{
							aa = fabs(Longitude) / 15;
							bb = fmod(fabs(Longitude), 15);
							if(bb > 7.5)
								++aa;
							utc_sta = t - aa * 3600;
						}

						// 白天不可见
						if(utc_sta.hour >= 8 && utc_sta.hour < 20)
						{
							// 便于画图
							//obsLine.t  = t_Receive;
							//obsLine.R1 = 0;
							//obsLine.V1 = 0;
							//obsLine.Elevation = -1.0;
							//obsLine.Azimuth   = -1.0;
							//obsLine.R0g       = 0;
							//obsLine.R0s       = 0;
							//obsLine.err_Iono  = 0;
							//obsLine.err_Trop  = 0;
							//obsLine.staName      = staname;
							//obsLine.satName      = satname;
							//obsLine.time_Doppler = DopplerTime0;
							//obsLine.t0 = t - DopplerTime0;
							//obsLine.t1 = t;
							//obsFile.m_data.push_back(obsLine);
							bflag = true;
							break;
						}
					}

					if(elevation2 < m_simuDefine.min_Elevation)
						flag2 = false;
					if(elevation3 < m_simuDefine.min_Elevation)
						flag3 = false;

					// 同一测站无法同时观测多个探测器，仿真观测数据
					if(Elevation < m_simuDefine.min_Elevation)
					{
						flag1 = false;
						if(elevation2 >= m_simuDefine.min_Elevation&&flag2 == true)
						{
							++tq2;
						}
						if(elevation3 >= m_simuDefine.min_Elevation&&flag3 == true)
						{
							++tq3;
						}
						// 便于画图
						//obsLine.t  = t_Receive;
						//obsLine.R1 = 0;
						//obsLine.V1 = 0;
						//obsLine.Elevation = -1.0;
						//obsLine.Azimuth   = -1.0;
						//obsLine.R0g       = 0;
						//obsLine.R0s       = 0;
						//obsLine.err_Iono  = 0;
						//obsLine.err_Trop  = 0;

						//obsLine.staName      = staname;
						//obsLine.satName      = satname;
						//obsLine.time_Doppler = DopplerTime0;

						//obsLine.t0 = t - DopplerTime0;
						//obsLine.t1 = t;
						//obsFile.m_data.push_back(obsLine);
						bflag = true;
					    break;
					}
					else if(Elevation >= m_simuDefine.min_Elevation&&elevation2 >= m_simuDefine.min_Elevation&&flag1 == false && flag2 == false&&tq1>=tq2)
					{
						++tq2;
						flag2 = true;
						// 便于画图
						//obsLine.t  = t_Receive;
						//obsLine.R1 = 0;
						//obsLine.V1 = 0;
						//obsLine.Elevation = -1.0;
						//obsLine.Azimuth   = -1.0;
						//obsLine.R0g       = 0;
						//obsLine.R0s       = 0;
						//obsLine.err_Iono  = 0;
						//obsLine.err_Trop  = 0;

						//obsLine.staName      = staname;
						//obsLine.satName      = satname;
						//obsLine.time_Doppler = DopplerTime0;

						//obsLine.t0 = t - DopplerTime0;
						//obsLine.t1 = t;
						//obsFile.m_data.push_back(obsLine);
						bflag = true;
					    break;
					}
					else if(Elevation >= m_simuDefine.min_Elevation&&elevation3 >= m_simuDefine.min_Elevation&&flag1 == false && flag3 == false&&tq1>=tq3)
					{
						++tq3;
						flag3 = true;
						// 便于画图
						//obsLine.t  = t_Receive;
						//obsLine.R1 = 0;
						//obsLine.V1 = 0;
						//obsLine.Elevation = -1.0;
						//obsLine.Azimuth   = -1.0;
						//obsLine.R0g       = 0;
						//obsLine.R0s       = 0;
						//obsLine.err_Iono  = 0;
						//obsLine.err_Trop  = 0;

						//obsLine.staName      = staname;
						//obsLine.satName      = satname;
						//obsLine.time_Doppler = DopplerTime0;

						//obsLine.t0 = t - DopplerTime0;
						//obsLine.t1 = t;
						//obsFile.m_data.push_back(obsLine);
						bflag = true;
					    break;
					}
					else if(Elevation >= m_simuDefine.min_Elevation&&elevation2 >= m_simuDefine.min_Elevation&&flag1 == false && flag2 == true)
					{
						++tq2;
						////flag1 = false;
						////flag2 = true;
						// 便于画图
						//obsLine.t  = t_Receive;
						//obsLine.R1 = 0;
						//obsLine.V1 = 0;
						//obsLine.Elevation = -1.0;
						//obsLine.Azimuth   = -1.0;
						//obsLine.R0g       = 0;
						//obsLine.R0s       = 0;
						//obsLine.err_Iono  = 0;
						//obsLine.err_Trop  = 0;

						//obsLine.staName      = staname;
						//obsLine.satName      = satname;
						//obsLine.time_Doppler = DopplerTime0;

						//obsLine.t0 = t - DopplerTime0;
						//obsLine.t1 = t;
						//obsFile.m_data.push_back(obsLine);
						bflag = true;
					    break;
					}
					else if(Elevation >= m_simuDefine.min_Elevation&&elevation3 >= m_simuDefine.min_Elevation&&flag1 == false && flag3 == true)
					{
						++tq3;
						////flag1 = false;
						////flag3 = true;
						// 便于画图
						//obsLine.t  = t_Receive;
						//obsLine.R1 = 0;
						//obsLine.V1 = 0;
						//obsLine.Elevation = -1.0;
						//obsLine.Azimuth   = -1.0;
						//obsLine.R0g       = 0;
						//obsLine.R0s       = 0;
						//obsLine.err_Iono  = 0;
						//obsLine.err_Trop  = 0;

						//obsLine.staName      = staname;
						//obsLine.satName      = satname;
						//obsLine.time_Doppler = DopplerTime0;

						//obsLine.t0 = t - DopplerTime0;
						//obsLine.t1 = t;
						//obsFile.m_data.push_back(obsLine);
						bflag = true;
					    break;
					}

					// 如果

					// 计算下行几何距离和径向速度
					double dR_down = sqrt(pow(staPV.x - satOrb.pos.x, 2)
						                + pow(staPV.y - satOrb.pos.y, 2)
										+ pow(staPV.z - satOrb.pos.z, 2));

					// 迭代计算上行信号延迟时间
					double dDelay_k_1 = 0;
					m_TimeCoordConvert.ECEF_J2000( m_TimeCoordConvert.UTC2GPST(tr_utc), x_j2000, x_ecf, true);
					POS6D staPV_tt;
					staPV_tt.x  = x_j2000[0];
					staPV_tt.y  = x_j2000[1];
					staPV_tt.z  = x_j2000[2];
					staPV_tt.vx = x_j2000[3];
					staPV_tt.vy = x_j2000[4];
					staPV_tt.vz = x_j2000[5];
					double dR_up =  sqrt(pow(staPV_tt.x - satOrb.pos.x, 2) + pow(staPV_tt.y - satOrb.pos.y, 2) + pow(staPV_tt.z - satOrb.pos.z, 2));
					delay = dR_up / SPEED_LIGHT;
					UTC tt_utc = tr_utc - delay;
					while(fabs(delay - dDelay_k_1) > 1.0E-10)
					{   // 更新延迟时间
						dDelay_k_1 = delay;
						// 根据 dDelay 计算地面信号时间
						tt_utc = tr_utc - delay;
						// 获得 J2000 惯性系下的观测站位置
						m_TimeCoordConvert.ECEF_J2000( m_TimeCoordConvert.UTC2GPST(tt_utc), x_j2000, x_ecf, true);
						staPV_tt.x  = x_j2000[0];
						staPV_tt.y  = x_j2000[1];
						staPV_tt.z  = x_j2000[2];
						staPV_tt.vx = x_j2000[3];
						staPV_tt.vy = x_j2000[4];
						staPV_tt.vz = x_j2000[5];
						// 计算上行几何距离和径向速度
						dR_up = sqrt(pow(staPV_tt.x - satOrb.pos.x, 2) + pow(staPV_tt.y - satOrb.pos.y, 2) + pow(staPV_tt.z - satOrb.pos.z, 2));
						delay = dR_up / SPEED_LIGHT;
					}

					dR[k] = 0.5 * (dR_up + dR_down);

					if(k == 0)
					{
						obsLine.Azimuth   = Azimuth;
						obsLine.Elevation = Elevation;
					}

				}

				if(bflag)
				{
					t = t + h;
					continue;
				}

				obsLine.t  = t;
				obsLine.R  = dR[0];         // 距离真值
				obsLine.R1 = obsLine.R;     // 原始测距值 
				obsLine.V  = (dR[2] - dR[1]) / DopplerTime0;
				obsLine.V1 = obsLine.V;	

				// 添加高斯白噪声  测试代码
				if(m_simuDefine.on_ObsRandnNoise)
				{ 
					// 测距随机误差
					//obsLine.R1 = obsLine.R1 + RandNormal(0,0.01); // 1米随机误差；激光测距1cm
					obsLine.R1 = obsLine.R1 + RandNormal(0,m_simuDefine.r_Noise); // 1米随机误差；激光测距1cm

					//obsLine.R1 = obsLine.R1 + RandNormal(m_staInfo.mu,m_staInfo.sigma); //测站的噪声信息

					// 测速随机误差
					//obsLine.V1 = obsLine.V1 + RandNormal(0,0.00003); // 速度0.1mm/s随机误差
					obsLine.V1 = obsLine.V1 + RandNormal(0,m_simuDefine.v_Noise); // 速度0.1mm/s随机误差
				}

				// 测站系统偏差
				if(m_simuDefine.on_StaZeroDelayNoise)
				{
					obsLine.R1 = obsLine.R1 + m_staInfo.StaZeroDelay;			
				}

				// 卫星系统偏差
				if(m_simuDefine.on_SatZeroDelayNoise)
				{
					obsLine.R1 = obsLine.R1 + m_satInfo.SatZeroDelay;
				}

				// 更新测站名字，卫星名字
				obsLine.staName = staname;
				obsLine.satName = satname;
				obsLine.time_Doppler = DopplerTime0;
				obsLine.t0 = t - DopplerTime0;
				obsLine.t1 = t;

				obsFile.m_data.push_back(obsLine); 

				flag1 = true;
				++tq1;

				t = t + h;
			}
			return true;
		}
	}
}
