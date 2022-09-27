#include "structDef.hpp"
#include "TQGNSSSatDynPOD.hpp"
#include "LeoGPSObsPreproc.hpp"
#include "GNSSBasicCorrectFunc.hpp"
#include "TimeCoordConvert.hpp"
#include "RuningInfoFile.hpp"

namespace NUDTTK
{
	namespace TQGNSSPod
	{

		TQGNSSSatDynPOD::TQGNSSSatDynPOD(void)
		{
			m_stepAdamsCowell = 75.0; 
			m_matAxisBody2RTN.MakeUnitMatrix(3); // 20150309, 谷德峰, 初始化为单位矩阵
		}

		TQGNSSSatDynPOD::~TQGNSSSatDynPOD(void)
		{
		}

		void TQGNSSSatDynPOD::setSP3File(SP3File sp3File)
		{
			m_sp3File = sp3File;
		}

		void TQGNSSSatDynPOD::setCLKFile(CLKFile clkFile)
		{
			m_clkFile = clkFile;
		}

		//CLKFile TQGNSSSatDynPOD::getRecClkFile()
		//{
		//	return m_recClkFile;
		//}

		void TQGNSSSatDynPOD::setStepAdamsCowell(double step)
		{
			m_stepAdamsCowell = step;
		}

		double  TQGNSSSatDynPOD::getStepAdamsCowell()
		{
			return m_stepAdamsCowell;
		}

		bool TQGNSSSatDynPOD::loadSP3File(string  strSp3FileName)
		{
			return m_sp3File.open(strSp3FileName);
		}

		bool TQGNSSSatDynPOD::loadCLKFile(string  strCLKFileName)
		{
			return m_clkFile.open(strCLKFileName);
		}

		bool TQGNSSSatDynPOD::loadEditedObsFile(string  strEditedObsFileName)
		{
			return m_editedObsFile.open(strEditedObsFileName);
		}

		bool TQGNSSSatDynPOD::adamsCowell_Interp_Leo(vector<TDT> interpTimelist, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist, vector<Matrix> &matRtPartiallist, double h, int q)
		{
			return adamsCowell_Interp(interpTimelist, dynamicDatum, orbitlist, matRtPartiallist, h, q);
		}

		// 子程序名称： initDynDatumEst   
		// 功能：对运动学轨道点位信息进行动力学平滑, 输出初始轨道参数
		// 变量类型：orbitlist        : 几何轨道
		//           dynamicDatum     : 动力学轨道根数 
		//           arclength        : 弧段的长度
		// 输入：orbitlist, arclength
		// 输出：dynamicDatum
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2008/11/14
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool TQGNSSSatDynPOD::initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength)
		{
			char info[200];
			// 提取粗间隔的插值点
			double  threshold_coarseorbit_interval = m_podParaDefine.threshold_initDynDatumEst;
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
				GPST t_GPS = orbitlist[s_i].t;
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
			dynamicDatum.bOn_EarthIrradianceAcc = false;
			dynamicDatum.bOn_RadialForceAcc     = false;
			dynamicDatum.bOn_ManeuverForceAcc  = false; // +
			dynamicDatum.bOn_NonconservativeForce = false;
			dynamicDatum.init(dynamicDatum.ArcLength, dynamicDatum.ArcLength, dynamicDatum.ArcLength, dynamicDatum.ArcLength, dynamicDatum.ArcLength);
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
			while(1)
			{
				k++;
				if(k >= m_podParaDefine.max_OrbitIterativeNum)
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
					adamsCowell_Interp_Leo(interpTimelist, dynamicDatum, interpOrbitlist, interpRtPartiallist, m_stepAdamsCowell);
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
					adamsCowell_Interp_Leo(interpTimelist_control, dynamicDatum, interpOrbitlist, interpRtPartiallist, m_stepAdamsCowell);
				}
				// 判断光压参数是否为 0, 因为短弧段定轨可能低轨卫星的完全存在于阴影中
				/*int count_SolarPressure = 0;
                for(int i = 0; i < int(count_measureorbit_control); i++)
				{
					double solarCoefficient =   pow(interpRtPartiallist[i].GetElement(0, 6), 2)
						                      + pow(interpRtPartiallist[i].GetElement(1, 6), 2)
											  + pow(interpRtPartiallist[i].GetElement(2, 6), 2);
					if(solarCoefficient != 0)
						count_SolarPressure++;
				}*/
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
				//dynamicDatum.solarPressureParaList[0].Cr = 0;

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
						double rms_fitresidual = 0; 
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
				sprintf(info, "初始轨道解算成功!(%d/%d)", count_measureorbit_control, orbitlist.size());
				RuningInfoFile::Add(info);
				printf("%s\n", info);
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

		// 子程序名称： weighting_Elevation   
		// 功能：根据高度角获得观测权重
		// 变量类型：Elevation        : 观测高度角
		//           weight_P_IF      : 伪码观测权值 
		//           weight_L_IF      : 相位观测权值
		// 输入：Elevation
		// 输出：weight_P_IF, weight_L_IF
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/07/10
		// 版本时间：
		// 修改记录：
		// 备注： 
		void TQGNSSSatDynPOD::weighting_Elevation(double Elevation, double& weight_P_IF, double& weight_L_IF)
		{
			if(Elevation <= m_podParaDefine.min_elevation)
			{
				weight_P_IF = 0.0;
				weight_L_IF = 0.0;
			}
			else
			{
				weight_P_IF = 1.0;
				weight_L_IF = 1.0;

				if(Elevation <= 30 && m_podParaDefine.bOn_WeightElevation)
				{
					weight_P_IF = 2 * sin(Elevation * PI / 180);	// GFZ软件的策略, 10-0.34, 5-0.17
					weight_L_IF = 2 * sin(Elevation * PI / 180);					
				}
			}
			//if(!m_podParaDefine.bOn_WeightElevation)
			//{
			//	if(Elevation <= m_podParaDefine.min_elevation)
			//	{
			//		weight_P_IF = 0.0;
			//		weight_L_IF = 0.0;
			//	}
			//	else
			//	{
			//		weight_P_IF = 1.0;
			//		weight_L_IF = 1.0;
			//	}
			//	return;
			//}

			//if(m_podParaDefine.flag_UseSingleFrequency == 2)
			//{
			//	double balance_obs_elevation    = 30.0; // 平衡角, 其对应的观测权值为 1.0
			//	double weight_min_obs_elevation = 0.10; // 最低仰角的观测权值
			//	if(Elevation <= m_podParaDefine.min_elevation)
			//	{
			//		weight_P_IF = 0.0;
			//		weight_L_IF = 0.0;
			//	}
			//	else if(Elevation <= 30)
			//	{
			//		double k = (1 - weight_min_obs_elevation)/(balance_obs_elevation - m_podParaDefine.min_elevation);
			//		weight_P_IF = weight_min_obs_elevation + k * (Elevation - m_podParaDefine.min_elevation);
			//		weight_L_IF = 1.0;
			//	}
			//	else
			//	{
			//		weight_P_IF = 1.0;
			//		weight_L_IF = 1.0;
			//	}
			//}
		}
		bool TQGNSSSatDynPOD::loadMixedEditedObsFile(string  strMixedEditedObsFileName)
		{
			return m_editedMixedObsFile.open(strMixedEditedObsFileName);
		}

		bool TQGNSSSatDynPOD::loadMixedEditedObsFile_5Obs(string  strMixedEditedObsFileName)
		{
			return m_editedMixedObsFile.open_5Obs(strMixedEditedObsFileName);
		}
		// 子程序名称： dynamicTQGnssMixedPOD   
		// 功能：dynamicTQGnssMixedPOD
		// 变量类型：editedMixedObsFilePath : 编辑后混合格式数据文件路径
        //           dynamicDatum           : 拟合后的初始动力学轨道参数
		//           t0_forecast            : 预报轨道初始时间, GPST
		//           t1_forecast            : 预报轨道终止时间, GPST
		//           forecastOrbList        : 预报轨道列表, 采用GPST, ITRF坐标系
		//           interval               : 预报轨道间隔
		//           bInitDynDatumEst       : 初始动力学轨道求解标记
		//           bForecast              : 预报标记, 默认true, 否则不进行预报, 用于初轨确定
		//           bResEdit               : 定轨 O-C 残差编辑标记开关, 默认 true, 否则不进行残差编辑
		// 输入：dynamicDatum, t0_forecast, t1_forecast, interval, bInitDynDatumEst, bForecast, bResEdit
		// 输出：dynamicDatum, forecastOrbList
		// 语言：C++
		// 创建者：邵凯
		// 创建时间：2022/03/23
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool TQGNSSSatDynPOD::dynamicTQGnssMixedPOD(string editedMixedObsFilePath, SatdynBasicDatum &dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval,  bool bInitDynDatumEst, bool bForecast, bool bResEdit)
		{
			char info[200];
			// 分析 editedMixedObsFilePath 路径, 提取根目录和文件名
			string folder;
			string editedMixedObsFileName;
			if(editedMixedObsFilePath.find_last_of("\\") != -1) // 防止默认为当前路径 folder == "" 时无法处理, 谷德峰, 2015/04/11 
			{
				folder = editedMixedObsFilePath.substr(0, editedMixedObsFilePath.find_last_of("\\")) + "\\";
				editedMixedObsFileName = editedMixedObsFilePath.substr(editedMixedObsFilePath.find_last_of("\\") + 1);
			}
			else
			{
				folder = "";
				editedMixedObsFileName = editedMixedObsFilePath;
			}
			
			string editedMixedObsFileName_noexp = editedMixedObsFileName.substr(0, editedMixedObsFileName.find_last_of("."));
			if(!loadMixedEditedObsFile_5Obs(editedMixedObsFilePath)) // 2014/12/07, 针对Mixed格式数据融合进行调整, GPS + BDS 
				return false;
			// 确认双频观测数据是否完整
			int nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_L1 = -1, nObsTypes_L2 = -1;
			for(int i = 0; i < m_editedMixedObsFile.m_header.byObsTypes; i++)
			{
				if(m_editedMixedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1 = i;
				if(m_editedMixedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2 = i;
				if(m_editedMixedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1 = i;
				if(m_editedMixedObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2 = i;
			}
			// 将 GNSS 卫星星历转换到 J2000 惯性系
			SP3File sp3File_J2000 = m_sp3File;		// 此处为了不改变sp3文件数据，2014-11-24，鞠 冰
			if(sp3File_J2000.m_data.size() <= 0)
			{
				printf("GNSS星历文件为空! \n");
				return false;
			}
			for(size_t s_i = 0; s_i < sp3File_J2000.m_data.size(); s_i++)
			{
				for(SP3SatMap::iterator it = sp3File_J2000.m_data[s_i].sp3.begin(); it != sp3File_J2000.m_data[s_i].sp3.end(); ++it)
				{
					double x_ecf[3];
					double x_j2000[3];
					x_ecf[0] = it->second.pos.x * 1000;  
					x_ecf[1] = it->second.pos.y * 1000; 
					x_ecf[2] = it->second.pos.z * 1000;
					m_TimeCoordConvert.ECEF_J2000(sp3File_J2000.m_data[s_i].t, x_j2000, x_ecf, false);
					it->second.pos.x = x_j2000[0] / 1000;  
					it->second.pos.y = x_j2000[1] / 1000; 
					it->second.pos.z = x_j2000[2] / 1000;
				}
			}

			// 进行初轨确定
			if(!bInitDynDatumEst)
			{
				vector<TimePosVel> orbitlist;
				double arclength_initDynDatumEst = 3600.0 * 6;       // 2022.04.06,jiong
				for(size_t s_i = 0; s_i < m_editedMixedObsFile.m_data.size(); s_i++)
				{
					if(m_editedMixedObsFile.m_data[s_i].byRAIMFlag == 2)
					{
						TimePosVel orb_i;
						orb_i.t   = m_editedMixedObsFile.m_data[s_i].t;
						orb_i.pos = m_editedMixedObsFile.m_data[s_i].pos;
						orb_i.vel = m_editedMixedObsFile.m_data[s_i].vel;
						if(orbitlist.size() == 0)
							orbitlist.push_back(orb_i);
						else
						{
							if(orb_i.t - orbitlist[0].t <= arclength_initDynDatumEst)
								orbitlist.push_back(orb_i);
							else
								break;
						}
					}
				}
				SatdynBasicDatum dynamicDatum_0;
				dynamicDatum_0.bOn_SolidTide     = dynamicDatum.bOn_SolidTide; // 20161208, 便于外部仿真控制
				dynamicDatum_0.bOn_OceanTide     = dynamicDatum.bOn_OceanTide;
				dynamicDatum_0.oceanTideType     = dynamicDatum.oceanTideType; // 20200503，潮汐类型开关
				dynamicDatum_0.bOn_OceanPoleTide = dynamicDatum.bOn_OceanPoleTide;
				dynamicDatum_0.bOn_SolidPoleTide = dynamicDatum.bOn_SolidPoleTide;
				dynamicDatum_0.bOn_ThirdBodyAcc  = dynamicDatum.bOn_ThirdBodyAcc;
				dynamicDatum_0.earthGravityType  = dynamicDatum.earthGravityType;
				if(!initDynDatumEst(orbitlist, dynamicDatum_0,  arclength_initDynDatumEst))
					return false;
				TDT t_End = TimeCoordConvert::GPST2TDT(m_editedMixedObsFile.m_data[m_editedMixedObsFile.m_data.size() - 1].t);
				dynamicDatum.T0 = dynamicDatum_0.T0;
				dynamicDatum.ArcLength = t_End - dynamicDatum.T0;
				dynamicDatum.X0 = dynamicDatum_0.X0;
				dynamicDatum.init(m_podParaDefine.period_SolarPressure, 
					              m_podParaDefine.period_AtmosphereDrag, 
								  m_podParaDefine.period_EmpiricalAcc,
								  m_podParaDefine.period_EarthIrradiance, 
								  m_podParaDefine.period_RadialEmpForce);
			}

			vector<Rinex2_1_LeoMixedEditedObsEpoch>  editedMixedObsEpochlist; // 混合格式数据
			if(!m_editedMixedObsFile.getEditedObsEpochList(editedMixedObsEpochlist))
				return false;
			
			// 2014/12/07, 针对Mixed格式数据融合进行调整, GPS + BDS 
			m_dataMixedGNSSlist.clear();
			MixedGNSSTQPODDatum dataMixedGPS(GPS_FREQUENCE_L1, GPS_FREQUENCE_L2);
			dataMixedGPS.cSatSystem = 'G';
			dataMixedGPS.index_P1 = nObsTypes_P1;
			dataMixedGPS.index_P2 = nObsTypes_P2;
			dataMixedGPS.index_L1 = nObsTypes_L1;
			dataMixedGPS.index_L2 = nObsTypes_L2;
			dataMixedGPS.editedObsEpochlist.clear();
			dataMixedGPS.editedObsSatlist.clear();
			if(dataMixedGPS.index_P1 != -1 && dataMixedGPS.index_P2 != -1 && dataMixedGPS.index_L1 != -1 && dataMixedGPS.index_L2 != -1) 
			{
				if(Rinex2_1_LeoMixedEditedObsFile::mixedGNSS2SingleSysEpochList(editedMixedObsEpochlist, dataMixedGPS.editedObsEpochlist, dataMixedGPS.cSatSystem))
				{
					if(Rinex2_1_LeoEditedObsFile::getEditedObsSatList(dataMixedGPS.editedObsEpochlist, dataMixedGPS.editedObsSatlist))
					{
						m_dataMixedGNSSlist.push_back(dataMixedGPS);
					}
				}
			}
			MixedGNSSTQPODDatum dataMixedBDS(BD_FREQUENCE_L1, BD_FREQUENCE_L2);//
			dataMixedBDS.cSatSystem = 'C';
			dataMixedBDS.index_P1 = nObsTypes_P1;
			dataMixedBDS.index_P2 = nObsTypes_P2;// 
			dataMixedBDS.index_L1 = nObsTypes_L1; 
			dataMixedBDS.index_L2 = nObsTypes_L2; // 
			dataMixedBDS.editedObsEpochlist.clear();
			dataMixedBDS.editedObsSatlist.clear();
			if(dataMixedBDS.index_P1 != -1 && dataMixedBDS.index_P2 != -1 && dataMixedBDS.index_L1 != -1 && dataMixedBDS.index_L2 != -1) 
			{
				if(Rinex2_1_LeoMixedEditedObsFile::mixedGNSS2SingleSysEpochList(editedMixedObsEpochlist, dataMixedBDS.editedObsEpochlist, dataMixedBDS.cSatSystem))
				{
					if(Rinex2_1_LeoEditedObsFile::getEditedObsSatList(dataMixedBDS.editedObsEpochlist, dataMixedBDS.editedObsSatlist))
					{
						m_dataMixedGNSSlist.push_back(dataMixedBDS);
					}
				}
			}
			//////GLONASS 2022.04.05
			MixedGNSSTQPODDatum dataMixedGLO(1605.375E+6, 1248.625E+6);//
			dataMixedGLO.cSatSystem = 'R';				  
			dataMixedGLO.index_P1 = nObsTypes_P1;
			dataMixedGLO.index_P2 = nObsTypes_P2;// 
			dataMixedGLO.index_L1 = nObsTypes_L1; 
			dataMixedGLO.index_L2 = nObsTypes_L2; // 
			dataMixedGLO.editedObsEpochlist.clear();
			dataMixedGLO.editedObsSatlist.clear();
			if(dataMixedGLO.index_P1 != -1 && dataMixedGLO.index_P2 != -1 && dataMixedGLO.index_L1 != -1 && dataMixedGLO.index_L2 != -1) 
			{
				if(Rinex2_1_LeoMixedEditedObsFile::mixedGNSS2SingleSysEpochList(editedMixedObsEpochlist, dataMixedGLO.editedObsEpochlist, dataMixedGLO.cSatSystem))
				{
					if(Rinex2_1_LeoEditedObsFile::getEditedObsSatList(dataMixedGLO.editedObsEpochlist, dataMixedGLO.editedObsSatlist))
					{
						m_dataMixedGNSSlist.push_back(dataMixedGLO);
					}
				}
			}
			// 整理观测值
			size_t count_MixedEpoch = editedMixedObsEpochlist.size(); 
			vector<POS3D> exyzBodyList[3];
			exyzBodyList[0].resize(count_MixedEpoch); // 记录每个时刻星固系坐标轴在J2000惯性系下的指向
			exyzBodyList[1].resize(count_MixedEpoch);
			exyzBodyList[2].resize(count_MixedEpoch);
			for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
			{
				m_dataMixedGNSSlist[s_m].mapDynEpochList.clear();
				m_dataMixedGNSSlist[s_m].P_IFEpochList.clear();
				m_dataMixedGNSSlist[s_m].mixedEpochIdList.clear();
				m_dataMixedGNSSlist[s_m].epochIdList.clear();
				for(size_t s_i = 0; s_i < count_MixedEpoch; s_i++)
					m_dataMixedGNSSlist[s_m].epochIdList.push_back(-1); // 记录混合历元序列在[当前格式]中的历元序号
				int s_index = -1; // 有效时间标签, 初始化为 0
				for(size_t s_i = 0; s_i < m_dataMixedGNSSlist[s_m].editedObsEpochlist.size(); s_i++)
				{
					bool bValid = true;
					int eyeableGNSSCount = 0;
					int nObsTime;
					for(Rinex2_1_EditedObsSatMap::iterator it = m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].editedObs.begin(); it != m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].editedObs.end(); ++it)
					{
						Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[m_dataMixedGNSSlist[s_m].index_P1];
						Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[m_dataMixedGNSSlist[s_m].index_P2];
						nObsTime = it->second.nObsTime;
						if( P1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL
						 && P2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
							eyeableGNSSCount++;
					}
					m_dataMixedGNSSlist[s_m].mixedEpochIdList.push_back(nObsTime);  // 记录当前历元序列在[混合格式]中的历元序号
					m_dataMixedGNSSlist[s_m].epochIdList[nObsTime] = int(s_i);
					if(eyeableGNSSCount < m_podParaDefine.min_eyeableGPScount) // 条件: 可视GNSS卫星个数大于等于 2, 保证钟差数据可解
						bValid = false;
					PODEpoch dynEpoch_i;
					dynEpoch_i.mapDatum.clear();
					if(bValid)
					{
						dynEpoch_i.eyeableGPSCount = eyeableGNSSCount;
						ObsEqEpoch epochP_IF; 
						epochP_IF.nObsTime = nObsTime; // 保持与 Mixed 格式中的时间相同
						epochP_IF.obsSatList.clear(); 
						for(Rinex2_1_EditedObsSatMap::iterator it = m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].editedObs.begin(); it != m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].editedObs.end(); ++it)
						{
							ObsEqEpochElement element_P_IF;
							Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[m_dataMixedGNSSlist[s_m].index_P1];
							Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[m_dataMixedGNSSlist[s_m].index_P2];
							element_P_IF.obs = P1.obs.data - ( P1.obs.data - P2.obs.data ) * m_dataMixedGNSSlist[s_m].coefficient_L2; // 记录原始观测信息 
							if(m_podParaDefine.flag_UseSingleFrequency == 1)
								element_P_IF.obs = P1.obs.data;
							element_P_IF.id_Sat = it->first;
							epochP_IF.obsSatList.push_back(element_P_IF);
							PODEpochElement datum_j;
							double weight_P_IF = 1.0;
							// 理论观测观测权值
							datum_j.weightCode   = 1.0;
							datum_j.weightCode  *= weight_P_IF * m_dataMixedGNSSlist[s_m].weightSystem; // 增加系统权值
							if(P1.byEditedMark1 != TYPE_EDITEDMARK_NORMAL || P2.byEditedMark1 != TYPE_EDITEDMARK_NORMAL)
								datum_j.weightCode = 0;
							dynEpoch_i.mapDatum.insert(PODEpochSatMap::value_type(element_P_IF.id_Sat, datum_j));
						}
						s_index++;
						dynEpoch_i.validIndex = s_index; // 并没有实际的意义, 请参考 mixedValidIndexList[nObsTime]
						m_dataMixedGNSSlist[s_m].P_IFEpochList.push_back(epochP_IF);
						m_dataMixedGNSSlist[s_m].mapDynEpochList.insert(map<int, PODEpoch>::value_type(nObsTime, dynEpoch_i)); // 以混合格式的时间为索引
					}
					else
					{// 无效点也要保存标记
						dynEpoch_i.eyeableGPSCount = 0;
						dynEpoch_i.validIndex = -1;
						for(Rinex2_1_EditedObsSatMap::iterator it = m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].editedObs.begin(); it != m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].editedObs.end(); ++it)
						{
							PODEpochElement datum_j;
							dynEpoch_i.mapDatum.insert(PODEpochSatMap::value_type(it->first, datum_j));
						}
						m_dataMixedGNSSlist[s_m].mapDynEpochList.insert(map<int, PODEpoch>::value_type(nObsTime, dynEpoch_i));
					}
				}
			}	
			for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m ++)
			{
				m_dataMixedGNSSlist[s_m].leoClockList.resize(count_MixedEpoch);	        // 钟差
				m_dataMixedGNSSlist[s_m].validSysIndexList.resize(count_MixedEpoch);    // 钟差有效解标识，以混合格式的时间为索引
		     	m_dataMixedGNSSlist[s_m].EyeableSysCountList.resize(count_MixedEpoch);
				int s_index = -1;		
				for(size_t s_i = 0; s_i < count_MixedEpoch; s_i++)
				{
					m_dataMixedGNSSlist[s_m].leoClockList[s_i] = editedMixedObsEpochlist[s_i].clock; // 初始化
					m_dataMixedGNSSlist[s_m].EyeableSysCountList[s_i] = 0;
					map<int, PODEpoch>::iterator it = m_dataMixedGNSSlist[s_m].mapDynEpochList.find(int(s_i));
					if(it != m_dataMixedGNSSlist[s_m].mapDynEpochList.end())
					{
						if(it->second.validIndex != -1)
						{
							m_dataMixedGNSSlist[s_m].EyeableSysCountList[s_i] = it->second.eyeableGPSCount;
						}
					}
					if(m_dataMixedGNSSlist[s_m].EyeableSysCountList[s_i] > 0)		// 	
					{
						s_index++;
						m_dataMixedGNSSlist[s_m].validSysIndexList[s_i] = s_index;
						m_dataMixedGNSSlist[s_m].validSysObsTimeList.push_back(int(s_i));  // // 钟差有效解的位置对应的时刻位置，以混合格式的时间为索引
					}
					else
					{
						m_dataMixedGNSSlist[s_m].validSysIndexList[s_i] = -1;

					}
				}
				m_dataMixedGNSSlist[s_m].count_clk = s_index + 1;
				sprintf(info, "%c 有效钟差点数量 %8d !", m_dataMixedGNSSlist[s_m].cSatSystem, m_dataMixedGNSSlist[s_m].count_clk);
			    RuningInfoFile::Add(info);
			}
			// 保存初轨确定结果
			SatdynBasicDatum dynamicDatum_Init = dynamicDatum; // 记录初始轨道根数
			char dynamicDatumFilePath[300];
			sprintf(dynamicDatumFilePath,"%sdynpod_%s.fit", folder.c_str(), editedMixedObsFileName_noexp.c_str());
			FILE * pFitFile = fopen(dynamicDatumFilePath, "w+");
			fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
			fprintf(pFitFile, "%3d. PCO  X    (cm)     %20.4f\n", 1,m_pcoAnt.x * 100.0);
			fprintf(pFitFile, "%3d. PCO  Y    (cm)     %20.4f\n", 2,m_pcoAnt.y * 100.0);
			fprintf(pFitFile, "%3d. PCO  Z    (cm)     %20.4f\n", 3,m_pcoAnt.z * 100.0);
			fprintf(pFitFile, "%3d.      X    (m)      %20.4f\n", 4,dynamicDatum_Init.X0.x);
			fprintf(pFitFile, "%3d.      Y    (m)      %20.4f\n", 5,dynamicDatum_Init.X0.y);
			fprintf(pFitFile, "%3d.      Z    (m)      %20.4f\n", 6,dynamicDatum_Init.X0.z);
			fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f\n", 7,dynamicDatum_Init.X0.vx);
			fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f\n", 8,dynamicDatum_Init.X0.vy);
			fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f\n", 9,dynamicDatum_Init.X0.vz);
			int k_Parameter = 9;
			if(dynamicDatum_Init.bOn_SolarPressureAcc && (dynamicDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_1PARA || dynamicDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_1PARA_AM || dynamicDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_MACRO))
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
			// 2022.04.04，韦春博，添加九参数光压模型
			else if(dynamicDatum_Init.bOn_SolarPressureAcc && dynamicDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
			{
				for(size_t s_i = 0; s_i < dynamicDatum_Init.solarPressureParaList.size(); s_i++)
				{
					k_Parameter++;
					//dynamicDatum_Init.solarPressureParaList[s_i].D0 = 0;
					fprintf(pFitFile, "%3d. %2d   D0   (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7);
					k_Parameter++;
					//dynamicDatum_Init.solarPressureParaList[s_i].DC1 = 0;
					fprintf(pFitFile, "%3d. %2d   DCOS (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].DC1 * 1.0E+7);
					k_Parameter++;
					//dynamicDatum_Init.solarPressureParaList[s_i].DS1 = 0;
					fprintf(pFitFile, "%3d. %2d   DSIN (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].DS1 * 1.0E+7);
					k_Parameter++;
					//dynamicDatum_Init.solarPressureParaList[s_i].Y0 = 0;
					fprintf(pFitFile, "%3d. %2d   Y0   (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7);
					k_Parameter++;
					//dynamicDatum_Init.solarPressureParaList[s_i].YC1 = 0;
					fprintf(pFitFile, "%3d. %2d   YCOS (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].YC1 * 1.0E+7);
					k_Parameter++;
					//dynamicDatum_Init.solarPressureParaList[s_i].YS1 = 0;
					fprintf(pFitFile, "%3d. %2d   YSIN (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].YS1 * 1.0E+7);
					k_Parameter++;
					//dynamicDatum_Init.solarPressureParaList[s_i].B0 = 0;
					fprintf(pFitFile, "%3d. %2d   B0   (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].B0 * 1.0E+7);
					k_Parameter++;
					//dynamicDatum_Init.solarPressureParaList[s_i].BC1 = 0;
					fprintf(pFitFile, "%3d. %2d   BCOS (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].BC1 * 1.0E+7);
					k_Parameter++;
					//dynamicDatum_Init.solarPressureParaList[s_i].BS1 = 0;
					fprintf(pFitFile, "%3d. %2d   BSIN (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].BS1 * 1.0E+7);
				}
			}
			else if(dynamicDatum_Init.bOn_SolarPressureAcc && dynamicDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_5PARA)
			{
				for(size_t s_i = 0; s_i < dynamicDatum_Init.solarPressureParaList.size(); s_i++)
				{
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].D0 = 0;
					fprintf(pFitFile, "%3d. %2d   D0   (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7);
					k_Parameter++;
					dynamicDatum_Init.solarPressureParaList[s_i].Y0 = 0;
					fprintf(pFitFile, "%3d. %2d   Y0   (1.0E-7) %20.4f\n", k_Parameter,  
																		   s_i+1,
																		   dynamicDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7);
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
			if(dynamicDatum_Init.bOn_AtmosphereDragAcc)
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
			if(dynamicDatum_Init.bOn_EmpiricalForceAcc && dynamicDatum_Init.empiricalForceType == TYPE_EMPIRICALFORCE_SPLINE)
			{
				size_t s_i;
				for(s_i = 0; s_i < dynamicDatum_Init.empiricalForceParaList.size(); s_i++)
				{// 20140320, 谷德峰修改, 使得三个方向经验力可选
					if(dynamicDatum_Init.bOn_EmpiricalForceAcc_R)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   A0_R (1.0E-7) %20.4f\n",    k_Parameter,
																				  s_i+1,
																				  dynamicDatum_Init.empiricalForceParaList[s_i].a0_R * 1.0E+7);
					}
					if(dynamicDatum_Init.bOn_EmpiricalForceAcc_T)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   A0_T (1.0E-7) %20.4f\n",    k_Parameter,
																				  s_i+1,
																				  dynamicDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7);
					}
					if(dynamicDatum_Init.bOn_EmpiricalForceAcc_N)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   A0_N (1.0E-7) %20.4f\n",    k_Parameter,
																				  s_i+1,
																				  dynamicDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7);
					}
				}
				s_i = dynamicDatum_Init.empiricalForceParaList.size() - 1;
				if(dynamicDatum_Init.bOn_EmpiricalForceAcc_R)
				{
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   A0_R (1.0E-7) %20.4f\n",        k_Parameter,
																				  s_i+2,
																				  dynamicDatum_Init.empiricalForceParaList[s_i].a1_R * 1.0E+7);
				}
				if(dynamicDatum_Init.bOn_EmpiricalForceAcc_T)
				{
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   A0_T (1.0E-7) %20.4f\n",        k_Parameter,
																				  s_i+2,
																				  dynamicDatum_Init.empiricalForceParaList[s_i].a1_T * 1.0E+7);
				}
				if(dynamicDatum_Init.bOn_EmpiricalForceAcc_N)
				{
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   A0_N (1.0E-7) %20.4f\n",        k_Parameter,
																				  s_i+2,
																				  dynamicDatum_Init.empiricalForceParaList[s_i].a1_N * 1.0E+7);
				}
			}
			if(dynamicDatum_Init.bOn_ManeuverForceAcc && dynamicDatum.bOnEst_Maneuver)
			{// 机动力, 2013/10/28, 谷德峰
				for(size_t s_i = 0; s_i < dynamicDatum_Init.maneuverForceParaList.size(); s_i++)
				{
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   MA0R (1.0E-4) %20.4f\n", k_Parameter,
						                                                   s_i+1,
						                                                   dynamicDatum_Init.maneuverForceParaList[s_i].a0_R * 1.0E+4);
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   MA0T (1.0E-4) %20.4f\n", k_Parameter,
						                                                   s_i+1,
						                                                   dynamicDatum_Init.maneuverForceParaList[s_i].a0_T * 1.0E+4);
					k_Parameter++;
					fprintf(pFitFile, "%3d. %2d   MA0N (1.0E-4) %20.4f\n", k_Parameter,
						                                                   s_i+1,
						                                                   dynamicDatum_Init.maneuverForceParaList[s_i].a0_N * 1.0E+4);
				}
			}
			fclose(pFitFile);
			// 迭代开始
			bool flag_robust = false;
			int  num_after_residual_edit = 0;
			bool flag_break = false;
			bool result = true;
			int  k = 0; // 记录迭代的次数
			for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m ++)
			{
				m_dataMixedGNSSlist[s_m].interpOrbitlist.clear();
				m_dataMixedGNSSlist[s_m].interpRtPartiallist.clear();
				m_dataMixedGNSSlist[s_m].interpTimelist.resize(count_MixedEpoch);
			}
			int OnEst_LEOAntPCO_x = int(m_podParaDefine.bOnEst_LEOAntPCO_X); 
			int OnEst_LEOAntPCO_y = int(m_podParaDefine.bOnEst_LEOAntPCO_Y); 
			int OnEst_LEOAntPCO_z = int(m_podParaDefine.bOnEst_LEOAntPCO_Z); 
			int count_EstPara_LEOAntPCO = OnEst_LEOAntPCO_x + OnEst_LEOAntPCO_y + OnEst_LEOAntPCO_z;
			double weight_pco = m_podParaDefine.apriorityRms_LIF / m_podParaDefine.apriorityRms_LEOAntPCO;
			POS3D pcoAntEst;
			pcoAntEst.x = 0;
			pcoAntEst.y = 0;
			pcoAntEst.z = 0;
			// 动力学参数个数统计
			int count_DynParameter = dynamicDatum.getAllEstParaCount(); 
			int count_SolarPressureParaList = 0;
			if(dynamicDatum.bOn_SolarPressureAcc)
				count_SolarPressureParaList = int(dynamicDatum.solarPressureParaList.size());
			int count_EmpiricalForceParaList = 0;
			if(dynamicDatum.bOn_EmpiricalForceAcc)
				count_EmpiricalForceParaList = int(dynamicDatum.empiricalForceParaList.size());
			int count_ManeuverForceParaList = 0;
			if(dynamicDatum.bOn_ManeuverForceAcc && dynamicDatum.bOnEst_Maneuver) // 机动力, 2013/10/28, 谷德峰
				count_ManeuverForceParaList = int(dynamicDatum.maneuverForceParaList.size());
			int count_SolarPressurePara = dynamicDatum.getSolarPressureParaCount();
			int index_EmpiricalParaBegin = 0;     // 记录经验力参数在整个待估动力学参数列表中的位置，2014/10/07，谷德峰
			int index_ManeuverParaBegin = 0;      // 记录机动力参数在整个待估动力学参数列表中的位置，2014/ 5/10，鞠  冰
			while(1)
			{
				k++;
				if(k >= m_podParaDefine.max_OrbitIterativeNum)
				{
					result = false;	// 2014/06/18,发散轨道比较用，鞠 冰
					sprintf(info, "迭代超过%d次, 发散!", k);
					printf("%s\n");
					RuningInfoFile::Add(info);
					break;
				}
				// 利用初始动力学轨道, 获得概略轨道点
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					for(size_t s_i = 0; s_i < count_MixedEpoch; s_i++)
						m_dataMixedGNSSlist[s_m].interpTimelist[s_i] = TimeCoordConvert::GPST2TDT(editedMixedObsEpochlist[s_i].t - m_dataMixedGNSSlist[s_m].leoClockList[s_i] / SPEED_LIGHT);
					m_dataMixedGNSSlist[s_m].interpOrbitlist.clear();
					m_dataMixedGNSSlist[s_m].interpRtPartiallist.clear();
					adamsCowell_Interp_Leo(m_dataMixedGNSSlist[s_m].interpTimelist, dynamicDatum, 
							                 m_dataMixedGNSSlist[s_m].interpOrbitlist, m_dataMixedGNSSlist[s_m].interpRtPartiallist, m_stepAdamsCowell);
				}
				printf("第%d次 adamsCowell_Interp_Leo is ok!\n", k);
                sprintf(info, "第%d次 adamsCowell_Interp_Leo is ok!", k);
				RuningInfoFile::Add(info);
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					// 更新 dynEpochList
					int s_index = -1; // 有效时间标签, 初始化为 0
					for(size_t s_i = 0; s_i < m_dataMixedGNSSlist[s_m].editedObsEpochlist.size(); s_i++)
					{
						int nObsTime = m_dataMixedGNSSlist[s_m].mixedEpochIdList[s_i];
						POSCLK posclk; // 概略点位置
						posclk.x = m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].pos.x;
						posclk.y = m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].pos.y;
						posclk.z = m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].pos.z;
						posclk.clk = m_dataMixedGNSSlist[s_m].leoClockList[nObsTime];
						GPST t_Receive = m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].t - posclk.clk / SPEED_LIGHT;
						// 计算理想情况下J2000惯性系下的星固系坐标轴方向[与轨道系RTN重合], x-轨道飞行方向(+T); z-卫星天底方向(-R); y-右手系, 轨道法向(-N) 
						POS3D exRTN = vectorNormal(m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].vel); // 飞行方向     
						POS3D eyRTN;
						vectorCross(eyRTN, m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].pos *(-1.0), exRTN); // 天底方向 x 飞行方向
						eyRTN = vectorNormal(eyRTN);                        
						POS3D  ezRTN;
						vectorCross(ezRTN, exRTN, eyRTN); // 右手系
						double correct_leorelativity = 0.0;
						if(m_podParaDefine.bOn_RecRelativity)
						{
							correct_leorelativity  = ( m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].pos.x * m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].vel.x 
													 + m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].pos.y * m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].vel.y
													 + m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].pos.z * m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].vel.z) * (-2) / SPEED_LIGHT;
						}
						Matrix matPCO_J2000(3, 1);    // J2000惯性系下的LEO卫星天线偏移矢量
						POS3D exBody, eyBody, ezBody; // 计算实际情况下J2000惯性系下的星固系坐标轴方向[与轨道系RTN不完全重合]
						POS3D exAnt, eyAnt, ezAnt;    // 天线系
						Matrix matPCO_Body(3, 1);
						matPCO_Body.SetElement(0, 0, m_pcoAnt.x * int(m_podParaDefine.bOn_LEOAntPCO) + pcoAntEst.x);
						matPCO_Body.SetElement(1, 0, m_pcoAnt.y * int(m_podParaDefine.bOn_LEOAntPCO) + pcoAntEst.y);
						matPCO_Body.SetElement(2, 0, m_pcoAnt.z * int(m_podParaDefine.bOn_LEOAntPCO) + pcoAntEst.z);
						Matrix matPCO_RTN = m_matAxisBody2RTN * matPCO_Body; // 2015/03/09, 谷德峰
						POS3D pcoAnt;
						pcoAnt.x = matPCO_RTN.GetElement(0, 0);
						pcoAnt.y = matPCO_RTN.GetElement(1, 0);
						pcoAnt.z = matPCO_RTN.GetElement(2, 0);
						POS3D offsetJ2000 = GNSSBasicCorrectFunc::correctLeoAntPCO_J2000(pcoAnt, m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].pos, m_dataMixedGNSSlist[s_m].interpOrbitlist[nObsTime].vel);
						matPCO_J2000.SetElement(0, 0, offsetJ2000.x);
						matPCO_J2000.SetElement(1, 0, offsetJ2000.y);
						matPCO_J2000.SetElement(2, 0, offsetJ2000.z);

						Matrix matRTN2J2000(3, 3);
						matRTN2J2000.SetElement(0, 0, exRTN.x);
						matRTN2J2000.SetElement(1, 0, exRTN.y);
						matRTN2J2000.SetElement(2, 0, exRTN.z);
						matRTN2J2000.SetElement(0, 1, eyRTN.x);
						matRTN2J2000.SetElement(1, 1, eyRTN.y);
						matRTN2J2000.SetElement(2, 1, eyRTN.z);
						matRTN2J2000.SetElement(0, 2, ezRTN.x);
						matRTN2J2000.SetElement(1, 2, ezRTN.y);
						matRTN2J2000.SetElement(2, 2, ezRTN.z);
						Matrix matATT = matRTN2J2000 * m_matAxisBody2RTN;
						exBody.x = matATT.GetElement(0, 0);
						exBody.y = matATT.GetElement(1, 0);
						exBody.z = matATT.GetElement(2, 0);
						exBody = vectorNormal(exBody);
						eyBody.x = matATT.GetElement(0, 1);
						eyBody.y = matATT.GetElement(1, 1);
						eyBody.z = matATT.GetElement(2, 1);
						eyBody = vectorNormal(eyBody);
						ezBody.x = matATT.GetElement(0, 2);
						ezBody.y = matATT.GetElement(1, 2);
						ezBody.z = matATT.GetElement(2, 2);
						ezBody = vectorNormal(ezBody);
						// 清华球卫星GNSS天线并非安装在天顶方向
						Matrix matAnt = matRTN2J2000 * m_matAxisAnt2Body;
						exAnt.x = matAnt.GetElement(0, 0);
						exAnt.y = matAnt.GetElement(1, 0);
						exAnt.z = matAnt.GetElement(2, 0);
						exAnt = vectorNormal(exAnt);
						eyAnt.x = matAnt.GetElement(0, 1);
						eyAnt.y = matAnt.GetElement(1, 1);
						eyAnt.z = matAnt.GetElement(2, 1);
						eyAnt = vectorNormal(eyAnt);
						ezAnt.x = matAnt.GetElement(0, 2);
						ezAnt.y = matAnt.GetElement(1, 2);
						ezAnt.z = matAnt.GetElement(2, 2);
						ezAnt = vectorNormal(ezAnt);
						exyzBodyList[0][nObsTime] = exBody;
						exyzBodyList[1][nObsTime] = eyBody;
						exyzBodyList[2][nObsTime] = ezBody;

						if(m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].validIndex != -1)
						{// 仅对有效点进行更新
							int eyeableGPSCount = 0;
							int j = 0;
							for(Rinex2_1_EditedObsSatMap::iterator it = m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].editedObs.begin(); it != m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].editedObs.end(); ++it)
							{								
								int id_PRN = it->first;  // 第j颗可见GNSS卫星的卫星号						
								char szSatName[4];
								sprintf(szSatName, "%1c%02d", m_dataMixedGNSSlist[s_m].cSatSystem, id_PRN);
								szSatName[3] = '\0';
								PODEpochSatMap::iterator datum_j = m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].mapDatum.find(id_PRN);
								double delay = 0;
								SP3Datum sp3Datum;
								bool bEphemeris = true;				
								if(!sp3File_J2000.getEphemeris_PathDelay(m_dataMixedGNSSlist[s_m].editedObsEpochlist[s_i].t, posclk, szSatName, delay, sp3Datum))
									bEphemeris = false;
								double distance = sqrt(pow(posclk.x - sp3Datum.pos.x, 2) + pow(posclk.y - sp3Datum.pos.y, 2) + pow(posclk.z - sp3Datum.pos.z, 2)); 
								datum_j->second.vecLos_A.x = (posclk.x - sp3Datum.pos.x) / distance;
								datum_j->second.vecLos_A.y = (posclk.y - sp3Datum.pos.y) / distance;
								datum_j->second.vecLos_A.z = (posclk.z - sp3Datum.pos.z) / distance;
								datum_j->second.vecLos_A.clk  = 1.0;
								// 计算信号发射时间 T_Transmit, 参考信号真实接收时间T_Receive
								GPST t_Transmit = t_Receive - delay;
								//CLKDatum ASDatum;
								//if(!m_clkFile.getSatClock(t_Transmit, id_PRN, ASDatum, 3, m_dataMixedGNSSlist[s_m].cSatSystem)) // 获得 GPS 信号发射时间的卫星钟差改正
								//	bEphemeris = false;
								if(!bEphemeris)
								{// 星历不完整, 屏蔽该数据
									datum_j->second.obscorrected_value = 0.0;
									datum_j->second.ionosphereDelay = 0.0;
									datum_j->second.weightCode  = 0.0; 
									datum_j->second.weightPhase = 0.0;
								}
								else
								{
									// 20150312, 考虑姿态的影响, 谷德峰
									// 在天线系下更新计算 Azimuth 和 Elevation, 天线系坐标轴 XYZ 分别对应着 exBody -eyBody -ezBody
									POS3D vecLosJ2000 = sp3Datum.pos - posclk.getPos();
									POS3D vecLosXYZ;
									vecLosXYZ.x = vectorDot(vecLosJ2000, exAnt);
									vecLosXYZ.y = vectorDot(vecLosJ2000, eyAnt); 
									vecLosXYZ.z = vectorDot(vecLosJ2000, ezAnt);     // 天线系
									vecLosXYZ = vectorNormal(vecLosXYZ); // 视线矢量在天线系XYZ下的投影
									it->second.Elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;  // 高度角, 注 igs atx 文件中为天顶角 = 90 - Elevation
									it->second.Azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
									if(it->second.Azimuth < 0)
									{// 变换到[0, 360]
										it->second.Azimuth += 360.0;
									}
									datum_j->second.ionosphereDelay = 0.0;
									// 除电离层总的修正量
									eyeableGPSCount++;
									// 获得 J2000 系下的太阳相对地心的位置(千米)
									POS3D sunPos;                     // 太阳相对地心位置
									TDB t_TDB = TimeCoordConvert::GPST2TDB(t_Transmit); // 获得 TDB 时间--提供太阳历参考时间
									double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日
									double P[3];
									m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, P);
									sunPos.x = P[0] * 1000; 
									sunPos.y = P[1] * 1000; 
									sunPos.z = P[2] * 1000; 
									// 1 GPS卫星钟差改正
									//double correct_gpsclk = ASDatum.clkBias * SPEED_LIGHT; 
									double correct_gpsclk = 0.0; 
									// 2 GPS卫星相对论改正
									double correct_gpsrelativity = 0.0;
									if(m_podParaDefine.bOn_GPSRelativity)
										correct_gpsrelativity = (sp3Datum.pos.x * sp3Datum.vel.x + sp3Datum.pos.y * sp3Datum.vel.y + sp3Datum.pos.z * sp3Datum.vel.z) * (-2) / SPEED_LIGHT;
									//**********************************************************************************************************************************************
									// 4 LEO卫星天线先验 PCV 修正 (地面天线校准结果)
									double correct_leopcv_igs = 0.0;
									// 5 LEO卫星天线 PCO 修正
									double correct_leopco = 0.0;
									// 6 LEO卫星天线 PCV 修正 (在轨天线校准结果)
									double correct_leopcv = 0.0;
									// 7 相位 wind-up 修正
									double correct_phasewindup = 0.0;
									// 8 LEO卫星天线 CRV 修正 (在轨天线校准结果)
									double correct_leocrv = 0.0;
									// 20150312, 考虑姿态的影响, 谷德峰
									datum_j->second.obscorrected_value = correct_gpsclk
																	   + correct_gpsrelativity
																	   + correct_leorelativity
																	   + correct_leopcv_igs
																	   + correct_leopco
																	   - correct_leopcv           // 20141128, 保持与IGS的PCV符号定义一致
																	   - correct_phasewindup
																	   - distance
																	   - posclk.clk;
									datum_j->second.obscorrected_value_code = datum_j->second.obscorrected_value - correct_leocrv; // 20161228, 谷德峰添加
								}
								j++;
							}
							// 更新可视卫星个数统计结果
							m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].eyeableGPSCount = eyeableGPSCount;
							if(m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].eyeableGPSCount <= 1)  // 2013.4.18
								m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].validIndex = -1;
							else
							{
								s_index++;
								m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].validIndex = s_index;
							}
						}
					}
				}
				// 残差编辑
				if(flag_robust && bResEdit)
				{
					for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
					{
						// 伪码残差大小计算
						double rms_oc_code = 0;
						int count_valid_code = 0;
						for(size_t s_i = 0; s_i < m_dataMixedGNSSlist[s_m].P_IFEpochList.size(); s_i++)
						{
							for(int s_j = 0; s_j < int(m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList.size()); s_j++)
							{
								int nObsTime = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].nObsTime;
								BYTE id_Sat = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].id_Sat;
								PODEpochSatMap::iterator it = m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].mapDatum.find(id_Sat);
								m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].res = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].obs + it->second.obscorrected_value_code - it->second.ionosphereDelay; 
								if(m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].robustweight == 1.0 && it->second.weightCode != 0)
								{// 正常点
									count_valid_code++;
									rms_oc_code += pow(m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].res, 2);
								}
							}
						}
						rms_oc_code = sqrt(rms_oc_code / count_valid_code);
					}
					num_after_residual_edit ++;
				}
				// 计算最终残差
				if(flag_break)
				{// 计算最终残差
					for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
					{
						// 伪码残差
						for(size_t s_i = 0; s_i < m_dataMixedGNSSlist[s_m].P_IFEpochList.size(); s_i++)
						{
							for(int s_j = 0; s_j < int(m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList.size()); s_j++)
							{
								int nObsTime = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].nObsTime;
								BYTE id_Sat = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].id_Sat;
								PODEpochSatMap::iterator it = m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].mapDatum.find(id_Sat);
								m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].res = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].obs + it->second.obscorrected_value_code - it->second.ionosphereDelay; 
							}
						}
						// 伪码残差大小计算
						double rms_oc_code = 0;
						int count_valid_code = 0;
						m_dataMixedGNSSlist[s_m].ocResP_IFEpochList.clear();
						for(size_t s_i = 0; s_i < m_dataMixedGNSSlist[s_m].P_IFEpochList.size(); s_i++)
						{
							O_CResEpoch ocResP_IFEpoch;
							ocResP_IFEpoch.ocResSatList.clear();
							int nObsTime = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].nObsTime;
							int id_EditedObsEpoch = m_dataMixedGNSSlist[s_m].epochIdList[m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].nObsTime];
							ocResP_IFEpoch.t = m_dataMixedGNSSlist[s_m].editedObsEpochlist[id_EditedObsEpoch].t;
							for(size_t s_j = 0; s_j < m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList.size(); s_j++)
							{
								O_CResEpochElement ocResElement;
								ocResElement.id_Sat = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].id_Sat;
								ocResElement.res = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].res;
								ocResElement.robustweight = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].robustweight;
								// 仅添加正常点
								// 2009/10/31, 屏蔽显示原来的野值标记
								PODEpochSatMap::iterator it = m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].mapDatum.find(ocResElement.id_Sat);
								ocResElement.Elevation = m_dataMixedGNSSlist[s_m].editedObsEpochlist[id_EditedObsEpoch].editedObs[ocResElement.id_Sat].Elevation;
								ocResElement.Azimuth   = m_dataMixedGNSSlist[s_m].editedObsEpochlist[id_EditedObsEpoch].editedObs[ocResElement.id_Sat].Azimuth;
								if(ocResElement.robustweight == 1.0 && it->second.weightCode != 0)
								{
									ocResP_IFEpoch.ocResSatList.push_back(ocResElement);
									count_valid_code++;
									rms_oc_code += pow(ocResElement.res, 2);
								}
							}
							if(ocResP_IFEpoch.ocResSatList.size() > 0)
								m_dataMixedGNSSlist[s_m].ocResP_IFEpochList.push_back(ocResP_IFEpoch);
						}
						rms_oc_code = sqrt(rms_oc_code / count_valid_code);
						sprintf(info, "GNSS(%c) ZD-OC PIF = %6.3fm", m_dataMixedGNSSlist[s_m].cSatSystem, rms_oc_code); // 
						printf("%s\n", info);
						RuningInfoFile::Add(info);
					}
					break;
				}
				//---------------------------------------
				//	| n_cc   n_cx    n_cp  n_cs|     |nc|
				//	| n_xc   n_xx    n_xp  n_xs|     |nx|
				//	| n_pc   n_px    n_pp  n_ps|     |np|
				//---------------------------------------
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					m_dataMixedGNSSlist[s_m].n_cc.resize(m_dataMixedGNSSlist[s_m].count_clk); // 对角矩阵
					m_dataMixedGNSSlist[s_m].n_cc_inv.resize(m_dataMixedGNSSlist[s_m].count_clk); // 对角矩阵
					for(int s_i = 0; s_i < m_dataMixedGNSSlist[s_m].count_clk; s_i++)
						m_dataMixedGNSSlist[s_m].n_cc[s_i] = 0.0;
					m_dataMixedGNSSlist[s_m].n_cx.Init(m_dataMixedGNSSlist[s_m].count_clk, count_DynParameter);
					m_dataMixedGNSSlist[s_m].n_cp.Init(m_dataMixedGNSSlist[s_m].count_clk, count_EstPara_LEOAntPCO);
					m_dataMixedGNSSlist[s_m].nc.Init(m_dataMixedGNSSlist[s_m].count_clk, 1);
				}
				Matrix n_xx(count_DynParameter, count_DynParameter);
				Matrix n_pp(count_EstPara_LEOAntPCO, count_EstPara_LEOAntPCO);	
				Matrix n_px(count_EstPara_LEOAntPCO, count_DynParameter);
				Matrix nx(count_DynParameter, 1);
				Matrix np(count_EstPara_LEOAntPCO, 1);	
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					// 伪码设计矩阵
					vector<Matrix> listMatrix_code_H_xt;   // 记录每个时刻非差码观测值对位置的偏导数
					vector<Matrix> listMatrix_code_H_c;    // 记录每个时刻非差码观测值对钟差的偏导数
					vector<Matrix> listMatrix_code_H_pco;  // 记录每个时刻非差码观测值对天线偏移的偏导数
					vector<Matrix> listMatrix_code_y;      // 记录非差码观测值
					int  count_code  = 0;                  // 非差码观测数据的个数                           
					for(size_t s_i = 0; s_i < m_dataMixedGNSSlist[s_m].P_IFEpochList.size(); s_i++)
					{
						int nObsTime = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].nObsTime; 
						int validIndex = m_dataMixedGNSSlist[s_m].validSysIndexList[nObsTime]; // 钟差有效时刻
						if(validIndex == -1)
							continue;
						int count_code_i = int(m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList.size());
						count_code += count_code_i;
						Matrix Matrix_H_xt(count_code_i, 3);
						Matrix Matrix_H_c(count_code_i, 1);
						Matrix Matrix_H_pco(count_code_i, count_EstPara_LEOAntPCO);
						Matrix Matrix_y(count_code_i, 1);
						for(int s_j = 0; s_j < count_code_i; s_j++)
						{
							BYTE id_Sat = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].id_Sat;
							PODEpochSatMap::iterator it = m_dataMixedGNSSlist[s_m].mapDynEpochList[nObsTime].mapDatum.find(id_Sat);
							double w = it->second.weightCode * m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].robustweight; // 观测权值 = 经验权值 * 编辑信息
							// 位置偏导数
							Matrix_H_xt.SetElement(s_j, 0, it->second.vecLos_A.x * w);
							Matrix_H_xt.SetElement(s_j, 1, it->second.vecLos_A.y * w);
							Matrix_H_xt.SetElement(s_j, 2, it->second.vecLos_A.z * w);
							// 钟差偏导数
							Matrix_H_c.SetElement(s_j, 0, w);
							// 天线偏移估计偏导数
							if(OnEst_LEOAntPCO_x)
								Matrix_H_pco.SetElement(s_j, 0,                                     w * vectorDot(exyzBodyList[0][nObsTime], it->second.vecLos_A.getPos()));
							if(OnEst_LEOAntPCO_y)
								Matrix_H_pco.SetElement(s_j, OnEst_LEOAntPCO_x,                     w * vectorDot(exyzBodyList[1][nObsTime], it->second.vecLos_A.getPos()));
							if(OnEst_LEOAntPCO_z)
								Matrix_H_pco.SetElement(s_j, OnEst_LEOAntPCO_x + OnEst_LEOAntPCO_y, w * vectorDot(exyzBodyList[2][nObsTime], it->second.vecLos_A.getPos()));
							// 观测矢量													
							double o_c = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].obs + it->second.obscorrected_value_code - it->second.ionosphereDelay;
							m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList[s_j].res = o_c; // 记录伪码残差
							if(w != 0.0)
								Matrix_y.SetElement(s_j, 0, w * o_c);
							else
								Matrix_y.SetElement(s_j, 0, 0.0);// 防止 0 * INF
						}
						listMatrix_code_H_xt.push_back(Matrix_H_xt);
						listMatrix_code_H_c.push_back(Matrix_H_c);
						listMatrix_code_H_pco.push_back(Matrix_H_pco);
						listMatrix_code_y.push_back(Matrix_y);
					}
					Matrix Matrix_H_x(count_code, count_DynParameter); // 观测数据对动力学参数的偏导数
					Matrix Matrix_Y(count_code + count_EstPara_LEOAntPCO, 1);
					Matrix Matrix_H_pco(count_code + count_EstPara_LEOAntPCO, count_EstPara_LEOAntPCO); // 观测数据对天线偏移参数的偏导数
					// 根据设计矩阵的分块特点, 快速计算 H_x、H_c 和 n_cc
					int k_index = 0;
					for(size_t s_i = 0; s_i < m_dataMixedGNSSlist[s_m].P_IFEpochList.size(); s_i++)
					{// 伪码
						int nObsTime = m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].nObsTime; 
						int validIndex = m_dataMixedGNSSlist[s_m].validSysIndexList[nObsTime]; // 钟差有效时刻
						if(validIndex == -1)
							continue;
						double sum_clk = 0;
						for(int s_j = 0; s_j < int(m_dataMixedGNSSlist[s_m].P_IFEpochList[s_i].obsSatList.size()); s_j++)
						{
							sum_clk += pow(listMatrix_code_H_c[s_i].GetElement(s_j, 0), 2);
							for(int s_k = 0; s_k < 6; s_k++)
							{// 初始位置速度
								double sum_posvel = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, s_k) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
												  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, s_k) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
												  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, s_k) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
								Matrix_H_x.SetElement(k_index, s_k, sum_posvel);
							}
							int beginPara = 6;
							if(dynamicDatum.bOn_SolarPressureAcc && (dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA|| dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA_AM|| dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_MACRO))
							{// 太阳光压
								for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
								{
									double sum_solar = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k ) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k ) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k ) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k, sum_solar);
								}
								beginPara += count_SolarPressureParaList;
							}
							// 2022.04.04，韦春博
							else if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
							{
								for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
								{
									// A_D0
									double sum_A_D0  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 9 + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 9 + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 9 + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 9 + 0, sum_A_D0);
									// A_DC
									double sum_A_DC  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 9 + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 9 + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 9 + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 9 + 1, sum_A_DC);
									// A_DS
									double sum_A_DS  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 9 + 2) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 +m_dataMixedGNSSlist[s_m]. interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 9 + 2) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 +m_dataMixedGNSSlist[s_m]. interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 9 + 2) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 9 + 2, sum_A_DS);
									// A_Y0
									double sum_A_Y0  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 9 + 3) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 9 + 3) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 9 + 3) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 9 + 3, sum_A_Y0);
									// A_YC
									double sum_A_YC  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 9 + 4) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 9 + 4) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 9 + 4) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 9 + 4, sum_A_YC);
									// A_YS
									double sum_A_YS  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 9 + 5) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 9 + 5) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 9 + 5) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 9 + 5, sum_A_YS);
									// A_X0
									double sum_A_X0  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 9 + 6) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 9 + 6) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 9 + 6) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 9 + 6, sum_A_X0);
									// A_XC
									double sum_A_XC  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 9 + 7) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 9 + 7) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 9 + 7) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 9 + 7, sum_A_XC);
									// A_XS
									double sum_A_XS  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 9 + 8) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 9 + 8) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 9 + 8) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 9 + 8, sum_A_XS);
								}
								beginPara += 9 * count_SolarPressureParaList;
							}
							else if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_5PARA)
							{
								for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
								{
									// A_D0
									double sum_A_D0  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 5 + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 5 + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 5 + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 5 + 0, sum_A_D0);
									// A_Y0
									double sum_A_Y0  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 5 + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 5 + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 5 + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 5 + 1, sum_A_Y0);
									// A_X0
									double sum_A_X0  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 5 + 2) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 5 + 2) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 5 + 2) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 5 + 2, sum_A_X0);
									// A_XC
									double sum_A_XC  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 5 + 3) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 5 + 3) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 5 + 3) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 5 + 3, sum_A_XC);
									// A_XS
									double sum_A_XS  = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara +  s_k * 5 + 4) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara +  s_k * 5 + 4) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara +  s_k * 5 + 4) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 5 + 4, sum_A_XS);
								}
								beginPara += 5 * count_SolarPressureParaList;
							}
							if(dynamicDatum.bOn_EmpiricalForceAcc && dynamicDatum.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
							{// 经验力
								index_EmpiricalParaBegin = beginPara;
								int count_sub =  + int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
												 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2  
												 + int(dynamicDatum.bOn_EmpiricalForceAcc_N) * 2; 
								for(int s_k = 0; s_k < int(dynamicDatum.empiricalForceParaList.size()); s_k++)
								{
									int i_sub = 0;
									if(dynamicDatum.bOn_EmpiricalForceAcc_R)
									{
										double sum_cr = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k * count_sub + i_sub + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k * count_sub + i_sub + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k * count_sub + i_sub + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
										Matrix_H_x.SetElement(k_index, beginPara + s_k * count_sub + i_sub + 0, sum_cr);
										double sum_sr = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k * count_sub + i_sub + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k * count_sub + i_sub + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k * count_sub + i_sub + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
										Matrix_H_x.SetElement(k_index, beginPara + s_k * count_sub + i_sub + 1, sum_sr);
									}
									i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2;
									if(dynamicDatum.bOn_EmpiricalForceAcc_T)
									{
										double sum_ct = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k * count_sub + i_sub + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k * count_sub + i_sub + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k * count_sub + i_sub + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
										Matrix_H_x.SetElement(k_index, beginPara + s_k * count_sub + i_sub + 0, sum_ct);
										double sum_st = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k * count_sub + i_sub + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k * count_sub + i_sub + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k * count_sub + i_sub + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
										Matrix_H_x.SetElement(k_index, beginPara + s_k * count_sub + i_sub + 1, sum_st);
										
									}
									i_sub = int(dynamicDatum.bOn_EmpiricalForceAcc_R) * 2 + int(dynamicDatum.bOn_EmpiricalForceAcc_T) * 2;
									if(dynamicDatum.bOn_EmpiricalForceAcc_N)
									{
										double sum_cn = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k * count_sub + i_sub + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k * count_sub + i_sub + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k * count_sub + i_sub + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
										Matrix_H_x.SetElement(k_index, beginPara + s_k * count_sub + i_sub + 0, sum_cn);
										double sum_sn = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k * count_sub + i_sub + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k * count_sub + i_sub + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													  + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k * count_sub + i_sub + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
										Matrix_H_x.SetElement(k_index, beginPara + s_k * count_sub + i_sub + 1, sum_sn);
										
									}
								}
								beginPara += count_sub * count_EmpiricalForceParaList;
							}
							if(dynamicDatum.bOn_ManeuverForceAcc && dynamicDatum.bOnEst_Maneuver)
							{// 机动力, 2013/10/28, 谷德峰
								index_ManeuverParaBegin = beginPara;	// 机动力参数起始位置，2014/5/10，鞠 冰
								for(int s_k = 0; s_k < int(dynamicDatum.maneuverForceParaList.size()); s_k++)
								{
									double sum_MA0_R = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k * 3 + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k * 3 + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k * 3 + 0) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 3 + 0, sum_MA0_R);
									double sum_MA0_T = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k * 3 + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k * 3 + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k * 3 + 1) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 3 + 1, sum_MA0_T);
									double sum_MA0_N = m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(0, beginPara + s_k * 3 + 2) * listMatrix_code_H_xt[s_i].GetElement(s_j, 0) 
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(1, beginPara + s_k * 3 + 2) * listMatrix_code_H_xt[s_i].GetElement(s_j, 1)
													 + m_dataMixedGNSSlist[s_m].interpRtPartiallist[nObsTime].GetElement(2, beginPara + s_k * 3 + 2) * listMatrix_code_H_xt[s_i].GetElement(s_j, 2);
									Matrix_H_x.SetElement(k_index, beginPara + s_k * 3 + 2, sum_MA0_N);
								}
								beginPara += 3 * count_ManeuverForceParaList;
							}
							for(int s_k = 0; s_k < count_EstPara_LEOAntPCO; s_k++)
								Matrix_H_pco.SetElement(k_index, s_k, listMatrix_code_H_pco[s_i].GetElement(s_j, s_k));
							Matrix_Y.SetElement(k_index, 0, listMatrix_code_y[s_i].GetElement(s_j, 0));
							k_index++;
						}
						// 当前系统钟差
						m_dataMixedGNSSlist[s_m].n_cc[validIndex] += sum_clk;
					}
					// nx, n_xx
					for(int s_i = 0; s_i < Matrix_H_x.GetNumColumns(); s_i++)
					{   
						// nx = H_x' * y
						double sum_y = 0;
						for(int s_j = 0; s_j < Matrix_H_x.GetNumRows(); s_j++)	
							sum_y += Matrix_H_x.GetElement(s_j, s_i) * Matrix_Y.GetElement(s_j, 0); // 列元素相乘
						nx.SetElement(s_i, 0, nx.GetElement(s_i, 0) + sum_y); // 2014/12/11, 混合系统调整为累加
						// n_xx = H_x' * H_x
						for(int s_j = s_i; s_j < Matrix_H_x.GetNumColumns(); s_j++)
						{
							double sum_x = 0;
							for(int s_k = 0; s_k < Matrix_H_x.GetNumRows(); s_k++)
								sum_x += Matrix_H_x.GetElement(s_k, s_i) * Matrix_H_x.GetElement(s_k, s_j);
							n_xx.SetElement(s_i, s_j, n_xx.GetElement(s_i, s_j) + sum_x); // 2014/12/11, 混合系统调整为累加
							n_xx.SetElement(s_j, s_i, n_xx.GetElement(s_i, s_j));         // 2014/12/25调整
						}
					}
					// np,n_pp
					for(int s_i = 0; s_i < Matrix_H_pco.GetNumColumns(); s_i++)
					{
						// np = H_p' * y
						double sum_y = 0;
						for(int s_j = 0; s_j < Matrix_H_pco.GetNumRows(); s_j++)	
							sum_y += Matrix_H_pco.GetElement(s_j, s_i) * Matrix_Y.GetElement(s_j, 0); // 列元素相乘
						np.SetElement(s_i, 0, np.GetElement(s_i, 0) + sum_y); // 2014/12/11, 混合系统调整为累加
						// n_pp = H_p' * H_p
						for(int s_j = s_i; s_j < Matrix_H_pco.GetNumColumns(); s_j++)
						{
							double sum_p = 0;
							for(int s_k = 0; s_k < Matrix_H_pco.GetNumRows(); s_k++)
								sum_p += Matrix_H_pco.GetElement(s_k, s_i) * Matrix_H_pco.GetElement(s_k, s_j);
							n_pp.SetElement(s_i, s_j, n_pp.GetElement(s_i, s_j) + sum_p); // 2014/12/11, 混合系统调整为累加
							n_pp.SetElement(s_j, s_i, n_pp.GetElement(s_i, s_j));         // 2014/12/25调整
						}						
					}
					// n_px
					for(int s_i = 0; s_i < Matrix_H_pco.GetNumColumns(); s_i++)
					{   
						//n_px = H_p' * H_x
						for(int s_j = 0; s_j < Matrix_H_x.GetNumColumns(); s_j++)
						{
							double sum_px = 0;
							for(int s_k = 0; s_k < Matrix_H_x.GetNumRows(); s_k++) // 参考 Matrix_H_x 行数, 伪方程部分对 n_px 没有贡献
								sum_px += Matrix_H_pco.GetElement(s_k, s_i) * Matrix_H_x.GetElement(s_k, s_j);
							n_px.SetElement(s_i, s_j, n_px.GetElement(s_i, s_j) + sum_px); // 2014/12/11, 混合系统调整为累加
						}
					}
					// nc, n_cx,, n_cp
					k_index = 0;
					for(int s_i = 0; s_i < int(listMatrix_code_H_c.size()); s_i++)
					{// nc   = H_c' * y
						for(int s_j = 0; s_j < listMatrix_code_H_c[s_i].GetNumRows(); s_j++)
						{
							for(int s_k = 0; s_k < count_DynParameter; s_k++)
								m_dataMixedGNSSlist[s_m].n_cx.SetElement(s_i, s_k, m_dataMixedGNSSlist[s_m].n_cx.GetElement(s_i, s_k) + listMatrix_code_H_c[s_i].GetElement(s_j, 0) * Matrix_H_x.GetElement(k_index, s_k));
							for(int s_k = 0; s_k < count_EstPara_LEOAntPCO; s_k++)
								m_dataMixedGNSSlist[s_m].n_cp.SetElement(s_i, s_k, m_dataMixedGNSSlist[s_m].n_cp.GetElement(s_i, s_k) + listMatrix_code_H_c[s_i].GetElement(s_j, 0) * Matrix_H_pco.GetElement(k_index, s_k));
							m_dataMixedGNSSlist[s_m].nc.SetElement(s_i, 0, m_dataMixedGNSSlist[s_m].nc.GetElement(s_i, 0) + listMatrix_code_H_c[s_i].GetElement(s_j, 0) * listMatrix_code_y[s_i].GetElement(s_j, 0));
							k_index ++;
						}
					}
				}
				// 添加伪方程-光压参数
				if(m_podParaDefine.bOn_CstSolarpressureP)
				{
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
					{
						//double weight_solar = 1.0E+12;	
						double weight_solar = 1.0E+9;
						if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
						{
							if(! m_podParaDefine.on_SRP9_D0)
							{// △D0[0]  = -(D0*) + ε									
								int index_D0 = 6 + 9 * (int)s_k + 0;
								n_xx.SetElement(index_D0, index_D0,  n_xx.GetElement(index_D0, index_D0) + weight_solar * weight_solar);
								nx.SetElement(index_D0, 0,           nx.GetElement(index_D0, 0)          - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].D0);
							}
							if(! m_podParaDefine.on_SRP9_DC1)
							{// △DCOS[1]  = -(DCOS*) + ε								
								int index_DCOS = 6 + 9 * (int)s_k + 1;
								n_xx.SetElement(index_DCOS, index_DCOS,  n_xx.GetElement(index_DCOS, index_DCOS) + weight_solar * weight_solar);
								nx.SetElement(index_DCOS, 0,             nx.GetElement(index_DCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].DC1);
							}
							if(! m_podParaDefine.on_SRP9_DS1)
							{// △DSIN[2]  = -(DSIN*) + ε								
								int index_DSIN = 6 + 9 * (int)s_k + 2;
								n_xx.SetElement(index_DSIN, index_DSIN,  n_xx.GetElement(index_DSIN, index_DSIN) + weight_solar * weight_solar);
								nx.SetElement(index_DSIN, 0,             nx.GetElement(index_DSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].DS1);
							}
							if(! m_podParaDefine.on_SRP9_Y0)
							{// △Y0[3]  = -(Y0*) + ε								
								int index_Y0 = 6 + 9 * (int)s_k + 3;
								n_xx.SetElement(index_Y0, index_Y0,  n_xx.GetElement(index_Y0, index_Y0) + weight_solar * weight_solar);
								nx.SetElement(index_Y0, 0,           nx.GetElement(index_Y0, 0)          - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].Y0);
							}
							if(! m_podParaDefine.on_SRP9_YC1)
							{// △YCOS[4]  = -(YCOS*) + ε								
								int index_YCOS = 6 + 9 * (int)s_k + 4;
								n_xx.SetElement(index_YCOS, index_YCOS,  n_xx.GetElement(index_YCOS, index_YCOS) + weight_solar * weight_solar);
								nx.SetElement(index_YCOS, 0,             nx.GetElement(index_YCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].YC1);
							}
							if(! m_podParaDefine.on_SRP9_YS1)
							{// △YSIN[5]  = -(YSIN*) + ε								
								int index_YSIN = 6 + 9 * (int)s_k + 5;
								n_xx.SetElement(index_YSIN, index_YSIN,  n_xx.GetElement(index_YSIN, index_YSIN) + weight_solar * weight_solar);
								nx.SetElement(index_YSIN, 0,             nx.GetElement(index_YSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].YS1);
							}
							if(! m_podParaDefine.on_SRP9_B0)
							{// △B0[6]  = -(B0*) + ε								
								int index_B0 = 6 + 9 * (int)s_k + 6;
								n_xx.SetElement(index_B0, index_B0,  n_xx.GetElement(index_B0, index_B0) + weight_solar * weight_solar);
								nx.SetElement(index_B0, 0,           nx.GetElement(index_B0, 0)          - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].B0);
							}
							if(! m_podParaDefine.on_SRP9_BC1)
							{// △BCOS[7]  = -(BCOS*) + ε								
								int index_BCOS = 6 + 9 * (int)s_k + 7;
								n_xx.SetElement(index_BCOS, index_BCOS,  n_xx.GetElement(index_BCOS, index_BCOS) + weight_solar * weight_solar);
								nx.SetElement(index_BCOS, 0,             nx.GetElement(index_BCOS, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].BC1);
							}
							if(! m_podParaDefine.on_SRP9_BS1)
							{// △BSIN[8]  = -(BSIN*) + ε								
								int index_BSIN = 6 + 9 * (int)s_k + 8;
								n_xx.SetElement(index_BSIN, index_BSIN,  n_xx.GetElement(index_BSIN, index_BSIN) + weight_solar * weight_solar);
								nx.SetElement(index_BSIN, 0,             nx.GetElement(index_BSIN, 0)            - weight_solar * weight_solar * dynamicDatum.solarPressureParaList[s_k].BS1);
							}
						}
					}
				}
				// 重新进行设计矩阵整合, 将动力学参数、模糊度参数、天线偏移估计参数合并
				/*
					| n_cc | n_cx   n_cp|     |nc|
					------------------------------
					| n_xc | n_xx   n_xp|     |nx|
					|      |                      
					| n_pc | n_px   n_pp|     |np|
				*/
				Matrix N_XX(count_DynParameter + count_EstPara_LEOAntPCO, count_DynParameter + count_EstPara_LEOAntPCO);
				int count_clk_all = 0;
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
					count_clk_all += m_dataMixedGNSSlist[s_m].count_clk;
				//sprintf(info, "count_clk_all %8d", count_clk_all); // 
				//printf("%s\n", info);
				//RuningInfoFile::Add(info);
				Matrix N_CX(count_clk_all, count_DynParameter + count_EstPara_LEOAntPCO);
				Matrix NX(count_DynParameter + count_EstPara_LEOAntPCO, 1);
				int count_clk_i = 0;
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					for(int s_i = count_clk_i; s_i < count_clk_i + m_dataMixedGNSSlist[s_m].count_clk; s_i++)
					{
						for(int s_j = 0; s_j < count_DynParameter; s_j++)
						    N_CX.SetElement(s_i, s_j, m_dataMixedGNSSlist[s_m].n_cx.GetElement(s_i - count_clk_i, s_j));
						for(int s_j = 0; s_j < count_EstPara_LEOAntPCO; s_j++)
							N_CX.SetElement(s_i, s_j + count_DynParameter, m_dataMixedGNSSlist[s_m].n_cp.GetElement(s_i - count_clk_i, s_j));
					}					
					count_clk_i += m_dataMixedGNSSlist[s_m].count_clk;
				}
				for(int s_i = 0; s_i < count_DynParameter; s_i++)
				{
					NX.SetElement(s_i, 0, nx.GetElement(s_i, 0));
					for(int s_j = 0; s_j < count_DynParameter; s_j++)
						N_XX.SetElement(s_i, s_j, n_xx.GetElement(s_i, s_j));
					for(int s_j = 0; s_j < count_EstPara_LEOAntPCO; s_j++)
						N_XX.SetElement(s_i, s_j + count_DynParameter, n_px.GetElement(s_j, s_i));
				}
				for(int s_i = 0; s_i < count_EstPara_LEOAntPCO; s_i++)
				{
					NX.SetElement(s_i + count_DynParameter, 0, np.GetElement(s_i, 0));
					for(int s_j = 0; s_j < count_DynParameter; s_j++)
						N_XX.SetElement(s_i + count_DynParameter, s_j, n_px.GetElement(s_i, s_j));
					for(int s_j = 0; s_j < count_EstPara_LEOAntPCO; s_j++)
						N_XX.SetElement(s_i + count_DynParameter, s_j + count_DynParameter, n_pp.GetElement(s_i, s_j));
				}
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					for(int s_i = 0; s_i < m_dataMixedGNSSlist[s_m].count_clk; s_i++)
						m_dataMixedGNSSlist[s_m].n_cc_inv[s_i] = 1.0 / m_dataMixedGNSSlist[s_m].n_cc[s_i];
				}
				// 开始轨道改进
				Matrix n_xc_cc_inv_nc(count_DynParameter + count_EstPara_LEOAntPCO, 1); // 2015/01/05, 避免高维矩阵 n_xc_cc_inv 的定义, 谷德峰
				Matrix n_xc_cc_inv_cx(count_DynParameter + count_EstPara_LEOAntPCO, count_DynParameter + count_EstPara_LEOAntPCO);
				count_clk_i = 0;
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					for(int s_i = 0; s_i < count_DynParameter + count_EstPara_LEOAntPCO; s_i++)
					{
						for(int s_j = count_clk_i; s_j < count_clk_i + m_dataMixedGNSSlist[s_m].count_clk; s_j++)
						{
							n_xc_cc_inv_nc.SetElement(s_i, 0, n_xc_cc_inv_nc.GetElement(s_i, 0) + N_CX.GetElement(s_j, s_i) * m_dataMixedGNSSlist[s_m].n_cc_inv[s_j - count_clk_i] * m_dataMixedGNSSlist[s_m].nc.GetElement(s_j - count_clk_i, 0)); 
						}
						for(int s_j = s_i; s_j < count_DynParameter + count_EstPara_LEOAntPCO; s_j++ )
						{
							double sum_k = 0;
							for(int s_k = count_clk_i; s_k < count_clk_i + m_dataMixedGNSSlist[s_m].count_clk; s_k ++)
								sum_k += N_CX.GetElement(s_k, s_j) * N_CX.GetElement(s_k, s_i) * m_dataMixedGNSSlist[s_m].n_cc_inv[s_k - count_clk_i];
							//if(s_m == 0)
							//{
							//	n_xc_cc_inv_cx.SetElement(s_i, s_j, sum_k);
							//	n_xc_cc_inv_cx.SetElement(s_j, s_i, sum_k);
							//}
							//else
							//{
								n_xc_cc_inv_cx.SetElement(s_i, s_j, n_xc_cc_inv_cx.GetElement(s_i, s_j) + sum_k);
								n_xc_cc_inv_cx.SetElement(s_j, s_i, n_xc_cc_inv_cx.GetElement(s_i, s_j));
							//}
						}
					}
					count_clk_i += m_dataMixedGNSSlist[s_m].count_clk;
				}
				// 计算相关系数矩阵
				Matrix matQ_xx;
				matQ_xx = (N_XX - n_xc_cc_inv_cx).Inv_Ssgj();
				Matrix matdx = matQ_xx * (NX - n_xc_cc_inv_nc); // 2015/01/05, 避免高维矩阵 n_xc_cc_inv 的定义, 谷德峰
				Matrix NC(count_clk_all,1); 
				count_clk_i = 0;
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					for(size_t s_i = count_clk_i; s_i < count_clk_i + m_dataMixedGNSSlist[s_m].count_clk; s_i ++)
						NC.SetElement(s_i, 0, m_dataMixedGNSSlist[s_m].nc.GetElement(s_i- count_clk_i, 0));
					count_clk_i += m_dataMixedGNSSlist[s_m].count_clk;
				}
				Matrix matdc(count_clk_all, 1);
				matdc = NC - N_CX * matdx;
				count_clk_i = 0;
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					for(size_t s_i = count_clk_i; s_i < count_clk_i + m_dataMixedGNSSlist[s_m].count_clk; s_i ++)
						matdc.SetElement(s_i, 0, m_dataMixedGNSSlist[s_m].n_cc_inv[s_i - count_clk_i] * matdc.GetElement(s_i, 0));
					count_clk_i += m_dataMixedGNSSlist[s_m].count_clk;
				}
				// 计算轨道改进量
				dynamicDatum.X0.x  += matdx.GetElement(0,0);
				dynamicDatum.X0.y  += matdx.GetElement(1,0);
				dynamicDatum.X0.z  += matdx.GetElement(2,0);
				dynamicDatum.X0.vx += matdx.GetElement(3,0);
				dynamicDatum.X0.vy += matdx.GetElement(4,0);
				dynamicDatum.X0.vz += matdx.GetElement(5,0);
				int beginPara = 6;
				if(dynamicDatum.bOn_SolarPressureAcc && (dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA|| dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA_AM|| dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_MACRO))
				{// 太阳光压
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
						dynamicDatum.solarPressureParaList[s_k].Cr += matdx.GetElement(beginPara + s_k, 0);
					beginPara += count_SolarPressureParaList;
				}
				// 2022.04.04，韦春博
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
					}
					beginPara += 9 * count_SolarPressureParaList;
				}
				else if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_5PARA)
				{
					for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
					{
						dynamicDatum.solarPressureParaList[s_k].D0  += matdx.GetElement(beginPara + s_k * 9 + 0, 0);
						dynamicDatum.solarPressureParaList[s_k].Y0  += matdx.GetElement(beginPara + s_k * 9 + 1, 0);
						dynamicDatum.solarPressureParaList[s_k].B0  += matdx.GetElement(beginPara + s_k * 9 + 2, 0);
						dynamicDatum.solarPressureParaList[s_k].BC1 += matdx.GetElement(beginPara + s_k * 9 + 3, 0);
						dynamicDatum.solarPressureParaList[s_k].BS1 += matdx.GetElement(beginPara + s_k * 9 + 4, 0);
					}
					beginPara += 5 * count_SolarPressureParaList;
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
				if(dynamicDatum.bOn_ManeuverForceAcc && dynamicDatum.bOnEst_Maneuver)
				{// 机动力, 2013/10/28, 谷德峰
					for(int s_k = 0; s_k < int(dynamicDatum.maneuverForceParaList.size()); s_k++)
					{
						dynamicDatum.maneuverForceParaList[s_k].a0_R += matdx.GetElement(beginPara + s_k * 3 + 0, 0);
						dynamicDatum.maneuverForceParaList[s_k].a0_T += matdx.GetElement(beginPara + s_k * 3 + 1, 0);
						dynamicDatum.maneuverForceParaList[s_k].a0_N += matdx.GetElement(beginPara + s_k * 3 + 2, 0);
					}
					beginPara += 3 * count_ManeuverForceParaList;
				}
				// 计算天线相位中心改进量
				if(OnEst_LEOAntPCO_x)
					pcoAntEst.x += matdx.GetElement(count_DynParameter + 0, 0);
				if(OnEst_LEOAntPCO_y)
					pcoAntEst.y += matdx.GetElement(count_DynParameter + OnEst_LEOAntPCO_x, 0);
				if(OnEst_LEOAntPCO_z)
					pcoAntEst.z += matdx.GetElement(count_DynParameter + OnEst_LEOAntPCO_x + OnEst_LEOAntPCO_y, 0);
				m_pcoAntEst = pcoAntEst;
				// 计算钟差改进量
				count_clk_i = 0;
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					for(size_t s_i = count_clk_i; s_i < count_clk_i + m_dataMixedGNSSlist[s_m].count_clk; s_i ++)
						m_dataMixedGNSSlist[s_m].leoClockList[m_dataMixedGNSSlist[s_m].validSysObsTimeList[s_i - count_clk_i]] += matdc.GetElement(s_i, 0);
					count_clk_i += m_dataMixedGNSSlist[s_m].count_clk;
				}
				// 记录轨道改进结果
				pFitFile = fopen(dynamicDatumFilePath, "w+");
				fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
				fprintf(pFitFile, "%3d. PCO  X    (cm)     %20.4f%10.4f%20.4f\n", 1,m_pcoAnt.x * 100.0,      m_pcoAntEst.x * 100.0,                        (m_pcoAnt.x + m_pcoAntEst.x) * 100.0);
				fprintf(pFitFile, "%3d. PCO  Y    (cm)     %20.4f%10.4f%20.4f\n", 2,m_pcoAnt.y * 100.0,      m_pcoAntEst.y * 100.0,                        (m_pcoAnt.y + m_pcoAntEst.y) * 100.0);
				fprintf(pFitFile, "%3d. PCO  Z    (cm)     %20.4f%10.4f%20.4f\n", 3,m_pcoAnt.z * 100.0,      m_pcoAntEst.z * 100.0,                        (m_pcoAnt.z + m_pcoAntEst.z) * 100.0);
				fprintf(pFitFile, "%3d.      X    (m)      %20.4f%10.4f%20.4f\n", 4,dynamicDatum_Init.X0.x,  dynamicDatum.X0.x  - dynamicDatum_Init.X0.x,  dynamicDatum.X0.x);
				fprintf(pFitFile, "%3d.      Y    (m)      %20.4f%10.4f%20.4f\n", 5,dynamicDatum_Init.X0.y,  dynamicDatum.X0.y  - dynamicDatum_Init.X0.y,  dynamicDatum.X0.y);
				fprintf(pFitFile, "%3d.      Z    (m)      %20.4f%10.4f%20.4f\n", 6,dynamicDatum_Init.X0.z,  dynamicDatum.X0.z  - dynamicDatum_Init.X0.z,  dynamicDatum.X0.z);
				fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f%10.4f%20.4f\n", 7,dynamicDatum_Init.X0.vx, dynamicDatum.X0.vx - dynamicDatum_Init.X0.vx, dynamicDatum.X0.vx);
				fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f%10.4f%20.4f\n", 8,dynamicDatum_Init.X0.vy, dynamicDatum.X0.vy - dynamicDatum_Init.X0.vy, dynamicDatum.X0.vy);
				fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f%10.4f%20.4f\n", 9,dynamicDatum_Init.X0.vz, dynamicDatum.X0.vz - dynamicDatum_Init.X0.vz, dynamicDatum.X0.vz);
				k_Parameter = 9;
				if(dynamicDatum.bOn_SolarPressureAcc && (dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA|| dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA_AM|| dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_MACRO))
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
				if(dynamicDatum.bOn_SolarPressureAcc && dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_5PARA)
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
						fprintf(pFitFile, "%3d. %2d   Y0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynamicDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].Y0 * 1.0E+7 - dynamicDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7, 
																						   dynamicDatum.solarPressureParaList[s_i].Y0 * 1.0E+7);
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
				if(dynamicDatum.bOn_ManeuverForceAcc && dynamicDatum.bOnEst_Maneuver)
				{// 机动力, 2013/10/28, 谷德峰
					for(size_t s_i = 0; s_i < dynamicDatum.maneuverForceParaList.size(); s_i++)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   MA0R (1.0E-4) %20.4f%10.4f%20.4f\n", k_Parameter,
																						   s_i+1,
																						   dynamicDatum_Init.maneuverForceParaList[s_i].a0_R * 1.0E+4,
																						   dynamicDatum.maneuverForceParaList[s_i].a0_R * 1.0E+4 - dynamicDatum_Init.maneuverForceParaList[s_i].a0_R * 1.0E+4,
																						   dynamicDatum.maneuverForceParaList[s_i].a0_R * 1.0E+4);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   MA0T (1.0E-4) %20.4f%10.4f%20.4f\n", k_Parameter,
																						   s_i+1,
																						   dynamicDatum_Init.maneuverForceParaList[s_i].a0_T * 1.0E+4,
																						   dynamicDatum.maneuverForceParaList[s_i].a0_T * 1.0E+4 - dynamicDatum_Init.maneuverForceParaList[s_i].a0_T * 1.0E+4,
																						   dynamicDatum.maneuverForceParaList[s_i].a0_T * 1.0E+4);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   MA0N (1.0E-4) %20.4f%10.4f%20.4f\n", k_Parameter,
																						   s_i+1,
																						   dynamicDatum_Init.maneuverForceParaList[s_i].a0_N * 1.0E+4,
																						   dynamicDatum.maneuverForceParaList[s_i].a0_N * 1.0E+4 - dynamicDatum_Init.maneuverForceParaList[s_i].a0_N * 1.0E+4,
																						   dynamicDatum.maneuverForceParaList[s_i].a0_N * 1.0E+4);
					}
				}				
				fclose(pFitFile);
				// 判断收敛条件
				double max_adjust_pos = 0;
				for(int i = 0; i < 3; i++)
					max_adjust_pos = max(max_adjust_pos, fabs(matdx.GetElement(i, 0)));
				sprintf(info, "第%d次 max_AdjustPos = %20.8lf", k, max_adjust_pos);
				RuningInfoFile::Add(info);
				// 2014/10/18, 谷德峰, P1 + L1
				//double threshold_adjust_pos = 1.0E-3;
				if(max_adjust_pos <= m_podParaDefine.threshold_max_adjustpos  
					|| k >= m_podParaDefine.max_OrbitIterativeNum || num_after_residual_edit > 0) 
				{
					//if(flag_robust == false && num_after_residual_edit == 0 && bResEdit)
					// 首次进行残差编辑, 2014/5/18, 鞠 冰
					if(flag_robust == false && bResEdit)
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
			// 更新轨道解, 2009/09/21
			for(size_t s_i = 0; s_i < count_MixedEpoch; s_i++)
			{
				TimePosVel posvel = m_dataMixedGNSSlist[0].interpOrbitlist[s_i];
				m_editedMixedObsFile.m_data[s_i].clock = m_dataMixedGNSSlist[0].leoClockList[s_i];
				double x_ecf[6];
				double x_j2000[6];
				x_j2000[0] = posvel.pos.x;  
				x_j2000[1] = posvel.pos.y;  
				x_j2000[2] = posvel.pos.z;
				x_j2000[3] = posvel.vel.x; 
				x_j2000[4] = posvel.vel.y; 
				x_j2000[5] = posvel.vel.z;
				m_TimeCoordConvert.J2000_ECEF(TimeCoordConvert::TDT2GPST(posvel.t), x_j2000, x_ecf);
				m_editedMixedObsFile.m_data[s_i].pos.x = x_ecf[0]; 
				m_editedMixedObsFile.m_data[s_i].pos.y = x_ecf[1]; 
				m_editedMixedObsFile.m_data[s_i].pos.z = x_ecf[2];
				m_editedMixedObsFile.m_data[s_i].vel.x = x_ecf[3]; 
				m_editedMixedObsFile.m_data[s_i].vel.y = x_ecf[4]; 
				m_editedMixedObsFile.m_data[s_i].vel.z = x_ecf[5];
				if(m_dataMixedGNSSlist[0].validSysIndexList[s_i] == -1)				
					m_editedMixedObsFile.m_data[s_i].byRAIMFlag = 0;			
				else
					m_editedMixedObsFile.m_data[s_i].byRAIMFlag = 2;
				m_editedMixedObsFile.m_data[s_i].pdop = 0.0;
			}		
			if(!bForecast) // 不进行轨道预报
				return result;
			// 进行轨道预报
			TDT t0_tdt = TimeCoordConvert::GPST2TDT(t0_forecast);
			TDT t1_tdt = TimeCoordConvert::GPST2TDT(t1_forecast);
			if(result)
			{
				vector<TimePosVel> orbitlist_ac;
				vector<Matrix> matRtPartiallist_ac;
				// 倒向积分, 积分区间 [para.T0, T_End   + h * 4], 为保证插值精度向两端进行扩展
				vector<TimePosVel> backwardOrbitlist_ac; 
			    vector<TimePosVel> forwardOrbitlist_ac; 
                double h = m_stepAdamsCowell; // 20150308, 谷德峰, 轨道积分步长修改为10.0秒
				//dynamicDatum.bOn_ManeuverForceAcc = true;	// 测试Admas-Cowell + Runge-Kutta积分器误差，2014-6-15，鞠 冰
				if(dynamicDatum.bOn_ManeuverForceAcc && dynamicDatum.bOnEst_Maneuver)
				{// 机动力, 2013/10/28, 谷德峰
					printf("机动力参数的值: \n");
					for(size_t s_i = 0; s_i < dynamicDatum.maneuverForceParaList.size(); s_i++)
					{
						printf("a0_R = %20.4f\n", dynamicDatum.maneuverForceParaList[s_i].a0_R * 1.0E+4);
						printf("a0_T = %20.4f\n", dynamicDatum.maneuverForceParaList[s_i].a0_T * 1.0E+4);
						printf("a0_N = %20.4f\n", dynamicDatum.maneuverForceParaList[s_i].a0_N * 1.0E+4);
					}
				}
				
				if(t0_tdt - dynamicDatum.T0 < h * 8.0)
				{
					if(dynamicDatum.bOn_ManeuverForceAcc)	// AdamsCowell_RK，2014/4/21，鞠 冰
					{
						AdamsCowell_RK(dynamicDatum, t0_tdt - h * 8.0, backwardOrbitlist_ac, matRtPartiallist_ac, -h, 11);
					}
					else
					{
						AdamsCowell(dynamicDatum, t0_tdt - h * 8.0, backwardOrbitlist_ac, matRtPartiallist_ac, -h, 11);
					}
					for(size_t s_i = backwardOrbitlist_ac.size() - 1; s_i > 0; s_i--)
						orbitlist_ac.push_back(backwardOrbitlist_ac[s_i]);
				}
				if(t1_tdt - dynamicDatum.T0 > h * 8.0)
				{
					if(dynamicDatum.bOn_ManeuverForceAcc)	// AdamsCowell_RK，2014/4/21，鞠 冰
					{
						AdamsCowell_RK(dynamicDatum, t1_tdt + h * 8.0, forwardOrbitlist_ac, matRtPartiallist_ac, h, 11);
					}
					else
					{
						AdamsCowell(dynamicDatum, t1_tdt + h * 8.0, forwardOrbitlist_ac, matRtPartiallist_ac, h, 11);
					}
					for(size_t s_i = 0; s_i < forwardOrbitlist_ac.size(); s_i++)
						orbitlist_ac.push_back(forwardOrbitlist_ac[s_i]);
				}
				forecastOrbList.clear();
				int k = 0;
				double span = t1_tdt - t0_tdt;
				// 临时文件，校验数据时刻有效位，2014/4/24，鞠 冰
				//FILE* pfile_t = fopen("C:\\tdt_interptimelist.dat","w+");
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
					// 临时文件，校验数据时刻有效位，2014/4/24，鞠 冰
					//fprintf(pfile_t, "%30.18f\n", spanSecond_t);
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
				//fclose(pfile_t);
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
					forecastOrbList[s_i].t = TimeCoordConvert::TDT2GPST(forecastOrbList[s_i].t);
					m_TimeCoordConvert.J2000_ECEF(forecastOrbList[s_i].t, x_j2000, x_ecf);
					forecastOrbList[s_i].pos.x = x_ecf[0]; 
					forecastOrbList[s_i].pos.y = x_ecf[1]; 
					forecastOrbList[s_i].pos.z = x_ecf[2];
					forecastOrbList[s_i].vel.x = x_ecf[3]; 
					forecastOrbList[s_i].vel.y = x_ecf[4]; 
					forecastOrbList[s_i].vel.z = x_ecf[5];
				}	
			}
			if(result)
			{// 输出钟差解
				for(size_t s_m = 0; s_m < m_dataMixedGNSSlist.size(); s_m++)
				{
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_data.clear();
					for(int s_i = 0; s_i < int(m_editedMixedObsFile.m_data.size()); s_i++)
					{
						CLKEpoch clkEpoch;
						clkEpoch.t = m_editedMixedObsFile.m_data[s_i].t;
						clkEpoch.ARList.clear();
						clkEpoch.ASList.clear();
						CLKDatum ARDatum;
						int validIndex = m_dataMixedGNSSlist[s_m].validSysIndexList[s_i];
						if(validIndex == -1)
						{
							ARDatum.count = 0;
							ARDatum.name = "LEO ";
						}
						else
						{
							ARDatum.count = 2;
							ARDatum.name = "LEO ";
							ARDatum.clkBias = m_dataMixedGNSSlist[s_m].leoClockList[s_i] / SPEED_LIGHT;
							ARDatum.clkBiasSigma = 0.0;
						}
						clkEpoch.ARList.insert(CLKMap::value_type(ARDatum.name, ARDatum));
						m_dataMixedGNSSlist[s_m].m_recClkFile.m_data.push_back(clkEpoch);
					}
					// 整理文件头
					DayTime T_Now;
					T_Now.Now();
					sprintf(m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.szRinexVersion,   "     2.00           ");
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.cFileType = 'C';
					sprintf(m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.szProgramName, "%-20s","NUDTTK");
					sprintf(m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.szAgencyName, "%-20s","NUDT");
					sprintf(m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.szFileDate, "%04d-%02d-%02d %02d:%02d:%02d ",T_Now.year,T_Now.month,T_Now.day,T_Now.hour,T_Now.minute,int(T_Now.second));
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.LeapSecond = 0;
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.ClockDataTypeCount = 1;
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.pstrClockDataTypeList.clear();
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.pstrClockDataTypeList.push_back("    AR");
					sprintf(m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.szACShortName, "%-3s","NDT");
					sprintf(m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.szACFullName, "%-55s","National University of Defense Technology");
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.nStaCount = 1;
					sprintf(m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.szStaCoordFrame, "%-50s","ITRF05");
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.pStaPosList.clear();
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.bySatCount = 0;
					m_dataMixedGNSSlist[s_m].m_recClkFile.m_header.pszSatList.clear();
				}
				printf("clocks have been solved!\n");
			}
			return result;
		} 
	}
}

