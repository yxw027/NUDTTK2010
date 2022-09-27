#include "BDSatDynPOD.hpp"
#include "CLKfile.hpp"
#include "SP3file.hpp"
#include "GPSMeoSatDynPOD.hpp"
#include "Rinex2_1_NavFile.hpp"
#include "SolidTides.hpp"
#include "GNSSBasicCorrectFunc.hpp"
#include "RuningInfoFile.hpp"
#include <direct.h>


namespace NUDTTK
{
	namespace BDPod
	{
		BDSatDynPOD::BDSatDynPOD(void)
		{
		}

		BDSatDynPOD::~BDSatDynPOD(void)
		{
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
		//bool BDSatDynPOD::initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength)
		//{
		//	// 提取粗间隔的插值点
		//	double  threshold_coarseorbit_interval = 600; // 10分钟以上, 一般为15分钟
		//	double  cumulate_time = threshold_coarseorbit_interval * 2; // 保证第一个点被加入
		//	vector<int> coarseindexList;                    
		//	coarseindexList.clear();
		//	vector<TimePosVel>  coarseorbitList;
		//	coarseorbitList.clear();
		//	for(size_t s_i = 0; s_i < orbitlist.size(); s_i++)
		//	{	
		//		if(s_i > 0)
		//		{
		//			cumulate_time += orbitlist[s_i].t - orbitlist[s_i - 1].t;
		//		}
		//		if(cumulate_time >= threshold_coarseorbit_interval || s_i == orbitlist.size() - 1)
		//		{
		//			cumulate_time = 0;
		//			coarseindexList.push_back(int(s_i));
		//			coarseorbitList.push_back(orbitlist[s_i]);
		//		}
		//	}
		//	// 为了保证初轨的插值速度精度, 需要前面连续M点间隔不超过 threshold_coarseorbit_interval * 2
		//	const int nLagrangePoint = 9; // 9 点 Lagrange 插值
		//	int M = 4;
		//	int k0 = 0;
		//	while(1)
		//	{
		//		if(k0 + nLagrangePoint > int(coarseorbitList.size()))
		//		{
		//			printf("无法找到足够的插值点来获取初始轨道速度, 初始轨道解算失败!\n");
		//			return false;
		//		}
  //              double max_interval = 0;
		//		for(int s_i = k0 + 1; s_i < k0 + M; s_i++)
		//		{
		//			double innterval_i = coarseorbitList[s_i].t - coarseorbitList[s_i - 1].t;
		//			if(max_interval < innterval_i)
		//			{
		//				max_interval = innterval_i;
		//			}
		//		}
		//		if(max_interval <= threshold_coarseorbit_interval * 2)
		//		{
		//			break;
		//		}
		//		else
		//		{
		//			k0++;
		//		}
		//	}
		//	int i_begin = coarseindexList[k0];
		//	for(int s_i = 0; s_i < i_begin; s_i++)
		//	{// 清除前面的几个无效的初值点
		//		orbitlist.erase(orbitlist.begin());
		//	}
		//	while(1)
		//	{// 清除超过弧长的点
		//		int npos = int(orbitlist.size()) - 1;
		//		if(orbitlist[npos].t - orbitlist[0].t > arclength)
		//			orbitlist.erase(orbitlist.begin() + npos);
		//		else
		//			break;
		//	}
		//	// 利用微分匹配平滑求速, 更新 k0 点的速度信息 
		//	double *xa_t = new double [nLagrangePoint];
		//	double *ya_X = new double [nLagrangePoint];
		//	double *ya_Y = new double [nLagrangePoint];
		//	double *ya_Z = new double [nLagrangePoint];
		//	for(int s_i = k0; s_i < k0 + nLagrangePoint; s_i++)
		//	{
		//		xa_t[s_i - k0] = coarseorbitList[s_i].t - coarseorbitList[k0].t;
		//		ya_X[s_i - k0] = coarseorbitList[s_i].pos.x;
		//		ya_Y[s_i - k0] = coarseorbitList[s_i].pos.y;
		//		ya_Z[s_i - k0] = coarseorbitList[s_i].pos.z;
		//	}
		//	InterploationLagrange(xa_t, ya_X, nLagrangePoint, 0.0, orbitlist[0].pos.x, orbitlist[0].vel.x);
		//	InterploationLagrange(xa_t, ya_Y, nLagrangePoint, 0.0, orbitlist[0].pos.y, orbitlist[0].vel.y);
		//	InterploationLagrange(xa_t, ya_Z, nLagrangePoint, 0.0, orbitlist[0].pos.z, orbitlist[0].vel.z);
		//	delete xa_t;
		//	delete ya_X;
		//	delete ya_Y;
		//	delete ya_Z;
		//	// 以上计算是为了得到可靠的初始轨道速度

		//	FILE* pfile = fopen("c:\\初轨确定.txt", "w+");
		//	//  时间坐标系统一, 坐标-J2000, 时间-TDT
		//	vector<TDT> interpTimelist;
		//	interpTimelist.resize(orbitlist.size());
		//	for(size_t s_i = 0; s_i < orbitlist.size(); s_i ++)
		//	{
		//		double x_ecf[6];
		//		double x_j2000[6];
		//		x_ecf[0] = orbitlist[s_i].pos.x;  
		//		x_ecf[1] = orbitlist[s_i].pos.y;  
		//		x_ecf[2] = orbitlist[s_i].pos.z;
		//		x_ecf[3] = orbitlist[s_i].vel.x; 
		//		x_ecf[4] = orbitlist[s_i].vel.y; 
		//		x_ecf[5] = orbitlist[s_i].vel.z;
		//		GPST t_GPS = orbitlist[s_i].t;
		//		m_TimeCoordConvert.ECEF_J2000(t_GPS, x_j2000, x_ecf);
		//		orbitlist[s_i].t = TimeCoordConvert::GPST2TDT(t_GPS);
		//		orbitlist[s_i].pos.x = x_j2000[0]; 
		//		orbitlist[s_i].pos.y = x_j2000[1]; 
		//		orbitlist[s_i].pos.z = x_j2000[2];
		//		orbitlist[s_i].vel.x = x_j2000[3]; 
		//		orbitlist[s_i].vel.y = x_j2000[4]; 
		//		orbitlist[s_i].vel.z = x_j2000[5];
		//		interpTimelist[s_i] = orbitlist[s_i].t;
		//	}
		//	dynamicDatum.T0 = orbitlist[0].t;
		//	dynamicDatum.X0 = orbitlist[0].getPosVel();
		//	dynamicDatum.ArcLength = orbitlist[orbitlist.size() - 1].t - dynamicDatum.T0; 
		//	dynamicDatum.init(dynamicDatum.ArcLength);
		//	// 2008/11/15
		//	TDT t_End = orbitlist[orbitlist.size() - 1].t;
		//	int  k = 0; // 记录迭代的次数
		//	bool flag_robust = false;
		//	bool flag_done   = false;
		//	double factor = 4.0;
		//	Matrix matW(int(orbitlist.size()), 3); // 观测权矩阵
		//	for(size_t s_i = 0; s_i < orbitlist.size(); s_i++)
		//	{
		//		for(size_t s_j = 0; s_j < 3; s_j++)
		//		{
		//			matW.SetElement(int(s_i), int(s_j), 1.0); 
		//		}
		//	}
		//	bool result = true;
		//	size_t count_measureorbit_control;
		//	vector<TDT> interpTimelist_control;
		//	int num_control = 0;
		//	while(1)
		//	{
		//		k++;
		//		if(k >= m_podParaDefine.max_OrbitIterativeNum)
		//		{
		//			result = false;
		//			printf("初轨确定程序迭代次数溢出(initDynDatumEst)!");
		//			break;
		//		}
		//		vector<TimePosVel> interpOrbitlist; // 插值序列
		//		vector<Matrix> interpRtPartiallist; // 插值偏导数序列
		//		if(k == 1)
		//		{
		//			adamsCowell_Interp(interpTimelist, dynamicDatum, interpOrbitlist, interpRtPartiallist);
		//			// 由于初轨的速度是通过曲线拟合的方式计算得到, 精度不高, 难以支撑较长弧段, 因此需要控制初轨拟合的弧段长度, 首先进行短弧段定轨
		//			count_measureorbit_control = orbitlist.size();
		//			interpTimelist_control = interpTimelist;
		//			for(int i = 0; i < int(orbitlist.size()); i++)
		//			{
		//				double error_interp = sqrt(pow(orbitlist[i].pos.x - interpOrbitlist[i].pos.x * matW.GetElement(i, 0), 2)
		//										 + pow(orbitlist[i].pos.y - interpOrbitlist[i].pos.y * matW.GetElement(i, 1), 2)
		//										 + pow(orbitlist[i].pos.z - interpOrbitlist[i].pos.z * matW.GetElement(i, 2), 2));
		//				if(error_interp >= 10000.0) // 当前点的预报残差阈值超过10km的点
		//				{
  //                          // 统计当前点以后的预报残差阈值超过 10km 的点, 如果“大部分”超过 10km, 则认为是该残差是由于预报精度较差造成的, 并非野值点造成
		//					int count_threshold_points = 0;
		//					for(int j = i; j < int(orbitlist.size()); j++)
		//					{
		//						double error_interp_j = sqrt(pow(orbitlist[j].pos.x - interpOrbitlist[j].pos.x, 2)
		//												   + pow(orbitlist[j].pos.y - interpOrbitlist[j].pos.y, 2)
		//												   + pow(orbitlist[j].pos.z - interpOrbitlist[j].pos.z, 2));
		//						if(error_interp_j >= 10000.0)
		//							count_threshold_points++;
		//					}
		//					if((count_threshold_points * 1.0 / (orbitlist.size() - i)) > 0.30)
		//					{ 
		//						count_measureorbit_control = i + 1;
		//						interpTimelist_control.resize(count_measureorbit_control);
		//						for(int j = 0; j < int(count_measureorbit_control); j++)
		//						{
		//							interpTimelist_control[j] = interpTimelist[j];
		//						}
		//						num_control++;
		//						fprintf(pfile, "正在第%2d次分布迭代, 解算点数 = %6d/%6d.\n", num_control, count_measureorbit_control, orbitlist.size());
		//						break;
		//					}
		//				}
		//			}
		//		}
		//		else
		//		{
		//			adamsCowell_Interp(interpTimelist_control, dynamicDatum, interpOrbitlist, interpRtPartiallist);
		//		}
		//		// 判断光压参数是否为 0, 因为短弧段定轨可能低轨卫星的完全存在于阴影中
		//		int count_SolarPressure = 0;
  //              for(int i = 0; i < int(count_measureorbit_control); i++)
		//		{
		//			double solarCoefficient =   pow(interpRtPartiallist[i].GetElement(0, 6), 2)
		//				                      + pow(interpRtPartiallist[i].GetElement(1, 6), 2)
		//									  + pow(interpRtPartiallist[i].GetElement(2, 6), 2);
		//			if(solarCoefficient != 0)
		//				count_SolarPressure++;
		//		}
		//		int NUM_M = 6; // 待估参数的个数: 6个初始轨道根数
		//		Matrix matH(int(count_measureorbit_control) * 3, NUM_M);
		//		Matrix matY(int(count_measureorbit_control) * 3, 1);  
		//		for(int i = 0; i < int(count_measureorbit_control); i++)
		//		{
		//			matY.SetElement(i * 3 + 0, 0, matW.GetElement(i, 0) * (orbitlist[i].pos.x - interpOrbitlist[i].pos.x));
		//			matY.SetElement(i * 3 + 1, 0, matW.GetElement(i, 1) * (orbitlist[i].pos.y - interpOrbitlist[i].pos.y));
		//			matY.SetElement(i * 3 + 2, 0, matW.GetElement(i, 2) * (orbitlist[i].pos.z - interpOrbitlist[i].pos.z));
		//			for(int j = 0; j < 6; j++)
		//			{
		//				matH.SetElement(i * 3 + 0, j, matW.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, j));
		//				matH.SetElement(i * 3 + 1, j, matW.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, j));
		//				matH.SetElement(i * 3 + 2, j, matW.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, j));
		//			}
		//		}
		//		Matrix matX = (matH.Transpose() * matH).Inv_Ssgj() * matH.Transpose() * matY; 
		//		dynamicDatum.X0.x  += matX.GetElement(0,0);
		//		dynamicDatum.X0.y  += matX.GetElement(1,0);
		//		dynamicDatum.X0.z  += matX.GetElement(2,0);
		//		dynamicDatum.X0.vx += matX.GetElement(3,0);
		//		dynamicDatum.X0.vy += matX.GetElement(4,0);
		//		dynamicDatum.X0.vz += matX.GetElement(5,0);
		//		dynamicDatum.solarPressureParaList[0].Cr = 0;

		//		// 判断收敛条件
		//		double max_pos = 0;
		//		double max_vel = 0;
		//		for(int i = 0; i < 3; i++)
		//		{
		//			max_pos = max(max_pos, fabs(matX.GetElement(i,     0)));
		//			max_vel = max(max_vel, fabs(matX.GetElement(i + 3, 0)));
		//		}

		//		if(max_pos <= 1.0E-3 || flag_done)
		//		{
		//			if(flag_robust == false && flag_done == false)
		//			{
		//				flag_robust = true;
		//				// 计算方差
		//				double rms_fitresidual = 0; 
		//				size_t count_normalpoint = 0;
		//				for(int i = 0; i < int(count_measureorbit_control); i++)
		//				{
		//					if(matW.GetElement(i, 0) == 1.0)
		//					{
		//						count_normalpoint++;
		//						rms_fitresidual += matY.GetElement(i * 3 + 0, 0) * matY.GetElement(i * 3 + 0, 0);
		//					}
		//					if(matW.GetElement(i, 1) == 1.0)
		//					{
		//						count_normalpoint++;
		//						rms_fitresidual += matY.GetElement(i * 3 + 1, 0) * matY.GetElement(i * 3 + 1, 0);
		//					}
		//					if(matW.GetElement(i, 2) == 1.0)
		//					{
		//						count_normalpoint++;
		//						rms_fitresidual += matY.GetElement(i * 3 + 2, 0) * matY.GetElement(i * 3 + 2, 0);
		//					}
		//				}
		//				rms_fitresidual = sqrt(rms_fitresidual / count_normalpoint);
		//				fprintf(pfile, "定轨残差 = %10.4f\n", rms_fitresidual);
  //                      int count_outliers = 0;
		//				for(int i = 0; i < int(count_measureorbit_control); i++)
		//				{
		//					if(fabs(matY.GetElement(i * 3 + 0, 0)) >= factor * rms_fitresidual)
		//					{
		//						matW.SetElement(i, 0, rms_fitresidual / fabs(matY.GetElement(i * 3 + 0, 0)));
		//						fprintf(pfile, "i = %5d, X = %14.4f\n", i, matY.GetElement(i * 3 + 0, 0));
		//						count_outliers++;
		//					}
		//					if(fabs(matY.GetElement(i * 3 + 1, 0)) >= factor * rms_fitresidual)
		//					{
		//						matW.SetElement(i, 1, rms_fitresidual / fabs(matY.GetElement(i * 3 + 1, 0)));
		//						fprintf(pfile, "i = %5d, Y = %14.4f\n", i, matY.GetElement(i * 3 + 1, 0));
		//						count_outliers++;
		//					}
		//					if(fabs(matY.GetElement(i * 3 + 2, 0)) >= factor * rms_fitresidual)
		//					{
		//						matW.SetElement(i, 2, rms_fitresidual / fabs(matY.GetElement(i * 3 + 2, 0)));
		//						fprintf(pfile, "i = %5d, Z = %14.4f\n", i, matY.GetElement(i * 3 + 2, 0));
		//						count_outliers++;
		//					}
		//				}
		//				flag_done = true;
		//				if(count_outliers > 0)
		//				{
		//					continue;
		//				}
		//			}
		//			if(count_measureorbit_control <= orbitlist.size() / 2 && num_control <= 3)
		//			{// 分布解算完毕，重新开始新的迭代
		//				flag_robust = false;
		//				flag_done = false;
		//				for(size_t s_i = 0; s_i < orbitlist.size(); s_i++)
		//				{
		//					for(size_t s_j = 0; s_j < 3; s_j++)
		//					{
		//						matW.SetElement(int(s_i), int(s_j), 1.0); 
		//					}
		//				}
		//				k = 0;
		//				continue;
		//			}
		//			fprintf(pfile, "经过%d次迭代收敛:\n%14.4f\n%14.4f\n%14.4f\n%14.4f\n%14.4f\n%14.4f\n", k, 
		//																								  dynamicDatum.X0.x,
		//																								  dynamicDatum.X0.y,
		//																								  dynamicDatum.X0.z,
		//																								  dynamicDatum.X0.vx,
		//																								  dynamicDatum.X0.vy,
		//																								  dynamicDatum.X0.vz);
		//			break;
		//		}
		//	}
		//	fclose(pfile);
		//	if(result)
		//	{
		//		return true;
		//	}
		//	else
		//	{
		//		printf("初始轨道解算失败(initDynDatumEst)!\n");
		//		return false;
		//	}
		//}

		// 子程序名称： dynamicPOD_pos   
		// 功能：根据轨道位置定轨, 编写目的通过拟合GPS卫星的轨道来改进轨道力学模型精度
		// 变量类型：orbitlist          : 待拟合的轨道列表, 采用GPST, ITRF坐标系
		//           dynamicDatum       : 拟合后的初始动力学轨道参数
		//           t_forecastBegin    : 预报轨道初始时间, GPST
		//           t_forecastEnd      : 预报轨道终止时间, GPST
		//           orbitlist_forecast : 预报轨道列表, 采用GPST, ITRF坐标系
		//           interval           : 预报轨道间隔
		//           bforecast          : 预报标记, 默认true, 否则不进行预报, 用于初轨确定
		// 输入：orbitlist, dynamicDatum, t_forecastBegin, t_forecastEnd, interval, bforecast
		// 输出：dynamicDatum, orbitlist_forecast
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2012/03/12
		// 版本时间：
		// 修改记录：
		// 备注： 
		//bool BDSatDynPOD::dynamicPOD_pos(vector<TimePosVel>  orbitlist, SatdynBasicDatum &dynamicDatum, GPST t_forecastBegin, GPST t_forecastEnd,  vector<TimePosVel> &orbitlist_forecast, double interval, bool bforecast)
		//{
		//	size_t count_orbitlist = orbitlist.size(); 
		//	if(count_orbitlist <= 0)
		//	{
		//		return false;
		//	}

		//	// 进行初轨确定
		//	SatdynBasicDatum dynamicDatum_0;
  //          if(!initDynDatumEst(orbitlist, dynamicDatum_0,  3600 * 3.0))
		//		return false;

		//	TDT t_End = TimeCoordConvert::GPST2TDT(orbitlist[orbitlist.size() - 1].t);
		//	dynamicDatum.T0 = dynamicDatum_0.T0;
		//	dynamicDatum.ArcLength = t_End - dynamicDatum.T0;
		//	dynamicDatum.X0 = dynamicDatum_0.X0;
		//	dynamicDatum.init(3600.0 * 24);

		//	//  时间坐标系统一, 坐标--J2000; 时间--TDT
		//	//  时间坐标系统一, 坐标-J2000, 时间-TDT
		//	vector<TDT> interpTimelist;
		//	interpTimelist.resize(orbitlist.size());
		//	for(size_t s_i = 0; s_i < orbitlist.size(); s_i ++)
		//	{
		//		double x_ecf[6];
		//		double x_j2000[6];
		//		x_ecf[0] = orbitlist[s_i].pos.x;  
		//		x_ecf[1] = orbitlist[s_i].pos.y;  
		//		x_ecf[2] = orbitlist[s_i].pos.z;
		//		x_ecf[3] = orbitlist[s_i].vel.x; 
		//		x_ecf[4] = orbitlist[s_i].vel.y; 
		//		x_ecf[5] = orbitlist[s_i].vel.z;
		//		GPST t_GPS = orbitlist[s_i].t;
		//		m_TimeCoordConvert.ECEF_J2000(t_GPS, x_j2000, x_ecf);
		//		orbitlist[s_i].t = TimeCoordConvert::GPST2TDT(t_GPS);
		//		orbitlist[s_i].pos.x = x_j2000[0]; 
		//		orbitlist[s_i].pos.y = x_j2000[1]; 
		//		orbitlist[s_i].pos.z = x_j2000[2];
		//		orbitlist[s_i].vel.x = x_j2000[3]; 
		//		orbitlist[s_i].vel.y = x_j2000[4]; 
		//		orbitlist[s_i].vel.z = x_j2000[5];
		//		interpTimelist[s_i] = orbitlist[s_i].t;
		//	}

		//	int on_solar = 1;
		//	int count_DynParameter = 6;
		//	if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
		//		count_DynParameter += int(dynamicDatum.solarPressureParaList.size()) * on_solar;
		//	else if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
		//		count_DynParameter += int(dynamicDatum.solarPressureParaList.size()) * 9 * on_solar;

		//	int  k = 0; // 记录迭代的次数
		//	bool flag_robust = false;
		//	bool flag_done   = false;
		//	double factor = 3.0;
		//	Matrix matW(int(orbitlist.size()), 3); // 观测权矩阵
		//	for(size_t s_i = 0; s_i < orbitlist.size(); s_i++)
		//	{
		//		for(size_t s_j = 0; s_j < 3; s_j++)
		//		{
		//			matW.SetElement(int(s_i), int(s_j), 1.0); 
		//		}
		//	}
		//	bool result = true;
		//	while(1)
		//	{
		//		k++;
		//		if(k >= m_podParaDefine.max_OrbitIterativeNum)
		//		{
		//			result = false;
		//			printf("轨道确定程序迭代次数溢出(dynamicPOD_pos)！");
		//			break;
		//		}

		//		vector<TimePosVel> interpOrbitlist; // 插值序列
		//		vector<Matrix> interpRtPartiallist; // 插值偏导数序列
		//		adamsCowell_Interp(interpTimelist, dynamicDatum, interpOrbitlist, interpRtPartiallist);
		//		Matrix matH(int(orbitlist.size()) * 3, count_DynParameter);
		//		Matrix matY(int(orbitlist.size()) * 3, 1);  
		//		for(int i = 0; i < int(orbitlist.size()); i++)
		//		{
		//			matY.SetElement(i * 3 + 0, 0, matW.GetElement(i, 0) * (orbitlist[i].pos.x - interpOrbitlist[i].pos.x));
		//			matY.SetElement(i * 3 + 1, 0, matW.GetElement(i, 1) * (orbitlist[i].pos.y - interpOrbitlist[i].pos.y));
		//			matY.SetElement(i * 3 + 2, 0, matW.GetElement(i, 2) * (orbitlist[i].pos.z - interpOrbitlist[i].pos.z));
		//			for(int j = 0; j < 6; j++)
		//			{
		//				matH.SetElement(i * 3 + 0, j, matW.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, j));
		//				matH.SetElement(i * 3 + 1, j, matW.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, j));
		//				matH.SetElement(i * 3 + 2, j, matW.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, j));
		//			}
		//			int index_begin_dynamicDatum = 6;
		//			int index_begin_est = 6;
		//			if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
		//			{
		//				for(int j = 0; j < int(on_solar * dynamicDatum.solarPressureParaList.size()); j++)
		//				{
		//					matH.SetElement(i * 3 + 0, index_begin_est + j, matW.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, index_begin_dynamicDatum + j));
		//					matH.SetElement(i * 3 + 1, index_begin_est + j, matW.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, index_begin_dynamicDatum + j));
		//					matH.SetElement(i * 3 + 2, index_begin_est + j, matW.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, index_begin_dynamicDatum + j));
		//				}
		//			}
		//			else if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
		//			{
		//				for(int j = 0; j < int(on_solar * dynamicDatum.solarPressureParaList.size() * 9); j++)
		//				{
		//					matH.SetElement(i * 3 + 0, index_begin_est + j, matW.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, index_begin_dynamicDatum + j));
		//					matH.SetElement(i * 3 + 1, index_begin_est + j, matW.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, index_begin_dynamicDatum + j));
		//					matH.SetElement(i * 3 + 2, index_begin_est + j, matW.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, index_begin_dynamicDatum + j));
		//				}
		//			}
		//		}
		//		Matrix matX = (matH.Transpose() * matH).Inv_Ssgj() * matH.Transpose() * matY; 
		//		dynamicDatum.X0.x  += matX.GetElement(0,0);
		//		dynamicDatum.X0.y  += matX.GetElement(1,0);
		//		dynamicDatum.X0.z  += matX.GetElement(2,0);
		//		dynamicDatum.X0.vx += matX.GetElement(3,0);
		//		dynamicDatum.X0.vy += matX.GetElement(4,0);
		//		dynamicDatum.X0.vz += matX.GetElement(5,0);
		//		int index_begin_est = 6;
		//		if(on_solar)
		//		{
		//			if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
		//			{
		//				for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
		//					dynamicDatum.solarPressureParaList[s_k].Cr +=  matX.GetElement(index_begin_est + s_k, 0);
		//			}
		//			else if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
		//			{
		//				for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
		//				{
		//					dynamicDatum.solarPressureParaList[s_k].A_D0 += matX.GetElement(index_begin_est + 9 * s_k,     0);
		//					dynamicDatum.solarPressureParaList[s_k].A_DC += matX.GetElement(index_begin_est + 9 * s_k + 1, 0);
		//					dynamicDatum.solarPressureParaList[s_k].A_DS += matX.GetElement(index_begin_est + 9 * s_k + 2, 0);
		//					dynamicDatum.solarPressureParaList[s_k].A_Y0 += matX.GetElement(index_begin_est + 9 * s_k + 3, 0);
		//					dynamicDatum.solarPressureParaList[s_k].A_YC += matX.GetElement(index_begin_est + 9 * s_k + 4, 0);
		//					dynamicDatum.solarPressureParaList[s_k].A_YS += matX.GetElement(index_begin_est + 9 * s_k + 5, 0);
		//					dynamicDatum.solarPressureParaList[s_k].A_X0 += matX.GetElement(index_begin_est + 9 * s_k + 6, 0);
		//					dynamicDatum.solarPressureParaList[s_k].A_XC += matX.GetElement(index_begin_est + 9 * s_k + 7, 0);
		//					dynamicDatum.solarPressureParaList[s_k].A_XS += matX.GetElement(index_begin_est + 9 * s_k + 8, 0);
		//				}
		//			}
		//		}
		//		else
		//		{
		//			if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
		//			{
		//				for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
		//					dynamicDatum.solarPressureParaList[s_k].Cr =  0.0;
		//			}
		//			else if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
		//			{
		//				for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
		//				{
		//					dynamicDatum.solarPressureParaList[s_k].A_D0 = 0.0;
		//					dynamicDatum.solarPressureParaList[s_k].A_DC = 0.0;
		//					dynamicDatum.solarPressureParaList[s_k].A_DS = 0.0;
		//					dynamicDatum.solarPressureParaList[s_k].A_Y0 = 0.0;
		//					dynamicDatum.solarPressureParaList[s_k].A_YC = 0.0;
		//					dynamicDatum.solarPressureParaList[s_k].A_YS = 0.0;
		//					dynamicDatum.solarPressureParaList[s_k].A_X0 = 0.0;
		//					dynamicDatum.solarPressureParaList[s_k].A_XC = 0.0;
		//					dynamicDatum.solarPressureParaList[s_k].A_XS = 0.0;
		//				}
		//			}
		//		}
		//		// 判断收敛条件
		//		double max_pos = 0;
		//		double max_vel = 0;
		//		for(int i = 0; i < 3; i++)
		//		{
		//			max_pos = max(max_pos, fabs(matX.GetElement(i,     0)));
		//			max_vel = max(max_vel, fabs(matX.GetElement(i + 3, 0)));
		//		}
		//	    if(max_pos <= 1.0E-4  || flag_done)
		//		{
		//			if(flag_robust == false && flag_done == false)
		//			{
		//				flag_robust = true;
		//				// 计算方差
		//				double rms_fitresidual = 0; 
		//				size_t count_normalpoint = 0;
		//				for(int i = 0; i < int(orbitlist.size()); i++)
		//				{
		//					if(matW.GetElement(i, 0) == 1.0)
		//					{
		//						count_normalpoint++;
		//						rms_fitresidual += matY.GetElement(i * 3 + 0, 0) * matY.GetElement(i * 3 + 0, 0);
		//					}
		//					if(matW.GetElement(i, 1) == 1.0)
		//					{
		//						count_normalpoint++;
		//						rms_fitresidual += matY.GetElement(i * 3 + 1, 0) * matY.GetElement(i * 3 + 1, 0);
		//					}
		//					if(matW.GetElement(i, 2) == 1.0)
		//					{
		//						count_normalpoint++;
		//						rms_fitresidual += matY.GetElement(i * 3 + 2, 0) * matY.GetElement(i * 3 + 2, 0);
		//					}
		//				}
		//				rms_fitresidual = sqrt(rms_fitresidual / count_normalpoint);
  //                      int count_outliers = 0;
		//				for(int i = 0; i < int(orbitlist.size()); i++)
		//				{
		//					if(fabs(matY.GetElement(i * 3 + 0, 0)) >= factor * rms_fitresidual)
		//					{
		//						matW.SetElement(i, 0, rms_fitresidual / fabs(matY.GetElement(i * 3 + 0, 0)));
		//						count_outliers++;
		//					}
		//					if(fabs(matY.GetElement(i * 3 + 1, 0)) >= factor * rms_fitresidual)
		//					{
		//						matW.SetElement(i, 1, rms_fitresidual / fabs(matY.GetElement(i * 3 + 1, 0)));
		//						count_outliers++;
		//					}
		//					if(fabs(matY.GetElement(i * 3 + 2, 0)) >= factor * rms_fitresidual)
		//					{
		//						matW.SetElement(i, 2, rms_fitresidual / fabs(matY.GetElement(i * 3 + 2, 0)));
		//						count_outliers++;
		//					}
		//				}
		//				flag_done = true;
		//				if(count_outliers > 0)
		//				{
		//					//cout<<"count_outliers = "<<count_outliers<<endl;
		//					continue;
		//				}
		//			}
		//			printf("经过%d次迭代收敛:\n%14.4f\n%14.4f\n%14.4f\n%14.4f\n%14.4f\n%14.4f\n", k, 
		//																						  dynamicDatum.X0.x,
		//																						  dynamicDatum.X0.y,
		//																						  dynamicDatum.X0.z,
		//																						  dynamicDatum.X0.vx,
		//																						  dynamicDatum.X0.vy,
		//																						  dynamicDatum.X0.vz);
		//			for(int s_k = 0; s_k < int(dynamicDatum.solarPressureParaList.size()); s_k++)
		//			{
		//				if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
		//					printf("%14.4f\n", dynamicDatum.solarPressureParaList[s_k].Cr);
		//				else if(dynamicDatum.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
		//					printf("%14.10f\n%14.10f\n%14.10f\n%14.10f\n%14.10f\n%14.10f\n%14.10f\n%14.10f\n%14.10f\n\n\n",dynamicDatum.solarPressureParaList[s_k].A_D0,
		//																												   dynamicDatum.solarPressureParaList[s_k].A_DC,
		//																												   dynamicDatum.solarPressureParaList[s_k].A_DS,
		//																												   dynamicDatum.solarPressureParaList[s_k].A_Y0,
		//																												   dynamicDatum.solarPressureParaList[s_k].A_YC,
		//																												   dynamicDatum.solarPressureParaList[s_k].A_YS,
		//																												   dynamicDatum.solarPressureParaList[s_k].A_X0,
		//																												   dynamicDatum.solarPressureParaList[s_k].A_XC,
		//																												   dynamicDatum.solarPressureParaList[s_k].A_XS);
		//			}
		//			break;
		//		}
		//	}
		//	
		//	// 进行轨道预报
		//	TDT t_Begin_TDT = TimeCoordConvert::GPST2TDT(t_forecastBegin);
		//	TDT t_End_TDT = TimeCoordConvert::GPST2TDT(t_forecastEnd);
		//	if(result)
		//	{
		//		vector<TimePosVel> orbitlist_ac;
		//		vector<Matrix> matRtPartiallist_ac;
		//		// 倒向积分，积分区间 [para.T0, T_End   + h * 4]，为保证插值精度向两端进行扩展
		//		vector<TimePosVel> backwardOrbitlist_ac; 
		//	    vector<TimePosVel> forwardOrbitlist_ac; 
  //              double h = 75.0;
		//		if(t_Begin_TDT - dynamicDatum.T0 < h * 8.0)
		//		{
		//			AdamsCowell(dynamicDatum, t_Begin_TDT - h * 8.0, backwardOrbitlist_ac, matRtPartiallist_ac, -h, 11);
		//			for(size_t s_i = backwardOrbitlist_ac.size() - 1; s_i > 0; s_i--)
		//				orbitlist_ac.push_back(backwardOrbitlist_ac[s_i]);
		//		}
		//		if(t_End_TDT - dynamicDatum.T0 > h * 8.0)
		//		{
		//			AdamsCowell(dynamicDatum, t_End_TDT + h * 8.0, forwardOrbitlist_ac, matRtPartiallist_ac, h, 11);
		//			for(size_t s_i = 0; s_i < forwardOrbitlist_ac.size(); s_i++)
		//				orbitlist_ac.push_back(forwardOrbitlist_ac[s_i]);
		//		}
		//		orbitlist_forecast.clear();
		//		int k = 0;
		//		double span = t_End_TDT - t_Begin_TDT;
		//		while(k * interval < span)             
		//		{
		//			TimePosVel point;
		//			point.t = t_Begin_TDT + k * interval;
		//			orbitlist_forecast.push_back(point);
		//			k++;
		//		}
		//		size_t count_forecastOrbit = orbitlist_forecast.size();
		//		size_t count_ac = orbitlist_ac.size();
		//		const int nLagrange = 8; 
		//		if(count_ac < nLagrange) // 如果数据点个数小于nLagrange返回, 要求弧段长度 > h * nlagrange = 4分钟
		//			return false;
		//		for(size_t s_i = 0; s_i < count_forecastOrbit; s_i++)
		//		{
		//			double spanSecond_t = orbitlist_forecast[s_i].t - orbitlist_ac[0].t; 
		//			int nLeftPos  = int(spanSecond_t / h);      
		//			int nLeftNum  = int(floor(nLagrange / 2.0));    
		//			int nRightNum = int(ceil(nLagrange / 2.0));
		//			int nBegin, nEnd;                                                    // 位于区间[0, nCount_AC-1]
		//			if(nLeftPos - nLeftNum + 1 < 0)                                      // nEnd - nBegin = nLagrange - 1 
		//			{
		//				nBegin = 0;
		//				nEnd   = nLagrange - 1;
		//			}
		//			else if(nLeftPos + nRightNum >= int(count_ac))
		//			{
		//				nBegin = int(count_ac) - nLagrange;
		//				nEnd   = int(count_ac) - 1;
		//			}
		//			else
		//			{
		//				nBegin = nLeftPos - nLeftNum + 1;
		//				nEnd   = nLeftPos + nRightNum;
		//			}
		//			// 轨道点
		//			TimePosVel interpOrbit; // 所有元素的参考时刻均相同
		//			interpOrbit.t = orbitlist_forecast[s_i].t;
		//			double *x = new double [nLagrange];
		//			double *y = new double [nLagrange];
		//			for(int i = nBegin; i <= nEnd; i++)
		//				x[i - nBegin] = orbitlist_ac[i].t - orbitlist_ac[0].t; // 参考相对时间点
		//			// X
		//			for(int i = nBegin; i <= nEnd; i++)
		//				y[i - nBegin] = orbitlist_ac[i].pos.x;
		//			InterploationLagrange(x, y, nLagrange, spanSecond_t, interpOrbit.pos.x);
		//			// Y
		//			for(int i = nBegin; i <= nEnd; i++)
		//				y[i - nBegin] = orbitlist_ac[i].pos.y;
		//			InterploationLagrange(x, y, nLagrange, spanSecond_t, interpOrbit.pos.y);
		//			// Z
		//			for(int i = nBegin; i <= nEnd; i++)
		//				y[i - nBegin] = orbitlist_ac[i].pos.z;
		//			InterploationLagrange(x, y, nLagrange, spanSecond_t, interpOrbit.pos.z);
		//			// Vx
		//			for(int i = nBegin; i <= nEnd; i++)
		//				y[i - nBegin] = orbitlist_ac[i].vel.x;
		//			InterploationLagrange(x, y, nLagrange, spanSecond_t, interpOrbit.vel.x);
		//			// Vy
		//			for(int i = nBegin; i <= nEnd; i++)
		//				y[i - nBegin] = orbitlist_ac[i].vel.y;
		//			InterploationLagrange(x, y, nLagrange, spanSecond_t, interpOrbit.vel.y);
		//			// Vz
		//			for(int i = nBegin; i <= nEnd; i++)
		//				y[i - nBegin] = orbitlist_ac[i].vel.z;
		//			InterploationLagrange(x, y, nLagrange, spanSecond_t, interpOrbit.vel.z);
		//			orbitlist_forecast[s_i] = interpOrbit;
		//			delete x;
		//		    delete y;
		//		}
		//		// 转换到地球固定坐标系, 坐标系: ITRF 系, 时间: GPS
		//		for(size_t s_i = 0; s_i < count_forecastOrbit; s_i++)
		//		{
		//			double x_ecf[6];
		//			double x_j2000[6];
		//			x_j2000[0] = orbitlist_forecast[s_i].pos.x;  
		//			x_j2000[1] = orbitlist_forecast[s_i].pos.y;  
		//			x_j2000[2] = orbitlist_forecast[s_i].pos.z;
		//			x_j2000[3] = orbitlist_forecast[s_i].vel.x; 
		//			x_j2000[4] = orbitlist_forecast[s_i].vel.y; 
		//			x_j2000[5] = orbitlist_forecast[s_i].vel.z;
		//			orbitlist_forecast[s_i].t = TimeCoordConvert::TDT2GPST(orbitlist_forecast[s_i].t);
		//			m_TimeCoordConvert.J2000_ECEF(orbitlist_forecast[s_i].t, x_j2000, x_ecf);
		//			orbitlist_forecast[s_i].pos.x = x_ecf[0]; 
		//			orbitlist_forecast[s_i].pos.y = x_ecf[1]; 
		//			orbitlist_forecast[s_i].pos.z = x_ecf[2];
		//			orbitlist_forecast[s_i].vel.x = x_ecf[3]; 
		//			orbitlist_forecast[s_i].vel.y = x_ecf[4]; 
		//			orbitlist_forecast[s_i].vel.z = x_ecf[5];
		//		}
		//	}
		//	return true;
		//}		


		// 子程序名称： sp3Fit   
		// 功能：拟合sp3轨道
		// 变量类型：strSp3FilePath   : sp3文件路径
		//           paraSp3Fit       : 拟合后参数
		//           bOnEst_EOP       : 地球旋转参数估计开关
		// 输入：strSp3FilePath
		// 输出：paraSp3Fit
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2012/09/19
		// 版本时间：
		// 修改记录：1.2012/09/28 由谷德峰修改, ut1 添加紧约束
		// 备注： 
		bool BDSatDynPOD::sp3Fit(string strSp3FilePath, Sp3FitParameter& paraSp3Fit, bool bOnEst_EOP)
		{
			char info[200];
			// 分析 strSp3FilePath 路径, 提取根目录和文件名
			string sp3FileName = strSp3FilePath.substr(strSp3FilePath.find_last_of("\\") + 1);
			string folder = strSp3FilePath.substr(0, strSp3FilePath.find_last_of("\\"));
			string sp3FileName_noexp = sp3FileName.substr(0, sp3FileName.find_last_of("."));
			// 读取sp3文件, 检查数据的完整性, 设定初始轨道
			SP3File sp3file;
			if(!sp3file.open(strSp3FilePath.c_str()))
			{
				sprintf(info,"%s 文件无法打开!", strSp3FilePath.c_str());
				RuningInfoFile::Add(info);
				printf("%s\n",info);
				return false;
			}

			paraSp3Fit.satParaList.clear();
			// 根据弧段长度来确定 paraSp3Fit.t0_xpyput1, 保证在“12h”的整数倍上, 20131209, 谷德峰
			double ArcLength = sp3file.m_data[sp3file.m_data.size() - 1].t - sp3file.m_data[0].t;
			if(fmod(ArcLength, 86400.0) >= 0.5 * 86400.0)
				paraSp3Fit.t0_xpyput1 = sp3file.m_data[0].t + 43200.0 * int (ceil(ArcLength / 86400.0));
			else 
				paraSp3Fit.t0_xpyput1 = sp3file.m_data[0].t + 43200.0 * int (floor(ArcLength / 86400.0));
			//paraSp3Fit.t0_xpyput1 = sp3file.m_data[0].t + 43200.0; // 12h处
			for(int i = 0; i < int(sp3file.m_header.pstrSatNameList.size()); i++) 
			{
				if(sp3file.m_header.pbySatAccuracyList[i] >= 15 || sp3file.m_header.pstrSatNameList[i][0] != 'C')//对于BDS卫星这一条件可以放宽
				{
					continue;// 精度较差的卫星不参与拟合
				}

				int nPRN = SP3File::getSatPRN(sp3file.m_header.pstrSatNameList[i]);
				Sp3Fit_SatdynDatum satdynDatum;
				satdynDatum.sp3orbitList_ECEF.clear();
                for(int j = 0; j < int(sp3file.m_data.size()); j++) 
				{
					TimePosVel posvel;
					posvel.t = sp3file.m_data[j].t;
					/*if((it = sp3file.m_data[j].sp3.find(nPRN)) != sp3file.m_data[j].sp3.end())
					{
						posvel.pos = it->second.pos * 1000.0;
						posvel.vel = it->second.vel * 1000.0;
                        satdynDatum.sp3orbitList_ECEF.push_back(posvel);
					}*/
					SP3Datum sp3Datum;
					if(sp3file.getEphemeris(posvel.t, nPRN, sp3Datum, 9 , 'C')) // 插值获得速度
					{
						posvel.pos = sp3Datum.pos;
						posvel.vel = sp3Datum.vel;
                        satdynDatum.sp3orbitList_ECEF.push_back(posvel);
					}
				}
				if(satdynDatum.sp3orbitList_ECEF.size() == sp3file.m_data.size())
				{
					// 初始化轨道初值
					satdynDatum.dynamicDatum_Init.T0 = TimeCoordConvert::GPST2TDT(satdynDatum.sp3orbitList_ECEF[0].t);
                    satdynDatum.dynamicDatum_Init.ArcLength = satdynDatum.sp3orbitList_ECEF[satdynDatum.sp3orbitList_ECEF.size() - 1].t - satdynDatum.sp3orbitList_ECEF[0].t;
					SP3Datum sp3Datum;
					if(!sp3file.getEphemeris(satdynDatum.sp3orbitList_ECEF[0].t, nPRN, sp3Datum, 9 , 'C'))
						return false;
					double x_ecf[6];
					double x_j2000[6];
					x_ecf[0] = sp3Datum.pos.x;  
					x_ecf[1] = sp3Datum.pos.y;  
					x_ecf[2] = sp3Datum.pos.z;
					x_ecf[3] = sp3Datum.vel.x; 
					x_ecf[4] = sp3Datum.vel.y; 
					x_ecf[5] = sp3Datum.vel.z;
					satdynDatum.dynamicDatum_Init.X0_ECEF.x  = sp3Datum.pos.x;
					satdynDatum.dynamicDatum_Init.X0_ECEF.y  = sp3Datum.pos.y;
					satdynDatum.dynamicDatum_Init.X0_ECEF.z  = sp3Datum.pos.z;
					satdynDatum.dynamicDatum_Init.X0_ECEF.vx = sp3Datum.vel.x;
					satdynDatum.dynamicDatum_Init.X0_ECEF.vy = sp3Datum.vel.y;
					satdynDatum.dynamicDatum_Init.X0_ECEF.vz = sp3Datum.vel.z;
					m_TimeCoordConvert.ECEF_J2000(satdynDatum.sp3orbitList_ECEF[0].t, x_j2000, x_ecf);
					satdynDatum.dynamicDatum_Init.X0.x  = x_j2000[0]; 
					satdynDatum.dynamicDatum_Init.X0.y  = x_j2000[1]; 
					satdynDatum.dynamicDatum_Init.X0.z  = x_j2000[2];
					satdynDatum.dynamicDatum_Init.X0.vx = x_j2000[3]; 
					satdynDatum.dynamicDatum_Init.X0.vy = x_j2000[4]; 
					satdynDatum.dynamicDatum_Init.X0.vz = x_j2000[5];
					satdynDatum.dynamicDatum_Init.solarPressureType = TYPE_SOLARPRESSURE_9PARA;
					satdynDatum.dynamicDatum_Init.init(m_podParaDefine.period_SolarPressure); // 20131209, 太阳光压周期由外部参数给定
					//satdynDatum.dynamicDatum_Init.init(satdynDatum.dynamicDatum_Init.ArcLength);
					satdynDatum.dynamicDatum_Est = satdynDatum.dynamicDatum_Init; 
					paraSp3Fit.satParaList.insert(Sp3Fit_SatdynDatumMap::value_type(nPRN, satdynDatum));
				}
			}
			// 获取参考地球旋转参数矩阵 matEP、matER、matPR_NR、matERDOT
			int  count_Epoch = int(sp3file.m_data.size());
			vector<Matrix> matPR_NRList;
			vector<Matrix> matERList_0;
			vector<Matrix> matEPList_0;
			vector<Matrix> matERDOTList;
			vector<TDT> interpTimelist;
			interpTimelist.resize(count_Epoch);
			for(int i = 0; i < count_Epoch; i++) 
			{
				interpTimelist[i] = TimeCoordConvert::GPST2TDT(sp3file.m_data[i].t);
				Matrix matPR_NR, matER, matEP, matER_DOT;
				m_TimeCoordConvert.Matrix_J2000_ECEF(sp3file.m_data[i].t, matPR_NR, matER, matEP, matER_DOT);
				matPR_NRList.push_back(matPR_NR);
				matERList_0.push_back(matER);
				matEPList_0.push_back(matEP);
				matERDOTList.push_back(matER_DOT);
			}
			int  count_solar = int(paraSp3Fit.satParaList.begin()->second.dynamicDatum_Init.solarPressureParaList.size());
			int  count_solar_period = 9; // 每个周期的太阳光压参数个数
			if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
				 count_solar_period  = 5;
			int  count_DynParameter = 6 + count_solar_period * count_solar; // 考虑多组太阳光压参数, 20131209, 谷德峰
			//int  count_DynParameter = 15;
			
			//count_DynParameter = 6 + 5 * count_solar;
			int  k = 0; // 记录迭代的次数
			bool result = true;
			bool last_iterative = false; 
			while(1)
			{
				k++;
				sprintf(info,"第%d次迭代...", k);
				RuningInfoFile::Add(info);
				printf("%s",info);
				if(k >= m_podParaDefine.max_OrbitIterativeNum)
				{
					result = false;
					sprintf(info,"拟合sp3轨道迭代次数溢出(sp3Fit)！");
					RuningInfoFile::Add(info);
					printf("%s\n",info);
					break;
				}
				vector<Matrix> N_orbList; 
				vector<Matrix> N_orb_xpyput1List;
				vector<Matrix> ny_orbList;
				//Matrix N_xpyput1(6, 6);
				//Matrix ny_xpyput1(6, 1);
				Matrix N_xpyput1(5, 5);  // ut1紧约束
				Matrix ny_xpyput1(5, 1); // ut1紧约束
				vector<Matrix> matEst_EPList;
				vector<Matrix> matEst_ERList;
				vector<Matrix> matEPList; // 每次参数估计改进使用
			    vector<Matrix> matERList;
				for(int i = 0; i < count_Epoch; i++) 
				{
					Matrix matEst_EP, matEst_ER;
					paraSp3Fit.getEst_EOP(sp3file.m_data[i].t, matEst_EP, matEst_ER);// 更新 matEP, matER
					matEPList.push_back(matEst_EP * matEPList_0[i]);
					matERList.push_back(matEst_ER * matERList_0[i]);
				}
				if(last_iterative) // 最后一次迭代
				{
					for(Sp3Fit_SatdynDatumMap::iterator it = paraSp3Fit.satParaList.begin(); it != paraSp3Fit.satParaList.end(); ++it)
					{
						// 还原初始轨道位置速度到ECEF坐标系下
						Matrix matJ2000Pos, matJ2000Vel, matECFPos,matECFVel;
						matJ2000Pos.Init(3,1);
						matJ2000Vel.Init(3,1);
						matECFPos.Init(3,1);
						matECFVel.Init(3,1);
						matJ2000Pos.SetElement(0,0,it->second.dynamicDatum_Est.X0.x);
						matJ2000Pos.SetElement(1,0,it->second.dynamicDatum_Est.X0.y);
						matJ2000Pos.SetElement(2,0,it->second.dynamicDatum_Est.X0.z);
						matJ2000Vel.SetElement(0,0,it->second.dynamicDatum_Est.X0.vx);
						matJ2000Vel.SetElement(1,0,it->second.dynamicDatum_Est.X0.vy);
						matJ2000Vel.SetElement(2,0,it->second.dynamicDatum_Est.X0.vz);
						matECFPos = matPR_NRList[0] * matJ2000Pos;
                        matECFVel = matPR_NRList[0] * matJ2000Vel;
						matECFVel = matERList[0] *  matECFVel + matERDOTList[0] * matECFPos;
						matECFPos = matERList[0] *  matECFPos;
						matECFPos = matEPList[0] *  matECFPos;
						matECFVel = matEPList[0] *  matECFVel;
						it->second.dynamicDatum_Est.X0_ECEF.x  = matECFPos.GetElement(0, 0);
						it->second.dynamicDatum_Est.X0_ECEF.y  = matECFPos.GetElement(1, 0);
						it->second.dynamicDatum_Est.X0_ECEF.z  = matECFPos.GetElement(2, 0);
						it->second.dynamicDatum_Est.X0_ECEF.vx = matECFVel.GetElement(0, 0);
						it->second.dynamicDatum_Est.X0_ECEF.vy = matECFVel.GetElement(1, 0);
						it->second.dynamicDatum_Est.X0_ECEF.vz = matECFVel.GetElement(2, 0);
						vector<TimePosVel> interpOrbitlist; // 插值序列
						vector<Matrix> interpRtPartiallist; // 插值偏导数序列
						adamsCowell_Interp(interpTimelist, it->second.dynamicDatum_Est, interpOrbitlist, interpRtPartiallist);
						it->second.fitorbitList_ECEF.clear();
						for(int i = 0; i < int(interpOrbitlist.size()); i++)
						{
							TimePosVel posvel;
							posvel.t = it->second.sp3orbitList_ECEF[i].t;
							matJ2000Pos.Init(3,1);
							matJ2000Vel.Init(3,1);
							matECFPos.Init(3,1);
							matECFVel.Init(3,1);
							matJ2000Pos.SetElement(0,0,interpOrbitlist[i].pos.x);
							matJ2000Pos.SetElement(1,0,interpOrbitlist[i].pos.y);
							matJ2000Pos.SetElement(2,0,interpOrbitlist[i].pos.z);
							matJ2000Vel.SetElement(0,0,interpOrbitlist[i].vel.x);
							matJ2000Vel.SetElement(1,0,interpOrbitlist[i].vel.y);
							matJ2000Vel.SetElement(2,0,interpOrbitlist[i].vel.z);
							matECFPos = matPR_NRList[i] * matJ2000Pos;
							matECFVel = matPR_NRList[i] * matJ2000Vel;
							matECFVel = matERList[i] *  matECFVel + matERDOTList[i] * matECFPos;
							matECFPos = matERList[i] *  matECFPos;
							matECFPos = matEPList[i] *  matECFPos;
							matECFVel = matEPList[i] *  matECFVel;
							posvel.pos.x = matECFPos.GetElement(0, 0);
							posvel.pos.y = matECFPos.GetElement(1, 0);
							posvel.pos.z = matECFPos.GetElement(2, 0);
							posvel.vel.x = matECFVel.GetElement(0, 0);
							posvel.vel.y = matECFVel.GetElement(1, 0);
							posvel.vel.z = matECFVel.GetElement(2, 0);
							it->second.fitorbitList_ECEF.push_back(posvel);
						}
					}
					break;
				}
				int count_oc = 0;
				double rms_oc = 0;
				//FILE * pTestFile = fopen("c:\\test.txt", "w+");
				for(Sp3Fit_SatdynDatumMap::iterator it = paraSp3Fit.satParaList.begin(); it != paraSp3Fit.satParaList.end(); ++it)
				{
					Matrix N_orb(count_DynParameter, count_DynParameter);
					//Matrix N_orb_xpyput1(count_DynParameter, 6);
					Matrix N_orb_xpyput1(count_DynParameter, 5); // ut1紧约束
					Matrix ny_orb(count_DynParameter, 1);
					vector<TimePosVel> interpOrbitlist; // 插值序列
					vector<Matrix> interpRtPartiallist; // 插值偏导数序列
					adamsCowell_Interp(interpTimelist, it->second.dynamicDatum_Est, interpOrbitlist, interpRtPartiallist);
					for(int i = 0; i < int(it->second.sp3orbitList_ECEF.size()); i++)
					{
						Matrix matHt_Orb(3, count_DynParameter); 
						//Matrix matHt_xpyput1(3, 6); // ut1紧约束
						Matrix matHt_xpyput1(3, 5); // 分成两部分来求,  xpyp和ut1 
						Matrix matYt(3, 1);
						// 轨道力学参数设计矩阵
						if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_9PARA)
						{
							for(int j = 0; j < count_DynParameter; j++)
							{
								matHt_Orb.SetElement(0, j, interpRtPartiallist[i].GetElement(0, j));
								matHt_Orb.SetElement(1, j, interpRtPartiallist[i].GetElement(1, j));
								matHt_Orb.SetElement(2, j, interpRtPartiallist[i].GetElement(2, j));
							}
						}
						else if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
						{
							for(int j = 0; j < 6; j++)
							{
								matHt_Orb.SetElement(0, j, interpRtPartiallist[i].GetElement(0, j));
								matHt_Orb.SetElement(1, j, interpRtPartiallist[i].GetElement(1, j));
								matHt_Orb.SetElement(2, j, interpRtPartiallist[i].GetElement(2, j));
							}
							for(size_t s_k = 0; s_k < it->second.dynamicDatum_Est.solarPressureParaList.size(); s_k++)
							{								
								matHt_Orb.SetElement(0, 6 + 5 * (int)s_k + 0, interpRtPartiallist[i].GetElement(0, 6 + 9 * (int)s_k + 0));
								matHt_Orb.SetElement(1, 6 + 5 * (int)s_k + 0, interpRtPartiallist[i].GetElement(1, 6 + 9 * (int)s_k + 0));
								matHt_Orb.SetElement(2, 6 + 5 * (int)s_k + 0, interpRtPartiallist[i].GetElement(2, 6 + 9 * (int)s_k + 0));
								matHt_Orb.SetElement(0, 6 + 5 * (int)s_k + 1, interpRtPartiallist[i].GetElement(0, 6 + 9 * (int)s_k + 3));
								matHt_Orb.SetElement(1, 6 + 5 * (int)s_k + 1, interpRtPartiallist[i].GetElement(1, 6 + 9 * (int)s_k + 3));
								matHt_Orb.SetElement(2, 6 + 5 * (int)s_k + 1, interpRtPartiallist[i].GetElement(2, 6 + 9 * (int)s_k + 3));
								for(int j = 2; j < 5; j++)
								{
									matHt_Orb.SetElement(0, 6 + 5 * (int)s_k + j, interpRtPartiallist[i].GetElement(0, 6 + 9 * (int)s_k + j + 4));
									matHt_Orb.SetElement(1, 6 + 5 * (int)s_k + j, interpRtPartiallist[i].GetElement(1, 6 + 9 * (int)s_k + j + 4));
									matHt_Orb.SetElement(2, 6 + 5 * (int)s_k + j, interpRtPartiallist[i].GetElement(2, 6 + 9 * (int)s_k + j + 4));
								}							
							}
						}
						// matPRNR-1 × matER-1 × matEP-1 × matECFPos_C = matJ2000Pos
						Matrix matH = matEPList[i] * matERList[i] * matPR_NRList[i]; // J2000->ECEF 矩阵
						Matrix matJ2000Pos_O(3, 1);
						matJ2000Pos_O.SetElement(0, 0, it->second.sp3orbitList_ECEF[i].pos.x);
						matJ2000Pos_O.SetElement(1, 0, it->second.sp3orbitList_ECEF[i].pos.y);
						matJ2000Pos_O.SetElement(2, 0, it->second.sp3orbitList_ECEF[i].pos.z);
						matJ2000Pos_O = matH.Transpose() * matJ2000Pos_O; // ECEF->J2000
						// O-C 残差
						matYt.SetElement(0, 0, matJ2000Pos_O.GetElement(0, 0) - interpOrbitlist[i].pos.x);
						matYt.SetElement(1, 0, matJ2000Pos_O.GetElement(1, 0) - interpOrbitlist[i].pos.y);
						matYt.SetElement(2, 0, matJ2000Pos_O.GetElement(2, 0) - interpOrbitlist[i].pos.z);
						//fprintf(pTestFile, "%2d  %s\n", it->first, matYt.Transpose().ToString().c_str());
				        
						rms_oc += (matYt.Transpose() * matYt).GetElement(0, 0);
						count_oc += 3;
						// 地球旋转参数设计矩阵
		                Matrix matHt_xpyp(3, 4);
						Matrix matECFPos_C(3, 1); // matECFPos_C = matEP × matER × matNR × matPR × matJ2000Pos
						matECFPos_C.SetElement(0, 0, interpOrbitlist[i].pos.x);
						matECFPos_C.SetElement(1, 0, interpOrbitlist[i].pos.y);
						matECFPos_C.SetElement(2, 0, interpOrbitlist[i].pos.z);
						matECFPos_C = matH * matECFPos_C;
						double spanSeconds = it->second.sp3orbitList_ECEF[i].t - paraSp3Fit.t0_xpyput1;
						// xp
						matHt_xpyp.SetElement(0, 0,  matECFPos_C.GetElement(2, 0)); 
						matHt_xpyp.SetElement(1, 0,  0);
						matHt_xpyp.SetElement(2, 0, -matECFPos_C.GetElement(0, 0));
						// xpDot
						matHt_xpyp.SetElement(0, 1,  matECFPos_C.GetElement(2, 0) * spanSeconds); 
						matHt_xpyp.SetElement(1, 1,  0);
						matHt_xpyp.SetElement(2, 1, -matECFPos_C.GetElement(0, 0) * spanSeconds);
						// yp
						matHt_xpyp.SetElement(0, 2,  0); 
						matHt_xpyp.SetElement(1, 2, -matECFPos_C.GetElement(2, 0));
						matHt_xpyp.SetElement(2, 2,  matECFPos_C.GetElement(1, 0));
						// ypDot
						matHt_xpyp.SetElement(0, 3,  0); 
						matHt_xpyp.SetElement(1, 3, -matECFPos_C.GetElement(2, 0) * spanSeconds);
						matHt_xpyp.SetElement(2, 3,  matECFPos_C.GetElement(1, 0) * spanSeconds);
						matHt_xpyp = matH.Transpose() * matHt_xpyp; // ECEF->J2000
						Matrix matHt_ut1(3, 2);
						// ut1
						matHt_ut1.SetElement(0, 0,  matECFPos_C.GetElement(1, 0)); 
						matHt_ut1.SetElement(1, 0, -matECFPos_C.GetElement(0, 0));
						// ut1Dot
						matHt_ut1.SetElement(0, 1,  matECFPos_C.GetElement(1, 0) * spanSeconds); 
						matHt_ut1.SetElement(1, 1, -matECFPos_C.GetElement(0, 0) * spanSeconds);
                        matHt_ut1 = matH.Transpose() * matEPList[i] * matHt_ut1; // ECEF->J2000
                        // 将matHt_xpyp和matHt_ut1合并为matHt_xpyput1
						for(int ii = 0; ii < 3; ii++)
						{
							for(int jj = 0; jj < 4; jj++)
							{
								matHt_xpyput1.SetElement(ii, jj, matHt_xpyp.GetElement(ii, jj));
							}
							for(int jj = 0; jj < 1; jj++) // ut1紧约束
							{
								matHt_xpyput1.SetElement(ii, jj + 4, matHt_ut1.GetElement(ii, jj + 1));
							}
						}
						N_orb = N_orb + matHt_Orb.Transpose() * matHt_Orb;
						N_xpyput1 = N_xpyput1 + matHt_xpyput1.Transpose() * matHt_xpyput1;
						N_orb_xpyput1 = N_orb_xpyput1 + matHt_Orb.Transpose() * matHt_xpyput1;
						ny_orb = ny_orb + matHt_Orb.Transpose() * matYt;
						ny_xpyput1 = ny_xpyput1 + matHt_xpyput1.Transpose() * matYt;
					}
					N_orbList.push_back(N_orb);
					N_orb_xpyput1List.push_back(N_orb_xpyput1);
					ny_orbList.push_back(ny_orb);
				}

				/*fclose(pTestFile);
				return true;*/
				rms_oc = sqrt(rms_oc / count_oc);
				sprintf(info,",  rms_oc  = %8.4f.", rms_oc);
				RuningInfoFile::Add(info);
				printf("%s\n",info);
				// 开始轨道改进
				//Matrix n_xo_oo_inv_ny(6, 1);
				//Matrix n_xo_oo_inv_ox(6, 6);
				Matrix n_xo_oo_inv_ny(5, 1); // ut1紧约束
				Matrix n_xo_oo_inv_ox(5, 5); // ut1紧约束
				for(int i = 0; i < int(N_orbList.size()); i++)
				{
					Matrix n_xo_oo_inv_i = N_orb_xpyput1List[i].Transpose() * N_orbList[i].Inv_Ssgj();
					n_xo_oo_inv_ny = n_xo_oo_inv_ny + n_xo_oo_inv_i * ny_orbList[i];
					n_xo_oo_inv_ox = n_xo_oo_inv_ox + n_xo_oo_inv_i * N_orb_xpyput1List[i];
				}
				Matrix matdx;
				if(bOnEst_EOP)
				{
					matdx = (N_xpyput1 - n_xo_oo_inv_ox).Inv_Ssgj() * (ny_xpyput1 - n_xo_oo_inv_ny);
					paraSp3Fit.xp     += matdx.GetElement(0, 0);
					paraSp3Fit.xpDot  += matdx.GetElement(1, 0);
					paraSp3Fit.yp     += matdx.GetElement(2, 0);
					paraSp3Fit.ypDot  += matdx.GetElement(3, 0);
					//paraSp3Fit.ut1  += matdx.GetElement(4, 0);  // ut1紧约束
					//paraSp3Fit.ut1Dot += matdx.GetElement(5, 0); // ut1紧约束
					paraSp3Fit.ut1Dot += matdx.GetElement(4, 0);
					int i = 0;
					for(Sp3Fit_SatdynDatumMap::iterator it = paraSp3Fit.satParaList.begin(); it != paraSp3Fit.satParaList.end(); ++it)
					{
						Matrix matdo_i = N_orbList[i].Inv_Ssgj() * (ny_orbList[i] - N_orb_xpyput1List[i] * matdx);
						it->second.dynamicDatum_Est.X0.x  += matdo_i.GetElement(0, 0);
						it->second.dynamicDatum_Est.X0.y  += matdo_i.GetElement(1, 0);
						it->second.dynamicDatum_Est.X0.z  += matdo_i.GetElement(2, 0);
						it->second.dynamicDatum_Est.X0.vx += matdo_i.GetElement(3, 0);
						it->second.dynamicDatum_Est.X0.vy += matdo_i.GetElement(4, 0);
						it->second.dynamicDatum_Est.X0.vz += matdo_i.GetElement(5, 0);
						for(size_t s_k = 0; s_k < it->second.dynamicDatum_Est.solarPressureParaList.size(); s_k++)
					    {// 考虑多组太阳光压参数, 20131209, 谷德峰
							if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_9PARA)
							{
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].D0 += matdo_i.GetElement( 6 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].DC1 += matdo_i.GetElement( 7 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].DS1 += matdo_i.GetElement( 8 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].Y0 += matdo_i.GetElement( 9 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].YC1 += matdo_i.GetElement(10 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].YS1 += matdo_i.GetElement(11 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].B0 += matdo_i.GetElement(12 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].BC1 += matdo_i.GetElement(13 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].BS1 += matdo_i.GetElement(14 + int(s_k) * count_solar_period, 0);
							}
							else if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
							{							
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].D0 += matdo_i.GetElement( 6 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].Y0 += matdo_i.GetElement( 7 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].B0 += matdo_i.GetElement( 8 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].BC1 += matdo_i.GetElement( 9 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].BS1 += matdo_i.GetElement(10 + int(s_k) * count_solar_period, 0);
							}
					    }
						i++;						
					}
				}
				else
				{
					int i = 0;
					for(Sp3Fit_SatdynDatumMap::iterator it = paraSp3Fit.satParaList.begin(); it != paraSp3Fit.satParaList.end(); ++it)
					{
						Matrix matdo_i = N_orbList[i].Inv_Ssgj() * ny_orbList[i];
						it->second.dynamicDatum_Est.X0.x  += matdo_i.GetElement(0, 0);
						it->second.dynamicDatum_Est.X0.y  += matdo_i.GetElement(1, 0);
						it->second.dynamicDatum_Est.X0.z  += matdo_i.GetElement(2, 0);
						it->second.dynamicDatum_Est.X0.vx += matdo_i.GetElement(3, 0);
						it->second.dynamicDatum_Est.X0.vy += matdo_i.GetElement(4, 0);
						it->second.dynamicDatum_Est.X0.vz += matdo_i.GetElement(5, 0);
						for(size_t s_k = 0; s_k < it->second.dynamicDatum_Est.solarPressureParaList.size(); s_k++)
					    {// 考虑多组太阳光压参数, 20131209, 谷德峰
							if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_9PARA)
							{
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].D0 += matdo_i.GetElement( 6 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].DC1 += matdo_i.GetElement( 7 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].DS1 += matdo_i.GetElement( 8 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].Y0 += matdo_i.GetElement( 9 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].YC1 += matdo_i.GetElement(10 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].YS1 += matdo_i.GetElement(11 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].B0 += matdo_i.GetElement(12 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].BC1 += matdo_i.GetElement(13 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].BS1 += matdo_i.GetElement(14 + int(s_k) * count_solar_period, 0);
								
							}
							else if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
							{							
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].D0 += matdo_i.GetElement( 6 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].Y0 += matdo_i.GetElement( 7 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].B0 += matdo_i.GetElement( 8 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].BC1 += matdo_i.GetElement( 9 + int(s_k) * count_solar_period, 0);
								it->second.dynamicDatum_Est.solarPressureParaList[s_k].BS1 += matdo_i.GetElement(10 + int(s_k) * count_solar_period, 0);
							}
					    }
						i++;						
					}
				}
			    if(k >= 2)
				{
					last_iterative = true;
				}
			}
			// 外推轨道, 并检核拟合精度
			char sp3RmsFilePath[300];
			sprintf(sp3RmsFilePath,"%s\\sp3fit_%s.rms", folder.c_str(), sp3FileName_noexp.c_str());
			FILE * pRmsFile = fopen(sp3RmsFilePath, "w+");
			fprintf(pRmsFile, "PRN     Total   delta-X   delta-Y   delta-Z  d-Radial   d-Along   d-Cross\n");
			char sp3FitFilePath[300];
			sprintf(sp3FitFilePath,"%s\\sp3fit_%s.fit", folder.c_str(), sp3FileName_noexp.c_str());
			FILE * pFitFile = fopen(sp3FitFilePath, "w+");
		    fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
			fprintf(pFitFile, "%3d. EOP  XP   (mas)    %20.4f%10.4f%20.4f\n",  1, 0.0, paraSp3Fit.xp * 180 / PI * 3600000,                      paraSp3Fit.xp * 180 / PI * 3600000);
			fprintf(pFitFile, "%3d. EOP  XPDOT(mas/d)  %20.4f%10.4f%20.4f\n",  2, 0.0, paraSp3Fit.xpDot * 86400.0  * 180 / PI * 3600000,        paraSp3Fit.xpDot * 86400.0  * 180 / PI * 3600000);
			fprintf(pFitFile, "%3d. EOP  YP   (mas)    %20.4f%10.4f%20.4f\n",  3, 0.0, paraSp3Fit.yp * 180 / PI * 3600000,                      paraSp3Fit.yp * 180 / PI * 3600000);
			fprintf(pFitFile, "%3d. EOP  YPDOT(mas/d)  %20.4f%10.4f%20.4f\n",  4, 0.0, paraSp3Fit.ypDot * 86400.0  * 180 / PI * 3600000,        paraSp3Fit.ypDot * 86400.0  * 180 / PI * 3600000);
			fprintf(pFitFile, "%3d. EOP  UT   (ms)     %20.4f%10.4f%20.4f\n",  5, 0.0, paraSp3Fit.ut1 * 86400.0 / (2 * PI) * 1.0E+3,            paraSp3Fit.ut1 * 86400.0 / (2 * PI) * 1.0E+3);
			fprintf(pFitFile, "%3d. EOP  UTDOT(ms/d)   %20.4f%10.4f%20.4f\n",  6, 0.0, paraSp3Fit.ut1Dot * 86400.0 / (2 * PI) * 1.0E+3 * 86400, paraSp3Fit.ut1Dot * 86400.0 / (2 * PI) * 1.0E+3 * 86400);
			paraSp3Fit.meanFitRms_X = 0;
			paraSp3Fit.meanFitRms_Y = 0;
			paraSp3Fit.meanFitRms_Z = 0;
			paraSp3Fit.meanFitRms_R = 0;
			paraSp3Fit.meanFitRms_T = 0;
			paraSp3Fit.meanFitRms_N = 0;
			paraSp3Fit.meanFitRms_Total = 0;
			int k_Parameter = 6;
			for(Sp3Fit_SatdynDatumMap::iterator it = paraSp3Fit.satParaList.begin(); it != paraSp3Fit.satParaList.end(); ++it)
			{
				fprintf(pFitFile, "\n");
				fprintf(pFitFile, "%3d. PN%2d X    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 1, 
					                                                               it->first,
					                                                               it->second.dynamicDatum_Init.X0_ECEF.x, 
																				   it->second.dynamicDatum_Est.X0_ECEF.x - it->second.dynamicDatum_Init.X0_ECEF.x, 
																				   it->second.dynamicDatum_Est.X0_ECEF.x);
				fprintf(pFitFile, "%3d. PN%2d Y    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 2, 
					                                                               it->first,
					                                                               it->second.dynamicDatum_Init.X0_ECEF.y, 
																				   it->second.dynamicDatum_Est.X0_ECEF.y - it->second.dynamicDatum_Init.X0_ECEF.y, 
																				   it->second.dynamicDatum_Est.X0_ECEF.y);
				fprintf(pFitFile, "%3d. PN%2d Z    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 3,  
					                                                               it->first,
					                                                               it->second.dynamicDatum_Init.X0_ECEF.z, 
																				   it->second.dynamicDatum_Est.X0_ECEF.z - it->second.dynamicDatum_Init.X0_ECEF.z, 
																				   it->second.dynamicDatum_Est.X0_ECEF.z);
				fprintf(pFitFile, "%3d. PN%2d XDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 4,  
					                                                               it->first,
					                                                               it->second.dynamicDatum_Init.X0_ECEF.vx, 
																				   it->second.dynamicDatum_Est.X0_ECEF.vx - it->second.dynamicDatum_Init.X0_ECEF.vx, 
																				   it->second.dynamicDatum_Est.X0_ECEF.vx);
				fprintf(pFitFile, "%3d. PN%2d YDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 5,  
					                                                               it->first,
					                                                               it->second.dynamicDatum_Init.X0_ECEF.vy, 
																				it->second.dynamicDatum_Est.X0_ECEF.vy - it->second.dynamicDatum_Init.X0_ECEF.vy, 
																				it->second.dynamicDatum_Est.X0_ECEF.vy);
				fprintf(pFitFile, "%3d. PN%2d ZDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 6,  
					                                                               it->first,
					                                                               it->second.dynamicDatum_Init.X0_ECEF.vz, 
																				   it->second.dynamicDatum_Est.X0_ECEF.vz - it->second.dynamicDatum_Init.X0_ECEF.vz, 
																				   it->second.dynamicDatum_Est.X0_ECEF.vz);
				for(size_t s_k = 0; s_k < it->second.dynamicDatum_Est.solarPressureParaList.size(); s_k++)
				{// 考虑多组太阳光压参数, 20131209, 谷德峰
					fprintf(pFitFile, "%3d. PN%2d D0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 7 + s_k * 9,  
																					   it->first,
																					   it->second.dynamicDatum_Init.solarPressureParaList[s_k].D0 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].D0 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_k].D0 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].D0 * 1.0E+7);
					fprintf(pFitFile, "%3d. PN%2d DCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 8 + s_k * 9,  
																					   it->first,
																					   it->second.dynamicDatum_Init.solarPressureParaList[s_k].DC1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].DC1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_k].DC1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].DC1 * 1.0E+7);
					fprintf(pFitFile, "%3d. PN%2d DSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 9 + s_k * 9,  
																					   it->first,
																					   it->second.dynamicDatum_Init.solarPressureParaList[s_k].DS1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].DS1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_k].DS1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].DS1 * 1.0E+7);
					fprintf(pFitFile, "%3d. PN%2d Y0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 10 + s_k * 9,  
																					   it->first,
																					   it->second.dynamicDatum_Init.solarPressureParaList[s_k].Y0 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].Y0 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_k].Y0 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].Y0 * 1.0E+7);
					fprintf(pFitFile, "%3d. PN%2d YCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 11 + s_k * 9,  
																					   it->first,
																					   it->second.dynamicDatum_Init.solarPressureParaList[s_k].YC1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].YC1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_k].YC1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].YC1 * 1.0E+7);
					fprintf(pFitFile, "%3d. PN%2d YSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 12 + s_k * 9,  
																					   it->first,
																					   it->second.dynamicDatum_Init.solarPressureParaList[s_k].YS1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].YS1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_k].YS1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].YS1 * 1.0E+7);
					fprintf(pFitFile, "%3d. PN%2d X0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 13 + s_k * 9,  
																					   it->first,
																					   it->second.dynamicDatum_Init.solarPressureParaList[s_k].B0 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].B0 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_k].B0 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].B0 * 1.0E+7);
					fprintf(pFitFile, "%3d. PN%2d XCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 14 + s_k * 9,  
																					   it->first,
																					   it->second.dynamicDatum_Init.solarPressureParaList[s_k].BC1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].BC1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_k].BC1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].BC1 * 1.0E+7);
					fprintf(pFitFile, "%3d. PN%2d XSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 15 + s_k * 9,  
																					   it->first,
																					   it->second.dynamicDatum_Init.solarPressureParaList[s_k].BS1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].BS1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_k].BS1 * 1.0E+7, 
																					   it->second.dynamicDatum_Est.solarPressureParaList[s_k].BS1 * 1.0E+7);
				}				
				k_Parameter = k_Parameter + count_DynParameter;

				it->second.fitrms_X = 0;
				it->second.fitrms_Y = 0;
				it->second.fitrms_Z = 0;
				it->second.fitrms_R = 0;
				it->second.fitrms_T = 0;
				it->second.fitrms_N = 0;				

				for(int i = 0; i < int(it->second.sp3orbitList_ECEF.size()); i++)
				{
					POS3D error = it->second.fitorbitList_ECEF[i].pos - it->second.sp3orbitList_ECEF[i].pos;
					it->second.fitrms_X += pow(error.x, 2);
				    it->second.fitrms_Y += pow(error.y, 2);
				    it->second.fitrms_Z += pow(error.z, 2);

					POS3D S_R;
					POS3D S_T;
					POS3D S_N;
					TimeCoordConvert::getCoordinateRTNAxisVector(m_TimeCoordConvert.GPST2UT1(it->second.sp3orbitList_ECEF[i].t), it->second.sp3orbitList_ECEF[i].getPosVel(), S_R, S_T, S_N);
					POS3D error_RTN;
					error_RTN.x  = vectorDot(error, S_R);
					error_RTN.y  = vectorDot(error, S_T);
					error_RTN.z  = vectorDot(error, S_N);
					it->second.fitrms_R += pow(error_RTN.x, 2);
					it->second.fitrms_T += pow(error_RTN.y, 2);
					it->second.fitrms_N += pow(error_RTN.z, 2);
				}
				it->second.fitrms_X = sqrt(it->second.fitrms_X / it->second.sp3orbitList_ECEF.size());
				it->second.fitrms_Y = sqrt(it->second.fitrms_Y / it->second.sp3orbitList_ECEF.size());
				it->second.fitrms_Z = sqrt(it->second.fitrms_Z / it->second.sp3orbitList_ECEF.size());
				it->second.fitrms_R = sqrt(it->second.fitrms_R / it->second.sp3orbitList_ECEF.size());
				it->second.fitrms_T = sqrt(it->second.fitrms_T / it->second.sp3orbitList_ECEF.size());
				it->second.fitrms_N = sqrt(it->second.fitrms_N / it->second.sp3orbitList_ECEF.size());
				it->second.fitrms_Total = sqrt(it->second.fitrms_X * it->second.fitrms_X 
					                         + it->second.fitrms_Y * it->second.fitrms_Y 
											 + it->second.fitrms_Z * it->second.fitrms_Z);

				fprintf(pRmsFile, "%3d %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f\n", it->first,
				                                                                  it->second.fitrms_Total,
																	              it->second.fitrms_X,
																				  it->second.fitrms_Y,
																				  it->second.fitrms_Z,
																				  it->second.fitrms_R,
																				  it->second.fitrms_T,
																				  it->second.fitrms_N);

				paraSp3Fit.meanFitRms_X += it->second.fitrms_X;
				paraSp3Fit.meanFitRms_Y += it->second.fitrms_Y;
				paraSp3Fit.meanFitRms_Z += it->second.fitrms_Z;
				paraSp3Fit.meanFitRms_R += it->second.fitrms_R;
				paraSp3Fit.meanFitRms_T += it->second.fitrms_T;
				paraSp3Fit.meanFitRms_N += it->second.fitrms_N;
				paraSp3Fit.meanFitRms_Total += it->second.fitrms_Total;
			}
			paraSp3Fit.meanFitRms_X /= paraSp3Fit.satParaList.size();
			paraSp3Fit.meanFitRms_Y /= paraSp3Fit.satParaList.size();
			paraSp3Fit.meanFitRms_Z /= paraSp3Fit.satParaList.size();
			paraSp3Fit.meanFitRms_R /= paraSp3Fit.satParaList.size();
			paraSp3Fit.meanFitRms_T /= paraSp3Fit.satParaList.size();
			paraSp3Fit.meanFitRms_N /= paraSp3Fit.satParaList.size();
			paraSp3Fit.meanFitRms_Total /= paraSp3Fit.satParaList.size();
			fprintf(pRmsFile, "=========================================================================\n");
			fprintf(pRmsFile, "MEAN%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f\n", paraSp3Fit.meanFitRms_Total,
				                                                                 paraSp3Fit.meanFitRms_X, 
																			     paraSp3Fit.meanFitRms_Y,
																			     paraSp3Fit.meanFitRms_Z,
																			     paraSp3Fit.meanFitRms_R,
																			     paraSp3Fit.meanFitRms_T,
																			     paraSp3Fit.meanFitRms_N);
			fclose(pRmsFile);
			fclose(pFitFile);	
			// 清理拟合超差的卫星
			Sp3Fit_SatdynDatumMap::iterator it_sat = paraSp3Fit.satParaList.begin();
			while(it_sat != paraSp3Fit.satParaList.end())
			{
				if(it_sat->second.fitrms_Total > m_podParaDefine.max_FitRms_Total)
				{
					Sp3Fit_SatdynDatumMap::iterator jt = it_sat;
					++it_sat;
					paraSp3Fit.satParaList.erase(jt);					
				}
				else
					++it_sat;
			}	//		

			// 2. sp3文件
			GPST t0 = sp3file.m_data.front().t;
			GPST t1 = sp3file.m_data.back().t;
			char outputSp3FilePath[100];
			sprintf(outputSp3FilePath,"%s\\%sfit.sp3",folder.c_str(),sp3FileName_noexp.c_str());
			double h_sp3 = sp3file.getEpochSpan();
			TDT t0_tdt = TimeCoordConvert::GPST2TDT(t0);
			TDT t1_tdt = TimeCoordConvert::GPST2TDT(t1);
			SP3File ndtSp3File;
			ndtSp3File.m_header.szSP3Version;
			sprintf(ndtSp3File.m_header.szSP3Version, "%2s","#c");
			sprintf(ndtSp3File.m_header.szPosVelFlag, "%1s","P");
			ndtSp3File.m_header.tmStart = t0;
			ndtSp3File.m_header.nNumberofEpochs = int((t1 - t0) / h_sp3) + 1;
			sprintf(ndtSp3File.m_header.szDataType, "%-5s","d+D");
			sprintf(ndtSp3File.m_header.szCoordinateSys, "%-5s","IGS08");
			sprintf(ndtSp3File.m_header.szOrbitType, "%-3s","FIT");
			sprintf(ndtSp3File.m_header.szAgency, "%-4s","NUDT");
			sprintf(ndtSp3File.m_header.szLine2Symbols, "%-2s","##");
			ndtSp3File.m_header.tmGPSWeek = TimeCoordConvert::GPST2WeekTime(t0);
			ndtSp3File.m_header.dEpochInterval = h_sp3;
			double dMJD = TimeCoordConvert::DayTime2MJD(t0);
			ndtSp3File.m_header.nModJulDaySt = long(floor(dMJD));    
			ndtSp3File.m_header.dFractionalDay = dMJD - ndtSp3File.m_header.nModJulDaySt;
			sprintf(ndtSp3File.m_header.szLine3Symbols, "%-2s","+ ");
			sprintf(ndtSp3File.m_header.szLine8Symbols, "%-2s","++");
			ndtSp3File.m_header.bNumberofSats = BYTE(paraSp3Fit.satParaList.size());
			ndtSp3File.m_header.pstrSatNameList.clear();
			ndtSp3File.m_header.pbySatAccuracyList.clear();
			for(Sp3Fit_SatdynDatumMap::iterator it = paraSp3Fit.satParaList.begin(); it != paraSp3Fit.satParaList.end(); ++it)
			{	
				char SatName[4];
				sprintf(SatName, "C%02d", it->first);
				ndtSp3File.m_header.pstrSatNameList.push_back(SatName);
				ndtSp3File.m_header.pbySatAccuracyList.push_back(3);
			}
			sprintf(ndtSp3File.m_header.szLine13Symbols, "%%c");
			sprintf(ndtSp3File.m_header.szFileType, "%-2s","C ");
			sprintf(ndtSp3File.m_header.szTimeSystem, "GPS");
			sprintf(ndtSp3File.m_header.szLine15Symbols, "%%f");
			ndtSp3File.m_header.dBaseforPosVel  = 0;
			ndtSp3File.m_header.dBaseforClkRate = 0;
			sprintf(ndtSp3File.m_header.szLine17Symbols, "%%i");
			sprintf(ndtSp3File.m_header.szLine19Symbols, "/*");
			sprintf(ndtSp3File.m_header.szLine19Comment, "%-57s", "National University of Defense Technology (NUDT).");
			ndtSp3File.m_data.clear();
			if(result)
			{
				for(Sp3Fit_SatdynDatumMap::iterator it = paraSp3Fit.satParaList.begin(); it != paraSp3Fit.satParaList.end(); ++it)
				{
					vector<TimePosVel> orbitlist_ac;
					vector<Matrix> matRtPartiallist_ac;
					double h = 75.0;
					adamsCowell_ac(t0_tdt, t1_tdt, it->second.dynamicDatum_Est, orbitlist_ac, matRtPartiallist_ac, h);
					int k = 0;
					double span = t1_tdt - t0_tdt;
					it->second.fitorbitList_ECEF.clear();
					while(k * h_sp3 < span)             
					{
						TimePosVel point;
						point.t = t0_tdt + k * h_sp3;
						it->second.fitorbitList_ECEF.push_back(point);
						k++;
					}
					size_t count_ac = orbitlist_ac.size();
					const int nlagrange = 8; 
					if(count_ac < nlagrange) // 如果数据点个数小于nlagrange返回，要求弧段长度 > h * nlagrange = 4分钟
						return false;
					for(size_t s_i = 0; s_i < it->second.fitorbitList_ECEF.size(); s_i++)
					{
						double spanSecond_t = it->second.fitorbitList_ECEF[s_i].t - orbitlist_ac[0].t; 
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
						interpOrbit.t = it->second.fitorbitList_ECEF[s_i].t;
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
						it->second.fitorbitList_ECEF[s_i] = interpOrbit;
						delete x;
						delete y;
					}
					// 转换到地球固定坐标系, 坐标系: ITRF 系, 时间: GPS
					for(size_t s_i = 0; s_i < it->second.fitorbitList_ECEF.size(); s_i++)
					{
						/*double x_ecf[6];
						double x_j2000[6];
						x_j2000[0] = it->second.orbitList_ECEF[s_i].pos.x;  
						x_j2000[1] = it->second.orbitList_ECEF[s_i].pos.y;  
						x_j2000[2] = it->second.orbitList_ECEF[s_i].pos.z;
						x_j2000[3] = it->second.orbitList_ECEF[s_i].vel.x; 
						x_j2000[4] = it->second.orbitList_ECEF[s_i].vel.y; 
						x_j2000[5] = it->second.orbitList_ECEF[s_i].vel.z;
						it->second.orbitList_ECEF[s_i].t = TimeCoordConvert::TDT2GPST(it->second.orbitList_ECEF[s_i].t);
						m_TimeCoordConvert.J2000_ECEF(it->second.orbitList_ECEF[s_i].t, x_j2000, x_ecf);
						it->second.orbitList_ECEF[s_i].pos.x = x_ecf[0]; 
						it->second.orbitList_ECEF[s_i].pos.y = x_ecf[1]; 
						it->second.orbitList_ECEF[s_i].pos.z = x_ecf[2];
						it->second.orbitList_ECEF[s_i].vel.x = x_ecf[3]; 
						it->second.orbitList_ECEF[s_i].vel.y = x_ecf[4]; 
						it->second.orbitList_ECEF[s_i].vel.z = x_ecf[5];*/
						Matrix matJ2000Pos, matJ2000Vel, matECFPos,matECFVel;
						matJ2000Pos.Init(3,1);
						matJ2000Vel.Init(3,1);
						matECFPos.Init(3,1);
						matECFVel.Init(3,1);
						matJ2000Pos.SetElement(0,0,it->second.fitorbitList_ECEF[s_i].pos.x);
						matJ2000Pos.SetElement(1,0,it->second.fitorbitList_ECEF[s_i].pos.y);
						matJ2000Pos.SetElement(2,0,it->second.fitorbitList_ECEF[s_i].pos.z);
						matJ2000Vel.SetElement(0,0,it->second.fitorbitList_ECEF[s_i].vel.x);
						matJ2000Vel.SetElement(1,0,it->second.fitorbitList_ECEF[s_i].vel.y);
						matJ2000Vel.SetElement(2,0,it->second.fitorbitList_ECEF[s_i].vel.z);
						it->second.fitorbitList_ECEF[s_i].t = TimeCoordConvert::TDT2GPST(it->second.fitorbitList_ECEF[s_i].t);
						Matrix matPR_NR, matER, matEP, matER_DOT;
						m_TimeCoordConvert.Matrix_J2000_ECEF(it->second.fitorbitList_ECEF[s_i].t, matPR_NR, matER, matEP, matER_DOT);
						Matrix matEst_EP, matEst_ER;
						paraSp3Fit.getEst_EOP(it->second.fitorbitList_ECEF[s_i].t, matEst_EP, matEst_ER);// 更新 matEP, matER
						matEP = matEst_EP * matEP;
						matER = matER;
						matECFPos = matPR_NR * matJ2000Pos;
                        matECFVel = matPR_NR * matJ2000Vel;
						matECFVel = matER *  matECFVel + matER_DOT * matECFPos;
						matECFPos = matER *  matECFPos;
						matECFPos = matEP *  matECFPos;
						matECFVel = matEP *  matECFVel;
						it->second.fitorbitList_ECEF[s_i].pos.x = matECFPos.GetElement(0, 0); 
						it->second.fitorbitList_ECEF[s_i].pos.y = matECFPos.GetElement(1, 0); 
						it->second.fitorbitList_ECEF[s_i].pos.z = matECFPos.GetElement(2, 0);
						it->second.fitorbitList_ECEF[s_i].vel.x = matECFVel.GetElement(0, 0); 
						it->second.fitorbitList_ECEF[s_i].vel.y = matECFVel.GetElement(1, 0); 
						it->second.fitorbitList_ECEF[s_i].vel.z = matECFVel.GetElement(2, 0);
					}
				}
				ndtSp3File.m_data.resize(paraSp3Fit.satParaList.begin()->second.fitorbitList_ECEF.size());
				for(size_t s_i = 0; s_i < ndtSp3File.m_data.size(); s_i++)
				{
					ndtSp3File.m_data[s_i].t = paraSp3Fit.satParaList.begin()->second.fitorbitList_ECEF[s_i].t;
					for(Sp3Fit_SatdynDatumMap::iterator it = paraSp3Fit.satParaList.begin(); it != paraSp3Fit.satParaList.end(); ++it)
					{
						SP3Datum datum;
						datum.pos = it->second.fitorbitList_ECEF[s_i].pos * 0.001;
						datum.vel = it->second.fitorbitList_ECEF[s_i].vel * 0.001;
						datum.clk = 0;
						datum.clkrate = 0;
						char SatName[4];
						sprintf(SatName, "C%02d", it->first);
						ndtSp3File.m_data[s_i].sp3.insert(SP3SatMap::value_type(SatName, datum));
					}
				}
				ndtSp3File.write(outputSp3FilePath);
			}
			printf(",  sp3fit is ok.\n");
			return result;
		}
		////   子程序名称： RelativityCorrect   
		////   作用：广义相对论修正，在一个统一的地心坐标系下进行
		////   类型：
		////         satPos          :  卫星在地心系位置（米）
		////         staPos          :  测站在地心系位置（米）
		////         gamma           :  相对论修正因子，一般情况下取1
		////   输入：sunPos，satPos, staPos
		////   输出：dR_relativity
		////   其它：
		////   语言： C++
		////   版本号：2013.7.10
		////   生成者：谷德峰
		////   修改者：
		////   修改记录：2013.12.5,去除太阳引力的影响
		//double BDSatDynPOD::graRelativityCorrect(POS3D satPos, POS3D staPos, double gamma)
		//{
		//	double cc = pow(SPEED_LIGHT, 2); // 光速的平方
		//	// 太阳引力场引起的相对论修正			
		//	POS3D  V_sat_sta = satPos - staPos;
		//	double r   = sqrt(V_sat_sta.x * V_sat_sta.x + V_sat_sta.y * V_sat_sta.y + V_sat_sta.z * V_sat_sta.z); // 卫星到观测站的距离
		//	// 地球引力场引起的相对论修正
		//	double r1 = sqrt(satPos.x * satPos.x + satPos.y * satPos.y + satPos.z * satPos.z); // 地心到卫星的距离
		//	double r2 = sqrt(staPos.x * staPos.x + staPos.y * staPos.y + staPos.z * staPos.z); // 地心到观测站的距离
		//	double dR2 = (GM_EARTH / cc) * log((r1 + r2 + r)/(r1 + r2 - r));
		//	double dR_relativity = (1 + gamma) * dR2;
		//	return dR_relativity; 

		// }
		
		// 子程序名称： getStaBaselineList_MiniPath   
		// 功能：基线选取
		// 变量类型：staList               : 测站坐标列表
		//           staBaseLineIdList_A   : 基线端点A索引
		//           staBaseLineIdList_B   : 基线端点B索引
		// 输入：staList
		// 输出：staBaseLineIdList_A, staBaseLineIdList_B
		// 语言：C++
		// 创建者：鞠冰
		// 创建时间：2012/12/12
		// 版本时间：
		// 修改记录：
		// 备注： 参考 Bernese 软件中基线选取算法(minimum path method)
		//void BDSatDynPOD::getStaBaselineList_MiniPath(vector<POS3D> staList, vector<int>& staBaseLineIdList_A, vector<int>& staBaseLineIdList_B)
		//{
		//	size_t num = staList.size();		// 测站总数
		//	staBaseLineIdList_A.clear();
		//	staBaseLineIdList_B.clear();
		//	vector<int>		flag;				// 测站标记
		//	vector<int>		AllLineIdList_A;	// 任意两测站连线端点A索引
		//	vector<int>		AllLineIdList_B;	// 任意两测站连线端点B索引
		//	vector<double>	LengthList_AB;		// 任意两测站之间连线长度
		//	// Step1: 计算所有可能的基线长度
		//	for(size_t k = 0; k < num; k++)
		//	{
		//		flag.push_back(0);
		//		for(size_t j = k + 1; j < num; j++)
		//		{
		//			AllLineIdList_A.push_back(int(k));
		//			AllLineIdList_B.push_back(int(j));
		//			POS3D AB = staList[k] - staList[j];
		//			double Length_AB = sqrt(AB.x * AB.x + AB.y * AB.y + AB.z * AB.z);
		//			LengthList_AB.push_back(Length_AB);
		//		}
		//	}
		//	// Step2: 将所有可能的基线按长度重新排列（冒泡算法）
		//	size_t nLines = LengthList_AB.size();
		//	for(size_t l = 0; l < nLines; l++)
		//	{
		//		for(size_t m = l + 1; m < nLines; m++)
		//		{
		//			if(LengthList_AB[m] < LengthList_AB[l])
		//			{
		//				int		temp1 = AllLineIdList_A[l];
		//				int		temp2 = AllLineIdList_B[l];
		//				double	temp3 = LengthList_AB[l];
		//				AllLineIdList_A[l] = AllLineIdList_A[m];
		//				AllLineIdList_B[l] = AllLineIdList_B[m];
		//				LengthList_AB[l]   = LengthList_AB[m];
		//				AllLineIdList_A[m] = temp1;
		//				AllLineIdList_B[m] = temp2;
		//				LengthList_AB[m]   = temp3;
		//			}
		//		}
		//	}
		//	// Step3: 选出（num - 1）条不相关的基线
		//	int maximum_flag = 0;
		//	flag[AllLineIdList_A[0]] = 1;
		//	flag[AllLineIdList_B[0]] = 1;
		//	maximum_flag = 1;
		//	staBaseLineIdList_A.push_back(AllLineIdList_A[0]);
		//	staBaseLineIdList_B.push_back(AllLineIdList_B[0]);
		//	int	index = 1;
		//	while(staBaseLineIdList_A.size() != (num - 1))
		//	{
		//		int flag1 = flag[AllLineIdList_A[index]];
		//		int flag2 = flag[AllLineIdList_B[index]];
		//		if(flag1 == flag2)
		//		{
		//			if(flag1 == 0 && flag2 == 0)
		//			{
		//				maximum_flag++;
		//				flag[AllLineIdList_A[index]] = maximum_flag;
		//				flag[AllLineIdList_B[index]] = maximum_flag;
		//				staBaseLineIdList_A.push_back(AllLineIdList_A[index]);
		//				staBaseLineIdList_B.push_back(AllLineIdList_B[index]);
		//				index++;
		//			}
		//			else
		//			{
		//				index++;
		//				continue;
		//			}
		//		}
		//		else
		//		{
		//			if(flag1 == 0)
		//			{
		//				flag[AllLineIdList_A[index]] = flag2;
		//				staBaseLineIdList_A.push_back(AllLineIdList_A[index]);
		//				staBaseLineIdList_B.push_back(AllLineIdList_B[index]);
		//			}
		//			else if(flag2 == 0)
		//			{
		//				flag[AllLineIdList_B[index]] = flag1;
		//				staBaseLineIdList_A.push_back(AllLineIdList_A[index]);
		//				staBaseLineIdList_B.push_back(AllLineIdList_B[index]);
		//			}
		//			else
		//			{
		//				int flag_min = min(flag1, flag2); 
		//				int flag_max = max(flag1, flag2);
		//				for(size_t k = 0; k < num; k++)
		//				{
		//					if(flag[k] == flag_min)
		//					{
		//						flag[k] = flag_max;
		//					}
		//				}
		//				staBaseLineIdList_A.push_back(AllLineIdList_A[index]);
		//				staBaseLineIdList_B.push_back(AllLineIdList_B[index]);
		//			}
		//			index++;
		//		}
		//	}				
		//}

		// 子程序名称： obsSingleDifferencePreproc   
		// 功能：同过测站A和B的观测文件形成单差文件, A-B
		// 变量类型：editedObsFile_A   : 测站A的观测文件
		//           editedObsFile_B   : 测站B的观测文件
		//           editedSdObsFile   : 测站A和测站B的单差文件
		// 输入：editedObsFile_A，editedObsFile_B
		// 输出：editedSdObsFile
		// 语言：C++
		// 创建者：刘俊宏、谷德峰
		// 创建时间：2012/12/11
		// 版本时间：
		// 修改记录：
		// 备注： 参考星载单差文件形成方法编写。由于不同接收机的观测数据类型可能不同，形成单差之前观测数据类型顺序不一定必须一致。
        //        暂时规定输出的顺序为P1,P2,L1,L2
		/*---------------------------------------------------------------------------------------------------
		   1. 以 A 的采样时刻为基准, 只有同时有效的数据才能进入干涉处理
		   2. 干涉周跳标记的策略, 双接收机一票否决制
		      只要卫星 A 的周跳或卫星 B 的发生周跳时, 则判定该处干涉相位数据出现新的模糊度
		      A 和 B 的采样时刻不一致, 当 A 没有的采样时刻, B 有, 且为周跳, 此时 B 的周跳可能被漏掉, 需要补记
		   3. 单星周跳的判定策略, 双频率一票否决制
		      只要 L1 和 L2 中有一个波段发生周跳, 则判定该处 L1 和 L2 均发生了周跳
		      这样处理的原因: 将 L1 和 L2 的无周跳区间进行了统一, 大大方便后面的干涉处理, 尤其是宽巷组合
		                      在数据预处理中, 本身并没有对 L1 和 L2 的周跳进行详细的区分
		      这样处理的缺点: 如果 L1 和 L2的数据进行分开处理, 则这样的周跳判定准则显得过于严格
		----------------------------------------------------------------------------------------------------*/		
		// 4.两个观测数据文件观测数据类型的排列方式可能不同，但只要有相同的数据类型就可以形成差分，
		// 因此将原程序调整为按观测数据类型寻找可差分数据，2014/1/10,刘俊宏
		bool BDSatDynPOD::obsSingleDifferencePreproc(Rinex2_1_EditedObsFile& editedObsFile_A, Rinex2_1_EditedObsFile& editedObsFile_B, Rinex2_1_EditedSdObsFile& editedSdObsFile)
		{
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1_A = -1, nObsTypes_L2_A = -1, nObsTypes_P1_A = -1, nObsTypes_P2_A = -1;
			int nObsTypes_L1_B = -1, nObsTypes_L2_B = -1, nObsTypes_P1_B = -1, nObsTypes_P2_B = -1;
			for(int i = 0; i < editedObsFile_A.m_header.byObsTypes; i++)
			{
				if(editedObsFile_A.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1_A = i;
				if(editedObsFile_A.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2_A = i;
				if(editedObsFile_A.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1_A = i;
				if(editedObsFile_A.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2_A = i;
			}
			for(int i = 0; i < editedObsFile_B.m_header.byObsTypes; i++)
			{
				if(editedObsFile_B.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1_B = i;
				if(editedObsFile_B.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2_B = i;
				if(editedObsFile_B.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1_B = i;
				if(editedObsFile_B.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2_B = i;
			}
			if(nObsTypes_L1_A == -1 || nObsTypes_L2_A == -1 || nObsTypes_P1_A == -1 || nObsTypes_P2_A == -1||
			   nObsTypes_L1_B == -1 || nObsTypes_L2_B == -1 || nObsTypes_P1_B == -1 || nObsTypes_P2_B == -1) 
			{
				printf("观测数据不完整！\n");
				return false;
			}
			 // 从预处理文件导入数据
			vector<Rinex2_1_EditedObsSat> editedObsSatList_A;
			if(!editedObsFile_A.getEditedObsSatList(editedObsSatList_A))  
			{
				printf("获取第一个测站的卫星列表失败！\n");
				return false;
			}
			vector<Rinex2_1_EditedObsSat> editedObsSatList_B;
			if(!editedObsFile_B.getEditedObsSatList(editedObsSatList_B)) 
			{
				printf("获取第二个测站的卫星列表失败！\n");
				return false;
			}
			vector<Rinex2_1_EditedSdObsSat> editedSdObsSatList; // 用于记录单差数据
			editedSdObsSatList.clear();
			for(size_t s_i_a = 0; s_i_a < editedObsSatList_A.size(); s_i_a++)
			{// 遍历每颗卫星, 以 A 测站为参考
				Rinex2_1_EditedSdObsSat dataList_sat_i;       // 用于记录本颗卫星的干涉数据
				dataList_sat_i.Id = editedObsSatList_A[s_i_a].Id;
				dataList_sat_i.editedObs.clear();
				bool bFind = false;
				size_t s_i_b = 0;
				for(s_i_b = 0; s_i_b < editedObsSatList_B.size(); s_i_b++)
				{// 对卫星的序号 PRN 进行匹配
					if(editedObsSatList_B[s_i_b].Id == editedObsSatList_A[s_i_a].Id)
					{
						bFind = true;
						break;
					}
				}
				if(!bFind)
					continue;
				size_t count_A = editedObsSatList_A[s_i_a].editedObs.size();   // 观测时间序列数据个数
				size_t count_B = editedObsSatList_B[s_i_b].editedObs.size();   // 观测时间序列数据个数(某颗固定卫星)
				double *pObsTime = new double [count_A]; // 相对时间序列
				DayTime *pObsGPST_A = new DayTime [count_A];
				DayTime *pObsGPST_B = new DayTime [count_B];
				int *pEditedFlag_A = new int [count_A]; // 编辑标记序列
				int *pEditedFlag_B = new int [count_B]; // 编辑标记序列
				DayTime t0 = editedObsSatList_A[s_i_a].editedObs.begin()->first; // 初始时间 - 用于计算相对时间(秒)
				int s_j = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatList_A[s_i_a].editedObs.begin(); it != editedObsSatList_A[s_i_a].editedObs.end(); ++it)
				{
					pObsTime[s_j] = it->first - t0;
					pObsGPST_A[s_j] = it->first;
					if( it->second.obsTypeList[nObsTypes_L1_A].byEditedMark1 == TYPE_EDITEDMARK_NORMAL
					 && it->second.obsTypeList[nObsTypes_L2_A].byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						pEditedFlag_A[s_j] = TYPE_EDITEDMARK_NORMAL;  // 正常
					else if(it->second.obsTypeList[nObsTypes_L1_A].byEditedMark1 == TYPE_EDITEDMARK_SLIP
					|| it->second.obsTypeList[nObsTypes_L2_A].byEditedMark1 == TYPE_EDITEDMARK_SLIP)
						pEditedFlag_A[s_j] = TYPE_EDITEDMARK_SLIP;    // 新的周跳标记
					else
						pEditedFlag_A[s_j] = TYPE_EDITEDMARK_OUTLIER; // 异常点
					s_j++;
				}
				s_j = 0;
				for(Rinex2_1_EditedObsEpochMap::iterator it = editedObsSatList_B[s_i_b].editedObs.begin(); it != editedObsSatList_B[s_i_b].editedObs.end(); ++it)
				{
					pObsGPST_B[s_j] = it->first;
					if( it->second.obsTypeList[nObsTypes_L1_B].byEditedMark1 == TYPE_EDITEDMARK_NORMAL
					 && it->second.obsTypeList[nObsTypes_L2_B].byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
						pEditedFlag_B[s_j] = TYPE_EDITEDMARK_NORMAL;  // 正常
					else if(it->second.obsTypeList[nObsTypes_L1_B].byEditedMark1 == TYPE_EDITEDMARK_SLIP
					|| it->second.obsTypeList[nObsTypes_L2_B].byEditedMark1 == TYPE_EDITEDMARK_SLIP)
						pEditedFlag_B[s_j] = TYPE_EDITEDMARK_SLIP;    // 新的周跳标记
					else
						pEditedFlag_B[s_j] = TYPE_EDITEDMARK_OUTLIER; // 异常点
					s_j++;
				}
				size_t k   = 0; // 记录新弧段起始点
				size_t k_i = k; // 记录新弧段终止点
				int k_now_b = 0;
				bool bSlipLast = false; // 计录上次最新的周跳信息是否得以保存
				while(1)
				{
					if(k_i + 1 >= count_A) // k_i 为时间序列终点
						goto NewArc;
					else
					{
						// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
						if(pEditedFlag_A[k_i+1] != TYPE_EDITEDMARK_SLIP)
						{
							k_i ++;
							continue;
						}
						else // k_i+1为新弧段的起点
							goto NewArc;
					}
					NewArc:  // 本弧段[k, k_i]数据处理 
					{
						Rinex2_1_EditedSdObsEpochMap editedObsSatList_Arc;// 记录正常单差点
						editedObsSatList_Arc.clear();
						vector<size_t> normalPointList; // 记录正常单差点的序号
						normalPointList.clear();
						vector<double> slipList; // 记录周跳标记
						slipList.clear();
						// 在 A 的基础上, 添加 B 的周跳信息
						// 只要在区间(k - 1 , k]内 B 发生周跳, 则 A 需要补充该周跳信息
						for(size_t s_k = k; s_k <= k_i; s_k++)
						{
							bFind = false;        // 寻找本时刻 B 星的观测数据
							bool bSlip_B = false;
							// 应该在相邻的时刻间(k - 1 , k], 寻找 B 是否发生周跳
							for(size_t s_k_b = k_now_b; s_k_b < count_B; s_k_b++)
							{
								double spanSec_B = pObsGPST_B[s_k_b] - t0;
								if(s_k == k)
								{// 即使周跳也没有影响
									break;
								}
								else
								{
									if(spanSec_B - pObsTime[s_k - 1] <= 0)
									{// (-oo, k - 1]
										continue;
									}
									else if(spanSec_B - pObsTime[s_k] <= 0 && spanSec_B - pObsTime[s_k - 1] > 0)
									{//(k - 1 , k]
										//
										if(pEditedFlag_B[s_k_b] == TYPE_EDITEDMARK_SLIP && bSlip_B == false)
										{
											bSlip_B = true;
										}
									}
									else
									{//(k, oo)
										k_now_b = int(s_k_b);
										break;
									}
								}
							}
							// 进行数据干涉
							Rinex2_1_EditedObsEpochMap::const_iterator it_A = editedObsSatList_A[s_i_a].editedObs.find(pObsGPST_A[s_k]);
							Rinex2_1_EditedObsEpochMap::const_iterator it_B = editedObsSatList_B[s_i_b].editedObs.find(pObsGPST_A[s_k]);
							if(it_A != editedObsSatList_A[s_i_a].editedObs.end() && it_B != editedObsSatList_B[s_i_b].editedObs.end())
							{
								bFind = true;
							}
							if(bSlip_B)
							{// 记录周跳点
								slipList.push_back(s_k);
								bSlipLast = true;
								//printf("slipList卫星PRN%2d周跳添加, 调整%4d位\n", editedObsSatList_A[s_i_a].Id, s_k - k);
							}
							// 观测时间匹配:  pdop有效; 观测数据正常
							// 此处有可能因数据的不完整而丢失部分周跳信息, 
							if(bFind)
							{
								if( it_A->second.obsTypeList[nObsTypes_P1_A].byEditedMark1 == TYPE_EDITEDMARK_NORMAL
								&&  it_A->second.obsTypeList[nObsTypes_P2_A].byEditedMark1 == TYPE_EDITEDMARK_NORMAL
								&& (it_A->second.obsTypeList[nObsTypes_L1_A].byEditedMark1 == TYPE_EDITEDMARK_NORMAL || it_A->second.obsTypeList[nObsTypes_L1_A].byEditedMark1 == TYPE_EDITEDMARK_SLIP)
								&& (it_A->second.obsTypeList[nObsTypes_L2_A].byEditedMark1 == TYPE_EDITEDMARK_NORMAL || it_A->second.obsTypeList[nObsTypes_L2_A].byEditedMark1 == TYPE_EDITEDMARK_SLIP)
								&&  it_B->second.obsTypeList[nObsTypes_P1_B].byEditedMark1 == TYPE_EDITEDMARK_NORMAL
								&&  it_B->second.obsTypeList[nObsTypes_P2_B].byEditedMark1 == TYPE_EDITEDMARK_NORMAL
								&& (it_B->second.obsTypeList[nObsTypes_L1_B].byEditedMark1 == TYPE_EDITEDMARK_NORMAL || it_B->second.obsTypeList[nObsTypes_L1_B].byEditedMark1 == TYPE_EDITEDMARK_SLIP)
								&& (it_B->second.obsTypeList[nObsTypes_L2_B].byEditedMark1 == TYPE_EDITEDMARK_NORMAL || it_B->second.obsTypeList[nObsTypes_L2_B].byEditedMark1 == TYPE_EDITEDMARK_SLIP)
								   )
								{
									Rinex2_1_EditedSdObsLine satNow;
									satNow.Azimuth_A = it_A->second.Azimuth;
									satNow.Elevation_A = it_A->second.Elevation;
									satNow.Azimuth_B = it_B->second.Azimuth;
									satNow.Elevation_B = it_B->second.Elevation;
									satNow.Id = it_A->second.Id;
									//satNow.nObsTime = it_A->second.nObsTime;
									satNow.ReservedField = 0;
									satNow.obsTypeList.clear();
									Rinex2_1_EditedObsDatum P1;
									P1.byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
									P1.byEditedMark2 = 0;
									P1.obs.data = it_A->second.obsTypeList[nObsTypes_P1_A].obs.data - it_B->second.obsTypeList[nObsTypes_P1_B].obs.data;
									Rinex2_1_EditedObsDatum P2;
									P2.byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
									P2.byEditedMark2 = 0;
									P2.obs.data = it_A->second.obsTypeList[nObsTypes_P2_A].obs.data - it_B->second.obsTypeList[nObsTypes_P2_B].obs.data;
									Rinex2_1_EditedObsDatum L1;
									Rinex2_1_EditedObsDatum L2;
									// 2009/08/11
									if(bSlip_B || pEditedFlag_A[s_k] == TYPE_EDITEDMARK_SLIP || bSlipLast)
									{
										L1.byEditedMark1 = TYPE_EDITEDMARK_SLIP;
										L1.byEditedMark2 = 0;
										L2.byEditedMark1 = TYPE_EDITEDMARK_SLIP;
										L2.byEditedMark2 = 0;
										bSlipLast = false; // 清除周跳遗留标记
									}
									else
									{
										L1.byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
										L1.byEditedMark2 = 0;
										L2.byEditedMark1 = TYPE_EDITEDMARK_NORMAL;
										L2.byEditedMark2 = 0;
									}
									L1.obs.data = it_A->second.obsTypeList[nObsTypes_L1_A].obs.data - it_B->second.obsTypeList[nObsTypes_L1_B].obs.data;
									L2.obs.data = it_A->second.obsTypeList[nObsTypes_L2_A].obs.data - it_B->second.obsTypeList[nObsTypes_L2_B].obs.data;
									satNow.obsTypeList.push_back(P1);
									satNow.obsTypeList.push_back(P2);
									satNow.obsTypeList.push_back(L1);
									satNow.obsTypeList.push_back(L2);
									editedObsSatList_Arc.insert(Rinex2_1_EditedSdObsEpochMap::value_type(pObsGPST_A[s_k], satNow));
									normalPointList.push_back(s_k);
								}
							}
						}
						if(pEditedFlag_A[k] == TYPE_EDITEDMARK_SLIP && editedObsSatList_Arc.size() > 0) 
						{// 将正常干涉点中的第一点调整为周跳标记点
							editedObsSatList_Arc.begin()->second.obsTypeList[2].byEditedMark1 = TYPE_EDITEDMARK_SLIP;
							editedObsSatList_Arc.begin()->second.obsTypeList[2].byEditedMark2 = 0;
							editedObsSatList_Arc.begin()->second.obsTypeList[3].byEditedMark1 = TYPE_EDITEDMARK_SLIP;
							editedObsSatList_Arc.begin()->second.obsTypeList[3].byEditedMark2 = 0;
							//printf("第一点, 卫星PRN%2d周跳标记更新, 调整%4d位\n", editedObsSatList_A[s_i_a].Id, normalPointList[0] - k);
						}
						for(Rinex2_1_EditedSdObsEpochMap::iterator it = editedObsSatList_Arc.begin(); it != editedObsSatList_Arc.end(); ++it)
						{
							dataList_sat_i.editedObs.insert(Rinex2_1_EditedSdObsEpochMap::value_type(it->first, it->second));
						}
						if( k_i + 1 >= count_A) // k_i为时间序列终点，跳出
							break;
						else  
						{   
							k   = k_i + 1;     // 新弧段的起点设置
							k_i = k;
							continue;
						}
					}
				}
				delete pObsTime;
				delete pObsGPST_A;
				delete pEditedFlag_A;
				delete pObsGPST_B;
				delete pEditedFlag_B;
				if(dataList_sat_i.editedObs.size() > 0)
					editedSdObsSatList.push_back(dataList_sat_i);
			}
			if(editedSdObsSatList.size() <= 0)
				return false;
			editedSdObsFile.m_data.clear();
			int k_now = 0;
			BYTE pbySatList[MAX_PRN_BD]; 
			for(int i = 0; i < MAX_PRN_BD; i++)
				pbySatList[i] = 0;
			for(size_t s_i = 0; s_i < editedObsFile_A.m_data.size(); s_i++)
			{
				Rinex2_1_EditedSdObsEpoch editedSdObsEpoch;
                editedSdObsEpoch.byEpochFlag = editedObsFile_A.m_data[s_i].byEpochFlag;
				editedSdObsEpoch.t = editedObsFile_A.m_data[s_i].t;
				editedSdObsEpoch.editedObs.clear();
				// 遍历每颗卫星的数据列表
				for(size_t s_j = 0; s_j < editedSdObsSatList.size(); s_j++)
				{// 判断当前时刻的数据是否符合要求(!前提是预处理期间, 时间标签未被改动!)
					Rinex2_1_EditedSdObsEpochMap::const_iterator it = editedSdObsSatList[s_j].editedObs.find(editedSdObsEpoch.t);
					if(it != editedSdObsSatList[s_j].editedObs.end())
					{
						pbySatList[editedSdObsSatList[s_j].Id] = 1;
						editedSdObsEpoch.editedObs.insert(Rinex2_1_EditedSdObsSatMap::value_type(editedSdObsSatList[s_j].Id, it->second));
					}
				}
				bool bFind = false;
				size_t s_k = k_now;
				for(s_k = k_now; s_k < editedObsFile_B.m_data.size(); s_k++)
				{
					double spanSec = editedObsFile_B.m_data[s_k].t - editedObsFile_A.m_data[s_i].t;
					// 阈值 1.0E-6
					if(spanSec < -1.0E-6)
						continue;
					else if(spanSec > 1.0E-6)
					{
						k_now = int(s_k);
						break;
					}
					else
					{// 时间匹配成功
						bFind = true;
						k_now = int(s_k) + 1;
						break;
					}
				}
				if(bFind)
				{
					editedSdObsEpoch.bySatCount    = int(editedSdObsEpoch.editedObs.size());					
			        editedSdObsEpoch.A_clock       = editedObsFile_A.m_data[s_i].clock;
					editedSdObsEpoch.A_tropZenithDelayPriori_H        = editedObsFile_A.m_data[s_i].tropZenithDelayPriori_H;
			        editedSdObsEpoch.A_tropZenithDelayPriori_W        = editedObsFile_A.m_data[s_i].tropZenithDelayPriori_W;
			        editedSdObsEpoch.A_tropZenithDelayEstimate        = editedObsFile_A.m_data[s_i].tropZenithDelayEstimate;
					editedSdObsEpoch.A_temperature = editedObsFile_A.m_data[s_i].temperature;
			        editedSdObsEpoch.A_humidity    = editedObsFile_A.m_data[s_i].humidity;
			        editedSdObsEpoch.A_pressure    = editedObsFile_A.m_data[s_i].pressure;					
					editedSdObsEpoch.B_clock       = editedObsFile_B.m_data[s_k].clock;
					editedSdObsEpoch.B_tropZenithDelayPriori_H        = editedObsFile_B.m_data[s_k].tropZenithDelayPriori_H;
			        editedSdObsEpoch.B_tropZenithDelayPriori_W        = editedObsFile_B.m_data[s_k].tropZenithDelayPriori_W;
			        editedSdObsEpoch.B_tropZenithDelayEstimate        = editedObsFile_B.m_data[s_k].tropZenithDelayEstimate;
					editedSdObsEpoch.B_temperature = editedObsFile_B.m_data[s_k].temperature;
			        editedSdObsEpoch.B_humidity    = editedObsFile_B.m_data[s_k].humidity;
			        editedSdObsEpoch.B_pressure    = editedObsFile_B.m_data[s_k].pressure;
					if(editedObsFile_A.m_data[s_i].byEpochFlag == 10 || editedObsFile_B.m_data[s_k].byEpochFlag == 10)
						editedSdObsEpoch.byEpochFlag = 10;
					//if(editedSdObsEpoch.bySatCount > 0)
					if(editedSdObsEpoch.bySatCount > 0 
					&& editedObsFile_A.m_data[s_i].byEpochFlag != 10 
					&& editedObsFile_B.m_data[s_k].byEpochFlag != 10)//剔除接收机钟差解算有误的历元，2014/5/3,刘俊宏
						editedSdObsFile.m_data.push_back(editedSdObsEpoch);
				}
			}
			if(editedSdObsFile.m_data.size() <= 0)
				return false;
			// 更新 nObsTime, 2013/01/09, 谷德峰
            for(size_t s_i = 0; s_i < editedSdObsFile.m_data.size(); s_i++)
			{
				for(Rinex2_1_EditedSdObsSatMap::iterator it = editedSdObsFile.m_data[s_i].editedObs.begin(); it != editedSdObsFile.m_data[s_i].editedObs.end(); ++it)
				{
					it->second.nObsTime = int(s_i);
				}
			}
			// 更新文件头信息
			editedSdObsFile.m_header = editedObsFile_A.m_header;
			editedSdObsFile.m_header.byObsTypes = 4;
			editedSdObsFile.m_header.pbyObsTypeList[0] = TYPE_OBS_P1;
			editedSdObsFile.m_header.pbyObsTypeList[1] = TYPE_OBS_P2;
			editedSdObsFile.m_header.pbyObsTypeList[2] = TYPE_OBS_L1;
			editedSdObsFile.m_header.pbyObsTypeList[3] = TYPE_OBS_L2;
			// 综合统计可视卫星列表
			editedSdObsFile.m_header.pbySatList.clear();
			for(int i = 0; i < MAX_PRN_BD; i++)
			{
				if(pbySatList[i] == 1)
					editedSdObsFile.m_header.pbySatList.push_back(BYTE(i));
			}
			editedSdObsFile.m_header.bySatCount = int(editedSdObsFile.m_header.pbySatList.size());
			editedSdObsFile.m_header.tmStart = editedSdObsFile.m_data[0].t;
			editedSdObsFile.m_header.tmEnd = editedSdObsFile.m_data[editedSdObsFile.m_data.size() - 1].t;
			// 更新预处理文件创建日期和注释行信息
			DayTime T_Now;
			T_Now.Now();
			sprintf(editedSdObsFile.m_header.szFileDate, "%04d-%02d-%02d %02d:%02d:%02d",
				                                         T_Now.year,
													     T_Now.month,
													     T_Now.day,
											             T_Now.hour,
													     T_Now.minute,
													     int(T_Now.second));
			sprintf(editedSdObsFile.m_header.szProgramName, "%-20s", "NUDT Toolkit 1.0");
			sprintf(editedSdObsFile.m_header.szProgramAgencyName, "%-20s", "NUDT");
			editedSdObsFile.m_header.pstrCommentList.clear();
			char szComment[100];
			sprintf(szComment,"%-60s%20s\n", "created by PROD::mainFuncPreproc program.", Rinex2_1_MaskString::szComment);
			editedSdObsFile.m_header.pstrCommentList.push_back(szComment);
			sprintf(editedSdObsFile.m_header.szFileType, "%-20s", "EDITED SDOBS");
			return true;
		}
		// 子程序名称： obsTriDiFFEdited_LIF   
		// 功能： 进行三差相位数据的周跳编辑, 如果无法诊断则返回 false
		// 变量类型： index_L1        : 观测类型L1索引
		//            index_L2        : 观测类型L2索引
        //            nPRN            : 卫星号
		//            epoch_j_1       : j_1 时刻的卫星单差观测数据
		//            epoch_j         : j   时刻的卫星单差观测数据
        //            mapCvDatum_j_1  : j_1 时刻的卫星单差改正数据 
		//            mapCvDatum_j    : j   时刻的卫星单差改正数据 
		//            max_res_ddd     : 编辑残差最大值
		//            slipFlag        : 周跳标记
		// 输入： index_L1, index_L2, nPRN, epoch_j_1, epoch_j, mapCvDatum_j_1, mapCvDatum_j 
		// 输出： res, slipFlag
		// 语言： C++
		// 创建者：谷德峰
		// 创建时间：2014/04/16
		// 版本时间：
		// 修改记录：2014/06/08由刘俊宏修改用于BDS数据处理
		// 备注：
        /*
			 以该卫星 id_Sat 为参考星, 计算双差历元差残差
			 理论上, 至少有 k >= 1 个 "不超差", 说明卫星 id_Sat 没问题, 都是"超差", 说明卫星 id_Sat 必然存在周跳 *
			 部分超差, 部分不超差, 可能是由于其它卫星存在周跳
			 两个时刻的正常共视卫星必须大于 2 颗
			 当 k_slip == 1 时, 且只存在 1 组双差数据存在周跳标记, 理论上这种情形应该不存在, 因为只要1个发生周跳, 相应的双差数据都应该被污染, 但残差结果上是存在这种情况的
			 为防止被漏掉, 继续更换参考星, 进行判断, 如果更换后 k_slip > 1, 则 id_sat 未发生周跳, 如果 k_slip == 1, 则 id_sat 要判断其发生周跳
		*/
		bool BDSatDynPOD::obsTriDiFFEdited_LIF(int index_L1, int index_L2, int id_sat, Rinex2_1_EditedSdObsEpoch epoch_j_1, Rinex2_1_EditedSdObsEpoch epoch_j, map<int, cvElement> &mapCvDatum_j_1, map<int, cvElement> &mapCvDatum_j, double &max_res_ddd, bool &slipFlag, double threshold_slip)
		{
			max_res_ddd = 0;
			slipFlag = false; // 周跳标记
			Rinex2_1_EditedSdObsSatMap::iterator it = epoch_j.editedObs.begin();
			while(it != epoch_j.editedObs.end())
			{
				int nPRN_i = it->second.Id;
				if(epoch_j_1.editedObs.find(nPRN_i) == epoch_j_1.editedObs.end())
				{// 剔除非公共卫星
					Rinex2_1_EditedSdObsSatMap::iterator jt = it;
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
			// 判断卫星 id_sat 的数据是否存在
			size_t count_gpssat = epoch_j.editedObs.size();
            if(epoch_j.editedObs.find(id_sat) == epoch_j.editedObs.end()
			|| mapCvDatum_j.find(id_sat) == mapCvDatum_j.end()
			|| mapCvDatum_j_1.find(id_sat) == mapCvDatum_j_1.end()
			|| count_gpssat < 2)
				return false;
			double weight_P_IF, weight_L_IF;
			double coefficient_IF = 1 / (1 - pow(BD_FREQUENCE_L1 / BD_FREQUENCE_L2, 2)); // 无电离层组合系数
			// 参考星的单差数据
			Rinex2_1_EditedSdObsSatMap::iterator it_1 = epoch_j_1.editedObs.find(id_sat);
			double obs_LIF_ref_j_1 = BD_WAVELENGTH_L1 * it_1->second.obsTypeList[index_L1].obs.data
				                  - (BD_WAVELENGTH_L1 * it_1->second.obsTypeList[index_L1].obs.data - BD_WAVELENGTH_L2 * it_1->second.obsTypeList[index_L2].obs.data) * coefficient_IF;
			it = epoch_j.editedObs.find(id_sat); 
			double obs_LIF_ref_j = BD_WAVELENGTH_L1 * it->second.obsTypeList[index_L1].obs.data
				                - (BD_WAVELENGTH_L1 * it->second.obsTypeList[index_L1].obs.data - BD_WAVELENGTH_L2 * it->second.obsTypeList[index_L2].obs.data) * coefficient_IF;
			double obs_diff_ref = (obs_LIF_ref_j   + mapCvDatum_j[id_sat].obscorrected_value)
				                - (obs_LIF_ref_j_1 + mapCvDatum_j_1[id_sat].obscorrected_value); // 单差 + 历元差
			// 计算当前时刻参考星的单差观测权值
			weighting_Elevation((it->second.Elevation_B + it->second.Elevation_A) * 0.5, weight_P_IF, weight_L_IF);
			double weight_ref_SD_L = weight_L_IF;
			int j = 0;
			int k_slip = 0;
			vector<int> slipIdSatList;
			vector<double> sdObs_noRef_List;
			vector<double> w_noRef_List;
			for(it = epoch_j.editedObs.begin(); it != epoch_j.editedObs.end(); ++it)
			{// 进入双差处理
				if(it->first == id_sat) // 非参考星
					continue;
				it_1 = epoch_j_1.editedObs.find(it->first);
				if(mapCvDatum_j.find(it->first) == mapCvDatum_j.end()
				|| mapCvDatum_j_1.find(it->first) == mapCvDatum_j_1.end())
					continue;
				else
				{
					if(mapCvDatum_j_1[it->first].valid == 0 || mapCvDatum_j[it->first].valid == 0)// 确保 mapCvDatum_j_1、mapCvDatum_j 修正有效
						continue;
					if(mapCvDatum_j_1[it->first].id_arc != mapCvDatum_j[it->first].id_arc)// 确保 mapCvDatum_j_1、mapCvDatum_j 在同一弧段
						continue;
				}
				// 计算当前时刻非参考星的单差观测权值
				weighting_Elevation((it->second.Elevation_B + it->second.Elevation_A) * 0.5, weight_P_IF, weight_L_IF);
				double weight_SD_L = weight_L_IF;
				if(  weight_SD_L == 0.0
				||   it->second.obsTypeList[index_L1].byEditedMark1 != TYPE_EDITEDMARK_NORMAL
				||   it->second.obsTypeList[index_L2].byEditedMark1 != TYPE_EDITEDMARK_NORMAL
				|| it_1->second.obsTypeList[index_L1].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER 
				|| it_1->second.obsTypeList[index_L2].byEditedMark1 == TYPE_EDITEDMARK_OUTLIER)
					continue;// 确保非参考星相位数据有效

				double   obs_LIF_j = BD_WAVELENGTH_L1 * it->second.obsTypeList[index_L1].obs.data
					              - (BD_WAVELENGTH_L1 * it->second.obsTypeList[index_L1].obs.data - BD_WAVELENGTH_L2 * it->second.obsTypeList[index_L2].obs.data) * coefficient_IF;
				double obs_LIF_j_1 = BD_WAVELENGTH_L1 * it_1->second.obsTypeList[index_L1].obs.data
					              - (BD_WAVELENGTH_L1 * it_1->second.obsTypeList[index_L1].obs.data - BD_WAVELENGTH_L2 * it_1->second.obsTypeList[index_L2].obs.data) * coefficient_IF;
				double obs_diff = (obs_LIF_j   + mapCvDatum_j[it->first].obscorrected_value)
					             -(obs_LIF_j_1 + mapCvDatum_j_1[it->first].obscorrected_value); // 单差 + 历元差
				double obs_ddd = obs_diff  - obs_diff_ref; // 双差处理
				
                // 计算当前时刻双差观测权值
				double weight_DD_L = 1 / sqrt(0.5 / (weight_ref_SD_L * weight_ref_SD_L) + 0.5 / (weight_SD_L * weight_SD_L)); // 双差权值, 正常情况下为 1
				if(fabs(max_res_ddd) < fabs(weight_DD_L * obs_ddd))
					max_res_ddd = weight_DD_L * obs_ddd; // 记录最大残差
				if(fabs(weight_DD_L * obs_ddd) > threshold_slip)
				{
					slipIdSatList.push_back(j);
					k_slip++;
				}
				sdObs_noRef_List.push_back(obs_diff);
				w_noRef_List.push_back(weight_SD_L);
				j++;
			}
			// 当 j >= 2, k_slip == 1 时, 且只存在 1 组双差数据存在周跳标记
			// [注: 理论上这种情形应该不存在, 因为只要1个发生周跳, 相应的双差数据都应该被污染, 但残差结果上是存在这种情况的]
			// 为防止被漏掉, 继续更换参考星, 进行判断, 如果更换后 k_slip > 1, 则 id_sat 未发生周跳, 如果 k_slip == 1, 则 id_sat 要判断其发生周跳
			if(j >= 2 && k_slip == 1)
			{// k_slip == sdObs_noRef_List.size()
				obs_diff_ref = sdObs_noRef_List[slipIdSatList[0]]; // 更换参考数据
				weight_ref_SD_L = w_noRef_List[slipIdSatList[0]];
				int k_slip_0 = 0;
				for(size_t s_i = 0; s_i < sdObs_noRef_List.size(); s_i++)
				{
					if(int(s_i) == slipIdSatList[0])
						continue;// 避免自己作双差
					double obs_tridiff = sdObs_noRef_List[s_i] - obs_diff_ref; // 双差处理
					double weight_SD_L = w_noRef_List[s_i];
					// 计算当前时刻双差观测权值
					double weight_DD_L = 1 / sqrt(0.5 / (weight_ref_SD_L * weight_ref_SD_L) + 0.5 / (weight_SD_L * weight_SD_L)); // 双差权值, 正常情况下为 1
					if(fabs(weight_DD_L * obs_tridiff) > threshold_slip)
						k_slip_0++;
				}
				if(k_slip_0 == 0) // 说明只存在 1 组双差数据存在周跳标记, 此时保证可靠性, 判定 id_sat 发生周跳
					slipFlag = true;
				else
					slipFlag = false; // 说明周跳发生在 slipIdSatList[0] 上
				return true;
			}
			else if(j == 1)
			{// 只有 1 组数据k_slip == 1, 此时保证可靠性, 判定id_sat发生周跳
				if(k_slip == 1)
					slipFlag = true;
				return true;
			}
			else
			{// 多组数据 k_slip >= 2
				//if((double(k_slip) / double(j)) >= 0.5)
				if(k_slip >= 2)
					slipFlag = true;
				return true;
			}
		}
		// 子程序名称： mainDDEdited   
		// 功能：根据相位三差（双差+历元差）数据进行单差观测数据编辑
		//       几何距离参考点: 天线 ARP -- GPS卫星相位中心
		// 变量类型：sp3File          : sp3文件[惯性系]
		//           editedSdObsFile  : 拟合后参数
		//           posAnt_A         : 测站A天线概略位置[地固系]
	    //           posAnt_B         : 测站B天线概略位置[地固系]
		//           outputFileName   : 输出文件名称
		// 输入：sp3File, editedSdObsFile, posclk_A, posclk_B, outputFileName
		// 输出：editedSdObsFile
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2014/04/16
		// 版本时间：
		// 修改记录：2014/06/08由刘俊宏修改用于BDS数据处理
		// 备注： 
		bool BDSatDynPOD::mainTriDiFFEdited(SP3File &sp3File, Rinex2_1_EditedSdObsFile &editedSdObsFile, POS3D posAnt_A, POS3D posAnt_B,string outputFileName)
		{
			int    FREQUENCE_1 = 1,FREQUENCE_2 = 2;   
			double coefficient_IF    = 1 / (1 - pow(BD_FREQUENCE_L1 / BD_FREQUENCE_L2, 2)); // 无电离层组合系数
			// 获取卫星数据结构
			vector<Rinex2_1_EditedSdObsSat> editedObsSatList;
			if(!editedSdObsFile.getEditedObsSatList(editedObsSatList)) 
				return false;
			char info[200];
			// 寻找观测类型观测序列中的序号
			int nObsTypes_L1 = -1, nObsTypes_L2 = -1, nObsTypes_P1 = -1, nObsTypes_P2 = -1;
			for(int i = 0; i < editedSdObsFile.m_header.byObsTypes; i++)
			{
				if(editedSdObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
					nObsTypes_L1 = i;
				if(editedSdObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
					nObsTypes_L2 = i;
				if(editedSdObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
					nObsTypes_P1 = i;
				if(editedSdObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
					nObsTypes_P2 = i;
			}
			if(nObsTypes_L1 == -1 || nObsTypes_L2 == -1 || nObsTypes_P1 == -1 || nObsTypes_P2 == -1) 
			{
				return false;
			}
			FILE *pFile = NULL;
			if(!outputFileName.empty())
				pFile = fopen(outputFileName.c_str(), "w+");
			int count_stat = 0;
			double rms_sdobs = 0.0;
			size_t count_slip = 0;
			vector<cvEpoch> cvEpochList; // 记录每个每个时刻的单差观测数据修正量
			size_t count_epoch = editedSdObsFile.m_data.size(); // 观测历元个数
			cvEpochList.resize(count_epoch);
			for(size_t s_i = 0; s_i < count_epoch; s_i++)
			{
				// 转换到 J2000 惯性系
				POSCLK posclk_A, posclk_B;
				posclk_A.clk = editedSdObsFile.m_data[s_i].A_clock; // 考虑到钟差影响
				posclk_B.clk = editedSdObsFile.m_data[s_i].B_clock;
				double P_J2000[3];
				double P_ITRF[3];
				P_ITRF[0] = posAnt_A.x;
				P_ITRF[1] = posAnt_A.y;
				P_ITRF[2] = posAnt_A.z;
				m_TimeCoordConvert.ECEF_J2000(editedSdObsFile.m_data[s_i].t - posclk_A.clk / SPEED_LIGHT, P_J2000, P_ITRF, false); // 坐标系转换, 考虑到接收机钟差
				posclk_A.x = P_J2000[0];
				posclk_A.y = P_J2000[1];
				posclk_A.z = P_J2000[2];
				P_ITRF[0] = posAnt_B.x;
				P_ITRF[1] = posAnt_B.y;
				P_ITRF[2] = posAnt_B.z;
				m_TimeCoordConvert.ECEF_J2000(editedSdObsFile.m_data[s_i].t - posclk_B.clk / SPEED_LIGHT, P_J2000, P_ITRF, false); // 坐标系转换, 考虑到接收机钟差
				posclk_B.x = P_J2000[0];
				posclk_B.y = P_J2000[1];
				posclk_B.z = P_J2000[2];
				for(Rinex2_1_EditedSdObsSatMap::iterator it = editedSdObsFile.m_data[s_i].editedObs.begin(); it != editedSdObsFile.m_data[s_i].editedObs.end(); ++it)
				{
					int id_Sat = it->first;  // 第 j 颗可见BDS卫星的卫星号
					cvElement datum_j;; // = dynEpochList[s_i].mapDatum.find(id_Sat);
					datum_j.obscorrected_value = 0.0;
					datum_j.valid  = 0;
					datum_j.id_arc = 0;
					// 根据测站 A、B 位置获得 BDS 卫星位置
					SP3Datum sp3Datum_A, sp3Datum_B;
					double delay_A = 0;
					double delay_B = 0;
					bool bEphemeris = true;
					char szSatName[4];
					sprintf(szSatName,"C%02d",id_Sat);
					szSatName[3] = '\0';
					if(!sp3File.getEphemeris_PathDelay(editedSdObsFile.m_data[s_i].t, posclk_A, szSatName, delay_A, sp3Datum_A)
					|| !sp3File.getEphemeris_PathDelay(editedSdObsFile.m_data[s_i].t, posclk_B, szSatName, delay_B, sp3Datum_B)) 
						bEphemeris = false;
					if(bEphemeris)
					{
						double correct_gpsrelativity_A = (sp3Datum_A.pos.x * sp3Datum_A.vel.x + sp3Datum_A.pos.y * sp3Datum_A.vel.y + sp3Datum_A.pos.z * sp3Datum_A.vel.z) * (-2) / SPEED_LIGHT;
						double correct_gpsrelativity_B = (sp3Datum_B.pos.x * sp3Datum_B.vel.x + sp3Datum_B.pos.y * sp3Datum_B.vel.y + sp3Datum_B.pos.z * sp3Datum_B.vel.z) * (-2) / SPEED_LIGHT;
						double distance_A = sqrt(pow(posclk_A.x - sp3Datum_A.pos.x, 2) + pow(posclk_A.y - sp3Datum_A.pos.y, 2) + pow(posclk_A.z - sp3Datum_A.pos.z, 2));
						double distance_B = sqrt(pow(posclk_B.x - sp3Datum_B.pos.x, 2) + pow(posclk_B.y - sp3Datum_B.pos.y, 2) + pow(posclk_B.z - sp3Datum_B.pos.z, 2));
						double correct_bdspcopcv_A = 0;
						double correct_bdspcopcv_B = 0;
						map<int, AntCorrectionBlk>::iterator it_BDSAntCorrectionBlk = m_mapBDSAntCorrectionBlk.find(id_Sat);
						if(it_BDSAntCorrectionBlk != m_mapBDSAntCorrectionBlk.end())
						{// BDS 卫星天线相位中心修正
							POS3D sunPos;                     // 太阳相对地心位置
							GPST t_Transmit_A = editedSdObsFile.m_data[s_i].t - posclk_A.clk / SPEED_LIGHT - delay_A;
							TDB t_TDB = TimeCoordConvert::GPST2TDB(t_Transmit_A); // 获得 TDB 时间--提供太阳历参考时间									
							double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
							double Pos[3];
							m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos);									
							sunPos.x = Pos[0] * 1000; 
							sunPos.y = Pos[1] * 1000; 
							sunPos.z = Pos[2] * 1000; 
							double correct_bdspco_A_F1 = 0; //测站A对BD卫星的第一个频点数据PCO修正
							double correct_bdspco_A_F2 = 0; //测站A对BD卫星的第二个频点数据PCO修正
							double correct_bdspco_B_F1 = 0; //测站B对BD卫星的第一个频点数据PCO修正
							double correct_bdspco_B_F2 = 0; //测站B对BD卫星的第二个频点数据PCO修正
							//correct_bdspco_A_F1 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, FREQUENCE_1 - 1, posclk_A.getPos(), sp3Datum_A.pos, sunPos, false);
							//correct_bdspco_A_F2 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, FREQUENCE_2 - 1, posclk_A.getPos(), sp3Datum_A.pos, sunPos, false);
							//correct_bdspco_B_F1 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, FREQUENCE_1 - 1, posclk_B.getPos(), sp3Datum_B.pos, sunPos, false);
							//correct_bdspco_B_F2 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, FREQUENCE_2 - 1, posclk_B.getPos(), sp3Datum_B.pos, sunPos, false);
							//correct_bdspcopcv_A = correct_bdspco_A_F1 - coefficient_IF * (correct_bdspco_A_F1 - correct_bdspco_A_F2); 
							//correct_bdspcopcv_B = correct_bdspco_B_F1 - coefficient_IF * (correct_bdspco_B_F1 - correct_bdspco_B_F2);//
							if(id_Sat > 5)  // IGSO卫星和MEO卫星PCO修正
							{	
								correct_bdspco_A_F1 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, 0, posclk_A.getPos(), sp3Datum_A.pos, sunPos, false);
								correct_bdspco_A_F2 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, 1, posclk_A.getPos(), sp3Datum_A.pos, sunPos, false);
								correct_bdspco_B_F1 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, 0, posclk_B.getPos(), sp3Datum_B.pos, sunPos, false);
								correct_bdspco_B_F2 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, 1, posclk_B.getPos(), sp3Datum_B.pos, sunPos, false);
								correct_bdspcopcv_A = correct_bdspco_A_F1 - coefficient_IF * (correct_bdspco_A_F1 - correct_bdspco_A_F2); 
								correct_bdspcopcv_B = correct_bdspco_B_F1 - coefficient_IF * (correct_bdspco_B_F1 - correct_bdspco_B_F2);
								//fprintf(pfile_PCO,"%3d %s %3d %3d %3d %14.6lf %14.6lf\n",total_iterator,t_Transmit_A.toString().c_str(),b_i + 1,s_i,id_Sat,correct_bdspco_A,correct_bdspco_B);
							}
							else  // GEO卫星PCO修正，20140824，刘俊宏
							{//经测试，分别由t_Transmit_A，t_Transmit_B计算的卫星性体系的差别很小，可以忽略。因此只计算一次卫星星体系即可
								POS3D vecLOS_A = vectorNormal(posclk_A.getPos() - sp3Datum_A.pos);	// 视线单位矢量，卫星指向测站
								POS3D vecLOS_B = vectorNormal(posclk_B.getPos() - sp3Datum_B.pos);	// 视线单位矢量，卫星指向测站
								POS6D bdsOrbposvel;
								bdsOrbposvel.setPos(sp3Datum_A.pos);
								bdsOrbposvel.setVel(sp3Datum_A.vel);													
								POS3D axisvec_R, axisvec_T, axisvec_N;													
								m_TimeCoordConvert.getCoordinateRTNAxisVector(m_TimeCoordConvert.GPST2UT1(t_Transmit_A), bdsOrbposvel, axisvec_R, axisvec_T, axisvec_N);
								POS3D ey, ez;													
								//轨道系坐标轴 R 对应星体系的"-Z"方向
								ez.x = - axisvec_R.x;
								ez.y = - axisvec_R.y;
								ez.z = - axisvec_R.z;
								//轨道系坐标轴 N 对应星体系的"-Y"方向
								ey.x = - axisvec_N.x;
								ey.y = - axisvec_N.y;
								ey.z = - axisvec_N.z;
								//轨道系坐标轴 T 对应星体系的"+X"方向(即ex)
								correct_bdspco_A_F1 = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_BDSAntCorrectionBlk->second, 0, vecLOS_A, axisvec_T, ey, ez, false);
								correct_bdspco_A_F2 = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_BDSAntCorrectionBlk->second, 1, vecLOS_A, axisvec_T, ey, ez, false);
								correct_bdspco_B_F1 = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_BDSAntCorrectionBlk->second, 0, vecLOS_B, axisvec_T, ey, ez, false);
								correct_bdspco_B_F2 = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_BDSAntCorrectionBlk->second, 1, vecLOS_B, axisvec_T, ey, ez, false);
								correct_bdspcopcv_A = correct_bdspco_A_F1 - coefficient_IF * (correct_bdspco_A_F1 - correct_bdspco_A_F2); 
								correct_bdspcopcv_B = correct_bdspco_B_F1 - coefficient_IF * (correct_bdspco_B_F1 - correct_bdspco_B_F2);
								//fprintf(pfile_PCO,"%3d %s %3d %3d %3d %14.6lf %14.6lf\n",total_iterator,t_Transmit_A.toString().c_str(),b_i + 1,s_i,id_Sat,correct_bdspco_A,correct_bdspco_B);
								//fprintf(pfile_PCO,"%14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf\n",axisvec_T_A.x,axisvec_T_A.y,axisvec_T_A.z,axisvec_T_B.x,axisvec_T_B.y,axisvec_T_B.z,ey_A.x,ey_A.y,ey_A.z,ey_B.x,ey_B.y,ey_B.z);
							}
						}				
						double obscorrected_value =  correct_gpsrelativity_A - correct_gpsrelativity_B
							                       + correct_bdspcopcv_A - correct_bdspcopcv_B
												   -(distance_A - distance_B);
						datum_j.obscorrected_value = obscorrected_value;
						datum_j.valid = 1; // 判断 obscorrected_value 是否有效
					}
					cvEpochList[s_i].mapDatum.insert(map<int, cvElement>::value_type(id_Sat, datum_j));	
				}
			}
			// 记录唯一标识的弧段标号, 用于周跳标记
			int id_arc = 0;
			for(size_t s_i = 0; s_i < editedObsSatList.size(); s_i++)
			{
				size_t count_obs_i = editedObsSatList[s_i].editedObs.size(); // 观测时间序列数据个数(某颗固定BDS卫星)
				int id_Sat = editedObsSatList[s_i].Id;
				double *pEpochTime    = new double[count_obs_i]; // 相对时间序列
				int    *pEditedFlag   = new int   [count_obs_i]; // 原有编辑标记序列 0-正常; 1-新的周跳标记; 2-异常
				int    *pEpochId      = new int   [count_obs_i];
				Rinex2_1_EditedSdObsEpochMap::iterator it0 = editedObsSatList[s_i].editedObs.begin();
				GPST t0 = it0->first; // 初始时间 - 用于计算相对时间(秒)
				int j = 0;
				for(Rinex2_1_EditedSdObsEpochMap::iterator it = editedObsSatList[s_i].editedObs.begin(); it != editedObsSatList[s_i].editedObs.end(); ++it)
				{
					pEpochTime[j] = it->first - t0;
					pEpochId[j] = it->second.nObsTime;
					// 用 pEditedFlag 标记据数据的有效性, 用来进行数据的遴选
					if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 == TYPE_EDITEDMARK_NORMAL && it->second.obsTypeList[nObsTypes_L2].byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
					{
						pEditedFlag[j] = 0; // 正常
					} 
					else if(it->second.obsTypeList[nObsTypes_L1].byEditedMark1 == TYPE_EDITEDMARK_SLIP || it->second.obsTypeList[nObsTypes_L2].byEditedMark1 == TYPE_EDITEDMARK_SLIP)
					{
						pEditedFlag[j] = 1; // 新的周跳标记
					}
					else
					{
						pEditedFlag[j] = 2; // 异常点
					}
					j++;
				}
				// 每个有效连续跟踪弧段数据, 对应一个新的模糊度参数, 进行处理
				size_t k   = 0; // 记录新弧段起始点
				size_t k_i = k; // 记录新弧段终止点
				while(1)
				{
					if(k_i + 1 >= count_obs_i) // k_i 为时间序列终点
						goto newArc_0;
					else
					{
						// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
						if((pEpochTime[k_i + 1] - pEpochTime[k_i] <= m_podParaDefine.max_arclengh)
						&& (pEditedFlag[k_i + 1] != 1))
						{
							k_i++;
							continue;
						}
						// 如果相邻两点的间隔时间 > max_arclengh, 或者下一点是周跳点则判断为新弧段
						else // k_i + 1 为新弧段的起点
							goto newArc_0;
					}
					newArc_0:  // 本弧段[k，k_i]数据处理 
					{
						// 记录唯一标识的弧段标号
						id_arc++;
						for(size_t s_k = k; s_k <= k_i; s_k++)
						{
							map<int, cvElement>::iterator it = cvEpochList[pEpochId[s_k]].mapDatum.find(id_Sat);
							if(it != cvEpochList[pEpochId[s_k]].mapDatum.end())
								it->second.id_arc = id_arc;
						}
						if(k_i + 1 >= count_obs_i) // k_i为时间序列终点, 跳出
							break;
						else  
						{   
							k   = k_i + 1; // 新弧段的起点设置
							k_i = k;
							continue;
						}
					}
				}
				delete pEpochTime;
				delete pEditedFlag;
				delete pEpochId;
			}
			
			int id_Arc = 0;
			for(size_t s_i = 0; s_i < editedObsSatList.size(); s_i++)
			{
				size_t count_obs_i = editedObsSatList[s_i].editedObs.size(); // 观测时间序列数据个数(某颗固定BDS卫星)
				int id_Sat = editedObsSatList[s_i].Id;
				double *pEpochTime    = new double[count_obs_i]; // 相对时间序列
				double *pP_IF         = new double[count_obs_i]; // 伪距观测数据序列
				double *pL_IF         = new double[count_obs_i]; // 相位观测数据序列
				int    *pEditedFlag   = new int   [count_obs_i]; // 原有编辑标记序列 0-正常; 1-新的周跳标记; 2-异常
				int    *pEpochId      = new int   [count_obs_i];
				int    *pSlip         = new int   [count_obs_i]; // 新编辑的记录序列 0-未编辑; 1-新的周跳标记; 2-新的异常点
				Rinex2_1_EditedSdObsEpochMap::iterator it0 = editedObsSatList[s_i].editedObs.begin();
				GPST t0 = it0->first; // 初始时间 - 用于计算相对时间(秒)
				int j = 0;
				for(Rinex2_1_EditedSdObsEpochMap::iterator it = editedObsSatList[s_i].editedObs.begin(); it != editedObsSatList[s_i].editedObs.end(); ++it)
				{
					pEpochTime[j] = it->first - t0;
					pEpochId[j] = it->second.nObsTime;
					Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[nObsTypes_P1];
					Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[nObsTypes_P2];
					Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[nObsTypes_L1];
					Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[nObsTypes_L2];
					pP_IF[j] = P1.obs.data - (P1.obs.data - P2.obs.data) * coefficient_IF;
					pL_IF[j] = BD_WAVELENGTH_L1 * L1.obs.data - (BD_WAVELENGTH_L1 * L1.obs.data - BD_WAVELENGTH_L2 * L2.obs.data) * coefficient_IF;
					// 用 pEditedFlag 标记据数据的有效性, 用来进行数据的遴选
                    pSlip[j] = 0; // 初始化未作任何标记
					if(L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
					{
						pEditedFlag[j] = 0; // 正常
					} 
					else if(L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP || L2.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
					{
						pEditedFlag[j] = 1; // 新的周跳标记
					}
					else
					{
						pEditedFlag[j] = 2; // 异常点
					}
					j++;
				}
				// 每个有效连续跟踪弧段数据, 对应一个新的模糊度参数, 进行处理
				size_t k   = 0; // 记录新弧段起始点
				size_t k_i = k; // 记录新弧段终止点
				while(1)
				{
					if(k_i + 1 >= count_obs_i) // k_i 为时间序列终点
						goto newArc;
					else
					{
						// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
						if((pEpochTime[k_i + 1] - pEpochTime[k_i] <= m_podParaDefine.max_arclengh)
						&& (pEditedFlag[k_i + 1] != 1))
						{
							k_i++;
							continue;
						}
						// 如果相邻两点的间隔时间 > max_arclengh, 或者下一点是周跳点则判断为新弧段
						else // k_i + 1 为新弧段的起点
							goto newArc;
					}
					newArc:  // 本弧段[k，k_i]数据处理 
					{
						id_Arc++;
						vector<size_t> unknownPointlist;
						unknownPointlist.clear();
						for(size_t s_k = k; s_k <= k_i; s_k++)
						{
							if(pEditedFlag[s_k] == 0 || pEditedFlag[s_k] == 1)
								unknownPointlist.push_back(s_k); // 仅对正常点进行编辑
						}
						size_t count_unknownpoints = unknownPointlist.size();
						if(count_unknownpoints > m_podParaDefine.min_arcpointcount)
						{
							vector<int> slipMarklist; // 1 - 周跳; 0 - 正常点
							slipMarklist.resize(count_unknownpoints);
							for(size_t s_ii = 1; s_ii < count_unknownpoints; s_ii++)
							{
								bool slipFlag;
								int nObsTime_j_1 = pEpochId[unknownPointlist[s_ii - 1]];
								int nObsTime_j   = pEpochId[unknownPointlist[s_ii]];
								double max_res_ddd; // 记录双差编辑残差
								if(obsTriDiFFEdited_LIF(nObsTypes_L1, nObsTypes_L2, id_Sat, editedSdObsFile.m_data[nObsTime_j_1], editedSdObsFile.m_data[nObsTime_j], cvEpochList[nObsTime_j_1].mapDatum, cvEpochList[nObsTime_j].mapDatum, max_res_ddd, slipFlag, m_podParaDefine.threshold_outlier_DDDEdit))
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
									pSlip[unknownPointlist[s_ii]] = 2; // 标记为野值
								}
							}
							// 第一点的判断
							if(slipMarklist[1] == 1)
								pSlip[unknownPointlist[0]] = 2;
							// 最后一点直接判断
							if(slipMarklist[count_unknownpoints - 1] == 1)
								pSlip[unknownPointlist[count_unknownpoints - 1]] = 2;
							size_t s_ii = 0;
							while(s_ii < unknownPointlist.size())
							{
								if(pSlip[unknownPointlist[s_ii]] == 0)
									s_ii++;
								else// 先将野值 erase
									unknownPointlist.erase(unknownPointlist.begin() + s_ii);
							}
							count_unknownpoints = unknownPointlist.size();
							// 进行周跳探测
							double res_ddd_arc_i = 0;
							int count_res_ddd_arc_i = 0;
							for(size_t s_ii = 1; s_ii < count_unknownpoints; s_ii++)
							{
								bool slipFlag;
								int nObsTime_j_1 = pEpochId[unknownPointlist[s_ii - 1]];
								int nObsTime_j   = pEpochId[unknownPointlist[s_ii]];
								double max_res_ddd;
								if(obsTriDiFFEdited_LIF(nObsTypes_L1, nObsTypes_L2, id_Sat, editedSdObsFile.m_data[nObsTime_j_1], editedSdObsFile.m_data[nObsTime_j], cvEpochList[nObsTime_j_1].mapDatum, cvEpochList[nObsTime_j].mapDatum, max_res_ddd, slipFlag, m_podParaDefine.threshold_slip_DDDEdit))
								{
									res_ddd_arc_i += pow(max_res_ddd, 2);
									count_res_ddd_arc_i ++;
									
									
									rms_sdobs += pow(max_res_ddd, 2);
									count_stat++;
									fprintf(pFile, "%s  %02d  %04d  %10.4lf\n", editedSdObsFile.m_data[pEpochId[unknownPointlist[s_ii]]].t.toString().c_str(),
									                                            id_Sat,
																				unknownPointlist[s_ii],
																				max_res_ddd);
									if(slipFlag)
									{
										sprintf(info, "C%02d  %s  %14.4f   +", id_Sat, editedSdObsFile.m_data[pEpochId[unknownPointlist[s_ii]]].t.toString().c_str(), max_res_ddd);
										RuningInfoFile::Add(info);
										pSlip[unknownPointlist[s_ii]] = 1;
										count_slip++;
									}
								}
								else
								{// 无法判断, 暂不标记
									//sprintf(info, "G%02d  %s  %16.4f ----", id_Sat, editedSdObsFile.m_data[pEpochId[unknownPointlist[s_ii]]].t.toString().c_str(), max_res_ddd);
									//RuningInfoFile::Add(info);
									//pSlip[unknownPointlist[s_ii]] = 1;
									//count_slip++;
								}
							}
							if(count_res_ddd_arc_i > 0)
							{
								res_ddd_arc_i = sqrt(res_ddd_arc_i / count_res_ddd_arc_i);
								//sprintf(info, "G%02d  %4d  %16.4f %d", id_Sat, id_Arc, res_ddd_arc_i, count_res_ddd_arc_i);
								//RuningInfoFile::Add(info);
							}
							// 第一点的判断
							if(count_unknownpoints > 1)
							{
								if(pSlip[unknownPointlist[1]] == 1)
									pSlip[unknownPointlist[0]] = 2;
							}
							// 记录 pSlip 标记
							// pEditedFlag 1 0 0 0 0 0 2 0 0 0 0 2 0
							// pSlip       0 0 2 0 0 0 0 0 1 0 0 0 0
                            // 保留 "pSlip = 1/2" 的编辑标记, 进行更新
							for(size_t s_k = k; s_k <= k_i; s_k++)
							{
								if(pSlip[s_k] == 1)
								{
									editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L1].byEditedMark1 = TYPE_EDITEDMARK_SLIP;
									editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L1].byEditedMark2 = 0;
									editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L2].byEditedMark1 = TYPE_EDITEDMARK_SLIP;
									editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L2].byEditedMark2 = 0;
								}
								if(pSlip[s_k] == 2)
								{
									editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L1].byEditedMark1 = TYPE_EDITEDMARK_OUTLIER;
									editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L1].byEditedMark2 = 0;
									editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L2].byEditedMark1 = TYPE_EDITEDMARK_OUTLIER;
									editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L2].byEditedMark2 = 0;	
								}
							}
							// 将首个 "pSlip = 0或1"  标记更新为 pEditedFlag[k] （如果pEditedFlag为周跳）
							if(pEditedFlag[k] == 1)
							{
								for(size_t s_k = k; s_k <= k_i; s_k++)
								{
									if(pSlip[s_k] == 0 || pSlip[s_k] == 1)
									{
										editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L1].byEditedMark1 = TYPE_EDITEDMARK_SLIP;
										editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L1].byEditedMark2 = 0;
										editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L2].byEditedMark1 = TYPE_EDITEDMARK_SLIP;
										editedSdObsFile.m_data[pEpochId[s_k]].editedObs[id_Sat].obsTypeList[nObsTypes_L2].byEditedMark2 = 0;
										break;
									}
								}
							}
						}
						if(k_i + 1 >= count_obs_i) // k_i为时间序列终点, 跳出
							break;
						else  
						{   
							k   = k_i + 1; // 新弧段的起点设置
							k_i = k;
							continue;
						}
					}
				}
				delete pEpochTime;
				delete pP_IF;
				delete pL_IF;
				delete pEditedFlag;
				delete pEpochId;
				delete pSlip;
			}
			rms_sdobs = sqrt(rms_sdobs / count_stat);
			sprintf(info, "%s    %16.4f %3d", outputFileName.c_str(), rms_sdobs, count_slip);
            RuningInfoFile::Add(info);
			if(pFile)
				fclose(pFile);
			return true;
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
		void BDSatDynPOD::weighting_Elevation(double Elevation, double& weight_P_IF, double& weight_L_IF)
		{
			if(Elevation <= m_podParaDefine.min_elevation)
			{
				weight_P_IF = 0.0;
				weight_L_IF = 0.0;
			}
			else
			{
				if(m_podParaDefine.bOn_WeightElevation)
				{
					//weight_P_IF = sin(Elevation*PI/180);	// ESA软件的策略
					//weight_L_IF = sin(Elevation*PI/180);
					if(Elevation <= 30)
					{
						weight_P_IF = 2*sin(Elevation*PI/180);	// GFZ软件的策略
						weight_L_IF = 2*sin(Elevation*PI/180);
						//weight_P_IF = sin(2*Elevation*PI/180);	// GFZ软件的策略
						//weight_L_IF = sin(2*Elevation*PI/180);
					}
					else
					{
						weight_P_IF = 1.0;
						weight_L_IF = 1.0;
					}
				}
				else
				{
					weight_P_IF = 1.0;
					weight_L_IF = 1.0;
				}
			}
		}

		// 子程序名称： lambdaSelected 
		// 功能：利用 LAMBDA 方法进行寻优, 搜索最佳的模糊度组合, 以满足模糊度固定的准则
		//       采用的方法是逐个寻优剔除
		// 变量类型：matAFloat         : 浮点解
		//           matQahat          : 浮点解协方差矩阵
		//           matSqnorm         : 对应的最优解和次优解
		//           matAFixed         : 固定解
		//           matSelectedFlag   : 选择固定的标记
		// 输入：matAFloat, matQahat, matSelectedFlag
		// 输出：matAFixed, matSqnorm
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2010/04/02
		// 版本时间：
		// 修改记录：
		// 备注：
		//bool BDSatDynPOD::lambdaSelected(Matrix matAFloat, Matrix matQahat, Matrix& matSqnorm, Matrix& matAFixed, Matrix matSelectedFlag)
		//{
		//	vector<int> validIndexList;
		//	for(int i = 0; i < matAFloat.GetNumRows(); i++)
		//	{
		//		if(matSelectedFlag.GetElement(i, 0) == 1.0)
		//			validIndexList.push_back(i);
		//	}
		//	int count_valid = int(validIndexList.size());
  //          Matrix matAFloat_valid(count_valid, count_valid);
		//    Matrix matQahat_valid(count_valid, count_valid);
		//	for(int i = 0; i < count_valid; i++)
		//	{
		//		matAFloat_valid.SetElement(i, 0, matAFloat.GetElement(validIndexList[i], 0));
		//		for(int j = 0; j < count_valid; j++)
		//			matQahat_valid.SetElement(i, j, matQahat.GetElement(validIndexList[i], validIndexList[j]));
		//	}
		//	Matrix matAFixed_valid(count_valid, 1);
		//	matAFixed = matAFloat;
		//	if(lambda::main(matAFloat_valid, matQahat_valid, matSqnorm, matAFixed_valid, 2))
		//	{
		//		for(int i = 0; i < count_valid; i++)
		//			matAFixed.SetElement(validIndexList[i], 0, matAFixed_valid.GetElement(i, 0));
		//		return true;
		//	}
		//	return false;//
		//}

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
		// 创建时间：2012/12/31
		// 版本时间：2012/12/31
		// 修改记录：
		// 备注： 默认步长选取75s, 11阶, 参考MIT
		bool BDSatDynPOD::adamsCowell_ac(TDT t0_Interp, TDT t1_Interp, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h, int q)
		{
			orbitlist_ac.clear();
			matRtPartiallist_ac.clear();
			TDT  t_Begin = t0_Interp; // 起始时间
			TDT  t_End   = t1_Interp; // 终止时间
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
			return true;
		}

		bool BDSatDynPOD::mainNetPod_dd(string inputSp3FilePath, string outputSp3FilePath, SatOrbEstParameter& paraSatOrbEst, GPST t0, GPST t1, double h_sp3,bool bResEdit)
		{
			int    FREQUENCE_1 = 1,FREQUENCE_2 = 2;                                          // 使用的数据频点，1代表B1，2代表B2, 3代表B3
			double BD_WAVELENGTH_W    =  SPEED_LIGHT / (BD_FREQUENCE_L1 - BD_FREQUENCE_L2);  // 宽巷载波波长
            double BD_WAVELENGTH_N    =  SPEED_LIGHT / (BD_FREQUENCE_L1 + BD_FREQUENCE_L2);  // 窄巷载波波长
			double coeff_mw = BD_WAVELENGTH_L2 * pow(BD_FREQUENCE_L2, 2) / (pow(BD_FREQUENCE_L1, 2) - pow(BD_FREQUENCE_L2, 2));
			double coefficient_IF     = 1 / (1 - pow(BD_FREQUENCE_L1 / BD_FREQUENCE_L2, 2)); // 无电离层组合系数
			double coefficient_IF_L1 = 1 / (1 - pow(BD_FREQUENCE_L2 / BD_FREQUENCE_L1, 2)); // 无电离层组合系数
			double coefficient_IF_L2 = 1 / (1 - pow(BD_FREQUENCE_L1 / BD_FREQUENCE_L2, 2)); // 无电离层组合系数
			string folder = outputSp3FilePath.substr(0, outputSp3FilePath.find_last_of("\\"));
			string sp3FileName = outputSp3FilePath.substr(outputSp3FilePath.find_last_of("\\") + 1);
			string sp3FileName_noexp = sp3FileName.substr(0, sp3FileName.find_last_of("."));
			char   codeOCPpath[300];
			char   phaseOCPpath[300];			
			char   info[200];
			sprintf(codeOCPpath,"%s\\%s_code.oc",folder.c_str(),sp3FileName_noexp.c_str());
			sprintf(phaseOCPpath,"%s\\%s_phase.oc",folder.c_str(),sp3FileName_noexp.c_str());			
			// 创建预处理目录
			string strPreprocFolder = folder + "\\Preproc";
			_mkdir(strPreprocFolder.c_str());
			SP3File sp3InputFile_j2000; // 用于"三差"观测数据编辑和 windup 修正
			if(!sp3InputFile_j2000.open(inputSp3FilePath))
				return false;
			for(size_t s_i = 0; s_i < sp3InputFile_j2000.m_data.size(); s_i++)
			{
				for(SP3SatMap::iterator it = sp3InputFile_j2000.m_data[s_i].sp3.begin(); it != sp3InputFile_j2000.m_data[s_i].sp3.end(); ++it)
				{
					double x_ecf[3];
					double x_j2000[3];
					x_ecf[0] = it->second.pos.x * 1000;  
					x_ecf[1] = it->second.pos.y * 1000; 
					x_ecf[2] = it->second.pos.z * 1000;
					m_TimeCoordConvert.ECEF_J2000(sp3InputFile_j2000.m_data[s_i].t, x_j2000, x_ecf, false);
					it->second.pos.x = x_j2000[0] / 1000;  
					it->second.pos.y = x_j2000[1] / 1000; 
					it->second.pos.z = x_j2000[2] / 1000;
				}
			}
			//// 获取 BDS 卫星发射天线相位中心偏移 ( 星固系, 用于GPS发射天线相位中心修正 )//2013/9/16,刘俊宏			
			//map<int, AntCorrectionBlk> mapBDSAntCorrectionBlk;m_mapBDSAntCorrectionBlk
			if(m_podParaDefine.bOn_BDSAntPCO)
			{
				for(int i = 1; i <= MAX_PRN_BD;i++)
				{
					int id_PRN = i;					
					char sNN[4];
					sprintf(sNN, "%1c%02d",'C',id_PRN);
					sNN[3] = '\0';
					AntCorrectionBlk datablk;
					if(m_AtxFile.getAntCorrectBlk(sNN, t0, datablk))
						m_mapBDSAntCorrectionBlk.insert(map<int, AntCorrectionBlk>::value_type(id_PRN, datablk));				
				}
			}
			// 1. 选取基线
			vector<int> staBaseLineIdList_A;
			vector<int> staBaseLineIdList_B;
			vector<POS3D>  staPosList;
			vector<string> staNameList;
			for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)
			{
				if(it->second.bUsed)
				{
					staPosList.push_back(it->second.posvel.getPos());
					staNameList.push_back(it->first);					
				}
			}
			GPSPod::GPSMeoSatDynPOD::getStaBaselineList_MiniPath(staPosList, staBaseLineIdList_A, staBaseLineIdList_B);
			 // 输出测站信息
			RuningInfoFile::Add("测站信息==============================");
			for(size_t s_i = 0; s_i < staPosList.size(); s_i++)
			{
				BLH blh;
				TimeCoordConvert::XYZ2BLH(staPosList[s_i], blh);
				sprintf(info, "%3d %s %16.4f %16.4f", int(s_i + 1), staNameList[s_i].c_str(), blh.B, blh.L);
				RuningInfoFile::Add(info);
			}            		
			// 形成单差文件
			// 输出三差编辑信息
			RuningInfoFile::Add("形成单差文件并进行三差编辑================================================");
			m_staBaseLineList.clear();
			for(size_t s_i = 0; s_i < staBaseLineIdList_A.size(); s_i++)
			{
				StaBaselineDatum blDatum;
				blDatum.name_A   = staNameList[staBaseLineIdList_A[s_i]];
                blDatum.posvel_A = m_mapStaDatum[blDatum.name_A].posvel;
				blDatum.name_B   = staNameList[staBaseLineIdList_B[s_i]];
				blDatum.posvel_B = m_mapStaDatum[blDatum.name_B].posvel;
				blDatum.arpAnt_A = m_mapStaDatum[blDatum.name_A].arpAnt; // 记录 ARP 数据. 用于整网解算修正 
				blDatum.arpAnt_B = m_mapStaDatum[blDatum.name_B].arpAnt; // 记录 ARP 数据. 用于整网解算修正
				char outputFileName[300];
				sprintf(outputFileName, "%s\\Preproc\\%s_%s.txt", folder.c_str(), blDatum.name_A.c_str(), blDatum.name_B.c_str());
				if(obsSingleDifferencePreproc(m_mapStaDatum[blDatum.name_A].obsFile, m_mapStaDatum[blDatum.name_B].obsFile, blDatum.editedSdObsFile))
				{		
					// 根据 sp3 数据对每条基线数据 editedSdObsFile 逐条进行再编辑, 20140414
					//测站天线概略位置, 概略钟差
					POS3D posAnt_A;
					POS3D posAnt_B;
					TimeCoordConvert::ENU2ECF(m_mapStaDatum[blDatum.name_A].posvel.getPos(), blDatum.arpAnt_A, posAnt_A);
					TimeCoordConvert::ENU2ECF(m_mapStaDatum[blDatum.name_B].posvel.getPos(), blDatum.arpAnt_B, posAnt_B);
					if(m_podParaDefine.bOn_DDDEdit)
					{//输出基线产品 进行三差编辑
						if(mainTriDiFFEdited(sp3InputFile_j2000, blDatum.editedSdObsFile, posAnt_A, posAnt_B, outputFileName))
						{
							//char szFileName[300];
							//sprintf(szFileName, "%s\\Preproc\\%s_%s.sdo", folder.c_str(), blDatum.name_A.c_str(), blDatum.name_B.c_str());
							//blDatum.editedSdObsFile.write(szFileName);//
							m_staBaseLineList.push_back(blDatum);
							m_mapStaDatum[blDatum.name_A].bUsed = true;
							m_mapStaDatum[blDatum.name_B].bUsed = true;
						}
					}
					else
					{//输出基线产品 不进行三差编辑
						//char szFileName[300];
						//sprintf(szFileName, "%s\\Preproc\\%s_%s_NOTriEdited.sdo", folder.c_str(), blDatum.name_A.c_str(), blDatum.name_B.c_str());
						//blDatum.editedSdObsFile.write(szFileName);//
						m_staBaseLineList.push_back(blDatum);
						m_mapStaDatum[blDatum.name_A].bUsed = true;
						m_mapStaDatum[blDatum.name_B].bUsed = true;	
					}
				}
				else
				{
					sprintf(info, "第%3d条基线没有观测数据！",s_i + 1);
					RuningInfoFile::Add(info);
					printf("%s",info);					
					return false;
				}
			}
			RuningInfoFile::Add("三差编辑完毕==============================================================");
			// 输出基线选取结果以、单差历元个数和双差数据个数
			RuningInfoFile::Add("基线信息==================================================================");
            for(size_t s_i = 0; s_i < staBaseLineIdList_A.size(); s_i++)
			{
				BLH blh_A, blh_B;
				TimeCoordConvert::XYZ2BLH(staPosList[staBaseLineIdList_A[s_i]], blh_A);
				TimeCoordConvert::XYZ2BLH(staPosList[staBaseLineIdList_B[s_i]], blh_B);
				POS3D AB = staPosList[staBaseLineIdList_B[s_i]] - staPosList[staBaseLineIdList_A[s_i]];
				double distance = sqrt(AB.x * AB.x + AB.y * AB.y + AB.z * AB.z);
				int DD_count = 0;
				for(size_t s_j = 0; s_j < m_staBaseLineList[s_i].editedSdObsFile.m_data.size(); s_j ++)
				{
					int SD_count = int(m_staBaseLineList[s_i].editedSdObsFile.m_data[s_j].editedObs.size());
					if( SD_count >= 2)
						DD_count += SD_count - 1;					
				}
				sprintf(info, "%3d %s %s %12.4f %12.4f %12.4f %12.4f %13.4f %10d %10d",
					          int(s_i + 1),
					          staNameList[staBaseLineIdList_A[s_i]].c_str(),
							  staNameList[staBaseLineIdList_B[s_i]].c_str(),                             
							  blh_A.B,
							  blh_A.L,							 
							  blh_B.B,
							  blh_B.L,							 
							  distance,
							  int(m_staBaseLineList[s_i].editedSdObsFile.m_data.size()),
							  DD_count);
				RuningInfoFile::Add(info);
			}	
			// 清除 m_mapStaDatum 中无效的测站
			// 统计待估测站的位置参数个数
			//FILE *p_tro = fopen("C:\\tro_time.cpp","w+");
			int count_StaParameter = 0;
			int k = 0;
			int sta_index = 0;
			StaDatumMap::iterator it_Sta = m_mapStaDatum.begin(); 
			while(it_Sta != m_mapStaDatum.end())
			{
				if(!it_Sta->second.bUsed)
				{
					StaDatumMap::iterator jt = it_Sta;
					++it_Sta;
					m_mapStaDatum.erase(jt);
				}
				else
				{
					it_Sta->second.id = k;					
					if(it_Sta->second.bOnEst_StaPos)
					{
						it_Sta->second.indexEst_StaPos = sta_index;
						sta_index++;
						count_StaParameter += 3;
					}
					++it_Sta;
					k++;
				}
			}
			// 通过单差文件确定测站的观测数据起止时间，方便确定对流层的估计区间，2013/7/8，刘俊宏
			if(m_podParaDefine.bOnEst_StaTropZenithDelay)
			{
				for(size_t s_i = 0; s_i < m_staBaseLineList.size(); s_i++)
				{//根据所有的基线给观测数据起止时间赋初值
					m_mapStaDatum[m_staBaseLineList[s_i].name_A].t0 = m_staBaseLineList[s_i].editedSdObsFile.m_data.front().t;
					m_mapStaDatum[m_staBaseLineList[s_i].name_B].t0 = m_staBaseLineList[s_i].editedSdObsFile.m_data.front().t;
					m_mapStaDatum[m_staBaseLineList[s_i].name_A].t1 = m_staBaseLineList[s_i].editedSdObsFile.m_data.back().t;
					m_mapStaDatum[m_staBaseLineList[s_i].name_B].t1 = m_staBaseLineList[s_i].editedSdObsFile.m_data.back().t;
				}
				for(size_t s_i = 0; s_i < m_staBaseLineList.size(); s_i++)
				{//根据所有的基线调整观测数据起止时间
					DayTime  t0_init = m_mapStaDatum[m_staBaseLineList[s_i].name_A].t0;
					DayTime  t0_new  = m_staBaseLineList[s_i].editedSdObsFile.m_data.front().t;
					if(t0_init - t0_new > 0)
					  m_mapStaDatum[m_staBaseLineList[s_i].name_A].t0 = t0_new;
					DayTime  t1_init = m_mapStaDatum[m_staBaseLineList[s_i].name_A].t1;
					DayTime  t1_new  = m_staBaseLineList[s_i].editedSdObsFile.m_data.back().t;
					if(t1_init - t1_new < 0)
					  m_mapStaDatum[m_staBaseLineList[s_i].name_A].t1 = t1_new;
					t0_init = m_mapStaDatum[m_staBaseLineList[s_i].name_B].t0;
					t0_new  = m_staBaseLineList[s_i].editedSdObsFile.m_data.front().t;
					if(t0_init - t0_new > 0)
					  m_mapStaDatum[m_staBaseLineList[s_i].name_B].t0 = t0_new;
					t1_init = m_mapStaDatum[m_staBaseLineList[s_i].name_B].t1;
					t1_new  = m_staBaseLineList[s_i].editedSdObsFile.m_data.back().t;
					if(t1_init - t1_new < 0)
					  m_mapStaDatum[m_staBaseLineList[s_i].name_B].t1 = t1_new;					
				}
				for(StaDatumMap::iterator it = m_mapStaDatum.begin();it != m_mapStaDatum.end();++it)
					it->second.init(m_podParaDefine.apriorityWet_TZD_period,m_podParaDefine.apriorityWet_TZD);
			}

			//fclose(p_tro);
			// 2. 统计待解算卫星观测数据个数, 从而初步确定可解 BD 卫星个数
			paraSatOrbEst.satParaList.clear();
			int pCountObs[MAX_PRN_BD];
			int pCountArc_sat[MAX_PRN_GPS];
			for(int i = 0; i < MAX_PRN_BD; i++)
			{
				pCountObs[i] = 0;
				pCountArc_sat[i] = 0;
			}
			for(size_t s_i = 0; s_i < m_staBaseLineList.size(); s_i++)
			{
				for(size_t s_j = 0; s_j < m_staBaseLineList[s_i].editedSdObsFile.m_data.size(); s_j++)
				{
					for(Rinex2_1_EditedSdObsSatMap::iterator it = m_staBaseLineList[s_i].editedSdObsFile.m_data[s_j].editedObs.begin(); it != m_staBaseLineList[s_i].editedSdObsFile.m_data[s_j].editedObs.end(); ++it)
					{
						pCountObs[it->first]++;
					}
				}
			}
			RuningInfoFile::Add("每颗卫星的单差数据========================================================");
			for(int i = 0; i < MAX_PRN_BD; i++)
			{
				if(pCountObs[i] > 0)
				{// 无效卫星可以在此处进行剔除
					SatDatum datum_i;
					datum_i.count_obs = pCountObs[i];
					datum_i.index = int(paraSatOrbEst.satParaList.size());
					//if(i > 5)
					paraSatOrbEst.satParaList.insert(SatDatumMap::value_type(i, datum_i));
					sprintf(info, "C%02d的单差观测数据个数  %10d", i, pCountObs[i]);
					RuningInfoFile::Add(info);
					//printf("%02d   %d\n",i, datum_i.count_obs);
				}
			}		
			
			// 清理无效卫星观测数据???????????????????????????????????
			// ???????????????????????????????????????????????????????
			// 测站天线相位中心结构
			map<string, AntCorrectionBlk> mapRecAntCorrectionBlk;
			for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); it++)
			{
				AntCorrectionBlk datablk;
				if(m_AtxFile.getAntCorrectBlk(it->second.szAntType, t0, datablk, false))
					mapRecAntCorrectionBlk.insert(map<string, AntCorrectionBlk>::value_type(it->second.szAntType, datablk));
				else
				{
					char szAntType_NONE[20 + 1] = "                NONE";// 天线类型
					for(int k = 0; k < 16; k++)
						szAntType_NONE[k] = it->second.szAntType[k];
					if(m_AtxFile.getAntCorrectBlk(szAntType_NONE, t0, datablk, false))
					{
						mapRecAntCorrectionBlk.insert(map<string, AntCorrectionBlk>::value_type(it->second.szAntType, datablk));
						sprintf(info, "警告: %s 接收天线\"%s\"相位中心信息采用\"%s\"替代!", it->first.c_str(), it->second.szAntType, szAntType_NONE);
						RuningInfoFile::Add(info);
					}
					else
					{
						sprintf(info, "警告: %s 接收天线\"%s\"相位中心信息缺失!", it->first.c_str(), it->second.szAntType);
						RuningInfoFile::Add(info);
					}
				}
			}
			RuningInfoFile::Add("宽巷模糊度解算信息====================================================");
			int count_All_FixedAmbiguity = 0;
			int count_All_UnFixedAmbiguity = 0;
			const double zeroweight_code  = 1.0E-8;
			const double zeroweight_phase = 1.0E-6;
			//FILE *pfile_W = fopen("F:\\MW_BDS.txt","w+");
			for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
			{
				// 确认观测数据是否完整
				int nObsTypes_P1 = -1, nObsTypes_P2 = -1, nObsTypes_L1 = -1, nObsTypes_L2 = -1;
				for(int i = 0; i < m_staBaseLineList[b_i].editedSdObsFile.m_header.byObsTypes; i++)
				{
					if(m_staBaseLineList[b_i].editedSdObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P1)
						nObsTypes_P1 = i;
					if(m_staBaseLineList[b_i].editedSdObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_P2)
						nObsTypes_P2 = i;
					if(m_staBaseLineList[b_i].editedSdObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L1)
						nObsTypes_L1 = i;
					if(m_staBaseLineList[b_i].editedSdObsFile.m_header.pbyObsTypeList[i] == TYPE_OBS_L2)
						nObsTypes_L2 = i;
				}
				if(nObsTypes_P1 == -1 || nObsTypes_P2 == -1 ||nObsTypes_L1 == -1 || nObsTypes_L2 == -1)
					continue;
				vector<Rinex2_1_EditedSdObsSat> editedObsSatList;
				if(!m_staBaseLineList[b_i].editedSdObsFile.getEditedObsSatList(editedObsSatList))  
					continue;
				// 整理有效观测时刻, 并作为最初的遴选观测数据条件
				size_t count_epoch = m_staBaseLineList[b_i].editedSdObsFile.m_data.size(); // 观测历元个数
				m_staBaseLineList[b_i].dynEpochList.resize(count_epoch);				
				int s_index = -1; // 有效时间标签, 初始化为 0
				for(size_t s_i = 0; s_i < count_epoch; s_i++)
				{
					// 条件: 可视BD卫星个数大于等于 3
					bool bValid = true;
					int eyeableGPSCount = 0;
					for(Rinex2_1_EditedSdObsSatMap::iterator it = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs.begin(); it != m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs.end(); ++it)
					{
						double min_elevation = m_podParaDefine.min_elevation;
						//if(it->first <= 5)
						//	min_elevation = 12;//测试合适GEO卫星的观测数据截止角，20140824，刘俊宏
						Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[nObsTypes_P1];
						Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[nObsTypes_P2];
						if( it->second.Elevation_A >= min_elevation //m_podParaDefine.min_elevation
						 && it->second.Elevation_B >= min_elevation //m_podParaDefine.min_elevation
						 && P1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL
						 && P2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
							eyeableGPSCount++;
					}
					if(eyeableGPSCount <= m_podParaDefine.min_eyeableGPSCount)//2013/04/17;//Mgex测站稀少，基线长，大量存在一条基线只有2颗共视卫星的情况，因此将可视卫星条件为3改为2。2013/10/4
						bValid = false;
					if(bValid)
					{
						m_staBaseLineList[b_i].dynEpochList[s_i].eyeableGPSCount = eyeableGPSCount;
						GPST t_Receive_A = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t - m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].A_clock / SPEED_LIGHT; // 接收机接收信号时间
						GPST t_Receive_B = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t - m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].B_clock / SPEED_LIGHT;
						for(Rinex2_1_EditedSdObsSatMap::iterator it = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs.begin(); it != m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs.end(); ++it)
						{
							Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[nObsTypes_P1];
							Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[nObsTypes_P2];
							Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[nObsTypes_L1];
							Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[nObsTypes_L2];
							PODEpochElement datum_j;
							double weight_P_IF = 1.0, weight_L_IF = 1.0;							
							// 单差数据综合A、B测站平均高度角(GPS).20140929
							weighting_Elevation((it->second.Elevation_B + it->second.Elevation_A) * 0.5, weight_P_IF, weight_L_IF);
							// 理论观测观测权值
							datum_j.weightCode   = m_podParaDefine.apriorityRms_LIF / m_podParaDefine.apriorityRms_PIF;
							datum_j.weightPhase  = 1.0;
							datum_j.weightCode  *= weight_P_IF;
							datum_j.weightPhase *= weight_L_IF;
							if(P1.byEditedMark1 != TYPE_EDITEDMARK_NORMAL || P2.byEditedMark1 != TYPE_EDITEDMARK_NORMAL)
								datum_j.weightCode = 0;
							// 相位观测数据的观测权: 将正常或周跳的标记看成是正常的, 其余的将权值设为0
							if((L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)||
							   (L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP   || L2.byEditedMark1 == TYPE_EDITEDMARK_SLIP))
							   ;
							else
								datum_j.weightPhase = 0.0;
							m_staBaseLineList[b_i].dynEpochList[s_i].mapDatum.insert(PODEpochSatMap::value_type(it->first, datum_j));
						}
						s_index++;
						m_staBaseLineList[b_i].dynEpochList[s_i].validIndex = s_index; // 记录有效时刻标记, 以方便通过时间标记nObsTime来查找其在有效参数中的位置
					}
					else
					{
						m_staBaseLineList[b_i].dynEpochList[s_i].eyeableGPSCount = 0;
						for(Rinex2_1_EditedSdObsSatMap::iterator it = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs.begin(); it != m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs.end(); ++it)
						{
							PODEpochElement datum_j;
							m_staBaseLineList[b_i].dynEpochList[s_i].mapDatum.insert(PODEpochSatMap::value_type(it->first, datum_j));
						}
						m_staBaseLineList[b_i].dynEpochList[s_i].validIndex = -1;
					}					
				}
				//FILE *pfile_IA = fopen("C:\\initial_ambiguity.txt","a+");
				// 以每颗可视且有效 BD 卫星的观测序列为基础, 整理载波观测方程信息
				vector<ObsEqArc> mw_ArcList;
				mw_ArcList.clear();
				vector<ObsEqArc> L_IF_ArcList;
				L_IF_ArcList.clear();
				vector<ObsEqArc> P_IF_ArcList;
				P_IF_ArcList.clear();				
				// 以每颗可视 BD 卫星的观测序列为基础, 整理载波相位观测方程信息
				int id_Ambiguity_GL = 0;
				for(size_t s_i = 0; s_i < editedObsSatList.size(); s_i++)
				{
					double windup_prev_A = DBL_MAX;
					double windup_prev_B = DBL_MAX;
					size_t count_obs_i = editedObsSatList[s_i].editedObs.size(); // 观测时间序列数据个数(某颗固定BD卫星)
					int id_Sat = editedObsSatList[s_i].Id;
					double *pEpochTime    = new double[count_obs_i]; // 相对时间序列
					double *pP1           = new double[count_obs_i]; // 伪距观测数据序列
					double *pP2           = new double[count_obs_i]; // 伪距观测数据序列
					double *pL1           = new double[count_obs_i]; // 相位观测数据序列
					double *pL2           = new double[count_obs_i]; // 相位观测数据序列
					double *pP_IF         = new double[count_obs_i]; // 伪距观测数据序列
					double *pL_IF         = new double[count_obs_i]; // 相位观测数据序列
					double *pMW           = new double[count_obs_i]; // MW组合观测数据序列
					int    *pEditedFlag   = new int   [count_obs_i]; // 编辑标记序列 0-正常; 1-新的周跳标记; 2-异常
					int    *pEpochId      = new int   [count_obs_i];
					Rinex2_1_EditedSdObsEpochMap::iterator it0 = editedObsSatList[s_i].editedObs.begin();
					GPST t0 = it0->first; // 初始时间 - 用于计算相对时间(秒)
					int j = 0;
					for(Rinex2_1_EditedSdObsEpochMap::iterator it = editedObsSatList[s_i].editedObs.begin(); it != editedObsSatList[s_i].editedObs.end(); ++it)
					{
						pEpochTime[j] = it->first - t0;
						pEpochId[j] = it->second.nObsTime;
						Rinex2_1_EditedObsDatum P1 = it->second.obsTypeList[nObsTypes_P1];
						Rinex2_1_EditedObsDatum P2 = it->second.obsTypeList[nObsTypes_P2];
						Rinex2_1_EditedObsDatum L1 = it->second.obsTypeList[nObsTypes_L1];
						Rinex2_1_EditedObsDatum L2 = it->second.obsTypeList[nObsTypes_L2];
						pP1[j] = P1.obs.data;
						pP2[j] = P2.obs.data;
						pL1[j] = BD_WAVELENGTH_L1 * L1.obs.data;
						pL2[j] = BD_WAVELENGTH_L2 * L2.obs.data;
						pP_IF[j] = P1.obs.data - (P1.obs.data - P2.obs.data) * coefficient_IF;
						pL_IF[j] = BD_WAVELENGTH_L1 * L1.obs.data - (BD_WAVELENGTH_L1 * L1.obs.data - BD_WAVELENGTH_L2 * L2.obs.data) * coefficient_IF;
						// 构造宽巷载波相位 widelane_L 和窄巷伪距 narrowlane_P
						double widelane_L   = (BD_FREQUENCE_L1 * L1.obs.data * BD_WAVELENGTH_L1 - BD_FREQUENCE_L2 * L2.obs.data * BD_WAVELENGTH_L2) / (BD_FREQUENCE_L1 - BD_FREQUENCE_L2);
						double narrowlane_P = (BD_FREQUENCE_L1 * P1.obs.data + BD_FREQUENCE_L2 * P2.obs.data) / (BD_FREQUENCE_L1 + BD_FREQUENCE_L2);
						pMW[j] = (widelane_L - narrowlane_P) / BD_WAVELENGTH_W; // melbourne-wuebbena 组合量
						// 用 pEditedFlag 标记据数据的有效性, 用来进行数据的遴选
						if(L1.byEditedMark1 == TYPE_EDITEDMARK_NORMAL && L2.byEditedMark1 == TYPE_EDITEDMARK_NORMAL)
							pEditedFlag[j] = 0; // 正常
						else if(L1.byEditedMark1 == TYPE_EDITEDMARK_SLIP || L2.byEditedMark1 == TYPE_EDITEDMARK_SLIP)
							pEditedFlag[j] = 1; // 新的周跳标记
						else
							pEditedFlag[j] = 2; // 异常点
						j++;
					}
					// 每个有效连续跟踪弧段数据, 对应一个新的模糊度参数, 进行处理
					size_t k   = 0; // 记录新弧段起始点
					size_t k_i = k; // 记录新弧段终止点
					while(1)
					{
						if(k_i + 1 >= count_obs_i) // k_i 为时间序列终点
							goto newArc;
						else
						{
							// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?, k_i + 1 是否有周跳点发生?
							if((pEpochTime[k_i + 1] - pEpochTime[k_i] <= m_podParaDefine.max_arclengh)
							&& (pEditedFlag[k_i + 1] != 1))
							{
								k_i++;
								continue;
							}
							// 如果相邻两点的间隔时间 > max_arclengh, 或者下一点是周跳点则判断为新弧段
							else // k_i + 1 为新弧段的起点
								goto newArc;
						}
						newArc:  // 本弧段[k，k_i]数据处理 
						{
							vector<size_t> normalPointList; // 记录正常数据
							normalPointList.clear();
							for(size_t s_k = k; s_k <= k_i; s_k++)
							{// 正常数据标记 0 或 1--2007/08/08
								// 由于码观测数据的标记与相位可能不同步, 是否需要增加对伪码正常的检查? 2007/08/20
								if((pEditedFlag[s_k] == 0 || pEditedFlag[s_k] == 1))
								{
									// 补充异常点判断,当相位观测权等于 0, 丢掉这个点, 避免增加以后计算的负担, 2007/08/16
									// 同时根据 weight_phase, 删除非有效点 -- 2007/08/16
									// 同时根据 weight_code,  删除非有效点 -- 2009/12/03
									// 只能放在这里, 放在前面会把周跳标记涂改掉
									if(m_staBaseLineList[b_i].dynEpochList[pEpochId[s_k]].mapDatum[id_Sat].weightPhase != 0
									&& m_staBaseLineList[b_i].dynEpochList[pEpochId[s_k]].mapDatum[id_Sat].weightCode  != 0
									&& m_staBaseLineList[b_i].dynEpochList[pEpochId[s_k]].validIndex != -1)
									{
										normalPointList.push_back(s_k); 
									}
								}
							}
							size_t count_arcpoints = normalPointList.size(); // 正常数据点个数
							if(count_arcpoints > m_podParaDefine.min_arcpointcount)
							{
								pCountArc_sat[id_Sat]++;
								ObsEqArc mw_Arc;
								mw_Arc.obsList.clear();
								ObsEqArc L_IF_Arc;
								L_IF_Arc.obsList.clear();
								ObsEqArc P_IF_Arc;
								P_IF_Arc.obsList.clear();
								// 计算概略相位模糊度
								double* pX  = new double [count_arcpoints];
								double* pW  = new double [count_arcpoints];
								double  mean = 0;
								double  var  = 0;
								double* pX_IF  = new double [count_arcpoints];
								double* pW_IF  = new double [count_arcpoints];
								double  mean_IF = 0;
								double  var_IF  = 0;
								for(size_t s_k = 0; s_k < count_arcpoints; s_k++)
								{
									pX[s_k] = pMW[normalPointList[s_k]];
									pX_IF[s_k] = pL_IF[normalPointList[s_k]] - pP_IF[normalPointList[s_k]];
									//if(id_Sat == 10)
									//	fprintf(pfile_IA,"%16.6lf\n",pX_IF[s_k]);
								}
								RobustStatMean(   pX,    pW, int(count_arcpoints),    mean,    var, 3);
								RobustStatMean(pX_IF, pW_IF, int(count_arcpoints), mean_IF, var_IF, 3);
								// 根据 RobustStatMean 来剔除低仰角的伪码野值, 来保证 MW 求解的正确性
								size_t count_mw_valid = 0;
								for(size_t s_k = 0; s_k < count_arcpoints; s_k++)
								{
									if(pW[s_k] != 1.0)
										count_mw_valid++;
								}								
								if(count_mw_valid > m_podParaDefine.min_arcpointcount)
								{
									mw_Arc.ambiguity   = mean;
									L_IF_Arc.ambiguity = mean_IF; 
									P_IF_Arc.ambiguity = 0;
									for(size_t s_k = 0; s_k < count_arcpoints; s_k++)
									{
										if(pW[s_k] != 1.0)
										{
											// 首先进行相位 windup 修正
											double correct_phasewindup_A = 0;
											double correct_phasewindup_B = 0;
											if(m_podParaDefine.bOn_PhaseWindUp && id_Sat > 5) // GEO 卫星不修正这一项误差
											{
												POSCLK posclk_A, posclk_B; 
												posclk_A.clk = m_staBaseLineList[b_i].editedSdObsFile.m_data[pEpochId[normalPointList[s_k]]].A_clock;
												posclk_B.clk = m_staBaseLineList[b_i].editedSdObsFile.m_data[pEpochId[normalPointList[s_k]]].B_clock;
												double P_J2000[3];
												double P_ITRF[3];
												P_ITRF[0] = m_staBaseLineList[b_i].posvel_A.x;
												P_ITRF[1] = m_staBaseLineList[b_i].posvel_A.y;
												P_ITRF[2] = m_staBaseLineList[b_i].posvel_A.z;
												m_TimeCoordConvert.ECEF_J2000( m_staBaseLineList[b_i].editedSdObsFile.m_data[pEpochId[normalPointList[s_k]]].t - posclk_A.clk / SPEED_LIGHT, P_J2000, P_ITRF, false); // 坐标系转换, 考虑到接收机钟差
												posclk_A.x = P_J2000[0];
												posclk_A.y = P_J2000[1];
												posclk_A.z = P_J2000[2];
												P_ITRF[0] = m_staBaseLineList[b_i].posvel_B.x;
												P_ITRF[1] = m_staBaseLineList[b_i].posvel_B.y;
												P_ITRF[2] = m_staBaseLineList[b_i].posvel_B.z;
												m_TimeCoordConvert.ECEF_J2000( m_staBaseLineList[b_i].editedSdObsFile.m_data[pEpochId[normalPointList[s_k]]].t - posclk_B.clk / SPEED_LIGHT, P_J2000, P_ITRF, false); // 坐标系转换, 考虑到接收机钟差
												posclk_B.x = P_J2000[0];
												posclk_B.y = P_J2000[1];
												posclk_B.z = P_J2000[2];
												double delay_A = 0;
												double delay_B = 0;
												SP3Datum sp3Datum_A, sp3Datum_B;
												if(sp3InputFile_j2000.getEphemeris_PathDelay(m_staBaseLineList[b_i].editedSdObsFile.m_data[pEpochId[normalPointList[s_k]]].t, posclk_A, id_Sat, delay_A, sp3Datum_A)
												&& sp3InputFile_j2000.getEphemeris_PathDelay(m_staBaseLineList[b_i].editedSdObsFile.m_data[pEpochId[normalPointList[s_k]]].t, posclk_B, id_Sat, delay_B, sp3Datum_B)) 
												{
													//GNSSBasicCorrectFunc::correctSp3EarthRotation(delay_A, sp3Datum_A); // 对 GPS 卫星星历进行地球自转改正
													//GNSSBasicCorrectFunc::correctSp3EarthRotation(delay_B, sp3Datum_B); // 对 GPS 卫星星历进行地球自转改正
													GPST t_Transmit_A = m_staBaseLineList[b_i].editedSdObsFile.m_data[pEpochId[normalPointList[s_k]]].t - posclk_A.clk / SPEED_LIGHT - delay_A;
													GPST t_Transmit_B = m_staBaseLineList[b_i].editedSdObsFile.m_data[pEpochId[normalPointList[s_k]]].t - posclk_B.clk / SPEED_LIGHT - delay_B;
													POS3D sunPos_A, sunPos_B; // 太阳相对地心位置
													TDB t_TDB_A = TimeCoordConvert::GPST2TDB(t_Transmit_A); // 获得 TDB 时间--提供太阳历参考时间
													double jd_A = TimeCoordConvert::DayTime2JD(t_TDB_A); // 获得儒略日
													TDB t_TDB_B = TimeCoordConvert::GPST2TDB(t_Transmit_B); 
													double jd_B = TimeCoordConvert::DayTime2JD(t_TDB_B); 
													double P_A[3], P_B[3];
													if(m_JPLEphFile.getSunPos_Delay_EarthCenter(jd_A, P_A)
													&& m_JPLEphFile.getSunPos_Delay_EarthCenter(jd_B, P_B))
													{// sunPos 为 J2000 惯性系坐标
														sunPos_A.x = P_A[0] * 1000; 
														sunPos_A.y = P_A[1] * 1000; 
														sunPos_A.z = P_A[2] * 1000;
														sunPos_B.x = P_B[0] * 1000; 
														sunPos_B.y = P_B[1] * 1000; 
														sunPos_B.z = P_B[2] * 1000;
														//map<int, int>::iterator it_BlockID = mapBlockID.find(id_Sat);
														//if(it_BlockID != mapBlockID.end())
														//{//BDS卫星按GPS卫星的Block IIR类型修正相位缠绕
															windup_prev_A = GNSSBasicCorrectFunc::correctStaAntPhaseWindUp(5, posclk_A.getPos(), sp3Datum_A.pos, sunPos_A, windup_prev_A);
															windup_prev_B = GNSSBasicCorrectFunc::correctStaAntPhaseWindUp(5, posclk_B.getPos(), sp3Datum_B.pos, sunPos_B, windup_prev_B);
															correct_phasewindup_A = windup_prev_A * SPEED_LIGHT / ((BD_FREQUENCE_L1 + BD_FREQUENCE_L2));
															correct_phasewindup_B = windup_prev_B * SPEED_LIGHT / ((BD_FREQUENCE_L1 + BD_FREQUENCE_L2));
														//}
													}
												}
											}
											//size_t index = listNormalPoint[s_k];			
											ObsEqArcElement raw_MW;
											raw_MW.id_Sat = id_Sat;
											raw_MW.nObsTime  = pEpochId[normalPointList[s_k]];
											raw_MW.obs = pMW[normalPointList[s_k]]; 
											mw_Arc.obsList.insert(ObsEqArcElementMap::value_type(raw_MW.nObsTime, raw_MW));

											ObsEqArcElement raw_L_IF;
											raw_L_IF.id_Sat = id_Sat;
											raw_L_IF.nObsTime = pEpochId[normalPointList[s_k]];
											raw_L_IF.obs = pL1[normalPointList[s_k]] - (pL1[normalPointList[s_k]] - pL2[normalPointList[s_k]]) * coefficient_IF;
											raw_L_IF.obs -= (correct_phasewindup_A - correct_phasewindup_B); // windup 修正放到此处
											L_IF_Arc.obsList.insert(ObsEqArcElementMap::value_type(raw_L_IF.nObsTime, raw_L_IF));

											ObsEqArcElement raw_P_IF;
											raw_P_IF.id_Sat = id_Sat;
											raw_P_IF.nObsTime = pEpochId[normalPointList[s_k]];
											raw_P_IF.obs = pP1[normalPointList[s_k]] - (pP1[normalPointList[s_k]] - pP2[normalPointList[s_k]]) * coefficient_IF;
											P_IF_Arc.obsList.insert(ObsEqArcElementMap::value_type(raw_P_IF.nObsTime, raw_P_IF));
										}
									}
									mw_Arc.id_Ambiguity_GL   = id_Ambiguity_GL; //20140929
									L_IF_Arc.id_Ambiguity_GL = id_Ambiguity_GL;
									P_IF_Arc.id_Ambiguity_GL = id_Ambiguity_GL;
									mw_ArcList.push_back(mw_Arc);
									L_IF_ArcList.push_back(L_IF_Arc);
									P_IF_ArcList.push_back(P_IF_Arc);
									id_Ambiguity_GL++; // 全局模糊度编号，20140929
								}
								delete pX;
								delete pW;
								delete pX_IF;
								delete pW_IF;
							}
							if(k_i + 1 >= count_obs_i) // k_i为时间序列终点, 跳出
								break;
							else  
							{   
								k   = k_i + 1; // 新弧段的起点设置
								k_i = k;
								continue;
							}
						}
					}
					delete pEpochTime;
					delete pP1;
					delete pL1;
					delete pP2;
					delete pL2;
					delete pP_IF;
					delete pL_IF;
					delete pEditedFlag;
					delete pEpochId;
					delete pMW;
				}
				//printf("M-W组合观测弧段%5d   ",mw_ArcList.size());
				//fclose(pfile_IA);
				// 整理参考模糊度, 分段区间, 寻找合适的参考模糊度
				int id_t0 = int(count_epoch);
				int id_t1 = 0;
				for(size_t s_i = 0; s_i < mw_ArcList.size(); s_i++)
				{
					ObsEqArcElementMap::const_iterator it_begin = mw_ArcList[s_i].obsList.begin(); 
					ObsEqArcElementMap::const_reverse_iterator it_end = mw_ArcList[s_i].obsList.rbegin(); 
					if(it_begin->first < id_t0)
						id_t0 = it_begin->first;
					if(it_end->first > id_t1)
						id_t1 = it_end->first;
				}
				int t0_subsec = id_t0;
				int t1_subsec;
				m_staBaseLineList[b_i].amSectionList.clear();
				bool bDone;
				do
				{
					AMBIGUITY_SECTION amSection;
					bDone = false;
					// 1  确定初始时刻t0_subsec, 确定方法: 检索每个弧段的起始点, 寻早最早的起始点 t0_subsec
					int t0_i = id_t1;
					for(size_t s_i = 0; s_i < mw_ArcList.size(); s_i++)
					{
						if(mw_ArcList[s_i].obsList.size() > 0)
						{
							if(mw_ArcList[s_i].obsList.begin()->first < t0_i)
								t0_i = mw_ArcList[s_i].obsList.begin()->first;
						}
					}
					t0_subsec = t0_i;
					int count_arcpoint_valid = 0;
					int id_ref_arc = -1;
					// 2  检索距离初始时刻 t0_subsec 最近的弧段, 
					//    最近的准则是该弧段的起始时刻满足一定范围; 
					//    然后在这些满足最近条件的弧段中, 选择有效长度最长的弧段作为参考模糊度
					for(size_t s_i = 0; s_i < mw_ArcList.size(); s_i++)
					{
						if(mw_ArcList[s_i].obsList.size() > 0)
						{
							if(mw_ArcList[s_i].obsList.begin()->first - t0_subsec < int(m_podParaDefine.min_arcpointcount))
							{ 
								if(int(mw_ArcList[s_i].obsList.size()) > count_arcpoint_valid)
								{
									count_arcpoint_valid = int(mw_ArcList[s_i].obsList.size());
									id_ref_arc = int(s_i);
								}
							}
						}
					}
					t1_subsec = mw_ArcList[id_ref_arc].obsList.rbegin()->first;
					// 3 确定参考模糊度的终点时刻作为本区间段的时间终点 t1_subsec;
					//   判断时间终点距离整个弧段的终点时刻t_end的距离, 
					//   如果满足t_end - t1_subsec < min/2, 则令时间终点t1_subsec = t_end
					if(id_t1 - t1_subsec < int(m_podParaDefine.min_arcpointcount / 2))
					{
						t1_subsec = id_t1;
						bDone = true;
					}
					// 4 擦除并整理每个弧段在 t1_subsec 时刻以前的数据, 构成本区间内的观测数据
					// mw_ArcList
					amSection.mw_ArcList.push_back(mw_ArcList[id_ref_arc]);
					mw_ArcList[id_ref_arc].obsList.clear();
					for(size_t s_i = 0; s_i < mw_ArcList.size(); s_i++)
					{
						ObsEqArc mw_Arc_i;
						mw_Arc_i.ambiguity = mw_ArcList[s_i].ambiguity;
						mw_Arc_i.id_Ambiguity_GL = mw_ArcList[s_i].id_Ambiguity_GL;
						ObsEqArcElementMap::iterator it = mw_ArcList[s_i].obsList.begin(); 
						while(it != mw_ArcList[s_i].obsList.end())
						{
							if(it->first <= t1_subsec)
							{
								ObsEqArcElementMap::iterator jt = it;
								mw_Arc_i.obsList.insert(ObsEqArcElementMap::value_type(jt->first, jt->second));
								++it;
								mw_ArcList[s_i].obsList.erase(jt);
								continue;
							}
							else
							{
								break;
							}
						}
						if(mw_Arc_i.obsList.size() >= m_podParaDefine.min_arcpointcount)
							amSection.mw_ArcList.push_back(mw_Arc_i);
					}
					// L_IF_ArcList
					amSection.L_IF_ArcList.push_back(L_IF_ArcList[id_ref_arc]);
					L_IF_ArcList[id_ref_arc].obsList.clear();
					for(size_t s_i = 0; s_i < L_IF_ArcList.size(); s_i++)
					{
						ObsEqArc L_IF_Arc_i;
						L_IF_Arc_i.ambiguity = L_IF_ArcList[s_i].ambiguity;
						L_IF_Arc_i.id_Ambiguity_GL = L_IF_ArcList[s_i].id_Ambiguity_GL;
						ObsEqArcElementMap::iterator it = L_IF_ArcList[s_i].obsList.begin(); 
						while(it != L_IF_ArcList[s_i].obsList.end())
						{
							if(it->first <= t1_subsec)
							{
								ObsEqArcElementMap::iterator jt = it;
								L_IF_Arc_i.obsList.insert(ObsEqArcElementMap::value_type(jt->first, jt->second));
								++it;
								L_IF_ArcList[s_i].obsList.erase(jt);
								continue;
							}
							else
							{
								break;
							}
						}
						if(L_IF_Arc_i.obsList.size() >= m_podParaDefine.min_arcpointcount)
							amSection.L_IF_ArcList.push_back(L_IF_Arc_i);
					}
					// P_IF_ArcList
					amSection.P_IF_ArcList.push_back(P_IF_ArcList[id_ref_arc]);
					P_IF_ArcList[id_ref_arc].obsList.clear();
					for(size_t s_i = 0; s_i < P_IF_ArcList.size(); s_i++)
					{
						ObsEqArc P_IF_Arc_i;
						P_IF_Arc_i.ambiguity = P_IF_ArcList[s_i].ambiguity;
						P_IF_Arc_i.id_Ambiguity_GL = P_IF_ArcList[s_i].id_Ambiguity_GL;
						ObsEqArcElementMap::iterator it = P_IF_ArcList[s_i].obsList.begin(); 
						while(it != P_IF_ArcList[s_i].obsList.end())
						{
							if(it->first <= t1_subsec)
							{
								ObsEqArcElementMap::iterator jt = it;
								P_IF_Arc_i.obsList.insert(ObsEqArcElementMap::value_type(jt->first, jt->second));
								++it;
								P_IF_ArcList[s_i].obsList.erase(jt);
								continue;
							}
							else
							{
								break;
							}
						}
						if(P_IF_Arc_i.obsList.size() >= m_podParaDefine.min_arcpointcount)
							amSection.P_IF_ArcList.push_back(P_IF_Arc_i);
					}
					amSection.id_t0 = t0_subsec;
					amSection.id_t1 = t1_subsec;
					amSection.ArcList2EpochList(amSection.mw_ArcList,   amSection.mw_EpochList);
					amSection.ArcList2EpochList(amSection.P_IF_ArcList, amSection.P_IF_EpochList);
					amSection.ArcList2EpochList(amSection.L_IF_ArcList, amSection.L_IF_EpochList);
					if(amSection.mw_ArcList.size() >= 2) //20140929，GPS,程序中阈值为3
						m_staBaseLineList[b_i].amSectionList.push_back(amSection);
					t0_subsec = t1_subsec + 1;
				}while(!bDone);				
				// 确定参考卫星, 条件是在一次观测中观测仰角最高的卫星, 对应的伪码观测最小
				int count_obs_dd = 0;
				for(size_t s_k = 0; s_k < m_staBaseLineList[b_i].amSectionList.size(); s_k++)
				{// 选择观测高度角最大的卫星					
					for(int s_i = 0; s_i < int(m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList.size()); s_i++)
					{  
						count_obs_dd += int(m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList.size()) - 1;
						m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDObs = 0;
						m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[0].id_Sat;
						m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefAmbiguity = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[0].id_Ambiguity;
						double Elevation = 0.0;
						int nObsTime = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].nObsTime;
						for(int s_j = 0; s_j < int(m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList.size()); s_j++)
						{		
							int id_Sat = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[s_j].id_Sat;
							double E_j = 0.5 * (m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].editedObs[id_Sat].Elevation_A + m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].editedObs[id_Sat].Elevation_B);
							if(E_j > Elevation && E_j < 80.0)
							{// 参考星的高度不要正对天顶
								Elevation = E_j;
								m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDObs = s_j;
								m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefSat = id_Sat;
								m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefAmbiguity = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[s_j].id_Ambiguity;
							}							
						}
						if(m_podParaDefine.bOnEst_StaTropZenithDelay)
						{
							//统计每个对流层估计区间的观测历元个数，以前按SD文件统计有问题，2014/05/13
							DayTime t_epoch = m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].t;
							int TZD_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].getIndexZenithDelayEstList(t_epoch);//获取该时刻在原始对流层估计区间的位置
							int TZD_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].getIndexZenithDelayEstList(t_epoch);
							m_mapStaDatum[m_staBaseLineList[b_i].name_A].zenithDelayEstList[TZD_A].nValid_Epoch ++;
							m_mapStaDatum[m_staBaseLineList[b_i].name_B].zenithDelayEstList[TZD_B].nValid_Epoch ++;
						}
					}
					for(int s_i = 0; s_i < int(m_staBaseLineList[b_i].amSectionList[s_k].P_IF_EpochList.size()); s_i++)
					{						
						m_staBaseLineList[b_i].amSectionList[s_k].P_IF_EpochList[s_i].id_DDObs = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDObs;
						m_staBaseLineList[b_i].amSectionList[s_k].P_IF_EpochList[s_i].id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefSat;
						m_staBaseLineList[b_i].amSectionList[s_k].P_IF_EpochList[s_i].id_DDRefAmbiguity = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefAmbiguity;
					}
					for(int s_i = 0; s_i < int(m_staBaseLineList[b_i].amSectionList[s_k].L_IF_EpochList.size()); s_i++)
					{  
						m_staBaseLineList[b_i].amSectionList[s_k].L_IF_EpochList[s_i].id_DDObs = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDObs;
						m_staBaseLineList[b_i].amSectionList[s_k].L_IF_EpochList[s_i].id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefSat;
						m_staBaseLineList[b_i].amSectionList[s_k].L_IF_EpochList[s_i].id_DDRefAmbiguity = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefAmbiguity;
					}
				}
				// 计算宽巷模糊度的浮点解及其协方差矩阵
				int count_FixedAmbiguity = 0;
				int count_UnFixedAmbiguity = 0;					
				for(size_t s_k = 0; s_k < m_staBaseLineList[b_i].amSectionList.size(); s_k++)
				{
					int count_ambiguity_dd_mw = int(m_staBaseLineList[b_i].amSectionList[s_k].mw_ArcList.size()) - 1;
					Matrix mw_nb, mw_Nbb;
					mw_nb.Init(count_ambiguity_dd_mw, 1);
					mw_Nbb.Init(count_ambiguity_dd_mw, count_ambiguity_dd_mw);
					for(int s_i = 0; s_i < int(m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList.size()); s_i++)
					{  					
						int count_obs = int(m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList.size());
						int nRow = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].nObsTime; 
						int id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefSat;
						int id_DDRefAmbiguity = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDRefAmbiguity; // 参考模糊度序号
						int id_DDObs = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].id_DDObs;
						double w_ref = m_staBaseLineList[b_i].dynEpochList[nRow].mapDatum[id_DDRefSat].weightCode
									 * m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[id_DDObs].robustweight; 
						w_ref *= m_podParaDefine.apriorityRms_PIF / m_podParaDefine.apriorityRms_LIF;// 恢复伪距的权重，防止矩阵病态，2013.11.27,刘俊宏						
						if(w_ref == 0)
							w_ref = zeroweight_code;
						Matrix matHB_i(count_obs - 1, count_ambiguity_dd_mw);
						Matrix matQ_i(count_obs - 1,  count_obs - 1);
						Matrix matW_i(count_obs - 1,  count_obs - 1);
						Matrix matZ_i(count_obs - 1,  1);
						vector<int> ambiguityList;
						ambiguityList.clear();
						vector<double> w_list;
						w_list.resize(count_obs - 1);
						int k = -1;
						m_mapStaDatum[m_staBaseLineList[b_i].name_A].count_obs_dd += count_obs - 1; // 2013/03/06,刘俊宏
						m_mapStaDatum[m_staBaseLineList[b_i].name_B].count_obs_dd += count_obs - 1; // 2013/03/06
						for(int s_j = 0; s_j < count_obs; s_j++)
						{							
							int id_Sat = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[s_j].id_Sat; 
							int id_Ambiguity = m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[s_j].id_Ambiguity;
							if(id_Sat == id_DDRefSat)
								continue;
							k++;
							double w = m_staBaseLineList[b_i].dynEpochList[nRow].mapDatum[id_Sat].weightCode // dynEpochList[nRow].mapDatum[id_DDRefSat].weightCode 改为 dynamicEpochList[nRow].datum[id_Sat].weightCode
									 * m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[s_j].robustweight;
							w *= m_podParaDefine.apriorityRms_PIF / m_podParaDefine.apriorityRms_LIF;// 恢复伪距的权重，防止矩阵病态，2013.11.27,刘俊宏
							if(w == 0)
								w = zeroweight_code;
							w_list[k] = w;
							// 根据 id_DDRefAmbiguity 和 id_Ambiguity 判断
							if(id_DDRefAmbiguity != 0)
								matHB_i.SetElement(k, id_DDRefAmbiguity - 1, -1.0);
							if(id_Ambiguity != 0)
							{
								ambiguityList.push_back(id_Ambiguity - 1);
								matHB_i.SetElement(k, id_Ambiguity - 1, 1.0);
							}
							// 经过概略点改进后的观测值
							double obsValue = (m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[s_j].obs)
											- (m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList[s_i].obsSatList[id_DDObs].obs);
							matZ_i.SetElement(k, 0, obsValue);							
						}
						if(id_DDRefAmbiguity != 0)
							ambiguityList.push_back(id_DDRefAmbiguity - 1);
						// 观测权矩阵---为了适应以后的数据编辑, 需要对权值进行调整?
						for(int i = 0; i < count_obs - 1; i++)
						{
							for(int j = 0; j < count_obs - 1; j++)
							{								
								if(i == j) // 对角线
									matQ_i.SetElement(i, j, 2.0 / (w_ref * w_ref) + 2.0 /(w_list[j] * w_list[j]));
								else // 非对角线
									matQ_i.SetElement(i, j, 2.0 / (w_ref * w_ref));
							}
						}
						matW_i = matQ_i.Inv();
						// 作分块运算, matHB_i 仅有限列非 0
						Matrix matW_Z_i  = matW_i * matZ_i; // [count_obs - 1, 1]
						for(int i = 0; i < int(ambiguityList.size()); i++)
						{
							for(int j = 0; j < count_obs - 1; j++)
							{
								mw_nb.SetElement(ambiguityList[i], 0,  mw_nb.GetElement(ambiguityList[i], 0)
																	 + matHB_i.GetElement(j, ambiguityList[i]) * matW_Z_i.GetElement(j, 0));
							}
						}
						for(int i = 0; i < int(ambiguityList.size()); i++)
						{
							Matrix matHB_W_ambiguityNumber(1, count_obs - 1);
							for(int j = 0; j < count_obs - 1; j++) // 列
							{
								for(int k = 0; k < count_obs - 1; k++)
								{
									matHB_W_ambiguityNumber.SetElement(0, j,  matHB_W_ambiguityNumber.GetElement(0, j)
																			+ matHB_i.GetElement(k, ambiguityList[i]) * matW_i.GetElement(j, k));
								}
							}
							/*
							  对于2个双差观测量

										 |w11   w12|   |bm1|                                                       |bm1|
							|bn1  bn2| * |         | * |   |  = |bn1 * w11 + bn2 * w21,   bn1 * w12 + bn2 * w22| * |   | 
										 |w21   w22|   |bm2|                                                       |bm2|

							*/
							for(int ii = 0; ii < int(ambiguityList.size()); ii++)
							{
								for(int j = 0; j < count_obs - 1; j++) // 列
								{
									mw_Nbb.SetElement(ambiguityList[i], ambiguityList[ii],  mw_Nbb.GetElement(ambiguityList[i], ambiguityList[ii])
																						  + matHB_W_ambiguityNumber.GetElement(0, j) * matHB_i.GetElement(j, ambiguityList[ii]));
								}
							}
						}
					}			
					Matrix matDDQ_MW = mw_Nbb.Inv();
					Matrix matDDFloat_MW = matDDQ_MW * mw_nb;
					Matrix matAFixed;
					Matrix matSqnorm;	
					//for(int i = 0; i < matDDFloat_MW.GetNumRows();i++)
					//	fprintf(pfile_W,"%3d  %3d  %3d %16.4lf\n",b_i + 1,s_k + 1,i + 1, matDDFloat_MW.GetElement(i,0));
					if(!m_podParaDefine.bOn_AmbiguityFix)
					{//2014年155_157天reun和sin1形成的基线，lamda:: main无法计算,其中lambda::lsearch出现死循环
						if(b_i == 0 && s_k == 0)
						{
							sprintf(info,"宽巷模糊度不固定!");
							RuningInfoFile::Add(info);
							printf("%s\n",info);
						}
						sprintf(info,"第%2d条基线第%2d个区间的历元个数%8d",b_i + 1,s_k + 1,int(m_staBaseLineList[b_i].amSectionList[s_k].mw_EpochList.size()));
						RuningInfoFile::Add(info);
						m_staBaseLineList[b_i].amSectionList[s_k].bDDAmFixed_MW = false;
						//m_staBaseLineList[b_i].amSectionList[s_k].matDDFixedFlag_MW = matSelectedFlag;
						m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_MW_List.clear();
						m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_list.clear();
						m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_UnFixed_list.clear();
						for(int ii = 0; ii < matDDFloat_MW.GetNumRows(); ii++)
						{
							double ambiguity_dd_L_IF = m_staBaseLineList[b_i].amSectionList[s_k].L_IF_ArcList[ii + 1].ambiguity - m_staBaseLineList[b_i].amSectionList[s_k].L_IF_ArcList[0].ambiguity;
							double ambiguity_DD_MW = matDDFloat_MW.GetElement(ii, 0);
							double ambiguity_DD_L1 = (ambiguity_dd_L_IF - ambiguity_DD_MW * coeff_mw) / BD_WAVELENGTH_N;
							m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_MW_List.push_back(ambiguity_DD_MW);
							m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_list.push_back(ambiguity_DD_L1);
							m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_UnFixed_list.push_back(ii); // 初始化时窄巷模糊度均未固定
						}
					}
					else
					{
						if(!lambda::main(matDDFloat_MW, matDDQ_MW, matSqnorm, matAFixed, 2))
						{
							sprintf(info,"第%2d条基线第%2d个区间labda 求解宽巷模糊度有误！",b_i + 1,s_k + 1);
							RuningInfoFile::Add(info);
							printf("%s\n",info);							
							return false;
						}
						double ksb = matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0);
						sprintf(info,"第%2d条基线第%2d个区间 ksb = %8.4f,  count_ambiguity_dd = %2d",b_i + 1,s_k + 1,ksb,count_ambiguity_dd_mw);
						RuningInfoFile::Add(info);
						Matrix matSelectedFlag(count_ambiguity_dd_mw, 1);
						Matrix matSelectedAFixed = matAFixed;
						for(int ii = 0; ii < count_ambiguity_dd_mw; ii++)
							matSelectedFlag.SetElement(ii, 0, 1.0);
						vector<int> validIndexList;
						while(ksb < m_podParaDefine.threhold_LAMBDA_ksb)
						{
							validIndexList.clear();
							for(int ii = 0; ii < count_ambiguity_dd_mw; ii++)
							{
								if(matSelectedFlag.GetElement(ii, 0) == 1.0)
									validIndexList.push_back(ii);
							}
							if(validIndexList.size() < 2) // 2013/02/25, 谷德峰修改, 至少两个才能进行筛选
							{
								matSelectedFlag.Init(count_ambiguity_dd_mw, 1);
								break;
							}
							int i_max = -1;
							double ksb_max = 0.0;
							for(int ii = 0; ii < int(validIndexList.size()); ii++)
							{								
								Matrix matSelectedFlag_i = matSelectedFlag;
								matSelectedFlag_i.SetElement(validIndexList[ii], 0, 0.0);
								if(GPSPod::GPSMeoSatDynPOD::lambdaSelected(matDDFloat_MW, matDDQ_MW, matSqnorm, matAFixed, matSelectedFlag_i))
								{
									if(matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0) > ksb_max)
									{
										i_max = validIndexList[ii];
										ksb_max = matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0);
										matSelectedAFixed = matAFixed;
									}
								}
							}
							// 更新模糊度固定标记
							if(i_max >= 0)
							{
								matSelectedFlag.SetElement(i_max, 0, 0.0);
								ksb = ksb_max;
							}
							else
							{
								matSelectedFlag.Init(count_ambiguity_dd_mw, 1);
								break;
							}
						}
						double max_AmFloat_AmFixed = 0.0;
						int count_FixedAmbiguity_k = 0;
						for(int ii = 0; ii < count_ambiguity_dd_mw; ii++)
						{
							if(matSelectedFlag.GetElement(ii, 0) == 1.0)
							{
								count_FixedAmbiguity_k++;
								if(max_AmFloat_AmFixed < fabs(matDDFloat_MW.GetElement(ii, 0) - matSelectedAFixed.GetElement(ii, 0)))
									max_AmFloat_AmFixed = fabs(matDDFloat_MW.GetElement(ii, 0) - matSelectedAFixed.GetElement(ii, 0));
							}
						}
						if(ksb >= m_podParaDefine.threhold_LAMBDA_ksb)
						{
							count_FixedAmbiguity   += count_FixedAmbiguity_k;
							count_UnFixedAmbiguity += count_ambiguity_dd_mw - count_FixedAmbiguity_k;
							// 输出到log文件里便于查看, 2014/04/06
							sprintf(info, "%02d %02d:%02d:%02d-%02d:%02d:%02d  %5d %5d %14.4f(%6.4f)", s_k+1,
																								  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t0].t.hour,
																								  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t0].t.minute,
																								  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t0].t.second),
																								  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t1].t.hour,
																								  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t1].t.minute,
																								  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t1].t.second),
																								  count_ambiguity_dd_mw, 
																								  count_FixedAmbiguity_k, 
																								  ksb,
																								  max_AmFloat_AmFixed);
							RuningInfoFile::Add(info);

							m_staBaseLineList[b_i].amSectionList[s_k].bDDAmFixed_MW = true;
							m_staBaseLineList[b_i].amSectionList[s_k].matDDFixedFlag_MW = matSelectedFlag;
							m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_MW_List.clear();
							m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_list.clear();
							m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_UnFixed_list.clear();							
							for(int ii = 0; ii < matSelectedAFixed.GetNumRows(); ii++)
							{
								double ambiguity_dd_L_IF = m_staBaseLineList[b_i].amSectionList[s_k].L_IF_ArcList[ii + 1].ambiguity - m_staBaseLineList[b_i].amSectionList[s_k].L_IF_ArcList[0].ambiguity;
								double ambiguity_DD_MW = matSelectedAFixed.GetElement(ii, 0);
								double ambiguity_DD_L1 = (ambiguity_dd_L_IF - ambiguity_DD_MW * coeff_mw) / BD_WAVELENGTH_N;
								m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_MW_List.push_back(ambiguity_DD_MW);
								m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_list.push_back(ambiguity_DD_L1);
								m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_UnFixed_list.push_back(ii); // 初始化时窄巷模糊度均未固定
							}
						}
						else
						{
							count_FixedAmbiguity   += 0;
							count_UnFixedAmbiguity += count_ambiguity_dd_mw;
							sprintf(info, "%02d %02d:%02d:%02d-%02d:%02d:%02d  %5d %5d %14.4f(%6.4f)", s_k+1,
																								  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t0].t.hour,
																								  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t0].t.minute,
																								  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t0].t.second),
																								  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t1].t.hour,
																								  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t1].t.minute,
																								  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_k].id_t1].t.second),
																								  count_ambiguity_dd_mw, 
																								  0, 
																								  ksb,
																								  max_AmFloat_AmFixed);
							RuningInfoFile::Add(info);

							m_staBaseLineList[b_i].amSectionList[s_k].bDDAmFixed_MW = false;
							m_staBaseLineList[b_i].amSectionList[s_k].matDDFixedFlag_MW = matSelectedFlag;
							m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_MW_List.clear();
							m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_list.clear();
							m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_UnFixed_list.clear();
							for(int ii = 0; ii < matDDFloat_MW.GetNumRows(); ii++)
							{
								double ambiguity_dd_L_IF = m_staBaseLineList[b_i].amSectionList[s_k].L_IF_ArcList[ii + 1].ambiguity - m_staBaseLineList[b_i].amSectionList[s_k].L_IF_ArcList[0].ambiguity;
								double ambiguity_DD_MW = matDDFloat_MW.GetElement(ii, 0);
								double ambiguity_DD_L1 = (ambiguity_dd_L_IF - ambiguity_DD_MW * coeff_mw) / BD_WAVELENGTH_N;
								m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_MW_List.push_back(ambiguity_DD_MW);
								m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_list.push_back(ambiguity_DD_L1);
								m_staBaseLineList[b_i].amSectionList[s_k].ambiguity_DD_L1_UnFixed_list.push_back(ii); // 初始化时窄巷模糊度均未固定
							}
						}
					}					
				}		
				if(m_podParaDefine.bOn_AmbiguityFix)
				{
					double rate_amFixed_MW = double(count_FixedAmbiguity) / (count_FixedAmbiguity + count_UnFixedAmbiguity);
					sprintf(info, "第%2d条基线，模糊度总个数%3d, 模糊度固定个数%3d, 宽巷模糊度固定成功率%.4f", b_i + 1,
																						   count_FixedAmbiguity + count_UnFixedAmbiguity,
																						   count_FixedAmbiguity,
																						   rate_amFixed_MW);
					RuningInfoFile::Add(info);
					printf("%s\n",info);
					count_All_FixedAmbiguity   += count_FixedAmbiguity;
					count_All_UnFixedAmbiguity += count_UnFixedAmbiguity;	
				}
			}
			if(m_podParaDefine.bOn_AmbiguityFix)
			{
				double rate_amFixed_MW_All = double(count_All_FixedAmbiguity) / (count_All_FixedAmbiguity + count_All_UnFixedAmbiguity);
				sprintf(info, "模糊度总个数%d, 模糊度固定个数%d, 宽巷模糊度固定成功率%5.3f", count_All_FixedAmbiguity + count_All_UnFixedAmbiguity,
																							 count_All_FixedAmbiguity,
																							 rate_amFixed_MW_All);
				RuningInfoFile::Add(info);
				printf("%s\n",info);
			}
			//fclose(pfile_W);
			RuningInfoFile::Add("宽巷模糊度求解完毕====================================================");			
			// 合并对流层参数估计区间,2013/7/8,刘俊宏
			if(m_podParaDefine.bOnEst_StaTropZenithDelay)
			{
				RuningInfoFile::Add("合并对流层参数估计区间================================================");
				for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)
				{
					size_t s_tro = 0;
					while(s_tro < it->second.zenithDelayEstList.size() - 1)
					{
						if(it->second.zenithDelayEstList[s_tro].nValid_Epoch > m_podParaDefine.min_Wet_TZD_ncount) 
							s_tro ++;
						else
						{
							if(it->second.zenithDelayEstList.size() == 2)
							{
								sprintf(info, "%s 仅有一个对流层估计区间，而且观测数据不足！",it->first.c_str());
								RuningInfoFile::Add(info);
								printf("%s\n",info);								
								return false;
							}
							else
							{
								if(s_tro == it->second.zenithDelayEstList.size() - 2)
								{//向前合并
									sprintf(info, "%s 区间%s—%s观测数据过少, 对流层天顶延迟估计区间合并！",it->first.c_str(), 
										                                                              it->second.zenithDelayEstList[s_tro - 1].t.toString().c_str(),
							                                                                          it->second.zenithDelayEstList[s_tro].t.toString().c_str());
									RuningInfoFile::Add(info);
									printf("%s\n",info);								
									it->second.zenithDelayEstList[s_tro - 1].nValid_Epoch += it->second.zenithDelayEstList[s_tro].nValid_Epoch;
									it->second.zenithDelayEstList.erase(it->second.zenithDelayEstList.begin() + s_tro);
								}
								else
								{//向后合并
									sprintf(info, "%s 区间%s—%s观测数据过少, 对流层天顶延迟估计区间合并！",it->first.c_str(), 
										                                                              it->second.zenithDelayEstList[s_tro].t.toString().c_str(),
							                                                                          it->second.zenithDelayEstList[s_tro + 1].t.toString().c_str());
									RuningInfoFile::Add(info);
									printf("%s\n",info);									
									it->second.zenithDelayEstList[s_tro].nValid_Epoch += it->second.zenithDelayEstList[s_tro + 1].nValid_Epoch;
									it->second.zenithDelayEstList.erase(it->second.zenithDelayEstList.begin() + s_tro + 1);
								}
							}
						}
					}
				}
				RuningInfoFile::Add("对流层参数估计区间合并完毕================================================");
			}			
			// 获取每个时刻的对流层估计参数位置,2013/9/25,刘俊宏
			if(m_podParaDefine.bOnEst_StaTropZenithDelay)
			{
				for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
				{					
					size_t count_epoch = m_staBaseLineList[b_i].editedSdObsFile.m_data.size(); // 观测历元个数
					m_staBaseLineList[b_i].id_ZenithDelayList_A.resize(count_epoch);
					m_staBaseLineList[b_i].id_ZenithDelayList_B.resize(count_epoch);					
					for(size_t s_i = 0; s_i < count_epoch; s_i++)
					{
						DayTime t_epoch = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t;
						m_staBaseLineList[b_i].id_ZenithDelayList_A[s_i] = m_mapStaDatum[m_staBaseLineList[b_i].name_A].getIndexZenithDelayEstList(t_epoch);
						m_staBaseLineList[b_i].id_ZenithDelayList_B[s_i] = m_mapStaDatum[m_staBaseLineList[b_i].name_B].getIndexZenithDelayEstList(t_epoch);
						
					}
				}				
			}
			sprintf(info, "初轨确定开始===============================================================");
			RuningInfoFile::Add(info);
			printf("%s\n",info);
			// 利用 sp3 星历进行初轨确定
			Sp3FitParameter paraSp3Fit;
			if(!sp3Fit(inputSp3FilePath, paraSp3Fit, m_podParaDefine.bOnEst_ERP))
			{
				sprintf(info, "初轨确定失败！");
				RuningInfoFile::Add(info);
				printf("%s\n",info);	
				return false;
			}
			paraSatOrbEst.t0_xpyput1 = paraSp3Fit.t0_xpyput1;
			paraSatOrbEst.xp         = paraSp3Fit.xp;
			paraSatOrbEst.xpDot      = paraSp3Fit.xpDot;
			paraSatOrbEst.yp         = paraSp3Fit.yp;
			paraSatOrbEst.ypDot      = paraSp3Fit.ypDot;
			paraSatOrbEst.ut1        = paraSp3Fit.ut1;
			paraSatOrbEst.ut1Dot     = paraSp3Fit.ut1Dot;
			SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); 
			while(it != paraSatOrbEst.satParaList.end())
			{
				int id_Sat = it->first;
				Sp3Fit_SatdynDatumMap::iterator jt = paraSp3Fit.satParaList.find(id_Sat);
				if(jt == paraSp3Fit.satParaList.end() || pCountArc_sat[id_Sat] == 0)
				{// 初轨无法确定
					sprintf(info, "C%02d初轨确定失败！ 有效观测弧段个数%d",id_Sat, pCountArc_sat[id_Sat]);
					RuningInfoFile::Add(info);
					printf("%s\n",info);
					SatDatumMap::iterator it0 = it;
					++it;
					paraSatOrbEst.satParaList.erase(it0);					
					continue;
				}
				else
				{					
					it->second.dynamicDatum_Init = jt->second.dynamicDatum_Est;
					it->second.dynamicDatum_Est  = jt->second.dynamicDatum_Est;
					++it;
					continue;
				}
			}
			sprintf(info, "初轨确定完毕===============================================================");
			RuningInfoFile::Add(info);
			printf("%s\n",info);			
			// 对卫星序号进行重排
			k = 0;
			for(SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); it != paraSatOrbEst.satParaList.end(); ++it)
			{
				it->second.index = k;
				k++;
			}
			// 迭代开始
			sprintf(info, "整网解算迭代开始===========================================================");
			RuningInfoFile::Add(info);
			printf("%s\n",info);			
			int  iterator_after_AmbFixed = 0;
			int  total_iterator = 0;
			bool flag_break = false;
			bool bDDAmFixed_L1 = false;
			bool result = true;			
			// 动力学参数个数统计: 卫星轨道力学参数(count_dyn_eachSat×N)  + 地球旋转参数(5) + 测站参数[3+13] × M
			int count_solar = int(paraSatOrbEst.satParaList.begin()->second.dynamicDatum_Init.solarPressureParaList.size());
			int count_solar_period = 9;  // 每个周期的太阳光压参数个数   
			if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
				count_solar_period  = 5;
			int count_dyn_eachSat = 6 + count_solar_period * count_solar; // 考虑多组太阳光压参数, 20131209, 谷德峰						
			int count_DynParameter = count_dyn_eachSat * int(paraSatOrbEst.satParaList.size());			
			if(m_podParaDefine.bOnEst_ERP)
				count_DynParameter += 5;
			if(m_podParaDefine.bOnEst_StaPos)				
				count_DynParameter += count_StaParameter;			
			if(m_podParaDefine.bOnEst_StaTropZenithDelay)
			{
				for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)
				{
					it->second.zenithIndex_0 = count_DynParameter;
					count_DynParameter += int(it->second.zenithDelayEstList.size());
				}
			}					
			int ddObs_count = 0;
			for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
			{
				m_staBaseLineList[b_i].ddObs_count = m_staBaseLineList[b_i].getDDObsCount();
				ddObs_count += m_staBaseLineList[b_i].ddObs_count;
				m_mapStaDatum[m_staBaseLineList[b_i].name_A].pos_Est = m_mapStaDatum[m_staBaseLineList[b_i].name_A].posvel.getPos();
				m_mapStaDatum[m_staBaseLineList[b_i].name_B].pos_Est = m_mapStaDatum[m_staBaseLineList[b_i].name_B].posvel.getPos();
				m_staBaseLineList[b_i].count_DD_MW_Fixed = 0;
				for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
				{
					m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed = int(m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List.size()); // 初始化为宽巷模糊度个数, 每一个宽巷对应一个窄巷
					m_staBaseLineList[b_i].count_DD_MW_Fixed += int(m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List.size());
				}
				m_staBaseLineList[b_i].N_bb.Init(m_staBaseLineList[b_i].count_DD_MW_Fixed, m_staBaseLineList[b_i].count_DD_MW_Fixed);
				m_staBaseLineList[b_i].n_xb.Init(count_DynParameter, m_staBaseLineList[b_i].count_DD_MW_Fixed);
				m_staBaseLineList[b_i].nb.Init(m_staBaseLineList[b_i].count_DD_MW_Fixed, 1);
				m_staBaseLineList[b_i].matQ_dd_L1.Init(m_staBaseLineList[b_i].count_DD_MW_Fixed, m_staBaseLineList[b_i].count_DD_MW_Fixed);
				// 初始化地球旋转参数矩阵
				size_t count_epoch = m_staBaseLineList[b_i].editedSdObsFile.m_data.size(); // 观测历元个数
				m_staBaseLineList[b_i].matPR_NRList.resize(count_epoch);
				m_staBaseLineList[b_i].matERList_A_0.resize(count_epoch);
				m_staBaseLineList[b_i].matERList_B_0.resize(count_epoch);
				m_staBaseLineList[b_i].matEPList_0.resize(count_epoch);				
				for(size_t s_i = 0; s_i < count_epoch; s_i++)
				{
					Matrix matPR_NR, matER_A, matER_B, matEP, matER_DOT_A, matER_DOT_B;
					m_TimeCoordConvert.Matrix_J2000_ECEF(m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t - m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].A_clock / SPEED_LIGHT, matPR_NR, matER_A, matEP, matER_DOT_A);
					m_TimeCoordConvert.Matrix_J2000_ECEF(m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t - m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].B_clock / SPEED_LIGHT, matPR_NR, matER_B, matEP, matER_DOT_B);
					m_staBaseLineList[b_i].matPR_NRList[s_i]   = matPR_NR;
					m_staBaseLineList[b_i].matERList_A_0[s_i]  = matER_A;
					m_staBaseLineList[b_i].matERList_B_0[s_i]  = matER_B;
					m_staBaseLineList[b_i].matEPList_0[s_i]    = matEP;					
				}
			}
			double factor_solar = 1.0;// 1.0E-7;//20140929
			double factor_vel   = 1.0;// 1.0E-3;
			double factor_eop   = 1.0;//1.0E-8;
			//FILE* pfile = fopen("c:\\mainOrbEst_dd.txt", "w+");
			Matrix matdx_s(count_DynParameter, 1);//改进测站位置和对流层估计时使用
			int k_num = 0;
			//FILE *pfile_1 = fopen("C:\\ocresiduals_code_k.cpp","w+");
			//FILE *pfile_2 = fopen("C:\\ocresiduals_phase_k.cpp","w+");
			//FILE *pfile_e = fopen("C:\\elevation_error_k.cpp","w+");
			//FILE *pfile_PCO = fopen("C:\\PCO_correction.cpp","w+");		
			//fclose(pfile_1);
			//fclose(pfile_2);
			//fclose(pfile_PCO);
			//FILE *pfile_gr = fopen("C:\\gravity_relative.cpp","w+");
			//FILE *pfile_sc = fopen("C:\\staSolidCor.cpp","w+");
			while(1)                                                                                                                                         
			{  
				//FILE *pfile_PCO = fopen("C:\\PCO_correction_GEO.cpp","a+");
				total_iterator++;
				if(bDDAmFixed_L1)
					iterator_after_AmbFixed++;				
				sprintf(info, "第%d次迭代...", total_iterator);
				RuningInfoFile::Add(info);
				printf("%s",info);				
				if(total_iterator >= m_podParaDefine.max_OrbitIterativeNum)
				{
					result = false;
					sprintf(info, "轨道迭代次数溢出(mainOrbEst_dd)!");
					RuningInfoFile::Add(info);
					printf("%s",info);					
					break;
				}
				// 轨道积分, 由于不知道传播时间延迟, 此处只积分出足够的准确时间点, 使用时再根据其进行插值
				for(SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); it != paraSatOrbEst.satParaList.end(); ++it)
					adamsCowell_ac(TimeCoordConvert::GPST2TDT(t0), TimeCoordConvert::GPST2TDT(t1), it->second.dynamicDatum_Est, it->second.acOrbitList, it->second.acRtPartialList);
				Matrix N_xx(count_DynParameter, count_DynParameter);  // 动力学参数
				Matrix n_xx_inv(count_DynParameter, count_DynParameter);
				Matrix nx(count_DynParameter, 1);
				Matrix matdx(count_DynParameter, 1);
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// 以先验星历拟合结果 paraSp3Fit 为观测数据, 增加初始轨道先验约束, 2014/05/11
				if(m_podParaDefine.bOnConstraints_GPSEphemeris)
				{					
					for(SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); it != paraSatOrbEst.satParaList.end(); ++it)
					{
						double weight_gpsEphemeris = m_podParaDefine.apriorityRms_LIF / m_podParaDefine.apriorityRms_GPSEphemeris; // 先验星历观测方程权值
						int id_Sat = it->first;
						if(id_Sat <= 5)
							weight_gpsEphemeris =  m_podParaDefine.apriorityRms_LIF / (m_podParaDefine.apriorityRms_GPSEphemeris * 5); //2014/09/29,刘俊宏修改用于兼容北斗
						int index  = it->second.index;
						Sp3Fit_SatdynDatumMap::iterator jt = paraSp3Fit.satParaList.find(id_Sat);
						if(jt != paraSp3Fit.satParaList.end())
						{
							// 每个采样观测时刻, 提供一个轨道位置约束
							GPST t = jt->second.fitorbitList_ECEF[0].t;
							GPST t_end =  jt->second.fitorbitList_ECEF[jt->second.fitorbitList_ECEF.size() - 1].t;
							while(t - t_end <= 0)
							{
								TimePosVel gpsOrb_priori_ECEF;
								if(jt->second.getInterpOrb_Fit(t, gpsOrb_priori_ECEF))
								{
									TDT t_TDT = TimeCoordConvert::GPST2TDT(t);
									TimePosVel gpsOrb; // 概略轨道
									Matrix interpRtPartial;// 概略轨道偏导数
									if(it->second.getEphemeris(t_TDT, gpsOrb)
									&& it->second.getInterpRtPartial(t_TDT, interpRtPartial)) // 获得对应概略轨道、偏导数
									{
										double P_ITRF[3];  // 地固坐标
										P_ITRF[0] = gpsOrb_priori_ECEF.pos.x;
										P_ITRF[1] = gpsOrb_priori_ECEF.pos.y;
										P_ITRF[2] = gpsOrb_priori_ECEF.pos.z;
										POS3D gpsOrb_priori_J2000;
										
										//double P_J2000[3]; // 惯性坐标, 用于坐标系转换
										//m_TimeCoordConvert.ECEF_J2000(t, P_J2000, P_ITRF, false); // 坐标系转换
										//gpsOrb_priori_J2000.x = P_J2000[0];
										//gpsOrb_priori_J2000.y = P_J2000[1];
										//gpsOrb_priori_J2000.z = P_J2000[2];

										 // 考虑到地球旋转参数改进影响, 20140529, 谷德峰
										Matrix matPR_NR, matER, matEP, matER_DOT;
										Matrix matJ2000Pos(3, 1), matECEFPos(3, 1);
										m_TimeCoordConvert.Matrix_J2000_ECEF(t, matPR_NR, matER, matEP, matER_DOT);
										Matrix matEst_EP, matEst_ER;
										paraSatOrbEst.getEst_EOP(t, matEst_EP, matEst_ER); // 更新 matEP, matER
										matEP = matEst_EP * matEP;
										matER = matEst_ER * matER;
										Matrix matH = matEP * matER * matPR_NR; // J2000->ECEF 矩阵
										matECEFPos.SetElement(0, 0, P_ITRF[0]);
										matECEFPos.SetElement(1, 0, P_ITRF[1]);
										matECEFPos.SetElement(2, 0, P_ITRF[2]);
										matJ2000Pos = matH.Transpose() * matECEFPos;
										gpsOrb_priori_J2000.x = matJ2000Pos.GetElement(0, 0);  
										gpsOrb_priori_J2000.y = matJ2000Pos.GetElement(1, 0); 
										gpsOrb_priori_J2000.z = matJ2000Pos.GetElement(2, 0); 

										// 计算其对 N_xx、nx 的贡献, 暂不考虑其对地球旋转参数影响
										Matrix matHt_Orb(3, count_dyn_eachSat); 
										Matrix matYt(3, 1);
										// O-C 残差
										matYt.SetElement(0, 0, weight_gpsEphemeris * (gpsOrb_priori_J2000.x - gpsOrb.pos.x));
										matYt.SetElement(1, 0, weight_gpsEphemeris * (gpsOrb_priori_J2000.y - gpsOrb.pos.y));
										matYt.SetElement(2, 0, weight_gpsEphemeris * (gpsOrb_priori_J2000.z - gpsOrb.pos.z));
										// 轨道力学参数设计矩阵
										if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_9PARA)
										{
											for(int j = 0; j < count_dyn_eachSat; j++)
											{
												matHt_Orb.SetElement(0, j, weight_gpsEphemeris * interpRtPartial.GetElement(0, j));
												matHt_Orb.SetElement(1, j, weight_gpsEphemeris * interpRtPartial.GetElement(1, j));
												matHt_Orb.SetElement(2, j, weight_gpsEphemeris * interpRtPartial.GetElement(2, j));
											}
										}
										else if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
										{
											for(int j = 0; j < 6; j++)
											{
												matHt_Orb.SetElement(0, j, weight_gpsEphemeris * interpRtPartial.GetElement(0, j));
												matHt_Orb.SetElement(1, j, weight_gpsEphemeris * interpRtPartial.GetElement(1, j));
												matHt_Orb.SetElement(2, j, weight_gpsEphemeris * interpRtPartial.GetElement(2, j));
											}
											for(size_t s_k = 0; s_k < it->second.dynamicDatum_Est.solarPressureParaList.size(); s_k++)
											{
												// D Y B + B 1/rev
												matHt_Orb.SetElement(0, 6 + 5 * (int)s_k + 0, weight_gpsEphemeris * interpRtPartial.GetElement(0, 6 + 9 * (int)s_k + 0));
												matHt_Orb.SetElement(1, 6 + 5 * (int)s_k + 0, weight_gpsEphemeris * interpRtPartial.GetElement(1, 6 + 9 * (int)s_k + 0));
												matHt_Orb.SetElement(2, 6 + 5 * (int)s_k + 0, weight_gpsEphemeris * interpRtPartial.GetElement(2, 6 + 9 * (int)s_k + 0));
												matHt_Orb.SetElement(0, 6 + 5 * (int)s_k + 1, weight_gpsEphemeris * interpRtPartial.GetElement(0, 6 + 9 * (int)s_k + 3));
												matHt_Orb.SetElement(1, 6 + 5 * (int)s_k + 1, weight_gpsEphemeris * interpRtPartial.GetElement(1, 6 + 9 * (int)s_k + 3));
												matHt_Orb.SetElement(2, 6 + 5 * (int)s_k + 1, weight_gpsEphemeris * interpRtPartial.GetElement(2, 6 + 9 * (int)s_k + 3));
												for(int j = 2; j < 5; j++)
												{
													matHt_Orb.SetElement(0, 6 + 5 * (int)s_k + j, weight_gpsEphemeris * interpRtPartial.GetElement(0, 6 + 9 * (int)s_k + j + 4));
													matHt_Orb.SetElement(1, 6 + 5 * (int)s_k + j, weight_gpsEphemeris * interpRtPartial.GetElement(1, 6 + 9 * (int)s_k + j + 4));
													matHt_Orb.SetElement(2, 6 + 5 * (int)s_k + j, weight_gpsEphemeris * interpRtPartial.GetElement(2, 6 + 9 * (int)s_k + j + 4));
												}
											}
										}
										Matrix N_orb  = matHt_Orb.Transpose() * matHt_Orb;
										Matrix ny_orb = matHt_Orb.Transpose() * matYt;
										for(int ii = 0; ii < count_dyn_eachSat; ii++)
										{
											for(int jj = 0; jj < count_dyn_eachSat; jj++)
											{
												N_xx.SetElement(index * count_dyn_eachSat + ii, index * count_dyn_eachSat + jj, N_xx.GetElement(index * count_dyn_eachSat + ii, index * count_dyn_eachSat + jj) + N_orb.GetElement(ii, jj));
											}
											nx.SetElement(index * count_dyn_eachSat + ii, 0, nx.GetElement(index * count_dyn_eachSat + ii, 0) + ny_orb.GetElement(ii, 0));
										}
									}
								}
								t = t + m_podParaDefine.sampleSpan;// 采样时间
							}
						}
					}
				}
			    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
				{
					// 更新 dynamicEpochList
					size_t count_epoch = m_staBaseLineList[b_i].editedSdObsFile.m_data.size(); // 观测历元个数
					int s_index = -1;     // 有效时间标签, 初始化为 0
					m_staBaseLineList[b_i].matEPList.resize(count_epoch);
					m_staBaseLineList[b_i].matERList_A.resize(count_epoch);
					m_staBaseLineList[b_i].matERList_B.resize(count_epoch);
					for(size_t s_i = 0; s_i < count_epoch; s_i++)
					{
			            // 更新地球旋转参数矩阵
						Matrix matEst_EP, matEst_ER_A, matEst_ER_B;
						paraSatOrbEst.getEst_EOP(m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t  - m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].A_clock / SPEED_LIGHT, matEst_EP, matEst_ER_A);// 更新 matEP, matER
						paraSatOrbEst.getEst_EOP(m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t  - m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].B_clock / SPEED_LIGHT, matEst_EP, matEst_ER_B);
						m_staBaseLineList[b_i].matEPList[s_i] = matEst_EP * m_staBaseLineList[b_i].matEPList_0[s_i];
						m_staBaseLineList[b_i].matERList_A[s_i] = matEst_ER_A * m_staBaseLineList[b_i].matERList_A_0[s_i];
						m_staBaseLineList[b_i].matERList_B[s_i] = matEst_ER_B * m_staBaseLineList[b_i].matERList_B_0[s_i];
						
						// matPRNR-1 × matER-1 × matEP-1 × matECFPos_C = matJ2000Pos
						POSCLK  pos_A, pos_B; // 概略点位置转换到 J2000
						POS3D   pos_A_cor,pos_B_cor; // 测站位置的固体潮改正量
						pos_A.clk = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].A_clock;
						pos_B.clk = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].B_clock;
						GPST t_gps =  m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t;
						double P_J2000[3]; // 惯性坐标, 用于坐标系转换
						double P_ITRF[3];  // 地固坐标
						if(m_podParaDefine.bOn_StaSolidTideCor)
						{						
							TDB t_TDB = m_TimeCoordConvert.GPST2TDB(t_gps); // 获得TDB时间--提供太阳历参考时间
							double jd_TDB = m_TimeCoordConvert.DayTime2JD(t_TDB); // 获得儒略日
							// 获得太阳位置 
							POS3D sunPos_ITRF;
							POS3D sunPos_J2000;
							m_JPLEphFile.getSunPos_Delay_EarthCenter(jd_TDB, P_J2000); 
							for(int i = 0; i < 3; i ++)
								P_J2000[i] = P_J2000[i] * 1000; // 换算成米
							sunPos_J2000.x = P_J2000[0];
							sunPos_J2000.y = P_J2000[1];
							sunPos_J2000.z = P_J2000[2];
							m_TimeCoordConvert.J2000_ECEF(t_gps, P_J2000, P_ITRF, false); // 坐标系转换
							sunPos_ITRF.x = P_ITRF[0];
							sunPos_ITRF.y = P_ITRF[1];
							sunPos_ITRF.z = P_ITRF[2];
							// 获得月球的位置
							POS3D moonPos_ITRF;
							m_JPLEphFile.getPlanetPos(JPLEph_Moon, jd_TDB, P_J2000);  // 获得J2000系下的太阳相对地心的位置（千米）
							for(int i = 0; i < 3; i ++)
								P_J2000[i] = P_J2000[i] * 1000;                       // 换算成米
							m_TimeCoordConvert.J2000_ECEF(t_gps, P_J2000, P_ITRF, false); // 坐标系转换
							moonPos_ITRF.x  = P_ITRF[0];
							moonPos_ITRF.y  = P_ITRF[1];
							moonPos_ITRF.z  = P_ITRF[2];	
							double xp = 0;
							double yp = 0;
							if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_2003)
								m_TimeCoordConvert.m_eopRapidFileIAU2000.getPoleOffset(m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(t_gps)), xp, yp);
							pos_A_cor = SolidTides::solidTideCorrect(t_gps, sunPos_ITRF, moonPos_ITRF, m_mapStaDatum[m_staBaseLineList[b_i].name_A].pos_Est, xp, yp);
							pos_B_cor = SolidTides::solidTideCorrect(t_gps, sunPos_ITRF, moonPos_ITRF, m_mapStaDatum[m_staBaseLineList[b_i].name_B].pos_Est, xp, yp);							
						}			
						POS3D pos_A_cor_ocean,pos_B_cor_ocean; // 测站位置的海潮改正量
						if(m_podParaDefine.bOn_StaOceanTides)
						{
							StaOceanTide sotDatum_A;
							if(m_staOldFile.getStaOceanTide(m_staBaseLineList[b_i].name_A, sotDatum_A))
								pos_A_cor_ocean = OceanTidesLoading::oceanTideLoadingCorrect(t_gps, m_mapStaDatum[m_staBaseLineList[b_i].name_A].pos_Est, sotDatum_A);
							StaOceanTide sotDatum_B;
							if(m_staOldFile.getStaOceanTide(m_staBaseLineList[b_i].name_B, sotDatum_B))
								pos_B_cor_ocean = OceanTidesLoading::oceanTideLoadingCorrect(t_gps, m_mapStaDatum[m_staBaseLineList[b_i].name_B].pos_Est, sotDatum_B);
						}
						P_ITRF[0] = m_mapStaDatum[m_staBaseLineList[b_i].name_A].pos_Est.x + pos_A_cor.x + pos_A_cor_ocean.x;
						P_ITRF[1] = m_mapStaDatum[m_staBaseLineList[b_i].name_A].pos_Est.y + pos_A_cor.y + pos_A_cor_ocean.y;
						P_ITRF[2] = m_mapStaDatum[m_staBaseLineList[b_i].name_A].pos_Est.z + pos_A_cor.z + pos_A_cor_ocean.z;
						
						// 考虑到地球旋转参数改进影响, 20140529, 谷德峰
						Matrix matJ2000Pos(3, 1), matECEFPos(3, 1);
						Matrix matH_A = m_staBaseLineList[b_i].matEPList[s_i] * m_staBaseLineList[b_i].matERList_A[s_i] * m_staBaseLineList[b_i].matPR_NRList[s_i]; // J2000->ECEF 矩阵
						matECEFPos.SetElement(0, 0, P_ITRF[0]);
						matECEFPos.SetElement(1, 0, P_ITRF[1]);
						matECEFPos.SetElement(2, 0, P_ITRF[2]);
						matJ2000Pos = matH_A.Transpose() * matECEFPos;
						pos_A.x = matJ2000Pos.GetElement(0, 0);  
						pos_A.y = matJ2000Pos.GetElement(1, 0); 
						pos_A.z = matJ2000Pos.GetElement(2, 0); 

						P_ITRF[0] = m_mapStaDatum[m_staBaseLineList[b_i].name_B].pos_Est.x + pos_B_cor.x + pos_B_cor_ocean.x;
						P_ITRF[1] = m_mapStaDatum[m_staBaseLineList[b_i].name_B].pos_Est.y + pos_B_cor.y + pos_B_cor_ocean.y;
						P_ITRF[2] = m_mapStaDatum[m_staBaseLineList[b_i].name_B].pos_Est.z + pos_B_cor.z + pos_B_cor_ocean.z;
						
						// 考虑到地球旋转参数改进影响,  20140529, 谷德峰
						Matrix matH_B = m_staBaseLineList[b_i].matEPList[s_i] * m_staBaseLineList[b_i].matERList_B[s_i] * m_staBaseLineList[b_i].matPR_NRList[s_i]; // J2000->ECEF 矩阵
						matECEFPos.SetElement(0, 0, P_ITRF[0]);
						matECEFPos.SetElement(1, 0, P_ITRF[1]);
						matECEFPos.SetElement(2, 0, P_ITRF[2]);
						matJ2000Pos = matH_B.Transpose() * matECEFPos;
						pos_B.x = matJ2000Pos.GetElement(0, 0);  
						pos_B.y = matJ2000Pos.GetElement(1, 0); 
						pos_B.z = matJ2000Pos.GetElement(2, 0); 

						TDT t_obs_TDT = TimeCoordConvert::GPST2TDT(m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t);
						//pos_A.clk = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].A_clock;
						//pos_B.clk = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].B_clock;
						GPST t_Receive_A = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t - pos_A.clk / SPEED_LIGHT;
						GPST t_Receive_B = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t - pos_B.clk / SPEED_LIGHT; 
						m_staBaseLineList[b_i].staPosList_A.insert(StaPosMap::value_type(int(s_i),pos_A.getPos()));
						m_staBaseLineList[b_i].staPosList_B.insert(StaPosMap::value_type(int(s_i),pos_B.getPos()));
						m_staBaseLineList[b_i].staECEFPosList_A.insert(StaPosMap::value_type(int(s_i),m_mapStaDatum[m_staBaseLineList[b_i].name_A].pos_Est + pos_A_cor + pos_A_cor_ocean));
						m_staBaseLineList[b_i].staECEFPosList_B.insert(StaPosMap::value_type(int(s_i),m_mapStaDatum[m_staBaseLineList[b_i].name_B].pos_Est + pos_B_cor + pos_B_cor_ocean));
						if(m_staBaseLineList[b_i].dynEpochList[s_i].validIndex != -1)
						{// 仅对有效点进行更新
							// 将同一时刻对流层修正使用的相同数值放在前面，以提高效率,2013/9/25
							GPST    t_epoch,t0_A,t1_A,t0_B,t1_B;
							int     index_t0_A,index_t0_B;
							double  estZTD_t0_A,estZTD_t1_A,estZTD_t0_B,estZTD_t1_B,t_coef_A,t_coef_B;
							if(m_podParaDefine.bOnEst_StaTropZenithDelay)
							{
								t_epoch     = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t;                                           // 当前历元的时间
								index_t0_A  = m_staBaseLineList[b_i].id_ZenithDelayList_A[s_i];                                               // t0时刻的对流层估计参数在该测站对流层估计参数列表中的位置
								index_t0_B  = m_staBaseLineList[b_i].id_ZenithDelayList_B[s_i];
								t0_A        = m_mapStaDatum[m_staBaseLineList[b_i].name_A].zenithDelayEstList[index_t0_A].t;                  // 与当前时刻有关的对流层估计区间的左端点([t0,t1])
								t1_A        = m_mapStaDatum[m_staBaseLineList[b_i].name_A].zenithDelayEstList[index_t0_A + 1].t;              // 与当前时刻有关的对流层估计区间的右端点
								t0_B        = m_mapStaDatum[m_staBaseLineList[b_i].name_B].zenithDelayEstList[index_t0_B].t;
								t1_B        = m_mapStaDatum[m_staBaseLineList[b_i].name_B].zenithDelayEstList[index_t0_B + 1].t;						
								estZTD_t0_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].zenithDelayEstList[index_t0_A].zenithDelay_Est;    // t0时刻对流层参数估计值
								estZTD_t1_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].zenithDelayEstList[index_t0_A + 1].zenithDelay_Est;// t1时刻对流层参数估计值
								estZTD_t0_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].zenithDelayEstList[index_t0_B].zenithDelay_Est;    
								estZTD_t1_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].zenithDelayEstList[index_t0_B + 1].zenithDelay_Est;							
								t_coef_A    = (t_epoch - t0_A)/(t1_A - t0_A);                                                                 // 与时间相关的系数
								t_coef_B    = (t_epoch - t0_B)/(t1_B - t0_B);
							}
							int    eyeableGPSCount = 0;
							int    j = 0;
							for(Rinex2_1_EditedSdObsSatMap::iterator it = m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs.begin(); it != m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs.end(); ++it)
							{
								int id_Sat = it->first;  // 第 j 颗可见BD卫星的卫星号
								PODEpochSatMap::iterator datum_j = m_staBaseLineList[b_i].dynEpochList[s_i].mapDatum.find(id_Sat);
								double delay_A = 0, delay_B = 0;
								TimePosVel bdsOrb_A, bdsOrb_B;
								bool bEphemeris = true;
								// 编写 getEphemeris_PathDelay 函数, 根据 m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].t, pos_A 计算传播延迟和 BD 卫星位置
								if(!paraSatOrbEst.satParaList[id_Sat].getEphemeris_PathDelay(t_obs_TDT, pos_A, delay_A, bdsOrb_A, datum_j->second.interpRtPartial_A)
								|| !paraSatOrbEst.satParaList[id_Sat].getEphemeris_PathDelay(t_obs_TDT, pos_B, delay_B, bdsOrb_B, datum_j->second.interpRtPartial_B)) 
									bEphemeris = false;								
								double distance_A = sqrt(pow(pos_A.x - bdsOrb_A.pos.x, 2) + pow(pos_A.y - bdsOrb_A.pos.y, 2) + pow(pos_A.z - bdsOrb_A.pos.z, 2));
								double distance_B = sqrt(pow(pos_B.x - bdsOrb_B.pos.x, 2) + pow(pos_B.y - bdsOrb_B.pos.y, 2) + pow(pos_B.z - bdsOrb_B.pos.z, 2));
								datum_j->second.vecLos_A.x = (pos_A.x - bdsOrb_A.pos.x) / distance_A;
								datum_j->second.vecLos_A.y = (pos_A.y - bdsOrb_A.pos.y) / distance_A;
								datum_j->second.vecLos_A.z = (pos_A.z - bdsOrb_A.pos.z) / distance_A;
								datum_j->second.vecLos_A.clk  = 1.0;
								datum_j->second.vecLos_B.x = (pos_B.x - bdsOrb_B.pos.x) / distance_B;
								datum_j->second.vecLos_B.y = (pos_B.y - bdsOrb_B.pos.y) / distance_B;
								datum_j->second.vecLos_B.z = (pos_B.z - bdsOrb_B.pos.z) / distance_B;
								datum_j->second.vecLos_B.clk  = 1.0;
								GPST t_Transmit_A = t_Receive_A - delay_A;
					    		GPST t_Transmit_B = t_Receive_B - delay_B;								
								if(!bEphemeris)
								{// 星历不完整, 屏蔽该数据									
									datum_j->second.obscorrected_value = 0;
									datum_j->second.weightCode  = 0;  
									datum_j->second.weightPhase = 0;
								}
								else
								{
									eyeableGPSCount++;
									// 获得 J2000 系下的太阳相对地心的位置(千米)
									POS3D sunPos;                     // 太阳相对地心位置
									TDB t_TDB = TimeCoordConvert::GPST2TDB(t_Transmit_A); // 获得 TDB 时间--提供太阳历参考时间									
									double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
									double Pos[3];
									m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos);									
									sunPos.x = Pos[0] * 1000; 
									sunPos.y = Pos[1] * 1000; 
									sunPos.z = Pos[2] * 1000; 								
									// 1 BD卫星相对论改正
									double correct_bdsrelativity_A = 0;
									double correct_bdsrelativity_B = 0;
									if(m_podParaDefine.bOn_BDSRelativity)
									{
										correct_bdsrelativity_A = (bdsOrb_A.pos.x * bdsOrb_A.vel.x + bdsOrb_A.pos.y * bdsOrb_A.vel.y + bdsOrb_A.pos.z * bdsOrb_A.vel.z) * (-2) / SPEED_LIGHT;
										correct_bdsrelativity_B = (bdsOrb_B.pos.x * bdsOrb_B.vel.x + bdsOrb_B.pos.y * bdsOrb_B.vel.y + bdsOrb_B.pos.z * bdsOrb_B.vel.z) * (-2) / SPEED_LIGHT;										
									}
									// 2 BD卫星天线 PCO 修正
									double correct_bdspco_A = 0;
									double correct_bdspco_B = 0;
									if(m_podParaDefine.bOn_BDSAntPCO)
									{//2013/9/16,刘俊宏			
										
											map<int, AntCorrectionBlk>::iterator it_BDSAntCorrectionBlk = m_mapBDSAntCorrectionBlk.find(id_Sat);
											if(it_BDSAntCorrectionBlk != m_mapBDSAntCorrectionBlk.end())
											{
												double correct_bdspco_A_F1 = 0; //测站A对BD卫星的第一个频点数据PCO修正
												double correct_bdspco_A_F2 = 0; //测站A对BD卫星的第二个频点数据PCO修正
												double correct_bdspco_B_F1 = 0; //测站B对BD卫星的第一个频点数据PCO修正
												double correct_bdspco_B_F2 = 0; //测站B对BD卫星的第二个频点数据PCO修正
												if(id_Sat > 5)  // IGSO卫星和MEO卫星PCO修正
												{	
													correct_bdspco_A_F1 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, FREQUENCE_1 - 1, pos_A.getPos(), bdsOrb_A.pos, sunPos, false);
													correct_bdspco_A_F2 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, FREQUENCE_2 - 1, pos_A.getPos(), bdsOrb_A.pos, sunPos, false);
													correct_bdspco_B_F1 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, FREQUENCE_1 - 1, pos_B.getPos(), bdsOrb_B.pos, sunPos, false);
													correct_bdspco_B_F2 = m_AtxFile.correctSatAntPCOPCV(it_BDSAntCorrectionBlk->second, FREQUENCE_2 - 1, pos_B.getPos(), bdsOrb_B.pos, sunPos, false);
													correct_bdspco_A = correct_bdspco_A_F1 - coefficient_IF * (correct_bdspco_A_F1 - correct_bdspco_A_F2); 
													correct_bdspco_B = correct_bdspco_B_F1 - coefficient_IF * (correct_bdspco_B_F1 - correct_bdspco_B_F2);
													//fprintf(pfile_PCO,"%3d %s %3d %3d %3d %14.6lf %14.6lf\n",total_iterator,t_Transmit_A.toString().c_str(),b_i + 1,s_i,id_Sat,correct_bdspco_A,correct_bdspco_B);
												}
												else  // GEO卫星PCO修正，20140824，刘俊宏
												{//经测试，分别由t_Transmit_A，t_Transmit_B计算的卫星性体系的差别很小，可以忽略。因此只计算一次卫星星体系即可
													POS3D vecLOS_A = vectorNormal(pos_A.getPos() - bdsOrb_A.pos);	// 视线单位矢量，卫星指向测站
													POS3D vecLOS_B = vectorNormal(pos_B.getPos() - bdsOrb_B.pos);	// 视线单位矢量，卫星指向测站
													POS6D bdsOrbposvel;
													bdsOrbposvel.setPos(bdsOrb_A.pos);
													bdsOrbposvel.setVel(bdsOrb_A.vel);													
													POS3D axisvec_R, axisvec_T, axisvec_N;													
													m_TimeCoordConvert.getCoordinateRTNAxisVector(m_TimeCoordConvert.GPST2UT1(t_Transmit_A), bdsOrbposvel, axisvec_R, axisvec_T, axisvec_N);
													POS3D ey, ez;													
													//轨道系坐标轴 R 对应星体系的"-Z"方向
													ez.x = - axisvec_R.x;
													ez.y = - axisvec_R.y;
													ez.z = - axisvec_R.z;
													//轨道系坐标轴 N 对应星体系的"-Y"方向
													ey.x = - axisvec_N.x;
													ey.y = - axisvec_N.y;
													ey.z = - axisvec_N.z;
													//轨道系坐标轴 T 对应星体系的"+X"方向(即ex)
													correct_bdspco_A_F1 = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_BDSAntCorrectionBlk->second, FREQUENCE_1 - 1, vecLOS_A, axisvec_T, ey, ez, false);
													correct_bdspco_A_F2 = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_BDSAntCorrectionBlk->second, FREQUENCE_2 - 1, vecLOS_A, axisvec_T, ey, ez, false);
													correct_bdspco_B_F1 = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_BDSAntCorrectionBlk->second, FREQUENCE_1 - 1, vecLOS_B, axisvec_T, ey, ez, false);
													correct_bdspco_B_F2 = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_BDSAntCorrectionBlk->second, FREQUENCE_2 - 1, vecLOS_B, axisvec_T, ey, ez, false);
													correct_bdspco_A = correct_bdspco_A_F1 - coefficient_IF * (correct_bdspco_A_F1 - correct_bdspco_A_F2); 
													correct_bdspco_B = correct_bdspco_B_F1 - coefficient_IF * (correct_bdspco_B_F1 - correct_bdspco_B_F2);
													//fprintf(pfile_PCO,"%3d %s %3d %3d %3d %14.6lf %14.6lf\n",total_iterator,t_Transmit_A.toString().c_str(),b_i + 1,s_i,id_Sat,correct_bdspco_A,correct_bdspco_B);
													//fprintf(pfile_PCO,"%14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf %14.6lf\n",axisvec_T_A.x,axisvec_T_A.y,axisvec_T_A.z,axisvec_T_B.x,axisvec_T_B.y,axisvec_T_B.z,ey_A.x,ey_A.y,ey_A.z,ey_B.x,ey_B.y,ey_B.z);
												}	
											}										
										
									}
									// 4 接收天线 ARP 修正
									double correct_recARP_A = 0;
									double correct_recARP_B = 0;
									if(m_podParaDefine.bOn_RecAntARP)
									{// 2013/09/23, 东北天坐标系下的天线偏移修正
										POS3D U_A,U_B;       // 垂直径向
										U_A =  pos_A.getPos();
										U_B =  pos_B.getPos();
										POS3D N_A, N_B;      // 北方向
										N_A.x = 0;
										N_A.y = 0;
										N_A.z = EARTH_R; 
										N_B.x = 0;
										N_B.y = 0;
										N_B.z = EARTH_R;     // 北极点	
										POS3D E_A,E_B;       // 东方向
										vectorCross(E_A,N_A,U_A);
										vectorCross(N_A,U_A,E_A);
										E_A = vectorNormal(E_A);
										N_A = vectorNormal(N_A);
										U_A = vectorNormal(U_A);
										vectorCross(E_B,N_B,U_B);
										vectorCross(N_B,U_B,E_B);
										E_B = vectorNormal(E_B);
										N_B = vectorNormal(N_B);
										U_B = vectorNormal(U_B);
										POS3D offsetJ2000_A = E_A * m_staBaseLineList[b_i].arpAnt_A.E + N_A * m_staBaseLineList[b_i].arpAnt_A.N + U_A * m_staBaseLineList[b_i].arpAnt_A.U;
										correct_recARP_A = -(offsetJ2000_A.x * datum_j->second.vecLos_A.x + offsetJ2000_A.y * datum_j->second.vecLos_A.y + offsetJ2000_A.z * datum_j->second.vecLos_A.z);
										POS3D offsetJ2000_B = E_B * m_staBaseLineList[b_i].arpAnt_B.E + N_B * m_staBaseLineList[b_i].arpAnt_B.N + U_B * m_staBaseLineList[b_i].arpAnt_B.U;
										correct_recARP_B = -(offsetJ2000_B.x * datum_j->second.vecLos_B.x + offsetJ2000_B.y * datum_j->second.vecLos_B.y + offsetJ2000_B.z * datum_j->second.vecLos_B.z);
									}
									// 4 接收天线 PCO/PCV 修正
									double correct_recpcopcv_A = 0;
									double correct_recpcopcv_B = 0;
									if(m_podParaDefine.bOn_RecAntPCOPCV)
									{
										map<string, AntCorrectionBlk>::iterator it_RecAntCorrectionBlk_A = mapRecAntCorrectionBlk.find(m_mapStaDatum[m_staBaseLineList[b_i].name_A].szAntType);
										map<string, AntCorrectionBlk>::iterator it_RecAntCorrectionBlk_B = mapRecAntCorrectionBlk.find(m_mapStaDatum[m_staBaseLineList[b_i].name_B].szAntType);
										if(it_RecAntCorrectionBlk_A != mapRecAntCorrectionBlk.end() && it_RecAntCorrectionBlk_B != mapRecAntCorrectionBlk.end())
										{
											correct_recpcopcv_A = coefficient_IF_L1 * m_AtxFile.correctRecAntPCOPCV(it_RecAntCorrectionBlk_A->second, 0, pos_A.getPos(), bdsOrb_A.pos, true)
												                + coefficient_IF_L2 * m_AtxFile.correctRecAntPCOPCV(it_RecAntCorrectionBlk_A->second, 1, pos_A.getPos(), bdsOrb_A.pos, true);
											correct_recpcopcv_B = coefficient_IF_L1 * m_AtxFile.correctRecAntPCOPCV(it_RecAntCorrectionBlk_B->second, 0, pos_B.getPos(), bdsOrb_B.pos, true)
												                + coefficient_IF_L2 * m_AtxFile.correctRecAntPCOPCV(it_RecAntCorrectionBlk_B->second, 1, pos_B.getPos(), bdsOrb_B.pos, true);
										}			
									}
									// 5 对流层湿分量修正
									double correct_trowet_A = 0;
									double correct_trowet_B = 0;
									if(m_podParaDefine.bOnEst_StaTropZenithDelay)
									{									
										correct_trowet_A = ((estZTD_t1_A - estZTD_t0_A) * t_coef_A + estZTD_t0_A ) / sin(m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs[id_Sat].Elevation_A * PI/180);
										correct_trowet_B = ((estZTD_t1_B - estZTD_t0_B) * t_coef_B + estZTD_t0_B ) / sin(m_staBaseLineList[b_i].editedSdObsFile.m_data[s_i].editedObs[id_Sat].Elevation_B * PI/180);

									}
									// 6 引力引起的相对论改正,bOn_BDSGravRelativity = false;20140929
									double correct_grarelativity_A = 0;
									double correct_grarelativity_B = 0;
									if(m_podParaDefine.bOn_BDSGraRelativity)
									{																		
										correct_grarelativity_A = GNSSBasicCorrectFunc::graRelativityCorrect(bdsOrb_A.pos, pos_A.getPos());
										correct_grarelativity_B = GNSSBasicCorrectFunc::graRelativityCorrect(bdsOrb_B.pos, pos_B.getPos());										
									}
									// 修正量
									datum_j->second.obscorrected_value = correct_bdsrelativity_A - correct_bdsrelativity_B
																	   + correct_bdspco_A  - correct_bdspco_B
																	   + correct_recARP_A - correct_recARP_B
																	   +(correct_recpcopcv_A - correct_recpcopcv_B)
																	   - (correct_trowet_A - correct_trowet_B)
																	   - (correct_grarelativity_A - correct_grarelativity_B)
																	   - (distance_A - distance_B);
								}
								j++;
							}
							// 更新可视卫星个数统计结果
							m_staBaseLineList[b_i].dynEpochList[s_i].eyeableGPSCount = eyeableGPSCount;
							if(m_staBaseLineList[b_i].dynEpochList[s_i].eyeableGPSCount <= 2)//2013/04/17
								m_staBaseLineList[b_i].dynEpochList[s_i].validIndex = -1;
							else
							{
								s_index++;
								m_staBaseLineList[b_i].dynEpochList[s_i].validIndex = s_index;
							}
						}
					}
				}
				if(total_iterator >= 2 && bResEdit && !flag_break)
				{// 进行残差编辑
					// 伪码残差					
					double rms_oc_code  = 0;
					int count_validcode = 0;
					for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
					{
						m_staBaseLineList[b_i].rms_oc_code = 0.0;
						int count_validcode_i = 0;
						for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
						{
							for(size_t s_i = 0; s_i < m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList.size(); s_i++)
							{  
								int count_obs = int(m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList.size());
								int nObsTime = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].nObsTime; 
								int id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].id_DDRefSat;     
								int id_DDObs = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].id_DDObs;
								double w_ref_P = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].weightCode
											   * m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[id_DDObs].robustweight 
											   * m_staBaseLineList[b_i].weight_baseline; 
								if(w_ref_P == 0) 
									w_ref_P = zeroweight_code;
								for(int s_j = 0; s_j < count_obs; s_j++)
								{
									int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
									if(id_Sat == id_DDRefSat)
										continue;
									// 经过概略点改进后的观测值
									double o_c_DD_P_IF = (m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].obscorrected_value)
													   - (m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[id_DDObs].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].obscorrected_value);
									double w_P = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].weightCode
										   * m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].robustweight
										   * m_staBaseLineList[b_i].weight_baseline;

									double w = 1 / sqrt(0.5 / (w_ref_P * w_ref_P) + 0.5 / (w_P * w_P)); // 双差残差, 与非差相差 2 倍

									if(m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].robustweight == 1.0
									&& m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].weightCode != 0.0)
									{
										count_validcode++;
										count_validcode_i++;
										m_staBaseLineList[b_i].rms_oc_code += pow(o_c_DD_P_IF * w, 2);
										rms_oc_code += pow(o_c_DD_P_IF * w, 2);
									}
								}
							}
						}
						m_staBaseLineList[b_i].rms_oc_code = sqrt(m_staBaseLineList[b_i].rms_oc_code / count_validcode_i);
					}
					rms_oc_code = sqrt(rms_oc_code / count_validcode);
					//sprintf(info, "伪码定轨残差 rms_oc_code = %.5f", rms_oc_code);
					//RuningInfoFile::Add(info);
					//printf("%s\n",info);//
					//printf("伪码定轨残差 rms_oc_code = %.5f\n", rms_oc_code);
					// 相位残差
					double rms_oc_phase = 0;
					int count_validphase = 0;
					for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
					{
						m_staBaseLineList[b_i].rms_oc_phase = 0;
						int count_validphase_i = 0;
						for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
						{
							for(size_t s_i = 0; s_i < m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList.size(); s_i++)
							{  
								int count_obs = int(m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList.size());
								int nObsTime = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].nObsTime; 
								int id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDRefSat;   
								int id_DDRefAmbiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDRefAmbiguity;   
								int id_DDObs = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDObs;
								double w_ref_L = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].weightPhase
											   * m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[id_DDObs].robustweight 
											   * m_staBaseLineList[b_i].weight_baseline; 
								if(w_ref_L == 0)
									w_ref_L = zeroweight_phase;
								for(int s_j = 0; s_j < count_obs; s_j++)
								{
									int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
									int id_Ambiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Ambiguity;
									if(id_Sat == id_DDRefSat)
										continue;
									double ambiguity_DD_L1 = 0;
									double ambiguity_DD_MW = 0;
									if(id_DDRefAmbiguity != 0)
									{
										ambiguity_DD_L1 -= m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[id_DDRefAmbiguity - 1] * BD_WAVELENGTH_N;
										ambiguity_DD_MW -= m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List[id_DDRefAmbiguity - 1] * coeff_mw;
									}
									if(id_Ambiguity != 0)
									{
										ambiguity_DD_L1 += m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[id_Ambiguity - 1] * BD_WAVELENGTH_N;
										ambiguity_DD_MW += m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List[id_Ambiguity - 1] * coeff_mw;
									}
									// 经过概略点改进后的观测值, 扣除双差模糊度的影响
									double o_c_DD_L_IF = (m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].obscorrected_value)
													   - (m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[id_DDObs].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].obscorrected_value)
													   -  ambiguity_DD_MW // 扣除宽巷模糊度的影响
													   -  ambiguity_DD_L1;
									double w_L = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].weightPhase
										   * m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].robustweight
										   * m_staBaseLineList[b_i].weight_baseline; 
									double w = 1 / sqrt(0.5 / (w_ref_L * w_ref_L) + 0.5 / (w_L * w_L));  // 双差残差, 与非差相差 2 倍
									if(m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].robustweight == 1.0
									&& m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].weightPhase != 0.0)
									{
										count_validphase++;
										count_validphase_i++;
										rms_oc_phase += pow(o_c_DD_L_IF * w, 2);
										m_staBaseLineList[b_i].rms_oc_phase += pow(o_c_DD_L_IF * w, 2);
									}
								}
							}
						}
						m_staBaseLineList[b_i].rms_oc_phase = sqrt(m_staBaseLineList[b_i].rms_oc_phase / count_validphase_i);
					}
					rms_oc_phase = sqrt(rms_oc_phase / count_validphase);
					//sprintf(info, "相位定轨残差 rms_oc_phase = %.5f", rms_oc_phase);
					//RuningInfoFile::Add(info);
					//printf("%s\n",info);//
					//printf("相位定轨残差 rms_oc_phase = %.5f\n", rms_oc_phase);
					// 根据伪码、相位残差更新权值
					for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
					{
						for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
						{
							for(size_t s_i = 0; s_i < m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList.size(); s_i++)
							{// 根据伪码残差更新权值
								int count_obs = int(m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList.size());
								int nObsTime = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].nObsTime; 
								int id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].id_DDRefSat;     
								int id_DDObs = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].id_DDObs;
								double w_ref_P = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].weightCode
											   * m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[id_DDObs].robustweight 
											   * m_staBaseLineList[b_i].weight_baseline; // 20140421, 谷德峰调整, 调整基线权重
								if(w_ref_P == 0)
									w_ref_P = zeroweight_code;
								for(int s_j = 0; s_j < count_obs; s_j++)
								{
									int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
									if(id_Sat == id_DDRefSat)
										continue;
									// 经过概略点改进后的观测值
									double o_c_DD_P_IF = (m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].obscorrected_value)
													   - (m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[id_DDObs].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].obscorrected_value);
									double w_P = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].weightCode
										   * m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].robustweight
										   * m_staBaseLineList[b_i].weight_baseline; 
									double w = 1 / sqrt(0.5 / (w_ref_P * w_ref_P) + 0.5 / (w_P * w_P)); // 双差残差与非差相差 2 倍 
									// 确定观测权值
									if(fabs(w * o_c_DD_P_IF) > rms_oc_code * m_podParaDefine.robustfactor_OC_edited)
										m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].robustweight = rms_oc_code / (fabs(w * o_c_DD_P_IF) * sqrt(2.0));
									    //m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].robustweight = 0.0;
									else
										m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].robustweight = 1.0;
								}
							}
							for(size_t s_i = 0; s_i < m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList.size(); s_i++)
							{// 根据相位残差更新权值  
								int count_obs = int(m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList.size());
								int nObsTime = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].nObsTime; 
								int id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDRefSat;   
								int id_DDRefAmbiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDRefAmbiguity;   
								int id_DDObs = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDObs;
								double w_ref_L = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].weightPhase
											   * m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[id_DDObs].robustweight 
											   * m_staBaseLineList[b_i].weight_baseline; 
								if(w_ref_L == 0)
									w_ref_L = zeroweight_phase;
								for(int s_j = 0; s_j < count_obs; s_j++)
								{
									int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
									int id_Ambiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Ambiguity;
									if(id_Sat == id_DDRefSat)
										continue;
									double ambiguity_DD_L1 = 0;
									double ambiguity_DD_MW = 0;
									if(id_DDRefAmbiguity != 0)
									{
										ambiguity_DD_L1 -= m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[id_DDRefAmbiguity - 1] * BD_WAVELENGTH_N;
										ambiguity_DD_MW -= m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List[id_DDRefAmbiguity - 1] * coeff_mw;
									}
									if(id_Ambiguity != 0)
									{
										ambiguity_DD_L1 += m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[id_Ambiguity - 1] * BD_WAVELENGTH_N;
										ambiguity_DD_MW += m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List[id_Ambiguity - 1] * coeff_mw;
									}
									// 经过概略点改进后的观测值, 扣除双差模糊度的影响
									double o_c_DD_L_IF = (m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].obscorrected_value)
													   - (m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[id_DDObs].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].obscorrected_value)
													   -  ambiguity_DD_MW // 扣除宽巷模糊度的影响
													   -  ambiguity_DD_L1;
									double w_L = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].weightPhase
										   * m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].robustweight
										   * m_staBaseLineList[b_i].weight_baseline; 
									double w = 1 / sqrt(0.5 / (w_ref_L * w_ref_L) + 0.5 / (w_L * w_L)); 
									// 确定观测权值
									if(fabs(w * o_c_DD_L_IF) > rms_oc_phase * m_podParaDefine.robustfactor_OC_edited)
										m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].robustweight = rms_oc_phase / (fabs(w * o_c_DD_L_IF) * sqrt(2.0));
									    //m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].robustweight = 0.0;
									else
										m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].robustweight = 1.0;
								}
							}
						}
					}
					//num_after_residual_edit++;
					//flag_robust = false;// 关闭编辑, 防止下次又重新进行编辑
				}
				if(flag_break)
				{// 输出定轨残差
					RuningInfoFile::Add("输出最终定轨残差==================");
					//FILE *pfile_C = fopen(codeOCPpath,"w+");
					FILE *pfile_P = fopen(phaseOCPpath,"w+");
					double rms_oc_code = 0;
					int count_validcode = 0;
					double rms_oc_phase = 0;
					int count_validphase = 0;
					// 输出定轨残差
					for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
					{
						m_staBaseLineList[b_i].ocResP_IFEpochList.clear();
						m_staBaseLineList[b_i].ocResL_IFEpochList.clear();
						m_staBaseLineList[b_i].rms_oc_code = 0;
						int count_validcode_i = 0;
						m_staBaseLineList[b_i].rms_oc_phase = 0;
						int count_validphase_i = 0;
						for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
						{
							// 计算伪码残差
							for(size_t s_i = 0; s_i < m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList.size(); s_i++)
							{  
								int count_obs = int( m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList.size());
								int nObsTime =  m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].nObsTime; 
								int id_DDRefSat =  m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].id_DDRefSat;     
								int id_DDObs =  m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].id_DDObs;
								O_CResEpoch ocResP_IFEpoch;
								ocResP_IFEpoch.ocResSatList.clear();
								ocResP_IFEpoch.t =  m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].t;
								for(int s_j = 0; s_j < count_obs; s_j++)
								{
									int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
									if(id_Sat == id_DDRefSat)
										continue;
									double o_c_DD_P_IF = (m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].obscorrected_value)
													   - (m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[id_DDObs].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].obscorrected_value);
									O_CResEpochElement ocResElement;
									ocResElement.id_Sat = id_Sat;
									ocResElement.res = o_c_DD_P_IF;
									ocResElement.robustweight = m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].robustweight;
									ocResElement.Elevation = m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].editedObs[id_Sat].Elevation_B;
									ocResElement.Azimuth = m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].editedObs[id_Sat].Azimuth_B;
									if(ocResElement.robustweight == 1.0 && m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].weightCode != 0)
									{
										ocResP_IFEpoch.ocResSatList.push_back(ocResElement);
										rms_oc_code += pow(o_c_DD_P_IF, 2);
										count_validcode++;
										m_staBaseLineList[b_i].rms_oc_code += pow(o_c_DD_P_IF, 2);
										count_validcode_i++;
										//fprintf(pfile_C,"%s %3d   %3d   %3d   %3d   %3d   %8.2lf %16.4lf\n",ocResP_IFEpoch.t.toString().c_str(),b_i,s_l,s_i,id_DDRefSat,id_Sat,ocResElement.Elevation,o_c_DD_P_IF);
									}
								}
								if(ocResP_IFEpoch.ocResSatList.size() > 0)
									m_staBaseLineList[b_i].ocResP_IFEpochList.push_back(ocResP_IFEpoch);
								
							}
							// 计算相位残差
							for(size_t s_i = 0; s_i < m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList.size(); s_i++)
							{  
								int count_obs = int(m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList.size());
								int nObsTime = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].nObsTime; 
								int id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDRefSat;   
								int id_DDRefAmbiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDRefAmbiguity;   
								int id_DDObs = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDObs;
								O_CResEpoch ocResL_IFEpoch;
								ocResL_IFEpoch.ocResSatList.clear();
								ocResL_IFEpoch.t = m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].t;
								for(int s_j = 0; s_j < count_obs; s_j++)
								{
									int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
									int id_Ambiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Ambiguity;
									if(id_Sat == id_DDRefSat)
										continue;
									double ambiguity_DD_L1 = 0;
									double ambiguity_DD_MW = 0;
									if(id_DDRefAmbiguity != 0)
									{
										ambiguity_DD_L1 -= m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[id_DDRefAmbiguity - 1] * BD_WAVELENGTH_N;
										ambiguity_DD_MW -= m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List[id_DDRefAmbiguity - 1] * coeff_mw;
									}
									if(id_Ambiguity != 0)
									{
										ambiguity_DD_L1 += m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[id_Ambiguity - 1] * BD_WAVELENGTH_N;
										ambiguity_DD_MW += m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List[id_Ambiguity - 1] * coeff_mw;
									}
									// 经过概略点改进后的观测值, 扣除双差模糊度的影响
									double o_c_DD_L_IF = (m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].obscorrected_value)
													   - (m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[id_DDObs].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].obscorrected_value)
													   -  ambiguity_DD_MW // 扣除宽巷模糊度的影响
													   -  ambiguity_DD_L1;
									O_CResEpochElement ocResElement;
									ocResElement.id_Sat = id_Sat;
									ocResElement.res = o_c_DD_L_IF;
									ocResElement.robustweight = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].robustweight;
									ocResElement.Elevation = m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].editedObs[id_Sat].Elevation_B;
									ocResElement.Azimuth = m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].editedObs[id_Sat].Azimuth_B;
									if(ocResElement.robustweight == 1.0 && m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].weightPhase != 0)
									{
										ocResL_IFEpoch.ocResSatList.push_back(ocResElement);
										rms_oc_phase += pow(o_c_DD_L_IF, 2);
										count_validphase++;
										m_staBaseLineList[b_i].rms_oc_phase += pow(o_c_DD_L_IF, 2);
										count_validphase_i++;
										fprintf(pfile_P,"%s %3d   %3d   %3d   %3d   %3d   %8.2lf %16.4lf\n",ocResL_IFEpoch.t.toString().c_str(),b_i,s_l,s_i,id_DDRefSat,id_Sat,ocResElement.Elevation,o_c_DD_L_IF);
									}
								}
								if(ocResL_IFEpoch.ocResSatList.size() > 0)
									m_staBaseLineList[b_i].ocResL_IFEpochList.push_back(ocResL_IFEpoch);
							}
						}
						m_staBaseLineList[b_i].rms_oc_code = sqrt(m_staBaseLineList[b_i].rms_oc_code / count_validcode_i);
						m_staBaseLineList[b_i].rms_oc_phase = sqrt(m_staBaseLineList[b_i].rms_oc_phase / count_validphase_i);
						sprintf(info, "%s %s 定轨残差, 伪码 =  %10.5f;  相位 =  %10.5f",   m_staBaseLineList[b_i].name_A.c_str(),
																						   m_staBaseLineList[b_i].name_B.c_str(),
																						   m_staBaseLineList[b_i].rms_oc_code,
																						   m_staBaseLineList[b_i].rms_oc_phase);
						RuningInfoFile::Add(info);
					}
					rms_oc_code = sqrt(rms_oc_code / count_validcode);
					rms_oc_phase = sqrt(rms_oc_phase / count_validphase);
					sprintf(info, "平均定轨残差, 伪码 =  %10.5f;  相位 =  %10.5f",rms_oc_code, rms_oc_phase);
					RuningInfoFile::Add(info);			
					printf("%s\n",info);
					//fclose(pfile_C);
					fclose(pfile_P);
					break;					
				}		
				k_num ++;
				//FILE *pfile_1 = fopen("C:\\ocresiduals_code_k.cpp","a+");
				//FILE *pfile_2 = fopen("C:\\ocresiduals_phase_k.cpp","a+");
				//FILE *pfile_tro = fopen("C:\\tro_fist_parameter.cpp","a+");
				for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
				{
					/*-----------------------------
						| n_xx   n_xb|     |nx|
						|            |   = |  |
						| n_bx   n_bb|     |nb|
						n_xx = H_x' * H_x
						nx   = H_x' * y
						n_bb = H_b' * H_b
						n_xb = H_x' * H_b
						nb   = H_b' * y
					-------------------------------*/
					
					for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
					{						
						// 伪码、相位双差联合方程
						m_staBaseLineList[b_i].amSectionList[s_l].n_xb.Init(count_DynParameter, m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed); // 窄巷模糊度固定后, 该值进行了更新
						m_staBaseLineList[b_i].amSectionList[s_l].N_bb.Init(m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed,  m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed);
						m_staBaseLineList[b_i].amSectionList[s_l].nb.Init(m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed, 1);
						Matrix matECEFPos_A(3, 1), matECEFPos_B(3, 1);										
						for(size_t s_i = 0; s_i < m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList.size(); s_i++)
						{ 							
							int count_obs = int(m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList.size());
							int nObsTime = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].nObsTime; 		
							// 考虑到潮汐修正, 2014/04/10, 谷德峰
							matECEFPos_A.SetElement(0, 0, m_staBaseLineList[b_i].staECEFPosList_A[nObsTime].x);
							matECEFPos_A.SetElement(1, 0, m_staBaseLineList[b_i].staECEFPosList_A[nObsTime].y);
							matECEFPos_A.SetElement(2, 0, m_staBaseLineList[b_i].staECEFPosList_A[nObsTime].z);
							matECEFPos_B.SetElement(0, 0, m_staBaseLineList[b_i].staECEFPosList_B[nObsTime].x);
							matECEFPos_B.SetElement(1, 0, m_staBaseLineList[b_i].staECEFPosList_B[nObsTime].y);
							matECEFPos_B.SetElement(2, 0, m_staBaseLineList[b_i].staECEFPosList_B[nObsTime].z);
							// 可以选择参考星, 暂时直接以第一颗卫星为参考卫星
							int id_DDRefSat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDRefSat;     
							int id_DDRefAmbiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDRefAmbiguity; // 参考模糊度序号
							int id_DDObs = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].id_DDObs;
							Matrix matEPECEFPos_A(3, 1), matEPECEFPos_B(3, 1);
							matEPECEFPos_A = m_staBaseLineList[b_i].matEPList[nObsTime].Transpose() * matECEFPos_A;
							matEPECEFPos_B = m_staBaseLineList[b_i].matEPList[nObsTime].Transpose() * matECEFPos_B;
							Matrix matM_ut1_A  = m_staBaseLineList[b_i].matERList_A[nObsTime] * m_staBaseLineList[b_i].matPR_NRList[nObsTime]; // 两个测站钟差不一致, matERList 有差别
							Matrix matM_ut1_B  = m_staBaseLineList[b_i].matERList_B[nObsTime] * m_staBaseLineList[b_i].matPR_NRList[nObsTime];
							Matrix matM_xpyp_A = m_staBaseLineList[b_i].matEPList[nObsTime] * matM_ut1_A; 
							Matrix matM_xpyp_B = m_staBaseLineList[b_i].matEPList[nObsTime] * matM_ut1_B; 
							matM_ut1_A  = matM_ut1_A.Transpose();
							matM_ut1_B  = matM_ut1_B.Transpose();
							matM_xpyp_A = matM_xpyp_A.Transpose();
							matM_xpyp_B = matM_xpyp_B.Transpose();
							double spanSeconds = m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime].t - paraSatOrbEst.t0_xpyput1;
							Matrix matLos_A(1, 3), matLos_B(1, 3);
							Matrix matHPA_xp(3, 1);
							Matrix matHPA_xpdot(3, 1);
							Matrix matHPA_yp(3, 1);
							Matrix matHPA_ypdot(3, 1);
							Matrix matHPA_ut1(3, 1);
							matHPA_xp.SetElement(0, 0, matM_xpyp_A.GetElement(0,2) * matECEFPos_A.GetElement(0, 0) - matM_xpyp_A.GetElement(0,0) * matECEFPos_A.GetElement(2, 0));
							matHPA_xp.SetElement(1, 0, matM_xpyp_A.GetElement(1,2) * matECEFPos_A.GetElement(0, 0) - matM_xpyp_A.GetElement(1,0) * matECEFPos_A.GetElement(2, 0));
							matHPA_xp.SetElement(2, 0, matM_xpyp_A.GetElement(2,2) * matECEFPos_A.GetElement(0, 0) - matM_xpyp_A.GetElement(2,0) * matECEFPos_A.GetElement(2, 0));
							matHPA_xpdot = matHPA_xp * spanSeconds;
							matHPA_yp.SetElement(0, 0, matM_xpyp_A.GetElement(0,1) * matECEFPos_A.GetElement(2, 0) - matM_xpyp_A.GetElement(0,2) * matECEFPos_A.GetElement(1, 0));
							matHPA_yp.SetElement(1, 0, matM_xpyp_A.GetElement(1,1) * matECEFPos_A.GetElement(2, 0) - matM_xpyp_A.GetElement(1,2) * matECEFPos_A.GetElement(1, 0));
							matHPA_yp.SetElement(2, 0, matM_xpyp_A.GetElement(2,1) * matECEFPos_A.GetElement(2, 0) - matM_xpyp_A.GetElement(2,2) * matECEFPos_A.GetElement(1, 0));
							matHPA_ypdot = matHPA_yp * spanSeconds;
							matHPA_ut1.SetElement(0, 0, matM_ut1_A.GetElement(0,1) * matEPECEFPos_A.GetElement(0, 0) - matM_ut1_A.GetElement(0,0) * matEPECEFPos_A.GetElement(1, 0));
							matHPA_ut1.SetElement(1, 0, matM_ut1_A.GetElement(1,1) * matEPECEFPos_A.GetElement(0, 0) - matM_ut1_A.GetElement(1,0) * matEPECEFPos_A.GetElement(1, 0));
							matHPA_ut1.SetElement(2, 0, matM_ut1_A.GetElement(2,1) * matEPECEFPos_A.GetElement(0, 0) - matM_ut1_A.GetElement(2,0) * matEPECEFPos_A.GetElement(1, 0));
							matHPA_ut1 = matHPA_ut1 * spanSeconds;
							Matrix matHPB_xp(3, 1);
							Matrix matHPB_xpdot(3, 1);
							Matrix matHPB_yp(3, 1);
							Matrix matHPB_ypdot(3, 1);
							Matrix matHPB_ut1(3, 1);
							matHPB_xp.SetElement(0, 0, matM_xpyp_B.GetElement(0,2) * matECEFPos_B.GetElement(0, 0) - matM_xpyp_B.GetElement(0,0) * matECEFPos_B.GetElement(2, 0));
							matHPB_xp.SetElement(1, 0, matM_xpyp_B.GetElement(1,2) * matECEFPos_B.GetElement(0, 0) - matM_xpyp_B.GetElement(1,0) * matECEFPos_B.GetElement(2, 0));
							matHPB_xp.SetElement(2, 0, matM_xpyp_B.GetElement(2,2) * matECEFPos_B.GetElement(0, 0) - matM_xpyp_B.GetElement(2,0) * matECEFPos_B.GetElement(2, 0));
							matHPB_xpdot = matHPB_xp * spanSeconds;
							matHPB_yp.SetElement(0, 0, matM_xpyp_B.GetElement(0,1) * matECEFPos_B.GetElement(2, 0) - matM_xpyp_B.GetElement(0,2) * matECEFPos_B.GetElement(1, 0));
							matHPB_yp.SetElement(1, 0, matM_xpyp_B.GetElement(1,1) * matECEFPos_B.GetElement(2, 0) - matM_xpyp_B.GetElement(1,2) * matECEFPos_B.GetElement(1, 0));
							matHPB_yp.SetElement(2, 0, matM_xpyp_B.GetElement(2,1) * matECEFPos_B.GetElement(2, 0) - matM_xpyp_B.GetElement(2,2) * matECEFPos_B.GetElement(1, 0));
							matHPB_ypdot = matHPB_yp * spanSeconds;
							matHPB_ut1.SetElement(0, 0, matM_ut1_B.GetElement(0,1) * matEPECEFPos_B.GetElement(0, 0) - matM_ut1_B.GetElement(0,0) * matEPECEFPos_B.GetElement(1, 0));
							matHPB_ut1.SetElement(1, 0, matM_ut1_B.GetElement(1,1) * matEPECEFPos_B.GetElement(0, 0) - matM_ut1_B.GetElement(1,0) * matEPECEFPos_B.GetElement(1, 0));
							matHPB_ut1.SetElement(2, 0, matM_ut1_B.GetElement(2,1) * matEPECEFPos_B.GetElement(0, 0) - matM_ut1_B.GetElement(2,0) * matEPECEFPos_B.GetElement(1, 0));
							matHPB_ut1 = matHPB_ut1 * spanSeconds;
							double w_ref_L = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].weightPhase
										   * m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[id_DDObs].robustweight
							               * m_staBaseLineList[b_i].weight_baseline; // 20140421, 谷德峰调整, 调整基线权重
							if(w_ref_L == 0)
								w_ref_L = zeroweight_phase;
							double w_ref_P = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].weightCode
										   * m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[id_DDObs].robustweight 
							               * m_staBaseLineList[b_i].weight_baseline; // 20140421, 谷德峰调整, 调整基线权重
							if(w_ref_P == 0)
								w_ref_P = zeroweight_code;
							Matrix matHX_i_A_j(count_obs - 1, 3);
							Matrix matHX_i_B_j(count_obs - 1, 3);
							Matrix matHX_i_A_r(1, 3);
							Matrix matHX_i_B_r(1, 3);
							Matrix matQP_i(count_obs - 1, count_obs - 1);
							Matrix matWP_i(count_obs - 1, count_obs - 1);
							Matrix matQL_i(count_obs - 1, count_obs - 1);
							Matrix matWL_i(count_obs - 1, count_obs - 1);
							Matrix matZP_i(count_obs - 1, 1);
							Matrix matZL_i(count_obs - 1, 1);
							Matrix matHd_i(count_obs - 1, count_DynParameter); // 动力学参数: 卫星轨道力学参数(15×N) + 测站参数[3+13] × M  + 地球旋转参数(5)
                            Matrix matHb_i(count_obs - 1, m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed); // 仅对未固定的模糊度进行求解
							vector<double> w_list_P, w_list_L;
							w_list_P.resize(count_obs - 1);
							w_list_L.resize(count_obs - 1);
							// 参考星只计算一次
							int index_r = paraSatOrbEst.satParaList[id_DDRefSat].index;
							POS3D EA_r =  m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].vecLos_A.getPos() * (-1.0); 
							POS3D EB_r =  m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].vecLos_B.getPos() * (-1.0); 
							matHX_i_A_r.SetElement(0, 0, EA_r.x);
							matHX_i_A_r.SetElement(0, 1, EA_r.y);
							matHX_i_A_r.SetElement(0, 2, EA_r.z);
							matHX_i_B_r.SetElement(0, 0, EB_r.x);
							matHX_i_B_r.SetElement(0, 1, EB_r.y);
							matHX_i_B_r.SetElement(0, 2, EB_r.z);
							// 1. 卫星轨道力学参数部分(count_dyn_eachSat×N)
							for(int s_k = 0; s_k < 6; s_k++)
							{// 当前参考卫星轨道对初始位置速度的偏导数
								double sum_posvel_A_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(0, s_k) * matHX_i_A_r.GetElement(0, 0) 
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(1, s_k) * matHX_i_A_r.GetElement(0, 1)
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(2, s_k) * matHX_i_A_r.GetElement(0, 2);
								double sum_posvel_B_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(0, s_k) * matHX_i_B_r.GetElement(0, 0) 
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(1, s_k) * matHX_i_B_r.GetElement(0, 1)
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(2, s_k) * matHX_i_B_r.GetElement(0, 2);
								if(s_k < 3)
									matHd_i.SetElement(0, s_k + index_r * count_dyn_eachSat,  sum_posvel_B_r - sum_posvel_A_r);
								else
									matHd_i.SetElement(0, s_k + index_r * count_dyn_eachSat, (sum_posvel_B_r - sum_posvel_A_r) * factor_vel);
							}
							if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_9PARA)
							{
								for(int s_k = 6; s_k < count_dyn_eachSat; s_k++)
								{// 当前参考卫星轨道对光压参数的偏导数
									double sum_solar_A_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(0, s_k) * matHX_i_A_r.GetElement(0, 0) 
														 + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(1, s_k) * matHX_i_A_r.GetElement(0, 1)
														 + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(2, s_k) * matHX_i_A_r.GetElement(0, 2);
									double sum_solar_B_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(0, s_k) * matHX_i_B_r.GetElement(0, 0) 
														 + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(1, s_k) * matHX_i_B_r.GetElement(0, 1)
														 + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(2, s_k) * matHX_i_B_r.GetElement(0, 2);
									matHd_i.SetElement(0, s_k + index_r * count_dyn_eachSat, (sum_solar_B_r - sum_solar_A_r) * factor_solar);
								}
							}
							else if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
							{								
								double sum_solar_A_r = 0;
								double sum_solar_B_r = 0;
								for(size_t s_k = 0; s_k < paraSatOrbEst.satParaList[id_DDRefSat].dynamicDatum_Est.solarPressureParaList.size(); s_k++)
								{
									sum_solar_A_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(0, 6 + 9 * (int)s_k + 0) * matHX_i_A_r.GetElement(0, 0) 
												  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(1, 6 + 9 * (int)s_k + 0) * matHX_i_A_r.GetElement(0, 1)
												  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(2, 6 + 9 * (int)s_k + 0) * matHX_i_A_r.GetElement(0, 2);
									sum_solar_B_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(0, 6 + 9 * (int)s_k + 0) * matHX_i_B_r.GetElement(0, 0) 
												  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(1, 6 + 9 * (int)s_k + 0) * matHX_i_B_r.GetElement(0, 1)
												  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(2, 6 + 9 * (int)s_k + 0) * matHX_i_B_r.GetElement(0, 2);
									matHd_i.SetElement(0, 6 + 5 * (int)s_k + 0 + index_r * count_dyn_eachSat, (sum_solar_B_r - sum_solar_A_r) * factor_solar);	

									sum_solar_A_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(0, 6 + 9 * (int)s_k + 3) * matHX_i_A_r.GetElement(0, 0) 
												  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(1, 6 + 9 * (int)s_k + 3) * matHX_i_A_r.GetElement(0, 1)
												  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(2, 6 + 9 * (int)s_k + 3) * matHX_i_A_r.GetElement(0, 2);
									sum_solar_B_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(0, 6 + 9 * (int)s_k + 3) * matHX_i_B_r.GetElement(0, 0) 
												  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(1, 6 + 9 * (int)s_k + 3) * matHX_i_B_r.GetElement(0, 1)
												  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(2, 6 + 9 * (int)s_k + 3) * matHX_i_B_r.GetElement(0, 2);
									matHd_i.SetElement(0, 6 + 5 * (int)s_k + 1 + index_r * count_dyn_eachSat, (sum_solar_B_r - sum_solar_A_r) * factor_solar);	
									for(int j = 2; j < 5; j++)
									{
										sum_solar_A_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(0, 6 + 9 * (int)s_k + j + 4) * matHX_i_A_r.GetElement(0, 0) 
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(1, 6 + 9 * (int)s_k + j + 4) * matHX_i_A_r.GetElement(0, 1)
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_A.GetElement(2, 6 + 9 * (int)s_k + j + 4) * matHX_i_A_r.GetElement(0, 2);
										sum_solar_B_r = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(0, 6 + 9 * (int)s_k + j + 4) * matHX_i_B_r.GetElement(0, 0) 
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(1, 6 + 9 * (int)s_k + j + 4) * matHX_i_B_r.GetElement(0, 1)
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].interpRtPartial_B.GetElement(2, 6 + 9 * (int)s_k + j + 4) * matHX_i_B_r.GetElement(0, 2);
										matHd_i.SetElement(0, 6 + 5 * (int)s_k + j + index_r * count_dyn_eachSat, (sum_solar_B_r - sum_solar_A_r) * factor_solar);										
									}								
								}//
							}
							int k_StaPara = count_dyn_eachSat * int(paraSatOrbEst.satParaList.size());
							//  2. 地球旋转参数部分(5)
							if(m_podParaDefine.bOnEst_ERP)
							{
								matLos_A.SetElement(0, 0, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].vecLos_A.x);
								matLos_A.SetElement(0, 1, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].vecLos_A.y);
								matLos_A.SetElement(0, 2, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].vecLos_A.z);
								matLos_B.SetElement(0, 0, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].vecLos_B.x);
								matLos_B.SetElement(0, 1, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].vecLos_B.y);
								matLos_B.SetElement(0, 2, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].vecLos_B.z);
								matHd_i.SetElement(0, k_StaPara + 0, ((matLos_B *    matHPB_xp).GetElement(0, 0) - (matLos_A *    matHPA_xp).GetElement(0, 0)) * factor_eop);
								matHd_i.SetElement(0, k_StaPara + 1, ((matLos_B * matHPB_xpdot).GetElement(0, 0) - (matLos_A * matHPA_xpdot).GetElement(0, 0)) * factor_eop);
								matHd_i.SetElement(0, k_StaPara + 2, ((matLos_B *    matHPB_yp).GetElement(0, 0) - (matLos_A *    matHPA_yp).GetElement(0, 0)) * factor_eop);
								matHd_i.SetElement(0, k_StaPara + 3, ((matLos_B * matHPB_ypdot).GetElement(0, 0) - (matLos_A * matHPA_ypdot).GetElement(0, 0)) * factor_eop);
								matHd_i.SetElement(0, k_StaPara + 4, ((matLos_B *   matHPB_ut1).GetElement(0, 0) - (matLos_A *   matHPA_ut1).GetElement(0, 0)) * factor_eop);
								k_StaPara = k_StaPara + 5;
							}
							//  3. 测站位置参数部分(3×M)
							if(m_podParaDefine.bOnEst_StaPos)
							{
								if(m_mapStaDatum[m_staBaseLineList[b_i].name_A].bOnEst_StaPos)
								{
									int index_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].indexEst_StaPos;
									matHd_i.SetElement(0, k_StaPara + 3 * index_A + 0,   EA_r.x);
								    matHd_i.SetElement(0, k_StaPara + 3 * index_A + 1,   EA_r.y);
									matHd_i.SetElement(0, k_StaPara + 3 * index_A + 2,   EA_r.z);
								}
								if(m_mapStaDatum[m_staBaseLineList[b_i].name_B].bOnEst_StaPos)
								{
									int index_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].indexEst_StaPos;
									matHd_i.SetElement(0, k_StaPara + 3 * index_B + 0, - EB_r.x);
									matHd_i.SetElement(0, k_StaPara + 3 * index_B + 1, - EB_r.y);
									matHd_i.SetElement(0, k_StaPara + 3 * index_B + 2, - EB_r.z);
								}
								k_StaPara = k_StaPara + count_StaParameter;	
							}
							//  4. 测站对流层参数部分
							int indexTZD_A = 0; //记录当前时刻测站A的对流层参数在整个估计参数列表中位置
							int indexTZD_B = 0; //记录当前时刻测站B的对流层参数在整个估计参数列表中位置							
							GPST t0_A,t1_A,t0_B,t1_B;
							Rinex2_1_EditedSdObsEpoch   epoch = m_staBaseLineList[b_i].editedSdObsFile.m_data[nObsTime];
							if(m_podParaDefine.bOnEst_StaTropZenithDelay)
							{								
								int index_A0 = m_mapStaDatum[m_staBaseLineList[b_i].name_A].zenithIndex_0;									
								int index_B0 = m_mapStaDatum[m_staBaseLineList[b_i].name_B].zenithIndex_0;
								int index_t0_A  = m_staBaseLineList[b_i].id_ZenithDelayList_A[nObsTime]; 
								int index_t0_B  = m_staBaseLineList[b_i].id_ZenithDelayList_B[nObsTime]; 
								t0_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].zenithDelayEstList[index_t0_A].t;
								t1_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].zenithDelayEstList[index_t0_A + 1].t;	
								t0_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].zenithDelayEstList[index_t0_B].t;
								t1_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].zenithDelayEstList[index_t0_B + 1].t;	
								indexTZD_A = index_A0 + index_t0_A;
								indexTZD_B = index_B0 + index_t0_B;
								matHd_i.SetElement(0, indexTZD_A,     -(t1_A - epoch.t) /(t1_A - t0_A)/sin(epoch.editedObs[id_DDRefSat].Elevation_A * PI/180));
								matHd_i.SetElement(0, indexTZD_A + 1, -(epoch.t - t0_A )/(t1_A - t0_A)/sin(epoch.editedObs[id_DDRefSat].Elevation_A * PI/180));
								matHd_i.SetElement(0, indexTZD_B,      (t1_B - epoch.t) /(t1_B - t0_B)/sin(epoch.editedObs[id_DDRefSat].Elevation_B * PI/180));
								matHd_i.SetElement(0, indexTZD_B + 1,  (epoch.t - t0_B) /(t1_B - t0_B)/sin(epoch.editedObs[id_DDRefSat].Elevation_B * PI/180));													
							}							
							for(int s_j = 1; s_j < count_obs - 1; s_j++)
							{
								k_StaPara = count_dyn_eachSat * int(paraSatOrbEst.satParaList.size());
								for(int s_k = 0; s_k < count_dyn_eachSat; s_k++)
									matHd_i.SetElement(s_j, s_k + index_r * count_dyn_eachSat, matHd_i.GetElement(0, s_k + index_r * count_dyn_eachSat));
								if(m_podParaDefine.bOnEst_ERP)
								{
									matHd_i.SetElement(s_j, k_StaPara + 0, matHd_i.GetElement(0, k_StaPara + 0));
									matHd_i.SetElement(s_j, k_StaPara + 1, matHd_i.GetElement(0, k_StaPara + 1));
									matHd_i.SetElement(s_j, k_StaPara + 2, matHd_i.GetElement(0, k_StaPara + 2));
									matHd_i.SetElement(s_j, k_StaPara + 3, matHd_i.GetElement(0, k_StaPara + 3));
									matHd_i.SetElement(s_j, k_StaPara + 4, matHd_i.GetElement(0, k_StaPara + 4));
									k_StaPara = k_StaPara + 5;
								}
								if(m_podParaDefine.bOnEst_StaPos)
								{
									if(m_mapStaDatum[m_staBaseLineList[b_i].name_A].bOnEst_StaPos)
									{
										int index_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].indexEst_StaPos;
										matHd_i.SetElement(s_j, k_StaPara + 3 * index_A + 0, matHd_i.GetElement(0, k_StaPara + 3 * index_A + 0));
										matHd_i.SetElement(s_j, k_StaPara + 3 * index_A + 1, matHd_i.GetElement(0, k_StaPara + 3 * index_A + 1));
										matHd_i.SetElement(s_j, k_StaPara + 3 * index_A + 2, matHd_i.GetElement(0, k_StaPara + 3 * index_A + 2));
									}
									if(m_mapStaDatum[m_staBaseLineList[b_i].name_B].bOnEst_StaPos)
									{
										int index_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].indexEst_StaPos;
										matHd_i.SetElement(s_j, k_StaPara + 3 * index_B + 0, matHd_i.GetElement(0, k_StaPara + 3 * index_B + 0));
										matHd_i.SetElement(s_j, k_StaPara + 3 * index_B + 1, matHd_i.GetElement(0, k_StaPara + 3 * index_B + 1));
										matHd_i.SetElement(s_j, k_StaPara + 3 * index_B + 2, matHd_i.GetElement(0, k_StaPara + 3 * index_B + 2));
									}
									k_StaPara = k_StaPara + count_StaParameter;	
								}
								if(m_podParaDefine.bOnEst_StaTropZenithDelay)
							    {
									matHd_i.SetElement(s_j, indexTZD_A,     matHd_i.GetElement(0, indexTZD_A));
									matHd_i.SetElement(s_j, indexTZD_A + 1, matHd_i.GetElement(0, indexTZD_A + 1));
									matHd_i.SetElement(s_j, indexTZD_B,     matHd_i.GetElement(0, indexTZD_B));
									matHd_i.SetElement(s_j, indexTZD_B + 1, matHd_i.GetElement(0, indexTZD_B + 1));
								}
							}
							int k = -1;
							for(int s_j = 0; s_j < count_obs; s_j++)
							{
								int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
								int id_Ambiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Ambiguity;
								if(id_Sat == id_DDRefSat)
									continue;
								k++;
								double w_P = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].weightCode
										   * m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].robustweight
										   * m_staBaseLineList[b_i].weight_baseline; // 20140421, 谷德峰调整, 调整基线权重
								if(w_P == 0)
									w_P = zeroweight_code;
								w_list_P[k] = w_P;
								double w_L = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].weightPhase 
										   * m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].robustweight
								           * m_staBaseLineList[b_i].weight_baseline; // 20140421, 谷德峰调整, 调整基线权重
								if(w_L == 0)
									w_L = zeroweight_phase;
								w_list_L[k] = w_L;
								// 双差消除了接收机钟差
								POS3D EA_j =  m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].vecLos_A.getPos() * (-1.0); 
								POS3D EB_j =  m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].vecLos_B.getPos() * (-1.0); 
								matHX_i_A_j.SetElement(k, 0, EA_j.x);
								matHX_i_A_j.SetElement(k, 1, EA_j.y);
								matHX_i_A_j.SetElement(k, 2, EA_j.z);
								matHX_i_B_j.SetElement(k, 0, EB_j.x);
								matHX_i_B_j.SetElement(k, 1, EB_j.y);
								matHX_i_B_j.SetElement(k, 2, EB_j.z);
								double ambiguity_DD_L1 = 0;
								double ambiguity_DD_MW = 0;
								if(id_DDRefAmbiguity != 0)
								{
									ambiguity_DD_L1 -= m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[id_DDRefAmbiguity - 1] * BD_WAVELENGTH_N;
									ambiguity_DD_MW -= m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List[id_DDRefAmbiguity - 1] * coeff_mw;
									int id_DD_L1_UnFixed = m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_DDRefAmbiguity - 1];
									if(id_DD_L1_UnFixed != -1) // 仅对未固定的模糊度进行改进
										matHb_i.SetElement(k, id_DD_L1_UnFixed, -BD_WAVELENGTH_N);
								}
								if(id_Ambiguity != 0)
								{
									ambiguity_DD_L1 += m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[id_Ambiguity - 1] * BD_WAVELENGTH_N;
									ambiguity_DD_MW += m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List[id_Ambiguity - 1] * coeff_mw;
									int id_DD_L1_UnFixed = m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_Ambiguity - 1];
									if(id_DD_L1_UnFixed != -1) // 仅对未固定的模糊度进行改进
										matHb_i.SetElement(k, id_DD_L1_UnFixed, BD_WAVELENGTH_N);
								}
								// 经过概略点改进后的观测值
								double o_c_DD_P_IF = (m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].obscorrected_value)
												   - (m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[id_DDObs].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].obscorrected_value);
								// 经过概略点改进后的观测值
								double o_c_DD_L_IF = (m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].obscorrected_value)
												   - (m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[id_DDObs].obs + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].obscorrected_value)
                                                   -  ambiguity_DD_MW  // 扣除宽巷模糊度的影响
												   -  ambiguity_DD_L1;
								matZP_i.SetElement(k, 0, o_c_DD_P_IF);
								matZL_i.SetElement(k, 0, o_c_DD_L_IF);
								//if(fabs(o_c_DD_P_IF) > 50)//增加鲁棒性控制,2013.7.7
								//{
								//	w_list_P[k] = 1.0e-8;
								//	m_staBaseLineList[b_i].amSectionList[s_l].P_IF_EpochList[s_i].obsSatList[s_j].robustweight = 0.0;
								//}
								//if(fabs(o_c_DD_L_IF) > 50)
								//{
								//	w_list_L[k] = 1.0e-8;
								//	m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].robustweight = 0.0;
								//}
								//printf("%16.4lf   %16.4lf\n",m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].obscorrected_value,
								//	                         m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_DDRefSat].obscorrected_value);
								//fprintf(pfile_1,"%s %2d %3d   %3d   %3d   %3d   %3d   %16.4lf\n",epoch.t.toString().c_str(),k_num,b_i,s_l,s_i,s_j,id_Sat,o_c_DD_P_IF);
								//fprintf(pfile_2,"%s %2d %3d   %3d   %3d   %3d   %3d   %16.4lf\n",epoch.t.toString().c_str(),k_num,b_i,s_l,s_i,s_j,id_Sat,o_c_DD_L_IF);
								// 1. 卫星轨道力学参数部分(15×N)
								int index_j = paraSatOrbEst.satParaList[id_Sat].index;
								for(int s_k = 0; s_k < 6; s_k++)
								{// 当前卫星 j 轨道对初始位置速度的偏导数
									double sum_posvel_A_j = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(0, s_k) * matHX_i_A_j.GetElement(k, 0) 
													      + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(1, s_k) * matHX_i_A_j.GetElement(k, 1)
													      + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(2, s_k) * matHX_i_A_j.GetElement(k, 2);
									double sum_posvel_B_j = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(0, s_k) * matHX_i_B_j.GetElement(k, 0) 
													      + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(1, s_k) * matHX_i_B_j.GetElement(k, 1)
													      + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(2, s_k) * matHX_i_B_j.GetElement(k, 2);
									if(s_k < 3)
										matHd_i.SetElement(k, s_k + index_j * count_dyn_eachSat ,  sum_posvel_A_j - sum_posvel_B_j);
									else
										matHd_i.SetElement(k, s_k + index_j * count_dyn_eachSat , (sum_posvel_A_j - sum_posvel_B_j) * factor_vel);
									//if(s_k < 3)//
									//	matHd_i.SetElement(k, s_k + index_j * count_dyn_eachSat , matHd_i.GetElement(k, s_k + index_j * count_dyn_eachSat) + sum_posvel_A_j - sum_posvel_B_j);
									//else
									//	matHd_i.SetElement(k, s_k + index_j * count_dyn_eachSat , matHd_i.GetElement(k, s_k + index_j * count_dyn_eachSat) + (sum_posvel_A_j - sum_posvel_B_j) * factor_vel);
								}
								if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_9PARA)
								{
									for(int s_k = 6; s_k < count_dyn_eachSat; s_k++)
									{// 当前卫星 j 轨道对光压参数的偏导数
										double sum_solar_A_j  = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(0, s_k) * matHX_i_A_j.GetElement(k, 0) 
															  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(1, s_k) * matHX_i_A_j.GetElement(k, 1)
															  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(2, s_k) * matHX_i_A_j.GetElement(k, 2);	
										double sum_solar_B_j  = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(0, s_k) * matHX_i_B_j.GetElement(k, 0) 
															  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(1, s_k) * matHX_i_B_j.GetElement(k, 1)
															  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(2, s_k) * matHX_i_B_j.GetElement(k, 2);	
										matHd_i.SetElement(k, s_k + index_j * count_dyn_eachSat, (sum_solar_A_j - sum_solar_B_j) * factor_solar);
										//matHd_i.SetElement(k, s_k + index_j * count_dyn_eachSat, matHd_i.GetElement(k, s_k + index_j * count_dyn_eachSat) + (sum_solar_A_j - sum_solar_B_j) * factor_solar);
									}
								}
								else if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
								{									
									double sum_solar_A_j = 0;
									double sum_solar_B_j = 0;
									for(size_t s_k = 0; s_k < paraSatOrbEst.satParaList[id_DDRefSat].dynamicDatum_Est.solarPressureParaList.size(); s_k++)
									{
										sum_solar_A_j = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(0, 6 + 9 * (int)s_k + 0) * matHX_i_A_j.GetElement(0, 0) 
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(1, 6 + 9 * (int)s_k + 0) * matHX_i_A_j.GetElement(0, 1)
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(2, 6 + 9 * (int)s_k + 0) * matHX_i_A_j.GetElement(0, 2);
										sum_solar_B_j = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(0, 6 + 9 * (int)s_k + 0) * matHX_i_B_j.GetElement(0, 0) 
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(1, 6 + 9 * (int)s_k + 0) * matHX_i_B_j.GetElement(0, 1)
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(2, 6 + 9 * (int)s_k + 0) * matHX_i_B_j.GetElement(0, 2);
										matHd_i.SetElement(k, 6 + 5 * (int)s_k + 0 + index_j * count_dyn_eachSat, (sum_solar_A_j - sum_solar_B_j) * factor_solar);	
										//matHd_i.SetElement(k, 6 + 5 * (int)s_k + 0 + index_r * count_dyn_eachSat, matHd_i.GetElement(k, 6 + 5 * (int)s_k + 0 + index_r * count_dyn_eachSat) + (sum_solar_A_j - sum_solar_B_j) * factor_solar);	

										sum_solar_A_j = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(0, 6 + 9 * (int)s_k + 3) * matHX_i_A_j.GetElement(0, 0) 
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(1, 6 + 9 * (int)s_k + 3) * matHX_i_A_j.GetElement(0, 1)
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(2, 6 + 9 * (int)s_k + 3) * matHX_i_A_j.GetElement(0, 2);
										sum_solar_B_j = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(0, 6 + 9 * (int)s_k + 3) * matHX_i_B_j.GetElement(0, 0) 
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(1, 6 + 9 * (int)s_k + 3) * matHX_i_B_j.GetElement(0, 1)
													  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(2, 6 + 9 * (int)s_k + 3) * matHX_i_B_j.GetElement(0, 2);
										matHd_i.SetElement(k, 6 + 5 * (int)s_k + 1 + index_j * count_dyn_eachSat, (sum_solar_A_j - sum_solar_B_j) * factor_solar);	
										//matHd_i.SetElement(k, 6 + 5 * (int)s_k + 1 + index_r * count_dyn_eachSat, matHd_i.GetElement(k, 6 + 5 * (int)s_k + 1 + index_r * count_dyn_eachSat) + (sum_solar_A_j - sum_solar_B_j) * factor_solar);	
										for(int j = 2; j < 5; j++)
										{
											sum_solar_A_j = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(0, 6 + 9 * (int)s_k + j + 4) * matHX_i_A_j.GetElement(0, 0) 
														  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(1, 6 + 9 * (int)s_k + j + 4) * matHX_i_A_j.GetElement(0, 1)
														  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_A.GetElement(2, 6 + 9 * (int)s_k + j + 4) * matHX_i_A_j.GetElement(0, 2);
											sum_solar_B_j = m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(0, 6 + 9 * (int)s_k + j + 4) * matHX_i_B_j.GetElement(0, 0) 
														  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(1, 6 + 9 * (int)s_k + j + 4) * matHX_i_B_j.GetElement(0, 1)
														  + m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].interpRtPartial_B.GetElement(2, 6 + 9 * (int)s_k + j + 4) * matHX_i_B_j.GetElement(0, 2);
											matHd_i.SetElement(k, 6 + 5 * (int)s_k + j + index_j * count_dyn_eachSat, (sum_solar_A_j - sum_solar_B_j) * factor_solar);		
											//matHd_i.SetElement(k, 6 + 5 * (int)s_k + j + index_r * count_dyn_eachSat, matHd_i.GetElement(k, 6 + 5 * (int)s_k + j + index_r * count_dyn_eachSat) + (sum_solar_A_j - sum_solar_B_j) * factor_solar);										
										}								
									}
								}
								int k_StaPara = count_dyn_eachSat * int(paraSatOrbEst.satParaList.size());
								//  2. 地球旋转参数部分(5)
								if(m_podParaDefine.bOnEst_ERP)
								{
									matLos_A.SetElement(0, 0, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].vecLos_A.x);
									matLos_A.SetElement(0, 1, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].vecLos_A.y);
									matLos_A.SetElement(0, 2, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].vecLos_A.z);
									matLos_B.SetElement(0, 0, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].vecLos_B.x);
									matLos_B.SetElement(0, 1, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].vecLos_B.y);
									matLos_B.SetElement(0, 2, m_staBaseLineList[b_i].dynEpochList[nObsTime].mapDatum[id_Sat].vecLos_B.z);
									matHd_i.SetElement(k, k_StaPara + 0, matHd_i.GetElement(k, k_StaPara + 0) + ((matLos_A *    matHPA_xp).GetElement(0, 0) - (matLos_B *    matHPB_xp).GetElement(0, 0)) * factor_eop);
									matHd_i.SetElement(k, k_StaPara + 1, matHd_i.GetElement(k, k_StaPara + 1) + ((matLos_A * matHPA_xpdot).GetElement(0, 0) - (matLos_B * matHPB_xpdot).GetElement(0, 0)) * factor_eop);
									matHd_i.SetElement(k, k_StaPara + 2, matHd_i.GetElement(k, k_StaPara + 2) + ((matLos_A *    matHPA_yp).GetElement(0, 0) - (matLos_B *    matHPB_yp).GetElement(0, 0)) * factor_eop);
									matHd_i.SetElement(k, k_StaPara + 3, matHd_i.GetElement(k, k_StaPara + 3) + ((matLos_A * matHPA_ypdot).GetElement(0, 0) - (matLos_B * matHPB_ypdot).GetElement(0, 0)) * factor_eop);
									matHd_i.SetElement(k, k_StaPara + 4, matHd_i.GetElement(k, k_StaPara + 4) + ((matLos_A *   matHPA_ut1).GetElement(0, 0) - (matLos_B *   matHPB_ut1).GetElement(0, 0)) * factor_eop);
									k_StaPara = k_StaPara + 5;
								}
								// 3. 测站位置参数部分[3] × M
								if(m_podParaDefine.bOnEst_StaPos)
								{
									// 根据测站在惯性系下的位置，计算东北天坐标系
									POS3D U_A,U_B;       // 垂直径向
									U_A =  m_staBaseLineList[b_i].staPosList_A[nObsTime];
									U_B =  m_staBaseLineList[b_i].staPosList_B[nObsTime];
									POS3D N_A,N_B;       // 北方向
									N_A.x = 0;
									N_A.y = 0;
									N_A.z = EARTH_R;     // 北极点
									N_B.x = 0;           // 2014/05/23, 谷德峰修改, N_A, N_B 不能混用
									N_B.y = 0;
									N_B.z = EARTH_R;     // 北极点	
									POS3D E_A,E_B;       // 东方向
									vectorCross(E_A,N_A,U_A);
									vectorCross(N_A,U_A,E_A);
									E_A = vectorNormal(E_A);
									N_A = vectorNormal(N_A);
									U_A = vectorNormal(U_A);
									vectorCross(E_B,N_B,U_B);
									vectorCross(N_B,U_B,E_B);
									E_B = vectorNormal(E_B);
									N_B = vectorNormal(N_B);
									U_B = vectorNormal(U_B);
									if(m_mapStaDatum[m_staBaseLineList[b_i].name_A].bOnEst_StaPos)
									{
										POS3D  sum_A;
										int index_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].indexEst_StaPos;
										sum_A.x = matHd_i.GetElement(k, k_StaPara + 3 * index_A + 0) - EA_j.x; 
										sum_A.y = matHd_i.GetElement(k, k_StaPara + 3 * index_A + 1) - EA_j.y; 
										sum_A.z = matHd_i.GetElement(k, k_StaPara + 3 * index_A + 2) - EA_j.z; 
										matHd_i.SetElement(k, k_StaPara + 3 * index_A + 0, vectorDot(sum_A,E_A));
										matHd_i.SetElement(k, k_StaPara + 3 * index_A + 1, vectorDot(sum_A,N_A));
										matHd_i.SetElement(k, k_StaPara + 3 * index_A + 2, vectorDot(sum_A,U_A));
									}
									if(m_mapStaDatum[m_staBaseLineList[b_i].name_B].bOnEst_StaPos)
									{
										POS3D  sum_B;
										int index_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].indexEst_StaPos;										
										sum_B.x = matHd_i.GetElement(k, k_StaPara + 3 * index_B + 0) + EB_j.x; 
										sum_B.y = matHd_i.GetElement(k, k_StaPara + 3 * index_B + 1) + EB_j.y; 
										sum_B.z = matHd_i.GetElement(k, k_StaPara + 3 * index_B + 2) + EB_j.z; 										
										matHd_i.SetElement(k, k_StaPara + 3 * index_B + 0, vectorDot(sum_B,E_B));
										matHd_i.SetElement(k, k_StaPara + 3 * index_B + 1, vectorDot(sum_B,N_B));
										matHd_i.SetElement(k, k_StaPara + 3 * index_B + 2, vectorDot(sum_B,U_B));
									}	
									k_StaPara = k_StaPara + count_StaParameter;	
								}
								// 4. 测站对流层参数部分
								if(m_podParaDefine.bOnEst_StaTropZenithDelay)
								{									
									matHd_i.SetElement(k, indexTZD_A,     matHd_i.GetElement(k, indexTZD_A)     + (t1_A - epoch.t)/(t1_A - t0_A)/sin(epoch.editedObs[id_Sat].Elevation_A * PI/180));
									matHd_i.SetElement(k, indexTZD_A + 1, matHd_i.GetElement(k, indexTZD_A + 1) + (epoch.t - t0_A )/(t1_A - t0_A)/sin(epoch.editedObs[id_Sat].Elevation_A * PI/180));
									matHd_i.SetElement(k, indexTZD_B,     matHd_i.GetElement(k, indexTZD_B)     - (t1_B - epoch.t)/(t1_B - t0_B)/sin(epoch.editedObs[id_Sat].Elevation_B * PI/180));
									matHd_i.SetElement(k, indexTZD_B + 1, matHd_i.GetElement(k, indexTZD_B + 1) - (epoch.t - t0_B )/(t1_B - t0_B)/sin(epoch.editedObs[id_Sat].Elevation_B * PI/180));													
								}
							}
							// 观测权矩阵, 为了适应以后的数据编辑, 需要对权值进行调整
							// 2009/09/04 对观测权值进行了调整, 权值 w_list->oo 时, 受 w_ref 限制, 但权值 w_list-> 0 时, 可以抑制野值影响
							for(int i = 0; i < count_obs - 1; i++)
							{
								for(int j = 0; j < count_obs - 1; j++)
								{
									if(i == j) // 对角线
									{
										matQP_i.SetElement(i, j, 2.0 / (w_ref_P * w_ref_P) + 2.0 /(w_list_P[j] * w_list_P[j]));
										matQL_i.SetElement(i, j, 2.0 / (w_ref_L * w_ref_L) + 2.0 /(w_list_L[j] * w_list_L[j]));
									}
									else       // 非对角线
									{
										matQP_i.SetElement(i, j, 2.0 / (w_ref_P * w_ref_P));
										matQL_i.SetElement(i, j, 2.0 / (w_ref_L * w_ref_L));
									}
								}
							}
							matWP_i = matQP_i.Inv();
							matWL_i = matQL_i.Inv();
							//Matrix matHd_W_i = matHd_i.Transpose() * matW_i; // [count_DynParameter, count_obs - 1]
							//Matrix matN_dd_i = matHd_W_i * matHd_i; // [count_DynParameter, count_DynParameter]
							//Matrix matnx_i = matHd_W_i * matZ_i; // [count_DynParameter, 1]
							//N_xx = N_xx + matN_dd_i;
							//nx   = nx   + matnx_i;
							// 设计矩阵中存在大量 0 元素, 直接计算效率较低
							// 1. 卫星轨道力学参数部分
							/*--------------------------------------------------  
							    matHd_i'     *      matW_i    =    matHd_W_i
							   s   0   0                           #   #   #
							   s   0   0                           #   #   #

							   0   s   0          w   w   w        #   #   #
							   0   s   0     *    w   w   w   =    #   #   #
                                                  w   w   w        
							   0   0   s                           #   #   #
							   0   0   s                           #   #   #

							   s   s   s                           @   @   @  
							   s   s   s                           @   @   @

							   e   e   e                           %   %   %   //地球旋转参数部分
							   e   e   e                           %   %   %

							   p   p   p                           *   *   *   //测站位置参数部分，每条基线涉及到两个测站
							   p   p   p                           *   *   *

							   p   p   p                           *   *   *   //测站位置参数部分
							   p   p   p                           *   *   *

							   T   T   T                           !   !   !   //测站对流层参数部分，每条基线涉及到两个测站
							   T   T   T                           !   !   !

							   T   T   T                           !   !   !
							   T   T   T                           !   !   !
                               ------------------------------------------------*/
							Matrix matHd_WP_i(count_DynParameter, count_obs - 1);
							Matrix matHd_WL_i(count_DynParameter, count_obs - 1);
							k_StaPara = count_dyn_eachSat * int(paraSatOrbEst.satParaList.size());
							int k_StaPara_E = k_StaPara;                       // 方便确定测站位置参数的的位置
							if(m_podParaDefine.bOnEst_ERP)
								k_StaPara_E += 5;		
							map<int, int> mapValidRow_Hd_W_i; // 并不是所有的 Hd_W_i 行均是有效行, 每个双差观测数据, 只有相应的卫星、观测站的参数行才非 0 
							mapValidRow_Hd_W_i.clear();       //2014/07/02
							for(int s_k = 0; s_k < count_DynParameter; s_k++)
							{// 计算matHd_W_i[s_k, k]
								int k = -1;
								int index_r = paraSatOrbEst.satParaList[id_DDRefSat].index;
								for(int s_j = 0; s_j < count_obs; s_j++)
								{
									int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
									if(id_Sat == id_DDRefSat)
									    continue;
									k++;
									int index_j = paraSatOrbEst.satParaList[id_Sat].index;
									// 对于非参考星, 设计矩阵块系数部分只有一列 k 为非0数据, 行区间为[index_j * count_dyn_eachSat, index_j * count_dyn_eachSat + count_dyn_eachSat)
									// 对于参考星,   设计矩阵块系数部分均非0, 行区间为[index_r * count_dyn_eachSat, index_r * count_dyn_eachSat + count_dyn_eachSat)
									// 对于测站A,    设计矩阵块(位置)系数部分均非0, 行区间为[k_StaPara_E + index_A * 3, k_StaPara_E + index_A * 3 + 3)
									// 对于测站B,    设计矩阵块(位置)系数部分均非0, 行区间为[k_StaPara_E + index_B * 3, k_StaPara_E + index_B * 3 + 3)
									// 对于测站A,    设计矩阵块(对流层)系数部分均非0, 行区间为[indexTZD_A, indexTZD_A + 1] 每个时刻的双差观测方程仅与两个对流层参数有关
									// 对于测站B,    设计矩阵块(对流层)系数部分均非0, 行区间为[indexTZD_B, indexTZD_B + 1]
									if((s_k >= index_j * count_dyn_eachSat && s_k < index_j * count_dyn_eachSat + count_dyn_eachSat)
									|| (s_k >= index_r * count_dyn_eachSat && s_k < index_r * count_dyn_eachSat + count_dyn_eachSat))
									{// 每颗卫星只计算跟自己相关的参考星系数部分, 避免重复, 再求和
										mapValidRow_Hd_W_i[s_k] = 1; // 记录 Hd_W_i 有效行
										for(int col_j = 0; col_j < count_obs - 1; col_j++)
										{
											matHd_WP_i.SetElement(s_k, col_j, matHd_WP_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWP_i.GetElement(k, col_j));
											matHd_WL_i.SetElement(s_k, col_j, matHd_WL_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWL_i.GetElement(k, col_j));
										}
									}
									if(m_podParaDefine.bOnEst_ERP)
									{
										if(s_k >= k_StaPara && s_k < k_StaPara + 5)
										{
											mapValidRow_Hd_W_i[s_k] = 1; // 记录 Hd_W_i 有效行
											for(int col_j = 0; col_j < count_obs - 1; col_j++)
											{
												matHd_WP_i.SetElement(s_k, col_j, matHd_WP_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWP_i.GetElement(k, col_j));
												matHd_WL_i.SetElement(s_k, col_j, matHd_WL_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWL_i.GetElement(k, col_j));
											}
										}
									}
									if(m_podParaDefine.bOnEst_StaPos)
									{										
										if(m_mapStaDatum[m_staBaseLineList[b_i].name_A].bOnEst_StaPos)
										{
											int index_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].indexEst_StaPos;
											if(s_k >= k_StaPara_E + index_A * 3 && s_k < k_StaPara_E + index_A * 3 + 3)	
											{
												mapValidRow_Hd_W_i[s_k] = 1; // 记录 Hd_W_i 有效行
												for(int col_j = 0; col_j < count_obs - 1; col_j++)
												{
													matHd_WP_i.SetElement(s_k, col_j, matHd_WP_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWP_i.GetElement(k, col_j));
													matHd_WL_i.SetElement(s_k, col_j, matHd_WL_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWL_i.GetElement(k, col_j));
												}//	
											}
										}
										if(m_mapStaDatum[m_staBaseLineList[b_i].name_B].bOnEst_StaPos)
										{
											int index_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].indexEst_StaPos;
											if(s_k >= k_StaPara_E + index_B * 3 && s_k < k_StaPara_E + index_B * 3 + 3)	
											{
												mapValidRow_Hd_W_i[s_k] = 1; // 记录 Hd_W_i 有效行
												for(int col_j = 0; col_j < count_obs - 1; col_j++)
												{
													matHd_WP_i.SetElement(s_k, col_j, matHd_WP_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWP_i.GetElement(k, col_j));
													matHd_WL_i.SetElement(s_k, col_j, matHd_WL_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWL_i.GetElement(k, col_j));
												}//	
											}
										}
									}									
									if(m_podParaDefine.bOnEst_StaTropZenithDelay)
									{
										if((s_k >= indexTZD_A && s_k <= indexTZD_A + 1)||(s_k >= indexTZD_B && s_k <= indexTZD_B + 1))
										{
											mapValidRow_Hd_W_i[s_k] = 1; // 记录 Hd_W_i 有效行
											for(int col_j = 0; col_j < count_obs - 1; col_j++)
											{
												matHd_WP_i.SetElement(s_k, col_j, matHd_WP_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWP_i.GetElement(k, col_j));
												matHd_WL_i.SetElement(s_k, col_j, matHd_WL_i.GetElement(s_k, col_j) + matHd_i.GetElement(k, s_k) * matWL_i.GetElement(k, col_j));
											}		
										}
									}									
								}
							}
							/*--------------------------------------------------
							   matHd_W_i     *     matHd_i
                               #   #   #           
							   #   #   #           
							                       
							   #   #   #
							   #   #   #           s s    0 0    0 0    s s  e e   p p   p p  ! !   ! !
												   0 0    s s    0 0    s s  e e   p p   p p  ! !   ! ! 
							   #   #   #		   0 0    0 0    s s    s s  e e   p p   p p  ! !   ! ! 
							   #   #   #										   
																				
							   @   @   @
							   @   @   @

                               %   %   %
							   %   %   %

							   *   *   *
							   *   *   *

							   *   *   *
							   *   *   *

							   !   !   !
							   !   !   !

							   !   !   !
							   !   !   !
							    matHd_W_i     *     matZ_i
                               #   #   #           
							   #   #   #           
							                       
							   #   #   #
							   #   #   #           z
												   z
							   #   #   #		   z
							   #   #   #
							   
							   @   @   @
							   @   @   @

							   %   %   %
							   %   %   %  

                               *   *   *
                               *   *   *

                               *   *   *
                               *   *   *

							   !   !   !
							   !   !   !

							   !   !   !
							   !   !   !									
							------------------------------------------------------*/
							//nx = nx + matHd_WP_i * matZP_i;
							//nx = nx + matHd_WL_i * matZL_i;
							for(map<int, int>::iterator it_line  = mapValidRow_Hd_W_i.begin(); it_line != mapValidRow_Hd_W_i.end(); ++it_line)
							{
								if(it_line->second == 1)
								{// 只计算有效行
									int s_k = it_line->first;
									for(int s_j = 0; s_j < count_obs - 1; s_j++)
									{
										nx.SetElement(s_k, 0, nx.GetElement(s_k, 0) + matHd_WP_i.GetElement(s_k, s_j) * matZP_i.GetElement(s_j, 0));
										nx.SetElement(s_k, 0, nx.GetElement(s_k, 0) + matHd_WL_i.GetElement(s_k, s_j) * matZL_i.GetElement(s_j, 0));
									}
									//计算 matN_dd_i[s_k, col_j]
									int k = -1;
									int index_r = paraSatOrbEst.satParaList[id_DDRefSat].index;
									for(int s_j = 0; s_j < count_obs; s_j++)
									{
										int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
										if(id_Sat == id_DDRefSat)
										{
											// 列序号[index_r * count_dyn_eachSat, index_r * count_dyn_eachSat + count_dyn_eachSat)
											for(int col_j = index_r * count_dyn_eachSat; col_j < index_r * count_dyn_eachSat + count_dyn_eachSat; col_j++)
											{
												double sum_P = 0;
												for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
													sum_P += matHd_WP_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
												double sum_L = 0;
												for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
													sum_L += matHd_WL_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
												N_xx.SetElement(s_k, col_j, N_xx.GetElement(s_k, col_j) + sum_P + sum_L);
											}
											// 列序号[k_StaPara, k_StaPara + 5)
											if(m_podParaDefine.bOnEst_ERP)
											{
												for(int col_j = k_StaPara; col_j < k_StaPara + 5; col_j++)
												{
													double sum_P = 0;
													for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
														sum_P += matHd_WP_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
													double sum_L = 0;
													for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
														sum_L += matHd_WL_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
													N_xx.SetElement(s_k, col_j, N_xx.GetElement(s_k, col_j) + sum_P + sum_L);
												}											
											}
											// 列序号[k_StaPara_E + index_A * 3, k_StaPara_E + index_A * 3 + 3)
											// 列序号[k_StaPara_E + index_B * 3, k_StaPara_E + index_B * 3 + 3)
											if(m_podParaDefine.bOnEst_StaPos)
											{
												if(m_mapStaDatum[m_staBaseLineList[b_i].name_A].bOnEst_StaPos)
												{
													int index_A = m_mapStaDatum[m_staBaseLineList[b_i].name_A].indexEst_StaPos;
													for(int col_j = k_StaPara_E + index_A * 3; col_j < k_StaPara_E + index_A * 3 + 3; col_j++)
													{
														double sum_P = 0;
														for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
															sum_P += matHd_WP_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
														double sum_L = 0;
														for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
															sum_L += matHd_WL_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
														N_xx.SetElement(s_k, col_j, N_xx.GetElement(s_k, col_j) + sum_P + sum_L);
													}
												}
												if(m_mapStaDatum[m_staBaseLineList[b_i].name_B].bOnEst_StaPos)
												{
													int index_B = m_mapStaDatum[m_staBaseLineList[b_i].name_B].indexEst_StaPos;
													for(int col_j = k_StaPara_E + index_B * 3; col_j < k_StaPara_E + index_B * 3 + 3; col_j++)
													{
														double sum_P = 0;
														for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
															sum_P += matHd_WP_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
														double sum_L = 0;
														for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
															sum_L += matHd_WL_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
														N_xx.SetElement(s_k, col_j, N_xx.GetElement(s_k, col_j) + sum_P + sum_L);
													}
												}
											}
											// 列序号[indexTZD_A, indexTZD_A + 1]
											// 列序号[indexTZD_B, indexTZD_B + 1]
											if(m_podParaDefine.bOnEst_StaTropZenithDelay)
											{
												for(int col_j = indexTZD_A; col_j <= indexTZD_A + 1; col_j++)
												{
													double sum_P = 0;
													for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
														sum_P += matHd_WP_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
													double sum_L = 0;
													for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
														sum_L += matHd_WL_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
													N_xx.SetElement(s_k, col_j, N_xx.GetElement(s_k, col_j) + sum_P + sum_L);
												}
												for(int col_j = indexTZD_B; col_j <= indexTZD_B + 1; col_j++)
												{
													double sum_P = 0;
													for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
														sum_P += matHd_WP_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
													double sum_L = 0;
													for(int sum_k = 0; sum_k < count_obs - 1; sum_k++)
														sum_L += matHd_WL_i.GetElement(s_k, sum_k) * matHd_i.GetElement(sum_k, col_j);
													N_xx.SetElement(s_k, col_j, N_xx.GetElement(s_k, col_j) + sum_P + sum_L);
												}
											}
											continue;
										}
										else
										{// 其它列
											k++;
											int index_j = paraSatOrbEst.satParaList[id_Sat].index;
											for(int col_j = index_j * count_dyn_eachSat; col_j < index_j * count_dyn_eachSat + count_dyn_eachSat; col_j++)
											{
												N_xx.SetElement(s_k, col_j, N_xx.GetElement(s_k, col_j)
																		  + matHd_WP_i.GetElement(s_k, k) * matHd_i.GetElement(k, col_j)
																		  + matHd_WL_i.GetElement(s_k, k) * matHd_i.GetElement(k, col_j));
											}
										}
									}
								}
							}													
							//nx = nx + matHd_WP_i * matZP_i;
							//nx = nx + matHd_WL_i * matZL_i;
							//// 增加对流层参数估计的绝对和相对约束，一个双差观测方程增加一组约束
							if(m_podParaDefine.bOnEst_StaTropZenithDelay)
							{								
								double weightTZD_abs = m_podParaDefine.apriorityRms_LIF / m_podParaDefine.apriorityRms_TZD_abs;
					            double weightTZD_rel_A = m_podParaDefine.apriorityRms_LIF / (m_podParaDefine.apriorityRms_TZD_rel * (t1_A - t0_A) / m_podParaDefine.apriorityWet_TZD_period);	
								double weightTZD_rel_B = m_podParaDefine.apriorityRms_LIF / (m_podParaDefine.apriorityRms_TZD_rel * (t1_B - t0_B) / m_podParaDefine.apriorityWet_TZD_period);	
								int  TroDDobsaccount = count_obs - 1;								
								for(int TZD_abs = indexTZD_A; TZD_abs <= indexTZD_A + 1;TZD_abs ++)
								{// 绝对约束									
									N_xx.SetElement(TZD_abs, TZD_abs, N_xx.GetElement(TZD_abs, TZD_abs) + weightTZD_abs * weightTZD_abs * TroDDobsaccount); 
									nx.SetElement(TZD_abs, 0, nx.GetElement(TZD_abs, 0) - weightTZD_abs * weightTZD_abs * (matdx_s.GetElement(TZD_abs,0) - m_podParaDefine.apriorityWet_TZD) * TroDDobsaccount);									
									for(int TZD_rel = indexTZD_A; TZD_rel <= indexTZD_A + 1;TZD_rel ++)
									{// 相对对约束
										if(TZD_abs == TZD_rel)
										{
											N_xx.SetElement(TZD_abs, TZD_rel, N_xx.GetElement(TZD_abs, TZD_rel) + weightTZD_rel_A * weightTZD_rel_A * TroDDobsaccount); 
											if(TZD_rel == indexTZD_A)//
												nx.SetElement(TZD_rel, 0, nx.GetElement(TZD_rel, 0) - weightTZD_rel_A * weightTZD_rel_A * (matdx_s.GetElement(TZD_rel,0) - matdx_s.GetElement(TZD_rel + 1,0)) * TroDDobsaccount);	
											else
												nx.SetElement(TZD_rel, 0, nx.GetElement(TZD_rel, 0) - weightTZD_rel_A * weightTZD_rel_A * (matdx_s.GetElement(TZD_rel,0) - matdx_s.GetElement(TZD_rel - 1,0)) * TroDDobsaccount);	
										}
										else
											N_xx.SetElement(TZD_abs, TZD_rel, N_xx.GetElement(TZD_abs, TZD_rel) - weightTZD_rel_A * weightTZD_rel_A * TroDDobsaccount); 
									}
								}
								for(int TZD_abs = indexTZD_B; TZD_abs <= indexTZD_B + 1;TZD_abs ++)
								{// 绝对约束
									N_xx.SetElement(TZD_abs, TZD_abs, N_xx.GetElement(TZD_abs, TZD_abs) + weightTZD_abs * weightTZD_abs * TroDDobsaccount); 
									nx.SetElement(TZD_abs, 0, nx.GetElement(TZD_abs, 0) - weightTZD_abs * weightTZD_abs * (matdx_s.GetElement(TZD_abs,0) - m_podParaDefine.apriorityWet_TZD) * TroDDobsaccount);
									for(int TZD_rel = indexTZD_B; TZD_rel <= indexTZD_B + 1;TZD_rel ++)
									{// 相对对约束(考虑与区间的时间长短有关系,2014/05/13)
										if(TZD_abs == TZD_rel)
										{
											N_xx.SetElement(TZD_abs, TZD_rel, N_xx.GetElement(TZD_abs, TZD_rel) + weightTZD_rel_B * weightTZD_rel_B * TroDDobsaccount); 
											if(TZD_rel == indexTZD_B)//
												nx.SetElement(TZD_rel, 0, nx.GetElement(TZD_rel, 0) - weightTZD_rel_B * weightTZD_rel_B * (matdx_s.GetElement(TZD_rel,0) - matdx_s.GetElement(TZD_rel + 1,0)) * TroDDobsaccount);	
											else
												nx.SetElement(TZD_rel, 0, nx.GetElement(TZD_rel, 0) - weightTZD_rel_B * weightTZD_rel_B * (matdx_s.GetElement(TZD_rel,0) - matdx_s.GetElement(TZD_rel - 1,0)) * TroDDobsaccount);
										}
										else
											N_xx.SetElement(TZD_abs, TZD_rel, N_xx.GetElement(TZD_abs, TZD_rel) - weightTZD_rel_B * weightTZD_rel_B * TroDDobsaccount); 
									}
								}		
					
							}
							// matN_bb_i 和 matnb_i 的求解可提高效率
							// matHb_i 每行对应一个双差观测数据, 至多 2 个模糊度列系数非 0, 当部分模糊度被固定后, 可能很多时刻模糊度系数均为 0
							//Matrix matHb_W_i = matHb_i.Transpose() * matWL_i; // [count_DD_L1_UnFixed, count_obs - 1]
							/*--------------------------------------------------  
							    matHb_i'     *       matWL_i      =   matHb_W_i
								0   0   0                             0   0   0
								b   0   0                             #   #   #
                                0   0   0                             0   0   0
								0   0   0            w   w   w        0   0   0
								0   0   0        *   w   w   w    =   0   0   0
								0	0	0			 w   w   w        0   0   0    
								0   0   0                             0   0   0
								0   0   0                             0   0   0
                            ---------------------------------------------------*/
							Matrix matHb_W_i(m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed, count_obs - 1);
							k = -1;
							map<int, int> mapValidRow_Hb_W_i;
							mapValidRow_Hb_W_i.clear();
							for(int s_j = 0; s_j < count_obs; s_j++)
							{
								int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
								int id_Ambiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Ambiguity;
								if(id_Sat == id_DDRefSat)
									continue;
								k++;
								if(id_DDRefAmbiguity != 0)
								{
									int id_DD_L1_UnFixed = m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_DDRefAmbiguity - 1];
									if(id_DD_L1_UnFixed != -1) // 仅对未固定的模糊度进行改进
									{// matHb_i'[id_DD_L1_UnFixed, k] 非 0
										mapValidRow_Hb_W_i[id_DD_L1_UnFixed] = 1;
										for(int col_j = 0; col_j < count_obs - 1; col_j++)
											matHb_W_i.SetElement(id_DD_L1_UnFixed, col_j, matHb_W_i.GetElement(id_DD_L1_UnFixed, col_j) + matHb_i.GetElement(k, id_DD_L1_UnFixed) * matWL_i.GetElement(k, col_j));
									}
								}
								if(id_Ambiguity != 0)
								{
									int id_DD_L1_UnFixed = m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_Ambiguity - 1];
									if(id_DD_L1_UnFixed != -1) // 仅对未固定的模糊度进行改进
									{
										mapValidRow_Hb_W_i[id_DD_L1_UnFixed] = 1;
										for(int col_j = 0; col_j < count_obs - 1; col_j++)
											matHb_W_i.SetElement(id_DD_L1_UnFixed, col_j, matHb_W_i.GetElement(id_DD_L1_UnFixed, col_j) + matHb_i.GetElement(k, id_DD_L1_UnFixed) * matWL_i.GetElement(k, col_j));
									}
								}
							}
							//Matrix matN_bb_i = matHb_W_i * matHb_i; // [count_DD_L1_UnFixed, count_DD_L1_UnFixed]
							//Matrix matnb_i = matHb_W_i * matZL_i; // [count_DD_L1_UnFixed, 1]
							//Matrix matN_db_i = matHd_WL_i * matHb_i;
							/*-------------------------------------------------------------------------------------  
							    matHb_W_i     *      matHb_i                    =    matN_bb_i
								0   0   0                                            0  0  0  0  0  0  0  0
								#   #   #                                            0  #  0  0  0  0  0  0
                                0   0   0            0  b  0  0  0  0  0  0          0  0  0  0  0  0  0  0
								0   0   0        *   0  0  0  0  0  0  0  0          0  0  0  0  0  0  0  0
								0   0   0            0  0  0  0  0  0  0  0          0  0  0  0  0  0  0  0
								0   0   0			                                 0  0  0  0  0  0  0  0
								0   0   0                                            0  0  0  0  0  0  0  0
								0   0   0                                            0  0  0  0  0  0  0  0

								matHd_WL_i     *      matHb_i                   =    matN_db_i
								#   #   #                                            0  #  0  0  0  0  0  0
                                #   #   #            0  b  0  0  0  0  0  0          0  #  0  0  0  0  0  0
								#   #   #        *   0  0  0  0  0  0  0  0          0  #  0  0  0  0  0  0
								#   #   #            0  0  0  0  0  0  0  0          0  #  0  0  0  0  0  0
								#   #   #			                                 0  #  0  0  0  0  0  0

								matHb_W_i     *      matZL_i                    =    matnb_i
								0   0   0                                            0 
								#   #   #                                            # 
                                0   0   0            z                               0 
								0   0   0        *   z                               0 
								0   0   0            z                               0 
								0   0   0			                                 0 
								0   0   0                                            0 
								0   0   0                                            0 
                            --------------------------------------------------------------------------------------*/
							//Matrix matnb_i(m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed, 1);
							for(int s_j = 0; s_j < count_obs -1; s_j++)
							{
								for(map<int, int>::iterator it_line  = mapValidRow_Hb_W_i.begin(); it_line != mapValidRow_Hb_W_i.end(); ++it_line)
								{
									if(it_line->second == 1)
									{
										int row = it_line->first;
										m_staBaseLineList[b_i].amSectionList[s_l].nb.SetElement(row, 0, 
										m_staBaseLineList[b_i].amSectionList[s_l].nb.GetElement(row, 0) + matHb_W_i.GetElement(row, s_j) * matZL_i.GetElement(s_j, 0));
									}
								}
							}
							//Matrix matN_bb_i(m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed, m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed);
							//Matrix matN_db_i(count_DynParameter, m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed);
							k = -1;
							for(int s_j = 0; s_j < count_obs; s_j++)
							{
								int id_Sat = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Sat; 
								int id_Ambiguity = m_staBaseLineList[b_i].amSectionList[s_l].L_IF_EpochList[s_i].obsSatList[s_j].id_Ambiguity;
								if(id_Sat == id_DDRefSat)
									continue;
								k++;
								if(id_DDRefAmbiguity != 0)
								{
									int id_DD_L1_UnFixed = m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_DDRefAmbiguity - 1];
									if(id_DD_L1_UnFixed != -1) // 仅对未固定的模糊度进行改进
									{// matHb_i[k, id_DD_L1_UnFixed] 非 0
										for(map<int, int>::iterator it_line  = mapValidRow_Hb_W_i.begin(); it_line != mapValidRow_Hb_W_i.end(); ++it_line)
										{
											if(it_line->second == 1)
											{
												int row = it_line->first;
												m_staBaseLineList[b_i].amSectionList[s_l].N_bb.SetElement(row, id_DD_L1_UnFixed, 
											    m_staBaseLineList[b_i].amSectionList[s_l].N_bb.GetElement(row, id_DD_L1_UnFixed) + matHb_W_i.GetElement(row, k) * matHb_i.GetElement(k, id_DD_L1_UnFixed));
											}
										}
										for(int i_row = 0; i_row < count_DynParameter; i_row++)
										{
											m_staBaseLineList[b_i].amSectionList[s_l].n_xb.SetElement(i_row, id_DD_L1_UnFixed, 
											m_staBaseLineList[b_i].amSectionList[s_l].n_xb.GetElement(i_row, id_DD_L1_UnFixed) + matHd_WL_i.GetElement(i_row, k) * matHb_i.GetElement(k, id_DD_L1_UnFixed));
										}
									}
								}
								if(id_Ambiguity != 0)
								{
									int id_DD_L1_UnFixed = m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_Ambiguity - 1];
									if(id_DD_L1_UnFixed != -1) // 仅对未固定的模糊度进行改进
									{// matHb_i[k, id_DD_L1_UnFixed] 非 0
										for(map<int, int>::iterator it_line  = mapValidRow_Hb_W_i.begin(); it_line != mapValidRow_Hb_W_i.end(); ++it_line)
										{
											if(it_line->second == 1)
											{
												int row = it_line->first;
												m_staBaseLineList[b_i].amSectionList[s_l].N_bb.SetElement(row, id_DD_L1_UnFixed, 
											    m_staBaseLineList[b_i].amSectionList[s_l].N_bb.GetElement(row, id_DD_L1_UnFixed) + matHb_W_i.GetElement(row, k) * matHb_i.GetElement(k, id_DD_L1_UnFixed));
											}
										}
										for(int i_row = 0; i_row < count_DynParameter; i_row++)
										{
											m_staBaseLineList[b_i].amSectionList[s_l].n_xb.SetElement(i_row, id_DD_L1_UnFixed, 
											m_staBaseLineList[b_i].amSectionList[s_l].n_xb.GetElement(i_row, id_DD_L1_UnFixed) + matHd_WL_i.GetElement(i_row, k) * matHb_i.GetElement(k, id_DD_L1_UnFixed));
										}
									}
								}
							}
							//// matN_bb_i 和 matnb_i 的求解可提高效率
							//Matrix matHb_W_i = matHb_i.Transpose() * matWL_i; // [count_DD_MW_Fixed, count_obs - 1]
							//Matrix matN_bb_i = matHb_W_i * matHb_i; // [count_DD_MW_Fixed, count_DD_MW_Fixed]
							//Matrix matnb_i   = matHb_W_i * matZL_i; // [count_DD_MW_Fixed, 1]							
							//m_staBaseLineList[b_i].amSectionList[s_l].n_xb = m_staBaseLineList[b_i].amSectionList[s_l].n_xb + matHd_WL_i * matHb_i;
							//m_staBaseLineList[b_i].amSectionList[s_l].N_bb = m_staBaseLineList[b_i].amSectionList[s_l].N_bb + matN_bb_i;
							//m_staBaseLineList[b_i].amSectionList[s_l].nb   = m_staBaseLineList[b_i].amSectionList[s_l].nb   + matnb_i;
						}
				    }
					// 当所有弧段更新完毕后, 统一计算每组模糊度浮点解改进值
					int k_sub = 0;
					for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
					{
						for(int i = 0; i < m_staBaseLineList[b_i].amSectionList[s_l].N_bb.GetNumRows(); i++)
						{
							for(int j = 0; j < m_staBaseLineList[b_i].amSectionList[s_l].N_bb.GetNumColumns(); j++)
								m_staBaseLineList[b_i].N_bb.SetElement(k_sub + i, k_sub + j, m_staBaseLineList[b_i].amSectionList[s_l].N_bb.GetElement(i, j));
						}
						for(int i = 0; i < count_DynParameter; i++)
						{
							for(int j = 0; j < m_staBaseLineList[b_i].amSectionList[s_l].n_xb.GetNumColumns(); j++)
								m_staBaseLineList[b_i].n_xb.SetElement(i, k_sub + j, m_staBaseLineList[b_i].amSectionList[s_l].n_xb.GetElement(i, j));
						}
						for(int i = 0; i < m_staBaseLineList[b_i].amSectionList[s_l].nb.GetNumRows(); i++)
							m_staBaseLineList[b_i].nb.SetElement(k_sub + i, 0, m_staBaseLineList[b_i].amSectionList[s_l].nb.GetElement(i, 0));
						k_sub += int(m_staBaseLineList[b_i].amSectionList[s_l].N_bb.GetNumRows());
					}					
				}
				//fclose(pfile_1);
				//fclose(pfile_2);
				//fclose(pfile_PCO);
				//fclose(pfile_e);
				//fclose(pfile_gr);
				//fclose(pfile_tro);
				//fclose(pfile_sc);
				// 增加测站位置约束方程
				if(m_podParaDefine.bOnEst_StaPos)
				{
					double weightSta   = 0;
					int    k_dynPara   = count_dyn_eachSat * int(paraSatOrbEst.satParaList.size());
					Matrix transCond(3,1);  // 用于增加区域网的测站非平移约束
					if(m_podParaDefine.bOnEst_ERP)
						k_dynPara += 5;
					for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)	
					{
						if(it->second.bOnEst_StaPos)
						{							
							//if(it->second.indexEst_StaPos != 2)
								//weightSta   = m_podParaDefine.apriorityRms_LIF / m_podParaDefine.apriorityRms_STA;
								weightSta   = m_podParaDefine.apriorityRms_LIF / it->second.sigma;
							//else
							//	weightSta   = m_podParaDefine.apriorityRms_LIF / 1.0e-3;
							for(int s_j = 0; s_j < 3;s_j++)
							{
								N_xx.SetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j, k_dynPara + it->second.indexEst_StaPos * 3 + s_j,
								N_xx.GetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j, k_dynPara + it->second.indexEst_StaPos * 3 + s_j) + weightSta * weightSta * it->second.count_obs_dd * 2);	
								nx.SetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j, 0,
								nx.GetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j, 0) - weightSta * weightSta * matdx_s.GetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j,0) * it->second.count_obs_dd * 2);
								transCond.SetElement(s_j, 0, transCond.GetElement(s_j, 0) + matdx_s.GetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j,0));
							}
							
						}
					}
					//// 增加区域网测站非平移约束
					//for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)	
					//{
					//	if(it->second.bOnEst_StaPos)
					//	{							
					//	    weightSta   = m_podParaDefine.apriorityRms_LIF / 1.0e-4;	
					//		for(int s_j = 0; s_j < 3;s_j++)
					//		{
					//			N_xx.SetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j, k_dynPara + it->second.indexEst_StaPos * 3 + s_j,
					//			N_xx.GetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j, k_dynPara + it->second.indexEst_StaPos * 3 + s_j) + weightSta * weightSta);	
					//			nx.SetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j, 0,
					//			nx.GetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j, 0) - weightSta * weightSta * (transCond.GetElement(s_j, 0)- matdx_s.GetElement(k_dynPara + it->second.indexEst_StaPos * 3 + s_j, 0)));
					//		}

					//	}
					//}
				}		
				// 先计算轨道改进量, 后计算模糊度改进量, 以避免出现高维矩阵 N_bb
				n_xx_inv = N_xx.Inv_Ssgj();
				Matrix n_xb_bb_inv_bx(count_DynParameter, count_DynParameter);
				Matrix nx_new = nx; // 20140412, 谷德峰修改, 下面还将用到 nx, 此处用nx_new来记录 nx - n_xb_bb_inv * nb
				for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
				{
					Matrix n_bx = m_staBaseLineList[b_i].n_xb.Transpose();
					Matrix n_xb_bb_inv = m_staBaseLineList[b_i].n_xb * m_staBaseLineList[b_i].N_bb.Inv_Ssgj(); 
					n_xb_bb_inv_bx = n_xb_bb_inv_bx + n_xb_bb_inv * n_bx;  
					nx_new = nx_new - n_xb_bb_inv * m_staBaseLineList[b_i].nb; 
				}

				Matrix matQ_xx = (N_xx - n_xb_bb_inv_bx).Inv_Ssgj();
                matdx = matQ_xx * nx_new;
				//n_xx_inv = N_xx.Inv_Ssgj();
				//// 先计算轨道改进量, 后计算模糊度改进量
				//Matrix n_xb_bb_inv_bx(count_DynParameter, count_DynParameter);
				//for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
				//{
				//	Matrix n_bx = m_staBaseLineList[b_i].n_xb.Transpose();
				//	Matrix n_xb_bb_inv = m_staBaseLineList[b_i].n_xb * m_staBaseLineList[b_i].N_bb.Inv_Ssgj(); 
				//	n_xb_bb_inv_bx = n_xb_bb_inv_bx + n_xb_bb_inv * n_bx;  
				//	nx = nx - n_xb_bb_inv * m_staBaseLineList[b_i].nb;  
				//	//m_staBaseLineList[b_i].matQ_dd_L1 = (m_staBaseLineList[b_i].N_bb - n_bx * n_xx_inv * m_staBaseLineList[b_i].n_xb).Inv();
				//}
				//FILE *pN_xx = fopen("C:N_xx.cpp","w+");
				//for(int i = 0; i < N_xx.GetNumRows(); i ++)
				//{
				//	for(int j = 0; j < N_xx.GetNumColumns(); j ++)
				//		fprintf(pN_xx,"%16.4lf  ",N_xx.GetElement(i,j));
				//	fprintf(pN_xx,"\n");
				//}
				//fclose(pN_xx);
				//printf("N_xx 矩阵输出完毕！\n");//
				//Matrix matQ_xx = (N_xx - n_xb_bb_inv_bx).Inv_Ssgj();
                //matdx = matQ_xx * nx;
				matdx_s = matdx_s + matdx; 
				for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
				{
					Matrix n_bb_inv = m_staBaseLineList[b_i].N_bb.Inv_Ssgj();
					m_staBaseLineList[b_i].matdb = n_bb_inv * (m_staBaseLineList[b_i].nb - m_staBaseLineList[b_i].n_xb.Transpose() * matdx);
					Matrix n_bx = m_staBaseLineList[b_i].n_xb.Transpose();
					Matrix n_bb_inv_bx = n_bb_inv * n_bx;
					m_staBaseLineList[b_i].matQ_dd_L1 = n_bb_inv + n_bb_inv_bx * matQ_xx * n_bb_inv_bx.Transpose();
					// 计算模糊度改进量
					int k_sub = 0;
					for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
					{
						for(size_t s_i = 0; s_i < m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list.size(); s_i++)
						{ 
							int id_DD_L1_UnFixed = m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[s_i];
							if(id_DD_L1_UnFixed != -1)
								m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[s_i] += m_staBaseLineList[b_i].matdb.GetElement(k_sub + id_DD_L1_UnFixed, 0);
						}
						k_sub += int(m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed);
					}
				}
				// 计算轨道改进量
				double max_adjust_pos = 0;// 判断收敛条件
				for(SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); it != paraSatOrbEst.satParaList.end(); ++it)
				{
					for(int ii = 0; ii < 3; ii++)
						max_adjust_pos = max(max_adjust_pos, fabs(matdx.GetElement(it->second.index * count_dyn_eachSat + ii, 0)));
					it->second.dynamicDatum_Est.X0.x  += matdx.GetElement(it->second.index * count_dyn_eachSat + 0, 0);
					it->second.dynamicDatum_Est.X0.y  += matdx.GetElement(it->second.index * count_dyn_eachSat + 1, 0);
					it->second.dynamicDatum_Est.X0.z  += matdx.GetElement(it->second.index * count_dyn_eachSat + 2, 0);
					it->second.dynamicDatum_Est.X0.vx += matdx.GetElement(it->second.index * count_dyn_eachSat + 3, 0) * factor_vel;
					it->second.dynamicDatum_Est.X0.vy += matdx.GetElement(it->second.index * count_dyn_eachSat + 4, 0) * factor_vel;
					it->second.dynamicDatum_Est.X0.vz += matdx.GetElement(it->second.index * count_dyn_eachSat + 5, 0) * factor_vel;
					for(size_t s_kk = 0; s_kk < it->second.dynamicDatum_Est.solarPressureParaList.size(); s_kk++)
				    {// 考虑多组太阳光压参数, 20131209, 谷德峰
						if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_9PARA)
						{
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].D0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  6 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].DC1 += matdx.GetElement(it->second.index * count_dyn_eachSat +  7 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].DS1 += matdx.GetElement(it->second.index * count_dyn_eachSat +  8 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].Y0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  9 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].YC1 += matdx.GetElement(it->second.index * count_dyn_eachSat + 10 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].YS1 += matdx.GetElement(it->second.index * count_dyn_eachSat + 11 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].B0 += matdx.GetElement(it->second.index * count_dyn_eachSat + 12 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].BC1 += matdx.GetElement(it->second.index * count_dyn_eachSat + 13 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].BS1 += matdx.GetElement(it->second.index * count_dyn_eachSat + 14 + int(s_kk) * count_solar_period, 0) * factor_solar;
						}
						else if (m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
						{
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].D0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  6 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].Y0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  7 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].B0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  8 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].BC1 += matdx.GetElement(it->second.index * count_dyn_eachSat +  9 + int(s_kk) * count_solar_period, 0) * factor_solar;
							it->second.dynamicDatum_Est.solarPressureParaList[s_kk].BS1 += matdx.GetElement(it->second.index * count_dyn_eachSat + 10 + int(s_kk) * count_solar_period, 0) * factor_solar;
						}
					}					
				}
				int k_StaPara = count_dyn_eachSat * int(paraSatOrbEst.satParaList.size());
				if(m_podParaDefine.bOnEst_ERP)
				{
					paraSatOrbEst.xp     += matdx.GetElement(k_StaPara + 0, 0) * factor_eop;
					paraSatOrbEst.xpDot  += matdx.GetElement(k_StaPara + 1, 0) * factor_eop;
					paraSatOrbEst.yp     += matdx.GetElement(k_StaPara + 2, 0) * factor_eop;
					paraSatOrbEst.ypDot  += matdx.GetElement(k_StaPara + 3, 0) * factor_eop;
					paraSatOrbEst.ut1Dot += matdx.GetElement(k_StaPara + 4, 0) * factor_eop;
					k_StaPara = k_StaPara + 5;
				}
				if(m_podParaDefine.bOnEst_StaPos)
				{									
					for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)
					{	
						if(it->second.bOnEst_StaPos)
						{
							// 根据测站在地固系下的位置，计算东北天坐标系
							POS3D S_U;       // 垂直径向
							S_U =  it->second.posvel.getPos();						
							POS3D S_N;       // 北方向
							S_N.x = 0;
							S_N.y = 0;
							S_N.z = EARTH_R; // 北极点								
							POS3D S_E;       // 东方向
							vectorCross(S_E,S_N,S_U);
							vectorCross(S_N,S_U,S_E);
							S_E = vectorNormal(S_E);
							S_N = vectorNormal(S_N);
							S_U = vectorNormal(S_U);
							POS3D  dS;
							dS = S_E * matdx.GetElement(k_StaPara + 3 * it->second.indexEst_StaPos + 0, 0) +
								 S_N * matdx.GetElement(k_StaPara + 3 * it->second.indexEst_StaPos + 1, 0) +
								 S_U * matdx.GetElement(k_StaPara + 3 * it->second.indexEst_StaPos + 2, 0);
							it->second.pos_Est.x += dS.x;
							it->second.pos_Est.y += dS.y;
							it->second.pos_Est.z += dS.z;							
						}
						else								
							it->second.pos_Est = it->second.posvel.getPos();
					}
					k_StaPara += count_StaParameter;				
				}
				if(m_podParaDefine.bOnEst_StaTropZenithDelay)
				{					
					for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)					
						for(size_t s_t = 0; s_t < it->second.zenithDelayEstList.size(); s_t++)						
							it->second.zenithDelayEstList[s_t].zenithDelay_Est += matdx.GetElement(it->second.zenithIndex_0 + int(s_t), 0);										
				}
				sprintf(info, "max_adjust_pos = %.5f", max_adjust_pos);
				RuningInfoFile::Add(info);
				printf("%s\n",info);
				// 1. 轨道动力学参数文件
				char dynpodFilePath[300];
				sprintf(dynpodFilePath,"%s\\dynpod_%s.fit", folder.c_str(), sp3FileName_noexp.c_str());
				FILE * pFitFile = fopen(dynpodFilePath, "w+");
				fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
				fprintf(pFitFile, "%3d. EOP  XP   (mas)    %20.4f%10.4f%20.4f\n",  1,paraSp3Fit.xp * 180 / PI * 3600000,                      (paraSatOrbEst.xp - paraSp3Fit.xp) * 180 / PI * 3600000,                          paraSatOrbEst.xp * 180 / PI * 3600000);
				fprintf(pFitFile, "%3d. EOP  XPDOT(mas/d)  %20.4f%10.4f%20.4f\n",  2,paraSp3Fit.xpDot * 86400.0  * 180 / PI * 3600000,        (paraSatOrbEst.xpDot  - paraSp3Fit.xpDot) * 86400.0  * 180 / PI * 3600000,        paraSatOrbEst.xpDot * 86400.0  * 180 / PI * 3600000);
				fprintf(pFitFile, "%3d. EOP  YP   (mas)    %20.4f%10.4f%20.4f\n",  3,paraSp3Fit.yp * 180 / PI * 3600000,                      (paraSatOrbEst.yp - paraSp3Fit.yp) * 180 / PI * 3600000,                          paraSatOrbEst.yp * 180 / PI * 3600000);
				fprintf(pFitFile, "%3d. EOP  YPDOT(mas/d)  %20.4f%10.4f%20.4f\n",  4,paraSp3Fit.ypDot * 86400.0  * 180 / PI * 3600000,        (paraSatOrbEst.ypDot - paraSp3Fit.ypDot) * 86400.0  * 180 / PI * 3600000,         paraSatOrbEst.ypDot * 86400.0  * 180 / PI * 3600000);
				fprintf(pFitFile, "%3d. EOP  UT   (ms)     %20.4f%10.4f%20.4f\n",  5,paraSp3Fit.ut1 * 86400.0 / (2 * PI) * 1.0E+3,            (paraSatOrbEst.ut1 - paraSp3Fit.ut1) * 180 / PI * 3600000,                        paraSatOrbEst.ut1 * 86400.0 / (2 * PI) * 1.0E+3);
				fprintf(pFitFile, "%3d. EOP  UTDOT(ms/d)   %20.4f%10.4f%20.4f\n",  6,paraSp3Fit.ut1Dot * 86400.0 / (2 * PI) * 1.0E+3 * 86400, (paraSatOrbEst.ut1Dot - paraSp3Fit.ut1Dot) * 86400.0 / (2 * PI) * 1.0E+3 * 86400, paraSatOrbEst.ut1Dot * 86400.0 / (2 * PI) * 1.0E+3 * 86400);
				int k_Parameter = 6;
				for(SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); it != paraSatOrbEst.satParaList.end(); ++it)
				{// 还原初始轨道位置速度到ECEF坐标系下
					Matrix matJ2000Pos, matJ2000Vel, matECFPos,matECFVel;
					matJ2000Pos.Init(3,1);
					matJ2000Vel.Init(3,1);
					matECFPos.Init(3,1);
					matECFVel.Init(3,1);
					matJ2000Pos.SetElement(0,0,it->second.dynamicDatum_Est.X0.x);
					matJ2000Pos.SetElement(1,0,it->second.dynamicDatum_Est.X0.y);
					matJ2000Pos.SetElement(2,0,it->second.dynamicDatum_Est.X0.z);
					matJ2000Vel.SetElement(0,0,it->second.dynamicDatum_Est.X0.vx);
					matJ2000Vel.SetElement(1,0,it->second.dynamicDatum_Est.X0.vy);
					matJ2000Vel.SetElement(2,0,it->second.dynamicDatum_Est.X0.vz);
					Matrix matPR_NR, matER, matEP, matER_DOT;
					m_TimeCoordConvert.Matrix_J2000_ECEF(TimeCoordConvert::TDT2GPST(it->second.dynamicDatum_Est.T0), matPR_NR, matER, matEP, matER_DOT);
					Matrix matEst_EP, matEst_ER;
					paraSatOrbEst.getEst_EOP(TimeCoordConvert::TDT2GPST(it->second.dynamicDatum_Est.T0), matEst_EP, matEst_ER);
					matEP = matEst_EP * matEP;
					matER = matEst_ER * matER; // 2013/04/24, 原程序漏乘 matEst_ER
					matECFPos = matPR_NR * matJ2000Pos;
					matECFVel = matPR_NR * matJ2000Vel;
					matECFVel = matER *  matECFVel + matER_DOT * matECFPos;
					matECFPos = matER *  matECFPos;
					matECFPos = matEP *  matECFPos;
					matECFVel = matEP *  matECFVel;
					it->second.dynamicDatum_Est.X0_ECEF.x  = matECFPos.GetElement(0, 0);
					it->second.dynamicDatum_Est.X0_ECEF.y  = matECFPos.GetElement(1, 0);
					it->second.dynamicDatum_Est.X0_ECEF.z  = matECFPos.GetElement(2, 0);
					it->second.dynamicDatum_Est.X0_ECEF.vx = matECFVel.GetElement(0, 0);
					it->second.dynamicDatum_Est.X0_ECEF.vy = matECFVel.GetElement(1, 0);
					it->second.dynamicDatum_Est.X0_ECEF.vz = matECFVel.GetElement(2, 0);
					fprintf(pFitFile, "\n");
					fprintf(pFitFile, "%3d. PN%2d X    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 1, 
																					   it->first,
																					   it->second.dynamicDatum_Init.X0_ECEF.x, 
																					   it->second.dynamicDatum_Est.X0_ECEF.x - it->second.dynamicDatum_Init.X0_ECEF.x, 
																					   it->second.dynamicDatum_Est.X0_ECEF.x);
					fprintf(pFitFile, "%3d. PN%2d Y    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 2, 
																					   it->first,
																					   it->second.dynamicDatum_Init.X0_ECEF.y, 
																					   it->second.dynamicDatum_Est.X0_ECEF.y - it->second.dynamicDatum_Init.X0_ECEF.y, 
																					   it->second.dynamicDatum_Est.X0_ECEF.y);
					fprintf(pFitFile, "%3d. PN%2d Z    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 3,  
																					   it->first,
																					   it->second.dynamicDatum_Init.X0_ECEF.z, 
																					   it->second.dynamicDatum_Est.X0_ECEF.z - it->second.dynamicDatum_Init.X0_ECEF.z, 
																					   it->second.dynamicDatum_Est.X0_ECEF.z);
					fprintf(pFitFile, "%3d. PN%2d XDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 4,  
																					   it->first,
																					   it->second.dynamicDatum_Init.X0_ECEF.vx, 
																					   it->second.dynamicDatum_Est.X0_ECEF.vx - it->second.dynamicDatum_Init.X0_ECEF.vx, 
																					   it->second.dynamicDatum_Est.X0_ECEF.vx);
					fprintf(pFitFile, "%3d. PN%2d YDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 5,  
																					   it->first,
																					   it->second.dynamicDatum_Init.X0_ECEF.vy, 
																					   it->second.dynamicDatum_Est.X0_ECEF.vy - it->second.dynamicDatum_Init.X0_ECEF.vy, 
																					   it->second.dynamicDatum_Est.X0_ECEF.vy);
					fprintf(pFitFile, "%3d. PN%2d ZDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 6,  
																					   it->first,
																					   it->second.dynamicDatum_Init.X0_ECEF.vz, 
																					   it->second.dynamicDatum_Est.X0_ECEF.vz - it->second.dynamicDatum_Init.X0_ECEF.vz, 
																					   it->second.dynamicDatum_Est.X0_ECEF.vz);
					for(size_t s_kk = 0; s_kk < it->second.dynamicDatum_Est.solarPressureParaList.size(); s_kk++)
					{// 考虑多组太阳光压参数, 20131209, 谷德峰
						fprintf(pFitFile, "%3d. PN%2d D0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 7,  
																						   it->first,
																						   it->second.dynamicDatum_Init.solarPressureParaList[s_kk].D0 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].D0 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_kk].D0 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].D0 * 1.0E+7);
						fprintf(pFitFile, "%3d. PN%2d DCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 8,  
																						   it->first,
																						   it->second.dynamicDatum_Init.solarPressureParaList[s_kk].DC1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].DC1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_kk].DC1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].DC1 * 1.0E+7);
						fprintf(pFitFile, "%3d. PN%2d DSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 9,  
																						   it->first,
																						   it->second.dynamicDatum_Init.solarPressureParaList[s_kk].DS1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].DS1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_kk].DS1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].DS1 * 1.0E+7);
						fprintf(pFitFile, "%3d. PN%2d Y0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 10,  
																						   it->first,
																						   it->second.dynamicDatum_Init.solarPressureParaList[s_kk].Y0 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].Y0 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_kk].Y0 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].Y0 * 1.0E+7);
						fprintf(pFitFile, "%3d. PN%2d YCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 11,  
																						   it->first,
																						   it->second.dynamicDatum_Init.solarPressureParaList[s_kk].YC1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].YC1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_kk].YC1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].YC1 * 1.0E+7);
						fprintf(pFitFile, "%3d. PN%2d YSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 12,  
																						   it->first,
																						   it->second.dynamicDatum_Init.solarPressureParaList[s_kk].YS1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].YS1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_kk].YS1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].YS1 * 1.0E+7);
						fprintf(pFitFile, "%3d. PN%2d X0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 13,  
																						   it->first,
																						   it->second.dynamicDatum_Init.solarPressureParaList[s_kk].B0 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].B0 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_kk].B0 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].B0 * 1.0E+7);
						fprintf(pFitFile, "%3d. PN%2d XCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 14,  
																						   it->first,
																						   it->second.dynamicDatum_Init.solarPressureParaList[s_kk].BC1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].BC1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_kk].BC1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].BC1 * 1.0E+7);
						fprintf(pFitFile, "%3d. PN%2d XSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter + 15,  
																						   it->first,
																						   it->second.dynamicDatum_Init.solarPressureParaList[s_kk].BS1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].BS1 * 1.0E+7 - it->second.dynamicDatum_Init.solarPressureParaList[s_kk].BS1 * 1.0E+7, 
																						   it->second.dynamicDatum_Est.solarPressureParaList[s_kk].BS1 * 1.0E+7);
					}
					k_Parameter = k_Parameter + count_dyn_eachSat;				
				}
													
				for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)
				{	
					fprintf(pFitFile, "\n");
					fprintf(pFitFile, "%3d. %s X    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 1,  
																					it->first.c_str(),
																					it->second.posvel.x, 
																					it->second.pos_Est.x - it->second.posvel.x, 
																					it->second.pos_Est.x);
					fprintf(pFitFile, "%3d. %s Y    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 2,  
																					it->first.c_str(),
																					it->second.posvel.y, 
																					it->second.pos_Est.y - it->second.posvel.y, 
																					it->second.pos_Est.y);
					fprintf(pFitFile, "%3d. %s Z    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 3,  
																					it->first.c_str(),
																					it->second.posvel.z, 
																					it->second.pos_Est.z - it->second.posvel.z, 
																					it->second.pos_Est.z);
									
					for(size_t s_t = 0; s_t < it->second.zenithDelayEstList.size(); s_t++)
					{
						fprintf(pFitFile, "%3d. %s TR%02d (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 3 + s_t + 1,  
																						  it->first.c_str(),
																						  s_t + 1,
																						  it->second.zenithDelayEstList[s_t].zenithDelay_Init, 
																						  it->second.zenithDelayEstList[s_t].zenithDelay_Est - it->second.zenithDelayEstList[s_t].zenithDelay_Init, 
																						  it->second.zenithDelayEstList[s_t].zenithDelay_Est);
					}

					k_Parameter = k_Parameter + 3 + int(it->second.zenithDelayEstList.size());						
				}
				fclose(pFitFile);
				//printf("max_adjust_pos = %.5f\n", max_adjust_pos);
				// 浮点解收敛条件
				if(m_podParaDefine.OrbitIterativeNum == 1)
					flag_break = true;
				if( total_iterator >= 2 || iterator_after_AmbFixed >= 3) 
				{// 窄巷模糊度固定后只迭代一次, 残差编辑后只迭代一次
					/*
					total_iterator = 1, iterator_after_AmbFixed = 0: 浮点解        + 改进               , 
					total_iterator = 2, iterator_after_AmbFixed = 0: 浮点解 + 编辑 + 改进 + 第 1 次窄巷短基线模糊度固定
					total_iterator = 3, iterator_after_AmbFixed = 1: 固定解 + 编辑 + 改进 + 第 2 次窄巷长基线模糊度固定
					total_iterator = 4, iterator_after_AmbFixed = 2: 固定解 + 编辑 + 改进 + 第 3 次窄巷长基线模糊度固定
					total_iterator = 5, iterator_after_AmbFixed = 3: 固定解 + 编辑 + 改进                              ,  
					total_iterator = 6, iterator_after_AmbFixed = 4: 更新轨道、定轨残差                             ,  
					*/
					if(iterator_after_AmbFixed >= 3)
					{
						flag_break = true;
						continue; // 跳出, 不进行模糊度固定
					}
					if(!m_podParaDefine.bOn_AmbiguityFix)
					{
						sprintf(info, "窄巷模糊度未固定!");
						RuningInfoFile::Add(info);
						printf("%s\n",info);
						if(total_iterator >= m_podParaDefine.OrbitIterativeNum)//20140929,控制最大迭代次数
							flag_break = true;													
					}
					else
					{// 求解窄巷模糊度
						RuningInfoFile::Add("窄巷模糊度固定信息====================================================");
						// 模糊度分步固定策略: 先固定一部分的模糊度 - 然后改进轨道、模糊度浮点解、协方差矩阵 - 再固定剩余的模糊度
                        //Matrix matdx(count_DynParameter, 1);
						count_All_FixedAmbiguity = 0; // 总模糊度个数
						count_All_UnFixedAmbiguity = 0; // 总未固定的模糊度个数
						for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
						{
							if(iterator_after_AmbFixed == 0) 
							{// 第 1 次窄巷模糊度固定, 考虑到概略轨道误差的影响, 仅固定短基线情况下的模糊度
								if(m_staBaseLineList[b_i].getStaDistance() >= m_podParaDefine.maxdistance_Ambfixed_short)
									m_staBaseLineList[b_i].bOn_AmbiguityFix = false;
								else
									m_staBaseLineList[b_i].bOn_AmbiguityFix = true;
							}
							else
							{// 考虑到概略轨道误差的影响, 超长基线模糊度不固定
								if(m_staBaseLineList[b_i].getStaDistance() >= m_podParaDefine.maxdistance_Ambfixed_long)
									m_staBaseLineList[b_i].bOn_AmbiguityFix = false;
								else
									m_staBaseLineList[b_i].bOn_AmbiguityFix = true;
							}
							int begin_subsection = 0; // 记录每个模糊度分块的模糊度参数起点
							int count_DD_Fixed_L1 = 0;
							int count_DD_UnFixed_L1 = 0;
							for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
							{
								// 统计窄巷模糊度浮点解个数 count_DD_Float_L1_i, count_DD_Float_L1_i 中始终包含宽巷未固定的部分, 这一部分无法固定
								// 根据 ambiguity_DD_L1_UnFixed_list 重新计算 count_DD_Float_L1_i, ambiguity_DD_L1_UnFixed_list 的长度为总的模糊度个数, 不要改变其总长度
								// 只在第 1 次模糊度固定迭代时, 才满足 count_DD_Float_L1_i =  m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list.size()
								int count_DD_Float_L1_i = 0;
								for(int i = 0; i < int(m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list.size()); i++)
								{//如果 ambiguity_DD_L1_UnFixed_list[i] == -1, 说明窄巷模糊度已经被固定 
									if(m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[i] >= 0) 
										count_DD_Float_L1_i++;
								}
								if(count_DD_Float_L1_i == 0)
									continue; // 没有待固定的窄巷模糊度
								Matrix matDDQ_L1(count_DD_Float_L1_i, count_DD_Float_L1_i); // 拷贝模糊度协方差矩阵 
								for(int i = 0; i < count_DD_Float_L1_i; i++)
								{
									for(int j = 0; j < count_DD_Float_L1_i; j++)
										matDDQ_L1.SetElement(i, j, m_staBaseLineList[b_i].matQ_dd_L1.GetElement(begin_subsection + i, begin_subsection + j));
								}
								begin_subsection += count_DD_Float_L1_i;
								// 模糊度浮点解
								Matrix matDDFloat_L1(count_DD_Float_L1_i, 1); // 拷贝模糊度浮点解 
								Matrix matSelectedFlag(count_DD_Float_L1_i, 1);
								vector<int> validIndexList_noMWFloat; // 记录 matSelectedFlag 中可固定的模糊度, 去除那些宽巷无法固定的部分
								vector<int> id_AmbiguityList; // 用于记录每个模糊度浮点解在ambiguity_DD_L1_list中的序号
								int i_k = 0;
								for(int i = 0; i < int(m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list.size()); i++)
								{ 
									if(m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[i] >= 0) 
									{// 如果 ambiguity_DD_L1_UnFixed_list[i] == -1, 说明窄巷模糊度已经被固定
										matDDFloat_L1.SetElement(i_k, 0, m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[i]); // 拷贝模糊度浮点解 
										// 首先, 继承宽巷的标记, 宽巷已经固定的, 才能继续再进行固定
									    // 其次, 继承窄巷的标记, 窄巷未被固定的, 才能继续再进行固定
										if(m_staBaseLineList[b_i].amSectionList[s_l].matDDFixedFlag_MW.GetElement(i, 0) == 1.0)
										{
											matSelectedFlag.SetElement(i_k, 0, 1.0); // 初始化 1, 表示为待固定模糊度参数
											validIndexList_noMWFloat.push_back(i_k);
										}
										id_AmbiguityList.push_back(i); // 记录每个模糊度浮点解在ambiguity_DD_L1_list中的序号
										i_k++;
									}
								}
								if(validIndexList_noMWFloat.size() == 0)
								{
									count_DD_UnFixed_L1 += count_DD_Float_L1_i;
									count_DD_Fixed_L1   += 0;
									continue;// 没有待固定的窄巷模糊度
								}
								Matrix matAFixed, matSqnorm;
								m_staBaseLineList[b_i].amSectionList[s_l].matDDFixedFlag_L1.Init(count_DD_Float_L1_i, 1); // 初始化均未固定

								//// 统计弧段数据个数
								//double mean_arcpoints = 0;
								//size_t min_arcpoints  = m_staBaseLineList[b_i].amSectionList[s_l].mw_ArcList[0].obsList.size();
								//size_t max_arcpoints  = m_staBaseLineList[b_i].amSectionList[s_l].mw_ArcList[0].obsList.size();
								//for(int ii = 0; ii < int(m_staBaseLineList[b_i].amSectionList[s_l].mw_ArcList.size()); ii++)
								//{
								//	mean_arcpoints += m_staBaseLineList[b_i].amSectionList[s_l].mw_ArcList[ii].obsList.size();
								//	min_arcpoints   = min_arcpoints > m_staBaseLineList[b_i].amSectionList[s_l].mw_ArcList[ii].obsList.size() ? m_staBaseLineList[b_i].amSectionList[s_l].mw_ArcList[ii].obsList.size() : min_arcpoints;
								//	max_arcpoints   = max_arcpoints < m_staBaseLineList[b_i].amSectionList[s_l].mw_ArcList[ii].obsList.size() ? m_staBaseLineList[b_i].amSectionList[s_l].mw_ArcList[ii].obsList.size() : max_arcpoints;
								//}
								//mean_arcpoints = mean_arcpoints / m_staBaseLineList[b_i].amSectionList[s_l].mw_ArcList.size();
								
								if(m_staBaseLineList[b_i].amSectionList[s_l].bDDAmFixed_MW)
								{
									if(!GPSPod::GPSMeoSatDynPOD::lambdaSelected(matDDFloat_L1, matDDQ_L1, matSqnorm, matAFixed, matSelectedFlag))
									{
										count_DD_UnFixed_L1 += count_DD_Float_L1_i;
										count_DD_Fixed_L1   += 0;
										continue; // 模糊度固定异常
									}
									double ksb = matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0); // 最优/次优之比
									Matrix matSelectedAFixed = matAFixed; 
									double threhold_LAMBDA_ksb = m_podParaDefine.threhold_LAMBDA_ksb;
									int count_Ambiguity_deleted_i = 0;
									while(ksb < threhold_LAMBDA_ksb)
									{
										validIndexList_noMWFloat.clear();
										count_Ambiguity_deleted_i++;
										threhold_LAMBDA_ksb = 3.0; // 非首次成功, 需要提高阈值增加可靠性
										for(int ii = 0; ii < count_DD_Float_L1_i; ii++)
										{
											if(matSelectedFlag.GetElement(ii, 0) == 1.0)
												validIndexList_noMWFloat.push_back(ii);
										}
										if(validIndexList_noMWFloat.size() < 2) // 2013/02/25, 谷德峰修改, 至少两个才能进行筛选
										{
											matSelectedFlag.Init(count_DD_Float_L1_i, 1);
											break;
										}

										//// 剔除 "最优/次优" 改进量最大的
										//int i_max = -1;
										//double ksb_max = 0.0;
										//for(int ii = 0; ii < int(validIndexList.size()); ii++)
										//{
										//	Matrix matSelectedFlag_i = matSelectedFlag;
										//	matSelectedFlag_i.SetElement(validIndexList[ii], 0, 0.0);
										//	if(lambdaSelected(matDDFloat_L1, matDDQ_L1, matSqnorm, matAFixed, matSelectedFlag_i))
										//	{
										//		if(matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0) > ksb_max)
										//		{
										//			i_max  = validIndexList[ii];
										//			ksb_max = matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0);
										//			matSelectedAFixed = matAFixed;
										//		}
										//	}
										//}
										// 更新模糊度固定标记
										//if(i_max >= 0)
										//{
										//	matSelectedFlag.SetElement(i_max, 0, 0.0);
										//	ksb = ksb_max;
										//}
										//else
										//{
										//	matSelectedFlag.Init(count_DD_Float_L1_i, 1);
										//	break;
										//}

										// 剔除 "浮点解" 改进量最大的
										int i_max = -1;
										double max_AmFloat_AmFixed = 0.0;
										for(int ii = 0; ii < int(validIndexList_noMWFloat.size()); ii++)
										{
											if(max_AmFloat_AmFixed < fabs(matDDFloat_L1.GetElement(validIndexList_noMWFloat[ii], 0) - matSelectedAFixed.GetElement(validIndexList_noMWFloat[ii], 0)))
											{
												i_max = validIndexList_noMWFloat[ii];
												max_AmFloat_AmFixed = fabs(matDDFloat_L1.GetElement(validIndexList_noMWFloat[ii], 0) - matSelectedAFixed.GetElement(validIndexList_noMWFloat[ii], 0));
											}
										}
										// 更新模糊度固定标记
										if(i_max >= 0)
										{
											matSelectedFlag.SetElement(i_max, 0, 0.0);
											if(GPSPod::GPSMeoSatDynPOD::lambdaSelected(matDDFloat_L1, matDDQ_L1, matSqnorm, matAFixed, matSelectedFlag))
											{
												ksb = matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0);
											}
										}
										else
										{
											matSelectedFlag.Init(count_DD_Float_L1_i, 1);
											break;
										}
									}
									// 统计模糊度固定个数
									m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed = 0; 
									double max_AmFloat_AmFixed = 0.0;
									for(int ii = 0; ii < count_DD_Float_L1_i; ii++)
									{
										if(matSelectedFlag.GetElement(ii, 0) == 1.0)
										{
											m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_AmbiguityList[ii]] = -1; // 20140421
											if(max_AmFloat_AmFixed < fabs(matDDFloat_L1.GetElement(ii, 0) - matSelectedAFixed.GetElement(ii, 0)))
												max_AmFloat_AmFixed = fabs(matDDFloat_L1.GetElement(ii, 0) - matSelectedAFixed.GetElement(ii, 0));
										}
										else
										{
											m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed++; // 更新个数统计结果
											m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_AmbiguityList[ii]] = m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed - 1; // 20140421
										}
										/*sprintf(info, "%14.4f %14.4f %3.1f", matDDFloat_L1.GetElement(ii, 0),
																			 matSelectedAFixed.GetElement(ii, 0),
																			 matSelectedFlag.GetElement(ii, 0));
										RuningInfoFile::Add(info);*/
									}
									
									if(ksb >= threhold_LAMBDA_ksb
									&& m_staBaseLineList[b_i].bOn_AmbiguityFix) 
									{
										count_DD_UnFixed_L1 += m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed;
										count_DD_Fixed_L1   += count_DD_Float_L1_i - m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed;
										sprintf(info, "%02d %02d:%02d:%02d-%02d:%02d:%02d  %5d %5d %14.4f(%6.4f)", s_l+1,
																											  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t0].t.hour,
																											  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t0].t.minute,
																											  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t0].t.second),
																											  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t1].t.hour,
																											  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t1].t.minute,
																											  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t1].t.second),
																											  count_DD_Float_L1_i, 
																											  count_DD_Float_L1_i - m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed, 
																											  ksb,
																											  max_AmFloat_AmFixed);
										RuningInfoFile::Add(info);
										m_staBaseLineList[b_i].amSectionList[s_l].bDDAmFixed_L1 = true;
										m_staBaseLineList[b_i].amSectionList[s_l].matDDFixedFlag_L1 = matSelectedFlag; // 更改窄巷模糊度固定标记 matDDFixedFlag_L1
										for(int ii = 0; ii < count_DD_Float_L1_i; ii++)
										{
											if(matSelectedFlag.GetElement(ii, 0) == 1.0)
											{// 更改窄巷模糊度浮点解
												m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[id_AmbiguityList[ii]] = matSelectedAFixed.GetElement(ii, 0);
											}
										}
									}
									else
									{
										count_DD_UnFixed_L1 += count_DD_Float_L1_i;
										count_DD_Fixed_L1   += 0;
										sprintf(info, "%02d %02d:%02d:%02d-%02d:%02d:%02d  %5d %5d %14.4f(%6.4f)", s_l+1,
																											  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t0].t.hour,
																											  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t0].t.minute,
																											  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t0].t.second),
																											  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t1].t.hour,
																											  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t1].t.minute,
																											  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t1].t.second),
																											  count_DD_Float_L1_i, 
																											  0, 
																											  ksb,
																											  max_AmFloat_AmFixed);
										RuningInfoFile::Add(info);
										m_staBaseLineList[b_i].amSectionList[s_l].bDDAmFixed_L1 = false;
										// 2014/01/13, 更新未固定模糊度, 谷德峰
										m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed = count_DD_Float_L1_i;
										m_staBaseLineList[b_i].amSectionList[s_l].matDDFixedFlag_L1.Init(count_DD_Float_L1_i, 1);
										for(int ii = 0; ii < count_DD_Float_L1_i; ii++)
											m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_AmbiguityList[ii]] = ii;
									}
								}
								else
								{
									count_DD_UnFixed_L1 += count_DD_Float_L1_i;
									count_DD_Fixed_L1   += 0;
									sprintf(info, "%02d %02d:%02d:%02d-%02d:%02d:%02d  %5d %5d %14.4f(%6.4f)", s_l+1,
																										  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t0].t.hour,
																										  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t0].t.minute,
																										  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t0].t.second),
																										  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t1].t.hour,
																										  m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t1].t.minute,
																										  int(m_staBaseLineList[b_i].editedSdObsFile.m_data[m_staBaseLineList[b_i].amSectionList[s_l].id_t1].t.second),
																										  count_DD_Float_L1_i, 
																										  0, 
																										  0,
																										  0);
									RuningInfoFile::Add(info);
									m_staBaseLineList[b_i].amSectionList[s_l].bDDAmFixed_L1 = false;
									m_staBaseLineList[b_i].amSectionList[s_l].matDDFixedFlag_L1.Init(count_DD_Float_L1_i, 1);
									// 2014/01/13, 更新未固定模糊度, 谷德峰
									m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed = count_DD_Float_L1_i; 
									for(int ii = 0; ii < count_DD_Float_L1_i; ii++)
										m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[id_AmbiguityList[ii]] = ii;
								}
							}
							// 更新 N_bb、n_xb、nb、matQ_dd_L1
							m_staBaseLineList[b_i].N_bb.Init(count_DD_UnFixed_L1, count_DD_UnFixed_L1);
							m_staBaseLineList[b_i].n_xb.Init(count_DynParameter, count_DD_UnFixed_L1);
							m_staBaseLineList[b_i].nb.Init(count_DD_UnFixed_L1, 1);
							m_staBaseLineList[b_i].matQ_dd_L1.Init(count_DD_UnFixed_L1, count_DD_UnFixed_L1);
							double rate_amFixed_L1 = 0.0;
							if(count_DD_Fixed_L1 + count_DD_UnFixed_L1 > 0)
								rate_amFixed_L1 = double(count_DD_Fixed_L1) / (count_DD_Fixed_L1 + count_DD_UnFixed_L1);
							sprintf(info, "%s %s 模糊度总个数%3d, 模糊度固定个数%3d, 窄巷模糊度固定成功率%5.3f", staNameList[staBaseLineIdList_A[b_i]].c_str(),
																											    staNameList[staBaseLineIdList_B[b_i]].c_str(),
																											    count_DD_Fixed_L1 + count_DD_UnFixed_L1,
																											    count_DD_Fixed_L1,
																											    rate_amFixed_L1);
							RuningInfoFile::Add(info);
							count_All_FixedAmbiguity   += count_DD_Fixed_L1;
							count_All_UnFixedAmbiguity += count_DD_UnFixed_L1;
						}
						double rate_amFixed_L1_All = double(count_All_FixedAmbiguity) / (count_All_FixedAmbiguity + count_All_UnFixedAmbiguity);
						sprintf(info, "模糊度总个数%d, 模糊度固定个数%d, 窄巷模糊度固定成功率%5.3f",count_All_FixedAmbiguity + count_All_UnFixedAmbiguity,
																								    count_All_FixedAmbiguity,
																								    rate_amFixed_L1_All);
						RuningInfoFile::Add(info);
						bDDAmFixed_L1 = true; // 标记已经进行模糊度固定
					}
						//// 求解窄巷模糊度
						//RuningInfoFile::Add("窄巷模糊度固定信息====================================================");
						//count_All_FixedAmbiguity = 0; // 总模糊度个数
						//count_All_UnFixedAmbiguity = 0; // 总未固定的模糊度个数

						////FILE *pfile = fopen("C:\\模糊度固定.txt","w+");
						////FILE *pfile_DD = fopen("C:\\matDDQ_L1.txt","w+");
						//Matrix matdx = n_xx_inv * nx;
						//for(size_t b_i = 0; b_i < m_staBaseLineList.size(); b_i++)
						//{
						//	if(iterator_after_AmbFixed == 0) 
						//	{// 第 1 次窄巷模糊度固定, 考虑到概略轨道误差的影响, 仅固定短基线情况下的模糊度
						//		if(m_staBaseLineList[b_i].getStaDistance() >= m_podParaDefine.maxdistance_Ambfixed_short)
						//			m_staBaseLineList[b_i].bOn_AmbiguityFix = false;
						//		else
						//			m_staBaseLineList[b_i].bOn_AmbiguityFix = true;
						//	}
						//	else
						//	{// 考虑到概略轨道误差的影响, 超长基线模糊度不固定
						//		if(m_staBaseLineList[b_i].getStaDistance() >= m_podParaDefine.maxdistance_Ambfixed_long)
						//			m_staBaseLineList[b_i].bOn_AmbiguityFix = false;
						//		else
						//			m_staBaseLineList[b_i].bOn_AmbiguityFix = true;
						//	}
						//	int k_sub = 0;
						//	int count_DD_Fixed_L1   = 0;
						//	int count_DD_UnFixed_L1 = 0;
						//	for(size_t s_l = 0; s_l < m_staBaseLineList[b_i].amSectionList.size(); s_l++)
						//	{
						//		int count_DD_Float_L1_i = int(m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_MW_List.size());
						//		Matrix matDDQ_L1(count_DD_Float_L1_i, count_DD_Float_L1_i);
						//		for(int i = 0; i < count_DD_Float_L1_i; i++)
						//		{									
						//			for(int j = 0; j < count_DD_Float_L1_i; j++)
						//			{
						//				matDDQ_L1.SetElement(i, j, m_staBaseLineList[b_i].matQ_dd_L1.GetElement(k_sub + i, k_sub + j));
						//			}
						//		}
						//		//fprintf(pfile_DD, "%s\n\n", matDDQ_L1.ToString().c_str());
						//		k_sub += count_DD_Float_L1_i;
						//		// 模糊度浮点解
						//		Matrix matDDFloat_L1(count_DD_Float_L1_i, 1);
						//		for(int i = 0; i < count_DD_Float_L1_i; i++)
						//			matDDFloat_L1.SetElement(i, 0, m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[i]);
						//		Matrix matAFixed;
						//		Matrix matSqnorm;
						//		m_staBaseLineList[b_i].amSectionList[s_l].matDDAdjust_L1.Init(count_DD_Float_L1_i, 1);
						//		m_staBaseLineList[b_i].amSectionList[s_l].matDDFixedFlag_L1.Init(count_DD_Float_L1_i, 1);
						//		if(m_staBaseLineList[b_i].amSectionList[s_l].bDDAmFixed_MW)
						//		{
						//			// 初始化 matSelectedFlag
						//			Matrix matSelectedFlag = m_staBaseLineList[b_i].amSectionList[s_l].matDDFixedFlag_MW;
						//			GPSPod::GPSMeoSatDynPOD::lambdaSelected(matDDFloat_L1, matDDQ_L1, matSqnorm, matAFixed, matSelectedFlag);
						//			double ksb = matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0);
						//			Matrix matSelectedAFixed = matAFixed;
						//			vector<int> validIndexList;
						//			while(ksb < m_podParaDefine.threhold_LAMBDA_ksb)
						//			{
						//				validIndexList.clear();
						//				for(int ii = 0; ii < count_DD_Float_L1_i; ii++)
						//				{
						//					if(matSelectedFlag.GetElement(ii, 0) == 1.0)
						//						validIndexList.push_back(ii);
						//				}
						//				if(validIndexList.size() < 2) // 2013/02/25, 谷德峰修改, 至少两个才能进行筛选
						//				{
						//					matSelectedFlag.Init(count_DD_Float_L1_i, 1);
						//					break;
						//				}
						//				int i_max = -1;
						//				double ksb_max = 0.0;
						//				for(int ii = 0; ii < int(validIndexList.size()); ii++)
						//				{
						//					Matrix matSelectedFlag_i = matSelectedFlag;
						//					matSelectedFlag_i.SetElement(validIndexList[ii], 0, 0.0);
						//					if(GPSPod::GPSMeoSatDynPOD::lambdaSelected(matDDFloat_L1, matDDQ_L1, matSqnorm, matAFixed, matSelectedFlag_i))
						//					{
						//						if(matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0) > ksb_max)
						//						{
						//							i_max  = validIndexList[ii];
						//							ksb_max = matSqnorm.GetElement(0, 1) / matSqnorm.GetElement(0, 0);
						//							matSelectedAFixed = matAFixed;
						//						}
						//					}
						//				}
						//				// 更新模糊度固定标记
						//				if(i_max >= 0)
						//				{
						//					matSelectedFlag.SetElement(i_max, 0, 0.0);
						//					ksb = ksb_max;
						//				}
						//				else
						//				{
						//					matSelectedFlag.Init(count_DD_Float_L1_i, 1);
						//					break;
						//				}
						//			}
						//			// 统计模糊度固定个数
						//			m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed = 0; 
						//			for(int ii = 0; ii < count_DD_Float_L1_i; ii++)
						//			{
						//				if(matSelectedFlag.GetElement(ii, 0) == 1.0)
						//				{
						//					count_DD_Fixed_L1++;
						//					m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[ii] = -1;
						//				}
						//				else
						//				{
						//					m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed++; // 更新个数统计结果
						//					m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[ii] = m_staBaseLineList[b_i].amSectionList[s_l].count_DD_L1_UnFixed - 1;
						//					count_DD_UnFixed_L1++;
						//				}
						//			}
						//			if(ksb >= m_podParaDefine.threhold_LAMBDA_ksb)
						//			{
						//				//printf("%5d %5d %14.4f\n", count_DD_Float_L1_i, validIndexList.size() - 1, ksb);
						//				m_staBaseLineList[b_i].amSectionList[s_l].bDDAmFixed_L1 = true;
						//				m_staBaseLineList[b_i].amSectionList[s_l].matDDFixedFlag_L1 = matSelectedFlag;
						//				for(int ii = 0; ii < matSelectedAFixed.GetNumRows(); ii++)
						//				{
						//					//fprintf(pfile, "%16.4f %16.4f %16.4f\n", m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[ii], 
						//					//	                                     matSelectedFlag.GetElement(ii, 0), 
						//					//									     matSelectedAFixed.GetElement(ii, 0));//
						//					if(matSelectedFlag.GetElement(ii, 0) == 1.0)
						//					{
						//						m_staBaseLineList[b_i].amSectionList[s_l].matDDAdjust_L1.SetElement(ii, 0, matSelectedAFixed.GetElement(ii, 0) - m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[ii]);
						//						m_staBaseLineList[b_i].amSectionList[s_l].ambiguity_DD_L1_list[ii] = matSelectedAFixed.GetElement(ii, 0);
						//					}
						//				}
						//				matdx = matdx - n_xx_inv * (m_staBaseLineList[b_i].amSectionList[s_l].n_xb * m_staBaseLineList[b_i].amSectionList[s_l].matDDAdjust_L1);
						//			}
						//			else
						//			{
						//				m_staBaseLineList[b_i].amSectionList[s_l].bDDAmFixed_L1 = false;
						//				//// 2014/01/13, 更新未固定模糊度, 谷德峰
						//				//amSectionList[s_l].count_DD_L1_UnFixed = count_DD_Float_L1_i; 
						//				//for(int ii = 0; ii < count_DD_Float_L1_i; ii++)
						//				//	amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[ii] = ii;
						//			}
						//		}
						//		else
						//		{
						//			m_staBaseLineList[b_i].amSectionList[s_l].bDDAmFixed_L1 = false;
						//			//// 2014/01/13, 更新未固定模糊度, 谷德峰
						//			//amSectionList[s_l].count_DD_L1_UnFixed = count_DD_Float_L1_i; 
						//			//for(int ii = 0; ii < count_DD_Float_L1_i; ii++)
						//			//	amSectionList[s_l].ambiguity_DD_L1_UnFixed_list[ii] = ii;
						//		}
						//	}
						//	// 更新
						//	m_staBaseLineList[b_i].N_bb.Init(count_DD_UnFixed_L1, count_DD_UnFixed_L1);
						//	m_staBaseLineList[b_i].n_xb.Init(count_DynParameter, count_DD_UnFixed_L1);
						//	m_staBaseLineList[b_i].nb.Init(count_DD_UnFixed_L1, 1);
						//	m_staBaseLineList[b_i].matQ_dd_L1.Init(count_DD_UnFixed_L1, count_DD_UnFixed_L1);
						//	double rate_amFixed_L1 = double(count_DD_Fixed_L1) / (count_DD_Fixed_L1 + count_DD_UnFixed_L1);
						//	sprintf(info, "模糊度总个数%d, 模糊度固定个数%d, 窄巷模糊度固定成功率%.4f", count_DD_Fixed_L1 + count_DD_UnFixed_L1,
						//																		   count_DD_Fixed_L1,
						//																		   rate_amFixed_L1);
						//	RuningInfoFile::Add(info);
						//	printf("%s\n",info);
						//	count_All_FixedAmbiguity   += count_DD_Fixed_L1;
						//	count_All_UnFixedAmbiguity += count_DD_UnFixed_L1;
						//	
						//	//printf("模糊度总个数%d, 模糊度固定个数%d, 窄巷模糊度固定成功率%.4f\n", count_DD_Fixed_L1 + count_DD_UnFixed_L1,
						//	//																	   count_DD_Fixed_L1,
						//	//																	   rate_amFixed_L1);//
						//}
						//double rate_amFixed_L1_All = double(count_All_FixedAmbiguity) / (count_All_FixedAmbiguity + count_All_UnFixedAmbiguity);
						//sprintf(info, "模糊度总个数%d, 模糊度固定个数%d, 窄巷模糊度固定成功率%5.3f",count_All_FixedAmbiguity + count_All_UnFixedAmbiguity,
						//																		    count_All_FixedAmbiguity,
						//																		    rate_amFixed_L1_All);
						//RuningInfoFile::Add(info);
						//printf("%s\n",info);
						//bDDAmFixed_L1 = true;
						//
						////fclose(pfile);
						////fclose(pfile_DD);
						////// 计算轨道改进量
						////for(SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); it != paraSatOrbEst.satParaList.end(); ++it)
						////{
						////	it->second.dynamicDatum_Est.X0.x  += matdx.GetElement(it->second.index * count_dyn_eachSat + 0, 0);
						////	it->second.dynamicDatum_Est.X0.y  += matdx.GetElement(it->second.index * count_dyn_eachSat + 1, 0);
						////	it->second.dynamicDatum_Est.X0.z  += matdx.GetElement(it->second.index * count_dyn_eachSat + 2, 0);
						////	it->second.dynamicDatum_Est.X0.vx += matdx.GetElement(it->second.index * count_dyn_eachSat + 3, 0) * factor_vel;
						////	it->second.dynamicDatum_Est.X0.vy += matdx.GetElement(it->second.index * count_dyn_eachSat + 4, 0) * factor_vel;
						////	it->second.dynamicDatum_Est.X0.vz += matdx.GetElement(it->second.index * count_dyn_eachSat + 5, 0) * factor_vel;
						////	for(size_t s_kk = 0; s_kk < it->second.dynamicDatum_Est.solarPressureParaList.size(); s_kk++)
						////	{// 考虑多组太阳光压参数, 20131209, 谷德峰
						////		if(m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_9PARA)
						////		{
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_D0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  6 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_DC += matdx.GetElement(it->second.index * count_dyn_eachSat +  7 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_DS += matdx.GetElement(it->second.index * count_dyn_eachSat +  8 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_Y0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  9 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_YC += matdx.GetElement(it->second.index * count_dyn_eachSat + 10 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_YS += matdx.GetElement(it->second.index * count_dyn_eachSat + 11 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_X0 += matdx.GetElement(it->second.index * count_dyn_eachSat + 12 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_XC += matdx.GetElement(it->second.index * count_dyn_eachSat + 13 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_XS += matdx.GetElement(it->second.index * count_dyn_eachSat + 14 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////		}
						////		else if (m_podParaDefine.solarPressure_Model == TYPE_SOLARPRESSURE_5PARA)
						////		{
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_D0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  6 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_Y0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  7 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_X0 += matdx.GetElement(it->second.index * count_dyn_eachSat +  8 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_XC += matdx.GetElement(it->second.index * count_dyn_eachSat +  9 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////			it->second.dynamicDatum_Est.solarPressureParaList[s_kk].A_XS += matdx.GetElement(it->second.index * count_dyn_eachSat + 10 + int(s_kk) * count_solar_period, 0) * factor_solar;
						////		}
						////	}													
						////}
						////int k_StaPara = count_dyn_eachSat * int(paraSatOrbEst.satParaList.size());
						////if(m_podParaDefine.bOnEst_ERP)
						////{							
						////	paraSatOrbEst.xp     += matdx.GetElement(k_StaPara + 0, 0) * factor_eop;
						////	paraSatOrbEst.xpDot  += matdx.GetElement(k_StaPara + 1, 0) * factor_eop;
						////	paraSatOrbEst.yp     += matdx.GetElement(k_StaPara + 2, 0) * factor_eop;
						////	paraSatOrbEst.ypDot  += matdx.GetElement(k_StaPara + 3, 0) * factor_eop;
						////	paraSatOrbEst.ut1Dot += matdx.GetElement(k_StaPara + 4, 0) * factor_eop;
						////	k_StaPara += 5;
						////}
						////if(m_podParaDefine.bOnEst_StaPos)
						////{										
						////	for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)
						////	{	
						////		if(it->second.bOnEst_StaPos)
						////		{
						////			// 根据测站在地固系下的位置，计算东北天坐标系
						////			POS3D S_U;       // 垂直径向
						////			S_U =  it->second.posvel.getPos();						
						////			POS3D S_N;       // 北方向
						////			S_N.x = 0;
						////			S_N.y = 0;
						////			S_N.z = EARTH_R; // 北极点								
						////			POS3D S_E;       // 东方向
						////			vectorCross(S_E,S_N,S_U);
						////			vectorCross(S_N,S_U,S_E);
						////			S_E = vectorNormal(S_E);
						////			S_N = vectorNormal(S_N);
						////			S_U = vectorNormal(S_U);
						////			POS3D  dS;
						////			dS = S_E * matdx.GetElement(k_StaPara + 3 * it->second.indexEst_StaPos + 0, 0) +
						////				 S_N * matdx.GetElement(k_StaPara + 3 * it->second.indexEst_StaPos + 1, 0) +
						////				 S_U * matdx.GetElement(k_StaPara + 3 * it->second.indexEst_StaPos + 2, 0);
						////			it->second.pos_Est.x += dS.x;
						////			it->second.pos_Est.y += dS.y;
						////			it->second.pos_Est.z += dS.z;
						////		}
						////		else								
						////			it->second.pos_Est = it->second.posvel.getPos();	
						////	}
						////	k_StaPara += count_StaParameter;
						////}
						////if(m_podParaDefine.bOnEst_StaTropZenithDelay)
						////{
						////	for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)					
						////		for(size_t s_t = 0; s_t < it->second.zenithDelayEstList.size(); s_t++)
						////			it->second.zenithDelayEstList[s_t].zenithDelay_Est += matdx.GetElement(it->second.zenithIndex_0 + int(s_t), 0);							       
						////}					
						////k_gaussnewton = 0; // 初始化次数, 重新开始迭代
					
				}
			}
			// 输出定轨结果 sp3 文件			
			// 2. sp3文件
			TDT t0_tdt = TimeCoordConvert::GPST2TDT(t0);
			TDT t1_tdt = TimeCoordConvert::GPST2TDT(t1);
			SP3File ndtSp3File;
			ndtSp3File.m_header.szSP3Version;
			sprintf(ndtSp3File.m_header.szSP3Version, "%2s","#c");
			sprintf(ndtSp3File.m_header.szPosVelFlag, "%1s","P");
			ndtSp3File.m_header.tmStart = t0;
			ndtSp3File.m_header.nNumberofEpochs = int((t1 - t0) / h_sp3) + 1;
			sprintf(ndtSp3File.m_header.szDataType, "%-5s","d+D");
			sprintf(ndtSp3File.m_header.szCoordinateSys, "%-5s","IGS08");
			sprintf(ndtSp3File.m_header.szOrbitType, "%-3s","FIT");
			sprintf(ndtSp3File.m_header.szAgency, "%-4s","NUDT");
			sprintf(ndtSp3File.m_header.szLine2Symbols, "%-2s","##");
			ndtSp3File.m_header.tmGPSWeek = TimeCoordConvert::GPST2WeekTime(t0);
			ndtSp3File.m_header.dEpochInterval = h_sp3;
			double dMJD = TimeCoordConvert::DayTime2MJD(t0);
			ndtSp3File.m_header.nModJulDaySt = long(floor(dMJD));    
			ndtSp3File.m_header.dFractionalDay = dMJD - ndtSp3File.m_header.nModJulDaySt;
			sprintf(ndtSp3File.m_header.szLine3Symbols, "%-2s","+ ");
			sprintf(ndtSp3File.m_header.szLine8Symbols, "%-2s","++");
			ndtSp3File.m_header.bNumberofSats = BYTE(paraSatOrbEst.satParaList.size());
			ndtSp3File.m_header.pstrSatNameList.clear();
			ndtSp3File.m_header.pbySatAccuracyList.clear();
			for(SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); it != paraSatOrbEst.satParaList.end(); ++it)
			{
				char SatName[4];
				sprintf(SatName, "C%02d", it->first);
				ndtSp3File.m_header.pstrSatNameList.push_back(SatName);
				ndtSp3File.m_header.pbySatAccuracyList.push_back(3);
			}
			sprintf(ndtSp3File.m_header.szLine13Symbols, "%%c");
			sprintf(ndtSp3File.m_header.szFileType, "%-2s","C ");
			sprintf(ndtSp3File.m_header.szTimeSystem, "BDS");
			sprintf(ndtSp3File.m_header.szLine15Symbols, "%%f");
			ndtSp3File.m_header.dBaseforPosVel  = 0;
			ndtSp3File.m_header.dBaseforClkRate = 0;
			sprintf(ndtSp3File.m_header.szLine17Symbols, "%%i");
			sprintf(ndtSp3File.m_header.szLine19Symbols, "/*");
			sprintf(ndtSp3File.m_header.szLine19Comment, "%-57s", "National University of Defense Technology (NUDT).");
			ndtSp3File.m_data.clear();
			if(result)
			{
				for(SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); it != paraSatOrbEst.satParaList.end(); ++it)
				{					
					vector<TimePosVel> orbitlist_ac;
					vector<Matrix> matRtPartiallist_ac;
					double h = 75.0;
					adamsCowell_ac(t0_tdt, t1_tdt, it->second.dynamicDatum_Est, orbitlist_ac, matRtPartiallist_ac, h);
					int k = 0;
					double span = t1_tdt - t0_tdt;
					it->second.orbitList_ECEF.clear();
					while(k * h_sp3 < span)             
					{
						TimePosVel point;
						point.t = t0_tdt + k * h_sp3;
						it->second.orbitList_ECEF.push_back(point);
						k++;
					}
					size_t count_ac = orbitlist_ac.size();
					const int nlagrange = 8; 
					if(count_ac < nlagrange) // 如果数据点个数小于nlagrange返回，要求弧段长度 > h * nlagrange = 4分钟
						return false;
					for(size_t s_i = 0; s_i < it->second.orbitList_ECEF.size(); s_i++)
					{
						double spanSecond_t = it->second.orbitList_ECEF[s_i].t - orbitlist_ac[0].t; 
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
						interpOrbit.t = it->second.orbitList_ECEF[s_i].t;
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
						it->second.orbitList_ECEF[s_i] = interpOrbit;
						delete x;
						delete y;
					}					
					// 转换到地球固定坐标系, 坐标系: ITRF 系, 时间: GPS
					for(size_t s_i = 0; s_i < it->second.orbitList_ECEF.size(); s_i++)
					{
						/*double x_ecf[6];
						double x_j2000[6];
						x_j2000[0] = it->second.orbitList_ECEF[s_i].pos.x;  
						x_j2000[1] = it->second.orbitList_ECEF[s_i].pos.y;  
						x_j2000[2] = it->second.orbitList_ECEF[s_i].pos.z;
						x_j2000[3] = it->second.orbitList_ECEF[s_i].vel.x; 
						x_j2000[4] = it->second.orbitList_ECEF[s_i].vel.y; 
						x_j2000[5] = it->second.orbitList_ECEF[s_i].vel.z;
						it->second.orbitList_ECEF[s_i].t = TimeCoordConvert::TDT2GPST(it->second.orbitList_ECEF[s_i].t);
						m_TimeCoordConvert.J2000_ECEF(it->second.orbitList_ECEF[s_i].t, x_j2000, x_ecf);
						it->second.orbitList_ECEF[s_i].pos.x = x_ecf[0]; 
						it->second.orbitList_ECEF[s_i].pos.y = x_ecf[1]; 
						it->second.orbitList_ECEF[s_i].pos.z = x_ecf[2];
						it->second.orbitList_ECEF[s_i].vel.x = x_ecf[3]; 
						it->second.orbitList_ECEF[s_i].vel.y = x_ecf[4]; 
						it->second.orbitList_ECEF[s_i].vel.z = x_ecf[5];*/
						Matrix matJ2000Pos, matJ2000Vel, matECFPos,matECFVel;
						matJ2000Pos.Init(3,1);
						matJ2000Vel.Init(3,1);
						matECFPos.Init(3,1);
						matECFVel.Init(3,1);
						matJ2000Pos.SetElement(0,0,it->second.orbitList_ECEF[s_i].pos.x);
						matJ2000Pos.SetElement(1,0,it->second.orbitList_ECEF[s_i].pos.y);
						matJ2000Pos.SetElement(2,0,it->second.orbitList_ECEF[s_i].pos.z);
						matJ2000Vel.SetElement(0,0,it->second.orbitList_ECEF[s_i].vel.x);
						matJ2000Vel.SetElement(1,0,it->second.orbitList_ECEF[s_i].vel.y);
						matJ2000Vel.SetElement(2,0,it->second.orbitList_ECEF[s_i].vel.z);
						it->second.orbitList_ECEF[s_i].t = TimeCoordConvert::TDT2GPST(it->second.orbitList_ECEF[s_i].t);
						Matrix matPR_NR, matER, matEP, matER_DOT;
						m_TimeCoordConvert.Matrix_J2000_ECEF(it->second.orbitList_ECEF[s_i].t, matPR_NR, matER, matEP, matER_DOT);
						Matrix matEst_EP, matEst_ER;
						paraSatOrbEst.getEst_EOP(it->second.orbitList_ECEF[s_i].t, matEst_EP, matEst_ER);// 更新 matEP, matER
						matEP = matEst_EP * matEP;
						matER = matEst_ER * matER; // 2013/04/24, 原程序漏乘 matEst_ER
						matECFPos = matPR_NR * matJ2000Pos;
						matECFVel = matPR_NR * matJ2000Vel;
						matECFVel = matER *  matECFVel + matER_DOT * matECFPos;
						matECFPos = matER *  matECFPos;
						matECFPos = matEP *  matECFPos;
						matECFVel = matEP *  matECFVel;
						it->second.orbitList_ECEF[s_i].pos.x = matECFPos.GetElement(0, 0); 
						it->second.orbitList_ECEF[s_i].pos.y = matECFPos.GetElement(1, 0); 
						it->second.orbitList_ECEF[s_i].pos.z = matECFPos.GetElement(2, 0);
						it->second.orbitList_ECEF[s_i].vel.x = matECFVel.GetElement(0, 0); 
						it->second.orbitList_ECEF[s_i].vel.y = matECFVel.GetElement(1, 0); 
						it->second.orbitList_ECEF[s_i].vel.z = matECFVel.GetElement(2, 0);
					}
				}
				ndtSp3File.m_data.resize(paraSatOrbEst.satParaList.begin()->second.orbitList_ECEF.size());
				for(size_t s_i = 0; s_i < ndtSp3File.m_data.size(); s_i++)
				{
					ndtSp3File.m_data[s_i].t = paraSatOrbEst.satParaList.begin()->second.orbitList_ECEF[s_i].t;
					for(SatDatumMap::iterator it = paraSatOrbEst.satParaList.begin(); it != paraSatOrbEst.satParaList.end(); ++it)
					{
						SP3Datum datum;
						datum.pos = it->second.orbitList_ECEF[s_i].pos * 0.001;
						datum.vel = it->second.orbitList_ECEF[s_i].vel * 0.001;
						datum.clk = 0;
						datum.clkrate = 0;
						char SatName[4];
						sprintf(SatName, "C%02d", it->first);
						ndtSp3File.m_data[s_i].sp3.insert(SP3SatMap::value_type(SatName, datum));
					}
				}
				if(!ndtSp3File.write(outputSp3FilePath))
				{
					sprintf(info,"精密轨道产品文件输出失败！");
					RuningInfoFile::Add(info);
					printf("%s",info);	
				}
			}
			// 输出定轨结果 TRO 文件			
			// 3. TRO文件
			if(result)
			{
				char  outputTROFilePath[300];
				GPST  t_tro = t0 + (t1 - t0)/2;
				WeekTime  GPSWT= TimeCoordConvert::GPST2WeekTime(t_tro);
				int WeekDay = (int)floor(GPSWT.second/86400.0);
				sprintf(outputTROFilePath,"%s\\NDT%04d%1d.TRO",folder.c_str(),GPSWT.week,WeekDay);
				TROZPDFile         zpdFile;							
				UTC  t_start   = m_TimeCoordConvert.TAI2UTC(m_TimeCoordConvert.GPST2TAI(t0));
				UTC  t_end     = m_TimeCoordConvert.TAI2UTC(m_TimeCoordConvert.GPST2TAI(t1));						
				DayTime T_now;
				T_now.Now();	// 获取当前系统时间
				//输出TRO头文件信息
				zpdFile.m_header.szFirstchar[0]='%';
				zpdFile.m_header.szFirstchar[1]='\0';
				sprintf(zpdFile.m_header.szSecondchar,"=");	
				sprintf(zpdFile.m_header.szDocType,"TRO");
				zpdFile.m_header.Version = 1.00;
				sprintf(zpdFile.m_header.szFileAgency,"NDT");
				zpdFile.m_header.FileTime.year = T_now.year%100;
				zpdFile.m_header.FileTime.doy  = T_now.doy();
				zpdFile.m_header.FileTime.second = int(T_now.hour * 3600 + T_now.minute * 60 + T_now.second);
				sprintf(zpdFile.m_header.szDataAgency,"IGS");
				zpdFile.m_header.StartTimeSolut.year = t_start.year%100;
				zpdFile.m_header.StartTimeSolut.doy  = t_start.doy();
				zpdFile.m_header.StartTimeSolut.second = int(t_start.hour * 3600 + t_start.minute * 60 + t_start.second);
				zpdFile.m_header.EndTimeSolut.year = t_end.year%100;
				zpdFile.m_header.EndTimeSolut.doy  = t_end.doy();
				zpdFile.m_header.EndTimeSolut.second = int(t_end.hour * 3600 + t_end.minute * 60 + t_end.second);
				sprintf(zpdFile.m_header.szObsCode,"P"); 
				sprintf(zpdFile.m_header.szSolutCont,"%-4s","MIX");
				//输出FILE/REFERENCE BLOCK
				zpdFile.m_data.m_FileRef.bBlockUse = true;
				sprintf(zpdFile.m_data.m_FileRef.szDesInfo,"%-60s","NDT,National University of Defense Technology,China");
				sprintf(zpdFile.m_data.m_FileRef.szOutputInfo,"%-60s","Total Troposphere Zenith Path Delay Product");
				sprintf(zpdFile.m_data.m_FileRef.szContactInfo,"%-60s","gudefeng_nudt@163.com");
				sprintf(zpdFile.m_data.m_FileRef.szSoftwareInfo,"%-60s","NUDTTK");
				sprintf(zpdFile.m_data.m_FileRef.szInputInfo,"%-60s","GFZ final BDS orbit and clock solutions");
				//输出TROP/DESCRIPTION BLOCK
				zpdFile.m_data.m_TroDes.bMapFunc  = true;
				zpdFile.m_data.m_TroDes.eleCutoff = int(m_podParaDefine.min_elevation);
				zpdFile.m_data.m_TroDes.sampInterval = m_podParaDefine.sampleSpan;
				zpdFile.m_data.m_TroDes.troInterval  = m_podParaDefine.zpdProductInterval;
				zpdFile.m_data.m_TroDes.pstrSolutField = zpdFile.m_data.m_TroDes.pstrSolutFieldNor;
				sprintf(zpdFile.m_data.m_TroDes.szMapFunc,"%-22s","GMF apriori,1/sin(e) est");
				//输出INPUT/ACKNOWLEDGMENTS BLOCK
				InputAck  ack;
				sprintf(ack.szAgencyCode,"IGS");
				sprintf(ack.szAgencyDes,"%-75s","International GNSS Service");
				zpdFile.m_data.m_InputAck.push_back(ack);
				//输出TROP/STA_COORDINATES BLOCK
				for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)
				{
					char staname[4 + 1];
					sprintf(staname,"%s",it->first.c_str());
					char StaName[4 + 1];
					for(int k = 0; k < 4; k++)
						StaName[k] = toCapital(staname[k]);
					StaName[4] = '\0';
					TroStaPos  staPos;
					staPos.pos.x = it->second.pos_Est.x;
					staPos.pos.y = it->second.pos_Est.y;
					staPos.pos.z = it->second.pos_Est.z;
					sprintf(staPos.szSiteCode,"%s",StaName);
					sprintf(staPos.szPointCode," A");
					sprintf(staPos.szSolutID,"   1");
					sprintf(staPos.szObsCode,"P");
					sprintf(staPos.szRefSys,"ITRF05");
					sprintf(staPos.szRemark,"NDT  ");
					
					zpdFile.m_data.m_StaPos.insert(TroStaPosMap::value_type(StaName,staPos));
				}
				//输出TROP/SOLUTION BLOCK
				t_start.hour   = 1;
				t_start.minute = 0;
				t_start.second = 0;
				GPST  t0_zpd  = m_TimeCoordConvert.UTC2GPST(t_start);
				vector<TroSolut>   zpdSolutlist;
				for(StaDatumMap::iterator it = m_mapStaDatum.begin(); it != m_mapStaDatum.end(); ++it)
				{
					char staname[4 + 1];
					sprintf(staname,"%s",it->first.c_str());
					char StaName[4 + 1];
					for(int k = 0; k < 4; k++)
						StaName[k] = toCapital(staname[k]);
					StaName[4] = '\0';
					zpdSolutlist.clear();
					BLH    blh;
					TimeCoordConvert::XYZ2BLH(it->second.pos_Est,blh);
					blh.B = blh.B * PI/180;//转化为弧度
					blh.L = blh.L * PI/180;	//	
					for( int i = 0;  t0_zpd + i * m_podParaDefine.zpdProductInterval - it->second.t1 <= 0; i ++)
					{
						if(t0_zpd + i * m_podParaDefine.zpdProductInterval - it->second.t0 >= 0)
						{	
							UTC t_epoch = m_TimeCoordConvert.TAI2UTC(m_TimeCoordConvert.GPST2TAI(t0_zpd + i * m_podParaDefine.zpdProductInterval));
							// 计算对流层延迟先验值
							double dmjd = TimeCoordConvert::DayTime2MJD(t_epoch);						
							double pressure,temperature,undu,zpdh,zpdw;						
							GlobalPT(dmjd,blh.B,blh.L,blh.H,pressure,temperature,undu);
							Saastamoinen_model(temperature,50,pressure,blh.B,blh.H + it->second.arpAnt.U - undu,zpdh,zpdw);						
							
							//计算湿分量估计值
							double zpdw_est = 0;
							GPST t_gps = m_TimeCoordConvert.UTC2GPST(t_epoch);
							int indexTZD = it->second.getIndexZenithDelayEstList(t_gps);
							double u = (t_gps - it->second.zenithDelayEstList[indexTZD].t)/(it->second.zenithDelayEstList[indexTZD + 1].t - it->second.zenithDelayEstList[indexTZD].t);
							zpdw_est = (it->second.zenithDelayEstList[indexTZD + 1].zenithDelay_Est
										- it->second.zenithDelayEstList[indexTZD].zenithDelay_Est) * u
										+ it->second.zenithDelayEstList[indexTZD].zenithDelay_Est;	
							TroSolut   tro_epoch;
							tro_epoch.EpochTime.year = t_epoch.year%100;
							tro_epoch.EpochTime.doy  = t_epoch.doy();
							tro_epoch.EpochTime.second = int(t_epoch.hour * 3600 + t_epoch.minute * 60 + t_epoch.second);
							sprintf(tro_epoch.szMarker,"%s",StaName);																		
							tro_epoch.TROTOT = (zpdh + zpdw + zpdw_est) * 1000;//单位统一为mm
							zpdSolutlist.push_back(tro_epoch);
						}
					}

					zpdFile.m_data.m_TroSolut.insert(TroSolutMap::value_type(StaName,zpdSolutlist));
				}
				if(!zpdFile.write(outputTROFilePath))
				{
					sprintf(info,"对流层产品文件输出失败！");
					RuningInfoFile::Add(info);
					printf("%s",info);	
				}
			}
			return result;
		}
	}
}
