#include "TQSatNetDynPOD.hpp"
#include "GNSSBasicCorrectFunc.hpp"
#include "SolidTides.hpp"
#include "OceanTidesLoading.hpp"

namespace NUDTTK
{
	namespace TQPod
	{
	
	    // 子程序名称： getEphemeris   
		// 功能：滑动lagrange插值获得任意时刻BD卫星星历
		// 变量类型： t                     :  UTC北京时
		//            interpOrbit           :  星历数值, 坐标单位: 米
		//            nLagrange             :  Lagrange 插值已知点个数, 默认为 9, 对应 8 阶 Lagrange 插值
		// 输入：t,  nLagrange
		// 输出：interpOrbit
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2012/12/31
		// 版本时间：
		// 修改记录：
		// 备注： 
		bool TQNETPOD_SatDatum::getEphemeris(UTC t, TimePosVel& interpOrbit, int nLagrange)
		{
			size_t count_ac = acOrbitList.size();
			const int nlagrange = 8; 
			if(count_ac < nlagrange) // 如果数据点个数小于nlagrange返回, 要求弧段长度 > h * nlagrange = 4分钟
				return false;
			double h = acOrbitList[1].t - acOrbitList[0].t;
			double spanSecond_t = t - acOrbitList[0].t;  // 相对观测时间, 初始时间为 orbitlist_ac[0].t
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
			interpOrbit.t = t;
			double *x = new double [nlagrange];
			double *y = new double [nlagrange];
			for(int i = nBegin; i <= nEnd; i++)
				x[i - nBegin] = acOrbitList[i].t - acOrbitList[0].t; // 参考相对时间点
			// X
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = acOrbitList[i].pos.x;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.pos.x);
			// Y
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = acOrbitList[i].pos.y;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.pos.y);
			// Z
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = acOrbitList[i].pos.z;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.pos.z);
			// Vx
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = acOrbitList[i].vel.x;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.x);
			// Vy
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = acOrbitList[i].vel.y;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.y);
			// Vz
			for(int i = nBegin; i <= nEnd; i++)
				y[i - nBegin] = acOrbitList[i].vel.z;
			InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.z);
			delete x;
			delete y;
			return true;
		}
		// 待更新
		bool TQNETPOD_SatDatum::getInterpRtPartial(UTC t, Matrix& interpRtPartial)
		{// 李康康 2019/09/05 测试代码
			bool bResult = true;
			const int countDynParameter = acRtPartialList[0].GetNumColumns();
			interpRtPartial.Init(3, countDynParameter);

			double spanSecond_t = t - acOrbitList[0].t; 
			double h = acOrbitList[1].t - acOrbitList[0].t;
			int nLeftPos  = int(spanSecond_t / h); // 首先寻找最靠近时间 T 的左端点, 从 0 开始计数
			int nBegin, nEnd;
			// 偏导数插值, 阶数2, 线性插值, 2008/06/27
			if(nLeftPos < 0) // nEnd - nBegin = nLagrange - 1 
			{
				nBegin = 0;
				nEnd   = 1;
				bResult = false;
			}
			else if(nLeftPos + 1 >= int(acRtPartialList.size()))
			{
				nBegin = int(acRtPartialList.size()) - 2;
				nEnd   = int(acRtPartialList.size()) - 1;
				bResult = false;
			}
			else
			{
				nBegin = nLeftPos;
				nEnd   = nLeftPos + 1;
			}
			double x_t[2];
			double y_t[2];
			x_t[0] = acOrbitList[nBegin].t - acOrbitList[0].t;
			x_t[1] = acOrbitList[nEnd].t   - acOrbitList[0].t;
			double u = (spanSecond_t - x_t[0])/(x_t[1] - x_t[0]);
			for(int ii = 0; ii < 3; ii++)
			{
				for(int jj = 0; jj < int(countDynParameter); jj++)
				{// 对矩阵的每个元素[ii, jj]进行插值
					y_t[0] = acRtPartialList[nBegin].GetElement(ii, jj);
					y_t[1] = acRtPartialList[nEnd].GetElement(ii, jj);
					double element = u * y_t[1] + (1 - u) * y_t[0];
					interpRtPartial.SetElement(ii, jj, element);
				}
			}
			return bResult;
		}
        // 子程序名称： getEphemeris_PathDelay   
		// 功能：根据接收机的概略位置、信号接收时间和卫星的先验轨道,
		//       计算卫星信号传播延迟时间(即卫星“准确的”信号反射时间)
		// 变量类型： t                  : 信号接收时间
		//            staPos             : 接收机概略位置, 单位：米
		//            delay              : 信号传播延迟时间, 单位：秒
		//            tqOrb              : 确定了正确的信号发射时间后, 顺便返回本颗BD卫星星历
		//            tqRtPartial        : 卫星的偏导数
		//            threshold          : 迭代阈值，默认 1.0E-007
		// 输入：t, staPos, m_acOrbitList, m_acRtPartialList, threshold
		// 输出：tqOrb, tqRtPartial
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2012/12/31
		// 版本时间：2017/11/02
		// 修改记录：
		// 备注： 
		bool TQNETPOD_SatDatum::getEphemeris_PathDelay(UTC t, POS3D staPos, double& delay, TimePosVel& tqOrb, Matrix& tqRtPartial, double threshold)
		{
			// 信号真实接收时间 = 观测时间(T) - 接收机钟差(receiverPos.dClock)
			UTC t_Receive  = t;
			UTC t_Transmit = t_Receive; // 初始化信号转发时间
			if(!getEphemeris(t_Transmit, tqOrb))
				return false;
			double distance = pow(staPos.x - tqOrb.pos.x, 2)
							+ pow(staPos.y - tqOrb.pos.y, 2)
							+ pow(staPos.z - tqOrb.pos.z, 2);
			distance = sqrt(distance); // 获得信号发射传播距离
			double delay_k_1 = 0;
			delay = distance / SPEED_LIGHT;  // 获得信号反射传播延迟
			//const double delay_max  = 1.0;   // 为了防止迭代dDelay溢出，这里设置一个阈值
			//const int    k_max      = 5;     // 迭代次数阈值，一般1次迭代就会收敛
			const double delay_max  = 300.0;   // 为了防止迭代dDelay溢出，这里设置一个阈值
			const int    k_max      = 10;     // 迭代次数阈值，一般1次迭代就会收敛
			int          k          = 0;
			while(fabs(delay - delay_k_1) > threshold)   // 迭代阈值控制, abs-->fabs, 2007/07/15
			{
				k++;
				if(fabs(delay) > delay_max || k > k_max) // 为防止 delay 溢出, 2007/04/06
				{
					//printf("%d %d %f delay 迭代发散!\n", t.hour, t.minute, t.second);
					printf("%s delay  迭代发散!\n",t.toString().c_str());
					return false;
				}
				// 更新信号反射时间
				t_Transmit = t_Receive - delay;
				if(!getEphemeris(t_Transmit, tqOrb))
					return false;
				// 更新概略距离
				distance =  pow(staPos.x - tqOrb.pos.x, 2)
						  + pow(staPos.y - tqOrb.pos.y, 2)
						  + pow(staPos.z - tqOrb.pos.z, 2);
				distance = sqrt(distance);
				// 更新延迟数据
				delay_k_1 = delay;
				delay = distance / SPEED_LIGHT;
			}
			if(int(acRtPartialList.size()) < 2) 
				return false;
			
			const int countDynParameter = acRtPartialList[0].GetNumColumns(); 
			tqRtPartial.Init(3, countDynParameter);

			double spanSecond_t = t_Transmit - acOrbitList[0].t; 
			double h = acOrbitList[1].t - acOrbitList[0].t;
			int nLeftPos  = int(spanSecond_t / h); // 首先寻找最靠近时间 T 的左端点, 从 0 开始计数
			int nBegin, nEnd; 
			// 偏导数插值, 阶数2, 线性插值, 2008/06/27
			if(nLeftPos < 0) // nEnd - nBegin = nLagrange - 1 
			{
				nBegin = 0;
				nEnd   = 1;
			}
			else if(nLeftPos + 1 >= int(acRtPartialList.size()))
			{
				nBegin = int(acRtPartialList.size()) - 2;
				nEnd   = int(acRtPartialList.size()) - 1;
			}
			else
			{
				nBegin = nLeftPos;
				nEnd   = nLeftPos + 1;
			}
			double x_t[2];
			double y_t[2];
			x_t[0] = acOrbitList[nBegin].t - acOrbitList[0].t;
			x_t[1] = acOrbitList[nEnd].t   - acOrbitList[0].t;
			double u = (spanSecond_t - x_t[0])/(x_t[1] - x_t[0]);
			for(int ii = 0; ii < 3; ii++)
			{
				for(int jj = 0; jj < int(countDynParameter); jj++)
				{// 对矩阵的每个元素[ii, jj]进行插值
					y_t[0] = acRtPartialList[nBegin].GetElement(ii, jj);
					y_t[1] = acRtPartialList[nEnd].GetElement(ii, jj);
					double element = u * y_t[1] + (1 - u) * y_t[0];
					tqRtPartial.SetElement(ii, jj, element);
				}
			}
			return true;
		}

		// 子程序名称： getEstParameters_n0   
		// 功能：计算待估参数初始位置: n0_SolarPressure、n0_EmpiricalForce、n0_ManeuverForce
		//       统计参数个数(不包括模糊度参数个数): count_EstDynParameters 
		// 变量类型：
		// 输入：
		// 输出：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：
		// 版本时间：2018/06/22
		// 修改记录：
		// 备注：
		void TQNETPOD_SatDatum::getEstParameters_n0()
		{
			// 动力学参数个数统计
			//count_EstDynParameters = dynDatum_Est.getAllEstParaCount(); // 包含所有动力学相关参数
			n0_SolarPressure  = 0; // 记录太阳光压参数起始位置
			n0_EmpiricalForce = 0; // 记录经验力参数在起始位置
			n0_ManeuverForce  = 0; // 记录机动力参数在起始位置

			int beginPara = 6;
			n0_SolarPressure = beginPara;
			if( dynDatum_Est.bOn_SolarPressureAcc
			&& (dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_1PARA
			||  dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_1PARA_AM
			||  dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_MACRO))
			{// 太阳光压
				beginPara += int(dynDatum_Est.solarPressureParaList.size());
			}
			else if( dynDatum_Est.bOn_SolarPressureAcc
			&& (dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_9PARA))
			{// 太阳光压
				beginPara += int(dynDatum_Est.solarPressureParaList.size()) * 9;
			}
			else if( dynDatum_Est.bOn_SolarPressureAcc
			&& (dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_5PARA))
			{// 太阳光压
				beginPara += int(dynDatum_Est.solarPressureParaList.size()) * 5;
			}

			n0_EmpiricalForce = beginPara;
			if(dynDatum_Est.bOn_EmpiricalForceAcc
			&& dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
			{// 经验力
				int count_sub =  + int(dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2  
								 + int(dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2  
								 + int(dynDatum_Est.bOn_EmpiricalForceAcc_N) * 2; 
				beginPara += count_sub * int(dynDatum_Est.empiricalForceParaList.size());
			}
			else if(dynDatum_Est.bOn_EmpiricalForceAcc
				 && dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_SPLINE)
			{
				int count_sub =  + int(dynDatum_Est.bOn_EmpiricalForceAcc_R) * 1  
								 + int(dynDatum_Est.bOn_EmpiricalForceAcc_T) * 1  
								 + int(dynDatum_Est.bOn_EmpiricalForceAcc_N) * 1;
				beginPara += count_sub * (int(dynDatum_Est.empiricalForceParaList.size()) + 1);
			}
			n0_ManeuverForce = beginPara;
			if(dynDatum_Est.bOn_ManeuverForceAcc
			&& dynDatum_Est.bOnEst_Maneuver)
			{
				beginPara += 3 * int(dynDatum_Est.maneuverForceParaList.size());
			}
		}

		void TQNETPOD_SatDatum::ocResOutput()
		{
			count_obs = 0;
			ocResRMS_Range = 0.0;
			ocResRMS_Doppler = 0.0;
			for(TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = mapUXBStaDatum.begin(); it_UXBSta != mapUXBStaDatum.end(); ++it_UXBSta)
			{
				for(map<UTC, TQUXBObsLine>::iterator jt_obs = it_UXBSta->second.obsList.begin(); jt_obs != it_UXBSta->second.obsList.end(); ++jt_obs)
				{
					if (jt_obs->second.rw_range == 1.0
					&&  jt_obs->second.rw_doppler == 1.0)
					{
						count_obs++;
						ocResRMS_Range += pow(jt_obs->second.oc_range * jt_obs->second.weight_range, 2);  
						ocResRMS_Doppler += pow(jt_obs->second.oc_doppler * jt_obs->second.weight_doppler, 2);
					}
				}
			}
			if(count_obs > 0)
			{
				ocResRMS_Range   = sqrt(ocResRMS_Range / count_obs);
				ocResRMS_Doppler = sqrt(ocResRMS_Doppler / count_obs);
			}
		}

		void TQNETPOD_SatDatum::ocResEdit(double factor)
		{
			for(TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = mapUXBStaDatum.begin(); it_UXBSta != mapUXBStaDatum.end(); ++it_UXBSta)
			{
				for(map<UTC, TQUXBObsLine>::iterator jt_obs = it_UXBSta->second.obsList.begin(); jt_obs != it_UXBSta->second.obsList.end(); ++jt_obs)
				{
					if(fabs(jt_obs->second.oc_range * jt_obs->second.weight_range) > ocResRMS_Range *  factor)
						jt_obs->second.rw_range = ocResRMS_Range / fabs(jt_obs->second.oc_range * jt_obs->second.weight_range);
					else
		                jt_obs->second.rw_range = 1.0;


					if(fabs(jt_obs->second.oc_doppler * jt_obs->second.weight_doppler) > ocResRMS_Doppler *  factor)
						jt_obs->second.rw_doppler = ocResRMS_Doppler / fabs(jt_obs->second.oc_doppler * jt_obs->second.weight_doppler);
					else
		                jt_obs->second.rw_doppler = 1.0;
				}
			}
		}

		// 子程序名称： updateDynDatum   
		// 功能：记录改进后的轨道dynDatum_Est
		// 变量类型：
		// 输入：
		// 输出：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：
		// 版本时间：2019/02/14
		// 修改记录：
		// 备注：
		void TQNETPOD_SatDatum::updateDynDatum()
		{
			dynDatum_Est.X0.x  += dx.GetElement(0,0);
			dynDatum_Est.X0.y  += dx.GetElement(1,0);
			dynDatum_Est.X0.z  += dx.GetElement(2,0);
			dynDatum_Est.X0.vx += dx.GetElement(3,0);
			dynDatum_Est.X0.vy += dx.GetElement(4,0);
			dynDatum_Est.X0.vz += dx.GetElement(5,0);
			if(dynDatum_Est.bOn_SolarPressureAcc && dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
			{// 太阳光压
				for(int s_k = 0; s_k < int(dynDatum_Est.solarPressureParaList.size()); s_k++)
					dynDatum_Est.solarPressureParaList[s_k].Cr +=  dx.GetElement(n0_SolarPressure + s_k, 0);
			}
			// 2015/10/18, 兼容多光压模型计算, 谷德峰
			else if(dynDatum_Est.bOn_SolarPressureAcc && dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
			{
				for(int s_k = 0; s_k < int(dynDatum_Est.solarPressureParaList.size()); s_k++)
				{
					dynDatum_Est.solarPressureParaList[s_k].D0  += dx.GetElement(n0_SolarPressure + s_k * 9 + 0, 0);
					dynDatum_Est.solarPressureParaList[s_k].DC1 += dx.GetElement(n0_SolarPressure + s_k * 9 + 1, 0);
					dynDatum_Est.solarPressureParaList[s_k].DS1 += dx.GetElement(n0_SolarPressure + s_k * 9 + 2, 0);
					dynDatum_Est.solarPressureParaList[s_k].Y0  += dx.GetElement(n0_SolarPressure + s_k * 9 + 3, 0);
					dynDatum_Est.solarPressureParaList[s_k].YC1 += dx.GetElement(n0_SolarPressure + s_k * 9 + 4, 0);
					dynDatum_Est.solarPressureParaList[s_k].YS1 += dx.GetElement(n0_SolarPressure + s_k * 9 + 5, 0);
					dynDatum_Est.solarPressureParaList[s_k].B0  += dx.GetElement(n0_SolarPressure + s_k * 9 + 6, 0);
					dynDatum_Est.solarPressureParaList[s_k].BC1 += dx.GetElement(n0_SolarPressure + s_k * 9 + 7, 0);
					dynDatum_Est.solarPressureParaList[s_k].BS1 += dx.GetElement(n0_SolarPressure + s_k * 9 + 8, 0);
				}
			}
			else if(dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_5PARA)	
			{
				for(int s_k = 0; s_k < int(dynDatum_Est.solarPressureParaList.size()); s_k++)
				{
					dynDatum_Est.solarPressureParaList[s_k].D0  += dx.GetElement(n0_SolarPressure + 5 * s_k,     0);
					dynDatum_Est.solarPressureParaList[s_k].Y0  += dx.GetElement(n0_SolarPressure + 5 * s_k + 1, 0);
					dynDatum_Est.solarPressureParaList[s_k].B0  += dx.GetElement(n0_SolarPressure + 5 * s_k + 2, 0);
					dynDatum_Est.solarPressureParaList[s_k].BC1 += dx.GetElement(n0_SolarPressure + 5 * s_k + 3, 0);
					dynDatum_Est.solarPressureParaList[s_k].BS1 += dx.GetElement(n0_SolarPressure + 5 * s_k + 4, 0);
				}
			}
			if(dynDatum_Est.bOn_EmpiricalForceAcc && dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
			{// 经验力
				int count_sub =  + int(dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
								 + int(dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2  
								 + int(dynDatum_Est.bOn_EmpiricalForceAcc_N) * 2; 
				for(int s_k = 0; s_k < int(dynDatum_Est.empiricalForceParaList.size()); s_k++)
				{
					int i_sub = 0;
					if(dynDatum_Est.bOn_EmpiricalForceAcc_R)
					{
						dynDatum_Est.empiricalForceParaList[s_k].cos_R += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 0, 0);
						dynDatum_Est.empiricalForceParaList[s_k].sin_R += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 1, 0);
					}
					i_sub = int(dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2;
					if(dynDatum_Est.bOn_EmpiricalForceAcc_T)
					{
						dynDatum_Est.empiricalForceParaList[s_k].cos_T += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 0, 0);
						dynDatum_Est.empiricalForceParaList[s_k].sin_T += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 1, 0);
					}
					i_sub = int(dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2 + int(dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2;
                    if(dynDatum_Est.bOn_EmpiricalForceAcc_N)
					{
						dynDatum_Est.empiricalForceParaList[s_k].cos_N += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 0, 0);
						dynDatum_Est.empiricalForceParaList[s_k].sin_N += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 1, 0);
					}
				}
			}
			else if(dynDatum_Est.bOn_EmpiricalForceAcc && dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
			{// 经验力
				int count_sub =  + int(dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
								 + int(dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3  
								 + int(dynDatum_Est.bOn_EmpiricalForceAcc_N) * 3; 
				for(int s_k = 0; s_k < int(dynDatum_Est.empiricalForceParaList.size()); s_k++)
				{
					int i_sub = 0;
					if(dynDatum_Est.bOn_EmpiricalForceAcc_R)
					{
						dynDatum_Est.empiricalForceParaList[s_k].a0_R  += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 0, 0);
						dynDatum_Est.empiricalForceParaList[s_k].cos_R += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 1, 0);
						dynDatum_Est.empiricalForceParaList[s_k].sin_R += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 2, 0);
					}
					i_sub = int(dynDatum_Est.bOn_EmpiricalForceAcc_R) * 1;
					if(dynDatum_Est.bOn_EmpiricalForceAcc_T)
					{
						dynDatum_Est.empiricalForceParaList[s_k].a0_T  += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 0, 0);
						dynDatum_Est.empiricalForceParaList[s_k].cos_T += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 1, 0);
						dynDatum_Est.empiricalForceParaList[s_k].sin_T += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 2, 0);
					}
					i_sub = int(dynDatum_Est.bOn_EmpiricalForceAcc_R) * 1 + int(dynDatum_Est.bOn_EmpiricalForceAcc_T) * 1;
                    if(dynDatum_Est.bOn_EmpiricalForceAcc_N)
					{
						dynDatum_Est.empiricalForceParaList[s_k].a0_N  += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 0, 0);
						dynDatum_Est.empiricalForceParaList[s_k].cos_N += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 1, 0);
						dynDatum_Est.empiricalForceParaList[s_k].sin_N += dx.GetElement(n0_EmpiricalForce + s_k * count_sub + i_sub + 2, 0);
					}
				}
			}

			// 卫星延迟参数
			if(count_EstSatZeroDelay > 0)
				satZeroDelay_Est += dx.GetElement(count_EstDynParameters, 0);

			// 测站延迟参数
			if(count_EstStaZeroDelay > 0)
			{
				for(TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = mapUXBStaDatum.begin(); it_UXBSta != mapUXBStaDatum.end(); ++it_UXBSta)
				{
					it_UXBSta->second.zeroDelay_Est += dx.GetElement(count_EstDynParameters + count_EstSatZeroDelay + it_UXBSta->second.id_ZeroDelay, 0);
				}
			}

		}

		void TQNETPOD_SatDatum::writeDynDatum(string pathDynPodFitFile)
		{
			/*if(!pathDynPodFitFile.empty())*/
			{
				// 后续请李康康整理 ?????? 

				// 记录轨道改进结果，每个卫星的改进结果分别保存到对应的文件
				FILE * pFitFile = fopen(pathDynPodFitFile.c_str(), "w+");
				fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
				int k_Parameter = 0;
				fprintf(pFitFile, "%3d. %-4s Delay(m)      %20.4f%10.4f%20.4f\n", k_Parameter,  satName.c_str(),  satZeroDelay_0,  satZeroDelay_Est,  satZeroDelay_0 + satZeroDelay_Est);
				k_Parameter++;
				for(TQNETPOD_UXBStaDatumMap::iterator jt_obs = mapUXBStaDatum.begin(); jt_obs != mapUXBStaDatum.end();++jt_obs)
                {
					k_Parameter++;
					fprintf(pFitFile, "%3d. %-4s Delay(m)      %20.4f%10.4f%20.4f\n", k_Parameter, jt_obs->first.c_str(), jt_obs->second.zeroDelay_0,  jt_obs->second.zeroDelay_Est,  jt_obs->second.zeroDelay_0 + jt_obs->second.zeroDelay_Est);
				}   
				fprintf(pFitFile, "%3d.      X    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 1,dynDatum_Init.X0.x,  dynDatum_Est.X0.x  - dynDatum_Init.X0.x,  dynDatum_Est.X0.x);
				fprintf(pFitFile, "%3d.      Y    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 2,dynDatum_Init.X0.y,  dynDatum_Est.X0.y  - dynDatum_Init.X0.y,  dynDatum_Est.X0.y);
				fprintf(pFitFile, "%3d.      Z    (m)      %20.4f%10.4f%20.4f\n", k_Parameter + 3,dynDatum_Init.X0.z,  dynDatum_Est.X0.z  - dynDatum_Init.X0.z,  dynDatum_Est.X0.z);
				fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 4,dynDatum_Init.X0.vx, dynDatum_Est.X0.vx - dynDatum_Init.X0.vx, dynDatum_Est.X0.vx);
				fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 5,dynDatum_Init.X0.vy, dynDatum_Est.X0.vy - dynDatum_Init.X0.vy, dynDatum_Est.X0.vy);
				fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f%10.4f%20.4f\n", k_Parameter + 6,dynDatum_Init.X0.vz, dynDatum_Est.X0.vz - dynDatum_Init.X0.vz, dynDatum_Est.X0.vz);
				k_Parameter += 6;
				if(dynDatum_Est.bOn_SolarPressureAcc && dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
				{
					for(size_t s_i = 0; s_i < dynDatum_Est.solarPressureParaList.size(); s_i++)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   CR            %20.4f%10.4f%20.4f\n", k_Parameter ,
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].Cr,
																						   dynDatum_Est.solarPressureParaList[s_i].Cr - dynDatum_Init.solarPressureParaList[s_i].Cr,
																						   dynDatum_Est.solarPressureParaList[s_i].Cr);
					}
				}
				if(dynDatum_Est.bOn_SolarPressureAcc && (dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_9PARA || dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_5PARA))
				{
					for(size_t s_i = 0; s_i < dynDatum_Est.solarPressureParaList.size(); s_i++)
					{
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   D0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].D0 * 1.0E+7 - dynDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].D0 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   DCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].DC1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].DC1 * 1.0E+7 - dynDatum_Init.solarPressureParaList[s_i].DC1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].DC1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   DSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].DS1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].DS1 * 1.0E+7 - dynDatum_Init.solarPressureParaList[s_i].DS1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].DS1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   Y0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].Y0 * 1.0E+7 - dynDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].Y0 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   YCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].YC1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].YC1 * 1.0E+7 - dynDatum_Init.solarPressureParaList[s_i].YC1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].YC1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   YSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].YS1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].YS1 * 1.0E+7 - dynDatum_Init.solarPressureParaList[s_i].YS1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].YS1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   B0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].B0 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].B0 * 1.0E+7 - dynDatum_Init.solarPressureParaList[s_i].B0 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].B0 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   BCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].BC1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].BC1 * 1.0E+7 - dynDatum_Init.solarPressureParaList[s_i].BC1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].BC1 * 1.0E+7);
						k_Parameter++;
						fprintf(pFitFile, "%3d. %2d   BSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																						   s_i+1,
																						   dynDatum_Init.solarPressureParaList[s_i].BS1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].BS1 * 1.0E+7 - dynDatum_Init.solarPressureParaList[s_i].BS1 * 1.0E+7, 
																						   dynDatum_Est.solarPressureParaList[s_i].BS1 * 1.0E+7);
					}
				}
				if(dynDatum_Est.bOn_EmpiricalForceAcc && dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
				{
					for(size_t s_i = 0; s_i < dynDatum_Est.empiricalForceParaList.size(); s_i++)
					{// 20140320, 谷德峰修改, 使得三个方向经验力可选
						if(dynDatum_Est.bOn_EmpiricalForceAcc_R)
						{
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_R * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_R * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_R * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_R * 1.0E+7);
						}
						if(dynDatum_Est.bOn_EmpiricalForceAcc_T)
						{
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   TCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_T * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_T * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   TSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_T * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_T * 1.0E+7);
						}
						if(dynDatum_Est.bOn_EmpiricalForceAcc_N)
						{
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   NCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_N * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_N * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   NSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_N * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_N * 1.0E+7);
						}
					}
				}
				// 20170427, 谷德峰修改, 增加常值项经验力
				if(dynDatum_Est.bOn_EmpiricalForceAcc && dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
				{
					for(size_t s_i = 0; s_i < dynDatum_Est.empiricalForceParaList.size(); s_i++)
					{// 20140320, 谷德峰修改, 使得三个方向经验力可选
						if(dynDatum_Est.bOn_EmpiricalForceAcc_R)
						{
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   R_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].a0_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].a0_R * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].a0_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].a0_R * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_R * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_R * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_R * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_R * 1.0E+7);
						}
						if(dynDatum_Est.bOn_EmpiricalForceAcc_T)
						{
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   T_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].a0_T * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].a0_T * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   TCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_T * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_T * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   TSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_T * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_T * 1.0E+7);
						}
						if(dynDatum_Est.bOn_EmpiricalForceAcc_N)
						{
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   N_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].a0_N * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].a0_N * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   NCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_N * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].cos_N * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   NSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																							   s_i+1,
																							   dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_N * 1.0E+7 - dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																							   dynDatum_Est.empiricalForceParaList[s_i].sin_N * 1.0E+7);
						}
					}
				}
				fclose(pFitFile);

			}
		}
		

		TQSatNetDynPOD::TQSatNetDynPOD(void)
		{
			m_stepAdamsCowell = 120.0;
		}

		TQSatNetDynPOD::~TQSatNetDynPOD(void)
		{
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
		bool TQSatNetDynPOD::adamsCowell_ac(UTC t0, UTC t1, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h, int q)
		{
			orbitlist_ac.clear();
			matRtPartiallist_ac.clear();
			TDT t_Begin = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t0)); // t0 - 3600.0 * 8
			TDT t_End   = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t1)); // t1 - 3600.0 * 8
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
			// TDT -> UTC 北京时
			for(size_t s_i = 0; s_i < orbitlist_ac.size(); s_i++)
				orbitlist_ac[s_i].t = m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::TDT2TAI(orbitlist_ac[s_i].t)); // 转换到北京时  + 3600.0 * 8
			return true;
		}

		// 子程序名称： orbitExtrapolation   
		// 功能：轨道外推程序
		// 变量类型：dynamicDatum      : 初始动力学轨道参数
		//           t0_forecast       : 预报轨道初始时间, GPST
		//           t1_forecast       : 预报轨道终止时间, GPST
		//           forecastOrbList   : 预报轨道列表, 采用GPST, ITRF坐标系
		//           interval          : 预报轨道间隔
		// 输入：dynamicDatum, t0_forecast, t1_forecast,interval
		// 输出：forecastOrbList
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2009/08/14
		// 版本时间：
		// 修改记录：
		// 备注： 
		void TQSatNetDynPOD::orbitExtrapolation(SatdynBasicDatum dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval)
		{
			 vector<TimePosVelAcc> forecastOrbList_PVA;
			 orbitExtrapolation(dynamicDatum, t0_forecast, t1_forecast,  forecastOrbList_PVA, interval);
			 forecastOrbList.clear();
			 for(size_t s_i = 0; s_i < forecastOrbList_PVA.size(); s_i++)
			 {
				 forecastOrbList.push_back(forecastOrbList_PVA[s_i].getTimePosVel());
			 }
		}

		// 子程序名称： orbitExtrapolation   
		// 功能：轨道外推程序
		// 变量类型：dynamicDatum      : 初始动力学轨道参数
		//           t0_forecast       : 预报轨道初始时间, GPST
		//           t1_forecast       : 预报轨道终止时间, GPST
		//           forecastOrbList   : 预报轨道列表, 采用GPST, ITRF坐标系
		//           interval          : 预报轨道间隔
		// 输入：dynamicDatum, t0_forecast, t1_forecast,interval
		// 输出：forecastOrbList
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2009/08/14
		// 版本时间：
		// 修改记录：2018/04/15 由谷德峰修改, 地固系下加速度直接转换存在不匹配问题, 调整为先坐标转换再微分平滑求解
		// 备注： 
		void TQSatNetDynPOD::orbitExtrapolation(SatdynBasicDatum dynamicDatum, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVelAcc> &forecastOrbList, double interval)
		{
			// 进行轨道预报
			TDT t0_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t0_forecast));
			TDT t1_tdt = TimeCoordConvert::GPST2TDT(m_TimeCoordConvert.UTC2GPST(t1_forecast));
			vector<TimePosVel> orbitlist_ac;
			vector<Matrix> matRtPartiallist_ac;
			// 倒向积分, 积分区间 [para.T0, T_End   + h * 4], 为保证插值精度向两端进行扩展
			vector<TimePosVel> backwardOrbitlist_ac; 
		    vector<TimePosVel> forwardOrbitlist_ac; 
            double h = m_stepAdamsCowell; // 20150308, 谷德峰, 轨道积分步长修改为10.0秒
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
			// 先将orbitlist_ac转换到地固系
			// 转换到地球固定坐标系, 坐标系: ITRF 系, 时间: GPS
			for(size_t s_i = 0; s_i < orbitlist_ac.size(); s_i++)
			{
				double x_ecf[6];
				double x_j2000[6];
				x_j2000[0] = orbitlist_ac[s_i].pos.x;  
				x_j2000[1] = orbitlist_ac[s_i].pos.y;  
				x_j2000[2] = orbitlist_ac[s_i].pos.z;
				x_j2000[3] = orbitlist_ac[s_i].vel.x; 
				x_j2000[4] = orbitlist_ac[s_i].vel.y; 
				x_j2000[5] = orbitlist_ac[s_i].vel.z;
				orbitlist_ac[s_i].t = TimeCoordConvert::TDT2GPST(orbitlist_ac[s_i].t);
				m_TimeCoordConvert.J2000_ECEF(orbitlist_ac[s_i].t, x_j2000, x_ecf); // 将位置、速度转换到地固系下
				orbitlist_ac[s_i].pos.x = x_ecf[0]; 
				orbitlist_ac[s_i].pos.y = x_ecf[1]; 
				orbitlist_ac[s_i].pos.z = x_ecf[2];
				orbitlist_ac[s_i].vel.x = x_ecf[3]; 
				orbitlist_ac[s_i].vel.y = x_ecf[4]; 
				orbitlist_ac[s_i].vel.z = x_ecf[5];
			}
			// 然后再进行插值
			double span = t1_forecast - t0_forecast;
			forecastOrbList.clear();
			forecastOrbList.resize(int(span / interval) + 1);
			for(size_t s_i = 0; s_i < forecastOrbList.size(); s_i++)
			{
				TimePosVelAcc point;
				point.t = t0_forecast + s_i * interval;
				point.t = m_TimeCoordConvert.UTC2GPST(point.t); // 转换到GPST, 20190626，谷德峰修改
				forecastOrbList[s_i] = point;
			}
			size_t count_ac = orbitlist_ac.size();
			const int nlagrange = 8; 
			if(count_ac < nlagrange) // 如果数据点个数小于nlagrange返回, 要求弧段长度 > h * nlagrange = 4分钟
			{
				printf("弧段长度过短!\n");
				return;
			}
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
				TimePosVelAcc interpOrbit; // 所有元素的参考时刻均相同
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
				InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.x, interpOrbit.acc.x);
				// vy
				for(int i = nBegin; i <= nEnd; i++)
					y[i - nBegin] = orbitlist_ac[i].vel.y;
				InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.y, interpOrbit.acc.y);
				// vz
				for(int i = nBegin; i <= nEnd; i++)
					y[i - nBegin] = orbitlist_ac[i].vel.z;
				InterploationLagrange(x, y, nlagrange, spanSecond_t, interpOrbit.vel.z, interpOrbit.acc.z);
				forecastOrbList[s_i] = interpOrbit;
				delete x;
			    delete y;
			}

			// TDT -> UTC 北京时
			for(size_t s_i = 0; s_i < forecastOrbList.size(); s_i++)
				forecastOrbList[s_i].t = m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(forecastOrbList[s_i].t)); // 转换到GPST, 20190626，谷德峰修改
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
		bool TQSatNetDynPOD::initDynDatumEst(vector<TimePosVel> orbitlist, SatdynBasicDatum &dynamicDatum, double arclength, double h)
		{
			char info[200];
			// 提取粗间隔的插值点
			//double  threshold_coarseorbit_interval = m_twrDefine.span_InitDynDatumCoarsePos;
			double  threshold_coarseorbit_interval = 300.0;
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
				//GPST t_GPS = m_TimeCoordConvert.UTC2GPST(orbitlist[s_i].t - 3600.0 * 8);
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
				if(k >= m_tqNetPODDefine.max_OrbIteration)
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
		// 功能：动力学轨道拟合TQ卫星的轨道位置点序列
		// 变量类型：obsOrbitList         : 测量轨道列表, 采用UTC, ITRF坐标系
        //           it_Sat               : 拟合后的初始动力学轨道参数, it_Sat->second.dynDatum_Init
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
		bool TQSatNetDynPOD::dynamicPOD_pos(vector<TimePosVel> obsOrbitList, TQNETPOD_SatDatumMap::iterator it_Sat, UTC t0_forecast, UTC t1_forecast,  vector<TimePosVel> &forecastOrbList, double interval, bool bInitDynDatumEst, bool bForecast, bool bResEdit)
		{
			char info[200];
			if(obsOrbitList.size() <= 0)
				return false;
			// 进行初轨确定
			if(!bInitDynDatumEst)
			{
				vector<TimePosVel> orbitlist;
				double arclength_initDynDatumEst = 3600.0 * 3; // 3小时 原先为3个小时，现在为了拟合鹊桥轨道改为1小时
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
				dynamicDatum_0.bOn_SolidTide    = it_Sat->second.dynDatum_Est.bOn_SolidTide; // 20161208, 便于外部仿真控制
				dynamicDatum_0.oceanTideType    = it_Sat->second.dynDatum_Est.oceanTideType; // 20200503，潮汐类型开关
				dynamicDatum_0.bOn_OceanTide    = it_Sat->second.dynDatum_Est.bOn_OceanTide;
				dynamicDatum_0.bOn_ThirdBodyAcc = it_Sat->second.dynDatum_Est.bOn_ThirdBodyAcc;
				dynamicDatum_0.bOn_SolidPoleTide    = it_Sat->second.dynDatum_Est.bOn_SolidPoleTide;
				dynamicDatum_0.bOn_OceanPoleTide    = it_Sat->second.dynDatum_Est.bOn_OceanPoleTide;
				dynamicDatum_0.earthGravityType  = it_Sat->second.dynDatum_Est.earthGravityType;
				if(!initDynDatumEst(orbitlist, dynamicDatum_0,  arclength_initDynDatumEst, m_stepAdamsCowell)) // 鹊桥积分步长暂时改为1s
					return false;
				GPST t_GPS = m_TimeCoordConvert.UTC2GPST(obsOrbitList[obsOrbitList.size() - 1].t); // 转换到北京时  - 3600.0 * 8
				TDT t_End = TimeCoordConvert::GPST2TDT(t_GPS);
				it_Sat->second.dynDatum_Est.T0 = dynamicDatum_0.T0;
				it_Sat->second.dynDatum_Est.ArcLength = t_End - it_Sat->second.dynDatum_Est.T0;
				it_Sat->second.dynDatum_Est.X0 = dynamicDatum_0.X0;
				it_Sat->second.dynDatum_Est.init(it_Sat->second.period_SolarPressure, it_Sat->second.dynDatum_Est.ArcLength, it_Sat->second.period_EmpiricalAcc);
			}
			it_Sat->second.dynDatum_Init = it_Sat->second.dynDatum_Est; // 记录初始轨道根数
			char dynamicDatumFilePath[300];
			sprintf(dynamicDatumFilePath,"%s\\初始轨道拟合结果-%s-%s", m_tqSatNetDynPODPath.c_str(), it_Sat->first.c_str(), m_tqNetPODDefine.nameDynPodFitFile.c_str());
			if(!m_tqSatNetDynPODPath.empty())
			{
				FILE * pFitFile = fopen(dynamicDatumFilePath, "w+");
				fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
				fprintf(pFitFile, "%3d.      X    (m)      %20.4f\n", 1,it_Sat->second.dynDatum_Init.X0.x);
				fprintf(pFitFile, "%3d.      Y    (m)      %20.4f\n", 2,it_Sat->second.dynDatum_Init.X0.y);
				fprintf(pFitFile, "%3d.      Z    (m)      %20.4f\n", 3,it_Sat->second.dynDatum_Init.X0.z);
				fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f\n", 4,it_Sat->second.dynDatum_Init.X0.vx);
				fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f\n", 5,it_Sat->second.dynDatum_Init.X0.vy);
				fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f\n", 6,it_Sat->second.dynDatum_Init.X0.vz);
				int k_Parameter = 6;
				if(it_Sat->second.dynDatum_Init.bOn_SolarPressureAcc && it_Sat->second.dynDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
				{
					for(size_t s_i = 0; s_i < it_Sat->second.dynDatum_Init.solarPressureParaList.size(); s_i++)
					{
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].Cr = 0.0;
						fprintf(pFitFile, "%3d. %2d   CR            %20.4f\n", k_Parameter ,
																				s_i+1,
																				it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].Cr);
					}
				}
				if(it_Sat->second.dynDatum_Init.bOn_SolarPressureAcc && (it_Sat->second.dynDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_9PARA || it_Sat->second.dynDatum_Init.solarPressureType == TYPE_SOLARPRESSURE_5PARA))
				{
					for(size_t s_i = 0; s_i < it_Sat->second.dynDatum_Init.solarPressureParaList.size(); s_i++)
					{
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].D0 = 0;
						fprintf(pFitFile, "%3d. %2d   D0   (1.0E-7) %20.4f\n", k_Parameter,  
																			   s_i+1,
																			   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7);
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].DC1 = 0;
						fprintf(pFitFile, "%3d. %2d   DCOS (1.0E-7) %20.4f\n", k_Parameter,  
																			   s_i+1,
																			   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].DC1 * 1.0E+7);
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].DS1 = 0;
						fprintf(pFitFile, "%3d. %2d   DSIN (1.0E-7) %20.4f\n", k_Parameter,  
																			   s_i+1,
																			   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].DS1 * 1.0E+7);
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].Y0 = 0;
						fprintf(pFitFile, "%3d. %2d   Y0   (1.0E-7) %20.4f\n", k_Parameter,  
																			   s_i+1,
																			   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7);
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].YC1 = 0;
						fprintf(pFitFile, "%3d. %2d   YCOS (1.0E-7) %20.4f\n", k_Parameter,  
																			   s_i+1,
																			   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].YC1 * 1.0E+7);
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].YS1 = 0;
						fprintf(pFitFile, "%3d. %2d   YSIN (1.0E-7) %20.4f\n", k_Parameter,  
																			   s_i+1,
																			   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].YS1 * 1.0E+7);
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].B0 = 0;
						fprintf(pFitFile, "%3d. %2d   B0   (1.0E-7) %20.4f\n", k_Parameter,  
																			   s_i+1,
																			   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].B0 * 1.0E+7);
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].BC1 = 0;
						fprintf(pFitFile, "%3d. %2d   BCOS (1.0E-7) %20.4f\n", k_Parameter,  
																			   s_i+1,
																			   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].BC1 * 1.0E+7);
						k_Parameter++;
						it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].BS1 = 0;
						fprintf(pFitFile, "%3d. %2d   BSIN (1.0E-7) %20.4f\n", k_Parameter,  
																			   s_i+1,
																			   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].BS1 * 1.0E+7);
					}
				}
				if(it_Sat->second.dynDatum_Init.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Init.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
				{
					for(size_t s_i = 0; s_i < it_Sat->second.dynDatum_Init.empiricalForceParaList.size(); s_i++)
					{// 20140320, 谷德峰修改, 使得三个方向经验力可选
						if(it_Sat->second.dynDatum_Init.bOn_EmpiricalForceAcc_R)
						{
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7);
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7);
						}
						if(it_Sat->second.dynDatum_Init.bOn_EmpiricalForceAcc_T)
						{
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   TCOS (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7);
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   TSIN (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7);
						}
						if(it_Sat->second.dynDatum_Init.bOn_EmpiricalForceAcc_N)
						{
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_N = 0.0;
							fprintf(pFitFile, "%3d. %2d   NCOS (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7);
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_N = 0.0;
							fprintf(pFitFile, "%3d. %2d   NSIN (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7);
						}
					}
				}
				// 20170427, 谷德峰修改, 增加常值项经验力
				if(it_Sat->second.dynDatum_Init.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Init.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
				{
					for(size_t s_i = 0; s_i < it_Sat->second.dynDatum_Init.empiricalForceParaList.size(); s_i++)
					{// 20140320, 谷德峰修改, 使得三个方向经验力可选
						if(it_Sat->second.dynDatum_Init.bOn_EmpiricalForceAcc_R)
						{
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_R = 0.0;
							fprintf(pFitFile, "%3d. %2d   R_A0 (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_R  * 1.0E+7);
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_R = 0.0;
							fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7);
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_R = 0.0;
							fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7);
						}
						if(it_Sat->second.dynDatum_Init.bOn_EmpiricalForceAcc_T)
						{
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   T_A0 (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_T  * 1.0E+7);
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   TCOS (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7);
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_T = 0.0;
							fprintf(pFitFile, "%3d. %2d   TSIN (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7);
						}
						if(it_Sat->second.dynDatum_Init.bOn_EmpiricalForceAcc_N)
						{
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_N = 0.0;
							fprintf(pFitFile, "%3d. %2d   N_A0 (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_N  * 1.0E+7);
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_N = 0.0;
							fprintf(pFitFile, "%3d. %2d   NCOS (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7);
							k_Parameter++;
							it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_N = 0.0;
							fprintf(pFitFile, "%3d. %2d   NSIN (1.0E-7) %20.4f\n",    k_Parameter,
																					  s_i+1,
																					  it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7);
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
				GPST t_GPS = m_TimeCoordConvert.UTC2GPST(obsOrbitList[s_i].t); // 转换到北京时  - 3600.0 * 8
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

			int count_DynParameter = it_Sat->second.dynDatum_Est.getAllEstParaCount(); 
			int count_SolarPressureParaList = 0;
			if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc)
				count_SolarPressureParaList = int(it_Sat->second.dynDatum_Est.solarPressureParaList.size());

			int count_EmpiricalForceParaList = 0;
			if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc)
				count_EmpiricalForceParaList = int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size());
			
			int count_SolarPressurePara = it_Sat->second.dynDatum_Est.getSolarPressureParaCount();
			
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
				if(k >= m_tqNetPODDefine.max_OrbIteration)
				{
					result = false;
					printf("迭代超过%d次, 发散!", k);
					break;
				}
				vector<TimePosVel> interpOrbitlist; // 插值序列
				vector<Matrix> interpRtPartiallist; // 插值偏导数序列
				adamsCowell_Interp(interpTimelist, it_Sat->second.dynDatum_Est, interpOrbitlist, interpRtPartiallist, m_stepAdamsCowell); // TQ h = 75.0  // 鹊桥1.0s
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
					if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc)
					{
						for(int j = 0; j < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size() * count_SolarPressurePara); j++)
						{
							matH.SetElement(i * 3 + 0, beginPara + j, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j));
							matH.SetElement(i * 3 + 1, beginPara + j, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j));
							matH.SetElement(i * 3 + 2, beginPara + j, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j));
						}
						beginPara += count_SolarPressurePara * count_SolarPressureParaList;
					}
					//if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc && it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
					//{// 太阳光压
					//	for(int j = 0; j < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size()); j++)
					//	{
					//		matH.SetElement(i * 3 + 0, beginPara + j, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j));
					//		matH.SetElement(i * 3 + 1, beginPara + j, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j));
					//		matH.SetElement(i * 3 + 2, beginPara + j, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j));
					//	}
					//	beginPara += count_SolarPressureParaList;
					//}
					//else if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc &&
					//	   (it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_9PARA || it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_9PARA_EX || it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_15PARA || it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_21PARA))
					//{
					//	// 20140320, 谷德峰添加
					//	for(int j = 0; j < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size()); j++)
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
					if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
					{// 经验力
						index_EmpiricalParaBegin = beginPara;
						int count_sub =  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
										 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2  
										 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 2; 
						for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
						{
							int i_sub = 0;
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
							{
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 1));
							}
							i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2;
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
							{
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 0, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 0));
								matH.SetElement(i * 3 + 0, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 0) * interpRtPartiallist[i].GetElement(0, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 1, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 1) * interpRtPartiallist[i].GetElement(1, beginPara + j * count_sub + i_sub + 1));
								matH.SetElement(i * 3 + 2, beginPara + j * count_sub + i_sub + 1, matWeight.GetElement(i, 2) * interpRtPartiallist[i].GetElement(2, beginPara + j * count_sub + i_sub + 1));
							}
							i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2;
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
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
					if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
					{// 经验力
						index_EmpiricalParaBegin = beginPara;
						int count_sub =  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
										 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3  
										 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 3; 
						for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
						{
							int i_sub = 0;
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
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
							i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3;
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
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
							i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3;
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
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
				it_Sat->second.dynDatum_Est.X0.x  += matdx.GetElement(0,0);
				it_Sat->second.dynDatum_Est.X0.y  += matdx.GetElement(1,0);
				it_Sat->second.dynDatum_Est.X0.z  += matdx.GetElement(2,0);
				it_Sat->second.dynDatum_Est.X0.vx += matdx.GetElement(3,0);
				it_Sat->second.dynDatum_Est.X0.vy += matdx.GetElement(4,0);
				it_Sat->second.dynDatum_Est.X0.vz += matdx.GetElement(5,0);
				int beginPara = 6;
				if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc && it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
				{// 太阳光压
					for(int s_k = 0; s_k < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size()); s_k++)
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].Cr +=  matdx.GetElement(beginPara + s_k, 0);
					beginPara += count_SolarPressureParaList;
				}
				// 2015/10/18, 兼容多光压模型计算, 谷德峰
				else if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc && it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
				{
					// 20140320, 谷德峰添加
					for(int s_k = 0; s_k < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size()); s_k++)
					{
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].D0  += matdx.GetElement(beginPara + s_k * 9 + 0, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].DC1 += matdx.GetElement(beginPara + s_k * 9 + 1, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].DS1 += matdx.GetElement(beginPara + s_k * 9 + 2, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].Y0  += matdx.GetElement(beginPara + s_k * 9 + 3, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].YC1 += matdx.GetElement(beginPara + s_k * 9 + 4, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].YS1 += matdx.GetElement(beginPara + s_k * 9 + 5, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].B0  += matdx.GetElement(beginPara + s_k * 9 + 6, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].BC1 += matdx.GetElement(beginPara + s_k * 9 + 7, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].BS1 += matdx.GetElement(beginPara + s_k * 9 + 8, 0);
					}
					beginPara += 9 * count_SolarPressureParaList;
				}
				else if(it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_5PARA)	
				{
					for(int s_k = 0; s_k < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size()); s_k++)
					{
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].D0  += matdx.GetElement(beginPara + 5 * s_k,     0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].Y0  += matdx.GetElement(beginPara + 5 * s_k + 1, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].B0  += matdx.GetElement(beginPara + 5 * s_k + 2, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].BC1 += matdx.GetElement(beginPara + 5 * s_k + 3, 0);
						it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].BS1 += matdx.GetElement(beginPara + 5 * s_k + 4, 0);
					}
				}
				if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
				{// 经验力
					int count_sub =  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
									 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2  
									 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 2; 
					for(int s_k = 0; s_k < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); s_k++)
					{
						int i_sub = 0;
						if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
						{
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].cos_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].sin_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
						i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2;
						if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
						{
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].cos_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].sin_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
						i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2;
                        if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
						{
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].cos_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].sin_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
						}
					}
					beginPara += count_sub * count_EmpiricalForceParaList;
				}
				// 20170427, 谷德峰修改, 增加常值项经验力
				if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
				{// 经验力
					int count_sub =  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3  
									 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3  
									 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 3; 
					for(int s_k = 0; s_k < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); s_k++)
					{
						int i_sub = 0;
						if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
						{
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].a0_R  += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].cos_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].sin_R += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
						i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3;
						if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
						{
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].a0_T  += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].cos_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].sin_T += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
						i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3;
                        if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
						{
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].a0_N  += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 0, 0);
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].cos_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 1, 0);
							it_Sat->second.dynDatum_Est.empiricalForceParaList[s_k].sin_N += matdx.GetElement(beginPara + s_k * count_sub + i_sub + 2, 0);
						}
					}
					beginPara += count_sub * count_EmpiricalForceParaList;
				}

				if(!m_tqSatNetDynPODPath.empty())
				{
					// 记录轨道改进结果
					FILE * pFitFile = fopen(dynamicDatumFilePath, "w+");
					fprintf(pFitFile, "  PARAMETER                         A PRIORI    ADJUST             POSTFIT\n");
					fprintf(pFitFile, "%3d.      X    (m)      %20.4f%10.4f%20.4f\n", 1,it_Sat->second.dynDatum_Init.X0.x,  it_Sat->second.dynDatum_Est.X0.x  - it_Sat->second.dynDatum_Init.X0.x,  it_Sat->second.dynDatum_Est.X0.x);
					fprintf(pFitFile, "%3d.      Y    (m)      %20.4f%10.4f%20.4f\n", 2,it_Sat->second.dynDatum_Init.X0.y,  it_Sat->second.dynDatum_Est.X0.y  - it_Sat->second.dynDatum_Init.X0.y,  it_Sat->second.dynDatum_Est.X0.y);
					fprintf(pFitFile, "%3d.      Z    (m)      %20.4f%10.4f%20.4f\n", 3,it_Sat->second.dynDatum_Init.X0.z,  it_Sat->second.dynDatum_Est.X0.z  - it_Sat->second.dynDatum_Init.X0.z,  it_Sat->second.dynDatum_Est.X0.z);
					fprintf(pFitFile, "%3d.      XDOT (m/s)    %20.4f%10.4f%20.4f\n", 4,it_Sat->second.dynDatum_Init.X0.vx, it_Sat->second.dynDatum_Est.X0.vx - it_Sat->second.dynDatum_Init.X0.vx, it_Sat->second.dynDatum_Est.X0.vx);
					fprintf(pFitFile, "%3d.      YDOT (m/s)    %20.4f%10.4f%20.4f\n", 5,it_Sat->second.dynDatum_Init.X0.vy, it_Sat->second.dynDatum_Est.X0.vy - it_Sat->second.dynDatum_Init.X0.vy, it_Sat->second.dynDatum_Est.X0.vy);
					fprintf(pFitFile, "%3d.      ZDOT (m/s)    %20.4f%10.4f%20.4f\n", 6,it_Sat->second.dynDatum_Init.X0.vz, it_Sat->second.dynDatum_Est.X0.vz - it_Sat->second.dynDatum_Init.X0.vz, it_Sat->second.dynDatum_Est.X0.vz);
					int k_Parameter = 7;
					if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc && it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_1PARA)
					{
						for(size_t s_i = 0; s_i < it_Sat->second.dynDatum_Est.solarPressureParaList.size(); s_i++)
						{
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   CR            %20.4f%10.4f%20.4f\n", k_Parameter ,
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].Cr,
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].Cr - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].Cr,
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].Cr);
						}
					}
					if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc && (it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_9PARA || it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_5PARA))
					{
						for(size_t s_i = 0; s_i < it_Sat->second.dynDatum_Est.solarPressureParaList.size(); s_i++)
						{
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   D0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].D0 * 1.0E+7 - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].D0 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].D0 * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   DCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].DC1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].DC1 * 1.0E+7 - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].DC1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].DC1 * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   DSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].DS1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].DS1 * 1.0E+7 - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].DS1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].DS1 * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   Y0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].Y0 * 1.0E+7 - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].Y0 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].Y0 * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   YCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].YC1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].YC1 * 1.0E+7 - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].YC1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].YC1 * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   YSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].YS1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].YS1 * 1.0E+7 - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].YS1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].YS1 * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   B0   (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].B0 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].B0 * 1.0E+7 - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].B0 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].B0 * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   BCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].BC1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].BC1 * 1.0E+7 - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].BC1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].BC1 * 1.0E+7);
							k_Parameter++;
							fprintf(pFitFile, "%3d. %2d   BSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,  
																							   s_i+1,
																							   it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].BS1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].BS1 * 1.0E+7 - it_Sat->second.dynDatum_Init.solarPressureParaList[s_i].BS1 * 1.0E+7, 
																							   it_Sat->second.dynDatum_Est.solarPressureParaList[s_i].BS1 * 1.0E+7);
						}
					}
					if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
					{
						for(size_t s_i = 0; s_i < it_Sat->second.dynDatum_Est.empiricalForceParaList.size(); s_i++)
						{// 20140320, 谷德峰修改, 使得三个方向经验力可选
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_R * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_R * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_R * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_R * 1.0E+7);
							}
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   TCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_T * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_T * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   TSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_T * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_T * 1.0E+7);
							}
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   NCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_N * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_N * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   NSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_N * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_N * 1.0E+7);
							}
						}
					}
					// 20170427, 谷德峰修改, 增加常值项经验力
					if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
					{
						for(size_t s_i = 0; s_i < it_Sat->second.dynDatum_Est.empiricalForceParaList.size(); s_i++)
						{// 20140320, 谷德峰修改, 使得三个方向经验力可选
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   R_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].a0_R * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].a0_R * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_R * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_R * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   RSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_R * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_R * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_R * 1.0E+7);
							}
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   T_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].a0_T * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].a0_T * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   TCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_T * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_T * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   TSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_T * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_T * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_T * 1.0E+7);
							}
							if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
							{
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   N_A0 (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].a0_N * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].a0_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].a0_N * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   NCOS (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_N * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].cos_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].cos_N * 1.0E+7);
								k_Parameter++;
								fprintf(pFitFile, "%3d. %2d   NSIN (1.0E-7) %20.4f%10.4f%20.4f\n", k_Parameter,
																								   s_i+1,
																								   it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_N * 1.0E+7 - it_Sat->second.dynDatum_Init.empiricalForceParaList[s_i].sin_N * 1.0E+7,
																								   it_Sat->second.dynDatum_Est.empiricalForceParaList[s_i].sin_N * 1.0E+7);
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

			if(bForecast) // 进行轨道预报
				orbitExtrapolation(it_Sat->second.dynDatum_Est, t0_forecast, t1_forecast,  it_Sat->second.orbList, interval);
		
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

		void TQSatNetDynPOD::updateSat_Obs(TQNETPOD_SatDatumMap::iterator it_Sat)
		{
			int id_ZeroDelay = -1; 
			char info[200];
			TQNETPOD_UXBStaDatumMap::iterator jt_Sta = it_Sat->second.mapUXBStaDatum.begin();
			while(jt_Sta != it_Sat->second.mapUXBStaDatum.end())
			{
				// 剪切数据 [t0, t1]
				map<UTC, TQUXBObsLine>::iterator jt_obs = jt_Sta->second.obsList.begin();
				while(jt_obs != jt_Sta->second.obsList.end())
				{
					if(jt_obs->first - it_Sat->second.t0 < 0.0 || jt_obs->first - it_Sat->second.t1 > 0)
					{
						map<UTC, TQUXBObsLine>::iterator jt_erase = jt_obs;
						++jt_obs;
						jt_Sta->second.obsList.erase(jt_erase); // 剔除数据
						continue;
					}
					else
					{ 
						// 后续请李康康整理 ?????? 
						//jt_obs->second.oc_doppler;

						bool bflag = false;
						double dR[3];
						// 获取观测时刻的测站概略坐标
						double P_J2000[6]; // 惯性坐标, 用于坐标系转换
						double P_ITRF[6];  // 地固坐标
						P_ITRF[0] = jt_Sta->second.pos_ITRF.x;
						P_ITRF[1] = jt_Sta->second.pos_ITRF.y;
						P_ITRF[2] = jt_Sta->second.pos_ITRF.z;
						
						// 循环计算t,t0,t1时刻测站概略坐标
						for(int k = 0; k < 3; k++)
						{
							// 获得测站在 J2000 惯性系中的位置
							// t, 用于距离计算
							UTC u_k = jt_obs->second.t;
							POS3D pos_J2000;
							m_TimeCoordConvert.ECEF_J2000(m_TimeCoordConvert.UTC2GPST(u_k),  P_J2000, P_ITRF, false);
							jt_obs->second.pos_J2000.x    = P_J2000[0];
							jt_obs->second.pos_J2000.y    = P_J2000[1];
							jt_obs->second.pos_J2000.z    = P_J2000[2];
							pos_J2000.x = P_J2000[0];
							pos_J2000.y = P_J2000[1];
							pos_J2000.z = P_J2000[2];

							if(k == 1)
							{
								u_k = jt_obs->second.t0;
								m_TimeCoordConvert.ECEF_J2000(m_TimeCoordConvert.UTC2GPST(u_k),  P_J2000, P_ITRF, false);
								jt_obs->second.pos_J2000_t0.x   = P_J2000[0];
								jt_obs->second.pos_J2000_t0.y   = P_J2000[1];
								jt_obs->second.pos_J2000_t0.z   = P_J2000[2];
								pos_J2000.x = P_J2000[0];
								pos_J2000.y = P_J2000[1];
								pos_J2000.z = P_J2000[2];
							}
							
							if(k == 2)
							{
								u_k = jt_obs->second.t1;
								m_TimeCoordConvert.ECEF_J2000(m_TimeCoordConvert.UTC2GPST(u_k),  P_J2000, P_ITRF, false);
								jt_obs->second.pos_J2000_t1.x   = P_J2000[0];
								jt_obs->second.pos_J2000_t1.y   = P_J2000[1];
								jt_obs->second.pos_J2000_t1.z   = P_J2000[2];
								pos_J2000.x = P_J2000[0];
								pos_J2000.y = P_J2000[1];
								pos_J2000.z = P_J2000[2];

							}
							
							// 根据先验卫星轨道orbJ2000File_0、测站坐标数据, 进行观测数据编辑
							POS6D tqPV_J2000, tqPV_ITRF;;
							double delay = jt_obs->second.getRange() / SPEED_LIGHT; // 初始化延迟数据
							UTC tr_utc = u_k - delay;
							if(!it_Sat->second.orbJ2000File_0.getPosVel(tr_utc, tqPV_J2000))
							{
								if(m_tqNetPODDefine.on_ObsEditedInfo)
								{
									sprintf(info, "剔除数据 %s %s %s 轨道缺失.", it_Sat->second.satName.c_str(), jt_Sta->second.nameSta.c_str(), jt_obs->second.t.toString().c_str());
									RuningInfoFile::Add(info);
									printf("%s\n",info);// 测试代码
								}
								map<UTC, TQUXBObsLine>::iterator jt_erase = jt_obs;
								++jt_obs;
								jt_Sta->second.obsList.erase(jt_erase); // 剔除数据
								bflag = true;
								break;
							}
							
							P_J2000[0] = tqPV_J2000.x;
							P_J2000[1] = tqPV_J2000.y;
							P_J2000[2] = tqPV_J2000.z;
							P_J2000[3] = tqPV_J2000.vx;
							P_J2000[4] = tqPV_J2000.vy;
							P_J2000[5] = tqPV_J2000.vz;
							m_TimeCoordConvert.J2000_ECEF(m_TimeCoordConvert.UTC2GPST(tr_utc), P_J2000, P_ITRF, true);
							tqPV_ITRF.x  = P_ITRF[0];
							tqPV_ITRF.y  = P_ITRF[1];
							tqPV_ITRF.z  = P_ITRF[2];
							tqPV_ITRF.vx = P_ITRF[3];
							tqPV_ITRF.vy = P_ITRF[4];
							tqPV_ITRF.vz = P_ITRF[5];

							GPST tr_gps = m_TimeCoordConvert.UTC2GPST(tr_utc);
							TDB t_TDB = m_TimeCoordConvert.GPST2TDB(tr_gps); // 获得TDB时间--提供太阳历参考时间
							double jd_TDB = m_TimeCoordConvert.DayTime2JD(t_TDB); // 获得儒略日

							//// 测试代码，添加对流层延迟
							//// 更新计算高度角和方位角
							//ENU geoENU;
							//TimeCoordConvert::ECF2ENU(jt_Sta->second.pos_ITRF, tqPV_ITRF.getPos(), geoENU);
							//jt_obs->second.Azimuth = atan2(geoENU.E, geoENU.N) * 180 / PI;
							//if(jt_obs->second.Azimuth < 0)
							//	jt_obs->second.Azimuth = jt_obs->second.Azimuth  + 360; // 保证方位角在 [0, 360) 之间
							//// 20150608, 卫星仰角计算考虑到地球扁率影响, 谷德峰
							//POS3D p_station = vectorNormal(jt_Sta->second.pos_ITRF);
							//POS3D p_sat = vectorNormal(tqPV_ITRF.getPos() - jt_Sta->second.pos_ITRF);					
							//p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); 
							//p_station = vectorNormal(p_station);
							//
							//double Elevation = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;

							//1. 对流层延迟
                            double dR_trop = 0.0;
							if(m_tqNetPODDefine.on_TropDelay)
								dR_trop = 0.0;

							//if(m_tqNetPODDefine.on_TropDelay) //对流层修正
							//{
							//	// 使用简化霍普菲尔德（Hopfield）模型修正对流层延迟
							//	double P_s = 1.0; // 测站的气温，大气压，水汽压 ，这里可能要修改测站的结构属性
							//	double T_s = 1.0;
							//	double e_s = 1.0;

							//	double h_d = 40136 + 148.72 * (T_s - 273.16);
							//	double h_s = 11000;
							//	double K_d = 155.2 * e-7 * P_s * (h_d - h_s) / T_s;
							//	double K_w = 155.2 * e-7 * 4810 * e_s * (h_d - h_s) / (T_s * T_s);

							//	double S_d = K_d / sqrt(sin(Elevation * Elevation + 6.25));
							//	double S_w = K_w / sqrt(sin(Elevation * Elevation + 2.25));

							//	dR_trop = S_d + S_w;
							//}

							// Saastamoinen模型
							//if(m_tqNetPODDefine.on_TropDelay) //对流层修正
							//{
							//	// 使用Saastamoinen模型修正对流层延迟
							//	double P = 1.0; // 测站的气温(°C)，大气压(hPa)，相对湿度(%) ，这里可能要修改测站的结构属性
							//	double T = 1.0;
							//	double e = 1.0;

							//	double k = 0.002277;
							//	double h = 0.0; // 测站高程(km)
							//	double u = 0.0; // 纬度(度)
							//	double w = 1 - 0.00266 * cos(2*u) - 0.00028 * h;
							//	double q = (0.05 + 1255/(T + 273.16) * 6.11 * e * pow(10,7.5 * T /(237.3 + T)));

							//	double S_d = k * P / w;
							//	double S_w = k * q / w;
							//	dR_trop = S_d + S_w;
							//}

							//2. 卫星反射器偏移改正
							double dR_satpco = 0.0;
							if(m_tqNetPODDefine.on_SatPCO)
								dR_satpco = 0.0;
							

							//3. 相对论修正
							double dR_GraRelativity = 0.0;
							
							if(m_tqNetPODDefine.on_GraRelativity)
							{
								dR_GraRelativity = GNSSBasicCorrectFunc::graRelativityCorrect(tqPV_J2000.getPos(), jt_obs->second.pos_J2000);
							}

							//4. 固体潮修正
							double dR_SolidTides = 0.0;
						
							if(m_tqNetPODDefine.on_SolidTides)
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
								if(m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04_1980 || m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04_2000A) 
									m_TimeCoordConvert.m_eopc04File.getPoleOffset(m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(tr_gps)),xp,yp); // 获得极移数据
								else if(m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04Total_1980 || m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04Total_2000A) 
									m_TimeCoordConvert.m_eopc04TotalFile.getPoleOffset(m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(tr_gps)),xp,yp); // 获得极移数据
								else
									m_TimeCoordConvert.m_eopRapidFileIAU2000.getPoleOffset(m_TimeCoordConvert.TAI2UTC(TimeCoordConvert::GPST2TAI(tr_gps)), xp, yp); // 获得极移数据

								POS3D posSolidTide_ECEF = SolidTides::solidTideCorrect(tr_gps, sunPos_ITRF, moonPos_ITRF, jt_Sta->second.pos_ITRF, xp, yp);
								POS3D vecLos = vectorNormal(tqPV_ITRF.getPos() - jt_Sta->second.pos_ITRF);
								dR_SolidTides = - vectorDot(posSolidTide_ECEF, vecLos);							
							}

							// 测站的海潮波暂时是缺失的
							double dR_OceanTides = 0.0;
							if(m_tqNetPODDefine.on_OceanTides)
								dR_OceanTides = 0.0;
														
							// 总的延迟量
							double corrected_value = dR_trop
								                    + dR_satpco
													+ dR_GraRelativity
													+ dR_SolidTides
													+ dR_OceanTides;
							
							if(k == 0)
							{
								jt_obs->second.dR_trop          = dR_trop;
								jt_obs->second.dR_satpco        = dR_satpco;
								jt_obs->second.dR_GraRelativity = dR_GraRelativity;
								jt_obs->second.dR_SolidTides    = dR_SolidTides;
								jt_obs->second.dR_OceanTides    = dR_OceanTides;

								jt_obs->second.corrected_value = jt_obs->second.dR_trop
															 + jt_obs->second.dR_satpco
															 + jt_obs->second.dR_GraRelativity
															 + jt_obs->second.dR_SolidTides
															 + jt_obs->second.dR_OceanTides;

							}

							// 迭代计算下行信号 reflect 时间 tr_utc, 获得反射时刻卫星轨道位置
							double dDelay_k_1 = 0;
							double dR_down = jt_obs->second.getRange();
							TimePosVel orbit;
							while(fabs(delay - dDelay_k_1) > 1.0E-10)
							{
								// 更新延迟时间
								dDelay_k_1 = delay;
								// 根据 dDelay 计算下行信号 reflect 时间
								tr_utc  = u_k - delay; 
								orbit.t = tr_utc;
								// 获得 J2000 惯性系下的卫星轨道 
								it_Sat->second.orbJ2000File_0.getPosVel(tr_utc, tqPV_J2000);
								// 计算下行几何距离
								dR_down = sqrt(pow(pos_J2000.x - tqPV_J2000.x, 2) +
											   pow(pos_J2000.y - tqPV_J2000.y, 2) +
											   pow(pos_J2000.z - tqPV_J2000.z, 2));

								delay = (dR_down + corrected_value) / SPEED_LIGHT;
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
								P_ITRF[0] = jt_Sta->second.pos_ITRF.x;
								P_ITRF[1] = jt_Sta->second.pos_ITRF.y;
								P_ITRF[2] = jt_Sta->second.pos_ITRF.z;
								GPST tt_gps = m_TimeCoordConvert.UTC2GPST(tt_utc); // 转换到北京时  - 3600.0 * 8
								m_TimeCoordConvert.ECEF_J2000(tt_gps, P_J2000, P_ITRF, false);
								POS3D staPos_J2000_tt;
								staPos_J2000_tt.x = P_J2000[0];
								staPos_J2000_tt.y = P_J2000[1];
								staPos_J2000_tt.z = P_J2000[2];
								// 计算上行几何距离
								dR_up = sqrt(pow(staPos_J2000_tt.x - tqPV_J2000.x, 2) +
											 pow(staPos_J2000_tt.y - tqPV_J2000.y, 2) +
											 pow(staPos_J2000_tt.z - tqPV_J2000.z, 2));
								delay = (dR_up + corrected_value) / SPEED_LIGHT;
							}


                            dR[k] = 0.5 * (dR_up + dR_down);

							if(k == 0)
							{
								jt_obs->second.dR_up    = dR_up;
								jt_obs->second.dR_down  = dR_down;
								jt_obs->second.oc_range = jt_obs->second.getRange()
												        - it_Sat->second.satZeroDelay_0
														- jt_Sta->second.zeroDelay_0
														- jt_obs->second.corrected_value
														- 0.5 * (dR_up + dR_down);

								//FILE * fp = fopen("C:\\软件开发\\NUDTTK-TQ\\release\\TQ\\TQPOD\\初始化轨道信息测距残差.txt","a+");
								//fprintf(fp,"初始化轨道信息\n");
								//fprintf(fp,"t = %s\n",jt_obs->second.t.toString().c_str());
								//fprintf(fp,"oc_range = %20.10lf\n",jt_obs->second.oc_range);
								//fclose(fp);

								// 更新计算高度角和方位角
								ENU geoENU;
								TimeCoordConvert::ECF2ENU(jt_Sta->second.pos_ITRF, tqPV_ITRF.getPos(), geoENU);
								jt_obs->second.Azimuth = atan2(geoENU.E, geoENU.N) * 180 / PI;
								if(jt_obs->second.Azimuth < 0)
									jt_obs->second.Azimuth = jt_obs->second.Azimuth  + 360; // 保证方位角在 [0, 360) 之间
								// 20150608, 卫星仰角计算考虑到地球扁率影响, 谷德峰
								POS3D p_station = vectorNormal(jt_Sta->second.pos_ITRF);
								POS3D p_sat = vectorNormal(tqPV_ITRF.getPos() - jt_Sta->second.pos_ITRF);					
								p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); 
								p_station = vectorNormal(p_station);					
								jt_obs->second.Elevation = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;

							}

						}

						if(bflag)
							continue;

						// 更新测速残差

						jt_obs->second.oc_doppler = jt_obs->second.getVel() - (dR[2] - dR[1]) / (jt_obs->second.t1 -jt_obs->second.t0);

						//FILE *fp = fopen("C:\\软件开发\\NUDTTK-TQ\\release\\TQ\\TQPOD\\初始化轨道信息测速残差.txt","a+");
						//fprintf(fp,"初始化轨道信息\n");
						//fprintf(fp,"t  = %s\n",jt_obs->second.t.toString().c_str());
						//fprintf(fp,"t0 = %s\n",jt_obs->second.t0.toString().c_str());
						//fprintf(fp,"t1 = %s\n",jt_obs->second.t1.toString().c_str());
						//fprintf(fp,"dR[0] = %20.10lf\n",dR[0]);
						//fprintf(fp,"dR[1] = %20.10lf\n",dR[1]);
						//fprintf(fp,"dR[2] = %20.10lf\n",dR[2]);
						//fprintf(fp,"jt_obs->second.getVel() = %20.10lf\n",jt_obs->second.getVel());
						//fprintf(fp,"oc_doppler = %20.10lf\n",jt_obs->second.oc_doppler);
						//fclose(fp);

						// 剔除仰角较低和残差较大的测站
						if(jt_obs->second.Elevation < m_tqNetPODDefine.min_Elevation
						|| fabs(jt_obs->second.oc_range) > m_tqNetPODDefine.max_Edit_OcRange
						|| fabs(jt_obs->second.oc_doppler) > m_tqNetPODDefine.max_Edit_OcDoppler)
						{
							if(m_tqNetPODDefine.on_ObsEditedInfo)
							{
								sprintf(info, "剔除数据 %s %s %s %10.1f %20.4f.", it_Sat->second.satName.c_str(), jt_Sta->second.nameSta.c_str(), jt_obs->second.t.toString().c_str(), jt_obs->second.Elevation, jt_obs->second.oc_range);
								RuningInfoFile::Add(info);
							}
							map<UTC, TQUXBObsLine>::iterator jt_erase = jt_obs;
							++jt_obs;
							jt_Sta->second.obsList.erase(jt_erase); // 剔除数据
							continue;
						}
					}
					++jt_obs;
					continue;
				}

				 //剔除观测数据较少的测站（激光定轨不用）
				if(int(jt_Sta->second.obsList.size()) <=  m_tqNetPODDefine.min_ArcPointCount)
				{
					if(m_tqNetPODDefine.on_ObsEditedInfo)
					{
						sprintf(info, "剔除数据 %s %s %d.", it_Sat->second.satName.c_str(), jt_Sta->second.nameSta.c_str(), jt_Sta->second.obsList.size());
						RuningInfoFile::Add(info);
					}
					TQNETPOD_UXBStaDatumMap::iterator jt_erase = jt_Sta;
					++jt_Sta;
					it_Sat->second.mapUXBStaDatum.erase(jt_erase); // 剔除数据
					continue;
				}

				id_ZeroDelay++;
				jt_Sta->second.id_ZeroDelay = id_ZeroDelay;
				++jt_Sta;
				continue;
			}
		}

		// 子程序名称： updateSta_AdamsCowell   
		// 功能：更新测站积分信息interpTimelist、interpOrbitlist、interpRtPartiallist，计算attXYZBodyList、mapDynEpochList
		// 变量类型：
		// 输入：
		// 输出：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：
		// 版本时间：2018/06/22
		// 修改记录：
		// 备注：
		void TQSatNetDynPOD::updateSat_AdamsCowell(TQNETPOD_SatDatumMap::iterator it_Sat) 
		{
			// 根据初轨进行轨道积分, 用于后续的概略距离修正
			adamsCowell_ac(it_Sat->second.t0, it_Sat->second.t1, it_Sat->second.dynDatum_Est, it_Sat->second.acOrbitList, it_Sat->second.acRtPartialList, m_stepAdamsCowell);
			for(TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = it_Sat->second.mapUXBStaDatum.begin(); it_UXBSta != it_Sat->second.mapUXBStaDatum.end(); ++it_UXBSta)
			{
				//for(map<UTC, TQUXBObsLine>::iterator jt_obs = it_UXBSta->second.obsList.begin(); jt_obs != it_UXBSta->second.obsList.end(); ++jt_obs)
				// 李康康修改  为了剔除不能得到卫星的延迟的时刻
				map<UTC, TQUXBObsLine>::iterator jt_obs = it_UXBSta->second.obsList.begin();
				while(jt_obs != it_UXBSta->second.obsList.end())
				{
					// 后续请李康康整理 ?????? 
					//jt_obs->second.oc_doppler;

					bool bflag = false;
					double dR[3];
					for(int k = 0; k < 3; k++)
					{
						double dR_up, dR_down;
						// 初始化测站位置和时刻
						UTC u_k = jt_obs->second.t;
						POS3D pos_J2000 = jt_obs->second.pos_J2000;
						
						if(k == 1)
						{
							u_k = jt_obs->second.t0;
							pos_J2000 = jt_obs->second.pos_J2000_t0;
						}

						if(k == 2)
						{
							u_k = jt_obs->second.t1;
							pos_J2000 = jt_obs->second.pos_J2000_t1;
						}

						TimePosVel tqOrb;
						Matrix tqRtPartial;
					    double delay_down;

						if(!it_Sat->second.getEphemeris_PathDelay(u_k,  pos_J2000,    delay_down, tqOrb, tqRtPartial))
						{
							printf("无法得到 %s 时光行时\n",u_k.toString().c_str());
							bflag = true;
							//map<UTC, TQUXBObsLine>::iterator jt_erase = jt_obs;
						    //++jt_obs;
						    //it_UXBSta->second.obsList.erase(jt_erase); // 剔除数据
							break;
						}

						// 更新 dR_down 和 dR_up
						dR_down = pow(pos_J2000.x - tqOrb.pos.x, 2) +
								  pow(pos_J2000.y - tqOrb.pos.y, 2) +
								  pow(pos_J2000.z - tqOrb.pos.z, 2);
						if(dR_down > 0)
							dR_down = sqrt(dR_down);
						else
						{
							bflag = true;
							break;
						}
						//printf("dR_down = %lf\n",dR_down); // 测试代码
						UTC tr_utc = u_k - delay_down; // 反射时刻 tr_utc
						// 迭代计算上行信号延迟时间
						double dDelay_k_1 = 0;
						dR_up = jt_obs->second.getRange();
						double delay_up = dR_up / SPEED_LIGHT;
						UTC tt_utc = tr_utc - delay_up;
						while(fabs(delay_up - dDelay_k_1) > 1.0E-10)
						{// 更新延迟时间
							dDelay_k_1 = delay_up;
							// 根据 dDelay 计算地面信号时间
							tt_utc = tr_utc - delay_up;
							// 获得 J2000 惯性系下的观测站位置
							double P_J2000[6]; // 惯性坐标, 用于坐标系转换
							double P_ITRF[6];  // 地固坐标
							P_ITRF[0] = it_UXBSta->second.pos_ITRF.x;
							P_ITRF[1] = it_UXBSta->second.pos_ITRF.y;
							P_ITRF[2] = it_UXBSta->second.pos_ITRF.z;
							GPST tt_gps = m_TimeCoordConvert.UTC2GPST(tt_utc); // 转换到北京时  - 3600.0 * 8
							m_TimeCoordConvert.ECEF_J2000(tt_gps, P_J2000, P_ITRF, false);
							POS3D staPos_J2000_tt;
							staPos_J2000_tt.x = P_J2000[0];
							staPos_J2000_tt.y = P_J2000[1];
							staPos_J2000_tt.z = P_J2000[2];
							// 计算上行几何距离
							dR_up = sqrt(pow(staPos_J2000_tt.x - tqOrb.pos.x, 2) +
										 pow(staPos_J2000_tt.y - tqOrb.pos.y, 2) +
										 pow(staPos_J2000_tt.z - tqOrb.pos.z, 2));

							delay_up = (dR_up + jt_obs->second.corrected_value) / SPEED_LIGHT;
						}

						dR[k] = 0.5 * (dR_up + dR_down);

						if(k == 0)
						{
							jt_obs->second.tqOrb = tqOrb;
							jt_obs->second.tqRtPartial = tqRtPartial;
							 
							jt_obs->second.dR_up   = dR_up;
							jt_obs->second.dR_down = dR_down;


							jt_obs->second.oc_range = jt_obs->second.getRange()
													- it_Sat->second.satZeroDelay_0
													- it_Sat->second.satZeroDelay_Est
													- it_UXBSta->second.zeroDelay_0
					     							- it_UXBSta->second.zeroDelay_Est
													- 0.5 * (jt_obs->second.dR_up + jt_obs->second.dR_down);

							//FILE * fp = fopen("C:\\软件开发\\NUDTTK-TQ\\release\\TQ\\TQPOD\\更新轨道信息测距残差.txt","a+");
							//fprintf(fp,"更新轨道信息\n");
							//fprintf(fp,"t = %s\n",jt_obs->second.t.toString().c_str());
							//fprintf(fp,"oc_range = %20.10lf\n",jt_obs->second.oc_range);
							//fclose(fp);

						}
						
						if(k == 1)
						{
							jt_obs->second.tqOrb_t0 = tqOrb;
							jt_obs->second.tqRtPartial_t0 = tqRtPartial;

						}
						if(k == 2)
						{
							jt_obs->second.tqOrb_t1 = tqOrb;
							jt_obs->second.tqRtPartial_t1 = tqRtPartial;

						}

					}
					if(bflag)
					{
						map<UTC, TQUXBObsLine>::iterator jt_erase = jt_obs;
						++jt_obs;
						it_UXBSta->second.obsList.erase(jt_erase); // 剔除数据
						continue;
					}
                    // 更新测速残差
					jt_obs->second.oc_doppler = jt_obs->second.getVel() - (dR[2] - dR[1]) / (jt_obs->second.t1 - jt_obs->second.t0);

					//FILE * fp = fopen("C:\\软件开发\\NUDTTK-TQ\\release\\TQ\\TQPOD\\更新轨道信息测速残差.txt","a+");
					//fprintf(fp,"更新轨道信息\n");
					//fprintf(fp,"t  = %s\n",jt_obs->second.t.toString().c_str());
					//fprintf(fp,"t0 = %s\n",jt_obs->second.t0.toString().c_str());
					//fprintf(fp,"t1 = %s\n",jt_obs->second.t1.toString().c_str());
					//fprintf(fp,"dR[0] = %20.10lf\n",dR[0]);
					//fprintf(fp,"dR[1] = %20.10lf\n",dR[1]);
					//fprintf(fp,"dR[2] = %20.10lf\n",dR[2]);
					//fprintf(fp,"(dR[2] - dR[1]) / dt    = %20.10lf\n",(dR[2] - dR[1]) / (jt_obs->second.t1 - jt_obs->second.t0));
					//fprintf(fp,"jt_obs->second.getVel() = %20.10lf\n",jt_obs->second.getVel());
					//fprintf(fp,"oc_doppler = %20.10lf\n",jt_obs->second.oc_doppler);
					//fclose(fp);

					++jt_obs;

				}
			}
		}

		void TQSatNetDynPOD::updateSat_NEQ(TQNETPOD_SatDatumMap::iterator it_Sat) 
		{
			it_Sat->second.count_EstDynParameters = it_Sat->second.dynDatum_Est.getAllEstParaCount(); 
			int count_SolarPressureParaList = 0;
			if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc)
				count_SolarPressureParaList = int(it_Sat->second.dynDatum_Est.solarPressureParaList.size());
			int count_EmpiricalForceParaList = 0;
			if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc)
				count_EmpiricalForceParaList = int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size());
			int count_SolarPressurePara = it_Sat->second.dynDatum_Est.getSolarPressureParaCount();

			it_Sat->second.count_EstSatZeroDelay = 0;
			if(it_Sat->second.on_EstSatZeroDelay)
				it_Sat->second.count_EstSatZeroDelay = 1;

			it_Sat->second.count_EstStaZeroDelay = 0;
			if(m_tqNetPODDefine.on_EstStaZeroDelay)
				it_Sat->second.count_EstStaZeroDelay = int(it_Sat->second.mapUXBStaDatum.size()); // 每个测站增加一个参数, 多星估计测站共用时，增加相对伪方程约束；如果部分测站不估计，增加零值伪方程
			else
			{
				// 测试代码，添加选择测站时延参数估计的开关
				for(TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = it_Sat->second.mapUXBStaDatum.begin();it_UXBSta != it_Sat->second.mapUXBStaDatum.end();it_UXBSta++)
				{
					it_UXBSta->second.on_EstStaZeroDelay = false;
				}
			}
			
			
			it_Sat->second.count_EstParameters = it_Sat->second.count_EstDynParameters + it_Sat->second.count_EstSatZeroDelay + it_Sat->second.count_EstStaZeroDelay;

			it_Sat->second.n_xx.Init(it_Sat->second.count_EstParameters, it_Sat->second.count_EstParameters);
			it_Sat->second.nx.Init(it_Sat->second.count_EstParameters, 1);

			for(TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = it_Sat->second.mapUXBStaDatum.begin(); it_UXBSta != it_Sat->second.mapUXBStaDatum.end(); ++it_UXBSta)
			{
				for(map<UTC, TQUXBObsLine>::iterator jt_obs = it_UXBSta->second.obsList.begin(); jt_obs != it_UXBSta->second.obsList.end(); ++jt_obs)
				{
					if(m_tqNetPODDefine.on_Used_Range)
					{
						POS3D vecLos = vectorNormal(jt_obs->second.tqOrb.pos - jt_obs->second.pos_J2000);
						double w = jt_obs->second.rw_range * jt_obs->second.weight_range;
						Matrix matH_pos_j(1, 3);
						matH_pos_j.SetElement(0, 0, w * vecLos.x);
						matH_pos_j.SetElement(0, 1, w * vecLos.y);
						matH_pos_j.SetElement(0, 2, w * vecLos.z);
						Matrix matHx_j(1, it_Sat->second.count_EstParameters);
						for(int s_k = 0; s_k < 6; s_k++)
						{// 初始位置速度
							double sum_posvel = jt_obs->second.tqRtPartial.GetElement(0, s_k) * matH_pos_j.GetElement(0, 0) 
											  + jt_obs->second.tqRtPartial.GetElement(1, s_k) * matH_pos_j.GetElement(0, 1)
											  + jt_obs->second.tqRtPartial.GetElement(2, s_k) * matH_pos_j.GetElement(0, 2);
							matHx_j.SetElement(0, s_k, sum_posvel);
						}
						int beginPara = 6;
						// 兼容多光压模型计算, 谷德峰
						if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc)
						{
							for(int j = 0; j < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size() * count_SolarPressurePara); j++)
							{
								double sum_solar = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j) * matH_pos_j.GetElement(0, 0) 
												 + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j) * matH_pos_j.GetElement(0, 1)
												 + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j) * matH_pos_j.GetElement(0, 2);
								matHx_j.SetElement(0, beginPara + j, sum_solar);
							}
							beginPara += count_SolarPressurePara * count_SolarPressureParaList;
						}
						if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
						{// 经验力
							int count_sub =  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
											 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2  
											 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 2; 
							for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
							{
								int i_sub = 0;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
								{
									double sum_cr = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_cr);

									double sum_sr = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_sr);
								}
								i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
								{
									double sum_ct = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_ct);

									double sum_st = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_st);
								}
								i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
								{
									double sum_cn = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_cn);

									double sum_sn = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_sn);
								}
							}
							beginPara += count_sub * count_EmpiricalForceParaList;
						}
						if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
						{// 经验力
							int count_sub =  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
											 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3  
											 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 3; 
							for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
							{
								int i_sub = 0;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
								{
									double sum_a0 = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
								i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
								{
									double sum_a0 = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
								i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
								{
									double sum_a0 = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr = jt_obs->second.tqRtPartial.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j.GetElement(0, 2);
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
							}
							beginPara += count_sub * count_EmpiricalForceParaList;
						}

						// 卫星延迟参数
						if(it_Sat->second.on_EstSatZeroDelay)
							matHx_j.SetElement(0, it_Sat->second.count_EstDynParameters, w);
						
						// 测站延迟参数
						if(m_tqNetPODDefine.on_EstStaZeroDelay)
							matHx_j.SetElement(0, it_Sat->second.count_EstDynParameters + it_Sat->second.count_EstSatZeroDelay + it_UXBSta->second.id_ZeroDelay, w); 
						
						it_Sat->second.n_xx = it_Sat->second.n_xx + matHx_j.Transpose() * matHx_j;
                        it_Sat->second.nx = it_Sat->second.nx + matHx_j.Transpose() * (w * jt_obs->second.oc_range);
					}

					if(m_tqNetPODDefine.on_Used_Doppler)
					{
						// 后续请李康康整理 ?????? 
						POS3D vecLos_t0 = vectorNormal(jt_obs->second.tqOrb_t0.pos - jt_obs->second.pos_J2000_t0);
						POS3D vecLos_t1 = vectorNormal(jt_obs->second.tqOrb_t1.pos - jt_obs->second.pos_J2000_t1);
						//double w = jt_obs->second.rw_doppler * jt_obs->second.weight_doppler * m_tqNetPODDefine.weight_doppler;
						double w = jt_obs->second.rw_doppler * m_tqNetPODDefine.weight_doppler;
						Matrix matH_pos_j_t0(1, 3);
						matH_pos_j_t0.SetElement(0, 0, w * vecLos_t0.x);
						matH_pos_j_t0.SetElement(0, 1, w * vecLos_t0.y);
						matH_pos_j_t0.SetElement(0, 2, w * vecLos_t0.z);
						Matrix matH_pos_j_t1(1, 3);
						matH_pos_j_t1.SetElement(0, 0, w * vecLos_t1.x);
						matH_pos_j_t1.SetElement(0, 1, w * vecLos_t1.y);
						matH_pos_j_t1.SetElement(0, 2, w * vecLos_t1.z);
						Matrix matHx_j(1, it_Sat->second.count_EstParameters);
						double dt = (jt_obs->second.t1 - jt_obs->second.t0);
						for(int s_k = 0; s_k < 6; s_k++)
						{// 初始位置速度
							double sum_posvel = jt_obs->second.tqRtPartial_t1.GetElement(0, s_k) * matH_pos_j_t1.GetElement(0, 0) 
											  + jt_obs->second.tqRtPartial_t1.GetElement(1, s_k) * matH_pos_j_t1.GetElement(0, 1)
											  + jt_obs->second.tqRtPartial_t1.GetElement(2, s_k) * matH_pos_j_t1.GetElement(0, 2)
											  - jt_obs->second.tqRtPartial_t0.GetElement(0, s_k) * matH_pos_j_t0.GetElement(0, 0)
											  - jt_obs->second.tqRtPartial_t0.GetElement(1, s_k) * matH_pos_j_t0.GetElement(0, 1)
											  - jt_obs->second.tqRtPartial_t0.GetElement(2, s_k) * matH_pos_j_t0.GetElement(0, 2);
							       sum_posvel = sum_posvel / dt;
							matHx_j.SetElement(0, s_k, sum_posvel);
						}

						int beginPara = 6;
						// 兼容多光压模型计算, 谷德峰
						if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc)
						{
							for(int j = 0; j < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size() * count_SolarPressurePara); j++)
							{
								double sum_solar =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j) * matH_pos_j_t1.GetElement(0, 0) 
												 + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j) * matH_pos_j_t1.GetElement(0, 1)
												 + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j) * matH_pos_j_t1.GetElement(0, 2)
								                 - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j) * matH_pos_j_t0.GetElement(0, 0) 
												 - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j) * matH_pos_j_t0.GetElement(0, 1)
												 - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j) * matH_pos_j_t0.GetElement(0, 2)) / dt;
								matHx_j.SetElement(0, beginPara + j, sum_solar);
							}
							beginPara += count_SolarPressurePara * count_SolarPressureParaList;
						}
						if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
						{// 经验力
							int count_sub =  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
											 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2  
											 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 2; 
							for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
							{
								int i_sub = 0;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
								{
									double sum_cr =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_cr);

									double sum_sr =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_sr);
								}
								i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
								{
									double sum_ct =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_ct);

									double sum_st =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_st);
								}
								i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
								{
									double sum_cn =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_cn);

									double sum_sn =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_sn);
								}
							}
							beginPara += count_sub * count_EmpiricalForceParaList;
						}
						if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
						{// 经验力
							int count_sub =  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
											 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3  
											 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 3; 
							for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
							{
								int i_sub = 0;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
								{
									double sum_a0 =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
								i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
								{
									double sum_a0 =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
								i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3 + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3;
								if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
								{
									double sum_a0 =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 0) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 0, sum_a0);

									double sum_cr =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 1) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 1, sum_cr);

									double sum_sr =(jt_obs->second.tqRtPartial_t1.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t1.GetElement(0, 0) 
												  + jt_obs->second.tqRtPartial_t1.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t1.GetElement(0, 1)
												  + jt_obs->second.tqRtPartial_t1.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t1.GetElement(0, 2)
												  - jt_obs->second.tqRtPartial_t0.GetElement(0, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t0.GetElement(0, 0) 
												  - jt_obs->second.tqRtPartial_t0.GetElement(1, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t0.GetElement(0, 1)
												  - jt_obs->second.tqRtPartial_t0.GetElement(2, beginPara + j * count_sub + i_sub + 2) * matH_pos_j_t0.GetElement(0, 2)) / dt;
									matHx_j.SetElement(0, beginPara + j * count_sub + i_sub + 2, sum_sr);
								}
							}
							beginPara += count_sub * count_EmpiricalForceParaList;
						}

						// 卫星延迟参数
						if(it_Sat->second.on_EstSatZeroDelay)
							matHx_j.SetElement(0, it_Sat->second.count_EstDynParameters, 0.0); // 需要注意仅用测速数据不能估计卫星延迟参数
						
						// 测站延迟参数
						if(m_tqNetPODDefine.on_EstStaZeroDelay)
							matHx_j.SetElement(0, it_Sat->second.count_EstDynParameters + it_Sat->second.count_EstSatZeroDelay + it_UXBSta->second.id_ZeroDelay, 0.0); // 需要注意仅用测速数据不能估计测站延迟参数

						it_Sat->second.n_xx = it_Sat->second.n_xx + matHx_j.Transpose() * matHx_j;
						it_Sat->second.nx = it_Sat->second.nx + matHx_j.Transpose() * (w * jt_obs->second.oc_doppler);

						//// 仅使用测速数据不加权
						//if(m_tqNetPODDefine.on_Used_Range)
						//{
						//	// 测试代码 给测速数据加权
						//	it_Sat->second.n_xx = it_Sat->second.n_xx + matHx_j.Transpose() * matHx_j * m_tqNetPODDefine.weight_doppler * m_tqNetPODDefine.weight_doppler;
						//	it_Sat->second.nx = it_Sat->second.nx + matHx_j.Transpose() * (w * jt_obs->second.oc_doppler * m_tqNetPODDefine.weight_doppler * m_tqNetPODDefine.weight_doppler);
						//}
						//else
						//{
						//	it_Sat->second.n_xx = it_Sat->second.n_xx + matHx_j.Transpose() * matHx_j;
						//	it_Sat->second.nx = it_Sat->second.nx + matHx_j.Transpose() * (w * jt_obs->second.oc_doppler);	
						//}
					}
				}
			}

			// 添加伪方程-光压参数
			for(int s_k = 0; s_k < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size()); s_k++)
			{
				double weight_solar = 1.0E+12;	
				if(it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
				{
					if(!it_Sat->second.on_SRP9_D0)
					{// △D0[0]  = -(D0*) + ε									
						int index_D0 = 6 + 9 * (int)s_k + 0;
						it_Sat->second.n_xx.SetElement(index_D0, index_D0,      it_Sat->second.n_xx.GetElement(index_D0, index_D0)     + weight_solar * weight_solar);
						it_Sat->second.nx.SetElement(index_D0, 0,               it_Sat->second.nx.GetElement(index_D0, 0)              - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].D0);
					}
					if(!it_Sat->second.on_SRP9_DC1)
					{// △DCOS[1]  = -(DCOS*) + ε								
						int index_DCOS = 6 + 9 * (int)s_k + 1;
						it_Sat->second.n_xx.SetElement(index_DCOS, index_DCOS,  it_Sat->second.n_xx.GetElement(index_DCOS, index_DCOS) + weight_solar * weight_solar);
						it_Sat->second.nx.SetElement(index_DCOS, 0,             it_Sat->second.nx.GetElement(index_DCOS, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].DC1);
					}
					if(!it_Sat->second.on_SRP9_DS1)
					{// △DSIN[2]  = -(DSIN*) + ε								
						int index_DSIN = 6 + 9 * (int)s_k + 2;
						it_Sat->second.n_xx.SetElement(index_DSIN, index_DSIN,  it_Sat->second.n_xx.GetElement(index_DSIN, index_DSIN) + weight_solar * weight_solar);
						it_Sat->second.nx.SetElement(index_DSIN, 0,             it_Sat->second.nx.GetElement(index_DSIN, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].DS1);
					}
					if(!it_Sat->second.on_SRP9_Y0)
					{// △Y0[3]  = -(Y0*) + ε								
						int index_Y0 = 6 + 9 * (int)s_k + 3;
						it_Sat->second.n_xx.SetElement(index_Y0, index_Y0,      it_Sat->second.n_xx.GetElement(index_Y0, index_Y0)     + weight_solar * weight_solar);
						it_Sat->second.nx.SetElement(index_Y0, 0,               it_Sat->second.nx.GetElement(index_Y0, 0)              - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].Y0);
					}
					if(!it_Sat->second.on_SRP9_YC1)
					{// △YCOS[4]  = -(YCOS*) + ε								
						int index_YCOS = 6 + 9 * (int)s_k + 4;
						it_Sat->second.n_xx.SetElement(index_YCOS, index_YCOS,  it_Sat->second.n_xx.GetElement(index_YCOS, index_YCOS) + weight_solar * weight_solar);
						it_Sat->second.nx.SetElement(index_YCOS, 0,             it_Sat->second.nx.GetElement(index_YCOS, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].YC1);
					}
					if(!it_Sat->second.on_SRP9_YS1)
					{// △YSIN[5]  = -(YSIN*) + ε								
						int index_YSIN = 6 + 9 * (int)s_k + 5;
						it_Sat->second.n_xx.SetElement(index_YSIN, index_YSIN,  it_Sat->second.n_xx.GetElement(index_YSIN, index_YSIN) + weight_solar * weight_solar);
						it_Sat->second.nx.SetElement(index_YSIN, 0,             it_Sat->second.nx.GetElement(index_YSIN, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].YS1);
					}
					if(!it_Sat->second.on_SRP9_B0)
					{// △B0[6]  = -(B0*) + ε								
						int index_B0 = 6 + 9 * (int)s_k + 6;
						it_Sat->second.n_xx.SetElement(index_B0, index_B0,      it_Sat->second.n_xx.GetElement(index_B0, index_B0)     + weight_solar * weight_solar);
						it_Sat->second.nx.SetElement(index_B0, 0,               it_Sat->second.nx.GetElement(index_B0, 0)              - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].B0);
					}
					if(!it_Sat->second.on_SRP9_BC1)
					{// △BCOS[7]  = -(BCOS*) + ε								
						int index_BCOS = 6 + 9 * (int)s_k + 7;
						it_Sat->second.n_xx.SetElement(index_BCOS, index_BCOS,  it_Sat->second.n_xx.GetElement(index_BCOS, index_BCOS) + weight_solar * weight_solar);
						it_Sat->second.nx.SetElement(index_BCOS, 0,             it_Sat->second.nx.GetElement(index_BCOS, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].BC1);
					}
					if(!it_Sat->second.on_SRP9_BS1)
					{// △BSIN[8]  = -(BSIN*) + ε								
						int index_BSIN = 6 + 9 * (int)s_k + 8;
						it_Sat->second.n_xx.SetElement(index_BSIN, index_BSIN,  it_Sat->second.n_xx.GetElement(index_BSIN, index_BSIN) + weight_solar * weight_solar);
						it_Sat->second.nx.SetElement(index_BSIN, 0,             it_Sat->second.nx.GetElement(index_BSIN, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_k].BS1);
					}
				}
			}

			if(m_tqNetPODDefine.on_EstStaZeroDelay)
			{
				// 测试程序，添加伪方程-测站延迟参数
				for(TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = it_Sat->second.mapUXBStaDatum.begin();it_UXBSta != it_Sat->second.mapUXBStaDatum.end();it_UXBSta++)
				{
					double weight_ZeroDelay = 0.0;
					if(!it_UXBSta->second.on_EstStaZeroDelay) // 对不估计测站时延参数的测站增加约束
						weight_ZeroDelay = 1.0E+4;            // 伪方程的观测权值不应该太大，防止大数吃小数
					int index_ZeroDelay = it_Sat->second.count_EstDynParameters + it_Sat->second.count_EstSatZeroDelay + it_UXBSta->second.id_ZeroDelay;
					it_Sat->second.n_xx.SetElement(index_ZeroDelay, index_ZeroDelay, it_Sat->second.n_xx.GetElement(index_ZeroDelay, index_ZeroDelay) + weight_ZeroDelay * weight_ZeroDelay);
					it_Sat->second.nx.SetElement(index_ZeroDelay, 0,                 it_Sat->second.nx.GetElement(index_ZeroDelay, 0)                 - weight_ZeroDelay * weight_ZeroDelay * it_UXBSta->second.zeroDelay_Est);
				}
			}
		}

		void TQSatNetDynPOD::updateSat_SOL(TQNETPOD_SatDatumMap::iterator it_Sat)
		{
			//// 测试代码 打印出法化矩阵及其逆和估计参数改进量 暂时添加
			//FILE * fp = fopen("C:\\软件开发\\NUDTTK-TQ\\release\\TQ\\TQPOD\\法化矩阵及其逆.txt","a+");
			//int n = 6;
			//if(it_Sat->second.on_EstSatZeroDelay)
			//	n = 7;
			//if(m_tqNetPODDefine.on_EstStaZeroDelay&&!(it_Sat->second.on_EstSatZeroDelay))
			//	n = 9;
			//if(m_tqNetPODDefine.on_EstStaZeroDelay&&it_Sat->second.on_EstSatZeroDelay)
			//	n = 10;

			//for(int k = 0; k < n;k++)  
			//{
			//	fprintf(fp,"it_Sat->second.n_xx.GetElement(%d,n):",k);

			//	for(int j= 0;j < n;j++)
			//	{
			//		
			//		fprintf(fp,"%20.10lf ",it_Sat->second.n_xx.GetElement(k,j));
			//	}
			//	fprintf(fp,"\n");
			//}

			//for(int k = 0; k < n;k++)
			//{
			//	fprintf(fp,"it_Sat->second.n_xx.Inv_Ssgj().GetElement(%d,n):",k);
			//	for(int j= 0; j < n;j++)
			//	{
			//		fprintf(fp,"%20.10lf ",it_Sat->second.n_xx.Inv_Ssgj().GetElement(k,j));
			//	}
			//	fprintf(fp,"\n");
			//}

			//for(int k = 0; k < n; k++)
			//{

			//	fprintf(fp,"it_Sat->second.nx.GetElement(%d,0) = %20.10lf\n",k,it_Sat->second.nx.GetElement(k,0));
		
			//}
			
			it_Sat->second.dx = it_Sat->second.n_xx.Inv_Ssgj() * it_Sat->second.nx;

			//for(int k = 0; k < n; k++)
			//{

			//	fprintf(fp,"it_Sat->second.dx.GetElement(%d,0) = %20.10lf\n",k,it_Sat->second.dx.GetElement(k,0));
		
			//}

			//fclose(fp);
		}

		void TQSatNetDynPOD::updateNET_ISLArc(TQNETPOD_InterSatLink &satISL)
		{
			//// 后续待整理 ??????
			//// 测试代码，李康康
			//TQNETPOD_SatDatumMap::iterator it_A = m_mapSatDatum.find(satISL.satName_A);
			//TQNETPOD_SatDatumMap::iterator it_B = m_mapSatDatum.find(satISL.satName_B);
			//vector<TQNETPOD_ISLArcElement> obsList;
			//for(size_t s_i = 0; s_i < satISL.tqISLFile.m_data.size(); ++s_i)
			//{
			//	UTC t_UTC = satISL.tqISLFile.m_data[s_i].t;
			//	double obs_code = satISL.tqISLFile.m_data[s_i].getCode();
			//	double obs_phase = satISL.tqISLFile.m_data[s_i].getPhase();
			//	TimePosVel interpOrbit_A,interpOrbit_B;
			//	if(it_A->second.getEphemeris(t_UTC, interpOrbit_A)
			//	&& it_B->second.getEphemeris(t_UTC, interpOrbit_B))
			//	{
			//		TQNETPOD_ISLArcElement obsElement;
			//		double range_0 = vectorMagnitude(interpOrbit_B.pos - interpOrbit_A.pos);
			//		obsElement.t = t_UTC;
			//		obsElement.range_0 = range_0;
			//		obsElement.obs_code = obs_code;
			//		obsElement.obs_phase = obs_phase;
			//		obsElement.oc_code   = obsElement.obs_code - obsElement.range_0;
			//		obsElement.oc_phase  = obsElement.obs_phase - obsElement.range_0;
			//		obsElement.rw_code   = 1.0;
			//		obsElement.rw_phase  = 1.0;
			//		obsList.push_back(obsElement);
			//	}
			//}

			//satISL.ISLArcList.clear();
			//size_t k   = 0; // 记录新弧段起始点
			//size_t k_i = k; // 记录新弧段终止点
			//while(1)
			//{
			//	if(k_i + 1 >= obsList.size())// k_i 为时间序列终点
			//		goto newArc;
			//	else
			//	{
			//		// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?
			//		if(fabs(obsList[k_i + 1].oc_phase  - obsList[k_i].oc_phase) <= 10
			//			|| (obsList[k_i + 1].t - obsList[k_i].t) <= 60 * 60)// 数据间断超过1小时，认为弧段断开
			//		{
			//			++k_i;
			//			continue;
			//		}
			//		else
			//			goto newArc;
			//	}
			//	newArc:  // 本弧段[k，k_i]数据处理 
			//	{
			//		TQNETPOD_ISLArc newISLArc;
			//		newISLArc.obsList.clear();
			//		newISLArc.ambiguity = 0;
			//		for(size_t s_k = k; s_k <= k_i; s_k++)
			//		{
			//			newISLArc.obsList.push_back(obsList[s_k]);
			//			newISLArc.ambiguity += obsList[s_k].oc_phase;
			//		}
			//		newISLArc.ambiguity /= (k_i - k + 1);
			//		if(int(newISLArc.obsList.size()) > m_tqNetPODDefine.min_ArcPointCount)
			//		{
			//			satISL.ISLArcList.push_back(newISLArc);
			//		}

			//		if(k_i + 1 >= obsList.size())
			//			break;
			//		else
			//		{
			//			k   = k_i + 1;
			//			k_i = k;
			//			continue;
			//		}
			//	}
			//}

			// 后续待整理 ??????
			// 测试代码，李康康
			TQNETPOD_SatDatumMap::iterator it_A = m_mapSatDatum.find(satISL.satName_A);
			TQNETPOD_SatDatumMap::iterator it_B = m_mapSatDatum.find(satISL.satName_B);
			vector<TQNETPOD_ISLArcElement> obsList;
			for(size_t s_i = 0; s_i < satISL.ISLArcList.size(); ++s_i)
			{
				for(size_t s_j = 0; s_j < satISL.ISLArcList[s_i].obsList.size(); ++s_j)
				{
					UTC t_UTC = satISL.ISLArcList[s_i].obsList[s_j].t;
					double obs_code = satISL.ISLArcList[s_i].obsList[s_j].obs_code;
					double obs_phase = satISL.ISLArcList[s_i].obsList[s_j].obs_phase;
					TimePosVel interpOrbit_A,interpOrbit_B;
					if(it_A->second.getEphemeris(t_UTC, interpOrbit_A)
					&& it_B->second.getEphemeris(t_UTC, interpOrbit_B))
					{
						TQNETPOD_ISLArcElement obsElement;
						double range_0 = vectorMagnitude(interpOrbit_B.pos - interpOrbit_A.pos);
						obsElement.t = t_UTC;
						obsElement.range_0 = range_0;
						obsElement.obs_code = obs_code;
						obsElement.obs_phase = obs_phase;
						obsElement.oc_code   = obsElement.obs_code - obsElement.range_0;
						obsElement.oc_phase  = obsElement.obs_phase - obsElement.range_0;
						obsElement.rw_code   = 1.0;
						obsElement.rw_phase  = 1.0;
						obsList.push_back(obsElement);
					}
				}
			}

			satISL.ISLArcList.clear();
			size_t k   = 0; // 记录新弧段起始点
			size_t k_i = k; // 记录新弧段终止点
			while(1)
			{
				if(k_i + 1 >= obsList.size())// k_i 为时间序列终点
					goto newArc;
				else
				{
					// 判断 k_i + 1 与 k_i 是否位于同一跟踪弧段?
					if(fabs(obsList[k_i + 1].oc_code - obsList[k_i].oc_code) <= 10
						|| fabs(obsList[k_i + 1].oc_phase  - obsList[k_i].oc_phase) <= 10
						|| (obsList[k_i + 1].t - obsList[k_i].t) <= 60 * 60)// 数据间断超过1小时，认为弧段断开
					{
						++k_i;
						continue;
					}
					else
						goto newArc;
				}
				newArc:  // 本弧段[k，k_i]数据处理 
				{
					TQNETPOD_ISLArc newISLArc;
					newISLArc.obsList.clear();
					newISLArc.ambiguity = 0;
					for(size_t s_k = k; s_k <= k_i; s_k++)
					{
						newISLArc.obsList.push_back(obsList[s_k]);
						newISLArc.ambiguity += obsList[s_k].oc_phase;
					}
					newISLArc.ambiguity /= (k_i - k + 1);
					if(int(newISLArc.obsList.size()) > m_tqNetPODDefine.min_ArcPointCount)
					{
						satISL.ISLArcList.push_back(newISLArc);
					}

					if(k_i + 1 >= obsList.size())
						break;
					else
					{
						k   = k_i + 1;
						k_i = k;
						continue;
					}
				}
			}
		}

		void TQSatNetDynPOD::updateNet_NEQ_SQL()
		{
			// 利用NEQ重新构造整网法方程
			int count_EstParameters_NET = 0;
			for(TQNETPOD_SatDatumMap::iterator it_Sat = m_mapSatDatum.begin(); it_Sat != m_mapSatDatum.end(); ++it_Sat)
			{
				it_Sat->second.n0_EstParameters = count_EstParameters_NET;
				count_EstParameters_NET += it_Sat->second.count_EstParameters;
			}

			// Inter satellite link
			int count_EstParameters_ISL = 0;
			if(m_tqNetPODDefine.on_Used_ISL_Phase)
			{
				for(size_t s_b = 0; s_b < m_ISLList.size(); s_b++)
				{
					m_ISLList[s_b].n0_ISLAmbiguity = count_EstParameters_NET + count_EstParameters_ISL;
					count_EstParameters_ISL += int(m_ISLList[s_b].ISLArcList.size());
				}
			}
			count_EstParameters_NET += count_EstParameters_ISL;

			Matrix n_xx(count_EstParameters_NET, count_EstParameters_NET);
			Matrix nx(count_EstParameters_NET, 1);
			for(TQNETPOD_SatDatumMap::iterator it_Sat = m_mapSatDatum.begin(); it_Sat != m_mapSatDatum.end(); ++it_Sat)
			{
				for(int s_i = 0; s_i < it_Sat->second.count_EstParameters; s_i++)
				{
					nx.SetElement(it_Sat->second.n0_EstParameters + s_i, 0, it_Sat->second.nx.GetElement(s_i, 0));
					for(int s_j = 0; s_j < it_Sat->second.count_EstParameters; s_j++)
					{
						n_xx.SetElement(it_Sat->second.n0_EstParameters + s_i, it_Sat->second.n0_EstParameters + s_j, 
										it_Sat->second.n_xx.GetElement(s_i, s_j));
					}
				}
			}

			// 整网同一测站零值延迟相等的约束
			// 注: 多星定轨时，同一测站的零值延迟公用一个参数，此时增加相对约束
			if(m_tqNetPODDefine.on_EstStaZeroDelay)
			{
				// 遍历所有测站
				vector<string> staNameList;
				for(TQNETPOD_SatDatumMap::iterator it_Sat = m_mapSatDatum.begin(); it_Sat != m_mapSatDatum.end(); ++it_Sat)
				{
					for(TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = it_Sat->second.mapUXBStaDatum.begin(); it_UXBSta != it_Sat->second.mapUXBStaDatum.end(); ++it_UXBSta)
					{
						bool bfind = false;
						for(size_t s_i = 0; s_i < staNameList.size(); s_i++)
						{
							if(staNameList[s_i] == it_UXBSta->second.nameSta)
							{
								bfind = true;
								break;
							}
						}
						if(!bfind)
							staNameList.push_back(it_UXBSta->second.nameSta);
					}
				}
				for(size_t s_i = 0; s_i < staNameList.size(); s_i++)
				{
					// 查找相对约束对应的卫星 A 和 B
					TQNETPOD_SatDatumMap::iterator it_Sat_A, it_Sat_B;
					TQNETPOD_UXBStaDatumMap::iterator it_UXBSta_A;
					int id_ZeroDelay_A = -1;
					for(TQNETPOD_SatDatumMap::iterator it_Sat = m_mapSatDatum.begin(); it_Sat != m_mapSatDatum.end(); ++it_Sat)
					{
						TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = it_Sat->second.mapUXBStaDatum.find(staNameList[s_i]);
						if(it_UXBSta != it_Sat->second.mapUXBStaDatum.end())
						{
							it_Sat_A = it_Sat;
							it_UXBSta_A = it_UXBSta;
							it_Sat_B = it_Sat++;
							break;
						}
					}

					for(it_Sat_B; it_Sat_B != m_mapSatDatum.end(); ++it_Sat_B)
					{
						TQNETPOD_UXBStaDatumMap::iterator it_UXBSta = it_Sat_B->second.mapUXBStaDatum.find(staNameList[s_i]);
						if(it_UXBSta != it_Sat_B->second.mapUXBStaDatum.end())
						{
							// 添加相对约束 : ZeroDelay_A - ZeroDelay_B = -dx_ZeroDelay + ε
							if(id_ZeroDelay_A != -1)
							{
								double weight_ZeroDelay = 1.0E+4; // 防止大数吃小数
								int indexA = it_Sat_A->second.n0_EstParameters
									       + it_Sat_A->second.count_EstDynParameters
										   + it_Sat_A->second.count_EstSatZeroDelay
										   + id_ZeroDelay_A;
                                int indexB = it_Sat_B->second.n0_EstParameters
									       + it_Sat_B->second.count_EstDynParameters
										   + it_Sat_B->second.count_EstSatZeroDelay
										   + it_UXBSta->second.id_ZeroDelay;		
								double dx_ZeroDelay = it_UXBSta_A->second.zeroDelay_0 + it_UXBSta_A->second.zeroDelay_Est
									                - it_UXBSta->second.zeroDelay_0   - it_UXBSta->second.zeroDelay_Est;
								nx.SetElement(indexA, 0, nx.GetElement(indexA, 0) - weight_ZeroDelay * weight_ZeroDelay * dx_ZeroDelay);
								nx.SetElement(indexB, 0, nx.GetElement(indexB, 0) + weight_ZeroDelay * weight_ZeroDelay * dx_ZeroDelay);
								n_xx.SetElement(indexA, indexA, n_xx.GetElement(indexA, indexA) + weight_ZeroDelay * weight_ZeroDelay);
								n_xx.SetElement(indexA, indexB, n_xx.GetElement(indexA, indexB) - weight_ZeroDelay * weight_ZeroDelay);
								n_xx.SetElement(indexB, indexA, n_xx.GetElement(indexB, indexA) - weight_ZeroDelay * weight_ZeroDelay);
								n_xx.SetElement(indexB, indexB, n_xx.GetElement(indexB, indexB) + weight_ZeroDelay * weight_ZeroDelay);
							}
						}
					}
				}
			}

			// Inter satellite link
			if(m_tqNetPODDefine.on_Used_ISL_Code || m_tqNetPODDefine.on_Used_ISL_Phase)
			{
				for(size_t s_b = 0; s_b < m_ISLList.size(); s_b++)
				{
					TQNETPOD_SatDatumMap::iterator it_A = m_mapSatDatum.find(m_ISLList[s_b].satName_A);
					TQNETPOD_SatDatumMap::iterator it_B = m_mapSatDatum.find(m_ISLList[s_b].satName_B);
					for(size_t s_k = 0; s_k < m_ISLList[s_b].ISLArcList.size(); s_k++)
					{
						for(size_t s_j = 0; s_j < m_ISLList[s_b].ISLArcList[s_k].obsList.size(); s_j++)
						{
							UTC t = m_ISLList[s_b].ISLArcList[s_k].obsList[s_j].t;
							TimePosVel interpOrbit[2];
							Matrix interpRtPartial[2];
							if(it_A->second.getEphemeris(t, interpOrbit[0])
							&& it_B->second.getEphemeris(t, interpOrbit[1])
							&& it_A->second.getInterpRtPartial(t, interpRtPartial[0])
							&& it_B->second.getInterpRtPartial(t, interpRtPartial[1]))
							{// 分别对A-[0]、B-[1]的轨道动力学参数求偏导: 初始位置速度、光压、经验力

					//			for(int k = 0; k <= 1; k++)
					//			{
					//				// 后续待整理 ??????
					//				//m_staBaselineList[s_b].KBRArcList[s_k].obsList[s_j].range_0;
					//				//m_staBaselineList[s_b].KBRArcList[s_k].obsList[s_j].res;

					//			}
								// 测试代码 李康康 将伪码测距和相位测距分开
								// 这里先暂时考虑伪码和相位测距
								if(m_tqNetPODDefine.on_Used_ISL_Code)
								{
									Matrix matHx_pos(1, 3);
									Matrix matHx_k(1, count_EstParameters_NET);
									double w = m_ISLList[s_b].ISLArcList[s_k].obsList[s_j].rw_code * m_tqNetPODDefine.weight_code;
									//POS3D vecLos = vectorNormal(interpOrbit[0].pos - interpOrbit[1].pos);
									POS3D vecLos;
									//double distance = sqrt(pow(interpOrbit[0].pos.x - interpOrbit[1].pos.x, 2)
									//				     + pow(interpOrbit[0].pos.y - interpOrbit[1].pos.y, 2)
									//				     + pow(interpOrbit[0].pos.z - interpOrbit[1].pos.z, 2));

									//vecLos.x = (interpOrbit[0].pos.x - interpOrbit[1].pos.x) / distance;
									//vecLos.y = (interpOrbit[0].pos.y - interpOrbit[1].pos.y) / distance;
									//vecLos.z = (interpOrbit[0].pos.z - interpOrbit[1].pos.z) / distance;
									TQNETPOD_SatDatumMap::iterator it_Sat = it_A;
									for(int k = 0; k <= 1; ++k)
									{
										if(k == 1)
											it_Sat = it_B;
										double distance = sqrt(pow(interpOrbit[0].pos.x - interpOrbit[1].pos.x, 2)
															 + pow(interpOrbit[0].pos.y - interpOrbit[1].pos.y, 2)
															 + pow(interpOrbit[0].pos.z - interpOrbit[1].pos.z, 2));

										vecLos.x = (interpOrbit[0].pos.x - interpOrbit[1].pos.x) / distance;
										vecLos.y = (interpOrbit[0].pos.y - interpOrbit[1].pos.y) / distance;
										vecLos.z = (interpOrbit[0].pos.z - interpOrbit[1].pos.z) / distance;

										if(k == 1) // 表示对卫星B求偏导
										{
											vecLos.x = -vecLos.x;
											vecLos.y = -vecLos.y;
											vecLos.z = -vecLos.z;
										}

										// 位置偏导数
										matHx_pos.SetElement(0, 0, vecLos.x * w);
										matHx_pos.SetElement(0, 1, vecLos.y * w);
										matHx_pos.SetElement(0, 2, vecLos.z * w);

										// 构造H矩阵
										TimePosVel interpOrbit_k = interpOrbit[0];
										Matrix interpRtPartial_k = interpRtPartial[0];
										if(k == 1)
										{
											interpOrbit_k = interpOrbit[1];
											interpRtPartial_k = interpRtPartial[1];
										}

										// 卫星动力学参数
										for(int j = 0; j < 6; ++j)
										{// 初始位置速度
											double sum_posvel = interpRtPartial_k.GetElement(0, j) * matHx_pos.GetElement(0, 0) 
															  + interpRtPartial_k.GetElement(1, j) * matHx_pos.GetElement(0, 1)
														      + interpRtPartial_k.GetElement(2, j) * matHx_pos.GetElement(0, 2);
											matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + j, sum_posvel);
										}

										// 兼容多光压模型计算
										int beginPara = it_Sat->second.n0_EstParameters + 6;
										int count_SolarPressureParaList = 0;
										if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc)
											count_SolarPressureParaList = int(it_Sat->second.dynDatum_Est.solarPressureParaList.size());
										int	count_SolarPressurePara = it_Sat->second.dynDatum_Est.getSolarPressureParaCount();
										if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc)
										{
											for(int j = 0; j < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size() * count_SolarPressurePara);++j)
											{
												double sum_solar = interpRtPartial_k.GetElement(0, beginPara + j) * matHx_pos.GetElement(0, 0) 
																 + interpRtPartial_k.GetElement(1, beginPara + j) * matHx_pos.GetElement(0, 1)
														         + interpRtPartial_k.GetElement(2, beginPara + j) * matHx_pos.GetElement(0, 2);
												matHx_k.SetElement(0, beginPara + j, sum_solar);
											}
											beginPara += count_SolarPressurePara * count_SolarPressureParaList;
										}

										if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
										{// 经验力
											int count_sub = + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
															+ int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2  
															+ int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 2;

											for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
											{
												int i_sub = 0;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);
												}
												i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);
												}
												i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2
													  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);
												}
											}
										}
										if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
										{// 经验力
											int count_sub = + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
															+ int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3  
															+ int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 3;

											for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
											{
												int i_sub = 0;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);

													double sum_ar = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2, sum_ar);
												}
												i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);

													double sum_ar = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2, sum_ar);
												}
												i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3
													  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);

													double sum_ar = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2, sum_ar);
												}
											}
										}
									}// 卫星循环
									n_xx = n_xx + matHx_k.Transpose() * matHx_k;
									nx = nx + matHx_k.Transpose() * (w * m_ISLList[s_b].ISLArcList[s_k].obsList[s_j].oc_code);
								}// 采用伪码数据

								// 使用相位激光测距数据
								if(m_tqNetPODDefine.on_Used_ISL_Phase)
								{
									Matrix matHx_pos(1, 3); // 观测量偏导矩阵
									Matrix matHx_k(1, count_EstParameters_NET);
									double w = m_ISLList[s_b].ISLArcList[s_k].obsList[s_j].rw_phase * m_tqNetPODDefine.weight_phase;
									for(int k = 0; k <= 1; ++k)
									{
										TQNETPOD_SatDatumMap::iterator it_Sat = it_A;
										if(k == 1)
											it_Sat = it_B;
										
										POS3D vecLos;
										double distance = sqrt(pow(interpOrbit[0].pos.x - interpOrbit[1].pos.x, 2)
														     + pow(interpOrbit[0].pos.y - interpOrbit[1].pos.y, 2)
														     + pow(interpOrbit[0].pos.z - interpOrbit[1].pos.z, 2));

										vecLos.x = (interpOrbit[0].pos.x - interpOrbit[1].pos.x) / distance;
										vecLos.y = (interpOrbit[0].pos.y - interpOrbit[1].pos.y) / distance;
										vecLos.z = (interpOrbit[0].pos.z - interpOrbit[1].pos.z) / distance;

										if(k == 1) // 表示对卫星B求偏导
										{
											vecLos.x = -vecLos.x;
											vecLos.y = -vecLos.y;
											vecLos.z = -vecLos.z;
										}

										// 位置偏导数
										matHx_pos.SetElement(0, 0, vecLos.x * w);
										matHx_pos.SetElement(0, 1, vecLos.y * w);
										matHx_pos.SetElement(0, 2, vecLos.z * w);

										// 构造H矩阵
										TimePosVel interpOrbit_k = interpOrbit[0];
										Matrix interpRtPartial_k = interpRtPartial[0];

										if(k == 1)
										{
											interpOrbit_k = interpOrbit[1];
											interpRtPartial_k = interpRtPartial[1];
										}

										// 卫星动力学参数
										for(int j = 0; j < 6; ++j)
										{// 初始位置速度
											double sum_posvel = interpRtPartial_k.GetElement(0, j) * matHx_pos.GetElement(0, 0) 
															  + interpRtPartial_k.GetElement(1, j) * matHx_pos.GetElement(0, 1)
														      + interpRtPartial_k.GetElement(2, j) * matHx_pos.GetElement(0, 2);
											matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + j, sum_posvel);
										}

										// 兼容多光压模型计算
										int beginPara = it_Sat->second.n0_EstParameters + 6;
										int count_SolarPressureParaList = 0;
										if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc)
											int count_SolarPressureParaList = int(it_Sat->second.dynDatum_Est.solarPressureParaList.size());
										int count_SolarPressurePara = it_Sat->second.dynDatum_Est.getSolarPressureParaCount();
										if(it_Sat->second.dynDatum_Est.bOn_SolarPressureAcc)
										{
											for(int j = 0; j < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size() * count_SolarPressurePara);++j)
											{
												double sum_solar = interpRtPartial_k.GetElement(0, beginPara + j) * matHx_pos.GetElement(0, 0) 
																 + interpRtPartial_k.GetElement(1, beginPara + j) * matHx_pos.GetElement(0, 1)
														         + interpRtPartial_k.GetElement(2, beginPara + j) * matHx_pos.GetElement(0, 2);
												matHx_k.SetElement(0, beginPara + j, sum_solar);
											}
											beginPara += count_SolarPressurePara * count_SolarPressureParaList;
										}

										if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_COSSIN)
										{// 经验力
											int count_sub = + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2  // 20140320, 谷德峰修改, 使得三个方向经验力可选
															+ int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2  
															+ int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 2;

											for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
											{
												int i_sub = 0;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);
												}
												i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);
												}
												i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 2
													  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 2;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);
												}
											}
										}
										if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc && it_Sat->second.dynDatum_Est.empiricalForceType == TYPE_EMPIRICALFORCE_A0COSSIN)
										{// 经验力
											int count_sub = + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3  // 20140320, 谷德峰修改, 使得三个方向经验力可选
															+ int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3  
															+ int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N) * 3;

											for(int j = 0; j < int(it_Sat->second.dynDatum_Est.empiricalForceParaList.size()); j++)
											{
												int i_sub = 0;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);

													double sum_ar = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2, sum_ar);
												}
												i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);

													double sum_ar = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2, sum_ar);
												}
												i_sub = int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_R) * 3
													  + int(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_T) * 3;
												if(it_Sat->second.dynDatum_Est.bOn_EmpiricalForceAcc_N)
												{
													double sum_cr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 0, sum_cr);
													
													double sum_sr = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 1, sum_sr);

													double sum_ar = interpRtPartial_k.GetElement(0, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 0) 
																  + interpRtPartial_k.GetElement(1, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 1)
																  + interpRtPartial_k.GetElement(2, it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2) * matHx_pos.GetElement(0, 2);
													matHx_k.SetElement(0, it_Sat->second.n0_EstParameters + it_Sat->second.n0_EmpiricalForce + j * count_sub + i_sub + 2, sum_ar);
												}
											}
										}

										// 估计模糊度？
										matHx_k.SetElement(0, m_ISLList[s_b].n0_ISLAmbiguity + int(s_k), w);
									}
									// 偏导数累加
									n_xx = n_xx + matHx_k.Transpose() * matHx_k;
									nx = nx + matHx_k.Transpose() * (w * m_ISLList[s_b].ISLArcList[s_k].obsList[s_j].oc_phase);
								}
							}
						}
					}

					// 伪方程-光压参数
					for(int k = 0; k <= 1; ++k)
					{
						TQNETPOD_SatDatumMap::iterator it_Sat = it_A;
						if(k == 1)
							it_Sat = it_B;
						// 添加伪方程-光压参数
						for(int s_kk = 0; s_kk < int(it_Sat->second.dynDatum_Est.solarPressureParaList.size()); s_kk++)
						{
							double weight_solar = 1.0E+12;	
							if(it_Sat->second.dynDatum_Est.solarPressureType == TYPE_SOLARPRESSURE_9PARA)
							{
								if(!it_Sat->second.on_SRP9_D0)
								{// △D0[0]  = -(D0*) + ε									
									int index_D0 = it_Sat->second.n0_EstParameters + 6 + 9 * (int)s_kk + 0;
									n_xx.SetElement(index_D0, index_D0,      n_xx.GetElement(index_D0, index_D0)     + weight_solar * weight_solar);
									nx.SetElement(index_D0, 0,               nx.GetElement(index_D0, 0)              - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_kk].D0);
								}
								if(!it_Sat->second.on_SRP9_DC1)
								{// △DCOS[1]  = -(DCOS*) + ε								
									int index_DCOS = it_Sat->second.n0_EstParameters + 6 + 9 * (int)s_kk + 1;
									n_xx.SetElement(index_DCOS, index_DCOS,  n_xx.GetElement(index_DCOS, index_DCOS) + weight_solar * weight_solar);
									nx.SetElement(index_DCOS, 0,             nx.GetElement(index_DCOS, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_kk].DC1);
								}
								if(!it_Sat->second.on_SRP9_DS1)
								{// △DSIN[2]  = -(DSIN*) + ε								
									int index_DSIN = it_Sat->second.n0_EstParameters + 6 + 9 * (int)s_kk + 2;
									n_xx.SetElement(index_DSIN, index_DSIN,  n_xx.GetElement(index_DSIN, index_DSIN) + weight_solar * weight_solar);
									nx.SetElement(index_DSIN, 0,             nx.GetElement(index_DSIN, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_kk].DS1);
								}
								if(!it_Sat->second.on_SRP9_Y0)
								{// △Y0[3]  = -(Y0*) + ε								
									int index_Y0 = it_Sat->second.n0_EstParameters + 6 + 9 * (int)s_kk + 3;
									n_xx.SetElement(index_Y0, index_Y0,      n_xx.GetElement(index_Y0, index_Y0)     + weight_solar * weight_solar);
									nx.SetElement(index_Y0, 0,               nx.GetElement(index_Y0, 0)              - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_kk].Y0);
								}
								if(!it_Sat->second.on_SRP9_YC1)
								{// △YCOS[4]  = -(YCOS*) + ε								
									int index_YCOS = it_Sat->second.n0_EstParameters + 6 + 9 * (int)s_kk + 4;
									n_xx.SetElement(index_YCOS, index_YCOS,  n_xx.GetElement(index_YCOS, index_YCOS) + weight_solar * weight_solar);
									nx.SetElement(index_YCOS, 0,             nx.GetElement(index_YCOS, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_kk].YC1);
								}
								if(!it_Sat->second.on_SRP9_YS1)
								{// △YSIN[5]  = -(YSIN*) + ε								
									int index_YSIN = it_Sat->second.n0_EstParameters + 6 + 9 * (int)s_kk + 5;
									n_xx.SetElement(index_YSIN, index_YSIN,  n_xx.GetElement(index_YSIN, index_YSIN) + weight_solar * weight_solar);
									nx.SetElement(index_YSIN, 0,             nx.GetElement(index_YSIN, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_kk].YS1);
								}
								if(!it_Sat->second.on_SRP9_B0)
								{// △B0[6]  = -(B0*) + ε								
									int index_B0 = it_Sat->second.n0_EstParameters + 6 + 9 * (int)s_kk + 6;
									n_xx.SetElement(index_B0, index_B0,      n_xx.GetElement(index_B0, index_B0)     + weight_solar * weight_solar);
									nx.SetElement(index_B0, 0,               nx.GetElement(index_B0, 0)              - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_kk].B0);
								}
								if(!it_Sat->second.on_SRP9_BC1)
								{// △BCOS[7]  = -(BCOS*) + ε								
									int index_BCOS = it_Sat->second.n0_EstParameters + 6 + 9 * (int)s_kk + 7;
									n_xx.SetElement(index_BCOS, index_BCOS,  n_xx.GetElement(index_BCOS, index_BCOS) + weight_solar * weight_solar);
									nx.SetElement(index_BCOS, 0,             nx.GetElement(index_BCOS, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_kk].BC1);
								}
								if(!it_Sat->second.on_SRP9_BS1)
								{// △BSIN[8]  = -(BSIN*) + ε								
									int index_BSIN = it_Sat->second.n0_EstParameters + 6 + 9 * (int)s_kk + 8;
									n_xx.SetElement(index_BSIN, index_BSIN,  n_xx.GetElement(index_BSIN, index_BSIN) + weight_solar * weight_solar);
									nx.SetElement(index_BSIN, 0,             nx.GetElement(index_BSIN, 0)            - weight_solar * weight_solar * it_Sat->second.dynDatum_Est.solarPressureParaList[s_kk].BS1);
								}
							}
						}
					}
				}			
			}

			// 更新整网解
			Matrix dx = n_xx.Inv_Ssgj() * nx;

			// 将网解分解到每个卫星
			for(TQNETPOD_SatDatumMap::iterator it_Sat = m_mapSatDatum.begin(); it_Sat != m_mapSatDatum.end(); ++it_Sat)
			{
				it_Sat->second.dx.Init(it_Sat->second.count_EstParameters, 1);
				for(int s_i = 0; s_i < it_Sat->second.count_EstParameters; s_i++)
				{
					int i_Net = it_Sat->second.n0_EstParameters + s_i;
					it_Sat->second.dx.SetElement(s_i, 0, dx.GetElement(i_Net, 0));
				}
			}

			// 星间测距模糊度解 测试代码李康康
			if(m_tqNetPODDefine.on_Used_ISL_Phase)
			{
				for(size_t s_b = 0; s_b < m_ISLList.size(); ++s_b)
				{
					for(size_t s_k = 0; s_k < m_ISLList[s_b].ISLArcList.size(); ++s_k)
					{
						int i_ISL = m_ISLList[s_b].n0_ISLAmbiguity + int(s_k);
						m_ISLList[s_b].ISLArcList[s_k].ambiguity += dx.GetElement(i_ISL, 0);
					}
				}
			}
			
		}

		// 子程序名称：updateISLResEdit
		// 功能：更新星间测距残差并编辑, 调整权值
		// 变量类型：
		// 输入：factor : 残差编辑因子, 默认3.0
		// 输出：
		// 语言：C++
		// 创建者：李康康
		// 创建时间：
		// 版本时间：2019/11/22
		// 修改记录：
		// 备注：测试代码
		void TQSatNetDynPOD::updateISLResEdit(double factor)
		{
			for(size_t s_b = 0; s_b < m_ISLList.size(); ++s_b)
			{
				TQNETPOD_SatDatumMap::iterator it_A = m_mapSatDatum.find(m_ISLList[s_b].satName_A);
				TQNETPOD_SatDatumMap::iterator it_B = m_mapSatDatum.find(m_ISLList[s_b].satName_B);
				
				// 更新 伪码
				if(m_tqNetPODDefine.on_Used_ISL_Code)
				{
					double rms_oc_code = 0.0;
					int count_valid_code = 0;
					for(size_t s_j = 0; s_j < m_ISLList[s_b].ISLArcList.size(); ++s_j)
					{
						for(size_t s_k = 0; s_k < m_ISLList[s_b].ISLArcList[s_j].obsList.size(); ++s_k)
						{
							UTC t = m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].t;
							TimePosVel interpOrbit[2];
							if(it_A->second.getEphemeris(t, interpOrbit[0])
							&& it_B->second.getEphemeris(t, interpOrbit[1]))
							{
								// 更新星间测距的 range_0 rw_code oc_code
								double range_0 = vectorMagnitude(interpOrbit[0].pos - interpOrbit[1].pos);
								m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].range_0 = range_0;
								m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].oc_code = m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].obs_code - range_0;

								if(1.0 == m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].rw_code)
								{
									++count_valid_code;
									rms_oc_code += pow(m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].oc_code, 2);
								}
							}
						}
					}
					rms_oc_code = sqrt(rms_oc_code / count_valid_code);
					// 更新权值
					for(size_t s_j = 0; s_j < m_ISLList[s_b].ISLArcList.size(); ++s_j)
					{
						for(size_t s_k = 0; s_k < m_ISLList[s_b].ISLArcList[s_j].obsList.size(); ++s_k)
						{
							if(fabs(m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].oc_code) > rms_oc_code * factor)
							{
								m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].rw_code = rms_oc_code / fabs(m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].oc_code);
							}
							else
								m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].rw_code = 1.0;
						}
					}
				}

				// 更新 相位
				if(m_tqNetPODDefine.on_Used_ISL_Phase)
				{
					double rms_oc_phase = 0.0;
					int count_valid_phase = 0;
					for(size_t s_j = 0; s_j < m_ISLList[s_b].ISLArcList.size(); ++s_j)
					{
						for(size_t s_k = 0; s_k < m_ISLList[s_b].ISLArcList[s_j].obsList.size(); ++s_k)
						{
							UTC t = m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].t;
							TimePosVel interpOrbit[2];
							if(it_A->second.getEphemeris(t, interpOrbit[0])
							&& it_B->second.getEphemeris(t, interpOrbit[1]))
							{
								// 更新星间测距的 range_0 rw_phase oc_phase
								double range_0 = vectorMagnitude(interpOrbit[0].pos - interpOrbit[1].pos);
								m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].range_0 = range_0;
								m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].oc_phase = m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].obs_phase - range_0;

								if(1.0 == m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].rw_phase)
								{
									++count_valid_phase;
									rms_oc_phase += pow(m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].oc_phase, 2);
								}
							}
						}
					}
					rms_oc_phase = sqrt(rms_oc_phase / count_valid_phase);
					// 更新权值
					for(size_t s_j = 0; s_j < m_ISLList[s_b].ISLArcList.size(); ++s_j)
					{
						for(size_t s_k = 0; s_k < m_ISLList[s_b].ISLArcList[s_j].obsList.size(); ++s_k)
						{
							if(fabs(m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].oc_phase) > rms_oc_phase * factor)
							{
								m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].rw_phase = rms_oc_phase / fabs(m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].oc_phase);
							}
							else
								m_ISLList[s_b].ISLArcList[s_j].obsList[s_k].rw_phase = 1.0;
						}
					}
				}
			}
				
		}
		
		bool TQSatNetDynPOD::mainNetPOD(UTC t0, UTC t1, double interval)
		{
			char info[200];

			// 1.  首先每个卫星单独进行完整地估计, 与原来程序类似
			for(TQNETPOD_SatDatumMap::iterator it_Sat = m_mapSatDatum.begin(); it_Sat != m_mapSatDatum.end(); ++it_Sat)
			{
				it_Sat->second.t0 = t0;
                it_Sat->second.t1 = t1;

				// 初始化观测数据信息
				updateSat_Obs(it_Sat);

				// 迭代开始
				const int max_iterator = 3; // 迭代次数阈值
				int num_iterator = 0;       // 记录迭代次数
				int num_after_ocEdit = 0;
				bool result = true;
				bool flag_break  = false;
			    bool flag_robust = false;

				it_Sat->second.getEstParameters_n0(); // 参数个数初始化

				while(1)
				{
					num_iterator++;
					if(num_iterator >=  m_tqNetPODDefine.max_OrbIteration)
					{
						result = false;	// 2014/06/18,发散轨道比较用，鞠 冰
						sprintf(info, "迭代超过%d次, 发散!", num_iterator);
						printf("%s\n");
						RuningInfoFile::Add(info);		
						break;
					}

					updateSat_AdamsCowell(it_Sat);

					sprintf(info, "卫星%s第%d次 adamsCowell_Interp_Leo is ok.", it_Sat->second.satName.c_str(), num_iterator);
					RuningInfoFile::Add(info);
					printf("%s\n", info);

                    // 更新残差
					it_Sat->second.ocResOutput();

					// 残差编辑
					if(flag_robust && m_tqNetPODDefine.on_ResEdit)
					{
						it_Sat->second.ocResEdit(m_tqNetPODDefine.ratio_ocEdit);
						num_after_ocEdit++;
					}

					if(flag_break)
					{
						break;
					}

					// 更新测站法方程信息
					updateSat_NEQ(it_Sat);

					// 更新单颗卫星的解
					updateSat_SOL(it_Sat);

					// 更新轨道改进结果
					it_Sat->second.updateDynDatum();

					char dynamicDatumFilePath[300];
					if(!m_tqSatNetDynPODPath.empty())
						sprintf(dynamicDatumFilePath,"%s\\%s-%s", m_tqSatNetDynPODPath.c_str(), it_Sat->first.c_str(), m_tqNetPODDefine.nameDynPodFitFile.c_str());

					// 保存轨道改进结果
					it_Sat->second.writeDynDatum(dynamicDatumFilePath);

					// 判断收敛条件
					double max_adjust_pos = 0;
					for(int i = 0; i < 3; i++)
					{
						max_adjust_pos = max(max_adjust_pos, fabs(it_Sat->second.dx.GetElement(i, 0)));
					}
					// 2020.11.18，安子聪
					printf("轨道改进量：%12.8f %12.8f %12.8f\n", fabs(it_Sat->second.dx.GetElement(0, 0)), 
						                                         fabs(it_Sat->second.dx.GetElement(1, 0)),
																 fabs(it_Sat->second.dx.GetElement(2, 0)));

					//sprintf(info, "max_adjust_pos = %10.4lf.", max_adjust_pos);
					//RuningInfoFile::Add(info);

					if(max_adjust_pos <= m_tqNetPODDefine.min_OrbPosIteration || num_iterator >= max_iterator || num_after_ocEdit > 0) 
					{
						if(flag_robust == false && m_tqNetPODDefine.on_ResEdit)
						{
							flag_robust = true; 
							continue;
						}
						else
						{
							if(m_tqNetPODDefine.on_ResEdit && num_after_ocEdit <= 1) // 编辑后迭代2次
								flag_break = false;
							else
								flag_break = true;
						}
					}
				}
				// 更新测站的轨道预报列表
				if(result && m_ISLList.size() == 0 && !m_tqNetPODDefine.on_EstStaZeroDelay)
				{
					orbitExtrapolation(it_Sat->second.dynDatum_Est, t0, t1,  it_Sat->second.orbList, interval);
					//return true; // 无星间链路和测站零值估计的情况下, 不进行网解

					if(it_Sat == (--m_mapSatDatum.end())) // 测试代码
						return true;
				}
			}


			// 3. 联合多星同步更新改进 * 
			// 迭代开始
			bool flag_Break_NET = false;
			bool result_NET = true;
			int  k_NET = 0; // 记录迭代的次数
			if(m_ISLList.size() > 0)
			{
				for(size_t s_b = 0; s_b < m_ISLList.size(); s_b++)
					updateNET_ISLArc(m_ISLList[s_b]); // 更新星间测距弧长信息
			}

			while(1)
			{
				k_NET++;
				if(k_NET >= m_tqNetPODDefine.max_OrbIteration)
				{
					result_NET = false;	
					sprintf(info, "警告: 网解迭代超过%d次发散.", k_NET);
					RuningInfoFile::Add(info);
					printf("%s\n", info);
					break;
				}

				// 利用NEQ重新构造整网法方程，更新网解
				updateNet_NEQ_SQL();

				// 判断网解收敛
				double max_AdjustPos = 0.0;
				for(TQNETPOD_SatDatumMap::iterator it_Sat = m_mapSatDatum.begin(); it_Sat != m_mapSatDatum.end(); ++it_Sat)
				{
					for(int i = 0; i < 3; i++)
						max_AdjustPos = max(max_AdjustPos, fabs(it_Sat->second.dx.GetElement(i, 0)));
					// 2020.11.18，安子聪
					printf("%s轨道改进量：%12.8f %12.8f %12.8f\n", it_Sat->first.c_str(), 
						                                         fabs(it_Sat->second.dx.GetElement(0, 0)), 
						                                         fabs(it_Sat->second.dx.GetElement(1, 0)),
																 fabs(it_Sat->second.dx.GetElement(2, 0)));
				}

				if(max_AdjustPos <= m_tqNetPODDefine.min_OrbPosIteration) 
					flag_Break_NET = true;

				// 逐个测站更新轨道
				for(TQNETPOD_SatDatumMap::iterator it_Sat = m_mapSatDatum.begin(); it_Sat != m_mapSatDatum.end(); ++it_Sat)
				{
					// 逐个卫星更新轨道改进结果
					it_Sat->second.updateDynDatum();

					char dynamicDatumFilePath[300];
					if(!m_tqSatNetDynPODPath.empty())
						sprintf(dynamicDatumFilePath,"%s\\%s-%s", m_tqSatNetDynPODPath.c_str(), it_Sat->first.c_str(), m_tqNetPODDefine.nameDynPodFitFile.c_str());

					// 逐个卫星保存轨道改进结果
					it_Sat->second.writeDynDatum(dynamicDatumFilePath);
					// 逐个卫星更新轨道积分
					updateSat_AdamsCowell(it_Sat);

					sprintf(info, "整网第%d次卫星%s adamsCowell_Interp_Leo is ok.", k_NET, it_Sat->first.c_str());
					RuningInfoFile::Add(info);
					printf("%s\n", info);

					// 更新残差
					it_Sat->second.ocResOutput();

					// 残差编辑
					if(m_tqNetPODDefine.on_ResEdit)
						it_Sat->second.ocResEdit(m_tqNetPODDefine.ratio_ocEdit);

					// 计算最终残差
					if(flag_Break_NET)
					{
						it_Sat->second.ocResOutput();

				//		for(size_t s_m = 0; s_m < it->second.dataMixedSysList.size(); s_m++)
				//		{
				//			sprintf(info, "GNSS(%c) ZD-OC PIF/LIF = %6.3fm/%6.3fm.", 
				//						  it_Sta->second.dataMixedSysList[s_m].cSys, 
				//						  it_Sta->second.dataMixedSysList[s_m].ocResRMS_P_IF, 
				//						  it_Sta->second.dataMixedSysList[s_m].ocResRMS_L_IF);
				//			printf("%s\n", info);
				//			RuningInfoFile::Add(info);
				//		}

						continue;
					}

					// 逐个测站更新NEQ
					updateSat_NEQ(it_Sat);
				}

				// 测试代码：更新星间伪码和相位测距残差并编辑。2019/11/22 李康康
				if(m_tqNetPODDefine.on_Used_ISL_Code||m_tqNetPODDefine.on_Used_ISL_Phase)
					updateISLResEdit();

				if(flag_Break_NET)
					break;
			}

			// 更新每个卫星的轨道预报
			if(result_NET)
			{
				for(TQNETPOD_SatDatumMap::iterator it_Sat = m_mapSatDatum.begin(); it_Sat != m_mapSatDatum.end(); ++it_Sat)
					orbitExtrapolation(it_Sat->second.dynDatum_Est, t0, t1,  it_Sat->second.orbList, interval);
			}
			return true;
		}


	}
}
