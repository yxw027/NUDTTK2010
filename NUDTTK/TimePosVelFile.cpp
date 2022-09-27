#include "TimePosVelFile.hpp"
#include <math.h>
#include "MathAlgorithm.hpp"
#include "TimePosVelFile.hpp"
#include "TimeCoordConvert.hpp"

using namespace NUDTTK::Math;

namespace NUDTTK
{
	TimePosVelFile::TimePosVelFile(void)
	{
	}

	TimePosVelFile::~TimePosVelFile(void)
	{
	}

	bool TimePosVelFile::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}

	bool TimePosVelFile::isValidEpochLine(string strLine, TimePosVel& timePosVel)
	{
		bool bFlag = true;
		char szTime_yyyy[100];
		char szTime_mm[100];
		char szTime_dd[100];
		char szTime_HH[100];
		char szTime_MM[100];
		char szTime_SS[100];
		char  szX[100];
		char  szY[100];
		char  szZ[100];
		char szVx[100];
		char szVy[100];
		char szVz[100];
		sscanf(strLine.c_str(),"%s%s%s%s%s%s%s%s%s%s%s%s",
							   szTime_yyyy,
							   szTime_mm,
							   szTime_dd,
							   szTime_HH,
							   szTime_MM,
							   szTime_SS,
							   szX,
							   szY,
							   szZ,
							   szVx,
							   szVy,
							   szVz);
		timePosVel.t.year    = atoi(szTime_yyyy);
		timePosVel.t.month   = atoi(szTime_mm);
		timePosVel.t.day     = atoi(szTime_dd);
		timePosVel.t.hour    = atoi(szTime_HH);
		timePosVel.t.minute  = atoi(szTime_MM);
		timePosVel.t.second  = atof(szTime_SS);
		timePosVel.pos.x = atof(szX);
		timePosVel.pos.y = atof(szY);
		timePosVel.pos.z = atof(szZ);
		timePosVel.vel.x = atof(szVx);
		timePosVel.vel.y = atof(szVy);
		timePosVel.vel.z = atof(szVz);
		double r = sqrt(timePosVel.pos.x * timePosVel.pos.x 
			          + timePosVel.pos.y * timePosVel.pos.y
					  + timePosVel.pos.z * timePosVel.pos.z);
		/*if(r <= 6000000)
			bFlag = false;*/
		if(r == 0)
			bFlag = false;
		return bFlag;
	}

	bool TimePosVelFile::open(string  strFileName)
	{
		FILE * pFile = fopen(strFileName.c_str(), "r+t");
		if(pFile == NULL) 
			return false;
		char line[400];
		m_data.clear();
		while(!feof(pFile))
		{
			if(fgets(line, 400, pFile))
			{
				TimePosVel timePosVel;
				if(isValidEpochLine(line, timePosVel))
					m_data.push_back(timePosVel);
			}
		}
		fclose(pFile);
		return true;
	}

	bool TimePosVelFile::write(string strFileName)
	{
		FILE* pFile = fopen(strFileName.c_str(), "w+");
        for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			fprintf(pFile, "%s %16.6f %16.6f %16.6f %16.6f %16.6f %16.6f\n", m_data[s_i].t.toString().c_str(),
																			 m_data[s_i].pos.x,
																			 m_data[s_i].pos.y,
																			 m_data[s_i].pos.z,
																			 m_data[s_i].vel.x,
																			 m_data[s_i].vel.y,
																			 m_data[s_i].vel.z);
		}
		fclose(pFile);
		return true;
	}

	// 子程序名称： getPosVel   
	// 功能：利用插值获得时间 t 的卫星轨道数据
	// 变量类型：t              : 包含年、月、日、时、分、秒
	//           posvel         : 轨道位置、速度
	//           nlagrange      : Lagrange插值已知点个数, 默认为8，对应nLagrange-1阶Lagrange插值
	//           bVelFromPos    : 速度是否采用位置微分平滑方法标记, 默认false
	// 输入：t, nlagrange
	// 输出：posvel
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/02/03
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool TimePosVelFile::getPosVel(DayTime t, POS6D& posvel, int nlagrange, bool bVelFromPos)
	{
		int nLagrange_left  = int(floor(nlagrange / 2.0));   
		int nLagrange_right = int(ceil (nlagrange / 2.0));
		if(int(m_data.size()) < nlagrange)           
			return false;
		// 插值时考虑到钟差的影响，以准确轨道采样时刻为参考
		DayTime t_Begin = m_data[0].t;               
		DayTime t_End   = m_data[m_data.size() - 1].t; 
		double  span_total = t_End - t_Begin;
		double  span_t = t - t_Begin;        
		if(span_t < 0 || span_t > span_total) 
			return false;
		int nLeftPos = -1;
		// 采用二分法, 2008/05/11
		size_t left  = 1;
		size_t right = m_data.size() - 1;
		int n = 0;
		while(left < right)
		{
			n++;
			int middle = int(left + right) / 2;
			double time_L = (m_data[middle - 1].t - t_Begin);
			double time_R = (m_data[middle].t - t_Begin);
			if(span_t >= time_L && span_t <= time_R) 
			{// 终止条件
				nLeftPos = middle - 1;
				break;
			}
			if(span_t < time_L) 
				right = middle - 1;
			else 
				left  = middle + 1;
		}
		if(right == left)
		{
			double time_L = (m_data[left - 1].t - t_Begin);
			double time_R = (m_data[left].t - t_Begin);
			if(span_t >= time_L && span_t <= time_R) 
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
			nEnd   = nlagrange - 1;
		}
		else if(nLeftPos + nLagrange_right >= int(m_data.size()))
		{
			nBegin = int(m_data.size()) - nlagrange;
			nEnd   = int(m_data.size()) - 1;
		}
		else
		{
			nBegin = nLeftPos - nLagrange_left + 1;
			nEnd   = nLeftPos + nLagrange_right;
		}
		double *xa_t =new double [nlagrange];
		double *ya_x =new double [nlagrange];
		double *ya_y =new double [nlagrange];
		double *ya_z =new double [nlagrange];
		double *ya_vx=new double [nlagrange];
		double *ya_vy=new double [nlagrange];
		double *ya_vz=new double [nlagrange];
		for(int i = nBegin; i <= nEnd; i++)
		{
			 xa_t[i - nBegin] = m_data[i].t - t_Begin;
			 ya_x[i - nBegin] = m_data[i].pos.x;
			 ya_y[i - nBegin] = m_data[i].pos.y;
			 ya_z[i - nBegin] = m_data[i].pos.z;
			ya_vx[i - nBegin] = m_data[i].vel.x;
			ya_vy[i - nBegin] = m_data[i].vel.y;
			ya_vz[i - nBegin] = m_data[i].vel.z;
		}
		// 通过插值给出位置速度
		if(!bVelFromPos)
		{
			InterploationLagrange(xa_t, ya_x,  nlagrange, span_t, posvel.x);
			InterploationLagrange(xa_t, ya_y,  nlagrange, span_t, posvel.y);
			InterploationLagrange(xa_t, ya_z,  nlagrange, span_t, posvel.z);
			InterploationLagrange(xa_t, ya_vx, nlagrange, span_t, posvel.vx);
			InterploationLagrange(xa_t, ya_vy, nlagrange, span_t, posvel.vy);
			InterploationLagrange(xa_t, ya_vz, nlagrange, span_t, posvel.vz);
		}
		else
		{
			InterploationLagrange(xa_t, ya_x,  nlagrange, span_t, posvel.x, posvel.vx);
			InterploationLagrange(xa_t, ya_y,  nlagrange, span_t, posvel.y, posvel.vy);
			InterploationLagrange(xa_t, ya_z,  nlagrange, span_t, posvel.z, posvel.vz);
		}
		delete xa_t;
		delete ya_x;
		delete ya_y;
		delete ya_z;
		delete ya_vx;
		delete ya_vy;
		delete ya_vz;
		return true;
	}

	// 子程序名称： getPosVelAcc   
	// 功能：利用插值获得时间 t 的卫星轨道数据(含加速度)
	// 变量类型：t              : 包含年、月、日、时、分、秒
	//           posvel         : 轨道位置、速度
	//           nlagrange      : Lagrange插值已知点个数，默认为8，对应nLagrange-1阶Lagrange插值
	// 输入：t, nlagrange
	// 输出：posvel
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2013/01/01
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool TimePosVelFile::getPosVelAcc(DayTime t, POS6D& posvel, POS3D& acc, int nlagrange)
	{
		int nLagrange_left  = int(floor(nlagrange / 2.0));   
		int nLagrange_right = int(ceil (nlagrange / 2.0));
		if(int(m_data.size()) < nlagrange)           
			return false;
		// 插值时考虑到钟差的影响，以准确轨道采样时刻为参考
		DayTime t_Begin = m_data[0].t;               
		DayTime t_End   = m_data[m_data.size() - 1].t; 
		double  span_total = t_End - t_Begin;
		double  span_t = t - t_Begin;        
		if(span_t < 0 || span_t > span_total) 
			return false;
		int nLeftPos = -1;
		// 采用二分法, 2008/05/11
		size_t left  = 1;
		size_t right = m_data.size() - 1;
		int n = 0;
		while(left < right)
		{
			n++;
			int middle = int(left + right) / 2;
			double time_L = (m_data[middle - 1].t - t_Begin);
			double time_R = (m_data[middle].t - t_Begin);
			if(span_t >= time_L && span_t <= time_R) 
			{// 终止条件
				nLeftPos = middle - 1;
				break;
			}
			if(span_t < time_L) 
				right = middle - 1;
			else 
				left  = middle + 1;
		}
		if(right == left)
		{
			double time_L = (m_data[left - 1].t - t_Begin);
			double time_R = (m_data[left].t - t_Begin);
			if(span_t >= time_L && span_t <= time_R) 
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
			nEnd   = nlagrange - 1;
		}
		else if(nLeftPos + nLagrange_right >= int(m_data.size()))
		{
			nBegin = int(m_data.size()) - nlagrange;
			nEnd   = int(m_data.size()) - 1;
		}
		else
		{
			nBegin = nLeftPos - nLagrange_left + 1;
			nEnd   = nLeftPos + nLagrange_right;
		}
		double *xa_t =new double [nlagrange];
		double *ya_x =new double [nlagrange];
		double *ya_y =new double [nlagrange];
		double *ya_z =new double [nlagrange];
		double *ya_vx=new double [nlagrange];
		double *ya_vy=new double [nlagrange];
		double *ya_vz=new double [nlagrange];
		for(int i = nBegin; i <= nEnd; i++)
		{
			 xa_t[i - nBegin] = m_data[i].t - t_Begin;
			 ya_x[i - nBegin] = m_data[i].pos.x;
			 ya_y[i - nBegin] = m_data[i].pos.y;
			 ya_z[i - nBegin] = m_data[i].pos.z;
			ya_vx[i - nBegin] = m_data[i].vel.x;
			ya_vy[i - nBegin] = m_data[i].vel.y;
			ya_vz[i - nBegin] = m_data[i].vel.z;
		}
		// 通过插值给出位值速度
		InterploationLagrange(xa_t, ya_x,  nlagrange, span_t, posvel.x);
		InterploationLagrange(xa_t, ya_y,  nlagrange, span_t, posvel.y);
		InterploationLagrange(xa_t, ya_z,  nlagrange, span_t, posvel.z);
		InterploationLagrange(xa_t, ya_vx, nlagrange, span_t, posvel.vx, acc.x);
		InterploationLagrange(xa_t, ya_vy, nlagrange, span_t, posvel.vy, acc.y);
		InterploationLagrange(xa_t, ya_vz, nlagrange, span_t, posvel.vz, acc.z);
		delete xa_t;
		delete ya_x;
		delete ya_y;
		delete ya_z;
		delete ya_vx;
		delete ya_vy;
		delete ya_vz;
		return true;
	}

	// 子程序名称： orbComparision_XYZ   
	// 功能：轨道位置和速度直接比对
	// 变量类型：t           : GPS时间(观测时间)
	//           posvel      : 轨道位置、速度
	//           error       : 轨道位置、速度误差
	// 输入：t, posvel
	// 输出：error
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/07/13
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool TimePosVelFile::orbComparision_XYZ(GPST t, TimePosVelClock posvel, POS6D& error)
	{
		// 信号真实接收时间 = 观测时间(T) - 接收机钟差(posvel.clk)
		GPST t_GPS = t - posvel.clk / SPEED_LIGHT;
		if(m_data.size() <= 0) 
			return false;
		GPST t0 = m_data[0].t;
		GPST t1 = m_data[m_data.size() - 1].t;
		if(t_GPS - t0 < 0 || t_GPS - t1 > 0) 
			return false;
		POS6D posvel_REF;
		if(!getPosVel(t_GPS, posvel_REF))
			return false;
		error.x  = posvel.pos.x - posvel_REF.x;
		error.y  = posvel.pos.y - posvel_REF.y;
		error.z  = posvel.pos.z - posvel_REF.z;
		error.vx = posvel.vel.x - posvel_REF.vx;
		error.vy = posvel.vel.y - posvel_REF.vy;
		error.vz = posvel.vel.z - posvel_REF.vz;
		return true;
	}

	//   子程序名称： orbComparision_RTN_ECF   
	//   作用：轨道位置和速度比对(用于地固坐标系下, 考虑了地固系旋转影响) 
	//   类型：t           : GPS时间(观测时间)
	//         posvel      : 轨道位置、速度
	//         error       : 轨道位置、速度误差
	//   输入：t, posvel
	//   输出：error 
	//   其它：
	//   语言：C++
	//   版本号：2008/07/13
	//   生成者：谷德峰
	//   修改者：
	bool TimePosVelFile::orbComparision_RTN_ECF(GPST t,TimePosVelClock posvel,POS6D& error)
	{
		// 信号真实接收时间 = 观测时间(T) - 接收机钟差(posvel.clk)
		GPST t_GPS = t - posvel.clk / SPEED_LIGHT;
		if(m_data.size() <= 0) 
			return false;
		GPST t0 = m_data[0].t;
		GPST t1 = m_data[m_data.size() - 1].t;
		if(t_GPS - t0 < 0 || t_GPS - t1 > 0) 
			return false;
		POS6D posvel_REF;
		if(!getPosVel(t_GPS, posvel_REF))
			return false;
		POS3D errPos_ECF; 
		POS3D errVel_ECF;
		errPos_ECF.x = posvel.pos.x - posvel_REF.x;
		errPos_ECF.y = posvel.pos.y - posvel_REF.y;
		errPos_ECF.z = posvel.pos.z - posvel_REF.z;
		errVel_ECF.x = posvel.vel.x - posvel_REF.vx;
		errVel_ECF.y = posvel.vel.y - posvel_REF.vy;
		errVel_ECF.z = posvel.vel.z - posvel_REF.vz;

		POS3D S_R;
		POS3D S_T;
		POS3D S_N;
		TimeCoordConvert::getCoordinateRTNAxisVector(t, posvel_REF, S_R, S_T, S_N);
		// 将 ITRF 系下的比对误差, 投影到 RTN 坐标系下
		error.x  = vectorDot(errPos_ECF, S_R);
		error.y  = vectorDot(errPos_ECF, S_T);
		error.z  = vectorDot(errPos_ECF, S_N);
		error.vx = vectorDot(errVel_ECF, S_R);
		error.vy = vectorDot(errVel_ECF, S_T);
		error.vz = vectorDot(errVel_ECF, S_N);
		return true;
	}

	//   子程序名称： orbComparision_RTN_ECI   
	//   作用：轨道位置和速度比对(用于惯性系下) 
	//   类型：t           : GPS时间(观测时间)
	//         posvel      : 轨道位置、速度
	//         error       : 轨道位置、速度误差
	//   输入：t, posvel
	//   输出：error 
	//   其它：
	//   语言：C++
	//   版本号：2008/07/13
	//   生成者：谷德峰
	//   修改者：
	bool TimePosVelFile::orbComparision_RTN_ECI(GPST t,TimePosVelClock posvel,POS6D& error)
	{
		// 信号真实接收时间 = 观测时间(T) - 接收机钟差(posvel.clk)
		GPST t_GPS = t - posvel.clk / SPEED_LIGHT;
		if(m_data.size() <= 0) 
			return false;
		GPST t0 = m_data[0].t;
		GPST t1 = m_data[m_data.size() - 1].t;
		if(t_GPS - t0 < 0 || t_GPS - t1 > 0) 
			return false;
		POS6D posvel_REF;
		if(!getPosVel(t_GPS, posvel_REF))
			return false;
		POS3D errPos_ECF; 
		POS3D errVel_ECF;
		errPos_ECF.x = posvel.pos.x - posvel_REF.x;
		errPos_ECF.y = posvel.pos.y - posvel_REF.y;
		errPos_ECF.z = posvel.pos.z - posvel_REF.z;
		errVel_ECF.x = posvel.vel.x - posvel_REF.vx;
		errVel_ECF.y = posvel.vel.y - posvel_REF.vy;
		errVel_ECF.z = posvel.vel.z - posvel_REF.vz;

		// 根据位置、速度计算卫星RTN坐标系
		POS3D S_R; // R 轴径向       ( -Z )
		S_R.x =  posvel.pos.x;
		S_R.y =  posvel.pos.y;
		S_R.z =  posvel.pos.z;
		POS3D S_T; // T 轴沿速度方向 (  X )
		S_T.x =  posvel.vel.x;
		S_T.y =  posvel.vel.y;
		S_T.z =  posvel.vel.z;
		POS3D S_N; // 右手系         ( -Y )
		vectorCross(S_N,S_R,S_T);
		vectorCross(S_T,S_N,S_R);
		S_T = vectorNormal(S_T);
		S_N = vectorNormal(S_N);
		S_R = vectorNormal(S_R);
		// 将 ITRF 系下的比对误差, 投影到 RTN 坐标系下
		error.x  = vectorDot(errPos_ECF, S_R);
		error.y  = vectorDot(errPos_ECF, S_T);
		error.z  = vectorDot(errPos_ECF, S_N);
		error.vx = vectorDot(errVel_ECF, S_R);
		error.vy = vectorDot(errVel_ECF, S_T);
		error.vz = vectorDot(errVel_ECF, S_N);
		return true;
	}
}