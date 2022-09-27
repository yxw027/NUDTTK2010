#include "SLROrbitComparison.hpp"
#include "cstgSLRObsFile.hpp"
#include "meritSLRObsFile.hpp"
#include "crdSLRObsFile.hpp"
#include "crdFrdSLRObsFile.hpp"
#include "MathAlgorithm.hpp"
#include "SLRPreproc.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace SLR
	{
		SLROrbitComparison::SLROrbitComparison(void)
		{
			m_att_model  = Body2J2000;
			m_bOn_YawAttitudeModel = false;
			m_constRangeBias = 0.0;
			m_bChecksum = true;
			m_staPos_model = Ssc;
		}

		SLROrbitComparison::~SLROrbitComparison(void)
		{
		}

		double SLREditedObsElement::getStaLosElevation()
		{
			return 90 - acos(vectorDot(vectorNormal(staPos_ECEF), vectorNormal(leoPV_ECEF.getPos() - staPos_ECEF)) ) * 180 / PI;
		}

		double SLREditedObsElement::getLeoLosElevation()
		{
			// 根据位置、速度计算卫星RTN坐标系
			POS3D S_R = leoPV_ECEF.getPos(); // R 轴径向       ( -Z )
			POS3D S_T = leoPV_ECEF.getVel(); // T 轴沿速度方向 (  X )
			POS3D S_N;                       // 右手系         ( -Y )
			vectorCross(S_N,S_R,S_T);
			vectorCross(S_T,S_N,S_R);
			S_T = vectorNormal(S_T);
			S_N = vectorNormal(S_N);
			S_R = vectorNormal(S_R);
			POS3D  p_sat = vectorNormal(leoPV_ECEF.getPos() - staPos_ECEF);
			POS3D vecLos_RTN;
			vecLos_RTN.z = vectorDot(p_sat,S_R);
			vecLos_RTN.x = vectorDot(p_sat,S_T);
			vecLos_RTN.y = vectorDot(p_sat,S_N);
			// 视线矢量---方向余弦
			double los_R = vecLos_RTN.z;
			double los_T = vecLos_RTN.x;
			double los_N = vecLos_RTN.y;
			double elevation = 90 - acos(vecLos_RTN.z) * 180 / PI;           // 高度角
			double azimuth   = atan2(vecLos_RTN.y, vecLos_RTN.x) * 180 / PI; // 方位角
			return elevation;
		}
		// 子程序名称： getOrbInterp 
		// 功能：通过滑动 lagrange 插值获得时刻 t 对应的单点轨道数据
		//       注: 程序中考虑到了轨道数据可能出现不连续的情况
		// 类型：  orbList    : 比对轨道数据序列, 序列最好不要太长, 否则会影响搜索速度
		//         t          : 轨道时刻, 准确时间
		//         point      : 时刻 t 对应的单点轨道数据
		//         nLagrange  : 插值点个数，阶数为 nLagrange - 1 
		// 输入：
		// 输出：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/08/29
		// 版本时间：2007/08/29
		// 修改记录：
		// 备注：
		bool SLROrbitComparison::getOrbInterp(vector<TimePosVel> orbList, GPST t, TimePosVel &point, unsigned int nLagrange)
		{
			// 轨道数据序列的个数
			size_t count = orbList.size();
			int nLagrange_left  = int(floor(nLagrange/2.0));   
			int nLagrange_right = int(ceil (nLagrange/2.0));
			if(count < int(nLagrange))
				return false;
			// 插值时考虑到钟差的影响，以准确轨道采样时刻为参考
			GPST t0 = orbList[0].t; // 轨道序列初始时间
			GPST t1 = orbList[count - 1].t; // 轨道序列结束时间
			double span_Total = t1 - t0;
			double span_T = t - t0;  // 换算成相对时间
			if(span_T < 0 || span_T > span_Total ) // 确保 span_T 在有效范围之内
				return false;
			// 搜索法寻找 t 时刻对应的轨道区间
			int nLeftPos = -1;
			for(size_t s_i = 1; s_i < count; s_i++)
			{// 顺序搜索，如果数据点 T 在轨道序列末端，可能会速度较慢，待进一步改进
				double time_L = (orbList[s_i-1].t - t0);
				double time_R = (orbList[s_i].t - t0);
				if(span_T >= time_L && span_T <= time_R)
				{
					nLeftPos = int(s_i - 1);
					break;
				}
			}
			if(nLeftPos == -1)
				return false;
			// 确定插值区间位置 [nBegin, nEnd], nEnd - nBegin + 1 = nLagrange
			int nBegin, nEnd; 
			if(nLeftPos - nLagrange_left + 1 < 0) 
			{
				nBegin = 0;
				nEnd   = nLagrange - 1;
			}
			else if( nLeftPos + nLagrange_right >= int(count))
			{
				nBegin = int(count) - nLagrange;
				nEnd   = int(count) - 1;
			}
			else
			{
				nBegin = nLeftPos - nLagrange_left + 1;
				nEnd   = nLeftPos + nLagrange_right;
			}
			// 整理插值参考点
			double *xa_t = new double [nLagrange];
			double *ya_X = new double [nLagrange];
			double *ya_Y = new double [nLagrange];
			double *ya_Z = new double [nLagrange];
			for( int i = nBegin; i <= nEnd; i++ )
			{
				xa_t[i - nBegin] = orbList[i].t - t0;
				ya_X[i - nBegin] = orbList[i].pos.x;
				ya_Y[i - nBegin] = orbList[i].pos.y;
				ya_Z[i - nBegin] = orbList[i].pos.z;
			}
			InterploationLagrange(xa_t, ya_X, nLagrange, span_T, point.pos.x, point.vel.x);
			InterploationLagrange(xa_t, ya_Y, nLagrange, span_T, point.pos.y, point.vel.y);
			InterploationLagrange(xa_t, ya_Z, nLagrange, span_T, point.pos.z, point.vel.z);
			point.t = t;
			delete xa_t;
			delete ya_X;
			delete ya_Y;
			delete ya_Z;
			return true;
		}

		// 子程序名称： getAttMatrixInterp 
		// 功能：通过滑动 lagrange 插值获得时刻 t 对应的姿态数据和姿态矩阵
		//       注: 程序中考虑到了姿态数据可能出现不连续的情况
		// 类型：  attList    : 比对轨道数据序列, 序列最好不要太长, 否则会影响搜索速度
		//         t          : 轨道时刻, 准确时间
		//         matATT     : 时刻 t 对应的姿态矩阵
		//         nLagrange  : 插值点个数，阶数为 nLagrange - 1 
		// 输入：
		// 输出：
		// 语言：C++
		// 创建者：邵凯
		// 创建时间：2021/06/24
		// 版本时间：2021/06/24
		// 修改记录：
		// 备注：
		bool SLROrbitComparison::getAttMatrixInterp(vector<TimeAttLine> attList, GPST t, Matrix &matATT, unsigned int nlagrange)
		{
			ATT_Q4 Q4;
			// 姿态数据序列的个数
			int nLagrange_left  = int(floor(nlagrange/2.0));   
			int nLagrange_right = int(ceil (nlagrange/2.0));
			if(attList.size() < size_t(nlagrange))
				return false;
			// 插值时考虑到钟差的影响, 以准确轨道采样时刻为参考
			DayTime t_Begin = attList[0].t; // 姿态序列初始时间
			DayTime t_End   = attList[attList.size() - 1].t; // 姿态序列结束时间
			double  span_Total = t_End - t_Begin;
			double  span_T     = t - t_Begin; // 换算成相对时间
			if(span_T < 0 || span_T > span_Total)  // 确保 span_T 在有效范围之内
				return false;
			// 采用二分法, 2008/05/11
			int nLeftPos = -1;
			size_t left  =  1;
			size_t right = attList.size() - 1;
			int n = 0;
			while(left < right)
			{
				n++;
				int middle  = int(left + right)/2;
				double time_L = (attList[middle - 1].t - t_Begin);
				double time_R = (attList[middle].t     - t_Begin);
				if(span_T >= time_L && span_T <= time_R) 
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
				double time_L = (attList[left - 1].t - t_Begin);
				double time_R = (attList[left].t     - t_Begin);
				if(span_T >= time_L && span_T <= time_R) 
				{// 终止条件
					nLeftPos = int(left) - 1;
				}
			}
			if(nLeftPos == -1)
				return false;
			// 确定插值区间位置 [nBegin, nEnd], nEnd - nBegin + 1 = nLagrange
			int nBegin, nEnd; 
			if(nLeftPos - nLagrange_left + 1 < 0) 
			{
				nBegin = 0;
				nEnd   = nlagrange - 1;
			}
			else if(nLeftPos + nLagrange_right >= int(attList.size()))
			{
				nBegin = int(attList.size()) - nlagrange;
				nEnd = int(attList.size()) - 1;
			}
			else
			{
				nBegin = nLeftPos - nLagrange_left + 1;
				nEnd   = nLeftPos + nLagrange_right;
			}
			// 整理插值参考点
			double *xa_t  = new double [nlagrange];
			double *ya_q1 = new double [nlagrange];
			double *ya_q2 = new double [nlagrange];
			double *ya_q3 = new double [nlagrange];
			double *ya_q4 = new double [nlagrange];
			for(int i = nBegin; i <= nEnd; i++)
			{
				 xa_t[i - nBegin] = attList[i].t - t_Begin;
				ya_q1[i - nBegin] = attList[i].Q4.q1;
				ya_q2[i - nBegin] = attList[i].Q4.q2;
				ya_q3[i - nBegin] = attList[i].Q4.q3;
				ya_q4[i - nBegin] = attList[i].Q4.q4;
			}
			double max_spanSecond = 0;
			for(int i = nBegin + 1; i <= nEnd; i++)
			{
				if(max_spanSecond < (xa_t[i - nBegin] - xa_t[i - 1 - nBegin]))
					max_spanSecond = (xa_t[i - nBegin] - xa_t[i - 1 - nBegin]);
			}
			InterploationLagrange( xa_t, ya_q1, nlagrange, span_T, Q4.q1);
			InterploationLagrange( xa_t, ya_q2, nlagrange, span_T, Q4.q2);
			InterploationLagrange( xa_t, ya_q3, nlagrange, span_T, Q4.q3);
			InterploationLagrange( xa_t, ya_q4, nlagrange, span_T, Q4.q4);
			delete xa_t;
			delete ya_q1;
			delete ya_q2;
			delete ya_q3;
			delete ya_q4;
			// 需要增加最大插值间隔的控制, 防止姿态数据缺失, 2分钟(0.06 * 60 = 3.6度)
			if(max_spanSecond > 60 * 2)
			{
				// printf("%s姿态缺失%.1f秒\n", t.ToString().c_str(), max_span);
				return false;
			}
			/*
				  惯性坐标系到星体坐标系
				|x|                   |x|
				|y|      =[姿态矩阵] *|y| 
				|z|星体               |z|惯性
			*/
			matATT.Init(3, 3);
			double q1 = Q4.q1;
			double q2 = Q4.q2;
			double q3 = Q4.q3;
			double q4 = Q4.q4;
			matATT.SetElement(0, 0,  q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);
			matATT.SetElement(0, 1,  2 * (q1 * q2 + q3 * q4));
			matATT.SetElement(0, 2,  2 * (q1 * q3 - q2 * q4));
			matATT.SetElement(1, 0,  2 * (q1 * q2 - q3 * q4));
			matATT.SetElement(1, 1, -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4);
			matATT.SetElement(1, 2,  2 * (q2 * q3 + q1 * q4));
			matATT.SetElement(2, 0,  2 * (q1 * q3 + q2 * q4));
			matATT.SetElement(2, 1,  2 * (q2 * q3 - q1 * q4));
			matATT.SetElement(2, 2, -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4);
			return true;
		}

		// 获得激光测站的坐标
		bool SLROrbitComparison::getStaPosvel(UTC t, int id, POS6D& posvel)
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
		////getStaPosvel_14
		//bool SLROrbitComparison::getStaPosvel_14(UTC t, int id, POS6D& posvel)
		//{
		//	for(size_t s_i = 0; s_i < m_staSscList_14.size(); s_i++)
		//	{
		//		if( m_staSscList_14[s_i].id == id
		//		 && t - m_staSscList_14[s_i].t0 >= 0 
		//		 && t - m_staSscList_14[s_i].t1 <= 0)
		//		{
		//			posvel.x  = m_staSscList_14[s_i].x;
		//			posvel.y  = m_staSscList_14[s_i].y;
		//			posvel.z  = m_staSscList_14[s_i].z;
		//			posvel.vx = m_staSscList_14[s_i].vx;
		//			posvel.vy = m_staSscList_14[s_i].vy;
		//			posvel.vz = m_staSscList_14[s_i].vz;
		//			double year = (t - m_staSscList_14[s_i].t) / (86400 * 365.25);
		//			posvel.x += posvel.vx * year;
		//			posvel.y += posvel.vy * year;
		//			posvel.z += posvel.vz * year;
		//			return true;
		//		}
		//	}
		//	return false;
		//}
		// 获得激光测站的偏心数据
		bool SLROrbitComparison::getStaEcc(UTC t, int id, ENU& ecc)
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

		// 子程序名称： getSubArcOrbList 
		// 功能：根据时间段的区间设置, 检索本时间段内的轨道子序列列表
		//       轨道序列的两端要有一定的余量, 以保证插值精度
		// 类型：  t0           :  起始时间
		//         t1           :  结束时间
		//         orbList      :  轨道列表
		//         nExtern      :  轨道序列的两端多空留点的数目
		// 输入：
		// 输出：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/12/30
		// 版本时间：2007/12/30
		// 修改记录：1.2015/11/17,刘俊宏修改，增加最长时间间隔判断
		// 备注：
		bool SLROrbitComparison::getSubArcOrbList(GPST t0, GPST t1, vector<TimePosVel> &orbList, int nExtern, double maxSpan)
		{
			// 根据激光弧段起止时间获得轨道数据序列, 同时为保证插值精度前后各延长 nExtern 个点
			size_t count = m_orbList.size();
			if(count < size_t(2 * nExtern))
				return false;
			int nBegin = -1;
			int nEnd   = int(count);
			double span_T  = t0 - m_orbList[0].t;  // 换算成相对时间
			// 采用二分法，2008 - 05 - 11
			size_t left  = 1;
			size_t right = count - 1;
			int n = 0;
			while(left < right)
			{
				n++;
				int    middle  = int(left + right)/2;
				double time_L = (m_orbList[middle - 1].t - m_orbList[0].t);
				double time_R = (m_orbList[middle].t     - m_orbList[0].t);
				if(span_T >= time_L && span_T <= time_R) 
				{// 终止条件
					nBegin = middle;
					break;
				}
				if(span_T < time_L) 
					right = middle - 1;
				else 
					left  = middle + 1;
			}
			if(right == left)
			{
				double time_L = (m_orbList[left - 1].t - m_orbList[0].t);
				double time_R = (m_orbList[left].t     - m_orbList[0].t);
				if(span_T >= time_L && span_T <= time_R) 
				{// 终止条件
					nBegin = int(left);
				}
			}
			if(nBegin == -1)
				return false;
			// nBegin 搜索成功后，开始搜索 nEnd
			for(size_t s_j = nBegin; s_j < count; s_j++)
			{
				if(m_orbList[s_j].t - t1 >= 0)
				{
					nEnd = int(s_j);
					break;
				}
			}
			// 向两边延伸
			nBegin = nBegin - nExtern;
			nEnd   = nEnd   + nExtern;
			if(nBegin < 0 || nEnd >= int(count))
				return false;
			// 整理轨道数据列表
			orbList.clear();
			for(int i = nBegin; i <= nEnd; i++)//2015/11/17,刘俊宏
			{
				if(i == nBegin)
					orbList.push_back(m_orbList[i]);
				else
				{
					if(fabs(m_orbList[i].t - m_orbList[i - 1].t) <= maxSpan)					
						orbList.push_back(m_orbList[i]);			
					else
						break;
				}
			}
			if(orbList.front().t - t0 > 0 || orbList.back().t - t1 < 0)
				return false;
			else
     			return true;
		}
		// 子程序名称： getSubArcAttList 
		// 功能：根据时间段的区间设置, 检索本时间段内的姿态子序列列表
		//       姿态序列的两端要有一定的余量, 以保证插值精度
		// 类型：  t0           :  起始时间
		//         t1           :  结束时间
		//         attList      :  姿态列表
		//         nExtern      :  姿态序列的两端多空留点的数目
		// 输入：
		// 输出：
		// 语言：C++
		// 创建者：邵凯
		// 创建时间：2021/06/24
		// 版本时间：2021/06/24
		// 修改记录：
		// 备注：
		bool SLROrbitComparison::getSubArcAttList(GPST t0, GPST t1, vector<TimeAttLine> &attList, int nExtern, double maxSpan)
		{
			// 根据激光弧段起止时间获得姿态数据序列, 同时为保证插值精度前后各延长 nExtern 个点
			size_t count = m_attList.size();
			if(count < size_t(2 * nExtern))
				return false;
			int nBegin = -1;
			int nEnd   = int(count);
			double span_T  = t0 - m_attList[0].t;  // 换算成相对时间
			// 采用二分法，2008 - 05 - 11
			size_t left  = 1;
			size_t right = count - 1;
			int n = 0;
			while(left < right)
			{
				n++;
				int    middle  = int(left + right)/2;
				double time_L = (m_attList[middle - 1].t - m_attList[0].t);
				double time_R = (m_attList[middle].t     - m_attList[0].t);
				if(span_T >= time_L && span_T <= time_R) 
				{// 终止条件
					nBegin = middle;
					break;
				}
				if(span_T < time_L) 
					right = middle - 1;
				else 
					left  = middle + 1;
			}
			if(right == left)
			{
				double time_L = (m_attList[left - 1].t - m_attList[0].t);
				double time_R = (m_attList[left].t     - m_attList[0].t);
				if(span_T >= time_L && span_T <= time_R) 
				{// 终止条件
					nBegin = int(left);
				}
			}
			if(nBegin == -1)
				return false;
			// nBegin 搜索成功后，开始搜索 nEnd
			for(size_t s_j = nBegin; s_j < count; s_j++)
			{
				if(m_attList[s_j].t - t1 >= 0)
				{
					nEnd = int(s_j);
					break;
				}
			}
			// 向两边延伸
			nBegin = nBegin - nExtern;
			nEnd   = nEnd   + nExtern;
			if(nBegin < 0 || nEnd >= int(count))
				return false;
			// 整理姿态数据列表
			attList.clear();
			for(int i = nBegin; i <= nEnd; i++)//2015/11/17,刘俊宏
			{
				if(i == nBegin)
					attList.push_back(m_attList[i]);
				else
				{
					if(fabs(m_attList[i].t - m_attList[i - 1].t) <= maxSpan)					
						attList.push_back(m_attList[i]);			
					else
						break;
				}
			}
			if(attList.front().t - t0 > 0 || attList.back().t - t1 < 0)
				return false;
			else
     			return true;
		}
		//bool SLROrbitComparison::main_cstg(string strCstgObsFileName, vector<SLREditedObsArc>& editedObsArc, double min_elevation, double threshold_res, bool bResEdit)
		//{
		//	cstgSLRObsFile obsFile;
  //          obsFile.open(strCstgObsFileName);
		//	size_t count_pass = obsFile.m_data.size();
		//	if(count_pass <= 0)
		//	{
		//		printf("cstg 激光数据为空!");
		//		return false;
		//	}
		//	editedObsArc.clear();
		//	for(size_t s_i = 0; s_i < count_pass; s_i++)
		//	{
		//		SLREditedObsArc editedObsArc_i;
		//		cstgSinglePassArc passArc = obsFile.m_data[s_i];
		//		size_t count = passArc.normalDataRecordList.size();
		//		// 激光波长换算成微米
		//		double Wavelength = passArc.normalHeaderRecord.Wavelength * 0.001;
		//		// 确定本弧段的起止时间
		//		GPST t0 = m_TimeCoordConvert.UTC2GPST(passArc.getTime(passArc.normalDataRecordList[0]));
		//		GPST t1 = m_TimeCoordConvert.UTC2GPST(passArc.getTime(passArc.normalDataRecordList[count - 1]));
		//		// 根据激光弧段起止时间获得轨道数据序列
		//		vector<TimePosVel> orbList;
		//		if(!getSubArcOrbList(t0, t1, orbList, 6))
		//			continue;
		//		// 获得激光发射时刻的测站位置(由于测站漂移的速度较小,一次跟踪弧端内的时间差异不做区分)
		//		POS6D staPV;
		//		if(!getStaPosvel(t0, passArc.normalHeaderRecord.nCDPPadID, staPV))
		//			continue;
		//		// 计算测站的大地经纬度
		//		BLH blh;
		//		m_TimeCoordConvert.XYZ2BLH(staPV.getPos(), blh);
		//		double fai = blh.B; 
		//		double h = blh.H; 
		//		// 获得测站的偏心数据(ilrs)
		//		ENU ecc;
		//		if(!getStaEcc(t0, passArc.normalHeaderRecord.nCDPPadID, ecc))
		//			continue;
		//		vector<SLREditedObsElement> editedSLRObsList_Arc;
		//		editedSLRObsList_Arc.clear();
		//		for(size_t s_j = 0; s_j < count; s_j++)
		//		{
		//			POS6D  leoPV;
		//			double P_J2000[3]; // 惯性坐标, 用于坐标系转换
		//			double P_ITRF[3];  // 地固坐标
		//			cstgDataRecord Record  = passArc.normalDataRecordList[s_j];
		//			double Temperature     = Record.SurfaceTemperature * 0.1;
		//			double Pressure        = Record.SurfacePressure * 0.1;
		//			double Humidity        = Record.SurfaceRelHumidity;
		//			SLREditedObsElement  editedLine;
		//			editedLine.wavelength  = Wavelength;
		//			editedLine.temperature = Temperature;
		//	        editedLine.pressure    = Pressure;
		//	        editedLine.humidity    = Humidity;
		//			editedLine.id = passArc.normalHeaderRecord.nCDPPadID;
		//			editedLine.staPos_ECEF = staPV.getPos();
		//			// 换算成单程激光距离
		//			editedLine.obs = Record.LaserRange * 1.0E-12 * SPEED_LIGHT / 2.0;
		//			// 计算地面激光 fire 时刻 editedLine.Ts ( 转换成 GPST )
		//			UTC ts_utc = passArc.getTime(Record);
		//			editedLine.Ts = m_TimeCoordConvert.UTC2GPST(ts_utc);
		//			// 获得测站在 J2000 惯性系中的位置(Ts 时刻)
		//			POS3D staPos_J2000;
		//			POS3D leoPos_J2000;
		//			P_ITRF[0] = staPV.x;
		//			P_ITRF[1] = staPV.y;
		//			P_ITRF[2] = staPV.z;
		//			m_TimeCoordConvert.ECEF_J2000(editedLine.Ts, P_J2000, P_ITRF, false);
		//			staPos_J2000.x = P_J2000[0];
		//			staPos_J2000.y = P_J2000[1];
		//			staPos_J2000.z = P_J2000[2];
		//			double delay = editedLine.obs / SPEED_LIGHT; // 初始化延迟数据
		//			GPST t = editedLine.Ts + delay; // 等效瞬时观测时间
		//			UTC tr_utc = ts_utc + delay;
		//			TDB t_TDB = m_TimeCoordConvert.GPST2TDB(t); // 获得TDB时间--提供太阳历参考时间
		//			double jd_TDB = m_TimeCoordConvert.DayTime2JD(t_TDB); // 获得儒略日
		//			// 根据 orbitlist 插值获得此 T 时刻附近的 LEO 卫星的轨道位置序列
		//			TimePosVel  orbit;
		//			if(!getOrbInterp(orbList, t, orbit))
		//				continue;
		//			leoPV.setPos(orbit.pos);
		//			leoPV.setVel(orbit.vel);
		//			// 计算卫星的仰角 E
		//			POS3D p_station = vectorNormal(staPV.getPos());
		//			POS3D p_sat = vectorNormal(leoPV.getPos() - staPV.getPos());
		//			double E = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;
		//			// 获得太阳位置 
		//			POS3D sunPos_ITRF;
		//			POS3D sunPos_J2000;
		//			m_JPLEphFile.getSunPos_Delay_EarthCenter(jd_TDB, P_J2000); 
		//			for(int i = 0; i < 3; i ++)
		//				P_J2000[i] = P_J2000[i] * 1000; // 换算成米
		//			sunPos_J2000.x = P_J2000[0];
		//			sunPos_J2000.y = P_J2000[1];
		//			sunPos_J2000.z = P_J2000[2];
		//			m_TimeCoordConvert.J2000_ECEF(t, P_J2000, P_ITRF, false); // 坐标系转换
		//			sunPos_ITRF.x = P_ITRF[0];
		//			sunPos_ITRF.y = P_ITRF[1];
		//			sunPos_ITRF.z = P_ITRF[2];
		//			// 获得月球的位置
		//			POS3D moonPos_ITRF;
		//			m_JPLEphFile.getPlanetPos(JPLEph_Moon, jd_TDB, P_J2000);  // 获得J2000系下的太阳相对地心的位置（千米）
		//			for(int i = 0; i < 3; i ++)
		//				P_J2000[i] = P_J2000[i] * 1000;                       // 换算成米
		//			m_TimeCoordConvert.J2000_ECEF(t, P_J2000, P_ITRF, false); // 坐标系转换
		//			moonPos_ITRF.x  = P_ITRF[0];
		//			moonPos_ITRF.y  = P_ITRF[1];
		//			moonPos_ITRF.z  = P_ITRF[2];
		//			// 获得极移数据()
		//			double xp = 0;
		//			double yp = 0;
		//			if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_2003)
		//				m_TimeCoordConvert.m_eopRapidFileIAU2000.getPoleOffset(tr_utc, xp, yp);
		//			if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_1996)
		//				m_TimeCoordConvert.m_eopc04File.getPoleOffset(tr_utc, xp, yp);
		//			// 计算各项修正
		//			/* 第一步：进行对流层改正 */
		//			editedLine.bOn_Trop = 1;
		//			editedLine.dr_correct_Trop = SLRPreproc::tropCorrect_Marini_IERS2010(Temperature, Pressure, Humidity, Wavelength, E, fai, h);
		//			/*editedLine.wavelength =  SLRPreproc::tropCorrect_Marini_IERS2003(Temperature, Pressure, Humidity, Wavelength, E, fai, h)
		//				                   - SLRPreproc::tropCorrect_Marini_IERS2010(Temperature, Pressure, Humidity, Wavelength, E, fai, h);*/
		//			/* 第二步：相对论改正     */
		//			editedLine.bOn_Relativity = 1;
		//			editedLine.dr_correct_Relativity = SLRPreproc::relativityCorrect(sunPos_ITRF, leoPV.getPos(), staPV.getPos());
		//			/* 第三步：测站偏心改正   */
		//			editedLine.bOn_StaEcc = 1;
		//			editedLine.dr_correct_StaEcc = SLRPreproc::staEccCorrect(staPV.getPos(), leoPV.getPos(), ecc);
		//			 // 获得卫星在 J200 0系下位置
		//			P_ITRF[0] = leoPV.x;
		//			P_ITRF[1] = leoPV.y;
		//			P_ITRF[2] = leoPV.z;
		//			m_TimeCoordConvert.ECEF_J2000(t, P_J2000, P_ITRF, false);
		//			leoPos_J2000.x = P_J2000[0];
		//			leoPos_J2000.y = P_J2000[1];
		//			leoPos_J2000.z = P_J2000[2];
		//			/* 第四步：卫星质心改正   */
		//			editedLine.bOn_SatMco = 1;
		//			editedLine.dr_correct_SatMco = 0.0;
		//		    // 利用姿态数据进行部位修正
		//			Matrix matATT;
		//			if(m_attFile.getAttMatrix(t, matATT))
		//			{
		//				matATT = matATT.Transpose(); 
		//				Matrix matPCO(3, 1);
		//				matPCO.SetElement(0, 0, m_mcoLaserRetroReflector.x);
		//				matPCO.SetElement(1, 0, m_mcoLaserRetroReflector.y);
		//				matPCO.SetElement(2, 0, m_mcoLaserRetroReflector.z);
		//				matPCO = matATT * matPCO;
		//				POS3D vecLos = vectorNormal(leoPos_J2000 - staPos_J2000);
		//				editedLine.dr_correct_SatMco = matPCO.GetElement(0, 0) * vecLos.x
		//									         + matPCO.GetElement(1, 0) * vecLos.y
		//										     + matPCO.GetElement(2, 0) * vecLos.z;
		//			}
		//			else
		//			{
		//				editedLine.dr_correct_SatMco = SLRPreproc::satMassCenterCorrect_ECEF(t, staPV.getPos(), leoPV, m_mcoLaserRetroReflector);
		//			}
		//			/* 第五步：潮汐改正       */
		//			editedLine.bOn_Tide = 1;
		//			StaOceanTide sotDatum;
		//			m_staOldFile.getStaOceanTide(passArc.normalHeaderRecord.nCDPPadID, sotDatum);
		//			//UT1 ut1 = m_TimeCoordConvert.GPST2UT1(t);
		//			//double gmst = 0.0; // 单位: 弧度
		//			//if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_2003)
		//			//{
		//			//	double jY2000_TDT = m_TimeCoordConvert.DayTime2J2000Year(m_TimeCoordConvert.GPST2TDT(t));
		//			//	gmst = m_TimeCoordConvert.IAU2000A_GMST(ut1, jY2000_TDT);
		//			//}
		//			//if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_1996)
		//			//	gmst = m_TimeCoordConvert.IAU1996_GMST(ut1);
		//			editedLine.dr_correct_Tide = SLRPreproc::tideCorrect(t, sunPos_ITRF, moonPos_ITRF, staPV.getPos(), leoPV.getPos(), sotDatum, xp, yp);
		//			// 总的延迟量
		//			editedLine.obscorrected_value =  editedLine.dr_correct_Trop
		//				                           + editedLine.dr_correct_Relativity
		//										   + editedLine.dr_correct_StaEcc
		//										   + editedLine.dr_correct_SatMco
		//										   + editedLine.dr_correct_Tide;
		//			double dDelay_k_1 = 0;
		//			double dR_up = editedLine.obs;
		//			while(fabs(delay - dDelay_k_1) > 1.0E-8)
		//			{
		//				// 更新延迟时间
		//				dDelay_k_1 = delay;
		//				// 根据 dDelay 计算上行激光 reflect 时间
		//				editedLine.Tr = editedLine.Ts + delay;
		//				// 获得 J2000 惯性系下的卫星轨道 
		//				getOrbInterp(orbList, editedLine.Tr, orbit);
		//				leoPV.setPos(orbit.pos);
		//				leoPV.setVel(orbit.vel);
		//				editedLine.leoPV_ECEF = leoPV;
		//				P_ITRF[0] = leoPV.x;
		//				P_ITRF[1] = leoPV.y;
		//				P_ITRF[2] = leoPV.z;
		//				m_TimeCoordConvert.ECEF_J2000(editedLine.Tr, P_J2000, P_ITRF, false);
		//				leoPos_J2000.x = P_J2000[0];
		//				leoPos_J2000.y = P_J2000[1];
		//				leoPos_J2000.z = P_J2000[2];
		//				// 修正地球运动对卫星状态矢量的影响
		//				// 获得地球运动的速度
		//				double PV[6];
		//				POS3D  EarthVel;
		//				m_JPLEphFile.getEarthPosVel(jd_TDB, PV);
		//				EarthVel.x = PV[3] * 1000;
		//				EarthVel.y = PV[4] * 1000;
		//				EarthVel.z = PV[5] * 1000;
		//				POS3D leoPos_J2000_xz = leoPos_J2000;// + EarthVel * delay;
		//				// 计算上行几何距离
		//				dR_up = sqrt(pow(staPos_J2000.x - leoPos_J2000_xz.x, 2) +
		//							 pow(staPos_J2000.y - leoPos_J2000_xz.y, 2) +
		//							 pow(staPos_J2000.z - leoPos_J2000_xz.z, 2));
		//				delay = (dR_up + editedLine.obscorrected_value) / SPEED_LIGHT;
		//			}
		//			// 激光反射时刻 editedLine.Tr, 卫星的轨道位置 leoPos_J2000
		//			// 迭代计算下行激光延迟时间
		//			dDelay_k_1 = 0;
		//			double dR_down = editedLine.obs;
		//			while(fabs(delay - dDelay_k_1) > 1.0E-8)
		//			{// 更新延迟时间
		//				dDelay_k_1 = delay;
		//				// 根据 dDelay 计算地面激光接收时间
		//				GPST TR = editedLine.Tr + delay;
		//				// 获得 J2000 惯性系下的观测站位置
		//				P_ITRF[0] = staPV.x;
		//				P_ITRF[1] = staPV.y;
		//				P_ITRF[2] = staPV.z;
		//				m_TimeCoordConvert.ECEF_J2000(TR, P_J2000, P_ITRF, false);
		//				staPos_J2000.x = P_J2000[0];
		//				staPos_J2000.y = P_J2000[1];
		//				staPos_J2000.z = P_J2000[2];
		//				// 修正地球运动对卫星状态矢量的影响
		//				// 获得地球运动的速度
		//				double PV[6];
		//				POS3D  EarthVel;
		//				m_JPLEphFile.getEarthPosVel(jd_TDB, PV);
		//				EarthVel.x = PV[3] * 1000;
		//				EarthVel.y = PV[4] * 1000;
		//				EarthVel.z = PV[5] * 1000;
		//				POS3D staPos_J2000_xz = staPos_J2000;// + EarthVel * delay;
		//				// 计算下行几何距离
		//				dR_down = sqrt(pow(staPos_J2000_xz.x - leoPos_J2000.x, 2) +
		//							   pow(staPos_J2000_xz.y - leoPos_J2000.y, 2) +
		//							   pow(staPos_J2000_xz.z - leoPos_J2000.z, 2));
		//				delay = (dR_down + editedLine.obscorrected_value) / SPEED_LIGHT;
		//			}
		//			editedLine.r_mean = 0.5 * (dR_down + dR_up);
		//			
		//			if(editedLine.getStaLosElevation() >= min_elevation 
		//			&& fabs(editedLine.r_mean - editedLine.obs + editedLine.obscorrected_value) <= threshold_res)
		//			{
		//				editedSLRObsList_Arc.push_back(editedLine);
		//			}
		//		}
		//		// 残差再编辑
		//		size_t countObs_Arc = editedSLRObsList_Arc.size();
		//		if(bResEdit)
		//		{
		//			if(countObs_Arc > 0)
		//			{
		//				double *x     = new double [countObs_Arc];
		//				double *y     = new double [countObs_Arc];
		//				double *y_fit = new double [countObs_Arc];
		//				double *w     = new double [countObs_Arc];
		//				for(size_t s_j = 0; s_j < countObs_Arc; s_j++)
		//				{
		//					x[s_j] = editedSLRObsList_Arc[s_j].Ts  - editedSLRObsList_Arc[0].Ts;
		//					y[s_j] = editedSLRObsList_Arc[s_j].obs - editedSLRObsList_Arc[s_j].obscorrected_value- editedSLRObsList_Arc[s_j].r_mean;
		//				}
		//				RobustPolyFit(x, y, w, int(countObs_Arc), y_fit, 3);
		//				editedObsArc_i.editedSLRObsList.clear();
		//				editedObsArc_i.rms  = 0;
		//				editedObsArc_i.mean = 0;
		//				for(size_t s_jj = 0; s_jj < countObs_Arc; s_jj++)
		//				{
		//					if(w[s_jj] == 1.0)
		//					{
		//						editedObsArc_i.editedSLRObsList.push_back(editedSLRObsList_Arc[s_jj]);
		//						editedObsArc_i.mean += editedSLRObsList_Arc[s_jj].obs - editedSLRObsList_Arc[s_jj].obscorrected_value- editedSLRObsList_Arc[s_jj].r_mean;
		//						editedObsArc_i.rms += pow(editedSLRObsList_Arc[s_jj].obs - editedSLRObsList_Arc[s_jj].obscorrected_value- editedSLRObsList_Arc[s_jj].r_mean, 2);
		//					}
		//				}
		//				delete x;
		//				delete y;
		//				delete y_fit;
		//				delete w;
		//			}
		//		}
		//		else
		//		{
		//			editedObsArc_i.editedSLRObsList.clear();
		//			editedObsArc_i.rms  = 0;
		//			editedObsArc_i.mean = 0;
		//			for(size_t s_jj = 0; s_jj < countObs_Arc; s_jj++)
		//			{
		//				editedObsArc_i.editedSLRObsList.push_back(editedSLRObsList_Arc[s_jj]);
		//				editedObsArc_i.mean += editedSLRObsList_Arc[s_jj].obs - editedSLRObsList_Arc[s_jj].obscorrected_value- editedSLRObsList_Arc[s_jj].r_mean;
		//				editedObsArc_i.rms  += pow(editedSLRObsList_Arc[s_jj].obs - editedSLRObsList_Arc[s_jj].obscorrected_value- editedSLRObsList_Arc[s_jj].r_mean, 2);
		//			}
		//			
		//		}
		//		if(editedObsArc_i.editedSLRObsList.size() > 0)
		//		{
		//			editedObsArc_i.id   = editedObsArc_i.editedSLRObsList[0].id;
		//			editedObsArc_i.mean = editedObsArc_i.mean / editedObsArc_i.editedSLRObsList.size();
		//			editedObsArc_i.rms  = sqrt(editedObsArc_i.rms / editedObsArc_i.editedSLRObsList.size());
		//			editedObsArc.push_back(editedObsArc_i);
		//			// printf("弧段%4d比对完毕, rms = %.4f!\n", s_i, editedObsArc_i.rms);
		//		}
		//	}
		//	// 输出比对文件
		//	if(editedObsArc.size() > 0)
		//	{
		//		string folder = strCstgObsFileName.substr(0, strCstgObsFileName.find_last_of("\\"));
		//		string obsFileName = strCstgObsFileName.substr(strCstgObsFileName.find_last_of("\\") + 1);
		//		string obsFileName_noexp = obsFileName.substr(0, obsFileName.find_last_of("."));
		//		char slrComparisonFilePath[300];
		//		sprintf(slrComparisonFilePath,"%s\\slrComparison_%s.txt", folder.c_str(), obsFileName_noexp.c_str());
		//		FILE * pFile = fopen(slrComparisonFilePath, "w+");
		//		double rms  = 0;
		//		double mean = 0;
		//		for(size_t s_i = 0; s_i < editedObsArc.size(); s_i++)
		//		{
		//			rms  += editedObsArc[s_i].rms;
		//			mean += editedObsArc[s_i].mean;
		//		}
		//		rms  = rms / editedObsArc.size();
		//		mean = mean / editedObsArc.size();
		//		fprintf(pFile, "## 总弧段个数           %10d\n",   editedObsArc.size());
		//		fprintf(pFile, "## 截至高度角  (deg)    %10.1f\n", min_elevation);
		//		fprintf(pFile, "## 总均值      (m)      %10.4f\n", mean);
		//		fprintf(pFile, "## 总均方根    (m)      %10.4f\n", rms);
		//		fprintf(pFile, "## 弧段序号    测站           个数          均值        均方根\n");
		//		for(size_t s_i = 0; s_i < editedObsArc.size(); s_i++)
		//		{
		//			fprintf(pFile, "   %-12d%4d%15d%14.4f%14.4f\n", s_i + 1, editedObsArc[s_i].id, editedObsArc[s_i].editedSLRObsList.size(), editedObsArc[s_i].mean, editedObsArc[s_i].rms);
		//		}
		//		fprintf(pFile, "## 弧段序号    测站           时间       波长(μm)      温度(K)       湿度(%%)      压强(mb)        高度角          残差\n");
		//		int year = editedObsArc[0].editedSLRObsList[0].Ts.year;
		//		UTC t0 = UTC(year, 1, 1, 0, 0, 0.0);
		//		for(size_t s_i = 0; s_i < editedObsArc.size(); s_i++)
		//		{
		//			for(size_t s_j = 0; s_j < editedObsArc[s_i].editedSLRObsList.size(); s_j++)
		//			{
		//				double res = editedObsArc[s_i].editedSLRObsList[s_j].obs
		//					       - editedObsArc[s_i].editedSLRObsList[s_j].obscorrected_value
		//						   - editedObsArc[s_i].editedSLRObsList[s_j].r_mean;
		//				double day = (editedObsArc[s_i].editedSLRObsList[s_j].Ts - t0) / 86400.0;
		//				fprintf(pFile, "   %-12d%4d%15.8f%14.5f%14.1f%14.1f%14.1f%14.1f%14.4f\n", s_i + 1, 
		//					                                                                editedObsArc[s_i].id, 
		//																					day, 
		//																					editedObsArc[s_i].editedSLRObsList[s_j].wavelength,
		//																					editedObsArc[s_i].editedSLRObsList[s_j].temperature,
		//																					editedObsArc[s_i].editedSLRObsList[s_j].humidity,
  //                                                                                          editedObsArc[s_i].editedSLRObsList[s_j].pressure,
		//																					editedObsArc[s_i].editedSLRObsList[s_j].getStaLosElevation(), 
		//																					res);
		//			}
		//		}
		//		fclose(pFile);
		//	}
		//	return true;
		//}

		// 子程序名称： mainOrbComparison 
		// 功能：激光比对
		// 变量类型：strSLRObsFileName  :  SLR 观测数据文件路径
		//           nObsFileType      :  观测数据类型: 0: cstg; 1: merit; 2: crd_npt; 3: crd_frd		
        //           editedObsArc      :  预处理后的观测数据弧段
		//           editedObsArc      :  预处理后的观测数据弧段		
        //           min_elevation     :  截止高度角, 默认值 15 度
		//           threshold_res     :  残差阈值,   默认值 0.3 米
		//           bResEdit          :  残差标记开关
		// 输入：strSLRObsFileName, nObsFileType, min_elevation, threshold_res, bResEdit
		// 输出：editedObsArc 
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2013/06/26
		// 版本时间：2013/06/26
		// 修改记录：1、2014/11/03,增加CRD格式的标准点数据处理。刘俊宏
        //           2、2019/10/31,增加CRD_FRD格式的标准点数据处理, 邵凯
		// 备注：
		bool SLROrbitComparison::mainOrbComparison(string strSLRObsFileName, int nObsFileType, vector<SLREditedObsArc>& editedObsArc, double min_elevation, double threshold_res, bool bResEdit)
		{
			if( nObsFileType!=0 && nObsFileType != 1 && nObsFileType != 2 && nObsFileType != 3)
			{
				printf("激光数据文件类型未知!\n");
				return false;
			}
			size_t count_pass = 0;
			cstgSLRObsFile obsFile_cstg;
			obsFile_cstg.m_bChecksum = m_bChecksum;
			if(nObsFileType == 0)
			{
				obsFile_cstg.open(strSLRObsFileName);
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
				obsFile_merit.open(strSLRObsFileName);
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
			crdSLRObsFile   obsFile_crd;
			if(nObsFileType == 2)        //2014/11/03
			{
				if(!obsFile_crd.open(strSLRObsFileName))
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
			crdFrdSLRObsFile   obsFile_crdFrd;
			if(nObsFileType == 3)        //2019/11/03
			{
				if(!obsFile_crdFrd.open(strSLRObsFileName))
				{
					printf("crd 激光数据文件打开失败!\n");
					return false;
				}
				count_pass = obsFile_crdFrd.m_data.size();
				if(count_pass <= 0)
				{
					printf("crd 激光数据为空!\n");
					return false;
				}
			}
			editedObsArc.clear();
			for(size_t s_i = 0; s_i < count_pass; s_i++)
			{
				SLREditedObsArc editedObsArc_i;
				size_t count;
				int nCDPPadID;
				string name_4c;
				GPST t0, t1;
				double Wavelength;
				if(nObsFileType == 0)
				{
					count = obsFile_cstg.m_data[s_i].normalDataRecordList.size();
					// 激光波长换算成微米
					Wavelength = obsFile_cstg.m_data[s_i].normalHeaderRecord.Wavelength * 0.001;
					// 确定本弧段的起止时间
					t0 = m_TimeCoordConvert.UTC2GPST(obsFile_cstg.m_data[s_i].getTime(obsFile_cstg.m_data[s_i].normalDataRecordList[0]));
					t1 = m_TimeCoordConvert.UTC2GPST(obsFile_cstg.m_data[s_i].getTime(obsFile_cstg.m_data[s_i].normalDataRecordList[count - 1]));
					nCDPPadID = obsFile_cstg.m_data[s_i].normalHeaderRecord.nCDPPadID;
				}
				if(nObsFileType == 1)
				{
					count = meritSinglePassArcList[s_i].size();
					// 激光波长换算成微米
					Wavelength = meritSinglePassArcList[s_i][0].Wavelength * 0.0001; // 换算成微米
					// 确定本弧段的起止时间
					t0 = m_TimeCoordConvert.UTC2GPST(meritSinglePassArcList[s_i][0].getTime());
					t1 = m_TimeCoordConvert.UTC2GPST(meritSinglePassArcList[s_i][count - 1].getTime());
					nCDPPadID = meritSinglePassArcList[s_i][0].StationID;
				}
				if(nObsFileType == 2)
				{
					count = obsFile_crd.m_data[s_i].crdDataRecordList.size();
					// 激光波长换算成微米
					Wavelength = obsFile_crd.m_data[s_i].crdConfig.Wavelength * 0.001;
					// 确定本弧段的起止时间
					t0 = m_TimeCoordConvert.UTC2GPST(obsFile_crd.m_data[s_i].getTime(obsFile_crd.m_data[s_i].crdDataRecordList.front()));
					t1 = m_TimeCoordConvert.UTC2GPST(obsFile_crd.m_data[s_i].getTime(obsFile_crd.m_data[s_i].crdDataRecordList.back()));
					nCDPPadID = obsFile_crd.m_data[s_i].crdHeader.nCDPPadID;
					name_4c   = obsFile_crd.m_data[s_i].crdHeader.name_4c;
				}
				if(nObsFileType == 3)
				{
					count = obsFile_crdFrd.m_data[s_i].crdDataRecordList.size();
					// 激光波长换算成微米
					Wavelength = obsFile_crdFrd.m_data[s_i].crdConfig.Wavelength * 0.001;
					// 确定本弧段的起止时间
					t0 = m_TimeCoordConvert.UTC2GPST(obsFile_crdFrd.m_data[s_i].getTime(obsFile_crdFrd.m_data[s_i].crdDataRecordList.front()));
					t1 = m_TimeCoordConvert.UTC2GPST(obsFile_crdFrd.m_data[s_i].getTime(obsFile_crdFrd.m_data[s_i].crdDataRecordList.back()));
					nCDPPadID = obsFile_crdFrd.m_data[s_i].crdHeader.nCDPPadID;
				}
				// 
				//if(!(nCDPPadID == 7090 || nCDPPadID == 7105 || nCDPPadID == 7501
				//	 || nCDPPadID == 7810 || nCDPPadID == 7825 || nCDPPadID == 7839
				//	 || nCDPPadID == 7840 || nCDPPadID == 7841 || nCDPPadID == 7941
				//	 || nCDPPadID == 8834))
				//	continue;
				// 根据激光弧段起止时间获得轨道数据序列
				vector<TimePosVel> orbList;
				if(!getSubArcOrbList(t0, t1, orbList, 6, 15 * 60.0))
					continue;
				// 根据激光弧段起止时间获得姿态数据序列
				vector<TimeAttLine> attList;
				attList.clear();
				//if(!getSubArcAttList(t0, t1, attList, 6, 15 * 60.0))
				//	printf("姿态数据为空!\n");
				getSubArcAttList(t0, t1, attList, 6, 15 * 60.0);
				// 获得激光发射时刻的测站位置(由于测站漂移的速度较小,一次跟踪弧端内的时间差异不做区分)
				POS6D staPV;
				if(nObsFileType == 2 && m_staPos_model == Snx)
				{
					POS3D pos_j, posSTD_j;
					if(m_snxFile.getStaPos(name_4c, pos_j, posSTD_j, true))
					{
						staPV.x = pos_j.x;
						staPV.y = pos_j.y;
						staPV.z = pos_j.z;
						staPV.vx = 0.0;
						staPV.vy = 0.0;
						staPV.vz = 0.0;
					}
					else
					{
						if(!getStaPosvel(t0, nCDPPadID, staPV))
						  continue;
					}
				}			
				else
				{
					if(!getStaPosvel(t0, nCDPPadID, staPV))
						continue;
				}
				// 计算测站的大地经纬度
				BLH blh;
				m_TimeCoordConvert.XYZ2BLH(staPV.getPos(), blh);
				double fai = blh.B; 
				double h = blh.H; 
				// 获得测站的偏心数据(ilrs)
				ENU ecc;
				if(!getStaEcc(t0, nCDPPadID, ecc))
					continue;
				vector<SLREditedObsElement> editedSLRObsList_Arc;
				editedSLRObsList_Arc.clear();
				for(size_t s_j = 0; s_j < count; s_j++)
				{
					POS6D  leoPV;
					double P_J2000[6]; // 惯性坐标, 用于坐标系转换
					double P_ITRF[6];  // 地固坐标
					double Temperature, Pressure, Humidity;
					UTC ts_utc;
					SLREditedObsElement  editedLine;
					editedLine.staPos_ECEF = staPV.getPos();
					editedLine.staBLH = blh;
					if(nObsFileType == 0)
					{
						Temperature = obsFile_cstg.m_data[s_i].normalDataRecordList[s_j].SurfaceTemperature * 0.1;
						Pressure    = obsFile_cstg.m_data[s_i].normalDataRecordList[s_j].SurfacePressure * 0.1;
						Humidity    = obsFile_cstg.m_data[s_i].normalDataRecordList[s_j].SurfaceRelHumidity;
						// 换算成单程激光距离
						editedLine.obs = obsFile_cstg.m_data[s_i].normalDataRecordList[s_j].LaserRange * 1.0E-12 * SPEED_LIGHT / 2.0;
						// 计算地面激光 fire 时刻 editedLine.Ts ( 转换成 GPST )
						ts_utc = obsFile_cstg.m_data[s_i].getTime(obsFile_cstg.m_data[s_i].normalDataRecordList[s_j]);
						editedLine.Ts = m_TimeCoordConvert.UTC2GPST(ts_utc);
					}
					if(nObsFileType == 1)
					{
						Temperature = meritSinglePassArcList[s_i][s_j].SurfaceTemperature * 0.1;
						Pressure    = meritSinglePassArcList[s_i][s_j].SurfacePressure * 0.1;
						Humidity    = meritSinglePassArcList[s_i][s_j].SurfaceRelHumidity;
						// 换算成单程激光距离
						editedLine.obs = meritSinglePassArcList[s_i][s_j].LaserRange * 1.0E-12 * SPEED_LIGHT / 2.0;
						// 计算地面激光 fire 时刻 editedLine.Ts ( 转换成 GPST )
						ts_utc = meritSinglePassArcList[s_i][s_j].getTime();
						editedLine.Ts = m_TimeCoordConvert.UTC2GPST(ts_utc);
					}
					if(nObsFileType == 2)
					{
						Temperature = obsFile_crd.m_data[s_i].crdDataRecordList[s_j].SurfaceTemperature;
						Pressure    = obsFile_crd.m_data[s_i].crdDataRecordList[s_j].SurfacePressure;
						Humidity    = obsFile_crd.m_data[s_i].crdDataRecordList[s_j].SurfaceRelHumidity;
						// 换算成单程激光距离
						if(obsFile_crd.m_data[s_i].crdHeader.nRangeType == 2)
							editedLine.obs = obsFile_crd.m_data[s_i].crdDataRecordList[s_j].TimeofFlight * SPEED_LIGHT / 2.0;
						else
							continue;//暂不做处理
						// 计算地面激光 fire 时刻 editedLine.Ts ( 转换成 GPST )
						ts_utc = obsFile_crd.m_data[s_i].getTime(obsFile_crd.m_data[s_i].crdDataRecordList[s_j]);
						editedLine.Ts = m_TimeCoordConvert.UTC2GPST(ts_utc);
					}
					if(nObsFileType == 3)
					{
						Temperature = obsFile_crdFrd.m_data[s_i].crdDataRecordList[s_j].SurfaceTemperature;
						Pressure    = obsFile_crdFrd.m_data[s_i].crdDataRecordList[s_j].SurfacePressure;
						Humidity    = obsFile_crdFrd.m_data[s_i].crdDataRecordList[s_j].SurfaceRelHumidity;
						// 换算成单程激光距离
						if(obsFile_crdFrd.m_data[s_i].crdHeader.nRangeType == 2)
							editedLine.obs = obsFile_crdFrd.m_data[s_i].crdDataRecordList[s_j].TimeofFlight * SPEED_LIGHT / 2.0;
						else
							continue;//暂不做处理
						// 计算地面激光 fire 时刻 editedLine.Ts ( 转换成 GPST )
						ts_utc = obsFile_crdFrd.m_data[s_i].getTime(obsFile_crdFrd.m_data[s_i].crdDataRecordList[s_j]);
						editedLine.Ts = m_TimeCoordConvert.UTC2GPST(ts_utc);
					}
					editedLine.wavelength  = Wavelength;
					editedLine.temperature = Temperature;
			        editedLine.pressure    = Pressure;
			        editedLine.humidity    = Humidity;
					editedLine.id = nCDPPadID;
					// 获得测站在 J2000 惯性系中的位置(Ts 时刻)
					POS3D staPos_J2000;
					POS6D leoPV_J2000;
					P_ITRF[0] = staPV.x;
					P_ITRF[1] = staPV.y;
					P_ITRF[2] = staPV.z;
					m_TimeCoordConvert.ECEF_J2000(editedLine.Ts, P_J2000, P_ITRF, false);
					staPos_J2000.x = P_J2000[0];
					staPos_J2000.y = P_J2000[1];
					staPos_J2000.z = P_J2000[2];
					double delay = editedLine.obs / SPEED_LIGHT; // 初始化延迟数据
					GPST t = editedLine.Ts + delay; // 等效瞬时观测时间
					UTC tr_utc = ts_utc + delay;
					TDB t_TDB = m_TimeCoordConvert.GPST2TDB(t); // 获得TDB时间--提供太阳历参考时间
					double jd_TDB = m_TimeCoordConvert.DayTime2JD(t_TDB); // 获得儒略日
					// 根据 orbitlist 插值获得此 T 时刻附近的 LEO 卫星的轨道位置序列
					TimePosVel  orbit;
					if(!getOrbInterp(orbList, t, orbit))
						continue;
					leoPV.setPos(orbit.pos);
					leoPV.setVel(orbit.vel);
					// 计算卫星的仰角 E
					POS3D p_station = vectorNormal(staPV.getPos());
					POS3D p_sat = vectorNormal(leoPV.getPos() - staPV.getPos());
					//double E = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;
					p_station.z = p_station.z / pow(1.0 - EARTH_F, 2); // 20150608, 考虑到地球扁率的影响, 卫星仰角的计算进行了修正, 谷德峰
					p_station = vectorNormal(p_station);
					double E = 90 - acos(vectorDot(p_station, p_sat)) * 180 / PI;
					// 获得太阳位置 
					POS3D sunPos_ITRF;
					POS3D sunPos_J2000;
					m_JPLEphFile.getSunPos_Delay_EarthCenter(jd_TDB, P_J2000); 
					for(int i = 0; i < 3; i ++)
						P_J2000[i] = P_J2000[i] * 1000; // 换算成米
					sunPos_J2000.x = P_J2000[0];
					sunPos_J2000.y = P_J2000[1];
					sunPos_J2000.z = P_J2000[2];
					m_TimeCoordConvert.J2000_ECEF(t, P_J2000, P_ITRF, false); // 坐标系转换
					sunPos_ITRF.x = P_ITRF[0];
					sunPos_ITRF.y = P_ITRF[1];
					sunPos_ITRF.z = P_ITRF[2];
					// 获得月球的位置
					POS3D moonPos_ITRF;
					m_JPLEphFile.getPlanetPos(JPLEph_Moon, jd_TDB, P_J2000);  // 获得J2000系下的太阳相对地心的位置（千米）
					for(int i = 0; i < 3; i ++)
						P_J2000[i] = P_J2000[i] * 1000;                       // 换算成米
					m_TimeCoordConvert.J2000_ECEF(t, P_J2000, P_ITRF, false); // 坐标系转换
					moonPos_ITRF.x  = P_ITRF[0];
					moonPos_ITRF.y  = P_ITRF[1];
					moonPos_ITRF.z  = P_ITRF[2];
					// 获得极移数据()
					double xp = 0;
					double yp = 0;
					//if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_2003)
					//	m_TimeCoordConvert.m_eopRapidFileIAU2000.getPoleOffset(tr_utc, xp, yp);
					//if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_1996)
					//	m_TimeCoordConvert.m_eopc04File.getPoleOffset(tr_utc, xp, yp);
					// 根据文件类型进行修改，邵凯，2018/05/09
					if(m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04_1980 || m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04_2000A) 
						m_TimeCoordConvert.m_eopc04File.getPoleOffset(tr_utc,xp,yp); // 获得极移数据
					else if(m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04Total_1980 || m_TimeCoordConvert.m_eopFileType == IERSEOPFILE_C04Total_2000A) 
						m_TimeCoordConvert.m_eopc04TotalFile.getPoleOffset(tr_utc,xp,yp); // 获得极移数据
					else
						m_TimeCoordConvert.m_eopRapidFileIAU2000.getPoleOffset(tr_utc, xp, yp); // 获得极移数据

					// 计算各项修正
					/* 第一步：进行对流层改正 */
					editedLine.bOn_Trop = 1;
					editedLine.dr_correct_Trop = SLRPreproc::tropCorrect_Marini_IERS2010(Temperature, Pressure, Humidity, Wavelength, E, fai, h);
					/*editedLine.wavelength =  SLRPreproc::tropCorrect_Marini_IERS2003(Temperature, Pressure, Humidity, Wavelength, E, fai, h)
						                   - SLRPreproc::tropCorrect_Marini_IERS2010(Temperature, Pressure, Humidity, Wavelength, E, fai, h);*/
					/* 第二步：相对论改正     */
					editedLine.bOn_Relativity = 1;
					editedLine.dr_correct_Relativity = SLRPreproc::relativityCorrect(sunPos_ITRF, leoPV.getPos(), staPV.getPos());
					/* 第三步：测站偏心改正   */
					editedLine.bOn_StaEcc = 1;
					editedLine.dr_correct_StaEcc = SLRPreproc::staEccCorrect(staPV.getPos(), leoPV.getPos(), ecc);
					 // 获得卫星在 J200 0系下位置
					P_ITRF[0] = leoPV.x;
					P_ITRF[1] = leoPV.y;
					P_ITRF[2] = leoPV.z;
					P_ITRF[3] = leoPV.vx;
					P_ITRF[4] = leoPV.vy;
					P_ITRF[5] = leoPV.vz;
					m_TimeCoordConvert.ECEF_J2000(t, P_J2000, P_ITRF, true);
					leoPV_J2000.x  = P_J2000[0];
					leoPV_J2000.y  = P_J2000[1];
					leoPV_J2000.z  = P_J2000[2];
					leoPV_J2000.vx = P_J2000[3];
					leoPV_J2000.vy = P_J2000[4];
					leoPV_J2000.vz = P_J2000[5];
					/* 第四步：卫星质心改正   */
					editedLine.bOn_SatMco = 1;
					editedLine.dr_correct_SatMco = 0.0;
				    // 利用姿态数据进行部位修正
					Matrix matJ2000_ECEF;   // J2000到ECEF转换矩阵
					m_TimeCoordConvert.Matrix_J2000_ECEF(t, matJ2000_ECEF); // 接收时刻坐标系转换旋转矩阵, 用于后面的 LEO PCO 修正
					// 根据 attList 插值获得此 T 时刻附近的 LEO 卫星的姿态矩阵，邵凯，2021.06.24
					Matrix matATT;			
					if(int(attList.size()) > 0 && getAttMatrixInterp(attList, t, matATT))
					{
						matATT = matATT.Transpose(); 
						if(m_att_model == Body2ECEF)
						{
							matATT =  matJ2000_ECEF.Transpose() * matATT.Transpose();
						}
						Matrix matPCO(3, 1);
						matPCO.SetElement(0, 0, m_mcoLaserRetroReflector.x);
						matPCO.SetElement(1, 0, m_mcoLaserRetroReflector.y);
						matPCO.SetElement(2, 0, m_mcoLaserRetroReflector.z);
						matPCO = matATT * matPCO;
						POS3D vecLos = vectorNormal(leoPV_J2000.getPos() - staPos_J2000);
						editedLine.dr_correct_SatMco = matPCO.GetElement(0, 0) * vecLos.x
											         + matPCO.GetElement(1, 0) * vecLos.y
												     + matPCO.GetElement(2, 0) * vecLos.z;
					}
					else
					{
						if(m_bOn_YawAttitudeModel)
						{	
							editedLine.dr_correct_SatMco = SLRPreproc::satMassCenterCorrect_YawAttitudeModel(staPos_J2000, leoPV_J2000.getPos(), sunPos_J2000, m_mcoLaserRetroReflector); // 20151125, 调整到惯性系下修正, 谷德峰							
						}
						else//可用于BDS - GEO卫星
							//editedLine.dr_correct_SatMco = SLRPreproc::satMassCenterCorrect_ECEF(t, staPV.getPos(), leoPV, m_mcoLaserRetroReflector);
							editedLine.dr_correct_SatMco = SLRPreproc::satMassCenterCorrect_J2000(staPos_J2000, leoPV_J2000, m_mcoLaserRetroReflector); // 20151125, 调整到惯性系下修正, 谷德峰
					}
					editedLine.dr_correct_SatMco = editedLine.dr_correct_SatMco + m_constRangeBias; // 20170424， 谷德峰针对HY2A的固定偏差修正添加
					/* 第五步：潮汐改正       */
					editedLine.bOn_Tide = 1;
					StaOceanTide sotDatum;
					m_staOldFile.getStaOceanTide(nCDPPadID, sotDatum);
					//UT1 ut1 = m_TimeCoordConvert.GPST2UT1(t);
					//double gmst = 0.0; // 单位: 弧度
					//if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_2003)
					//{
					//	double jY2000_TDT = m_TimeCoordConvert.DayTime2J2000Year(m_TimeCoordConvert.GPST2TDT(t));
					//	gmst = m_TimeCoordConvert.IAU2000A_GMST(ut1, jY2000_TDT);
					//}
					//if(m_TimeCoordConvert.m_iersConventions == IERSCONVENTIONS_1996)
					//	gmst = m_TimeCoordConvert.IAU1996_GMST(ut1);
					editedLine.dr_correct_Tide = SLRPreproc::tideCorrect(t, sunPos_ITRF, moonPos_ITRF, staPV.getPos(), leoPV.getPos(), sotDatum, xp, yp);
					// 总的延迟量
					editedLine.obscorrected_value =  editedLine.dr_correct_Trop
						                           + editedLine.dr_correct_Relativity
												   + editedLine.dr_correct_StaEcc
												   + editedLine.dr_correct_SatMco
												   + editedLine.dr_correct_Tide;
					double dDelay_k_1 = 0;
					double dR_up = editedLine.obs;
					while(fabs(delay - dDelay_k_1) > 1.0E-8)
					{
						// 更新延迟时间
						dDelay_k_1 = delay;
						// 根据 dDelay 计算上行激光 reflect 时间
						editedLine.Tr = editedLine.Ts + delay;
						// 获得 J2000 惯性系下的卫星轨道 
						getOrbInterp(orbList, editedLine.Tr, orbit);
						leoPV.setPos(orbit.pos);
						leoPV.setVel(orbit.vel);
						editedLine.leoPV_ECEF = leoPV;
						P_ITRF[0] = leoPV.x;
						P_ITRF[1] = leoPV.y;
						P_ITRF[2] = leoPV.z;
						m_TimeCoordConvert.ECEF_J2000(editedLine.Tr, P_J2000, P_ITRF, false);
						leoPV_J2000.x = P_J2000[0];
						leoPV_J2000.y = P_J2000[1];
						leoPV_J2000.z = P_J2000[2];
						// 修正地球运动对卫星状态矢量的影响
						// 获得地球运动的速度
						double PV[6];
						POS3D  EarthVel;
						m_JPLEphFile.getEarthPosVel(jd_TDB, PV);
						EarthVel.x = PV[3] * 1000;
						EarthVel.y = PV[4] * 1000;
						EarthVel.z = PV[5] * 1000;
						POS3D leoPos_J2000_xz = leoPV_J2000.getPos();// + EarthVel * delay;
						// 计算上行几何距离
						dR_up = sqrt(pow(staPos_J2000.x - leoPos_J2000_xz.x, 2) +
									 pow(staPos_J2000.y - leoPos_J2000_xz.y, 2) +
									 pow(staPos_J2000.z - leoPos_J2000_xz.z, 2));
						delay = (dR_up + editedLine.obscorrected_value) / SPEED_LIGHT;
					}
					// 激光反射时刻 editedLine.Tr, 卫星的轨道位置 leoPos_J2000
					// 迭代计算下行激光延迟时间
					dDelay_k_1 = 0;
					double dR_down = editedLine.obs;
					while(fabs(delay - dDelay_k_1) > 1.0E-8)
					{// 更新延迟时间
						dDelay_k_1 = delay;
						// 根据 dDelay 计算地面激光接收时间
						GPST TR = editedLine.Tr + delay;
						// 获得 J2000 惯性系下的观测站位置
						P_ITRF[0] = staPV.x;
						P_ITRF[1] = staPV.y;
						P_ITRF[2] = staPV.z;
						m_TimeCoordConvert.ECEF_J2000(TR, P_J2000, P_ITRF, false);
						staPos_J2000.x = P_J2000[0];
						staPos_J2000.y = P_J2000[1];
						staPos_J2000.z = P_J2000[2];
						// 修正地球运动对卫星状态矢量的影响
						// 获得地球运动的速度
						double PV[6];
						POS3D  EarthVel;
						m_JPLEphFile.getEarthPosVel(jd_TDB, PV);
						EarthVel.x = PV[3] * 1000;
						EarthVel.y = PV[4] * 1000;
						EarthVel.z = PV[5] * 1000;
						POS3D staPos_J2000_xz = staPos_J2000;// + EarthVel * delay;
						// 计算下行几何距离
						dR_down = sqrt(pow(staPos_J2000_xz.x - leoPV_J2000.x, 2) +
									   pow(staPos_J2000_xz.y - leoPV_J2000.y, 2) +
									   pow(staPos_J2000_xz.z - leoPV_J2000.z, 2));
						delay = (dR_down + editedLine.obscorrected_value) / SPEED_LIGHT;
					}
					editedLine.r_mean = 0.5 * (dR_down + dR_up);

					if(editedLine.getStaLosElevation() >= min_elevation 
					&& fabs(editedLine.r_mean - editedLine.obs + editedLine.obscorrected_value) <= threshold_res)
					{
						editedSLRObsList_Arc.push_back(editedLine);
					}
				}
				// 残差再编辑
				size_t countObs_Arc = editedSLRObsList_Arc.size();
				if(bResEdit)
				{
					if(countObs_Arc > 0)
					{
						double *x     = new double [countObs_Arc];
						double *y     = new double [countObs_Arc];
						double *y_fit = new double [countObs_Arc];
						double *w     = new double [countObs_Arc];
						for(size_t s_j = 0; s_j < countObs_Arc; s_j++)
						{
							x[s_j] = editedSLRObsList_Arc[s_j].Ts  - editedSLRObsList_Arc[0].Ts;
							y[s_j] = editedSLRObsList_Arc[s_j].obs - editedSLRObsList_Arc[s_j].obscorrected_value- editedSLRObsList_Arc[s_j].r_mean;
						}
						RobustPolyFit(x, y, w, int(countObs_Arc), y_fit, 3);
						editedObsArc_i.editedSLRObsList.clear();
						editedObsArc_i.rms  = 0;
						editedObsArc_i.mean = 0;
						for(size_t s_jj = 0; s_jj < countObs_Arc; s_jj++)
						{
							if(w[s_jj] == 1.0)
							{
								editedObsArc_i.editedSLRObsList.push_back(editedSLRObsList_Arc[s_jj]);
								editedObsArc_i.mean += editedSLRObsList_Arc[s_jj].obs - editedSLRObsList_Arc[s_jj].obscorrected_value- editedSLRObsList_Arc[s_jj].r_mean;
								editedObsArc_i.rms += pow(editedSLRObsList_Arc[s_jj].obs - editedSLRObsList_Arc[s_jj].obscorrected_value- editedSLRObsList_Arc[s_jj].r_mean, 2);
							}
						}
						delete x;
						delete y;
						delete y_fit;
						delete w;
					}
				}
				else
				{
					editedObsArc_i.editedSLRObsList.clear();
					editedObsArc_i.rms  = 0;
					editedObsArc_i.mean = 0;
					for(size_t s_jj = 0; s_jj < countObs_Arc; s_jj++)
					{
						editedObsArc_i.editedSLRObsList.push_back(editedSLRObsList_Arc[s_jj]);
						editedObsArc_i.mean += editedSLRObsList_Arc[s_jj].obs - editedSLRObsList_Arc[s_jj].obscorrected_value- editedSLRObsList_Arc[s_jj].r_mean;
						editedObsArc_i.rms  += pow(editedSLRObsList_Arc[s_jj].obs - editedSLRObsList_Arc[s_jj].obscorrected_value- editedSLRObsList_Arc[s_jj].r_mean, 2);
					}
				}
				if(editedObsArc_i.editedSLRObsList.size() > 0)
				{
					editedObsArc_i.id   = editedObsArc_i.editedSLRObsList[0].id;
					editedObsArc_i.mean = editedObsArc_i.mean / editedObsArc_i.editedSLRObsList.size();
					editedObsArc_i.rms  = sqrt(editedObsArc_i.rms / editedObsArc_i.editedSLRObsList.size());
					editedObsArc.push_back(editedObsArc_i);
					// printf("弧段%4d比对完毕, rms = %.4f!\n", s_i, editedObsArc_i.rms);
				}
			}

			// 输出比对文件
			if(editedObsArc.size() > 0)
			{
				string folder = strSLRObsFileName.substr(0, strSLRObsFileName.find_last_of("\\"));
				string obsFileName = strSLRObsFileName.substr(strSLRObsFileName.find_last_of("\\") + 1);
				string obsFileName_noexp = obsFileName.substr(0, obsFileName.find_last_of("."));
				char slrComparisonFilePath[300];
				sprintf(slrComparisonFilePath,"%s\\slrComparison_%s.txt", folder.c_str(), obsFileName_noexp.c_str());
				FILE * pFile = fopen(slrComparisonFilePath, "w+");
				double rms  = 0;
				double mean = 0;
				for(size_t s_i = 0; s_i < editedObsArc.size(); s_i++)
				{
					rms  += editedObsArc[s_i].rms;
					mean += editedObsArc[s_i].mean;
				}
				rms  = rms / editedObsArc.size();
				mean = mean / editedObsArc.size();
				fprintf(pFile, "## 总弧段个数           %10d\n",   editedObsArc.size());
				fprintf(pFile, "## 截至高度角  (deg)    %10.1f\n", min_elevation);
				fprintf(pFile, "## 总均值      (m)      %10.4f\n", mean);
				fprintf(pFile, "## 总均方根    (m)      %10.4f\n", rms);
				fprintf(pFile, "## 弧段序号    测站             X             Y             Z              B              L         H           个数          均值        均方根\n");
				for(size_t s_i = 0; s_i < editedObsArc.size(); s_i++)
				{
					int deg_B    = int(floor(editedObsArc[s_i].editedSLRObsList[0].staBLH.B));
                    int min_B    = int(floor((editedObsArc[s_i].editedSLRObsList[0].staBLH.B - deg_B) * 60.0));
                    double sec_B = editedObsArc[s_i].editedSLRObsList[0].staBLH.B * 3600.0 - deg_B * 3600.0 - min_B * 60.0;
					int deg_L    = int(floor(editedObsArc[s_i].editedSLRObsList[0].staBLH.L));
                    int min_L    = int(floor((editedObsArc[s_i].editedSLRObsList[0].staBLH.L - deg_L) * 60.0));
                    double sec_L = editedObsArc[s_i].editedSLRObsList[0].staBLH.L * 3600.0 - deg_L * 3600.0 - min_L * 60.0;
					fprintf(pFile, "   %-12d%4d%14.4f%14.4f%14.4f  %4d %3d %4.1f  %4d %3d %4.1f%10.4f%15d%14.4f%14.4f\n", 
						               s_i + 1, 
									   editedObsArc[s_i].id, 
									   editedObsArc[s_i].editedSLRObsList[0].staPos_ECEF.x,
									   editedObsArc[s_i].editedSLRObsList[0].staPos_ECEF.y,
									   editedObsArc[s_i].editedSLRObsList[0].staPos_ECEF.z,
									   deg_B,
                                       min_B,
									   sec_B,
									   deg_L,
                                       min_L,
									   sec_L,
									   editedObsArc[s_i].editedSLRObsList[0].staBLH.H,
									   editedObsArc[s_i].editedSLRObsList.size(), 
									   editedObsArc[s_i].mean, 
									   editedObsArc[s_i].rms);
				}
				fprintf(pFile, "## 弧段序号    测站           时间    弧长       波长(μm)      温度(K)       湿度(%%)      压强(mb)        高度角         残差\n");
				int year = editedObsArc[0].editedSLRObsList[0].Ts.year;
				UTC t0 = UTC(year, 1, 1, 0, 0, 0.0);
				for(size_t s_i = 0; s_i < editedObsArc.size(); s_i++)
				{
					for(size_t s_j = 0; s_j < editedObsArc[s_i].editedSLRObsList.size(); s_j++)
					{
						double res = editedObsArc[s_i].editedSLRObsList[s_j].obs
							       - editedObsArc[s_i].editedSLRObsList[s_j].obscorrected_value
								   - editedObsArc[s_i].editedSLRObsList[s_j].r_mean;
						double dt = editedObsArc[s_i].editedSLRObsList[editedObsArc[s_i].editedSLRObsList.size() - 1].Tr - editedObsArc[s_i].editedSLRObsList[0].Tr;
						double day = (editedObsArc[s_i].editedSLRObsList[s_j].Ts - t0) / 86400.0 + 1; //与DOY对应，从1开始	
						fprintf(pFile, "   %-12d%4d%15.8f%8.2f%14.5f%14.1f%14.1f%14.1f%14.1f%14.4f\n", 
							                                                                s_i + 1, 
							                                                                editedObsArc[s_i].id,
																							day, 
																							dt / 60.0,
																							editedObsArc[s_i].editedSLRObsList[s_j].wavelength,
																							editedObsArc[s_i].editedSLRObsList[s_j].temperature,
																							editedObsArc[s_i].editedSLRObsList[s_j].humidity,
                                                                                            editedObsArc[s_i].editedSLRObsList[s_j].pressure,
																							editedObsArc[s_i].editedSLRObsList[s_j].getStaLosElevation(), 
																							res);
					}
				}
				fclose(pFile);
			}

			// 利用仿真数据更新实测数据，记录保存
			//{
            // editedSLRObsList_Arc editedSLRObsList_Arc[s_jj].obs - editedSLRObsList_Arc[s_jj].obscorrected_value- editedSLRObsList_Arc[s_jj].r_mean
			// editedLine.obs = obsFile_crd.m_data[s_i].crdDataRecordList[s_j].TimeofFlight * SPEED_LIGHT / 2.0;
			// obsFile_crd.write()
			//}
			return true;
		}
	}
}
