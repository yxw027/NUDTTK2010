#include "TQISLObsSimu.hpp"

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
		bool TQSatISLInfo::getEphemeris(UTC t, TimePosVel& satOrb, int nLagrange)
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

		TQISLObsSimu::TQISLObsSimu(void)
		{
		}

		TQISLObsSimu::~TQISLObsSimu(void)
		{
		}
		
		bool TQISLObsSimu::simuISLObsFile(TQISLObsFile &obsFile, UTC t0, UTC t1, double h)
		{
			srand((unsigned)time(NULL));
			UTC t = t0;
			obsFile.m_data.clear();
			while(t - t1 <= 0.0)
			{
				TimePosVel satOrbA,satOrbB;
				if(!m_satInfoA.getEphemeris(t,satOrbA)||!m_satInfoB.getEphemeris(t,satOrbB))// 插值失败
				{
					t = t + h;
					continue;
				}
				double biasedRange = sqrt(pow(satOrbA.pos.x - satOrbB.pos.x, 2) 
					                    + pow(satOrbA.pos.y - satOrbB.pos.y, 2) 
					                    + pow(satOrbA.pos.z - satOrbB.pos.z, 2));
				TQISLObsLine obsLine;
				obsLine.t        = t;
				obsLine.satNameA = m_satInfoA.satName;
				obsLine.satNameB = m_satInfoB.satName;
				obsLine.range    = biasedRange;  // 真值
				// 伪码
				double cBasError = 0.0;
				double cError    = 0.0;
				// 添加伪码误差：系统误差 + 随机误差？
				if(m_simuDefine.on_ISLCodeSysBias) // 伪码系统误差
				{
					cBasError = 1;
				}
				if(m_simuDefine.on_ISLCodeRandnNoise) // 伪码随机误差
				{
					cError = RandNormal(0,1);
				}
				obsLine.ISL_Code = biasedRange + cBasError + cError;
				// 添加相位误差
				// 相位测距？？？ 相位测距的误差是什么？和模糊度有关怎么体现？
				double initPhase = 0.0;  // 初始相位值
				double pError = 0.0;     // 随机误差
				if(m_simuDefine.on_ISLPhaseRandnNoise)
				{
					pError = RandNormal(0,0.001);
				}
				obsLine.ISL_Phase =  biasedRange + pError + initPhase * m_simuDefine.WAVELENGTH;	
				obsFile.m_data.push_back(obsLine);
				t = t + h;
			}
			return true;
		}
	}
}
