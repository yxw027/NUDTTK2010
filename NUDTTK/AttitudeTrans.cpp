#include "AttitudeTrans.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	AttitudeTrans::AttitudeTrans(void)
	{
	}

	AttitudeTrans::~AttitudeTrans(void)
	{
	}

	// 子程序名称： getAttMatrix   
	// 功能：根据四元素求姿态旋转矩阵
	// 变量类型：Q4          : 四元素
	//           matAtt      : 姿态矩阵
	// 输入：Q4
	// 输出：matAtt
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2009/05/23
	// 版本时间：
	// 修改记录：
	// 备注： 
	Matrix AttitudeTrans::getAttMatrix(ATT_Q4 Q4)
	{
		/*     惯性坐标系到星体坐标系
			|x|                   |x|
			|y|      =[姿态矩阵] *|y| 
			|z|星体               |z|惯性
		*/
		Matrix matAtt(3, 3);
		double q1 = Q4.q1;
		double q2 = Q4.q2;
		double q3 = Q4.q3;
		double q4 = Q4.q4;
		matAtt.SetElement(0, 0,  q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);
		matAtt.SetElement(0, 1,  2 * (q1 * q2 + q3 * q4));
		matAtt.SetElement(0, 2,  2 * (q1 * q3 - q2 * q4));
		matAtt.SetElement(1, 0,  2 * (q1 * q2 - q3 * q4));
		matAtt.SetElement(1, 1, -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4);
		matAtt.SetElement(1, 2,  2 * (q2 * q3 + q1 * q4));
		matAtt.SetElement(2, 0,  2 * (q1 * q3 + q2 * q4));
		matAtt.SetElement(2, 1,  2 * (q2 * q3 - q1 * q4));
		matAtt.SetElement(2, 2, -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4);
		return matAtt;
	}

	// 子程序名称： getAttMatrix   
	// 功能：根据欧拉角求姿态旋转矩阵
	// 变量类型：eulerAngle  : 欧拉角
	//           matAtt      : 姿态矩阵
	// 输入：eulerAngle
	// 输出：matAtt
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2009/05/23
	// 版本时间：
	// 修改记录：
	// 备注： 
	Matrix AttitudeTrans::getAttMatrix(EULERANGLE eulerAngle)
	{
		Matrix matAtt;
		matAtt.MakeUnitMatrix(3);
		Matrix matX = TimeCoordConvert::rotate(eulerAngle.xRoll,  1, 3);
		Matrix matY = TimeCoordConvert::rotate(eulerAngle.yPitch, 2, 3);
		Matrix matZ = TimeCoordConvert::rotate(eulerAngle.zYaw,   3, 3);
		for(int i = 2; i >= 0; i--)
		{
			switch(eulerAngle.szSequence[i])
			{
			case '1':
				matAtt = matAtt * matX;
				break;
			case '2':
				matAtt = matAtt * matY;
				break;
			case '3':
				matAtt = matAtt * matZ;
				break;
			default:
				break;
			}
		}
		return matAtt;
	}

	// 子程序名称： getAttMatrix   
	// 功能：根据轨道位置、速度计算轨道坐标系对应的转换矩阵, 通过该矩阵
	//       可以方便的将轨道坐标系的坐标矢量转换到对应的绝对坐标系(j2000)
	// 变量类型：pv_j2000    : 位置速度(j2000)
	//           matAtt      : 姿态矩阵
	// 输入：pv_j2000
	// 输出：matAtt
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2009/05/23
	// 版本时间：
	// 修改记录：
	// 备注：
	Matrix AttitudeTrans::getAttMatrix(POS6D pv_j2000)
	{
		POS3D S_Z; // Z 轴指向地心
		S_Z.x = -pv_j2000.x;
		S_Z.y = -pv_j2000.y;
		S_Z.z = -pv_j2000.z;
		POS3D S_X; // X 轴沿速度方向
		S_X.x = pv_j2000.vx;
		S_X.y = pv_j2000.vy;
		S_X.z = pv_j2000.vz;
		POS3D S_Y; // 右手系
		vectorCross(S_Y, S_Z, S_X);
		vectorCross(S_X, S_Y, S_Z);
		S_X = vectorNormal(S_X);
		S_Y = vectorNormal(S_Y);
		S_Z = vectorNormal(S_Z);
		Matrix matAtt(3, 3);
		matAtt.SetElement(0, 0, S_X.x);
		matAtt.SetElement(1, 0, S_X.y);
		matAtt.SetElement(2, 0, S_X.z);
		matAtt.SetElement(0, 1, S_Y.x);
		matAtt.SetElement(1, 1, S_Y.y);
		matAtt.SetElement(2, 1, S_Y.z);
		matAtt.SetElement(0, 2, S_Z.x);
		matAtt.SetElement(1, 2, S_Z.y);
		matAtt.SetElement(2, 2, S_Z.z);
		return matAtt;
	}

	// 子程序名称： AttMatrix2EulerAngle   
	// 功能：根据姿态矩阵计算欧拉角
	// 变量类型：matAtt         : 姿态矩阵
	//           eulerAngle     : 欧拉角
	//           szSequence     : 旋转顺序
	// 输入：matAtt, szSequence
	// 输出：eulerAngle
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2009/05/23
	// 版本时间：
	// 修改记录：
	// 备注：
	void AttitudeTrans::AttMatrix2EulerAngle(Matrix matAtt, EULERANGLE& eulerAngle,const char* szSequence,bool unit)
	{
		if(strcmp(szSequence, "312") == 0)
		{
			strcpy(eulerAngle.szSequence, szSequence);
			eulerAngle.szSequence[3] = '\0';
            // 计算欧拉角
			double sin_x =  matAtt.GetElement(1,2);
			double tan_z = -matAtt.GetElement(1,0) / matAtt.GetElement(1,1);
			double tan_y = -matAtt.GetElement(0,2) / matAtt.GetElement(2,2);
			// 由于相同的三角函数值对应两个角[-PI PI]
			double x[2];
			x[0] = asin(sin_x);
			if(x[0] >= 0)
				x[1] =  PI - x[0];
			else
				x[1] = -PI - x[0];
			double y[2];
			y[0] = atan(tan_y);
			if(y[0] >= 0)
				y[1] = -PI + y[0];
			else
				y[1] =  PI + y[0];
			double z[2];
			z[0] = atan(tan_z);
			if(z[0] >= 0)
				z[1] = -PI + z[0];
			else
				z[1] =  PI + z[0];
			// 最小范数原则, 最大最小准则
			// 固定 z 轴角范围为[-PI/2,PI/2], 解决符号问题
			double max;
			double min = 3.0;
			for(int i = 0; i < 2; i++)
			{
				for(int j = 0; j < 2; j++)
				{
					EULERANGLE eulerAngle_i;
					strcpy(eulerAngle_i.szSequence, eulerAngle.szSequence);
					eulerAngle_i.xRoll  = x[i];
					eulerAngle_i.yPitch = y[j];
					eulerAngle_i.zYaw   = z[0];
					Matrix matAtt_i = getAttMatrix(eulerAngle_i);
					max = (matAtt_i - matAtt).Abs().Max();
					if(min > max)
					{
						min = max;
						if(unit)
						{
							eulerAngle.xRoll  = eulerAngle_i.xRoll;
							eulerAngle.yPitch = eulerAngle_i.yPitch;
							eulerAngle.zYaw   = eulerAngle_i.zYaw;
						}
						else
						{
							eulerAngle.xRoll  = eulerAngle_i.xRoll  * 180/PI;
							eulerAngle.yPitch = eulerAngle_i.yPitch * 180/PI;
							eulerAngle.zYaw   = eulerAngle_i.zYaw   * 180/PI;
					    }
				   }
			    }
		  }
		}
	}

	// 子程序名称： AttMatrix2Q4   
	// 功能：根据姿态矩阵计算四元数
	//       会得到两组四元数, 符号相反?
	// 变量类型：matAtt    : 姿态矩阵
	//           q         : 四元素
	// 输入：matAtt
	// 输出：q
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2010/05/29
	// 版本时间：
	// 修改记录：
	// 备注：
	void AttitudeTrans::AttMatrix2Q4(Matrix matAtt, ATT_Q4& q)
	{
		EULERANGLE eulerAngle;
        AttMatrix2EulerAngle(matAtt, eulerAngle, "312");
		// 312
		ATT_Q4 q1;
		q1.q1 = 0;
		q1.q2 = 0;
		q1.q3 = sin(eulerAngle.zYaw / 2);
		q1.q4 = cos(eulerAngle.zYaw / 2);
		ATT_Q4 q2;
		q2.q1 = sin(eulerAngle.xRoll / 2);
		q2.q2 = 0;
		q2.q3 = 0;
		q2.q4 = cos(eulerAngle.xRoll / 2);
		ATT_Q4 q3;
		q3.q1 = 0;
		q3.q2 = sin(eulerAngle.yPitch / 2);
		q3.q3 = 0;
		q3.q4 = cos(eulerAngle.yPitch / 2);
		q = q1 * q2 * q3;
	}

    // 子程序名称： J2000_Q4_2_RTN_EulerAngle   
	// 功能：根据惯性系下四元素计算轨道系下姿态角
	// 变量类型：q_j2000           : 四元素
	//           pv_j2000          : 轨道位置、速度
	//           eulerAngle_rtn    : 轨道系下的欧拉角
	//           szSequence        : 旋转顺序
	// 输入：q_j2000, pv_j2000, eulerAngle_rtn, szSequence
	// 输出：eulerAngle_rtn
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2009/05/23
	// 版本时间：
	// 修改记录：
	// 备注：调用 AttMatrix2EulerAngle
	void AttitudeTrans::J2000_Q4_2_RTN_EulerAngle(ATT_Q4 q_j2000, POS6D pv_j2000, EULERANGLE& eulerAngle_rtn, const char* szSequence)
	{
		Matrix matAtt_J2000 = getAttMatrix(q_j2000);  // 惯性系->星体系
		Matrix matAtt_RTN   = getAttMatrix(pv_j2000); // 轨道系->惯性系
		matAtt_J2000 = matAtt_J2000 * matAtt_RTN;
		/*
				轨道坐标系(RTN)到星体坐标系

				|x|                                                   |x|
				|y|                =     [matAtt_J2000 * matAtt_RTN] *|y| 
				|z|星体                                               |z|轨道坐标系(RTN)
		*/
		AttMatrix2EulerAngle(matAtt_J2000, eulerAngle_rtn, szSequence);
	}
}