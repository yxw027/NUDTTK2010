#include "Quaternion.hpp"
namespace NUDTTK
{
	Quaternion::Quaternion(void)
	{
		this->q1=1.0;													// 标量部分 :cos(thita/2)   
		this->q2=0.0;													// 矢量部分i:sin(thita/2)*xi
		this->q3=0.0;													// 矢量部分j:sin(thita/2)*yj
		this->q4=0.0;													// 矢量部分K:sin(thita/2)*zk
	}

	Quaternion::~Quaternion(void)
	{
	}

	Quaternion::Quaternion(double q1,double q2,double q3,double q4)	    // 参数初始化构造函数
	{
		this->q1=q1;
		this->q2=q2;
		this->q3=q3;
		this->q4=q4;
	}
		
	Quaternion::Quaternion(const Quaternion& other)                     // 拷贝构造函数
	{
		this->q1=other.q1;
		this->q2=other.q2;
		this->q3=other.q3;
		this->q4=other.q4;
	}

	// 打印四元数
	bool Quaternion::PrintQuaternion()                            
	{
	
		cout<<"q1:"<<this->q1<<endl;
		cout<<"q2:"<<this->q2<<endl;
		cout<<"q3:"<<this->q3<<endl;
		cout<<"q4:"<<this->q4<<endl;
		return 0;
	}
			
	// 转置，标量不变，矢量相反
	Quaternion Quaternion::qconj()	
	{
		Quaternion out;

		out.q1= this->q1;
		out.q2=-this->q2;
		out.q3=-this->q3;
		out.q4=-this->q4;

		return out;
		
	}	
		
	// 归一化	
	Quaternion  Quaternion::qnormlz()
	{
		Quaternion out;

		double norm=sqrt(this->q1*this->q1+this->q2*this->q2+this->q3*this->q3+this->q4*this->q4);
		out.q1=this->q1/norm;
		out.q2=this->q2/norm;
		out.q3=this->q3/norm;
		out.q4=this->q4/norm;

		return out;
	}	

		
	// 四元数乘法	
	Quaternion	Quaternion::operator*(const Quaternion& other) const
	{     
		Quaternion Out;	
	
		Out.q1 = this->q1 * other.q1 - this->q2 * other.q2 - this->q3 * other.q3 - this->q4 * other.q4;	
		Out.q2 = this->q1 * other.q2 + this->q2 * other.q1 + this->q3 * other.q4 - this->q4 * other.q3;
		Out.q3 = this->q1 * other.q3 - this->q2 * other.q4 + this->q3 * other.q1 + this->q4 * other.q2;
		Out.q4 = this->q1 * other.q4 + this->q2 * other.q3 - this->q3 * other.q2 + this->q4 * other.q1;
			
		return Out;
	}
		
		
	// 四元数转方向余弦矩阵（DCM).    Note:b->i!!!;  NUDTTK:i->b;
	Matrix Quaternion::q2DCM(Quaternion q) const
	{
		Matrix matAtt(3, 3);
		double q1 = q.q1;
		double q2 = q.q2;
		double q3 = q.q3;
		double q4 = q.q4;         
		matAtt.SetElement(0, 0,  q1 * q1 +q2 * q2 - q3 * q3 - q4 * q4);
		matAtt.SetElement(0, 1,  2 * (q2 * q3 - q1 * q4));
		matAtt.SetElement(0, 2,  2 * (q2 * q4 + q1 * q3));
		matAtt.SetElement(1, 0,  2 * (q2 * q3 + q1 * q4));
		matAtt.SetElement(1, 1,  q1* q1 - q2 * q2 + q3 * q3 - q4 * q4);
		matAtt.SetElement(1, 2,  2 * (q3 * q4 - q1 * q2));
		matAtt.SetElement(2, 0,  2 * (q2 * q4 - q1 * q3));
		matAtt.SetElement(2, 1,  2 * (q3 * q4 + q1 * q2));
		matAtt.SetElement(2, 2,  q1* q1 - q2 * q2 - q3 * q3 + q4 * q4);
		return matAtt;
	}	
		
	// 四元数转欧拉角
	EULERANGLE Quaternion::q2EULERANGLE(Quaternion qnb) const
	{
		
		EULERANGLE out;
    
		double q11 = qnb.q1*qnb.q1, q12 = qnb.q1*qnb.q2,q13 = qnb.q1*qnb.q3,q14 = qnb.q1*qnb.q4; 
		double q22 = qnb.q2*qnb.q2, q23 = qnb.q2*qnb.q3, q24 = qnb.q2*qnb.q4;     
		double q33 = qnb.q3*qnb.q3, q34 = qnb.q3*qnb.q4;  
		double q44 = qnb.q4*qnb.q4;
		double C12=2*(q23-q14);
		double C22=q11-q22+q33-q44;
		double C31=2*(q24-q13), C32=2*(q34+q12), C33=q11-q22-q33+q44;

		out.xRoll=asin(C32);
		out.yPitch=atan2(-C31,C33);
		out.zYaw=atan2(-C12,C22);

		return out;
	}
}