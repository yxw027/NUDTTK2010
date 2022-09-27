//////////////////////////////////////////////////////////////////////
// Quaternion.h
// 四元数类Quaternion的声明
//////////////////////////////////////////////////////////////////////
#pragma once
#include <math.h>
#include <string>
#include <structDef.hpp>
#include <Matrix.hpp>
#include "iostream"
using namespace std;
namespace NUDTTK
{
	class Quaternion 
	{	
	public:
			// 构造与析构
		Quaternion();										      // 默认构造函数,构造成单位四元数
		Quaternion(double q1,double q2,double q3,double q4);	  // 参数初始化构造函数
		Quaternion(const Quaternion& other);                      // 拷贝构造函数
		virtual ~Quaternion();	                                  // 析构函数
		
		// 打印
		bool PrintQuaternion();
		
		// 转置
		Quaternion qconj();
		
		// 归一化
		Quaternion qnormlz();
		
		// 四元数乘法
		Quaternion	operator*(const Quaternion& other) const;

		// 四元数数乘（数乘以四元数）  
		Quaternion	operator*(const double factor) const;

		// 四元数加法
		Quaternion	operator+(const Quaternion& other) const;
		
		// 四元数转方向余弦矩阵（DCM)
		Matrix q2DCM(Quaternion q) const;
		
		// 四元数转欧拉角
		EULERANGLE q2EULERANGLE(Quaternion qnb) const;
		
	
	public:
		double q1;       // 标量部分 :cos(thita/2)   
		double q2;       // 矢量部分i:sin(thita/2)*xi
		double q3;       // 矢量部分j:sin(thita/2)*yj
		double q4;       // 矢量部分K:sin(thita/2)*zk
	};
}
