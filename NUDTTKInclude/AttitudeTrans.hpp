#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Matrix.hpp"

//  Copyright 2014, The National University of Defense Technology at ChangSha
using namespace NUDTTK;
namespace NUDTTK
{
	class AttitudeTrans
	{
	public:
		AttitudeTrans(void);
	public:
		~AttitudeTrans(void);
	public:
		static Matrix getAttMatrix(ATT_Q4 Q4);
		static Matrix getAttMatrix(EULERANGLE eulerAngle);
		static Matrix getAttMatrix(POS6D pv_j2000);
		static void AttMatrix2EulerAngle(Matrix matAtt, EULERANGLE& eulerAngle, const char* szSequence = "312",bool unit = true);  //true Ä¬ÈÏ»¡¶È
		static void AttMatrix2Q4(Matrix matAtt, ATT_Q4& q);
		static void J2000_Q4_2_RTN_EulerAngle(ATT_Q4 q_j2000, POS6D pv_j2000, EULERANGLE& eulerAngle_rtn, const char* szSequence = "312");
	};
}
