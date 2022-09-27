#pragma once
#include "Matrix.hpp"

//  Copyright 2012, The National University of Defense Technology at ChangSha

namespace NUDTTK
{
	namespace LAMBDA
	{
		// 由朱书博根据国外 lambda 的 matlab 算法改写完成
		class lambda
		{
		public:
			lambda(void);
		public:
			~lambda(void);
		public:
			static void   ldl(double **Q, double **L, double **D, int n);
			static int    round(double a);
			static void   inv(double **A, int n); 
			static void   matrixMultiply(double **A, double **B, double **C, int n);   
			static void   threeMultiply(double **A, double **B, double **C, int n);
			static void   decorrel(double **Q, double *a, int **Z, double **D, double **L, double *z, int n);  
			static int    sign(double a);
			static double gamma(double x);  
			static double chistart(double **D, double **L, double *a, int ncands, double factor, int n);
			static void   reform(double *Q, int *b, int ncands);  
			static void   lsearch(double *afloat, double **L, double **D, double Chi2, double **afixed, double *sqnorm, int &ierr, int n, int ncands);
			static bool   main(Matrix matA_Float, Matrix matQahat, Matrix& matSqnorm, Matrix& matA_Fixed, int ncands = 2);
		};
	}
}
