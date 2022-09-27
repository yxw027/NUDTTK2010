#pragma once
namespace NUDTTK
{
	namespace Math
	{
		double Round(double x); 
		double Factorial(int n);	
		void   Sort(double x[], int n, bool bAscend = true);
		void   Sort(double x[], int sequence[], int n, bool bAscend = true);
		void   LegendreFunc(long double** P, int n, double u);
		void   LegendreFuncDerivative(long double** P, long double** DP, int n, double u);
		double Combination(int n, int k);
		void   InterploationLagrange(double xa[],double ya[],int n,double x,double& y);
		void   InterploationLagrange(double xa[],double ya[],int n,double x,double& y,double& dy);
		double Median(double x[],int n);
		double Mad(double x[],int n);
		bool   PolyFit(double x[], double y[], int n, double y_fit[], int m = 3);
		bool   VandrakFilter(double x[] ,double y[] ,double w[],int n,double eps,double y_fit[]);
		bool   KinematicRobustVandrakFilter(double x[] ,double y[] ,double w[],int n,double dEPS,double y_fit[], double threshold_max,double threshold_min,int nwidth, double factor = 3);		// 鲁棒计算均方根
		double RobustStatRms(double x[], int n, double factor = 6);
		double RobustStatRms(double x[], double marker[], int n, double factor = 6);
		bool   RobustStatMean(double x[],double w[],int n,double& dMean,double& dVar, double factor = 6);
		bool   RobustPolyFit(double x[], double y[], double w[], int n, double y_fit[], int m = 3);
		double RandNormal(double mu = 0, double sigma = 1.0);
	}
}