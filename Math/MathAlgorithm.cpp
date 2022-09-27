#include "MathAlgorithm.hpp"
#include <windows.h>
#include <math.h>
#include "Matrix.hpp"
#include "constDef.hpp"

namespace NUDTTK
{
	namespace Math
	{
		// 子程序名称： Round  
		// 功能：四舍五入函数
		// 变量类型：x           : 待四舍五入的数 
		//           floor_x   　: 四舍五入结果
		// 输入：x
		// 输出：floor_x
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/10/10
		// 版本时间：2007/10/10
		// 修改记录：
		// 备注： 
		double Round(double x)
		{
			double floor_x = floor(x);
			double y = x - floor_x;
			if( y > 0.5 )
				return floor_x + 1.0;
			else
				return floor_x;
		}

		// 子程序名称： Factorial  
		// 功能：利用函数递归计算阶乘
		// 变量类型：n  : n 的阶乘
		// 输入：n
		// 输出：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/11/30
		// 版本时间：
		// 修改记录：
		// 备注： 
		double Factorial(int n)
		{
			if( n == 0 || n == 1 )
				return 1.0;
			else 
				return n * Factorial(n - 1);
		}
		//   子程序名称： sort   
		//   作用：数据排序，采用冒泡法(升序)
		//   类型：x          :  观测数据序列
		//         n          :  观测数据个数
		//   输入：x，n
		//   输出：x
		//   其它：
		//   语言：C++
		//   版本号：2007.11.13
		//   生成者：谷德峰
		//   修改者：
		void Sort(double x[], int n, bool bAscend)
		{
			for(int i = 0; i < n; i++)
			{
				for(int j = i + 1; j < n; j++)
				{
					if(bAscend)
					{
						if(x[j] < x[i])
						{
							double temp = x[i];
							x[i] = x[j];
							x[j] = temp;
						}
					}
					else
					{
						if(x[j] > x[i])
						{
							double temp = x[i];
							x[i] = x[j];
							x[j] = temp;
						}
					}
				}
			}
		}

		//   子程序名称： sort   
		//   作用：数据排序，采用冒泡法(升序)
		//   类型：x          :  观测数据序列
		//         n          :  观测数据个数
		//   输入：x，n
		//   输出：x
		//   其它：
		//   语言：C++
		//   版本号：2007.11.13
		//   生成者：谷德峰
		//   修改者：
		void Sort(double x[], int sequence[], int n, bool bAscend)
		{
			for(int i = 0; i < n; i++)
			{
				sequence[i] = i;
			}
			for(int i = 0; i < n; i++)
			{
				for(int j = i + 1; j < n; j++)
				{
					if(bAscend)
					{
						if(x[j] < x[i])
						{
							double temp = x[i];
							x[i] = x[j];
							x[j] = temp;
							int    pos  = sequence[i];
							sequence[i] = sequence[j];
							sequence[j] = pos;
						}
					}
					else
					{
						if(x[j] > x[i])
						{
							double temp = x[i];
							x[i] = x[j];
							x[j] = temp;
                            int    pos  = sequence[i];
							sequence[i] = sequence[j];
							sequence[j] = pos;
						}
					}
				}
			}
		}

		// 子程序名称： LegendreFunc  
		// 功能：利用递推形式计算归一化的勒让德函数值
		// 变量类型：P        : 勒让德函数值 [n+1,n+1]
		//           n        : 阶数, n >= 2 
		//           u        : u = sin(fai), [-1,1]
		// 输入：n, u
		// 输出：P
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/11/23
		// 版本时间：
		// 修改记录：
		// 备注： 参考刘林的《航天器轨道理论》和李济生的《人造卫星精密轨道确定》
		void LegendreFunc(long double** P, int n, double u)
		{
			double cosu = sqrt(1 - u * u);
			for(int i = 0; i <= n; i++)
			{
				memset(P[i], 0, sizeof(long double) * (n + 1));
			}
			// 计算带谐项
			P[0][0] = 1;
			P[1][0] = sqrt(3.0) * u;
			for(int i = 2; i <= n; i++)
				P[i][0] = (sqrt(4.0 * i * i - 1.0) / i) * u * P[i-1][0] -
						  ((i - 1.0)/i) * sqrt((2 * i + 1.0)/(2 * i - 3.0)) * P[i-2][0];
			// 计算扇谐项，对角线上的元素
			P[1][1] = sqrt(3.0) * cosu;
			for(int i = 2; i <= n; i++)
				P[i][i] = sqrt((2.0 * i + 1.0) / (2.0 * i)) * cosu * P[i-1][i-1];
			for(int i = 2; i <= n; i++)
				P[i][i-1] = sqrt(2.0 * i + 1.0) * u * P[i-1][i-1];
			// 计算田谐项
			for(int i = 3; i <= n; i++)
			{
				for(int j = 1; j <= i - 2; j++)
				{
					P[i][j] = sqrt((4.0 * i * i - 1.0)/(i * i - j * j)) * u * P[i-1][j] -
							  sqrt(((2 * i + 1.0)*(i - 1.0 + j)*(i - 1.0 - j))/((2 * i - 3.0)*(i + j)*(i - j))) * P[i-2][j];
				}
			}
		}

		// 子程序名称： LegendreFuncDerivative  
		// 功能：利用递推形式计算归一化的勒让德函数值
		// 变量类型：P        : 勒让德函数值       [n+1,n+1]
		//           DP       : 勒让德函数的导数值 [n+1,n+1]
		//           n        : 阶数, n >= 2 
		//           u        : u = sin(fai), [-1,1]
		// 输入：n, u
		// 输出：P, DP
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/11/23
		// 版本时间：
		// 修改记录：
		// 备注： 参考李济生的《人造卫星精密轨道确定》P103, 调用 LegendreFunc
		void LegendreFuncDerivative(long double** P, long double** DP,int n, double u)
		{
			// 首先计算勒让德函数值 P
			LegendreFunc(P, n, u);
			// 递推计算勒让德函数的导数值 DP
			double cosu = sqrt(1.0 - u * u);
			for(int i = 0; i <= n; i++)
			{
				memset(DP[i], 0, sizeof(long double) * (n + 1));
			}
			// 计算带谐项
			DP[0][0] = 0;
			DP[1][0] = sqrt(3.0) * cosu;
			for(int i = 2; i <= n; i++)
				DP[i][0] = (sqrt(4 * i * i - 1.0) / i) * (u * DP[i-1][0] + cosu * P[i-1][0]) -
						   ((i - 1.0) / i) * sqrt((2.0 * i + 1.0) / (2.0 * i - 3.0)) * DP[i-2][0];
			// 计算扇谐项，对角线上的元素
			DP[1][1] = -sqrt(3.0) * u;
			for(int i = 2; i <= n; i++)
				DP[i][i] = sqrt((2.0 * i + 1.0) / (2.0 * i)) * ( -u * P[i-1][i-1] + cosu * DP[i-1][i-1]);
			// 计算田谐项
			for(int i = 2; i <= n; i++)
				DP[i][i-1] = sqrt(2.0 * i + 1.0) * (u * DP[i-1][i-1] + cosu * P[i-1][i-1]);
			for(int i = 3; i <= n; i++)
				for(int j = 1; j <= i - 2; j++)
					DP[i][j] = sqrt((4.0 * i * i - 1.0)/(i * i - j * j)) * ( u * DP[i-1][j] + cosu * P[i-1][j]) -
							   sqrt(((2 * i + 1.0)*(i - 1.0 + j)*(i - 1.0 - j))/((2 * i - 3.0)*(i + j)*(i - j))) * DP[i-2][j];
		}

		// 子程序名称： Combination  
		// 功能：计算组合数
		// 变量类型：n  : 总数
		//           k  : n choose k
		// 输入：n, u
		// 输出：P, DP
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/11/30
		// 版本时间：
		// 修改记录：
		// 备注： 调用 Factorial
		double Combination(int n, int k)
		{
			return Factorial(n) / (Factorial(n-k) * Factorial(k));
		}

		// 子程序名称： InterploationLagrange  
		// 功能：Lagrange插值
		// 变量类型：xa  :  已知点横坐标
		//           ya  :  已知点纵坐标
		//           n   :  已知点个数，n-1阶
		//           x   :  插值点横坐标
		//           y   :  插值点纵坐标
		//           dy  :  插值点速度
		// 输入：xa, ya, n, x
		// 输出：y，dy
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/1/23
		// 版本时间：
		// 修改记录：
		// 备注： 
		void InterploationLagrange(double xa[],double ya[],int n,double x,double& y,double& dy)
		{
			//判断插值点是否正好落在已知点上
			int nFlag=-1;
			for(int k=0;k<n;k++)
			{
				if(fabs(x-xa[k]) < 1.0E-16)
				{
					nFlag=k;//待插值点正好落在第k个基准点
					break;
				}
			}
			y=0;dy=0;
			//如果插值点未落在已知点上，此时导数计算过程可简化
			if(nFlag==-1)
			{
				for(int k=0;k<n;k++)
				{
					double L=1;//第k项插值多项式
					for(int j=0;j<n;j++)
					{
						if(k!=j)
							L=L*(x-xa[j])/(xa[k]-xa[j]);
					}
					//累加 y
					y=y+ya[k]*L;
					double dL=0;
					for(int j=0;j<n;j++)
					{
						if(k!=j)
							dL=dL+L/(x-xa[j]);//求导数时，每项扣除一个乘子
					}
					dy=dy+ya[k]*dL;
				}
			}
			else//插值点正好落在已知点上
			{
				for(int k=0;k<n;k++)
				{
					double L=1.0,dFenmu=1.0;
					for(int j=0;j<n;j++)
					{
						if(k!=j)
						{
							L=L*(x-xa[j])/(xa[k]-xa[j]);
							dFenmu=dFenmu*(xa[k]-xa[j]);
						}
					}
					//累加 y
					y=y+ya[k]*L;
					if(nFlag==k)////插值点正好落在第k个已知点上
					{
						double dL=0;
						for(int j=0;j<n;j++)
						{
							if(k!=j)
							{
								double dFenzi=1;
								for(int i=0;i<n;i++)
								{
									if(k!=i&&j!=i)
										dFenzi=dFenzi*(x-xa[i]);//分子连乘
								}
								dL=dL+dFenzi/dFenmu;//各项偏导数累加
							}
						}
						dy=dy+ya[k]*dL;
					}
					else//%插值点未落在第k个已知点上，此时只有一项非0
					{
						double dFenzi=1;
						for(int j=0;j<n;j++)
						{
							if(k!=j&&nFlag!=j)
								dFenzi=dFenzi*(x-xa[j]);//分子连乘
						}
						double dL=dFenzi/dFenmu;
						dy=dy+ya[k]*dL;
					}
				}
			}
		}

		// 子程序名称： InterploationLagrange  
		// 功能：Lagrange插值
		// 变量类型：xa  :  已知点横坐标
		//           ya  :  已知点纵坐标
		//           n   :  已知点个数，n-1阶
		//           x   :  插值点横坐标
		//           y   :  插值点纵坐标
		// 输入：xa, ya, n, x 
		// 输出：y
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/1/21
		// 版本时间：
		// 修改记录：
		// 备注：
		void InterploationLagrange(double xa[],double ya[],int n,double x,double& y)
		{
			double dy = 0;
			InterploationLagrange(xa, ya, n, x, y, dy);
		}

		// 子程序名称： Median   
		// 功能：获得序列的中位数
		// 变量类型：x                :  观测数据序列
		//           n                :  观测数据个数
		// 输入：x,n
		// 输出：
		// 其它：
		// 语言：C++
		// 生成者：谷德峰
		// 创建时间：2007/5/11
		// 版本时间：
		// 修改记录：
		// 备注：
		double Median(double x[],int n)
		{
			int nResidual = n - (n/2)*2;
			int nMedian = n/2 + 1;
			// 利用选择排序法从小到大进行排序--只排nMedian个
			for(int i=0;i<nMedian;i++)
			{
				for(int j=i+1;j<n;j++)
				{
					if(x[j]<x[i])
					{
						double temp = x[i];
						x[i] = x[j];
						x[j] = temp;
					}
				}
			}
			// n奇数
			if( nResidual == 1) // nMedian >= 1
				return x[nMedian-1];
			// n偶数
			else // nMedian >= 2
				return (x[nMedian-2] + x[nMedian-1])/2;
		}

		// 子程序名称： Mad   
		// 功能：Median Absolute Deviation 方法, 常用于robust估计中
		// 变量类型：x                :  观测数据序列
		//           n                :  观测数据个数
		// 输入：x,n
		// 输出：
		// 其它：
		// 语言：C++
		// 生成者：谷德峰
		// 创建时间：2007/5/11
		// 版本时间：
		// 修改记录：
		// 备注：
		double Mad(double x[],int n)
		{
			double* pAbsolute = new double [n];
			for(int i = 0; i < n; i++)
				pAbsolute[i] = fabs(x[i]);
			double dMAD = Median(pAbsolute,n)/0.6745;
			delete pAbsolute;
			return dMAD;
		}

		// 子程序名称： PolyFit   
		// 功能：多项式最优平滑
		// 变量类型：x                :  横坐标 x[i] 的值域为 [-1,1]
		//           y                :  纵坐标
		//           n                :  待平滑数据个数
		//           y_fit            :  平滑输出值
		//           m                :  多项式阶数, 2 =< m <= n, 默认取3
		// 输入：x, y, w, n, eps
		// 输出：y_fit
		// 其它：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/11/18
		// 版本时间：
		// 修改记录：
		// 备注：
		bool PolyFit(double x[], double y[], int n, double y_fit[], int m)
		{
			if(m < 1 || m > n)
				return false;
			Matrix matC(n, m);
			Matrix matY(n, 1);
			double *xx = new double [n];
			for(int i = 0; i < n; i++)
				xx[i] = x[i] - x[0];
			for(int i = 0; i < n; i++)
			{
				matY.SetElement(i, 0, y[i]);
				matC.SetElement(i, 0, 1.0);
				for(int j = 1; j < m; j++)
					matC.SetElement(i, j, pow(xx[i], j));
			}
			delete xx;
			Matrix matS     = (matC.Transpose() * matC).Inv() * matC.Transpose() * matY;
			Matrix matY_Fit = matC * matS;
			for(int i = 0; i < n; i++)
				y_fit[i] = matY_Fit.GetElement(i,0);
			return true;
		}

		// 子程序名称： RobustPolyFit   
		// 功能：鲁棒多项式平滑
		// 变量类型：x                :  横坐标
		//           y                :  纵坐标
		//           n                :  待平滑数据个数
		//           w                :  编辑后的权值
		//           y_fit            :  平滑输出值
		//           m                :  多项式阶数, 2 =< m <= n, 默认取3
		// 输入：x, y, n,  m
		// 输出：y_fit, w
		// 其它：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2009/11/18
		// 版本时间：
		// 修改记录：
		// 备注：
		bool RobustPolyFit(double x[], double y[], double w[], int n, double y_fit[], int m)
		{
			double* w_new = new double [n];
			for(int i = 0; i < n; i++)
			{
				w[i] = 1.0;
				w_new[i] = 1.0;
			}

			if(m < 1 || m > n)
			{
				delete w_new;
				return false;
			}

			int nLoop = 0;
			int nLoop_max = 6; // 设置一个迭代次数阈值，避免迭代在临界处震荡，无法收敛
			while(1)
			{
				nLoop ++;
				Matrix matC(n, m);
				Matrix matY(n, 1);
				double *xx = new double [n];
				for(int i = 0; i < n; i++)
					xx[i] = x[i] - x[0];
				for(int i = 0; i < n; i++)
				{
					matY.SetElement(i, 0, w[i] * y[i]);
					matC.SetElement(i, 0, w[i] * 1.0);
					for(int j = 1; j < m; j++)
						matC.SetElement(i, j, w[i] * pow(xx[i], j));
				}
				delete xx;
				Matrix matS = (matC.Transpose() * matC).Inv() * matC.Transpose() * matY;
				Matrix matY_Fit = matC * matS;
				// 计算均方根
				double rms = 0;
				int kk = 0;
				for(int i = 0; i < n; i++)
				{
					y_fit[i] = matY_Fit.GetElement(i,0);
					if(w[i] == 1.0)
					{
						kk++;
						rms += pow(y[i] - y_fit[i], 2);
					}
				}
				rms = sqrt(rms / kk);
				bool bEqual = true;
				for(int i = 0; i < n; i++)
				{   
					if(fabs(y[i] - y_fit[i]) >= 3.0 * rms)
						w_new[i] = 0;
					else
						w_new[i] = 1;
					bEqual = bEqual & (w_new[i] == w[i]);
				}
				if(bEqual || nLoop > nLoop_max)
				{
					break;
				}
				else // 更新观测权值
					memcpy(w, w_new, sizeof(double) * n);
			}
			delete w_new;
			return true;
		}

		// 子程序名称： VandrakFilter   
		// 功能：VandrakFilter 平滑拟合
		// 变量类型：x      :  观测数据横坐标
		//           y      :  观测数据纵坐标
		//           w      :  观测权，默认等权
		//           n      :  观测数据个数
		//           eps    :  平滑因子
		//           y_fit  :  vandrak平滑拟合的输出值
		// 输入：x, y, w, n, eps
		// 输出：y_fit
		// 其它：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2007/1/21
		// 版本时间：2012/4/13
		// 修改记录：2012/04/13, 由谷德峰进行修改, 优化了7对角矩阵的存储
		// 备注：
		bool VandrakFilter(double x[] ,double y[] ,double w[],int n,double eps,double y_fit[])
		{
			// Vandrak拟合要求数据个数至少为4个
			if(n < 4)
				return false;
			eps = eps * (x[n-2]-x[1]) / (n-3);
			Matrix matABCD(4,n-3);
			for(int i = 0; i < n - 3;i++)
			{
				double a = 6*sqrt(x[i+2]-x[i+1])/((x[i]  -x[i+1])*(x[i]  -x[i+2]) *(x[i]  -x[i+3]));
				double b = 6*sqrt(x[i+2]-x[i+1])/((x[i+1]-x[i])  *(x[i+1]-x[i+2]) *(x[i+1]-x[i+3]));
				double c = 6*sqrt(x[i+2]-x[i+1])/((x[i+2]-x[i])  *(x[i+2]-x[i+1]) *(x[i+2]-x[i+3]));
				double d = 6*sqrt(x[i+2]-x[i+1])/((x[i+3]-x[i])  *(x[i+3]-x[i+1]) *(x[i+3]-x[i+2]));
				matABCD.SetElement(0,i,a);
				matABCD.SetElement(1,i,b);
				matABCD.SetElement(2,i,c);
				matABCD.SetElement(3,i,d);
			}
			Matrix matY(n,1);
			for(int i = 0; i < n; i++)
				matY.SetElement(i,0,y[i] * w[i] * eps); /*Y = (X(2,:).*P)'.*e;*/
			//Matrix matA(n,n);
			Matrix matA(7,n); 
			/*       七对角矩阵, 采用7行结构来记录, 优化后的矩阵行号 = 3 - (i - j), 同时保证列结构不变
			   matA(0,:)------ i - j =  3, i = 3:n-1, j = 0:n-4， n-3个元素
			   matA(1,:)------ i - j =  2, i = 2:n-1, j = 0:n-3， n-2个元素
			   matA(2,:)------ i - j =  1, i = 1:n-1, j = 0:n-2， n-1个元素
			   matA(3,:)------ i - j =  0, i = 0:n-1, j = 0:n-1， n  个元素
			   matA(4,:)------ i - j = -1, i = 0:n-2, j = 1:n-1， n-1个元素
			   matA(5,:)------ i - j = -2, i = 0:n-3, j = 2:n-1， n-2个元素
			   matA(6,:)------ i - j = -3, i = 0:n-4, j = 3:n-1， n-3个元素
			   
			  |3  4  5  6  *  *  *|
			  |2  3  4  5  6  *  *|
			  |1  2  3  4  5  6  *|
			  |0  1  2  3  4  5  6|
              |*  0  1  2  3  4  5|
			  |*  *  0  1  2  3  4|
			  |*  *  *  0  1  2  3|
			*/

			for(int j = 0; j < n; j++)
			{
				// 对角线元素--A_0j
				//matA.SetElement(j, j, w[j] * eps);
				matA.SetElement(3, j, w[j] * eps);
				for(int i = j; i >= j - 3;i--)
				{
					int nCount = j-i;                    // 分辨a b c d--0 1 2 3
					if( i>=0 && i <= n - 4 )             // 确保[0，n-4]
					{
						//matA.SetElement(j,j,matA.GetElement(j,j)+pow(matABCD.GetElement(nCount,i),2));
						matA.SetElement(3,j,matA.GetElement(3,j)+pow(matABCD.GetElement(nCount,i),2));
					}
				}
				// A_1j--> matA(4,:)
				for(int i = j; i >= j - 2; i--)
				{
					int nCount = j - i;
					if( i >= 0 && i <= n-4 )             // 确保[0，n-4]
					{
						//matA.SetElement(j,j+1,matA.GetElement(j,j+1)+matABCD.GetElement(nCount,i)*matABCD.GetElement(nCount+1,i));
						matA.SetElement(4,j+1,matA.GetElement(4,j+1)+matABCD.GetElement(nCount,i)*matABCD.GetElement(nCount+1,i));
					}
				}
				// A_2j--> matA(5,:)
				for(int i = j; i >= j - 1; i--)
				{
					int nCount = j-i;
					if( i >= 0 && i <= n-4 )             // 确保[0，n-4]
					{
						//matA.SetElement(j,j+2,matA.GetElement(j,j+2)+matABCD.GetElement(nCount,i)*matABCD.GetElement(nCount+2,i));
						matA.SetElement(5,j+2,matA.GetElement(5,j+2)+matABCD.GetElement(nCount,i)*matABCD.GetElement(nCount+2,i));
					}
				}
				// A_3j--> matA(6,:)
				if( j <= n - 4 )
				{
					//matA.SetElement(j,j+3,matABCD.GetElement(0,j)*matABCD.GetElement(3,j));
					matA.SetElement(6,j+3,matABCD.GetElement(0,j)*matABCD.GetElement(3,j));
				}
				// A_-1j--> matA(2,:)
				for(int i = j - 1; i >= j - 3; i--)
				{
					int nCount = j-1-i;
					if( i >= 0 && i <= n-4 )             // 确保[0，n-4]
					{
						//matA.SetElement(j,j-1,matA.GetElement(j,j-1)+matABCD.GetElement(nCount,i)*matABCD.GetElement(nCount+1,i));
						matA.SetElement(2,j-1,matA.GetElement(2,j-1)+matABCD.GetElement(nCount,i)*matABCD.GetElement(nCount+1,i));
					}
				}
				// A_-2j--> matA(1,:)
				for(int i = j - 2; i >= j - 3; i--)
				{
					int nCount = j-2-i;
					if( i >= 0 && i <= n-4 )             // 确保[0，n-4]
					{
						//matA.SetElement(j,j-2,matA.GetElement(j,j-2)+matABCD.GetElement(nCount,i)*matABCD.GetElement(nCount+2,i));
						matA.SetElement(1,j-2,matA.GetElement(1,j-2)+matABCD.GetElement(nCount,i)*matABCD.GetElement(nCount+2,i));
					}
				}
				// A_-3j--> matA(0,:)
				if( j-3 >= 0 && j-3 <= n-4)
				{
					//matA.SetElement(j,j-3,matABCD.GetElement(0,j-3)*matABCD.GetElement(3,j-3));
					matA.SetElement(0,j-3,matABCD.GetElement(0,j-3)*matABCD.GetElement(3,j-3));
				}
			}

			// 当点数过多时，矩阵的维数会过高，速度会很慢
			//CMatrix matY_fit = matA.Inv()*matY;  // 此部分可通过追赶法进一步优化
			//for(i=0;i<n;i++)
			//	y_fit[i] = matY_fit.GetElement(i,0);

			// 20071013, 将 Vondrak 拟合的求解方法, 改为追赶法
			/* 追赶法求解7对角带型线性方程组 */
			for(int i = 0; i < n - 1; i++ )
			{// 根据对角线元素进行单位化
				//double element_ii = matA.GetElement(i,i);
				double element_ii = matA.GetElement(3,i);
				matY.SetElement(i, 0, matY.GetElement(i, 0) / element_ii);
				for(int k = i; k < n ; k++ )
				{
					//matA.SetElement(i, k, matA.GetElement(i, k) / element_ii);
					if(3 - (i-k) <= 6)
					{
						matA.SetElement(3 - (i-k), k, matA.GetElement(3 - (i-k), k) / element_ii);
					}
				}
				
				int N = ( i + 3 < n - 1 ) ? i + 3 : n - 1;
				for(int j = i + 1; j <= N; j++)
				{// 逐行进行消元，最多只进行3行 
					//double element_ji = matA.GetElement(j, i);
					double element_ji = matA.GetElement(3 - (j-i), i);
					matY.SetElement(j, 0, matY.GetElement(j, 0) - element_ji * matY.GetElement(i, 0));
					for(int k = i; k < n ; k++ )
					{
						//matA.SetElement(j, k, matA.GetElement(j, k) - element_ji * matA.GetElement(i, k));
						if(3 - (i-k) <= 6)
						{ 
							matA.SetElement(3 - (j-k), k, matA.GetElement(3 - (j-k), k) - element_ji * matA.GetElement(3 - (i-k), k));
						}
					}
				}
			}
			// 最后一行
			//matY.SetElement(n-1, 0, matY.GetElement(n-1, 0) / matA.GetElement(n-1, n-1));
			//matA.SetElement(n-1, n-1, 1);
			matY.SetElement(n-1, 0, matY.GetElement(n-1, 0) / matA.GetElement(3, n-1));
			matA.SetElement(3, n-1, 1);
			// 追赶法求解
			y_fit[n-1] = matY.GetElement(n-1, 0);
			//y_fit[n-2] = matY.GetElement(n-2, 0) - matA.GetElement(n-2, n-1) * y_fit[n-1];
			y_fit[n-2] = matY.GetElement(n-2, 0) - matA.GetElement(4, n-1) * y_fit[n-1];
			//y_fit[n-3] = matY.GetElement(n-3, 0) - matA.GetElement(n-3, n-2) * y_fit[n-2] - matA.GetElement(n-3, n-1) * y_fit[n-1];
			y_fit[n-3] = matY.GetElement(n-3, 0) - matA.GetElement(4, n-2) * y_fit[n-2] - matA.GetElement(5, n-1) * y_fit[n-1];
			//y_fit[n-4] = matY.GetElement(n-4, 0) - matA.GetElement(n-4, n-3) * y_fit[n-3] - matA.GetElement(n-4, n-2) * y_fit[n-2] - matA.GetElement(n-4, n-1) * y_fit[n-1];
			y_fit[n-4] = matY.GetElement(n-4, 0) - matA.GetElement(4, n-3) * y_fit[n-3] - matA.GetElement(5, n-2) * y_fit[n-2] - matA.GetElement(6, n-1) * y_fit[n-1];
			for(int i = n - 5; i >= 0; i--)
			{
				//y_fit[i] = matY.GetElement(i, 0) - matA.GetElement(i, i+1) * y_fit[i+1] - matA.GetElement(i, i+2) * y_fit[i+2] - matA.GetElement(i, i+3) * y_fit[i+3];
				y_fit[i] = matY.GetElement(i, 0) - matA.GetElement(4, i+1) * y_fit[i+1] - matA.GetElement(5, i+2) * y_fit[i+2] - matA.GetElement(6, i+3) * y_fit[i+3];
			}
			/* 7对角带型线性方程组求解完毕!*/
			return true;
		}

		// 子程序名称： KinematicRobustVandrakFilter   
		// 功能：动态鲁棒 vandrak 拟合（适合在观测噪声动态变化的场合）
		// 变量类型：x                :  观测数据横坐标
		//           y                :  观测数据纵坐标
		//           w                :  观测权0或1, w中可包含野值信息
		//           n                :  观测数据个数
		//           eps              :  平滑因子
		//           y_fit            :  Vandrak平滑拟合的输出值
		//           threshold_max    :  标准差的阈值
		//           threshold_min    :  标准差的阈值
		//           nwidth           :  动态噪声的窗口宽度
		//           factor           :  鲁棒因子, 默认为 3
		// 输入：x, y, w, n, eps, threshold_mad, nwidth
		// 输出：w, y_fit
		// 其它：
		// 语言：C++
		// 版本号：2007/5/11
		// 生成者：谷德峰
		// 修改者：
		bool KinematicRobustVandrakFilter(double x[] ,double y[] ,double w[],int n,double eps,double y_fit[],double threshold_max,double threshold_min,int nwidth, double factor)
		{
			bool bResult = true;
			double* error_fit = new double [n];
			double* pmad      = new double [n];
			double* w_old     = new double [n];
			double* w_new     = new double [n];
			memcpy(w_old, w, sizeof(double)*n); //w---包含原有的野值标记
			int nLoop = 0;
			int nLoop_max = 6; // 设置一个迭代次数阈值，避免迭代在临界处震荡，无法收敛
			int n_valid = 0;
			while(1)
			{
				// 进行 Vandrak 拟合
				if(!VandrakFilter(x,y,w_old,n,eps,y_fit))
				{
					bResult = false;
					break;
				}
				nLoop ++;
				// 计算拟合残差
				for(int i = 0; i < n; i++)
				{
					error_fit[i] = y[i] - y_fit[i];
				}
				// 根据残差error_fit和nwidth，利用MAD鲁棒方法，计算每一点的标准差
				int nResidual = nwidth - (nwidth/2)*2;
				int nleftwidth,nrightwidth;
				if( nResidual == 1) // 奇数
				{
					nleftwidth  = nwidth/2;
					nrightwidth = nwidth/2;
				}
				else                // 偶数
				{
					nleftwidth  = nwidth/2 - 1;
					nrightwidth = nwidth/2;
				}
				// 计算拟合残差
				if( n > nwidth )
				{   // [ 0, nleftwidth )
					for(int i = 0;i < nleftwidth; i++)
						pmad[i] = Mad( error_fit, nwidth);
					// [ n - nrightwidth, n )
					for(int i = n - nrightwidth; i<n; i++)
						pmad[i] = Mad( error_fit + n - nwidth, nwidth);// 指针error_fit右移 n - nwidth
					// [ nleftwidth, n - nrightwidth )
					for(int i = nleftwidth; i < n - nrightwidth; i++)
						pmad[i] = Mad( error_fit + i - nleftwidth, nwidth);
				}
				else
				{// MAD 方法需要排序，计算时比较耗时，这里进行约束
					double dMAD = 0;
					if(n <= 500)
						dMAD = Mad( error_fit, n);
					else
					{
						dMAD = 0;
						for(int i = 0; i < n; i++)
							dMAD += error_fit[i] * error_fit[i];
						dMAD = sqrt(dMAD / n);
					}
					for(int i = 0; i < n; i++)
						pmad[i] = dMAD;
				}
				// 为了识别“成片”野值，根据先验信息，设置一个合理的threshold_max threshold_min
				for(int i = 0; i < n; i++)
				{
					if(pmad[i]>=threshold_max)  // 避免真正的野值被漏掉
						pmad[i] = threshold_max;
					if(pmad[i]<=threshold_min)
						pmad[i] = threshold_min;// 避免好的观测数据被误判
				}
				bool bEqual = true;
				n_valid = 0;
				for(int i = 0; i < n; i++)
				{// 保留原有 w 的野值标记
					if(fabs(error_fit[i]) >= factor * pmad[i] || w[i] == 0)
						w_new[i] = 0;
					else
					{
						w_new[i] = 1;
						n_valid++;
					}
					bEqual = bEqual & (w_new[i]==w_old[i]);
				}
				if(bEqual || nLoop > nLoop_max)
				{
					bResult = true;
					break;
				}
				else // 更新观测权值
					memcpy(w_old, w_new, sizeof(double)*n);
			}
			memcpy(w, w_new, sizeof(double)*n);// 返回观测权值，其中保留原有的野值标记
			delete w_old;
			delete w_new;
			delete pmad;
			delete error_fit;
			if(n_valid >= 4)
				return bResult;
			else
			{// 无法拟合，将y_fit[i]赋值为y[i]
				for(int i = 0; i < n; i++)
					y_fit[i] = y[i];
				return false;
			}
		}

		// 子程序名称： RobustStatRms   
		// 功能：零均值(差分数据)数据的均方根的鲁棒估计
		// 变量类型：x                :  零均值数据序列
		//           marker           :  标记数据是否超过鲁棒阈值, 1 - 野值，0 - 正常
		//           n                :  数据个数
		//           factor           :  鲁棒控制因子
		// 输入：x, n, factor
		// 输出：marker, dVar
		// 其它：
		// 语言：C++
		// 版本号：2007/7/2
		// 生成者：谷德峰
		// 修改者：2008/4/7
		double RobustStatRms(double x[], double marker[], int n, double factor)
		{
			double  dVar = 0;
			for(int i = 0; i < n; i++)
			{
				marker[i] = 0;      // 初始时刻认为所有点均为正常点
				dVar   = dVar + x[i] * x[i];
			}
			dVar = sqrt(dVar/(n-1));
			while(1)
			{
				double* pQ1 = new double [n];
				int    k = 0;
				double s = 0;
				for(int i = 0; i < n; i++)
				{
					if( fabs(x[i]) > factor * dVar )
						pQ1[i] = 1;   // 野值
					else
					{
						pQ1[i] = 0; 
						k++;
						s = s + x[i]*x[i];
					}
				}
				dVar = sqrt( s/(k-1) );
				// 判断pQ1与pQ0
				bool bfind = 0;
				for(int i = 0; i < n; i++)
				{
					if(pQ1[i] != marker[i])
						bfind = 1;      // pQ1!=pQ0
					marker[i] = pQ1[i]; // 令pQ0=pQ1
				}
				delete[] pQ1;
				if( bfind == 0 )     // 当pQ0与pQ1相等，则迭代结束
					break;
				
			}
			return dVar;
		}


		// 子程序名称： RobustStatRms   
		// 功能：零均值(差分数据)数据的均方根的鲁棒估计
		// 变量类型：x                :  零均值数据序列
		//           n                :  数据个数
		//           factor           :  鲁棒控制因子
		// 输入：x, n, factor
		// 输出：dVar
		// 其它：
		// 语言：C++
		// 版本号：2007/7/2
		// 生成者：谷德峰
		// 修改者：2008/4/7
		double   RobustStatRms(double x[], int n, double factor)
		{
			double* marker = new double [n];
			double  dVar = RobustStatRms(x, marker, n, factor);
			delete[]  marker;
			return  dVar;
		}

		// 子程序名称： RobustStatMean   
		// 功能：均值恒定数据的鲁棒估计,返回均值、方差及每个样本点的野值标记
		// 变量类型：x                :  恒定均值数据序列
		//           n                :  数据个数
		// 输入：x, n
		// 输出：dMean
		// 其它：
		// 语言：C++
		// 版本号：2007/8/12
		// 生成者：谷德峰
		// 修改者：
		bool RobustStatMean(double x[],double w[],int n,double& dMean,double& dVar, double factor)
		{
			// 初始化均值和方差
			dMean = 0;
			for(int i = 0; i < n; i++)
			{
				w[i]  = 0;           // 初始时刻认为所有点均为正常点
				dMean = dMean + x[i];
			}
			dMean = dMean / n;
			// 计算方差
			dVar  = 0;
			for(int i = 0; i < n; i++)
				dVar = dVar + pow(x[i] - dMean, 2);
			dVar = sqrt(dVar/(n-1));

			const int    nn_max  = 10; // 迭代的最大次数阈值
			int          nn = 0; 
			while(1)
			{
				nn++;
				if(nn > nn_max)              // 迭代次数控制
					return false;
				double* pw = new double [n]; // 根据残差大小和方差大小关系进行野值判断
				for(int i = 0; i < n; i++)
				{
					if( fabs( x[i]- dMean ) > factor * dVar )
						pw[i] = 1;   // 野值
					else
						pw[i] = 0; 
				}
				// 判断pw与w是否相等
				bool bfind = 0;
				for(int i = 0; i < n; i++)
				{
					if(pw[i] != w[i])
						bfind = 1;   // pw != w
					w[i] = pw[i];    // 更新w
				}
				
				if( bfind == 0 )     // 当pw与w相等，则迭代结束
				{
					delete pw;
					break;
				}
				// 更新均值和方差
				dMean = 0;
				int k = 0;
				for(int i = 0; i < n; i++)
				{
					if( pw[i] == 0 ) // 正常点才计算
					{
						k++;
						dMean = dMean + x[i];
					}
				}
				dMean = dMean / k;
				dVar  = 0;
				for(int i = 0; i < n; i++)
				{
					if( pw[i] == 0 ) // 正常点才计算
					{
						dVar = dVar + pow(x[i] - dMean, 2);
					}
				}
				dVar = sqrt(dVar/(k-1));	
				delete pw;
			}
			return true;
		}

		// 子程序名称： RandNormal   
		// 功能：正态噪声生成
		// 变量类型：mu              : 均值
		//           sigma           : 方差
		// 输入：mu, sigma
		// 输出：x
		// 其它：
		// 语言：C++
		// 版本号：2009/9/18
		// 生成者：涂佳
		// 修改者：
		double RandNormal(double mu, double sigma)
		{
			double x;
			double dRan1;
			double dRan2;
			double value;
			do
			{
				dRan1 = (double)rand() / RAND_MAX;
				dRan2 = (double)rand() / RAND_MAX;
				value = sqrt(2.0) * sqrt(-log(dRan2)) * sin(2 * PI * dRan1);
			}while((dRan1 == 0 || dRan2 == 0) || fabs(value) > 3.0);
			//x= mu + sqrt(2.0) * sigma * sqrt(-log(dRan2)) * sin(2 * PI * dRan1);
			x = mu + sigma * value;
			return x;
		}
	}
}