//////////////////////////////////////////////////////////////////////
// Matrix.h
//
// 操作矩阵的类 Matrix 的声明接口
//
// 周长发编制,谷德峰改编 2006/11 去掉CString 加入string
//////////////////////////////////////////////////////////////////////
#pragma once
#include <math.h>
#include <windows.h>
#include <string>

using namespace std;
namespace NUDTTK
{
	class Matrix  
	{
		// 公有接口函数
	public:
		// 构造与析构
		Matrix();										// 基础构造函数
		Matrix(int nRows, int nCols);					// 指定行列构造函数
		Matrix(int nRows, int nCols, double value[]);	// 指定数据构造函数
		Matrix(int nSize);								// 方阵构造函数
		Matrix(int nSize, double value[]);				// 指定数据方阵构造函数
		Matrix(const Matrix& other);					// 拷贝构造函数
		BOOL	Init(int nRows, int nCols);				// 初始化矩阵	
		BOOL	MakeUnitMatrix(int nSize);				// 将方阵初始化为单位矩阵
		virtual ~Matrix();								// 析构函数

		// 输入与显示
		// 将矩阵转换为字符串
		string ToString(string sDelim = " ", bool bLineBreak = true);
		// 将矩阵的指定行转换为字符串
		string RowToString(int nRow, string sDelim = " ");
		// 将矩阵的指定列转换为字符串
		string ColToString(int nCol, string sDelim = "\n");

		// 元素与值操作
		BOOL	SetElement(int nRow, int nCol, double value);	// 设置指定元素的值
		double	GetElement(int nRow, int nCol) const;			// 获取指定元素的值
		void    SetData(double value[]);						// 设置矩阵的值
		int		GetNumColumns() const;							// 获取矩阵的列数
		int		GetNumRows() const;								// 获取矩阵的行数
		int     GetRowVector(int nRow, double* pVector) const;	// 获取矩阵的指定行矩阵
		int     GetColVector(int nCol, double* pVector) const;	// 获取矩阵的指定列矩阵
		double* GetData() const;								// 获取矩阵的值

		// 数学操作
		Matrix& operator=(const Matrix& other);
		BOOL operator==(const Matrix& other) const;
		BOOL operator!=(const Matrix& other) const;
		Matrix	operator+(const Matrix& other) const;
		Matrix	operator-(const Matrix& other) const;
		Matrix	operator*(double value) const;
		Matrix	operator/(double value) const;
		Matrix	operator*(const Matrix& other) const;
		// 复矩阵乘法
		BOOL CMul(const Matrix& AR, const Matrix& AI, const Matrix& BR, const Matrix& BI, Matrix& CR, Matrix& CI) const;
		// 矩阵的转置
		Matrix Transpose() const;
		// 矩阵每个元素取绝对值
		Matrix Abs() const;
		// 矩阵求逆
		Matrix Inv() const;
		Matrix Inv_Ssgj() const;
	    Matrix InvertTriAngle() const; // 三对角矩阵的求逆
		// 返回矩阵最大元素值
		double  Max(int& nRow,int& nCol);
		double  Max();
		// 返回矩阵最小元素值
		double  Min(int& nRow,int& nCol);
		double  Min();
		// 三对角矩阵的求逆
		Matrix InvertTriangle() const;

		// 算法
		// 实矩阵求逆的全选主元高斯－约当法
		BOOL InvertGaussJordan();                                               
		// 复矩阵求逆的全选主元高斯－约当法
		BOOL InvertGaussJordan(Matrix& mtxImag);                                 
		// 对称正定矩阵的求逆
		BOOL InvertSsgj();                                              
		// 托伯利兹矩阵求逆的埃兰特方法
		BOOL InvertTrench();                                                    
		// 求行列式值的全选主元高斯消去法
		double DetGauss();                                                              
		// 求矩阵秩的全选主元高斯消去法
		int RankGauss();
		// 对称正定矩阵的乔里斯基分解与行列式的求值
		BOOL DetCholesky(double* dblDet);                                                               
		// 矩阵的三角分解
		BOOL SplitLU(Matrix& mtxL, Matrix& mtxU);                                     
		// 一般实矩阵的QR分解
		BOOL SplitQR(Matrix& mtxQ);                                                      
		// 一般实矩阵的奇异值分解
		BOOL SplitUV(Matrix& mtxU, Matrix& mtxV, double eps = 0.000001);                                       
		// 求广义逆的奇异值分解法
		BOOL GInvertUV(Matrix& mtxAP, Matrix& mtxU, Matrix& mtxV, double eps = 0.000001);
		// 约化对称矩阵为对称三对角阵的豪斯荷尔德变换法
		BOOL MakeSymTri(Matrix& mtxQ, Matrix& mtxT, double dblB[], double dblC[]);
		// 实对称三对角阵的全部特征值与特征向量的计算
		BOOL SymTriEigenv(double dblB[], double dblC[], Matrix& mtxQ, int nMaxIt = 60, double eps = 0.000001);
		// 约化一般实矩阵为赫申伯格矩阵的初等相似变换法
		void MakeHberg();
		// 求赫申伯格矩阵全部特征值的QR方法
		BOOL HBergEigenv(double dblU[], double dblV[], int nMaxIt = 60, double eps = 0.000001);
		// 求实对称矩阵特征值与特征向量的雅可比法
		BOOL JacobiEigenv(double dblEigenValue[], Matrix& mtxEigenVector, int nMaxIt = 60, double eps = 0.000001);
		// 求实对称矩阵特征值与特征向量的雅可比过关法
		BOOL JacobiEigenv2(double dblEigenValue[], Matrix& mtxEigenVector, double eps = 0.000001);

		// 保护性数据成员
	public:
		int	m_nNumColumns;			// 矩阵列数
		int	m_nNumRows;				// 矩阵行数
		double*	m_pData;			// 矩阵数据缓冲区

	private:
		void ppp(double a[], double e[], double s[], double v[], int m, int n);
		void sss(double fg[2], double cs[2]);
	};
}
