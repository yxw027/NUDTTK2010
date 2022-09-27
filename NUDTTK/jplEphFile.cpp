#include "jplEphFile.hpp"
#include <math.h>
#include "constDef.hpp"

namespace NUDTTK
{
	JPLEphFile::JPLEphFile(void)
	{
	}

	JPLEphFile::~JPLEphFile(void)
	{
	}

	// 子程序名称： open   
	// 功能：数据解析 
	// 变量类型：strJPLEphfileName : 观测数据文件路径
	// 输入：strJPLEphfileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/03/27
	// 版本时间：
	// 修改记录：1.增加DE421和DE430文件读取，2021.4.8，王凯
	// 备注： 
	bool JPLEphFile::open(string  strJPLEphfileName)
	{
		FILE * pJPLEphfile = fopen(strJPLEphfileName.c_str(),"r+b");
		if(pJPLEphfile==NULL) 
			return false;
		for (int i=0; i<3 ; i++)
		{
			fread  (TTL[i] ,sizeof(char),LabelSize,pJPLEphfile);
			TTL[i][LabelSize]='\0';
		}
		for (int i=0; i<NConstants ; i++)
		{
			fread(constant_name[i] ,sizeof(char),ConstantNameLength,pJPLEphfile);
			constant_name[i][ConstantNameLength]='\0';
		}

		fread(&startDate       ,sizeof(double),1,pJPLEphfile);
		fread(&endDate         ,sizeof(double),1,pJPLEphfile);
		fread(&daysPerInterval ,sizeof(double),1,pJPLEphfile);

		fread(&ncon, sizeof(long),  1, pJPLEphfile);// 固定常量个数
		fread(&au,   sizeof(double),1, pJPLEphfile);
		fread(&emrat,sizeof(double),1, pJPLEphfile);
		
		fread(coeffInfo ,sizeof(JPLEphCoeffInfo),JPLEph_NItems,pJPLEphfile);// 行星系数信息
		fread(&nutationCoeffInfo ,sizeof(JPLEphCoeffInfo),1,pJPLEphfile);   // 章动系数信息
		fread(&DENum,sizeof(int),1, pJPLEphfile);
		switch (DENum)
		{
		case 200:
			recordSize = DE200RecordSize;
			break;
		case 405:
			recordSize = DE405RecordSize;
			break;
		case 406:
			recordSize = DE406RecordSize;
			break;
		case 421:
			recordSize = DE421RecordSize;//DE421
			break;
		case 430:
			recordSize = DE430RecordSize;//DE430
			break;
		default:
			return false;
		}
		fread(&librationCoeffInfo ,sizeof(JPLEphCoeffInfo),1,pJPLEphfile); // libration系数信息
		// 数据以上共2856个字节
		fseek(pJPLEphfile, recordSize*4, 0);
		// 固定常量
		for (int i=0; i<NConstants; i++)
		{
			fread(&constant_value[i],sizeof(double),1, pJPLEphfile);
			//cout<<constant_name[i]<<"    "<<constant_value[i]<<endl;
		}
		fseek(pJPLEphfile, recordSize*8, 0);
		unsigned int nRecords = (unsigned int) ((endDate - startDate) / daysPerInterval);// 间隔个数
		records.resize(nRecords);
		for (unsigned int i = 0; i < nRecords; i++)
		{
			records[i].coeffs = new double[recordSize / 2]; // 2007-03-28 修改，coeffs 大小改为 recordSize / 2
			fread(records[i].coeffs, sizeof(double), recordSize/2, pJPLEphfile);
			records[i].t0=records[i].coeffs[0];
			records[i].t1=records[i].coeffs[1];
			records[i].count_coeffs = recordSize / 2;
		}
		fclose(pJPLEphfile);
		return true;
	}

	unsigned int JPLEphFile::getDENumber() const
	{
		return DENum;
	}

	double JPLEphFile::getStartDate() const
	{
		return startDate;
	}

	double JPLEphFile::getEndDate() const
	{
		return endDate;
	}

	// 子程序名称： getPlanetPos
	// 功能：计算行星的位置 (J2000，月球是地心系，其它是太阳系，但非日心系) 
	// 变量类型：planet      :行星的标记
	//           t           :时刻，儒略日
	//           P           :位置矢量[3] 千米
	// 输入：planet，t
	// 输出：P
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/03/27
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool JPLEphFile::getPlanetPos(JPLEphItem planet, double t, double P[])
	{
		// 确保时间范围[startDate，endDate]
		if (t < startDate || t > endDate)
			return false;
		int recNo = (int) ((t - startDate) / daysPerInterval);//找到大区间
		// 当t == endDate，确保recNo不越界 
		if (recNo >= int(records.size()))
			recNo = int(records.size() - 1);
		const JPLEphRecord* rec = &records[recNo]; // 将指针指向该大区间的系数数据
		
		if(coeffInfo[planet].nSubinterv >= 1 && coeffInfo[planet].nSubinterv <= 32 && coeffInfo[planet].nCoeffs <= MaxChebyshevCoeffs)
		{
		}
		else
		{
			printf("nSubinterv < 1 or nSubinterv > 32 or nCoeffs > MaxChebyshevCoeffs!\n");
			return false;
		}

		// u是规范化系数[-1, 1]
		// coeffs是Chebyshev多项式系数指针，本程序指针的移动堪称经典
		double u = 0.0;
		double* coeffs = NULL;
		if (coeffInfo[planet].nSubinterv == -1)
		{
			coeffs = rec->coeffs + coeffInfo[planet].offset - 1;// 2008-07-16
			u = 2.0 * (t - rec->t0) / daysPerInterval - 1.0;// 找到大区间所对应的时间rec->t0
		}
		else
		{
			double daysPerSubinterv = daysPerInterval / coeffInfo[planet].nSubinterv;
			int    subinterv = (int) ((t - rec->t0) / daysPerSubinterv);
			double subintervStartDate = rec->t0 + daysPerSubinterv * (double) subinterv;// 子间隔的时间起点
			// 系数指针由起点rec->coeffs，向后移动coeffInfo[planet].offset，移到指定的行星处
			// 再移动subinterv * coeffInfo[planet].nCoeffs * 3，移到指定的子间隔处
			coeffs = rec->coeffs + coeffInfo[planet].offset  - 1 + subinterv * coeffInfo[planet].nCoeffs * 3; // 2008-07-16
			u = 2.0 * (t - subintervStartDate) / daysPerSubinterv - 1.0;// 将时间映射到切比晓夫多项式的自变量区间[-1，1]
		}
		double cc[MaxChebyshevCoeffs];
		unsigned int nCoeffs = coeffInfo[planet].nCoeffs;               // 行星planet每个子间隔系数的个数
		for (int i = 0; i < 3; i++)
		{
			cc[0] = 1.0;
			cc[1] = u;
			P[i] = coeffs[i * nCoeffs] + coeffs[i * nCoeffs + 1] * u;   // 前两项累加和的初始化
			for (unsigned int j = 2; j < nCoeffs; j++)
			{
				cc[j] = 2.0 * u * cc[j - 1] - cc[j - 2];
				P[i] += coeffs[i * nCoeffs + j] * cc[j];
			}
		}
		return true;
	}

	// 子程序名称： getPlanetPosVel
	// 功能：计算行星的位置和速度, (J2000，月球是地心系，其它是太阳系，但非日心系) 
	// 变量类型：planet      :行星的标记
	//           t           :时刻，儒略日
	//           PV          :位置速度矢量[6] 单位：千米，千米/秒
	// 输入：planet，t
	// 输出：PV
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/03/27
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool JPLEphFile::getPlanetPosVel(JPLEphItem planet, double t, double PV[])
	{
		// 确保时间范围[startDate，endDate]
		if (t < startDate||t > endDate)
			return false;
		size_t recNo = (int) ((t - startDate) / daysPerInterval);//找到大区间
		// 当t == endDate，确保recNo不越界 
		if (recNo >= records.size())
			recNo = records.size() - 1;
		const JPLEphRecord* rec = &records[recNo];              //将指针指向该大区间的系数数据

		if(coeffInfo[planet].nSubinterv >= 1 && coeffInfo[planet].nSubinterv <= 32 && coeffInfo[planet].nCoeffs <= MaxChebyshevCoeffs)
		{
		}
		else
		{
			printf("nSubinterv < 1 or nSubinterv > 32 or nCoeffs > MaxChebyshevCoeffs!\n");
			return false;
		}

		// u是规范化系数[-1, 1]
		// coeffs是Chebyshev多项式系数指针，本程序指针的移动堪称经典
		double u = 0.0;
		double* coeffs = NULL;
		if (coeffInfo[planet].nSubinterv == -1)
		{
			coeffs = rec->coeffs + coeffInfo[planet].offset   - 1;
			u = 2.0 * (t - rec->t0) / daysPerInterval - 1.0;// 找到大区间所对应的时间rec->t0
		}
		else
		{
			double daysPerSubinterv = daysPerInterval / coeffInfo[planet].nSubinterv;
			int    subinterv = (int) ((t - rec->t0) / daysPerSubinterv);
			double subintervStartDate = rec->t0 + daysPerSubinterv * (double) subinterv;// 子间隔的时间起点
			// 系数指针由起点rec->coeffs，向后移动coeffInfo[planet].offset，移到指定的行星处
			// 再移动subinterv * coeffInfo[planet].nCoeffs * 3，移到指定的子间隔处
			coeffs = rec->coeffs + coeffInfo[planet].offset  - 1 + subinterv * coeffInfo[planet].nCoeffs * 3;
			u = 2.0 * (t - subintervStartDate) / daysPerSubinterv - 1.0; // 将时间映射到切比晓夫多项式的自变量区间[-1，1]
		}
		double cc [MaxChebyshevCoeffs];
		double ccv[MaxChebyshevCoeffs];
		unsigned int nCoeffs = coeffInfo[planet].nCoeffs;                    // 行星planet每个子间隔系数的个数
		double B = 2*coeffInfo[planet].nSubinterv/(daysPerInterval*86400.0); // 计算单位比例因子
		if( nCoeffs == 1 )
		{
			for (int i = 0; i < 3; i++)
			{
				PV[i]   = coeffs[i * nCoeffs];
				PV[i+3] = 0;
			}
		}
		else if( nCoeffs == 2 )
		{
			for (int i = 0; i < 3; i++)
			{
				PV[i]   = coeffs[i * nCoeffs] + coeffs[i * nCoeffs + 1] * u;
				PV[i+3] = coeffs[i * nCoeffs + 1];
				PV[i+3] = B * PV[i+3];
			}
		}
		else if( nCoeffs == 3 )
		{
			for (int i = 0; i < 3; i++)
			{
				PV[i]   = coeffs[i * nCoeffs] + coeffs[i * nCoeffs + 1] * u + coeffs[i * nCoeffs + 2] * (2*u*u - 1);
				PV[i+3] = coeffs[i * nCoeffs+1] + coeffs[i * nCoeffs + 2] * 4 * u;
				PV[i+3] = B * PV[i+3];
			}
		}
		else // nCoeffs > 3
		{
			for (int i = 0; i < 3; i++)
			{
				// 计算位置
				cc[0] = 1.0;
				cc[1] = u;
				PV[i] = coeffs[i * nCoeffs] + coeffs[i * nCoeffs + 1] * u;// 前两项累加和的初始化 + T0 + T1
				for (unsigned int j = 2; j < nCoeffs; j++)
				{
					cc[j] = 2.0 * u * cc[j - 1] - cc[j - 2];              // Tj
					PV[i] += coeffs[i * nCoeffs + j] * cc[j];             // + Tj
				}
				// 计算速度
				ccv[1]  = 1.0;
				ccv[2]  = 4 * u;
				PV[i+3] = coeffs[i * nCoeffs+1] * ccv[1] + coeffs[i * nCoeffs + 2] * ccv[2];// 前两项累加和的初始化 + T1' + T2'
				for (unsigned int j = 3; j < nCoeffs; j++)
				{
					ccv[j]  = 2.0 * u * ccv[j - 1] + 2 * cc[j - 1] - ccv[j - 2];     //  Tj' 
					PV[i+3] += coeffs[i * nCoeffs + j] * ccv[j];                     //  + Tj'                       
				}
				PV[i+3] = B * PV[i+3];
			}
		}
		return true;
	}


	// 子程序名称： getNutation
	// 功能：计算地球的黄经章动和黄赤交角章动
	// 变量类型：t            :时刻，儒略日
	//           delta_psi    :黄经章动 (度)
	//           delta_eps    :交角章动 (度)
	// 输入：t
	// 输出：delta_psi, delta_eps
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/03/27
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool JPLEphFile::getNutation(double t, double& delta_psi, double& delta_eps)
	{
		// 确保时间范围[startDate，endDate]
		if (t < startDate || t > endDate)
			return false;
		size_t recNo = (int) ((t - startDate) / daysPerInterval);// 找到大区间
		// 当t == endDate，确保 recNo 不越界 
		if (recNo >= records.size())
			recNo = records.size() - 1;
		JPLEphRecord* rec = &records[recNo];                     // 将指针指向该大区间的系数数据

		if(nutationCoeffInfo.nSubinterv >= 1 && nutationCoeffInfo.nSubinterv <= 32 && nutationCoeffInfo.nCoeffs <= MaxChebyshevCoeffs)
		{
		}
		else
		{
			printf("nSubinterv < 1 or nSubinterv > 32 or nCoeffs > MaxChebyshevCoeffs!\n");
			return false;
		}
		
		// u是规范化系数[-1, 1]
		// coeffs是Chebyshev多项式系数指针，本程序指针的移动堪称经典
		double u = 0.0;
		double* coeffs = NULL;
		if ( nutationCoeffInfo.nSubinterv == -1 )
		{
			coeffs = rec->coeffs + nutationCoeffInfo.offset - 1; // 2007-03-28 修改添加 "-1"
			u = 2.0 * (t - rec->t0) / daysPerInterval - 1.0;     // 找到大区间所对应的时间rec->t0
		}
		else
		{
			double daysPerSubinterv = daysPerInterval / nutationCoeffInfo.nSubinterv;
			int    subinterv = (int) ((t - rec->t0) / daysPerSubinterv);
			double subintervStartDate = rec->t0 + daysPerSubinterv * (double) subinterv;// 子间隔的时间起点
			// 系数指针由起点rec->coeffs，向后移动nutationCoeffInfo.offset，移到指定的章动处
			// 再移动subinterv * nutationCoeffInfo.nCoeffs * 2，移到指定的子间隔处
			// 2007-03-28 修改添加 "-1"
			coeffs = rec->coeffs + nutationCoeffInfo.offset  - 1 + subinterv * nutationCoeffInfo.nCoeffs * 2;//章动只有两个分量
			u = 2.0 * (t - subintervStartDate) / daysPerSubinterv - 1.0;// 将时间映射到切比晓夫多项式的自变量区间[-1，1]
		}
		double cc[MaxChebyshevCoeffs];
		unsigned int nCoeffs = nutationCoeffInfo.nCoeffs;// 每个子间隔系数的个数
		double P[2];
		for (int i = 0; i < 2; i++)
		{
			cc[0] = 1.0;
			cc[1] = u;
			P[i]  = coeffs[i * nCoeffs] + coeffs[i * nCoeffs + 1] * u;// 前两项累加和的初始化
			for (unsigned int j = 2; j < nCoeffs; j++)
			{
				cc[j] = 2.0 * u * cc[j - 1] - cc[j - 2];
				P[i] += coeffs[i * nCoeffs + j] * cc[j];
			}
		}
		delta_psi = P[0]*180/PI; // 换算成度
		delta_eps = P[1]*180/PI; // 换算成度
		return true;
	}

	// 子程序名称： getMoonLibration
	// 功能：计算月球的天平动参数
	// 变量类型：t        : 时刻，儒略日
	//           omiga    : 月球天球坐标系Xe轴（春分点方向）到升交点的角距(度)
	//           i_s      : 月球赤道面的倾角(度)
	//           lamda    : 月球固定坐标系Xb轴到升交点的角距(度)
	// 输入：t
	// 输出：omiga, i_s, lamda
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2018/09/13
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool JPLEphFile::getMoonLibration(double t, double& omiga, double& i_s, double& lamda)
	{
		// 确保时间范围[startDate，endDate]
		if (t < startDate || t > endDate)
			return false;
		size_t recNo = (int) ((t - startDate) / daysPerInterval);// 找到大区间
		// 当t == endDate，确保 recNo 不越界 
		if (recNo >= records.size())
			recNo = records.size() - 1;
		JPLEphRecord* rec = &records[recNo];                     // 将指针指向该大区间的系数数据

		if(librationCoeffInfo.nSubinterv >= 1 && librationCoeffInfo.nSubinterv <= 32 && librationCoeffInfo.nCoeffs <= MaxChebyshevCoeffs)
		{
		}
		else
		{
			printf("nSubinterv < 1 or nSubinterv > 32 or nCoeffs > MaxChebyshevCoeffs!\n");
			return false;
		}
		
		// u是规范化系数[-1, 1]
		// coeffs是Chebyshev多项式系数指针，本程序指针的移动堪称经典
		double u = 0.0;
		double* coeffs = NULL;
		if ( librationCoeffInfo.nSubinterv == -1 )
		{
			coeffs = rec->coeffs + librationCoeffInfo.offset - 1; // 2007-03-28 修改添加 "-1"
			u = 2.0 * (t - rec->t0) / daysPerInterval - 1.0;      // 找到大区间所对应的时间rec->t0
		}
		else
		{
			double daysPerSubinterv = daysPerInterval / librationCoeffInfo.nSubinterv;
			int    subinterv = (int) ((t - rec->t0) / daysPerSubinterv);
			double subintervStartDate = rec->t0 + daysPerSubinterv * (double) subinterv;// 子间隔的时间起点
			// 系数指针由起点rec->coeffs，向后移动librationCoeffInfo.offset，移到指定的章动处
			// 再移动subinterv * librationCoeffInfo.nCoeffs * 3，移到指定的子间隔处
			// 2007-03-28 修改添加 "-1"
			coeffs = rec->coeffs + librationCoeffInfo.offset  - 1 + subinterv * librationCoeffInfo.nCoeffs * 3;//章动只有两个分量
			u = 2.0 * (t - subintervStartDate) / daysPerSubinterv - 1.0;// 将时间映射到切比晓夫多项式的自变量区间[-1，1]
		}
		double cc[MaxChebyshevCoeffs];
		unsigned int nCoeffs = librationCoeffInfo.nCoeffs;// 每个子间隔系数的个数
		double P[3];
		for (int i = 0; i < 3; i++)
		{
			cc[0] = 1.0;
			cc[1] = u;
			P[i]  = coeffs[i * nCoeffs] + coeffs[i * nCoeffs + 1] * u;// 前两项累加和的初始化
			for (unsigned int j = 2; j < nCoeffs; j++)
			{
				cc[j] = 2.0 * u * cc[j - 1] - cc[j - 2];
				P[i] += coeffs[i * nCoeffs + j] * cc[j];
			}
		}
		omiga = P[0]*180/PI; // 换算成度
		i_s   = P[1]*180/PI; // 换算成度
		lamda = P[2]*180/PI; // 换算成度
		return true;
	}

	// 子程序名称： getMoonLibration
	// 功能：计算月球的天平动参数及其变化率
	// 变量类型：t        : 时刻，儒略日
	//           omiga    : 月球天球坐标系Xe轴（春分点方向）到升交点的角距(度)
	//           omiga_dot: 变化率
	//           i_s      : 月球赤道面的倾角(度)
	//           i_s_dot  : 变化率
	//           lamda    : 月球固定坐标系Xb轴到升交点的角距(度)
	//           lamda_dot: 变化率
	// 输入：t
	// 输出：omiga, i_s, lamda
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2018/09/13
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool JPLEphFile::getMoonLibration(double t, double& omiga, double& omiga_dot, double& i_s, double& i_s_dot, double& lamda, double& lamda_dot)
	{
		// 确保时间范围[startDate，endDate]
		if (t < startDate || t > endDate)
			return false;
		size_t recNo = (int) ((t - startDate) / daysPerInterval);// 找到大区间
		// 当t == endDate，确保 recNo 不越界 
		if (recNo >= records.size())
			recNo = records.size() - 1;
		JPLEphRecord* rec = &records[recNo];                     // 将指针指向该大区间的系数数据

		if(librationCoeffInfo.nSubinterv >= 1 && librationCoeffInfo.nSubinterv <= 32 && librationCoeffInfo.nCoeffs <= MaxChebyshevCoeffs)
		{
		}
		else
		{
			printf("nSubinterv < 1 or nSubinterv > 32 or nCoeffs > MaxChebyshevCoeffs!\n");
			return false;
		}
		
		// u是规范化系数[-1, 1]
		// coeffs是Chebyshev多项式系数指针，本程序指针的移动堪称经典
		double u = 0.0;
		double* coeffs = NULL;
		if ( librationCoeffInfo.nSubinterv == -1 )
		{
			coeffs = rec->coeffs + librationCoeffInfo.offset - 1; // 2007-03-28 修改添加 "-1"
			u = 2.0 * (t - rec->t0) / daysPerInterval - 1.0;      // 找到大区间所对应的时间rec->t0
		}
		else
		{
			double daysPerSubinterv = daysPerInterval / librationCoeffInfo.nSubinterv;
			int    subinterv = (int) ((t - rec->t0) / daysPerSubinterv);
			double subintervStartDate = rec->t0 + daysPerSubinterv * (double) subinterv;// 子间隔的时间起点
			// 系数指针由起点rec->coeffs，向后移动librationCoeffInfo.offset，移到指定的章动处
			// 再移动subinterv * librationCoeffInfo.nCoeffs * 3，移到指定的子间隔处
			// 2007-03-28 修改添加 "-1"
			coeffs = rec->coeffs + librationCoeffInfo.offset  - 1 + subinterv * librationCoeffInfo.nCoeffs * 3;//章动只有两个分量
			u = 2.0 * (t - subintervStartDate) / daysPerSubinterv - 1.0;// 将时间映射到切比晓夫多项式的自变量区间[-1，1]
		}
		double cc [MaxChebyshevCoeffs];
		double ccv[MaxChebyshevCoeffs];
		unsigned int nCoeffs = librationCoeffInfo.nCoeffs;// 每个子间隔系数的个数
		double B = 2 * librationCoeffInfo.nSubinterv / (daysPerInterval * 86400.0); // 计算单位比例因子
		double PV[6];
		if( nCoeffs == 1 )
		{
			for (int i = 0; i < 3; i++)
			{
				PV[i]   = coeffs[i * nCoeffs];
				PV[i+3] = 0;
			}
		}
		else if( nCoeffs == 2 )
		{
			for (int i = 0; i < 3; i++)
			{
				PV[i]   = coeffs[i * nCoeffs] + coeffs[i * nCoeffs + 1] * u;
				PV[i+3] = coeffs[i * nCoeffs + 1];
				PV[i+3] = B * PV[i+3];
			}
		}
		else if( nCoeffs == 3 )
		{
			for (int i = 0; i < 3; i++)
			{
				PV[i]   = coeffs[i * nCoeffs] + coeffs[i * nCoeffs + 1] * u + coeffs[i * nCoeffs + 2] * (2*u*u - 1);
				PV[i+3] = coeffs[i * nCoeffs+1] + coeffs[i * nCoeffs + 2] * 4 * u;
				PV[i+3] = B * PV[i+3];
			}
		}
		else // nCoeffs > 3
		{
			for (int i = 0; i < 3; i++)
			{
				// 计算位置
				cc[0] = 1.0;
				cc[1] = u;
				PV[i] = coeffs[i * nCoeffs] + coeffs[i * nCoeffs + 1] * u;// 前两项累加和的初始化 + T0 + T1
				for (unsigned int j = 2; j < nCoeffs; j++)
				{
					cc[j] = 2.0 * u * cc[j - 1] - cc[j - 2];              // Tj
					PV[i] += coeffs[i * nCoeffs + j] * cc[j];             // + Tj
				}
				// 计算速度
				ccv[1]  = 1.0;
				ccv[2]  = 4 * u;
				PV[i+3] = coeffs[i * nCoeffs+1] * ccv[1] + coeffs[i * nCoeffs + 2] * ccv[2];// 前两项累加和的初始化 + T1' + T2'
				for (unsigned int j = 3; j < nCoeffs; j++)
				{
					ccv[j]  = 2.0 * u * ccv[j - 1] + 2 * cc[j - 1] - ccv[j - 2];     //  Tj' 
					PV[i+3] += coeffs[i * nCoeffs + j] * ccv[j];                     //  + Tj'                       
				}
				PV[i+3] = B * PV[i+3];
			}
		}
		omiga     = PV[0]*180/PI; // 换算成度
		i_s       = PV[1]*180/PI; // 换算成度
		lamda     = PV[2]*180/PI; // 换算成度
		omiga_dot = PV[3]*180/PI; // 换算成度
		i_s_dot   = PV[4]*180/PI; // 换算成度
		lamda_dot = PV[5]*180/PI; // 换算成度
		return true;
	}

	// 子程序名称： getPlanetPos_EarthCenter
	// 功能：通过平移变换, 计算行星的地心位置
	//       其中月球的位置数据相对地心给出的, 可直接调用 getPlanetPos 函数, 其它行星需要通过转换获得
	// 变量类型：planet      :行星的标记
	//          t            :时刻，儒略日(太阳时)
	//          P            :位置矢量[3] (千米)
	// 输入：planet, t
	// 输出：P
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/7/9
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool JPLEphFile::getPlanetPos_EarthCenter(JPLEphItem planet, double t, double P[])
	{
		if(!getPlanetPos(planet, t, P))                                         // 计算行星的位置(太阳系)
			return false; // 太阳在太阳系位置
		double P_E[3],P_EM[3],P_M[3];
		if(!getPlanetPos(JPLEph_EarthMoonBary, t, P_EM))                        // 地月质心位置(太阳系)
			return false; // 地月质心在太阳系位置
		if(!getPlanetPos(JPLEph_Moon, t, P_M))                                  // 月球质心位置(地心系)
			return false;
		for(int i = 0; i < 3; i++)
		{// P_E[i] 为太阳质心相对地心的偏移量
			P_E[i] = P_EM[i] - ( MASS_MOON/( MASS_MOON + MASS_EARTH )) * P_M[i]; // 计算地球质心位置(太阳系)
			P[i]   = P[i] - P_E[i];
		}
		return true;
	}

	bool JPLEphFile::getPlanetPosVel_EarthCenter(JPLEphItem planet, double t, double PV[])
	{
		if(!getPlanetPosVel(planet, t, PV))                                      // 计算行星的位置(太阳系)
			return false; // 太阳在太阳系位置
		double P_E[6],P_EM[6],P_M[6];
		if(!getPlanetPosVel(JPLEph_EarthMoonBary, t, P_EM))                      // 地月质心位置(太阳系)
			return false; // 地月质心在太阳系位置
		if(!getPlanetPosVel(JPLEph_Moon, t, P_M))                                // 月球质心位置(地心系)
			return false;
		for(int i = 0; i < 6; i++)
		{// P_E[i] 为太阳质心相对地心的偏移量
			P_E[i] = P_EM[i] - ( MASS_MOON/( MASS_MOON + MASS_EARTH )) * P_M[i]; // 计算地球质心位置(太阳系)
			PV[i]  = PV[i] - P_E[i];
		}
		return true;
	}

	// 子程序名称： getSunPos_Delay_EarthCenter
	// 功能：通过平移变换, 计算太阳的 J2000 地心位置
	//       考虑到太阳的传播延迟 8 分 10 秒, 需要考虑传播过程中地球的平移运动影响
	// 变量类型：t           : 时刻, 儒略日, 太阳时
	//           P           : 位置矢量[3] (千米), 当前时刻 t 的地心J2000系
	//           threshold   : 迭代阈值, 默认 1.0E-007
	// 输入：t, threshold
	// 输出：P
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/7/16
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool JPLEphFile::getSunPos_Delay_EarthCenter(double t, double P[], double threshold)
	{
		// 第一步: 计算 t 时刻地心在太阳质心系位置
		double P_E[3],P_EM[3],P_M[3];
		if(!getPlanetPos(JPLEph_EarthMoonBary, t, P_EM))   // 地月质心位置(太阳系)
			return false; // 地月质心在太阳系位置
		if(!getPlanetPos(JPLEph_Moon, t, P_M))             // 月球质心位置(地心系)
			return false;
		for(int i = 0; i < 3; i++)
		{	
			P_E[i] = P_EM[i] - (MASS_MOON/(MASS_MOON + MASS_EARTH)) * P_M[i]; // 计算地球质心位置(太阳系)
	    }
		// 第二步: 计算太阳在 t - DELAY_SUN_EARTH 时刻的太阳质心系位置
		// DELAY_SUN_EARTH = 490 / 86400.0; // 太阳光的传播延迟, 8分10秒(天)
		double t_Transmit = t;
		if(!getPlanetPos(JPLEph_Sun, t_Transmit, P)) // 计算太阳的位置(太阳系)
			return false; // 太阳在太阳系位置
		double distance = pow(P[0] - P_E[0], 2) + pow(P[1] - P_E[1], 2) + pow(P[2] - P_E[2], 2);
		distance = sqrt(distance) * 1000.0; 
		double delay = distance / SPEED_LIGHT;
		double delay_k_1 = 0;
		const double delay_max  = 86400;   // 为了防止迭代dDelay溢出，这里设置一个阈值
		const int    k_max      = 5;       // 迭代次数阈值，一般1次迭代就会收敛？ 
		int          k          = 0;
		while(fabs(delay - delay_k_1) > threshold)   // 迭代阈值控制, abs-->fabs, 2007-07-15
		{
			k++;
			if(fabs(delay) > delay_max || k > k_max) // 为防止 delay 溢出, 2007-04-06
			{
				printf("getSunDelayPos_Geocenter 迭代发散!\n");
				return false;
			}
			// 更新 GPS 信号发射时间
			t_Transmit = t - delay / 86400.0;
			if(!getPlanetPos(JPLEph_Sun, t_Transmit, P))
				return false;
			// 更新概略距离
			distance = pow(P[0] - P_E[0], 2) + pow(P[1] - P_E[1], 2) + pow(P[2] - P_E[2], 2);
		    distance = sqrt(distance) * 1000.0;
			// 更新延迟数据
			delay_k_1 = delay;
			delay = distance / SPEED_LIGHT;
		}
		P[0] = P[0] - P_E[0];
		P[1] = P[1] - P_E[1];
		P[2] = P[2] - P_E[2];
		return true;
	}

	// 子程序名称： getEarthPosVel
	// 功能：计算地球的位置和速度，(J2000，日心系)
	// 变量类型：t           :时刻，儒略日
	//           PV          :位置速度矢量[6]，单位：千米，千米/秒
	// 输入：t
	// 输出：PV
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/3/27
	// 版本时间：
	// 修改记录：1.2018.07.12, 谷德峰修改, 此前计算得到的是在太阳系, 非日心系
	// 备注： 调用 getPlanetPosVel_EarthCenter
	bool JPLEphFile::getEarthPosVel(double t, double PV[])
	{
		//double P_EM[6], P_M[6];
		//if(!getPlanetPosVel(JPLEph_EarthMoonBary, t, P_EM))                   // 地月质心位置速度(太阳系)
		//	return false; 
		//if(!getPlanetPosVel(JPLEph_Moon, t, P_M))                             // 月球质心位置速度(太阳系)
		//	return false;
		//for(int i = 0; i < 6; i++)
		//{	
		//	PV[i] = P_EM[i] - (MASS_MOON/( MASS_MOON + MASS_EARTH)) * P_M[i];  // 计算地球质心位置速度(太阳系)
		//}

		bool result = getPlanetPosVel_EarthCenter(JPLEph_Sun, t, PV);
		for(int i = 0; i < 6; i++)
			PV[i] = -PV[i]; // 更换符号
		return result;
	}
}