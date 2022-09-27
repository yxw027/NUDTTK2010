#include "SolidTides.hpp"
#include "TimeCoordConvert.hpp"

namespace NUDTTK
{
	double SolidTides::IERS2003_Table7_5a[11][9] = 
	{{ 1,  0, 2,  0,  2, -0.08,  0.00, -0.01,  0.01},
	 { 0,  0, 2,  0,  1, -0.10,  0.00,  0.00,  0.00},
	 { 0,  0, 2,  0,  2, -0.51,  0.00, -0.02,  0.03},
	 { 1,  0, 0,  0,  0,  0.06,  0.00,  0.00,  0.00},
	 { 0,  1, 2, -2,  2, -0.06,  0.00,  0.00,  0.00},
	 { 0,  0, 2, -2,  2, -1.23, -0.07,  0.06,  0.01},
	 { 0,  0, 0,  0, -1, -0.22,  0.01,  0.01,  0.00},
	 { 0,  0, 0,  0,  0, 12.00, -0.78, -0.67, -0.03},
	 { 0,  0, 0,  0,  1,  1.73, -0.12, -0.10,  0.00},
	 { 0, -1, 0,  0,  0, -0.50, -0.01,  0.03,  0.00},
	 { 0,  0,-2,  2, -2, -0.11,  0.01,  0.01,  0.00}};

	double SolidTides::IERS2003_Table7_5b[5][9] = 
	{{ 0, 0,  0, 0,  1,  0.47,  0.16,  0.23,  0.07},
	 { 0, 0, -2, 2, -2, -0.20, -0.11, -0.12, -0.05},
	 {-1, 0,  0, 0,  0, -0.11, -0.09, -0.08, -0.04},
	 { 0, 0, -2, 0, -2, -0.13, -0.15, -0.11, -0.07},
	 { 0, 0, -2, 0, -1, -0.05, -0.06, -0.05, -0.03}};

	SolidTides::SolidTides(void)
	{
	}

	SolidTides::~SolidTides(void)
	{
	}

	// 子程序名称： solidTideCorrect
	// 功能：计算固体潮引起的测站位置偏移改正量
	// 变量类型：t            :  参考时刻
	//           sunPos       :  太阳在地心系下位置（ITRF, 米）
	//           moonPos      :  月亮在地心系下位置（ITRF, 米）
	//           staPos       :  测站在地心系下位置（ITRF, 米）
	//           xp           :  极移
	//           yp           :  极移
	// 输入：t, sunPos, moonPos, staPos, xp, yp
	// 输出：posDisplacement_ECEF 
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2013/06/13
	// 版本时间：2013/06/13
	// 修改记录：
	// 备注：主要参考 IERS2003
	POS3D SolidTides::solidTideCorrect(GPST t, POS3D sunPos, POS3D moonPos, POS3D staPos, double xp, double yp)
	{
		// 计算测站大地经纬度
		BLH staBLH;
		TimeCoordConvert::XYZ2BLH(staPos, staBLH);
		double fai = staBLH.B * PI / 180; // 测站的纬度
		double lamda = staBLH.L * PI / 180; // 测站的东经
		// 地心距离
		double r_Sun  = sqrt(sunPos.x  * sunPos.x  + sunPos.y  * sunPos.y  + sunPos.z  * sunPos.z);
		double r_Moon = sqrt(moonPos.x * moonPos.x + moonPos.y * moonPos.y + moonPos.z * moonPos.z);
		double r_Sta  = sqrt(staPos.x * staPos.x + staPos.y * staPos.y + staPos.z * staPos.z);
		POS3D unitvec_Sun  = vectorNormal(sunPos);
		POS3D unitvec_Moon = vectorNormal(moonPos);
		POS3D unitvec_Sta  = vectorNormal(staPos);
		// 第一步: 时域改正(TA)
		// 1-1: in-phase, for degree 2
		double h2 = 0.6078 - 0.0006 * (3 * sin(fai) * sin(fai) - 1) / 2; // IERS2003
		double l2 = 0.0847 + 0.0002 * (3 * sin(fai) * sin(fai) - 1) / 2;
		double dotvalue_Sun_Sta  = vectorDot(unitvec_Sun,  unitvec_Sta);
		double dotvalue_Moon_Sta = vectorDot(unitvec_Moon, unitvec_Sta);
		// 太阳改正, 改进量较大, 厘米量级
		POS3D delta_pos_sun_TA_d2  = unitvec_Sta * (3 * (h2/2.0 - l2) * pow(dotvalue_Sun_Sta,2)  - h2/2.0);
		delta_pos_sun_TA_d2 = delta_pos_sun_TA_d2 + unitvec_Sun * (3 * l2 * dotvalue_Sun_Sta);
		delta_pos_sun_TA_d2 = delta_pos_sun_TA_d2 * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		POS3D delta_pos_sun_TA = delta_pos_sun_TA_d2;
		// 月球改正, 改进量较大, 分米量级
		POS3D delta_pos_moon_TA_d2 = unitvec_Sta * (3 * (h2/2.0 - l2) * pow(dotvalue_Moon_Sta,2) - h2/2.0);
		delta_pos_moon_TA_d2 = delta_pos_moon_TA_d2 + unitvec_Moon * (3 * l2 * dotvalue_Moon_Sta);
		delta_pos_moon_TA_d2 = delta_pos_moon_TA_d2 * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		POS3D delta_pos_moon_TA = delta_pos_moon_TA_d2;
		// 1-2: in-phase, for degree 3
		// Only the Moon’s contribution (j = 2) need be computed, the term due to the Sun being quite ignorable.
		// The transverse part of the displacement does not exceed 0.2 mm, but the radial displacement can reach 1.7mm.
		double h3 = 0.292; 
		double l3 = 0.015;
		//// 太阳
		//POS3D delta_pos_sun_TA_d3 = unitvec_Sta * (h3 * 2.5 * pow(dotvalue_Sun_Sta,3)  - h3 * 1.5 * dotvalue_Sun_Sta);
        //delta_pos_sun_TA_d3  = delta_pos_sun_TA_d3 + (unitvec_Sun - unitvec_Sta * dotvalue_Sun_Sta) * (l3 * (7.5 * pow(dotvalue_Sun_Sta,2)  - 1.5));
		//delta_pos_sun_TA_d3 = delta_pos_sun_TA_d3 * ((GM_SUN * pow(r_Sta, 5)) / (GM_EARTH * pow(r_Sun, 4)));
		//delta_pos_sun_TA = delta_pos_sun_TA + delta_pos_sun_TA_d3;
		// 月亮
		POS3D delta_pos_moon_TA_d3 = unitvec_Sta * (h3 * 2.5 * pow(dotvalue_Moon_Sta,3)  - h3 * 1.5 * dotvalue_Moon_Sta);
        delta_pos_moon_TA_d3 = delta_pos_moon_TA_d3 + (unitvec_Moon - unitvec_Sta * dotvalue_Moon_Sta) * (l3 * (7.5 * pow(dotvalue_Moon_Sta,2)  - 1.5));
		delta_pos_moon_TA_d3 = delta_pos_moon_TA_d3 * ((GM_MOON * pow(r_Sta, 5)) / (GM_EARTH * pow(r_Moon, 4)));
		delta_pos_moon_TA = delta_pos_moon_TA + delta_pos_moon_TA_d3;
        // 1-3: out-of-phase for degree 2 only
		BLH sunBLH, moonBLH;
		TimeCoordConvert::XYZ2BLH(sunPos,  sunBLH);
		TimeCoordConvert::XYZ2BLH(moonPos, moonBLH);
		double fai_sun    = sunBLH.B  * PI / 180; // 太阳的地心纬度
		double lamda_sun  = sunBLH.L  * PI / 180; // 太阳的地心东经
		double fai_moon   = moonBLH.B * PI / 180; // 月球的地心纬度
		double lamda_moon = moonBLH.L * PI / 180; // 月球的地心东经
		ENU delta_enu_sun_TA_diur, delta_enu_moon_TA_diur;
		ENU delta_enu_sun_TA_semi, delta_enu_moon_TA_semi;
		delta_enu_sun_TA_diur.U  = -0.75 * (-0.0025) * sin(2 * fai_sun) * sin(2 * fai) * sin(lamda - lamda_sun) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_diur.U = -0.75 * (-0.0025) * sin(2 * fai_moon) * sin(2 * fai) * sin(lamda - lamda_moon) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		delta_enu_sun_TA_diur.E  = -1.50 * (-0.0007) * sin(2 * fai_sun) * sin(fai) * cos(lamda - lamda_sun) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_diur.E = -1.50 * (-0.0007) * sin(2 * fai_moon) * sin(fai) * cos(lamda - lamda_moon) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		delta_enu_sun_TA_diur.N  = -1.50 * (-0.0007) * sin(2 * fai_sun) * cos(2 * fai) * sin(lamda - lamda_sun) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_diur.N = -1.50 * (-0.0007) * sin(2 * fai_moon) * cos(2 * fai) * sin(lamda - lamda_moon) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		delta_enu_sun_TA_semi.U  = -0.75 * (-0.0022) * pow(cos(fai_sun), 2) * pow(cos(fai), 2) * sin(2 * (lamda - lamda_sun)) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_semi.U = -0.75 * (-0.0022) * pow(cos(fai_moon), 2) * pow(cos(fai), 2) * sin(2 * (lamda - lamda_moon)) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		delta_enu_sun_TA_semi.E  = -0.75 * (-0.0007) * pow(cos(fai_sun), 2) * (-2.0) * cos(fai) * cos(2 * (lamda - lamda_sun)) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_semi.E = -0.75 * (-0.0007) * pow(cos(fai_moon), 2) * (-2.0) * cos(fai) * cos(2 * (lamda - lamda_moon)) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		delta_enu_sun_TA_semi.N  = -0.75 * (-0.0007) * pow(cos(fai_sun), 2) * sin(2 * fai) * sin(2 * (lamda - lamda_sun)) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_semi.N = -0.75 * (-0.0007) * pow(cos(fai_moon), 2) * sin(2 * fai) * sin(2 * (lamda - lamda_moon)) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		// 1-4: contribution from latitude dependence, 2014/10/14, 谷德峰补充
		// The contributions of the l(1) term to the transverse displacements caused
		// by the diurnal and semidiurnal tides could be up to 0.8 mm and 1.0 mm respectively.
		ENU delta_enu_sun_TA_diur_l1, delta_enu_moon_TA_diur_l1;
		ENU delta_enu_sun_TA_semi_l1, delta_enu_moon_TA_semi_l1;
		double P1_sin_fai_sun = 0.0;
		if(cos(lamda_sun) != 0)
			P1_sin_fai_sun = 3 * (sunPos.x / r_Sun)  * (sunPos.z / r_Sun) * (1.0 / cos(lamda_sun));
		else
			P1_sin_fai_sun = 3 * (sunPos.y / r_Sun)  * (sunPos.z / r_Sun) * (1.0 / sin(lamda_sun));
		double P2_sin_fai_sun = 0.0;
		if(sin(2 * lamda_sun) != 0)
			P2_sin_fai_sun = 6 * (sunPos.x / r_Sun)  * (sunPos.y / r_Sun) * (1.0 / sin(2 * lamda_sun));
		else
			P2_sin_fai_sun = 3 * ((sunPos.x + sunPos.y) / r_Sun)  * ((sunPos.x - sunPos.y) / r_Sun) * (1.0 / cos(2 * lamda_sun));
		double P1_sin_fai_moon = 0.0;
		if(cos(lamda_moon) != 0)
			P1_sin_fai_moon = 3 * (moonPos.x / r_Moon)  * (moonPos.z / r_Moon) * (1.0 / cos(lamda_moon));
		else
			P1_sin_fai_moon = 3 * (moonPos.y / r_Moon)  * (moonPos.z / r_Moon) * (1.0 / sin(lamda_moon));
		double P2_sin_fai_moon = 0.0;
		if(sin(2 * lamda_moon) != 0)
			P2_sin_fai_moon = 6 * (moonPos.x / r_Moon)  * (moonPos.y / r_Moon) * (1.0 / sin(2 * lamda_moon));
		else
			P2_sin_fai_moon = 3 * ((moonPos.x + moonPos.y) / r_Moon)  * ((moonPos.x - moonPos.y) / r_Moon) * (1.0 / cos(2 * lamda_moon));
		delta_enu_sun_TA_diur_l1.U  =  0.0;
		delta_enu_moon_TA_diur_l1.U =  0.0;
		delta_enu_sun_TA_diur_l1.N  = -0.0012 * sin(fai) * P1_sin_fai_sun * sin(fai) * cos(lamda - lamda_sun) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_diur_l1.N = -0.0012 * sin(fai) * P1_sin_fai_moon * sin(fai) * cos(lamda - lamda_moon) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		delta_enu_sun_TA_diur_l1.E  =  0.0012 * sin(fai) * P1_sin_fai_sun * cos(2 * fai) * sin(lamda - lamda_sun) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_diur_l1.E =  0.0012 * sin(fai) * P1_sin_fai_moon * cos(2 * fai) * sin(lamda - lamda_moon) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		delta_enu_sun_TA_semi_l1.U  =  0.0;
		delta_enu_moon_TA_semi_l1.U =  0.0;
		delta_enu_sun_TA_semi_l1.N  = -0.0012 * sin(fai) * cos(fai) * P2_sin_fai_sun * cos(2 * (lamda - lamda_sun)) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_semi_l1.N = -0.0012 * sin(fai) * cos(fai) * P2_sin_fai_moon * cos(2 * (lamda - lamda_moon)) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));
		delta_enu_sun_TA_semi_l1.E  = -0.0012 * sin(fai) * cos(fai) * P2_sin_fai_sun * sin(fai) * sin(2 * (lamda - lamda_sun)) * ((GM_SUN * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Sun, 3)));
		delta_enu_moon_TA_semi_l1.E = -0.0012 * sin(fai) * cos(fai) * P2_sin_fai_moon * sin(fai) * sin(2 * (lamda - lamda_moon)) * ((GM_MOON * pow(r_Sta, 4)) / (GM_EARTH * pow(r_Moon, 3)));

		// 第二步: 频域改正(FA), 考虑由于第一步采用了与频率无关的 Love 数和 Shida 数而导致的进一步改正
		// 2-1: diurnal tides: Sum over all the components of Table 7.5a
		// 根据 jY2000_TDT 计算下面天文量(单位角秒)
		double jY2000_TDT = TimeCoordConvert::DayTime2J2000Year(TimeCoordConvert::GPST2TDT(t));
		double T  = jY2000_TDT;
		double T2 = T  * jY2000_TDT;
		double T3 = T2 * jY2000_TDT;
		double T4 = T3 * jY2000_TDT;
		double L      = 134.96340251 * 3600 + 1717915923.2178 * T + 31.8792 * T2 + 0.051635 * T3 - 0.00024470 * T4; // 月球的平近点角
		double L1     = 357.52910918 * 3600 +  129596581.0481 * T -  0.5532 * T2 - 0.000136 * T3 - 0.00001149 * T4; // 太阳的平近点角
		double F      =  93.27209062 * 3600 + 1739527262.8478 * T - 12.7512 * T2 - 0.001037 * T3 + 0.00000417 * T4; // 月球纬度的平均角距
		double D      = 297.85019547 * 3600 + 1602961601.2090 * T -  6.3706 * T2 + 0.006593 * T3 - 0.00003169 * T4; // 日月相对地球的平均夹角
		double Omiga  = 125.04455501 * 3600 -    6962890.2665 * T +  7.4722 * T2 + 0.007702 * T3 - 0.00005939 * T4; // 月球平轨道在黄道上升交点的黄经，由当日平春分点起量 
		ENU delta_enu_FA_diur;
		delta_enu_FA_diur.E = 0.0;
		delta_enu_FA_diur.N = 0.0;
		delta_enu_FA_diur.U = 0.0;
		for(int i = 0;i < 11; i++)
		{
			double Argument = IERS2003_Table7_5a[i][0]*L + IERS2003_Table7_5a[i][1]*L1 + IERS2003_Table7_5a[i][2]*F + IERS2003_Table7_5a[i][3]*D + IERS2003_Table7_5a[i][4]*Omiga;
			delta_enu_FA_diur.U += sin(2 *fai) *(IERS2003_Table7_5a[i][5] * sin(Argument*PI/(3600*180) + lamda) + IERS2003_Table7_5a[i][6] * cos(Argument*PI/(3600*180) + lamda)); 
			delta_enu_FA_diur.E += sin(fai) *(IERS2003_Table7_5a[i][7] * cos(Argument*PI/(3600*180) + lamda) - IERS2003_Table7_5a[i][8] * sin(Argument*PI/(3600*180) + lamda)); 
			delta_enu_FA_diur.N += cos(2 *fai) *(IERS2003_Table7_5a[i][7] * sin(Argument*PI/(3600*180) + lamda) + IERS2003_Table7_5a[i][8] * cos(Argument*PI/(3600*180) + lamda)); 
		}
		// 2-2: long-period tides: Sum over all the components of Table 7.5b
		ENU delta_enu_FA_longperiod;
		delta_enu_FA_longperiod.E = 0.0;
		delta_enu_FA_longperiod.N = 0.0;
		delta_enu_FA_longperiod.U = 0.0;
		for(int i = 0;i < 5; i++)
		{
			double Argument = IERS2003_Table7_5b[i][0]*L + IERS2003_Table7_5b[i][1]*L1 + IERS2003_Table7_5b[i][2]*F + IERS2003_Table7_5b[i][3]*D + IERS2003_Table7_5b[i][4]*Omiga;
			delta_enu_FA_longperiod.U += (1.5 * pow(sin(fai), 2) - 0.5)  *(IERS2003_Table7_5b[i][5] * cos(Argument*PI/(3600*180)) + IERS2003_Table7_5b[i][6] * sin(Argument*PI/(3600*180))); // 2014/10/13, ((IERS2003_Table7_5b[i][5] * sin(Argument*PI/(3600*180)) + IERS2003_Table7_5b[i][6] * cos(Argument*PI/(3600*180)))
			delta_enu_FA_longperiod.N += sin(2 *fai) *(IERS2003_Table7_5b[i][7] * cos(Argument*PI/(3600*180) + lamda) + IERS2003_Table7_5b[i][8] * sin(Argument*PI/(3600*180) + lamda)); // 2014/10/13, (IERS2003_Table7_5b[i][7] * sin(Argument*PI/(3600*180) + lamda) + IERS2003_Table7_5b[i][8] * cos(Argument*PI/(3600*180) + lamda));
		}
		// mm 换算成 m
		delta_enu_FA_diur = delta_enu_FA_diur * 0.001;
		delta_enu_FA_longperiod = delta_enu_FA_longperiod * 0.001;
		//// 第三部: 计算永久潮汐项, [该项修正导致 SLR 比对残差变大, 故暂时未开启]
		/*ENU delta_enu_Permanent;
		double p2_sinfai = (3 * pow(sin(fai), 2) - 1) / 2; 
		delta_enu_Permanent.E =   0.0;
		delta_enu_Permanent.N = (-0.0252 + 0.0001 * p2_sinfai) * sin(2 *fai);
		delta_enu_Permanent.U = (-0.1206 + 0.0001 * p2_sinfai) * p2_sinfai;*/
		// 第四步: 计算极潮引起的测站位移, 径向最大位移约 25mm, 横向最大位移约 7mm
		double sita = PI / 2 - fai;
		double m1 =   xp - (0.054 + 0.00083 * jY2000_TDT); 
		double m2 = -(yp - (0.357 + 0.00395 * jY2000_TDT));
		ENU delta_enu_Pole;
		delta_enu_Pole.E =  0.009 * cos(sita)     * (m1 * sin(lamda) - m2 * cos(lamda));
		delta_enu_Pole.N =  0.009 * cos(2 * sita) * (m1 * cos(lamda) + m2 * sin(lamda));
		delta_enu_Pole.U = -0.032 * sin(2 * sita) * (m1 * cos(lamda) + m2 * sin(lamda));
		// 转换到地固系
		ENU delta_enu_Total = delta_enu_sun_TA_diur
			                + delta_enu_moon_TA_diur
							+ delta_enu_sun_TA_semi
							+ delta_enu_moon_TA_semi
							+ delta_enu_sun_TA_diur_l1
							+ delta_enu_moon_TA_diur_l1
							+ delta_enu_sun_TA_semi_l1
							+ delta_enu_moon_TA_semi_l1
							+ delta_enu_FA_diur
							+ delta_enu_FA_longperiod
							//+ delta_enu_Permanent
							+ delta_enu_Pole;
		POS3D posDisplacement_ECEF;
		TimeCoordConvert::ENU2ECF(staPos, delta_enu_Total, posDisplacement_ECEF);
		posDisplacement_ECEF = posDisplacement_ECEF - staPos + delta_pos_sun_TA + delta_pos_moon_TA;
		return posDisplacement_ECEF;
	}
}
