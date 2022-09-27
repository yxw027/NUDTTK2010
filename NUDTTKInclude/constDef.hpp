#pragma once
//  Copyright 2012, The National University of Defense Technology at ChangSha

#define PI 3.1415926535897932384626433832795	    // 圆周率

// 地球椭球定义: 采用 IGS00, ITRF2000 系列的定义, 采用 IGS00, ITRF2000 系列的定义 6378136.30f, 1 / 298.2572221
const double EARTH_R			=  6378136.60f;   // IERS 2010      
const double EARTH_F            =  1 / 298.25642; // 大地子午圈的扁率, 

const double WGS84_GM_EARTH     = 3.986005E+14;     // 进一步确认
const double WGS84_EARTH_W      = 7.2921151467E-5;

// CGCS2000参考椭球定义的基本常数为
const double CGCS2000_GM_EARTH  = 3.986004418E+14; 
const double CGCS2000_EARTH_W   = 7.2921150E-5;
const double CGCS2000_EARTH_R   = 6378137.0f;        
const double CGCS2000_EARTH_F   = 1 / 298.257222101; 

// 行星的质量
const double MASS_EARTH         =  5.973242E+24;    // 地球质量, 单位kg
const double MASS_MOON          =  7.35E+22;        // 月球质量, 单位kg
const double MASS_SUN           =  1.9889E+30;      // 太阳质量, 单位kg

// 地球基本常数
const double G_EARTH            =  6.67259E-11;     // 地球引力常数, 单位m^3kg^-1s^-2
const double GE_EARTH           =  9.780327;        // 平均赤道重力, 单位ms^-2
const double DENSITY_SEAWATER   =  1025;            // 海水的密度, kgm^-3
//const double GM_EARTH           =  3.986004415E+14; // 地球引力常数
const double GM_EARTH           =  3.986004418E+14; // 地球引力常数,IERS
const double EARTH_W            =  7.2921151467E-5; // 地球平均角速度, 单位 rad/s, 7.2921151467E-5
const double SPEED_LIGHT        =  299792458;       // 光速, 米/秒
const double SOLAR_PRESSURE     =  4.5604E-6;       // 地球附近的太阳光压强常数, 单位 牛顿/米^2
const double AU                 =  1.49597870E+11;  // 天文单位距离
const double S0                 =  1367;            // 1AU的太阳辐射，单位 W/m^2
const double EPS_0              =  84381.4059;      // 单位 as
const double Re			        =  6371000;         // 地球平均半径, m
const double ALBEDO             =  0.3;              // 地球反照率常量

// 其他行星基本常数
//const double GM_MOON            =  0.4902802627E+13;           // 月球引力常数 (m^3 / s^2)
//const double GM_SUN             =  1.32712440E+20;             // 太阳引力常数 (m^3 / s^2)
//const double GM_MERCURY         =  1.32712440E+20 / 6023600.0; // 水星引力常数
//const double GM_VENUS           =  1.32712440E+20 / 408523.5;  // 金星引力常数
//const double GM_MARS            =  1.32712440E+20 / 3098710.0; // 火星引力常数
//const double GM_JUPITER         =  1.32712440E+20 / 1047.355;  // 木星引力常数
//const double GM_SATURN          =  1.32712440E+20 / 3498.5;    // 土星引力常数
//const double GM_URANUS          =  1.32712440E+20 / 22869.0;   // 天王星引力常数
//const double GM_NEPTUNE         =  1.32712440E+20 / 19314.0;   // 海王星引力常数
//const double GM_PLUTO           =  1.32712440E+20 / 3000000.0; // 冥王星引力常数
// IERS2010
const double GM_MOON            =  0.4902800076E+13;           // 月球引力常数 (m^3 / s^2)
const double GM_SUN             =  1.32712442099E+20;             // 太阳引力常数 (m^3 / s^2)
const double GM_MERCURY         =  1.32712442099E+20 / 6023597.400017; // 水星引力常数
const double GM_VENUS           =  1.32712442099E+20 / 408523.718655;  // 金星引力常数
const double GM_MARS            =  1.32712442099E+20 / 3098703.590267; // 火星引力常数
const double GM_JUPITER         =  1.32712442099E+20 / 1047.348625;  // 木星引力常数
const double GM_SATURN          =  1.32712442099E+20 / 3497.901768;    // 土星引力常数
const double GM_URANUS          =  1.32712442099E+20 / 22902.981613;   // 天王星引力常数
const double GM_NEPTUNE         =  1.32712442099E+20 / 19412.237346;   // 海王星引力常数
const double GM_PLUTO           =  1.32712442099E+20 / 135836683.767599; // 冥王星引力常数

#define MAX_PRN_GPS             100   // GPS卫星号的最大值
#define MAX_PRN_BD              100   // BD卫星号的最大值
#define MAX_PRN                 100   // 卫星号的最大值        
#define MAX_ID_STATION           100  // 测站号的最大值
#define NULL_PRN                255  // 无效卫星序号
#define MAX_ID_DORISSTATION      160  // DORIS测站最大标号

const double BD_FREQUENCE_L1     =  1561.098E+6;//B1I
const double BD_FREQUENCE_L2     =  1207.140E+6;//B3I
const double BD_FREQUENCE_L5     =  1268.520E+6;
const double BD_WAVELENGTH_L1    =  SPEED_LIGHT / BD_FREQUENCE_L1; 
const double BD_WAVELENGTH_L2    =  SPEED_LIGHT / BD_FREQUENCE_L2; 
const double BD_WAVELENGTH_L5    =  SPEED_LIGHT / BD_FREQUENCE_L5; 
const double BD_FREQUENCE_B1C    =  1575.42E+6;
const double BD_FREQUENCE_B2a    =  1176.45E+6;
//
const double GPS_FREQUENCE_L1    =  1575.42E+6;
const double GPS_FREQUENCE_L2    =  1227.6E+6;
const double GPS_WAVELENGTH_L1   =  SPEED_LIGHT /  GPS_FREQUENCE_L1;                     // L1波段载波波长
const double GPS_WAVELENGTH_L2   =  SPEED_LIGHT /  GPS_FREQUENCE_L2;                     // L2波段载波波长
const double GPS_WAVELENGTH_W    =  SPEED_LIGHT / (GPS_FREQUENCE_L1 - GPS_FREQUENCE_L2); // 宽巷载波波长
const double GPS_WAVELENGTH_N    =  SPEED_LIGHT / (GPS_FREQUENCE_L1 + GPS_FREQUENCE_L2); // 窄巷载波波长




