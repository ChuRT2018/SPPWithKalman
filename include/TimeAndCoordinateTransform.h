#pragma once
#ifndef _INCLUDES_TIMEANDCOORDINATETRANSFORM_
#define _INCLUDES_TIMEANDCOORDINATETRANSFORM_
#include <windows.h>
#include <iostream>


namespace  GPSCurriculumDesign {
#define POWE 0.00669437999013
#define ACCURACBLH2XYZ 1e-15
#define EARTHSEMIMAJORAXIS 6378137
#define PI 3.1415926535897932384626433832795



	/*通用时间*/
	typedef struct COMMONTIME
	{
		unsigned short Year;
		unsigned short Month;
		unsigned short Day;
		unsigned short Hour;
		unsigned short Minute;
		double Second;
	};


	/* GPS时间定义 */
	typedef struct GPSTIME
	{
		unsigned short Week;
		double
			SecOfWeek;
		GPSTIME()
		{
			Week = 0;
			SecOfWeek = 0.0;
		}
	};

	/* 简化儒略日 */
	typedef struct MJDTIME
	{
		int Days;
		double FracDay;
		MJDTIME()
		{
			Days = 0;
			FracDay = 0.0;
		}
	};

	/*笛卡尔坐标*/
	typedef struct XYZ {
		double x;
		double y;
		double z;
	};

	/*大地坐标*/
	typedef struct BLH {
		double longitude;
		double latitude;
		double height;
	};

	typedef struct ENU {
		double e;
		double n;
		double u;
	};

	/*
	** CommonTime to MJD transformation
	**
	** JD = INT[365.25 * Y] + INT[30.6001 * (M + 1)] + D + UT/24 + 1720981.5
	** MJD = JD - 2400000.5
	**     = INT[365.25 * Y] + INT[30.6001 * (M + 1)] + D + UT/24 - 679019
	** if M <= 2 : y = Y- 1,m = M + 12
	** if M > 2 : y = Y, m = M
	** y : year
	** M : month
	** D : day
	** UT: Universal time
	** INT[] : round numbers
	*/
	bool CommonTime2MjdTime(COMMONTIME commonTime, MJDTIME& mjdTime);



	/*
	** MJD to CommonTime transformation
	**
	** a = INT[JD + 0.5]
	** b = a + 1537
	** c = INT[(b - 122.1) / 365.25]
	** d = INT[365.25 * c]
	** e = INT[(b - d) / 30.6001]
	** D = b - d - INT[30.6001 * e] + FRAC[JD + 0.5]
	** M = e - 1 - 12 * INT[e / 14]
	** Y = c - 4715 - INT[(7 + M) / 10]
	** N = mod{INT[JD + 0.5],7}
	** ==> 0 = MONDAY 1 = TUESDAY 2 = WEDNESDAY 3 = THURSDAY
	** ==> 4 = FRIDAY 5 = SATURDAY 6 = SUNDAY
	** INT[] : round numbers
	*/
	bool MjdTime2CommonTimeTime(MJDTIME mjdTime, COMMONTIME& commonTime);



	/*
	** MJD to GPSTime transformation
	**
	** GPS WEEK = INT[(MJD - 44244) / 7]
	** GPS SECOND = (MJD - 44244 - GPSWEEK * 7) * 86400
	** INT[] : round numbers
	*/
	bool Mjd2GPSTime(MJDTIME mjdTime, GPSTIME& gpsTime);



	/*
	** GPSTime to MJD transformation
	**
	** MJD = 44244 + GPSWEEK * 7 + SECONDOFWEEK / 86400
	*/
	bool GPSTime2Mjd(GPSTIME gpsTime, MJDTIME& mjdTime);



	/*
	** GPSTime to CommonTime transformation
	**
	** GPSTime ==> MJD ==> CommonTime
	*/
	bool GPSTime2CommnoTime(GPSTIME gpsTime, COMMONTIME& commonTime);



	/*
	** CommonTime to GPSTime transformation
	**
	** CommonTime ==> MJD ==> GPSTime
	*/
	bool CommonTime2GPSTime(COMMONTIME commonTime, GPSTIME& gpsTime);



	/*
	** BLH to XYZ transform
	**
	** X = (N + H) * cosB * cosL
	** Y = (N + H) * cosB * sinL
	** Z = [N*(1 + e^2) + H] * sinB = [N * b^2 / a^2 + H] * sinB
	** where N = a / sqrt(1 - e^2 * sinB^2)
	**       第一偏心率平方e^2 =0.00669437999013
	*/
	bool BLH2XYZ(BLH blh, XYZ& xyz);



	/*
	** XYZ2BLH
	**
	** LOOP:
	** L = atan(Y / X);
	** B = atan((Z + deltaZ) / sqrt(X^2 + Y^2))
	** H = sqrt(X^2 + Y^2 + (Z + deltaZ)^2) - N
	** where:
	** sinB = (Z + deltaZ) / sqrt(X^2 + Y^2 + (Z + deltaZ)^2)
	** N = a / sqrt(1 - e^2 * sinB ^2)
	** detlaZ = N * e^2 * sinB
	** initial detlaZ with e^2 * Z
	*/
	bool XYZ2BLH(XYZ xyz, BLH& blh);


	bool XYZ2ENU(XYZ originXYZ, XYZ pendingXYZ, ENU& enu);
}
#endif
