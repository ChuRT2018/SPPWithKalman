// TimeAndCoordinateTransform.cpp : 定义控制台应用程序的入口点。
//

#include "../include/TimeAndCoordinateTransform.h"


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
bool GPSCurriculumDesign::CommonTime2MjdTime(COMMONTIME commonTime, MJDTIME & mjdTime)
{
	if (commonTime.Month <= 2) {
		commonTime.Year--;
		commonTime.Month += 12;
	}
	double day = (int)(365.25 * commonTime.Year)
		+ (int)(30.6001 * (commonTime.Month + 1)) + commonTime.Day + commonTime.Hour / 24.0 - 679019;
	mjdTime.Days = (int)day;
	mjdTime.FracDay = day - (int)day;
	return true;
}


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
bool GPSCurriculumDesign::MjdTime2CommonTimeTime(MJDTIME mjdTime, COMMONTIME & commonTime)
{
	double JD = mjdTime.Days + mjdTime.FracDay + 2400000.5;
	int a = (int)(JD + 0.5);
	int b = a + 1537;
	int c = (int)((b - 122.1) / 365.25);
	int d = (int)(365.25  * c);
	int e = (int)((b - d) / 30.6001);

	commonTime.Day = b - d - (int)(e * 30.6001);
	commonTime.Month = e - 1 - 12 * (int)(e / 14);
	commonTime.Year = c - 4715 - (int)((7.0f + commonTime.Month) / 10);
	commonTime.Hour = (int)(((JD + 0.5) - a) * 24);
	double minute = (((JD + 0.5) - a) * 24 - (int)(((JD + 0.5) - a) * 24)) * 60;
	commonTime.Minute = (int)minute;
	commonTime.Second = (minute - (int)minute) * 60;
	return true;
}


/*
** MJD to GPSTime transformation
**
** GPS WEEK = INT[(MJD - 44244) / 7]
** GPS SECOND = (MJD - 44244 - GPSWEEK * 7) * 86400
** INT[] : round numbers
*/
bool GPSCurriculumDesign::Mjd2GPSTime(MJDTIME mjdTime, GPSTIME & gpsTime)
{

	gpsTime.Week = (int)((mjdTime.Days + mjdTime.FracDay - 44244) / 7);
	gpsTime.SecOfWeek = (mjdTime.Days + mjdTime.FracDay - 44244 - gpsTime.Week * 7) * 86400;
	return true;
}


/*
** GPSTime to MJD transformation
**
** MJD = 44244 + GPSWEEK * 7 + SECONDOFWEEK / 86400
*/
bool GPSCurriculumDesign::GPSTime2Mjd(GPSTIME gpsTime, MJDTIME & mjdTime)
{
	double mjd = 44244 + gpsTime.Week * 7 + gpsTime.SecOfWeek / 86400;
	mjdTime.Days = (int)mjd;
	mjdTime.FracDay = mjd - (int)mjd;
	return true;
}


/*
** GPSTime to CommonTime transformation
**
** GPSTime ==> MJD ==> CommonTime
*/
bool GPSCurriculumDesign::GPSTime2CommnoTime(GPSTIME gpsTime, COMMONTIME & commonTime)
{
	MJDTIME mjd;
	GPSTime2Mjd(gpsTime, mjd);
	MjdTime2CommonTimeTime(mjd, commonTime);
	return true;
}


/*
** CommonTime to GPSTime transformation
**
** CommonTime ==> MJD ==> GPSTime
*/
bool GPSCurriculumDesign::CommonTime2GPSTime(COMMONTIME commonTime, GPSTIME & gpsTime)
{
	MJDTIME mjd;
	CommonTime2MjdTime(commonTime, mjd);
	Mjd2GPSTime(mjd, gpsTime);
	return true;
}


/*
** BLH to XYZ transform
**
** X = (N + H) * cosB * cosL
** Y = (N + H) * cosB * sinL
** Z = [N * (1 + e^2) + H] * sinB = [N * b^2 / a^2 + H] * sinB
** where N = a / sqrt(1 - e^2 * sinB^2)
**       第一偏心率平方e^2 =0.00669437999013
*/
bool GPSCurriculumDesign::BLH2XYZ(BLH blh, XYZ& xyz) {
	double sinB = sin(blh.longitude);
	double cosB = cos(blh.longitude);
	double sinL = sin(blh.latitude);
	double cosL = cos(blh.latitude);
	double N = 1.0 * EARTHSEMIMAJORAXIS / sqrt(1 - POWE * sinB * sinB);
	
	xyz.x = (N + blh.height) * cosB * cosL;
	xyz.y = (N + blh.height) * cosB * sinL;
	xyz.z = (N * (1 + POWE) + blh.height) * sinB;
	return true;
}


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
bool GPSCurriculumDesign::XYZ2BLH(XYZ xyz, BLH& blh) {
	if (xyz.x == 0) {
		blh = { 0 , 0 , 0 };
		return false;
	}
	blh.longitude = atan(xyz.y / xyz.x);
	const double x = xyz.x;
	const double y = xyz.y;
	const double z = xyz.z;
	const double distanceXY = sqrt(x * x + y * y);
	double B1 = 0;
	double B = atan(z / distanceXY);
	double sinB = 0;
	double n = 0;
	int loopCount = 0;
	while (fabs(B - B1) > ACCURACBLH2XYZ && loopCount < 10) {
		
		B1 = B;
		sinB = sin(B1);
		n = EARTHSEMIMAJORAXIS / sqrt(1 - POWE * sinB * sinB);
		B = atan((z + n * POWE * sinB) / distanceXY);
		
	}

	blh.latitude = B;
	blh.height = distanceXY / cos(B) - n;
	//double detlaZ = POWE * xyz.z;
	//double lastDetlaZ = 0.0;
	//
	//double distanceXYZ = sqrt(x * x + y * y + (z + detlaZ) * (z + detlaZ));
	//double sinB = (z + detlaZ) / distanceXYZ;
	//double n = EARTHSEMIMAJORAXIS / sqrt(1 - POWE * sinB * sinB);
	//const double distanceXY = sqrt(x * x + y * y);
	//int loopcount = 0;
	//while (loopcount < 15 && abs(detlaZ - lastDetlaZ) >= ACCURACBLH2XYZ) {
	//	blh.longitude = atan((xyz.z + detlaZ) / distanceXY);
	//	blh.height = distanceXYZ - n;
	//	
	//	lastDetlaZ = detlaZ;

	//	detlaZ = n * POWE * sinB;
	//	distanceXYZ = sqrt(x * x + y * y + (z + detlaZ) * (z + detlaZ));
	//	sinB = (z + detlaZ) / distanceXYZ;
	//	n = n = EARTHSEMIMAJORAXIS / sqrt(1 - POWE * sinB * sinB);
	//	loopcount++;
	//}

	////blh.latitude = blh.latitude * 180 / PI;
	////blh.longitude = blh.longitude * 180 / PI;
	
	return true;
}

bool GPSCurriculumDesign::XYZ2ENU(XYZ originXYZ, XYZ pendingXYZ, ENU & enu)
{
	double deltaX = pendingXYZ.x - originXYZ.x;
	double deltaY = pendingXYZ.y - originXYZ.y;
	double deltaZ = pendingXYZ.z - originXYZ.z;

	BLH originBLH;
	XYZ2BLH(originXYZ, originBLH);
	
	if (originBLH.longitude < 0)
		originBLH.longitude += PI;

	double sinB = sin(originBLH.latitude);
	double cosB = cos(originBLH.latitude);
	double sinL = sin(originBLH.longitude);
	double cosL = cos(originBLH.longitude);
	enu.n = -sinB * cosL * deltaX - sinB * sinL * deltaY + cosB * deltaZ;
	enu.e = -sinL * deltaX + cosL * deltaY;
	enu.u = cosB * cosL * deltaX + cosB * sinL * deltaY + sinB * deltaZ;
	return true;
}
