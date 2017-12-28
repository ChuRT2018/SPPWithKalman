#include "../include/includes.h"
using namespace GPSCurriculumDesign;


double hW = 11000;

bool GPSCurriculumDesign::TimeCorrect(
	gpsephem gpsephem_,
	const double t,
	const unsigned long week,
	const double ek,
	double& correctedT)
{
	double af0 = gpsephem_.af0;
	double af1 = gpsephem_.af1;
	double af2 = gpsephem_.af2;

	double deltaTr = FT * gpsephem_.ecc * sqrt(gpsephem_.A) * sin(ek);
	double deltaT = 7 * 86400 * (week - gpsephem_.week) + t - gpsephem_.toc;

	if (deltaT > 302400.0)
		deltaT -= 86400.0 * 7;
	else if (deltaT < -302400.0)
		deltaT += 86400.0 * 7;

	correctedT = af0 + af1 * deltaT
		+ af2 * deltaT * deltaT + deltaTr - gpsephem_.tgd;
	return true;
}

bool GPSCurriculumDesign::Klobutchar(
	ionutc ionutc_,
	const double GPSt,
	const double latitude,
	const double longitude, const double azimuth,
	const double angle, double &correct)
{
	double latitude_ = latitude / PI;
	double longitude_ = longitude / PI;
	double azimuth_ = azimuth;
	double angle_ = angle / PI;
	double psi = 0.0137 / (angle_ + 0.11) - 0.022;
	double phiI = latitude_ + psi * cos(azimuth_);
	phiI = phiI > 0.416 ? 0.416 :
		phiI < -0.416 ? -0.416 : phiI;

	double lambdaI = longitude_ + psi * sin(azimuth_) / cos(phiI * PI);
	double phiM = phiI + 0.064 * cos(/*lambdaI * PI - 1.617*/(lambdaI - 1.617) * PI);
	double t = 43200 * lambdaI + GPSt;


	while (t > 86400) {
		t -= 86400;
	}
	while (t < 0) {
		t += 86400;
	}

	double aI = ionutc_.a0 + ionutc_.a1 * phiM + ionutc_.a2 * phiM * phiM + ionutc_.a3 * phiM * phiM * phiM;
	aI = aI < 0 ? 0 : aI;
	double pI = ionutc_.b0 + ionutc_.b1 * phiM + ionutc_.b2 * phiM * phiM + ionutc_.b3 * phiM * phiM * phiM;
	pI = pI < 72000 ? 72000 : pI;

	double xI = 2 * PI * (t - 50400) / pI;
	double xI2 = xI * xI;

	double f = 1 + 16 * (0.53 - angle_) * (0.53 - angle_) * (0.53 - angle_);

	if (abs(xI) <= 1.57) {
		correct = ((double)5 * 1e-9 + aI * (1 - xI2 / 2 + xI2 * xI2 / 24)) * f;
	}
	else {
		correct = (double)5 * 1e-9 * f;
	}
	correct *= SPEEDOFLIGHT;
	return true;
}

bool GPSCurriculumDesign::Hopefield(
	const double h,
	const double angle,
	double &correct)
{
	if (abs(h) > 40000) { correct = 0; return false; }
	double p = P0 * pow((1 - 0.0000226 * (h - H0)), 5.225);
	double temperatur = T0 - 0.0065 * (h - H0);
	double hD = 40136 + 148.72 * (temperatur - 273.16);
	double kD = 155.2 * 1e-7 * p * (hD - h) / temperatur;
	double rh = RH0 * exp(-0.0006396 * (h - H0));
	double eInThisFun = rh * exp(-37.2465 + 0.213166 * temperatur
		- 0.000256908 * temperatur * temperatur);

	double kW = 155.2 * 1e-7 * 4810 * eInThisFun * (hW - h) / temperatur / temperatur;

	double angle1 = sqrt(angle * angle + 6.25);
	double angle2 = sqrt(angle * angle + 2.25);

	correct = kD / sin(angle1 * PI / 180) + kW / sin(angle2 * PI / 180);


	return true;
}

bool GPSCurriculumDesign::SetTimeDelta(double& newSecond, unsigned long& newWeek, const double oldSecond, const unsigned long oldWeek, const double timedelta)
{
	double secondDelta = oldSecond - timedelta;
	if (secondDelta < 0) {
		newWeek = oldWeek - 1;
		newSecond = secondDelta + 7 * 86400;
	}
	else if (secondDelta > 86400 * 7) {
		newWeek = oldWeek + 1;
		newSecond = secondDelta - 7 * 86400;
	}
	else {
		newWeek = oldWeek;
		newSecond = secondDelta;
	}
	return true;
}


bool GPSCurriculumDesign::CalculateOrbitalParameter(const unsigned long GPSWeek, const double GPSSecond, const gpsephem gpsephem_, orbitalParameter& orbitalParameter_)
{

	double deltaTime = 7 * 86400 * (GPSWeek - gpsephem_.week)
		+ GPSSecond - gpsephem_.toe;

	if (abs(deltaTime) > 2 * 3600) {
		return false;
	}
	double e = gpsephem_.ecc;
	double A = gpsephem_.A/* * it->second.A*/;
	double n0 = sqrt(GM / A / A / A);
	double tk = deltaTime;
	/*tk = tk > 302400 ? tk - 604800 :
		tk < -302400 ? tk + 604800 : tk;*/
	double n = n0 + gpsephem_.deltaN;
	double mk = gpsephem_.M0 + n * tk;
	double ek = mk;
	double deltaEk = 100;
	while (abs(deltaEk) > DELTAEK) {
		deltaEk = mk + e * sin(ek) - ek;
		ek = ek + deltaEk;
	}

	double sEk = sin(ek);
	double cEk = cos(ek);

	double cVk = cos(ek) - e;
	double sVk = sqrt(1 - e * e) * sin(ek);
	double vk = atan2(sVk, cVk);
	//double vk = atan2(sqrt(1 - e) * sEk, (cEk - e));

	//ek = acos((e + cos(vk)) / (1 + e * cos(vk)));


	double phiK = vk + gpsephem_.omega;

	double s2phiK = sin(2 * phiK);
	double c2phiK = cos(2 * phiK);

	double deltaUk = gpsephem_.cus * s2phiK + gpsephem_.cuc * c2phiK;
	double deltaRk = gpsephem_.crs * s2phiK + gpsephem_.crc * c2phiK;
	double deltaIk = gpsephem_.cis * s2phiK + gpsephem_.cic * c2phiK;

	double uK = phiK + deltaUk;
	double rK = A * (1 - e * cEk) + deltaRk;
	double iK = gpsephem_.i0 + deltaIk + gpsephem_.dotI * tk;


	double omegaK = gpsephem_.omega0 +
		(gpsephem_.dotOmega - OMEGAE) * tk -
		OMEGAE * gpsephem_.toe;

	orbitalParameter_.e = e;
	orbitalParameter_.n = n;
	orbitalParameter_.ek = ek;
	orbitalParameter_.vk = vk;
	orbitalParameter_.phiK = phiK;
	orbitalParameter_.uk = uK;
	orbitalParameter_.rk = rK;
	orbitalParameter_.ik = iK;
	orbitalParameter_.omegaK = omegaK;

	return true;
}

bool GPSCurriculumDesign::SatellitePositionAndVelocity(
	satellitePandV& satellitePandV_, double& correctT, double& clockVelo, 
	const unsigned long GPSWeek,
	const double GPSSecond,
	const gpsephem gpsephem_
) {

	orbitalParameter orbitalParameter_;
	double deltaTimeCorrect = 100;
	double second = GPSSecond;
	unsigned long week = GPSWeek;
	double lastTimeCorrect = 0;
	double currentTimeCorrect = 0;
	while (deltaTimeCorrect > 1e-9)
	{		
		if (!CalculateOrbitalParameter(week, second, gpsephem_, orbitalParameter_)) {
			return false;
		}

		TimeCorrect(gpsephem_, second, week, orbitalParameter_.ek, currentTimeCorrect);
		SetTimeDelta(
			second,
			week,
			GPSSecond, GPSWeek,
			currentTimeCorrect);
		deltaTimeCorrect = abs(currentTimeCorrect - lastTimeCorrect);
		lastTimeCorrect = currentTimeCorrect;
	}

	correctT = currentTimeCorrect;
	if (!CalculateOrbitalParameter(
		week,second,
		gpsephem_, orbitalParameter_))
	{
		return false;
	}


	double x = orbitalParameter_.rk * cos(orbitalParameter_.uk);
	double y = orbitalParameter_.rk * sin(orbitalParameter_.uk);


	double cOmegak = cos(orbitalParameter_.omegaK);
	double sOmegak = sin(orbitalParameter_.omegaK);
	double sIk = sin(orbitalParameter_.ik);
	double cIk = cos(orbitalParameter_.ik);

	satellitePandV_.x = x * cOmegak - y * cIk * sOmegak;
	satellitePandV_.y = x * sOmegak + y * cIk * cOmegak;
	satellitePandV_.z = y * sIk;

	double e = orbitalParameter_.e;
	double cVk = cos(orbitalParameter_.vk / 2);
	double cEk = cos(orbitalParameter_.ek / 2);
	double c2phiK = cos(2 * orbitalParameter_.phiK);
	double s2phiK = sin(2 * orbitalParameter_.phiK);
	double dotEk = orbitalParameter_.n / (1 - e * cos(orbitalParameter_.ek));
	double dotphiK = sqrt((1 + e) / (1 - e)) * cVk * cVk / cEk / cEk * dotEk;
	double dotUk = 2 * (gpsephem_.cus * c2phiK - gpsephem_.cuc * s2phiK) * dotphiK + dotphiK;
	double dotRk = gpsephem_.A * orbitalParameter_.e * sin(orbitalParameter_.ek) * dotEk + 2 * (gpsephem_.crs * c2phiK - gpsephem_.crc * s2phiK) * dotphiK;
	double dotIk = gpsephem_.dotI + 2 * (gpsephem_.cis * c2phiK - gpsephem_.cic * s2phiK) * dotphiK;
	double dotOmegaK = gpsephem_.dotOmega - OMEGAE;

	double dotXk = dotRk * cos(orbitalParameter_.uk) - orbitalParameter_.rk * dotUk * sin(orbitalParameter_.uk);
	double dotYk = dotRk * sin(orbitalParameter_.uk) + orbitalParameter_.rk * dotUk * cos(orbitalParameter_.uk);

	satellitePandV_.vx =
		cOmegak * dotXk - sOmegak * cIk * dotYk -
		(x * sOmegak + y * cOmegak * cIk) * dotOmegaK + y * sOmegak * sIk * dotIk;

	satellitePandV_.vy =
		sOmegak * dotXk + cOmegak * cIk * dotYk +
		(x * cOmegak - y * sOmegak * cIk) * dotOmegaK + y * cOmegak * sIk * dotIk;

	satellitePandV_.vz = sIk * dotYk + y * cIk * dotIk;

	clockVelo = gpsephem_.af1 +
		gpsephem_.af2 *(7 * 86400 * (week - gpsephem_.week) + second - gpsephem_.toc) +
		FT * sqrt(gpsephem_.A)*cos(orbitalParameter_.e) * dotEk;

	return true;
}

/*
*  calculate position with least square 
*/
bool GPSCurriculumDesign::ObservationPositionAndVelocity(
	const ionutc ionutc_,
	std::unordered_map<unsigned long, gpsephem> gpsephems,
	std::vector<range> ranges,
	const unsigned long GPSWeek,
	const double GPSt,
	std::vector<satellitePandV>& satellitePandVs,
	observationPAndV& currentPAndV)
{

	const int satelliteNum = ranges.size();
	if (satelliteNum < 4) return false;
	XYZ originXYZ = { 0,0,0 };
	if (currentPAndV.x != 0 && currentPAndV.y != 0) {
		originXYZ.x = currentPAndV.x;
		originXYZ.y = currentPAndV.y;
		originXYZ.z = currentPAndV.z;
	}
	satellitePandV satellitePandV_;
	std::vector<satellitePandV> innerVectorOfSates;
	std::vector<double> psrs, timeCorrects, dopps,clockVelos;
	double deltaObs = 100000;
	int loopNum = 0;

	unsigned long GPSWeekAfterCorrect = GPSWeek;
	double GPSSecondAfterCorrect = GPSt;

	double satelliteX, satelliteY, satelliteZ, r;
	double satelliteVx, satelliteVy, satelliteVz;
	BLH obsBLH;
	XYZ2BLH(originXYZ, obsBLH);



	for (int i = 0; i < satelliteNum; i++) {

		double deltaTime = (ranges[i].psr - currentPAndV.ct) / SPEEDOFLIGHT;

		std::unordered_map<unsigned long, gpsephem>::iterator it
			= gpsephems.find(ranges[i].PRN);

		if (it == gpsephems.end()) {
			return false;
		}
		gpsephem gpsephem_ = it->second;

		SetTimeDelta(GPSSecondAfterCorrect, GPSWeekAfterCorrect,
			GPSt, GPSWeek, deltaTime);

		double timeCorrect = 0;
		double clockVelo = 0;
		if (!SatellitePositionAndVelocity(
			satellitePandV_, timeCorrect,clockVelo, GPSWeekAfterCorrect,
			GPSSecondAfterCorrect, gpsephem_)) {
			continue;
		}
		
		XYZ satelliteXYZ = { satellitePandV_.x,satellitePandV_.y, satellitePandV_.z };

		ENU enu;
		XYZ2ENU(originXYZ, satelliteXYZ, enu);
		double angle = atan2(enu.u, sqrt(enu.e * enu.e + enu.n * enu.n));
		if (angle < 0) angle += PI;
		angle = angle * 180 / PI;
		if(angle < 15) continue;
		//XYZ satelliteXYZ = { satellitePandV_.x,satellitePandV_.y, satellitePandV_.z };
		//BLH blh;
		//XYZ2BLH(satelliteXYZ, blh);
		//
		//ENU enu;
		//XYZ2ENU(originXYZ, satelliteXYZ, enu);
		//double angle = atan2(enu.u , sqrt(enu.e * enu.e + enu.n * enu.n));
		//if (angle < 0) angle += PI;
		//double azimuth = atan2(enu.e, enu.n);
		//
		//angle = angle * 180 / PI;
		//if(angle < 15) continue;
		//
		//Klobutchar(ionutc_, GPSt + currentPAndV.ct / SPEEDOFLIGHT,
		//	blh.latitude, blh.longitude, azimuth, angle * PI / 180, klobutcharCorrect);
		//Hopefield(obsBLH.height, angle, hopefieldCorrect);
		//
		//klobutcharCorrects.push_back(klobutcharCorrect * SPEEDOFLIGHT);
		//hopefieldCorrects.push_back(hopefieldCorrect);
		//
		//deltaTime = deltaTime - timeCorrect/* - currentPAndV.ct / SPEEDOFLIGHT*/;
		//
		//satellitePandV_.x = cos(deltaTime * OMEGAE) * satellitePandV_.x +
		//	sin(deltaTime * OMEGAE) * satellitePandV_.y;
		//satellitePandV_.y = cos(deltaTime * OMEGAE) * satellitePandV_.y
		//	- sin(deltaTime * OMEGAE) * satellitePandV_.x;
		//satellitePandV_.z = satellitePandV_.z;
		//satellitePandV_.PRN = ranges[i].PRN;

		innerVectorOfSates.push_back(satellitePandV_);
		psrs.push_back(ranges[i].psr);
		dopps.push_back(ranges[i].dopp);
		clockVelos.push_back(clockVelo);
		timeCorrects.push_back(timeCorrect);
	}


	int innerSatelliteNum = innerVectorOfSates.size();
	if (innerSatelliteNum < 4) return false;
	Matrix<double> b = Matrix<double>(innerSatelliteNum, 4);
	Matrix<double> bT = Matrix<double>(4, innerSatelliteNum);
	Matrix<double> w = Matrix<double>(innerSatelliteNum, 1);
	Matrix<double> wForV = Matrix<double>(innerSatelliteNum, 1);
	Matrix<double> xHat = Matrix<double>(4, 1);
	Matrix<double> bTbInv = Matrix<double>(4, 4);
	Matrix<double> obsV = Matrix<double>(4, 1);
	while (deltaObs > DELTAOBS && loopNum < 10) {
		for (int i = 0; i < innerSatelliteNum; i++) {

			satelliteX = innerVectorOfSates[i].x;
			satelliteY = innerVectorOfSates[i].y;
			satelliteZ = innerVectorOfSates[i].z;
			satelliteVx = innerVectorOfSates[i].vx;
			satelliteVy = innerVectorOfSates[i].vy;
			satelliteVz = innerVectorOfSates[i].vz;
			//double deltaTime = (r +currentPAndV.ct) / SPEEDOFLIGHT ;
			double deltaTime = (psrs[i] - currentPAndV.ct) / SPEEDOFLIGHT;

			satelliteX = cos(deltaTime * OMEGAE) * satelliteX +
				sin(deltaTime * OMEGAE) * satelliteY;
			satelliteY = cos(deltaTime * OMEGAE) * satelliteY
				- sin(deltaTime * OMEGAE) * satelliteX;
			
			XYZ satelliteXYZ = { satelliteX,satelliteY, satelliteZ };
			BLH blh;
			XYZ2BLH(satelliteXYZ, blh);
			if (blh.longitude < 0){
				blh.longitude += PI;
			}
			if (blh.latitude < 0) {
				blh.latitude += PI;
			}

			ENU enu;
			XYZ2ENU(originXYZ, satelliteXYZ, enu);
			double angle = atan2(enu.u, sqrt(enu.e * enu.e + enu.n * enu.n));
			if (angle < 0) angle += PI;
			double azimuth = atan2(enu.e, enu.n);

			angle = angle * 180 / PI;
			double klobutcharCorrect = 0;
			double hopefieldCorrect = 0;
			Klobutchar(ionutc_, GPSt /*- currentPAndV.ct / SPEEDOFLIGHT*/,
				blh.latitude, blh.longitude, azimuth, angle * PI / 180, klobutcharCorrect);
			Hopefield(obsBLH.height, angle, hopefieldCorrect);

			r = sqrt(
				(satelliteX - originXYZ.x) * (satelliteX - originXYZ.x) +
				(satelliteY - originXYZ.y) * (satelliteY - originXYZ.y) +
				(satelliteZ - originXYZ.z) * (satelliteZ - originXYZ.z)
			);

			b.ChangeElement(i, 0, (originXYZ.x - satelliteX) / r);
			b.ChangeElement(i, 1, (originXYZ.y - satelliteY) / r);
			b.ChangeElement(i, 2, (originXYZ.z - satelliteZ) / r);
			b.ChangeElement(i, 3, 1);

			double wi = psrs[i] - r - currentPAndV.ct +
				timeCorrects[i] * SPEEDOFLIGHT
				- klobutcharCorrect - hopefieldCorrect;
			
			double d = (satelliteX - originXYZ.x) * satelliteVx +
				(satelliteY - originXYZ.y) * satelliteVy +
				(satelliteZ - originXYZ.z) * satelliteVz;
			
			double wForVi = -dopps[i] * CDIVIDEF1 - d / r + clockVelos[i] * SPEEDOFLIGHT;
			w.ChangeElement(i, 0, wi);
			wForV.ChangeElement(i, 0, wForVi);
		}
		bT = b.t();
		bTbInv = (bT * b).Inv();
		xHat = bTbInv * bT * w;

		double vx = xHat.GetElement(0, 0);
		double vy = xHat.GetElement(1, 0);
		double vz = xHat.GetElement(2, 0);

		currentPAndV.x += vx;
		currentPAndV.y += vy;
		currentPAndV.z += vz;
		currentPAndV.ct += xHat.GetElement(3, 0);

		originXYZ.x = currentPAndV.x;
		originXYZ.y = currentPAndV.y;
		originXYZ.z = currentPAndV.z;

		//XYZ2BLH(originXYZ, obsBLH);

		deltaObs = vx * vx + vy * vy + vz * vz;
		loopNum++;
		
	}
	
	if (loopNum >= 10) {
		return false;
	}
	currentPAndV.pdop = sqrt(bTbInv.GetElement(0, 0) + 
		bTbInv.GetElement(1, 1) +
		bTbInv.GetElement(2, 2));

	obsV = bTbInv * bT * wForV;
	currentPAndV.vx = obsV.GetElement(0, 0);
	currentPAndV.vy = obsV.GetElement(1, 0);
	currentPAndV.vz = obsV.GetElement(2, 0);
	//Matrix<double> v = Matrix<double>(innerSatelliteNum, 1);
	//Matrix<double> vTv = v.t() * v;

	//double sigma = sqrt(vTv.GetElement(1, 1) / (innerSatelliteNum - 4));


	

	return true;
}

/*
* calculate position with kalman
*/
bool GPSCurriculumDesign::ObservationPositionAndVelocityKalman(
	const ionutc ionutc_,
	std::unordered_map<unsigned long, gpsephem> gpsephems,
	std::vector<range> ranges,
	const unsigned long GPSWeek,
	const double GPSt,
	std::vector<satellitePandV>& satellitePandVs,
	observationPAndV& currentPAndV){

	const int satelliteNum = ranges.size();
	if (satelliteNum < 4) return false;
	XYZ originXYZ = { 0,0,0 };
	if (currentPAndV.x != 0 && currentPAndV.y != 0) {
		originXYZ.x = currentPAndV.x;
		originXYZ.y = currentPAndV.y;
		originXYZ.z = currentPAndV.z;
	}
	satellitePandV satellitePandV_;
	std::vector<satellitePandV> innerVectorOfSates;
	std::vector<double> psrs, timeCorrects, dopps, clockVelos;
	double deltaObs = 100000;
	int loopNum = 0;

	unsigned long GPSWeekAfterCorrect = GPSWeek;
	double GPSSecondAfterCorrect = GPSt;

	double satelliteX, satelliteY, satelliteZ, r;
	BLH obsBLH;
	XYZ2BLH(originXYZ, obsBLH);



	for (int i = 0; i < satelliteNum; i++) {

		double deltaTime = (ranges[i].psr - currentPAndV.ct) / SPEEDOFLIGHT;

		std::unordered_map<unsigned long, gpsephem>::iterator it
			= gpsephems.find(ranges[i].PRN);

		if (it == gpsephems.end()) {
			return false;
		}
		gpsephem gpsephem_ = it->second;

		SetTimeDelta(GPSSecondAfterCorrect, GPSWeekAfterCorrect,
			GPSt, GPSWeek, deltaTime);

		double timeCorrect = 0;
		double clockVelo = 0;
		if (!SatellitePositionAndVelocity(
			satellitePandV_, timeCorrect, clockVelo, GPSWeekAfterCorrect,
			GPSSecondAfterCorrect, gpsephem_)) {
			continue;
		}

		XYZ satelliteXYZ = { satellitePandV_.x,satellitePandV_.y, satellitePandV_.z };

		ENU enu;
		XYZ2ENU(originXYZ, satelliteXYZ, enu);
		double angle = atan2(enu.u, sqrt(enu.e * enu.e + enu.n * enu.n));
		if (angle < 0) angle += PI;
		angle = angle * 180 / PI;
		if (angle < 15) continue;

		innerVectorOfSates.push_back(satellitePandV_);
		psrs.push_back(ranges[i].psr);
		dopps.push_back(ranges[i].dopp);
		clockVelos.push_back(clockVelo);
		timeCorrects.push_back(timeCorrect);
	}


	int innerSatelliteNum = innerVectorOfSates.size();
	if (innerSatelliteNum < 4) return false;
	//innerSatelliteNum = 3;
	Matrix<double> b = Matrix<double>(innerSatelliteNum, 4);
	Matrix<double> bT = Matrix<double>(4, innerSatelliteNum);
	Matrix<double> KB = Matrix<double>(4, 4);
	Matrix<double> w = Matrix<double>(innerSatelliteNum, 1);
	Matrix<double> KW = Matrix<double>(4, 1);
	Matrix<double> K = Matrix<double>(4,innerSatelliteNum);
	Matrix<double> I = Matrix<double>(4, 4);
	I.ChangeElement(0, 0, 1);
	I.ChangeElement(1, 1, 1);
	I.ChangeElement(2, 2, 1);
	I.ChangeElement(3, 3, 1);
	Matrix<double> Ddetla = Matrix<double>(innerSatelliteNum, innerSatelliteNum);
	
	/*Matrix<double> wForV = Matrix<double>(innerSatelliteNum, 1);
	Matrix<double> obsV = Matrix<double>(4, 1);*/
	//while (deltaObs > DELTAOBS && loopNum++ < 10) {
		for (int i = 0; i < innerSatelliteNum; i++) {
			Ddetla.ChangeElement(i, i, 5);
			satelliteX = innerVectorOfSates[i].x;
			satelliteY = innerVectorOfSates[i].y;
			satelliteZ = innerVectorOfSates[i].z;
			double deltaTime = (psrs[i] - currentPAndV.ct) / SPEEDOFLIGHT;

			satelliteX = cos(deltaTime * OMEGAE) * satelliteX +
				sin(deltaTime * OMEGAE) * satelliteY;
			satelliteY = cos(deltaTime * OMEGAE) * satelliteY
				- sin(deltaTime * OMEGAE) * satelliteX;

			XYZ satelliteXYZ = { satelliteX,satelliteY, satelliteZ };
			BLH blh;
			XYZ2BLH(satelliteXYZ, blh);
			if (blh.longitude < 0) {
				blh.longitude += PI;
			}
			if (blh.latitude < 0) {
				blh.latitude += PI;
			}

			ENU enu;
			XYZ2ENU(originXYZ, satelliteXYZ, enu);
			double angle = atan2(enu.u, sqrt(enu.e * enu.e + enu.n * enu.n));
			if (angle < 0) angle += PI;
			double azimuth = atan2(enu.e, enu.n);

			angle = angle * 180 / PI;
			double klobutcharCorrect = 0;
			double hopefieldCorrect = 0;
			Klobutchar(ionutc_, GPSt /*- currentPAndV.ct / SPEEDOFLIGHT*/,
				blh.latitude, blh.longitude, azimuth, angle * PI / 180, klobutcharCorrect);
			Hopefield(obsBLH.height, angle, hopefieldCorrect);

			r = sqrt(
				(satelliteX - originXYZ.x) * (satelliteX - originXYZ.x) +
				(satelliteY - originXYZ.y) * (satelliteY - originXYZ.y) +
				(satelliteZ - originXYZ.z) * (satelliteZ - originXYZ.z)
			);

			b.ChangeElement(i, 0, (originXYZ.x - satelliteX) / r);
			b.ChangeElement(i, 1, (originXYZ.y - satelliteY) / r);
			b.ChangeElement(i, 2, (originXYZ.z - satelliteZ) / r);
			b.ChangeElement(i, 3, 1);

			/*double wi = psrs[i] - r - currentPAndV.ct +
				timeCorrects[i] * SPEEDOFLIGHT
				- klobutcharCorrect - hopefieldCorrect;*/
			double wi = psrs[i] - r +
				timeCorrects[i] * SPEEDOFLIGHT
				- klobutcharCorrect - hopefieldCorrect
				- currentPAndV.ct;
			w.ChangeElement(i, 0, wi);
		}
		
		
		bT = b.t();

		K = D * bT * (((b * D * bT) + Ddetla)).Inv();
		
		KB = K * b;
		KW =  K * w;
		
		//deltaObs = vx * vx + vy * vy + vz * vz;

		currentPAndV.x += KW.GetElement(0, 0);
		currentPAndV.y += KW.GetElement(1, 0);
		currentPAndV.z += KW.GetElement(2, 0);
		currentPAndV.ct += KW.GetElement(3, 0);

		D = (I - KB) * D;
	//}

	

	currentPAndV.pdop = 0;
	currentPAndV.vx = 0;
	currentPAndV.vy = 0;
	currentPAndV.vz = 0;
	return true;
	
}

//bool GPSCurriculumDesign::ObservationPositionAndVelocity(
//	const ionutc ionutc_,
//	std::unordered_map<unsigned long, gpsephem> gpsephems,
//	std::vector<range> ranges,
//	const unsigned long GPSWeek,
//	const double GPSt,
//	std::vector<satellitePandV>& satellitePandVs,
//	observationPAndV& currentPAndV)
//{
//	double klobutcharCorrect = 0;
//	double hopefieldCorrect = 0;
//	double timeCorrect = 0;
//	currentPAndV.ct = 0;
//	const int satelliteNum = ranges.size();
//	if (satelliteNum < 4) return false;
//	XYZ originXYZ = { 0,0,0 };
//	if (currentPAndV.x != 0 && currentPAndV.y != 0) {
//		originXYZ.x = currentPAndV.x;
//		originXYZ.y = currentPAndV.y;
//		originXYZ.z = currentPAndV.z;
//	}
//	satellitePandV satellitePandV_;
//	std::vector<satellitePandV> innerVectorOfSates;
//	std::vector<double> psrS;
//	double deltaObs = 100000;
//	int loopNum = 0;
//
//	double deltaTimeLast = 0, deltaTimeCurrent;
//	unsigned long GPSWeekAfterCorrect = GPSWeek;
//	double GPSSecondAfterCorrect = GPSt;
//
//	double satelliteX, satelliteY, satelliteZ, r;
//	BLH obsBLH = { 0, 0, 0 };
//	
//	{
//		
//		for (int i = 0; i < satelliteNum; i++) {
//			
//			double deltaTime = ranges[i].psr / SPEEDOFLIGHT;
//			
//			std::unordered_map<unsigned long, gpsephem>::iterator it
//				= gpsephems.find(ranges[i].PRN);
//
//			if (it == gpsephems.end()) {
//				return false;
//			}
//			gpsephem gpsephem_ = it->second;
//			
//			SetTimeDelta(GPSSecondAfterCorrect, GPSWeekAfterCorrect,
//				GPSt, GPSWeek,-deltaTime);
//			if (!SatellitePositionAndVelocity(
//				satellitePandV_, timeCorrect, GPSWeekAfterCorrect,
//				GPSSecondAfterCorrect, gpsephem_)) {
//				continue;
//			}
//
//			satellitePandV_.x = satellitePandV_.x +
//				(deltaTimeCurrent + timeCorrect) * OMEGAE * satellitePandV_.y;
//			satellitePandV_.y = satellitePandV_.y
//				- (deltaTimeCurrent + timeCorrect)  * OMEGAE * satellitePandV_.x;
//			satellitePandV_.z = satellitePandV_.z;
//			satellitePandV_.PRN = ranges[i].PRN;
//
//			innerVectorOfSates.push_back(satellitePandV_);
//			psrS.push_back(ranges[i].psr);
//		}
//	}
//	
//	
//	while (deltaObs > DELTAOBS && loopNum < 10) {
//		innerVectorOfSates.clear();
//		psrS.clear();
//
//
//		for (int i = 0; i < satelliteNum; i++) {
//			GPSWeekAfterCorrect = GPSWeek;
//			GPSSecondAfterCorrect = GPSt;
//			timeCorrect = 0;
//			bool originSatePosFlag = true;
//			XYZ originSatePos = { 0, 0, 0 };
//			int loopInner = 0;
//
//			deltaTimeCurrent = 0.075 + currentPAndV.ct;
//			deltaTimeLast = 0;
//
//			
//			while (abs(deltaTimeCurrent - deltaTimeLast) * SPEEDOFLIGHT > 1 && loopInner < 10)
//			{
//				loopInner++;
//				deltaTimeLast = deltaTimeCurrent;
//				
//				SetTimeDelta(GPSSecondAfterCorrect, GPSWeekAfterCorrect,
//					GPSt, GPSWeek, -deltaTimeLast);
//
//
//				if (!SatellitePositionAndVelocity(
//					satellitePandV_, timeCorrect,
//					GPSWeekAfterCorrect, GPSSecondAfterCorrect,
//					gpsephem_)) {
//					continue;
//				}
//
//				r = sqrt((satellitePandV_.x - originXYZ.x) * (satellitePandV_.x - originXYZ.x) +
//					(satellitePandV_.y - originXYZ.y) * (satellitePandV_.y - originXYZ.y) +
//					(satellitePandV_.y - originXYZ.y) * (satellitePandV_.y - originXYZ.y));
//
//				deltaTimeCurrent = r / SPEEDOFLIGHT + currentPAndV.ct;
//			}
//
//			if (loopInner >= 15) {
//				continue;
//			}
//
//			/////correct time
//			//deltaTimeCurrent = ranges[i].psr / SPEEDOFLIGHT
//			//	+ currentPAndV.ct / SPEEDOFLIGHT;
//			//deltaTimeLast = 0;
//
//
//			//std::unordered_map<unsigned long, gpsephem>::iterator it
//			//	= gpsephems.find(ranges[i].PRN);
//
//			//if (it == gpsephems.end()) {
//			//	return false;
//			//}
//
//			//orbitalParameter orbitalParameter_;
//			//gpsephem gpsephem_ = it->second;
//			//SetTimeDelta(
//			//	GPSSecondAfterCorrect,
//			//	GPSWeekAfterCorrect,
//			//	GPSt, GPSWeek,
//			//	deltaTimeCurrent);
//
//			//if (!CalculateOrbitalParameter(
//			//	GPSWeekAfterCorrect,
//			//	GPSSecondAfterCorrect,
//			//	gpsephem_,
//			//	orbitalParameter_)) {
//			//	return false;
//			//}
//
//			//TimeCorrect(
//			//	gpsephem_, 
//			//	GPSSecondAfterCorrect, 
//			//	GPSWeekAfterCorrect, 
//			//	orbitalParameter_.ek, timeCorrect);
//
//			//while (abs(deltaTimeCurrent - deltaTimeLast) * SPEEDOFLIGHT > 1
//			//	&& loopInner < 15
//			//	)
//			//{
//			//	loopInner++;
//
//			//	deltaTimeLast = deltaTimeCurrent;
//
//			//	SetTimeDelta(GPSSecondAfterCorrect, GPSWeekAfterCorrect,
//			//		GPSt, GPSWeek,
//			//		deltaTimeLast + timeCorrect);
//
//			//	if (
//			//		!SatellitePositionAndVelocity(
//			//			satellitePandV_,
//			//			timeCorrect,
//			//			GPSWeekAfterCorrect,
//			//			GPSSecondAfterCorrect,
//			//			gpsephem_)
//			//		) {
//			//		continue;
//			//	}
//
//			//	/*if (originSatePosFlag) {
//			//		originSatePos.x = satellitePandV_.x;
//			//		originSatePos.y = satellitePandV_.y;
//			//		originSatePos.z = satellitePandV_.z;
//			//	}*/
//			//	
//			//	
//			//	//if (originSatePosFlag) {
//			//		/*satellitePandV_.x = cos((deltaTimeCurrent + timeCorrect) * OMEGAE) * satellitePandV_.x +
//			//			sin((deltaTimeCurrent + timeCorrect) * OMEGAE) * satellitePandV_.y;
//			//		satellitePandV_.y = -sin((deltaTimeCurrent + timeCorrect)  * OMEGAE) * satellitePandV_.x +
//			//			cos((deltaTimeCurrent + timeCorrect)  * OMEGAE) * satellitePandV_.y;
//			//		satellitePandV_.z = satellitePandV_.z;*/
//			//	//}
//			//	//originSatePosFlag = false;
//
//			satellitePandV_.x = satellitePandV_.x +
//				(deltaTimeCurrent + timeCorrect) * OMEGAE * satellitePandV_.y;
//			satellitePandV_.y = satellitePandV_.y
//				- (deltaTimeCurrent + timeCorrect)  * OMEGAE * satellitePandV_.x;
//			satellitePandV_.z = satellitePandV_.z;
//			satellitePandV_.PRN = ranges[i].PRN;
//
//			//	satelliteX = satellitePandV_.x;
//			//	satelliteY = satellitePandV_.y;
//			//	satelliteZ = satellitePandV_.z;
//
//			//	r = sqrt(
//			//		(satelliteX - originXYZ.x) * (satelliteX - originXYZ.x) +
//			//		(satelliteY - originXYZ.y) * (satelliteY - originXYZ.y) +
//			//		(satelliteZ - originXYZ.z) * (satelliteZ - originXYZ.z)
//			//	);
//
//			//	deltaTimeCurrent = r / SPEEDOFLIGHT + currentPAndV.ct / SPEEDOFLIGHT;
//			//}
//
//			//if (loopInner >= 15) {
//			//	continue;
//			//}
//			/////end of time correct
//
//
//
//			innerVectorOfSates.push_back(satellitePandV_);
//			psrS.push_back(ranges[i].psr);
//		}
//
//		int innerSatelliteNum = innerVectorOfSates.size();
//		if (innerSatelliteNum < 4) return false;
//		Matrix<double> b = Matrix<double>(innerSatelliteNum, 4);
//		Matrix<double> w = Matrix<double>(innerSatelliteNum, 1);
//		Matrix<double> v = Matrix<double>(4, 1);
//
//		for (int i = 0; i < innerSatelliteNum; i++) {
//
//			satelliteX = innerVectorOfSates[i].x;
//			satelliteY = innerVectorOfSates[i].y;
//			satelliteZ = innerVectorOfSates[i].z;
//
//			XYZ satelliteXYZ = { satelliteX,satelliteY, satelliteZ };
//			BLH blh;
//			XYZ2BLH(satelliteXYZ, blh);
//
//			ENU enu;
//			XYZ2ENU(originXYZ, satelliteXYZ, enu);
//			double angle = atan2(enu.u, sqrt(enu.e * enu.e + enu.n * enu.n));
//			double azimuth = atan2(enu.e, enu.n);
//
//			Klobutchar(ionutc_, GPSt, blh.latitude, blh.longitude, azimuth / PI, angle / PI, klobutcharCorrect);
//			Hopefield(obsBLH.height/*52*/, angle * 180 / PI, hopefieldCorrect);
//
//			r = sqrt(
//				(satelliteX - originXYZ.x) * (satelliteX - originXYZ.x) +
//				(satelliteY - originXYZ.y) * (satelliteY - originXYZ.y) +
//				(satelliteZ - originXYZ.z) * (satelliteZ - originXYZ.z)
//			);
//
//			b.ChangeElement(i, 0, (originXYZ.x - innerVectorOfSates[i].x) / r);
//			b.ChangeElement(i, 1, (originXYZ.y - innerVectorOfSates[i].y) / r);
//			b.ChangeElement(i, 2, (originXYZ.z - innerVectorOfSates[i].z) / r);
//			b.ChangeElement(i, 3, 1);
//
//			double wi = psrS[i] - r - currentPAndV.ct * SPEEDOFLIGHT +
//				timeCorrect * SPEEDOFLIGHT
//				+ klobutcharCorrect * SPEEDOFLIGHT + hopefieldCorrect;
//			w.ChangeElement(i, 0, wi);
//
//		}
//		Matrix<double> bT = b.t();
//		v = (bT * b).Inv() * bT * w;
//
//		double vx = v.GetElement(0, 0);
//		double vy = v.GetElement(1, 0);
//		double vz = v.GetElement(2, 0);
//
//		currentPAndV.x += vx;
//		currentPAndV.y += vy;
//		currentPAndV.z += vz;
//		currentPAndV.ct += v.GetElement(3, 0) / SPEEDOFLIGHT;
//
//		originXYZ.x = currentPAndV.x;
//		originXYZ.y = currentPAndV.y;
//		originXYZ.z = currentPAndV.z;
//
//		XYZ2BLH(originXYZ, obsBLH);
//
//		deltaObs = vx * vx + vy * vy + vz * vz;
//		loopNum++;
//	}
//	if (loopNum >= 10) {
//		return false;
//	}
//	/*for(int i = 0 ; i < innerVectorOfSates.size();i++)
//		fprintf(fout, "%d %015.7f %015.7f %015.7f\n",
//			innerVectorOfSates[i].PRN,
//			innerVectorOfSates[i].x,
//			innerVectorOfSates[i].y,
//			innerVectorOfSates[i].z);*/
//	return true;
//}

