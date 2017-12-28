#pragma once
#ifndef INCLUDES_H_
#define INCLUDES_H_

#include <iostream>
#include <fstream>
#include <string.h>
#include <string.h>
#include <vector>
#include <assert.h>
#include <unordered_map>

#include "TimeAndCoordinateTransform.h"
#include "Matrix.hpp"

namespace  GPSCurriculumDesign {


#define MAXRAWLEN 255
#define MEMERYDATA 4096

#define POLYCRC32 0xEDB88320u /* CRC32 polynomial */
#define PI 3.1415926535897932384626433832795 
#define SPEEDOFLIGHT  299792458
#define GM 3.986004415e14
#define OMEGAE 7.2921151467e-5
#define DELTAEK 1e-10
#define DELTAOBS 1e-4
#define E 0.0818191908425523357668632596693//1.25958691351e-2
#define POWE 0.00669437999013
#define FT -4.442807633e-10
#define FION 2.440388852916845384917909576999
#define H0 0
#define T0 288.15
#define P0 1013.25
#define RH0 0.5
#define CDIVIDEF1 0.19029367279836488047631742646405

enum dataformate {
	BINARY = 0,
	ASCII = 1,
	NEMA = 2,
	RESERVED = 3
};

enum responsebit {
	ORIGINAL = 0,
	RESPONDE = 1
};

enum timestatus {
	UNKNOWN = 20,				//Time validity is unknown
	APPROXIMATE = 60,			//Time is set approximately.
	COARSEADJUSTING = 80,		//Time is approaching coarse precision
	COARSE = 100,				//This time is valid to coarse precision
	COARSESTEERING = 120,		//Time is coarse set, and is being steered
	FREEWHEELING = 130,			//Position is lost, and the range bias cannot 
								//...be calculated.
	FINEADJUSTING = 140,		//Time is adjusting to fine precision.
	FINE = 160,					//Time has fine precision.
	FINEBACKUPSTEERING = 170,	//Time is fine set and is being steered by 
								//...the backup system.
	FINESTEERING = 180,			//Time is fine-set and is being steered.
	SATTIME = 200				//Time from satellite. This is only used in logs
								//...containing satellite data such as ephemeris and
								//...almanac.
};

enum antispoofing { FalseAntispoofing = 0, TrueAntispoofing = 1 };

typedef struct subframe {
	unsigned char inside[30];
};

typedef struct head {
	unsigned char headLength; // 1
	unsigned short messageID; // 2
	char messageType;  // 1
	unsigned char portAddress;  //1
	unsigned short messageLength; // 2
	unsigned short sequence; // 2 
	unsigned char idleTime; // 1
	unsigned char timeStatus; // 1
	unsigned short week; // 2
	long gpsSecondBin = -1; // 4 
	float gpsSecondASCII = -1; // 4
	unsigned long receiverStatus; // 4
	unsigned short reserved; // 2
	unsigned short receiver; // 2
};

typedef struct range {
	unsigned short PRN; //2
	unsigned short glofreq; //2
	double psr; //8
	float psrStd; //4
	double adr; //8
	float adrStd; //4
	float dopp; //4
	float C; //4
	float locktime; //4
	unsigned long ch_tr_status; //4
};

typedef struct rangecmp {

};

typedef struct gpsephem {
	unsigned long PRN; // 4
	double tow; // 8
	unsigned long health; // 4
	unsigned long iode1; // 4
	unsigned long iode2; // 4
	unsigned long week; // 4
	unsigned long zweek; // 4
	double toe; // 8
	double A; // 8
	double deltaN; // 8
	double M0; // 8
	double ecc; // 8
	double omega; // 8
	double cuc; // 8
	double cus; // 8
	double crc; // 8
	double crs; // 8
	double cic; // 8
	double cis; // 8
	double i0; // 8
	double dotI; // 8
	double omega0; // 8
	double dotOmega; // 8
	unsigned long iodc; //4
	double toc; // 8
	double tgd; // 8
	double af0; // 8
	double af1; // 8
	double af2; // 8
	antispoofing AS; // 4
	double N; // 8
	double URA; // 8
};

typedef struct ionutc {
	double a0; // 8
	double a1; // 8
	double a2; // 8
	double a3; // 8
	double b0; // 8
	double b1; // 8
	double b2; // 8
	double b3; // 8
	unsigned long utcwn; // 4
	unsigned long tot; // 4
	double A0; // 8
	double A1; // 8
	unsigned long wnlsf; // 4
	unsigned long dn; // 4
	long deltatls; // 4
	long deltatutc; //4
};

typedef struct rawephem {
	unsigned long prn; //4
	unsigned long refweek; // 4
	unsigned long refsec; // 4
	subframe subframe1; // 30
	subframe subframe2; // 30
	subframe subframe3; // 30
};

typedef struct satellitePandV {
	double x;
	double y;
	double z;
	double vx;
	double vy;
	double vz;
};

typedef struct observationPAndV {
	double x;
	double y;
	double z;
	double ct;
	double vx;
	double vy;
	double vz;
	double pdop;
};

typedef struct psrPOS {
	long solStat;  //4
	long posType; //4
	double lat; //8
	double lon; //8
	double hgt; //8
};

typedef struct orbitalParameter {
	double e;      //轨道长半轴
	double n;      //平均角速度
	double ek;     //偏近点角
	double phiK;
	double vk;     //真近点角
	double uk;     //经过改正的升交角距
	double rk;     //经过改正的向径
	double ik;     //经过改正的轨道倾角
	double omegaK; //改正后的升交点经度
};

bool Findhead(FILE* FObs);
bool DecodeHead(FILE* FObs, head& header, unsigned char* pdata);
bool DecodeRange(unsigned char*pdata, int headerLen, int messageLen, std::vector<range>& ranges);
bool DecodeGPSEphem(unsigned char* pdata, int headerLen, int messageLen, std::unordered_map<unsigned long,gpsephem>& gpsephems, unsigned long& PRN);
bool DecodeIonutc(unsigned char* pdata, int headerLen, int messageLen, ionutc& ionutc_);
bool DecodeRawEphem(unsigned char* pdata, int headerLen, int messageLen, std::unordered_map<unsigned long, rawephem>& rawephems);
bool DecodePsrPos(unsigned char* pdata, int headerLen, int messageLen, psrPOS& psrPos_);
bool CheckCRC(FILE* FObs,const unsigned char* pdata, const int len);
unsigned int crc32(const unsigned char *buff, int len);

bool TimeCorrect(gpsephem gpsephem_,const double t, const unsigned long week, const double ek, double& correctedT);
bool Klobutchar(ionutc ionutc_, const double GPSt, const double latitude, const double longitude, const double azimuth, const double angle, double &correct);
bool Hopefield(const double h, const double angle, double &correct);
bool SetTimeDelta(double& newSecond, unsigned long& newWeek, const double oldSecond, const unsigned long oldWeek, const  double timedelta);

template<typename _T>
inline bool Hex2Dec(unsigned char** pdata, _T& t, int totalLen, int& postion);

template<typename _T>
inline bool Hex2Dec(unsigned char ** pdata, _T & t, int totalLen, int & postion)
{
	if (*pdata == nullptr) return false;
	if (postion + sizeof(_T) > totalLen) return false;
	t = *((_T*)*pdata);
	postion += sizeof(_T);
	*pdata += sizeof(_T);
	return true;
}

bool CalculateOrbitalParameter(const unsigned long GPSWeek, 
	const double GPSSecond,
	const gpsephem gpsephem_,
	orbitalParameter& orbitalParameter_);

bool SatellitePositionAndVelocity(
	satellitePandV& satellitePandV_, 
	double& correctT, double& clockVelo,
	const unsigned long GPSWeek,
	const double GPSSecond,
	const gpsephem gpsephem_);


bool ObservationPositionAndVelocity(
	const ionutc ionutc_,
	std::unordered_map<unsigned long, gpsephem> gpsephems,
	std::vector<range> ranges,
	const unsigned long GPSWeek,
	const double GPSt, 
	std::vector<satellitePandV>& satellitePandVs,
	observationPAndV& currentPAndV);


bool ObservationPositionAndVelocityKalman(
	const ionutc ionutc_,
	std::unordered_map<unsigned long, gpsephem> gpsephems,
	std::vector<range> ranges,
	const unsigned long GPSWeek,
	const double GPSt,
	std::vector<satellitePandV>& satellitePandVs,
	observationPAndV& currentPAndV);
//inline bool Hex2Char(unsigned char* pdata, char& c, int totalLen, int& position);
//inline bool Hex2UChar(unsigned char* pdata, unsigned char& uc, int totalLen, int& position);
//inline bool Hex2Short(unsigned char* pdata, short& s, int totalLen, int& position);
//inline bool Hex2UShort(unsigned char* pdata, unsigned short& us, int totalLen, int& position);
//inline bool Hex2Int(unsigned char* pdata, int& i, int totalLen, int& position);
//inline bool Hex2UInt(unsigned char* pdata, unsigned int& ui, int totalLen, int& position);
//inline bool Hex2Long(unsigned char* pdata, long& l, int totalLen, int& position);
//inline bool Hex2ULong(unsigned char* pdata, unsigned long& ul, int totalLen, int& position);
//inline bool Hex2Float(unsigned char* pdata, float& f, int totalLen, int& position);
//inline bool Hex2Double(unsigned char* pdata, double& d, int totalLen, int& position);

extern FILE* fout;
extern Matrix<double> D;
}
#endif // !INCLUDES_H_