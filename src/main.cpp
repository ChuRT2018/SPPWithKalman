// DecodeFile.cpp : 定义控制台应用程序的入口点。
//

#include "../include/includes.h"
#include <Windows.h>
#include <fstream>
#include <iostream>

#define LOGOBSP

using namespace GPSCurriculumDesign;

FILE* GPSCurriculumDesign::fout = nullptr;
Matrix<double> GPSCurriculumDesign::D = Matrix<double>();
int main()
{

	//unsigned char buff[MAXRAWLEN];
	FILE* FObs;

	//FILE* fout;
	if ((fout = fopen(std::string("..\\data\\ObsPos.txt").c_str(), "w")) == NULL) {
		return 0;
	}

	std::string  fileName = "..\\data\\20150207.bin";
	if ((FObs = fopen(fileName.c_str(), "rb")) == NULL) {
		printf("Cannot open GPS obs file.\n");
		return 0;
	}

	std::unordered_map<unsigned long, gpsephem> gpsephems;
	std::unordered_map<unsigned long, rawephem> rawephems;
	ionutc ionutc_;
	std::vector<range> ranges;
	std::vector<satellitePandV> satellitePandVs;
	//observationPAndV obsPandV = { 0 , 0 , 0 , 0 , 0 , 0 , 0, 0 };
	observationPAndV obsPandV = { -2268145.619375 , 5010313.897310 , 3219225.220075 , 57154229.94994342 , 0 , 0 ,0,0};
	/*zero for not update this epoch and one for updated in this epoch*/
	//int PRNIndex[33] = { 0 };
	int count = 0;
	bool getIonutc = false;
	bool getGPSEphem = false;
	D = Matrix<double>(4, 4,0);

	D.ChangeElement(0, 0, 100);
	D.ChangeElement(1, 1, 100);
	D.ChangeElement(2, 2, 100);
	D.ChangeElement(3, 3, 1000);

	while (count < 5000) {
		
		head header;
		unsigned char pdata[MEMERYDATA];
		int len = 0;

		bool getHeader = Findhead(FObs) && DecodeHead(FObs, header, pdata);
		if (!getHeader) {
			break;
		}

		if (!CheckCRC(FObs, pdata, header.headLength + header.messageLength)) {
			continue;
		}

		if (header.messageID == 7) {
			//GPSEPHEM
			unsigned long PRN = 0;
			getGPSEphem = DecodeGPSEphem(
				pdata,
				header.headLength,
				header.messageLength,
				gpsephems, PRN);
			std::cout << PRN << std::endl;

		}
		else if (header.messageID == 8) {
			//IONUTC

			getIonutc = DecodeIonutc(
				pdata,
				header.headLength,
				header.messageLength,
				ionutc_);

		}
		else if (header.messageID == 41) {
			//RAWEPHEM
			bool getRawEphem = DecodeRawEphem(
				pdata,
				header.headLength,
				header.messageLength,
				rawephems
			);

			
		}

		else if (header.messageID == 43) {
			//RANGE
			ranges.clear();
			bool getRange = DecodeRange(
				pdata,
				header.headLength,
				header.messageLength,
				ranges);
			//double x, y, z, Vx, Vy, Vz;


			if (!(getRange && getGPSEphem && getIonutc)) {
				continue;
			}
			
			//if (count == 2700)
			//	std::cout << " stop here ";
			if (!ObservationPositionAndVelocityKalman(
				ionutc_,
				gpsephems,
				ranges,
				header.week,
				header.gpsSecondBin / 1000,
				satellitePandVs,
				obsPandV)) {
				continue;
			}
			std::cout << ". ";
			count++;

#ifdef LOGOBSP
			XYZ xyz = { obsPandV.x, obsPandV.y, obsPandV.z };
			BLH blh;
			XYZ2BLH(xyz, blh);
			fprintf(fout, "%015.7f %015.7f %015.7f %015.7f %015.7f %015.7f %015.7f\n",
				blh.longitude * 180 / PI + 180, blh.latitude * 180 / PI, blh.height,
				obsPandV.vx, obsPandV.vy, obsPandV.vz, obsPandV.pdop);
#endif
			ranges.clear();
			satellitePandVs.clear();
		}
		else if (header.messageID == 47) {
			psrPOS psrPOS_;
			bool getBestPos = DecodePsrPos(pdata,
				header.headLength,
				header.messageLength,
				psrPOS_);
			/*if (getBestPos) {
				std::cout << psrPOS_.lat
					<< " " << psrPOS_.lon
					<< " " << psrPOS_.hgt << std::endl;
			}*/
		}
		else if (header.messageID == 140) {
			//RANGECMP
		}
	}
	fclose(fout);
	fclose(FObs);
	system("pause");
	return 0;
}
