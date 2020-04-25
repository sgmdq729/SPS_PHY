#ifndef SNR_BLER_H
#define SNR_BLER_H

#include <unordered_map>
#include <cmath>

using namespace std;

static unordered_map<int, float> BLER_MAP_190{
	{-10, 0.9999},
	{-9, 0.9993},
	{-8, 0.9983},
	{-7, 0.9942},
	{-6, 0.9851},
	{-5, 0.964},
	{-4, 0.9247},
	{-3, 0.8671},
	{-2, 0.7927},
	{-1, 0.6657},
	{0, 0.5485},
	{1, 0.4256},
	{2, 0.3219},
	{3, 0.2295},
	{4, 0.1459},
	{5, 0.1078},
	{6, 0.0661},
	{7, 0.0431},
	{8, 0.00245},
	{9, 0.0138},
	{10, 0.0088},
	{11, 0.0048},
	{12, 0.0017},
	{13, 0.0013},
	{14, 0.0008},
	{15, 0.0007}
};

static unordered_map<int, float> BLER_MAP_300{
	{-10, 1},
	{-9, 0.9997},
	{-8, 0.9992},
	{-7, 0.9974},
	{-6, 0.99},
	{-5, 0.9759},
	{-4, 0.9483},
	{-3, 0.8969},
	{-2, 0.8211},
	{-1, 0.716},
	{0, 0.5888},
	{1, 0.4602},
	{2, 0.3317},
	{3, 0.2262},
	{4, 0.1336},
	{5, 0.0793},
	{6, 0.0435},
	{7, 0.0221},
	{8, 0.0092},
	{9, 0.0039},
	{10, 0.0011},
	{11, 0.0003},
	{12, 0},
	{13, 0},
	{14, 0},
	{15, 0}
};

inline float getBLER_300(float sinr) {
	if (sinr < -10) {
		return 1;
	}
	else if (sinr > 15) {
		return 0;
	}
	int x0 = floor(sinr);
	int x1 = ceil(sinr);
	float y0 = BLER_MAP_300[x0];
	float y1 = BLER_MAP_300[x1];
	return y0 + ((y1 - y0) * (sinr - x0)) / (x1 - x0);
}

inline float dB2mw(float dBm) {
	return pow(10, dBm / 10.);
}

inline float mw2dB(float mw) {
	return 10 * log10(mw);
}

#endif