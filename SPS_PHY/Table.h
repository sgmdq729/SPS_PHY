#ifndef TABLE
#define TABLE

#include <unordered_map>
#include <string>
#include <vector>
#include <unordered_set>
#include <tuple>

using namespace std;
typedef std::tuple<int, float, float> ftuple;

/**
 @breif std::pairに対するハッシュを定義
 */
struct HashPair {
	template<class T1, class T2>
	size_t operator()(const pair<T1, T2>& p)const {
		auto hash1 = hash<T1>{}(p.first);
		auto hash2 = hash<T2>{}(p.second);
		size_t seed = 0;
		seed ^= hash1 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= hash2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		return seed;
	}
};

/**
 @breif 2車両間の位置する関係を定義
 */
enum class PositionRelation {
	HOL_PAR,	//横平行
	VER_PAR,	//縦平行
	NORMAL		//1回の回折
};

/**
 @breif 2つのlane_id間の関係を定義
 @param lane_id, {junction_id}
 */
static unordered_map<pair<int, int>, PositionRelation, HashPair> RELATION_TABLE{
	{make_pair(0, 1), PositionRelation::HOL_PAR},
	{make_pair(0, 2), PositionRelation::HOL_PAR},
	{make_pair(0, 3), PositionRelation::HOL_PAR},

	{make_pair(1, 0), PositionRelation::HOL_PAR},
	{make_pair(1, 2), PositionRelation::HOL_PAR},
	{make_pair(1, 3), PositionRelation::HOL_PAR},

	{make_pair(2, 0), PositionRelation::HOL_PAR},
	{make_pair(2, 1), PositionRelation::HOL_PAR},
	{make_pair(2, 3), PositionRelation::HOL_PAR},

	{make_pair(3, 0), PositionRelation::HOL_PAR},
	{make_pair(3, 1), PositionRelation::HOL_PAR},
	{make_pair(3, 3), PositionRelation::HOL_PAR},

	{make_pair(4, 5), PositionRelation::VER_PAR},
	{make_pair(4, 6), PositionRelation::VER_PAR},
	{make_pair(4, 7), PositionRelation::VER_PAR},

	{make_pair(5, 4), PositionRelation::VER_PAR},
	{make_pair(5, 6), PositionRelation::VER_PAR},
	{make_pair(5, 7), PositionRelation::VER_PAR},

	{make_pair(6, 4), PositionRelation::VER_PAR},
	{make_pair(6, 5), PositionRelation::VER_PAR},
	{make_pair(6, 7), PositionRelation::VER_PAR},

	{make_pair(7, 4), PositionRelation::VER_PAR},
	{make_pair(7, 5), PositionRelation::VER_PAR},
	{make_pair(7, 6), PositionRelation::VER_PAR},

	{make_pair(0, 4), PositionRelation::NORMAL},
	{make_pair(0, 5), PositionRelation::NORMAL},
	{make_pair(0, 6), PositionRelation::NORMAL},
	{make_pair(0, 7), PositionRelation::NORMAL},

	{make_pair(1, 4), PositionRelation::NORMAL},
	{make_pair(1, 5), PositionRelation::NORMAL},
	{make_pair(1, 6), PositionRelation::NORMAL},
	{make_pair(1, 7), PositionRelation::NORMAL},

	{make_pair(2, 4), PositionRelation::NORMAL},
	{make_pair(2, 5), PositionRelation::NORMAL},
	{make_pair(2, 6), PositionRelation::NORMAL},
	{make_pair(2, 7), PositionRelation::NORMAL},

	{make_pair(3, 4), PositionRelation::NORMAL},
	{make_pair(3, 5), PositionRelation::NORMAL},
	{make_pair(3, 6), PositionRelation::NORMAL},
	{make_pair(3, 7), PositionRelation::NORMAL},

	{make_pair(4, 0), PositionRelation::NORMAL},
	{make_pair(4, 1), PositionRelation::NORMAL},
	{make_pair(4, 2), PositionRelation::NORMAL},
	{make_pair(4, 3), PositionRelation::NORMAL},

	{make_pair(5, 0), PositionRelation::NORMAL},
	{make_pair(5, 1), PositionRelation::NORMAL},
	{make_pair(5, 2), PositionRelation::NORMAL},
	{make_pair(5, 3), PositionRelation::NORMAL},

	{make_pair(6, 0), PositionRelation::NORMAL},
	{make_pair(6, 1), PositionRelation::NORMAL},
	{make_pair(6, 2), PositionRelation::NORMAL},
	{make_pair(6, 3), PositionRelation::NORMAL},

	{make_pair(7, 0), PositionRelation::NORMAL},
	{make_pair(7, 1), PositionRelation::NORMAL},
	{make_pair(7, 2), PositionRelation::NORMAL},
	{make_pair(7, 3), PositionRelation::NORMAL},

	{make_pair(10, 1), PositionRelation::NORMAL},
	{make_pair(10, 2), PositionRelation::NORMAL},
	{make_pair(10, 3), PositionRelation::NORMAL},
	{make_pair(10, 5), PositionRelation::NORMAL},
	{make_pair(10, 6), PositionRelation::NORMAL},
	{make_pair(10, 7), PositionRelation::NORMAL},

	{make_pair(11, 1), PositionRelation::NORMAL},
	{make_pair(11, 2), PositionRelation::NORMAL},
	{make_pair(11, 3), PositionRelation::NORMAL},
	{make_pair(11, 4), PositionRelation::NORMAL},
	{make_pair(11, 6), PositionRelation::NORMAL},
	{make_pair(11, 7), PositionRelation::NORMAL},

	{make_pair(12, 1), PositionRelation::NORMAL},
	{make_pair(12, 2), PositionRelation::NORMAL},
	{make_pair(12, 3), PositionRelation::NORMAL},
	{make_pair(12, 4), PositionRelation::NORMAL},
	{make_pair(12, 5), PositionRelation::NORMAL},
	{make_pair(12, 7), PositionRelation::NORMAL},

	{make_pair(13, 1), PositionRelation::NORMAL},
	{make_pair(13, 2), PositionRelation::NORMAL},
	{make_pair(13, 3), PositionRelation::NORMAL},
	{make_pair(13, 4), PositionRelation::NORMAL},
	{make_pair(13, 5), PositionRelation::NORMAL},
	{make_pair(13, 6), PositionRelation::NORMAL},

	{make_pair(14, 0), PositionRelation::NORMAL},
	{make_pair(14, 2), PositionRelation::NORMAL},
	{make_pair(14, 3), PositionRelation::NORMAL},
	{make_pair(14, 5), PositionRelation::NORMAL},
	{make_pair(14, 6), PositionRelation::NORMAL},
	{make_pair(14, 7), PositionRelation::NORMAL},

	{make_pair(15, 0), PositionRelation::NORMAL},
	{make_pair(15, 2), PositionRelation::NORMAL},
	{make_pair(15, 3), PositionRelation::NORMAL},
	{make_pair(15, 4), PositionRelation::NORMAL},
	{make_pair(15, 6), PositionRelation::NORMAL},
	{make_pair(15, 7), PositionRelation::NORMAL},

	{make_pair(16, 0), PositionRelation::NORMAL},
	{make_pair(16, 2), PositionRelation::NORMAL},
	{make_pair(16, 3), PositionRelation::NORMAL},
	{make_pair(16, 4), PositionRelation::NORMAL},
	{make_pair(16, 5), PositionRelation::NORMAL},
	{make_pair(16, 7), PositionRelation::NORMAL},

	{make_pair(17, 0), PositionRelation::NORMAL},
	{make_pair(17, 2), PositionRelation::NORMAL},
	{make_pair(17, 3), PositionRelation::NORMAL},
	{make_pair(17, 4), PositionRelation::NORMAL},
	{make_pair(17, 5), PositionRelation::NORMAL},
	{make_pair(17, 6), PositionRelation::NORMAL},

	{make_pair(18, 0), PositionRelation::NORMAL},
	{make_pair(18, 1), PositionRelation::NORMAL},
	{make_pair(18, 3), PositionRelation::NORMAL},
	{make_pair(18, 5), PositionRelation::NORMAL},
	{make_pair(18, 6), PositionRelation::NORMAL},
	{make_pair(18, 7), PositionRelation::NORMAL},

	{make_pair(19, 0), PositionRelation::NORMAL},
	{make_pair(19, 1), PositionRelation::NORMAL},
	{make_pair(19, 3), PositionRelation::NORMAL},
	{make_pair(19, 4), PositionRelation::NORMAL},
	{make_pair(19, 6), PositionRelation::NORMAL},
	{make_pair(19, 7), PositionRelation::NORMAL},

	{make_pair(20, 0), PositionRelation::NORMAL},
	{make_pair(20, 1), PositionRelation::NORMAL},
	{make_pair(20, 3), PositionRelation::NORMAL},
	{make_pair(20, 4), PositionRelation::NORMAL},
	{make_pair(20, 5), PositionRelation::NORMAL},
	{make_pair(20, 7), PositionRelation::NORMAL},

	{make_pair(21, 0), PositionRelation::NORMAL},
	{make_pair(21, 1), PositionRelation::NORMAL},
	{make_pair(21, 3), PositionRelation::NORMAL},
	{make_pair(21, 4), PositionRelation::NORMAL},
	{make_pair(21, 5), PositionRelation::NORMAL},
	{make_pair(21, 6), PositionRelation::NORMAL},

	{make_pair(22, 0), PositionRelation::NORMAL},
	{make_pair(22, 1), PositionRelation::NORMAL},
	{make_pair(22, 2), PositionRelation::NORMAL},
	{make_pair(22, 5), PositionRelation::NORMAL},
	{make_pair(22, 6), PositionRelation::NORMAL},
	{make_pair(22, 7), PositionRelation::NORMAL},

	{make_pair(23, 0), PositionRelation::NORMAL},
	{make_pair(23, 1), PositionRelation::NORMAL},
	{make_pair(23, 2), PositionRelation::NORMAL},
	{make_pair(23, 4), PositionRelation::NORMAL},
	{make_pair(23, 6), PositionRelation::NORMAL},
	{make_pair(23, 7), PositionRelation::NORMAL},

	{make_pair(24, 0), PositionRelation::NORMAL},
	{make_pair(24, 1), PositionRelation::NORMAL},
	{make_pair(24, 2), PositionRelation::NORMAL},
	{make_pair(24, 4), PositionRelation::NORMAL},
	{make_pair(24, 5), PositionRelation::NORMAL},
	{make_pair(24, 7), PositionRelation::NORMAL},

	{make_pair(25, 0), PositionRelation::NORMAL},
	{make_pair(25, 1), PositionRelation::NORMAL},
	{make_pair(25, 2), PositionRelation::NORMAL},
	{make_pair(25, 4), PositionRelation::NORMAL},
	{make_pair(25, 5), PositionRelation::NORMAL},
	{make_pair(25, 6), PositionRelation::NORMAL},

	{make_pair(1, 10), PositionRelation::NORMAL},
	{make_pair(2, 10), PositionRelation::NORMAL},
	{make_pair(3, 10), PositionRelation::NORMAL},
	{make_pair(5, 10), PositionRelation::NORMAL},
	{make_pair(6, 10), PositionRelation::NORMAL},
	{make_pair(7, 10), PositionRelation::NORMAL},

	{make_pair(1, 11), PositionRelation::NORMAL},
	{make_pair(2, 11), PositionRelation::NORMAL},
	{make_pair(3, 11), PositionRelation::NORMAL},
	{make_pair(4, 11), PositionRelation::NORMAL},
	{make_pair(6, 11), PositionRelation::NORMAL},
	{make_pair(7, 11), PositionRelation::NORMAL},

	{make_pair(1, 12), PositionRelation::NORMAL},
	{make_pair(2, 12), PositionRelation::NORMAL},
	{make_pair(3, 12), PositionRelation::NORMAL},
	{make_pair(4, 12), PositionRelation::NORMAL},
	{make_pair(5, 12), PositionRelation::NORMAL},
	{make_pair(7, 12), PositionRelation::NORMAL},

	{make_pair(1, 13), PositionRelation::NORMAL},
	{make_pair(2, 13), PositionRelation::NORMAL},
	{make_pair(3, 13), PositionRelation::NORMAL},
	{make_pair(4, 13), PositionRelation::NORMAL},
	{make_pair(5, 13), PositionRelation::NORMAL},
	{make_pair(6, 13), PositionRelation::NORMAL},

	{make_pair(0, 14), PositionRelation::NORMAL},
	{make_pair(2, 14), PositionRelation::NORMAL},
	{make_pair(3, 14), PositionRelation::NORMAL},
	{make_pair(5, 14), PositionRelation::NORMAL},
	{make_pair(6, 14), PositionRelation::NORMAL},
	{make_pair(7, 14), PositionRelation::NORMAL},

	{make_pair(0, 15), PositionRelation::NORMAL},
	{make_pair(2, 15), PositionRelation::NORMAL},
	{make_pair(3, 15), PositionRelation::NORMAL},
	{make_pair(4, 15), PositionRelation::NORMAL},
	{make_pair(6, 15), PositionRelation::NORMAL},
	{make_pair(7, 15), PositionRelation::NORMAL},

	{make_pair(0, 16), PositionRelation::NORMAL},
	{make_pair(2, 16), PositionRelation::NORMAL},
	{make_pair(3, 16), PositionRelation::NORMAL},
	{make_pair(4, 16), PositionRelation::NORMAL},
	{make_pair(5, 16), PositionRelation::NORMAL},
	{make_pair(7, 16), PositionRelation::NORMAL},

	{make_pair(0, 17), PositionRelation::NORMAL},
	{make_pair(2, 17), PositionRelation::NORMAL},
	{make_pair(3, 17), PositionRelation::NORMAL},
	{make_pair(4, 17), PositionRelation::NORMAL},
	{make_pair(5, 17), PositionRelation::NORMAL},
	{make_pair(6, 17), PositionRelation::NORMAL},

	{make_pair(0, 18), PositionRelation::NORMAL},
	{make_pair(1, 18), PositionRelation::NORMAL},
	{make_pair(3, 18), PositionRelation::NORMAL},
	{make_pair(5, 18), PositionRelation::NORMAL},
	{make_pair(6, 18), PositionRelation::NORMAL},
	{make_pair(7, 18), PositionRelation::NORMAL},

	{make_pair(0, 19), PositionRelation::NORMAL},
	{make_pair(1, 19), PositionRelation::NORMAL},
	{make_pair(3, 19), PositionRelation::NORMAL},
	{make_pair(4, 19), PositionRelation::NORMAL},
	{make_pair(6, 19), PositionRelation::NORMAL},
	{make_pair(7, 19), PositionRelation::NORMAL},

	{make_pair(0, 20), PositionRelation::NORMAL},
	{make_pair(1, 20), PositionRelation::NORMAL},
	{make_pair(3, 20), PositionRelation::NORMAL},
	{make_pair(4, 20), PositionRelation::NORMAL},
	{make_pair(5, 20), PositionRelation::NORMAL},
	{make_pair(7, 20), PositionRelation::NORMAL},

	{make_pair(0, 21), PositionRelation::NORMAL},
	{make_pair(1, 21), PositionRelation::NORMAL},
	{make_pair(3, 21), PositionRelation::NORMAL},
	{make_pair(4, 21), PositionRelation::NORMAL},
	{make_pair(5, 21), PositionRelation::NORMAL},
	{make_pair(6, 21), PositionRelation::NORMAL},

	{make_pair(0, 22), PositionRelation::NORMAL},
	{make_pair(1, 22), PositionRelation::NORMAL},
	{make_pair(2, 22), PositionRelation::NORMAL},
	{make_pair(5, 22), PositionRelation::NORMAL},
	{make_pair(6, 22), PositionRelation::NORMAL},
	{make_pair(7, 22), PositionRelation::NORMAL},

	{make_pair(0, 23), PositionRelation::NORMAL},
	{make_pair(1, 23), PositionRelation::NORMAL},
	{make_pair(2, 23), PositionRelation::NORMAL},
	{make_pair(4, 23), PositionRelation::NORMAL},
	{make_pair(6, 23), PositionRelation::NORMAL},
	{make_pair(7, 23), PositionRelation::NORMAL},

	{make_pair(0, 24), PositionRelation::NORMAL},
	{make_pair(1, 24), PositionRelation::NORMAL},
	{make_pair(2, 24), PositionRelation::NORMAL},
	{make_pair(4, 24), PositionRelation::NORMAL},
	{make_pair(5, 24), PositionRelation::NORMAL},
	{make_pair(7, 24), PositionRelation::NORMAL},

	{make_pair(0, 25), PositionRelation::NORMAL},
	{make_pair(1, 25), PositionRelation::NORMAL},
	{make_pair(2, 25), PositionRelation::NORMAL},
	{make_pair(4, 25), PositionRelation::NORMAL},
	{make_pair(5, 25), PositionRelation::NORMAL},
	{make_pair(6, 25), PositionRelation::NORMAL}
};

/**
 @breif 各交差点間の直線距離を定義
 @param pair<origin_id, dest_id>, distance
 */
static unordered_map <pair<int, int>, float, HashPair> DIRECT_DISTANCE_TABLE = {
	{make_pair(100,110), 250},
	{make_pair(100,120), 500},
	{make_pair(100,130), 750},
	{make_pair(100,140), 433},
	{make_pair(100,150), 499.989},
	{make_pair(100,160), 661.43},
	{make_pair(100,170), 866.019},
	{make_pair(100,180), 866},
	{make_pair(100,190), 901.363},
	{make_pair(100,200), 999.978},
	{make_pair(100,210), 1145.62},
	{make_pair(100,220), 1299},
	{make_pair(100,230), 1322.84},
	{make_pair(100,240), 1391.91},
	{make_pair(100,250), 1499.97},

	{make_pair(110,100), 250},
	{make_pair(110,120), 250},
	{make_pair(110,130), 500},
	{make_pair(110,140), 499.989},
	{make_pair(110,150), 433},
	{make_pair(110,160), 499.989},
	{make_pair(110,170), 661.43},
	{make_pair(110,180), 901.363},
	{make_pair(110,190), 866},
	{make_pair(110,200), 901.363},
	{make_pair(110,210), 999.978},
	{make_pair(110,220), 1322.84},
	{make_pair(110,230), 1299},
	{make_pair(110,240), 1322.84},
	{make_pair(110,250), 1391.91},

	{make_pair(120,100), 500},
	{make_pair(120,110), 250},
	{make_pair(120,130), 250},
	{make_pair(120,140), 661.43},
	{make_pair(120,150), 499.989},
	{make_pair(120,160), 433},
	{make_pair(120,170), 499.989},
	{make_pair(120,180), 999.978},
	{make_pair(120,190), 901.363},
	{make_pair(120,200), 866},
	{make_pair(120,210), 901.363},
	{make_pair(120,220), 1391.91},
	{make_pair(120,230), 1322.84},
	{make_pair(120,240), 1299},
	{make_pair(120,250), 1322.84},

	{make_pair(130,100), 750},
	{make_pair(130,110), 500},
	{make_pair(130,120), 250},
	{make_pair(130,140), 866.019},
	{make_pair(130,150), 661.43},
	{make_pair(130,160), 499.989},
	{make_pair(130,170), 433},
	{make_pair(130,180), 1145.62},
	{make_pair(130,190), 999.978},
	{make_pair(130,200), 901.363},
	{make_pair(130,210), 866},
	{make_pair(130,220), 1499.97},
	{make_pair(130,230), 1391.91},
	{make_pair(130,240), 1322.84},
	{make_pair(130,250), 1299},

	{make_pair(140,100), 433},
	{make_pair(140,110), 499.989},
	{make_pair(140,120), 661.43},
	{make_pair(140,130), 866.019},
	{make_pair(140,150), 250},
	{make_pair(140,160), 500},
	{make_pair(140,170), 750},
	{make_pair(140,180), 433},
	{make_pair(140,190), 499.989},
	{make_pair(140,200), 661.43},
	{make_pair(140,210), 866.019},
	{make_pair(140,220), 866},
	{make_pair(140,230), 901.363},
	{make_pair(140,240), 999.978},
	{make_pair(140,250), 1145.62},

	{make_pair(150,100), 499.989},
	{make_pair(150,110), 433},
	{make_pair(150,120), 499.989},
	{make_pair(150,130), 661.43},
	{make_pair(150,140), 250},
	{make_pair(150,160), 250},
	{make_pair(150,170), 500},
	{make_pair(150,180), 499.989},
	{make_pair(150,190), 433},
	{make_pair(150,200), 499.989},
	{make_pair(150,210), 661.43},
	{make_pair(150,220), 901.363},
	{make_pair(150,230), 866},
	{make_pair(150,240), 901.363},
	{make_pair(150,250), 999.978},

	{make_pair(160,100), 661.43},
	{make_pair(160,110), 499.989},
	{make_pair(160,120), 433},
	{make_pair(160,130), 499.989},
	{make_pair(160,140), 500},
	{make_pair(160,150), 250},
	{make_pair(160,170), 250},
	{make_pair(160,180), 661.43},
	{make_pair(160,190), 499.989},
	{make_pair(160,200), 433},
	{make_pair(160,210), 499.989},
	{make_pair(160,220), 999.978},
	{make_pair(160,230), 901.363},
	{make_pair(160,240), 866},
	{make_pair(160,250), 901.363},

	{make_pair(170,100), 866.019},
	{make_pair(170,110), 661.43},
	{make_pair(170,120), 499.989},
	{make_pair(170,130), 433},
	{make_pair(170,140), 750},
	{make_pair(170,150), 500},
	{make_pair(170,160), 250},
	{make_pair(170,180), 866.019},
	{make_pair(170,190), 661.43},
	{make_pair(170,200), 499.989},
	{make_pair(170,210), 433},
	{make_pair(170,220), 1145.62},
	{make_pair(170,230), 999.978},
	{make_pair(170,240), 901.363},
	{make_pair(170,250), 866},

	{make_pair(180,100), 866},
	{make_pair(180,110), 901.363},
	{make_pair(180,120), 999.978},
	{make_pair(180,130), 1145.62},
	{make_pair(180,140), 433},
	{make_pair(180,150), 499.989},
	{make_pair(180,160), 661.43},
	{make_pair(180,170), 866.019},
	{make_pair(180,190), 250},
	{make_pair(180,200), 500},
	{make_pair(180,210), 750},
	{make_pair(180,220), 433},
	{make_pair(180,230), 499.989},
	{make_pair(180,240), 661.43},
	{make_pair(180,250), 866.019},

	{make_pair(190,100), 901.363},
	{make_pair(190,110), 866},
	{make_pair(190,120), 901.363},
	{make_pair(190,130), 999.978},
	{make_pair(190,140), 499.989},
	{make_pair(190,150), 433},
	{make_pair(190,160), 499.989},
	{make_pair(190,170), 661.43},
	{make_pair(190,180), 250},
	{make_pair(190,200), 250},
	{make_pair(190,210), 500},
	{make_pair(190,220), 499.989},
	{make_pair(190,230), 433},
	{make_pair(190,240), 499.989},
	{make_pair(190,250), 661.43},

	{make_pair(200,100), 999.978},
	{make_pair(200,110), 901.363},
	{make_pair(200,120), 866},
	{make_pair(200,130), 901.363},
	{make_pair(200,140), 661.43},
	{make_pair(200,150), 499.989},
	{make_pair(200,160), 433},
	{make_pair(200,170), 499.989},
	{make_pair(200,180), 500},
	{make_pair(200,190), 250},
	{make_pair(200,210), 250},
	{make_pair(200,220), 661.43},
	{make_pair(200,230), 499.989},
	{make_pair(200,240), 433},
	{make_pair(200,250), 499.989},

	{make_pair(210,100), 1145.62},
	{make_pair(210,110), 999.978},
	{make_pair(210,120), 901.363},
	{make_pair(210,130), 866},
	{make_pair(210,140), 866.019},
	{make_pair(210,150), 661.43},
	{make_pair(210,160), 499.989},
	{make_pair(210,170), 433},
	{make_pair(210,180), 750},
	{make_pair(210,190), 500},
	{make_pair(210,200), 250},
	{make_pair(210,220), 866.019},
	{make_pair(210,230), 661.43},
	{make_pair(210,240), 499.989},
	{make_pair(210,250), 433},

	{make_pair(220,100), 1299},
	{make_pair(220,110), 1322.84},
	{make_pair(220,120), 1391.91},
	{make_pair(220,130), 1499.97},
	{make_pair(220,140), 866},
	{make_pair(220,150), 901.363},
	{make_pair(220,160), 999.978},
	{make_pair(220,170), 1145.62},
	{make_pair(220,180), 433},
	{make_pair(220,190), 499.989},
	{make_pair(220,200), 661.43},
	{make_pair(220,210), 866.019},
	{make_pair(220,230), 250},
	{make_pair(220,240), 500},
	{make_pair(220,250), 750},

	{make_pair(230,100), 1322.84},
	{make_pair(230,110), 1299},
	{make_pair(230,120), 1322.84},
	{make_pair(230,130), 1391.91},
	{make_pair(230,140), 901.363},
	{make_pair(230,150), 866},
	{make_pair(230,160), 901.363},
	{make_pair(230,170), 999.978},
	{make_pair(230,180), 499.989},
	{make_pair(230,190), 433},
	{make_pair(230,200), 499.989},
	{make_pair(230,210), 661.43},
	{make_pair(230,220), 250},
	{make_pair(230,240), 250},
	{make_pair(230,250), 500},

	{make_pair(240,100), 1391.91},
	{make_pair(240,110), 1322.84},
	{make_pair(240,120), 1299},
	{make_pair(240,130), 1322.84},
	{make_pair(240,140), 999.978},
	{make_pair(240,150), 901.363},
	{make_pair(240,160), 866},
	{make_pair(240,170), 901.363},
	{make_pair(240,180), 661.43},
	{make_pair(240,190), 499.989},
	{make_pair(240,200), 433},
	{make_pair(240,210), 499.989},
	{make_pair(240,220), 500},
	{make_pair(240,230), 250},
	{make_pair(240,250), 250},

	{make_pair(250,100), 1499.97},
	{make_pair(250,110), 1391.91},
	{make_pair(250,120), 1322.84},
	{make_pair(250,130), 1299},
	{make_pair(250,140), 1145.62},
	{make_pair(250,150), 999.978},
	{make_pair(250,160), 901.363},
	{make_pair(250,170), 866},
	{make_pair(250,180), 866.019},
	{make_pair(250,190), 661.43},
	{make_pair(250,200), 499.989},
	{make_pair(250,210), 433},
	{make_pair(250,220), 750},
	{make_pair(250,230), 500},
	{make_pair(250,240), 250}
};

/**
 @breif 各交差点間の道のり距離を定義
 @param pair<origin_id, dest_id>, distance
 */
static unordered_map <pair<int, int>, float, HashPair> DISTANCE_TABLE = {
	{make_pair(100,110), 250},
	{make_pair(100,120), 500},
	{make_pair(100,130), 750},
	{make_pair(100,140), 433},
	{make_pair(100,150), 683},
	{make_pair(100,160), 933},
	{make_pair(100,170), 1183},
	{make_pair(100,180), 866},
	{make_pair(100,190), 1116},
	{make_pair(100,200), 1366},
	{make_pair(100,210), 1616},
	{make_pair(100,220), 1299},
	{make_pair(100,230), 1549},
	{make_pair(100,240), 1799},
	{make_pair(100,250), 2049},

	{make_pair(110,100), 250},
	{make_pair(110,120), 250},
	{make_pair(110,130), 500},
	{make_pair(110,140), 683},
	{make_pair(110,150), 433},
	{make_pair(110,160), 683},
	{make_pair(110,170), 933},
	{make_pair(110,180), 1116},
	{make_pair(110,190), 866},
	{make_pair(110,200), 1116},
	{make_pair(110,210), 1366},
	{make_pair(110,220), 1549},
	{make_pair(110,230), 1299},
	{make_pair(110,240), 1549},
	{make_pair(110,250), 1799},

	{make_pair(120,100), 500},
	{make_pair(120,110), 250},
	{make_pair(120,130), 250},
	{make_pair(120,140), 933},
	{make_pair(120,150), 683},
	{make_pair(120,160), 433},
	{make_pair(120,170), 683},
	{make_pair(120,180), 1366},
	{make_pair(120,190), 1116},
	{make_pair(120,200), 866},
	{make_pair(120,210), 1116},
	{make_pair(120,220), 1799},
	{make_pair(120,230), 1549},
	{make_pair(120,240), 1299},
	{make_pair(120,250), 1549},

	{make_pair(130,100), 750},
	{make_pair(130,110), 500},
	{make_pair(130,120), 250},
	{make_pair(130,140), 1183},
	{make_pair(130,150), 933},
	{make_pair(130,160), 683},
	{make_pair(130,170), 433},
	{make_pair(130,180), 1616},
	{make_pair(130,190), 1366},
	{make_pair(130,200), 1116},
	{make_pair(130,210), 866},
	{make_pair(130,220), 2049},
	{make_pair(130,230), 1799},
	{make_pair(130,240), 1549},
	{make_pair(130,250), 1299},

	{make_pair(140,100), 433},
	{make_pair(140,110), 683},
	{make_pair(140,120), 933},
	{make_pair(140,130), 1183},
	{make_pair(140,150), 250},
	{make_pair(140,160), 500},
	{make_pair(140,170), 750},
	{make_pair(140,180), 433},
	{make_pair(140,190), 683},
	{make_pair(140,200), 933},
	{make_pair(140,210), 1183},
	{make_pair(140,220), 866},
	{make_pair(140,230), 1116},
	{make_pair(140,240), 1366},
	{make_pair(140,250), 1616},

	{make_pair(150,100), 683},
	{make_pair(150,110), 433},
	{make_pair(150,120), 683},
	{make_pair(150,130), 933},
	{make_pair(150,140), 250},
	{make_pair(150,160), 250},
	{make_pair(150,170), 500},
	{make_pair(150,180), 683},
	{make_pair(150,190), 433},
	{make_pair(150,200), 683},
	{make_pair(150,210), 933},
	{make_pair(150,220), 1116},
	{make_pair(150,230), 866},
	{make_pair(150,240), 1116},
	{make_pair(150,250), 1366},

	{make_pair(160,100), 933},
	{make_pair(160,110), 683},
	{make_pair(160,120), 433},
	{make_pair(160,130), 683},
	{make_pair(160,140), 500},
	{make_pair(160,150), 250},
	{make_pair(160,170), 250},
	{make_pair(160,180), 933},
	{make_pair(160,190), 683},
	{make_pair(160,200), 433},
	{make_pair(160,210), 683},
	{make_pair(160,220), 1366},
	{make_pair(160,230), 1116},
	{make_pair(160,240), 866},
	{make_pair(160,250), 1116},

	{make_pair(170,100), 1183},
	{make_pair(170,110), 933},
	{make_pair(170,120), 683},
	{make_pair(170,130), 433},
	{make_pair(170,140), 750},
	{make_pair(170,150), 500},
	{make_pair(170,160), 250},
	{make_pair(170,180), 1183},
	{make_pair(170,190), 933},
	{make_pair(170,200), 683},
	{make_pair(170,210), 433},
	{make_pair(170,220), 1616},
	{make_pair(170,230), 1366},
	{make_pair(170,240), 1116},
	{make_pair(170,250), 866},

	{make_pair(180,100), 866},
	{make_pair(180,110), 1116},
	{make_pair(180,120), 1366},
	{make_pair(180,130), 1616},
	{make_pair(180,140), 433},
	{make_pair(180,150), 683},
	{make_pair(180,160), 933},
	{make_pair(180,170), 1183},
	{make_pair(180,190), 250},
	{make_pair(180,200), 500},
	{make_pair(180,210), 750},
	{make_pair(180,220), 433},
	{make_pair(180,230), 683},
	{make_pair(180,240), 933},
	{make_pair(180,250), 1183},

	{make_pair(190,100), 1116},
	{make_pair(190,110), 866},
	{make_pair(190,120), 1116},
	{make_pair(190,130), 1366},
	{make_pair(190,140), 683},
	{make_pair(190,150), 433},
	{make_pair(190,160), 683},
	{make_pair(190,170), 933},
	{make_pair(190,180), 250},
	{make_pair(190,200), 250},
	{make_pair(190,210), 500},
	{make_pair(190,220), 683},
	{make_pair(190,230), 433},
	{make_pair(190,240), 683},
	{make_pair(190,250), 933},

	{make_pair(200,100), 1366},
	{make_pair(200,110), 1116},
	{make_pair(200,120), 866},
	{make_pair(200,130), 1116},
	{make_pair(200,140), 933},
	{make_pair(200,150), 683},
	{make_pair(200,160), 433},
	{make_pair(200,170), 683},
	{make_pair(200,180), 500},
	{make_pair(200,190), 250},
	{make_pair(200,210), 250},
	{make_pair(200,220), 933},
	{make_pair(200,230), 683},
	{make_pair(200,240), 433},
	{make_pair(200,250), 683},

	{make_pair(210,100), 1616},
	{make_pair(210,110), 1366},
	{make_pair(210,120), 1116},
	{make_pair(210,130), 866},
	{make_pair(210,140), 1183},
	{make_pair(210,150), 933},
	{make_pair(210,160), 683},
	{make_pair(210,170), 433},
	{make_pair(210,180), 750},
	{make_pair(210,190), 500},
	{make_pair(210,200), 250},
	{make_pair(210,220), 1183},
	{make_pair(210,230), 933},
	{make_pair(210,240), 683},
	{make_pair(210,250), 433},

	{make_pair(220,100), 1299},
	{make_pair(220,110), 1549},
	{make_pair(220,120), 1799},
	{make_pair(220,130), 2049},
	{make_pair(220,140), 866},
	{make_pair(220,150), 1116},
	{make_pair(220,160), 1366},
	{make_pair(220,170), 1616},
	{make_pair(220,180), 433},
	{make_pair(220,190), 683},
	{make_pair(220,200), 933},
	{make_pair(220,210), 1183},
	{make_pair(220,230), 250},
	{make_pair(220,240), 500},
	{make_pair(220,250), 750},

	{make_pair(230,100), 1549},
	{make_pair(230,110), 1299},
	{make_pair(230,120), 1549},
	{make_pair(230,130), 1799},
	{make_pair(230,140), 1116},
	{make_pair(230,150), 866},
	{make_pair(230,160), 1116},
	{make_pair(230,170), 1366},
	{make_pair(230,180), 683},
	{make_pair(230,190), 433},
	{make_pair(230,200), 683},
	{make_pair(230,210), 933},
	{make_pair(230,220), 250},
	{make_pair(230,240), 250},
	{make_pair(230,250), 500},

	{make_pair(240,100), 1799},
	{make_pair(240,110), 1549},
	{make_pair(240,120), 1299},
	{make_pair(240,130), 1549},
	{make_pair(240,140), 1366},
	{make_pair(240,150), 1116},
	{make_pair(240,160), 866},
	{make_pair(240,170), 1116},
	{make_pair(240,180), 933},
	{make_pair(240,190), 683},
	{make_pair(240,200), 433},
	{make_pair(240,210), 683},
	{make_pair(240,220), 500},
	{make_pair(240,230), 250},
	{make_pair(240,250), 250},

	{make_pair(250,100), 2049},
	{make_pair(250,110), 1799},
	{make_pair(250,120), 1549},
	{make_pair(250,130), 1299},
	{make_pair(250,140), 1616},
	{make_pair(250,150), 1366},
	{make_pair(250,160), 1116},
	{make_pair(250,170), 866},
	{make_pair(250,180), 1183},
	{make_pair(250,190), 933},
	{make_pair(250,200), 683},
	{make_pair(250,210), 433},
	{make_pair(250,220), 750},
	{make_pair(250,230), 500},
	{make_pair(250,240), 250},
};

/**
 @breif　交差点の座標を定義
 @param lane_id, {junction_id}
 */
static unordered_map <int, pair<float, float>> JUNCTION_TABLE = {
	{100, make_pair(-375, 649.5)},
	{110, make_pair(-125, 649.5)},
	{120, make_pair(125, 649.5)},
	{130, make_pair(375, 649.5)},

	{140, make_pair(-375, 216.5)},
	{150, make_pair(-125, 216.5)},
	{160, make_pair(125, 216.5)},
	{170, make_pair(375, 216.5)},

	{180, make_pair(-375, -216.5)},
	{190, make_pair(-125, -216.5)},
	{200, make_pair(125, -216.5)},
	{210, make_pair(375, -216.5)},

	{220, make_pair(-375, -649.5)},
	{230, make_pair(125, -649.5)},
	{240, make_pair(125, -649.5)},
	{250, make_pair(375, -649.5)}	
};
/**
 @breif レーンIDに対する隣接する交差点を定義
 @param unordered_map<lane_id, tuple<junction_id, x, y>>
 */
static unordered_map<int, vector<ftuple>> ADJACENT_JUNCTION_TABLE = {
	/**横*/
	{0, vector<ftuple>{make_tuple(100, -375, 649.5)}},
	{1, vector<ftuple>{make_tuple(100, -375, 649.5)}},
	{2, vector<ftuple>{make_tuple(100, -375, 649.5), make_tuple(110, -125, 649.5)}},
	{3, vector<ftuple>{make_tuple(100, -375, 649.5), make_tuple(110, -125, 649.5)}},
	{4, vector<ftuple>{make_tuple(110, -125, 649.5), make_tuple(120, 125, 649.5)}},
	{5, vector<ftuple>{make_tuple(110, -125, 649.5), make_tuple(120, 125, 649.5)}},
	{6, vector<ftuple>{make_tuple(120, 125, 649.5), make_tuple(130, 375, 649.5)}},
	{7, vector<ftuple>{make_tuple(120, 125, 649.5), make_tuple(130, 375, 649.5)}},
	{8, vector<ftuple>{make_tuple(130, 375, 649.5)}},
	{9, vector<ftuple>{make_tuple(130, 375, 649.6)}},

	{10, vector<ftuple>{make_tuple(140, -375, 216.5)}},
	{11, vector<ftuple>{make_tuple(140, -375, 216.5)}},
	{12, vector<ftuple>{make_tuple(140, -375, 216.5), make_tuple(150, -125, 216.5)}},
	{13, vector<ftuple>{make_tuple(140, -375, 216.5), make_tuple(150, -125, 216.5)}},
	{14, vector<ftuple>{make_tuple(150, -125, 216.5), make_tuple(160, 125, 216.5)}},
	{15, vector<ftuple>{make_tuple(150, -125, 216.5), make_tuple(160, 125, 216.5)}},
	{16, vector<ftuple>{make_tuple(160, 125, 216.5), make_tuple(170, 375, 216.5)}},
	{17, vector<ftuple>{make_tuple(160, 125, 216.5), make_tuple(170, 375, 216.5)}},
	{18, vector<ftuple>{make_tuple(170, 375, 216.5)}},
	{19, vector<ftuple>{make_tuple(170, 375, 216.5)}},

	{20, vector<ftuple>{make_tuple(180, -375, -216.5)}},
	{21, vector<ftuple>{make_tuple(180, -375, -216.5)}},
	{22, vector<ftuple>{make_tuple(180, -375, -216.5), make_tuple(190, -125, -216.5)}},
	{23, vector<ftuple>{make_tuple(180, -375, -216.5), make_tuple(190, -125, -216.5)}},
	{24, vector<ftuple>{make_tuple(190, -125, -216.5), make_tuple(200, 125, -216.5)}},
	{25, vector<ftuple>{make_tuple(190, -125, -216.5), make_tuple(200, 125, -216.5)}},
	{26, vector<ftuple>{make_tuple(200, 125, -216.5), make_tuple(210, 375, -216.5)}},
	{27, vector<ftuple>{make_tuple(200, 125, -216.5), make_tuple(210, 375, -216.5)}},
	{28, vector<ftuple>{make_tuple(210, 375, -216.5)}},
	{29, vector<ftuple>{make_tuple(210, 375, -216.5)}},

	{30, vector<ftuple>{make_tuple(220, -375, -649.5)}},
	{31, vector<ftuple>{make_tuple(220, -375, -649.5)}},
	{32, vector<ftuple>{make_tuple(220, -375, -649.5), make_tuple(230, -125, -649.5)}},
	{33, vector<ftuple>{make_tuple(220, -375, -649.5), make_tuple(230, -125, -649.5)}},
	{34, vector<ftuple>{make_tuple(230, -125, -649.5), make_tuple(240, 125, -649.5)}},
	{35, vector<ftuple>{make_tuple(230, -125, -649.5), make_tuple(240, 125, -649.5)}},
	{36, vector<ftuple>{make_tuple(240, 125, -649.5), make_tuple(250, 375, -649.5)}},
	{37, vector<ftuple>{make_tuple(240, 125, -649.5), make_tuple(250, 375, -649.5)}},
	{38, vector<ftuple>{make_tuple(250, 375, -649.5)}},
	{39, vector<ftuple>{make_tuple(250, 375, -649.5)}},
	/**縦*/
	{40, vector<ftuple>{make_tuple(100, -375, 649.5)}},
	{41, vector<ftuple>{make_tuple(100, -375, 649.5)}},
	{42, vector<ftuple>{make_tuple(100, -375, 649.5), make_tuple(140, -375, 216.5)}},
	{43, vector<ftuple>{make_tuple(100, -375, 649.5), make_tuple(140, -375, 216.5)}},
	{44, vector<ftuple>{make_tuple(140, -375, 216.5), make_tuple(180, -375, -216.5)}},
	{45, vector<ftuple>{make_tuple(140, -375, 216.5), make_tuple(180, -375, -216.5)}},
	{46, vector<ftuple>{make_tuple(180, -375, -216.5), make_tuple(220, -375, -649.5)}},
	{47, vector<ftuple>{make_tuple(180, -375, -216.5), make_tuple(220, -375, -649.5)}},
	{48, vector<ftuple>{make_tuple(220, -375, -649.5)}},
	{49, vector<ftuple>{make_tuple(220, -375, -649.5)}},

	{50, vector<ftuple>{make_tuple(110, -125, 649.5)}},
	{51, vector<ftuple>{make_tuple(110, -125, 649.5)}},
	{52, vector<ftuple>{make_tuple(110, -125, 649.5), make_tuple(150, -125, 216.5)}},
	{53, vector<ftuple>{make_tuple(110, -125, 649.5), make_tuple(150, -125, 216.5)}},
	{54, vector<ftuple>{make_tuple(150, -125, 216.5), make_tuple(190, -125, -216.5)}},
	{55, vector<ftuple>{make_tuple(150, -125, 216.5), make_tuple(190, -125, -216.5)}},
	{56, vector<ftuple>{make_tuple(190, -125, -216.5), make_tuple(230, -125, -649.5)}},
	{57, vector<ftuple>{make_tuple(190, -125, -216.5), make_tuple(230, -125, -649.5)}},
	{58, vector<ftuple>{make_tuple(230, -125, -649.5)}},
	{59, vector<ftuple>{make_tuple(230, -125, -649.5)}},

	{60, vector<ftuple>{make_tuple(120, 125, 649.5)}},
	{61, vector<ftuple>{make_tuple(120, 125, 649.5)}},
	{62, vector<ftuple>{make_tuple(120, 125, 649.5), make_tuple(160, 125, 216.5)}},
	{63, vector<ftuple>{make_tuple(120, 125, 649.5), make_tuple(160, 125, 216.5)}},
	{64, vector<ftuple>{make_tuple(160, 125, 216.5), make_tuple(200, 125, -216.5)}},
	{65, vector<ftuple>{make_tuple(160, 125, 216.5), make_tuple(200, 125, -216.5)}},
	{66, vector<ftuple>{make_tuple(200, 125, -216.5), make_tuple(240, 125, -649.5)}},
	{67, vector<ftuple>{make_tuple(200, 125, -216.5), make_tuple(240, 125, -649.5)}},
	{68, vector<ftuple>{make_tuple(240, 125, -649.5)}},
	{69, vector<ftuple>{make_tuple(240, 125, -649.5)}},

	{70, vector<ftuple>{make_tuple(130, 375, 649.5)}},
	{71, vector<ftuple>{make_tuple(130, 375, 649.5)}},
	{72, vector<ftuple>{make_tuple(130, 375, 649.5), make_tuple(170, 375, 216.5)}},
	{73, vector<ftuple>{make_tuple(130, 375, 649.5), make_tuple(170, 375, 216.5)}},
	{74, vector<ftuple>{make_tuple(170, 375, 216.5), make_tuple(210, 375, -216.5)}},
	{75, vector<ftuple>{make_tuple(170, 375, 216.5), make_tuple(210, 375, -216.5)}},
	{76, vector<ftuple>{make_tuple(210, 375, -216.5), make_tuple(250, 375, -649.5)}},
	{77, vector<ftuple>{make_tuple(210, 375, -216.5), make_tuple(250, 375, -649.5)}},
	{78, vector<ftuple>{make_tuple(250, 375, -649.5)}},
	{79, vector<ftuple>{make_tuple(250, 375, -649.5)}},
	/**交差点*/
	{100,  vector<ftuple>{make_tuple(110, -125, 649.5), make_tuple(140, -375, 216.5)}},
	{110,  vector<ftuple>{make_tuple(100, -375, 649.5), make_tuple(150, -125, 216.5), make_tuple(120, 125, 649.5)}},
	{120,  vector<ftuple>{make_tuple(110, -125, 649.5), make_tuple(160, 125, 216.5), make_tuple(130, 375, 649.5)}},
	{130,  vector<ftuple>{make_tuple(120, 125, 649.5), make_tuple(170, 375, 216.5)}},

	{140,  vector<ftuple>{make_tuple(180, -375, -216.5), make_tuple(150, -125, 216.5), make_tuple(100, -375, 649.5)}},
	{150,  vector<ftuple>{make_tuple(140, -375, 216.5), make_tuple(190, -125, -216.5), make_tuple(160, 125, 216.5), make_tuple(110, -125, 649.5)}},
	{160,  vector<ftuple>{make_tuple(150, -125, 216.5), make_tuple(200, 125, -216.5), make_tuple(170, 375, 216.5), make_tuple(120, 125, 649.5)}},
	{170,  vector<ftuple>{make_tuple(160, 125, 216.5), make_tuple(210, 375, -216.5), make_tuple(130, 375, 649.5)}},

	{180,  vector<ftuple>{make_tuple(220, -375, -649.5), make_tuple(190, -125, -216.5), make_tuple(140, -375, 216.5)}},
	{190,  vector<ftuple>{make_tuple(180, -375, -216.5), make_tuple(230, -125, -649.5), make_tuple(200, 125, -216.5), make_tuple(150, -125, 216.5)}},
	{200,  vector<ftuple>{make_tuple(190, -125, -216.5), make_tuple(240, 125, -649.5), make_tuple(210, 375, -216.5), make_tuple(160, 125, 216.5)}},
	{210,  vector<ftuple>{make_tuple(200, 125, -216.5), make_tuple(250, 375, -649.5), make_tuple(170, 375, 216.5)}},

	{220,  vector<ftuple>{make_tuple(230, -125, -649.5), make_tuple(180, -375, -216.5)}},
	{230,  vector<ftuple>{make_tuple(220, -375, -649.5), make_tuple(240, 125, -649.5), make_tuple(190, -125, -216.5)}},
	{240,  vector<ftuple>{make_tuple(230, -125, -649.5), make_tuple(250, 375, -649.5), make_tuple(200, 125, -216.5)}},
	{250,  vector<ftuple>{make_tuple(240, 125, -649.5), make_tuple(210, 375, -216.5)}}
};

/**
 @breif レーンIDに対する隣接する交差点を定義
 @param lane_id, {junction_id}
 */
static unordered_set <pair<int, int>, HashPair> LOS_TABLE = {
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
	{4, 4},
	{5, 5},
	{6, 6},
	{7, 7},

	{10, 0},
	{0, 10},
	{10, 4},
	{4, 10},

	{10, 11},
	{10, 12},
	{10, 13},
	{10, 14},
	{10, 18},
	{10, 22},

	{11, 0},
	{0, 11},
	{11, 5},
	{5, 11},

	{11, 10},
	{11, 12},
	{11, 13},
	{11, 15},
	{11, 19},
	{11, 23},

	{12, 0},
	{0, 12},
	{12, 6},
	{6, 12},

	{12, 10},
	{12, 11},
	{12, 13},
	{12, 16},
	{12, 20},
	{12, 24},

	{13, 0},
	{0, 13},
	{13, 7},
	{7, 13},

	{13, 10},
	{13, 11},
	{13, 12},
	{13, 17},
	{13, 21},
	{13, 25},

	{14, 1},
	{1, 14},
	{14, 4},
	{4, 14},

	{14, 10},
	{14, 15},
	{14, 16},
	{14, 17},
	{14, 18},
	{14, 22},

	{15, 1},
	{1, 15},
	{15, 5},
	{5, 15},

	{15, 11},
	{15, 14},
	{15, 16},
	{15, 17},
	{15, 19},
	{15, 23},

	{16, 1},
	{1, 16},
	{16, 6},
	{6, 16},

	{16, 12},
	{16, 14},
	{16, 15},
	{16, 17},
	{16, 20},
	{16, 24},

	{17, 1},
	{1, 17},
	{17, 7},
	{7, 17},

	{17, 13},
	{17, 14},
	{17, 15},
	{17, 16},
	{17, 21},
	{17, 25},

	{18, 2},
	{2, 18},
	{18, 4},
	{4, 18},

	{18, 10},
	{18, 14},
	{18, 19},
	{18, 20},
	{18, 21},
	{18, 22},

	{19, 2},
	{2, 19},
	{19, 5},
	{5, 19},

	{19, 11},
	{19, 15},
	{19, 18},
	{19, 20},
	{19, 21},
	{19, 23},

	{20, 2},
	{2, 20},
	{20, 6},
	{6, 20},

	{20, 12},
	{20, 16},
	{20, 18},
	{20, 19},
	{20, 21},
	{20, 24},

	{21, 2},
	{2, 21},
	{21, 7},
	{7, 21},

	{21, 13},
	{21, 17},
	{21, 18},
	{21, 19},
	{21, 20},
	{21, 25},

	{22, 3},
	{3, 22},
	{22, 4},
	{4, 22},

	{22, 10},
	{22, 14},
	{22, 18},
	{22, 23},
	{22, 24},
	{22, 25},

	{23, 3},
	{3, 23},
	{23, 5},
	{5, 23},

	{23, 11},
	{23, 15},
	{23, 19},
	{23, 22},
	{23, 24},
	{23, 25},

	{24, 3},
	{3, 24},
	{24, 6},
	{6, 24},

	{24, 12},
	{24, 16},
	{24, 20},
	{24, 22},
	{24, 23},
	{24, 25},

	{25, 3},
	{3, 25},
	{25, 7},
	{7, 25},

	{25, 13},
	{25, 17},
	{25, 21},
	{25, 22},
	{25, 23},
	{25, 24}
};

#endif