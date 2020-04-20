#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include <random>
#include <iostream>
#include <map>
#include <string>
#include "Table.h"

/**円周率*/
constexpr double PI = 3.14159265358979323846;
/**光速(m/s)*/
constexpr double C = 299792458;
/**送信電力(dBm)*/
constexpr int TX_POWER = 23;
/**周波数(GHz)*/
constexpr float FREQ = 6;
/**波長(m)*/
constexpr double LAMBDA = C / (FREQ * 1000000000);
/**有効アンテナ高(m)*/
constexpr float EFFECTIVE_ANTENNA_HEIGHTS = 0.5;
/**道路幅(m)*/
constexpr int STREET_WIDTH = 20;
/**LOS伝搬のブレイクポイント*/
constexpr float D_BP = 4 * EFFECTIVE_ANTENNA_HEIGHTS * EFFECTIVE_ANTENNA_HEIGHTS * FREQ * 1000000000 * (1 / C);
/**C1,C2*/
constexpr int C1 = 5;
constexpr int C2 = 15;
/**T1,T2*/
constexpr int T1 = 1;
constexpr int T2 = 100;

using namespace std;

/**
 * @class Vehicle
 * @breif 車両クラス
 */
class Vehicle {
private:
	/**時刻(ms)*/
	int subframe = 0;
	/**車両id*/
	const string id;
	/**x座標*/
	float x = 0;
	/**y座標*/
	float y = 0;
	/**lane id*/
	int laneID;
	/**RC*/
	int RC = 0;
	/**次に送信するsubframeとsubCH*/
	pair<int, int> nextResource;
	/**センシングリスト*/
	vector<vector<float>> sensingList;
	/**乱数関連*/
	random_device seed;
	mt19937 engine;
	/**RCとSB用の乱数生成器*/
	uniform_int_distribution<> distRC, distSB;
	/**リソース再選択判定用の乱数生成器*/
	uniform_real_distribution<> distResourceKeep;

	pair<int, int> preTxResourceLocation;
	bool isReselection = false;
	int preSetRC = -1;
	int setRC = 0;
	int num_subCH = 0;
	float prob_resource_keep = 0;

	/**
	 * WINNER+B1 LOSモデル
	 * @param v 相手車両のインスタンス
	 */
	float calcLOS(const Vehicle* v);

	/**
	 * WINNER+B1 NLOSモデル
	 * @param v 相手車両のインスタンス
	 */
	float calcNLOS(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS 横並列
	 * @param v 相手車両のインスタンス
	 */
	float calcNLOSHolPar(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS 縦並列
	 * @param v 相手車両のインスタンス
	 */
	float calcNLOSVerPar(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS
	 * @param v 相手車両のインスタンス
	 */
	float calcNLOSNormal(const Vehicle* v);

public:
	/**
	 * コンストラクタ
	 * @param id 車両ID
	 * @param x,y 座標
	 * @param subframe 生起時のsubframe
	 */
	Vehicle(string id, float x, float y, string lane_id, int subframe);

	/**
	 * 2車両間の受信電力計算
	 * @param v 相手車両のインスタンス
	 */
	void calcRecvPower(const Vehicle* v);


	/**
	 * 車両の座標を更新
	 * @param x x座標
	 * @param y y座標
	 * @param lane_id laneのid
	 */
	void positionUpdate(float x, float y, string lane_id);

};

inline Vehicle::Vehicle(string id, float x, float y, string lane_id, int subframe) : id(id)
{
	this->x = x;
	this->y = y;
	this->laneID = stoi(lane_id.substr(1, 3));
	cout << id << " generated in " << laneID << endl;
}

inline void Vehicle::positionUpdate(float x, float y, string lane_id) {
	this->x = x;
	this->y = y;
	this->laneID = stoi(lane_id.substr(1, 3));
}

inline void Vehicle::calcRecvPower(const Vehicle* v) {
	/**キャッシュがあるか確認*/

	/**キャッシュがない場合は計算*/
	/**LOSかNLOSか*/
	if (LOS_TABLE.count(make_pair(this->laneID / 10, v->laneID / 10))) {
		/**LOS*/
		cout << "(" << id << "," << v->id << "): LOS" << endl;
	}
	else {
		/**NLOS*/
		calcNLOS(v);
	}
}

inline float Vehicle::calcLOS(const Vehicle* v) {
	/**2車両間の距離を計算*/
	float dis = sqrt((x - v->x) * (x - v->x) + (y - v->y) * (y - v->y));
	if (10 < dis) {
		/**自由空間伝搬損失*/
		return 20 * log10(4 * PI * dis / LAMBDA);
	}
	else if (D_BP < dis) {
		/**WINNER+ LOS 10m<dis<D_BP*/
		return 22.7 * log10(dis) + 27.0 + 20 * log10(FREQ);
	}
	else {
		/**WINNER+ LOS D_BP<dis*/
		return 40 * log10(dis) + 7.56 - 17.3 * log10(EFFECTIVE_ANTENNA_HEIGHTS)
			- 17.3 * log10(EFFECTIVE_ANTENNA_HEIGHTS) + 2.7 * log10(FREQ);
	}
}

inline float Vehicle::calcNLOS(const Vehicle* v) {
	/**<距離, pair<junction_id, junction_id>*/
	map<float, pair<int, int>> pathMap;

	/**自車両と相手車両との最短経路を求める*/
	for (auto&& p1 : ADJACENT_JUNCTION_TABLE[laneID]) {
		double d1 = sqrt((x - get<1>(p1)) * (x - get<1>(p1)) + (y - get<2>(p1)) * (y - get<2>(p1)));
		int junction1 = get<0>(p1);
		for (auto&& p2 : ADJACENT_JUNCTION_TABLE[v->laneID]) {
			double d2 = sqrt((v->x - get<1>(p2)) * (v->x - get<1>(p2)) + (v->y - get<2>(p2)) * (v->y - get<2>(p2)));
			int junction2 = get<0>(p2);
			pathMap[d1 + d2 + DISTANCE_TABLE[make_pair(junction1, junction2)]] = make_pair(junction1, junction2);
		}
	}

	auto&& minElem = pathMap.begin();
	int minJunction1 = pathMap.begin()->second.first;
	int minJunction2 = pathMap.begin()->second.second;

	/**2車両が位置するパターンで切り替え*/
	switch (RELATION_TABLE[make_pair(laneID / 10, v->laneID / 10)]) {
	case PositionRelation::HOL_PAR:
		cout << "(" << id << "," << v->id << "): NLOS, HOL_PAR" << " min:(" << minElem->second.first << ", " << minElem->second.second << ") : " << minElem->first << "m" << endl;
		break;
	case PositionRelation::VER_PAR:
		cout << "(" << id << "," << v->id << "): NLOS, VER_PAR" << " min:(" << minElem->second.first << ", " << minElem->second.second << ") : " << minElem->first << "m" << endl;
		break;
	case PositionRelation::NORMAL:
		cout << "(" << id << "," << v->id << "): NLOS, NORMAL" << " min:(" << minElem->second.first << ", " << minElem->second.second << ") : " << minElem->first << "m" << endl;
		break;
	default:
		cerr << "unknown Relation: " << static_cast<int>(RELATION_TABLE[make_pair(laneID / 10, v->laneID / 10)]) << endl;
		exit(-1);
	}

	return -1;
}

inline float Vehicle::calcNLOSHolPar(const Vehicle* v) {
	return -1;
}

inline float Vehicle::calcNLOSVerPar(const Vehicle* v) {
	return -1;
}

inline float Vehicle::calcNLOSNormal(const Vehicle* v) {
	return -1;
}

#endif