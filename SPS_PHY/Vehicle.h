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
/**アンテナゲイン(dBi)*/
constexpr int ANNTENA_GAIN = 3;
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
	/**2車両間の受信電力のキャッシュ*/
	unordered_map<pair<string, string>, float, HashPair, less<>> recvPowerMap;
	/**合計受信電力*/
	float sumRecvPower;
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
	 * 2点間の距離を求める
	 * @param x1, x2, y1, y2 各座標
	 * @retval 距離
	 */
	float getDistance(float x1, float x2, float y1, float y2);

	/**
	 * WINNER+B1 LOSモデル
	 * @param v 相手車両のインスタンス
	 * @retval LOS伝搬損失
	 */
	float calcLOS(float d);

	/**
	 * WINNER+B1 NLOSモデル
	 * @param v 相手車両のインスタンス
	 * @param NLOS伝搬損失
	 */
	float calcNLOS(const Vehicle* v);

	/**
	 * NLOSの計算
	 * @param d1 経路1
	 * @param d2 経路2
	 * @retval 経路1と経路2をそれぞれd1，d2とした際の小さい方の伝搬損失
	 */
	float getNLOS(float d1, float d2);

	/**
	* WINNER+B1 NLOS
	* @param d1
	* @param d2
	* @retval あるd1，d2によるNLOS伝搬損失
	*/
	float NLOS(float d1, float d2);

	/**
	 * 最短経路のうち、自車両と相手車両の最寄交差点を求める
	 * @param v 相手車両のインスタンス
	 * @retval pair<自車の最寄交差点, 相手車両の最寄交差点>
	 */
	pair<int, int> getMinJunction(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS 横並列
	 * @param v 相手車両のインスタンス
	 */
	float NLOSHolPar(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS 縦並列
	 * @param v 相手車両のインスタンス
	 */
	float NLOSVerPar(const Vehicle* v);



public:
	/**
	 * コンストラクタ
	 * @param id 車両ID
	 * @param x,y 座標
	 * @param subframe 生起時のsubframe
	 */
	Vehicle(string id, float x, float y, string lane_id, int subframe);

	/**
	 * 車両の座標を更新
	 * @param x x座標
	 * @param y y座標
	 * @param lane_id laneのid
	 */
	void positionUpdate(float x, float y, string lane_id);

	/**
	 * 2車両間の受信電力計算，計算結果をキャッシュとして保存
	 * @param v 相手車両のインスタンス
	 */
	void calcRecvPower(const Vehicle* v);

	/**
	* パケットの受信成功を判断
	* @param v 相手車両のインスタンス
	*/
	void decisionPacket(const Vehicle* v);
};

/***************************************関数の定義***************************************/
inline Vehicle::Vehicle(string id, float x, float y, string lane_id, int subframe) : id(id)
{
	this->x = x;
	this->y = y;
	this->laneID = stoi(lane_id.substr(1, 3));
	//cout << id << " generated in " << laneID << endl;
}

inline void Vehicle::positionUpdate(float x, float y, string lane_id) {
	this->x = x;
	this->y = y;
	this->laneID = stoi(lane_id.substr(1, 3));
}




/**************************************伝搬損失関係**************************************/

inline float Vehicle::getDistance(float x1, float x2, float y1, float y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 * y2) * (y1 * y2));
}

inline void Vehicle::calcRecvPower(const Vehicle* v) {
	sumRecvPower = 0;
	float pathLoss = 0;
	float fadingLoss = 0;
	float shadowingLoss = 0;
	/**キャッシュがあるか確認*/

	/**キャッシュがない場合は計算*/
	/**LOSかNLOSか*/
	if (LOS_TABLE.count(make_pair(this->laneID / 10, v->laneID / 10))) {
		/**LOS*/
		cout << "(" << id << "," << v->id << "): LOS";
		pathLoss = calcLOS(getDistance(x, v->x, y, v->y));
	}
	else {
		/**NLOS*/
		pathLoss = calcNLOS(v);
	}
	cout << " path loss:" << pathLoss << endl;
	float recvPower_dB = TX_POWER + ANNTENA_GAIN + ANNTENA_GAIN - pathLoss - fadingLoss - shadowingLoss;
	recvPowerMap[make_pair(min(id, v->id), max(id, v->id))] = recvPower_dB;
}

inline float Vehicle::calcLOS(float d) {
	/**2車両間の距離を計算*/
	
	if (10 < d) {
		/**自由空間伝搬損失*/
		return 20 * log10(4 * PI * d / LAMBDA);
	}
	else if (D_BP < d) {
		/**WINNER+ LOS 10m<dis<D_BP*/
		return 22.7 * log10(d) + 27.0 + 20 * log10(FREQ);
	}
	else {
		/**WINNER+ LOS D_BP<dis*/
		return 40 * log10(d) + 7.56 - 17.3 * log10(EFFECTIVE_ANTENNA_HEIGHTS)
			- 17.3 * log10(EFFECTIVE_ANTENNA_HEIGHTS) + 2.7 * log10(FREQ);
	}
}

inline float Vehicle::calcNLOS(const Vehicle* v) {
	pair<int, int> minElem = getMinJunction(v);
	/**2車両が位置するパターンで切り替え*/
	switch (RELATION_TABLE[make_pair(laneID / 10, v->laneID / 10)]) {

	case PositionRelation::NORMAL:
		/**2車両が並列に位置していない場合*/
		cout << "(" << id << "," << v->id << "): NLOS, NORMAL" << " min:(" << minElem.first << ", " << minElem.second << ")";
		return NLOS(abs(x - v->x), abs(y - v->y));

	case PositionRelation::HOL_PAR:
		/**2車両が横並列に位置してる場合*/
		cout << "(" << id << "," << v->id << "): NLOS, HOL_PAR" << " min:(" << minElem.first << ", " << minElem.second << ")";
		return NLOSHolPar(v);

	case PositionRelation::VER_PAR:
		/**2車両が縦並列に位置している場合*/
		cout << "(" << id << "," << v->id << "): NLOS, VER_PAR" << " min:(" << minElem.first << ", " << minElem.second << ")";
		return NLOSVerPar(v);
	default:
		cerr << "unknown Relation: " << static_cast<int>(RELATION_TABLE[make_pair(laneID / 10, v->laneID / 10)]) << endl;
		exit(-1);
	}
}

inline float Vehicle::getNLOS(float d1, float d2) {
	float n_j = max(2.8 - 0.0024 * d1, 1.84);
	return calcLOS(d1) + 17.3 - 12.5 * n_j + 10 * n_j * log10(d2) + 3 * log10(FREQ);
}

inline float Vehicle::NLOS(float d1, float d2) {
	return min(getNLOS(d1, d2), getNLOS(d2, d1));
}

inline pair<int, int> Vehicle::getMinJunction(const Vehicle* v) {
	map<float, pair<int, int>> pathMap;
	for (auto&& p1 : ADJACENT_JUNCTION_TABLE[laneID]) {
		float d1 = getDistance(x, get<1>(p1), y, get<2>(p1));
		int junction1 = get<0>(p1);
		for (auto&& p2 : ADJACENT_JUNCTION_TABLE[v->laneID]) {
			float d2 = getDistance(v->x, get<1>(p2), v->y, get<2>(p2));
			int junction2 = get<0>(p2);
			pathMap[d1 + d2 + DISTANCE_TABLE[make_pair(junction1, junction2)]] = make_pair(junction1, junction2);
		}
	}
	return pathMap.begin()->second;
}

inline float Vehicle::NLOSHolPar(const Vehicle* v) {
	pair<int, int> minJunction = getMinJunction(v);
	pair<float, float> junction1 = JUNCTION_TABLE[minJunction.first];
	pair<float, float> junction2 = JUNCTION_TABLE[minJunction.second];
	float d1 = abs(x - junction1.first);
	float d2 = abs(y - v->y);
	float d3 = abs(v->x - junction2.first);
	return max(NLOS(d1, d2 + d3), NLOS(d1 + d2, d3));
}

inline float Vehicle::NLOSVerPar(const Vehicle* v) {
	pair<int, int> minJunction = getMinJunction(v);
	pair<float, float> junction1 = JUNCTION_TABLE[minJunction.first];
	pair<float, float> junction2 = JUNCTION_TABLE[minJunction.second];
	float d1 = abs(y - junction1.second);
	float d2 = abs(x - v->x);
	float d3 = abs(v->y - junction2.second);
	return max(NLOS(d1, d2 + d3), NLOS(d1 + d2, d3));
}

#endif