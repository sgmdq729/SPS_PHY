#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include <random>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include "Table.h"
#include "SNR_BLER.h"

//#define FIX_SEED

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
/**システム帯域幅(Hz)*/
constexpr int BAND_WIDTH = 10 * 1000000;
/**雑音指数(dB)*/
constexpr int NOISE_FIGURE = 9;
/**雑音電力(mw)*/
const float NOISE_POWER = dB2mw(-174 + 10 * log10(BAND_WIDTH) + NOISE_FIGURE);
/**道路幅(m)*/
constexpr int STREET_WIDTH = 20;
/**LOS伝搬のブレイクポイント*/
constexpr float D_BP = 4 * EFFECTIVE_ANTENNA_HEIGHTS * EFFECTIVE_ANTENNA_HEIGHTS * FREQ * 1000000000 * (1 / C);
/**RRI*/
constexpr int RRI = 100;
/**センシングウィンドウ*/
constexpr int SENSING_WINDOW = 1000;
/**C1,C2*/
constexpr int C1 = 5;
constexpr int C2 = 15;
/**T1,T2*/
constexpr int T1 = 1;
constexpr int T2 = 100;
/**PRRの間隔*/
constexpr int PRR_border = 25;

using namespace std;

/**
 * @class Vehicle
 * @breif 車両クラス
 */
class Vehicle {
private:
	/**パケットサイズのモード 0:300byte, 1:190byte */
	const int packet_size_mode;
	/**伝搬損失モデルのモード 0:WINNER+B1, 1:freespace */
	const int prop_mode;
	/**リソース選択方式のモード 0:original 1:proposed */
	const int scheme_mode;
	/**サブチャネル数*/
	const int numSubCH;
	/**リソース維持確率*/
	const float probKeep;
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
	pair<int, int> txResource;
	/**2車両間の受信電力のキャッシュ<pair<id, id>, recvPower(mw)>*/
	unordered_map<pair<string, string>, float, HashPair> recvPowerMap;
	/**合計受信電力(mw)<subCH, sumRecvPower>*/
	unordered_map<int, float> sumRecvPower;
	/**センシングリスト*/
	vector<vector<float>> sensingList;
	/**乱数関連*/
	random_device seed;
	mt19937 engine;
	/**RCとSB用の乱数生成器*/
	uniform_int_distribution<> distRC, distSB;
	/**[0,1]の乱数生成器*/
	uniform_real_distribution<> dist;
	/**PRR計測<tx-rx distance, pair<num_success/num_fail>>*/
	unordered_map<int, pair<int, int>> resultMap;
	/**関数ポインタ配列*/
	/**BLER 0:300byte 1:190byte*/
	float (*getBLER[2])(float) = {getBLER_300, getBLER_190};
	/**path loss 0:WINNER+B1 1:freespace*/
	float (Vehicle::* getPathLoss[2])(const Vehicle*) = { &Vehicle::calcWINNER, &Vehicle::calcFreespace };
	/**resource reselection scheme 0:original 1:proposal*/
	void (Vehicle::* resourceReselection[2])(int) = { &Vehicle::originalSPS, &Vehicle::proposalSPS };

	/**
	 * 2点間の距離を求める
	 * @param x1, x2, y1, y2 各座標
	 * @param v 相手車両のインスタンス
	 * @retval 距離
	 */
	float getDistance(float x1, float x2, float y1, float y2);
	float getDistance(const Vehicle* v);


	/**
	 * リソース再選択
	 * @param subframe
	 */
	void originalSPS(int subframe);
	void proposalSPS(int subframe);

	/**
	 * 自由空間伝搬損失
	 * @param d 距離
	 * @param v 相手車両のインスタンス
	 * @retval 伝搬損失
	 */
	float calcFreespace(float d);
	float calcFreespace(const Vehicle* v);

	/**
	 * WINNER+B1モデル
	 * @param v 相手車両のインスタンス
	 */
	float calcWINNER(const Vehicle* v);

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
	int getMinJunctionHolPar(const Vehicle* v);
	int getMinJunctionVerPar(const Vehicle* v);

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
	 * @param lane_id 位置しているレーンID
	 * @param prob リソース維持確率
	 * @param size パケットサイズ 0:300byte 1:190byte
	 * @param prop 伝搬モデル 0:WINNTER+B1 1:自由空間
	 * @param scheme リソース再選択方式 0:original 1:proposal
	 * @param dummy 途中で生起する車両用のコンストラクタを識別するためのダミー変数
	 */
	Vehicle(string id, float x, float y, string lane_id, float prob, int size, int prop, int scheme);
	Vehicle(string id, float x, float y, string lane_id, float prob, int size, int prop, int scheme, int dummy);

	/**
	 * 車両インスタンスのIDのゲッター
	 * @retval ID
	 */
	string getID() {
		return id;
	}

	/**
	 * レーンIDゲッター
	 * @retval lane_ID
	 */
	int getLaneID() {
		return laneID;
	}

	/**
	 * RCのゲッター
	 * @retval RC
	 */
	int getRC() {
		return RC;
	}

	/**
	 * 減算した後のRCを返す
	 * @retval --RC
	 */
	int getDecRC() {
		return --RC;
	}

	/**
	 * txResourceのゲッター
	 * @retval txResource
	 */
	pair<int, int> getResource() {
		return txResource;
	}

	/**
	 * resultMapのゲッター
	 * @retval resultMap
	 */
	unordered_map<int, pair<int, int>> getResult() {
		return resultMap;
	}

	/**
	 * sumRecvPowerをリセット
	 */
	void resetRecvPower() {
		sumRecvPower.clear();
	}

	/**
	 * リソース再選択の判定
	 * @param subframe
	 */
	void decisionReselection(int subframe);

	/**
	 * 途中から生起した車両用 リソースを再選択
	 * @param subframe サブフレーム
	 */
	void resourceSelection(int subframe) {
		(this->*resourceReselection[scheme_mode])(subframe);
	}

	/**
	 * 車両の座標を更新
	 * @param x x座標
	 * @param y y座標
	 * @param lane_id laneのid
	 */
	void positionUpdate(float x, float y, string lane_id);

	/**
	 * sensingListの更新
	 * @param t 前回のイベント時間との差
	 */
	void sensingListUpdate(int t);

	/**
	 * 2車両間の受信電力計算，計算結果をキャッシュとして保存
	 * @param v 相手車両のインスタンス
	 * @param cache キャッシュ
	 */
	void calcRecvPower(const Vehicle* v, unordered_map<pair<string, string>, float, HashPair>& caceh);

	/**
	 * パケットの受信成功を判断
	 * @param v 相手車両のインスタンス
	 * @param cache キャッシュ
	 */
	void decisionPacket(const Vehicle* v, unordered_map<pair<string, string>, float, HashPair>& cache);

	/**
	 * 半二重送信のパケット受信判定
	 * @param v 相手送信車両のインスタンス
	 */
	void calcHalfDup(const Vehicle* v);
};






/***************************************関数の定義***************************************/
inline Vehicle::Vehicle(string id, float x, float y, string lane_id, float prob, int packet_size, int prop, int scheme)
	: id(id), probKeep(prob), numSubCH(packet_size + 2), packet_size_mode(packet_size), prop_mode(prop), scheme_mode(scheme)
{
	this->x = x;
	this->y = y;
	this->laneID = stoi(lane_id.substr(1, 3));

#ifndef FIX_SEED
	engine.seed(seed());
#else
	engine.seed(stoi(id));
#endif

	uniform_int_distribution<> distTXTime, distTXSubCH;
	uniform_real_distribution<>::param_type param(0.0, 1.0);
	uniform_int_distribution<>::param_type paramRC(C1, C2);
	uniform_int_distribution<>::param_type paramTXTime(1, 100);
	uniform_int_distribution<>::param_type paramTXSubCH(0, numSubCH - 1);

	dist.param(param);
	distRC.param(paramRC);
	distTXTime.param(paramTXTime);
	distTXSubCH.param(paramTXSubCH);

	RC = distRC(engine);
	txResource = make_pair(distTXTime(engine), distTXSubCH(engine));
	sensingList.assign(SENSING_WINDOW, vector<float>(numSubCH, 0));
}

inline Vehicle::Vehicle(string id, float x, float y, string lane_id, float prob, int packet_size, int prop, int scheme, int dummy)
	: id(id), probKeep(prob), numSubCH(packet_size + 2), packet_size_mode(packet_size), prop_mode(prop), scheme_mode(scheme)
{
	this->x = x;
	this->y = y;
	this->laneID = stoi(lane_id.substr(1, 3));

#ifndef FIX_SEED
	engine.seed(seed());
#else
	engine.seed(stoi(id));
#endif

	uniform_real_distribution<>::param_type param(0.0, 1.0);
	uniform_int_distribution<>::param_type paramRC(C1, C2);

	dist.param(param);
	distRC.param(paramRC);

	RC = 15;
	txResource = make_pair(INT_MAX, INT_MAX);
	sensingList.assign(SENSING_WINDOW, vector<float>(numSubCH, 0));
}

inline void Vehicle::positionUpdate(float x, float y, string lane_id) {
	this->x = x;
	this->y = y;
	this->laneID = stoi(lane_id.substr(1, 3));
}


inline float Vehicle::getDistance(const Vehicle* v) {
	return sqrt((x - v->x) * (x - v->x) + (y - v->y) * (y - v->y));
}

inline float Vehicle::getDistance(float x1, float x2, float y1, float y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

/**************************************MAC**************************************/

inline void Vehicle::sensingListUpdate(int t) {
	sensingList.assign(sensingList.begin() + t, sensingList.end());
	int gapSize = SENSING_WINDOW - sensingList.size();
	sensingList.insert(sensingList.end(), gapSize, vector<float>(numSubCH, 0));
}

inline void Vehicle::decisionReselection(int subframe) {
	/**RCチェック*/
	if (--RC == 0){
		if (dist(engine) > probKeep) {
			/**リソース再選択*/
			(this->*resourceReselection[scheme_mode])(subframe);
		}
		else {
			/**リソース再選択しない場合*/
			txResource.first += RRI;
			RC = distRC(engine);
		}
	}
	else {
		txResource.first += RRI;
	}
}

inline void Vehicle::originalSPS(int subframe) {
	multimap<float, pair<int, int>> map;
	/**線形平均を計算*/
	for (int i = 0; i < RRI;i++) {
		for (int j = 0;j < numSubCH;j++) {
			float sum = 0;
			for (int k = 0; k < 10; k++) {
				sum += sensingList[i + (100 * k)][j];
			}
			map.emplace(make_pair(sum, make_pair(i, j)));
		}
	}
	/**上位20%の位置を検索*/
	auto border = distance(map.begin(), map.upper_bound(next(map.begin(),
		(int)ceil(((T2 - T1) * numSubCH) * 0.2))->first));
	uniform_int_distribution<>::param_type paramSB(0, border - 1);
	distSB.param(paramSB);
	auto nextResource = next(map.begin(), distSB(engine));
	/**送信リソース更新*/
	txResource.first = nextResource->second.first + subframe + 1;
	txResource.second = nextResource->second.second;
	RC = distRC(engine);
}

inline void Vehicle::proposalSPS(int subframe) {
	cerr << "not implement" << endl;
	exit(-100);
	RC = distRC(engine);
}

/**************************************PHY**************************************/

inline float Vehicle::calcFreespace(float d) {
	return 20.0 * log10(4 * PI * d / LAMBDA);
}

inline float Vehicle::calcFreespace(const Vehicle* v) {
	return 20.0 * log10(4 * PI * getDistance(v) / LAMBDA);
}

/**TODO
 * チャネルごとに受信電力をキャッシュ
 */
inline void Vehicle::calcRecvPower(const Vehicle* v, unordered_map<pair<string, string>, float, HashPair>& cache) {
	float pathLoss = 0;
	float fadingLoss = 0;
	float shadowingLoss = 0;
	float recvPower_mw = 0;

	/**キャッシュがあるか確認*/
	if (cache.count(make_pair(min(id, v->id), max(id, v->id))) == 0) {
		/**キャッシュがない場合は計算*/
		pathLoss = (this->*getPathLoss[prop_mode])(v);
		float recvPower_dB = TX_POWER + ANNTENA_GAIN + ANNTENA_GAIN - pathLoss - fadingLoss - shadowingLoss;
		recvPower_mw = dB2mw(recvPower_dB);
		sumRecvPower[v->txResource.second] += recvPower_mw;
		cache[make_pair(min(id, v->id), max(id, v->id))] = recvPower_mw;
	}
	else {
		sumRecvPower[v->txResource.second] += cache[make_pair(min(id, v->id), max(id, v->id))];
		recvPower_mw = cache[make_pair(min(id, v->id), max(id, v->id))];
	}
	/**sensingList更新*/
	sensingList[SENSING_WINDOW - 1][v->txResource.second] += recvPower_mw;
}

inline float Vehicle::calcWINNER(const Vehicle* v) {
	/**LOSかNLOSか*/
	if (LOS_TABLE.count(make_pair(laneID / 10, v->laneID / 10))) {
		/**LOS*/
		return calcLOS(getDistance(v));
	}
	else {
		/**NLOS*/
		return calcNLOS(v);
	}
}

inline float Vehicle::calcLOS(float d) {
	/**2車両間の距離を計算*/

	if (d < 10) {
		/**自由空間伝搬損失*/
		return calcFreespace(d);
	}
	else if (d < D_BP) {
		/**WINNER+ LOS 10m<dis<D_BP*/
		return 22.7 * log10(d) + 27.0 + 20.0 * log10(FREQ);
	}
	else {
		/**WINNER+ LOS D_BP<dis*/
		return 40 * log10(d) + 7.56 - 17.3 * log10(EFFECTIVE_ANTENNA_HEIGHTS)
			- 17.3 * log10(EFFECTIVE_ANTENNA_HEIGHTS) + 2.7 * log10(FREQ);
	}
}

inline float Vehicle::calcNLOS(const Vehicle* v) {
	/**2車両が位置するパターンで切り替え*/
	switch (RELATION_TABLE[make_pair(laneID / 10, v->laneID / 10)]) {

	case PositionRelation::NORMAL:
		return NLOS(abs(x - v->x), abs(y - v->y));

	case PositionRelation::HOL_PAR:
		return NLOSHolPar(v);

	case PositionRelation::VER_PAR:
		return NLOSVerPar(v);
	default:
		cerr << "unknown Relation: " << static_cast<int>(RELATION_TABLE[make_pair(laneID / 10, v->laneID / 10)]) << endl;
		exit(-1);
	}
}

inline float Vehicle::NLOS(float d1, float d2) {
	return min(getNLOS(d1, d2), getNLOS(d2, d1));
}

inline float Vehicle::NLOSHolPar(const Vehicle* v) {
	int minJunction = getMinJunctionHolPar(v);
	pair<float, float> junction1 = JUNCTION_TABLE[minJunction];
	float d1 = abs(v->x - junction1.first);
	float d2 = abs(y - v->y);
	float d3 = abs(x - junction1.first);
	return max(NLOS(d1, d2 + d3), NLOS(d1 + d2, d3));
}

inline float Vehicle::NLOSVerPar(const Vehicle* v) {
	int minJunction = getMinJunctionVerPar(v);
	pair<float, float> junction1 = JUNCTION_TABLE[minJunction];
	float d1 = abs(v->y - junction1.second);
	float d2 = abs(x - v->x);
	float d3 = abs(y - junction1.second);
	return max(NLOS(d1, d2 + d3), NLOS(d1 + d2, d3));
}

inline float Vehicle::getNLOS(float d1, float d2) {
	float n_j = max(2.8 - 0.0024 * d1, 1.84);
	return calcLOS(d1) + 17.3 - 12.5 * n_j + 10.0 * n_j * log10(d2) + 3.0 * log10(FREQ);
}

inline int Vehicle::getMinJunctionHolPar(const Vehicle* v) {
	map<float, int> pathMap;
	for (auto&& p : ADJACENT_JUNCTION_TABLE[laneID]) {
		float d1 = abs(x - get<1>(p));
		float d2 = abs(get<1>(p) - v->x);
		pathMap[d1 + d2] = get<0>(p);
	}
	return pathMap.begin()->second;
}

inline int Vehicle::getMinJunctionVerPar(const Vehicle* v) {
	map<float, int> pathMap;
	for (auto&& p : ADJACENT_JUNCTION_TABLE[laneID]) {
		float d1 = abs(y - get<2>(p));
		float d2 = abs(get<2>(p) - v->y);
		pathMap[d1 + d2] = get<0>(p);
	}
	return pathMap.begin()->second;
}

inline void Vehicle::decisionPacket(const Vehicle* v, unordered_map<pair<string, string>, float, HashPair>& cache) {
	float recvPower_mw = cache[make_pair(min(id, v->id), max(id, v->id))];

	float sinr_mw = recvPower_mw / (sumRecvPower[v->txResource.second] - recvPower_mw + NOISE_POWER);
	float sinr_dB = mw2dB(sinr_mw);
	float rand = dist(engine);
	float bler = (*getBLER[packet_size_mode])(sinr_dB);
	int index = (int)(floor(getDistance(v)) / PRR_border) * PRR_border;

	if (rand > bler) {
		resultMap[index].first++;
	}
	else {
		resultMap[index].second++;
	}
}

inline void Vehicle::calcHalfDup(const Vehicle* v) {
	sensingList[SENSING_WINDOW - 1] = vector<float>(numSubCH, FLT_MAX);
	int index = (int)(floor(getDistance(v)) / PRR_border) * PRR_border;
	resultMap[index].second++;
}

#endif