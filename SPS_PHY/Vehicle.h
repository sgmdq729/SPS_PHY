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

/**�~����*/
constexpr double PI = 3.14159265358979323846;
/**����(m/s)*/
constexpr double C = 299792458;
/**���M�d��(dBm)*/
constexpr int TX_POWER = 23;
/**���g��(GHz)*/
constexpr float FREQ = 6;
/**�g��(m)*/
constexpr double LAMBDA = C / (FREQ * 1000000000);
/**�L���A���e�i��(m)*/
constexpr float EFFECTIVE_ANTENNA_HEIGHTS = 0.5;
/**�A���e�i�Q�C��(dBi)*/
constexpr int ANNTENA_GAIN = 3;
/**�V�X�e���ш敝(Hz)*/
constexpr int BAND_WIDTH = 10 * 1000000;
/**�G���w��(dB)*/
constexpr int NOISE_FIGURE = 9;
/**�G���d��(mw)*/
const float NOISE_POWER = dB2mw(-174 + 10 * log10(BAND_WIDTH) + NOISE_FIGURE);
/**���H��(m)*/
constexpr int STREET_WIDTH = 20;
/**LOS�`���̃u���C�N�|�C���g*/
constexpr float D_BP = 4 * EFFECTIVE_ANTENNA_HEIGHTS * EFFECTIVE_ANTENNA_HEIGHTS * FREQ * 1000000000 * (1 / C);
/**RRI*/
constexpr int RRI = 100;
/**�Z���V���O�E�B���h�E*/
constexpr int SENSING_WINDOW = 1000;
/**C1,C2*/
constexpr int C1 = 5;
constexpr int C2 = 15;
/**T1,T2*/
constexpr int T1 = 1;
constexpr int T2 = 100;
/**PRR�̊Ԋu*/
constexpr int PRR_border = 25;

using namespace std;

/**
 * @class Vehicle
 * @breif �ԗ��N���X
 */
class Vehicle {
private:
	/**�p�P�b�g�T�C�Y�̃��[�h 0:300byte, 1:190byte */
	const int packet_size_mode;
	/**�`���������f���̃��[�h 0:WINNER+B1, 1:freespace */
	const int prop_mode;
	/**���\�[�X�I������̃��[�h 0:original 1:proposed */
	const int scheme_mode;
	/**�T�u�`���l����*/
	const int numSubCH;
	/**���\�[�X�ێ��m��*/
	const float probKeep;
	/**����(ms)*/
	int subframe = 0;
	/**�ԗ�id*/
	const string id;
	/**x���W*/
	float x = 0;
	/**y���W*/
	float y = 0;
	/**lane id*/
	int laneID;
	/**RC*/
	int RC = 0;
	/**���ɑ��M����subframe��subCH*/
	pair<int, int> txResource;
	/**2�ԗ��Ԃ̎�M�d�͂̃L���b�V��<pair<id, id>, recvPower(mw)>*/
	unordered_map<pair<string, string>, float, HashPair> recvPowerMap;
	/**���v��M�d��(mw)<subCH, sumRecvPower>*/
	unordered_map<int, float> sumRecvPower;
	/**�Z���V���O���X�g*/
	vector<vector<float>> sensingList;
	/**�����֘A*/
	random_device seed;
	mt19937 engine;
	/**RC��SB�p�̗���������*/
	uniform_int_distribution<> distRC, distSB;
	/**[0,1]�̗���������*/
	uniform_real_distribution<> dist;
	/**PRR�v��<tx-rx distance, pair<num_success/num_fail>>*/
	unordered_map<int, pair<int, int>> resultMap;
	/**�֐��|�C���^�z��*/
	/**BLER 0:300byte 1:190byte*/
	float (*getBLER[2])(float) = {getBLER_300, getBLER_190};
	/**path loss 0:WINNER+B1 1:freespace*/
	float (Vehicle::* getPathLoss[2])(const Vehicle*) = { &Vehicle::calcWINNER, &Vehicle::calcFreespace };
	/**resource reselection scheme 0:original 1:proposal*/
	void (Vehicle::* resourceReselection[2])(int) = { &Vehicle::originalSPS, &Vehicle::proposalSPS };

	/**
	 * 2�_�Ԃ̋��������߂�
	 * @param x1, x2, y1, y2 �e���W
	 * @param v ����ԗ��̃C���X�^���X
	 * @retval ����
	 */
	float getDistance(float x1, float x2, float y1, float y2);
	float getDistance(const Vehicle* v);


	/**
	 * ���\�[�X�đI��
	 * @param subframe
	 */
	void originalSPS(int subframe);
	void proposalSPS(int subframe);

	/**
	 * ���R��ԓ`������
	 * @param d ����
	 * @param v ����ԗ��̃C���X�^���X
	 * @retval �`������
	 */
	float calcFreespace(float d);
	float calcFreespace(const Vehicle* v);

	/**
	 * WINNER+B1���f��
	 * @param v ����ԗ��̃C���X�^���X
	 */
	float calcWINNER(const Vehicle* v);

	/**
	 * WINNER+B1 LOS���f��
	 * @param v ����ԗ��̃C���X�^���X
	 * @retval LOS�`������
	 */
	float calcLOS(float d);

	/**
	 * WINNER+B1 NLOS���f��
	 * @param v ����ԗ��̃C���X�^���X
	 * @param NLOS�`������
	 */
	float calcNLOS(const Vehicle* v);

	/**
	 * NLOS�̌v�Z
	 * @param d1 �o�H1
	 * @param d2 �o�H2
	 * @retval �o�H1�ƌo�H2�����ꂼ��d1�Cd2�Ƃ����ۂ̏��������̓`������
	 */
	float getNLOS(float d1, float d2);

	/**
	* WINNER+B1 NLOS
	* @param d1
	* @param d2
	* @retval ����d1�Cd2�ɂ��NLOS�`������
	*/
	float NLOS(float d1, float d2);

	/**
	 * �ŒZ�o�H�̂����A���ԗ��Ƒ���ԗ��̍Ŋ�����_�����߂�
	 * @param v ����ԗ��̃C���X�^���X
	 * @retval pair<���Ԃ̍Ŋ�����_, ����ԗ��̍Ŋ�����_>
	 */
	int getMinJunctionHolPar(const Vehicle* v);
	int getMinJunctionVerPar(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS ������
	 * @param v ����ԗ��̃C���X�^���X
	 */
	float NLOSHolPar(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS �c����
	 * @param v ����ԗ��̃C���X�^���X
	 */
	float NLOSVerPar(const Vehicle* v);


public:
	/**
	 * �R���X�g���N�^
	 * @param id �ԗ�ID
	 * @param x,y ���W
	 * @param lane_id �ʒu���Ă��郌�[��ID
	 * @param prob ���\�[�X�ێ��m��
	 * @param size �p�P�b�g�T�C�Y 0:300byte 1:190byte
	 * @param prop �`�����f�� 0:WINNTER+B1 1:���R���
	 * @param scheme ���\�[�X�đI����� 0:original 1:proposal
	 * @param dummy �r���Ő��N����ԗ��p�̃R���X�g���N�^�����ʂ��邽�߂̃_�~�[�ϐ�
	 */
	Vehicle(string id, float x, float y, string lane_id, float prob, int size, int prop, int scheme);
	Vehicle(string id, float x, float y, string lane_id, float prob, int size, int prop, int scheme, int dummy);

	/**
	 * �ԗ��C���X�^���X��ID�̃Q�b�^�[
	 * @retval ID
	 */
	string getID() {
		return id;
	}

	/**
	 * ���[��ID�Q�b�^�[
	 * @retval lane_ID
	 */
	int getLaneID() {
		return laneID;
	}

	/**
	 * RC�̃Q�b�^�[
	 * @retval RC
	 */
	int getRC() {
		return RC;
	}

	/**
	 * ���Z�������RC��Ԃ�
	 * @retval --RC
	 */
	int getDecRC() {
		return --RC;
	}

	/**
	 * txResource�̃Q�b�^�[
	 * @retval txResource
	 */
	pair<int, int> getResource() {
		return txResource;
	}

	/**
	 * resultMap�̃Q�b�^�[
	 * @retval resultMap
	 */
	unordered_map<int, pair<int, int>> getResult() {
		return resultMap;
	}

	/**
	 * sumRecvPower�����Z�b�g
	 */
	void resetRecvPower() {
		sumRecvPower.clear();
	}

	/**
	 * ���\�[�X�đI���̔���
	 * @param subframe
	 */
	void decisionReselection(int subframe);

	/**
	 * �r�����琶�N�����ԗ��p ���\�[�X���đI��
	 * @param subframe �T�u�t���[��
	 */
	void resourceSelection(int subframe) {
		(this->*resourceReselection[scheme_mode])(subframe);
	}

	/**
	 * �ԗ��̍��W���X�V
	 * @param x x���W
	 * @param y y���W
	 * @param lane_id lane��id
	 */
	void positionUpdate(float x, float y, string lane_id);

	/**
	 * sensingList�̍X�V
	 * @param t �O��̃C�x���g���ԂƂ̍�
	 */
	void sensingListUpdate(int t);

	/**
	 * 2�ԗ��Ԃ̎�M�d�͌v�Z�C�v�Z���ʂ��L���b�V���Ƃ��ĕۑ�
	 * @param v ����ԗ��̃C���X�^���X
	 * @param cache �L���b�V��
	 */
	void calcRecvPower(const Vehicle* v, unordered_map<pair<string, string>, float, HashPair>& caceh);

	/**
	 * �p�P�b�g�̎�M�����𔻒f
	 * @param v ����ԗ��̃C���X�^���X
	 * @param cache �L���b�V��
	 */
	void decisionPacket(const Vehicle* v, unordered_map<pair<string, string>, float, HashPair>& cache);

	/**
	 * ����d���M�̃p�P�b�g��M����
	 * @param v ���著�M�ԗ��̃C���X�^���X
	 */
	void calcHalfDup(const Vehicle* v);
};






/***************************************�֐��̒�`***************************************/
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
	/**RC�`�F�b�N*/
	if (--RC == 0){
		if (dist(engine) > probKeep) {
			/**���\�[�X�đI��*/
			(this->*resourceReselection[scheme_mode])(subframe);
		}
		else {
			/**���\�[�X�đI�����Ȃ��ꍇ*/
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
	/**���`���ς��v�Z*/
	for (int i = 0; i < RRI;i++) {
		for (int j = 0;j < numSubCH;j++) {
			float sum = 0;
			for (int k = 0; k < 10; k++) {
				sum += sensingList[i + (100 * k)][j];
			}
			map.emplace(make_pair(sum, make_pair(i, j)));
		}
	}
	/**���20%�̈ʒu������*/
	auto border = distance(map.begin(), map.upper_bound(next(map.begin(),
		(int)ceil(((T2 - T1) * numSubCH) * 0.2))->first));
	uniform_int_distribution<>::param_type paramSB(0, border - 1);
	distSB.param(paramSB);
	auto nextResource = next(map.begin(), distSB(engine));
	/**���M���\�[�X�X�V*/
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
 * �`���l�����ƂɎ�M�d�͂��L���b�V��
 */
inline void Vehicle::calcRecvPower(const Vehicle* v, unordered_map<pair<string, string>, float, HashPair>& cache) {
	float pathLoss = 0;
	float fadingLoss = 0;
	float shadowingLoss = 0;
	float recvPower_mw = 0;

	/**�L���b�V�������邩�m�F*/
	if (cache.count(make_pair(min(id, v->id), max(id, v->id))) == 0) {
		/**�L���b�V�����Ȃ��ꍇ�͌v�Z*/
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
	/**sensingList�X�V*/
	sensingList[SENSING_WINDOW - 1][v->txResource.second] += recvPower_mw;
}

inline float Vehicle::calcWINNER(const Vehicle* v) {
	/**LOS��NLOS��*/
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
	/**2�ԗ��Ԃ̋������v�Z*/

	if (d < 10) {
		/**���R��ԓ`������*/
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
	/**2�ԗ����ʒu����p�^�[���Ő؂�ւ�*/
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