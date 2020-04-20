#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
#include <random>
#include <iostream>
#include <map>
#include <string>
#include "Table.h"

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
/**���H��(m)*/
constexpr int STREET_WIDTH = 20;
/**LOS�`���̃u���C�N�|�C���g*/
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
 * @breif �ԗ��N���X
 */
class Vehicle {
private:
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
	pair<int, int> nextResource;
	/**2�ԗ��Ԃ̎�M�d�͂̃L���b�V��*/
	unordered_map<pair<string, string>, float, HashPair, less<>> recvPowerMap;
	/**���v��M�d��*/
	float sumRecvPower;
	/**�Z���V���O���X�g*/
	vector<vector<float>> sensingList;
	/**�����֘A*/
	random_device seed;
	mt19937 engine;
	/**RC��SB�p�̗���������*/
	uniform_int_distribution<> distRC, distSB;
	/**���\�[�X�đI�𔻒�p�̗���������*/
	uniform_real_distribution<> distResourceKeep;

	pair<int, int> preTxResourceLocation;
	bool isReselection = false;
	int preSetRC = -1;
	int setRC = 0;
	int num_subCH = 0;
	float prob_resource_keep = 0;

	/**
	 * 2�_�Ԃ̋��������߂�
	 * @param x1, x2, y1, y2 �e���W
	 * @retval ����
	 */
	float getDistance(float x1, float x2, float y1, float y2);

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
	pair<int, int> getMinJunction(const Vehicle* v);

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
	 * @param subframe ���N����subframe
	 */
	Vehicle(string id, float x, float y, string lane_id, int subframe);

	/**
	 * �ԗ��̍��W���X�V
	 * @param x x���W
	 * @param y y���W
	 * @param lane_id lane��id
	 */
	void positionUpdate(float x, float y, string lane_id);

	/**
	 * 2�ԗ��Ԃ̎�M�d�͌v�Z�C�v�Z���ʂ��L���b�V���Ƃ��ĕۑ�
	 * @param v ����ԗ��̃C���X�^���X
	 */
	void calcRecvPower(const Vehicle* v);

	/**
	* �p�P�b�g�̎�M�����𔻒f
	* @param v ����ԗ��̃C���X�^���X
	*/
	void decisionPacket(const Vehicle* v);
};

/***************************************�֐��̒�`***************************************/
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




/**************************************�`�������֌W**************************************/

inline float Vehicle::getDistance(float x1, float x2, float y1, float y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 * y2) * (y1 * y2));
}

inline void Vehicle::calcRecvPower(const Vehicle* v) {
	sumRecvPower = 0;
	float pathLoss = 0;
	float fadingLoss = 0;
	float shadowingLoss = 0;
	/**�L���b�V�������邩�m�F*/

	/**�L���b�V�����Ȃ��ꍇ�͌v�Z*/
	/**LOS��NLOS��*/
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
	/**2�ԗ��Ԃ̋������v�Z*/
	
	if (10 < d) {
		/**���R��ԓ`������*/
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
	/**2�ԗ����ʒu����p�^�[���Ő؂�ւ�*/
	switch (RELATION_TABLE[make_pair(laneID / 10, v->laneID / 10)]) {

	case PositionRelation::NORMAL:
		/**2�ԗ�������Ɉʒu���Ă��Ȃ��ꍇ*/
		cout << "(" << id << "," << v->id << "): NLOS, NORMAL" << " min:(" << minElem.first << ", " << minElem.second << ")";
		return NLOS(abs(x - v->x), abs(y - v->y));

	case PositionRelation::HOL_PAR:
		/**2�ԗ���������Ɉʒu���Ă�ꍇ*/
		cout << "(" << id << "," << v->id << "): NLOS, HOL_PAR" << " min:(" << minElem.first << ", " << minElem.second << ")";
		return NLOSHolPar(v);

	case PositionRelation::VER_PAR:
		/**2�ԗ����c����Ɉʒu���Ă���ꍇ*/
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