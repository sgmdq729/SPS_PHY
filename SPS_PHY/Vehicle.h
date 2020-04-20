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
	 * WINNER+B1 LOS���f��
	 * @param v ����ԗ��̃C���X�^���X
	 */
	float calcLOS(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS���f��
	 * @param v ����ԗ��̃C���X�^���X
	 */
	float calcNLOS(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS ������
	 * @param v ����ԗ��̃C���X�^���X
	 */
	float calcNLOSHolPar(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS �c����
	 * @param v ����ԗ��̃C���X�^���X
	 */
	float calcNLOSVerPar(const Vehicle* v);

	/**
	 * WINNER+B1 NLOS
	 * @param v ����ԗ��̃C���X�^���X
	 */
	float calcNLOSNormal(const Vehicle* v);

public:
	/**
	 * �R���X�g���N�^
	 * @param id �ԗ�ID
	 * @param x,y ���W
	 * @param subframe ���N����subframe
	 */
	Vehicle(string id, float x, float y, string lane_id, int subframe);

	/**
	 * 2�ԗ��Ԃ̎�M�d�͌v�Z
	 * @param v ����ԗ��̃C���X�^���X
	 */
	void calcRecvPower(const Vehicle* v);


	/**
	 * �ԗ��̍��W���X�V
	 * @param x x���W
	 * @param y y���W
	 * @param lane_id lane��id
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
	/**�L���b�V�������邩�m�F*/

	/**�L���b�V�����Ȃ��ꍇ�͌v�Z*/
	/**LOS��NLOS��*/
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
	/**2�ԗ��Ԃ̋������v�Z*/
	float dis = sqrt((x - v->x) * (x - v->x) + (y - v->y) * (y - v->y));
	if (10 < dis) {
		/**���R��ԓ`������*/
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
	/**<����, pair<junction_id, junction_id>*/
	map<float, pair<int, int>> pathMap;

	/**���ԗ��Ƒ���ԗ��Ƃ̍ŒZ�o�H�����߂�*/
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

	/**2�ԗ����ʒu����p�^�[���Ő؂�ւ�*/
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