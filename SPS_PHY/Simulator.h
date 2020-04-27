#ifndef SIMULATOR
#define SIMULATOR

#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <utils/traci/TraCIAPI.h>
#include "Vehicle.h"

constexpr int SPS_WARM = 1500;		//(ms)
constexpr int SUMO_WARM = 100;		//(s)
constexpr int SIM_TIME = SPS_WARM + (100 * 1000);	//(ms)

using namespace std;

/**
 * @class Simulator
 * @breif SPS�V�~�����[�V����
 */
class Simulator {
private:
	/**���\�[�X�ێ��m��*/
	const float probKeep;
	/**�T�u�`���l����*/
	const int numSubCH;
	/**�V�~�����[�V��������*/
	int subframe = 0;
	/**���̃C�x���g����*/
	int nextEventSubframe = 0;
	/**���̃C�x���g���ԂƂ̍�*/
	int timeGap = 0;
	/**���O��subframe*/
	int preSubframe = 0;
	/**Vehicle�N���X�̃C���X�^���X���i�[����R���e�i*/
	map<string, Vehicle*, less<>> vehicleList;
	/**���N����Vehicle�C���X�^���X���ꎟ�i�[����R���e�i*/
	unordered_map<string, Vehicle*> depVehicleList;
	/**���̃C�x���g���Ԃɑ��M���s���ԗ��C���X�^���X���i�[����vector*/
	map<string, Vehicle*, less<>> txVeCollection;
	/**���̃C�x���g���Ԃɑ��M���s��Ȃ��ԗ��C���X�^���X���i�[����vector*/
	map<string, Vehicle*, less<>> rxVeCollection;
	/**100ms�P�ʂł̓`�������̃L���b�V��*/
	unordered_map<pair<string, string>, float, HashPair> recvPowerCache;
	/**SUMO��API*/
	TraCIAPI sumo;
	/**���ʂ��L�^����t�@�C����*/
	string fname;
	/**PRR�v��*/
	unordered_map<int, pair<int, int>> resultMap;

	void write_result(string fname);
public:
	/**
	 * �R���X�g���N�^
	 * @brief SUMO�ւ̐ڑ��ƈ�莞�Ԃ����
	 * @param port SUMO�ւ̐ڑ��|�[�g
	 * @param fname ���ʂ��L�^����t�@�C����
	 */
	Simulator(int port, int numSubCH, float prob) : numSubCH(numSubCH), probKeep(prob) {
		sumo.connect("localhost", port);
		sumo.simulationStep(SUMO_WARM);
		run();
	}
	Simulator(int port, string fname, int numSubCH, float prob) : numSubCH(numSubCH), probKeep(prob) {
		sumo.connect("localhost", port);
		sumo.simulationStep(SUMO_WARM);
		this->fname = fname;
		run();
		write_result(fname);
	}
	void run();
};

/**
 * @breif SPS�V�~�����[�V����
 */
inline void Simulator::run() {
	/**�ԗ��C���X�^���X�̐���*/
	for (const string veID : sumo.vehicle.getIDList()) {
		vehicleList[veID] = new Vehicle(veID, float(sumo.vehicle.getPosition(veID).x), float(sumo.vehicle.getPosition(veID).y), sumo.vehicle.getLaneID(veID), numSubCH, probKeep);
	}
	/**SIM_TIME�������Ԃ�i�߂�*/
	while (subframe < SIM_TIME) {


		/**100ms���Ɏԗ������X�V*/
		if (preSubframe != 0 && (preSubframe % 100) >= (subframe % 100)) {
			recvPowerCache.clear();
			sumo.simulationStep();
			/**���������ԗ����폜*/

			/**TODO*/
			/**rxVeCollection, txVeCollection������폜����K�v*/
			for (auto&& arrivedID : sumo.simulation.getArrivedIDList()) {
				for (auto&& resultElem : vehicleList[arrivedID]->getResult()) {
					resultMap[resultElem.first].first += resultElem.second.first;
					resultMap[resultElem.first].second += resultElem.second.second;
				}
				delete(vehicleList[arrivedID]);
				txVeCollection.erase(arrivedID);
				vehicleList.erase(arrivedID);
			}

			/**�ԗ��̈ʒu�����X�V*/
			for (auto&& veElem : vehicleList) {
				veElem.second->positionUpdate(sumo.vehicle.getPosition(veElem.first).x,
					sumo.vehicle.getPosition(veElem.first).y, sumo.vehicle.getLaneID(veElem.first));
			}
			/**���N���Ă���15���M������ɑ��M���\�[�X������*/
			auto&& itr = depVehicleList.begin();
			while (itr != depVehicleList.end()) {
				if (itr->second->getDecRC() == 0) {
					itr->second->resourceReselection(subframe);
					depVehicleList.erase(itr++);
				}
				else
					++itr;
			}
			/**���N�����ԗ����i�[*/
			for (auto&& depID : sumo.simulation.getDepartedIDList()) {
				auto tmp = new Vehicle(depID, sumo.vehicle.getPosition(depID).x,
					sumo.vehicle.getPosition(depID).y, sumo.vehicle.getLaneID(depID), numSubCH, probKeep, 1);
				vehicleList[depID] = tmp;
				depVehicleList[depID] = tmp;
			}
		}

		rxVeCollection.clear();
		/**��M�ԗ������߂�*/
		set_difference(vehicleList.begin(), vehicleList.end(),
			txVeCollection.begin(), txVeCollection.end(), inserter(rxVeCollection, rxVeCollection.end()));

		/**��M�d�͌v�Z*/
		for (auto&& txVe : txVeCollection) {
			for (auto&& rxVe : rxVeCollection) {
				rxVe.second->sensingListUpdate(timeGap);
				rxVe.second->calcRecvPower(txVe.second, recvPowerCache);
			}
		}

		/**�p�P�b�g��M����*/
		for (auto&& txVe : txVeCollection) {
			for (auto&& rxVe : vehicleList) {
				if (txVe != rxVe) {
					/**���ԗ��ɑ΂���p�P�b�g��M����*/
					if (txVeCollection.count(rxVe.first) == 0) {
						/**���鑗�M�ԗ��ɑ΂����M�ԗ��̃p�P�b�g��M����*/
						rxVe.second->decisionPacket(txVe.second, recvPowerCache);
					}
					else {
						/**���鑗�M�ԗ��ɑ΂��鑼�̑��M�ԗ��̃p�P�b�g��M����*/
						rxVe.second->calcHalfDup(txVe.second);
					}
				}
			}
			/**���\�[�X�đI�𔻒�*/
			if (txVe.second->getDecRC() == 0) {
				txVe.second->SPS(subframe);
			}
		}

		txVeCollection.clear();

		/**���̃C�x���g���Ԃ̌���,���̎��Ԃɑ΂��đ��M�ԗ��Ǝ�M�ԗ��̏W�����v�Z*/
		nextEventSubframe = INT_MAX;
		for (auto&& veElem : vehicleList) {
			if (nextEventSubframe > veElem.second->getResource().first) {
				/**�ŒZ�̃C�x���g���Ԃ��������ꍇ*/
				txVeCollection.clear();
				txVeCollection.emplace(veElem.first, veElem.second);
				nextEventSubframe = veElem.second->getResource().first;
			}
			else if (nextEventSubframe == veElem.second->getResource().first) {
				/**�ŒZ�̃C�x���g���ԂƓ������Ԃ̏ꍇ*/
				txVeCollection.emplace(veElem.first, veElem.second);
			}
		}

		timeGap = nextEventSubframe - subframe;
		preSubframe = subframe;
		subframe = nextEventSubframe;
	}

	/**�ԗ��C���X�^���X���f���[�g*/
	for (auto&& veElem : vehicleList) {
		/**PRR�v��*/
		for (auto&& resultElem : veElem.second->getResult()) {
			resultMap[resultElem.first].first += resultElem.second.first;
			resultMap[resultElem.first].second += resultElem.second.second;
		}
		delete(veElem.second);
	}
	/**SUMO�ؒf*/
	sumo.close();
}

/**
 * @breif ���ʂ̏�������
 */
inline void Simulator::write_result(string fname) {
	ofstream result("resutl.csv");
	for (auto&& elem : resultMap) {
		result << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second);
	}
	result.close();
}

#endif