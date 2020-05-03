#ifndef SIMULATOR
#define SIMULATOR

#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <utils/traci/TraCIAPI.h>
#include "Vehicle.h"

constexpr int SPS_WARM = 1500;		//(ms)
constexpr int SIM_TIME = SPS_WARM + (1000 * 1000);	//(ms)

using namespace std;
typedef unsigned long long int ull;

/**
 * @class Simulator
 * @breif SPS�V�~�����[�V����
 */
class Simulator {
private:
	/**�p�P�b�g�T�C�Y�̃��[�h 0:300byte, 1:190byte */
	const int packet_size_mode;
	/**�`���������f���̃��[�h 0:WINNER, 1:���R��� */
	const int prop_mode;
	/**���\�[�X�I������̃��[�h 0:original 1:proposed */
	const int scheme_mode;
	/**���\�[�X�ێ��m��*/
	const float probKeep;
	/**SUMO���̃V�~�����[�V��������*/
	int timestep;
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
	const string fname;
	/**PRR�v��*/
	map<int, pair<ull, ull>> resultMap;

	/**
	 * @breif �V�~�����[�V�������s�֐�
	 */
	void run();

	/**
	 * @breif ���ʂ̏�������
	 * @param �t�@�C����
	 */
	void write_result(string fname);
public:
	/**
	 * �R���X�g���N�^
	 * @brief SUMO�ւ̐ڑ��ƈ�莞�Ԃ����
	 * @param port SUMO�ւ̐ڑ��|�[�g
	 * @param fname ���ʂ��L�^����t�@�C����
	 */
	//Simulator(int port, int numSubCH, float prob, int sumo_warm) : numSubCH(numSubCH), probKeep(prob){
	//	sumo.connect("localhost", port);
	//	sumo.simulationStep(sumo_warm);
	//	run();
	//}
	Simulator(string fname, int port, float prob, int sumo_warm, int packet_mode, int prop_mode, int scheme_mode)
		: fname(fname), probKeep(prob), packet_size_mode(packet_mode), prop_mode(prop_mode), scheme_mode(scheme_mode)
	{
		timestep = sumo_warm * 10;
		sumo.connect("localhost", port);
		sumo.simulationStep(sumo_warm);
		run();
		write_result(fname);
	}
};

/**
 * @breif SPS�V�~�����[�V����
 */
inline void Simulator::run() {
	/**�ԗ��C���X�^���X�̐���*/
	for (const string veID : sumo.vehicle.getIDList()) {
		vehicleList[veID] = new Vehicle(veID, float(sumo.vehicle.getPosition(veID).x), float(sumo.vehicle.getPosition(veID).y),
			sumo.vehicle.getLaneID(veID), probKeep, packet_size_mode, prop_mode, scheme_mode);
	}
	/**SIM_TIME�������Ԃ�i�߂�*/
	while (subframe < SIM_TIME) {

		/**100ms���Ɏԗ������X�V*/
		if (preSubframe != 0 && (preSubframe % 100) >= (subframe % 100)) {
			timestep++;
			recvPowerCache.clear();
			sumo.simulationStep();
			/**���������ԗ����폜*/
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
					itr->second->resourceSelection(subframe);
					depVehicleList.erase(itr++);
				}
				else
					++itr;
			}
			/**���N�����ԗ����i�[*/
			for (auto&& depID : sumo.simulation.getDepartedIDList()) {
				auto tmp = new Vehicle(depID, sumo.vehicle.getPosition(depID).x,
					sumo.vehicle.getPosition(depID).y, sumo.vehicle.getLaneID(depID),
					probKeep, packet_size_mode, prop_mode, scheme_mode, 1);
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
			if (subframe >= SPS_WARM) {
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
			}
			/**���\�[�X�đI�𔻒�*/
			txVe.second->decisionReselection(subframe);
		}

		/**���̃C�x���g���Ԃ̌���,���̎��Ԃɑ΂��đ��M�ԗ��Ǝ�M�ԗ��̏W�����v�Z*/
		txVeCollection.clear();
		nextEventSubframe = INT_MAX;
		for (auto&& veElem : vehicleList) {
			veElem.second->resetRecvPower();
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

inline void Simulator::write_result(string fname) {
	ofstream result(fname + ".csv");
	for (auto&& elem : resultMap) {
		result << elem.first << "," << elem.second.first << "," << elem.second.second << ","
			<< (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}
	result.close();
}

#endif