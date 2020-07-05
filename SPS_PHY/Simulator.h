#ifndef SIMULATOR
#define SIMULATOR

#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <map>
#include <vector>
#include <utils/traci/TraCIAPI.h>
#include "Vehicle.h"

constexpr int SPS_WARM = 3000;		//(ms)
constexpr int SIM_TIME = SPS_WARM + (1000 * 1000);	//(ms)
constexpr float COL_DISTANCE = 268.664;
constexpr int SAVE_EACH_PACKET_PRR_NUM = 100000;

using namespace std;
typedef unsigned long long int ull;

//ofstream ////logger("log.xml");

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
	/**���\�[�X�I������̃��[�h 0:original 1:proposed 2:random */
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
	unordered_map<string, Vehicle*> depList;
	/**���M���s���ԗ��C���X�^���X���i�[����vector*/
	map<string, Vehicle*, less<>> txCollection;
	/**���M���s��Ȃ��ԗ��C���X�^���X���i�[����vector*/
	unordered_map<string, Vehicle*> rxCollection;
	/**txVeCollection�̂���C���X�^���X�ȊO���i�[����R���e�i*/
	unordered_map<string, Vehicle*> otherTxCollection;
	/**100ms�P�ʂł̓`�������̃L���b�V��*/
	unordered_map<pair<string, string>, float, HashPair> recvPowerCache;
	/**SUMO��API*/
	TraCIAPI sumo;
	/**PRR�v��*/
	map<int, pair<ull, ull>> resultMap;
	map<int, pair<ull, ull>> resultLOSMap;
	map<int, pair<ull, ull>> resultNLOSMap;
	map<int, pair<ull, ull>> resultNoInterMap;
	map<int, ull> resultColMap;
	map<int, double> resultEachPacketSumPRRMap;
	map<int, vector<float>> resultEachPacketPRRMap;

	ull numSendPacket = 0;

	ull totalNumSendPacket = 0;


	/**
	 * @breif �V�~�����[�V�������s�֐�
	 */
	void run();

	/**
	 * @breif PRR�̒l���L�^
	 * @param v �ԗ��C���X�^���X
	 */
	void saveResult(Vehicle* v);

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
	Simulator(string fname, int port, float prob, int sumo_warm, int packet_mode, int prop_mode, int scheme_mode)
		: probKeep(prob), packet_size_mode(packet_mode), prop_mode(prop_mode), scheme_mode(scheme_mode)
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
		vehicleList.emplace(make_pair(veID, new Vehicle(veID, float(sumo.vehicle.getPosition(veID).x), float(sumo.vehicle.getPosition(veID).y),
			sumo.vehicle.getLaneID(veID), probKeep, packet_size_mode, prop_mode, scheme_mode)));
	}
	/**SIM_TIME�������Ԃ�i�߂�*/
	while (subframe < SIM_TIME) {
		//logger << "<subframe=\"" << subframe << "\"/>" << endl;
		/**��M�ԗ���sensingList���X�V*/
		for (auto&& rxVe : rxCollection) {
			rxVe.second->sensingListUpdate(timeGap);
		}

		/**100ms���Ɏԗ������X�V*/
		if (preSubframe != 0 && (preSubframe % 100) >= (subframe % 100)) {
			timestep++;
			sumo.simulationStep();
			recvPowerCache.clear();
			/**���������ԗ����폜*/
			for (auto&& arrivedID : sumo.simulation.getArrivedIDList()) {
				saveResult(vehicleList[arrivedID]);
				delete(vehicleList[arrivedID]);
				txCollection.erase(arrivedID);
				rxCollection.erase(arrivedID);
				vehicleList.erase(arrivedID);
			}

			/**�ԗ��̈ʒu�����X�V*/
			for (auto&& veElem : vehicleList) {
				veElem.second->positionUpdate(sumo.vehicle.getPosition(veElem.first).x,
					sumo.vehicle.getPosition(veElem.first).y, sumo.vehicle.getLaneID(veElem.first));
			}
			/**���N���Ă���15���M������ɑ��M���\�[�X������*/
			auto&& itr = depList.begin();
			while (itr != depList.end()) {
				if (itr->second->getDecRC() == 0) {
					itr->second->resourceSelection(subframe);
					depList.erase(itr++);
				}
				else
					++itr;
			}
			/**���N�����ԗ����i�[*/
			for (auto&& depID : sumo.simulation.getDepartedIDList()) {
				//cout << depID << ":" << subframe << endl;
				auto tmp = new Vehicle(depID, sumo.vehicle.getPosition(depID).x,
					sumo.vehicle.getPosition(depID).y, sumo.vehicle.getLaneID(depID),
					probKeep, packet_size_mode, prop_mode, scheme_mode, 1);
				vehicleList.emplace(make_pair(depID, tmp));
				rxCollection.emplace(make_pair(depID, tmp));
				depList.emplace(make_pair(depID, tmp));
			}
		}

		/**��M�d�͌v�Z*/
		for (auto&& txVe : txCollection) {
			txVe.second->txSensingListUpdate(timeGap);
			for (auto&& rxVe : rxCollection) {
				rxVe.second->calcRecvPower(txVe.second, recvPowerCache);
			}
		}

		/**�p�P�b�g��M����*/
		for (auto txItr = txCollection.begin(); txItr != txCollection.end(); ++txItr) {
			if ((*txItr).second->isIn()) {
				if (subframe >= SPS_WARM) {
					(*txItr).second->countNumSendPacket();
					totalNumSendPacket++;
					/**txItr�ȊO�̑��M�ԗ��̏W�������߂�*/
					otherTxCollection.clear();
					set_difference(txCollection.begin(), txCollection.end(), txItr, next(txItr),
						inserter(otherTxCollection, otherTxCollection.end()));

					/**���鑗�M�ԗ��ɑ΂����M�ԗ��̃p�P�b�g��M����*/
					for (auto&& rxVe : rxCollection) {
						rxVe.second->decisionPacket((*txItr).second, recvPowerCache);
					}

					/**����d���M�̌v��*/
					for (auto&& otherTxVe : otherTxCollection) {
						otherTxVe.second->calcHalfDup((*txItr).second);
					}


					/**�p�P�b�g�Փ˃`�F�b�N*/
					if (otherTxCollection.size() == 0) {
						/**�����T�u�t���[���ő��M�ԗ������Ȃ��ꍇ*/
						//if ((*txItr).second->getNumCol() != 0)
							//logger << "    id=\"" << (*txItr).second->getID() << "\" account, counter=\"" << (*txItr).second->getNumCol() << "\"" << endl;
						(*txItr).second->accountCollision();
					}
					else {
						bool flag = false;
						for (auto&& otherTxVe : otherTxCollection) {
							if ((*txItr).second->getResource().second == otherTxVe.second->getResource().second && (*txItr).second->getDistance(otherTxVe.second) < COL_DISTANCE) {
								/**�����T�u�t���[���C�����T�u�`���l���ɑ��M�ԗ������݂���ꍇ*/
								//logger << "    id=\"" << (*txItr).second->getID() << "\" collision, coutner=\"" << (*txItr).second->getNumCol() + 1 << "\"" << endl;
								(*txItr).second->countCollision();
								flag = true;
								break;
							}
						}
						if (!flag) {
							/**�����T�u�t���[���C�����T�u�`���l���ɑ��M�ԗ������݂��Ȃ��ꍇ*/
							//if ((*txItr).second->getNumCol() != 0)
								//logger << "    id=\"" << (*txItr).second->getID() << "\" account, counter=\"" << (*txItr).second->getNumCol() << "\"" << endl;
							(*txItr).second->accountCollision();
						}
					}
				}
			}
		}

		for (auto&& ve : txCollection) {
			ve.second->decisionReselection(subframe);
			ve.second->accountEachPRR(totalNumSendPacket);
		}

		/**���̃C�x���g���Ԃ̌���,���̎��Ԃɑ΂��đ��M�ԗ��Ǝ�M�ԗ��̏W�����v�Z*/
		txCollection.clear();
		nextEventSubframe = INT_MAX;
		for (auto&& veElem : vehicleList) {
			veElem.second->clearRecvPower();
			if (nextEventSubframe > veElem.second->getResource().first) {
				/**�ŒZ�̃C�x���g���Ԃ��������ꍇ*/
				txCollection.clear();
				txCollection.emplace(veElem.first, veElem.second);
				nextEventSubframe = veElem.second->getResource().first;
			}
			else if (nextEventSubframe == veElem.second->getResource().first) {
				/**�ŒZ�̃C�x���g���ԂƓ������Ԃ̏ꍇ*/
				txCollection.emplace(veElem.first, veElem.second);
			}
		}

		/**��M�ԗ������߂�*/
		rxCollection.clear();
		set_difference(vehicleList.begin(), vehicleList.end(),
			txCollection.begin(), txCollection.end(), inserter(rxCollection, rxCollection.end()));

		timeGap = nextEventSubframe - subframe;
		preSubframe = subframe;
		subframe = nextEventSubframe;
	}

	/**�ԗ��C���X�^���X���f���[�g*/
	for (auto&& veElem : vehicleList) {
		/**PRR�v��*/
		saveResult(veElem.second);
		delete(veElem.second);
	}
	/**SUMO�ؒf*/
	sumo.close();
}

inline void Simulator::saveResult(Vehicle* v) {
	numSendPacket += v->getNumSendPacket();
	for (auto&& resultElem : v->getResult()) {
		resultMap[resultElem.first].first += resultElem.second.first;
		resultMap[resultElem.first].second += resultElem.second.second;
	}
	for (auto&& resultElem : v->getLOSResult()) {
		resultLOSMap[resultElem.first].first += resultElem.second.first;
		resultLOSMap[resultElem.first].second += resultElem.second.second;
	}
	for (auto&& resultElem : v->getNLOSResult()) {
		resultNLOSMap[resultElem.first].first += resultElem.second.first;
		resultNLOSMap[resultElem.first].second += resultElem.second.second;
	}
	for (auto&& resultElem : v->getNoInterResult()) {
		resultNoInterMap[resultElem.first].first += resultElem.second.first;
		resultNoInterMap[resultElem.first].second += resultElem.second.second;
	}
	for (auto&& resultElem : v->getColResult()) {
		resultColMap[resultElem.first] += resultElem.second;
	}
	for (auto&& resultElem : v->getEachPacketSumPRRMap()) {
		resultEachPacketSumPRRMap[resultElem.first] += resultElem.second;
	}
	for (auto&& resultElem : v->getEachPacketPRRMap()) {
		resultEachPacketPRRMap[resultElem.first].insert(resultEachPacketPRRMap[resultElem.first].end(), resultElem.second.begin(), resultElem.second.end());
	}
}

inline void Simulator::write_result(string fname) {
	ofstream result("result/" + fname + ".csv");
	ofstream resultLOS("result/" + fname + "_LOS.csv");
	ofstream resultNLOS("result/" + fname + "_NLOS.csv");
	ofstream resultNoInter("result/" + fname + "_noInter.csv");
	ofstream resultCol("result/" + fname + "_col.csv");
	ofstream resultEachPacketSum("result/" + fname + "_each_sum.csv");

	for (auto&& elem : resultMap) {
		result << elem.first << "," << elem.second.first << "," << elem.second.second << ","
			<< (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}
	for (auto&& elem : resultLOSMap) {
		resultLOS << elem.first << "," << elem.second.first << "," << elem.second.second << ","
			<< (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}
	for (auto&& elem : resultNLOSMap) {
		resultNLOS << elem.first << "," << elem.second.first << "," << elem.second.second << ","
			<< (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}
	for (auto&& elem : resultNoInterMap) {
		resultNoInter << elem.first << "," << elem.second.first << "," << elem.second.second << ","
			<< (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}

	int sum = 0;

	for (auto&& elem : resultColMap) {
		sum += elem.second;
	}

	for (auto&& elem : resultColMap) {
		resultCol << elem.first << "," << elem.second << "," << (double)elem.second / (double)sum << endl;
	}

	for (auto&& elem : resultEachPacketSumPRRMap) {
		resultEachPacketSum << elem.first << "," << elem.second << "," << numSendPacket << endl;
	}

	map<int, int> sumMap;
	map<int, map<float, int>> sumEachPRRMap;

	for (auto&& elem : resultEachPacketPRRMap) {
		for (auto num : elem.second) {
			sumEachPRRMap[elem.first][num]++;
			sumMap[elem.first]++;
		}
	}

	for (auto&& elem : sumEachPRRMap) {
		ofstream resultEachPacket("result/each/" + fname + "_each_" + to_string(elem.first) + ".csv");
		for (auto&& elem2 : elem.second) {
			resultEachPacket << elem2.first << "," << elem2.second << "," << (double)elem2.second / (double)sumMap[elem.first] << endl;
		}
		resultEachPacket.close();
	}

	result.close();
	resultLOS.close();
	resultNLOS.close();
	resultNoInter.close();
	resultCol.close();
	resultEachPacketSum.close();
}

#endif