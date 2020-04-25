#ifndef SIMULATOR
#define SIMULATOR

#include <utils/traci/TraCIAPI.h>
#include "Vehicle.h"

constexpr int SPS_WARM = 1500;		//(ms)
constexpr int SUMO_WARM = 2000;		//(s)
constexpr int SIM_TIME = SPS_WARM + (100 * 1000);	//(ms)


/**
 * @class Simulator
 * @breif SPS�V�~�����[�V����
 */
class Simulator {
private:
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
	vector<Vehicle*> vehicleList;
	/**���N����Vehicle�C���X�^���X���ꎟ�i�[����R���e�i*/
	vector<Vehicle*> depVehicleList;
	/**SUMO��API*/
	TraCIAPI sumo;
	/**���ʂ��L�^����t�@�C����*/
	string fname;

	void write_result();
public:
	/**
	 * �R���X�g���N�^
	 * @brief SUMO�ւ̐ڑ��ƈ�莞�Ԃ����
	 * @param port SUMO�ւ̐ڑ��|�[�g
	 * @param fname ���ʂ��L�^����t�@�C����
	 */
	Simulator(int port, int numSubCH) : numSubCH(numSubCH) {
		sumo.connect("localhost", port);
		sumo.simulationStep(SUMO_WARM);
		run();
	}
	Simulator(int port, string fname) : numSubCH(numSubCH) {
		sumo.connect("localhost", port);
		sumo.simulationStep(SUMO_WARM);
		this->fname = fname;
		run();
	}
	void run();
};

/**
 * @breif SPS�V�~�����[�V����
 */
inline void Simulator::run() {
	/**�ԗ��C���X�^���X�̐���*/
	for (const string veID : sumo.vehicle.getIDList()) {
		vehicleList.emplace_back(new Vehicle(veID, float(sumo.vehicle.getPosition(veID).x), float(sumo.vehicle.getPosition(veID).y), sumo.vehicle.getLaneID(veID), subframe));
	}

	/**SIM_TIME�������Ԃ�i�߂�*/
	while (subframe < SIM_TIME) {


	}


	/**SUMO�ؒf*/
	sumo.close();
}

/**
 * @breif ���ʂ̏�������
 */
inline void Simulator::write_result() {

}

#endif