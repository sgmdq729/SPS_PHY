#ifndef SIMULATOR
#define SIMULATOR

#include <utils/traci/TraCIAPI.h>
#include "Vehicle.h"

constexpr int SPS_WARM = 1500;		//(ms)
constexpr int SUMO_WARM = 2000;		//(s)
constexpr int SIM_TIME = SPS_WARM + (100 * 1000);	//(ms)


/**
 * @class Simulator
 * @breif SPSシミュレーション
 */
class Simulator {
private:
	/**サブチャネル数*/
	const int numSubCH;
	/**シミュレーション時間*/
	int subframe = 0;
	/**次のイベント時間*/
	int nextEventSubframe = 0;
	/**次のイベント時間との差*/
	int timeGap = 0;
	/**直前のsubframe*/
	int preSubframe = 0;
	/**Vehicleクラスのインスタンスを格納するコンテナ*/
	vector<Vehicle*> vehicleList;
	/**生起したVehicleインスタンスを一次格納するコンテナ*/
	vector<Vehicle*> depVehicleList;
	/**SUMOのAPI*/
	TraCIAPI sumo;
	/**結果を記録するファイル名*/
	string fname;

	void write_result();
public:
	/**
	 * コンストラクタ
	 * @brief SUMOへの接続と一定時間から回し
	 * @param port SUMOへの接続ポート
	 * @param fname 結果を記録するファイル名
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
 * @breif SPSシミュレーション
 */
inline void Simulator::run() {
	/**車両インスタンスの生成*/
	for (const string veID : sumo.vehicle.getIDList()) {
		vehicleList.emplace_back(new Vehicle(veID, float(sumo.vehicle.getPosition(veID).x), float(sumo.vehicle.getPosition(veID).y), sumo.vehicle.getLaneID(veID), subframe));
	}

	/**SIM_TIMEだけ時間を進める*/
	while (subframe < SIM_TIME) {


	}


	/**SUMO切断*/
	sumo.close();
}

/**
 * @breif 結果の書き込み
 */
inline void Simulator::write_result() {

}

#endif