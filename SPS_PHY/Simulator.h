#ifndef SIMULATOR
#define SIMULATOR

#include <unordered_map>
#include <algorithm>
#include <utils/traci/TraCIAPI.h>
#include "Vehicle.h"

constexpr int SPS_WARM = 1500;		//(ms)
constexpr int SUMO_WARM = 100;		//(s)
constexpr int SIM_TIME = SPS_WARM + (100 * 1000);	//(ms)

using namespace std;

/**
 * @class Simulator
 * @breif SPSシミュレーション
 */
class Simulator {
private:
	/**リソース維持確率*/
	const float probKeep;
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
	unordered_map<string, Vehicle*> vehicleList;
	/**生起したVehicleインスタンスを一次格納するコンテナ*/
	unordered_map<string, Vehicle*> depVehicleList;
	/**SUMOのAPI*/
	TraCIAPI sumo;
	/**結果を記録するファイル名*/
	string fname;
	/**PRR計測*/
	unordered_map<int, pair<int, int>> resultMap;

	void write_result();
public:
	/**
	 * コンストラクタ
	 * @brief SUMOへの接続と一定時間から回し
	 * @param port SUMOへの接続ポート
	 * @param fname 結果を記録するファイル名
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
	}
	void run();
};

/**
 * @breif SPSシミュレーション
 */
inline void Simulator::run() {
	/**車両インスタンスの生成*/
	for (const string veID : sumo.vehicle.getIDList()) {
		vehicleList[veID] = new Vehicle(veID, float(sumo.vehicle.getPosition(veID).x), float(sumo.vehicle.getPosition(veID).y), sumo.vehicle.getLaneID(veID), numSubCH, probKeep);
	}
	/**SIM_TIMEだけ時間を進める*/
	while (subframe < SIM_TIME) {
		vector<Vehicle*> allVeCollection;
		vector<Vehicle*> txVeCollection;
		vector<Vehicle*> rxVeCollection;

		/**100ms毎に車両情報を更新*/
		if (preSubframe != 0 && (preSubframe % 100) >= (subframe % 100)) {
			sumo.simulationStep();
			/**到着した車両を削除*/
			for (auto&& arrivedID : sumo.simulation.getArrivedIDList()) {
				for (auto&& resultElem : vehicleList[arrivedID]->getResult()) {
					resultMap[resultElem.first].first += resultElem.second.first;
					resultMap[resultElem.first].second += resultElem.second.second;
				}
				delete(vehicleList[arrivedID]);
				vehicleList.erase(arrivedID);
			}
			/**車両の位置情報を更新*/
			for (auto&& veElem : vehicleList) {
				veElem.second->positionUpdate(sumo.vehicle.getPosition(veElem.first).x, 
					sumo.vehicle.getPosition(veElem.first).y, sumo.vehicle.getLaneID(veElem.first));
			}
			/**生起してから15送信周期後に送信リソースを決定*/
			auto&& itr = depVehicleList.begin();
			while (itr != depVehicleList.end()) {
				if (itr->second->getDecRC() == 0) {
					itr->second->resourceReselection(subframe);
					depVehicleList.erase(itr++);
				}
				else
					++itr;
			}
			/**生起した車両を格納*/
			for (auto&& depID : sumo.simulation.getDepartedIDList()) {
				auto tmp = new Vehicle(depID, sumo.vehicle.getPosition(depID).x,
					sumo.vehicle.getPosition(depID).y, sumo.vehicle.getLaneID(depID), numSubCH, probKeep);
			}
		}

		/**次のイベント時間の検索,その時間に対して送信車両と受信車両の集合を計算*/
		nextEventSubframe = INT_MAX;
		for (auto&& veElem : vehicleList) {
			allVeCollection.emplace_back(veElem.second);
			if (nextEventSubframe < veElem.second->getResource().first) {
				txVeCollection.clear();
				txVeCollection.emplace_back(veElem.second);
				nextEventSubframe = veElem.second->getResource().first;
			}
			else if (nextEventSubframe == veElem.second->getResource().first) {
				txVeCollection.emplace_back(veElem.second);
			}
		}
		/**全車両インスタンスと送信車両インスタンスの差集合を求める*/
		sort(allVeCollection.begin(), allVeCollection.end());
		sort(txVeCollection.begin(), txVeCollection.end());
		set_difference(allVeCollection.begin(), allVeCollection.end(),
			txVeCollection.begin(), txVeCollection.end(), back_inserter(rxVeCollection));



	}

	/**車両インスタンスをデリート*/
	for (auto&& veElem : vehicleList) {
		delete(veElem.second);
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