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
	map<string, Vehicle*, less<>> vehicleList;
	/**生起したVehicleインスタンスを一次格納するコンテナ*/
	unordered_map<string, Vehicle*> depVehicleList;
	/**次のイベント時間に送信を行う車両インスタンスを格納するvector*/
	map<string, Vehicle*, less<>> txVeCollection;
	/**次のイベント時間に送信を行わない車両インスタンスを格納するvector*/
	map<string, Vehicle*, less<>> rxVeCollection;
	/**100ms単位での伝搬損失のキャッシュ*/
	unordered_map<pair<string, string>, float, HashPair> recvPowerCache;
	/**SUMOのAPI*/
	TraCIAPI sumo;
	/**結果を記録するファイル名*/
	string fname;
	/**PRR計測*/
	unordered_map<int, pair<int, int>> resultMap;

	void write_result(string fname);
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
		write_result(fname);
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


		/**100ms毎に車両情報を更新*/
		if (preSubframe != 0 && (preSubframe % 100) >= (subframe % 100)) {
			recvPowerCache.clear();
			sumo.simulationStep();
			/**到着した車両を削除*/

			/**TODO*/
			/**rxVeCollection, txVeCollectionからも削除する必要*/
			for (auto&& arrivedID : sumo.simulation.getArrivedIDList()) {
				for (auto&& resultElem : vehicleList[arrivedID]->getResult()) {
					resultMap[resultElem.first].first += resultElem.second.first;
					resultMap[resultElem.first].second += resultElem.second.second;
				}
				delete(vehicleList[arrivedID]);
				txVeCollection.erase(arrivedID);
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
					sumo.vehicle.getPosition(depID).y, sumo.vehicle.getLaneID(depID), numSubCH, probKeep, 1);
				vehicleList[depID] = tmp;
				depVehicleList[depID] = tmp;
			}
		}

		rxVeCollection.clear();
		/**受信車両を求める*/
		set_difference(vehicleList.begin(), vehicleList.end(),
			txVeCollection.begin(), txVeCollection.end(), inserter(rxVeCollection, rxVeCollection.end()));

		/**受信電力計算*/
		for (auto&& txVe : txVeCollection) {
			for (auto&& rxVe : rxVeCollection) {
				rxVe.second->sensingListUpdate(timeGap);
				rxVe.second->calcRecvPower(txVe.second, recvPowerCache);
			}
		}

		/**パケット受信判定*/
		for (auto&& txVe : txVeCollection) {
			for (auto&& rxVe : vehicleList) {
				if (txVe != rxVe) {
					/**他車両に対するパケット受信判定*/
					if (txVeCollection.count(rxVe.first) == 0) {
						/**ある送信車両に対する受信車両のパケット受信判定*/
						rxVe.second->decisionPacket(txVe.second, recvPowerCache);
					}
					else {
						/**ある送信車両に対する他の送信車両のパケット受信判定*/
						rxVe.second->calcHalfDup(txVe.second);
					}
				}
			}
			/**リソース再選択判定*/
			if (txVe.second->getDecRC() == 0) {
				txVe.second->SPS(subframe);
			}
		}

		txVeCollection.clear();

		/**次のイベント時間の検索,その時間に対して送信車両と受信車両の集合を計算*/
		nextEventSubframe = INT_MAX;
		for (auto&& veElem : vehicleList) {
			if (nextEventSubframe > veElem.second->getResource().first) {
				/**最短のイベント時間を見つけた場合*/
				txVeCollection.clear();
				txVeCollection.emplace(veElem.first, veElem.second);
				nextEventSubframe = veElem.second->getResource().first;
			}
			else if (nextEventSubframe == veElem.second->getResource().first) {
				/**最短のイベント時間と同じ時間の場合*/
				txVeCollection.emplace(veElem.first, veElem.second);
			}
		}

		timeGap = nextEventSubframe - subframe;
		preSubframe = subframe;
		subframe = nextEventSubframe;
	}

	/**車両インスタンスをデリート*/
	for (auto&& veElem : vehicleList) {
		/**PRR計上*/
		for (auto&& resultElem : veElem.second->getResult()) {
			resultMap[resultElem.first].first += resultElem.second.first;
			resultMap[resultElem.first].second += resultElem.second.second;
		}
		delete(veElem.second);
	}
	/**SUMO切断*/
	sumo.close();
}

/**
 * @breif 結果の書き込み
 */
inline void Simulator::write_result(string fname) {
	ofstream result("resutl.csv");
	for (auto&& elem : resultMap) {
		result << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second);
	}
	result.close();
}

#endif