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
 * @breif SPSシミュレーション
 */
class Simulator {
private:
	/**パケットサイズのモード 0:300byte, 1:190byte */
	const int packet_size_mode;
	/**伝搬損失モデルのモード 0:WINNER, 1:自由空間 */
	const int prop_mode;
	/**リソース選択方式のモード 0:original 1:proposed */
	const int scheme_mode;
	/**リソース維持確率*/
	const float probKeep;
	/**SUMO内のシミュレーション時間*/
	int timestep;
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
	const string fname;
	/**PRR計測*/
	map<int, pair<ull, ull>> resultMap;

	/**
	 * @breif シミュレーション実行関数
	 */
	void run();

	/**
	 * @breif 結果の書き込み
	 * @param ファイル名
	 */
	void write_result(string fname);
public:
	/**
	 * コンストラクタ
	 * @brief SUMOへの接続と一定時間から回し
	 * @param port SUMOへの接続ポート
	 * @param fname 結果を記録するファイル名
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
 * @breif SPSシミュレーション
 */
inline void Simulator::run() {
	/**車両インスタンスの生成*/
	for (const string veID : sumo.vehicle.getIDList()) {
		vehicleList[veID] = new Vehicle(veID, float(sumo.vehicle.getPosition(veID).x), float(sumo.vehicle.getPosition(veID).y),
			sumo.vehicle.getLaneID(veID), probKeep, packet_size_mode, prop_mode, scheme_mode);
	}
	/**SIM_TIMEだけ時間を進める*/
	while (subframe < SIM_TIME) {

		/**100ms毎に車両情報を更新*/
		if (preSubframe != 0 && (preSubframe % 100) >= (subframe % 100)) {
			timestep++;
			recvPowerCache.clear();
			sumo.simulationStep();
			/**到着した車両を削除*/
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
					itr->second->resourceSelection(subframe);
					depVehicleList.erase(itr++);
				}
				else
					++itr;
			}
			/**生起した車両を格納*/
			for (auto&& depID : sumo.simulation.getDepartedIDList()) {
				auto tmp = new Vehicle(depID, sumo.vehicle.getPosition(depID).x,
					sumo.vehicle.getPosition(depID).y, sumo.vehicle.getLaneID(depID),
					probKeep, packet_size_mode, prop_mode, scheme_mode, 1);
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
			if (subframe >= SPS_WARM) {
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
			}
			/**リソース再選択判定*/
			txVe.second->decisionReselection(subframe);
		}

		/**次のイベント時間の検索,その時間に対して送信車両と受信車両の集合を計算*/
		txVeCollection.clear();
		nextEventSubframe = INT_MAX;
		for (auto&& veElem : vehicleList) {
			veElem.second->resetRecvPower();
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

inline void Simulator::write_result(string fname) {
	ofstream result(fname + ".csv");
	for (auto&& elem : resultMap) {
		result << elem.first << "," << elem.second.first << "," << elem.second.second << ","
			<< (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}
	result.close();
}

#endif