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
 * @breif SPSシミュレーション
 */
class Simulator {
private:
	/**パケットサイズのモード 0:300byte, 1:190byte */
	const int packet_size_mode;
	/**伝搬損失モデルのモード 0:WINNER, 1:自由空間 */
	const int prop_mode;
	/**リソース選択方式のモード 0:original 1:proposed 2:random */
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
	unordered_map<string, Vehicle*> depList;
	/**送信を行う車両インスタンスを格納するvector*/
	map<string, Vehicle*, less<>> txCollection;
	/**送信を行わない車両インスタンスを格納するvector*/
	unordered_map<string, Vehicle*> rxCollection;
	/**txVeCollectionのあるインスタンス以外を格納するコンテナ*/
	unordered_map<string, Vehicle*> otherTxCollection;
	/**100ms単位での伝搬損失のキャッシュ*/
	unordered_map<pair<string, string>, float, HashPair> recvPowerCache;
	/**SUMOのAPI*/
	TraCIAPI sumo;
	/**PRR計測*/
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
	 * @breif シミュレーション実行関数
	 */
	void run();

	/**
	 * @breif PRRの値を記録
	 * @param v 車両インスタンス
	 */
	void saveResult(Vehicle* v);

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
 * @breif SPSシミュレーション
 */
inline void Simulator::run() {
	/**車両インスタンスの生成*/
	for (const string veID : sumo.vehicle.getIDList()) {
		vehicleList.emplace(make_pair(veID, new Vehicle(veID, float(sumo.vehicle.getPosition(veID).x), float(sumo.vehicle.getPosition(veID).y),
			sumo.vehicle.getLaneID(veID), probKeep, packet_size_mode, prop_mode, scheme_mode)));
	}
	/**SIM_TIMEだけ時間を進める*/
	while (subframe < SIM_TIME) {
		//logger << "<subframe=\"" << subframe << "\"/>" << endl;
		/**受信車両のsensingListを更新*/
		for (auto&& rxVe : rxCollection) {
			rxVe.second->sensingListUpdate(timeGap);
		}

		/**100ms毎に車両情報を更新*/
		if (preSubframe != 0 && (preSubframe % 100) >= (subframe % 100)) {
			timestep++;
			sumo.simulationStep();
			recvPowerCache.clear();
			/**到着した車両を削除*/
			for (auto&& arrivedID : sumo.simulation.getArrivedIDList()) {
				saveResult(vehicleList[arrivedID]);
				delete(vehicleList[arrivedID]);
				txCollection.erase(arrivedID);
				rxCollection.erase(arrivedID);
				vehicleList.erase(arrivedID);
			}

			/**車両の位置情報を更新*/
			for (auto&& veElem : vehicleList) {
				veElem.second->positionUpdate(sumo.vehicle.getPosition(veElem.first).x,
					sumo.vehicle.getPosition(veElem.first).y, sumo.vehicle.getLaneID(veElem.first));
			}
			/**生起してから15送信周期後に送信リソースを決定*/
			auto&& itr = depList.begin();
			while (itr != depList.end()) {
				if (itr->second->getDecRC() == 0) {
					itr->second->resourceSelection(subframe);
					depList.erase(itr++);
				}
				else
					++itr;
			}
			/**生起した車両を格納*/
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

		/**受信電力計算*/
		for (auto&& txVe : txCollection) {
			txVe.second->txSensingListUpdate(timeGap);
			for (auto&& rxVe : rxCollection) {
				rxVe.second->calcRecvPower(txVe.second, recvPowerCache);
			}
		}

		/**パケット受信判定*/
		for (auto txItr = txCollection.begin(); txItr != txCollection.end(); ++txItr) {
			if ((*txItr).second->isIn()) {
				if (subframe >= SPS_WARM) {
					(*txItr).second->countNumSendPacket();
					totalNumSendPacket++;
					/**txItr以外の送信車両の集合を求める*/
					otherTxCollection.clear();
					set_difference(txCollection.begin(), txCollection.end(), txItr, next(txItr),
						inserter(otherTxCollection, otherTxCollection.end()));

					/**ある送信車両に対する受信車両のパケット受信判定*/
					for (auto&& rxVe : rxCollection) {
						rxVe.second->decisionPacket((*txItr).second, recvPowerCache);
					}

					/**半二重送信の計上*/
					for (auto&& otherTxVe : otherTxCollection) {
						otherTxVe.second->calcHalfDup((*txItr).second);
					}


					/**パケット衝突チェック*/
					if (otherTxCollection.size() == 0) {
						/**同じサブフレームで送信車両がいない場合*/
						//if ((*txItr).second->getNumCol() != 0)
							//logger << "    id=\"" << (*txItr).second->getID() << "\" account, counter=\"" << (*txItr).second->getNumCol() << "\"" << endl;
						(*txItr).second->accountCollision();
					}
					else {
						bool flag = false;
						for (auto&& otherTxVe : otherTxCollection) {
							if ((*txItr).second->getResource().second == otherTxVe.second->getResource().second && (*txItr).second->getDistance(otherTxVe.second) < COL_DISTANCE) {
								/**同じサブフレーム，同じサブチャネルに送信車両が存在する場合*/
								//logger << "    id=\"" << (*txItr).second->getID() << "\" collision, coutner=\"" << (*txItr).second->getNumCol() + 1 << "\"" << endl;
								(*txItr).second->countCollision();
								flag = true;
								break;
							}
						}
						if (!flag) {
							/**同じサブフレーム，同じサブチャネルに送信車両が存在しない場合*/
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

		/**次のイベント時間の検索,その時間に対して送信車両と受信車両の集合を計算*/
		txCollection.clear();
		nextEventSubframe = INT_MAX;
		for (auto&& veElem : vehicleList) {
			veElem.second->clearRecvPower();
			if (nextEventSubframe > veElem.second->getResource().first) {
				/**最短のイベント時間を見つけた場合*/
				txCollection.clear();
				txCollection.emplace(veElem.first, veElem.second);
				nextEventSubframe = veElem.second->getResource().first;
			}
			else if (nextEventSubframe == veElem.second->getResource().first) {
				/**最短のイベント時間と同じ時間の場合*/
				txCollection.emplace(veElem.first, veElem.second);
			}
		}

		/**受信車両を求める*/
		rxCollection.clear();
		set_difference(vehicleList.begin(), vehicleList.end(),
			txCollection.begin(), txCollection.end(), inserter(rxCollection, rxCollection.end()));

		timeGap = nextEventSubframe - subframe;
		preSubframe = subframe;
		subframe = nextEventSubframe;
	}

	/**車両インスタンスをデリート*/
	for (auto&& veElem : vehicleList) {
		/**PRR計上*/
		saveResult(veElem.second);
		delete(veElem.second);
	}
	/**SUMO切断*/
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