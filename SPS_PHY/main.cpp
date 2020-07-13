#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define NOMINMAX
#include <stdio.h>
#include <direct.h>
#include <iostream>
#include <Windows.h>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>
#include <unordered_map>

#include "Simulator.h"
#include "Table.h"
#include <utils/traci/TraCIAPI.h>

using namespace std;
typedef unsigned long long int ull;

void runSUMO(string port, int test_num, string filePath) {
	STARTUPINFO si = { 0 };
	PROCESS_INFORMATION pi = { 0 };
	char* cstr = new char[filePath.size() + 1];
	strcpy_s(cstr, filePath.size() + 1, filePath.c_str());
	BOOL bResult = CreateProcess(
		NULL,
		cstr,
		NULL,
		NULL,
		FALSE,
		CREATE_NEW_CONSOLE,
		NULL,
		NULL,
		&si,
		&pi
	);
	delete[] cstr;
	CloseHandle(pi.hThread);
	CloseHandle(pi.hProcess);
}

void process(int basePort, int start, int end, float prob, int T1, int T2, int threadNum, int packet_mode, int prop_mode, int scheme_mode, int myid) {
	for (int i = start + myid; i <= end; i += threadNum) {
		printf("test%d\n", i);
		string port(to_string(basePort + myid));
		string exePath("sumo -c test" + to_string(i) + ".sumocfg --remote-port " + port);
		string resultFname("test" + to_string(i));
		Sleep(5000);
		runSUMO(port, i, exePath);
		Simulator simulator(resultFname, stoi(port), prob, T1, T2, packet_mode, prop_mode, scheme_mode);
	}
}

vector<string> split(string& input, char delimiter)
{
	istringstream stream(input);
	string field;
	vector<string> result;
	while (getline(stream, field, delimiter)) {
		result.push_back(field);
	}
	return result;
}

int main() {
	
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	int port, start, end, threadNum, T1, T2;
	int packet_size_mode, propagation_mode, scheme_mode;
	float prob = 0.;
	vector<thread> threads;

	cout << "##### input simulation parameter ####" << endl;
	cout << "port << ";	cin >> port;
	cout << "start << "; cin >> start;
	cout << "end << "; cin >> end;
	cout << "T1(1) << "; cin >> T1;
	cout << "T2(20,50,100) << "; cin >> T2;
	cout << "thread num << "; cin >> threadNum;

	cout << endl << "#### input mode parameter ####" << endl;
	cout << "packet size: 300byte(0), 190byte(1) << "; cin >> packet_size_mode;
	cout << "propagation mode: WINNER+B1(0), freespace(1), LOS only(2) << "; cin >> propagation_mode;
	cout << "scheme mode: original(0), proposed(1), random(2) << "; cin >> scheme_mode;

	if (scheme_mode == 0 || scheme_mode == 1) {
		cout << "resource keep probability << "; cin >> prob;
	}
	//else if (scheme_mode == 1) {
	//	cerr << "not implemention" << endl;
	//	exit(-1);
	//}
	_mkdir("result");
	_mkdir("result/each");

	auto start_time = chrono::system_clock::now();
	for (int i = 0; i < threadNum; i++) {
		threads.emplace_back(thread(process, port, start, end, prob, T1, T2, threadNum, packet_size_mode, propagation_mode, scheme_mode, i));
	}
	for (auto& thread : threads) {
		thread.join();
	}

	map<int, pair<ull, ull>> resultPairMap;
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + ".csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultPairMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultPairMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream output("result/sum_result.csv");
	for (auto&& elem : resultPairMap) {
		output << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}

	resultPairMap.clear();
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_LOS.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultPairMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultPairMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream outputLOS("result/sum_LOS_result.csv");
	for (auto&& elem : resultPairMap) {
		outputLOS << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}

	resultPairMap.clear();
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_NLOS.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultPairMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultPairMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream outputNLOS("result/sum_NLOS_result.csv");
	for (auto&& elem : resultPairMap) {
		outputNLOS << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}

	resultPairMap.clear();
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_noInter.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultPairMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultPairMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream outputNoInter("result/sum_noInter_result.csv");
	for (auto&& elem : resultPairMap) {
		outputNoInter << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}

	map<int, ull> resultColMap;
	ull sum = 0;
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_col.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultColMap[stoi(strvec[0])] += stoi(strvec[1]);
			sum += stoi(strvec[1]);
		}
		ifs.close();
	}
	ofstream outputCol("result/sum_col_result.csv");
	for (auto&& elem : resultColMap) {
		outputCol << elem.first << "," << elem.second << "," << (double)elem.second/(double)sum << endl;
	}

	map<int, float> resultEachSumMap;
	sum = 0;
	ull allSendPackets = 0;
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_each_sum.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultEachSumMap[stoi(strvec[0])] += stof(strvec[1]);
			sum = stoi(strvec[2]);
		}
		allSendPackets += sum;
		ifs.close();
	}
	ofstream outputEachSum("result/sum_each_sum_result.csv");
	for (auto&& elem : resultEachSumMap) {
		outputEachSum << elem.first << "," << (double)elem.second / (double)allSendPackets << endl;
	}

	output.close();
	outputLOS.close();
	outputNLOS.close();
	outputNoInter.close();
	outputCol.close();
	outputEachSum.close();
	
	auto end_time = chrono::system_clock::now();
	double elapsed_time = chrono::duration_cast<chrono::minutes>(end_time - start_time).count();
	cout << "elapsed time: " << elapsed_time << "(m)" << endl;
	system("pause");
}
