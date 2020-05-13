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

void process(int basePort, int start, int end, float prob, int sumo_warm, int threadNum, int packet_mode, int prop_mode, int scheme_mode, int myid, float tileSize, int divNum) {
	for (int i = start + myid; i <= end; i += threadNum) {
		printf("test%d\n", i);
		string port(to_string(basePort + myid));
		string exePath("sumo -c test" + to_string(i) + ".sumocfg --remote-port " + port);
		string resultFname("result/test" + to_string(i));
		runSUMO(port, i, exePath);
		Sleep(100);
		Simulator simulator(resultFname, stoi(port), prob, sumo_warm, packet_mode, prop_mode, scheme_mode, tileSize, divNum);
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
	constexpr double PI = 3.14159265358979323846;
	constexpr double EARTH_CIRCUM = 2 * PI * 6378137;
	
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	int port, start, end, threadNum, sumo_warm;
	int packet_size_mode, propagation_mode, scheme_mode;
	int LoD, divNum, tileSize;
	float prob = 0.;
	vector<thread> threads;

	cout << "##### input simulation parameter ####" << endl;
	cout << "port << ";	cin >> port;
	cout << "start << "; cin >> start;
	cout << "end << "; cin >> end;
	cout << "sumo warm << "; cin >> sumo_warm;
	cout << "thread num << "; cin >> threadNum;

	cout << endl << "#### input mode parameter ####" << endl;
	cout << "packet size: 300byte(0), 190byte(1) << "; cin >> packet_size_mode;
	cout << "propagation mode: WINNER+B1(0), freespace(1) << "; cin >> propagation_mode;
	cout << "scheme mode: original(0), proposed(1), random(2) << "; cin >> scheme_mode;

	_mkdir("result");
	if (scheme_mode == 0) {
		cout << "resource keep probability << "; cin >> prob;
	}
	else if (scheme_mode == 1) {
		cout << "LoD << "; cin >> LoD;
		cout << "division number << "; cin >> divNum;
		if (!(divNum == 2 || divNum == 5 || divNum == 10)) {
			cerr << "wrong division number:" << divNum << endl;
			exit(-1);
		}
		tileSize = (cos(PI / 5) * EARTH_CIRCUM) / pow(2, LoD);
	}


	auto start_time = chrono::system_clock::now();
	for (int i = 0; i < threadNum; i++) {
		threads.emplace_back(thread(process, port, start, end, prob, sumo_warm, threadNum, packet_size_mode, propagation_mode, scheme_mode, i, tileSize, divNum));
	}
	for (auto& thread : threads) {
		thread.join();
	}

	map<int, pair<ull, ull>> resultMap;
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + ".csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream output("result/sum_result.csv");
	for (auto&& elem : resultMap) {
		output << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}

	resultMap.clear();
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_LOS.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream outputLOS("result/sum_LOS_result.csv");
	for (auto&& elem : resultMap) {
		outputLOS << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}

	resultMap.clear();
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_NLOS.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream outputNLOS("result/sum_NLOS_result.csv");
	for (auto&& elem : resultMap) {
		outputNLOS << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}

	resultMap.clear();
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_noInter.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream outputNoInter("result/sum_noInter_result.csv");
	for (auto&& elem : resultMap) {
		outputNoInter << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}


	resultMap.clear();
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_noInter_LOS.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream outputNoInterLOS("result/sum_noInter_LOS_result.csv");
	for (auto&& elem : resultMap) {
		outputNoInterLOS << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}


	resultMap.clear();
	for (int i = start; i <= end; i++) {
		ifstream ifs("result/test" + to_string(i) + "_noInter_NLOS.csv");

		string line;
		while (getline(ifs, line)) {

			vector<string> strvec = split(line, ',');
			resultMap[stoi(strvec[0])].first += stoi(strvec[1]);
			resultMap[stoi(strvec[0])].second += stoi(strvec[2]);
		}
		ifs.close();
	}
	ofstream outputNoInterNLOS("result/sum_noInter_NLOS_result.csv");
	for (auto&& elem : resultMap) {
		outputNoInterNLOS << elem.first << "," << (double)elem.second.first / ((double)elem.second.first + (double)elem.second.second) << endl;
	}

	output.close();
	outputLOS.close();
	outputNLOS.close();
	outputNoInter.close();
	outputNoInterLOS.close();
	outputNoInterNLOS.close();
	
	auto end_time = chrono::system_clock::now();
	double elapsed_time = chrono::duration_cast<chrono::minutes>(end_time - start_time).count();
	cout << "elapsed time: " << elapsed_time << "(m)" << endl;
	system("pause");
}
