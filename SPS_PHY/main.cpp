#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define NOMINMAX
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <string>
#include <chrono>
#include <unordered_map>
#include <map>
//#include "Simulator.h"
#include "Table.h"
#include "Vehicle.h"
#include <utils/traci/TraCIAPI.h>

using namespace std;

void runSUMO(string port, int test_num) {
	STARTUPINFO si = { 0 };
	PROCESS_INFORMATION pi = { 0 };
	string exePath("sumo -c E:/SUMO/test" + to_string(test_num) + ".sumocfg --remote-port " + port);
	char* cstr = new char[exePath.size() + 1];
	strcpy_s(cstr, exePath.size() + 1, exePath.c_str());
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


void runSUMO(string port) {
	STARTUPINFO si = { 0 };
	PROCESS_INFORMATION pi = { 0 };
	string exePath("sumo -c E:/SUMO/test" + to_string(1) + ".sumocfg --remote-port " + port);
	char* cstr = new char[exePath.size() + 1];
	strcpy_s(cstr, exePath.size() + 1, exePath.c_str());
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
 

//int main() {
//	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
//
//
//	string port;
//	int start, base, end;
//
//	port = "1337";
//	start = base = end = 1;
//
//	//cout << "port << ";	cin >> port;
//	//cout << "start << "; cin >> process_num;
//	//cout << "base << "; cin >> base;
//	//cout << "end << "; cin >> end;
//	auto start_time = chrono::system_clock::now();
//	for (int i = start; i < end; i += base) {
//		string fname("test" + to_string(i) + ".csv");
//		runSUMO(port, i);
//
//		Sleep(100);
//
//		//Simulator simulator(stoi(port));
//		Simulator simulator(stoi(port), fname);
//	}
//	auto end_time = chrono::system_clock::now();
//	double elapsed_time = chrono::duration_cast<chrono::minutes>(end_time - start_time).count();
//}

//int main() {
//	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
//	TraCIAPI sumo;
//	string port;
//	port = "1337";
//	int numSubCH = 2;
//
//	vector<Vehicle*> vehicleList;
//	auto start_time = chrono::system_clock::now();
//	string sumo_addr("sumo -c E:/urban5.sumocfg --remote-port " + port);
//	runSUMO(port, 0, sumo_addr);
//	Sleep(100);
//	sumo.connect("localhost", stoi(port));
//	float simstep;
//	cin >> simstep;
//	system("cls");
//	sumo.simulationStep(simstep);
//	for (string veID : sumo.vehicle.getIDList()) {
//		vehicleList.emplace_back(new Vehicle(veID, sumo.vehicle.getPosition(veID).x, sumo.vehicle.getPosition(veID).y,
//			sumo.vehicle.getLaneID(veID), numSubCH));
//	}
//
//	for (auto&& v1 : vehicleList) {
//		for (auto&& v2 : vehicleList) {
//			if (v1 == v2)
//				continue;
//				v1->calcRecvPower(v2);
//		}
//	}
//
//	for (auto&& v1 : vehicleList) {
//		for (auto&& v2 : vehicleList) {
//			if (v1 == v2)
//				continue;
//				v1->decisionPacket(v2);
//		}
//	}
//
//	sumo.close();
//	auto end_time = chrono::system_clock::now();
//	double elapsed_time = chrono::duration_cast<chrono::minutes>(end_time - start_time).count();
//	return 0;
//}

int main() {
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	unordered_map<string, Vehicle*> vehicleList;

	for (int i = 0; i < 10;i++) {
		string str = to_string(i);
		vehicleList[to_string(i)] = new Vehicle(to_string(i), 0, 0, "11111", 2, 0.);
	}

	auto&& itr = vehicleList.begin();
	while (itr != vehicleList.end()) {
		if (itr->first == "0") {
			delete(vehicleList["0"]);
			vehicleList.erase(itr++);
		}
		else
			++itr;
	}

	for (auto&& elem : vehicleList) {
		cout << elem.first << endl;
		delete(elem.second);
	}
}