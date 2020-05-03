#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define NOMINMAX
#include <stdio.h>
#include <iostream>
#include <Windows.h>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include "Simulator.h"
#include "Table.h"
#include "Vehicle.h"
#include <utils/traci/TraCIAPI.h>

using namespace std;

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

void process(int basePort, int start, int end, float prob, int sumo_warm, int threadNum, int packet_mode, int prop_mode, int scheme_mode, int myid) {
	for (int i = start + myid; i <= end; i += threadNum) {
		printf("test%d\n", i);
		string port(to_string(basePort + myid));
		string exePath("sumo -c E:/sumo_urban/50/test" + to_string(i) + ".sumocfg --remote-port " + port);
		string resultFname("result_re/test" + to_string(i));
		runSUMO(port, i, exePath);
		Sleep(100);
		Simulator simulator(resultFname, stoi(port), prob, sumo_warm, packet_mode, prop_mode, scheme_mode);
	}
}
 

int main() {
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	int port, start, end, threadNum, sumo_warm;
	int packet_size_mode, propagation_mode, scheme_mode;
	float prob;
	vector<thread> threads;

	cout << "##### input simulation parameter ####" << endl;
	cout << "port << ";	cin >> port;
	cout << "start << "; cin >> start;
	cout << "end << "; cin >> end;
	cout << "resource keep probability << "; cin >> prob;
	cout << "sumo warm << "; cin >> sumo_warm;
	cout << "thread num << "; cin >> threadNum;

	cout << endl << "#### input mode parameter ####" << endl;
	cout << "packet size: 300byte(0), 190byte(1) << "; cin >> packet_size_mode;
	cout << "propagation_mode: WINNER+B1(0), freespace(1) << "; cin >> propagation_mode;
	cout << "scheme mode: original(0), proposed(1) << "; cin >> scheme_mode;

	auto start_time = chrono::system_clock::now();
	for (int i = 0; i < threadNum; i++) {
		threads.emplace_back(thread(process, port, start, end, prob, sumo_warm, threadNum, packet_size_mode, propagation_mode, scheme_mode, i));
	}
	for (auto& thread : threads) {
		thread.join();
	}
	
	auto end_time = chrono::system_clock::now();
	double elapsed_time = chrono::duration_cast<chrono::minutes>(end_time - start_time).count();
	cout << "elapsed time: " << elapsed_time << "(m)" << endl;
}
