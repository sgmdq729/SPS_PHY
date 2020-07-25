#ifndef PACKET
#define PACKET

class Vehicle;
class Packet {
private:
	Vehicle* v_p;
	int reserveTime;
	float recvPower_mw;
	float recvPower_dB;
	bool packetOk;
	bool noInterPacketOk;
	bool noMonitor;
public:
	/**
	 *@param id 送信車両ID
	 *@param rri 送信周期
	 *@param mw 受信電力(mw)
	 *@param dB 受信電力(dB)
	 *@param f1 受信成功か
	 *@param f2 半二重かどうか
	 */
	Packet(Vehicle* v, int reserveTime, float mw, float dB, bool f1, bool f3, bool f2) :
		v_p(v), reserveTime(reserveTime), recvPower_mw(mw), recvPower_dB(dB), packetOk(f1), noInterPacketOk(f3), noMonitor(f2) {}

	Vehicle* getVe() {
		return v_p;
	}

	int getReserve() {
		return reserveTime;
	}

	float getRecvPower_mw() {
		return recvPower_mw;
	}

	float getRecvPower_dB() {
		return recvPower_dB;
	}

	bool isPacketOk() {
		return packetOk;
	}
	
	bool isNoInterPacketOk() {
		return noInterPacketOk;
	}

	bool isNoMonitor() {
		return noMonitor;
	}
};

#endif