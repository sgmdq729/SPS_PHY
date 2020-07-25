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
	 *@param id ���M�ԗ�ID
	 *@param rri ���M����
	 *@param mw ��M�d��(mw)
	 *@param dB ��M�d��(dB)
	 *@param f1 ��M������
	 *@param f2 ����d���ǂ���
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