#ifndef PACKET
#define PACKET
class Packet {
private:
	string senderID;
	int reserveTime;
	float recvPower_mw;
	float recvPower_dB;
	bool packetOk;
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
	Packet(string id, int t, float mw, float dB, bool f1, bool f2) :
		senderID(id), reserveTime(t), recvPower_mw(mw), recvPower_dB(dB), packetOk(f1), noMonitor(f2) {}

	string getID() {
		return senderID;
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

	bool isNoMonitor() {
		return noMonitor;
	}
};

#endif