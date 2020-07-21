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
	 *@param id ���M�ԗ�ID
	 *@param rri ���M����
	 *@param mw ��M�d��(mw)
	 *@param dB ��M�d��(dB)
	 *@param f1 ��M������
	 *@param f2 ����d���ǂ���
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