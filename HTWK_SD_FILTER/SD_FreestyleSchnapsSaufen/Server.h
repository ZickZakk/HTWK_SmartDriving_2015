#ifndef _SERVER_H_
#define _SERVER_H_

class Server
{
private:
	int port;
	int stopLinesLeft;
	void task1(void);

public:
	Server(const int &port);
	~Server();

	void startServer(void);
	int getCurrentStopLinesLeft(void);
};

#endif