#ifndef _CLIENT_H_
#define _CLIENT_H_

class Client
{
private:
	std::string host;
	int portNo;
	struct sockaddr_in svrAdd;
	struct hostent *server;
	int listenFd;

public:
	Client();
	~Client();

	bool connectToServer(const std::string &host, const int &port);
	void sendValue(const int &value);
};

#endif