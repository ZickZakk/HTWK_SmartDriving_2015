#include "stdafx.h"

Client::Client()
{
}

Client::~Client()
{

}

bool Client::connectToServer(const std::string &host, const int &port)
{
	this->host = host;
	this->portNo = port;

	this->server = gethostbyname(this->host.c_str());
	listenFd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	if(listenFd < 0)
    {
        LOG_INFO("Cannot open socket");

        return false;
    }

	if(server == NULL)
    {
        LOG_INFO("Host does not exist");
        return false;
    }

	bzero((char *) &svrAdd, sizeof(svrAdd));
    svrAdd.sin_family = AF_INET;
    
    bcopy((char *) server->h_addr, (char *) &svrAdd.sin_addr.s_addr, server->h_length);
    
    svrAdd.sin_port = htons(portNo);
    
	int test = connect(listenFd, (struct sockaddr*) &svrAdd, sizeof(svrAdd));

	return test;
}

void Client::sendValue(const int &value)
{
	static std::string tmpValue = value + "";
	write(listenFd, tmpValue.c_str(), strlen(tmpValue.c_str()));
}