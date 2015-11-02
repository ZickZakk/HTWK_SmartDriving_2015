/**
*
* ADTF Demo Source.
*
* @file
* Copyright &copy; Audi Electronics Venture GmbH. All rights reserved
*
* $Author: belkera $
* $Date: 2011-06-30 16:51:21 +0200 (Thu, 30 Jun 2011) $
* $Revision: 26514 $
*
* @remarks
*
*/
#include "stdafx.h"

void task1();

static int connFd;
const int numberOfThreads = 1;

Server::Server(const int &port)
{
	this->port = port;

}

Server::~Server()
{}

void Server::startServer(void){
            int listenFd;
            socklen_t len; //store size of the address
            struct sockaddr_in svrAdd, clntAdd;
            
            std::thread threadA[numberOfThreads];
            
            //create socket
            listenFd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
            
            if(listenFd < 0)
            {
                return ;
            }
            
            bzero((char*) &svrAdd, sizeof(svrAdd));
            
            svrAdd.sin_family = AF_INET;
            svrAdd.sin_addr.s_addr = INADDR_ANY;
            svrAdd.sin_port = htons(this->port);
            
            //bind socket
            if(bind(listenFd, (struct sockaddr *)&svrAdd, sizeof(svrAdd)) < 0)
            {
                return ;
            }
            
            listen(listenFd, 5);
            
            len = sizeof(clntAdd);
            
            int noThread = 0;
                          
                //this is where client connects. svr will hang in this mode until client conn
                connFd = accept(listenFd, (struct sockaddr *)&clntAdd, &len);
                
                if (connFd < 0)
                {
                    return ;
                }
                
                threadA[0] = std::thread(&Server::task1,this);
                
                
            
            for(int i = 0; i < numberOfThreads; i++)
            {
                threadA[0].join();
            }
       return ;     
}

int Server::getCurrentStopLinesLeft(void){
	return this->stopLinesLeft;
}

void Server::task1 (void)
{
	char temp[100];
	bzero(temp, 100);

	//Lesen der absolvierten Haltelinien	
	read(connFd, temp, sizeof(temp));

	this->stopLinesLeft = atoi(temp);

	// wenn am Ende, threads loeschen
	if(this->stopLinesLeft == 0)
	{	
		close(connFd);
	}

	return;
}
