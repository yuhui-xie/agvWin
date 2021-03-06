

#ifndef __SOCKET_INCLUDED
#define __SOCKET_INCLUDED

#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <string>
#ifdef _WIN32
#define WIN32
#endif
#ifdef WIN32
        #include <winsock.h>
		#pragma comment(lib, "Wsock32.lib")
#else
        #include <unistd.h>
        #include <netdb.h>
        #include <sys/types.h>
        #include <sys/socket.h>
        #include <netinet/in.h>
        #include <arpa/inet.h>
        #include <sys/wait.h>
#endif

namespace TD
{

class Socket
{
        public: 
                
                /**
                 * Creates a new socket.
                 * 
                 * @param type Type of the socket (SOCK_STREAM / SOCK_DGRAM)
                 * @param domain Domain of the socket
                 * @param protocol Protocol to use
                 */
                Socket(int type = SOCK_STREAM, int domain = AF_INET, int protocol = 0);
                
                /**
                 * Wraps a created socket.
                 * 
                 * @param sockfd Descriptor of the socket to wrap
                 * @param flag Does nothing
                 */
                Socket(int sockfd, bool flag);
                
                /**
                 * Destructor. Doesn't closes the socket!
                 */
                virtual ~Socket();

				//crti:2016-06-29,restart the socket
				int socket_restart();
                
                /**
                 * Bind the socket to a local port.
                 * 
                 * @param port Port to listem
                 */
                void bind(int port);
                
                /**
                 * Start to listem on the binded port
                 * 
                 * @param backlog Connection queue size
                 */
                void listen(int backlog = 5);
                
                /**
                 * Blocks and waits for a connection.
                 * 
                 * @return The socket to handle the connected client
                 */
                Socket accept();                
                
                /**
                 * Connects to a remote host.
                 * 
                 * @param server Server name or ip
                 * @param port Port of the server to connect
                 */
                void connect(const char* server, int port);
                
                void setTimeout(unsigned int timeout);
                
                /**
                 * Gets the address of the remote computer.
                 * 
                 * @return Address of the remote computer
                 */
                const char* getPeerName();
                
                /**
                 * Gets the address of the local computer.
                 * 
                 * @return Address of the local computer
                 */
                const char* getHostName();
                
                /**
                 * Blocks and send a message. May not send all the info.
                 * 
                 * @param msg Message to send
                 * @return How much chars where sent
                 */
                int send(const char* msg);
                
                /**
                 * Blocks and send a message. May not send all the info.
                 * 
                 * @param msg Message to send
                 * @return How much chars where sent
                 */
                int send(std::string& msg);
                
                /**
                 * Blocks and send all the message.
                 * 
                 * @param msg Message to send
                 * @return Lenght of the sent data
                 */             

                int _send(const char* msg, int size);


                int sendAll(const char* msg);
                
                /**
                 * Blocks and send all the message.
                 * 
                 * @param msg Message to send
                 * @return Lenght of the sent data
                 */             
                int sendAll(std::string& msg);
                
                /**
                 * Like sendAll(msg) + sendAll(\n)
                 * 
                 * @param msg Message to send
                 * @return Lenght of the sent data
                 */
                int sendln(const char* msg);
                
                /**
                 * Like sendAll(msg) + sendAll(\n)
                 * 
                 * @param msg Message to send
                 * @return Lenght of the sent data
                 */
                int sendln(std::string& msg);
                
                std::string recv( int len, int flags = 0);
                /**
                 * Blocks and receive a message.
                 * 
                 * @param len Size, in chars, of the receive buffer (max info to receive)
                 * @param flags Optional flags
                 * @return Buffer with the received message.
                 */
                int recv(char* buffer, int len = 1024, int flags = 0);

                
                /**
                 * Close the socket for in / out operations.
                 */ 
                void close();
                
                /**
                 * Shutdown a socket operation.
                 * A type of:
                 * 0 -> Closes the input
                 * 1 -> Closes the output
                 * 2 -> Acts like the close(), shutting down both input and output
                 * 
                 * @param type Thing to shutdown
                 */
                void shutdown(int type);
                
                /**
                 * Frees the buffer.
                 * 
                 * There's no need to call it, besides when a really huge recv 
                 * where called and you don't need it another big recv.
                 */ 
                 void clearBuffer();    

        private:
                /**
                 * Returns the ip of a host (DNS lookup).
                 * 
                 * @param server Server to lookup
                 * @return Ip of the server
                 */
                struct in_addr* getHostByName(const char* server);
        
                int sockfd;
                struct sockaddr_in my_addr;
                void error(int code);
                char* buffer;
                int buffer_size;
                
                int allocate_size;
                #ifdef WIN32
                        WSADATA wsa_data;
                #endif
};

}

#endif
