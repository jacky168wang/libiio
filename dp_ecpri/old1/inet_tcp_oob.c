/*
	https://docs.oracle.com/cd/E19253-01/817-4415/sockets-85885/index.html

Out-of-Band Data

The stream socket abstraction includes out-of-band data. Out-of-band data is a logically independent transmission channel between a pair of connected stream sockets. Out-of-band data is delivered independent of normal data. The out-of-band data facilities must support the reliable delivery of at least one out-of-band message at a time. This message can contain at least one byte of data. At least one message can be pending delivery at any time.

With in-band signaling, urgent data is delivered in sequence with normal data, and the message is extracted from the normal data stream. The extracted message is stored separately. Users can choose between receiving the urgent data in order and receiving the data out of sequence, without having to buffer the intervening data.

Using MSG_PEEK, you can peek at out-of-band data. If the socket has a process group, a SIGURG signal is generated when the protocol is notified of its existence. A process can set the process group or process ID to deliver SIGURG to with the appropriate fcntl(2) call, as described in Interrupt-Driven Socket I/O for SIGIO. If multiple sockets have out-of-band data waiting for delivery, a select(3C) call for exceptional conditions can determine which sockets have such data pending.

A logical mark is placed in the data stream at the point at which the out-of-band data was sent. The remote login and remote shell applications use this facility to propagate signals between client and server processes. When a signal is received, all data up to the mark in the data stream is discarded.

To send an out-of-band message, apply the MSG_OOB flag to send(3SOCKET) or sendto(3SOCKET). To receive out-of-band data, specify MSG_OOB to recvfrom(3SOCKET) or recv(3SOCKET). If out-of-band data is taken in line the MSG_OOB flag is not needed. The SIOCATMARK ioctl(2) indicates whether the read pointer currently points at the mark in the data stream:

int yes;
ioctl(s, SIOCATMARK, &yes);

If yes is 1 on return, the next read returns data after the mark. Otherwise, assuming out-of-band data has arrived, the next read provides data sent by the client before sending the out-of-band signal. The routine in the remote login process that flushes output on receipt of an interrupt or quit signal is shown in the following example. This code reads the normal data up to the mark to discard the normal data, then reads the out-of-band byte.

A process can also read or peek at the out-of-band data without first reading up to the mark. Accessing this data when the underlying protocol delivers the urgent data in-band with the normal data, and sends notification of its presence only ahead of time, is more difficult. An example of this type of protocol is TCP, the protocol used to provide socket streams in the Internet family. With such protocols, the out-of-band byte might not yet have arrived when recv(3SOCKET) is called with the MSG_OOB flag. In that case, the call returns the error of EWOULDBLOCK. Also, the amount of in-band data in the input buffer might cause normal flow control to prevent the peer from sending the urgent data until the buffer is cleared. The process must then read enough of the queued data to clear the input buffer before the peer can send the urgent data.

*/

/*
---- Asynchronous Socket I/O
Asynchronous communication between processes is required in applications that simultaneously handle multiple requests. Asynchronous sockets must be of the SOCK_STREAM type. To make a socket asynchronous, you issue a fcntl(2) call, as shown in the following example.

#include <fcntl.h>
#include <sys/file.h>
...
int fileflags;
int s;
...
s = socket(AF_INET6, SOCK_STREAM, 0);
...
if (fileflags = fcntl(s, F_GETFL ) == -1)
    perror("fcntl F_GETFL");
    exit(1);
}
if (fcntl(s, F_SETFL, fileflags | FNDELAY | FASYNC) == -1)
    perror("fcntl F_SETFL, FNDELAY | FASYNC");
    exit(1);
}

---- Interrupt-Driven Socket I/O
The SIGIO signal notifies a process when a socket, or any file descriptor, has finished a data transfer.

	// step1: Set up a SIGIO signal handler with the signal() or sigvec() calls.
	signal(SIGIO, io_handler);

    // step2:  Use fcntl() to let the kernel route the SIGIO/SIGURG signal to one process by assigning its ID and group-ID.
    // The default process group of a socket is group 0.
	if (fcntl(s, F_SETOWN, getpid()) < 0) {
		perror("fcntl F_SETOWN");
		exit(1);
	}

    // step3: Convert the socket to asynchronous, as shown in Asynchronous Socket I/O.
*/



/* 发送带外数据的客户端程序 */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char* argv[])
{
    if( argc <= 2 )
    {
        printf( "usage: %s ip_address port_number\r\n", basename( argv[0] ) );
        return 1;
    }
    const char* ip = argv[1];
    int port = atoi( argv[2] );

    struct sockaddr_in server_address;
    bzero( &server_address, sizeof( server_address ) );
    server_address.sin_family = AF_INET;
    inet_pton( AF_INET, ip, &server_address.sin_addr );
    server_address.sin_port = htons( port );

    int sockfd = socket( PF_INET, SOCK_STREAM, 0 );
    assert( sockfd >= 0 );
    if ( connect( sockfd, ( struct sockaddr* )&server_address, sizeof( server_address ) ) < 0 )
    {
        printf( "connection failed\r\n" );
    }
    else
    {
        printf( "send oob data out\r\n" );
        const char* oob_data = "abc";
        const char* normal_data = "123";
        send( sockfd, normal_data, strlen( normal_data ), 0 );
        send( sockfd, oob_data, strlen( oob_data ), MSG_OOB );//MSG_OOB标志的send发送带外数据，注意带外数据，接收端的带外缓存只有1B所以只有c是真正的带外数据，同理带MSG_OOB标志的recv能接收带外数据
        send( sockfd, normal_data, strlen( normal_data ), 0 );//发送普通数据
    }

    close( sockfd );
    return 0;
}

/* 带MSG_OOB标志的recv */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#define BUF_SIZE 1024

int main( int argc, char* argv[] )
{
    if( argc <= 2 )
    {
        printf( "usage: %s ip_address port_number\r\n", basename( argv[0] ) );
        return 1;
    }
    const char* ip = argv[1];
    int port = atoi( argv[2] );

    struct sockaddr_in address;
    bzero( &address, sizeof( address ) );
    address.sin_family = AF_INET;
    inet_pton( AF_INET, ip, &address.sin_addr );
    address.sin_port = htons( port );

    int sock = socket( PF_INET, SOCK_STREAM, 0 );
    assert( sock >= 0 );

    int ret = bind( sock, ( struct sockaddr* )&address, sizeof( address ) );
    assert( ret != -1 );

    ret = listen( sock, 5 );
    assert( ret != -1 );

    struct sockaddr_in client;
    socklen_t client_addrlength = sizeof( client );
    int connfd = accept( sock, ( struct sockaddr* )&client, &client_addrlength );
    if ( connfd < 0 )
    {
        printf( "errno is: %d\r\n", errno );
    }
    else
    {
        char buffer[ BUF_SIZE ];

        memset( buffer, '\0', BUF_SIZE );
        ret = recv( connfd, buffer, BUF_SIZE-1, 0 );
        printf( "got %d bytes of normal data '%s'\r\n", ret, buffer );

        memset( buffer, '\0', BUF_SIZE );
        ret = recv( connfd, buffer, BUF_SIZE-1, MSG_OOB );//带MSG_OOB标志的recv函数能接收带外数据
        printf( "got %d bytes of oob data '%s'\r\n", ret, buffer );

        memset( buffer, '\0', BUF_SIZE );
        ret = recv( connfd, buffer, BUF_SIZE-1, 0 );
        printf( "got %d bytes of normal data '%s'\r\n", ret, buffer );

        close( connfd );
    }

    close( sock );
    return 0;
}


/* 通过select返回异常事件来检测带外数据到来 */

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>

int main( int argc, char* argv[] )
{
	if( argc <= 2 )
	{
		printf( "usage: %s ip_address port_number\r\n", basename( argv[0] ) );
		return 1;
	}
	const char* ip = argv[1];
	int port = atoi( argv[2] );
	printf( "ip is %s and port is %d\r\n", ip, port );

	int ret = 0;
        struct sockaddr_in address;//服务端地址
        bzero( &address, sizeof( address ) );
        address.sin_family = AF_INET;
        inet_pton( AF_INET, ip, &address.sin_addr );
        address.sin_port = htons( port );

	int listenfd = socket( PF_INET, SOCK_STREAM, 0 );
	assert( listenfd >= 0 );

        ret = bind( listenfd, ( struct sockaddr* )&address, sizeof( address ) );
	assert( ret != -1 );

	ret = listen( listenfd, 5 );
	assert( ret != -1 );

	struct sockaddr_in client_address;//客户端地址
        socklen_t client_addrlength = sizeof( client_address );
	int connfd = accept( listenfd, ( struct sockaddr* )&client_address, &client_addrlength );
	if ( connfd < 0 )
	{
		printf( "errno is: %d\r\n", errno );
		close( listenfd );
	}

	char remote_addr[INET_ADDRSTRLEN];
	//printf( "connected with ip: %s and port: %d\r\n", inet_ntop( AF_INET, &client_address.sin_addr, remote_addr, INET_ADDRSTRLEN ), ntohs( client_address.sin_port ) );

	char buf[1024];
        fd_set read_fds;//可读事件集合
        fd_set exception_fds;//异常事件集合(含有带外数据的事件是异常事件)

        FD_ZERO( &read_fds );//事件集合清零
        FD_ZERO( &exception_fds );

        int nReuseAddr = 1;
	setsockopt( connfd, SOL_SOCKET, SO_OOBINLINE, &nReuseAddr, sizeof( nReuseAddr ) );//
	while( 1 )
	{
		memset( buf, '\0', sizeof( buf ) );
		FD_SET( connfd, &read_fds );//将文件描述符connfd和可读事件集合绑定,由于select内核在线修改事件集合以通知应用程序就绪事件，所以下次select调用前都需要重新绑定
		FD_SET( connfd, &exception_fds );

        	ret = select( connfd + 1, &read_fds, NULL, &exception_fds, NULL );//select无限期等待事件就绪
		//printf( "select one\r\n" );
        	if ( ret < 0 )
        	{
                	printf( "selection failure\r\n" );
                	break;
        	}

        	if ( FD_ISSET( connfd, &read_fds ) )//处理可读事件，这里表示客户端发送普通数据到来
		{
        		ret = recv( connfd, buf, sizeof( buf )-1, 0 );
			if( ret <= 0 )
			{
				break;
			}
			printf( "get %d bytes of normal data: %s\r\n", ret, buf );
		}
		else if( FD_ISSET( connfd, &exception_fds ) )//处理异常事件，这里表示客户端发送带外数据到来
        	{
        		ret = recv( connfd, buf, sizeof( buf )-1, MSG_OOB );//MSG_OOB标志
			if( ret <= 0 )
			{
				break;
			}
			printf( "get %d bytes of oob data: %s\r\n", ret, buf );
        	}

	}

	close( connfd );
	close( listenfd );
	return 0;
}


/* 采用SIGURG信号检测带外数据的到来 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>

#define BUF_SIZE 1024//缓冲区大小

static int connfd;

void sig_urg( int sig )//信号处理函数(SIGURG信号处理带外数据)
{
    int save_errno = errno;
    
    char buffer[ BUF_SIZE ];
    memset( buffer, '\0', BUF_SIZE );
    int ret = recv( connfd, buffer, BUF_SIZE-1, MSG_OOB );//接收带外数据
    printf( "got %d bytes of oob data '%s'\r\n", ret, buffer );//输出带外数据

    errno = save_errno;
}

void addsig( int sig, void ( *sig_handler )( int ) )//信号安装
{
    struct sigaction sa;//sigaction结构体
    memset( &sa, '\0', sizeof( sa ) );
    sa.sa_handler = sig_handler;
    sa.sa_flags |= SA_RESTART;//被信号中断的系统调用自动重启
    sigfillset( &sa.sa_mask );//设置全部信号为进程信号掩码
    assert( sigaction( sig, &sa, NULL ) != -1 );//信号处理
}

int main( int argc, char* argv[] )
{
    if( argc <= 2 )
    {
        printf( "usage: %s ip_address port_number\r\n", basename( argv[0] ) );
        return 1;
    }
    const char* ip = argv[1];
    int port = atoi( argv[2] );

    struct sockaddr_in address;//服务端地址
    bzero( &address, sizeof( address ) );
    address.sin_family = AF_INET;
    inet_pton( AF_INET, ip, &address.sin_addr );
    address.sin_port = htons( port );

    int sock = socket( PF_INET, SOCK_STREAM, 0 );
    assert( sock >= 0 );

    int ret = bind( sock, ( struct sockaddr* )&address, sizeof( address ) );
    assert( ret != -1 );

    ret = listen( sock, 5 );
    assert( ret != -1 );

    struct sockaddr_in client;//客户端地址
    socklen_t client_addrlength = sizeof( client );
    connfd = accept( sock, ( struct sockaddr* )&client, &client_addrlength );//建立客户端连接
    if ( connfd < 0 )
    {
        printf( "errno is: %d\r\n", errno );
    }
    else
    {
        addsig( SIGURG, sig_urg );//添加SIGURG信号
        fcntl( connfd, F_SETOWN, getpid() );//SIGURG的前提条件是进程必须持有文件描述符connfd

        char buffer[ BUF_SIZE ];
        while( 1 )
        {
            memset( buffer, '\0', BUF_SIZE );
            ret = recv( connfd, buffer, BUF_SIZE-1, 0 );//接收客户端发送来的数据，若有带外数据到来则激活SIGURG信号处理函数
            if( ret <= 0 )
            {
                break;
            }
            printf( "got %d bytes of normal data '%s'\r\n", ret, buffer );
        }

        close( connfd );
    }

    close( sock );
    return 0;
}

