#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>

void error(char *msg) {
    perror(msg);
    exit(0);
}

void sendData(int sockfd, char* s){
    int n;
    char buffer[32];
    strcpy(buffer, s);
    if ( (n = write( sockfd, buffer, strlen(buffer) ) ) < 0 )
        error( const_cast<char *>( "ERROR writing to socket") );
}

char* getData(int sockfd){
    char *buffer;
    buffer  = (char *)malloc(32);
    int n;
    if ( (n = read(sockfd,buffer,31) ) < 0 )
        error( const_cast<char *>( "ERROR reading from socket") );
    buffer[n] = '\0';
    return buffer;
}

void sendInt( int sockfd, int x ) {
    int n;
    char buffer[32];
    sprintf( buffer, "%d\n", x );
    if ( (n = write( sockfd, buffer, strlen(buffer) ) ) < 0 )
        error( const_cast<char *>( "ERROR writing to socket") );
    buffer[n] = '\0';
}

int getInt( int sockfd ) {
  char buffer[32];
  int n;
  if ( (n = read(sockfd,buffer,31) ) < 0 )
       error( const_cast<char *>( "ERROR reading from socket") );
  buffer[n] = '\0';
  return atoi( buffer );
}



int main(int argc, char *argv[])
{
    int sockfd, portno = 5200, n;
    //char serverIp[] = "10.42.0.42";
    //char serverIp[] = "172.20.10.12";
    char serverIp[] = "192.168.43.204";
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];
    int data;
    if (argc < 3) {
      // error( const_cast<char *>( "usage myClient2 hostname port\n" ) );
      printf( "contacting %s on port %d\n", serverIp, portno );
      // exit(0);
    }
    if ( ( sockfd = socket(AF_INET, SOCK_STREAM, 0) ) < 0 )
        error( const_cast<char *>( "ERROR opening socket") );

    if ( ( server = gethostbyname( serverIp ) ) == NULL )
        error( const_cast<char *>("ERROR, no such host\n") );
   
    bzero( (char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy( (char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);
    if ( connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0)
        error( const_cast<char *>( "ERROR connecting") );
	
	bool ifbreak = false;
	while(!ifbreak){
		printf("Please enter command:\n\r");
		scanf("%32s", buffer);
                          if(strcmp(buffer, "drive") == 0) n = 0;
                          else if(strcmp(buffer, "move") == 0) n = 1;
                          else if(strcmp(buffer, "stop") == 0) n = 2;
                          else if(strcmp(buffer, "fix_turn") == 0) n = 3;
                          else if(strcmp(buffer, "turn") == 0) n = 4;
                          else if(strcmp(buffer, "goto") == 0) n = 5;
                          else if(strcmp(buffer, "return") == 0) n = 6;
                          else if(strcmp(buffer, "reset") == 0) n = 7;
                          else if(strcmp(buffer, "dump") == 0) n = 8;
                          else if(strcmp(buffer, "quit") == 0) n = -2;
                          else n = -1;
                          printf("Sending: %d\r\n", n);
		sendInt(sockfd, n);
        char posX[256], posY[256];
		switch( n ){
                                        case 0:
                                                    printf("Robot Moving\n\r");
                                                    break;
                                        case 1:
				printf("Enter linear velocity...\n\r");
				scanf("%32s", buffer);
				sendData(sockfd, buffer);
				printf("Enter angular velocity...\n\r");
				scanf("%32s", buffer);
				sendData(sockfd, buffer);
				break;
                                        case 2:
                                                    printf("Robot Stop....\n\r");
                                                    break;
                                        case 3:
                                                    printf("Robot Turning....\n\r");
                                                    break;
                                        case 4:
                                                    printf("Enter angular velocity...\n\r");
                                                    scanf("%32s", buffer);
                                                    sendData(sockfd, buffer);
                                                    printf("Robot Turning with angular velocity: %s\n\r", buffer);
                                                    break;
                                        case 5: 
                                                    printf("Enter X coordinate\n\r");
                                                    scanf("%32s", posX);
                                                    sendData(sockfd, posX);
                                                    printf("Enter Y coordinate\n\r");
                                                    scanf("%32s", posY);
                                                    sendData(sockfd, posY);
                                                    printf("Move to (%s, %s)...\n\r", posX, posY);
                                                    break;
                                        case 6:
                                                    printf("Return to original position...\n\r");
                                                    break;
                                        case 7:
                                                    printf("Enter X coordinate\n\r");
                                                    scanf("%32s", posX);
                                                    sendData(sockfd, posX);
                                                    printf("Enter Y coordinate\n\r");
                                                    scanf("%32s", posY);
                                                    sendData(sockfd, posY);
                                                    printf("Reset origin to (%s, %s)...\n\r", posX, posY);
                                                    break;
                                        case 8:
                                                    printf("Dumping...\n\r");
                                                    for(int i = 0; i < 12; i++) {
                                                        printf("%s\n\r", getData(sockfd));
                                                        char* temp = (char*) malloc(3);
                                                        strcpy(temp, "Y");
                                                        sendData(sockfd, temp);
                                                    }
                                                    break;
                                        case -1:
                                                    printf("Unable to recognize the command, please enter your command again\r\n");
                                                    break;
			case -2:
				ifbreak = true;
				break;
			default:
				break;
		}
		//printf("%d\n",n);
	}
    close( sockfd );
    return 0;
}