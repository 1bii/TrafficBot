#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

void error( char *msg ) {
    perror(  msg );
    exit(1);
}


void sendData( int sockfd, int x ) {
    int n;

    char buffer[32];
    sprintf( buffer, "%3d\n", x );
    if ( (n = write( sockfd, buffer, strlen(buffer) ) ) < 0 )
        error( const_cast<char *>( "ERROR writing to socket") );
    bzero(buffer, sizeof(buffer));
    //buffer[n] = '\0';
}

int createSocket()
{
    int sockfd, newsockfd, portno = 52000, clilen;
    struct sockaddr_in serv_addr, cli_addr;
    int cmd, data;

    printf( "using port #%d\n", portno );

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error( const_cast<char *>("ERROR opening socket") );
    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons( portno );
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
        sizeof(serv_addr)) < 0)
        error( const_cast<char *>( "ERROR on binding" ) );
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    printf( "waiting for new client...\n" );
    if ( ( newsockfd = accept( sockfd, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen) ) < 0 )
        error( const_cast<char *>("ERROR on accept") );
    printf( "opened new communication with client\n" );
    return newsockfd;
}

Mat readImg(std::string path){
    Mat img;
    img = imread(path, CV_LOAD_IMAGE_COLOR);  
    if(! img.data )                       
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
    }
    resize(img,img,Size(48,48));
    return img;
}

int main(int argc, char *argv[]) {
    int n;
    char buffer[256];
    int newsockfd = createSocket();
    int idx_c = 0;
    std::string path;
    Mat img;
    printf("Sending command\n");
    while(idx_c < 3){
        if(idx_c == 0){
            path = "/home/yen-ting/291/final/test/test_20.ppm";
            printf("reading img from: %s\n", path.c_str());
            img = readImg(path);
            sprintf( buffer, "r" );
            if ( (n = write( newsockfd, buffer, strlen(buffer) ) ) < 0 )
                error( const_cast<char *>( "ERROR writing to socket") );
            bzero(buffer,sizeof(buffer));
        }
        else if(idx_c == 1){
            path = "/home/yen-ting/291/final/test/test_60.ppm";
            printf("reading img from: %s\n", path.c_str());
            img = readImg(path);
            sprintf( buffer, "r" );
            if ( (n = write( newsockfd, buffer, strlen(buffer) ) ) < 0 )
                error( const_cast<char *>( "ERROR writing to socket") );
            bzero(buffer,sizeof(buffer));
        }
        else{
            sprintf( buffer, "q" );
            if ( (n = write( newsockfd, buffer, strlen(buffer) ) ) < 0 )
                error( const_cast<char *>( "ERROR writing to socket") );
            bzero(buffer,sizeof(buffer));
            break;
        }

        //waiting for ack
        while((n = read(newsockfd, buffer, 256) ) < 0 ){;}
        buffer[n] = '\0';
        printf("Ack: %s\n", buffer);
        bzero(buffer, sizeof(buffer));
        

        

        //Send image
        Mat_<Vec3b>::iterator it = img.begin<Vec3b>();
        Mat_<Vec3b>::iterator itend = img.end<Vec3b>();
        int image_size = 48*48*3;
        char send_buffer[image_size];
        int idx = 0;
        for(;it!=itend;it++){
            send_buffer[idx] = (*it)[2];     //R
            send_buffer[idx+1] = (*it)[1]; //G
            send_buffer[idx+2] = (*it)[0]; //B
            idx+=3;
        }
         printf("Sending Picture as Byte Array with size: %lu\n", sizeof(send_buffer));
         write(newsockfd, send_buffer, sizeof(send_buffer));
         bzero(send_buffer, sizeof(send_buffer));

         while((n = read(newsockfd, buffer, 256) ) < 0 ){;}
         buffer[n] = '\0';
         printf("Predict: %s\n", buffer);
         idx_c++;
    }
    close( newsockfd );
    return 0;
 }