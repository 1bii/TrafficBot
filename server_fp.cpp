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
#include <csignal>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/pose2d.hpp>
#include <ecl/linear_algebra.hpp>
#include "kobuki_driver/kobuki.hpp"
#include "/home/pi/OpenNI2/Samples/SimpleRead/AstraManager.h"

using namespace cv;
using namespace std;

int imageCnt = 0;

class KobukiManager {
public:
    KobukiManager() {
        kobuki::Parameters parameters;
        parameters.sigslots_namespace = "/kobuki";
        parameters.device_port = "/dev/ttyUSB0";
        parameters.enable_acceleration_limiter = false;
        kobuki.init(parameters);
        kobuki.enable();
        px = 0.0;
        py = 0.0;
        speedLin = 0.03;
        speedAng = 0.5;
        range = 0.05;
        sleep(1);
    }

    ~KobukiManager() {
        kobuki.setBaseControl(0,0); 
        kobuki.disable();
    }

    Mat getSign(VideoCapture& cap) {
        Mat imgOriginal, imgHSV;
        int findCircleCnt = 0, noCount = 0;
        Point prevCenter(0, 0);
        bool setCenter = false;
        bool bSuccess = cap.read(imgOriginal); // read a new frame from video
        if (!bSuccess) {//if not success, break loop 
            cout << "Cannot read a frame from video stream" << endl;
            //return false;
        }
        //namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
        //namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE );
        
        while(true) {
            bool bSuccess = cap.read(imgOriginal); // read a new frame from video
            if (!bSuccess) {//if not success, break loop
                cout << "Cannot read a frame from video stream" << endl;
                //return false;
            }
            imgOriginal.convertTo(imgOriginal, CV_32FC3, 1.0/255.0);
            Scalar m;
            m=cv::mean(imgOriginal);
            imgOriginal-=m;
            imgOriginal+=Scalar(0.3,0.3,0.3); // Changing this you can adjust color balance.
            normalize(imgOriginal,imgOriginal,0,1,cv::NORM_MINMAX);
            imgOriginal.convertTo(imgOriginal, CV_8UC3, 255.0);

            cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
            //cvtColor( imgOriginal, src_gray, CV_BGR2GRAY ); //Convert the captured frame from BGR to GRAY
        
            Mat mask1, mask2, mask3;
            inRange(imgHSV, Scalar(0, 70, 80), Scalar(15, 255, 255), mask1);
            inRange(imgHSV, Scalar(165, 70, 80), Scalar(180, 255, 255), mask2);
            inRange(imgHSV, Scalar(90, 70, 80), Scalar(150, 255, 255), mask3);
        
            Mat imgThresholded = mask1 | mask2 | mask3;

            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            vector<Vec3f> circles;

            /// Apply the Hough Transform to find the circles
            HoughCircles( imgThresholded, circles, CV_HOUGH_GRADIENT, 1, imgThresholded.cols, 30, 20, 30, 0 );
            if(circles.size() == 0) {
                cout << "No circle" << endl;
                noCount++;
                if(noCount >= 5) {
                    noCount = 0;
                    findCircleCnt = 0;
                }
                continue;
            }
            else {
                cout << "found circle" << endl;
                int height = imgOriginal.rows;
                int width = imgOriginal.cols;
                /// Draw the circles detected
                
                Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
                int radius = cvRound(circles[0][2]);
                if (center.x + 1.6*radius > width or center.x - 1.6*radius < 0 or center.y + 1.6*radius > height or center.y - 1.6*radius < 0) {
                    cout << "out of boundary" << endl;
                    noCount++;
                    continue;
                }
                if(setCenter == false) {
                    cout << "center set and continue" << endl;
                    setCenter = true;
                    prevCenter = center;
                    continue;
                }
                if(center.x - prevCenter.x > 50 or center.x - prevCenter.x < -50 or center.y - prevCenter.y > 50 or center.y - prevCenter.y < -50) {
                    cout << "not like previous circle" << endl;
                    prevCenter = center;
                    noCount++;
                    continue;
                }
                prevCenter = center;
                // circle center
                //circle( imgThresholded, center, 3, 255, -1, 8, 0 );
                // circle outline
                //circle( imgThresholded, center, radius, 255, 3, 8, 0 );
                
                Rect rect(center.x - 1.6 * radius, center.y - 1.6 * radius, 3.2 * radius, 3.2 * radius);

                Mat cropped;
                imgOriginal(rect).copyTo(cropped);
                resize(cropped, cropped, Size(60, 60));
                
                if(findCircleCnt >= 1) {
                    cout << "write image" << endl;
                    roadSign = cropped;
                    char filename[30];
                    sprintf(filename, "im%d.ppm", imageCnt);
                    imwrite(filename, roadSign);
                    imageCnt++;
                    return roadSign;
                }
                findCircleCnt++;
            }
            
            //imshow("Thresholded Image", imgThresholded); //show the thresholded image
            //imshow( "Hough Circle Transform Demo", imgOriginal );
            waitKey(1);
        }
        return roadSign;
    }

    void robotMove(float lin_v, float ang_v){
        kobuki.setBaseControl(lin_v, ang_v);
        return;
    }
    
    Mat getRoadSign() {
        return roadSign;
    }

    void run() {
        robotMove(speedLin, 0.0);
        nano_sleep(1000);
    }

    void set20() {
        speedLin = 0.03;
        run();
    }

    void set60() {
        speedLin = 0.12;
        run();
    }

    void turnRight() {
        robotMove(0.0, 0.0);
        nano_sleep(1000);
        nowHead = kobuki.getHeading();
        double targetHead = nowHead - ecl::pi/2;
        if(targetHead < -ecl::pi) {
            targetHead += 2*ecl::pi;
        }
        robotMove(0.0, -speedAng);
        while(nowHead - targetHead > range or nowHead - targetHead < -range) {
            nano_sleep(1000);
            nowHead = kobuki.getHeading();
        }
        robotMove(0.0, 0.0);
        sleep(1);
    }

    void turnLeft() {
        robotMove(0.0, 0.0);
        nano_sleep(1000);
        nowHead = kobuki.getHeading();
        double targetHead = nowHead + ecl::pi/2;
        if(targetHead > ecl::pi) {
            targetHead -= 2*ecl::pi;
        }
        robotMove(0.0, speedAng);
        while(nowHead - targetHead > range or nowHead - targetHead < -range) {
            nano_sleep(1000);
            nowHead = kobuki.getHeading();
        }
        robotMove(0.0, 0.0);
        sleep(1);
    }

    void stop() {
        robotMove(0.0, 0.0);
        sleep(3);
        run();
    }

private:
    kobuki::Kobuki kobuki;
    kobuki::CoreSensors::Data coreData;
    double px, py, nowHead, speedLin, speedAng, range;
    bool obstacle;
    ecl::Sleep sleep;
    ecl::NanoSleep nano_sleep;
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    ecl::Pose2D<double> pose;
    AstraManager Astra;
    openni::DepthPixel* depth;
    Mat roadSign;
};

void error( char *msg ) {
    perror(  msg );
    exit(1);
}

void sendDataChar(int sockfd, char* s){
    int n;
    char buffer[32];
    strcpy(buffer, s);
    if ( (n = write( sockfd, buffer, strlen(buffer) ) ) < 0 )
        error( const_cast<char *>( "ERROR writing to socket") );
}

void sendDataInt( int sockfd, int x ) {
    int n;
    char buffer[32];
    sprintf( buffer, "%3d\n", x );
    if ( (n = write( sockfd, buffer, strlen(buffer) ) ) < 0 )
        error( const_cast<char *>( "ERROR writing to socket") );
    bzero(buffer, sizeof(buffer));
    //buffer[n] = '\0';
}

char* getData(int sockfd){
    char* str = (char*) malloc(32);
    int n;
    if ( (n = read(sockfd,str,31) ) < 0 )
        error( const_cast<char *>( "ERROR reading from socket") );
    str[n] = '\0';
    return str;
}

int createSocket(int portno) {
    
     int sockfd, newsockfd, clilen;
     char buffer[256];
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     int cmd, data;
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0)
         error( const_cast<char *>("ERROR opening socket") );
     bzero((char *) &serv_addr, sizeof(serv_addr));

     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons( portno );
     if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
       error( const_cast<char *>( "ERROR on binding" ) );
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     printf( "waiting for new client...\n" );
    if ( ( newsockfd = accept( sockfd, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen) ) < 0 )
            error( const_cast<char *>("ERROR on accept") );
    printf( "opened new communication with client\n" );

    return newsockfd;
}

void sendImage(int newsockfd, Mat img){

    //int height = img.rows;
    //int width = img.cols;
    int n;
    char buffer[256];
    /*
    //Send Picture Size
    printf("Sending Picture Size: (%d, %d)\n", width, height);
    sendDataInt(newsockfd, width);
    sendDataInt(newsockfd, height);

    //waiting for ack
    while((n = read(newsockfd, buffer, 256) ) < 0 ){;}
    buffer[n] = '\0';
    printf("Ack: %s\n", buffer);
    bzero(buffer, sizeof(buffer));*/

    //Send image
    Mat_<Vec3b>::iterator it = img.begin<Vec3b>();
    Mat_<Vec3b>::iterator itend = img.end<Vec3b>();
    int image_size = 60*60*3;
    char send_buffer[image_size];
    int idx = 0;
    for(;it!=itend;it++){
        send_buffer[idx] = (*it)[2];   //R
        send_buffer[idx+1] = (*it)[1]; //G
        send_buffer[idx+2] = (*it)[0]; //B
        idx+=3;
    }
    printf("Sending Picture as Byte Array with size: %lu\n", sizeof(send_buffer));
    cout << "Really Sended " << write(newsockfd, send_buffer, sizeof(send_buffer)) << endl;
    bzero(send_buffer, sizeof(send_buffer));

    // while((n = read(newsockfd, buffer, 256) ) < 0 ){;}
    // buffer[n] = '\0';
    // printf("Predict: %s\n", buffer);
    // return;
}

bool shutdown_req = false;

void signalHandler(int signum) {
    shutdown_req = true;
}

int func( int a ) {
    return 2 * a;
}

void sendInt( int sockfd, int x ) {
    int n;

    char buffer[32];
    sprintf( buffer, "%d\n", x );
    if ( (n = write( sockfd, buffer, strlen(buffer) ) ) < 0 )
        error( const_cast<char *>( "ERROR writing to socket") );
    //buffer[n] = '\0';
}

int getInt( int sockfd ) {
    char buffer[32];
    int n;

    if ( (n = read(sockfd,buffer,31) ) < 0 )
        error( const_cast<char *>( "ERROR reading from socket") );
    buffer[n] = '\0';
    return atoi( buffer );
}

float getFloat(int sockfd){
    char buffer[32];
    int n;
    if ( (n = read(sockfd,buffer,31) ) < 0 )
        error( const_cast<char *>( "ERROR reading from socket") );
    buffer[n] = '\0';
    return atof( buffer );
}
/*
void RobotMoveForward(KobukiManager* kbi_mngr){
    std::cout<<"ROBOT MOVING"<<std::endl;
    ecl::Pose2D<double> pose;
    ecl::Sleep sleep(1);
    kbi_mngr->robotMove(0.2, 0.0);
    return;
}

void RobotStop(KobukiManager* kbi_mngr){
    std::cout<<"ROBOT MOVING"<<std::endl;
    ecl::Pose2D<double> pose;
    ecl::Sleep sleep(1);
    kbi_mngr->robotMove(0.0, 0.0);
    return;
}

void RobotMove(KobukiManager* kbi_mngr, float lv, float av){
    std::cout<<"ROBOT MOVING"<<std::endl;
    ecl::Pose2D<double> pose;
    ecl::Sleep sleep(1);
    kbi_mngr->robotMove(lv, av);
    int s = 0;
    sleep(1);
    while(s < 5){
        kbi_mngr->updatePose();
        kbi_mngr->getPose();
        sleep(1);
        s+=1;
    }
    kbi_mngr->robotMove(0.0, 0.0);
}
void RobotTurn(KobukiManager* kbi_mngr, float av = 0.2){
    ecl::Sleep sleep(1);
    kbi_mngr->robotMove(0.0, av);
    return;
}

void DumpPose(KobukiManager* kbi_mngr, int sockfd){
    kobuki::CoreSensors::Data cd = kbi_mngr->getCoreData();
    sleep(1);
    char* data = (char*) malloc(32);
    sprintf(data, "time stamp = %d\n", cd.time_stamp);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "bumper = %d\n", cd.bumper);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "wheel drop = %d\n", cd.wheel_drop);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "cliff = %d\n", cd.cliff);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "left encoder = %d\n", cd.left_encoder);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "right encoder = %d\n", cd.right_encoder);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "left pwm = %s\n", cd.left_pwm);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "right pwm = %s\n", cd.right_pwm);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "buttons = %d\n", cd.buttons);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "battery = %d\n", cd.battery);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "charger = %d\n", cd.charger);
    sendData(sockfd, data);
    getData(sockfd);
    sprintf(data, "over current = %d\n", cd.over_current);
    sendData(sockfd, data);
    getData(sockfd);
    return;
}*/

int main(int argc, char *argv[]) {
    int n;
    int port = atoi(argv[1]);
    cout << "port = " << port << endl;
    //int cmd, data;
    char cmd[2];
    int count = 0;
    float x[100], y[100];
    int numObj = 0;
    KobukiManager kobuki_manager;
    ecl::Pose2D<double> pose;
    ecl::Sleep sleep(1);
    bool ifquit = false, sendIm = true;
    //bool ifclieshut = false;
    Mat img;
    VideoCapture cap(0);
    char Ack[2];
    char a = 'r';

    signal(SIGINT, signalHandler);

    int newsockfd = createSocket(port);

    cout << "Run kobuki..." << endl;
    kobuki_manager.run();

    while (!ifquit) {
        //if(kobuki_manager.getSign(cap)) {
            write(newsockfd, &a, 1);
            //cmd = getInt( newsockfd );
            while(read(newsockfd, Ack, 1) < 0) {}
            Ack[1] = '\0';
            cout << "Ack = " << Ack << endl;
            if (strcmp(Ack, "a") == 0) {
                //printf("I have received the message: %s\n\r", Ack);
                cout << "Send Image!!!" << endl;
                if(sendIm) sendImage(newsockfd, kobuki_manager.getSign(cap));
                sendIm = false;
                while(read(newsockfd, cmd, 1) < 0) {}
                cmd[1] = '\0';
                cout << "Number attempts: " << count << endl;
                count++;
                cout << "Command = " << cmd << endl;
                switch(cmd[0]) {
                    case '0':
                        kobuki_manager.set20();
                        break;
                    case '1':
                        kobuki_manager.set60();
                        break;
                    case '2':
                        kobuki_manager.stop();
                        break;
                    case '3':
                        kobuki_manager.turnRight();
                        kobuki_manager.run();
                        break;
                    case '4':
                        kobuki_manager.turnLeft();
                        kobuki_manager.run();
                        break;
                    case '5':
                        kobuki_manager.run();
                        count--;
                        break;
                    default:
                        break;
                }

                sendIm = true;
                sleep(1);
                
                if (count > 20) {
                    a = 'q';
                    write(newsockfd, &a, 1);
                    close(newsockfd);
                    ifquit = true;
                }
            }
        //}
    } 

    // int sockfd, newsockfd, portno = 5200, clilen;
    // char buffer[256];
    // struct sockaddr_in serv_addr, cli_addr;
    // printf( "using port #%d\n", portno );

    // sockfd = socket(AF_INET, SOCK_STREAM, 0) ;
    // if (sockfd < 0)
    //     error( const_cast<char *>("ERROR opening socket") );
    // bzero((char *) &serv_addr, sizeof(serv_addr));

    // serv_addr.sin_family = AF_INET;
    // serv_addr.sin_addr.s_addr = INADDR_ANY;
    // serv_addr.sin_port = htons( portno );
    // if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    //     error( const_cast<char *>( "ERROR on binding" ) );
    // listen(sockfd,5);
    // clilen = sizeof(cli_addr);

    return 0;
}

    // while ( !ifquit ) {
    //     printf( "waiting for new client...\n" );
    //     if ( ( newsockfd = accept( sockfd, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen) ) < 0 )
    //         error( const_cast<char *>("ERROR on accept") );
    //     printf( "opened new communication with client\n" );
    //     while ( !ifclieshut ) {
    //         cmd = getInt( newsockfd );
    //         printf("CMD: %d\n\r", cmd);
    //         double posX, posY;
    //         switch(cmd){
    //             case 0:
    //                 RobotMoveForward(&kobuki_manager);
    //                 break;
    //             case 1:
    //                 float av, lv;
    //                 lv = getFloat(newsockfd);
    //                 std::cout<<"LV: "<<lv<<std::endl;
    //                 av = getFloat(newsockfd);
    //                 std::cout<<"AV: "<<av<<std::endl;
    //                 RobotMove(&kobuki_manager, lv, av);
    //                 break;
    //             case 2: 
    //                 RobotStop(&kobuki_manager);
    //                 break;
    //             case 3:
    //                 RobotTurn(&kobuki_manager);
    //                 break;
    //             case 4:
    //                 av = getFloat(newsockfd);
    //                 std::cout<<"AV: "<<av<<std::endl;
    //                 RobotTurn(&kobuki_manager, av);
    //                 break;
    //             case 5:
    //                 posX = getFloat(newsockfd);
    //                 std::cout << "X cooridinate to move to: " << posX << std::endl;
    //                 posY = getFloat(newsockfd);
    //                 std::cout << "Y cooridinate to move to: " << posY << std::endl;
    //                 kobuki_manager.goTo(posX, posY);
    //                 break;
    //             case 6:
    //                 kobuki_manager.Return();
    //                 break;
    //             case 7:
    //                 posX = getFloat(newsockfd);
    //                 std::cout << "New X cooridinate: " << posX << std::endl;
    //                 posY = getFloat(newsockfd);
    //                 std::cout << "New Y cooridinate: " << posY << std::endl;
    //                 kobuki_manager.resetOrigin(posX, posY);
    //                 break;
    //             case 8:
    //                 std::cout<<"Dumping sensor data:"<<std::endl;
    //                 DumpPose(&kobuki_manager, newsockfd);
    //                 break;
    //             case -2:
    //                 ifclieshut = true;
    //                 ifquit = true;
    //                 break;          
    //             default: 
    //                 break;  
    //         }
    //     }
    //     close( newsockfd );

    //  }