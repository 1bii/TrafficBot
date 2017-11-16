#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
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
#include <csignal>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/pose2d.hpp>
#include <ecl/linear_algebra.hpp>
#include "kobuki_driver/kobuki.hpp"
#include "/home/pi/OpenNI2/Samples/SimpleRead/AstraManager.h"

using namespace cv;
using namespace std;



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
        speedLin = 0.06;
        speedAng = 0.5;
        range = 0.05;
        sleep(1);
        cap(0);
    }

    ~KobukiManager() {
        kobuki.setBaseControl(0,0); 
        kobuki.disable();
    }

    bool getSign() {
		Mat imgOriginal, imgHSV;
		int findCircleCnt = 0, noCount = 0;
		bool bSuccess = cap.read(imgOriginal); // read a new frame from video
	    if (!bSuccess) {//if not success, break loop 
	        cout << "Cannot read a frame from video stream" << endl;
	        return false;
	    }
	    //namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
	    //namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE );
	    
        while(noCount <= 5 && findCircleCnt <= 2) {
	    	bool bSuccess = cap.read(imgOriginal); // read a new frame from video
	        if (!bSuccess) {//if not success, break loop
	            cout << "Cannot read a frame from video stream" << endl;
	            return false;
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
	        HoughCircles( imgThresholded, circles, CV_HOUGH_GRADIENT, 1, imgThresholded.cols, 25, 20, 20, 0 );
	        if(circles.size() == 0) {
                noCount++;
                continue;
            }
            cout << "found " << circles.size() << " circles" << endl;
	        int height = imgOriginal.rows;
	        int width = imgOriginal.cols;
	        /// Draw the circles detected
	        for( size_t i = 0; i < circles.size(); i++ ) {
	            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	            int radius = cvRound(circles[i][2]);
	            if (center.x + 2*radius > width or center.x - 2*radius < 0 or center.y + 2*radius > height or center.y - 2*radius < 0) {
	                continue;
	            }
	            // circle center
	            //circle( imgOriginal, center, 3, 255, -1, 8, 0 );
	            // circle outline
	            //circle( imgOriginal, center, radius, 255, 3, 8, 0 );
	            
	            Rect rect(center.x - 2 * radius, center.y - 2 * radius, 4 * radius, 4 * radius);

	            Mat cropped;
	            imgOriginal(rect).copyTo(cropped);
	            resize(cropped, cropped, Size(48, 48));
	            imwrite("test.ppm", cropped);
	            cout << "write image" << endl;
		    	if(findCircleCnt >= 2) {
	                roadSign = cropped;
	                return true;
	            }
		    	findCircleCnt++;
	        }
	        //imshow("Thresholded Image", imgThresholded); //show the thresholded image
	        //imshow( "Hough Circle Transform Demo", imgOriginal );
	        waitKey(1);
	    }
	    return false;
	}

    void robotMove(float lin_v, float ang_v){
        kobuki.setBaseControl(lin_v, ang_v);
        return;
    }

    Mat getRoadSign() {
        return roadSign;
    }
    /*
    void resetOdometry(){
        kobuki.resetOdometry();
        return;
    }
    
    char* getPosition(){
        std::cout<<"x: "<<px<<", "<<"y: "<<py<<", heading: "<<kobuki.getHeading()<<std::endl;
        char str[32];
        char* result;
        //sprintf(result, "x: %f, y: %f, heading: %f", px, py, kobuki.getHeading());
        sprintf(str, "hello world\0");
        strcpy(result, str);
        return result;
    }

    kobuki::CoreSensors::Data getCoreData() {
        coreData = kobuki.getCoreSensorData();
        return coreData;
    }

    void updatePose(){
        ecl::Pose2D<double> pose_update;
        ecl::linear_algebra::Vector3d pose_update_rates;
        kobuki.updateOdometry(pose_update, pose_update_rates);
        pose *= pose_update;
    }

    void getPose(){
        std::cout << "Encoder: current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
        std::cout<<"Gyro: Heading: "<<kobuki.getHeading()<<std::endl;
    }*/

    void gotoX(double x) {
        kobuki.updateOdometry(pose_update, pose_update_rates);
        pose *= pose_update;

        double startX = pose.x();
        double endX = startX + x;

        cout << "poseX: " << pose.x() << endl;
        cout << "startX: " << startX << endl;
        cout << "endX: " << endX << endl;

        if(x == 0) return;
        px += x;
        nowHead = kobuki.getHeading();
        double range = 0.05;
        double angV = 0.5;
        double linear_velocity = 0.1;
        double linear_t;
        int int_linear_t;

        robotMove(0.0, 0.0);
        nano_sleep(1000);

        if(x > 0) {
            if(nowHead > 0) {
                robotMove(0.0, (-1)*angV);
            }
            else {
                robotMove(0.0, angV);
            }
            nowHead = kobuki.getHeading();
            while(nowHead > range or nowHead < -range){
                nano_sleep(1000);
                nowHead = kobuki.getHeading();
                if(nowHead > 0) {
                    robotMove(0.0, (-1)*angV);
                }
                else {
                    robotMove(0.0, angV);
                }
            }
            robotMove(0.0, 0.0);
            nano_sleep(1e5);
            robotMove(linear_velocity, 0.0);
            while(pose.x() < endX) {
            	if(getSign(cap)) cout << "Yeah fantastic baby" << endl;
            	else cout << "Fuck you all" << endl;
            	kobuki.updateOdometry(pose_update, pose_update_rates);
            	pose *= pose_update;
            	nano_sleep(1000);
            }
            /*
            linear_t = x / linear_velocity;
            robotMove(linear_velocity, 0.0);
            int_linear_t = int(linear_t);
            sleep(int_linear_t);
            nano_sleep((linear_t - int_linear_t) * 1e9);
            */
        }

        if(x < 0) {
            if (nowHead > 0){
                robotMove(0.0, angV);
            }
            else {
                robotMove(0.0, -angV);  
            }
            nowHead = kobuki.getHeading();
            while((nowHead < 3) and (nowHead > -3)){
                nano_sleep(1000);
                nowHead = kobuki.getHeading();
                if (nowHead > 0){
                    robotMove(0.0, angV);
                }
                else {
                    robotMove(0.0, -angV);  
                }
            }
            robotMove(0.0, 0.0);
            nano_sleep(1e5);
            robotMove(linear_velocity, 0.0);
            while(pose.x()> endX) {
            	kobuki.updateOdometry(pose_update, pose_update_rates);
            	pose *= pose_update;
            	nano_sleep(1000);
            }
            /*
            linear_t = -x / linear_velocity;
            robotMove(linear_velocity, 0.0);
            int_linear_t = int(linear_t);
            sleep(int_linear_t);
            nano_sleep((linear_t - int_linear_t) * 1e9);
            */
        }
        robotMove(0.0, 0.0);
        nano_sleep(1000);
    }

    void gotoY(double y) {
        kobuki.updateOdometry(pose_update, pose_update_rates);
        pose *= pose_update;

        double startY = pose.y();
        double endY = startY + y;

        if(y == 0) return;
        py += y;
        nowHead = kobuki.getHeading();
        double range = 0.05;
        double angV = 0.5;
        double linear_velocity = 0.3;
        double linear_t;
        int int_linear_t;

        robotMove(0.0, 0.0);
        nano_sleep(1000);

        if (y > 0) {
            if(nowHead < ecl::pi/2 and nowHead > -ecl::pi/2) {
                robotMove(0.0, angV);
            }
            else {
                robotMove(0.0, -angV);
            }
            while(nowHead < ecl::pi/2 - range or nowHead > ecl::pi/2 + range){
                nano_sleep(1000);
                nowHead = kobuki.getHeading();
                if(nowHead < ecl::pi/2 and nowHead > -ecl::pi/2) {
                    robotMove(0.0, angV);
                }
                else {
                    robotMove(0.0, -angV);
                }
            }
            robotMove(0.0, 0.0);
            nano_sleep(1e5);
            robotMove(linear_velocity, 0.0);
            while(pose.y() < endY) {
            	if(getSign(cap)) cout << "Yeah fantastic baby" << endl;
            	else cout << "Fuck you all" << endl;
            	kobuki.updateOdometry(pose_update, pose_update_rates);
            	pose *= pose_update;
            	nano_sleep(1000);
            }
            //linear_t = y / linear_velocity;
        }
        if (y < 0) {
            if(nowHead < ecl::pi/2 and nowHead > -ecl::pi/2) {
                robotMove(0.0, -angV);
            }
            else {
                robotMove(0.0, angV);
            }
            while(nowHead > -(ecl::pi/2 - range) or nowHead < -(ecl::pi/2 + range)){
                nano_sleep(1000);
                nowHead = kobuki.getHeading();
                if(nowHead < ecl::pi/2 and nowHead > -ecl::pi/2) {
                    robotMove(0.0, -angV);
                }
                else {
                    robotMove(0.0, angV);
                }
            }
            robotMove(0.0, 0.0);
            nano_sleep(1e5);
            robotMove(linear_velocity, 0.0);
            while(pose.y() > endY) {
            	if(getSign(cap)) cout << "Yeah fantastic baby" << endl;
            	else cout << "Fuck you all" << endl;
            	kobuki.updateOdometry(pose_update, pose_update_rates);
            	pose *= pose_update;
            	nano_sleep(1000);
            }
            //linear_t = -y / linear_velocity;
        }
        /*robotMove(0.0, 0.0);
        nano_sleep(1e5);
        robotMove(linear_velocity, 0.0);
        int_linear_t = int(linear_t);
        sleep(int_linear_t);
        nano_sleep((linear_t - int_linear_t) * 1e9);*/
        robotMove(0.0, 0.0);
        nano_sleep(1000);
    }

    void goTo(double x, double y){
    	gotoX(x);
        gotoY(y);
    }

    void goToWO(double x, double y){
        bool Xdirect = true;
        double diff = 0.1;
        int x_sign = 1, y_sign = 1;

        if(x < 0) {
            x_sign = -1;
            x *= x_sign;
        }
        if(y < 0) {
            y_sign = -1;
            y *= y_sign;
        }

        while(x + y > 0.01) {
            printf("x = %f, y = %f\n", x, y);
            depth = Astra.getDepth();
            obstacle = Astra.DetectBarrier(100, depth);
            printf("%s\n", obstacle ? "blocked" : "clear");
            if(!obstacle) {
                Xdirect = !Xdirect;
            }
            if(x <= 0.01) {
                Xdirect = false;
            }
            if(y <= 0.01) {
                Xdirect = true;
            }
            printf("%s\n", Xdirect ? "goX" : "notGoX");
            if(Xdirect) {
                gotoX(x_sign*diff);
                x -= diff;
            }
            else {
                gotoY(y_sign*diff);
                y -= diff;
            }
        }
    }

    void resetOrigin(double x, double y) {
        px -= x;
        py -= y;
        return;
    }

    void Return(){
        goToWO(-px, -py);
        px = 0.0;
        py = 0.0;
        return;
    }

    void run20() {
        speedLin = 0.06;
        speedAng = 0.0;
        robotMove(speedLin, speedAng);
    }

    void run60() {
        speedLin = 0.2;
        speedAng = 0.0;
        robotMove(speedLin, speedAng);
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
    }

    void stop() {
        robotMove(0.0, 0.0);
        sleep(3);
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
    VideoCapture cap;
};
/*
bool shutdown_req = false;
void signalHandler(int signum) {
    shutdown_req = true;
}


void error( char *msg ) {
    perror(  msg );
    exit(1);
}

int func( int a ) {
    return 2 * a;
}

void sendData(int sockfd, char* s){
    int n;
    char buffer[32];
    strcpy(buffer, s);
    if ( (n = write( sockfd, buffer, strlen(buffer) ) ) < 0 )
        error( const_cast<char *>( "ERROR writing to socket") );
}

char* getData(int sockfd){
    char* str = (char*) malloc(32);
    int n;
    if ( (n = read(sockfd,str,31) ) < 0 )
        error( const_cast<char *>( "ERROR reading from socket") );
    str[n] = '\0';
    return str;
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
}

int main(int argc, char *argv[]) {
    int sockfd, newsockfd, portno = 5200, clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;
    int cmd, data;
    float x[100], y[100];
    int numObj = 0;
    KobukiManager kobuki_manager;
    ecl::Pose2D<double> pose;
    ecl::Sleep sleep(1);
    
    printf( "using port #%d\n", portno );

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error( const_cast<char *>("ERROR opening socket") );
    bzero((char *) &serv_addr, sizeof(serv_addr));

    signal(SIGINT, signalHandler);

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons( portno );
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
        error( const_cast<char *>( "ERROR on binding" ) );
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    bool ifquit = false;
    bool ifclieshut = false;

    while ( !ifquit ) {
        printf( "waiting for new client...\n" );
        if ( ( newsockfd = accept( sockfd, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen) ) < 0 )
            error( const_cast<char *>("ERROR on accept") );
        printf( "opened new communication with client\n" );
        while ( !ifclieshut ) {
            cmd = getInt( newsockfd );
            printf("CMD: %d\n\r", cmd);
            double posX, posY;
            switch(cmd){
                case 0:
                    RobotMoveForward(&kobuki_manager);
                    break;
                case 1:
                    float av, lv;
                    lv = getFloat(newsockfd);
                    std::cout<<"LV: "<<lv<<std::endl;
                    av = getFloat(newsockfd);
                    std::cout<<"AV: "<<av<<std::endl;
                    RobotMove(&kobuki_manager, lv, av);
                    break;
                case 2: 
                    RobotStop(&kobuki_manager);
                    break;
                case 3:
                    RobotTurn(&kobuki_manager);
                    break;
                case 4:
                    av = getFloat(newsockfd);
                    std::cout<<"AV: "<<av<<std::endl;
                    RobotTurn(&kobuki_manager, av);
                    break;
                case 5:
                    posX = getFloat(newsockfd);
                    std::cout << "X cooridinate to move to: " << posX << std::endl;
                    posY = getFloat(newsockfd);
                    std::cout << "Y cooridinate to move to: " << posY << std::endl;
                    kobuki_manager.goTo(posX, posY);
                    break;
                case 6:
                    kobuki_manager.Return();
                    break;
                case 7:
                    posX = getFloat(newsockfd);
                    std::cout << "New X cooridinate: " << posX << std::endl;
                    posY = getFloat(newsockfd);
                    std::cout << "New Y cooridinate: " << posY << std::endl;
                    kobuki_manager.resetOrigin(posX, posY);
                    break;
                case 8:
                    std::cout<<"Dumping sensor data:"<<std::endl;
                    DumpPose(&kobuki_manager, newsockfd);
                    break;
                case -2:
                    ifclieshut = true;
                    ifquit = true;
                    break;          
                default: 
                    break;  
            }
        }
        close( newsockfd );

     }
     return 0;
}*/