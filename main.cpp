#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <cmath>
#include <string>





std::pair<float, float> toTheball(float lineAngle, float robotAngle, float robot_x, float ball_x){

    if(robot_x < ball_x){

        if(fabs(lineAngle - robotAngle) > 0.1){
                if(lineAngle < 0){
                    return std::make_pair(0.0, -1.5);
                }
                else{
                    return  std::make_pair(0.0, 1.5);
                }
        }
        else{
            return std::make_pair(2.0, 0.0);
        }
    }
    else{

        if(fabs(robotAngle) - 3.1415 > 0.1){
            if(robotAngle < 0){
                return std::make_pair(0.0, -1.5);
            }
            else{
                return  std::make_pair(0.0, 1.5);
            }

        }
        else{
            return  std::make_pair(2.0, 0.0);
        }

    }
}


int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);
    // Desired frequency
    int desiredFrequency = 60;

    int robot = 0;
    float kick;


    while(true) {

        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();
        SSL_DetectionBall ball = vision->getLastBallDetection();
        SSL_DetectionRobot robotDet = vision->getLastRobotDetection(false, robot);
        //robotDet.orientation();



        // Process vision and actuator commands
        vision->processNetworkDatagrams();

        float desl_y = robotDet.y() - ball.y();
        float desl_x = robotDet.x() - ball.x();
        float ballDistance = hypot(desl_x, desl_y);
        float robot_BallAngle = atan(desl_y/desl_x);
        float graus = robotDet.orientation() * 57.2958;

        std::cout << graus << '\n';

        if(ballDistance < 230){
             kick = 5.0;
        }
        else{
            kick = 0.0;
        }


        std::pair<float, float> answer = toTheball(robot_BallAngle, robotDet.orientation(), robotDet.x(), ball.x());


        actuator->sendCommand(false, robot, answer.first, 0.0, answer.second, false, kick);

        //actuator->sendCommand(false, 0, 0, 0, 0.5);




        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }



    return a.exec();
}
