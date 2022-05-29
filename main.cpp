#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <cmath>


struct data {
    float vx, vy, vw, kick = 0;
    bool dibble = false;
};

typedef struct data Struct;


Struct direct_to_target(float target, float playerAngle)
{
    Struct a;
    float vel = 4;




    if(fabs(target - playerAngle) > 0.2){
        if(target > playerAngle){
            if(fabs(target - playerAngle) > M_PI){
                a.vw = -vel;
            }
            else{
                a.vw = vel;
            }
        }
        else{
            if(fabs(playerAngle - target) > M_PI){
                a.vw = vel;
            }
            else{
                a.vw = -vel;
            }
        }
    }
    else{
        a.vw = 0;
    }
        return a;
}



int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    int robot = 0;
    float playerAngle, trave1, trave2, kickAngle;
    Struct result;


    while(true) {

        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();
        SSL_DetectionBall ball = vision->getLastBallDetection();
        SSL_DetectionRobot player = vision->getLastRobotDetection(false, robot);
        //robotDet.orientation();



        // Process vision and actuator commands
        vision->processNetworkDatagrams();


        float desl_y = (ball.y() - player.y());             //resultado em pixel
        float desl_x = (ball.x() - player.x());             //resultado em pixel
        float ballDistance = hypot(desl_x, desl_y);         //resultado em pixels
        float BallAngle = acos(desl_x/ballDistance); //resultado em rad
        trave1 = acos((4502 - player.x()) / hypot((4502 - player.x()),(494 - player.y())));     //resultado em rad
        trave2 = acos((4502 - player.x()) / hypot((4502 - player.x()),(-494 - player.y())));    //resultado em rad


        // ângulo vai de 0 a 2pi ao invés de ir de -pi à +pi
        if(ball.y() < player.y()){
            BallAngle = 2*M_PI - BallAngle;
        }
        playerAngle = player.orientation();
        if(playerAngle < 0){
            playerAngle = 2*M_PI + playerAngle;
        }

        if(player.y() > -494){
            trave2 = -trave2;
            if(ball.y() > 494){
                trave1 = -trave1;
            }
        }
        kickAngle = (trave1 + trave2)/2;
        if(kickAngle < 0){
            kickAngle = 2*M_PI + kickAngle;
        }




        if(ballDistance < 160){
            result = direct_to_target(kickAngle, playerAngle);
            result.vx = 2;

            if(player.x() > 3000){
                result.dibble = false;
                result.kick = 5;
            }
        }

        else{
            result.dibble = true;
            result = direct_to_target(BallAngle, playerAngle);
            result.vx = 1 + 0.5*(ballDistance/1000);
        }

        std::cout << ballDistance << '\n';



        //actuator->sendCommand(false, robot, result.vx, result.vy, result.vw, result.dibble, result.kick);
        actuator->sendCommand(false, robot, result.vx, result.vy, result.vw, true, result.kick);



        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }



    return a.exec();
}

//Struct direct_to_ball(float BallAngle, float playerAngle, float y_player, float y_ball, float vel, float ballDistance)
//{
//    Struct a;
//    //a.vx = 1.5 + 0.4*(ballDistance/1000);
//    a.vy = 0;
//    a.vx = 1;

//    // ângulo vai de 0 a 2pi ao invés de ir de -pi à +pi
//    if(y_ball < y_player){
//        BallAngle = 2*M_PI - BallAngle;
//    }
//    if(playerAngle < 0){
//        playerAngle = 2*M_PI + playerAngle;
//    }
//    std::cout << playerAngle*57.296 << '\n';
//    std::cout << BallAngle*57.296 << '\n';
//    // Seleciona o alvo do jogador, pode ser a bola, o gol ou outro jogador
//    if(dominio == False){
//        target = BallAngle;
//    }
//    else{
//        target = kickAngle;
//    }

//    if(fabs(BallAngle - playerAngle) > 0.15){
//        if(BallAngle > playerAngle){
//            if(fabs(BallAngle - playerAngle) > M_PI){
//                a.vw = -vel;
//            }
//            else{
//                a.vw = vel;
//            }
//        }
//        else{
//            if(fabs(playerAngle - BallAngle) > M_PI){
//                a.vw = vel;
//            }
//            else{
//                a.vw = -vel;
//            }
//        }
//    }
//    else{
//        a.vw = 0;
//    }
//    return a;
//}

