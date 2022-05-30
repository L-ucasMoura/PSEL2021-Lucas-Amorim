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

Struct passar(float activeAngle, float passiveAngle, float yactive, float ypassive, float playersDistance){
   Struct p;
   float vel = 3;
   p.vx = 0.5;

    if(fabs(passiveAngle - activeAngle) > 0.3){

        p.dibble = true;

        if(ypassive >= yactive){
            p.vw = vel;
        }
        else{
            p.vw = -vel;
        }
    }
    else{
        p.vw = 0;
        p.kick = 1 + 0.8*playersDistance/1000;
    }
    return p;
}


Struct dominio(float playerAngle, float ballAngle){
    Struct d;
    d.dibble = true;
    float vel = 3;

    if(fabs(ballAngle - playerAngle) > 0.2){
        d.vx = 0;
        if(ballAngle > playerAngle){
            if(fabs(ballAngle - playerAngle) > M_PI){
                d.vw = -vel;
            }
            else{
                d.vw = vel;
            }
        }
        else{
            if(fabs(playerAngle - ballAngle) > M_PI){
                d.vw = vel;
            }
            else{
                d.vw = -vel;
            }
        }
    }
    else{
        d.vw = 0;
        d.vx = 3;
    }
        return d;
}


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
        SSL_DetectionRobot player2 = vision->getLastRobotDetection(false, 1);


        // Process vision and actuator commands
        vision->processNetworkDatagrams();

        float p_y = (player2.y() - player.y());
        float p_x = (player2.x() - player.x());
        float playerDistance = hypot(p_x, p_y);
        float passAngle = acos(p_x/playerDistance);


        // player/ball measures
        float desl_y = (ball.y() - player.y());             //resultado em pixel
        float desl_x = (ball.x() - player.x());             //resultado em pixel
        float ballDistance = hypot(desl_x, desl_y);         //resultado em pixels
        float BallAngle = acos(desl_x/ballDistance);        //resultado em rad
        trave1 = acos((4502 - player.x()) / hypot((4502 - player.x()),(494 - player.y())));     //resultado em rad
        trave2 = acos((4502 - player.x()) / hypot((4502 - player.x()),(-494 - player.y())));    //resultado em rad


        if(player2.y() < player.y()){
            passAngle = -passAngle;
            //passAngle = 2*M_PI - passAngle;
        }
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

//        //Ir até robo
//        if(ballDistance < 160){
//            result = direct_to_target(passAngle, playerAngle);
//            result.vx = 1;

//            //passe
//            if(playerDistance < 3000){
//                result.dibble = false;
//                result.kick = (playerDistance/1000) +1;
//            }
//        }


//        //Ir até o gol
//        if(ballDistance < 160){
//            result = direct_to_target(kickAngle, playerAngle);
//            result.vx = 2;

//            //chute
//            if(player.x() > 3000){
//                result.dibble = false;
//                result.kick = 5;
//            }
//        }


        if(ballDistance > 300){
            result = direct_to_target(BallAngle, playerAngle);
            result.vx = 1 + 0.5*(ballDistance/1000);
        }
        else{
            if(ballDistance < 150 && fabs(BallAngle - playerAngle) < 0.4){
                result = passar(player.orientation(), passAngle, player.y(), player2.y(), playerDistance);
            }
            else{
                result = dominio(playerAngle, BallAngle);
            }
        }


        std::cout << passAngle*57.296 << '\n';
        std::cout << player.orientation()*57.296 << '\n';




        actuator->sendCommand(false, robot, result.vx, result.vy, result.vw, result.dibble, result.kick);



        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }



    return a.exec();
}


