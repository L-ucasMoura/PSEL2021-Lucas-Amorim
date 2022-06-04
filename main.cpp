#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <cmath>

struct detect{
    SSL_DetectionBall ball;
    SSL_DetectionRobot player;
};

struct jogador
{
    int robot;
    float vx, vw, kick;
    bool dribble;

    detect det;
};



struct data {
    float vx, vy, vw, kick = 0;
    bool dibble = false;
};

typedef struct data Struct;

std::pair <float, float> line_and_AngleMaker(detect V){


    float Distance = hypot((V.ball.x() - V.player.x()), (V.ball.y() - V.player.y()));
    float Angle = acos((V.ball.x() - V.player.x())/Distance);

    if(V.ball.y() < V.player.y()){
        Angle = 2*M_PI - Angle;
    }

    return std::make_pair (Distance, Angle);
}


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


    bool inicio = true;
    //int armador, atacante, meia;
    float playerAngle, trave1, trave2, kickAngle;
    Struct result;

    jogador armador, meia, atacante;
    detect x;


    while(true) {


        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();
        SSL_DetectionBall ball = vision->getLastBallDetection();
        SSL_DetectionRobot player0 = vision->getLastRobotDetection(false, 0);
        SSL_DetectionRobot player1 = vision->getLastRobotDetection(false, 1);
        SSL_DetectionRobot player2 = vision->getLastRobotDetection(false, 2);


        armador.det.ball = ball;
        armador.det.player = player0;

//        distances.d1 = line_and_AngleMaker(x).first;
//        distances.d2 = line_and_AngleMaker(x).first;
//        distances.d3 = line_and_AngleMaker(x).first;

//        if(inicio && distances.d1 > 0 && distances.d2 > 0 && distances.d3 > 0){
//            players = elect(distances);
//            inicio = false;
//            std::cout << players.d1 << '\n';
//            std::cout << players.d2 << '\n';
//            std::cout << players.d3 << '\n';
//        }



        // Process vision and actuator commands
        vision->processNetworkDatagrams();


        playerAngle = player0.orientation();
        if(playerAngle < 0){
            playerAngle = 2*M_PI + playerAngle;
        }
//        SSL_DetectionRobot active = player0;


        std::pair<float, float> geometry = line_and_AngleMaker(armador.det);



        // geometry.first = distance, geometry.second = angle
        if(geometry.first > 300){
            result = direct_to_target(geometry.second, playerAngle);
            result.vx = 1 + 0.5*(geometry.first/1000);
        }
        else{
            result = dominio(playerAngle, geometry.second);
            }

//        else{
//            if(geometry.first < 150 && fabs(geometry.second - playerAngle) < 0.4){
//                result = passar(player0.orientation(), passAngle, player0.y(), player1.y(), playerDistance);
//            }
//            else{
//                result = dominio(playerAngle, BallAngle);
//            }
//        }







        actuator->sendCommand(false, 0, result.vx, result.vy, result.vw, result.dibble, result.kick);



        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }



    return a.exec();
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


//float p_y = (player1.y() - player0.y());
//float p_x = (player1.x() - player0.x());
//float playerDistance = hypot(p_x, p_y);
//float passAngle = acos(p_x/playerDistance);


// // player/ball measures
//float desl_y = (ball.y() - player0.y());             //resultado em pixel
//float desl_x = (ball.x() - player0.x());             //resultado em pixel
//float ballDistance = hypot(desl_x, desl_y);         //resultado em pixels
//float BallAngle = acos(desl_x/ballDistance);        //resultado em rad
//trave1 = acos((4502 - player0.x()) / hypot((4502 - player0.x()),(494 - player0.y())));     //resultado em rad
//trave2 = acos((4502 - player0.x()) / hypot((4502 - player0.x()),(-494 - player0.y())));    //resultado em rad


//if(player1.y() < player0.y()){
//    passAngle = -passAngle;
//    //passAngle = 2*M_PI - passAngle;
//}
// // ângulo vai de 0 a 2pi ao invés de ir de -pi à +pi
//if(ball.y() < player0.y()){
//    BallAngle = 2*M_PI - BallAngle;
//}
//playerAngle = player0.orientation();
//if(playerAngle < 0){
//    playerAngle = 2*M_PI + playerAngle;
//}

//if(player0.y() > -494){
//    trave2 = -trave2;
//    if(ball.y() > 494){
//        trave1 = -trave1;
//    }
//}
//kickAngle = (trave1 + trave2)/2;
//if(kickAngle < 0){
//    kickAngle = 2*M_PI + kickAngle;
//}


//kkk elect(kkk e){
//    using namespace std;
//    // Declaring vector of pairs
//        vector< pair <int,float> > vect;

//        // Initializing 1st and 2nd element of
//        // pairs with array values
//        int players[] = {0, 1, 2};
//        float distances[] = {e.d1, e.d2, e.d3};

//        // Entering values in vector of pairs
//        for (int i=0; i<3; i++)
//            vect.push_back( make_pair(distances[i], players[i]));


//        // Using simple sort() function to sort
//           sort(vect.begin(), vect.end());


//        kkk result;
//        result.d1 = vect[0].second;
//        result.d2 = vect[1].second;
//        result.d3 = vect[2].second;

//        return result;
//}
