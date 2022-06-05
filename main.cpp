#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <cmath>

struct detect{
    SSL_DetectionBall ball;
    SSL_DetectionRobot activePlayer;
    SSL_DetectionRobot targetPlayer;
};

struct measures{
    float ballAngle, targetAngle, ballDistance, targetDistance, playerOrientation;

};

struct commands{
    float vx, vw, kick;
    bool dibble = false;
};

struct jogador
{
    int robot;

    commands com;

    measures param;

    detect det;
};

struct task_Atribution{
    float armador, meia, atacante;
};




measures line_and_AngleMaker(detect V){

    measures m;

    m.ballDistance = hypot((V.ball.x() - V.activePlayer.x()), (V.ball.y() - V.activePlayer.y()));
    m.ballAngle = acos((V.ball.x() - V.activePlayer.x())/m.ballDistance);

    if(V.ball.y() < V.activePlayer.y()){
        m.ballAngle = 2*M_PI - m.ballAngle;
    }

    m.targetDistance = hypot((V.targetPlayer.x() - V.activePlayer.x()), (V.targetPlayer.y() - V.activePlayer.y()));
    m.targetAngle = acos((V.targetPlayer.x() - V.activePlayer.x())/m.targetDistance);

    if(V.targetPlayer.y() < V.activePlayer.y()){
        m.targetAngle = 2*M_PI - m.targetAngle;
    }

    m.playerOrientation = V.activePlayer.orientation();
    if(m.playerOrientation < 0){
        m.playerOrientation = 2*M_PI + m.playerOrientation;
    }


    return m;
}

float velx(measures m){
    float v (m.ballDistance);
    return v;
}

task_Atribution elect(task_Atribution t){
    using namespace std;
    // Declaring vector of pairs
    vector< pair <int,float> > vect;

    // Initializing 1st and 2nd element of
    // pairs with array values
    int players[] = {0, 1, 2};
    float distances[] = {t.armador, t.meia, t.atacante};

    // Entering values in vector of pairs
    for (int i=0; i<3; i++)
        vect.push_back( make_pair(distances[i], players[i]));


    // Using simple sort() function to sort
       sort(vect.begin(), vect.end());


    task_Atribution r;
    r.armador = vect[0].second;
    r.meia = vect[1].second;
    r.atacante = vect[2].second;

    return r;
}


commands passar(float activeAngle, float passiveAngle, float yactive, float ypassive, float playersDistance){
   commands p;
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


commands dominio(float playerAngle, float ballAngle){
    commands d;
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


float align(float target, float playerAngle)
{
    float vw;
    float vel = 4;


    if(fabs(target - playerAngle) > 0.2){
        if(target > playerAngle){
            if(fabs(target - playerAngle) > M_PI){
                vw = -vel;
            }
            else{
                vw = vel;
            }
        }
        else{
            if(fabs(playerAngle - target) > M_PI){
                vw = vel;
            }
            else{
                vw = -vel;
            }
        }
    }
    else{
        vw = 0;
    }

    return vw;
}


commands go_to_target(measures m){
    commands go;
    float functionVel = (m.ballDistance/1000);
    // Buscar a bola
    if(m.ballDistance > 300){
        go.dibble = false;
        go.vx = functionVel;
        go.vw = align(m.ballAngle, m.playerOrientation);
    }
    else{
    // Dominar a bola
        go.dibble = true;
        if(fabs(m.ballAngle - m.playerOrientation) > 0.2){
            go.vx = 0;
            go.vw = align(m.ballAngle, m.playerOrientation);
        }
        else{
            go.vx = 2;
            go.vw = 0;
        }
    }

     return go;
}


commands catcher(jogador armador){


//    float playerAngle = armador.det.activePlayer.orientation();
//    if(playerAngle < 0){
//        playerAngle = 2*M_PI + playerAngle;
//    }

    armador.param.ballAngle = line_and_AngleMaker(armador.det).ballAngle;
    armador.param.ballDistance = line_and_AngleMaker(armador.det).ballDistance;
    armador.param.targetAngle = line_and_AngleMaker(armador.det).targetAngle;
    armador.param.targetDistance = line_and_AngleMaker(armador.det).targetDistance;
    armador.param.playerOrientation = line_and_AngleMaker(armador.det).playerOrientation;

    std::cout << armador.param.targetAngle * 57.296 << '\n';
    std::cout << armador.param.ballAngle * 57.296 << '\n';
    std::cout << armador.param.playerOrientation * 57.296 << '\n';

    return armador.com;

}


void mid_field(){

}

void attacker(){

}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    jogador armador, meia, atacante;


    while(true) {


        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        SSL_DetectionBall ball = vision->getLastBallDetection();
        SSL_DetectionRobot player_a = vision->getLastRobotDetection(false, 0);
        SSL_DetectionRobot player_b = vision->getLastRobotDetection(false, 1);
        SSL_DetectionRobot player_c = vision->getLastRobotDetection(false, 2);

        armador.det.activePlayer = player_a; armador.det.targetPlayer = player_b;

        meia.det.activePlayer = player_b; meia.det.targetPlayer = player_c;

        atacante.det.activePlayer = player_c;

        armador.det.ball = meia.det.ball = atacante.det.ball = ball;




        // Process vision and actuator commands
        vision->processNetworkDatagrams();




//        std::pair<float, float> geometry = line_and_AngleMaker(armador.det);


//        // geometry.first = distance, geometry.second = angle
//        if(geometry.first > 300){
//            result = direct_to_target(geometry.second, playerAngle);
//            result.vx = 1 + 0.5*(geometry.first/1000);
//        }
//        else{
//            result = dominio(playerAngle, geometry.second);
//            }

//        else{
//            if(geometry.first < 150 && fabs(geometry.second - playerAngle) < 0.4){
//                result = passar(player0.orientation(), passAngle, player0.y(), player1.y(), playerDistance);
//            }
//            else{
//                result = dominio(playerAngle, BallAngle);
//            }
//        }




        armador.com = catcher(armador);


        //actuator->sendCommand(false, 0, armador.com.vx, 0, armador.com.vw, 0, 0);



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




//float direct_to_target(measure m, float target, float playerAngle)
//{
//    float vw;
//    float vel = 4;


//    if(fabs(target - playerAngle) > 0.2){
//        if(target > playerAngle){
//            if(fabs(target - playerAngle) > M_PI){
//                vw = -vel;
//            }
//            else{
//                vw = vel;
//            }
//        }
//        else{
//            if(fabs(playerAngle - target) > M_PI){
//                vw = vel;
//            }
//            else{
//                vw = -vel;
//            }
//        }
//    }
//    else{
//        vw = 0;
//    }
//        return vw;
//}
