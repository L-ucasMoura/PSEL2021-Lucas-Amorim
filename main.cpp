#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <cmath>

QVector<bool> dominio = {false, false, false};


struct detect{
    SSL_DetectionBall ball;
    SSL_DetectionRobot activePlayer;
    SSL_DetectionRobot firstOption;
    SSL_DetectionRobot secondOption;
};

struct measures{
    float ballAngle, targetAngle, ballDistance, targetDistance, playerOrientation, initialAngle;

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
        m.ballAngle = -m.ballAngle;
    }

    m.targetDistance = hypot((V.firstOption.x() - V.activePlayer.x()), (V.firstOption.y() - V.activePlayer.y()));
    m.targetAngle = acos((V.firstOption.x() - V.activePlayer.x())/m.targetDistance);

    if(V.firstOption.y() < V.activePlayer.y()){
        m.targetAngle = -m.targetAngle;
    }


    m.playerOrientation = V.activePlayer.orientation();



    return m;
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


float align(float target, float playerAngle){
    float vel = 4;
    float vw;

    if(fabs(playerAngle - target) > 0.2){
        if(playerAngle > target){
            if(fabs(playerAngle - target) >= M_PI){
                vw = vel;
            }
            else{
                vw = -vel;
            }
        }
        else{
            if(fabs(playerAngle - target) > M_PI){
                vw = -vel;
            }
            else{
                vw = vel;
            }
        }
    }
    else{
        vw = 0;
    }


    return vw;
}


commands go_to_ball(measures m){
    commands go;
    float functionVel = 1 + 0.5*(m.ballDistance/1000);
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


std::pair<float, float> get_position(detect p, int n, float y_position){


    float goal_ballDistance = 5000 - p.ball.x();

    float field_division = goal_ballDistance/3;
    float x_position = p.ball.x()+(n*field_division);

    float d = hypot((x_position - p.activePlayer.x()), (y_position - p.activePlayer.y()));

    float angle = acos(( x_position - p.activePlayer.x() )/d);
    if(y_position < p.activePlayer.y()){angle = -angle;}

    //std::cout << angle*57.2960 << '\n';


    return std::make_pair(angle, d);
}


commands x(float angle, float distance, float orientation){
    commands c;
    if(distance > 300){
        c.vw = align(angle, orientation);
        c.vx = 1;
    }
    else{
        c.vw = align(angle, orientation);
        c.vx = 0;
     }
    return c;
}


commands catcher(jogador armador){


    armador.param.ballAngle = line_and_AngleMaker(armador.det).ballAngle;
    armador.param.ballDistance = line_and_AngleMaker(armador.det).ballDistance;
    armador.param.targetAngle = line_and_AngleMaker(armador.det).targetAngle;
    armador.param.targetDistance = line_and_AngleMaker(armador.det).targetDistance;
    armador.param.playerOrientation = line_and_AngleMaker(armador.det).playerOrientation;

    //nenhum dos jogadores tem domínio da bola, então cabe ao armador ir buscá-la p/ iniciar a jogada
    if(dominio[1] == false && dominio[2] == false){
        //bola não dominada
        if(armador.param.ballDistance > 150){
            armador.com = go_to_ball(armador.param);
            dominio[0]= false;
        }

        //bola dominada
        else{
            dominio[0] = true;

            //rotina de passe
            if(fabs(armador.det.activePlayer.x() - armador.det.firstOption.x()) < 1900 || armador.param.targetDistance < 1500){

                //alinhar passe
                armador.com.vw = align(armador.param.targetAngle, armador.det.activePlayer.orientation());
                armador.com.vx = 0;

                //efetuar passe à frente do alvo
                if(fabs(armador.param.playerOrientation - armador.param.targetAngle) < 0.4){
                    armador.com.kick = 2 + 0.5*(armador.param.targetDistance/1000);
                    dominio[1] = true;
                    dominio[0] = false;
                }
            }

            //correr na direção do gol
            else{

                //alinhar com área adversária
                armador.com.vw = align(0, armador.param.playerOrientation);
                armador.com.vx = 0;

                // correr em frente
                if(fabs(armador.param.playerOrientation) < 0.2){
                    armador.com.vx = 0.5;
                }
            }
        }
    }
    //comportamento após efetuar o passe
    else{
        armador.com.vw = 0; armador.com.vx = 0; armador.com.kick = 0;
    }

    return armador.com;

}


commands mid_field(jogador meia){
    
    meia.param.ballAngle = line_and_AngleMaker(meia.det).ballAngle;
    meia.param.ballDistance = line_and_AngleMaker(meia.det).ballDistance;
    meia.param.targetAngle = line_and_AngleMaker(meia.det).targetAngle;
    meia.param.targetDistance = line_and_AngleMaker(meia.det).targetDistance;
    meia.param.playerOrientation = line_and_AngleMaker(meia.det).playerOrientation;

    float distance;
    float y = -1500;

    if(meia.det.ball.y() != 0){y= meia.det.ball.y()/fabs(meia.det.ball.y()) *-1500;}

    // def posição inicial
    meia.param.initialAngle = get_position(meia.det, 1, y).first;
    distance = get_position(meia.det, 1, y).second;

    //Qnd estiver sem a posse da bola, o meia vai se posicionar em uma das laterais pra receber o passe
    if(dominio[0] == false && dominio[1] == false){

        // ir para posição inicial
        if(distance > 300){
            meia.com.vw = align(meia.param.initialAngle, meia.param.playerOrientation);
            meia.com.vx = 1.5;
        }
        else{
            meia.com.vw = align(meia.param.ballAngle, meia.param.playerOrientation);
            meia.com.vx = 0;
         }
    }

    else{
        //Qnd o domínio for atribuido ao meia
        if(dominio[1] == true && meia.det.activePlayer.x() < meia.det.ball.x() && meia.param.ballDistance < 1500){
            meia.com = go_to_ball(meia.param);
        }
    }

    //meia.com = go_to_ball(meia.param);
    
    return meia.com;

}


commands attacker(jogador atacante){

    
    atacante.param.ballAngle = line_and_AngleMaker(atacante.det).ballAngle;
    atacante.param.ballDistance = line_and_AngleMaker(atacante.det).ballDistance;
    atacante.param.targetAngle = line_and_AngleMaker(atacante.det).targetAngle;
    atacante.param.targetDistance = line_and_AngleMaker(atacante.det).targetDistance;
    atacante.param.playerOrientation = line_and_AngleMaker(atacante.det).playerOrientation;

    float k = 1500;
    if(atacante.det.ball.y() != 0){k= atacante.det.ball.y()/fabs(atacante.det.ball.y()) *1500;}

    atacante.param.initialAngle = get_position(atacante.det, 2, k).first;
    float distance = get_position(atacante.det, 2, k).second;


    atacante.com = x(atacante.param.initialAngle, distance, atacante.det.activePlayer.orientation());

    return atacante.com;
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



        armador.det.activePlayer = player_a; armador.det.firstOption = player_b; armador.det.secondOption = player_c;
        meia.det.activePlayer = player_b; meia.det.firstOption = player_c; armador.det.secondOption = player_a;
        atacante.det.activePlayer = player_c; atacante.det.firstOption = player_a; atacante.det.secondOption = player_b;
        armador.det.ball = meia.det.ball = atacante.det.ball = ball;


        if(fabs(ball.x()) > 5000 || fabs(ball.y()) > 3250){
            dominio = {false, false, false};
        }

        // Process vision and actuator commands
        vision->processNetworkDatagrams();


//        else{
//            if(geometry.first < 150 && fabs(geometry.second - playerAngle) < 0.4){
//                result = passar(player0.orientation(), passAngle, player0.y(), player1.y(), playerDistance);
//            }
//            else{
//                result = dominio(playerAngle, BallAngle);
//            }
//        }


        std::cout << dominio[0]<< dominio[1] << dominio[2] << '\n';

        armador.com = catcher(armador);
        meia.com = mid_field(meia);
        atacante.com = attacker(atacante);


        actuator->sendCommand(false, 0, armador.com.vx, 0, armador.com.vw, armador.com.dibble, armador.com.kick);
        actuator->sendCommand(false, 1, meia.com.vx, 0, meia.com.vw, meia.com.dibble);
        actuator->sendCommand(false, 2, atacante.com.vx, 0, atacante.com.vw);



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





//commands passar(float activeAngle, float passiveAngle, float yactive, float ypassive, float playersDistance){
//   commands p;
//   float vel = 3;
//   p.vx = 0.5;

//    if(fabs(passiveAngle - activeAngle) > 0.3){

//        p.dibble = true;

//        if(ypassive >= yactive){
//            p.vw = vel;
//        }
//        else{
//            p.vw = -vel;
//        }
//    }
//    else{
//        p.vw = 0;
//        p.kick = 1 + 0.8*playersDistance/1000;
//    }
//    return p;
//}
