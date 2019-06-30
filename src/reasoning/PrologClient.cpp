#include "PrologClient.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <json_prolog/prolog.h>
#include <unordered_map>

#include "PrologClient.h"

using namespace std;
using namespace cv;
using namespace aruco;


void PrologClient::test_prolog_query() {
    json_prolog::PrologQueryProxy bdgs = _pl.query("rdf_custom_has2(A,B,C)");
    int cont=0;
    for(auto const& bdg: bdgs){
            //cont+1;
            //cout<<std::to_string(cont)<<endl;
            cout << "A = "<< bdg["A"] << endl;

    }
}

PrologClient::Instance::Instance(const std::string &classType, uint id) : _class(classType),_id(id) {
}

const std::string &PrologClient::Instance::Get_class() const noexcept {
    return _class;
}

uint PrologClient::Instance::Get_id() const noexcept {
    return _id;
}


void PrologClient::create_action_takecards(){
    

}

void PrologClient::create_instance (const Instance& instance){
    PrologQueryProxy bdgs = _pl.query("rdf_costom_instance_from_class('"+ _NAMESPACE + instance._class+ ",_," +instance._name+ "', ObjInst)");
    }
}

/*void PrologClient::create_opponent(){
    for(i=;i<(num)=;i++){
        Instance alfons("asdfgd","fasdfasd")M
        create_instance(alfons);
    }

}*/
void PrologClient::create_nao(){
   PrologQueryProxy bdgs = _pl.query("rdf_costom_instance_from_class('" + _NAMESPACE + "Player_Nao, ObjInst')");
}

void PrologClient::create_game(int game_num){
    PrologQueryProxy bdgs = _pl.query("rdf_costom_instance_from_class('" + _NAMESPACE + "MemoryGame,_," +IntToStr(game_num)+ "', ObjInst)");
}

void PrologClient::create_ArucoToIdObjectMappings(){}

void associateTurnToPlayer(const Instance& Player_instance,const Instance& Turn_instance){
    PrologQueryProxy bdgs = _pl.query("is_in_turn('"+ _NAMESPACE + Player_instance._class+ "','"+ _NAMESPACE + Turn_instance._class+ "')");

}

struct Position{
    unsigned int _x;
    unsigned int _y;
}

struct TurnCard {
    Position _position;
}

void save_action(const TurnCard& action){
    //create prolog instance of TurnCard
}

struct ActionTyp2 {

}

void save_action(const ActionTyp2& action){

}



void PrologClient::Create_time_point_if_not_exists(uint timeInstant) {

    if (Time_point_already_exists(timeInstant)) {
        return;
    }

    auto bdgs = _pl.query(
            "rdf_costom_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimePoint',_," +
            std::to_string(timeInstant) + ",ObjInst)");
    logQueryResult(bdgs);
}

bool PrologClient::Time_point_already_exists(uint timeInstant) {
    auto bdgs = _pl.query(
            "owl_individual_of('http://knowrob.org/kb/knowrob.owl#TimePoint_" + std::to_string(timeInstant) + "','http://knowrob.org/kb/knowrob.owl#TimePoint')"
            );

    return bdgs.begin()==bdgs.begin();
}

