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

PrologClient::Instance::Instance(const std::string &classType, const std::string &name) : _class(classType),_name(name) {
}

PrologClient::Instance::Property(std::string name,std::string value): _name(name), _value(value) {
}

const std::string &PrologClient::Instance::Get_class() const noexcept {
    return _class;
}

uint PrologClient::Instance::Get_id() const noexcept {
    return _id;
}

uint PrologClient::Instance::Get_id() const noexcept {
    return _name;
}




void PrologClient::create_instance (const Instance& instance){ 
    PrologQueryProxy bdgs = _pl.query("rdf_costom_instance_from_class('"+ _NAMESPACE + instance._class+ ",_," +instance._name+ "', ObjInst)");
 }



void PrologClient::assert_property(const Instance& instance,const Property& property){
    PrologQueryProxy bdgs = _pl.query("rdf_assert('" + _NAMESPACE + instance._class + "_" +instance._name+ "','" + _NAMESPACE + property._name + "','" + property._value + "')");
                                                /* (instance, propertyName, propertyValue/Instance) */
}

void PrologClient::assert_timeStamp_property(const Instance& instance,const Instance& timeStamp){
    PrologQueryProxy bdgs = _pl.query("rdf_assert('" + _NAMESPACE + instance._class + "_" +instance._name+ "','" + _NAMESPACE + "hasTimeStamp','" + timeStamp._name + "')");
                                                    /*(instance, hasTimeStamp, TimeStamp_x) */
}

void PrologClient::assert_CardPosition_property(const Instance& instance,const Instance& position){
    PrologQueryProxy bdgs = _pl.query("rdf_assert('" + _NAMESPACE + instance._class + "_" +instance._name+ "','" + _NAMESPACE + "hasPositin','" + position._name + "')");
                                                    /*(instance, hasPosition, intance_from_cardposition) */
}

void PrologClient::delete_instance(const Instance& instance){
    PrologQueryProxy bdgs = _pl.query("del('" + _NAMESPACE + instance._class+ "','" + _NAMESPACE + instance._class+  "',[])");

}


struct Position{
    unsigned int _x;
    unsigned int _y;
    //unsigned int _time;
}

struct TurnCard {
    Position _position;
}

struct EqualCards {
    TurnCard _Card1;
    TurnCard _Card2; 
}

struct Classification{
    std::string _class;
}

struct KnownCard{
    Instance _knwon_card;
    position _position;
    Classification _class;
}
struct UnknownCard{
    Instance _unknown_card;
    postion _position;
}

struct Turn{
    Instance _turn;
    Instance _player;
    Instance _round;
    uint _timeInstance;
}

struct Player{
    std::string _name;
    uint _num; //Nao is always 1, after that is 2,3,4,5,.....
}


struct Round{
    Instance _round;
    turn _current_turn;
    uint _timeInstance

}

void PrologClient::save_action_TurnCard(const TurnCard& action,uint timeInstant){
    //create prolog instance of TurnCard
    Instance turn_card("TurnOneCard","Turn_Card");
    create_instance(turn_card);
    //assert Timestamp_instance as property from TurnCard
    Instance timeStamp("TimeStamp","TimeStamp" + std::to_string(timeInstant));
    create_instance(timeStamp);
    assert_timeStamp_property(turn_card,timeStamp);
    //assert CardPosition as an instance and linkt it to turn_card as two separate propertys
    Instance card_position("CardPosition","Card_position" +std::to_string(timeInstant));
    Property x_pos("hasXCoordinate",std::to_string(TurnCard._x));
    Porperty y_pos("hasYCoordinate",std::to_string(TurnCard._y));
    assert_property(card_position,x_pos);
    assert_property(card_position,y_pos);
    assert_property(turn_card,card_position);

    //So now there should be a Instance od TurnCard which has as propertys: a timestamp(instance of timestamp) and a postion(instance of CardPosition)
    

}


void PrologClient::save_action_turn_equal_cards(const EqualCard& action,uint timeInstant){
    save_action_TurnCard(action._Card1, timeInstant);
    save_action_TurnCard(action._Card2, timeInstant);
    Instance turn_equal_cards("TurnEqualCards","Turn_Equal_Cards");
    //assert Timestamp_instance as property from turn_equal_cards
    Instance timeStamp("TimeStamp","TimeStamp" + std::to_string(timeInstant));
    create_instance(timeStamp);
    assert_timeStamp_property(turn_equal_cards,timeStamp); 

}


void PrologClient::save_action_turn_two_unkown_cards(const UnknownCard& Card1, consta UnknownCard& Card2,uint TimeInstance){
    
    save_action_TurnCard(Card1._unknown_card, timeInstant);
    save_action_TurnCard(Card2._unknown_card, timeInstant);

}

void PrologClient::instanciate_one_unknowncard(uint x_pos,uint y_pos,uint count){
    //sdt::string name<<"unknown_card_"+std::to_string(i)+std:to_string(j);
    Instance unknown_card_ins[count]("UnknownCard","UnkownCard_"+std::to_string(x_pos)+std:to_string(y_pos));
    create_instance(unknown_card_ins[count]);
    Instance card_position("CardPosition","Card_position"+std::to_string(x_pos)+std:to_string(y_pos));
    Property _x_pos("hasXCoordinate",std::to_string(x_pos));
    unknonw_card_ins[count]._position._x=x_pos;
    Porperty _y_pos("hasYCoordinate",std::to_string(y_pos));
    unknonw_card_ins[count]._position._y=y_pos;
    assert_property(card_position,_x_pos);
    assert_property(card_position,_y_pos);
    assert_property(unkown_card_ins[count],card_position);
}

void PrologClient::instanciate_all_unknownCards(){
    //Do at the beginnnig 
    Instance unknown_card_ins[12];
    int count=0;
    for (int a=1,a<4,a++){
        for(int b=1,b<=4,b++){
            instanciate_all_unknownCards(a,b,count);
            UnknownCard._unknown_card[count]=unknown_card_ins[count];
            count=count+1;
        }
    }

}


void PrologClient::associateTurnToPlayer(const Instance& Player_instance,const Instance& Turn_instance){
    PrologQueryProxy bdgs = _pl.query("is_in_turn('"+ _NAMESPACE + Player_instance._class+ "','"+ _NAMESPACE + Turn_instance._class+ "')");

}


void PrologClient::create_Player_instance(const Player& player){
    Instance player("Player","Player_"+std::to_String(player._num));
    create_instance(player);
}



void PrologClient::create_turn_instance(const Turn& turn){
    Instance turn("Turn","Turn_"+std::to_String(turn._timeInstance)+turn._round._name);
    turn._turn=turn;
    create_instance(turn);
    //Associate a player
    Property associatedToPlayer("associatedToPlayer",turn._player._name);
    assert_property(turn,associatedToPlayer);
    //Associate a timeStampt
    Instance timeStamp("TimeStamp","TimeStamp" + std::to_string(timeInstant));
    create_instance(timeStamp);
    assert_timeStamp_property(turn,timeStamp);

}


void PrologClient::create_round_instance(const Round& round){
    Instance round("Round","Round_"+std::to_String(round._timeInstance));
    create_instance(round);
}



void PrologClient::associate_currente_turn__to_round(const Instance& turn,const Round& round){
    delete_instance(round._current_turn);
    round._current_turn = turn;
    Property hasCurrentTurn("hasCurrentTurn",turn);
    assert_property(round._round,hasCurrentTurn);
}


/*
void PrologClient::create_action_takecards(){
    

}
 */

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

void PrologClient::create_nao(){
   PrologQueryProxy bdgs = _pl.query("rdf_costom_instance_from_class('" + _NAMESPACE + "Player_Nao, ObjInst')");
}

void PrologClient::create_game(int game_num){
    PrologQueryProxy bdgs = _pl.query("rdf_costom_instance_from_class('" + _NAMESPACE + "MemoryGame,_," +std::to_String(game_num)+ "', ObjInst)");

    //Initialize timeStamps
}

void PrologClient::create_ArucoToIdObjectMappings(){}

