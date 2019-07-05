#include <iostream>
#include <fstream>
#include <pthread.h>


/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>

// Message File INCLUDES
#include <Noa/ConcealedCard.h>
#include <msg/ExposedCard.h>
#include <msg/Cards.h>
#include <msg/Position.h>




/*********************************************************************
 * CUSTOM CLASS
 * ******************************************************************/

#include "prolog/PrologClient.h"


using namespace reasoning;

void Callback(const std_msgs::String::ConstPtr &msg)
{
    
    uint position_counter=0;

    for (const  auto val&: msg->concealed_card_list){
        CardPosition position[position_counter](msg->concealed_card_list[val].position.x,msg->concealed_card_list[val].position.y);
        ConcealedCard concealed_card[val](msg->concealed_card_list[val].id,position);
        position_counter=+1;

    }
    for (const  auto val&: msg->exposed_card_list){
        CardPosition position[position_counter](msg->exposed_card_list[val].position.x,msg->exposed_card_list[val].position.y); 
        ExposedCard Exposed_Card[val](msg->exposed_card_list[val].id,msg->exposed_card_list[val].class_type,position);
        position_counter=+1;
    }

    for (const auto val&: msg->no_card_list){
        CardPosition no_card[position_counter](msg->exposed_card_list[val].position.x,msg->exposed_card_list[val].position.y); 
        //NoCard No_Card(position);
        position_counter=+1;
    }


}




int main( int argc, char** argv )
{

    ros::init(argc, argv, "StateProcessor");

    ROS_INFO_STREAM("Subscribing to Perception Node");

    ros::NodeHandle n;
    ros::Rate r(60);

    ros::Subscriber sub=n.subscribe("Cards",100,
                                   Callback);


    //State Processing
    for ()//all exposed_cards
    {
        //save_turn_card
    }
    






    ros::spinOnce();

    r.sleep();

}
