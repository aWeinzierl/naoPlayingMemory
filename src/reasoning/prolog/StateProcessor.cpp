#include <iostream>
#include <fstream>
#include <pthread.h>
#include <queue> 


/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>


/*********************************************************************
 * CUSTOM CLASS
 * ******************************************************************/

#include "prolog/PrologClient.h"


namespace reasoning{

    void StateProcessor::process_new_state(const std::vector<ConcealedCard>& ConcealedCard,const std::vector<ExposedCard>& ExposedCard,const std::vector<CardPosition>& NoCards)

        //State Processing
        for ()//all exposed_cards
        {
            //save_turn_card
        }
        
        //12 ques-->1 pro card
        //--->1 array: que[12][]--> each roch means one diferente card
        //filter die zustand füur que speichert
        //bei state anderung production von state
        //0-->ConcealedCard   1-->ExposedCard    2--> NoCard
        if(/*first time*/){
            //create 12 queues
        }
        else{
            //analise State from cards
            for(const auto card&: ExposedCard){
                //go to que from thar card id
                mapping.find(card.get_Position())->second.push(1);
            }
            for(const auto card&: ConcealedCard){
                mapping.find(card.get_Position())->second.push(0);
            }
            for(const auto card&: NoCard){
                mapping.find(card)->second.push
            }
        }

        //filter auf die que laufen lassen



    }
}