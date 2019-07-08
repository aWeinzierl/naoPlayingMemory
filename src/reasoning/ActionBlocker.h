#pragma once

#include <ros/subscriber.h>
#include <nao_playing_memory/Cards.h>
#include <ros/node_handle.h>

#include "model/ExposedCard.h"
#include "model/ConcealedCard.h"
#include "model/CardPosition.h"

class ActionBlocker {

    ros::Subscriber _sub;
    ros::NodeHandle _n;
    ros::Rate _ros_rate;

    reasoning::ConcealedCard _object_of_desire;
    bool _has_found_action;

    void reveal_card_callback(const nao_playing_memory::Cards::ConstPtr &msg);

    void remove_card_callback(const nao_playing_memory::Cards::ConstPtr &msg);


public:

    ActionBlocker(unsigned int ros_rate);

    void wait_until_card_is_revealed(unsigned int card_id);

    void wait_until_card_is_removed(reasoning::CardPosition card_position);
};