#pragma once

#include <ros/subscriber.h>
#include <nao_playing_memory/Cards.h>
#include <ros/node_handle.h>

#include "model/ExposedCard.h"
#include "model/ConcealedCard.h"
#include "model/CardPosition.h"

#include "StateProcessor.h"

class ActionBlocker {

    ros::Subscriber _sub;
    ros::NodeHandle _n;
    ros::Rate _ros_rate;

    reasoning::StateProcessor _sp;

    void vision_callback(const nao_playing_memory::Cards::ConstPtr &msg);

public:

    ActionBlocker(unsigned int ros_rate, unsigned int persistence);

    void wait_until_card_is_covered(unsigned int card_id);

    reasoning::ExposedCard wait_until_card_is_revealed(const reasoning::CardPosition &card_pos);

    void wait_until_card_is_removed(const reasoning::CardPosition& card_position);

    void wait_until_cards_removed(const std::vector<reasoning::CardPosition>& card_positions);

    reasoning::ExposedCard wait_until_any_card_is_revealed();

    void spin();
};