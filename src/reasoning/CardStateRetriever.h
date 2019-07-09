#pragma once

#include <tuple>
#include <vector>
#include <mutex>

#include <ros/subscriber.h>
#include <nao_playing_memory/Cards.h>
#include <ros/node_handle.h>
#include <nonstd/optional.hpp>

#include "model/ConcealedCard.h"
#include "model/ExposedCard.h"
#include "model/CardPosition.h"

using AllCards = std::tuple<
        std::vector<reasoning::ConcealedCard>,
        std::vector<reasoning::ExposedCard>,
        std::vector<reasoning::CardPosition>>;

class CardStateRetriever {

    ros::Subscriber _sub;
    ros::NodeHandle _n;
    ros::Rate _ros_rate;
    nonstd::optional<AllCards> _cards;
    std::mutex _cards_mutex;

    void vision_callback(const nao_playing_memory::Cards::ConstPtr &msg);

public:

    explicit CardStateRetriever(unsigned int ros_rate);


    AllCards retrieve_current_state();
};