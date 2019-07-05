#pragma once

#include <ros/ros.h>

#include "nao_playing_memory/Cards.h"
#include "nao_playing_memory/ConcealedCard.h"
#include "nao_playing_memory/Position.h"

#include "model/ConcealedCard.h"
#include "model/CardPosition.h"
#include "model/ExposedCard.h"

#include "prolog/StateProcessor.h"

class NodeManager {
private:
    ros::NodeHandle _n;
    reasoning::StateProcessor _sp;
    ros::Subscriber _sub;
public:

    NodeManager();

    static reasoning::CardPosition cardPositionMap(const nao_playing_memory::Position &position);

    void vision_callback(const nao_playing_memory::Cards::ConstPtr &msg);
};
