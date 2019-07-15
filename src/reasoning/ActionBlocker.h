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

    /// callback used to retrieve the latest state of the board
    /// \param msg
    void vision_callback(const nao_playing_memory::Cards::ConstPtr &msg);

    /// sleep with ros at the given rate
    void spin();

public:

    /// constructs the actionblocker completely
    /// \param ros_rate frequency to check for detected actions
    /// \param persistence necessary time which the a change has to persist until it is detected,
    /// in periods with respect to the ros_rate frequency
    ActionBlocker(unsigned int ros_rate, unsigned int persistence);

    /// waits until all cards with the given ids have been covered at least once
    /// \param card_ids ids of the cards
    void wait_until_cards_covered(const std::vector<unsigned int> &card_ids);

    /// waits until the card at the position is revealed
    /// \param card_pos position of the card which is waited for
    /// \return the card which has been revealed
    reasoning::ExposedCard wait_until_card_is_revealed(const reasoning::CardPosition &card_pos);

    /// blocks until the card at the given position has been removed
    /// \param card_position position of the card on the board
    void wait_until_card_is_removed(const reasoning::CardPosition& card_position);

    /// waits until all cards have been removed at least once
    /// \param card_positions positions of the cards which should have been removed
    void wait_until_cards_removed(const std::vector<reasoning::CardPosition>& card_positions);

    /// wait until a certain amount of cards has been revealed
    /// \param amount_of_cards
    /// \return all cards which have been revealed
    std::vector<reasoning::ExposedCard> wait_until_enough_cards_revealed(unsigned int amount_of_cards);

    /// wait until all cards have been revealed at least once
    /// \param card_ids ids of the cards to be revealed
    void wait_until_cards_revealed(const std::vector<unsigned int> &card_ids);
};