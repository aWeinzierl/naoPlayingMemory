#pragma once

#include <json_prolog/prolog.h>
#include <nonstd/optional.hpp>

#include "Instance.h"
#include "ObjectProperty.h"
#include "DataProperty.h"
#include "../model/ExposedCard.h"
#include "../model/ConcealedCard.h"

using namespace json_prolog;

namespace reasoning {

    using RevealCardAction = ExposedCard;
    using CoverCardAction = ConcealedCard;
    using RemoveCardAction = CardPosition;
    using StartGameAction = struct{};

    class PrologClient {
        json_prolog::Prolog _pl;

        static constexpr char _ALLOWED_CHARS_FOR_RANDOM_NAMES[] = "1234567890";

        static constexpr uint _RANDOM_NAME_LENGTH = 16;

        static const uint _ALLOWED_CHARS_FOR_RANDOM_NAMES_LEN = strlen(PrologClient::_ALLOWED_CHARS_FOR_RANDOM_NAMES);

        const std::string _NAMESPACE = "https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#";

        /// generate a random string with the allowed characters
        /// \param length length of the string
        /// \return generated string
        static std::string generate_random_string(uint length) noexcept;

        /// create a time stamp instance from a time_instant
        /// \param time_instant
        /// \return instance of the timeStamp
        static Instance create_time_stamp(uint time_instant) noexcept;

        /// checks if the given instance already exists in the ontology
        /// \param instance
        /// \return the instance exists
        bool instance_already_exists(const Instance &instance);

        /// save a instance
        /// \param instance
        void save(const Instance &instance);

        /// save a object property
        /// \param instance_of_interest
        /// \param property
        void save_property(const Instance &instance_of_interest, const ObjectProperty &property);

        /// save a data property with a string
        /// \param instance_of_interest
        /// \param property
        void save_property(const Instance &instance_of_interest, const DataProperty<std::string> &property);

        /// save a data property with a unsigned int type
        /// \param instance_of_interest
        /// \param property
        void save_property(const Instance &instance_of_interest, const DataProperty<unsigned int> &property);

        /// check if a card with the id already exists
        /// \param id
        /// \return instance of the card
        nonstd::optional<Instance> card_already_exists(uint id);

        /// checks if a card with that position already exists in the ontology
        /// \param card_position
        /// \return
        nonstd::optional<Instance> card_already_exists(const CardPosition &card_position);

        /// check if a player exists in the ontology
        /// \param player_name
        /// \return instance of the player
        nonstd::optional<Instance> player_already_exists(const std::string &player_name);

    public:

        /// delete cards in ontology
        /// \param id1 id of the first card
        /// \param id2 id of the second card
        void delete_cards(unsigned int id1, unsigned int id2);

        /// save information received by the action
        /// \param reveal_card_action the action object
        void save_action(/*const std::string &player_name,*/ const RevealCardAction &reveal_card_action/*, unsigned int time_instant*/);

        /// save information received by the action
        /// \param player_name name of the player that executed the action
        /// \param cover_card_action action object with the information
        /// \param time_instant time instant when the action has been executed
        void save_action(const std::string &player_name, const CoverCardAction &cover_card_action, unsigned int time_instant);


        /// save information received by the action
        /// \param player_name name of the player that executed the action
        /// \param cover_card_action action object with the information
        /// \param time_instant time instant when the action has been executed
        void save_action(const std::string &player_name, const RemoveCardAction &remove_card_action, unsigned int time_instant);


        /// save information received by the action
        /// \param player_name name of the player that executed the action
        /// \param cover_card_action action object with the information
        /// \param time_instant time instant when the action has been executed
        void save_action(const std::string &player_name, StartGameAction start_game_action, unsigned int time_instant);

        /// add the concealed card to the ontology
        /// \param concealed_card information of the card
        void save(const ConcealedCard &concealed_card);

        /// decide which cards to turn
        /// \return cards which should be turned
        std::vector<ConcealedCard> decide_action();

        /// search a matching card for the given one - in the ontology
        /// \param concealed_card card for which a card should be searched
        /// \return matching card, if found
        nonstd::optional<ConcealedCard> search_paired_card(const ConcealedCard& concealed_card);

        /// checks if the two cards have the same class
        /// \param concealed_card card 1
        /// \param concealed_card2 card 2
        /// \return one of the cards if they have the same class
        nonstd::optional<ConcealedCard> search_if_paired_card(const ConcealedCard &concealed_card,const ConcealedCard &concealed_card2);

        /// find a random not yet revealed card
        /// \return card if such one has found
        nonstd::optional<ConcealedCard> search_random_card();

        /// check if two cards are equal
        /// \param id1 id of the first card
        /// \param id2 id of the second card
        /// \return if they are equal
        bool are_cards_equal( unsigned int id1,const unsigned int id2);

        /// reset the ontology
        void reset();
    };


}