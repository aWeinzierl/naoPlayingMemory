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

        static std::string generate_random_string(uint length) noexcept;

        const std::string _NAMESPACE = "https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#";

        static Instance create_time_stamp(uint time_instant) noexcept;

        bool instance_already_exists(const Instance &instance);

        void save(const Instance &instance);

        void save_property(const Instance &instance_of_interest, const ObjectProperty &property);

        void save_property(const Instance &instance_of_interest, const DataProperty<std::string> &property);

        void save_property(const Instance &instance_of_interest, const DataProperty<unsigned int> &property);

        nonstd::optional<Instance> card_already_exists(uint id);

        nonstd::optional<Instance> card_already_exists(const CardPosition &card_position);

        nonstd::optional<Instance> player_already_exists(const std::string &player_name);

    public:

        void delete_cards(unsigned int id1, unsigned int id2);

        void save_action(/*const std::string &player_name,*/ const RevealCardAction &reveal_card_action/*, unsigned int time_instant*/);

        void save_action(const std::string &player_name, const CoverCardAction &cover_card_action, unsigned int time_instant);

        void save_action(const std::string &player_name, const RemoveCardAction &remove_card_action, unsigned int time_instant);

        void save_action(const std::string &player_name, StartGameAction start_game_action, unsigned int time_instant);

        void save(const ConcealedCard &concealed_card);

        //void test_prolog_query();

        std::vector<ConcealedCard> decide_action();

        nonstd::optional<ConcealedCard> search_paired_card(const ConcealedCard& concealed_card);

        nonstd::optional<ConcealedCard> search_random_card();

        bool are_cards_equal( unsigned int id1,const unsigned int id2);

        void reset();
    };


}