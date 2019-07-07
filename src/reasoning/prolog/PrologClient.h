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

        static constexpr char _ALLOWED_CHARS_FOR_RANDOM_NAMES[] =
                "abcdefghijklmnaoqrstuvwxyz"
                "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                "1234567890";

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

    public:

        void delete_instance(const Instance &instance);

        void save_action(const Instance &player, const RevealCardAction &reveal_card_action, unsigned int time_instant);

        void save_action(const Instance &player, const CoverCardAction &cover_card_action, unsigned int time_instant);

        void save_action(const Instance &player, const RemoveCardAction &remove_card_action, unsigned int time_instant);

        void save_action(const Instance &placer, StartGameAction start_game_action, unsigned int time_instant);

        void save(const ConcealedCard &concealed_card);

        void test_prolog_query();
    };


}