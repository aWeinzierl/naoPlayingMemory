#pragma once

#include <string>

#include <json_prolog/prolog.h>

#include "classes/Card.h"
#include "classes/Instance.h"
#include "ObjectProperty.h"
#include "DataProperty.h"
#include "nonstd/optional.hpp"
#include "../../perception/vision/GridBoard.h"

using namespace json_prolog;

namespace reasoning {
        struct ConcealedCard{
                Position position;
                uint id;
        };

        struct ExposedCard{
                Position position;
                uint id;
                std::string class_type;
        };

    class PrologClient {
        json_prolog::Prolog _pl;
        
        static constexpr char _ALLOWED_CHARS_FOR_RANDOM_NAMES[] =
                "abcdefghijklmnaoqrstuvwxyz"
                "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                "1234567890";

        static constexpr uint _RANDOM_NAME_LENGTH = 16;

        static const uint _ALLOWED_CHARS_FOR_RANDOM_NAMES_LEN=strlen(_ALLOWED_CHARS_FOR_RANDOM_NAMES);

        const std::string _NAMESPACE = "https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#";

        static Instance create_time_stamp(uint time_instant) noexcept;

        std::string generate_random_string(uint length);

        bool instance_already_exists(const Instance& instance);

        void save(const Instance &instance);

        void save_property(const Instance &instance_of_interest, const ObjectProperty& property);

        void save_property(const Instance &instance_of_interest, const DataProperty& property);

        nonstd::optional<Instance> position_already_exists(const Position &position);

        nonstd::optional<Instance> card_already_exists(uint id);

    public:

        void delete_instance(const Instance &instance);

        void save_turn_card(const Instance &player, const  ExposedCard &Exposed_Card, uint time_instant);

        void instantiate_one_unknowncard(const ConcealedCard &ConcealedCard);
    };

    


}