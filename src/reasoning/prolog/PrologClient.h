#pragma once

#include <string>

#include <json_prolog/prolog.h>

#include "classes/Card.h"
#include "classes/Instance.h"
#include "ObjectProperty.h"
#include "nonstd/optional.hpp"
#include "../../perception/vision/GridBoard.h"

namespace reasoning {

    class PrologClient {
        json_prolog::Prolog _pl;
        static constexpr char _ALLOWED_CHARS_FOR_RANDOM_NAMES[] =
                "abcdefghijklmnaoqrstuvwxyz"
                "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                "1234567890";
        static constexpr uint _RANDOM_NAME_LENGTH = 16;

        const std::string _NAMESPACE = "https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#";

        static constexpr Instance create_time_stamp(uint time_instant) noexcept;

        std::string generate_random_string(uint length);

        bool instance_already_exists(const Instance& instance);

        nonstd::optional<Instance> position_already_exists(CardPosition position);

        void save(const Instance &instance);

        void save_property(const Instance &instance_of_interest, const ObjectProperty& property);

    public:



        PrologClient();

        uint CreateTypeInstance(const std::string &associatedClass);



        void save_TimeStamp_property(const Instance &instance, const Instance &timeStamp);

        void save_performed_action(const std::string &player_name, const TurnCard &action, uint time_instant);
        void delete_instance(const Instance &instance);

        void save_performed_action(const TurnCard &action, uint timeInstant);

        void save(const EqualCard &action, uint timeInstant);

        void save(const unknownCard &Card1, consta unknownCard & Card2,uint TimeInstance);

        void instantiate_one_unknowncard(uint i, uint j, uint count);

        void instantiate_all_unknownCards();

        void associate_turn_to_player(const Instance &Player_instance, const Instance &Turn_instance);

        void save(const player &player);

        void save(const turn &turn);

        void save(const round &round);

        void associate_current_turn_to_round(const Instance &turn, const round round);


    };


}