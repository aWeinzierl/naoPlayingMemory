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


        struct Position {
            unsigned int _x;
            unsigned int _y;
            //unsigned int _time;
        };


        struct EqualCards {
            Card _Card1;
            Card _Card2;
        };

        struct classification {
            std::string _class;
        };

        struct knownCard {
            Card _Card;
        };

        struct unknownCard {
            Instance _unknown_card;
            Position _position;
        };

        struct turn {
            Instance _turn;
            Instance _player;
            Instance _round;
            uint _timeInstance;
        };

        struct player {
            std::string _name;
        };


        struct round {
            Instance _round;
            turn _current_turn;
            uint _timeInstance;

        };

    class PrologClient {
        json_prolog::Prolog _pl;
        //ObjectProperty;
        static constexpr char _ALLOWED_CHARS_FOR_RANDOM_NAMES[] =
                "abcdefghijklmnaoqrstuvwxyz"
                "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                "1234567890";

        uint _ALLOWED_CHARS_FOR_RANDOM_NAMES_LEN;

        static constexpr uint _RANDOM_NAME_LENGTH = 16;

        const std::string _NAMESPACE = "https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#";

        static constexpr Instance create_time_stamp(uint time_instant) noexcept;

        std::string generate_random_string(uint length);

        bool instance_already_exists(const Instance& instance);

        void save(const Instance &instance);

        void save_property(const Instance &instance_of_interest, const ObjectProperty& property);

        void save_property(const Instance &instance_of_interest, const DataProperty& property);

        nonstd::optional<Instance> PrologClient::position_already_exists(const Position &position);

        nonstd::optional<Instance> PrologClient::concealed_card_already_exists(const ConcealedCard &ConcealedCard);

        nonstd::optional<Instance> PrologClient::card_already_exists(const uint id);


    public:

        

        PrologClient();
        



        void save_TimeStamp_property(const Instance &instance, const Instance &timeStamp);

        void delete_instance(const Instance &instance);

        void save_turn_card(const Instance &player, const uint id, uint time_instant);

        void instantiate_one_unknowncard(const ConcealedCard &ConcealedCard);

        void associate_turn_to_player(const Instance &Player_instance, const Instance &Turn_instance);

        void associate_current_turn_to_round(const Instance &turn, const round &round);
        
        void create_game(int game_num);

        void create_nao();

        /*

        void Create_time_point_if_not_exists(uint timeInstant);

        bool Time_point_already_exists(uint timeInstant);

        void logQueryResult(json_prolog::PrologQueryProxy &bdgs); 

        */
        

    };

    


}