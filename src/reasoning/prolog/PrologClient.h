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
            Instance _knwon_card;
            Position _position;
            classification _class;
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

        nonstd::optional<Instance> position_already_exists(CardPosition position);

        void save(const Instance &instance);

        void save_property(const Instance &instance_of_interest, const ObjectProperty& property);
        void save_property(const Instance &instance_of_interest, const DataProperty& property);

    public:

        

        PrologClient();
        
        /*void assert_property(const Instance &instance, const ObjectProperty &property);

        void save_CardPosition_property(const Instance &instance, const Instance &position);
        */
        uint CreateTypeInstance(const std::string &associatedClass);


        void save_TimeStamp_property(const Instance &instance, const Instance &timeStamp);

        void save_performed_action(const std::string &player_name, const Card &action, uint time_instant);
        void delete_instance(const Instance &instance);

        void save_turn_card(const std::string &player_name, const Card &action, uint time_instant);

        void save(const knownCard &Card1,const knownCard &Card2, uint timeInstant);
        
        void save(const unknownCard &Card1, const unknownCard & Card2,uint TimeInstance);

        void instantiate_one_unknowncard(uint i, uint j, uint count);

        void instantiate_all_unknownCards();

        void associate_turn_to_player(const Instance &Player_instance, const Instance &Turn_instance);

        void save(const player &player);

        void save(const turn &turn);

        void save(const round &round);

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