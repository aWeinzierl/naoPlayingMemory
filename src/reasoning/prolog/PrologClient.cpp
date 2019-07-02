#include "PrologClient.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <json_prolog/prolog.h>
#include <unordered_map>

#include "PrologClient.h"

namespace reasoning {

    PrologClient::Instance::Instance(const std::string &classType, const std::string &name) : _class(classType),
                                                                                              _name(name) {
    }

    PrologClient::Instance::Property(std::string
    name,
    std::string value
    ):

    _name (name), _value(value) {
    }

    const std::string &PrologClient::Instance::get_class() const noexcept {
        return _class;
    }

    uint PrologClient::Instance::get_id() const noexcept {
        return _id;
    }

    uint PrologClient::Instance::get_id() const noexcept {
        return _name;
    }


    void PrologClient::save(const Instance &instance) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_costom_instance_from_class('" + _NAMESPACE + instance._class + ",_," + instance._name +
                "', ObjInst)");
    }


    void PrologClient::assert_property(const Instance &instance, const Property &property) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_assert('" + _NAMESPACE + instance._class + "_" + instance._name + "','" + _NAMESPACE +
                property._name + "','" + property._value + "')");
        /* (instance, propertyName, propertyValue/Instance) */
    }

    void PrologClient::save_TimeStamp_property(const Instance &instance, const Instance &timeStamp) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_assert('" + _NAMESPACE + instance._class + "_" + instance._name + "','" + _NAMESPACE +
                "hasTimeStamp','" + timeStamp._name + "')");
        /*(instance, hasTimeStamp, TimeStamp_x) */
    }

    void PrologClient::save_CardPosition_property(const Instance &instance, const Instance &position) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_assert('" + _NAMESPACE + instance._class + "_" + instance._name + "','" + _NAMESPACE +
                "hasPositin','" + position._name + "')");
        /*(instance, hasPosition, intance_from_cardposition) */
    }

    void PrologClient::delete_instance(const Instance &instance) {
        PrologQueryProxy bdgs = _pl.query(
                "del('" + _NAMESPACE + instance._class + "','" + _NAMESPACE + instance._class + "',[])");

    }


    struct Position {
        unsigned int _x;
        unsigned int _y;
        //unsigned int _time;
    }


    struct EqualCards {
        TurnCard _Card1;
        TurnCard _Card2;
    }

    struct classification {
        std::string _class;
    }

    struct knownCard {
        Instance _knwon_card;
        position _position;
        classification _class;
    }
    struct unknownCard {
        Instance _unknown_card;
        postion _position;
    }

    struct turn {
        Instance _turn;
        Instance _player;
        Instance _round;
        uint _timeInstance;
    }

    struct player {
        std::string _name;
    }


    struct round {
        Instance _round;
        turn _current_turn;
        uint _timeInstance

    }


    void PrologClient::save_property(const Instance &instance_of_interest, const ObjectProperty& property) {
        _pl.query(
                "rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" + instance_of_interest.get_name() +
                "','" + _NAMESPACE + property.get_name() +
                "','" + _NAMESPACE + property.get_value().get_class() + "_" + property.get_value().get_name()  + "')");
    }


    bool PrologClient::instance_already_exists(const Instance &instance) {
        auto bdgs = _pl.query(
                "owl_individual_of('" + _NAMESPACE + instance.get_class() + "_" + instance.get_name()
                + "','http://knowrob.org/kb/knowrob.owl#TimePoint')"
        );

        return bdgs.begin() == bdgs.begin();
    }


    std::string PrologClient::generate_random_string(uint length) {
        std::mt19937_64 gen{std::random_device()()};
        std::uniform_int_distribution<size_t> dist{0, _ALLOWED_CHARS_FOR_RANDOM_NAMES.length() - 1};
        std::string ret;

        std::generate_n(
                std::back_inserter(ret),
                length,
                [&] { return _ALLOWED_CHARS_FOR_RANDOM_NAMES[dist(gen)]; });
        return ret;

    }

    void
    PrologClient::save_performed_action(const std::string &player_name, const TurnCard &action, uint time_instant) {
        auto time_stamp = create_time_stamp(time_instant);
        if (!instance_already_exists(time_stamp)) {
            save(time_stamp);
        }

        const std::string turn_one_card_class = "TurnOneCard";
        Instance turn_card(turn_one_card_class, generate_random_string(_RANDOM_NAME_LENGTH));
        while (instance_already_exists(turn_card)) {
            turn_card = Instance(turn_one_card_class, generate_random_string(_RANDOM_NAME_LENGTH));
        }
        save(turn_card);

        auto hasTimeStamp = ObjectProperty("hasTimeStamp", time_stamp);
        save_property(turn_card, hasTimeStamp);

        Instance card_position("CardPosition", generate_random_string(_RANDOM_NAME_LENGTH));


        auto hasXCoordinate = ("hasXCoordinate", po)

        Property x_pos("hasXCoordinate", std::to_string(TurnCard._x));
        Porperty y_pos("hasYCoordinate", std::to_string(TurnCard._y));
    }


    void PrologClient::save(const EqualCard &action, uint timeInstant) {
        save(action._Card1, timeInstant);
        save(action._Card2, timeInstant);
        Instance turn_equal_cards("TurnEqualCards", "Turn_Equal_Cards");
        //assert Timestamp_instance as property from turn_equal_cards
        Instance timeStamp("TimeStamp", "TimeStamp" + std::to_string(timeInstant));
        save(timeStamp);
        save_TimeStamp_property(turn_equal_cards, timeStamp);

    }

    Instance PrologClient::create_time_stamp(uint time_instant) {
        return Instance("TimeStamp", std::to_string(time_instant));
    }

    nonstd::optional<Instance> PrologClient::position_already_exists(Position position) {
        //TODO: check if position exists
        //if it exists return instance of it
        return nonstd::nullopt;
    }

    void PrologClient::save(const unknownCard &Card1, consta unknownCard &

    Card2,
    uint TimeInstance
    ) {

    save(Card1
    ._unknown_card, timeInstant);
    save(Card2
    ._unknown_card, timeInstant);

}

void PrologClient::instantiate_one_unknowncard(uint i, uint j, uint count) {
    //sdt::string name<<"unknown_card_"+std::to_string(i)+std:to_string(j);
    Instance
    unknown_card_ins[count]("UnknownCard", "UnkownCard_" + std::to_string(i) + std:to_string(j));
    save(unknown_card_ins[count]);
    Instance
    card_position("CardPosition", "Card_position" + std::to_string(i) + std:to_string(j));
    Property x_pos("hasXCoordinate", std::to_string(i));
    unknonw_card_ins[count]._position._x = i;
    Porperty y_pos("hasYCoordinate", std::to_string(j));
    unknonw_card_ins[count]._position._y = j;
    assert_property(card_position, x_pos);
    assert_property(card_position, y_pos);
    assert_property(unkown_card_ins[count], card_position);
}

void PrologClient::instantiate_all_unknownCards() {
    //Do at the beginnnig 
    Instance unknown_card_ins[12];
    int count = 0;
    for (int a = 1; a < 4; ++a) {
        for (int b = 1; b < 4; ++b) {
            instantiate_one_unknowncard(a, b, count);
            unknownCard._unknown_card[count] = unknown_card_ins[count];
            count = count + 1;
        }
    }

}


void PrologClient::associate_turn_to_player(const Instance &Player_instance, const Instance &Turn_instance) {
    PrologQueryProxy bdgs = _pl.query(
            "is_in_turn('" + _NAMESPACE + Player_instance._class + "','" + _NAMESPACE + Turn_instance._class + "')");
}


void PrologClient::create_Player_instance(const player &player) {
    Instance player("Player", "Player_" + std::to_String(player._num));
    create_instance(player);
}


void PrologClient::create_turn_instance(const turn &turn) {
    Instance turn("Turn", "Turn_" + std::to_String(turn._timeInstance) + turn._round._name);
    turn._turn = turn;
    create_instance(turn);
    //Associate a player
    Property associatedToPlayer("associatedToPlayer", turn._player._name);
    assert_property(turn, associatedToPlayer);
    //Associate a timeStampt
    Instance timeStamp("TimeStamp", "TimeStamp" + std::to_string(timeInstant));
    create_instance(timeStamp);
    assert_timeStamp_property(turn, timeStamp);

}


void PrologClient::create_round_instance(const round &round) {
    Instance round("Round", "Round_" + std::to_String(round._timeInstance));
    create_instance(round);
}


void PrologClient::associate_current_turn_to_round(const Instance &turn, const round round) {
    delete_instance(round._current_turn);
    round._current_turn = turn;
    Property hasCurrentTurn("hasCurrentTurn", turn);
    assert_property(round._round, hasCurrentTurn);
}


/*
void PrologClient::create_action_takecards(){
    

}
 */

void PrologClient::Create_time_point_if_not_exists(uint timeInstant) {

    if (Time_point_already_exists(timeInstant)) {
        return;
    }

    auto bdgs = _pl.query(
            "rdf_costom_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimePoint',_," +
            std::to_string(timeInstant) + ",ObjInst)");
    logQueryResult(bdgs);
}

bool PrologClient::Time_point_already_exists(uint timeInstant) {
    auto bdgs = _pl.query(
            "owl_individual_of('http://knowrob.org/kb/knowrob.owl#TimePoint_" + std::to_string(timeInstant) +
            "','http://knowrob.org/kb/knowrob.owl#TimePoint')"
    );

    return bdgs.begin() == bdgs.begin();
}

void PrologClient::create_nao() {
    PrologQueryProxy bdgs = _pl.query("rdf_costom_instance_from_class('" + _NAMESPACE + "Player_Nao, ObjInst')");
}

void PrologClient::create_game(int game_num) {
    PrologQueryProxy bdgs = _pl.query(
            "rdf_costom_instance_from_class('" + _NAMESPACE + "MemoryGame,_," + IntToStr(game_num) + "', ObjInst)");
}

void PrologClient::create_ArucoToIdObjectMappings() {}

}