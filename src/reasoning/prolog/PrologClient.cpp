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
#include <cstring>
#include "PrologClient.h"


namespace reasoning {

    //saves intance
    void PrologClient::save(const Instance &instance) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_costom_instance_from_class('" + _NAMESPACE + instance.get_class() + ",_," + instance.get_name() +
                "', ObjInst)");
    }


    /*void PrologClient::save_TimeStamp_property(const Instance &instance, const Instance &timeStamp) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_assert('" + _NAMESPACE + instance.get_class() + "_" + instance.get_name() + "','" + _NAMESPACE +
                "hasTimeStamp','" + timeStamp.get_name() + "')");
        /*(instance, hasTimeStamp, TimeStamp_x) 
    }*/

    void PrologClient::delete_instance(const Instance &instance) {
        PrologQueryProxy bdgs = _pl.query(
                "del('" + _NAMESPACE + instance.get_class() + "','" + _NAMESPACE + instance.get_class() + "',[])");

    }

  

    //Creates ObjectProperty
    void PrologClient::save_property(const Instance &instance_of_interest, const ObjectProperty& property) {
        _pl.query(
                "rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" + instance_of_interest.get_name() +
                "','" + _NAMESPACE + property.get_name() +
                "','" + _NAMESPACE + property.get_value().get_class() + "_" + property.get_value().get_name()  + "')");
    }
    //creates DataPorperty
    void PrologClient::save_property(const Instance &instance_of_interest, const DataProperty& property) {
        _pl.query(
                "rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" + instance_of_interest.get_name() +
                "','" + _NAMESPACE + property.get_name() +
                "','" + to_string(property.get_value())  + "')");
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
        std::uniform_int_distribution<size_t> dist{0,PrologClient::_ALLOWED_CHARS_FOR_RANDOM_NAMES_LEN - 1};
        std::string ret;

        std::generate_n(
                std::back_inserter(ret),
                length,
                [&] { return PrologClient::_ALLOWED_CHARS_FOR_RANDOM_NAMES[dist(gen)]; });
        return ret;

    }

    //creates instance of turn_Card action  
    void
    PrologClient::save_turn_card(const Instance &player, const  ExposedCard &Exposed_Card, uint time_instant) {
        
        //create time_stamp
        auto time_stamp = create_time_stamp(time_instant);
        if (!instance_already_exists(time_stamp)) {
            save(time_stamp);
        }
        //create action(TurnOneCard) instance
        const std::string turn_one_card_class = "TurnOneCard";
        Instance turn_card(turn_one_card_class, generate_random_string(_RANDOM_NAME_LENGTH));
        while (instance_already_exists(turn_card)) {
            turn_card = Instance(turn_one_card_class, generate_random_string(_RANDOM_NAME_LENGTH));
        }
        save(turn_card);

        //associate to timestamp property
        auto hasTimeStamp = ObjectProperty("hasTimeStamp", time_stamp);
        save_property(turn_card, hasTimeStamp);

        //link to card instance
        //search card instance
        auto card1 = card_already_exists(Exposed_Card.id);
        if (!card1.has_value()){
            throw new logic_error("Card_already_exist");
        }
        //create class instance

        //associate class to card
        DataProperty class_prop("hasClass",Exposed_Card.class_type);
        save_property(class_prop,card1.value());

        //associate card to action turn card
        ObjectProperty card_prop("hasCard",card1.value());
        save_property(turn_card,card_prop);
        
        //associate action to player
        ObjectProperty hasAction("hasAction",turn_card);
        save_property(player,hasAction);

    }

    Instance PrologClient::create_time_stamp(uint time_instant) noexcept {
        return Instance("TimeStamp", std::to_string(time_instant));
    }

    nonstd::optional<Instance> PrologClient::position_already_exists(const Position &position) {
        //TODO: check if position exists
        //if it exists return instance of it
        return nonstd::nullopt;
    }

    


    nonstd::optional<Instance> PrologClient::card_already_exists(const uint id) {
   
        PrologQueryProxy bdgs=_pl.query("owl_has(Instance,'" + _NAMESPACE + "hasMarkerId' ,'" + to_string(id) + "')");
        if(bdgs.begin()==bdgs.end()){
            return nonstd::nullopt;
        }
        auto instance_bdg = *(bdgs.begin());
        Instance instance("Card", instance_bdg["Instance"]);
        return instance;
    }

    void PrologClient::instantiate_one_unknowncard(const ConcealedCard &Concealed_Card) {

        auto card = card_already_exists(Concealed_Card.id);
        if (!card.has_value()){
            throw new logic_error("Card_already_exist");
            //exit function----->
        }
        //sdt::string name<<"unknown_card_"+std::to_string(i)+std:to_string(j);
        Instance  unknown_card_ins("UnknownCard", "Card_" + to_string(Concealed_Card.id));
        save(unknown_card_ins);
        Instance card_pos("CardPosition","CardPosition" + to_string(Concealed_Card.position._x)+to_string(Concealed_Card.position._x));
        ObjectProperty card_position("hasPosition", card_pos);
        DataProperty x_pos("hasXCoordinate", Concealed_Card.position._x);
        DataProperty y_pos("hasYCoordinate", Concealed_Card.position._y);
        save_property(card_pos, x_pos);
        save_property(card_pos, y_pos);
        save_property(unknown_card_ins,card_position);   
    }



    /*void PrologClient::associate_turn_to_player(const Instance &Player_instance, const Instance &Turn_instance) {
        PrologQueryProxy bdgs = _pl.query(
                "is_in_turn('" + _NAMESPACE + Player_instance.get_class(9) + "','" + _NAMESPACE + Turn_instance.get_class() + "')");
    }


    void PrologClient::create_Player_instance(const player &player) {
        Instance player("Player", "Player_" + std::to_String(player._num));
        save(player);
    }


    void PrologClient::create_turn_instance(const turn &turn) {
        Instance turn_i("Turn", "Turn_" + std::to_String(turn._timeInstance) + turn._round._name);
        turn._turn = turn_i;
        save(turn_i);
        //Associate a player
        ObjectProperty associatedToPlayer("associatedToPlayer", turn._player._name);
        save_property(turn_i, associatedToPlayer);
        //Associate a timeStampt
        Instance timeStamp("TimeStamp", "TimeStamp" + std::to_string(turn._timeInstance));
        save(timeStamp);
        assert_timeStamp_property(turn_i, timeStamp);
    }


    void PrologClient::create_round_instance(const round &round) {
        Instance round("Round", "Round_" + std::to_String(round._timeInstance));
        save(round);
    }


    void PrologClient::associate_current_turn_to_round(const Instance &turn, const round &round) {
        delete_instance(round._current_turn);
        round._current_turn = turn;
        ObjectProperty hasCurrentTurn("hasCurrentTurn", turn);
        save_property(round._round, hasCurrentTurn);
    }*/


}