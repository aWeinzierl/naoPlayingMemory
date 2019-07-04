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


    //saves intance
    void PrologClient::save(const Instance &instance) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_costom_instance_from_class('" + _NAMESPACE + instance.get_class() + ",_," + instance.get_name() +
                "', ObjInst)");
    }


    /*void PrologClient::assert_property(const Instance &instance, const ObjectProperty &property) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_assert('" + _NAMESPACE + instance.get_class()+ "_" + instance.get_name() + "','" + _NAMESPACE +
                property.get_name() + "','" + property.get_value() + "')");
        /* (instance, propertyName, propertyValue/Instance) 
    }*/

    void PrologClient::save_TimeStamp_property(const Instance &instance, const Instance &timeStamp) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_assert('" + _NAMESPACE + instance.get_class() + "_" + instance.get_name() + "','" + _NAMESPACE +
                "hasTimeStamp','" + timeStamp._name + "')");
        /*(instance, hasTimeStamp, TimeStamp_x) */
    }

    void PrologClient::save_CardPosition_property(const Instance &instance, const Instance &position) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_assert('" + _NAMESPACE + instance.get_class() + "_" + instance.get_name() + "','" + _NAMESPACE +
                "hasPositin','" + position.get_name() + "')");
        /*(instance, hasPosition, intance_from_cardposition) */
    }

    void PrologClient::delete_instance(const Instance &instance) {
        PrologQueryProxy bdgs = _pl.query(
                "del('" + _NAMESPACE + instance._class + "','" + _NAMESPACE + instance._class + "',[])");

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
        std::uniform_int_distribution<size_t> dist{0, _ALLOWED_CHARS_FOR_RANDOM_NAMES.length() - 1};
        std::string ret;

        std::generate_n(
                std::back_inserter(ret),
                length,
                [&] { return _ALLOWED_CHARS_FOR_RANDOM_NAMES[dist(gen)]; });
        return ret;

    }

    //creates instance of turn_Card
    void
    PrologClient::save_turn_card(const std::string &player_name, const Card &action, uint time_instant) {
        
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

        //create card_position instance and the coordinates
        Instance card_position("CardPosition", "Card_Position_"+to_string(action._position._x)+to_string(action._position._y));
        save(card_position);
                        //auto hasXCoordinate = ("hasXCoordinate", po);
        DataProperty x_pos("hasXCoordinate", std::to_string(action._position._x));
        DataProperty y_pos("hasYCoordinate", std::to_string(action._position._y));
        save_property_value(card_position,x_pos);
        save_property_value(card_position,y_pos);
        save_property(turn_card,card_position);



    }

    //turn equal cards
    void PrologClient::save(const knownCard &Card1,const knownCard &Card2, uint timeInstant) {
        save(action._Card1, timeInstant);
        save(action._Card2, timeInstant);
        Instance turn_equal_cards("TurnEqualCards", "Turn_Equal_Cards");
        save(turn_equal_cards);
        //assert Timestamp_instance as property from turn_equal_cards
        Instance timeStamp("TimeStamp", "TimeStamp" + std::to_string(timeInstant));
        save(timeStamp);
        save_TimeStamp_property(turn_equal_cards, timeStamp);

    }


    //pair equal_Cards    


    Instance PrologClient::create_time_stamp(uint time_instant) {
        return Instance("TimeStamp", std::to_string(time_instant));
    }

    nonstd::optional<Instance> PrologClient::position_already_exists(const Position &position) {
        //TODO: check if position exists
        //if it exists return instance of it
        return nonstd::nullopt;
    }

    nonstd::optional<Instance> PrologClient::concealed_card_already_exists(const ConcealedCard &ConcealedCard) {
        //TODO: check if position exists
        //if it exists return instance of it
        return nonstd::nullopt;
    }

    //Turns 2 unknown cards
    void PrologClient::save(const unknownCard &Card1, const unknownCard &Card2,uint TimeInstance) {
        save(Card1._unknown_card, timeInstant);
        save(Card2._unknown_card, timeInstant);
        Instance turn_two_unknown_cards("TurnTwoUnknownCards", "Turn_Two_Unknown_Cards");
        save(turn_two_unknown_cards);
        //assert Timestamp_instance as property from turn_equal_cards
        Instance timeStamp("TimeStamp", "TimeStamp" + std::to_string(timeInstant));
        save(timeStamp);
        save_TimeStamp_property(turn_two_unknown_cards, timeStamp);
    }


// change count by Marker Id -->ConcealedCard[]-->position,Id-->
    void PrologClient::instantiate_one_unknowncard(const ConcealedCard &ConcealedCard) {#
        if (!concealed_card_already_exists(ConcealedCard).hasValue()){
            return;
        }
        //sdt::string name<<"unknown_card_"+std::to_string(i)+std:to_string(j);
        Instance  unknown_card_ins[ConcealedCard._id]("UnknownCard", "Card_" + std::to_string(i) + std:to_string(j));
        save(unknown_card_ins[ConcealedCard._id]);
        Instance card_position("CardPosition", "Card_position" + std::to_string(i) + std:to_string(j));
        DataProperty x_pos("hasXCoordinate", x_pos1);
        unknown_card_ins[ConcealedCard._id]._position._x = i;
        DataProperty y_pos("hasYCoordinate", y_pos1);
        unknown_card_ins[ConcealedCard._id]._position._y = j;
        save_property_value(card_position, x_pos);
        save_property_value(card_position, y_pos);
        save_property_value(unknown_card_ins[ConcealedCard._id],card_position);

   
    }




    void PrologClient::associate_turn_to_player(const Instance &Player_instance, const Instance &Turn_instance) {
        PrologQueryProxy bdgs = _pl.query(
                "is_in_turn('" + _NAMESPACE + Player_instance._class + "','" + _NAMESPACE + Turn_instance._class + "')");
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


    void PrologClient::associate_current_turn_to_round(const Instance &turn, const round round) {
        delete_instance(round._current_turn);
        round._current_turn = turn;
        ObjectProperty hasCurrentTurn("hasCurrentTurn", turn);
        save_property(round._round, hasCurrentTurn);
    }

    void PrologClient::create_nao() {
        Instance nao("Player","Nao");
        save(nao);
        //PrologQueryProxy bdgs = _pl.query("rdf_costom_instance_from_class('" + _NAMESPACE + "Player_Nao, ObjInst')");
    }


    void PrologClient::create_game(int game_num) {
        Instance game("MemoryGame","Game_"+std::to_string(game_num));
        save(game);
    }

    /*
    void PrologClient::assert_MemoryGame_status(const Instance& status){


    }
    */




    /*
    void PrologClient::create_action_takecards(){
        

    }
    */



    /*
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

    void PrologClient::logQueryResult(json_prolog::PrologQueryProxy &bdgs) const {
        for (auto const &bdg : bdgs) {
            std::cout << bdg["ObjInst"] << "\n";
        }
    }

    */

    /*void PrologClient::create_ArucoToIdObjectMappings() {}

    }*/

}