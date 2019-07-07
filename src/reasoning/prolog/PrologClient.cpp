#include "PrologClient.h"

#include <string>
#include <exception>
#include <iostream>

namespace reasoning {
    constexpr char PrologClient::_ALLOWED_CHARS_FOR_RANDOM_NAMES[];

    //saves intance
    void PrologClient::save(const Instance &instance) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_costom_instance_from_class('" + _NAMESPACE + instance.get_class() + "',_," + instance.get_name() +
                ", ObjInst)");
    }

    void PrologClient::delete_instance(const Instance &instance) {
        PrologQueryProxy bdgs = _pl.query(
                "del('" + _NAMESPACE + instance.get_class() + "','" + _NAMESPACE + instance.get_class() + "',[])");

    }


    //Creates ObjectProperty
    void PrologClient::save_property(const Instance &instance_of_interest, const ObjectProperty &property) {
        _pl.query(
                "rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" + instance_of_interest.get_name() +
                "','" + _NAMESPACE + property.get_name() +
                "','" + _NAMESPACE + property.get_value().get_class() + "_" + property.get_value().get_name() + "')");
    }

    //creates DataPorperty
    void PrologClient::save_property(const Instance &instance_of_interest, const DataProperty<unsigned int> &property) {
        _pl.query(
                "rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" + instance_of_interest.get_name() +
                "','" + _NAMESPACE + property.get_name() +
                "','" + std::to_string(property.get_value()) + "')");

        std::cout<<"rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" + instance_of_interest.get_name() +
                "','" + _NAMESPACE + property.get_name() +
                "','" + std::to_string(property.get_value()) + "')"<<std::endl;
    }

    void PrologClient::save_property(const Instance &instance_of_interest, const DataProperty<std::string> &property) {
        _pl.query(
                "rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" + instance_of_interest.get_name() +
                "','" + _NAMESPACE + property.get_name() +
                "','" + property.get_value() + "')");
    }

    bool PrologClient::instance_already_exists(const Instance &instance) {
        auto bdgs = _pl.query(
                "owl_individual_of('" + _NAMESPACE + instance.get_class() + "_" + instance.get_name()
                + "','http://knowrob.org/kb/knowrob.owl#TimePoint')"
        );

        return bdgs.begin() == bdgs.begin();
    }

    std::string PrologClient::generate_random_string(uint length) noexcept {
        std::mt19937_64 gen{std::random_device()()};
        std::uniform_int_distribution<size_t> dist{0, PrologClient::_ALLOWED_CHARS_FOR_RANDOM_NAMES_LEN - 1};
        std::string ret;

        std::generate_n(
                std::back_inserter(ret),
                length,
                [&] { return PrologClient::_ALLOWED_CHARS_FOR_RANDOM_NAMES[dist(gen)]; });
        return ret;
    }

    //creates instance of turn_Card action  
    void
    PrologClient::save_action(const std::string &player_name, const RevealCardAction &reveal_card_action,
                              unsigned int time_instant) {

        auto player = player_already_exists(player_name).value();

        //create time_stamp
        auto time_stamp = create_time_stamp(time_instant);
        if (!instance_already_exists(time_stamp)) {
            save(time_stamp);
            DataProperty<unsigned int> time_stamp_prop("hasTime", time_instant);
            save_property(time_stamp, time_stamp_prop);
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
        auto card = card_already_exists(reveal_card_action.get_id());
        if (!card.has_value()) {
            throw new std::logic_error("Card_already_exist");
        }
        //create class instance

        //associate class to card
        DataProperty<std::string> class_prop("hasClass", reveal_card_action.get_class());
        save_property(card.value(), class_prop);

        ObjectProperty card_prop("hasCard", card.value());
        save_property(turn_card, card_prop);

        ObjectProperty hasAction("hasAction", turn_card);

        save_property(player, hasAction);
    }

    Instance PrologClient::create_time_stamp(uint time_instant) noexcept {
        return Instance("TimeStamp", std::to_string(time_instant));
    }

    nonstd::optional<Instance> PrologClient::card_already_exists(const uint id) {

        PrologQueryProxy bdgs = _pl.query(
                "owl_has(Instance,'" + _NAMESPACE + "hasMarkerId' ,'" + std::to_string(id) + "')");
        if (bdgs.begin() == bdgs.end()) {
            return nonstd::nullopt;
        }
        auto instance_bdg = *(bdgs.begin());
        Instance instance("Card", instance_bdg["Instance"]);
        return instance;
    }

    nonstd::optional<Instance> PrologClient::card_already_exists(const CardPosition &card_position) {
        PrologQueryProxy bdgs = _pl.query(
                "owl_has(Position,'" + _NAMESPACE + "hasXCoordinate' ,'" + std::to_string(card_position.get_x()) + "'),"
                                                                                                                   "owl_has(Position,'" +
                _NAMESPACE + "hasYCoordinate' ,'" + std::to_string(card_position.get_y()) + "'),"
                                                                                            "rdf_has(CardInstance, '" +
                _NAMESPACE + "hasPosition" + "',Position)");
        if (bdgs.begin() == bdgs.end()) {
            return nonstd::nullopt;
        }
        auto instance_bdg = *(bdgs.begin());
        Instance instance("Card", instance_bdg["CardInstance"]);
        return instance;
    }

    void PrologClient::save(const ConcealedCard &concealed_card) {

        auto card = card_already_exists(concealed_card.get_id());
        if (!card.has_value()) {
            throw new std::logic_error("Card_already_exist");
            //exit function----->
        }
        Instance unknown_card_ins("UnknownCard", std::to_string(concealed_card.get_id()));
        save(unknown_card_ins);
        Instance card_pos("CardPosition", "CardPosition" + std::to_string(concealed_card.get_position().get_x()) +
                                          std::to_string(concealed_card.get_position().get_x()));
        ObjectProperty card_position("hasPosition", card_pos);
        DataProperty<unsigned int> x_pos("hasXCoordinate", concealed_card.get_position().get_x());
        DataProperty<unsigned int> y_pos("hasYCoordinate", concealed_card.get_position().get_y());
        save_property(card_pos, x_pos);
        save_property(card_pos, y_pos);
        save_property(unknown_card_ins, card_position);
    }

    void
    PrologClient::save_action(const std::string &player_name, StartGameAction start_game_action,
                              unsigned int time_instant) {

        auto player = player_already_exists(player_name).value();

        //create time_stamp
        auto time_stamp = create_time_stamp(time_instant);
        if (!instance_already_exists(time_stamp)) {
            save(time_stamp);
        }
        //create action(TurnOneCard) instance
        Instance start_game("StartGame", "StartGame_" + generate_random_string(_RANDOM_NAME_LENGTH));
        while (instance_already_exists(start_game)) {
            start_game = Instance("StartGame", "StartGame_" + generate_random_string(_RANDOM_NAME_LENGTH));
        }
        save(start_game);

        DataProperty<unsigned int> time_stamp_prop("hasTime", time_instant);
        save_property(time_stamp, time_stamp_prop);

        ObjectProperty player_has_action("hasAction", start_game);
        save_property(player, player_has_action);
    }

    void PrologClient::test_prolog_query() {
        for (int i = 11; i > 0; --i) {
            auto ts = create_time_stamp(i);
            save(ts);
            DataProperty<unsigned int> time_stamp("hasTime", i);
            save_property(ts, time_stamp);
        }


        PrologQueryProxy bdgs = _pl.query("all_times(Time)");
        for (PrologQueryProxy::iterator it = bdgs.begin(); it != bdgs.end(); it++) {
            PrologBindings bdg = *it;
            std::cout << "Time = " << bdg["Time"] << std::endl;
        }
        //create Start game instance
        Instance start_game("StartGame","1");
        save(start_game);
        auto ts = create_time_stamp(50);
        save(ts);



        DataProperty<unsigned int> time_stamp("hasTime",50);

        save_property(ts,time_stamp);
        ObjectProperty init("hasTimeStamp",ts);
        std::cout<<"Im here4"<<std::endl;
        save_property(start_game,init);


        PrologQueryProxy bdgs2 = _pl.query("start_game(GameStatus)");
        for(PrologQueryProxy::iterator it=bdgs2.begin();it != bdgs2.end(); it++){
            PrologBindings bdg= *it;
            std::cout<< "GameStatus = "<< bdg["GameStatus"] << std::endl;
        }

        PrologQueryProxy bdgs3 = _pl.query("canPlayAttempt('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#1', Action)");
        for(PrologQueryProxy::iterator it=bdgs3.begin();it != bdgs3.end(); it++){
            PrologBindings bdg= *it;
            std::cout<< "Action = "<< bdg["Action"] << std::endl;
        }

        PrologQueryProxy bdgs3 = _pl.query("canPlayAttempt('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#1', Action)");
        for(PrologQueryProxy::iterator it=bdgs3.begin();it != bdgs3.end(); it++){
            PrologBindings bdg= *it;
            std::cout<< "Action = "<< bdg["Action"] << std::endl;
        }
        
    }

    nonstd::optional<Instance> PrologClient::player_already_exists(const std::string &player_name) {

        PrologQueryProxy bdgs = _pl.query(
                "owl_has(Instance,'" + _NAMESPACE + "hasName' ,'" + player_name + "'),"
                                                                                  "rdfs_instance_of(Instance, Player)");
        if (bdgs.begin() == bdgs.end()) {
            return nonstd::nullopt;
        }
        auto instance_bdg = *(bdgs.begin());
        Instance instance("Player", instance_bdg["Instance"]);
        return instance;

    }

    void PrologClient::save_action(const std::string &player_name, const RemoveCardAction &remove_card_action,
                                   unsigned int time_instant) {

        auto player = player_already_exists(player_name).value();

        //create time_stamp
        auto time_stamp = create_time_stamp(time_instant);
        if (!instance_already_exists(time_stamp)) {
            save(time_stamp);
        }
        DataProperty<unsigned int> time_stamp_prop("hasTime", time_instant);
        save_property(time_stamp, time_stamp_prop);


        //create action(TurnOneCard) instance
        Instance remove_card_instance("RemoveCard", generate_random_string(_RANDOM_NAME_LENGTH));
        while (instance_already_exists(remove_card_instance)) {
            remove_card_instance = Instance("StartGame", generate_random_string(_RANDOM_NAME_LENGTH));
        }
        save(remove_card_instance);

        ObjectProperty time_stamp_ob_prop("hasTimeStamp", time_stamp);
        save_property(remove_card_instance, time_stamp_ob_prop);

        auto card_instance = card_already_exists(remove_card_action).value();
        ObjectProperty has_card_prop("hasCard", card_instance);
        save_property(remove_card_instance, has_card_prop);

        ObjectProperty player_has_action("hasAction", remove_card_instance);
        save_property(player, player_has_action);
    }
}