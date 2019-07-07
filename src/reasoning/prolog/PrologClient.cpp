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


    }

    void PrologClient::save_property(const Instance &instance_of_interest, const DataProperty<std::string> &property) {
        _pl.query(
                "rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" + instance_of_interest.get_name() +
                "','" + _NAMESPACE + property.get_name() +
                "','" + property.get_value() + "')");

        std::cout<<"rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" + instance_of_interest.get_name() +
                   "','" + _NAMESPACE + property.get_name() +
                   "','" + property.get_value() + "')"<<std::endl;
    }

    bool PrologClient::instance_already_exists(const Instance &instance) {

        auto bdgs = _pl.query(
                "rdfs_individual_of('" + _NAMESPACE + instance.get_class() + "_" + instance.get_name()
                + "','" + _NAMESPACE + instance.get_class() + "')"
        );

        return bdgs.begin() == bdgs.end();
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
        while (!instance_already_exists(turn_card)) {
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
            throw new std::logic_error("Card does not exist");
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
                "rdf_has(Instance,'" + _NAMESPACE + "hasMarkerId' ,'" + std::to_string(id) + "')");

        if (bdgs.begin() == bdgs.end()) {
            return nonstd::nullopt;
        }
        auto instance_bdg = *(bdgs.begin());
        std::string card_name = instance_bdg["Instance"];
        Instance instance("Card", card_name.erase(0,5+_NAMESPACE.length()));
        return instance;
    }

    nonstd::optional<Instance> PrologClient::card_already_exists(const CardPosition &card_position) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_has(Position,'" + _NAMESPACE + "hasXCoordinate' ,'" + std::to_string(card_position.get_x()) + "'),"
                                                                                                                   "rdf_has(Position,'" +
                _NAMESPACE + "hasYCoordinate' ,'" + std::to_string(card_position.get_y()) + "'),"
                                                                                            "rdf_has(CardInstance, '" +
                _NAMESPACE + "hasPosition" + "',Position)");
        if (bdgs.begin() == bdgs.end()) {
            return nonstd::nullopt;
        }
        auto instance_bdg = *(bdgs.begin());
        std::string card_name = instance_bdg["CardInstance"];
        Instance instance("Card", card_name.erase(0,5+_NAMESPACE.length()));
        return instance;
    }

    void PrologClient::save(const ConcealedCard &concealed_card) {

        auto card = card_already_exists(concealed_card.get_id());

        if (card.has_value()) {
            std::cout<<"We are here"<<std::endl;
            throw new std::logic_error("Card_already_exist");
            //exit function----->
        }
        Instance unknown_card_ins("Card", std::to_string(concealed_card.get_id()));
        save(unknown_card_ins);
        Instance card_pos("CardPosition", std::to_string(concealed_card.get_position().get_x()) +
                                          std::to_string(concealed_card.get_position().get_y()));
        ObjectProperty card_position("hasPosition", card_pos);
        DataProperty<unsigned int> x_pos("hasXCoordinate", concealed_card.get_position().get_x());
        DataProperty<unsigned int> y_pos("hasYCoordinate", concealed_card.get_position().get_y());
        save_property(card_pos, x_pos);
        save_property(card_pos, y_pos);
        save_property(unknown_card_ins, card_position);
        DataProperty<unsigned int> card_id("hasMarkerId", concealed_card.get_id());
        save_property(unknown_card_ins,card_id);
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



    nonstd::optional<Instance> PrologClient::player_already_exists(const std::string &player_name) {

        PrologQueryProxy bdgs = _pl.query(
                "rdfs_individual_of(Instance, '" + _NAMESPACE + "Player'),"
                "rdf_has(Instance,'" + _NAMESPACE + "hasName' ," + player_name + ")");
        if (bdgs.begin() == bdgs.end()) {
            return nonstd::nullopt;
        }
        auto instance_bdg = *(bdgs.begin());
        std::string player_str = instance_bdg["Instance"];
        Instance instance("Player", player_str.erase(0,7+_NAMESPACE.length()));
        return instance;

    }

    void PrologClient::save_action(const std::string &player_name, const RemoveCardAction &remove_card_action,
                                   unsigned int time_instant) {
        auto card_instance = card_already_exists(remove_card_action);

        auto player = player_already_exists(player_name).value();

        //create time_stamp
        auto time_stamp = create_time_stamp(time_instant);
        if (!instance_already_exists(time_stamp)) {
            save(time_stamp);
        }
        //create action(TurnOneCard) instance
        Instance remove_card_instance("RemoveCard", generate_random_string(_RANDOM_NAME_LENGTH));
        while (instance_already_exists(remove_card_instance)) {
            remove_card_instance = Instance("StartGame", generate_random_string(_RANDOM_NAME_LENGTH));
        }
        save(remove_card_instance);

        DataProperty<unsigned int> time_stamp_prop("hasTime", time_instant);
        save_property(time_stamp, time_stamp_prop);


        ObjectProperty player_has_action("hasAction", remove_card_instance);
        save_property(player, player_has_action);

        //TODO associate removed card
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
        auto ts = create_time_stamp(5);
        save(ts);



        DataProperty<unsigned int> time_stamp("hasTime",5);

        save_property(ts,time_stamp);
        ObjectProperty init("hasTimeStamp",ts);
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
        CardPosition c1_pos(1,2);
        CardPosition c2_pos(2,3);
        ConcealedCard C1_c(1,c1_pos);
        ConcealedCard C2_c(2,c2_pos);
        save(C1_c);
        save(C2_c);

        std::cout<<"concealed cards were created"<<std::endl;

        ExposedCard C1("banana",1, c1_pos);
        ExposedCard C2("banana",2, c2_pos);
        save_action("Nao",C1,7);
        save_action("Nao",C2,9);

        std::cout<<"exposedcards were created"<<std::endl;

        PrologQueryProxy bdgs4 = _pl.query("findTwoEqualCards(Card1, Card2)");
        for(PrologQueryProxy::iterator it=bdgs4.begin();it != bdgs4.end(); it++){
            PrologBindings bdg= *it;
            std::cout<< "Card1 = "<< bdg["Card1"] << std::endl;
            std::cout<< "Card2 = "<< bdg["Card2"] << std::endl;
        }
        
    }
}