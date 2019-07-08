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


        //create Start game instance with a Tim
        Instance start_game("StartGame","1");
        save(start_game);
        auto ts = create_time_stamp(11);
        save(ts);
        DataProperty<unsigned int> time_stamp("hasTime",11);
        save_property(ts,time_stamp);
        ObjectProperty init("hasTimeStamp",ts);
        save_property(start_game,init);


        PrologQueryProxy bdgs2 = _pl.query("start_game(GameStatus)");
        for(PrologQueryProxy::iterator it=bdgs2.begin();it != bdgs2.end(); it++){
            PrologBindings bdg= *it;
            std::cout<< "GameStatus = "<< bdg["GameStatus"] << std::endl;
        }
     
        CardPosition c1_pos(1,2);
        CardPosition c2_pos(2,3);
        ConcealedCard C1_c(1,c1_pos);
        ConcealedCard C2_c(2,c2_pos);
        save(C1_c);
        save(C2_c);
        std::cout<<"---------Testing CanPlayAttempt ---------"<<std::endl;

        PrologQueryProxy bdgs3 = _pl.query("canPlayAttempt('https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#1', Action, Card)");
        for(PrologQueryProxy::iterator it=bdgs3.begin();it != bdgs3.end(); it++){
            PrologBindings bdg= *it;
            std::cout<< "Action = "<< bdg["Action"] << std::endl;
            std::cout<<"Card: "<<bdg["Card"]<<std::endl;
            break;

        }

        
        std::cout<<"---------Testing Act ---------"<<std::endl;
        std::cout<<"Testing act(Card1,Card2) with only  Unknown Cards"<<std::endl;
        PrologQueryProxy bdgs4 = _pl.query("act(Card1,Card2)");
        for(PrologQueryProxy::iterator it=bdgs4.begin();it != bdgs4.end(); it++){
            PrologBindings bdg= *it;
            std::cout<<"Possible Cards"<< std::endl;
            std::cout<<"Card1 = "<< bdg["Card1"] << std::endl;
            std::cout<<"Card2 = "<< bdg["Card2"] << std::endl;
            
            
        }   

        ExposedCard C1("banana",1, c1_pos);
        ExposedCard C2("banana",2, c2_pos);
        save_action("Nao",C1,7);
        save_action("Nao",C2,9);

        std::cout<<"Testing act(Card1,Card2) with only  Known Cards"<<std::endl;
        PrologQueryProxy bdgs5 = _pl.query("act(Card1,Card2)");
        for(PrologQueryProxy::iterator it=bdgs5.begin();it != bdgs5.end(); it++){
            PrologBindings bdg= *it;
            std::cout<<"Possible Cards"<< std::endl;
            std::cout<<"Card1 = "<< bdg["Card1"] << std::endl;
            std::cout<<"Card2 = "<< bdg["Card2"] << std::endl;
                 
        }

        CardPosition c3_pos(3,2);
        CardPosition c4_pos(1,3);
        ConcealedCard C3_c(3,c3_pos);
        ConcealedCard C4_c(4,c4_pos);
        save(C3_c);
        save(C4_c);

        std::cout<<"Testing act(Card1,Card2) with both  Known and Unknown Cards"<<std::endl;
        auto bdgs6 = _pl.once("act(Card1,Card2)");
        std::cout<<"1: "<<bdgs6["Card1"]<<std::endl;
        std::cout<<"2: "<<bdgs6["Card2"]<<std::endl;
        /*for(PrologQueryProxy::iterator it=bdgs6.begin();it != bdgs6.end(); it++){
            PrologBindings bdg= *it;
            std::cout<<"Possible Cards"<< std::endl;
            std::cout<<"Card1 = "<< bdg["Card1"] << std::endl;
            std::cout<<"Card2 = "<< bdg["Card2"] << std::endl;
            break;
        }*/
        ExposedCard C3("apple",3, c3_pos);
        save_action("Nao",C3,9);


        std::cout<<"---------Finished Act ---------"<<std::endl;
        std::cout<<"-----Testing pickRandomCard_Class_Or_Without(Card)---------"<<std::endl;
        std::cout<<"Testing with 1 pair, one known and one unknown"<<std::endl;       
        auto bdgs7 = _pl.once("pickRandomCard_Class_Or_Without(Card)");
        std::cout<<"Possible Card"<< std::endl;
        std::cout<<"Random Card = "<< bdgs7["Card"] << std::endl;
        /*for(PrologQueryProxy::iterator it=bdgs7.begin();it != bdgs7.end(); it++){
            PrologBindings bdg= *it;
            std::cout<<"Possible Cards"<< std::endl;
            std::cout<<"Random Card = "<< bdg["Card"] << std::endl;
    
        }*/
        
        ExposedCard C4("mango",4, c4_pos);
        save_action("Nao",C4,10);

        std::cout<<"Testing with 1 pair and 2 known "<<std::endl;
        auto bdgs8 = _pl.once("pickRandomCard_Class_Or_Without(Card)");
        std::cout<<"Possible Card"<< std::endl;
        std::cout<<"Random Card = "<< bdgs8["Card"] << std::endl;

        /*for(PrologQueryProxy::iterator it=bdgs8.begin();it != bdgs8.end(); it++){
            PrologBindings bdg= *it;
            std::cout<<"Possible Cards"<< std::endl;
            std::cout<<"Random Card = "<< bdg["Card"] << std::endl;
    
        
        }*/
        std::cout<<"----- Finished ----- Testing pickRandomCard_Class_Or_Without(Card)---------"<<std::endl;

        std::cout<<"-----------testing instance creation---------------"<<std::endl;

        std::string test_card_name = "50";

        /*
        PrologQueryProxy bdgs9 = _pl.query("unknownCardInstanciation("+ test_card_name +",CardInstance)");
        for (PrologQueryProxy::iterator it = bdgs9.begin(); it != bdgs9.end(); it++) {
            PrologBindings bdg = *it;
            std::cout << "Instance = " << bdg["CardInstance"] << std::endl;
        }*/
        
        /*
        Results:
        -----Testing pickRandomCard(Card)---------
        Testing with 1 pair, one known and one unknown
        Possible Cards
        Random Card = https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#Card_4
        Testing with 1 pair and 2 known                                                                 //it can't choose a card that has a class......!!!!!!!!
        ----- Finished ----- Testing pickRandomCard(Card)---------
        */

        
    }


    void PrologClient::save_action(const std::string &player_name, const RemoveCardAction &remove_card_action,
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
        Instance remove_card_instance("RemoveCard", generate_random_string(_RANDOM_NAME_LENGTH));
        while (instance_already_exists(remove_card_instance)) {
            remove_card_instance = Instance("RemoveCard", generate_random_string(_RANDOM_NAME_LENGTH));
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

    void PrologClient::save_action(const std::string &player_name, const CoverCardAction &cover_card_action,
                                   unsigned int time_instant) {

        auto time_stamp = create_time_stamp(time_instant);
        if (!instance_already_exists(time_stamp)) {
            save(time_stamp);
            DataProperty<unsigned int> time_stamp_prop("hasTime", time_instant);
            save_property(time_stamp, time_stamp_prop);
        }

        Instance cover_card_instance("CoverCard", generate_random_string(_RANDOM_NAME_LENGTH));
        while (instance_already_exists(cover_card_instance)) {
            cover_card_instance = Instance("CoverCard", generate_random_string(_RANDOM_NAME_LENGTH));
        }
        save(cover_card_instance);

        ObjectProperty time_stamp_ob_prop("hasTimeStamp", time_stamp);
        save_property(cover_card_instance, time_stamp_ob_prop);

        auto card_instance = card_already_exists(cover_card_action.get_id()).value();
        ObjectProperty has_card_prop("hasCard", card_instance);
        save_property(cover_card_instance, has_card_prop);

        auto player = player_already_exists(player_name).value();
        ObjectProperty player_has_action("hasAction", cover_card_instance);
        save_property(player, player_has_action);
    }
     
   
    std::vector<unsigned int> PrologClient::decide_action()
    {
        std::cout<<"Im gonna decide which action to do"<<std::endl;
        //TODO  try ONCE instead of query
        PrologQueryProxy bdgs6 = _pl.query("act_id(Card1_id,Card2_id)");
        for(PrologQueryProxy::iterator it=bdgs6.begin();it != bdgs6.end(); it++){
            PrologBindings bdg= *it;
            std::cout<<"Possible Cards"<< std::endl;
            std::cout<<"Card1 = "<< bdg["Card1_id"] << std::endl;
            std::string card1_id = bdg["Card1_id"];
                //return this Card to the Nao
            std::cout<<"Card2 = "<< bdg["Card2_id"] << std::endl;
            std::string card2_id = bdg["CarCard2_idd2"];
            /*if(card2_id[0].compare("_")){ //check if card is give back or empty ex:"_16"
                //return this Card to the Nao
                std::cout<<"THe second card dosen't exist"<<std::endl;
            }*/

            //TODO give back the value of both cards

        }
    }

    void PrologClient::reset() {
        _pl = json_prolog::Prolog();
    }
}