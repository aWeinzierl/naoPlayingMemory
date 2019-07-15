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
        std::cout << "rdf_assert('" + _NAMESPACE + instance_of_interest.get_class() + "_" +
                     instance_of_interest.get_name() +
                     "','" + _NAMESPACE + property.get_name() +
                     "','" + property.get_value() + "')" << std::endl;


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
        std::uniform_int_distribution <size_t> dist{0, PrologClient::_ALLOWED_CHARS_FOR_RANDOM_NAMES_LEN - 1};
        std::string ret;

        std::generate_n(
                std::back_inserter(ret),
                length,
                [&] { return PrologClient::_ALLOWED_CHARS_FOR_RANDOM_NAMES[dist(gen)]; });
        return ret;
    }

    //creates instance of turn_Card action  

    Instance PrologClient::create_time_stamp(uint time_instant) noexcept {
        return Instance("TimeStamp", std::to_string(time_instant));
    }

    nonstd::optional <Instance> PrologClient::card_already_exists(const uint id) {


        PrologQueryProxy bdgs = _pl.query(
                "rdf_has(Instance,'" + _NAMESPACE + "hasMarkerId' ,'" + std::to_string(id) + "')");

        if (bdgs.begin() == bdgs.end()) {
            return nonstd::nullopt;
        }
        auto instance_bdg = *(bdgs.begin());
        std::string card_name = instance_bdg["Instance"];
        Instance instance("Card", card_name.erase(0, 5 + _NAMESPACE.length()));
        return instance;
    }

    nonstd::optional <Instance> PrologClient::card_already_exists(const CardPosition &card_position) {
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
        Instance instance("Card", card_name.erase(0, 5 + _NAMESPACE.length()));
        return instance;
    }

    void PrologClient::save(const ConcealedCard &concealed_card) {

        auto card = card_already_exists(concealed_card.get_id());

        if (card.has_value()) {
            std::cout << "We are here" << std::endl;
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
        save_property(unknown_card_ins, card_id);
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


    nonstd::optional <Instance> PrologClient::player_already_exists(const std::string &player_name) {

        PrologQueryProxy bdgs = _pl.query(
                "rdfs_individual_of(Instance, '" + _NAMESPACE + "Player'),"
                                                                "rdf_has(Instance,'" + _NAMESPACE + "hasName' ," +
                player_name + ")");
        if (bdgs.begin() == bdgs.end()) {
            return nonstd::nullopt;
        }
        auto instance_bdg = *(bdgs.begin());
        std::string player_str = instance_bdg["Instance"];
        Instance instance("Player", player_str.erase(0, 7 + _NAMESPACE.length()));
        return instance;

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


    std::vector <reasoning::ConcealedCard> PrologClient::decide_action() {
        std::cout << "Print out DeletedCards " << std::endl;
        PrologQueryProxy bdgs5 = _pl.query(
                "rdf_has(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#isDeleted',true)");
        for (PrologQueryProxy::iterator it = bdgs5.begin(); it != bdgs5.end(); it++) {
            PrologBindings bdg = *it;
            std::cout << "Card1 = " << bdg["Card"] << std::endl;

        }


        std::cout << "Im gonna decide which action to do" << std::endl;
        auto bdg = _pl.once("act_id_pos(C1id,C1X,C1Y,C2id,C2X,C2Y)");
        std::cout << "Cards1:" << bdg["C1id"] << std::endl;
        std::cout << "Cx:" << bdg["C1X"] << std::endl;
        std::cout << "Cy:" << bdg["C1Y"] << std::endl;
        std::cout << "Cards2:" << bdg["C2id"] << std::endl;
        std::cout << "Cx:" << bdg["C2X"] << std::endl;
        std::cout << "Cy:" << bdg["C2Y"] << std::endl;
        std::string card2_id = bdg["C2id"];
        std::string card2_x = bdg["C2X"];
        std::string card2_y = bdg["C2X"];
        std::string card1_id = bdg["C1id"];
        std::string card1_x = bdg["C1X"];
        std::string card1_y = bdg["C1Y"];


        if (card2_id[0] == '_') {
            return {
                    ConcealedCard((unsigned int) std::stoi(card1_id),
                                  CardPosition(
                                          (unsigned int) std::stoi(bdg["C1X"].toString()),
                                          (unsigned int) std::stoi(bdg["C1Y"].toString())))
            };
        } else {
            return {
                    ConcealedCard((unsigned int) std::stoi(card1_id),
                                  CardPosition(
                                          (unsigned int) std::stoi(bdg["C1X"].toString()),
                                          (unsigned int) std::stoi(bdg["C1Y"].toString()))),
                    ConcealedCard((unsigned int) std::stoi(card2_id),
                                  CardPosition(
                                          (unsigned int) std::stoi(bdg["C2X"].toString()),
                                          (unsigned int) std::stoi(bdg["C2Y"].toString())))

            };
        }
    }

    nonstd::optional <ConcealedCard> PrologClient::search_paired_card(const ConcealedCard &concealed_card) {


        std::cout << "Print out ClassDatabase " << std::endl;
        PrologQueryProxy bdgs5 = _pl.query(
                "rdf_has(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass',Class)");
        for (PrologQueryProxy::iterator it = bdgs5.begin(); it != bdgs5.end(); it++) {
            PrologBindings bdg = *it;
            std::cout << "Card1 = " << bdg["Card"] << std::endl;
            std::cout << "Class = " << bdg["Class"] << std::endl;
        }


        std::cout << "Im gonna search for a paired Card for " << concealed_card.get_position().get_x()
                  << concealed_card.get_position().get_x() << std::endl;
        auto bdgs = _pl.query(
                "findTwoEqualCards_pos('" + _NAMESPACE + "Card_" + std::to_string(concealed_card.get_id()) +
                "', C2,C2id,C2X,C2Y)");

        if (bdgs.begin() == bdgs.end()) {
            std::cout << "No Pair" << std::endl;
            return {};
        }


        auto bdg = *(bdgs.begin());
        std::string card2_id = bdg["C2id"];

        return nonstd::optional<ConcealedCard>({
                                                       (unsigned int) std::stoi(card2_id),
                                                       CardPosition(
                                                               (unsigned int) std::stoi(bdg["C2X"].toString()),
                                                               (unsigned int) std::stoi(bdg["C2Y"].toString()))
                                               });


    }

    nonstd::optional <ConcealedCard>
    PrologClient::search_if_paired_card(const ConcealedCard &concealed_card, const ConcealedCard &concealed_card2) {

        std::cout << "Print out ClassDatabase " << std::endl;
        PrologQueryProxy bdgs5 = _pl.query(
                "rdf_has(Card,'https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#hasClass',Class)");
        for (PrologQueryProxy::iterator it = bdgs5.begin(); it != bdgs5.end(); it++) {
            PrologBindings bdg = *it;
            std::cout << "Card1 = " << bdg["Card"] << std::endl;
            std::cout << "Class = " << bdg["Class"] << std::endl;
        }


        std::cout << "Im gonna search for a paired Card for " << concealed_card.get_position().get_x()
                  << concealed_card.get_position().get_x() << std::endl;
        auto bdgs = _pl.query(
                "findTwoEqualCards_pos('" + _NAMESPACE + "Card_" + std::to_string(concealed_card.get_id()) +
                "','" + _NAMESPACE + "Card_" + std::to_string(concealed_card2.get_id()) + "',C2id,C2X,C2Y)");

        if (bdgs.begin() == bdgs.end()) {
            std::cout << "No Pair" << std::endl;
            return {};
        }


        auto bdg = *(bdgs.begin());
        std::string card2_id = bdg["C2id"];

        return nonstd::optional<ConcealedCard>({
                                                       (unsigned int) std::stoi(card2_id),
                                                       CardPosition(
                                                               (unsigned int) std::stoi(bdg["C2X"].toString()),
                                                               (unsigned int) std::stoi(bdg["C2Y"].toString()))
                                               });


    }

    nonstd::optional <ConcealedCard> PrologClient::search_random_card() {
        std::cout << "Im gonna search for a random Card" << std::endl;
        auto bdgs = _pl.query("pickRandomCard_pos(C1,CX,CY)");
        PrologBindings bdg = *(bdgs.begin());
        std::string card2_id = bdg["C1"].toString().erase(0, _NAMESPACE.length() + 5);

        std::cout << "The random card is: " << bdg["C1"] << std::endl;
        std::cout << "x: " << bdg["CX"] << "  y: " << bdg["CY"] << std::endl;


        if (bdg.begin() != bdg.end()) {
            return nonstd::optional<ConcealedCard>({
                                                           (unsigned int) std::stoi(card2_id),
                                                           CardPosition(
                                                                   (unsigned int) std::stoi(bdg["CX"].toString()),
                                                                   (unsigned int) std::stoi(bdg["CY"].toString()))
                                                   });
        } else {
            return {};
        }
    }

    bool PrologClient::are_cards_equal(const unsigned int id1, const unsigned int id2) {
        //findTwoEqualCards_with_id(C1id,C2id,C1,C2)
        std::cout << "I am comparring the opponets turned cards" << std::endl;
        auto bdgs = _pl.query(
                "findTwoEqualCards_with_id('" + std::to_string(id1) + "','" + std::to_string(id2) + "',C1,C2)");
        if (bdgs.begin() == bdgs.end()) {
            return false;
        }
        return true;
    }

    void PrologClient::delete_cards(const unsigned int id1, const unsigned int id2) {
        PrologQueryProxy bdgs = _pl.query("delete_cards('" + std::to_string(id1) + "','" + std::to_string(id2) + "')");
        //auto bdgs13 = _pl.once("delete_cards('2','3')");
    }

    void PrologClient::save_action(const RevealCardAction &reveal_card_action) {
        PrologQueryProxy bdgs = _pl.query(
                "giveClass_pos('" + std::to_string(reveal_card_action.get_position().get_x()) + "','" +
                std::to_string(reveal_card_action.get_position().get_y()) + "','" + reveal_card_action.get_class() +
                "')");
    }

    void PrologClient::reset() {
        //reset_prolog(C)
        std::cout << "Reseting Prolog " << std::endl;
        PrologQueryProxy bdgs5 = _pl.query("reset_prolog(C)");
        for (PrologQueryProxy::iterator it = bdgs5.begin(); it != bdgs5.end(); it++) {
            PrologBindings bdg = *it;
            std::cout << "Card  = " << bdg["C"] << std::endl;
        }

    }
}