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
    constexpr char PrologClient::_ALLOWED_CHARS_FOR_RANDOM_NAMES[];

    //saves intance
    void PrologClient::save(const Instance &instance) {
        PrologQueryProxy bdgs = _pl.query(
                "rdf_costom_instance_from_class('" + _NAMESPACE + instance.get_class() + ",_," + instance.get_name() +
                "', ObjInst)");
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
                "','" + to_string(property.get_value()) + "')");
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
        auto card = card_already_exists(Exposed_Card.get_id());
        if (!card.has_value()) {
            throw new logic_error("Card_already_exist");
        }
        //create class instance

        //associate class to card
        DataProperty<std::string> class_prop("hasClass", Exposed_Card.get_class());
        save_property(card.value(), class_prop);

        ObjectProperty card_prop("hasCard", card.value());
        save_property(turn_card, card_prop);

        ObjectProperty hasAction("hasAction", turn_card);
        //TODO delete_instance(delete_old_action);
        save_property(player, hasAction);
    }

    Instance PrologClient::create_time_stamp(uint time_instant) noexcept {
        return Instance("TimeStamp", std::to_string(time_instant));
    }

    nonstd::optional<Instance> PrologClient::position_already_exists(const CardPosition &position) {
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

        auto card = card_already_exists(Concealed_Card.get_id());
        if (!card.has_value()){
            throw new logic_error("Card_already_exist");
            //exit function----->
        }
        //sdt::string name<<"unknown_card_"+std::to_string(i)+std:to_string(j);
        Instance  unknown_card_ins("UnknownCard", "Card_" + to_string(Concealed_Card.get_id()));
        save(unknown_card_ins);
        Instance card_pos("CardPosition","CardPosition" + to_string(Concealed_Card.get_position().get_x())+to_string(Concealed_Card.get_position().get_x()));
        ObjectProperty card_position("hasPosition", card_pos);
        DataProperty<unsigned int> x_pos("hasXCoordinate", Concealed_Card.get_position().get_x());
        DataProperty<unsigned int> y_pos("hasYCoordinate", Concealed_Card.get_position().get_y());
        save_property(card_pos, x_pos);
        save_property(card_pos, y_pos);
        save_property(unknown_card_ins, card_position);
    }
}