#pragma once

#include <string>

#include <json_prolog/prolog.h>



class PrologClient {
    json_prolog::Prolog _pl;


    const std::string _NAMESPACE = "https://github.com/aWeinzierl/naoPlayingMemory/blob/master/owl/Robot.owl#";

public:

    struct Instance{
    private:
        std::string _class;
        std::string _name;
        //std::string _timeStamp;

    public:
        Instance(const std::string& classType, std::string name);
        const std::string& Get_class() const noexcept;
        uint Get_id() const noexcept;
    };
    struct Property{
        private:
            std::string _name;
            std::string _value;
        public:
        Property(std::string name, std::string  value);
        const std::string& Get_name() const noexcept;
    }
    PrologClient();

    uint CreateTypeInstance(const std::string &associatedClass);

    void test_prolog_query();
    void create_instance (const Instance& instance);
    void assert_property(const Instance& instance,const Property& property);
    void assert_timeStamp_property(const Instance& instance,const Instance& timeStamp);
    void assert_CardPosition_property(const Instance& instance,const Instance& position);
    void delete_instance(const Instance& instance);
    void save_action_TurnCard(const TurnCard& action,uint timeInstant);
    void save_action_turn_equal_cards(const EqualCard& action,uint timeInstant);
    void save_action_turn_two_unkown_cards(const unknownCard& Card1, consta unknownCard& Card2,uint TimeInstance);
    void instanciate_one_unknowncard(uint i,uint j,uint count);
    void instanciate_all_unknownCards();
    void associateTurnToPlayer(const Instance& Player_instance,const Instance& Turn_instance);
    void create_Player_instance(const player& player);
    void create_turn_instance(const turn& turn);
    void create_round_instance(const round& round);
    void associate_currente_turn__to_round(const Instance& turn,const round round);


};