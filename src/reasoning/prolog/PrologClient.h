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

    void create_instance (const Instance& instance);
    void assert_property(const Instance& instance,const Property& property);
    void save_TimeStamp_property(const Instance &instance, const Instance &timeStamp);
    void save_CardPosition_property(const Instance &instance, const Instance &position);
    void delete_instance(const Instance& instance);
    void save(const TurnCard &action, uint timeInstant);
    void save(const EqualCard &action, uint timeInstant);
    void save(const unknownCard &Card1, consta unknownCard& Card2,uint TimeInstance);
    void instantiate_one_unknowncard(uint i, uint j, uint count);
    void instantiate_all_unknownCards();
    void associate_turn_to_player(const Instance &Player_instance, const Instance &Turn_instance);
    void save(const player &player);
    void save(const turn &turn);
    void save(const round &round);
    void associate_current_turn_to_round(const Instance &turn, const round round);


};