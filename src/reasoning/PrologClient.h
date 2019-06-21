#pragma once

#include <string>
#include <iostream>
#include <unordered_map>

#include <json_prolog/prolog.h>

enum class DIRECTION{
    UP,
    DOWN,
    LEFT,
    RIGHT,
    AWAY,
    CLOSER,
};


class PrologClient {
    std::unordered_map<std::string, unsigned int> class_to_current_id_associations_;
    json_prolog::Prolog _pl;



public:

    struct Instance{
        std::string _class;
        uint _id;
    public:
        Instance(const std::string& classType, uint id);
        const std::string& Get_class() const noexcept;
        uint Get_id() const noexcept;
    };

    PrologClient();

    uint CreateTypeInstance(const std::string &associatedClass);

    void Register_motion_for_object(const Instance &instance, uint timeInstant, DIRECTION direction);

    void Register_start_time_for_instance(const Instance &instance, uint timeInstant);

    bool Time_point_already_exists(uint timeInstant);

    void Create_time_point_if_not_exists(uint timeInstant);

    void logQueryResult(json_prolog::PrologQueryProxy &bdgs) const;

    void PrintMovementsOfInstance(Instance instance, DIRECTION direction);
};