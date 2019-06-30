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

    public:
        Instance(const std::string& classType, std::string name);
        const std::string& Get_class() const noexcept;
        uint Get_id() const noexcept;
    };

    PrologClient();

    uint CreateTypeInstance(const std::string &associatedClass);

    void test_prolog_query();


};