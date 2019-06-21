#include "PrologClient.h"

PrologClient::PrologClient() {
        class_to_current_id_associations_ = {
                {"Carrot",  0},
                {"Donut",   0},
                {"HotWing", 0},
        };
    }

uint PrologClient::CreateTypeInstance(const std::string &associatedClass) {
    auto supposedId = class_to_current_id_associations_.find(associatedClass)->second + 1;

    json_prolog::PrologQueryProxy bdgs = _pl.query(
            "rdf_costom_instance_from_class('http://knowrob.org/kb/knowrob.owl#" + associatedClass + "',_," +
            std::to_string(supposedId) + ",ObjInst)");

    class_to_current_id_associations_[associatedClass] = supposedId;

    logQueryResult(bdgs);

    return supposedId;
}

void PrologClient::logQueryResult(json_prolog::PrologQueryProxy &bdgs) const {
    for (auto const &bdg : bdgs) {
        std::cout << bdg["ObjInst"] << "\n";
    }
}

void PrologClient::Register_motion_for_object(const Instance &instance, uint timeInstant,
                                              DIRECTION direction) {
    std::string movement;
    if (direction==DIRECTION::LEFT){
        movement = "MoveLeft";
    } else if (direction == DIRECTION::RIGHT){
        movement = "MoveRight";
    } else if(direction == DIRECTION::CLOSER){
        movement="MoveToward";
    } else if (direction == DIRECTION::AWAY){
        movement= "MoveForward";
    }else if (direction == DIRECTION::UP){
        movement = "MoveUp";
    }else if (direction == DIRECTION::DOWN) {
        movement = "MoveDown";
    }

    auto bdgs = _pl.query(
            "rdf_assert('http://knowrob.org/kb/Nao_computables.owl#" + instance.Get_class() +"_"+ std::to_string(instance.Get_id()) +
    "','http://knowrob.org/kb/Nao_computables.owl#"+ movement +"','http://knowrob.org/kb/knowrob.owl#TimePoint_"+std::to_string(timeInstant)+ "')");
}

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
            "owl_individual_of('http://knowrob.org/kb/knowrob.owl#TimePoint_" + std::to_string(timeInstant) + "','http://knowrob.org/kb/knowrob.owl#TimePoint')"
            );

    return bdgs.begin()==bdgs.begin();
}

void PrologClient::PrintMovementsOfInstance(PrologClient::Instance instance, DIRECTION direction) {
    std::string movement;
    if (direction==DIRECTION::LEFT){
        movement = "MoveLeft";
    } else if (direction == DIRECTION::RIGHT){
        movement = "MoveRight";
    } else if(direction == DIRECTION::CLOSER){
        movement="MoveToward";
    } else if (direction == DIRECTION::AWAY){
        movement= "MoveForward";
    }else if (direction == DIRECTION::UP){
        movement = "MoveUp";
    }else if (direction == DIRECTION::DOWN) {
        movement = "MoveDown";
    }

    auto bdgs = _pl.query(
            "rdf_has('http://knowrob.org/kb/Nao_computables.owl#"
            + instance.Get_class() + "_" + std::to_string(instance.Get_id()) + "','http://knowrob.org/kb/Nao_computables.owl#"+ movement + "',ObjInst)");

    std::cout << "History for this direction: ";
    logQueryResult(bdgs);
}

PrologClient::Instance::Instance(const std::string &classType, uint id) : _class(classType),_id(id) {
}

const std::string &PrologClient::Instance::Get_class() const noexcept {
    return _class;
}

uint PrologClient::Instance::Get_id() const noexcept {
    return _id;
}
