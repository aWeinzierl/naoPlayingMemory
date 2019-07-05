#include <utility>

#include "Instance.h"

namespace reasoning{

    Instance::Instance( std::string classType, std::string name) 
    : _class(std::move(classType)), _name(std::move(name)){
    }

    const std::string &Instance::get_name() const noexcept {
        return _name;
    }

    const std::string &Instance::get_class() const noexcept {
        return _class;
    }

   
}