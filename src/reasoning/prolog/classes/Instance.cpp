#include "Instance.h"

namespace reasoning{

    const std::string &Instance::get_name() const noexcept {
        return _name;
    }

    const std::string &Instance::get_class() const noexcept {
        return _class;
    }

    Instance::Instance(const std::string &classType, std::string name)
    : _class(classType), _name(std::move(name)){
    }
}