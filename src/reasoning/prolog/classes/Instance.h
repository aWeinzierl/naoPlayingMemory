#pragma once

#include <string>

namespace reasoning {

    struct Instance {
    private:
        std::string _class;
        std::string _name;

    public:
        Instance(const std::string &classType, std::string name);

        const std::string &get_class() const noexcept;

        const std::string &get_name() const noexcept;
    };
}