#include <utility>

#include "DataProperty.h"

namespace reasoning{

    template<typename DataType>
    DataProperty<DataType>::DataProperty(std::string name, DataType value) noexcept
    : _name(std::move(name)), _value(value){
    }

    template<typename DataType>
    const std::string &DataProperty<DataType>::get_name() const noexcept {
        return _name;
    }
    template<typename DataType>
    const DataType &DataProperty<DataType>::get_value() const noexcept {
        return _value;
    }

    template class DataProperty<std::string>;
    template class DataProperty<unsigned int>;
}

