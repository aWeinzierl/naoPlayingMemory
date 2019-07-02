#pragma once



class CardPosition {
    unsigned int _x;
    unsigned int _y;

    CardPosition(unsigned int x, unsigned int y) noexcept ;

    unsigned int get_x() const noexcept ;
    unsigned int get_y() const noexcept ;
};
