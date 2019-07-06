#pragma once

namespace reasoning{

    struct CardPosition {
    private:
        unsigned int _x;
        unsigned int _y;
    public:
        CardPosition(unsigned int x, unsigned int y) noexcept ;

        unsigned int get_x() const noexcept ;
        unsigned int get_y() const noexcept ;
       
    };
    
    bool operator==(const CardPosition& a, const CardPosition& b)  ;
}