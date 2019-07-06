#include "FilterRecognizeTurn.h"

namespace reasoning {
    FilterRecognizeTurn::FilterRecognizeTurn(const unsigned int size) noexcept:
    buffer(size){
    }


    bool update(const state state) {
        //filter auf die que laufen lassen
       
        state comparator = buffer[0];
        for(int i=1;i<5;i++){
            if(comparator!=buffer[i]){
                return false;
            }
        }
        return true;
    }

}