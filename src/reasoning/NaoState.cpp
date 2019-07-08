
#include "NaoState.h"

NaoState::NaoState(){
    _is_bored= true;
    _is_my_turn = true;
}

bool NaoState::is_bored() {
    return _is_bored;
}
