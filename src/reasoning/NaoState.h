#pragma once

struct NaoState {
    bool _is_bored;

    bool _is_my_turn;

    bool is_bored();
    bool is_my_turn();

    NaoState();
};
