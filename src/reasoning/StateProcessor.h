#pragma once

#include <string>
#include <queue>
#include <unordered_map>
#include <mutex>

#include <boost/circular_buffer.hpp>

#include "FilterRecognizeTurn.h"
#include "prolog/PrologClient.h"
#include "State.h"

namespace reasoning {

    struct ActionDetections {
        std::vector <RevealCardAction> reveal_card;
        std::vector <CoverCardAction> cover_card;
        std::vector <RemoveCardAction> remove_card;
    };

    struct Hash {
        /// calculate a unique, deterministic hash of a card position
        /// \param cardPosition
        /// \return hash
        std::size_t operator()(const CardPosition &cardPosition) const {
            return (cardPosition.get_x() << 1u) ^ cardPosition.get_y();
        }
    };

    /// StateProcessor - detects actions regarding cards on a 3*4 grid
    class StateProcessor {

        ActionDetections _actions;
        std::mutex _actions_mutex;
        std::unordered_map <CardPosition, FilterRecognizeTurn, Hash> _position_to_filter;


        /// factory to create a position to filter map for the grid
        /// \param persistence amount of updates are needed to distinguish a change from noise
        /// \return map of the grid positions to the corresponding filter at that position
        std::unordered_map <CardPosition, FilterRecognizeTurn, Hash> generate_filters(unsigned int persistence);

        /// @brief pass a new state to a filter at a certain position and check if it produces an action
        /// \param card_position position in the grid
        /// \param state current state of the card
        /// \return true if a filter has detected a change
        bool state_update_triggers_filter(const CardPosition &card_position, State state);

    public:
        /// constructs a directly usable state processor
        /// \param persistence amount of updates are needed to distinguish a change from noise
        explicit StateProcessor(unsigned int persistence);

        /// supply a new set of cards to provide a update on the state of the board
        /// \param concealed_card cards which are not visible
        /// \param exposed_card cards of which the class can be seen
        /// \param unknown no card or different state at the given position
        void process_new_state(const std::vector <ConcealedCard> &concealed_card,
                               const std::vector <ExposedCard> &exposed_card,
                               const std::vector <CardPosition> &unknown);

        /// returns all the actions which have been detected since the last call. Resets the found actions.
        /// \return actions which have been detected
        ActionDetections retrieve_actions();

        /// reset the actions which have been found since the last reset (e.g. by retrieve_actions)
        void reset_found_actions();
    };
}