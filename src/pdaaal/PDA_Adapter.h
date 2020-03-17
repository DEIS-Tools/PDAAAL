//
// Created by Morten on 17-03-2020.
//

#ifndef PDAAAL_PDA_ADAPTER_H
#define PDAAAL_PDA_ADAPTER_H

#include "TypedPDA.h"

namespace pdaaal {

    template<typename T, typename W = void, typename C = std::less<W>>
    class PDA_Adapter : public TypedPDA<T,W,C> {
    public:
        explicit PDA_Adapter(const std::unordered_set<T> &all_labels) : TypedPDA<T,W,C>{all_labels} {};

        using state_t = typename TypedPDA<T,W,C>::template WPDA<W,C>::state_t;

        [[nodiscard]] size_t initial() const { return _initial_id; }
        [[nodiscard]] const std::vector<uint32_t>& initial_stack() const { return _initial_stack; }
        [[nodiscard]] size_t terminal() const { return 0; }

    protected:
        virtual void finalize() {
            _initial_id = this->states().size();
            this->states_mutable().emplace_back(_initial);
            for (auto& s : this->states_mutable()) {
                if (!s._pre.empty() && s._pre.front() == 0) {
                    s._pre.erase(std::begin(s._pre));
                    s._pre.emplace_back(_initial_id);
                }
            }
            for (auto& r : this->states_mutable()[_initial_id]._rules) {
                r._labels.clear();
                r._labels.merge(false, _initial_stack, std::numeric_limits<size_t>::max());
            }
        }

    private:
        const std::vector<uint32_t> _initial_stack{std::numeric_limits<uint32_t>::max() - 2}; // std::numeric_limits<uint32_t>::max() is reserved for epsilon transitions (and max-1 for a flag in trace_t).
        size_t _initial_id = 0;
        state_t _initial;
    };
}

#endif //PDAAAL_PDA_ADAPTER_H
