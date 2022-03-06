#pragma once

#include "components/states/abstract_state.h"
#include "ecs.h"

class StateComponent : public ecs::Component<StateComponent>
{
public:
    StateComponent()           = default;
    ~StateComponent() override = default;

    template <typename T>
    void ChangeState()
    {
        static_assert(std::is_base_of<AbstractState, T>::value);
        state.reset(new T(this->componentId));
    };

    std::shared_ptr<AbstractState> GetState() { return state; }

private:
    std::shared_ptr<AbstractState> state;
};
