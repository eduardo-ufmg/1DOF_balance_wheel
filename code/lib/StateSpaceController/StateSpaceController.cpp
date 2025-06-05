#include "StateSpaceController.h"

StateSpaceController::StateSpaceController() : ref(0)
{
}

float StateSpaceController::compute(float angle, float rate)
{
    // TODO: Implement state-space control law
    return 0.0f;
}

void StateSpaceController::setReference(float r)
{
    ref = r;
}
