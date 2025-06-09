#include "LQR.h"

LQR::LQR() : ref(0)
{
}

float LQR::compute(float angle, float rate)
{
    // TODO: Implement state-space control law
    return 0.0f;
}

void LQR::setReference(float r)
{
    ref = r;
}
