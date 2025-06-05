#pragma once

class StateSpaceController {
public:
  StateSpaceController();
  float compute(float angle, float rate);
  void setReference(float ref);

private:
  float ref;
  // Add state-space matrices as needed
};
