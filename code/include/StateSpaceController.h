#ifndef STATE_SPACE_CONTROLLER_H
#define STATE_SPACE_CONTROLLER_H

class StateSpaceController {
public:
  StateSpaceController();
  void setParameters(float *A, float *B, float *C, float *K, int n);
  float compute(float *state, float reference);

private:
  float *A, *B, *C, *K;
  int n;
};

#endif // STATE_SPACE_CONTROLLER_H
