// The interface must be transparent to the user, meaning that the user should not need to know about the underlying implementation details.
// No parameter is to be hardcoded, all parameters must be passed through the constructor.
// The class name is to be `Motor`.
// Implement a interface for the 24H404H070 nidec motor. Those are its specifications:
// - Brushless DC motor
// - Brake is active low
// - PWM is active low, meaning that the duty cycle refers to the time the signal is low
// - There is a direction pin
// - The expected frequency for the PWM signal is 20 kHz, but it may be set by the user
// - The expected resolution for the PWM signal is 8 bits, but it may be set by the user