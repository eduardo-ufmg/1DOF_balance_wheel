import matplotlib.pyplot as plt
import re
import numpy as np
from scipy.optimize import curve_fit

def parse_kd_data(filepath):
    """
    Parses the Kd experiment data file.

    Args:
        filepath (str): The path to the kd.txt file.

    Returns:
        dict: A dictionary where keys are PWM values (int) and
              values are lists of tuples (time_ms, angle_rad).
    """
    data = {}
    current_pwm = None
    pwm_pattern = re.compile(r"PWM: (\d+)")
    data_pattern = re.compile(r"([\d\.]+),\s*([\d\.-]+)")

    try:
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                pwm_match = pwm_pattern.match(line)
                if pwm_match:
                    current_pwm = int(pwm_match.group(1))
                    if current_pwm not in data:
                        data[current_pwm] = []
                    continue

                data_match = data_pattern.match(line)
                if data_match and current_pwm is not None:
                    try:
                        time_ms = float(data_match.group(1))
                        angle_rad = float(data_match.group(2))
                        data[current_pwm].append((time_ms, angle_rad))
                    except ValueError:
                        print(f"Skipping malformed data line for PWM {current_pwm}: {line}")
                        continue
    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None
    return data

def motion_model(t_seconds, theta0, omega0_rps, alpha_rps2):
    """
    Equation of motion with constant angular acceleration.
    theta(t) = theta0 + omega0*t + 0.5*alpha*t^2

    Args:
        t_seconds (array-like): Time in seconds (s).
        theta0 (float): Initial angle in radians (rad).
        omega0_rps (float): Initial angular velocity in radians per second (rad/s).
        alpha_rps2 (float): Angular acceleration in radians per second squared (rad/s^2).

    Returns:
        array-like: Angle in radians at time t.
    """
    return theta0 + omega0_rps * t_seconds + 0.5 * alpha_rps2 * t_seconds**2

def linear_model(x, m, c):
    """
    Linear equation y = mx + c.

    Args:
        x (array-like): Independent variable.
        m (float): Slope (angular coefficient).
        c (float): Intercept.

    Returns:
        array-like: Dependent variable.
    """
    return m * x + c

def plot_kd_data(kd_data):
    """
    Plots the time vs. angle for each PWM step, fits for acceleration (in rad/s^2),
    then plots acceleration vs. PWM, fits for Kd, and displays the results.

    Args:
        kd_data (dict): Data parsed by parse_kd_data.
    """
    if not kd_data:
        print("No data to plot.")
        return

    num_pwm_steps = len(kd_data)
    if num_pwm_steps == 0:
        print("No PWM steps found in data.")
        return

    cols = int(num_pwm_steps**0.5)
    rows = (num_pwm_steps + cols - 1) // cols

    fig_angle_time, axs_angle_time = plt.subplots(rows, cols, figsize=(cols * 6, rows * 5), squeeze=False)
    axs_angle_time = axs_angle_time.flatten()

    sorted_pwm_values = sorted(kd_data.keys())
    
    print("\n--- Computed Angular Accelerations (rad/s^2) ---")
    
    fitted_accelerations = []
    actual_pwm_values_for_fit = []

    for i, pwm in enumerate(sorted_pwm_values):
        step_data = kd_data[pwm]
        
        ax = axs_angle_time[i]
        ax.set_title(f"PWM: {pwm}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (rad)")
        ax.grid(True)

        if not step_data:
            print(f"PWM {pwm}: No data points, skipping plot and fit.")
            ax.text(0.5, 0.5, "No data", horizontalalignment='center', verticalalignment='center')
            continue

        times_ms = np.array([d[0] for d in step_data])
        times_s = times_ms / 1000.0
        angles = np.array([d[1] for d in step_data])

        ax.plot(times_s, angles, marker='.', linestyle='-', label='Data')

        if len(step_data) >= 3:
            try:
                initial_guess = [angles[0], 0.0, 0.0] 
                params, covariance = curve_fit(motion_model, times_s, angles, p0=initial_guess, maxfev=5000)
                theta0_fit, omega0_fit_rps, alpha_fit_rps2 = params
                
                print(f"PWM {pwm}: Acceleration (α) = {alpha_fit_rps2:.8e} rad/s²")
                fitted_accelerations.append(alpha_fit_rps2)
                actual_pwm_values_for_fit.append(pwm)

                angles_fit = motion_model(times_s, theta0_fit, omega0_fit_rps, alpha_fit_rps2)
                fit_label = f'Fit (α={alpha_fit_rps2:.4e} rad/s²)'
                ax.plot(times_s, angles_fit, linestyle='--', color='red', label=fit_label)

            except RuntimeError:
                print(f"PWM {pwm}: Could not fit a curve for angle vs. time. Plotting raw data only.")
                ax.text(0.05, 0.95, "Fit failed (angle-time)",
                            transform=ax.transAxes, fontsize=9,
                            verticalalignment='top', color='red')
            except Exception as e:
                print(f"PWM {pwm}: Error during angle-time fitting: {e}. Plotting raw data only.")
                ax.text(0.05, 0.95, "Fit error (angle-time)",
                            transform=ax.transAxes, fontsize=9,
                            verticalalignment='top', color='red')
        else:
            print(f"PWM {pwm}: Not enough data points ({len(step_data)}) for angle-time fit. Plotting raw data only.")
            ax.text(0.05, 0.95, "Too few points for fit (angle-time)",
                        transform=ax.transAxes, fontsize=9,
                        verticalalignment='top', color='orange')
        
        if ax.has_data():
             ax.legend(loc='best', fontsize='small')

    for j in range(i + 1, len(axs_angle_time)): # Hide unused subplots for angle-time
        fig_angle_time.delaxes(axs_angle_time[j])

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig_angle_time.suptitle("Kd Experiment: Angle vs. Time with Acceleration Fit (rad/s²)", y=0.99, fontsize=16)
    plt.show()

    # --- Fit Acceleration vs. PWM ---
    if len(actual_pwm_values_for_fit) >= 2 and len(fitted_accelerations) == len(actual_pwm_values_for_fit):
        pwm_array = np.array(actual_pwm_values_for_fit)
        accel_array = np.array(fitted_accelerations)

        try:
            # Initial guess for Kd (slope) and intercept
            # Slope: (max_accel - min_accel) / (max_pwm - min_pwm) if more than one point, else 1
            # Intercept: min_accel if points exist, else 0
            if len(pwm_array) > 1 and (pwm_array.max() - pwm_array.min()) != 0:
                 slope_guess = (accel_array.max() - accel_array.min()) / (pwm_array.max() - pwm_array.min())
            else:
                 slope_guess = 1.0
            intercept_guess = accel_array.min() if len(accel_array) > 0 else 0.0
            
            popt_kd, pcov_kd = curve_fit(linear_model, pwm_array, accel_array, p0=[slope_guess, intercept_guess])
            Kd, intercept_kd = popt_kd
            
            print(f"\n--- Kd Fit (Acceleration vs. PWM) ---")
            print(f"Kd (Angular Coefficient) = {Kd:.8e} (rad/s²)/PWM")
            print(f"Intercept = {intercept_kd:.8e} rad/s²")

            accel_fit_kd = linear_model(pwm_array, Kd, intercept_kd)

            fig_accel_pwm, ax_accel_pwm = plt.subplots(figsize=(8, 6))
            ax_accel_pwm.plot(pwm_array, accel_array, marker='o', linestyle='None', label='Calculated Accelerations')
            ax_accel_pwm.plot(pwm_array, accel_fit_kd, linestyle='--', color='green', 
                              label=f'Linear Fit (Kd={Kd:.4e})')
            ax_accel_pwm.set_xlabel("PWM")
            ax_accel_pwm.set_ylabel("Angular Acceleration (rad/s²)")
            ax_accel_pwm.set_title("Acceleration vs. PWM with Kd Fit")
            ax_accel_pwm.legend()
            ax_accel_pwm.grid(True)
            plt.tight_layout()
            plt.show()

        except RuntimeError:
            print("\nCould not fit a line to Acceleration vs. PWM data.")
        except Exception as e:
            print(f"\nAn error occurred during Kd fitting: {e}")
    else:
        print("\nNot enough data points to fit Acceleration vs. PWM.")


if __name__ == "__main__":
    filepath = 'data.txt'
    parsed_data = parse_kd_data(filepath)

    if parsed_data:
        plot_kd_data(parsed_data)