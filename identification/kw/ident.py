import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import pandas as pd
import os

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

def fit_and_plot_pwm_speed(filepath):
    """
    Reads PWM vs. Speed data from a file, fits a linear model to determine Kw,
    and plots the data with the fit.

    Args:
        filepath (str): The path to the data file (e.g., 'data.txt').
    """
    try:
        # Read the data using pandas
        df = pd.read_csv(filepath)
        
        # Ensure column names are stripped of leading/trailing whitespace
        df.columns = df.columns.str.strip()
        
        # Verify that the expected columns are present
        if 'PWM' not in df.columns or 'Speed (rad/s)' not in df.columns:
            print(f"Error: The file {filepath} must contain 'PWM' and 'Speed (rad/s)' columns.")
            print(f"Found columns: {df.columns.tolist()}")
            return

        pwm_values = df['PWM'].values.astype(float)
        speed_values = df['Speed (rad/s)'].values.astype(float)

    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        return
    except pd.errors.EmptyDataError:
        print(f"Error: The file {filepath} is empty.")
        return
    except Exception as e:
        print(f"Error reading or parsing data from {filepath}: {e}")
        return

    if len(pwm_values) < 2 or len(speed_values) < 2 or len(pwm_values) != len(speed_values):
        print("Not enough data points or mismatched data lengths to perform a linear fit.")
        return

    try:
        # Initial guess for Kw (slope) and intercept
        if (pwm_values.max() - pwm_values.min()) != 0:
            slope_guess = (speed_values.max() - speed_values.min()) / (pwm_values.max() - pwm_values.min())
        else:
            slope_guess = 1.0
        intercept_guess = speed_values.min() if len(speed_values) > 0 else 0.0
        
        params, covariance = curve_fit(linear_model, pwm_values, speed_values, p0=[slope_guess, intercept_guess])
        Kw, intercept = params

        print(f"\n--- Kw Fit (Speed vs. PWM) from {filepath} ---")
        print(f"Kw (Angular Coefficient) = {Kw:.8e} (rad/s)/PWM")
        print(f"Intercept = {intercept:.8e} rad/s")

        speed_fit = linear_model(pwm_values, Kw, intercept)

        plt.figure(figsize=(10, 6))
        plt.plot(pwm_values, speed_values, marker='o', linestyle='None', label='Measured Data')
        plt.plot(pwm_values, speed_fit, linestyle='--', color='red',
                 label=f'Linear Fit (Kw={Kw:.4e})')
        plt.xlabel("PWM")
        plt.ylabel("Speed (rad/s)")
        plt.title(f"Speed vs. PWM with Kw Fit (from {os.path.basename(filepath)})")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    except RuntimeError:
        print("\nCould not fit a line to Speed vs. PWM data.")
    except Exception as e:
        print(f"\nAn error occurred during Kw fitting: {e}")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_name = "data.txt"
    data_file_path = os.path.join(script_dir, file_name)
    
    fit_and_plot_pwm_speed(data_file_path)