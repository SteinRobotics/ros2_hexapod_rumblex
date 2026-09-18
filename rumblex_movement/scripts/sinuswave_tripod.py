#!/usr/bin/env python3
"""Plot clipped sine and cosine waves over the range [0, 2π]."""

import matplotlib.pyplot as plt 
import numpy as np  

plt.rcParams.update({"font.size": 12})

def main() -> None:
    
    x_vals = np.linspace(0.0, 2.0 * np.pi, 1000)
    phase = np.sin(x_vals)

    cos_vals = np.clip(np.cos(x_vals), 0.0, None)  
    cos_vals *= cos_vals   
    cos_plus_pi_vals = np.clip(np.cos(x_vals + np.pi), 0.0, None)
    cos_plus_pi_vals *= cos_plus_pi_vals  

    sin_start_vals = np.clip(np.sin(x_vals), 0.0, None) # for startup
    sin_start_vals *= sin_start_vals
    sin_start_vals[x_vals > (np.pi / 4.0)] = cos_vals[x_vals > (np.pi / 4.0)]
    sin_start_vals[x_vals > np.pi ] = 0

    # first possible stopping point using first Tripod
    sin_stopping_first_vals = np.sin(x_vals) # for stopping, no clipping!!!!! Value is 7π/4!!!! 
    sin_stopping_first_vals *= sin_stopping_first_vals
    sin_stopping_first_vals[x_vals <= (7 / 4) * np.pi ] = cos_vals[x_vals <= (7 / 4) * np.pi ]

    # second possible stopping point using second Tripod
    sin_stopping_second_vals = np.sin(x_vals) # for stopping, no clipping!!!!! Value is 3/4π!!!
    sin_stopping_second_vals *= sin_stopping_second_vals
    sin_stopping_second_vals[x_vals <= (3 / 4) * np.pi ] = cos_plus_pi_vals[x_vals <= (3 / 4) * np.pi ]
    sin_stopping_second_vals[x_vals > np.pi ] = 0

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(x_vals, sin_start_vals, label="LiftLeg FirstTripod startup", color="#16a34a", linewidth=2, linestyle="--")
    ax.plot(x_vals, sin_stopping_first_vals, label="LiftLeg FirstTripod stopping", color="#1c610e", linewidth=2, linestyle="--")
    ax.plot(x_vals, cos_vals, label="LiftLeg FirstTripod", color="#25eb7b", linewidth=2)
    ax.plot(x_vals, sin_stopping_second_vals, label="LiftLeg SecondTripod stopping", color="#1c3a8a", linewidth=2, linestyle="--")
    ax.plot(x_vals, cos_plus_pi_vals, label="LiftLeg SecondTripod", color="#2563eb", linewidth=2)
    ax.plot(x_vals, phase, label="Phase (sin(x))", color="#eb4034", linewidth=3)



    tick_labels = ["0", "π/4", "π/2", "3/4π", "π", "5/4π", "3/2π", "7/4π", "2π"]
    ax.set_xticks([0.0, np.pi / 4.0, np.pi / 2.0, 3.0 * np.pi / 4.0, np.pi, 5.0 * np.pi / 4.0, 3.0 * np.pi / 2.0, 7.0 * np.pi / 4.0, 2.0 * np.pi])
    ax.set_xticklabels(tick_labels, fontsize=12)
    step = np.pi / 2.0
    num_lines = int((2.0 * np.pi) / step) + 1
    for idx in range(num_lines):
        x = idx * step
        ax.axvline(x=x, color="#292c30", linestyle="--", linewidth=0.9)


    ax.set_title("Clipped sine and cosine over one cycle", fontsize=12)
    ax.set_xlabel("x (radians)", fontsize=12)
    ax.set_ylabel("Amplitude", fontsize=12)
    ax.set_xlim(0.0, 2.0 * np.pi)
    ax.set_ylim(-1.05, 1.05)
    ax.grid(True, linestyle="--", alpha=0.4)


    ax.tick_params(axis="both", labelsize=12)
    ax.legend(loc="upper right", fontsize=12)

    fig.tight_layout()
    plt.show(block=True)


if __name__ == "__main__":
    main()