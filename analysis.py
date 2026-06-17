import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_and_clean_data(file_path):
    if not os.path.exists(file_path):
        print(f"❌ Error: The file '{file_path}' was not found.")
        sys.exit(1)

    print(f"📦 Loading dataset: {file_path}")
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip()
    return df

def find_column(df, keywords):
    for col in df.columns:
        if any(kw.lower() in col.lower() for kw in keywords):
            return col
    return None

def main():
    # --- CONFIGURATION ---
    csv_filename = "powertrain_telemetry_dump.csv"
    # ---------------------

    df = load_and_clean_data(csv_filename)

    # 1. Dynamically locate columns
    time_col = find_column(df, ['time', 'timestamp'])
    vel_col = find_column(df, ['velocity', 'speed', 'carvelocityx'])
    acc_col = find_column(df, ['accel', 'acceleration'])
    pedal_col = find_column(df, ['pedal'])
    torque_col = find_column(df, ['torque', 'finaltorque'])
    rpm_col = find_column(df, ['rpm', 'enginerpm'])

    # Validation
    missing = [name for name, col in zip(["Time", "Velocity", "Acceleration", "Pedal", "Torque", "RPM"],
                                         [time_col, vel_col, acc_col, pedal_col, torque_col, rpm_col]) if not col]
    if missing:
        print(f"\n❌ Error: Missing columns: {', '.join(missing)}")
        sys.exit(1)

    df = df.sort_values(by=time_col).reset_index(drop=True)

    # Convert velocity to absolute in case of reversing
    df[vel_col] = df[vel_col].abs()

    # ===== AUTOMATED PERFORMANCE ANALYSIS =====
    print("\n" + "="*50)
    print(" 🏎️  POWERTRAIN DYNAMIC PERFORMANCE REPORT")
    print("="*50)

    # Find when the car ACTUALLY starts moving (e.g., > 0.5 km/h)
    moving_data = df[df[vel_col] > 0.5]

    if moving_data.empty:
        print("⚠️  Vehicle never moved in this telemetry log.")
        sys.exit(0)

    start_idx = moving_data.index[0]
    t_start = df.loc[start_idx, time_col]
    print(f"🏁 Launch Detected At: {t_start:.3f} seconds")

    # --- 0 to 100 km/h ---
    to_100 = df.loc[start_idx:][df.loc[start_idx:, vel_col] >= 100.0]
    t_100 = None
    if not to_100.empty:
        idx_100 = to_100.index[0]
        # Linear interpolation for dead-accurate millisecond timing
        t_100 = np.interp(100.0,
                          [df.loc[idx_100-1, vel_col], df.loc[idx_100, vel_col]],
                          [df.loc[idx_100-1, time_col], df.loc[idx_100, time_col]])

        time_0_100 = t_100 - t_start
        avg_accel_100 = df.loc[start_idx:idx_100, acc_col].mean()

        print(f"\n🚀 0 - 100 km/h:      {time_0_100:.3f} seconds")
        print(f"   ↳ Avg Accel:       {avg_accel_100:.3f} G")
    else:
        print("\n🚀 0 - 100 km/h:      [DID NOT REACH 100 KM/H]")

    # --- 100 to 200 km/h ---
    to_200 = df.loc[start_idx:][df.loc[start_idx:, vel_col] >= 200.0]
    if t_100 is not None and not to_200.empty:
        idx_200 = to_200.index[0]
        t_200 = np.interp(200.0,
                          [df.loc[idx_200-1, vel_col], df.loc[idx_200, vel_col]],
                          [df.loc[idx_200-1, time_col], df.loc[idx_200, time_col]])

        time_100_200 = t_200 - t_100
        avg_accel_200 = df.loc[idx_100:idx_200, acc_col].mean()

        print(f"🔥 100 - 200 km/h:    {time_100_200:.3f} seconds")
        print(f"   ↳ Avg Accel:       {avg_accel_200:.3f} G")
    else:
        print("🔥 100 - 200 km/h:    [DID NOT REACH 200 KM/H]")

    # --- Distance Traveled ---
    # Convert km/h to m/s, then integrate over dt to find distance in meters
    dt = df[time_col].diff().fillna(0)
    vel_ms = df[vel_col] / 3.6
    total_distance = (vel_ms * dt).sum()

    print("\n[ SYSTEM MAXIMUMS ]")
    print(f"Top Speed:         {df[vel_col].max():.2f} km/h")
    print(f"Max Acceleration:  {df[acc_col].max():.3f} G")
    print(f"Max Motor Torque:  {df[torque_col].max():.2f} Nm")
    print(f"Max Motor RPM:     {df[rpm_col].max():.0f} RPM")
    print(f"Distance Traveled: {total_distance:.1f} meters")
    print("="*50)

    # ===== VISUALIZATION SYSTEM =====
    print("\n🎨 Compiling engineering telemetry dashboard...")

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("Tesla Powertrain Dynamic Analysis", fontsize=16, fontweight='bold')

    # --- Subplot 1: Velocity & Pedal ---
    ax1.plot(df[time_col], df[vel_col], color='#1f77b4', linewidth=2.5, label='Velocity (km/h)')
    ax1.set_ylabel("Speed (km/h)", fontsize=11, fontweight='bold', color='#1f77b4')
    ax1.grid(True, linestyle='--', alpha=0.6)

    # Add vertical lines for benchmarks
    if t_100: ax1.axvline(x=t_100, color='grey', linestyle=':', alpha=0.8)
    if not to_200.empty: ax1.axvline(x=t_200, color='grey', linestyle=':', alpha=0.8)

    ax1_twin = ax1.twinx()
    ax1_twin.fill_between(df[time_col], 0, df[pedal_col]*100, color='green', alpha=0.15, label='Pedal Position %')
    ax1_twin.plot(df[time_col], df[pedal_col]*100, color='green', alpha=0.5, linewidth=1)
    ax1_twin.set_ylabel("Pedal %", fontsize=11, color='green')
    ax1.set_title("Velocity Output vs. Driver Input", fontsize=12)

    # --- Subplot 2: Torque & RPM (The EV Power Curve) ---
    ax2.plot(df[time_col], df[torque_col], color='#ff7f0e', linewidth=2, label='Motor Torque (Nm)')
    ax2.set_ylabel("Torque (Nm)", fontsize=11, fontweight='bold', color='#ff7f0e')
    ax2.grid(True, linestyle='--', alpha=0.6)

    ax2_twin = ax2.twinx()
    ax2_twin.plot(df[time_col], df[rpm_col], color='#9467bd', linewidth=2, label='Motor RPM')
    ax2_twin.set_ylabel("RPM", fontsize=11, fontweight='bold', color='#9467bd')
    ax2.set_title("Inverter Load: Torque vs RPM", fontsize=12)

    # --- Subplot 3: Acceleration ---
    ax3.plot(df[time_col], df[acc_col], color='#d62728', linewidth=2, label='Acceleration (G)')
    ax3.fill_between(df[time_col], 0, df[acc_col], where=(df[acc_col]>0), color='#d62728', alpha=0.2)
    ax3.fill_between(df[time_col], 0, df[acc_col], where=(df[acc_col]<0), color='blue', alpha=0.2)
    ax3.set_xlabel("Time (seconds)", fontsize=12, fontweight='bold')
    ax3.set_ylabel("G-Force", fontsize=11, fontweight='bold', color='#d62728')
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.set_title("Chassis G-Force", fontsize=12)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
