import serial
import struct
import numpy as np
import time

# ================= USER CONFIG =================
SERIAL_PORT = '/dev/cu.usbmodem338F397533351'
BAUD_RATE = 115200
DURATION_SEC = 30

PACKET_FMT = '<BfffffffffffffB'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

# ================= HELPERS =================
def normalize(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v

def normalize_rows(m):
    return m / np.linalg.norm(m, axis=1)[:, None]

AXES = ['X', 'Y', 'Z']

# ================= DATA COLLECTION =================
def collect_samples():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.reset_input_buffer()

    samples = []

    print(f"\nMove the board RANDOMLY for {DURATION_SEC} seconds...")
    print("Rotate, flip, yaw, pitch — slow & smooth is best.\n")

    t0 = time.time()
    while time.time() - t0 < DURATION_SEC:
        if ser.read(1) != b'\xAA':
            continue

        data = ser.read(PACKET_SIZE - 1)
        if len(data) != PACKET_SIZE - 1:
            continue

        u = struct.unpack('<fffffffffffffB', data)
        if u[-1] != 0x55:
            continue

        ax, ay, az = u[0:3]
        mx, my, mz = u[6:9]

        samples.append([ax, ay, az, mx, my, mz])

    ser.close()
    print(f"Collected {len(samples)} samples\n")
    return np.array(samples)

# ================= AXIS SOLVERS =================
def solve_accel_axes(acc):
    acc_n = normalize_rows(acc)

    g_mean = normalize(np.mean(acc_n, axis=0))
    print("Mean gravity (sensor frame):", g_mean)

    M = np.zeros((3,3))

    # Body Z = gravity
    z_idx = np.argmax(np.abs(g_mean))
    M[2, z_idx] = np.sign(g_mean[z_idx])

    # Remaining axes (horizontal plane)
    rem = np.array([i for i in range(3) if i != z_idx])

    # Variance tells us which axis moved more (horizontal)
    var = np.var(acc_n[:, rem], axis=0)

    # Larger variance → more horizontal motion
    order = rem[np.argsort(var)]

    M[0, order[1]] = 1   # Body X
    M[1, order[0]] = 1   # Body Y

    return M


def solve_mag_axes(acc, mag):
    acc_n = normalize_rows(acc)
    mag_n = normalize_rows(mag)

    # Remove gravity from mag
    mag_h = mag_n - (mag_n * acc_n).sum(axis=1)[:, None] * acc_n
    mag_h = normalize_rows(mag_h)

    north_mean = normalize(np.mean(mag_h, axis=0))
    print("Mean magnetic north (sensor frame):", north_mean)

    M = np.zeros((3,3))

    # Body X = North
    x_idx = np.argmax(np.abs(north_mean))
    M[0, x_idx] = np.sign(north_mean[x_idx])

    rem = [i for i in range(3) if i != x_idx]
    M[1, rem[0]] = 1
    M[2, rem[1]] = 1

    return M

# ================= OUTPUT =================
def print_mapping(name, M):
    print(f"\n{name} AXIS MAPPING")
    print("-" * 30)
    for i in range(3):
        j = np.argmax(np.abs(M[i]))
        sign = '+' if M[i, j] > 0 else '-'
        print(f"Body {AXES[i]} = {sign} Sensor {AXES[j]}")

def print_c_macros(name, M):
    print(f"\n// {name} C MACROS")
    for i in range(3):
        j = np.argmax(np.abs(M[i]))
        sign = '+' if M[i, j] > 0 else '-'
        print(f"#define {name}_{AXES[i]} {sign}{AXES[j]}")

# ================= MAIN =================
if __name__ == "__main__":
    samples = collect_samples()

    acc = samples[:, 0:3]
    mag = samples[:, 3:6]

    M_acc = solve_accel_axes(acc)
    M_mag = solve_mag_axes(acc, mag)

    print_mapping("ACCEL", M_acc)
    print_mapping("MAG", M_mag)

    print_c_macros("ACC", M_acc)
    print_c_macros("MAG", M_mag)

    print("\nCalibration complete.")
    print("Apply mappings BEFORE bias correction in firmware.")
