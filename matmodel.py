import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

mu_kerbin = 3.5316e12
R_kerbin = 600000.0
g0 = 9.81
rho0 = 1.225
H = 5600.0

M_start = 141200.0
M_prop_6 = 80600.0
M_6_dry = 60600.0
M_prop_5 = 8000.0
M_5_dry = 23000.0
M_prop_4 = 4000.0
M_4_dry = 14500.0

Isp_6 = 280.0
T_6 = 1893900.0
Isp_5 = 285.0
T_5 = 247000.0
Isp_4 = 300.0
T_4 = 52000.0

mdot_6 = T_6 / (Isp_6 * g0)
mdot_5 = T_5 / (Isp_5 * g0)
mdot_4 = T_4 / (Isp_4 * g0)
t_6 = M_prop_6 / mdot_6
t_5 = M_prop_5 / mdot_5
t_4 = M_prop_4 / mdot_4

Cd = 0.2
diameter = 5.0
S = 3.1416 * (diameter / 2) ** 2
turn_start = 250
turn_end = 40000

def gravity(h): return mu_kerbin / (R_kerbin + h) ** 2
def density(h): return rho0 * np.exp(-h / H)

def mass(t):
    if t < t_6:
        return M_start - mdot_6 * t
    elif t < t_6 + t_5:
        return M_prop_5 + M_5_dry - mdot_5 * (t - t_6)
    elif t < t_6 + t_5 + t_4:
        return M_prop_4 + M_4_dry - mdot_4 * (t - t_6 - t_5)
    else:
        return M_4_dry

def thrust(t):
    if t < t_6:
        return T_6
    elif t < t_6 + t_5:
        return T_5
    elif t < t_6 + t_5 + t_4:
        return T_4
    else:
        return 0.0

def angle_of_attack(h):
    if h <= turn_start:
        return 90.0
    elif turn_start < h < turn_end:
        return 90.0 - 90.0 * ((h - turn_start) / (turn_end - turn_start))
    elif h >= turn_end:
        if h < 200000:
            return 90.0 - 90.0 * (0.5 + ((h - turn_end) / (200000 - turn_end)) * 0.5)
        else:
            return 0.0
    return 0.0

def rocket_equations(t, y):
    vx, vy, x, h = y
    v = np.sqrt(vx ** 2 + vy ** 2)
    m = max(mass(t), M_4_dry)
    T = thrust(t)
    g = gravity(h)
    rho = density(h)
    theta = np.radians(angle_of_attack(h))

    if v > 0 and h < 70000:
        Fd = 0.5 * rho * v ** 2 * Cd * S
        Fdx = -Fd * (vx / v)
        Fdy = -Fd * (vy / v)
    else:
        Fdx, Fdy = 0.0, 0.0

    if m > 0 and T > 0:
        Tx = T * np.cos(theta)
        Ty = T * np.sin(theta)
    else:
        Tx, Ty = 0.0, 0.0

    dvx_dt = (Tx + Fdx) / m if m > 0 else 0.0
    dvy_dt = (Ty + Fdy) / m - g if m > 0 else -g
    return [dvx_dt, dvy_dt, vx, vy]

# Сначала загружаем данные KSP, чтобы узнать время
times_ksp, speeds_ksp, altitudes_ksp, masses_ksp = [], [], [], []
try:
    with open('flight_data.txt', 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) >= 4:
                times_ksp.append(float(parts[0]))
                speeds_ksp.append(float(parts[1]))
                altitudes_ksp.append(float(parts[2]))
                masses_ksp.append(float(parts[3]))
except:
    pass

# Определяем время симуляции
if times_ksp:
    t_max = max(times_ksp)  # Берем максимальное время из KSP
else:
    t_max = t_6 + t_5 + t_4 + 100

y0 = [0.0, 0.0, 0.0, 0.0]
t_eval = np.linspace(0, t_max, 3000)

solution = solve_ivp(rocket_equations, [0, t_max], y0, t_eval=t_eval, method='RK45', rtol=1e-6)
t = solution.t
vx, vy, x, h = solution.y
v = np.sqrt(vx ** 2 + vy ** 2)
mass_vals = np.array([mass(ti) for ti in t])

plt.figure(figsize=(12, 8))

plt.subplot(2, 2, 1)
plt.plot(t, v, 'b-', linewidth=2, label='Модель')
if times_ksp:
    plt.plot(times_ksp, speeds_ksp, 'r-', linewidth=1, label='KSP', alpha=0.7)
    plt.xlim(0, t_max)  # Ограничиваем по времени KSP
plt.xlabel('Время (с)')
plt.ylabel('Скорость (м/с)')
plt.grid(True, alpha=0.3)
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(t, h / 1000, 'b-', linewidth=2, label='Модель')
if times_ksp:
    plt.plot(times_ksp, np.array(altitudes_ksp) / 1000, 'r-', linewidth=1, label='KSP', alpha=0.7)
    plt.xlim(0, t_max)  # Ограничиваем по времени KSP
plt.xlabel('Время (с)')
plt.ylabel('Высота (км)')
plt.grid(True, alpha=0.3)
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(t, mass_vals / 1000, 'b-', linewidth=2, label='Модель')
if times_ksp:
    plt.plot(times_ksp, np.array(masses_ksp) / 1000, 'r-', linewidth=1, label='KSP', alpha=0.7)
    plt.xlim(0, t_max)  # Ограничиваем по времени KSP
plt.xlabel('Время (с)')
plt.ylabel('Масса (т)')
plt.grid(True, alpha=0.3)
plt.legend()

plt.tight_layout()
plt.savefig('comparison.png', dpi=300)
plt.show()
