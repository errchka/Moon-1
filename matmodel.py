import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

mu_kerbin = 3.53e12  # Гравитационный параметр Кербина [м³/с²]
R_kerbin = 600000.0  # Радиус Кербина [м]
g0 = 9.81  # Ускорение свободного падения [м/с²]
rho0 = 1.225  # Плотность на уровне моря [кг/м³]
H = 5000.0  # Шкала высоты KSP (не 5600!) [м]
M_start = 141200.0  # Стартовая масса (6-я ступень с топливом)
M_prop_6 = 80600.0  # Топливо 6-й ступени
M_6_dry = 60600.0  # Масса после выработки топлива 6-й ступени
M_5_start = 31000.0  # Начало 5-й ступени
M_prop_5 = 8000.0  # Топливо 5-й ступени
M_5_dry = 23000.0  # Конец 5-й ступени
M_4_start = 18500.0  # Начало 4-й ступени
M_prop_4 = 4000.0  # Топливо 4-й ступени
M_4_dry = 14500.0  # Конец 4-й ступени

M_3_start = 12800.0  # Начало 3-й ступени
M_prop_3 = 1900.0  # Топливо 3-й ступени
M_3_dry = 10900.0  # Конец 3-й ступени

# Времена работы ступеней
t_6 = 90.0
t_5 = 59.0
t_4 = 70.0
t_3 = 107.0
Isp_6_atm = 280.0  # Удельный импульс в атмосфере [с]
Isp_6_vac = 320.0  # Удельный импульс в вакууме [с]
T_6_one_atm = 568750.0  # Тяга одного "Шкипера" в атм [Н]
T_6_one_vac = 650000.0  # Тяга одного "Шкипера" в вакууме [Н]
T_6_total_atm = 6 * T_6_one_atm  # Суммарная тяга 6 двигателей
T_6_total_vac = 6 * T_6_one_vac
Isp_5_vac = 300.0  # Удельный импульс в вакууме [с]
T_5_vac = 260000.0  # Тяга в вакууме [Н]
Isp_4_vac = 355.0  # Удельный импульс в вакууме [с]
T_4_vac = 125000.0  # Тяга в вакууме [Н]
Isp_3_vac = 345.0  # Удельный импульс в вакууме [с]
T_3_vac = 60000.0  # Тяга в вакууме [Н]
Cd = 0.2  # Коэффициент сопротивления для ракеты
S = 4.91  # Площадь поперечного сечения [м²]
def gravity(h):
    return mu_kerbin / (R_kerbin + h) ** 2
def density(h):
    return rho0 * np.exp(-h / H)
def atmospheric_pressure(h):
    return np.exp(-h / H) if h < 70000 else 0
def mass(t):
    if t < t_6:
        # Ступень 6 работает
        return M_start - M_prop_6 * (t / t_6)
    elif t < t_6 + t_5:
        # Ступень 5 работает
        return M_6_dry - M_prop_5 * ((t - t_6) / t_5)
    elif t < t_6 + t_5 + t_4:
        # Ступень 4 работает
        return M_5_dry - M_prop_4 * ((t - t_6 - t_5) / t_4)
    elif t < t_6 + t_5 + t_4 + t_3:
        # Ступень 3 работает
        return M_4_dry - M_prop_3 * ((t - t_6 - t_5 - t_4) / t_3)
    else:
        # Все двигатели выключены
        return M_3_dry
def thrust(t, h):
    # Относительное давление на высоте h
    p = atmospheric_pressure(h)
    if t < t_6:
        # Ступень 6: интерполяция между атмосферной и вакуумной тягой
        T_6 = T_6_total_vac - (T_6_total_vac - T_6_total_atm) * p
        return T_6
    elif t < t_6 + t_5:
        # Ступень 5: вакуумные параметры
        return T_5_vac
    elif t < t_6 + t_5 + t_4:
        # Ступень 4: вакуумные параметры
        return T_4_vac
    elif t < t_6 + t_5 + t_4 + t_3:
        # Ступень 3: вакуумные параметры
        return T_3_vac
    else:
        return 0.0


def angle_of_attack(t):
    if t < 10:
        return 90.0
    elif t < 30:
        return 90.0 - (t - 10) * (10.0 / 20.0)  # 90° → 80°
    elif t < 60:
        return 80.0 - (t - 30) * (35.0 / 30.0)  # 80° → 45°
    elif t < 120:
        return 45.0 - (t - 60) * (25.0 / 60.0)  # 45° → 20°
    elif t < 180:
        return 20.0 - (t - 120) * (15.0 / 60.0)  # 20° → 5°
    else:
        return max(0.0, 5.0 - (t - 180) * (5.0 / 60.0))  # 5° → 0°
def rocket_equations(t, y):
    vx, vy, x, h = y
    v = np.sqrt(vx ** 2 + vy ** 2)
    m = mass(t)
    T = thrust(t, h)
    g = gravity(h)
    rho = density(h)
    theta = np.radians(angle_of_attack(t))
    if v > 0:
        Fd = 0.5 * rho * v ** 2 * Cd * S
        Fdx = -Fd * (vx / v)
        Fdy = -Fd * (vy / v)
    else:
        Fdx, Fdy = 0.0, 0.0

    # Проекции тяги
    Tx = T * np.cos(theta)
    Ty = T * np.sin(theta)

    # Уравнения движения
    dvx_dt = (Tx + Fdx) / m
    dvy_dt = (Ty + Fdy) / m - g
    dx_dt = vx
    dh_dt = vy
    return [dvx_dt, dvy_dt, dx_dt, dh_dt]
y0 = [0.0, 0.0, 0.0, 0.0]  # [vx, vy, x, h]
t_max = t_6 + t_5 + t_4 + t_3 + 50
t_eval = np.linspace(0, t_max, 2000)
solution = solve_ivp(
    rocket_equations,
    [0, t_max],
    y0,
    t_eval=t_eval,
    method='RK45',
    rtol=1e-6,
    atol=1e-9
)
t_model = solution.t
vx_model = solution.y[0]
vy_model = solution.y[1]
x_model = solution.y[2]
h_model = solution.y[3]
v_model = np.sqrt(vx_model ** 2 + vy_model ** 2)
mass_model = np.array([mass(ti) for ti in t_model])
try:
    f = open('luna17_flight_data.csv', 'r', encoding='utf-8')
    times_ksp, speeds_ksp, altitudes_ksp, distances_ksp, masses_ksp = [], [], [], [], []
    for line in f:
        line = line.strip()
        if line:
            try:
                parts = line.split(',')
                if len(parts) >= 5:
                    times_ksp.append(float(parts[0]))
                    speeds_ksp.append(float(parts[1]))
                    altitudes_ksp.append(float(parts[2]))
                    distances_ksp.append(float(parts[3]))
                    masses_ksp.append(float(parts[4]))
            except ValueError:
                print(f"Ошибка в строке: {line}")
                continue

    f.close()
    if times_ksp:
        print(f"✓ Загружено {len(times_ksp)} точек данных из KSP")
    else:
        print("✗ Файл flight_data.txt пуст или содержит ошибки")

except FileNotFoundError:
    times_ksp, speeds_ksp, altitudes_ksp, masses_ksp = [], [], [], []
fig, axes = plt.subplots(3, 1, figsize=(12, 12))
color_model = 'blue'
color_ksp = 'green'
linewidth_model = 2
linewidth_ksp = 1.5

# 1. СКОРОСТЬ ОТ ВРЕМЕНИ
ax1 = axes[0]
# График модели
ax1.plot(t_model, v_model, color=color_model, linewidth=linewidth_model,
         label='Модель (полная скорость)', linestyle='-')

# График KSP если есть данные
if times_ksp:
    ax1.plot(times_ksp, speeds_ksp, color=color_ksp, linewidth=linewidth_ksp,
             label='KSP данные', linestyle='-', alpha=0.8)
stage_times = [t_6, t_6 + t_5, t_6 + t_5 + t_4]
stage_labels = []

for stage_time, label in zip(stage_times, stage_labels):
    if stage_time < t_model[-1]:
        idx = np.argmin(np.abs(t_model - stage_time))
        ax1.axvline(x=stage_time, color='red', linestyle=':', alpha=0.5)
        ax1.text(stage_time, v_model[idx] * 0.7, label, rotation=90,
                 verticalalignment='bottom', fontsize=9, color='red')

ax1.set_xlabel('Время (с)', fontsize=12)
ax1.set_ylabel('Скорость (м/с)', fontsize=12)
ax1.set_title('СКОРОСТЬ: Модель vs KSP', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend(loc='best')
ax1.set_xlim(0, min(t_model[-1], max(times_ksp) if times_ksp else t_model[-1]))

# 2. ВЫСОТА ОТ ВРЕМЕНИ
ax2 = axes[1]

# График модели (в км)
ax2.plot(t_model, h_model / 1000, color=color_model, linewidth=linewidth_model,
         label='Модель', linestyle='-')

# График KSP если есть данные (в км)
if times_ksp:
    ax2.plot(times_ksp, [alt / 1000 for alt in altitudes_ksp], color=color_ksp,
             linewidth=linewidth_ksp, label='KSP данные', linestyle='-', alpha=0.8)

# Ключевые высоты
ax2.axhline(y=70, color='orange', linestyle='--', alpha=0.7, label='Граница атмосферы (70 км)')
ax2.axhline(y=200, color='red', linestyle='--', alpha=0.7, label='Целевая орбита (200 км)')

ax2.set_xlabel('Время (с)', fontsize=12)
ax2.set_ylabel('Высота (км)', fontsize=12)
ax2.set_title('ВЫСОТА: Модель vs KSP', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend(loc='best')
ax2.set_xlim(0, min(t_model[-1], max(times_ksp) if times_ksp else t_model[-1]))

# 3. МАССА ОТ ВРЕМЕНИ
ax3 = axes[2]

# График модели (в тоннах)
ax3.plot(t_model, mass_model / 1000, color=color_model, linewidth=linewidth_model,
         label='Модель', linestyle='-')

# График KSP если есть данные (в тоннах)
if times_ksp:
    ax3.plot(times_ksp, [mass / 1000 for mass in masses_ksp], color=color_ksp,
             linewidth=linewidth_ksp, label='KSP данные', linestyle='-', alpha=0.8)
ax3.set_xlabel('Время (с)', fontsize=12)
ax3.set_ylabel('Масса (т)', fontsize=12)
ax3.set_title('МАССА: Модель vs KSP', fontsize=14, fontweight='bold')
ax3.grid(True, alpha=0.3)
ax3.legend(loc='best')
ax3.set_xlim(0, min(t_model[-1], max(times_ksp) if times_ksp else t_model[-1]))

plt.tight_layout()
# Сохраняем и показываем графики
plt.savefig('luna17_comparison.png', dpi=300, bbox_inches='tight')
plt.show()
