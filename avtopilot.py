import time
import krpc
import matplotlib.pyplot as plt

# Параметры гравитационного разворота
turn_start_altitude = 250
turn_end_altitude = 40000
target_altitude = 200000

# Подключение к KSP
conn = krpc.connect(name='Launch into orbit')
vessel = conn.space_center.active_vessel
srf_frame = vessel.orbit.body.reference_frame

# Сбор данных
times, speeds, altitudes, masses = [], [], [], []
start_time = time.time()

# Потоки данных
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
srf_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'speed')
stage_resources = vessel.resources_in_decouple_stage(stage=5, cumulative=False)
srb_fuel = conn.add_stream(stage_resources.amount, 'LiquidFuel')

# Предстартовая подготовка
vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

# Старт
vessel.control.activate_next_stage()

# Основной цикл
srbs_separated = False
turn_angle = 0

while True:
    # Запись данных
    times.append(time.time() - start_time)
    speeds.append(srf_speed())
    altitudes.append(altitude())
    masses.append(vessel.mass)

    # Гравитационный разворот
    if (altitude() > turn_start_altitude) and (altitude() < turn_end_altitude):
        frac = ((altitude() - turn_start_altitude) / (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)

    # Отделение ускорителей
    if not srbs_separated and srb_fuel() < 0.1:
        vessel.control.activate_next_stage()
        srbs_separated = True
        time.sleep(0.1)

    # Выход из цикла при достижении апогея
    if apoapsis() > target_altitude * 0.9:
        break

    time.sleep(0.1)

# Точная настройка апогея
vessel.control.throttle = 0.25
while apoapsis() < target_altitude:
    time.sleep(0.1)
vessel.control.throttle = 0.0

# Выход из атмосферы
while altitude() < 70500:
    time.sleep(0.1)

print('Launch complete')

# графики

# 1. Скорость от времени
plt.figure(figsize=(10, 6))
plt.plot(times, speeds, 'b-')
plt.xlabel('Время (с)')
plt.ylabel('Скорость (м/c)')
plt.grid(True)
plt.show()

# 2. Высота от времени
plt.figure(figsize=(10, 6))
plt.plot(times, altitudes, 'b-')
plt.xlabel('Время (с)')
plt.ylabel('Высота (м)')
plt.grid(True)
plt.show()

# 3. Масса от времени
plt.figure(figsize=(10, 6))
plt.plot(times, masses, 'b-')
plt.xlabel('Время (с)')
plt.ylabel('Масса (кг)')
plt.grid(True)
plt.show()

# Сохранение данных
with open('flight_data.txt', 'w', encoding='utf-8') as f:
    for t, s, a, m in zip(times, speeds, altitudes, masses):
        f.write(f"{t:.2f},{s:.2f},{a:.2f},{m:.2f}\n")
