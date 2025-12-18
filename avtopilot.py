import krpc
import time
import math
import matplotlib.pyplot as plt

conn = krpc.connect(name='Луна-17 Автопилот')
vessel = conn.space_center.active_vessel
space_center = conn.space_center

vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0
body = vessel.orbit.body

launchpad_frame = body.reference_frame
surface_frame = vessel.surface_reference_frame
orbital_frame = vessel.orbital_reference_frame
def horizontal_distance():
    pos = vessel.position(launchpad_frame)
    return math.sqrt(pos[0] ** 2 + pos[2] ** 2)
# Потоки данных
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
surface_altitude = conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
speed = conn.add_stream(getattr, vessel.flight(body.reference_frame), 'speed')
vertical_speed = conn.add_stream(getattr, vessel.flight(surface_frame), 'vertical_speed')
horizontal_speed = conn.add_stream(getattr, vessel.flight(surface_frame), 'horizontal_speed')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')
time_to_apo = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
time_to_peri = conn.add_stream(getattr, vessel.orbit, 'time_to_periapsis')
def get_stage_fuel(stage_num):
    liquid_fuel = vessel.resources.amount('LiquidFuel')
    oxidizer = vessel.resources.amount('Oxidizer')
    return liquid_fuel > 0.1 and oxidizer > 0.1
autopilot = vessel.auto_pilot
autopilot.reference_frame = surface_frame
autopilot.engage()

times, speeds, altitudes, distances, masses = [], [], [], [], []
start_time = time.time()
log_file = open('luna17_flight_log.txt', 'w', encoding='utf-8')
def log_message(message):
    timestamp = time.time() - start_time
    log_file.write(f"{timestamp:.1f}: {message}\n")
    print(f"{timestamp:.1f}: {message}")

def record_data():
    times.append(time.time() - start_time)
    speeds.append(speed())
    altitudes.append(surface_altitude())
    distances.append(horizontal_distance())
    masses.append(vessel.mass)
vessel.control.activate_next_stage()
time.sleep(2)

log_message("СТУПЕНЬ 6")
while altitude() < 10000:
    current_alt = altitude()
    current_speed = speed()
    if current_alt < 1000:
        target_pitch = 90.0
    elif current_alt < 5000:
        target_pitch = 88.0
    elif current_alt < 8000:
        target_pitch = 85.0
    else:
        target_pitch = 82.0
    autopilot.target_pitch_and_heading(target_pitch, 90.0)
    record_data()
    if len(times) % 50 == 0:
        log_message(
            f"Высота: {current_alt / 1000:.1f} км, Скорость: {current_speed:.0f} м/с, TWR: {(vessel.thrust / (vessel.mass * 9.81)):.2f}")

    time.sleep(0.1)
while altitude() < 70000:
    current_alt = altitude()
    current_speed = speed()
    if current_alt < 15000:
        target_pitch = 80.0
        throttle = 1.0
    elif current_alt < 25000:
        target_pitch = 70.0
        if current_speed > 800 and current_alt < 20000:
            throttle = 0.8
        else:
            throttle = 1.0
    elif current_alt < 40000:
        target_pitch = 55.0
        throttle = 1.0
    elif current_alt < 55000:
        target_pitch = 40.0
        throttle = 1.0
    else:
        target_pitch = 30.0
        throttle = 1.0
    autopilot.target_pitch_and_heading(target_pitch, 90.0)
    vessel.control.throttle = throttle
    if current_alt > 45000 and current_speed > 1800:
        if vessel.resources.amount('LiquidFuel') < 1000:
            log_message("СТУПЕНЬ 6: Топливо на исходе")
            break
    record_data()
    if len(times) % 50 == 0:
        log_message(
            f"Высота: {current_alt / 1000:.1f} км, Скорость: {current_speed:.0f} м/с, Угол: {target_pitch:.0f}°")

    time.sleep(0.1)
log_message("ОТДЕЛЕНИЕ СТУПЕНИ 6")
vessel.contro.throttle = 0.0
time.sleep(0.5)
vessel.control.activate_next_stage()
time.sleep(1)
log_message("СТУПЕНЬ 5")
vessel.control.throttle = 1.0
time.sleep(2)

while altitude() < 120000 and get_stage_fuel(5):
    current_alt = altitude()
    current_speed = speed()
    apo = apoapsis()
    if apo < 80000:
        target_pitch = 25.0
    elif apo < 150000:
        target_pitch = 15.0
    elif apo < 180000:
        target_pitch = 10.0
    else:
        target_pitch = 5.0
    if apo > 190000:
        target_pitch = 0.0
        if apo > 210000:
            vessel.control.throttle = 0.5
        else:
            vessel.control.throttle = 1.0
    else:
        vessel.control.throttle = 1.0
    autopilot.target_pitch_and_heading(target_pitch, 90.0)
    if apo > 200000 and horizontal_speed() > 2200:
        log_message("Достигнут целевой апогей 200 км")
        break
    record_data()
    if len(times) % 50 == 0:
        log_message(
            f"Высота: {current_alt / 1000:.1f} км, Апогей: {apo / 1000:.1f} км, Скорость: {current_speed:.0f} м/с")

    time.sleep(0.1)
while True:
    if vessel.resources.amount('LiquidFuel') < 1000:
        log_message("ОТДЕЛЕНИЕ СТУПЕНИ 5")
        vessel.control.throttle = 0.0
        time.sleep(0.5)
        vessel.control.activate_next_stage()
        time.sleep(1)
        break
log_message("СТУПЕНЬ 4")
autopilot.reference_frame = orbital_frame
autopilot.target_direction = (0, 1, 0)
time.sleep(1)
vessel.control.throttle = 1.0

target_altitude = 200000
orbit_circularized = False
while not orbit_circularized and get_stage_fuel(4):
    current_alt = altitude()
    apo = apoapsis()
    peri = periapsis()
    t_apo = time_to_apo()
    if apo < target_altitude:
        if t_apo > 100:
            vessel.control.throttle = 0.0
        else:
            vessel.control.throttle = 1.0
    elif peri < target_altitude - 5000:
        if t_apo < 30:  # Близко к апоцентру
            vessel.control.throttle = 1.0
        else:
            vessel.control.throttle = 0.0
    else:
        vessel.control.throttle = 0.0
        orbit_circularized = True
    eccentricity = abs(apo - peri) / (apo + peri + 2 * 600000)
    if eccentricity < 0.05 and peri > 140000 and apo < 250000:
        orbit_circularized = True
        log_message(f"ОРБИТА ДОСТИГНУТА! Перигей: {peri / 1000:.1f} км, Апогей: {apo / 1000:.1f} км")

    record_data()
    if len(times) % 50 == 0:
        log_message(f"Апогей: {apo / 1000:.1f} км, Перигей: {peri / 1000:.1f} км, Эксцентриситет: {eccentricity:.3f}")

    time.sleep(0.1)
log_message("=== ОТДЕЛЕНИЕ СТУПЕНИ 4 ===")
vessel.control.throttle = 0.0
time.sleep(1)
vessel.control.activate_next_stage()
time.sleep(1)
for i in range(5):
    log_message(f"Ожидание... {5 - i}")
    time.sleep(1)
autopilot.disengage()
vessel.control.sas = True
time.sleep(1)
log_file.close()
# 1. Скорость от времени
plt.figure(figsize=(12, 8))

plt.subplot(3, 2, 1)
plt.plot(times, speeds, 'b-', linewidth=2)
plt.xlabel('Время (с)')
plt.ylabel('Скорость (м/с)')
plt.title('Скорость от времени')
plt.grid(True, alpha=0.3)

# 2. Высота от времени
plt.subplot(3, 2, 2)
plt.plot(times, [alt / 1000 for alt in altitudes], 'g-', linewidth=2)
plt.xlabel('Время (с)')
plt.ylabel('Высота (км)')
plt.title('Высота от времени')
plt.grid(True, alpha=0.3)

# 3. Масса от времени
plt.subplot(3, 2, 3)
plt.plot(times, [mass / 1000 for mass in masses], 'r-', linewidth=2)
plt.xlabel('Время (с)')
plt.ylabel('Масса (т)')
plt.title('Масса от времени')
plt.grid(True, alpha=0.3)

# 4. Горизонтальное расстояние от времени
plt.subplot(3, 2, 4)
plt.plot(times, [dist / 1000 for dist in distances], 'm-', linewidth=2)
plt.xlabel('Время (с)')
plt.ylabel('Расстояние (км)')
plt.title('Горизонтальное расстояние от времени')
plt.grid(True, alpha=0.3)

plt.subplot(3, 2, 6)
plt.plot(times, speeds, 'b-', linewidth=1, alpha=0.5, label='Полная')
plt.axhline(y=2300, color='r', linestyle='--', alpha=0.5, label='Орбитальная (2300 м/с)')
plt.xlabel('Время (с)')
plt.ylabel('Скорость (м/с)')
plt.title('Достижение орбитальной скорости')
plt.legend()
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('luna17_flight_profile.png', dpi=300, bbox_inches='tight')
plt.show()
with open('luna17_flight_data.csv', 'w', encoding='utf-8') as f:
    f.write("Время(с),Скорость(м/с),Высота(м),Расстояние(м),Масса(кг)\n")
    for t, s, a, d, m in zip(times, speeds, altitudes, distances, masses):
        f.write(f"{t:.2f},{s:.2f},{a:.2f},{d:.2f},{m:.2f}\n")
