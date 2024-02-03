import datetime, time
from numpy.polynomial import Polynomial
import matplotlib.pyplot as plt


def time_sec() -> float:
    # Get the current time
    current_time = datetime.datetime.now().time()

    # Calculate the time in seconds
    time_in_seconds = float(current_time.hour * 3600 + current_time.minute * 60 + current_time.second + current_time.microsecond / 1000000.0)
    return time_in_seconds

def tau(t_0: float, t_f: float) -> float:
    return (time_sec() - t_0) / (t_f - t_0)

# guidance definition
pos_guid = Polynomial([0,0,0, 10, -15, 6]) # 6t^5 - 15t^4 + 10t^3
vel_guid = Polynomial([0,0, 30, -60, 30]) # 30t^4 - 60t^3 + 30t^2
acc_guid = Polynomial([0, 60, -180, 120]) # 120t^3 - 180t^2 + 60t

q_i = 250 # m
q_f = 15 # m

t_0 = time_sec() # s
t_f = t_0 + 10. # s
print(f't_0: {t_0}, t_f: {t_f}')
q = q_i + (q_f - q_i) * pos_guid
q_dot = (q_f - q_i) / (t_f - t_0) * vel_guid
q_double_dot = (q_f - q_i) / (t_f - t_0) ** 2 * acc_guid # reference acceleration

tauu = tau(t_0, t_f)
times = [tauu]
positions = [q(tauu)]
velocities = [q_dot(tauu)]
accelerations = [q_double_dot(tauu)]
positions_norm = [pos_guid(tauu)]
velocities_norm = [vel_guid(tauu)]
accelerations_norm = [acc_guid(tauu)]

t = t_0
while t < t_f:
    t = time_sec()
    tauu = tau(t_0, t_f)
    # print(f'tau: {tauu}, t: {t}')
    times.append(tauu)
    positions.append(q(tauu))
    velocities.append(q_dot(tauu))
    accelerations.append(q_double_dot(tauu))
    positions_norm.append(pos_guid(tauu))
    velocities_norm.append(vel_guid(tauu))
    accelerations_norm.append(acc_guid(tauu))
    time.sleep(0.1)

# plot position, velocity, acceleration from t_0 to t_f
plt.scatter(times, positions, label='position')
plt.scatter(times, velocities, label='velocity')
plt.scatter(times, accelerations, label='acceleration')
plt.scatter(times, positions_norm, label='position_norm')
plt.scatter(times, velocities_norm, label='velocity_norm')
plt.scatter(times, accelerations_norm, label='acceleration_norm')
plt.legend()
plt.show()

