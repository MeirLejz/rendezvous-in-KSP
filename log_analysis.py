import matplotlib.pyplot as plt

nav_plane_pos = []
nav_plane_vel = []
guid_plane_pos = []
guid_plane_vel = []
timestamp = []

with open("rendezvous_docking.log", "r") as file:
    for line in file:
        if line.startswith("INFO - Nav,"):
            values = line.strip().split(",")[1:]
            # in_plane_nav_values = [float(value) for value in values[1:3]]
            nav_messages = [float(value) for value in values]
            nav_plane_pos.append(nav_messages[1:3])
            nav_plane_vel.append(nav_messages[4:6])
            timestamp.append(nav_messages[0])
        elif line.startswith("INFO - Guid,"):
            values = line.strip().split(",")[1:]
            # in_plane_nav_values = [float(value) for value in values[0:2]]
            guid_messages = [float(value) for value in values]
            guid_plane_pos.append(guid_messages[0:2])
            guid_plane_vel.append(guid_messages[2:4])
nav_plane_pos = list(zip(*nav_plane_pos))
nav_plane_vel = list(zip(*nav_plane_vel))
guid_plane_pos = list(zip(*guid_plane_pos))
guid_plane_vel = list(zip(*guid_plane_vel))

fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

ax1.plot(nav_plane_pos[1], nav_plane_pos[0], label="Nav")
ax1.plot(guid_plane_pos[1], guid_plane_pos[0], label="Guid")

# Add points every 5 minutes
previous_timestamp = 0
for i in range(len(timestamp)):
    if (
        timestamp[i] > previous_timestamp + 300
    ):  # Add a point every 5 minutes (300 seconds)
        previous_timestamp = timestamp[i]
        ax1.scatter(nav_plane_pos[1][i], nav_plane_pos[0][i], color="red")
        timestamp_annot = divmod(timestamp[i], 60)
        ax1.annotate(
            f"{timestamp_annot[0]:.0f}:{timestamp_annot[1]:.1f}",
            (nav_plane_pos[1][i], nav_plane_pos[0][i]),
            textcoords="offset points",
            xytext=(0, 10),
            ha="center",
        )
        ax1.scatter(guid_plane_pos[1][i], guid_plane_pos[0][i], color="red")
        ax1.annotate(
            f"{timestamp_annot[0]:.0f}:{timestamp_annot[1]:.1f}",
            (guid_plane_pos[1][i], guid_plane_pos[0][i]),
            textcoords="offset points",
            xytext=(0, 10),
            ha="center",
        )

ax2.plot(timestamp, nav_plane_vel[0], label="Nav, R-bar")
ax2.plot(timestamp[1:], guid_plane_vel[0], label="Guid, R-bar")
ax3.plot(timestamp, nav_plane_vel[1], label="Nav, V-bar")
ax3.plot(timestamp[1:], guid_plane_vel[1], label="Guid, V-bar")

# # Add points every 5 minutes
# previous_timestamp = 0
# for i in range(len(timestamp)):
#     if (
#         timestamp[i] > previous_timestamp + 300
#     ):  # Add a point every 5 minutes (300 seconds)
#         previous_timestamp = timestamp[i]
#         ax2.scatter(nav_plane_vel[1][i], nav_plane_vel[0][i], color="red")
#         timestamp_annot = divmod(timestamp[i], 60)
#         ax2.annotate(
#             f"{timestamp_annot[0]:.0f}:{timestamp_annot[1]:.1f}",
#             (nav_plane_vel[1][i], nav_plane_vel[0][i]),
#             textcoords="offset points",
#             xytext=(0, 10),
#             ha="center",
#         )
#         ax2.scatter(guid_plane_vel[1][i], guid_plane_vel[0][i], color="red")
#         ax2.annotate(
#             f"{timestamp_annot[0]:.0f}:{timestamp_annot[1]:.1f}",
#             (guid_plane_vel[1][i], guid_plane_vel[0][i]),
#             textcoords="offset points",
#             xytext=(0, 10),
#             ha="center",
#         )


plt.legend()
plt.show()
