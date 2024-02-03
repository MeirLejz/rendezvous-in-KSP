class StreamHelper:
    def __init__(self, conn, chaser, target):
        self.ut = conn.add_stream(getattr, conn.space_center, "ut")
        self.rel_pos = conn.add_stream(chaser.position, target.orbital_reference_frame)
        self.rel_vel = conn.add_stream(chaser.velocity, target.orbital_reference_frame)
