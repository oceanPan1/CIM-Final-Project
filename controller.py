class PoseController:
    def __init__(self, target_distance_m=1.0, yaw_threshold_deg=5.0, dist_threshold_m=0.05):
        self.target_z = target_distance_m
        self.yaw_thresh = yaw_threshold_deg
        self.dist_thresh = dist_threshold_m

    def interpret(self, pose: dict) -> dict:
        yaw = pose['yaw_deg']
        z   = pose['z_dist']
        x   = pose['x_offset']

        # --- Alignment logic ---
        if yaw > self.yaw_thresh:
            rotation = "ROTATE_RIGHT"
        elif yaw < -self.yaw_thresh:
            rotation = "ROTATE_LEFT"
        else:
            rotation = "ALIGNED"

        # --- Distance logic ---
        if z > self.target_z + self.dist_thresh:
            movement = "MOVE_FORWARD"
        elif z < self.target_z - self.dist_thresh:
            movement = "MOVE_BACKWARD"
        else:
            movement = "AT_TARGET"

        # --- Lateral offset (informational) ---
        if x > 0.05:
            lateral = "TAG_RIGHT"
        elif x < -0.05:
            lateral = "TAG_LEFT"
        else:
            lateral = "CENTERED"

        return {
            'rotation':   rotation,
            'movement':   movement,
            'lateral':    lateral,
            'yaw_deg':    round(yaw, 2),
            'z_dist_m':   round(z, 4),
            'x_offset_m': round(x, 4),
            'y_offset_m': round(pose['y_offset'], 4),  # add this line
        }