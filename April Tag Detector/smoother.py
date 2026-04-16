class EMAFilter:
    """Exponential Moving Average filter for smoothing pose data."""

    def __init__(self, alpha=0.3):
        """
        alpha: smoothing factor (0 < alpha < 1).
               Lower = smoother but laggier. 0.3 is a good start.
        """
        self.alpha = alpha
        self._state = {}

    def update(self, key, value):
        if key not in self._state:
            self._state[key] = value
        else:
            self._state[key] = self.alpha * value + (1 - self.alpha) * self._state[key]
        return self._state[key]

    def smooth_pose(self, pose: dict) -> dict:
        tag_id = pose['id']
        return {
            **pose,
            'x_offset': self.update(f'{tag_id}_x', pose['x_offset']),
            'y_offset': self.update(f'{tag_id}_y', pose['y_offset']),
            'z_dist':   self.update(f'{tag_id}_z', pose['z_dist']),
            'yaw_deg':  self.update(f'{tag_id}_yaw', pose['yaw_deg']),
        }