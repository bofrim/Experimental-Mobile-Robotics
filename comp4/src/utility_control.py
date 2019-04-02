class PID(object):
    def __init__(self, kp, ki, kd, reference_value):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ref = reference_value
        self.proportional_errror = 0
        self.integrated_error = 0
        self.derived_error = 0

    @property
    def output(self):
        return (
            self.kp * self.proportional_errror
            + self.ki * self.integrated_error
            + self.kd * self.derived_error
        )

    def update_state(self, value):
        previous_error = self.proportional_errror
        self.proportional_errror = self.ref - value
        self.integrated_error += value
        self.derived_error = self.proportional_errror - previous_error
