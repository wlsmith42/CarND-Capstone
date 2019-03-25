
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

class Controller(object):
    def __init__(self, *args, **kwargs):
        
        max_abs_angle = kwargs.get('max_steer_angle')        
        self.fuel_capacity = kwargs.get('fuel_capacity')
        self.vehicle_mass = kwargs.get('vehicle_mass')
        self.wheel_radius = kwargs.get('wheel_radius')
        self.accel_limit = kwargs.get('accel_limit')
        self.decel_limit = kwargs.get('decel_limit')
        self.brake_deadband = kwargs.get('brake_deadband')
        self.max_acceleration = 1.5
        self.wheel_base = kwargs.get('wheel_base')
        self.steer_ratio = kwargs.get('steer_ratio')
        self.max_steer_angle = kwargs.get('max_steer_angle')
        self.max_lat_accel = kwargs.get('max_lat_accel')
        
        self.throttle_controller = PID(0.3, 0.01, 0.1, 0, 0.2)
        self.vel_lpf = LowPassFilter(0.5, 0.02)
	self.brake_lpf = LowPassFilter(0.5, 0.02)

        self.yaw_controller = YawController(
            self.wheel_base, self.steer_ratio, 0.1,
            self.max_lat_accel, self.max_steer_angle) 

        total_vehicle_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY # 1.774,933
        # max torque (1.0 throttle) and  max brake torque (deceleration lmt)
        self.max_acc_torque = total_vehicle_mass * self.max_acceleration * self.wheel_radius #567
        self.max_brake_torque = total_vehicle_mass * abs(self.decel_limit) * self.wheel_radius #2095
        
        self.last_time = rospy.get_time()

    def control(self,desired_linear_velocity,desired_angular_velocity,current_linear_velocity,dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
       
        current_linear_velocity = self.vel_lpf.filt(current_linear_velocity)

        steering = self.yaw_controller.get_steering(desired_linear_velocity,desired_angular_velocity, current_linear_velocity)
        
        velocity_error = desired_linear_velocity - current_linear_velocity
	velocity_error = max(self.decel_limit,velocity_error)
	velocity_error = min(velocity_error,self.accel_limit)

        self.last_vel = current_linear_velocity
        
        # find the time duration and a new timestamp
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(velocity_error, sample_time)
        brake = 0
        
        if desired_linear_velocity <= 0.3:
            throttle = 0
            brake = 0.4*self.max_brake_torque
	    if current_linear_velocity <= 0.3:
		brake = self.max_brake_torque # Torque N*m
	brake = min(brake,self.max_brake_torque)
	brake = max(0.0, brake)

	brake = self.brake_lpf.filt(brake)


        return throttle, brake, steering

