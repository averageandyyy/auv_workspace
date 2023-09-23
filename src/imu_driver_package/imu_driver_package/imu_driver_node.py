import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import time

class ImuDriverNode(Node):
    def __init__(self):
        super().__init__("imu_driver_node")
        self.imu_publisher = self.create_publisher(Imu, "imu_data", 10)
        
        # Change I2C bus number to actual configuration
        self.i2c_bus = smbus.SMBus(1)

    
    def read_imu_data(self):
        # Define IMU sensor's I2C address
        imu_address = 0x68

        # Register addresses for IMU sensors
        ACCEL_XOUT_H = 0x3B
        ACCEL_XOUT_L = 0x3C
        
        ACCEL_YOUT_H = 0x3D
        ACCEL_YOUT_L = 0x3E
        
        ACCEL_ZOUT_H = 0x3F
        ACCEL_ZOUT_L = 0x40

        GYRO_XOUT_H = 0x43
        GYRO_XOUT_L = 0x44

        GYRO_YOUT_H = 0x45
        GYRO_YOUT_L = 0x46

        GYRO_ZOUT_H = 0x47
        GYRO_ZOUT_L = 0x48

        TEMP_OUT_H = 0x41
        TEMP_OUT_L = 0x42

        # Reading accelerometer (6 consecutive bytes)
        accel_data = self.i2c_bus.read_i2c_block_data(imu_address, ACCEL_XOUT_H, 6)

        # Reading gyroscope data (6 bytes)
        gyro_data = self.i2c_bus.read_i2c_block_data(imu_address, GYRO_XOUT_H, 6)

        # Reading temperature data (2 bytes)
        temp_data = self.i2c_bus.read_i2c_block_data(imu_address, TEMP_OUT_H, 2)

        imu_msg = self.parse_raw_data(accel_data, gyro_data, temp_data)

        return imu_msg
    
    def parse_raw_data(self, accel_data, gyro_data, temp_data):
        imu_msg = Imu()

        # Processing acceleration data
        accel_scale = 16384.0 # Scale factor (Check datasheet)
        # Combining acceleration data in respective axes
        accel_x = (accel_data[0] << 8) | accel_data[1]
        accel_y = (accel_data[2] << 8) | accel_data[3]
        accel_z = (accel_data[4] << 8) | accel_data[5]
        imu_msg.linear_acceleration.x = accel_x / accel_scale
        imu_msg.linear_acceleration.y = accel_y / accel_scale
        imu_msg.linear_acceleration.z = accel_z / accel_scale

        # Processing gyroscope data
        gyro_scale = 131.0 # Check datasheet
        gyro_x = (gyro_data[0] << 8) | gyro_data[1]
        gyro_y = (gyro_data[2] << 8) | gyro_data[3]
        gyro_z = (gyro_data[4] << 8) | gyro_data[5]
        imu_msg.angular_velocity.x = gyro_x / gyro_scale
        imu_msg.angular_velocity.y = gyro_y / gyro_scale
        imu_msg.angular_velocity.z = gyro_z / gyro_scale



        # Populate with orientation data
        
        
        
        temp_raw = (temp_data[0] << 8) | temp_data[1]
        temperature_celsius = (temp_raw / 340.0) + 36.53
        # Check datasheet for actual conversion
        imu_msg.temperature = temperature_celsius




        return imu_msg

