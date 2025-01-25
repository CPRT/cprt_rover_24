#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_ens160
import board

from interfaces.msg import GasSensorReading


class GasSensor(Node):
    def __init__(self):
        super().__init__("gas_sensor")

        i2c = board.I2C()

        self.declare_parameter("sea_level_pressure_hpa", 1013.25)

        try:
            self.bms280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
            self.ens160 = adafruit_ens160.ENS160(i2c)
        except RuntimeError:
            raise RuntimeError("Gas Sensor Not Connected")
        
        self.bms280.sea_level_pressure = self.get_parameter("sea_level_pressure_hpa").get_parameter_value().double_value
        self.bms280.mode = adafruit_bme280.MODE_NORMAL

        # self.ens160.reset() # not necessary AFAIK
        self.ens160.mode = adafruit_ens160.MODE_STANDARD

        self.sensor_reading_pub = self.create_publisher(
            GasSensorReading, "gas_sensor", 10
        )
        self.create_timer(0.2, self.loop)

    def loop(self):
        if self.sensor_reading_pub.get_subscription_count() > 0:
            temperature = self.bms280.temperature
            humidity = self.bms280.humidity

            reading = GasSensorReading()
            reading.header.stamp = self.get_clock().now().to_msg()
            reading.temperature_c = temperature
            reading.pressure_pa = self.bms280.pressure * 100  # convert from hPa to Pa
            reading.humidity_rh = humidity

            self.ens160.temperature_compensation = temperature
            self.ens160.humidity_compensation = humidity
            
            reading.co2_ppm = self.ens160.eCO2
            reading.tvoc_ppb = self.ens160.TVOC

            self.sensor_reading_pub.publish(reading)


def main(args=None):
    rclpy.init(args=args)
    node = GasSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
