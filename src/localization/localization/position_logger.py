#!/usr/bin/env python3
import csv

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from interfaces.srv import DeleteSavedPosition, GetSavedPosition, GetSavedPositionList


class PositionLoggerNode(Node):
    def __init__(self):
        super().__init__("position_logger")

        # position format: float, float, float, boolean (Latitude, Longitude, Timestamp, Error)
        self.positions = []
        self.declare_parameter("position_logger_logfile", "saved_positions.csv")
        self.filename = (
            self.get_parameter("position_logger_logfile")
            .get_parameter_value()
            .string_value
        )

        try:
            # if file exists, load data into the list
            with open(self.filename, newline="") as file:
                # skip header row
                reader = csv.reader(file)
                next(reader, None)
                for position in reader:
                    self.positions.append(
                        [
                            float(position[0]),
                            float(position[1]),
                            float(position[2]),
                            position[3] == "True",
                        ]
                    )
        except FileNotFoundError:
            # if it doesn't, create a new one with the header column
            with open(self.filename, "w", newline="") as file:
                csv.writer(file).writerow(
                    ["Latitude", "Longitude", "Timestamp", "Error"]
                )

        self.create_service(
            DeleteSavedPosition, "delete_saved_position", self.delete_position
        )
        self.create_service(GetSavedPosition, "get_saved_position", self.get_position)
        self.create_service(
            GetSavedPositionList, "get_saved_position_list", self.get_position_list
        )
        self.create_service(
            Trigger, "save_current_position", self.save_current_position
        )
        self.get_clock().now().to_msg()

    def get_position_list(self):
        return self.positions

    def get_position(self, index):
        try:
            return self.positions[index]
        except IndexError:
            return None  # TODO: I suspect ros will not appreciate this

    def save_current_position(self):
        position = []  # TODO: use ROS to get data
        # /fix (message type: navsatfix)
        self.positions.append(position)
        with open(self.filename, "a", newline="") as file:
            csv.writer(file).writerow(position)
        return [True, ""]

    def delete_position(self, index):
        try:
            self.positions.pop(index)
        except IndexError:
            return
        # if the item was removed successfully from the list, erase the file and write the list to it
        with open(self.filename, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Latitude", "Longitude", "Timestamp", "Error"])
            for position in self.positions:
                writer.writerow(position)


def main(args=None):
    rclpy.init(args=args)
    node = PositionLoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
