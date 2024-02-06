import sys

from pymavlink import mavutil
import time
import threading

class FakeDrone:
    def __init__(self, port=8000):
        super().__init__()
        self.port = port
        self.ip = 'localhost'
        self.connection_method = 'udp'
        self.is_connected = False
        self.heartbeat_timeout = time.time()
        self.last_msg_time = time.time() - 2
        self.dist_send_time = time.time()
        self.battery_send_time = time.time()
        self.mavlink_socket = self.create_connection()
        self.dist = 0
        self.dist_append = 1
        self.battery_value = 75
        self.arm_result = 2
        self.arm_started = False
        self.begin_arm_started = False
        self.pioner_sdk_started = False
        self.wait_log_time = time.time()
        self.send_messages()

    def run(self) -> None:
        #message handler
        while True:
            pass

    def send_messages(self):
        while True:
            self.send_heartbeat()
            self.send_distance()
            self.send_battery()
            self.receive_message()
            # print(f'Arm started: {self.arm_started}')

    def send_heartbeat(self):
        if time.time() - self.heartbeat_timeout > 1:
            self.heartbeat_timeout = time.time()
            # print('send heartbeat')
            self.mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                   mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_distance(self):
        max_dist = 100
        if time.time() - self.dist_send_time > 1:
            self.dist_send_time = time.time()
            if self.dist >= max_dist:
                self.dist_append = -1
            elif self.dist <= 0:
                self.dist_append = 1
            self.dist += self.dist_append
            msg = mavutil.mavlink.MAVLink_distance_sensor_message(
                time_boot_ms=0,
                min_distance=0,
                max_distance=100, type=0, id=0, orientation=0, covariance=0,
                current_distance=self.dist
            )
            # print(msg.get_type())
            self.mavlink_socket.mav.send(msg)

    def send_battery(self):
        if time.time() - self.battery_send_time > 1:
            self.battery_send_time = time.time()
            if self.battery_value > 0:
                self.battery_value -= 1
                msg = mavutil.mavlink.MAVLink_battery_status_message(
                    id=1,
                    battery_function=3,
                    type=3,
                    temperature=25,
                    voltages=[self.battery_value, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    current_battery=100,
                    current_consumed=0,
                    energy_consumed=0,
                    battery_remaining=50,
                )
                # print(msg.get_type())
                self.mavlink_socket.mav.send(msg)


    def receive_message(self):
        # if time.time() - self.last_msg_time > 1:
        #     self.last_msg_time = time.time()
        msg = self.mavlink_socket.recv_msg()
        if msg is not None:
            self.last_msg_time = time.time()
            # print(msg)
            if msg.get_type() == 'COMMAND_LONG':
                self.send_long_response(msg)
                # print(self.arm_started)
        self.check_messages(msg)


    def create_connection(self):
        mav_socket = mavutil.mavlink_connection('%s:%s:%s' % (self.connection_method, self.ip, self.port))
        return mav_socket

    def change_port(self, port):
        self.port = port

    def send_long_response(self, message):
        if message.command == 400 and message.param1 == 1:
            self.arm()
            # print('skip thread')
            self.begin_arm_started = True

    def arm(self):
        # print('start arm')
        self.arm_result = 5
        start_arm_time = time.time()
        while self.arm_result:
            if time.time() - start_arm_time >= 4:
                self.arm_started = True
                self.arm_result = 0
            msg = mavutil.mavlink.MAVLink_command_ack_message(
                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                result=self.arm_result,
            )
            # print('send_long_response')
            # print(msg.get_type())
            self.mavlink_socket.mav.send(msg)

    def check_messages(self, msg):
        if msg is None and time.time() - self.last_msg_time > 2:
            if time.time() - self.wait_log_time > 1:
                self.wait_log_time = time.time()
                print('Wainting piosdk connection...')
        elif not self.pioner_sdk_started and msg is not None:
            self.pioner_sdk_started = True
            print('Piodesk connected')

if __name__ == '__main__':
    if len(sys.argv) > 1:
        drone = FakeDrone(int(sys.argv[1]))
    else:
        drone = FakeDrone()

