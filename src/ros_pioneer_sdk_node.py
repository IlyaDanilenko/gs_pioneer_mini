#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, cv2
from cv_bridge import CvBridge
from pioneer_sdk import Pioneer
from rospy import Service, Publisher
from time import sleep
import numpy as np
from gs_interfaces.srv import Position, PositionResponse
from gs_interfaces.srv import Led, LedResponse
from gs_interfaces.srv import Event, EventResponse
from std_msgs.msg import ColorRGBA, Float32, Int32
from sensor_msgs.msg import Image

class PioneerMiniNode():
    def handle_event(self, req):
        if self.state_event != req.event:
            # send_log(f"send: Event - {req.event}")
            if req.event == 0:
                self.board.arm()
                self.state_callback_event = 56
            elif req.event == 1:
                self.board.takeoff()
                self.state_callback_event = 51
            elif req.event == 2:
                self.board.land()
                self.state_callback_event = 26
            elif req.event == 3:
                self.board.disarm()
                self.state_callback_event = 26
            else:
                return EventResponse(0)
            self.state_event = req.event
            self.callback_event_publisher.publish(self.state_callback_event)
        return EventResponse(1)

    def handle_board_led(self, req):
        if req.leds != self.state_board_led:
            for i in range(0, len(req.leds)):
                if req.leds[i] != self.state_board_led[i]:
                    self.board.led_control(i, req.leds[0], req.leds[1], req.leds[2])
                    self.state_board_led[i] = req.leds[i]
        return LedResponse(True)

    def handle_local_pos(self, req):
        request_position = [req.position.x,req.position.y,req.position.z]
        if request_position != self.state_position:
            self.board.go_to_local_point(request_position[0], request_position[1], request_position[3])
            if self.board.point_reached():
                self.state_callback_event = 42
                return PositionResponse(True)
        return PositionResponse(True)

    def __init__(self, rate = None):
        self.state_event = -1
        self.state_callback_event = 0
        self.state_position = [0.0, 0.0, 0.0]
        self.state_board_led=[]
        self.board = Pioneer()
        self.bridge = CvBridge()
        self.rate = rate

        self.local_position_service = Service("geoscan/flight/set_local_position", Position, self.handle_local_pos)
        self.board_led_service = Service("geoscan/led/board/set", Led, self.handle_board_led)
        self.event_service = Service("geoscan/flight/set_event", Event, self.handle_event)

        self.altitude_publisher = Publisher("geoscan/sensors/altitude", Float32, queue_size=10)
        self.image_publisher = Publisher("pioneer_mini_camera/image_raw", Image, queue_size=10)
        self.callback_event_publisher = Publisher("geoscan/flight/callback_event", Int32, queue_size=10)

        for _ in range(0,4):
            self.state_board_led.append(ColorRGBA())
    
    def __get_altitude(self):
        alt = self.board.get_dist_sensor_data()

        if alt is None:
            alt = 0.0

        return alt

    def __get_image(self):
        image = cv2.imdecode(np.frombuffer(self.board.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        return image

    def spin(self):
        self.altitude_publisher.publish(self.__get_altitude())
        self.image_publisher.publish(self.__get_image())
        
        if self.rate is not None:
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("ros_pioneer_sdk_node")
    rate = rospy.Rate(100)
    pioneer_mini_node = PioneerMiniNode(rate)
    while not rospy.is_shutdown():
        pioneer_mini_node.spin()
        