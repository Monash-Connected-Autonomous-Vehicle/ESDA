import sys

from robot_localization.srv import SetDatum
from sensor_msgs.msg import NavSatFix
import rclpy
from rclpy.node import Node


class InitialDatumSetter(Node):

    def __init__(self):
        super().__init__('initial_datum_setter')
        
        # set datum client
        self.cli = self.create_client(SetDatum, 'datum')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.set_datum_req = SetDatum.Request()

        # gps (navsatfix) subscription
        self.subscription = self.create_subscription(NavSatFix, 'navsatfix', self.send_initial_datum, 10)
        self.subscription  # prevent unused variable warning

    def send_initial_datum(self, navsatfix_msg):
        # initial orentation is neutral/no rotations
        self.set_datum_req.orientation.x = 0
        self.set_datum_req.orientation.y = 0
        self.set_datum_req.orientation.z = 0
        self.set_datum_req.orientation.w = 1
        
        # determine validity of GNSS reading before setting datum
        if (navsatfix_msg is not None and navsatfix_msg.status.status >= 0):
            print("DATUM SETTER SETTING: [" + str(navsatfix_msg.latitude) + str(navsatfix_msg.longitude) + str(navsatfix_msg.altitude) + "]")
        
            self.set_datum_req.position.latitude = navsatfix_msg.latitude
            self.set_datum_req.position.longitude = navsatfix_msg.longitude
            self.set_datum_req.position.altitude = navsatfix_msg.altitude
     
            self.future = self.cli.call_async(self.set_datum_req)
            rclpy.spin_until_future_complete(self, self.future)
            
            print(self.future.result())
            self.destroy_node()


def main():
    rclpy.init()
    initial_datum_setter = InitialDatumSetter()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
