
import rospy
import telnetlib
import re
import math


class Cognex:

    def __init__(self):
        self.ip = rospy.get_param("~camera_ip")
        self.user = "admin"
        self.password = ""

    def connect(self):
        self.tn = telnetlib.Telnet(self.ip)
        telnet_user = self.user+'\r\n'
        self.tn.write(telnet_user.encode('ascii'))
        telnet_password = self.password + "\r\n"
        self.tn.write(telnet_password.encode('ascii'))
        # Logged in!

    def read(self, name = None):
        self.tn.write(b"SE8\r\n")
        rospy.sleep(0.5)
        self.tn.write(b"GVPattern_1.Result\r\n")
        rospy.sleep(0.5)
        output = self.tn.read_very_eager()

        return self._extract_pos(output)
    
    def _extract_pos(self,output):
        output_str = output.decode('latin-1')
        match = re.search(r'\(([^)]+)\) ([^ ]+)° score = ([^ ]+)', output_str)
        if match:
            result = match.group(0)
            numbers = re.findall(r"[-+]?\d*\.\d+|\d+", result)
            numbers = [float(num) for num in numbers][:3]
            return self._convert_units(numbers)
        else:
            return None
    
    def _convert_units(self,numbers):
        """
        numbers [x [mm],y [mm], angle [deg]]

        returns [x [m], y [m],  angle [rad]]
        """
        x_meters = numbers[0] / 1000
        y_meters = numbers[1] / 1000
        
        angle_radians = math.radians(numbers[2])
        
        return [x_meters, y_meters, angle_radians]
