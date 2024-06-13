
import rospy
import telnetlib
import re
import math


class Cognex:

    def __init__(self, cam_config):
        self.ip = cam_config.general["camera_ip"]
        self.objects = cam_config.objects
        self.user = "admin"
        self.password = ""
        self.current_job = None
        # print(self.objects)

    def connect(self):
        self.tn = telnetlib.Telnet(self.ip)
        telnet_user = self.user+'\r\n'
        self.tn.write(telnet_user.encode('ascii'))
        telnet_password = self.password + "\r\n"
        self.tn.write(telnet_password.encode('ascii'))
        # print(self.read("ring"))
        # Logged in!

    def switch_job(self, job_id):
        self.tn.write(f"SJ{job_id}\r\n".encode('ascii'))
        rospy.sleep(1.0)
    
    def read(self, name):
        # if name is None:
        #     raise ValueError("Name of object to read is required")
        if self.current_job != name and name is not None:
            self.switch_job(self.objects[name]["id"])
            self.current_job = name
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
