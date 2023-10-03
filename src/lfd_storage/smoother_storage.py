
import json
import rospy
import pickle
import os

from lfd_interface.srv import GetDemonstration, DemoCount


class SmootherStorage(object):
    def __init__(self, demo_name):
        self.demo_name = demo_name
        self.working_dir = rospy.get_param("/working_dir")
        self.working_dir = os.path.join(self.working_dir, "smoother")

    def import_timing(self):
        path = os.path.join(self.working_dir, 
                            "timing/{}.pickle".format(self.demo_name))
        return self._import_pickle(path)

    def import_new_timing(self):
        path = os.path.join(self.working_dir, 
                            "timing_new/{}.pickle".format(self.demo_name))
        return self._import_pickle(path)
    
    def import_metadata(self):
        path = os.path.join(self.working_dir, 
                            "metadata/{}.pickle".format(self.demo_name))
        return self._import_pickle(path)
    
    def import_tolerances(self):
        path = os.path.join(self.working_dir, 
                            "tolerances/{}.pickle".format(self.demo_name))
        return self._import_pickle(path)

    #TODO
    # def import_waypoints_as_demo(self):
    #     demo = Demonstration()
    #     demo.read_from_json("waypoints/{}.json".format(self.demo_name))
    #     return demo
    
    def import_waypoints(self):
        path = os.path.join(self.working_dir, 
                            "waypoints/{}.json".format(self.demo_name))
        return self._import_json(path)        
    
    def import_original_traj(self):
        rospy.wait_for_service("get_demonstration")
        sc_lfd_storage = rospy.ServiceProxy("get_demonstration", GetDemonstration)
        resp = sc_lfd_storage(name=self.demo_name)
        return resp.Demonstration.joint_trajectory    

    def import_pydrake_traj(self):
        path = os.path.join(self.working_dir, 
                            "traj/{}.pickle".format(self.demo_name))
        return self._import_pickle(path)

    def import_dmp_traj(self):
        path = os.path.join(self.working_dir, 
                            "dmp/{}.pickle".format(self.demo_name))
        return self._import_pickle(path)          

    def _import_pickle(self, path):
        with open(path, 'rb') as file:
            return pickle.load(file)

    def _import_json(self, path):
        with open(path, 'r') as file:
            return json.loads(file.read())

    def export_pydrake_traj(self, traj):
        path = os.path.join(self.working_dir, 
                            "traj/{}.pickle".format(self.demo_name))
        self._export_pickle(path, traj)

    def export_timing(self, timing):
        path = os.path.join(self.working_dir, 
                            "timing/{}.pickle".format(self.demo_name))
        self._export_pickle(path, timing)
    
    def export_waypoints(self, waypoints):
        path = os.path.join(self.working_dir, 
                            "waypoints/{}.json".format(self.demo_name))
        self._export_json(path, waypoints)

    def _export_pickle(self, path, data):
        with open(path, 'wb') as file:
            pickle.dump(data,file)

    def _export_json(self, path, data):
        with open(path, 'w') as file:
            file.write(json.dumps(data))