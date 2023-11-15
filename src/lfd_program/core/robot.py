
class RobotProgram:

    def __init__(self) -> None:
        pass

    def gripper_open(self):
        raise NotImplementedError()
    
    
    def gripper_grasp(self):
        raise NotImplementedError()
    

    def move(self, move_method, *args, **kwargs):
        move_method(*args, **kwargs)