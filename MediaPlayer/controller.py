from Intrinsic import load_intrinsic_calibration
from Extrinsic import load_extrinsic_calibration
import subprocess
import os

class calibration_utils():
    '''
    This class allows to load the calibration values of the vision system
        - Intrinsic: this property stores a matrix transformation that accounts for optical distortion. Only one required per camera set
        - Extrinsic: stores a matrix tranformation per camera. 
    '''
    def __init__(self):
        # Calibration variables
        self.intrinsic_calibration = None
        self.extrinsic_calibrations = None
    
    def load_calibration(self):

        path = os.getcwd()
        path_list = path.split(os.path.sep)
        path_final = os.path.join(os.path.sep,  *(path_list[1:-1]+['TestTrackVision', 'config']))
        
        os.chdir(path_final)
        path = os.getcwd()
        print(path)

        #shell=True, executable="/bin/bash"
        subprocess.Popen(['/bin/sh' , 'local_env_vars.sh'])
        '''
        # Calibration variables
        self.intrinsic_calibration = load_intrinsic_calibration(abs_path=os.path.dirname(
            os.getenv(key="CAM_PORTS_PATH")), file_name="cam_intrinsic_{}X{}.yaml".format(
                int(os.environ.get("VIDEO_WIDTH", 640)), int(os.environ.get("VIDEO_HEIGHT", 360))))

        self.extrinsic_calibrations = { key:load_extrinsic_calibration(
            abs_path=os.path.join(os.path.dirname(os.getenv(key="CAM_PORTS_PATH")), 
            "{}_extrinsic.yaml".format(key))) for key in ['CAM1', 'CAM2', 'CAM3'] }
        
        print('Load of calibration variables')
        '''
    
if __name__=='__main__':
    calibration_object = calibration_utils()
    calibration_object.load_calibration()


