#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive) [--model=<model>] [--js] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer|latent)] [--camera=(single|stereo)] [--meta=<key:value> ...] [--myconfig=<filename>]
    manage.py (train) [--tub=<tub1,tub2,..tubn>] [--file=<file> ...] (--model=<model>) [--transfer=<model>] [--type=(linear|categorical|rnn|imu|behavior|3d|localizer)] [--continuous] [--aug] [--myconfig=<filename>]


Options:
    -h --help               Show this screen.
    --js                    Use physical joystick.
    -f --file=<file>        A text file containing paths to tub files, one per line. Option may be used more than once.
    --meta=<key:value>      Key/Value strings describing describing a piece of meta data about this drive. Option may be used more than once.
    --myconfig=filename     Specify myconfig file to use. 
                            [default: myconfig.py]
"""
import os
import time

from docopt import docopt
import numpy as np

import donkeycar as dk
######################################
'''超声波'''
import pigpio        #树莓派引脚类
pi = pigpio.pi()

#加入回调onenet的数据
import requests
import json

#加入gps
import L76X
import math
import datetime
import coordTransform_utils
import re

import sys
import imp

#语音
imp.reload(sys)

#onenet
headers = {'api-key': 'fK=PEyZjPraXWFdfQqfR2K88tEU='}
data = {'datastreams': [{'id': 'latitude', 'datapoints': [{'value': 7}]}]}
jdata = json.dumps(data)
#puturl = 'http://api.heclouds.com/devices/627018347/datapoints'
puturl = 'http://api.heclouds.com/devices/639504816/datapoints'
#geturl = 'http://api.heclouds.com/devices/627018347/datastreams/temp'
geturl1 = 'http://api.heclouds.com/devices/639504816/datastreams/latitude'
geturl2 = 'http://api.heclouds.com/devices/639504816/datastreams/longitude'
count=18  #全依靠GPS路径转弯计数
#经纬度
latitudeG=39.961133#初始化纬度全局变量
longitudeG=116.355525#初始化经度全局变量
detection=0#检测到经纬度在转折点范围内
Num=0 #当前处在路线中第几段直线
getPoints=0 #在开启小车的时候进行路径规划并获取经纬度的操作，仅执行一次
distance=0#小车距离下一个目标转弯点的距离
addNum=0#检测是否满足切换下一个转弯点经纬度的条件
straightCount=0#直行占空比
flag=1#当前阶段听GPS还是图像识别，0是GPS
count1=10#图像识别路径转弯计数


from donkeycar.parts.csbcsb import Sensor   #传感器类
#range = Sensor(pi, cfg.RANGE_GPIOS)
#V.add(range, outputs=['range/cms'], threaded=True)


#######################################
#import parts
from donkeycar.parts.transform import Lambda, TriggeredCallback, DelayedTrigger
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.controller import LocalWebController, \
    JoystickController, WebFpv
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.behavior import BehaviorPart
from donkeycar.parts.file_watcher import FileWatcher
from donkeycar.parts.launch import AiLaunch
from donkeycar.utils import *
from math import radians, cos, sin, asin, sqrt

def drive(cfg, model_path=None, use_joystick=False, model_type=None, camera_type='single', meta=[]):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    if cfg.DONKEY_GYM:
        #the simulator will use cuda and then we usually run out of resources
        #if we also try to use cuda. so disable for donkey_gym.
        os.environ["CUDA_VISIBLE_DEVICES"]="-1"

    if model_type is None:
        if cfg.TRAIN_LOCALIZER:
            model_type = "localizer"
        elif cfg.TRAIN_BEHAVIORS:
            model_type = "behavior"
        else:
            model_type = cfg.DEFAULT_MODEL_TYPE

    #Initialize car
    V = dk.vehicle.Vehicle()

    print("cfg.CAMERA_TYPE", cfg.CAMERA_TYPE)
    if camera_type == "stereo":

        if cfg.CAMERA_TYPE == "WEBCAM":
            from donkeycar.parts.camera import Webcam

            camA = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam = 0)
            camB = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam = 1)

        elif cfg.CAMERA_TYPE == "CVCAM":
            from donkeycar.parts.cv import CvCam

            camA = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam = 0)
            camB = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, iCam = 1)
        else:
            raise(Exception("Unsupported camera type: %s" % cfg.CAMERA_TYPE))

        V.add(camA, outputs=['cam/image_array_a'], threaded=True)
        V.add(camB, outputs=['cam/image_array_b'], threaded=True)

        from donkeycar.parts.image import StereoPair

        V.add(StereoPair(), inputs=['cam/image_array_a', 'cam/image_array_b'],
            outputs=['cam/image_array'])
    elif cfg.CAMERA_TYPE == "D435":
        from donkeycar.parts.realsense435i import RealSense435i
        cam = RealSense435i(
            enable_rgb=cfg.REALSENSE_D435_RGB,
            enable_depth=cfg.REALSENSE_D435_DEPTH,
            enable_imu=cfg.REALSENSE_D435_IMU,
            device_id=cfg.REALSENSE_D435_ID)
        V.add(cam, inputs=[],
              outputs=['cam/image_array', 'cam/depth_array',
                       'imu/acl_x', 'imu/acl_y', 'imu/acl_z',
                       'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z'],
              threaded=True)

    else:
        if cfg.DONKEY_GYM:
            from donkeycar.parts.dgym import DonkeyGymEnv

        inputs = []
        threaded = True
        if cfg.DONKEY_GYM:
            from donkeycar.parts.dgym import DonkeyGymEnv 
            cam = DonkeyGymEnv(cfg.DONKEY_SIM_PATH, host=cfg.SIM_HOST, env_name=cfg.DONKEY_GYM_ENV_NAME, conf=cfg.GYM_CONF, delay=cfg.SIM_ARTIFICIAL_LATENCY)
            threaded = True
            inputs = ['angle', 'throttle']
        elif cfg.CAMERA_TYPE == "PICAM":
            from donkeycar.parts.camera import PiCamera
            cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, framerate=cfg.CAMERA_FRAMERATE, vflip=cfg.CAMERA_VFLIP, hflip=cfg.CAMERA_HFLIP)
        elif cfg.CAMERA_TYPE == "WEBCAM":
            from donkeycar.parts.camera import Webcam
            cam = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        elif cfg.CAMERA_TYPE == "CVCAM":
            from donkeycar.parts.cv import CvCam
            cam = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        elif cfg.CAMERA_TYPE == "CSIC":
            from donkeycar.parts.camera import CSICamera
            cam = CSICamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, framerate=cfg.CAMERA_FRAMERATE, gstreamer_flip=cfg.CSIC_CAM_GSTREAMER_FLIP_PARM)
        elif cfg.CAMERA_TYPE == "V4L":
            from donkeycar.parts.camera import V4LCamera
            cam = V4LCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, framerate=cfg.CAMERA_FRAMERATE)
        elif cfg.CAMERA_TYPE == "MOCK":
            from donkeycar.parts.camera import MockCamera
            cam = MockCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
        elif cfg.CAMERA_TYPE == "IMAGE_LIST":
            from donkeycar.parts.camera import ImageListCamera
            cam = ImageListCamera(path_mask=cfg.PATH_MASK)
        else:
            raise(Exception("Unkown camera type: %s" % cfg.CAMERA_TYPE))

        V.add(cam, inputs=inputs, outputs=['cam/image_array'], threaded=threaded)

    if use_joystick or cfg.USE_JOYSTICK_AS_DEFAULT:
        #modify max_throttle closer to 1.0 to have more power
        #modify steering_scale lower than 1.0 to have less responsive steering
        if cfg.CONTROLLER_TYPE == "MM1":
            from donkeycar.parts.robohat import RoboHATController            
            ctr = RoboHATController(cfg)
        elif "custom" == cfg.CONTROLLER_TYPE:
            #
            # custom controller created with `donkey createjs` command
            #
            from my_joystick import MyJoystickController
            ctr = MyJoystickController(
                throttle_dir=cfg.JOYSTICK_THROTTLE_DIR,
                throttle_scale=cfg.JOYSTICK_MAX_THROTTLE,
                steering_scale=cfg.JOYSTICK_STEERING_SCALE,
                auto_record_on_throttle=cfg.AUTO_RECORD_ON_THROTTLE)
            ctr.set_deadzone(cfg.JOYSTICK_DEADZONE)
        else:
            from donkeycar.parts.controller import get_js_controller

            ctr = get_js_controller(cfg)

            if cfg.USE_NETWORKED_JS:
                from donkeycar.parts.controller import JoyStickSub
                netwkJs = JoyStickSub(cfg.NETWORK_JS_SERVER_IP)
                V.add(netwkJs, threaded=True)
                ctr.js = netwkJs
        
        V.add(ctr, 
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

    else:
        #This web controller will create a web server that is capable
        #of managing steering, throttle, and modes, and more.
        ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT, mode=cfg.WEB_INIT_MODE)
        
        V.add(ctr,
          inputs=['cam/image_array', 'tub/num_records'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

    #this throttle filter will allow one tap back for esc reverse
    th_filter = ThrottleFilter()
    V.add(th_filter, inputs=['user/throttle'], outputs=['user/throttle'])

    #See if we should even run the pilot module.
    #This is only needed because the part run_condition only accepts boolean
    class PilotCondition:
        def run(self, mode):
            if mode == 'user':
                return False
            else:
                return True

    V.add(PilotCondition(), inputs=['user/mode'], outputs=['run_pilot'])
    
    
    ############################################3
    '''
    class Sensor():
        
        """
        表示连接到Donkey Car的距离测量传感器类。++
        """
        def __init__(self, pi, range_gpios):
            """
            初始化超声波传感器驱动器
            """
            self.pi = pi
            if range_gpios is not None:
                from donkeycar.parts.csb2 import Driver
                self.range = Driver(self.pi, range_gpios[0], range_gpios[1])
            else:
                self.range = None
            self.distance = None
    
        def update_loop_body(self):
            """
            获取测量距离，并将其存储在实例变量距离中
            """
            self.distance = self.range.read()
            time.sleep(2)
    
        def update(self):
            """
            实现在另一个线程中执行的处理。
           调用距离传感器类并定期将结果测量为实例变量distance并存储        
            """
            if self.range is not None:
                while True:
                    self.update_loop_body()
            else:
                return None
        
        def run_threaded(self):
            return self.distance
        
        def run(self):
            if self.range is not None:
                self.distance = self.range.read()
            return self.distance
        
        def shutdown(self):
            """
            关闭线程
            """
            if self.range is not None:
                self.range.cancel()
                self.range = None
                self.distance = None
   '''
    range = Sensor(pi, cfg.RANGE_GPIOS)
    V.add(range, outputs=['range/cms'], threaded=True)


    #############################################

    class LedConditionLogic:
        def __init__(self, cfg):
            self.cfg = cfg

        def run(self, mode, recording, recording_alert, behavior_state, model_file_changed, track_loc):
            #returns a blink rate. 0 for off. -1 for on. positive for rate.

            if track_loc is not None:
                led.set_rgb(*self.cfg.LOC_COLORS[track_loc])
                return -1

            if model_file_changed:
                led.set_rgb(self.cfg.MODEL_RELOADED_LED_R, self.cfg.MODEL_RELOADED_LED_G, self.cfg.MODEL_RELOADED_LED_B)
                return 0.1
            else:
                led.set_rgb(self.cfg.LED_R, self.cfg.LED_G, self.cfg.LED_B)

            if recording_alert:
                led.set_rgb(*recording_alert)
                return self.cfg.REC_COUNT_ALERT_BLINK_RATE
            else:
                led.set_rgb(self.cfg.LED_R, self.cfg.LED_G, self.cfg.LED_B)

            if behavior_state is not None and model_type == 'behavior':
                r, g, b = self.cfg.BEHAVIOR_LED_COLORS[behavior_state]
                led.set_rgb(r, g, b)
                return -1 #solid on

            if recording:
                return -1 #solid on
            elif mode == 'user':
                return 1
            elif mode == 'local_angle':
                return 0.5
            elif mode == 'local':
                return 0.1
            return 0

    if cfg.HAVE_RGB_LED and not cfg.DONKEY_GYM:
        from donkeycar.parts.led_status import RGB_LED
        led = RGB_LED(cfg.LED_PIN_R, cfg.LED_PIN_G, cfg.LED_PIN_B, cfg.LED_INVERT)
        led.set_rgb(cfg.LED_R, cfg.LED_G, cfg.LED_B)

        V.add(LedConditionLogic(cfg), inputs=['user/mode', 'recording', "records/alert", 'behavior/state', 'modelfile/modified', "pilot/loc"],
              outputs=['led/blink_rate'])

        V.add(led, inputs=['led/blink_rate'])

    def get_record_alert_color(num_records):
        col = (0, 0, 0)
        for count, color in cfg.RECORD_ALERT_COLOR_ARR:
            if num_records >= count:
                col = color
        return col

    class RecordTracker:
        def __init__(self):
            self.last_num_rec_print = 0
            self.dur_alert = 0
            self.force_alert = 0

        def run(self, num_records):
            if num_records is None:
                return 0

            if self.last_num_rec_print != num_records or self.force_alert:
                self.last_num_rec_print = num_records

                if num_records % 10 == 0:
                    print("recorded", num_records, "records")

                if num_records % cfg.REC_COUNT_ALERT == 0 or self.force_alert:
                    self.dur_alert = num_records // cfg.REC_COUNT_ALERT * cfg.REC_COUNT_ALERT_CYC
                    self.force_alert = 0

            if self.dur_alert > 0:
                self.dur_alert -= 1

            if self.dur_alert != 0:
                return get_record_alert_color(num_records)

            return 0

    rec_tracker_part = RecordTracker()
    V.add(rec_tracker_part, inputs=["tub/num_records"], outputs=['records/alert'])

    if cfg.AUTO_RECORD_ON_THROTTLE and isinstance(ctr, JoystickController):
        #then we are not using the circle button. hijack that to force a record count indication
        def show_record_acount_status():
            rec_tracker_part.last_num_rec_print = 0
            rec_tracker_part.force_alert = 1
        ctr.set_button_down_trigger('circle', show_record_acount_status)

    #Sombrero
    if cfg.HAVE_SOMBRERO:
        from donkeycar.parts.sombrero import Sombrero
        s = Sombrero()

    #IMU
    if cfg.HAVE_IMU:
        from donkeycar.parts.imu import IMU
        imu = IMU(sensor=cfg.IMU_SENSOR, dlp_setting=cfg.IMU_DLP_CONFIG)
        V.add(imu, outputs=['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
            'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z'], threaded=True)

    class ImgPreProcess():
        '''
        preprocess camera image for inference.
        normalize and crop if needed.
        '''
        def __init__(self, cfg):
            self.cfg = cfg

        def run(self, img_arr):
            return normalize_and_crop(img_arr, self.cfg)

    if "coral" in model_type:
        inf_input = 'cam/image_array'
    else:
        inf_input = 'cam/normalized/cropped'
        V.add(ImgPreProcess(cfg),
            inputs=['cam/image_array'],
            outputs=[inf_input],
            run_condition='run_pilot')

    # Use the FPV preview, which will show the cropped image output, or the full frame.
    if cfg.USE_FPV:
        V.add(WebFpv(), inputs=['cam/image_array'], threaded=True)

    #Behavioral state
    if cfg.TRAIN_BEHAVIORS:
        bh = BehaviorPart(cfg.BEHAVIOR_LIST)
        V.add(bh, outputs=['behavior/state', 'behavior/label', "behavior/one_hot_state_array"])
        try:
            ctr.set_button_down_trigger('L1', bh.increment_state)
        except:
            pass

        inputs = [inf_input, "behavior/one_hot_state_array"]
    #IMU
    elif model_type == "imu":
        assert(cfg.HAVE_IMU)
        #Run the pilot if the mode is not user.
        inputs=[inf_input,
            'imu/acl_x', 'imu/acl_y', 'imu/acl_z',
            'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z']
    else:
        inputs=[inf_input]

    def load_model(kl, model_path):
        start = time.time()
        print('loading model', model_path)
        kl.load(model_path)
        print('finished loading in %s sec.' % (str(time.time() - start)) )

    def load_weights(kl, weights_path):
        start = time.time()
        try:
            print('loading model weights', weights_path)
            kl.model.load_weights(weights_path)
            print('finished loading in %s sec.' % (str(time.time() - start)) )
        except Exception as e:
            print(e)
            print('ERR>> problems loading weights', weights_path)

    def load_model_json(kl, json_fnm):
        start = time.time()
        print('loading model json', json_fnm)
        from tensorflow.python import keras
        try:
            with open(json_fnm, 'r') as handle:
                contents = handle.read()
                kl.model = keras.models.model_from_json(contents)
            print('finished loading json in %s sec.' % (str(time.time() - start)) )
        except Exception as e:
            print(e)
            print("ERR>> problems loading model json", json_fnm)

    if model_path:
        #When we have a model, first create an appropriate Keras part
        kl = dk.utils.get_model_by_type(model_type, cfg)

        model_reload_cb = None

        if '.h5' in model_path or '.uff' in model_path or 'tflite' in model_path or '.pkl' in model_path:
            #when we have a .h5 extension
            #load everything from the model file
            load_model(kl, model_path)

            def reload_model(filename):
                load_model(kl, filename)

            model_reload_cb = reload_model

        elif '.json' in model_path:
            #when we have a .json extension
            #load the model from there and look for a matching
            #.wts file with just weights
            load_model_json(kl, model_path)
            weights_path = model_path.replace('.json', '.weights')
            load_weights(kl, weights_path)

            def reload_weights(filename):
                weights_path = filename.replace('.json', '.weights')
                load_weights(kl, weights_path)

            model_reload_cb = reload_weights

        else:
            print("ERR>> Unknown extension type on model file!!")
            return

        #this part will signal visual LED, if connected
        V.add(FileWatcher(model_path, verbose=True), outputs=['modelfile/modified'])

        #these parts will reload the model file, but only when ai is running so we don't interrupt user driving
        V.add(FileWatcher(model_path), outputs=['modelfile/dirty'], run_condition="ai_running")
        V.add(DelayedTrigger(100), inputs=['modelfile/dirty'], outputs=['modelfile/reload'], run_condition="ai_running")
        V.add(TriggeredCallback(model_path, model_reload_cb), inputs=["modelfile/reload"], run_condition="ai_running")

        outputs=['pilot/angle', 'pilot/throttle']

        if cfg.TRAIN_LOCALIZER:
            outputs.append("pilot/loc")

        V.add(kl, inputs=inputs,
            outputs=outputs,
            run_condition='run_pilot')
    
    if cfg.STOP_SIGN_DETECTOR:
        from donkeycar.parts.object_detector.stop_sign_detector import StopSignDetector
        V.add(StopSignDetector(cfg.STOP_SIGN_MIN_SCORE, cfg.STOP_SIGN_SHOW_BOUNDING_BOX), inputs=['cam/image_array', 'pilot/throttle'], outputs=['pilot/throttle', 'cam/image_array'])

#GPS控制线程，将硬件检测到的经纬度生成全局变量方便小车实时受控，并上传到onenet给微信小程序
    class GPS_Control:
     def __init__(self):
        self.latitude = 0
        self.longitude = 0

     def run_threaded(self):
        return self.latitude, self.longitude

     def update(self):
        while True:
            self.latitude, self.longitude = self.run()
            print(self.latitude)
     def run(self): 
              #GPS模块读取数据
              x=L76X.L76X()
              x.L76X_Set_Baudrate(9600)
              x.L76X_Send_Command(x.SET_NMEA_BAUDRATE_115200)
              time.sleep(0.5)
              x.L76X_Set_Baudrate(115200)
              x.L76X_Gat_GNRMC()
              if(x.Status == 1):
                print ('Already positioned')
              else:
                print ('No positioning')

              print ('Time %d:'%x.Time_H+'%d:'%x.Time_M+'%d'%x.Time_S)
          
              print ('Lat = %f'%x.Lat+'Lon = %f'%x.Lon)
#              x.L76X_Baidu_Coordinates(x.Lat, x.Lon)
#              print ('Baidu coordinate %f'%x.Lat_Baidu+',%f'%x.Lon_Baidu)    
              x.L76X_Baidu_Coordinates(x.Lat, x.Lon)
              print ('Baidu coordinate %f'%x.Lat_Baidu+',%f'%x.Lon_Baidu)  
              #百度坐标系转高德坐标系
              amap=coordTransform_utils.bd09_to_gcj02(x.Lon_Baidu, x.Lat_Baidu)
              print ('AMap coordinate %f'%amap[0]+',%f'%amap[1])
              global latitudeG
              global longitudeG
              latitudeG=format(amap[1],'.6f')
              longitudeG=format(amap[0],'.6f')
              a1 = requests.get(url=geturl1, headers=headers)
              print(a1.text)

              a2 = requests.get(url=geturl2, headers=headers)
              print(a2.text)
              latitude = amap[1]  # 经纬度全局变量赋值并上传
              longitude = amap[0]
              CurTime = datetime.datetime.now()
              value_Lat = {'datastreams': [{"id": "latitude", "datapoints": [{"at": CurTime.isoformat(), "value": latitude}]}]}
              value_Lon = {'datastreams': [{"id": "longitude", "datapoints": [{"at": CurTime.isoformat(), "value": longitude}]}]}
              jdata_Lat = json.dumps(value_Lat)  # 对数据进行JSON格式化编码
              jdata_Lon = json.dumps(value_Lon) 
              print("当前的标准时间为： %s" % CurTime.isoformat())
              print("上传的纬度值为: %.3f" % latitude)
              print("上传的经度值为: %.3f" % longitude)

        # 打印json内容
              print(jdata_Lat)
              print(jdata_Lon)
              r1 = requests.post(url=puturl, headers=headers, data=jdata_Lat)
              print(r1.text)
              r2 = requests.post(url=puturl, headers=headers, data=jdata_Lon)
              print(r2.text)
              #resp = http_put()
              #print("OneNET请求结果:\n %s" %resp)
              #time.sleep(5)        
              return (latitude,longitude)
    V.add(GPS_Control(), inputs=[], outputs=['mypilot/latitude','mypilot/longitude'], threaded=True)
    


    #Choose what inputs should change the car.
    class DriveMode():
        def run(self, mode,
                    user_angle, user_throttle,
                    pilot_angle, pilot_throttle):
           
            #超声波距离
            dis=range.dis_get()
            print('dis=',dis)
            if mode == 'user':
                return user_angle, user_throttle

            elif mode == 'local_angle':
                return pilot_angle if pilot_angle else 0.0, user_throttle

            else:
                   #超声波检测到前方有障碍物停车，无法和后面GPS功能同时工作，因此以备注形式放在这里
                   # if(dis<50):
                   #      user_throttle=0.00
                   #      user_angle=0.00
                   global count
                   global latitudeG
                   global longitudeG
                   global detection
                   global Num
                   global getPoints
                   global distance
                   global addNum
                   global straightCount
                   global count1
                   global flag

                   url = "https://api.heclouds.com/devices/639504816/datapoints?datastream_id=led_pi"
                   API_KEY = "fK=PEyZjPraXWFdfQqfR2K88tEU="
                   headers = {'api-key':API_KEY}
                    #获得微信小程序传来的是否开启小车结果0/1并打印
                   res = requests.get(url, headers=headers)
                   t: str = res.text
                   params = json.loads(t)
                   x = params['data']['datastreams']
                   y=x[0]['datapoints'][0]['value']#成功取出onenet上数据
                   print(y)
                   print(type(y))
                  
                   
                   url = "https://api.heclouds.com/devices/639504816/datapoints?datastream_id=mode"
                    #获得微信小程序传来的当前小车执行模式1/2/3/4结果并打印
                   res1 = requests.get(url, headers=headers)
                   t1: str = res1.text
                   params1 = json.loads(t1)
                   x1 = params1['data']['datastreams']
                   y1=x1[0]['datapoints'][0]['value']#成功取出onenet上数据
                   print(y1)
                   print(type(y1))
                   
# =============================================================================
#                    if(y==1&&getPoints==0)：
#                    将getPoints置为1，进行路径规划
# =============================================================================
      
# =============================================================================
#                    #路径规划API实时生成路径（选定模式并点击开始导航后，根据当前所处位置和终点，即使生成路径规划）
#                    #路径规划结果最终体现在action，turnPos和destination数组，以及转折点个数stepNum上
#                    #模拟时我们为了检验效果是否良好，为了方便，因此指定起终点后单独运行的路径规划代码，将获得的结果数组以及stepNum写进代码中
#                    #可以看到代码可以应对各种转向情况，并不是写死的，特此说明
#                    url="https://restapi.amap.com/v4/direction/bicycling?origin=116.355568,39.960524&destination=116.355772,39.960697&extensions=all&output=xml&key=b506637e16ad5d7a76d1d9ccb43abad5"
#                    API_KEY = "b506637e16ad5d7a76d1d9ccb43abad5"
#                    headers = {'api-key':API_KEY}
#                    res = requests.get(url, headers=headers)
#                    t: str = res.text
#                    params = json.loads(t)
#                    x = params['data']['paths'][0]['steps']
#                    #step=[[]for m in range(len(x))]
#                    stepNum=len(x)
#                    action=[[]for m in range(len(x))]
#                    turnPos=[[]for m in range(len(x))]
#                    for m in range(len(x)-1):
#                         lon_lat=re.split(r";|,",x[m]['polyline'])
#                         length=int(len(lon_lat)/2)
#                         #b=[[[]for k in range(2)] for i in range(length)]
#                         c=[[]for k in range(2)]
#                         for i in range(length):
#                             #b[i][0].append(float(lon_lat[2*(i)]))
#                             #b[i][1].append(float(lon_lat[2*(i)+1]))
#                             if(i==length-1):
#                                 c[0].append(float(lon_lat[0]))
#                                 c[1].append(float(lon_lat[1]))
#                         #step[m]=b
#                         action[m]=x[m]['action']
#                         turnPos[m]=c
#                     print(action)
#                     #print(step)
#                     print(turnPos)
# =============================================================================
                   


                    #路径规划API实时生成路径（选定模式并点击开始导航后，根据当前所处位置和终点，即使生成路径规划）
                    #路径规划结果最终体现在action，turnPos和destination数组，以及转折点个数stepNum上
                    #模拟时我们为了检验效果是否良好，为了方便，因此指定起终点后单独运行的路径规划代码，将获得的结果数组以及stepNum写进代码中
                    #可以看到代码可以应对各种转向情况，并不是写死的，只是将上述变量直接写入代码了
                    #几个模式代码之间（除了导览模式要求多次播报外），区别仅在于这几个变量不同，语音播报内容不同，特此说明
            
            if(y=='1'):   #微信小程序选择开始导航
                if(y1==4):      #选择模式为校园导览
                   # #起点116.356367,39.960714
                   action=['右转','右转','右转','']
                   turnPos=[[[116.356329],[39.961273]],[[116.356413],[39.961233]],[[116.356335],[39.961119]],[[116.356238],[39.961195]]]
                   destination=[[116.356329],[39.961273]]
                   straightCount=straightCount+1
                   #球坐标系计算当前所处位置和下一个转折点之间的距离（单位米）
                   if(Num<4-1):
                     lon1,lat1,lon2,lat2=map(radians,[float(longitudeG),float(latitudeG), turnPos[Num+1][0][0], turnPos[Num+1][1][0]])
                   if(Num==4-1):
                     lon1,lat1,lon2,lat2=map(radians,[float(longitudeG),float(latitudeG), destination[0][0], destination[1][0]])
                   dlon = lon2-lon1
                   dlat = lat2- lat1
                   a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
                   c = 2 * asin(sqrt(a)) 
                   r = 6371 # 地球平均半径，单位为公里
                   distance=c * r * 1000
                               
                   print(distance)
                   print(longitudeG)
                   print(latitudeG)
                   print('num')
                   print(Num)
                   print('count')
                   print(count)
                   if(distance<6):  #距离转折点距离小于6米
                       detection=detection+1   
                       if(detection==1):   #在转折点范围内第一次检测到转折点 计数控制初始化
                            count=18
                            addNum=1
                       if(count>13):      #count在14～18期间小车根据action数组中的值转向
                         if(action[Num]=='左转'):
                           pilot_angle=-0.17
                           pilot_throttle=0.2
                           print('左转')
                         if(action[Num]=='右转'):
                           pilot_angle=0.17
                           pilot_throttle=0.2
                           print('右转')
                         count=count-1
                       if count>0 and count<=13:     #转向结束后停车播报
                          pilot_angle=0
                          pilot_throttle=0
                          count=count-1
                          print('暂停播报')
                          if(Num==0):
                            music_path_1 = '/home/pi/mycar/music/spot1.mp3'
                            os.system('mplayer %s' % music_path_1)
                          if(Num==1):
                            music_path_2 = '/home/pi/mycar/music/spot2.mp3'
                            os.system('mplayer %s' % music_path_2)
                          if(Num==2):
                            music_path_3 = '/home/pi/mycar/music/spot3.mp3'
                            os.system('mplayer %s' % music_path_3)
                          if(Num==3):
                            music_path_4 = '/home/pi/mycar/music/spot4.mp3'
                            os.system('mplayer %s' % music_path_4)
                       if(count==0):       #播报结束直行
                         if straightCount%3==0 or straightCount%3==2:
                            pilot_angle=0
                            pilot_throttle=0.2
                         else:
                            pilot_angle=0.01
                            pilot_throttle=0.2
                         print('直行111')
                       if(Num==4-1):      #到达终点范围内，停车
                          pilot_angle=0
                          pilot_throttle=0
                          print('停车')
                      
                          
                   if(distance>=6):     #距离转折点距离大于6米，转折点检测变量清零（为下一次检测做好准备），直行
                       detection=0
                       if straightCount%5==0 or straightCount%5==2 or straightCount%5==4:
                            pilot_angle=0
                            pilot_throttle=0.2
                       else:
                            pilot_angle=0.001
                            pilot_throttle=0.2
                       print('直行222')
                       

                       if(addNum==1): #转折点切换后，转折点切换变量置零
                           Num=Num+1
                           addNum=0
                   print('pilot_throttle')    
                   print(pilot_throttle)            
                   print('pilot_angle')    
                   print(pilot_angle)   
                   #超声波检测到障碍物距离小于50厘米停车（控制权优先级高于GPS，因此放在最后）
                   if(dis<50):
                    pilot_throttle=0.00
                    pilot_angle=0.00
                   return pilot_angle if pilot_angle else 0.0, pilot_throttle * cfg.AI_THROTTLE_MULT if pilot_throttle else 0.0


 
                 
                if(y1==2):    #领钥匙模式
                   action=['左转', '']
                   turnPos=[[[116.3573], [39.960767]], [[116.357236], [39.962264]]]
                   destination=[[116.356716],[39.96226]]
                   straightCount=straightCount+1
                   #球坐标系计算当前所处位置和下一个转折点之间的距离（单位米）
                   if(Num<2-1):
                     lon1,lat1,lon2,lat2=map(radians,[float(longitudeG),float(latitudeG), turnPos[Num+1][0][0], turnPos[Num+1][1][0]])
                   if(Num==2-1):
                     lon1,lat1,lon2,lat2=map(radians,[float(longitudeG),float(latitudeG), destination[0][0], destination[1][0]])
                   dlon = lon2-lon1
                   dlat = lat2- lat1
                   a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
                   c = 2 * asin(sqrt(a)) 
                   r = 6371 # 地球平均半径，单位为公里
                   distance=c * r * 1000
                               
                   print(distance)
                   print(longitudeG)
                   print(latitudeG)
                   print('num')
                   print(Num)
                   print('count')
                   print(count)
                   if(distance<6):   #距离转折点距离小于6米
                       detection=detection+1
                       if(detection==1):  #在转折点范围内第一次检测到转折点 计数控制初始化
                            count=18
                            addNum=1
                       if(count>13):    #count在14～18期间小车根据action数组中的值转向
                         if(action[Num]=='左转'):
                           pilot_angle=-0.15
                           pilot_throttle=0.22
                           print('左转')
                         if(action[Num]=='右转'):
                           pilot_angle=0.17
                           pilot_throttle=0.22
                           print('右转')
                         count=count-1
                       if count>0 and count<=13 and Num==1:   #即将到达终点播报注意事项
                          pilot_angle=0
                          pilot_throttle=0
                          print('暂停播报')
                          if (count==13):
                            music_path_2 = '/home/pi/mycar/music/key.mp3'
                            os.system('mplayer %s' % music_path_2)
                          count=count-1
                       if count>0 and count<=13 and Num==0:
                          count=0
                       if(count==0):   #转向结束后直行
                         if straightCount%5==1 or straightCount%5==3:
                            pilot_angle=0
                            pilot_throttle=0.22
                         else:
                            pilot_angle=0.04
                            pilot_throttle=0.22
                         print('直行111')
                       if(Num==2-1):  #到达终点停车
                          pilot_angle=0
                          pilot_throttle=0
                          print('停车')
                      
                          
                   if(distance>=6):  #距离转折点距离大于6米，转折点检测变量清零（为下一次检测做好准备），直行
                       detection=0
                       if straightCount%5==0 or straightCount%5==2 or straightCount%5==4:
                            pilot_angle=0
                            pilot_throttle=0.22
                       else:
                            pilot_angle=0.04
                            pilot_throttle=0.22
                       print('直行222')
                       

                       if(addNum==1):   #转折点切换后，转折点切换变量置零
                           Num=Num+1
                           addNum=0
                   print('pilot_throttle')    
                   print(pilot_throttle)            
                   print('pilot_angle')    
                   print(pilot_angle)   
                   #超声波检测到障碍物距离小于50厘米停车（控制权优先级高于GPS，因此放在最后）
                   if(dis<50):
                    pilot_throttle=0.00
                    pilot_angle=0.00
                   return pilot_angle if pilot_angle else 0.0, pilot_throttle * cfg.AI_THROTTLE_MULT if pilot_throttle else 0.0

                
                if(y1==1):  #报道领材料模式
                   action=['右转', '']
                   turnPos=[[[116.35635], [39.960968]], [[116.356337], [39.961189]]]
                   destination=[[116.356807],[39.961199]]
                   straightCount=straightCount+1
                   #球坐标系计算当前所处位置和下一个转折点之间的距离（单位米）
                   if(Num<2-1):
                     lon1,lat1,lon2,lat2=map(radians,[float(longitudeG),float(latitudeG), turnPos[Num+1][0][0], turnPos[Num+1][1][0]])
                   if(Num==2-1):
                     lon1,lat1,lon2,lat2=map(radians,[float(longitudeG),float(latitudeG), destination[0][0], destination[1][0]])
                   dlon = lon2-lon1
                   dlat = lat2- lat1
                   a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
                   c = 2 * asin(sqrt(a)) 
                   r = 6371 # 地球平均半径，单位为公里
                   distance=c * r * 1000
                               
                   print(distance)
                   print(longitudeG)
                   print(latitudeG)
                   print('num')
                   print(Num)
                   print('count')
                   print(count)
                   if(distance<6):    #距离转折点距离小于6米
                       detection=detection+1
                       if(detection==1):   #在转折点范围内第一次检测到转折点 计数控制初始化
                            count=18
                            addNum=1
                       if(count>13):   #count在14～18期间小车根据action数组中的值转向
                         if(action[Num]=='左转'):
                           pilot_angle=-0.17
                           pilot_throttle=0.2
                           print('左转')
                         if(action[Num]=='右转'):
                           pilot_angle=0.17
                           pilot_throttle=0.2
                           print('右转')
                         count=count-1
                       if count>0 and count<=13 and Num==1:  #即将到达终点播报注意事项
                          pilot_angle=0
                          pilot_throttle=0
                          count=count-1
                          print('暂停播报')
                          if (count==13):
                            music_path_2 = '/home/pi/mycar/music/material.mp3'
                            os.system('mplayer %s' % music_path_2)
                       if count>0 and count<=13 and Num==0:
                          count=count-1
                       if(count==0):   #转向结束后直行
                         if straightCount%5==0 or straightCount%5==2 or straightCount%5==4:
                            pilot_angle=0
                            pilot_throttle=0.2
                         else:
                            pilot_angle=0.01
                            pilot_throttle=0.2
                         print('直行111')
                       if(Num==2-1):  #到达终点停车
                          pilot_angle=0
                          pilot_throttle=0
                          print('停车')
                      
                          
                   if(distance>=6):   #距离转折点距离大于6米，转折点检测变量清零（为下一次检测做好准备），直行
                       detection=0
                       if straightCount%5==0 or straightCount%5==2 or straightCount%5==4:
                            pilot_angle=0
                            pilot_throttle=0.2
                       else:
                            pilot_angle=0.01
                            pilot_throttle=0.2
                       print('直行222')
                       

                       if(addNum==1):   #转折点切换后，转折点切换变量置零
                           Num=Num+1
                           addNum=0
                   print('pilot_throttle')    
                   print(pilot_throttle)            
                   print('pilot_angle')    
                   print(pilot_angle)   
                   #超声波检测到障碍物距离小于50厘米停车（控制权优先级高于GPS，因此放在最后）
                   if(dis<50):
                    pilot_throttle=0.00
                    pilot_angle=0.00
                   return pilot_angle if pilot_angle else 0.0, pilot_throttle * cfg.AI_THROTTLE_MULT if pilot_throttle else 0.0
                
                
                if(y1==3):
                   print("暂未开启该路线，敬请期待")
            
            #微信小程序点按暂停小车
            if(y=='0'):
                pilot_angle=0
                pilot_throttle=0
                print('stopcar')
                return pilot_angle if pilot_angle else 0.0, pilot_throttle * cfg.AI_THROTTLE_MULT if pilot_throttle else 0.0
            
              

    V.add(DriveMode(),
          inputs=['user/mode', 'user/angle', 'user/throttle',
                  'pilot/angle', 'pilot/throttle'],
          outputs=['angle', 'throttle'])


    #to give the car a boost when starting ai mode in a race.
    aiLauncher = AiLaunch(cfg.AI_LAUNCH_DURATION, cfg.AI_LAUNCH_THROTTLE, cfg.AI_LAUNCH_KEEP_ENABLED)

    V.add(aiLauncher,
        inputs=['user/mode', 'throttle'],
        outputs=['throttle'])

    if isinstance(ctr, JoystickController):
        ctr.set_button_down_trigger(cfg.AI_LAUNCH_ENABLE_BUTTON, aiLauncher.enable_ai_launch)


    class AiRunCondition:
        '''
        A bool part to let us know when ai is running.
        '''
        def run(self, mode):
            if mode == "user":
                return False
            return True

    V.add(AiRunCondition(), inputs=['user/mode'], outputs=['ai_running'])

    #Ai Recording
    class AiRecordingCondition:
        '''
        return True when ai mode, otherwize respect user mode recording flag
        '''
        def run(self, mode, recording):
            if mode == 'user':
                return recording
            return True

    if cfg.RECORD_DURING_AI:
        V.add(AiRecordingCondition(), inputs=['user/mode', 'recording'], outputs=['recording'])

    #Drive train setup
    if cfg.DONKEY_GYM or cfg.DRIVE_TRAIN_TYPE == "MOCK":
        pass
    elif cfg.DRIVE_TRAIN_TYPE == "SERVO_ESC":
        from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

        steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        steering = PWMSteering(controller=steering_controller,
                                        left_pulse=cfg.STEERING_LEFT_PWM,
                                        right_pulse=cfg.STEERING_RIGHT_PWM)

        throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
        throttle = PWMThrottle(controller=throttle_controller,
                                        max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                        zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                                        min_pulse=cfg.THROTTLE_REVERSE_PWM)

        V.add(steering, inputs=['angle'], threaded=True)
        V.add(throttle, inputs=['throttle'], threaded=True)


    elif cfg.DRIVE_TRAIN_TYPE == "DC_STEER_THROTTLE":
        from donkeycar.parts.actuator import Mini_HBridge_DC_Motor_PWM

        steering = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_LEFT, cfg.HBRIDGE_PIN_RIGHT)
        throttle = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_FWD, cfg.HBRIDGE_PIN_BWD)

        V.add(steering, inputs=['angle'])
        V.add(throttle, inputs=['throttle'])


    elif cfg.DRIVE_TRAIN_TYPE == "DC_TWO_WHEEL":
        from donkeycar.parts.actuator import TwoWheelSteeringThrottle, Mini_HBridge_DC_Motor_PWM

        left_motor = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_LEFT_FWD, cfg.HBRIDGE_PIN_LEFT_BWD)
        right_motor = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_RIGHT_FWD, cfg.HBRIDGE_PIN_RIGHT_BWD)
        two_wheel_control = TwoWheelSteeringThrottle()

        V.add(two_wheel_control,
                inputs=['throttle', 'angle'],
                outputs=['left_motor_speed', 'right_motor_speed'])

        V.add(left_motor, inputs=['left_motor_speed'])
        V.add(right_motor, inputs=['right_motor_speed'])

    elif cfg.DRIVE_TRAIN_TYPE == "SERVO_HBRIDGE_PWM":
        from donkeycar.parts.actuator import ServoBlaster, PWMSteering
        steering_controller = ServoBlaster(cfg.STEERING_CHANNEL) #really pin
        #PWM pulse values should be in the range of 100 to 200
        assert(cfg.STEERING_LEFT_PWM <= 200)
        assert(cfg.STEERING_RIGHT_PWM <= 200)
        steering = PWMSteering(controller=steering_controller,
                                        left_pulse=cfg.STEERING_LEFT_PWM,
                                        right_pulse=cfg.STEERING_RIGHT_PWM)


        from donkeycar.parts.actuator import Mini_HBridge_DC_Motor_PWM
        motor = Mini_HBridge_DC_Motor_PWM(cfg.HBRIDGE_PIN_FWD, cfg.HBRIDGE_PIN_BWD)

        V.add(steering, inputs=['angle'], threaded=True)
        V.add(motor, inputs=["throttle"])
        
    elif cfg.DRIVE_TRAIN_TYPE == "MM1":
        from donkeycar.parts.robohat import RoboHATDriver
        V.add(RoboHATDriver(cfg), inputs=['angle', 'throttle'])
    
    elif cfg.DRIVE_TRAIN_TYPE == "PIGPIO_PWM":
        from donkeycar.parts.actuator import PWMSteering, PWMThrottle, PiGPIO_PWM
        steering_controller = PiGPIO_PWM(cfg.STEERING_PWM_PIN, freq=cfg.STEERING_PWM_FREQ, inverted=cfg.STEERING_PWM_INVERTED)
        steering = PWMSteering(controller=steering_controller,
                                        left_pulse=cfg.STEERING_LEFT_PWM, 
                                        right_pulse=cfg.STEERING_RIGHT_PWM)
        
        throttle_controller = PiGPIO_PWM(cfg.THROTTLE_PWM_PIN, freq=cfg.THROTTLE_PWM_FREQ, inverted=cfg.THROTTLE_PWM_INVERTED)
        throttle = PWMThrottle(controller=throttle_controller,
                                            max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                            zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                            min_pulse=cfg.THROTTLE_REVERSE_PWM)
        V.add(steering, inputs=['angle'], threaded=True)
        V.add(throttle, inputs=['throttle'], threaded=True)

    # OLED setup
    if cfg.USE_SSD1306_128_32:
        from donkeycar.parts.oled import OLEDPart
        auto_record_on_throttle = cfg.USE_JOYSTICK_AS_DEFAULT and cfg.AUTO_RECORD_ON_THROTTLE
        oled_part = OLEDPart(cfg.SSD1306_128_32_I2C_BUSNUM, auto_record_on_throttle=auto_record_on_throttle)
        V.add(oled_part, inputs=['recording', 'tub/num_records', 'user/mode'], outputs=[], threaded=True)

    #add tub to save data

    inputs=['cam/image_array',
            'user/angle', 'user/throttle',
            'range/cms',#新增传感器数据
            'user/mode']

    types=['image_array',
           'float', 'float',
           'float',#对应新增传感器数据的类型
           'str']
    
   
    

    if cfg.TRAIN_BEHAVIORS:
        inputs += ['behavior/state', 'behavior/label', "behavior/one_hot_state_array"]
        types += ['int', 'str', 'vector']

    if cfg.CAMERA_TYPE == "D435" and cfg.REALSENSE_D435_DEPTH:
        inputs += ['cam/depth_array']
        types += ['gray16_array']

    if cfg.HAVE_IMU or (cfg.CAMERA_TYPE == "D435" and cfg.REALSENSE_D435_IMU):
        inputs += ['imu/acl_x', 'imu/acl_y', 'imu/acl_z',
            'imu/gyr_x', 'imu/gyr_y', 'imu/gyr_z']

        types +=['float', 'float', 'float',
           'float', 'float', 'float']

    if cfg.RECORD_DURING_AI:
        inputs += ['pilot/angle', 'pilot/throttle']
        types += ['float', 'float']

    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types, user_meta=meta)
    V.add(tub, inputs=inputs, outputs=["tub/num_records"], run_condition='recording')

    if cfg.PUB_CAMERA_IMAGES:
        from donkeycar.parts.network import TCPServeValue
        from donkeycar.parts.image import ImgArrToJpg
        pub = TCPServeValue("camera")
        V.add(ImgArrToJpg(), inputs=['cam/image_array'], outputs=['jpg/bin'])
        V.add(pub, inputs=['jpg/bin'])

    if type(ctr) is LocalWebController:
        if cfg.DONKEY_GYM:
            print("You can now go to http://localhost:%d to drive your car." % cfg.WEB_CONTROL_PORT)
        else:
            print("You can now go to <your hostname.local>:%d to drive your car." % cfg.WEB_CONTROL_PORT)
    elif isinstance(ctr, JoystickController):
        print("You can now move your joystick to drive your car.")
        #tell the controller about the tub
        ctr.set_tub(tub)

        if cfg.BUTTON_PRESS_NEW_TUB:

            def new_tub_dir():
                V.parts.pop()
                tub = th.new_tub_writer(inputs=inputs, types=types, user_meta=meta)
                V.add(tub, inputs=inputs, outputs=["tub/num_records"], run_condition='recording')
                ctr.set_tub(tub)

            ctr.set_button_down_trigger('cross', new_tub_dir)
        ctr.print_controls()

    #run the vehicle for 20 seconds
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ,
            max_loop_count=cfg.MAX_LOOPS)
########################################################################3

if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config(myconfig=args['--myconfig'])

    if args['drive']:
        model_type = args['--type']
        camera_type = args['--camera']

        drive(cfg, model_path=args['--model'], use_joystick=args['--js'],
              model_type=model_type, camera_type=camera_type,
              meta=args['--meta'])

    if args['train']:
        from train import multi_train, preprocessFileList

        tub = args['--tub']
        model = args['--model']
        transfer = args['--transfer']
        model_type = args['--type']
        continuous = args['--continuous']
        aug = args['--aug']
        dirs = preprocessFileList( args['--file'] )

        if tub is not None:
            tub_paths = [os.path.expanduser(n) for n in tub.split(',')]
            dirs.extend( tub_paths )

        if model_type is None:
            model_type = cfg.DEFAULT_MODEL_TYPE
            print("using default model type of", model_type)

        multi_train(cfg, dirs, model, transfer, model_type, continuous, aug)

