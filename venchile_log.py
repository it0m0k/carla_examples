#!/usr/bin/env python3
# -*- coding: utf-8  -*-

# $ python -m pip install websocket-client
import websocket

import glob
import os
import sys
import time
import datetime
import math
import json
import copy

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import textwrap
import weakref

webSocket = None

def finalizeWebSocket():
    global webSocket

    if None != webSocket:
        webSocket.close()
    exit()


def list_options(client):
    world = client.get_world()
    vehicles = world.get_actors().filter('vehicle.*')
    if len(vehicles) == 0:
        print('no vehicle')
    else:
        print('available vehicles:\n')
        for v in vehicles:
            print('%d : %s' % (v.id, v.type_id))


class Player(dict):
    def __init__(self, vehicle):
        self.__vehicle = vehicle
        self.__gnss_sensor = GnssSensor(vehicle)
        self.__imu_sensor = IMUSensor(vehicle)
        self.__first_time = True
        self.__current_value = {
            'accPedalPosition': 0,
            'bodyInformation': {
                'dataType': 'VEHICLEDATA_BODYINFO',  # FIXED
                'parkBrakeActive': False,
                "driverDoorAjar": True,  # FIXED
                "ignitionStableStatus": "MISSING_FROM_TRANSMITTER",  # FIXED
                "ignitionStatus": "UNKNOWN",  # FIXED
                "passengerDoorAjar": True,  # FIXED
                "rearLeftDoorAjar": True,  # FIXED
                "rearRightDoorAjar": True  # FIXED
            },
            'driverBraking': 'NO',
            'gps': {
                'compassDirection': '',
                'heading': 0,
                'latitudeDegrees': 0,
                'longitudeDegrees': 0,
                'speed': 0,
                'utcDay': 0,
                'utcHours': 0,
                'utcMinutes': 0,
                'utcMonth': 0,
                'utcSeconds': 0,
                'utcYear': 0,
            },
            'headLampStatus': {
                'ambientLightSensorStatus': 'UNKNOWN',  # FIXED
                'dataType': 'VEHICLEDATA_HEADLAMPSTATUS',  # FIXED
                'highBeamsOn': False,
                'lowBeamsOn': True
            },
            'prndl': '',
            'speed': 0,
            'steeringWheelAngle': 0,  # -2000 - +2000
            'turnSignal': ''
            # 'rpm' : 0, # n/a
        }
        self.__prev_value = copy.deepcopy(self.__current_value)

    def tick(self):
        transform = self.__vehicle.get_transform()
        velocity = self.__vehicle.get_velocity()
        control = self.__vehicle.get_control()
        light = self.__vehicle.get_light_state()
        utcnow = datetime.datetime.utcnow()

        curr = self.__current_value

        curr['accPedalPosition'] = round(control.throttle, 2)
        curr['bodyInformation']['parkBrakeActive'] = control.hand_brake
        curr['driverBraking'] = 'YES' if control.brake else 'NO'
        curr['speed'] = round(3.6 * math.sqrt(velocity.x**2 +
                                              velocity.y**2 + velocity.z**2), 2)

        compass = self.__imu_sensor.compass
        curr['gps']['compassDirection'] = 'NORTH' if abs(
            compass) < 67.5 else ''
        curr['gps']['compassDirection'] += 'SOUTH' if abs(
            compass) > 112.5 else ''
        curr['gps']['compassDirection'] += 'EAST' if 157.5 > compass > 22.5 else ''
        curr['gps']['compassDirection'] += 'WEST' if 22.5 > compass > -157.5 else ''
        curr['gps']['heading'] = round(
            compass, 2) if compass > 0 else round(360 + compass, 2)
        curr['gps']['latitudeDegrees'] = self.__gnss_sensor.lat
        curr['gps']['longitudeDegrees'] = self.__gnss_sensor.lon
        curr['gps']['speed'] = curr['speed']
        curr['gps']['utcYear'] = utcnow.year
        curr['gps']['utcMonth'] = utcnow.month
        curr['gps']['utcDay'] = utcnow.day
        curr['gps']['utcHours'] = utcnow.hour
        curr['gps']['utcMinutes'] = utcnow.minute
        curr['gps']['utcSeconds'] = utcnow.second

        curr['headLampStatus']['highBeamsOn'] = light & carla.VehicleLightState.HighBeam == carla.VehicleLightState.HighBeam
        curr['headLampStatus']['lowBeamsOn'] = light & carla.VehicleLightState.LowBeam == carla.VehicleLightState.LowBeam
        if control.reverse:
            curr['prndl'] = 'REVERSE'
        elif control.gear == 0:
            curr['prndl'] = 'NEUTRAL'
        elif control.gear == 1:
            curr['prndl'] = 'FIRST'
        elif control.gear == 2:
            curr['prndl'] = 'SECOND'
        elif control.gear == 3:
            curr['prndl'] = 'THIRD'
        elif control.gear == 4:
            curr['prndl'] = 'FOURTH'
        elif control.gear == 5:
            curr['prndl'] = 'FIFTH'
        elif control.gear == 6:
            curr['prndl'] = 'SIXTH'
        elif control.gear == 7:
            curr['prndl'] = 'SEVENTH'
        elif control.gear == 8:
            curr['prndl'] = 'EIGHTH'
        else:
            curr['prndl'] = 'FAULT'

        curr['steeringWheelAngle'] = round(
            control.steer * 360, 2)    # TODO: Adjustment degrees
        if light & (carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker) == (carla.VehicleLightState.RightBlinker | carla.VehicleLightState.LeftBlinker):
            curr['turnSignal'] = 'BOTH'
        elif light & carla.VehicleLightState.RightBlinker:
            curr['turnSignal'] = 'RIGHT'
        elif light & carla.VehicleLightState.LeftBlinker:
            curr['turnSignal'] = 'LEFT'
        else:
            curr['turnSignal'] = 'OFF'

        return self.__diff()

    def __diff(self):
        if self.__first_time:
            self.__first_time = False

        vehicle_data = {}
        for k, v in self.__current_value.items():
            if self.__prev_value[k] != v:
                vehicle_data[k] = v

        self.__prev_value = copy.deepcopy(self.__current_value)
        return vehicle_data

    def destroy(self):
        actors = [
            self.__gnss_sensor.sensor,
            self.__imu_sensor.sensor,
        ]
        for actor in actors:
            if actor is not None:
                actor.destroy()


class GnssSensor(object):
    """ Class for GNSS sensors"""

    def __init__(self, parent_actor):
        """Constructor method"""
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(blueprint, carla.Transform(carla.Location(x=1.0, z=2.8)),
                                        attach_to=self._parent)
        # We need to pass the lambda a weak reference to
        # self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        """GNSS method"""
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(
                sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(
                sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


def yes_no_input(prompt="Please respond with 'yes' or 'no' [y/N]:", default=False):
    while True:
        choice = input(prompt).lower()
        if choice == '':
            return default
        if choice in ['y', 'ye', 'yes']:
            return True
        elif choice in ['n', 'no']:
            return False


def main():
    global webSocket
    outfile = None

    #実行時引数の設定
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the CARLA host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to the CARLA (default: 2000)')
    argparser.add_argument(
        '--fps',
        metavar='N',
        type=float,
        default=1.0,
        help='set fixed FPS, zero for variable FPS (default: 1.0)')
    argparser.add_argument(
        '--vid',
        metavar='VID',
        type=int,
        help='vehicle id')
    argparser.add_argument(
        '-l', '--list',
        action='store_true',
        help='list vehicle id')
    argparser.add_argument(
        '--sdl',
        metavar='URL',
        help='sdl-core-broker (ex: wss://xxxxxx.m.sdl.tools:444)')
    argparser.add_argument(
        '-t', '--path-through',
        action='store_true',
        help='connect to sdl by path through mode')
    argparser.add_argument(
        '-o', '--out',
        action='store_true',
        help='output log file')
    argparser.add_argument(
        '--outfile',
        help='(default: carla_log_YYYYMMDDHHMMSS.json)',
        default='carlalog_' + datetime.datetime.now().strftime('%Y%m%d%H%M%S') + '.json')
    argparser.add_argument(
        '-f', '--force',
        action='store_true',
        help='force overwrite when outfile is exists')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true')

    args = argparser.parse_args()

    # print(args)
    #fpsが引数の時の処理
    if args.fps is not None:
        update_freq = 1.0 / args.fps if args.fps > 0.0 else 0.0

    #vidの引数の時の処理（車両の情報がリストにない時用）
    if args.vid is None and args.list is False:
        print('argument --vid or --list is required')
        sys.exit()

    #.get_worldでシミュレーション上でアクティブなオブジェクトを取得
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    print('connecting carla')
    try:
        world = client.get_world()
    except:
        print('failed to connect carla')
        sys.exit()
    print('connected')

    #-lが引数の時の処理
    if args.list:
        list_options(client)
        sys.exit()
    
    #-outが引数の時の処理
    if args.out is True:
        if os.path.isfile(args.outfile):
            if yes_no_input('\'' + args.outfile + '\' is exists. overrite ? [y/N]:') is False:
                sys.exit()
        outfile = open(args.outfile, 'w')

    elapsed_time = 0.0

    #シミュレーション上に存在する引数に指定した車両の情報を取得
    #なかった時の例外処理
    #-v,-verboseが指定されたときは情報を表示
    #指定したtype_idが車両じゃなかったときの処理
    vehicle = world.get_actors().find(args.vid)
    if vehicle is None:
        print('vehicle not found')
        sys.exit()
    if args.verbose:
        print(vehicle)
    if vehicle.type_id[0:8] != 'vehicle.':
        print('actor is not a vehicle')
        sys.exit()

    #-sdlが引数の時の処理
    #カーナビとかとスマホをつなぐための処理
    if args.sdl is not None:
        try:
            # WebSocket connect
            webSocket = websocket.WebSocket()
            print('connecting sdl')
            webSocket.connect(args.sdl)
            print('connected sdl')

            if False == webSocket.connected:
                print("[ERR] WebSocket Connection Error")
                finalizeWebSocket()
            print('connected')
        except:
            print('failed to connect carla')
            finalizeWebSocket()

    player = Player(vehicle)

    start_seconds = world.wait_for_tick(seconds=30.0).timestamp.elapsed_seconds
    try:
        while True:
            timestamp = world.wait_for_tick(seconds=30.0).timestamp
            elapsed_time += timestamp.delta_seconds
            if elapsed_time > update_freq:
                vehicle_data = player.tick()
                if len(vehicle_data) > 0:
                    if args.out and outfile is not None:
                        if outfile.tell() == 0:
                            outfile.write('[\n')
                        else:
                            outfile.write(',\n')
                    elapsed_seconds = round(
                        timestamp.elapsed_seconds - start_seconds, 6)
                    sdl_data = {
                        'jsonrpc': '2.0',
                        'method': 'VehicleInfo.OnVehicleData',
                        'params': vehicle_data,
                        'elapsedSeconds': elapsed_seconds,
                        'elapsedSecondsStr': str(datetime.timedelta(seconds=elapsed_seconds))
                    }
                    if args.verbose:
                        print(json.dumps(sdl_data, indent=2))
                    if args.out:
                        outfile.write(json.dumps(sdl_data))
                        outfile.flush()
                        os.fsync(outfile.fileno())
                    if args.sdl:
                        if not args.path_through:
                            sdl_data = {'notRpc': 'wrap', 'data': sdl_data}
                        webSocket.send(json.dumps(sdl_data))
                elapsed_time = 0.0
    except KeyboardInterrupt:
        pass
    finally:
        if outfile is not None:
            if outfile.tell() > 0:
                outfile.write('\n]')
            outfile.close()
        player.destroy()
        finalizeWebSocket()


if __name__ == '__main__':

    main()