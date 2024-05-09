from rclpy.time import Time
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu
from geodesy.utm import fromLatLong
from ros2bag_tools.exporter import Exporter, ExporterError
import json
from pathlib import Path

class IMUExporter(Exporter):

    def __init__(self):
        self._args = None
        self._f = None

    @staticmethod
    def add_arguments(parser):
        parser.add_argument('--dir', '-d', required=False,
                    type=str, help='Path to output file')
        parser.add_argument('--name', default='%t.txt',
                    help="""Filename pattern of output pcd files.
                    Placeholders:
                        %%tpc ... topic
                        %%t   ... timestamp
                        %%i   ... sequence index""")


    def open(self, args):
        self._dir = Path(args.dir)
        self._dir.mkdir(parents=True, exist_ok=True)
        self._name = args.name
        self._i = 0

    def write(self, topic, msg, t):
        if isinstance(msg, Imu):
            tpc_path = topic.lstrip('/').replace('/', '_')
            filename = self._name.replace('%tpc', tpc_path)
            filename = filename.replace('%t', str(t))
            filename = filename.replace('%i', str(self._i).zfill(8))
            imu_path = self._dir / filename
            # Header header
            # 
            # geometry_msgs/Quaternion orientation
            # float64[9] orientation_covariance # Row major about x, y, z axes
            # 
            # geometry_msgs/Vector3 angular_velocity
            # float64[9] angular_velocity_covariance # Row major about x, y, z axes
            # 
            # geometry_msgs/Vector3 linear_acceleration
            # float64[9] linear_acceleration_covariance # Row major x, y z 
            message = dict()
            message["orientation_covariance"] = msg.orientation
            message["angular_velocity"] = msg.angular_velocity
            message["linear_acceleration"] = msg.linear_acceleration
            
            json_object = json.dumps(message, indent=4)
            with open(str(imu_path), "w") as f:
                f.write(json_object)

    def close(self):
        self._f.close()
