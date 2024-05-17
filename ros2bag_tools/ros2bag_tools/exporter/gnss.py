from rclpy.time import Time
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import NavSatFix
from geodesy.utm import fromLatLong
from ros2bag_tools.exporter import Exporter, ExporterError
import json
from pathlib import Path

class GNSSExporter(Exporter):

    def __init__(self):
        self._args = None

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
        if isinstance(msg, NavSatFix):
            tpc_path = topic.lstrip('/').replace('/', '_')
            filename = self._name.replace('%tpc', tpc_path)
            filename = filename.replace('%t', str(t))
            filename = filename.replace('%i', str(self._i).zfill(8))
            GNSS_path = self._dir / filename

            message = dict()
            message["latitude"] = msg.latitude
            message["longitude"] = msg.longitude
            message["altitude"] = msg.altitude
            message["status_service"] = msg.status.service
            message["status"] = msg.status.status
            message["position_covariance"] = [x for x in msg.position_covariance]
            message["position_covariance_type"] = msg.position_covariance_type
            
            
            json_object = json.dumps(message, indent=4)
            with open(str(GNSS_path), "w+") as f:
                f.write(json_object)

