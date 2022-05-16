from typing import Optional, List
import cv2
import skimage.transform
import rospy

# IMPORT ROS
from std_msgs.msg import UInt64, Bool
from sensor_msgs.msg import Image
import cv_bridge

# IMPORT EAGERX
import eagerx.core.register as register
from eagerx.utils.utils import Msg
from eagerx.core.entities import EngineNode
from eagerx.core.constants import process


class CameraRender(EngineNode):
    @staticmethod
    @register.spec("CameraRender", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        process: Optional[int] = process.NEW_PROCESS,
        color: Optional[str] = "cyan",
        shape: Optional[List[int]] = None,
        camera_idx: int = 0,
    ):
        """CameraRender spec"""
        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick"]
        spec.config.outputs = ["image"]

        # Modify custom node params
        spec.config.shape = shape if shape else [480, 480]
        spec.config.camera_idx = camera_idx

    def initialize(self, shape, camera_idx):
        self.cv_bridge = cv_bridge.CvBridge()
        self.cam = None
        self.width, self.height = shape
        self.camera_idx = camera_idx
        self.render_toggle = False
        self.render_toggle_pub = rospy.Subscriber("%s/env/render/toggle" % self.ns, Bool, self._set_render_toggle)

    @register.states()
    def reset(self):
        # This sensor is stateless (in contrast to e.g. a Kalman filter).
        pass

    @register.inputs(tick=UInt64)
    @register.outputs(image=Image)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        if self.render_toggle:
            ret, img = self.cam.read()
            try:
                msg = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
            except ImportError as e:
                rospy.logwarn_once("[%s] %s. Using numpy instead." % (self.ns_name, e))
                kwargs = dict(
                    output_shape=(self.height, self.width),
                    mode="edge",
                    order=1,
                    preserve_range=True,
                )
                img = skimage.transform.resize(img, **kwargs).astype(img.dtype)
                data = img.tobytes("C")
                msg = Image(data=data, height=self.height, width=self.width, encoding="bgr8")
        else:
            msg = Image()
        return dict(image=msg)

    def _set_render_toggle(self, msg):
        if msg.data:
            rospy.loginfo("[%s] START RENDERING!" % self.name)
            self.cam = cv2.VideoCapture(self.camera_idx)
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        else:
            rospy.loginfo("[%s] STOPPED RENDERING!" % self.name)
            if self.cam is not None:
                self.cam.release()
                self.cam = None
        self.render_toggle = msg.data
