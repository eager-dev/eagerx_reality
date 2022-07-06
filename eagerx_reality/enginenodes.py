from typing import Optional, List, Any
import numpy as np
from gym.spaces import Box, Discrete
import cv2
import skimage.transform

# IMPORT EAGERX
import eagerx
import eagerx.core.register as register
from eagerx.utils.utils import Msg
from eagerx.core.entities import EngineNode
from eagerx.core.specs import NodeSpec, ObjectSpec


class CameraRender(EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: int = eagerx.NEW_PROCESS,
        color: str = "cyan",
        shape: Optional[List[int]] = None,
        camera_idx: int = 0,
    ) -> NodeSpec:
        """CameraRender spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=process, color=color)
        spec.config.update(inputs=["tick"], outputs=["image"])

        # Modify custom node params
        spec.config.shape = shape if isinstance(shape, list) else [480, 480]
        spec.config.camera_idx = camera_idx

        # Set image space
        spec.outputs.image.space = Box(low=0, high=255, shape=(spec.config.shape[0], spec.config.shape[1], 3), dtype="uint8")
        return spec

    def initialize(self, spec: NodeSpec, object_spec: ObjectSpec, simulator: Any):
        self.cam = None
        self.height, self.width = spec.config.shape
        self.camera_idx = spec.config.camera_idx
        self.render_toggle = False
        self.sub_toggle = self.backend.Subscriber("%s/env/render/toggle" % self.ns, "bool", self._set_render_toggle)

    @register.states()
    def reset(self):
        # This sensor is stateless (in contrast to e.g. a Kalman filter).
        pass

    @register.inputs(tick=Discrete(99999))  # Dummy, because a node must have at least one input.
    @register.outputs(image=None)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        if self.render_toggle:
            self.cam: cv2.VideoCapture
            ret, cv_img = self.cam.read()
            if ret:
                kwargs = dict(
                    output_shape=(self.height, self.width),
                    mode="edge",
                    order=1,
                    preserve_range=True,
                )
                img = skimage.transform.resize(cv_img, **kwargs).astype("uint8")
            else:
                img = np.zeros((self.height, self.width, 3), dtype="uint8")
        else:
            img = np.zeros((self.height, self.width, 3), dtype="uint8")
        return dict(image=img)

    def _set_render_toggle(self, msg):
        if msg:
            self.backend.logdebug("[%s] START RENDERING!" % self.name)
            self.cam = cv2.VideoCapture(self.camera_idx)
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        else:
            self.backend.logdebug("[%s] STOPPED RENDERING!" % self.name)
            if self.cam is not None:
                self.cam.release()
                self.cam = None
        self.render_toggle = msg

    def shutdown(self):
        self.backend.logdebug(f"[{self.name}] {self.name}.shutdown() called.")
        self.sub_toggle.unregister()

        # Release camera resources
        if self.cam is not None:
            self.cam.release()
            self.cam = None
