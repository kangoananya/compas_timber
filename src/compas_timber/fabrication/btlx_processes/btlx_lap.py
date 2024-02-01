import math
from collections import OrderedDict

from compas.geometry import Line
from compas.geometry import Plane
from compas.geometry import angle_vectors_signed
from compas.geometry import cross_vectors

from compas_timber.fabrication import BTLx
from compas_timber.fabrication import BTLxProcess
from compas_timber.utils.compas_extra import intersection_line_plane


class BTLxLap(object):
    """
    Represents a lap process for timber fabrication.

    Parameters
    ----------
    part : :class:`~compas_timber.fabrication.btlx_part.BTLxPart`
        The BTLxPart object representing the beam.
    joint_name : str, optional
        The name of the joint. Defaults to None.

    """

    PROCESS_TYPE = "Lap"

    def __init__(self, part, joint_name=None):
        self.part = part
        self.apply_process = True
        self.reference_surface = self.part.reference_surface_planes(1)
        self.generate_process()
        if joint_name:
            self.name = joint_name
        else:
            self.name = "jack cut"

    @property
    def header_attributes(self):
        """the following attributes are required for all processes, but the keys and values of header_attributes are process specific."""
        return {
            "Name": self.name,
            "Process": "yes",
            "Priority": "0",
            "ProcessID": "0",
            "ReferencePlaneID": "1",
        }

    @property
    def process_params(self):
        """This property is required for all process types. It returns a dict with the geometric parameters to fabricate the joint."""

        if self.apply_process:
            """the following attributes are specific to Jack Cut"""
            od = OrderedDict(
                [
                    ("Orientation", str(self.orientation)),
                    ("StartX", "{:.{prec}f}".format(self.startX, prec=BTLx.POINT_PRECISION)),
                    ("StartY", "{:.{prec}f}".format(self.startY, prec=BTLx.POINT_PRECISION)),
                    ("Angle", "{:.{prec}f}".format(self.angle, prec=BTLx.ANGLE_PRECISION)),
                    ("Inclination", "{:.{prec}f}".format(self.inclination, prec=BTLx.ANGLE_PRECISION)),
                    ("Slope", "{:.{prec}f}".format(self.slope, prec=BTLx.ANGLE_PRECISION)),
                    ("Length", "{:.{prec}f}".format(self.length, prec=BTLx.POINT_PRECISION)),
                    ("Width", "{:.{prec}f}".format(self.width, prec=BTLx.POINT_PRECISION)),
                    ("Depth", "{:.{prec}f}".format(self.depth, prec=BTLx.POINT_PRECISION)),
                    ("LeadAngleParallel", str(self.lead_angle_parallel)),
                    ("LeadAngle", "{:.{prec}f}".format(self.lead_angle, prec=BTLx.ANGLE_PRECISION)),
                    ("LeadInclinationParallel", str(self.lead_inclination_parallel)),
                    (
                        "LeadInclination",
                        "{:.{prec}f}".format(self.lead_inclination, prec=BTLx.ANGLE_PRECISION),
                    ),
                ]
            )
            return od
        else:
            return None

    def generate_process(self):
        """This is an internal method to generate process parameters"""
        self.orientation = "start"
        self.startX = 0.0
        self.startY = 0.0
        self.angle = 90.0
        self.inclination = 90.0
        self.slope = 0.0
        self.length = 200.0
        self.width = 50.0
        self.depth = 40.0
        self.lead_angle_parallel = "yes"
        self.lead_angle = 90.0
        self.lead_inclination_parallel = "yes"
        self.lead_inclination = 90.0

    @classmethod
    def create_process(cls, part, frame, joint_name=None):
        lap_process = BTLxLap(part, frame, joint_name)
        return BTLxProcess(BTLxLap.PROCESS_TYPE, lap_process.header_attributes, lap_process.process_params)
