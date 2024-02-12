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

    def __init__(self, part, joint_name=None, process_params=None):
        self.part = part
        self.apply_process = True
        self.reference_surface = self.part.reference_surface_planes(2)
        self.generate_process(process_params)
        if joint_name:
            self.name = joint_name
        else:
            self.name = "Lap Joint"

    @property
    def header_attributes(self):
        """the following attributes are required for all processes, but the keys and values of header_attributes are process specific."""
        return {
            "Name": self.name,
            "Process": "yes",
            "Priority": "0",
            "ProcessID": "0",
            "ReferencePlaneID": self.reference_id,
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

    def generate_process(self, process_params=None):
        """This is an internal method to generate process parameters"""
        self.orientation = process_params["Orientation"]
        self.startX = float(process_params["startX"])
        self.startY = float(process_params["startY"])
        self.angle = float(process_params["Angle"])
        self.inclination = float(process_params["Inclination"])
        self.slope = float(process_params["Slope"])
        self.length = float(process_params["Length"])
        self.width = float(process_params["Width"])
        self.depth = float(process_params["Depth"])
        self.lead_angle_parallel = "yes"
        self.lead_angle = 90.0
        self.lead_inclination_parallel = "yes"
        self.lead_inclination = 90.0

        self.reference_id = process_params["ReferencePlaneID"]

    @classmethod
    def create_process(cls, part, joint, process_params=None):
        lap_process = BTLxLap(part, joint, process_params=process_params)
        return BTLxProcess(BTLxLap.PROCESS_TYPE, lap_process.header_attributes, lap_process.process_params)
