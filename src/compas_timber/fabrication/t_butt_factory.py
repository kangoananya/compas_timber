import math
from collections import OrderedDict
from compas.geometry import Frame
from compas.geometry import Line
from compas.geometry import Plane
from compas.geometry import cross_vectors
from compas.geometry import angle_vectors_signed
from compas_timber.parts.beam import Beam
from compas_timber.connections.joint import Joint
from compas_timber.utils.compas_extra import intersection_line_plane
from compas_timber.connections import TButtJoint
from compas_timber.connections import LButtJoint
from compas_timber.connections import LMiterJoint
from compas_timber.fabrication import BTLx
from compas_timber.fabrication import BTLxJoint
from compas_timber.fabrication import BTLxProcess
from compas_timber.fabrication import BTLxJackCut


class TButtFactory(object):
    def __init__(self):
        pass

    @classmethod
    def apply_processes(cls, btlx_joint):
        part = btlx_joint.parts[str(btlx_joint.joint.main_beam.key)]
        cut_plane = btlx_joint.joint.cutting_plane
        BTLxJackCut.apply_processes(cut_plane, part, btlx_joint)


BTLxJoint.register_joint(TButtJoint, TButtFactory)
