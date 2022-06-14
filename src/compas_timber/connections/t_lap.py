from compas.geometry import intersection_line_line, intersection_line_plane, distance_point_point, angle_vectors
from compas.geometry import Vector, Point, Plane
from ..connections.joint import Joint


class TLapJoint(Joint):
    def __init__(self, assembly, beamA, beamB):

        super(TLapJoint, self).__init__(assembly, [beamA, beamB])
        self.assembly = assembly
        #self.frame = None  # will be needed as coordinate system for structural calculations for the forces at the joint

    @property
    def joint_type(self):
        return 'T-Lap'

    def add_feature(self):
        pass