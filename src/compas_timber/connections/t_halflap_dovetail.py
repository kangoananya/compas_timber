from compas.geometry import Frame

from compas_timber.parts import CutFeature

from .joint import Joint
from .joint import beam_side_incidence
from .solver import JointTopology

import math

from compas.geometry import Frame
from compas.geometry import Plane
from compas.geometry import Point, Vector
from compas.geometry import Polyhedron
from compas.geometry import Translation
from compas.geometry import angle_vectors
from compas.geometry import intersection_line_plane
from compas.geometry import intersection_plane_plane
from compas.geometry import midpoint_point_point
from compas_timber.parts import MillVolume
from compas_timber.utils import intersection_line_line_3D


class THalfLapJoint(Joint):
    """Represents a T-Butt type joint which joins the end of a beam along the length of another beam,
    trimming the main beam.

    This joint type is compatible with beams in T topology.

    Please use `TButtJoint.create()` to properly create an instance of this class and associate it with an assembly.

    Parameters
    ----------
    assembly : :class:`~compas_timber.assembly.TimberAssembly`
        The assembly associated with the beams to be joined.
    main_beam : :class:`~compas_timber.parts.Beam`
        The main beam to be joined.
    cross_beam : :class:`~compas_timber.parts.Beam`
        The cross beam to be joined.

    Attributes
    ----------
    beams : list(:class:`~compas_timber.parts.Beam`)
        The beams joined by this joint.
    cutting_plane_main : :class:`~compas.geometry.Frame`
        The frame by which the main beam is trimmed.
    cutting_plane_cross : :class:`~compas.geometry.Frame`
        The frame by which the cross beam is trimmed.
    joint_type : str
        A string representation of this joint's type.

    """

    SUPPORTED_TOPOLOGY = JointTopology.TOPO_T

    def __init__(self, main_beam=None, cross_beam=None, frame=None, key=None):
        super(THalfLapJoint, self).__init__(frame, key)
        self.main_beam_key = main_beam.key if main_beam else None
        self.cross_beam_key = cross_beam.key if cross_beam else None
        self.main_beam = main_beam
        self.cross_beam = cross_beam
        self.features = []

    @property
    def data(self):
        data_dict = {
            "main_beam_key": self.main_beam_key,
            "cross_beam_key": self.cross_beam_key,
        }
        data_dict.update(Joint.data.fget(self))
        return data_dict

    @classmethod
    def from_data(cls, value):
        instance = cls(frame=Frame.from_data(value["frame"]), key=value["key"], gap=value["gap"])
        instance.main_beam_key = value["main_beam_key"]
        instance.cross_beam_key = value["cross_beam_key"]
        return instance

    @property
    def beams(self):
        return [self.main_beam, self.cross_beam]

    @property
    def joint_type(self):
        return "T-HalfLap"

    def restore_beams_from_keys(self, assembly):
        """After de-serialization, resotres references to the main and cross beams saved in the assembly."""
        self.main_beam = assembly.find_by_key(self.main_beam_key)
        self.cross_beam = assembly.find_by_key(self.cross_beam_key)

    @property
    def cutting_plane(self):
        angles_faces = beam_side_incidence(self.main_beam, self.cross_beam)
        cfr = min(angles_faces, key=lambda x: x[0])[1]
        cfr = Frame(cfr.point, cfr.yaxis, cfr.xaxis)  # flip normal towards the inside of main beam
        return angles_faces

    def get_end_face(self):
        angles_faces_main = beam_side_incidence(self.main_beam, self.cross_beam)
        efr = max(angles_faces_main, key=lambda x: x[0])[1]
        return efr

    def intersection_polygon(self):
        angles_faces_main = beam_side_incidence(self.main_beam, self.cross_beam)
        sorted_frames_main = [f[1] for f in sorted(angles_faces_main, key=lambda x: x[0])]
        # frames on cross beam front and back
        cfr = sorted_frames_main[0]
        efr = sorted_frames_main[-1]

        angles_faces_cross = beam_side_incidence(self.cross_beam, self.main_beam)
        sorted_frames_cross = [f[1] for f in sorted(angles_faces_cross, key=lambda x: x[0])]
        # frames on main beam sides
        mfr0 = sorted_frames_cross[0]
        mfr1 = sorted_frames_cross[-1]

        i0 = intersection_plane_plane(Plane.from_frame(cfr), Plane.from_frame(mfr0))
        i1 = intersection_plane_plane(Plane.from_frame(cfr), Plane.from_frame(mfr1))
        i2 = intersection_plane_plane(Plane.from_frame(efr), Plane.from_frame(mfr0))
        i3 = intersection_plane_plane(Plane.from_frame(efr), Plane.from_frame(mfr1))

        intersection_lines = [i0, i1, i2, i3]

        top = self.cross_beam.frame.translated(self.cross_beam.frame.zaxis * (self.cross_beam.height / 2.0))
        tpl = Plane.from_frame(top)

        pts_top = [Point.from_data(intersection_line_plane(i, tpl)) for i in intersection_lines]

        T = Translation.from_vector(-self.cross_beam.frame.zaxis * (self.cross_beam.height / 2.0))

        pts = []

        for p in pts_top:
            pts.append(Point.from_data(p).transformed(T))

        pts.extend(pts_top)
        pts_a = pts
        pts_b = [pt.transformed(T) for pt in pts]

        pts_a, reordered = self.reorder_pts(pts_a)
        pts_b, reordered = self.reorder_pts(pts_b)

        if not reordered:
            pts_0 = pts[0].copy() #if not reordered else pts[1].copy()
            pts_4 = pts[4].copy() #if not reordered else pts[5].copy()

            pts_a[0] += Vector.from_start_end(pts_b[0], pts_b[1]).unitized() * 0.01
            pts_a[4] += Vector.from_start_end(pts_b[4], pts_b[5]).unitized() * 0.01
        else:
            pts_0 = pts[0].copy() #if not reordered else pts[1].copy()
            pts_4 = pts[4].copy() #if not reordered else pts[5].copy()

            pts_a[1] -= Vector.from_start_end(pts_b[0], pts_b[1]).unitized() * 0.01
            pts_a[5] -= Vector.from_start_end(pts_b[4], pts_b[5]).unitized() * 0.01

        pts_c = (
            [pts_0.copy(), pts_a[0], pts_a[2], pts_4.copy(), pts_a[4], pts_a[6]]
            if not reordered
            else [pts_0.copy(), pts_a[1], pts_a[3], pts_4.copy(), pts_a[5], pts_a[7]]
        )

        return pts_a, pts_b, [pts_c, reordered]

    def reorder_pts(self, pts):
        angle_between = angle_vectors(
            Vector.from_start_end(self.main_beam.centerline.start, self.main_beam.centerline.end),
            Vector.from_start_end(self.cross_beam.centerline.start, self.cross_beam.centerline.end),
        )
        print(angle_between)
        reordered = False
        if angle_between < math.pi / 2:
            print("sort")
            reordered = True
            pts_sorted = [
                pts[1].copy(),
                pts[0].copy(),
                pts[3].copy(),
                pts[2].copy(),
                pts[5].copy(),
                pts[4].copy(),
                pts[7].copy(),
                pts[6].copy(),
            ]
        else:
            pts_sorted = pts
        return pts_sorted, reordered

    def create_negative_volumes(self, polyhedron_pts_a, polyhedron_pts_b, polyhedron_pts_c):
        negative_beam_a = Polyhedron(
            polyhedron_pts_a,
            [
                [3, 2, 0, 1],  # top
                [6, 7, 5, 4],  # bottom
                [0, 2, 6, 4],  # left
                [0, 4, 5, 1],  # back
                [5, 7, 3, 1],  # right
                [2, 3, 7, 6],
            ],
        )

        negative_beam_b = Polyhedron(
            polyhedron_pts_b,
            [
                [3, 2, 0, 1],  # top
                [6, 7, 5, 4],  # bottom
                [0, 2, 6, 4],  # left
                [0, 4, 5, 1],  # back
                [5, 7, 3, 1],  # right
                [2, 3, 7, 6],
            ],
        )

        prism_pts, reordered = polyhedron_pts_c[0], polyhedron_pts_c[1]
        negative_beam_a_dovetail = Polyhedron(
            prism_pts,
            [
                [1, 2, 0] if not reordered else [0, 2, 1],
                [3, 5, 4] if not reordered else [4, 5, 3] ,
                [3, 4, 1, 0] if not reordered else [0, 1, 4, 3],
                [4, 5, 2, 1] if not reordered else [1, 2, 5, 4],
                [0, 2, 5, 3] if not reordered else [3, 5, 2, 0],
            ],
        )

        return negative_beam_a, negative_beam_b, negative_beam_a_dovetail

    def add_features(self):
        brep_a, brep_b, brep_adv = self.create_negative_volumes(*self.intersection_polygon())
        plane_a = self.get_end_face()
        start_a, end_a = self.main_beam.extension_to_plane(plane_a)
        self.main_beam.add_blank_extension(start_a, end_a, self.key)
        self.main_beam.add_features(MillVolume(brep_b))
        self.main_beam.add_features(MillVolume(brep_adv))
        self.main_beam.add_features(CutFeature(plane_a))
        self.cross_beam.add_features(MillVolume(brep_a))
