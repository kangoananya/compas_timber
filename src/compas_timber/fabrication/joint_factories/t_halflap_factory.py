from compas_timber.connections import THalfLapJoint
from compas_timber.fabrication import BTLx
from compas_timber.fabrication import BTLxLap
from compas_timber.fabrication import BTLxJackCut

import math

from compas.geometry import Frame
from compas.geometry import Vector
from compas.geometry import angle_vectors_signed
from compas.geometry import cross_vectors
from compas.geometry import dot_vectors
from compas.geometry import distance_point_point_sqrd
from compas.geometry import is_coplanar


class THalfLapFactory(object):
    """Factory class for creating T-Butt joints."""

    def __init__(self):
        pass

    @classmethod
    def apply_processings(cls, joint, parts):
        """
        Apply processings to the joint and its associated parts.

        Parameters
        ----------
        joint : :class:`~compas_timber.connections.joint.Joint`
            The joint object.
        parts : dict
            A dictionary of the BTLxParts connected by this joint, with part keys as the dictionary keys.

        Returns
        -------
        None

        """

        main, cross = parts[str(joint.main_beam.key)], parts[str(joint.cross_beam.key)]
        main_params, cross_params = THalfLapFactory.create_lap_parameters(main, cross, joint)
        cut_frame = main_params["cut_frame"]
        main.processings.append(BTLxJackCut.create_process(main, cut_frame))
        main.processings.append(BTLxLap.create_process(main, joint.name, main_params))
        cross.processings.append(BTLxLap.create_process(cross, joint.name, cross_params))

    @staticmethod
    def create_lap_parameters(main, cross, joint):
        """
        main: BTLxPart corresponding to the main beam
        cross: BTLxPart corresponding to the cross beam
        joint: THalfLapJoint
        """
        main_params = {}
        cross_params = {}

        beam = main.beam
        main_reference_plane = main.reference_surface_planes(2)
        cross_reference_plane = cross.reference_surface_planes(4)

        features = beam.features
        for feature in features:
            if feature.name == "MillVolume":
                mill_feature = feature
            elif feature.name == "CutFeature":
                cut_feature = feature

        mill_vol = mill_feature.volume.vertices

        a, b, c, d = [mill_feature.volume.vertices[i] for i in [1, 7, 5, 3]]
        e, f, g, h = [mill_feature.volume.vertices[i] for i in [0, 2, 4, 6]]

        face_orientation = cross_vectors((d - a), (b - a))
        if dot_vectors(main_reference_plane.zaxis, face_orientation) > 0 and is_coplanar([e,f,g,main_reference_plane.point], 0.1):
            lap_plane = Frame(a, d - a, b - a)
            cut_face = a, b, c, d
        else:
            lap_plane = Frame(e, f - e, g - e)
            cut_face = e, f, g, h

        other_vector = cross.beam.frame.xaxis

        angle_beams = math.pi - abs(
            angle_vectors_signed(main_reference_plane.xaxis, other_vector, main_reference_plane.normal)
        )
        lap_slope = math.degrees(
            angle_vectors_signed(main_reference_plane.yaxis, lap_plane.yaxis, main_reference_plane.normal)
        )
        lap_inclination = math.degrees(
            angle_vectors_signed(main_reference_plane.xaxis, lap_plane.xaxis, main_reference_plane.normal)
        )

        if angle_beams < 0:
            angle_beams = abs(angle_beams)
            _ref_edge = False
        else:
            angle_beams = math.pi - angle_beams
            _ref_edge = True

        startX_main = beam.width / abs(math.tan(angle_beams))
        if joint.ends[str(main.key)] == "end":
            if _ref_edge:
                startX_main = beam.blank_length - startX_main
            else:
                startX_main = beam.blank_length + startX_main

        main_params["Angle"] = math.degrees(angle_beams)
        main_params["startX"] = startX_main
        main_params["startY"] = 0.0
        main_params["Inclination"] = 90.0 #90 - lap_inclination
        main_params["Slope"] = 0.0 #min(abs(lap_slope), 180 - abs(lap_slope))
        main_params["Length"] = cross.beam.width #cross.beam.width / math.cos(math.pi / 2 - angle_beams)
        main_params["Depth"] = abs(
            dot_vectors(Vector.from_start_end(main_reference_plane.point, lap_plane.point), main_reference_plane.zaxis)
        )
        main_params["Width"] = main.beam.width
        main_params["Orientation"] = joint.ends[str(main.key)]

        main_params["cut_frame"] = cut_feature.cutting_plane
        main_params["ReferencePlaneID"] = "2"

        lap_point = sorted(cut_face, key=lambda p: distance_point_point_sqrd(p, cross_reference_plane.point))[0]
        cross_params["Angle"] = 180 - math.degrees(angle_beams)
        cross_params["startX"] = dot_vectors(
            Vector.from_start_end(cross_reference_plane.point, lap_point), cross_reference_plane.xaxis
        )
        cross_params["startY"] = 0.0
        cross_params["Inclination"] = 90.0 #90 - lap_inclination
        cross_params["Slope"] = 0.0 #min(abs(lap_slope), 180 - abs(lap_slope))
        cross_params["Length"] = main.beam.width #main.beam.width / math.cos(math.pi / 2 - angle_beams)
        cross_params["Depth"] = cross.beam.height - main_params["Depth"]
        cross_params["Width"] = cross.beam.width
        cross_params["Orientation"] = joint.ends[str(cross.key)]

        cross_params["ReferencePlaneID"] = "4"

        return main_params, cross_params


BTLx.register_joint(THalfLapJoint, THalfLapFactory)
