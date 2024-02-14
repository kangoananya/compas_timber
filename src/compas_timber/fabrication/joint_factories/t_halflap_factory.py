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
        main.processings.append(BTLxJackCut.create_process(main, THalfLapFactory._get_main_cutting_frame(joint)))
        main.processings.append(BTLxLap.create_process(main, joint.name, main_params))
        cross.processings.append(BTLxLap.create_process(cross, joint.name, cross_params))

    @staticmethod
    def _get_main_lap_plane(main, cross, joint):
        """
        Get the plane of the lap joint on the main beam.

        Parameters
        ----------
        joint : :class:`~compas_timber.connections.joint.Joint`
            The joint object.

        Returns
        -------
        cut_frame : compas.geometry.Frame
            The frame of the lap joint on the main beam.
        """
        main_reference_plane = main.reference_surface_planes(2)

        features = joint.features
        for feature in features:
            if feature.name == "Main Lap":
                mill_feature = feature

        a, b, c, d = [mill_feature.volume.vertices[i] for i in [1, 7, 5, 3]]
        e, f, g, h = [mill_feature.volume.vertices[i] for i in [0, 2, 4, 6]]

        face_orientation = cross_vectors((d - a), (b - a))

        if (
            dot_vectors(main_reference_plane.zaxis, face_orientation) > 0
            and joint.ends[str(cross.key)] == "start"
            and joint.ends[str(main.key)] == "start"
        ):
            lap_plane = Frame(a, d - a, b - a)
            cut_face = a, b, c, d
        else:
            lap_plane = Frame(e, f - e, g - e)
            cut_face = e, f, g, h

        return lap_plane, cut_face

    @staticmethod
    def _get_main_cutting_frame(joint):
        """
        Get the frame of the cutting plane on the main beam.

        Parameters
        ----------
        joint : :class:`~compas_timber.connections.joint.Joint`
            The joint object.

        Returns
        -------
        cut_frame : compas.geometry.Frame
            The frame of the cutting plane on the main beam.
        """
        for feature in joint.features:
            if feature.name == "CutFeature":
                return feature.cutting_plane

    @staticmethod
    def _get_beams_angle(main_reference_plane, cross_reference_plane
                         ):
        """
        Get the angle between the main and cross beams.

        Parameters
        ----------
        main_reference_plane : compas.geometry.Plane
            The reference plane of the main beam.
        cross_reference_plane : compas.geometry.Plane
            The reference plane of the cross beam.

        Returns
        -------
        angle_beams : float
            The angle between the main and cross beams.
        """
        angle_beams = angle_vectors_signed(
            main_reference_plane.xaxis, cross_reference_plane.xaxis, main_reference_plane.normal
        )
        if angle_beams < 0:
            angle_beams = abs(angle_beams)
            _ref_edge = True
        else:
            angle_beams =  math.pi - angle_beams
            _ref_edge = False
        return angle_beams, _ref_edge

    @staticmethod
    def create_lap_parameters(main, cross, joint):
        """
        main: BTLxPart corresponding to the main beam
        cross: BTLxPart corresponding to the cross beam
        joint: THalfLapJoint
        """
        reference_id_main = 2
        reference_id_cross = 4

        beam = main.beam
        main_reference_plane = main.reference_surface_planes(reference_id_main)
        cross_reference_plane = cross.reference_surface_planes(reference_id_cross)

        lap_plane, _cut_face = THalfLapFactory._get_main_lap_plane(main, cross, joint)
        angle_beams, _ref_edge = THalfLapFactory._get_beams_angle(main_reference_plane, cross_reference_plane)

        lap_slope = math.degrees(
            angle_vectors_signed(main_reference_plane.yaxis, lap_plane.yaxis, main_reference_plane.normal)
        )
        lap_inclination = math.degrees(
            angle_vectors_signed(main_reference_plane.xaxis, lap_plane.xaxis, main_reference_plane.normal)
        )

        _diagonal_length = beam.width / abs(math.tan(angle_beams))

        if joint.ends[str(main.key)] == "end":
            if _ref_edge:
                startX_main = beam.blank_length - _diagonal_length
            else:
                startX_main = beam.blank_length + _diagonal_length
        else:
            startX_main = _diagonal_length if angle_beams < math.pi / 2 else 0

        angle_main = angle_beams if joint.ends[str(main.key)] == "start" else math.pi - angle_beams
        length_main = cross.beam.width  # / math.cos(math.pi / 2 - angle_beams)
        depth_main = abs(
            dot_vectors(Vector.from_start_end(main_reference_plane.point, lap_plane.point), main_reference_plane.zaxis)
        )
        main_params = THalfLapFactory._create_beam_params(
            angle_main, startX_main, length_main, depth_main, beam, joint, reference_id_main
        )

        lap_point = sorted(_cut_face, key=lambda p: distance_point_point_sqrd(p, cross_reference_plane.point))[0]

        if joint.ends[str(cross.key)] == "start":
            startX_cross = dot_vectors(
                Vector.from_start_end(cross_reference_plane.point, lap_point), cross_reference_plane.xaxis
            )
        else:
            startX_cross = main.beam.width / math.sin(angle_main) + dot_vectors(
                Vector.from_start_end(cross_reference_plane.point, lap_point), cross_reference_plane.xaxis
            )

        angle_cross = angle_beams if joint.ends[str(cross.key)] == "start" else math.pi - angle_beams
        depth_cross = cross.beam.height - main_params["Depth"]
        startX_cross = startX_cross if angle_beams > math.pi / 2 else startX_cross + _diagonal_length
        length_cross = main.beam.width  # / math.cos(math.pi / 2 - angle_beams)
        cross_params = THalfLapFactory._create_beam_params(
            angle_cross, startX_cross, length_cross, depth_cross, cross.beam, joint, reference_id_cross
        )

        return main_params, cross_params

    @staticmethod
    def _create_beam_params(angle, startX, length, depth, beam, joint, reference_plane_id):
        params = {}
        params["Angle"] = math.degrees(angle)
        params["startX"] = startX
        params["startY"] = 0.0
        params["Inclination"] = 90.0
        params["Slope"] = 0.0
        params["Length"] = length
        params["Depth"] = depth
        params["Width"] = beam.width
        params["Orientation"] = joint.ends[str(beam.key)]
        params["ReferencePlaneID"] = str(reference_plane_id)
        return params


BTLx.register_joint(THalfLapJoint, THalfLapFactory)
