from compas.geometry import Frame

from compas_timber.connections.lap_joint import LapJoint
from compas_timber.parts import CutFeature
from compas_timber.parts import MillVolume

from .joint import BeamJoinningError
from .solver import JointTopology


class THalfLapJoint(LapJoint):
    """Represents a T-Lap type joint which joins the end of a beam along the length of another beam,
    trimming the main beam.

    This joint type is compatible with beams in T topology.

    Please use `THalfLapJoint.create()` to properly create an instance of this class and associate it with an assembly.

    Parameters
    ----------
    main_beam : :class:`~compas_timber.parts.Beam`
        The main beam to be joined.
    cross_beam : :class:`~compas_timber.parts.Beam`
        The cross beam to be joined.
    flip_lap_side : bool
        If True, the lap is flipped to the other side of the beams.
    cut_plane_bias : float
        Allows lap to be shifted deeper into one beam or the other. Value should be between 0 and 1.0 without completely cutting through either beam. Default is 0.5.

    Attributes
    ----------
    beams : list(:class:`~compas_timber.parts.Beam`)
        The beams joined by this joint.
    main_beam : :class:`~compas_timber.parts.Beam`
        The main beam to be joined.
    cross_beam : :class:`~compas_timber.parts.Beam`
        The cross beam to be joined.
    main_beam_key : str
        The key of the main beam.
    cross_beam_key : str
        The key of the cross beam.
    features : list(:class:`~compas_timber.parts.Feature`)
        The features created by this joint.
    joint_type : str
        A string representation of this joint's type.

    """

    SUPPORTED_TOPOLOGY = JointTopology.TOPO_T

    def __init__(self, main_beam=None, cross_beam=None, flip_lap_side=False, cut_plane_bias=0.5, frame=None, key=None):
        super(THalfLapJoint, self).__init__(main_beam, cross_beam, flip_lap_side, cut_plane_bias, frame, key)

    @property
    def joint_type(self):
        return "T-HalfLap"

    def add_features(self):
        assert self.main_beam and self.cross_beam  # should never happen

        try:
            main_cutting_frame = self.get_main_cutting_frame()
            negative_brep_main_beam, negative_brep_cross_beam = self._create_negative_volumes()
        except Exception as ex:
            raise BeamJoinningError(beams=self.beams, joint=self, debug_info=str(ex))

        start_main, end_main = self.main_beam.extension_to_plane(main_cutting_frame)
        extension_tolerance = 0.01  # TODO: this should be proportional to the unit used
        self.main_beam.add_blank_extension(start_main + extension_tolerance, end_main + extension_tolerance, self.key)

        main_beam_mill = MillVolume(negative_brep_main_beam)
        cross_beam_mill = MillVolume(negative_brep_cross_beam)
        main_beam_mill.name = "Main Lap"
        cross_beam_mill.name = "Cross Lap"

        self.main_beam.add_features(main_beam_mill)
        self.cross_beam.add_features(cross_beam_mill)
        self.features.extend([main_beam_mill, cross_beam_mill])

        trim_frame = Frame(main_cutting_frame.point, main_cutting_frame.xaxis, -main_cutting_frame.yaxis)
        f_main = CutFeature(trim_frame)
        self.main_beam.add_features(f_main)
        self.features.append(f_main)
