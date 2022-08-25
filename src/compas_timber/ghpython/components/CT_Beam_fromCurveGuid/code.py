import Rhino
import Rhino.Geometry as rg
import rhinoscriptsyntax as rs

from compas_rhino.geometry import RhinoCurve
from compas_rhino.conversions import vector_to_compas, line_to_compas
from compas_ghpython.utilities import unload_modules
from compas_timber.parts.beam import Beam as ctBeam
from compas_timber.utils.rhino_object_name_attributes import update_rhobj_attributes_name

import Grasshopper.Kernel as ghk
warning = ghk.GH_RuntimeMessageLevel.Warning
error = ghk.GH_RuntimeMessageLevel.Error

if not refCrv:
    ghenv.Component.AddRuntimeMessage(warning, "Input parameter refCrv failed to collect data")
if not Width: 
    ghenv.Component.AddRuntimeMessage(warning, "Input parameter Width failed to collect data")
if not Height: 
    ghenv.Component.AddRuntimeMessage(warning, "Input parameter Height failed to collect data")

#=============================


if not ZVector: ZVector=[None]
if not Category: Category=[None]
if not Group: Group=[None]


if refCrv and Height and Width:
    #check list lengths for consistency
    n = len(refCrv)
    if len(ZVector) not in (0,1,n): 
        ghenv.Component.AddRuntimeMessage(error, " In 'ZVector' I need either none, one or the same number of inputs as the refCrv parameter.")
    if len(Width) not in (1,n): 
        ghenv.Component.AddRuntimeMessage(error, " In 'W' I need either one or the same number of inputs as the refCrv parameter.")
    if len(Height) not in (1,n): 
        ghenv.Component.AddRuntimeMessage(error, " In 'H' I need either one or the same number of inputs as the refCrv parameter.")
    if len(Category) not in (0,1,n): 
        ghenv.Component.AddRuntimeMessage(error, " In 'Category' I need either none, one or the same number of inputs as the refCrv parameter.")
    if len(Group) not in (0,1,n): 
        ghenv.Component.AddRuntimeMessage(error, " In 'Group' I need either none, one or the same number of inputs as the refCrv parameter.")

    #duplicate data
    if len(ZVector)!=n: ZVector = [ZVector[0] for _ in range(n)]
    if len(Width)!=n: Width = [Width[0] for _ in range(n)]
    if len(Height)!=n: Height = [Height[0] for _ in range(n)]
    if len(Category)!=n: Category = [Category[0] for _ in range(n)]
    if len(Group)!=n: Group = [Group[0] for _ in range(n)]


    Beam = []
    for guid,z,w,h,c,g  in zip(refCrv, ZVector,Width, Height, Category, Group):
        if guid==None or w==None or h==None:
            ghenv.Component.AddRuntimeMessage(warning, "Some of the input values are Null")
        else:
            crv = Rhino.RhinoDoc.ActiveDoc.Objects.FindId(guid).Geometry
            line = rg.Line(crv.PointAtStart,crv.PointAtEnd)
            
            line = line_to_compas(line)
            if z: z = vector_to_compas(z) 
            else: None
            
            beam = ctBeam.from_centreline(line,z,w,h)

            beam.attributes['rhino_guid']= guid
            beam.attributes['category']= c
            beam.attributes['group'] = g

            if update_attrs:
                update_rhobj_attributes_name(guid,"width", str(w))
                update_rhobj_attributes_name(guid,"height", str(h))
                update_rhobj_attributes_name(guid,"zaxis", str(list(beam.frame.zaxis)))
                update_rhobj_attributes_name(guid,"category", c)
            
            Beam.append(beam)
