#!/usr/bin/env python3
"""Export exact delivered forward collision panels as a mm-unit STEP assembly.
STEP is a fixture layout, not aircraft manufacturing data. Mesh render detail
(rivets, workers, cloth, cases) remains in GLB/DAE. Requires CadQuery + trimesh.
"""
from pathlib import Path
import json
import cadquery as cq
import trimesh as tm
ROOT=Path(__file__).resolve().parents[1];folder=ROOT/'ros2/bellybot_aircraft_environment/models/nb_hold/meshes'
a=cq.Assembly(name='NB_forward_hold_fixture');checks=[]
for p in sorted(folder.glob('collision_*.stl')):
 m=tm.load_mesh(p,process=True);faces=[]
 for tri in m.triangles*1000:
  wire=cq.Wire.makePolygon([cq.Vector(*v) for v in tri],close=True);faces.append(cq.Face.makeFromWires(wire))
 solid=cq.Solid.makeSolid(cq.Shell.makeShell(faces));assert solid.isValid(),p.name
 a.add(solid,name=p.stem,color=cq.Color(.78,.81,.77));checks.append({'name':p.stem,'valid':True,'volume_mm3':solid.Volume()})
out=ROOT/'exports/NB_Forward_Hold_Layout_mm.step';a.save(str(out));imported=cq.importers.importStep(str(out)).val();assert imported.isValid()
(ROOT/'docs/step_check.json').write_text(json.dumps({'valid':True,'solid_count':len(imported.Solids()),'units':'mm','scope':'Delivered forward-hold collision surfaces, not aircraft structure certification','parts':checks},indent=2));print('STEP export checked:',len(imported.Solids()),'solids')
