#!/usr/bin/env python3
"""Package this self-contained release with file hashes; run checks first."""
from pathlib import Path
import hashlib,json,shutil,zipfile,xml.etree.ElementTree as E
R=Path(__file__).resolve().parents[1];P=R/'ros2/bellybot_aircraft_environment'
for d in list(R.rglob('__pycache__')):shutil.rmtree(d)
for f in R.rglob('*'):
 if f.is_file() and f.suffix in ['.pyc','.nbc','.nbi']:f.unlink()
assert json.loads((R/'docs/validation.json').read_text())['offline_checks_passed']
assert json.loads((R/'docs/controller_contract_checks.json').read_text())['passed']
check=json.loads((R/'docs/resolution_checks.json').read_text());assert check['min_corner_door_gap_m']>0 and check['min_corner_buffer_gap_m']>0
assets={}
for f in sorted(P.glob('models/*/model.sdf')):
 m=E.parse(f).getroot().find('model');assets[f.parent.name]={'links':len(m.findall('link')),'joints':len(m.findall('joint')),'static':m.findtext('static')=='true','assigned_mass_kg':sum(float(l.findtext('inertial/mass','0')) for l in m.findall('link')),'collisions':len(m.findall('.//collision'))}
(R/'docs/assets.json').write_text(json.dumps(assets,indent=2))
manifest={'revision':'2.0','date':'2026-10-10','robot':'Bundled unchanged BellyBot DockStack C v3 ROS package and original CAD source','new_scenarios':['entry_resolved','transfer_resolved','transfer_cutaway'],'robot_dimension_changed':False,'aircraft_dimensions_enlarged':False,'verification':['CAD valid STEP reimports','Entry/pivot whole-triangle clearance','Sampled ramp envelope and transfer motion','477 conservative robot/buffer pose bounds','10 meaningful controller interlock/limit checks','Local resources, inertia and URDF graphs'],'runtime_tested':False,'supports_12_bags_per_min_demonstrated':False,'preview_video':'30 seconds, CAD kinematics, time compressed; not physics or throughput evidence','setup_condition':'Entry fixtures removed before work fixtures installed; docked service platform assumed; changeover not automated','primary_CAD':'exports/BellyBot_Aircraft_Work_Assembly_R2_mm.step','primary_review':'docs/Resolution_Review_R2.pdf'}
(R/'docs/release_manifest.json').write_text(json.dumps(manifest,indent=2))
files=sorted(f for f in R.rglob('*') if f.is_file() and f.name!='MANIFEST.sha256');hashes=[hashlib.sha256(f.read_bytes()).hexdigest()+'  '+f.relative_to(R).as_posix() for f in files];(R/'MANIFEST.sha256').write_text('\n'.join(hashes)+'\n')
out=R.parent/(R.name+'.zip')
with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for f in sorted(R.rglob('*')):
  if f.is_file():z.write(f,arcname=R.name+'/'+f.relative_to(R).as_posix())
with zipfile.ZipFile(out) as z:
 assert z.testzip() is None
 for row in hashes:
  expected,relative=row.split('  ',1);assert hashlib.sha256(z.read(R.name+'/'+relative)).hexdigest()==expected,relative
print(json.dumps({'zip':str(out.resolve()),'files':len(files)+1,'bytes':out.stat().st_size,'sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'archive_hashes_checked':len(files)},indent=2))
