#!/usr/bin/env python3
"""CPU rasterized views of the delivered meshes; not an image-generation mockup."""
from pathlib import Path
import sys,math,json
import numpy as np
import trimesh as tm
from PIL import Image,ImageDraw,ImageFont
from numba import njit
ROOT=Path(__file__).resolve().parents[1];PKG=ROOT/'ros2/bellybot_aircraft_environment'
@njit(cache=True)
def raster(tris,colors,w,h):
 im=np.empty((h,w,3),np.uint8);im[:,:,0]=237;im[:,:,1]=241;im[:,:,2]=237;depth=np.full((h,w),-1e20)
 for i in range(len(tris)):
  v=tris[i];x0,y0,z0=v[0];x1,y1,z1=v[1];x2,y2,z2=v[2];den=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
  if abs(den)<1e-10:continue
  xmin=max(0,int(math.floor(min(x0,x1,x2))));xmax=min(w-1,int(math.ceil(max(x0,x1,x2))));ymin=max(0,int(math.floor(min(y0,y1,y2))));ymax=min(h-1,int(math.ceil(max(y0,y1,y2))))
  for y in range(ymin,ymax+1):
   for x in range(xmin,xmax+1):
    xx=x+.5;yy=y+.5;a=((y1-y2)*(xx-x2)+(x2-x1)*(yy-y2))/den;b=((y2-y0)*(xx-x2)+(x0-x2)*(yy-y2))/den;c=1-a-b
    if a<0 or b<0 or c<0:continue
    z=a*z0+b*z1+c*z2
    if z>depth[y,x]:depth[y,x]=z;im[y,x]=colors[i]
 return im

def collect(file,skip=()):
 scene=tm.load(file,force='scene');out=[]
 for node in scene.graph.nodes_geometry:
  if any(s in node for s in skip):continue
  T,gn=scene.graph[node];m=scene.geometry[gn].copy();m.apply_transform(T);out.append((node,m))
 return out

def render(parts,file,title,subtitle,el=26,az=-58,size=(1800,1100),bounds=None):
 tris=[];colors=[];light=np.array([-.1,-.4,.85]);light/=np.linalg.norm(light)
 for name,m in parts:
  tri=m.triangles;norm=m.face_normals;col=m.visual.face_colors[:,:3];shade=.50+.50*np.maximum(0,norm@light)
  tris.append(tri);colors.append(np.clip(col*shade[:,None],0,255).astype(np.uint8))
 tri=np.concatenate(tris);cols=np.concatenate(colors);e,a=math.radians(el),math.radians(az);normal=np.array([math.cos(e)*math.cos(a),math.cos(e)*math.sin(a),math.sin(e)]);right=np.array([-math.sin(a),math.cos(a),0]);up=np.cross(normal,right);mat=np.stack([right,up,normal],axis=1);tri=tri@mat
 if bounds is None:lo=tri[:,:,:2].min((0,1));hi=tri[:,:,:2].max((0,1))
 else:p=np.array(bounds)@mat;lo=p[:,:2].min(0);hi=p[:,:2].max(0)
 w,h=size;scale=min(w*.90/(hi[0]-lo[0]),h*.72/(hi[1]-lo[1]));tri[:,:,0]=(tri[:,:,0]-(lo[0]+hi[0])/2)*scale+w/2;tri[:,:,1]=-(tri[:,:,1]-(lo[1]+hi[1])/2)*scale+h*.54
 im=Image.fromarray(raster(tri,cols,w,h));d=ImageDraw.Draw(im)
 def font(n,b=False):return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans'+('-Bold' if b else '')+'.ttf',n)
 d.rectangle([0,0,w,130],fill='#153e3b');d.text((50,27),title,font=font(36,True),fill='white');d.text((52,81),subtitle,font=font(20),fill='#bed1c4')
 d.text((50,h-52),'Actual delivered meshes  |  Reference-based test geometry; intermediate dimensions are assumptions',font=font(19),fill='#425e58');im.save(file)
 return im

def main():
 cut=collect(ROOT/'exports/presentation_cutaway.glb',skip=['bulkhead_0','end_protect_0'])
 robot=collect(ROOT/'exports/bellybot_reference_pose.glb') if (ROOT/'exports/bellybot_reference_pose.glb').exists() else []
 render(cut+robot,ROOT/'docs/environment_overview.png','BELLYBOT / AIRCRAFT ENVIRONMENT','7.70 m forward-hold fixture  /  1.12 m ceiling  /  side door and apron belt loader')
 interior=collect(PKG/'models/nb_hold_cutaway/meshes/visual.glb',skip=['bulkhead_0','end_protect_0'])
 workers=collect(PKG/'models/manual_workers/meshes/visual.glb')
 render(interior+workers,ROOT/'docs/manual_workspace.png','MANUAL PROCESS / REFERENCE SCENE','Kneeling worker proxies, low roof and narrow floor; posture illustration, not a biomechanical assessment',el=19,az=-115)
 render(interior+robot,ROOT/'docs/robot_workspace.png','BELLYBOT / WORKING CLEARANCE','Robot aligned along the hold. Chassis turning and upper-arm reach need separate swept checks.',el=28,az=-110)
 # Front cross-section shows profile directly; strip long shell to a .6 m slice.
 section=collect(PKG/'models/nb_hold/meshes/visual.glb',skip=['bulkhead','end_protect','open_door','folded_curtain','net_'])
 selected=[]
 for n,m in section:
  if m.bounds[0,0]>=3.0 and m.bounds[1,0]<=3.9:selected.append((n,m))
 # Explicit exact profile slice ensures floor/roof appear even when panel extents straddle selection.
 floor=tm.creation.box([.02,1.22,.045],transform=tm.transformations.translation_matrix([3.5,0,-.0225]));floor.visual.face_colors=[130,148,136,255];selected.append(('floor',floor))
 render(selected+robot,ROOT/'docs/cross_section.png','CROSS SECTION / STOWED BELLYBOT','1.22 m flat floor  /  1.10 m crawler width  /  no full chassis pivot assumed',el=0,az=180)
 # Preview contains environment and stationary robot meshes only, no physical task animation.
 sc=tm.Scene()
 for n,m in cut+robot:sc.add_geometry(m,node_name=n)
 sc.export(ROOT/'exports/environment_with_bellybot_preview.glb')
 print('Rendered four mesh-based views',flush=True)
if __name__=='__main__':main()

# Optional perspective reference view, using near-plane clipping of actual triangles.
def perspective(parts,file):
 eye=np.array([1.05,-1.10,.79]);target=np.array([4.1,.05,.47]);forward=target-eye;forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);up=np.cross(right,forward);cam=np.stack([right,up,forward],axis=1)
 tri_out=[];colors=[];light=np.array([-.1,-.2,.96]);light/=np.linalg.norm(light)
 for name,m in parts:
  t=(m.triangles-eye)@cam;shade=.65+.35*np.abs(m.face_normals@light);cs=np.clip(m.visual.face_colors[:,:3]*shade[:,None],0,255).astype(np.uint8)
  for tri,col in zip(t,cs):
   poly=list(tri);out=[]
   for i in range(len(poly)):
    A=poly[i-1];B=poly[i];ina=A[2]>=.045;inb=B[2]>=.045
    if ina!=inb:out.append(A+(B-A)*((.045-A[2])/(B[2]-A[2])))
    if inb:out.append(B)
   if len(out)<3:continue
   for i in range(1,len(out)-1):tri_out.append([out[0],out[i],out[i+1]]);colors.append(col)
 tri=np.array(tri_out);w,h=1600,1000;f=900.;z=tri[:,:,2].copy();tri[:,:,0]=w/2+f*tri[:,:,0]/z;tri[:,:,1]=h/2-f*tri[:,:,1]/z;tri[:,:,2]=1/z
 im=Image.fromarray(raster(tri,np.array(colors),w,h));d=ImageDraw.Draw(im);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',28)
 d.rectangle([0,0,w,75],fill='#153e3b');d.text((30,20),'INTERIOR / LOW-CEILING MANUAL WORKSPACE',font=font,fill='white');im.save(file)

if __name__=='__main__':
 parts=collect(PKG/'models/nb_hold/meshes/visual.glb')+collect(PKG/'models/manual_workers/meshes/visual.glb')
 perspective(parts,ROOT/'docs/interior_reference.png')
