#!/usr/bin/env python3
"""CAD-derived stills and MP4; software rasterizer, optional Numba acceleration."""
import math,itertools,subprocess,sys,time
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from model import *
from motion import sample,DURATION
try:
    from numba import njit
except ImportError:
    def njit(*args,**kwargs):return lambda f:f

CACHE={}
def font(n,bold=False):
    return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans'+('-Bold' if bold else '')+'.ttf',n)

@njit(cache=True)
def raster(tri,colors,width,height):
    pixels=np.empty((height,width,3),np.uint8);pixels[:,:,0]=246;pixels[:,:,1]=247;pixels[:,:,2]=244
    depth=np.full((height,width),-1e30)
    for i in range(len(tri)):
        v=tri[i];x0,y0,z0=v[0];x1,y1,z1=v[1];x2,y2,z2=v[2]
        den=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
        if abs(den)<1e-10:continue
        xmin=max(0,int(math.floor(min(x0,x1,x2))));xmax=min(width-1,int(math.ceil(max(x0,x1,x2))))
        ymin=max(0,int(math.floor(min(y0,y1,y2))));ymax=min(height-1,int(math.ceil(max(y0,y1,y2))))
        for y in range(ymin,ymax+1):
            yy=y+.5
            for x in range(xmin,xmax+1):
                xx=x+.5;a=((y1-y2)*(xx-x2)+(x2-x1)*(yy-y2))/den
                b=((y2-y0)*(xx-x2)+(x0-x2)*(yy-y2))/den;c=1-a-b
                if a<0 or b<0 or c<0:continue
                z=a*z0+b*z1+c*z2
                if z>depth[y,x]:
                    depth[y,x]=z
                    for k in range(3):pixels[y,x,k]=colors[i,k]
    return pixels

def mesh(p):
    key=p['name']
    if key not in CACHE:
        vertices,faces=p['shape'].tessellate(.6,.3)
        vs=np.array([v.toTuple() for v in vertices]);CACHE[key]=vs[np.array(faces)]
    return CACHE[key]

def camera(view):
    el,az=map(math.radians,{'iso':(25,-53),'front':(10,0),'side':(0,-90),'top':(90,-90),'detail':(20,-70)}[view])
    normal=np.array([math.cos(el)*math.cos(az),math.cos(el)*math.sin(az),math.sin(el)])
    right=np.array([-math.sin(az),math.cos(az),0]);up=np.cross(normal,right)
    return np.stack([right,up,normal],axis=1)

def render(q,path=None,view='iso',bag=False,feed=True,size=(1600,1050),bounds=None,links=None):
    ts=fk(q);tris=[];cols=[]
    light=np.array([.15,-.5,.85]);light/=np.linalg.norm(light)
    for p in PARTS:
        if links is not None and p['link'] not in links:continue
        if p['link']=='sample_bag' and not bag:continue
        if p['link'] in LINKS and LINKS[p['link']]['group']=='infeed' and not feed:continue
        t=ts[p['link']];tri=mesh(p)@t[:3,:3].T+t[:3,3]
        n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-10)
        color=np.array(COLORS[p['color']])[None,:]*(.57+.43*np.maximum(n@light,0))[:,None]
        tris.append(tri);cols.append((color*255).astype(np.uint8))
    mat=camera(view);tri=np.concatenate(tris)@mat
    if bounds is None:lo=tri[:,:,:2].min(axis=(0,1));hi=tri[:,:,:2].max(axis=(0,1))
    else:
        points=np.array(bounds)@mat;lo=points[:,:2].min(axis=0);hi=points[:,:2].max(axis=0)
    width,height=size;scale=min(width*.9/(hi[0]-lo[0]),height*.78/(hi[1]-lo[1]))
    tri[:,:,0]=(tri[:,:,0]-(lo[0]+hi[0])/2)*scale+width/2
    tri[:,:,1]=-(tri[:,:,1]-(lo[1]+hi[1])/2)*scale+height/2
    im=Image.fromarray(raster(tri,np.concatenate(cols),width,height))
    if path:im.save(path)
    return im

def sheet():
    docs=ROOT/'docs'
    for name,q,view,bag,feed in [('home',POSES['home'],'iso',False,True),
      ('placement',POSES['placement'],'iso',True,True),('floor_access',POSES['floor_insert'],'iso',False,False),
      ('floor_side',POSES['floor_insert'],'side',False,False),('layer_match',POSES['batch_receive'],'iso',False,True)]:
        print('Rendering',name,flush=True);render(q,docs/(name+'.png'),view,bag,feed)
    render(POSES['home'],docs/'yaw_mechanism.png',view='iso',feed=False,
           links={'lift_stage_3','yaw_carrier','extension_mid','tray_carriage'})
    page=Image.new('RGB',(2400,1800),'#f6f7f4');d=ImageDraw.Draw(page)
    d.text((70,40),'BELLYBOT / DOCK & STACK — REVISION 3',font=font(46,True),fill='#173f36')
    d.text((73,110),'Connected yaw frame + bearing mounts + retained slide guides | Actual CAD geometry',font=font(25),fill='#536960')
    entries=[('placement',(40,180),'01  SKETCH CONFIGURATION','Central mast / circular yaw stage / thin belt cassette / 10 rotating wheels'),
      ('yaw_mechanism',(1240,180),'02  CORRECTED SUPPORT FRAME','Tray hidden to expose the bolted bearing, cranked ribs and captured guides'),
      ('layer_match',(40,915),'03  REPEAT TRANSFERS AT ONE HEIGHT','Home belt raised to 495 mm; robot and infeed lift together for layer batches'),
      ('floor_side',(1240,915),'04  FLOOR ACCESS PRESERVED','30.92 deg floor pitch / 1000 mm reach / 2 mm nominal nose clearance')]
    for name,pos,title,caption in entries:
        im=Image.open(docs/(name+'.png')).resize((1120,735));page.paste(im,pos)
        d.text((pos[0]+20,pos[1]+5),title,font=font(25,True),fill='#173f36')
        d.text((pos[0]+20,pos[1]+698),caption,font=font(18),fill='#536960')
    d.line((70,1690,2330,1690),fill='#a8bbb2',width=2)
    d.text((73,1710),'CONNECTED SOLIDS CHECKED | Strength, bag contact and sustained 12 bags/min still require testing.',font=font(23,True),fill='#173f36')
    d.text((73,1752),'4.66 s modeled fixed-layer cycle; floor and yaw routes are slower. No Gazebo or hardware performance claim.',font=font(23),fill='#536960')
    page.save(docs/'BellyBot_DockStack_v3_overview.png')

def video():
    # Direct tessellation animation: truthful commissioning preview, no physical
    # grasp/transfer implied. Does not include an attached or teleported bag.
    out=ROOT/'docs/BellyBot_DockStack_v3_motion.mp4'
    width,height=1280,800;fps=15
    bounds=list(itertools.product([-2050,1480],[-650,1100],[0,900]))
    cmd=['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}',
         '-r',str(fps),'-i','-','-an','-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    for i in range(round(DURATION*fps)):
        t=i/fps;q,dq,label=sample(t);im=render(q,size=(width,height),bounds=bounds)
        d=ImageDraw.Draw(im)
        d.text((34,25),'BELLYBOT | Dock & Stack v3',font=font(32,True),fill='#173f36')
        d.text((35,70),f'{t:04.1f} s  /  {label}',font=font(22),fill='#536960')
        total=sum(q[f'lift_stage_{j}_joint'] for j in [1,2,3]);reach=q['extension_mid_joint']+q['extension_tip_joint']
        d.text((35,710),f'Lift {total*1000:.0f} mm | Pitch {math.degrees(q["tray_pitch_joint"]):.1f} deg | Reach {reach*1000:.0f} mm',font=font(23),fill='#173f36')
        d.text((35,754),'CAD kinematics only - no baggage contact, actuator or throughput validation.',font=font(20),fill='#536960')
        proc.stdin.write(im.tobytes())
        if i%(fps*5)==0:print(f'Video {t:.0f}/{DURATION}s',flush=True)
    proc.stdin.close();assert proc.wait()==0

if __name__=='__main__':
    build()
    if '--video-only' not in sys.argv:sheet()
    if '--video' in sys.argv or '--video-only' in sys.argv:video()
