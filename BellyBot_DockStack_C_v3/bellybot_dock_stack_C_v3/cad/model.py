"""Dock & Stack v3: connected yaw frame, RU178X bearing envelope and retained slides. Geometry in mm; all joint values in SI.

This is the column/tray concept from the illustration, separate from the
earlier articulated-arm Concept C. Selected dimensions are parametric.
"""
from pathlib import Path
import json, math
import numpy as np
import cadquery as cq
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
P=json.loads((ROOT/'cad/parameters.json').read_text())
PACKAGE='bellybot_dock_stack_description'
PKG=ROOT/'ros2'/PACKAGE
COLORS={'teal':(.114,.620,.459),'teal_dark':(.07,.35,.28),'dark':(.12,.15,.17),
 'rubber':(.07,.085,.09),'silver':(.68,.73,.76),'light':(.87,.9,.9),
 'orange':(.96,.39,.06),'yellow':(.99,.77,.12),'red':(.85,.07,.06),
 'glass':(.05,.19,.24),'cyan':(.4,.83,.95),'white':(.96,.97,.95)}
PARTS=[];LINKS={}

def box(l,w,h,x=0,y=0,z=0):
    return cq.Workplane('XY').box(l,w,h).translate((x,y,z)).val()

def rounded(l,w,h,r,x=0,y=0,z=0):
    return cq.Workplane('XY').box(l,w,h).edges('|Z').fillet(r).translate((x,y,z)).val()

def tube(w,h,t,z):
    return box(w,w,h,z=z).cut(box(w-2*t,w-2*t,h+2,z=z))

def cyl(r,h,x=0,y=0,z=0,axis='z'):
    return cq.Solid.makeCylinder(r,h,cq.Vector(x,y,z),cq.Vector(*{'x':(1,0,0),'y':(0,1,0),'z':(0,0,1)}[axis]))

def ring(ro,ri,h,z=0):return cyl(ro,h,z=z).cut(cyl(ri,h+2,z=z-1))
def compound(shapes):return cq.Compound.makeCompound(shapes)
def cb(size,xyz=(0,0,0)):return dict(kind='box',size=size,xyz=xyz,rpy=(0,0,0))
def cc(radius,length,xyz=(0,0,0),rpy=(0,0,0)):return dict(kind='cylinder',radius=radius,length=length,xyz=xyz,rpy=rpy)
def walls(w,h,t,z):
    return [cb((w,t,h),(0,s*(w-t)/2,z)) for s in [-1,1]]+[
      cb((t,w-2*t,h),(s*(w-t)/2,0,z)) for s in [-1,1]]

def link(name,parent=None,joint=None,kind='fixed',origin=(0,0,0),axis=(0,0,1),
         lower=0,upper=0,velocity=0,effort=0,mass=0,collisions=None,group='robot'):
    LINKS[name]=dict(parent=parent,joint=joint,kind=kind,origin=list(origin),axis=list(axis),
        lower=lower,upper=upper,velocity=velocity,effort=effort,mass=mass,
        collisions=collisions or [],group=group)

def add(link,name,shape,color='silver'):
    if not shape.isValid() or shape.Volume()<=0:raise ValueError('Invalid CAD solid '+name)
    PARTS.append(dict(link=link,name=name,shape=shape,color=color))

def stadium(flat,width,r,thickness,xc=0,yc=0,zc=0):
    outer=box(flat,width,2*r,xc,yc,zc)
    inner=box(flat,width+2,2*(r-thickness),xc,yc,zc)
    for x in [-flat/2,flat/2]:
        outer=outer.fuse(cyl(r,width,xc+x,yc-width/2,zc,'y'))
        inner=inner.fuse(cyl(r-thickness,width+2,xc+x,yc-width/2-1,zc,'y'))
    return outer.cut(inner)

def track_lugs(flat,width,r):
    """Radial tread pads follow the closed stadium; no fake extra wheels."""
    pads=[]
    for x in np.linspace(-flat/2+18,flat/2-18,24):
        pads.append(box(27,width-8,4,float(x),0,r+2))
    for end in [-1,1]:
        for a in np.linspace(-75,75,6):
            angle=math.radians(a);xx=end*(flat/2+(r+2)*math.cos(angle));zz=(r+2)*math.sin(angle)
            pad=box(4,width-8,22).rotate((0,0,0),(0,1,0),-end*a).translate((xx,0,zz))
            pads.append(pad)
    return compound(pads)

def boltheads(points,r=4,h=3):return compound([cyl(r,h,x,y,z) for x,y,z in points])

def xz_plate(points,width,y):
    return cq.Workplane('XZ').polyline(points).close().extrude(width).translate((0,y+width/2,0)).val()

def yz_rail(points,length,x=0,side=1):
    return cq.Workplane('YZ').polyline([(side*y,z) for y,z in points]).close().extrude(length).translate((x-length/2,0,0)).val()

def drilled(shape,points,r,z,h):
    for x,y in points:shape=shape.cut(cyl(r,h,x,y,z))
    return shape

def build_yaw_carrier(previous):
    """Real bearing mount envelope; continuous cranked load path below moving ties.
    RU178X dimensions and bolt circles are catalog-based. Internal raceways,
    preload, threads, gearhead and motor internals are not supplier production CAD.
    """
    outer_pts=[(108.5*math.cos(a),108.5*math.sin(a)) for a in np.linspace(0,2*math.pi,12,endpoint=False)]
    inner_pts=[(69.5*math.cos(a),69.5*math.sin(a)) for a in np.linspace(0,2*math.pi,12,endpoint=False)]
    plate=rounded(330,330,12,18,z=396).cut(cyl(14,14,z=389))
    plate=drilled(plate,outer_pts,3.4,389,14)
    add(previous,'mast_bearing_mount_plate',plate,'dark')
    LINKS[previous]['collisions'] += [cb((330,151,12),(0,y,396)) for y in [-89.5,89.5]]
    LINKS[previous]['collisions'] += [cb((151,28,12),(x,0,396)) for x in [-89.5,89.5]]
    # Outer race is stationary; inner race rotates on the same vertical axis.
    outer=ring(120,90,28,402)
    outer=drilled(outer,outer_pts,4.5,401,30)
    outer=drilled(outer,outer_pts,7,421.4,9)
    add(previous,'RU178X_outer_race_envelope',outer,'silver')
    add(previous,'outer_race_mount_screws',compound([cyl(4,30.4,x,y,391).fuse(cyl(6.5,8,x,y,421.4)) for x,y in outer_pts]),'dark')
    # Coaxial servo/reduction-drive packaging; shaft passes through the plate bore.
    add(previous,'yaw_gearhead_mount_flange',ring(60,14,10,380),'silver')
    add(previous,'yaw_gearhead_housing',cyl(38,55,z=325).cut(cyl(13,57,z=324)),'dark')
    add(previous,'yaw_servo_motor_envelope',cyl(36,80,z=245),'dark')
    add(previous,'yaw_motor_end_cap',cyl(38,8,z=237),'teal_dark')
    link('yaw_carrier',previous,'yaw_joint','revolute',(0,0,P['yaw_origin_z']),
         lower=-math.radians(P['yaw_limit_deg']),upper=math.radians(P['yaw_limit_deg']),
         velocity=P['yaw_velocity'],effort=300,mass=16,
         collisions=[cc(190,10,(0,0,34))])
    inner=drilled(ring(89.5,57.5,28,0),inner_pts,3.4,-1,30)
    add('yaw_carrier','RU178X_inner_race_envelope',inner,'dark')
    adapter=drilled(ring(84,57.5,1,28),inner_pts,4.5,27,3)
    add('yaw_carrier','inner_race_adapter_shim',adapter,'silver')
    flange=drilled(cyl(190,10,z=29),inner_pts,4.5,28,12)
    flange=drilled(flange,inner_pts,7,30.4,9)
    add('yaw_carrier','rotating_carrier_flange',flange,'teal')
    add('yaw_carrier','inner_race_mount_screws',compound([cyl(4,24,x,y,8).fuse(cyl(6.5,8,x,y,30.4)) for x,y in inner_pts]),'dark')
    add('yaw_carrier','yaw_drive_output_shaft',cyl(12.5,61,z=-32),'silver')
    # Four continuous cranked ribs stay outside the square mast corner below z=402.
    profile=[(145,29),(280,29),(290,19),(290,-26),(260,-26),(250,-16),(250,13),(145,13)]
    for side in [-1,1]:
        for y in [-90,90]:
            rib=xz_plate([(side*x,z) for x,z in profile],16,y)
            add('yaw_carrier',f'carrier_cranked_rib_{side}_{y}',rib,'teal_dark')
            LINKS['yaw_carrier']['collisions'] += [cb((135,16,16),(side*212.5,y,21)),cb((40,16,45),(side*270,y,-3.5))]
    for x in [-275,275]:
        cross=box(50,860,20,x,0,-16).cut(box(44,862,14,x,0,-16))
        add('yaw_carrier',f'carrier_lower_crossbeam_{x}',cross,'teal_dark')
        LINKS['yaw_carrier']['collisions'].append(cb((50,860,20),(x,0,-16)))
        for y in [-410,410]:
            pedestal=box(50,40,28,x,y,8)
            add('yaw_carrier',f'rail_mount_pedestal_{x}_{y}',pedestal,'teal_dark')
            LINKS['yaw_carrier']['collisions'].append(cb((50,40,28),(x,y,8)))
    for y in [-410,410]:
        rail=box(870,24,60,0,y,52).cut(box(872,18,54,0,y,52))
        add('yaw_carrier',f'fixed_extension_rail_{y}',rail,'silver')
        LINKS['yaw_carrier']['collisions'].append(cb((870,24,60),(0,y,52)))
        # Rail bolts through top and bottom faces into the pedestal; no floating heads.
        screws=[]
        for x in [-275,275]:
            for dx in [-14,14]:screws.append(cyl(3,77,x+dx,y,8).fuse(cyl(5,5,x+dx,y,82)))
        add('yaw_carrier',f'fixed_rail_mount_screws_{y}',compound(screws),'dark')

def build_slide_guides():
    """Two retained dovetail-gib interfaces; moving blocks have 0.2 mm model clearance.
    Custom guide envelopes, not a claimed supplier bearing or actuator selection.
    """
    for side in [-1,1]:
        # Fixed-to-middle guide attached to inner face of the fixed beam.
        profile=[(398,22),(393,22),(387,17),(385,17),(385,47),(387,47),(393,42),(398,42)]
        rail=yz_rail(profile,830,side=side).translate((0,0,20))
        add('yaw_carrier',f'fixed_dovetail_guide_{side}',rail,'dark')
        # Enlarged female void, opening toward the mounting beam.
        cavity=[(400,21.8),(393.1,21.8),(387.1,16.8),(384.8,16.8),(384.8,47.2),(387.1,47.2),(393.1,42.2),(400,42.2)]
        for x in [-280,-120]:
            shoe=box(54,25,44,x,side*384.5,32).cut(yz_rail(cavity,56,x,side))
            add('extension_mid',f'mid_retained_gib_block_{side}_{x}',shoe,'teal_dark')
        # Middle-to-tip guide is on the opposite beam face, keeping shoe paths separate.
        profile2=[(353,24),(348,24),(342,19),(340,19),(340,49),(342,49),(348,44),(353,44)]
        add('extension_mid',f'middle_dovetail_guide_{side}',yz_rail(profile2,830,side=side),'silver')
        cavity2=[(355,23.8),(348.1,23.8),(342.1,18.8),(339.8,18.8),(339.8,49.2),(342.1,49.2),(348.1,44.2),(355,44.2)]
        for x in [-280,-120]:
            shoe=box(54,20,44,x,side*342,34).cut(yz_rail(cavity2,56,x,side))
            add('tray_carriage',f'tip_retained_gib_block_{side}_{x}',shoe,'teal_dark')
        for x in [-315,415]:
            add('yaw_carrier',f'fixed_slide_end_stop_{side}_{x}',box(16,23,24,x,side*386.5,52),'orange')
            add('extension_mid',f'middle_slide_end_stop_{side}_{x}',box(16,18,24,x,side*344,34),'orange')

def build():
    PARTS.clear();LINKS.clear()
    L,W=P['base_length'],P['base_width'];flat=P['track_contact_length'];tw=P['track_width'];R=P['track_radius']
    # Tread pads reach -4 relative to the nominal belt plane; base is lifted 4 mm
    # in all geometry to make the actual tread bottoms the z=0 floor datum.
    track_z=R+4
    basecol=[cb((510,360,110),(x,0,110)) for x in [-385,385]]
    basecol += [cb((260,50,110),(0,y,110)) for y in [-155,155]]
    basecol += walls(240,P['column_length'],5,95+P['column_length']/2)
    for x in [-392.5,392.5]:basecol.append(cb((535,820,7),(x,0,256.5)))
    for y in [-270,270]:basecol.append(cb((250,290,7),(0,y,256.5)))
    link('base_link',mass=60,collisions=basecol)
    add('base_link','lower_chassis',rounded(1280,370,110,30,z=110).cut(box(252,252,112,z=110)),'dark')
    for x in [-427.5,-142.5,142.5,427.5]:
        add('base_link',f'crossbeam_{x}',box(35,790,34,x,0,112),'dark')
    shell=rounded(L,850,88,58,z=216).cut(rounded(L-14,836,82,51,z=211))
    shell=shell.cut(box(252,252,120,z=220))
    add('base_link','teal_body_shell',shell,'teal')
    add('base_link','mast_seal_collar',rounded(280,280,9,18,z=264.5).cut(box(246,246,12,z=264.5)),'dark')
    for x in [-420,420]:
        panel=rounded(280,490,3,18,x=x,z=261.5)
        add('base_link',f'access_lid_{x}',panel,'teal_dark')
        pts=[(x+sx*112,sy*210,263) for sx in [-1,1] for sy in [-1,1]]
        add('base_link',f'lid_bolts_{x}',boltheads(pts,4,2),'silver')
    # Flush flank vents and rub rails, front bumper, camera and lights.
    for side in [-1,1]:
        add('base_link',f'flank_rubrail_{side}',rounded(900,10,15,4,0,side*425,200),'dark')
        vents=[box(45,3,4,x,side*426,z) for x in [-440,-390,390,440] for z in [211,219,227]]
        add('base_link',f'vent_slots_{side}',compound(vents),'dark')
    add('base_link','front_bumper',rounded(32,650,30,9,662,0,190),'rubber')
    add('base_link','bumper_sensor_strip',box(3,610,5,679,0,192),'yellow')
    add('base_link','depth_camera_housing',rounded(48,138,48,8,668,0,231),'dark')
    for y in [-37,37]:
        add('base_link',f'camera_bezel_{y}',cyl(14,3,692,y,231,'x'),'silver')
        add('base_link',f'camera_lens_{y}',cyl(10,2,695,y,231,'x'),'glass')
    for y in [-250,250]:
        add('base_link',f'front_light_base_{y}',box(8,150,18,656,y,236),'dark')
        add('base_link',f'front_status_light_{y}',box(2,140,7,661,y,238),'cyan')
    add('base_link','estop_plate',box(8,60,58,-663,-250,228),'yellow')
    add('base_link','estop_button',cyl(19,16,-683,-250,232,'x'),'red')
    add('base_link','rear_charge_port',box(8,100,45,-665,180,214),'dark')
    # Exactly five rollers per side, with machined recesses, hubs and bolt circles.
    for side,sgn in [('left',1),('right',-1)]:
        ty=sgn*P['track_centres']/2;track=side+'_track_link'
        link(track,'base_link',side+'_track_mount',origin=(0,ty,track_z),mass=14,
             collisions=[cb((flat,tw,12),(0,0,-track_z+6))])
        add(track,side+'_track_belt',stadium(flat,tw,R,12),'rubber')
        add(track,side+'_continuous_floor_shoe',box(flat,tw,4,z=-R-2),'rubber')
        add(track,side+'_tread_pads',track_lugs(flat,tw,R),'dark')
        for i,x in enumerate(np.linspace(-flat/2,flat/2,P['roadwheels_per_side'])):
            n=f'{side}_wheel_{i}_link'
            link(n,'base_link',f'{side}_wheel_{i}_joint','continuous',(float(x),ty,track_z),(0,1,0),
                 velocity=10,effort=40,mass=2)
            w=tw-30;r=P['roller_radius']
            wheel=cyl(r,w,0,-w/2,0,'y')
            for face in [-1,1]:
                start=-w/2-1 if face<0 else w/2-7
                wheel=wheel.cut(cyl(r-9,8,0,start,0,'y'))
            add(n,f'{side}_wheel_{i}_rim',wheel,'silver')
            add(n,f'{side}_wheel_{i}_hub',cyl(19,w+12,0,-w/2-6,0,'y'),'dark')
            bolts=[]
            for face in [-1,1]:
                for a in range(0,360,60):
                    angle=math.radians(a);xx=36*math.cos(angle);zz=36*math.sin(angle)
                    bolts.append(cyl(4,3,xx,face*(w/2-5)-1.5,zz,'y'))
            add(n,f'{side}_wheel_{i}_bolts',compound(bolts),'dark')
            # Orange index is outside the recessed face and clearly rotates.
            add(n,f'{side}_wheel_{i}_index',compound([cyl(5,2,48,f*(w/2+1)-1,0,'y') for f in [-1,1]]),'orange')
    H=P['column_length']
    add('base_link','mast_outer',tube(240,H,5,95+H/2),'teal_dark')
    add('base_link','mast_outer_flange',box(260,260,8,z=360).cut(box(224,224,10,z=360)),'dark')
    previous='base_link'
    for i,(width,mass) in enumerate([(210,6),(180,5),(150,12)],1):
        n=f'lift_stage_{i}';bottom=95+10*i
        link(n,previous,f'lift_stage_{i}_joint','prismatic',lower=0,upper=.2,
             velocity=P['lift_stage_velocity'],effort=3000,mass=mass,
             collisions=walls(width,H,5,bottom+H/2))
        add(n,f'mast_stage_{i}',tube(width,H,5,bottom+H/2),'silver' if i<3 else 'dark')
        # Wiper collars are contained within the moving tube's external width,
        # avoiding lip collisions as the next smaller stage slides through.
        collar=box(width,width,6,z=bottom+H-3).cut(box(width-12,width-12,8,z=bottom+H-3))
        add(n,f'mast_wiper_{i}',collar,'rubber')
        # Guard ribs on exposed band remain on their own stage envelope.
        if i==1:
            ribs=[]
            for zz in np.arange(bottom+H-45,bottom+H-6,6):
                ribs.append(box(width,width,3,z=float(zz)).cut(box(width-6,width-6,5,z=float(zz))))
            add(n,'guard_rib_bands',compound(ribs),'dark')
        previous=n
    build_yaw_carrier(previous)
    # Two serial horizontal stages; both remain supported at independent limits.
    link('extension_mid','yaw_carrier','extension_mid_joint','prismatic',origin=(0,0,P['deck_rise_mm']),axis=(1,0,0),
         lower=0,upper=P['extension_stage_stroke']/1000,velocity=P['extension_stage_velocity'],effort=1400,mass=8,
         collisions=[cb((830,20,54),(0,y,34)) for y in [-363,363]])
    for y in [-363,363]:add('extension_mid',f'middle_slide_{y}',box(830,20,54,0,y,34).cut(box(832,14,48,0,y,34)),'dark')
    # Rearward tie clears the full swept length of the independently moving tip runners.
    tie=box(26,746,6,-475,0,25)
    for y in [-363,363]:tie=tie.fuse(box(80,20,6,-445,y,25))
    add('extension_mid','middle_stage_cross_tie',tie,'teal_dark')
    LINKS['extension_mid']['collisions'] += [cb((26,746,6),(-475,0,25))]+[cb((80,20,6),(-445,y,25)) for y in [-363,363]]
    link('tray_carriage','extension_mid','extension_tip_joint','prismatic',axis=(1,0,0),lower=0,upper=P['extension_stage_stroke']/1000,
         velocity=P['extension_stage_velocity'],effort=1400,mass=6,
         collisions=[cb((880,16,48),(0,y,34)) for y in [-325,325]]+[cb((24,660,6),(-455,0,39))])
    for y in [-325,325]:
        add('tray_carriage',f'outboard_pitch_runner_{y}',box(880,16,48,0,y,34).cut(box(882,12,44,0,y,34)),'silver')
        add('tray_carriage',f'pitch_bearing_pedestal_{y}',box(35,18,68,-430,y,59).cut(cyl(10,24,-430,y-12,81.5,'y')),'teal_dark')
        add('tray_carriage',f'pitch_bearing_housing_{y}',cyl(24,20,-430,y-10,81.5,'y').cut(cyl(10,22,-430,y-11,81.5,'y')),'teal')
    rear_tie=box(24,660,6,-455,0,39)
    for y in [-325,325]:rear_tie=rear_tie.fuse(box(42,16,6,-446,y,39))
    add('tray_carriage','pitch_rear_crossmember',rear_tie,'silver')
    LINKS['tray_carriage']['collisions'] += [cb((42,16,6),(-446,y,39)) for y in [-325,325]]
    add('tray_carriage','pitch_servo_envelope',rounded(100,70,58,12,-430,-475,65).cut(cyl(10,72,-430,-511,81.5,'y')),'dark')
    add('tray_carriage','pitch_motor_mount_bridge',box(60,185,6,-430,-410,100).fuse(box(60,60,3,-430,-475,95.5)),'teal_dark')
    link('tray','tray_carriage','tray_pitch_joint','revolute',origin=(-430,0,81.5),axis=(0,1,0),
         lower=0,upper=math.radians(P['tray_pitch_limit_deg']),velocity=P['tray_pitch_velocity'],effort=450,mass=10,
         collisions=[cb((900,600,2),(0,0,49))]+[cb((880,20,21),(0,y,60.5)) for y in [-290,290]])
    add('tray','thin_support_pan',rounded(900,600,2,9,z=49),'silver')
    for y in [-290,290]:
        add('tray',f'silver_side_rail_{y}',box(880,20,21,0,y,60.5),'silver')
        add('tray',f'rail_recess_{y}',box(790,2,5,0,y+11*np.sign(y),60.5),'dark')
        add('tray',f'guide_support_{y}',rounded(770,10,30,3,0,y,86),'teal_dark')
        add('tray',f'soft_side_guide_{y}',rounded(770,13,12,4,0,y,107),'rubber')
        LINKS['tray']['collisions'].append(cb((770,13,42),(0,y,92)))
    for x in [-447,447]:add('tray',f'end_plate_{x}',box(4,575,12,x,0,56),'dark')
    for y in [-290,290]:add('tray',f'cassette_hinge_ear_{y}',box(22,18,50,-430,y,75),'teal_dark')
    # Hazard tabs are outboard; they do not consume the belt's luggage clearance.
    for y in [-305,305]:
        add('tray',f'hazard_tab_{y}',box(95,6,22,330,y,61),'yellow')
        stripes=[box(8,1,22,xx,y+3.6*np.sign(y),61) for xx in [302,323,344,365]]
        add('tray',f'hazard_stripes_{y}',compound(stripes),'dark')
    link('tray_belt','tray','tray_belt_mount',mass=1,collisions=[cb((860,550,1.5),(0,0,72.25))])
    add('tray_belt','tray_belt_loop',stadium(860,550,11.5,1.5,zc=61.5),'rubber')
    # Fine top belt ribs are very shallow; real moving contact is the SDF plugin.
    add('tray_belt','tray_belt_texture',compound([box(2,538,.15,float(x),0,73.075) for x in np.linspace(-418,418,40)]),'dark')
    for end,x in [('rear',-430),('front',430)]:
        n=f'tray_{end}_roller'
        link(n,'tray',n+'_joint','continuous',(x,0,61.5),(0,1,0),velocity=165,effort=20,mass=.5)
        add(n,n+'_drum',cyl(10,548,0,-274,0,'y'),'silver')
        add(n,n+'_index',compound([cyl(2,2,6,s*275-1,0,'y') for s in [-1,1]]),'orange')
    # Passive replaceable 3 mm leading edge. Taper retains the 25 mm cassette at its root.
    wedge=cq.Workplane('XZ').polyline([(447,48),(490,48),(490,51),(447,73)]).close().extrude(550).translate((0,275,0)).val()
    add('tray','replaceable_tapered_floor_nose',wedge,'silver')
    # Inscribed stair collision approximation follows the wedge; no hidden pickup attachment.
    for i in range(8):
        xx=447+i*43/8;top=73-(i+1)*22/8
        LINKS['tray']['collisions'].append(cb((43/8,550,top-48),(xx+43/16,0,(top+48)/2)))
    # Shift the complete tray subtree into its real rear pitch-bearing frame.
    pivot=np.array([-430,0,81.5])
    for part in PARTS:
        if part['link'] in ['tray','tray_belt']:part['shape']=part['shape'].translate(tuple(-pivot))
    for n in ['tray','tray_belt']:
        for c in LINKS[n]['collisions']:c['xyz']=(np.array(c['xyz'])-pivot).tolist()
    for n in ['tray_front_roller','tray_rear_roller']:LINKS[n]['origin']=(np.array(LINKS[n]['origin'])-pivot).tolist()
    # Rotation-key stub is outside the cassette and clears its fixed bearing bore.
    add('tray','pitch_drive_stub',cyl(9,225,0,-510,0,'y'),'dark')
    add('tray','pitch_idler_stub',cyl(9,60,0,285,0,'y'),'dark')
    build_slide_guides()
    build_infeed()
    # Match the conveyor terminal to the revised 495 mm home belt height.
    for part in PARTS:
        if part['link'] in LINKS and LINKS[part['link']]['group']=='infeed' and part['link']!='infeed_base':
            part['shape']=part['shape'].translate((0,0,P['deck_rise_mm']))
    for name,data in LINKS.items():
        if data['group']=='infeed' and name!='infeed_base':
            for c in data['collisions']:c['xyz']=(np.array(c['xyz'])+np.array([0,0,P['deck_rise_mm']])).tolist()
    # Roller mesh coordinates are local to their moving axes: undo the extra shift there.
    for part in PARTS:
        if part['link'] in ['infeed_front_roller','infeed_rear_roller']:part['shape']=part['shape'].translate((0,0,-P['deck_rise_mm']))
    for n in ['infeed_front_roller','infeed_rear_roller']:LINKS[n]['origin'][2]+=P['deck_rise_mm']
    build_bag()

def build_infeed():
    # The whole terminal conveyor carriage slides on a stationary support frame.
    # At q=0.4 its nose is 25 mm behind the home tray's rear edge.
    link('infeed_base',mass=24,group='infeed',collisions=[cb((700,720,10),(-1550,0,5))])
    add('infeed_base','infeed_spreader',rounded(700,720,10,18,-1550,0,5),'dark')
    link('infeed_lift','infeed_base','infeed_lift_joint','prismatic',lower=0,upper=.25,velocity=P['infeed_lift_velocity'],effort=2000,mass=6,group='infeed')
    for x in [-1800,-1300]:
        for y in [-260,260]:
            add('infeed_base',f'infeed_leg_{x}_{y}',box(34,34,370,x,y,195).cut(box(28,28,372,x,y,195)),'silver')
            add('infeed_base',f'infeed_foot_{x}_{y}',cyl(34,8,x,y,10),'dark')
            for dy in [-15.5,15.5]:LINKS['infeed_base']['collisions'].append(cb((34,3,370),(x,y+dy,195)))
            for dx in [-15.5,15.5]:LINKS['infeed_base']['collisions'].append(cb((3,28,370),(x+dx,y,195)))
            add('infeed_lift',f'infeed_inner_leg_{x}_{y}',box(24,24,360,x,y,230),'dark')
            LINKS['infeed_lift']['collisions'].append(cb((24,24,360),(x,y,230)))
    for y in [-255,255]:
        add('infeed_lift',f'infeed_fixed_rail_{y}',box(950,40,35,-1570,y,405),'teal_dark')
        LINKS['infeed_lift']['collisions'].append(cb((950,40,35),(-1570,y,405)))
    link('infeed_carriage','infeed_lift','infeed_extension_joint','prismatic',axis=(1,0,0),lower=0,upper=.4,
         velocity=P['infeed_velocity'],effort=800,mass=8,group='infeed',
         collisions=[cb((1000,25,32),(-1370,y,449)) for y in [-296,296]])
    for y in [-296,296]:
        add('infeed_carriage',f'infeed_sideframe_{y}',box(1000,25,32,-1370,y,449),'teal')
        add('infeed_carriage',f'infeed_slide_runner_{y}',box(650,22,16,-1480,y*.86,433),'silver')
    for x in [-1700,-1200]:add('infeed_carriage',f'infeed_crossbar_{x}',box(30,590,8,x,0,434),'dark')
    link('infeed_belt','infeed_carriage','infeed_belt_mount',mass=1,group='infeed',
         collisions=[cb((960,550,2),(-1370,0,474))])
    add('infeed_belt','infeed_belt_loop',stadium(960,550,12,2,xc=-1370,zc=463),'rubber')
    for end,x in [('rear',-1850),('front',-890)]:
        n='infeed_'+end+'_roller'
        link(n,'infeed_carriage',n+'_joint','continuous',(x,0,463),(0,1,0),velocity=165,effort=20,mass=.5,group='infeed')
        add(n,n+'_drum',cyl(10,548,0,-274,0,'y'),'silver')
        add(n,n+'_index',compound([cyl(2,2,6,s*275-1,0,'y') for s in [-1,1]]),'orange')

def build_bag():
    L,W,H=P['bag_length'],P['bag_width'],P['bag_height']
    add('sample_bag','orange_bag_shell',rounded(L,W,H,28,z=H/2),'orange')
    ribs=[]
    for x in np.linspace(-L/2+65,L/2-65,9):
        ribs.append(rounded(6,W-50,1.2,2,float(x),0,H+.6))
    add('sample_bag','bag_shell_ribs',compound(ribs),'orange')
    add('sample_bag','bag_seam',rounded(L+1,W+1,3,28,z=H*.48).cut(rounded(L-3,W-3,5,26,z=H*.48)),'dark')
    add('sample_bag','bag_handle',rounded(130,15,15,5,0,-W/2-8,H*.6),'dark')
    for x in [-L/2+35,L/2-35]:
        for y in [-W/2+35,W/2-35]:
            add('sample_bag',f'bag_corner_{x}_{y}',box(32,32,18,x,y,H-8),'dark')

def transform(xyz=(0,0,0),axis=(0,0,1),angle=0):
    t=np.eye(4);t[:3,3]=xyz
    if angle:t[:3,:3]=Rotation.from_rotvec(np.array(axis)*angle).as_matrix()
    return t

def fk(q=None):
    q=q or {};ts={}
    for name,d in LINKS.items():
        t=transform(d['origin']);v=q.get(d['joint'],0)
        if d['kind']=='prismatic':t[:3,3]+=np.array(d['axis'])*1000*v
        elif d['kind'] in ['revolute','continuous']:t[:3,:3]=Rotation.from_rotvec(np.array(d['axis'])*v).as_matrix()
        ts[name]=ts[d['parent']]@t if d['parent'] else t
    ts['sample_bag']=ts['tray']@transform((430,0,-8.35))
    return ts

def located(shape,t):
    rv=Rotation.from_matrix(t[:3,:3]).as_rotvec();ang=np.linalg.norm(rv)
    if ang>1e-12:shape=shape.rotate((0,0,0),tuple(rv/ang),math.degrees(ang))
    return shape.translate(tuple(t[:3,3]))

def configuration(lift=0,yaw=0,extension=0,infeed=.4,wheels=0,pitch=0,feed_height=0):
    q={f'lift_stage_{i}_joint':lift/3 for i in [1,2,3]}
    q.update(yaw_joint=yaw,extension_mid_joint=extension/2,extension_tip_joint=extension/2,infeed_extension_joint=infeed,tray_pitch_joint=pitch,infeed_lift_joint=feed_height)
    q.update({f'{s}_wheel_{i}_joint':wheels for s in ['left','right'] for i in range(5)})
    return q

def floor_pitch(lift=0):
    # Tip underside in tray frame = (920,0,-33.5) mm; hinge = 503.5 + lift.
    # 920 sin(p) + 33.5 cos(p) = hinge height - floor clearance.
    return math.asin((503.5+lift*1000-P['floor_nose_clearance'])/math.hypot(920,33.5))-math.atan2(33.5,920)

POSES={'home':configuration(),'floor_approach':configuration(extension=.84,pitch=floor_pitch()),
       'floor_insert':configuration(extension=1.,pitch=floor_pitch()),
       'batch_receive':configuration(.10,0,0,feed_height=.10),
       'batch_reach':configuration(.10,0,.80,feed_height=.10),'placement':configuration(.1,math.radians(65),.5,infeed=.15),
       'unloaded_full_stroke':configuration(.6,math.radians(-35),0,infeed=.15),
       'full_reach':configuration(.2,math.radians(95),.6,infeed=.15)}

def allowed_lift(bag_height_m,roof_m=None,margin_m=None):
    roof_m=P['local_roof_assumption']/1000 if roof_m is None else roof_m
    margin_m=P['roof_margin']/1000 if margin_m is None else margin_m
    return min(.6,max(0,roof_m-margin_m-P['tray_surface_z']/1000-bag_height_m))

def assembly(q,bag=False,feed=True):
    a=cq.Assembly(name='BELLYBOT_Concept_C_Dock_Stack');ts=fk(q)
    for p in PARTS:
        if p['link']=='sample_bag' and not bag:continue
        if p['link'] in LINKS and LINKS[p['link']]['group']=='infeed' and not feed:continue
        a.add(located(p['shape'],ts[p['link']]),name=p['name'],color=cq.Color(*COLORS[p['color']]))
    return a

if __name__=='__main__':
    build();print(f'{len(PARTS)} part groups; {len(LINKS)} articulated links')
