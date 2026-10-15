"""Feedback-driven, one-bag commissioning sequence. No rigid attachments.
Inputs are measured joint positions and measured bag world poses (not commanded).
Commissioning speed prioritizes contact verification; 12 bags/min is not claimed.
"""
import math
P=math.pi
class Transfer:
 def __init__(self):self.state='IDLE';self.started=0;self.h=0;self.error='';self.output={}
 def start(self,t,arm):
  if self.state not in ('IDLE','DONE'):raise ValueError('Transfer already active or faulted')
  if abs(arm[1])>.005 or arm[2]>.005 or arm[3]>.005 or not -.005<=arm[0]<=.13:raise ValueError('Receive requires level, centred, retracted tray and lift <=130 mm')
  self.h=max(0.,arm[0]);self.state='PARK';self.started=t
 def fail(self,reason):self.state='FAULT';self.error=reason
 def step(self,t,q,bag=None):
  # Home commands; actuator targets only advance after measured feedback agrees.
  out=dict(corner_lift=0.,corner_angle=0.,buffer_lift=0.,bridge_angle=-P/2,loader=0.,door=0.,corner=0.,bridge=0.,buffer=0.,tray=0.)
  if self.state in ('FAULT','IDLE','DONE'):return out
  if t-self.started>60:self.fail('Phase timeout');return out
  def change(s):self.state=s;self.started=t
  def close(k,v,tol=.004):return abs(q[k]-v)<tol
  H=.198+self.h;B=.130+self.h
  if bag is not None and self.state!='PARK':
   surface=.300+q['corner_lift'] if self.state in ('RECEIVE','SETTLE','LIFT','ROTATE') else .498+self.h
   # All supplied models are 260..380 mm high. This deliberately loose centre-Z
   # band detects a fallen/unsupported bag without mistaking model size for perception.
   if not surface+.10<=bag.get('z',float('nan'))<=surface+.25:self.fail('Bag is not supported at the expected conveyor height');return out
  if self.state in ('LIFT','ROTATE','RETURN','LOWER') and not close('bridge_angle',-P/2,.025):self.fail('Bridge unfolded during indexed motion');return out
  if self.state in ('ROTATE','RETURN') and q['corner_lift']<.13:self.fail('Rotary deck too low to clear incoming belt');return out
  if self.state in ('BRIDGE','TRANSFER','TRAY_SETTLE') and (not close('corner_angle',P/2,.02) or not close('corner_lift',H,.025) or not close('buffer_lift',B,.025)):
   print(f"[LOST ALIGNMENT] corner_angle={q['corner_angle']:.4f} (err {abs(q['corner_angle']-P/2):.4f}), corner_lift={q['corner_lift']:.4f} (err {abs(q['corner_lift']-H):.4f}), buffer_lift={q['buffer_lift']:.4f} (err {abs(q['buffer_lift']-B):.4f})", flush=True)
   self.fail('Transfer surfaces lost alignment');return out
  if self.state not in ('PARK',) and bag is None:self.fail('No fresh tracked bag pose');return out
  if self.state=='PARK':
   # Fold the nose first. At startup the neutral URDF bridge is horizontal.
   out.update(corner_lift=q['corner_lift'],corner_angle=q['corner_angle'],buffer_lift=q['buffer_lift'])
   if close('bridge_angle',-P/2,.02):
    out.update(corner_angle=0.)
    if close('corner_angle',0,.015):out.update(corner_lift=0.,buffer_lift=0.)
    if close('corner_angle',0,.015) and close('corner_lift',0) and close('buffer_lift',0) and bag is not None:change('RECEIVE')
  elif self.state=='RECEIVE':
   if abs(bag['x']-1.3)>.025 or abs(math.atan2(math.sin(bag['yaw']-P/2),math.cos(bag['yaw']-P/2)))>math.radians(3):self.fail('Bag is not aligned with door belt');return out
   out.update(loader=.25,door=.25,corner=.25)
   if bag['y']>=-.015:out.update(loader=0.,door=0.,corner=0.);change('SETTLE')
  elif self.state=='SETTLE':
   if abs(bag['y'])>.065:self.fail('Bag did not stop inside the rotary deck');return out
   if bag['speed']<.02 and t-self.started>.35:change('LIFT')
  elif self.state=='LIFT':
   out.update(corner_lift=H,buffer_lift=B)
   if int(t*5)%10==0:print(f"[CORE LIFT] corner={q['corner_lift']:.4f}/{H:.4f} (err {abs(q['corner_lift']-H):.4f}), buffer={q['buffer_lift']:.4f}/{B:.4f} (err {abs(q['buffer_lift']-B):.4f})", flush=True)
   if close('corner_lift',H,.025) and close('buffer_lift',B,.025):change('ROTATE')
  elif self.state=='ROTATE':
   out.update(corner_lift=H,corner_angle=P/2,buffer_lift=B)
   if math.hypot(bag['x']-1.3,bag['y'])>.07:self.fail('Bag slipped during rotation');return out
   if close('corner_angle',P/2,.01) and bag['speed']<.03:change('BRIDGE')
  elif self.state=='BRIDGE':
   out.update(corner_lift=H,corner_angle=P/2,buffer_lift=B,bridge_angle=0.)
   if abs(bag['yaw'])>math.radians(4):self.fail('Bag did not rotate with the table');return out
   if close('bridge_angle',0,.015):change('TRANSFER')
  elif self.state=='TRANSFER':
   out.update(corner_lift=H,corner_angle=P/2,buffer_lift=B,bridge_angle=0.,corner=.25,bridge=.25,buffer=.25,tray=.25)
   if abs(bag['y'])>.035:self.fail('Bag drifted off the conveyor centreline');return out
   if bag['x']>=4.09:out.update(corner=0.,bridge=0.,buffer=0.,tray=0.);change('TRAY_SETTLE')
  elif self.state=='TRAY_SETTLE':
   out.update(corner_lift=H,corner_angle=P/2,buffer_lift=B,bridge_angle=0.)
   if bag['x']>4.18:self.fail('Bag overshot the receiving tray');return out
   if bag['speed']<.02 and t-self.started>.35:change('FOLD')
  elif self.state=='FOLD':
   out.update(corner_lift=H,corner_angle=P/2,buffer_lift=B)
   if close('bridge_angle',-P/2,.015):change('RETURN')
  elif self.state=='RETURN':
   out.update(corner_lift=H,corner_angle=0.,buffer_lift=0.)
   if close('corner_angle',0,.01):change('LOWER')
  elif self.state=='LOWER':
   if close('corner_lift',0,.015) and close('buffer_lift',0,.015):change('DONE')
  return out
