#!/usr/bin/env python3
"""Finite, repeated Gazebo belt command with a zero command on exit."""
import argparse,subprocess,time
p=argparse.ArgumentParser();p.add_argument('--speed',type=float,default=.15);p.add_argument('--seconds',type=float,default=3.0);a=p.parse_args()
if not -.5<=a.speed<=.5 or not 0<a.seconds<=120:p.error('Speed must be within +/-0.5 m/s and duration within (0,120] s.')
def send(v):subprocess.run(['gz','topic','-t','/environment/loader/speed','-m','gz.msgs.Double','-p',f'data: {v}'],check=True,timeout=2)
try:
 t=time.monotonic()
 while time.monotonic()-t<a.seconds:send(a.speed);time.sleep(.1)
finally:send(0)
