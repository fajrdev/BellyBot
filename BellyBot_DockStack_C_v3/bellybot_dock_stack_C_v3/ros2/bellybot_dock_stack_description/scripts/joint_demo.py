#!/usr/bin/env python3
"""RViz-only CSV playback. Runs once then holds; no reset discontinuity."""
import csv
from bisect import bisect_right
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

class Demo(Node):
    def __init__(self):
        super().__init__('dock_stack_joint_demo')
        p=Path(get_package_share_directory('bellybot_dock_stack_description'))
        with (p/'config/joint_demo.csv').open() as f:
            rows=csv.reader(f);self.names=next(rows)[1:];self.rows=[list(map(float,r)) for r in rows]
        self.ts=[r[0] for r in self.rows];self.start=self.get_clock().now()
        self.pub=self.create_publisher(JointState,'joint_states',10);self.create_timer(1/60,self.tick)
    def tick(self):
        now=self.get_clock().now();t=min(max(0,(now-self.start).nanoseconds/1e9),self.ts[-1])
        i=max(0,min(bisect_right(self.ts,t)-1,len(self.rows)-2));a,b=self.rows[i:i+2];u=(t-a[0])/(b[0]-a[0])
        msg=JointState();msg.header.stamp=now.to_msg();msg.name=self.names
        msg.position=[x+(y-x)*u for x,y in zip(a[1:],b[1:])];self.pub.publish(msg)

def main():
    rclpy.init();n=Demo()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
