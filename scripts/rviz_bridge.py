#!/usr/bin/env python

import roslib
import rospy 
import tf
from sensor_msgs.msg import JointState, Imu, RelativeHumidity
from ecn_bluerov.msg import ADC
from geometry_msgs.msg import WrenchStamped

class Listener:
    def __init__(self):
        
        self.imu = Imu()
        self.imu_received = False
        self.imu_sub = rospy.Subscriber('lsm', Imu, self.ImuCallBack) 
        
        self.thruster_received = False
        self.thruster_sub = rospy.Subscriber('thruster_command', JointState, self.ThrusterCallBack)
        
        self.leak = 0
        self.leak_sub = rospy.Subscriber('tensions', ADC, self.ADCCallBack)
        
    def ImuCallBack(self, msg): 
        self.imu_received = True
        self.imu = msg

    def ThrusterCallBack(self, msg): 
        self.thruster_received = True
        self.thruster = msg
        
    def ADCCallBack(self, msg): 
        self.leak = msg.leak

if __name__ == '__main__':
    
    rospy.init_node('odom_to_tf')   
    
    br = tf.TransformBroadcaster()
                
    listener = Listener()
    
    # wait for thruster to initialize wrench dimension
    while not rospy.is_shutdown():   
        if listener.thruster_received:
            n_th = len(listener.thruster.name)
            break
    use_position = (len(listener.thruster.name) == len(listener.thruster.position))
        
    wrench_pub = [rospy.Publisher(name + '_wrench', WrenchStamped, queue_size=1) for name in listener.thruster.name]
    wrench = [WrenchStamped() for name in listener.thruster.name]
    for i,w in enumerate(wrench):
        w.header.frame_id = listener.thruster.name[i]
        
    rh_pub = rospy.Publisher('leak', RelativeHumidity, queue_size=1)
    rh = RelativeHumidity()
    rh.header.frame_id = 'leak_link'

    T =1./50
    ratio = 1./5
    while not rospy.is_shutdown():        
        if listener.imu_received:            
            q = listener.imu.orientation                        
            #br.sendTransform((t.x, t.y, t.z), (q.x,q.y,q.z,q.w), rospy.Time.now(), listener.odom.child_frame_id, listener.odom.header.frame_id)
            br.sendTransform((0,0,0,0), (-q.x,-q.y,-q.z,q.w), rospy.Time.now(), 'base_link_R', 'lsm')
            
        if listener.thruster_received:
            for i,w in enumerate(wrench):
                w.header.stamp = rospy.Time.now()
                if use_position:
                    w.wrench.force.z = listener.thruster.position[i]*ratio
                else:
                    w.wrench.force.z = listener.thruster.effort[i]*ratio                     
                wrench_pub[i].publish(w)
                
        rh.relative_humidity = listener.leak/5000.
        rh.header.stamp = rospy.Time.now()
        rh_pub.publish(rh)
        
            
            
        rospy.sleep(T)
