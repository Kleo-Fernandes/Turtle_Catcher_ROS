#!usr/bin/env python3
import math
import rclpy 
from rclpy.node import  Node 
from turtlesim.msg import Pose 
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.srv import KillTurtle
from turtlesim.srv import Kill
from functools import partial

class turtle_sim_node(Node):
    def __init__(self):
        super().__init__("Turtle_controller")
        self.pose_: Pose = None
        self.target_turtle:Turtle=None

        self.target_pose_subscription=self.create_subscription(TurtleArray,"alive_Turtles",self.subscriber_callback_alive_turtle,10)
        self.cmd_publisher=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.pose_subscriber=self.create_subscription(Pose,"turtle1/pose",self.subscriber_callback,10)
    
        self.timer_callback=self.create_timer(0.01,self.control_loop) # this is the control loop callback where we will send the commads at 100 hz or 0.01 sec command will be sent to help with correct soordinates and updating etc
         
        self.catch_turtle_client=self.create_client(KillTurtle,"kill_Turtle")
    def subscriber_callback(self,msg:Pose):
        self.pose_=msg

    def subscriber_callback_alive_turtle(self,msg:TurtleArray):#we pass msg.Turles as usually to access a component we use like pose.x , so always access the component no exceptioomn for this
        if len(msg.turtles)>0:
            self.target_turtle=msg.turtles[0]
        

    def call_catch_turtle_Service(self,turtle_name):
        while not self.catch_turtle_client.wait_for_service(1.0):
            self.get_logger().warn("waiting for catch turtle service  server...")
        
        request=KillTurtle.Request()
        request.name=turtle_name
        future=self.catch_turtle_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle,turtle_name=turtle_name))


    def callback_call_catch_turtle(self,future,turtle_name):
        response:KillTurtle.Response=future.result()
        if not response.success:
            self.get_logger().error("The Turtle could not be removed"+turtle_name )
    def control_loop(self):
        
        if self.pose_==None or self.target_turtle==None:
            return 
        

        dist_x=self.target_turtle.x -self.pose_.x
        dist_y=self.target_turtle.y -self.pose_.y
        distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)
        cmd = Twist()
        
        if distance>0.5:
            #position
            cmd.linear.x = 2*distance    #multiplied by 2 for better performance based on experience

            #orientation
            goal_theta=math.atan2(dist_y,dist_x)
            diff=goal_theta-self.pose_.theta

            if diff>math.pi:     #normalization
                diff-=2*math.pi
            elif diff<-math.pi:
                diff+=2*math.pi

            cmd.angular.z=6*diff  #multiplied by 6 for better performance based on experience
        

            
        else:
            #target reached
            cmd.linear.x=0.0
            cmd.angular.z=0.0
            self.call_catch_turtle_Service(self.target_turtle.name)
            self.target_turtle=None #update so that control stops 
        self.cmd_publisher.publish(cmd)


def main():
    rclpy.init()
    node=turtle_sim_node()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__=="__main__":
    main()