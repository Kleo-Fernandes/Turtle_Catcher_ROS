#!usr/bin/env python3
import rclpy 
from rclpy.node import  Node 
from turtlesim.srv import Spawn
import random
import math
from my_robot_interfaces.msg import Turtle 
from my_robot_interfaces.msg import TurtleArray
from functools import partial
from my_robot_interfaces.srv import KillTurtle
from turtlesim.srv import Kill

#in rqt_graph the spawner basically makes use of the cmd_vel to send data to the simulator to display the turtle 
class turtle_spawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner_node") 
        self.turtle_name_prefix_="turtle_name"
        self.turtle_counter_=0
        self.Alive_Turtles=[]
        self.spawn_client_=self.create_client(Spawn,"/spawn")   # the server can perform the same function multiple times 
        self.spawn_turtle_timer=self.create_timer(2.0,self.spawn_new_turtle)
        
        self.pose_publisher_=self.create_publisher(TurtleArray,"alive_Turtles",10)

        self.kill_server_service=self.create_service(KillTurtle,"kill_Turtle",self.KillTurtle_service_callback)
        self.kill_client=self.create_client(Kill,"/kill")


#turtle data
    def spawn_new_turtle(self):
        self.turtle_counter_+=1
        name=self.turtle_name_prefix_ + str(self.turtle_counter_)
        x=random.uniform(0.0,11.0)  # gives a random float value between the range
        y=random.uniform(0.0,11.0)
        theta=random.uniform(0.0,2*math.pi)
        self.call_back_spawn(name,x,y,theta)

    def Turtle_list_publisher(self):
        msg=TurtleArray()
        msg.turtles=self.Alive_Turtles
        self.pose_publisher_.publish(msg)


#the server forward the request of the kill to the /kill service server , this you could do if you want to add additional function like boolean , if you tthink about it 
    def KillTurtle_service_callback(self,request:KillTurtle.Request, response:KillTurtle.Response):
        self.call_kill_service(request.name)  #killTurle has name in interface 
        response.success =True
        return response

    


#we dont import x, y and z using self as parallel operations become diifficult as variable values will be constantly updated
    def call_back_spawn(self, turtle_name, x, y, theta):
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("waiting for add 2 ints server...")
        
        request=Spawn.Request()
        request.x=x
        request.y=y
        request.theta=theta
        request.name=turtle_name

        future=self.spawn_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call,request=request) )  #this method takes add done callback as an argument for a callback done confirmation so i can crete a method that will b e activate when callback is done and that method is passed in arguments
#add done callback also expect a method that takes future as an argument , partial increases the argument passed for the function in argument
    def callback_call(self,future,request):
        response=future.result()
        if response.name!="":
            self.get_logger().info("Got response "+ response.name)   #the service only gives the reponse as name other parameters like x,y theta are not given so we take it from request 
            new_turtle=Turtle()
            new_turtle.name=response.name
            new_turtle.x=request.x
            new_turtle.y=request.y
            new_turtle.theta=request.theta
            self.Alive_Turtles.append(new_turtle)
            self.Turtle_list_publisher()

    def call_kill_service(self,new_turtle):
        while not self.kill_client.wait_for_service(1.0):
            self.get_logger().warn("waiting for kill server...")
        
        request=Kill.Request()
        request.name=new_turtle
        future=self.kill_client.call_async(request)
        future.add_done_callback(partial(self.kill_client_done_callback,request=request))


    def kill_client_done_callback(self,future,request:Kill.Request):
        for (i,turtle) in enumerate(self.Alive_Turtles):
            if turtle.name==request.name:
                self.Alive_Turtles.pop(i)
                self.Turtle_list_publisher()
                break
def main():
    rclpy.init()
    node=turtle_spawner()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__=="__main__":
    main()