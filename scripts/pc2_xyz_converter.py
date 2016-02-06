#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospkg

from subprocess import call

import sys, select, termios, tty, time

from PyQt4.QtCore import QThread

msg = """
Reading from the keyboard for scan capture!
---------------------------
p - output scan topic
a - toggle capturing data
s - save data points
c - clear points
   
CTRL-C to quit
"""

cmdBindings = {
        'p',
        'a',
        's',
        'c'}

print msg


# def getKey():
#     tty.setraw(sys.stdin.fileno())
#     select.select([sys.stdin], [], [], 1)
#     key = sys.stdin.read(1)
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key



# # Qt - Worker Thread to get keyboard input in nonblocking call
class KeyboardWorkerThread(QThread):
    def __init__(self, name, sleep, parent = None):
        QThread.__init__(self, parent)
        self.sleep = sleep
        self.name  = name
        self.c = None
        self.old_settings = termios.tcgetattr(sys.stdin)

    def isData(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def run(self):    
        self.old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            while True:

                if self.isData():
                    c = sys.stdin.read(1)
                    if c == '\x1b':         # x1b is ESC
                        print "Stopped looking at kb input."
                        break
                    else:
                        self.c = c
                        print c

        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)    

    def get_c(self):
        return self.c
    
    def pop_c(self):
        c = self.c
        self.c = None
        return c

 


class PC2_XYZ():
    def __init__(self, pc2_topic, xyz_output_file):
        self.is_running = True # this is init set to true, and only set to false when the kb thread is not running as detected by the update

        self.xyz_output_file = xyz_output_file
        self.pc2_topic = pc2_topic
        self.sub = rospy.Subscriber(self.pc2_topic, PointCloud2, self.pc_cb) 

        self.points = []
        self.add_points = False
        self.convert_points = False     

        # self.get_command() # this works, lets try a threaded example...

        self.kb_thread = KeyboardWorkerThread("kb_thread", .3)
        rospy.on_shutdown(self.shutdown)
        self.kb_thread.start()
        
    # def get_char(self):
    #     return self.kb_thread.pop_c()

    # def isData(self):
    #     return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    # def get_command(self):

    #     self.old_settings = termios.tcgetattr(sys.stdin)
    #     try:
    #         tty.setcbreak(sys.stdin.fileno())

    #         while True:

    #             if self.isData():
    #                 c = sys.stdin.read(1)
    #                 if c == '\x1b':         # x1b is ESC
    #                     print "Stopped looking at kb input."
    #                     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings) 
    #                     break
    #                 else:
    #                     print c

    #     finally:
    #             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)    

    def update_state(self):
        # self.cmd = self.kb_thread.pop_c()
        if not self.kb_thread.isRunning():
            rospy.signal_shutdown("KB Thread Killed")
            self.is_running = False
        else:
            self.cmd = self.kb_thread.pop_c()

            if self.cmd == 'a':
                self.add_points = not self.add_points # toggle adding points
                print "Toggling adding points: Add points? ", self.add_points
            elif self.cmd == 's':
                self.convert_points = True
                print "Convert points"
                self.convert_points_to_mesh()
            elif self.cmd == 'c':
                self.points = []
                print "Clearing points"
            elif self.cmd == 'p':
                print "Scan topic subscribed to: ", self.pc2_topic 

    def convert_points_to_mesh(self):
        print "Reading points"
  
        f = open(self.xyz_output_file, 'w')
        
        for scan in self.points:
            for xyz in scan:

                to_write = '%(1)f %(2)f %(3)f\n' % {'1':xyz[0], '2':xyz[1], '3':xyz[2]}
                f.write(to_write)
            
        f.close()

        self.xyz_2_mesh(self.xyz_output_file)
        
        self.convert_points = False

    def pc_cb(self, data):
        # if self.convert_points:
            # print "Reading points"
            # gen = pc2.read_points(data, field_names=("x", "y", "z"))
            # f = open(self.xyz_output_file, 'w')
            
            # for xyz in gen:

            #     to_write = '%(1)f %(2)f %(3)f\n' % {'1':xyz[0], '2':xyz[1], '3':xyz[2]}
            #     f.write(to_write)
                
            # f.close()

            # self.xyz_2_mesh(self.xyz_output_file)
            
            # self.convert_points = False
        
        if self.add_points:
            self.points.append(pc2.read_points(data, field_names=("x","y","z")))


    def xyz_2_mesh(self, input_file):
        print
        print "calling meshlabserver"

        rospack = rospkg.RosPack()

        dir_path = rospack.get_path("pcl_mesh_converter")
        filter_path = dir_path + "/filters/xyz_to_mesh.mlx"
        print "PCL mesh converter filter path: ", filter_path

        # input_file = dir_path + input_file

        call(["/usr/bin/meshlabserver", "-i", input_file, "-o", "points_mesh.ply", "-s", filter_path])

        call(["/usr/bin/meshlabserver", "-i", "points_mesh.ply", "-o", "points_mesh.xyz"])

    ### threaded way to do it
    def shutdown(self):
        old_settings = self.kb_thread.old_settings

        self.kb_thread.quit()
        self.kb_thread.terminate()
        
        print "terminated termios and restored settings"
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

   

if __name__ == '__main__':

    # start getting keyboard input
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('points_to_xyz', anonymous=False)

    pc2_topic = rospy.get_param("~pointcloud2_topic")
    intermediate_file = rospy.get_param("~intermediate_file")
    
    pc2_xyz = PC2_XYZ(pc2_topic, intermediate_file)

    # rospy.spin(), not needed because of while loop
    while(pc2_xyz.is_running):
        pc2_xyz.update_state()
        time.sleep(0.1)

    # terminate getting keyboard input
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
