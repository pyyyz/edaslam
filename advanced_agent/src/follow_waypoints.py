#!/usr/bin/env python

import threading
import rospy
import actionlib
import smach_ros
import std_msgs
import time
import tf
import random

from smach              import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg  import PoseWithCovarianceStamped, PoseArray, PointStamped, Twist, PoseStamped
from std_srvs.srv       import Empty
from math               import atan2, radians, sqrt, isnan
from nav_msgs.msg       import Odometry
from std_msgs.msg       import Bool
from sensor_msgs.msg    import LaserScan
from gazebo_msgs.msg    import ModelState
from gazebo_msgs.srv    import SetModelState
from kobuki_msgs.msg    import Sound
from nav_msgs.msg       import OccupancyGrid

#### CONSTANTS #####
WAYPOINTS_TOPIC         = '/follow_waypoints/waypoints'
#RVIZ>>> pose estimate tool was mappedto this topic instead of original topic
SET_WAYPNT_TOPIC        = '/follow_waypoints/set_waypnt'
PATH_READY_TOPIC        = '/follow_waypoints/path_ready'
PAUSE_FOLLOW_TOPIC      = '/follow_waypoints/pause'
MOVE_BASE_CANCEL_TOPIC  = '/move_base/cancel'
END_RUN_TOPIC           = '/follow_waypoints/end'
END_WAYPNT_TOPIC        = '/sm/waypnt_reached'
CONTINUOUS_RUN          = True
RUNS                    = 10   #default value in case rospy.get_param is not ready.
INITALPOSE_TOPIC        = '/initialpose'
#SM_START_TOPIC        = '/sm/start'
CANDIDATE_STATUS_TOPIC  = '/follow_waypoints/cc_status'
CANDIDATE_CELL_SELECTED         = '/follow_waypoints/cc_selected'
CANDIDATE_CELL_MAP_SELECTED     = '/follow_waypoints/cc_map'
CMD_VEL_TOPIC           = '/cmd_vel_mux/input/navi'
ODOM_TOPIC              = '/odom'
CANDIDATE_CELL_TOPIC    = '/overlaping_map/point_candidate_cell'
CANDIDATE_MAP_TOPIC     = '/map_compose'
LASER_TOPIC             = '/scan'
TB2_SOUND_TOPIC         = '/mobile_base/commands/sound'
CENTINELL               = -4
DELTA                   = 0.2
PI                      = 3.1415
SECURE_DIST             = 1.5
CCEL_INHIBIT_WIN_TIME   = 40


#### GLOBAL VARIABLES ####
waypoints           = []
reset_world         = False
copy_wypnts         = True
main_wypnts_lst     = []
runs                = RUNS
cc_rcvd             = False
yaw                 = CENTINELL
angle_grad          = 0
odom_sub            = None


#move base 
SUCCESS             = 3


def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = 'map'
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

#  INIT STATE #
class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['follow_path'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.poseArray_publisher = rospy.Publisher(WAYPOINTS_TOPIC, PoseArray, queue_size=1, latch=True)
        self.initialize_path_queue()
        #self.sm_start_pub = rospy.Publisher (SM_START_TOPIC, std_msgs.msg.Empty, queue_size=1)
        self.start_pub = rospy.Publisher (PATH_READY_TOPIC, std_msgs.msg.Empty, queue_size=1, latch=True)

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish (convert_PoseWithCovArray_to_PoseArray(waypoints))

    def execute(self, userdata):
        global waypoints
        global main_wypnts_lst
        global copy_wypnts
        global reset_world
        global RUNS
        global runs

        self.path_ready = False
        # Start thread to listen for when the path is ready (this function will end then)
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message(PATH_READY_TOPIC, std_msgs.msg.Empty)
            rospy.loginfo('Recieved path READY message')
            self.path_ready = True

        if not reset_world:
            ready_thread = threading.Thread(target=wait_for_path_ready)
            ready_thread.start()

            topic = SET_WAYPNT_TOPIC 
            rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
            rospy.loginfo("To start following waypoints: 'rostopic pub /follow_waypoints/path_ready std_msgs/Empty -1'")

            self.poseArray_publisher.publish (convert_PoseWithCovArray_to_PoseArray(waypoints))
            # Wait for published waypoints
            while not self.path_ready:
                try:
                    pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
                except rospy.ROSException as e:
                    if 'timeout exceeded' in e.message:
                        continue  # no new waypoint within timeout, looping...
                    else:
                        raise e
                rospy.loginfo("Recieved new waypoint")
                waypoints.append(pose)
                # publish waypoint queue as pose array so that you can see them in rviz, etc.
                self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
            # Path is ready! save it for next run and start FollowPath 
            if copy_wypnts:
                copy_wypnts = False
                main_wypnts_lst = list (waypoints) 
                #load parameters
                if CONTINUOUS_RUN and rospy.has_param ('/follow_waypoints/runs'):
                    RUNS = rospy.get_param ('/follow_waypoints/runs')
                    runs = RUNS
        else:
            reset_world = False
            waypoints   = list (main_wypnts_lst)
            rospy.loginfo("Reloaded waypoint")
            self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
            #send start signal
            self.start_pub.publish (std_msgs.msg.Empty ())
        return 'follow_path'



def get_cc_angle (y, x):
    return atan2 (y, x)

def atan2_to_grad (a_rad):
    if (a_rad >= 0):
       a_grad = a_rad * 180/PI
    else:
       a_grad = (2*PI + a_rad) * 180/PI
    return a_grad

class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['path_complete', 'cc_rcvd'], input_keys=['waypoints'], output_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        self.end_waypnt_pub     = rospy.Publisher (END_WAYPNT_TOPIC, std_msgs.msg.Empty, queue_size=10)
        self.tb2_sound_pub      = rospy.Publisher (TB2_SOUND_TOPIC,  Sound, queue_size=10)
        self.lock   = threading.Lock () 
        self.status = None

        #this parameter is to invert the rotation in order to compare the "real" active loc with a kind of stupid active loc
        self.negative_loc = False
        if rospy.has_param ('/overlapping_cell/negative_loc'):
            self.negative_loc = rospy.get_param ('/overlapping_cell/negative_loc')

        #this is for set a rangtime without candidate cell action
        self.inhibit_wndw = False
        if rospy.has_param ('/overlapping_cell/inhibit_wndw'):
            self.inhibit_wndw = rospy.get_param ('/overlapping_cell/inhibit_wndw')
        self.last_time = time.time ()
        #publish de candidate cell the robot is goint to align with
        self.candidate_cell_selected_pub        = rospy.Publisher (CANDIDATE_CELL_SELECTED, PointStamped, queue_size=10)
        self.candidate_cell_map_selected_pub    = rospy.Publisher (CANDIDATE_CELL_MAP_SELECTED, OccupancyGrid, queue_size=10)


    def candidate_cell_callback (self, data):
        global cc_rcvd
        global cc_point_x
        global cc_point_y
        global angle_grad
        global candidate_cell
        if (not cc_rcvd) and ((not self.inhibit_wndw) or ((self.inhibit_wndw and (time.time () - self.last_time)) > CCEL_INHIBIT_WIN_TIME)):
            self.lock.acquire ()
            cc_rcvd     = True
            self.lock.release()
            #do some calculations to get the angle
            cc_point_x = data.point.x
            cc_point_y = data.point.y
            candidate_cell = data
            #caclulate de angle to add to the actual odometry angle
            angle_rad = get_cc_angle (cc_point_y, cc_point_x)
            if (self.negative_loc == False):
                angle_grad = atan2_to_grad (angle_rad)
            else:
                angle_grad = random.randint (1, 361)
            self.last_time = time.time ()
            #send stop action to the actionLib.SimpleActionClient.
            self.client.cancel_goal ()


    def candidate_map_callback (self, data):
        global cc_map
        cc_map = data 
        #copy data to a global variable

    def execute(self, userdata):
        global waypoints
        global cc_rcvd
        global candidate_cell
        global cc_map
        # Execute waypoints each in sequence
        waypoint    = None 
        self.status = None
        #for waypoint in waypoints:
        self.lock.acquire ()
        cc_rcvd = False
        self.lock.release()
        candidate_cell_sub      = rospy.Subscriber(CANDIDATE_CELL_TOPIC, PointStamped, self.candidate_cell_callback)
        candidate_cell_map_sub  = rospy.Subscriber(CANDIDATE_MAP_TOPIC, OccupancyGrid, self.candidate_map_callback)
        while waypoints != [] and not cc_rcvd:
            #Publish next waypoint as goal
            waypoint    = waypoints.pop(0)
            goal        = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal (goal)
            #Waits for the server to finish performing the action
            self.client.wait_for_result ()
            self.status = self.client.get_state ()
            rospy.loginfo ('$$$$ STATUS OF MOVE BASE GOAL:>>' + str(self.status) + '<< $$$$')  
            #result is deprecated. Status is used instead
            #result = self.client.get_result ()
            #rospy.loginfo ('$$$$   RESULT OF MOVE BASE GOAL:>>' + str(result) + '<<   $$$$')  

            if self.status == SUCCESS:
                self.end_waypnt_pub.publish (std_msgs.msg.Empty ())
                #self.tb2_sound_pub.publish (Sound.CLEANINGSTART)
                #time.sleep (8)                                      #this is for wait the human to meassure distance to point.
                #self.tb2_sound_pub.publish (Sound.CLEANINGEND)

        candidate_cell_sub.unregister ()
        candidate_cell_map_sub.unregister ()

        if cc_rcvd:
            # readd the waypoint to the waypoints list
            if waypoint != None and self.status != None and self.status != SUCCESS:
                waypoints.insert (0, waypoint)
            self.candidate_cell_selected_pub.publish (candidate_cell)
            self.candidate_cell_map_selected_pub.publish (cc_map)
            return 'cc_rcvd'
        else:
            return 'path_complete'                #mission accomplished 


class Go_to_candidate_cell (State):
    def __init__(self):
        State.__init__(self, outcomes=['waypoints_not_empty', 'waypoints_empty'])
        self.velocity_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
        self.vel_msg = Twist()
        self.vel_msg.linear.x  = 0
        self.vel_msg.linear.y  = 0
        self.vel_msg.linear.z  = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 1.0
        #CHECK: is this start pub useful?
        self.start_pub = rospy.Publisher (PATH_READY_TOPIC, std_msgs.msg.Empty, queue_size=1, latch=True)
        ### This topic is to know when start and end going to candidate cell ###
        self.candidate_cell_status_pub = rospy.Publisher (CANDIDATE_STATUS_TOPIC, Bool, queue_size=1)

        #this parameter is to travel to the candidate cell, instead of just look at the point by rotating
        self.traveling = False 
        if rospy.has_param ('/overlapping_cell/traveling'):
            self.traveling = rospy.get_param ('/overlapping_cell/traveling')


    def odom_callback (self, data):
        global yaw
        global odom_position_x
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion (quaternion)
        yaw = euler[2]           #counterclock rotation about z axis, in rads (atan2)
        #rospy.logwarn ("odom angle = " + str (yaw))
        odom_position_x = data.pose.pose.position.x

    def laser_callback (self, data):
        global laser_mid_value
        global laser_left_value
        global laser_right_value
        ranges_size = (data.angle_max - data.angle_min) / data.angle_increment;
        mid_angle = ranges_size / 2;
        quad_angle = ranges_size / 4;
        laser_mid_value   = data.ranges [int (mid_angle)]
        laser_left_value  = data.ranges [int (mid_angle - quad_angle)]
        laser_right_value = data.ranges [int (mid_angle + quad_angle)]
        

    def execute (self, userdata):
        global cc_rcvd
        global cc_point_x
        global cc_point_y 
        global odom_sub
        global yaw  
        global angle_grad
        global odom_position_x
        global laser_mid_value
        global laser_left_value
        global laser_right_value

        yaw      = CENTINELL
        cc_status = Bool ();
        cc_status.data = True;
        laser_mid_value = 0 
        #send a start_proccessing msg
        self.candidate_cell_status_pub.publish (cc_status)
        rospy.loginfo ('Executing state GO_TO_CANDIDATE_CELL')
        odom_sub  = rospy.Subscriber (ODOM_TOPIC, Odometry, self.odom_callback)
        laser_sub = rospy.Subscriber (LASER_TOPIC, LaserScan, self.laser_callback)

        while yaw == CENTINELL:
            rospy.loginfo ('Waiting for odometry subscritpion ready...')
            time.sleep (0.1)
        odom_grad = atan2_to_grad (yaw);

        #rospy.loginfo (rospy.get_caller_id () + ' x = ' + str(cc_point_x) + ' y = ' + str(cc_point_y))
        #rospy.loginfo (rospy.get_caller_id () + ' angle_grad = ' + str (angle_grad))
        #rospy.loginfo (rospy.get_caller_id () + ' odom  = ' + str (yaw) + ' odom_grad  = ' + str (odom_grad))
        
        rospy.loginfo ('Line up with ccel...in progress')
        
        if (angle_grad > 180):
            angle_diff = angle_grad - 360 
            self.vel_msg.angular.z = -1.0
        else:   
            angle_diff = angle_grad
            self.vel_msg.angular.z = 1.0
        angle_goal = (odom_grad + angle_diff) % 360
        #angle_goal to atan 
        if (angle_goal <= 180):
            angle_goal_rad = radians (angle_goal)
        else:
            angle_goal_rad = -1 * radians (360 - angle_goal)

        while abs (angle_goal_rad - yaw) > DELTA:
            self.velocity_pub.publish (self.vel_msg)
            #rospy.loginfo (rospy.get_caller_id () + ' odom_rad  = ' + str (yaw) + ' angle_goal_rad - yaw = ' + str (angle_goal_rad - yaw))
            time.sleep (0.1)

        rospy.loginfo (' Line up with ccel..DONE')
        #rospy.loginfo (' angle_goal_rad = ' + str (angle_goal_rad))
        #rospy.loginfo (' odom           = ' + str (yaw))

        self.vel_msg.angular.z = 0

        if self.traveling == True: 
            #Navigate to the point
            rospy.loginfo ('Navigating towards ccel...in progress')
            odom_position_x_init  = odom_position_x
            #calculate distance from 0,0 to point. Sould transfer point to meters.
            MAP_RESOLUTION = 0.05
            dist_to_point = sqrt (pow (cc_point_x * MAP_RESOLUTION, 2) + pow (cc_point_y * MAP_RESOLUTION, 2))
            self.vel_msg.linear.x = 1
            #check odometry in order to stop before crash
            while (abs (odom_position_x - odom_position_x_init) < (dist_to_point / 2)) and (not isnan (laser_mid_value)) and (laser_mid_value > SECURE_DIST) and (not isnan (laser_left_value)) and (laser_left_value > SECURE_DIST) and (not isnan (laser_right_value)) and (laser_right_value > SECURE_DIST):    #-1 as a margin to avoid colision with the wall
                self.velocity_pub.publish (self.vel_msg)
                time.sleep (0.1)
                rospy.loginfo ('laser laser laser!!!! ==' + str (laser_mid_value))
                rospy.loginfo ('************* x = ' + str(cc_point_x) + ' y = ' + str(cc_point_y))
                rospy.loginfo ('odom_init = ' + str (odom_position_x_init) + '  odom_pos_x = ' + str (odom_position_x))
                rospy.loginfo ('dist_to_point -1 = ' + str (dist_to_point / 2) + '  diff = ' + str (abs (odom_position_x - odom_position_x_init)))
            self.vel_msg.linear.x = 0 
            rospy.loginfo (' ccel Reached..DONE')

        self.velocity_pub.publish (self.vel_msg)

        odom_sub.unregister ()
        laser_sub.unregister ()
        cc_status.data = False;
        #send a end_proccessing msg
        self.candidate_cell_status_pub.publish (cc_status)
        if (waypoints != []):
            return 'waypoints_not_empty'
        else:
            return 'waypoints_empty'

class PathComplete (State):
    def __init__(self):
        State.__init__(self, outcomes=['end_run', 'reset'])
        self.end_run_pub = rospy.Publisher (END_RUN_TOPIC, std_msgs.msg.Empty, queue_size=10)


    def execute(self, userdata):
        global CONTINUOUS_RUN
        global RUNS
        global runs
        runs = runs -1 
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        if CONTINUOUS_RUN:
            rospy.loginfo('##### RUN:  ' + str(RUNS - runs) + '             #####')
        if runs == 0:   
            rospy.loginfo('######   END EXPERIMENT   #####')
        rospy.loginfo('###############################')
        self.end_run_pub.publish (std_msgs.msg.Empty ())
        if CONTINUOUS_RUN and runs > 0:
            return 'reset'
        else:
            return 'end_run'


class ResetWorld (State):
    def __init__(self):
        State.__init__(self, outcomes=['end_run'])
        self.initial_pose = PoseWithCovarianceStamped ()
        self.restart_amcl_pub = rospy.Publisher (INITALPOSE_TOPIC, PoseWithCovarianceStamped, queue_size=10)

    def set_initial_pose (self):
            self.initial_pose.header.seq      = 0 
            self.initial_pose.header.stamp    = rospy.Time.now()
            self.initial_pose.header.frame_id = "map"

            self.initial_pose.pose.pose.position.x = 0.0
            self.initial_pose.pose.pose.position.y = 0.0
            self.initial_pose.pose.pose.position.z = 0.0

            self.initial_pose.pose.pose.orientation.x = 0.0
            self.initial_pose.pose.pose.orientation.y = 0.0
            self.initial_pose.pose.pose.orientation.z = 0.0
            self.initial_pose.pose.pose.orientation.w = 1 

            self.initial_pose.pose.covariance = [0.18383006637503171, 0.009122813038866376, 0.0, 0.0, 0.0, 0.0, 
                                                    0.00912281303886638, 0.22333427167689107,  0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0600141237715029] 
    def execute(self, userdata):
        global waypoints
        global reset_world
        #reset gazebo service
        rospy.wait_for_service ('gazebo/reset_world')
        reset_world_srv = rospy.ServiceProxy ('gazebo/reset_world/', Empty)
        reset_world_srv ()

        model_state_srv_msg = ModelState () 
        model_state_srv_msg.model_name      = 'mobile_base'
        model_state_srv_msg.pose.position.x = 0
        model_state_srv_msg.pose.position.y = 0
        model_state_srv_msg.pose.position.z = 0
        model_state_srv_msg.pose.orientation.x = 0 
        model_state_srv_msg.pose.orientation.y = 0 
        model_state_srv_msg.pose.orientation.z = 0 
        model_state_srv_msg.pose.orientation.w = 1

        rospy.wait_for_service ('gazebo/set_model_state')
        try:
            set_model_state_srv = rospy.ServiceProxy ('gazebo/set_model_state/', SetModelState)
            resp = set_model_state_srv (model_state_srv_msg)

        except rospy.ServiceException, e:
            rospy.logerr ('Service call failed: %s', e)


        #clearing costmap
        rospy.wait_for_service ('move_base/clear_costmaps')
        clear_costmaps_srv = rospy.ServiceProxy ('move_base/clear_costmaps', Empty)
        clear_costmaps_srv ()

        #reset amcl initial pose and cov
        self.set_initial_pose ()
        #send this msg twice is recomended in case one msg drop
        self.restart_amcl_pub.publish (self.initial_pose)
        time.sleep (1)
        self.restart_amcl_pub.publish (self.initial_pose)
        #reload waypoints list
        reset_world = True

        #clearing costmap
        clear_costmaps_srv ()
        
        return 'end_run'
   

def main():
    rospy.init_node('follow_waypoints')

    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'follow_path':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'path_complete':'PATH_COMPLETE', 'cc_rcvd':'GO_TO_CANDIDATE_CELL'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('GO_TO_CANDIDATE_CELL', Go_to_candidate_cell(), 
                               transitions={'waypoints_not_empty':'FOLLOW_PATH', 
                                            'waypoints_empty':'PATH_COMPLETE'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                transitions={'end_run':'success', 'reset':'RESET_WORLD'})
        StateMachine.add('RESET_WORLD', ResetWorld(),
                           transitions={'end_run':'GET_PATH'})

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_FLW_WPNTS')
        sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
if __name__ == '__main__':
    main()
