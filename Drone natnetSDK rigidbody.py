import time
import sys
import io
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData
import math
import drone_script

drone_pos = 0
target_pos = 0

drone_id = None
target_id = None

drone_rot = 0
target_rot = 0
distance = []



# This is a callback function that gets connected to the NatNet client
# and called once per mocap frame.
def receive_new_frame(data_dict):
    order_list=[ "rigidBodyCount"  "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
    
    dump_args = False
    if dump_args == True:
        out_string = "    "
        for key in data_dict:
            out_string += key + "="
            if key in data_dict :
                out_string += data_dict[key] + " "
            out_string+="/"
            print(out_string)


#List for XYZ kordinater 1.Drone og 2.Target
rigid_body_1_data = []
rigid_body_2_data = []

rigid_body_1_rotdata = []
rigid_body_2_rotdata = []

# Initialize previous positions with None
prev_drone_pos = None
prev_target_pos = None

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(new_id, position, rotation):
    global drone_pos, target_pos, drone_id, target_id, drone_rot, target_rot, name, prev_drone_pos, prev_target_pos, dron_var, targ_var, rotatjon
    name = new_id
    
    if new_id == 1:
        target_pos = position
        target_id = new_id
        target_rot = rotation

        # Save the information for Rigid body 1 in the list
        rigid_body_1_data.append([round(coord * 100, 0) for coord in position[:3]])

       #numbers for quternion equation. Henter ut X,Y,Z,W
        rigid_body_1_rotdata.append([round(coord * 100, 0) for coord in target_rot[:4]])
     
        

    elif new_id == 2:
        drone_pos = position
        drone_id = new_id
        drone_rot = rotation

        # Save the information for Rigid body 2 in the list
        rigid_body_2_data.append([round(coord * 100, 0) for coord in position[:3]])
  
        
#Euler formula calcultions        
def euler_from_quaternion(x, y, z, w):

        global yaw
        global pitch
        global yaw
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        yaw = yaw_z * (180 / math.pi)
        pitch = pitch_y * (180 / math.pi)
        roll = roll_x * (180 / math.pi)
        
        return roll, pitch, yaw 
    

def add_lists(totals, totals_tmp):
    totals[0]+=totals_tmp[0]
    totals[1]+=totals_tmp[1]
    totals[2]+=totals_tmp[2]
    return totals

def print_configuration(natnet_client):
    print("Connection Configuration:")
    print("  Client:          %s"% natnet_client.local_ip_address)
    print("  Server:          %s"% natnet_client.server_ip_address)
    print("  Command Port:    %d"% natnet_client.command_port)
    print("  Data Port:       %d"% natnet_client.data_port)

    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s"% natnet_client.multicast_address)
    else:
        print("  Using Unicast")

    #NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()
"""
    print("  NatNet Server Info")
    print("    Application Name %s" %(application_name))
    print("    NatNetVersion  %d %d %d %d"% (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
    print("    ServerVersion  %d %d %d %d"% (server_version[0], server_version[1], server_version[2], server_version[3]))
    print("  NatNet Bitstream Requested")
    print("    NatNetVersion  %d %d %d %d"% (nat_net_requested_version[0], nat_net_requested_version[1],\
       nat_net_requested_version[2], nat_net_requested_version[3]))
    #print("command_socket = %s"%(str(natnet_client.command_socket)))
    #print("data_socket    = %s"%(str(natnet_client.data_socket)))
"""

def print_commands(can_change_bitstream):
    outstring = "\nCommands:\n"
    outstring += "How to operate the Tello-drone:\n"
    outstring += " \n 's'  Start's the Tello-Drone takeoff (wait for takeoff status to say ok before giving more commands(give the drone ca.5sec))"
    outstring += " \n 'r'  To rotate the Tello-Drone's pitch to follow the target \n(only press once after takeoff)"
    outstring += " \n 'x'  To move the drone to target and get coordinates (Only press 'x' once to move to target, press again if target has changed position)"
    outstring += " \n 'q'  To shutdown the drone and stop the streamingclient \n"
    print(outstring)

def request_data_descriptions(s_client):
    # Request the model definitions
    s_client.send_request(s_client.command_socket, s_client.NAT_REQUEST_MODELDEF,    "",  (s_client.server_ip_address, s_client.command_port) )

def test_classes():
    return

def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict

#To filter out some unwanted messages from the stream
class FilteredStdout:
    def __init__(self, original_stdout):
        self.original_stdout = original_stdout
        self.suppress = False

    def write(self, message):
        if 'Begin Packet' in message:
            self.suppress = True
        if not self.suppress:
            self.original_stdout.write(message)
        if 'End Packet' in message:
            self.suppress = False
            # Optional: to maintain formatting
            self.original_stdout.write("\n")  

    def flush(self):
        self.original_stdout.flush()

# Save a reference to the original standard output
original_stdout = sys.stdout

# Replace sys.stdout with your filter
sys.stdout = FilteredStdout(original_stdout)

if __name__ == "__main__":

    optionsDict = {}
    optionsDict["clientAddress"] = "192.168.12.69"
    optionsDict["serverAddress"] = "192.168.12.77"
    optionsDict["use_multicast"] = True

    # This will create a new NatNet client
    optionsDict = my_parse_args(sys.argv, optionsDict)

    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.new_frame_listener = receive_new_frame
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    is_looping = True
    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on. \n (Make sure you have the correct ip writen inn the script).")
        try:
            sys.exit(2)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    print_configuration(streaming_client)
    print("\n")
    print_commands(streaming_client.can_change_bitstream_version())


    while is_looping:
        inchars = input('Enter command or (\'h\' for list of commands)\n' "Remember to have the correct ip typed in drone startup under 's' key \n" "Remember to press 'r' to orient the drone before using move command 'x' for the first time!\n")
        if len(inchars)>0:
            c1 = inchars[0].lower()
            if c1 == 'h' :
                print_commands(streaming_client.can_change_bitstream_version())
                
            elif c1 == 'x' :
                #streaming_client.shutdown()
                
                index = len(rigid_body_1_data) - 1
                index2 = len(rigid_body_2_data) -1
                
                targ_var = (rigid_body_2_data[index2:])
                dron_var = (rigid_body_1_data[index:])
                targ_var = targ_var[0]
                dron_var = dron_var[0]
                
                x_move = targ_var[0] - dron_var[0]
                y_move = targ_var[1] - dron_var[1]
                z_move = targ_var[2] - dron_var[2]
                z_move = -z_move

                
                drone_script.move(x_move, z_move, 0)
                
                rotatjon = euler_from_quaternion(target_rot[0], target_rot[1], target_rot[2], target_rot[3])
                
                print(" drone:", dron_var , "\n","target:", targ_var)
                print("\n", "Tello-drone travel vector:" , x_move, y_move , z_move)
                
                print("\n"," Yaw:"  , pitch)
                print('Tello-drone moving to target')
            
                
            elif c1 == 'r':
                index = len(rigid_body_1_data) - 1
                index2 = len(rigid_body_2_data) -1
                
                targ_var = (rigid_body_2_data[index2:])
                dron_var = (rigid_body_1_data[index:])
                targ_var = targ_var[0]
                dron_var = dron_var[0]
                
                x_move = targ_var[0] - dron_var[0]
                y_move = targ_var[1] - dron_var[1]
                z_move = targ_var[2] - dron_var[2]
                z_move = -z_move

                
                rotatjon = euler_from_quaternion(target_rot[0], target_rot[1], target_rot[2], target_rot[3])
                
                print(" drone:", dron_var , "\n","target:", targ_var)
                print("\n", "TEST:" , x_move, y_move , z_move)
                
                print("\n"," Yaw:"  , pitch)
                print("Tello-Drone rotating to target")
                drone_script.rotate(pitch)
                
                
            elif c1 == 's':
                drone_script.start("192.168.10.3")
                print('Tello-drone taking off!(Pleas give the drone 5sec to settle before entering commands)')

            elif c1 == 'q':
                is_looping = False
                drone_script.stop()
                drone_script.stopUpdatingPos()
                streaming_client.shutdown()
                break
            else:
                print("Error: Command %s not recognized"%c1)
            print("Ready...\n")
    print("exiting (reset consol to start again)")

