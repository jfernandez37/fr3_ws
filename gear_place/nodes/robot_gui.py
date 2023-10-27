#!/usr/bin/env python3

import tkinter as tk
from functools import partial
import rclpy
import os

CAMERA_TYPES = ["depth","color"]
COMMAND_TYPES = ["open_gripper",
                 "cartesian_movement",
                 "scanning","pick_up_single_gear",
                 "pick_up_multiple_gears",
                 "put_down_gear","moving_gears",
                 "move_to_named_pose",
                 "enable_conveyor",
                 "disable_conveyor",
                 "move_conveyor",
                 "sleep"]
SCAN_TYPES = ["single", "grid"]
STARTING_POSITIONS = ["current","home","high_scan","rotate_scan_1","rotate_scan_2","above_conveyor","position_1","position_2"]
PUT_DOWN_TYPES = ["force", "camera", "value"]
CARTESIAN_TYPES = ["standard","angle","smooth"]
MOVEMENT_TYPES = ["pick_up","above"]
CONVEYOR_DIRECTIONS = ["forward","backward"]
class FR3_GUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.selected_commands = []
        self.title('FR3 control gui')
        self.resizable(width=False, height=False)
        self.grid_columnconfigure(0, weight=1)

        self.parameters = {}

        # Get the current screen width and height
        self.screen_width = self.winfo_screenwidth()
        self.screen_height = self.winfo_screenheight()
        

        self.geometry('800x750')

        self.save_all_button = tk.Button(self, text="Save all", command=self.destroy)
        self.save_all_button.pack(pady=5, side=tk.BOTTOM)
        self.add_new_command_button = tk.Button(self,text="Add command", command=self.add_command)
        self.add_new_command_button.pack(pady=5, side=tk.BOTTOM)
        
        self.current_widgets = [self.add_new_command_button,self.save_all_button]
        
        # tk command type variable
        self.command_type = tk.StringVar()
        self.command_type.set(COMMAND_TYPES[0])

        self.parameters["command_type"] = tk.StringVar()

        # tk cartesian parameters
        self.parameters["type"] = tk.StringVar()
        self.parameters["type"].set(CARTESIAN_TYPES[0])

        self.parameters["x"] = tk.StringVar()
        self.parameters["x"].set("0.0")

        self.parameters["y"] = tk.StringVar()
        self.parameters["y"].set("0.0")
        
        self.parameters["z"] = tk.StringVar()
        self.parameters["z"].set("0.0")

        self.parameters["v_max"] = tk.StringVar()
        self.parameters["v_max"].set("0.0")

        self.parameters["acc"] = tk.StringVar()
        self.parameters["acc"].set("0.0")
        
        self.parameters["angle"] = tk.StringVar()
        self.parameters["angle"].set("0.0")

        # tk scanning parameters
        self.parameters["scan_type"] = tk.StringVar()
        self.parameters["scan_type"].set(SCAN_TYPES[0])

        self.parameters["robot_moves"] = tk.StringVar()
        self.parameters["robot_moves"].set("")

        # tk pick up gear parameters
        self.parameters["depth_or_color"] = tk.StringVar()
        self.parameters["depth_or_color"].set(CAMERA_TYPES[0])

        self.parameters["object_width"] = tk.StringVar()
        self.parameters["object_width"].set("0.0095")

        self.parameters["starting_position"] = tk.StringVar()
        self.parameters["starting_position"].set(STARTING_POSITIONS[0])

        self.parameters["yellow"] = tk.StringVar()
        self.parameters["yellow"].set("0")

        self.parameters["red"] = tk.StringVar()
        self.parameters["red"].set("0")

        self.parameters["green"] = tk.StringVar()
        self.parameters["green"].set("0")

        self.parameters["put_down_type"] = tk.StringVar()
        self.parameters["put_down_type"].set(PUT_DOWN_TYPES[0])

        self.parameters["force"] = tk.StringVar()
        self.parameters["force"].set("0.0")

        self.parameters["put_down_pose"] = tk.StringVar()
        self.parameters["put_down_pose"].set(STARTING_POSITIONS[0])

        # tk moving gear parameter
        self.parameters["movement_type"] = tk.StringVar()
        self.parameters["movement_type"].set(MOVEMENT_TYPES[0])

        # tk named pose parameter
        self.parameters["name_pose"] = tk.StringVar()
        self.parameters["name_pose"].set(STARTING_POSITIONS[1])

        # tk conveyor parameters
        self.parameters["conveyor_speed"] = tk.StringVar()
        self.parameters["conveyor_speed"].set(0.0)

        self.parameters["conveyor_direction"] = tk.StringVar()
        self.parameters["conveyor_direction"].set(CONVEYOR_DIRECTIONS[0])

        # tk sleep parameter
        self.parameters["duration"] = tk.StringVar()
        self.parameters["duration"].set(0.0)

    def add_command(self):
        self.clear_window()
        self.command_type_menu = tk.OptionMenu(self,self.command_type, *COMMAND_TYPES)
        self.command_type_menu.pack(pady=5, side=tk.TOP)
        self.save_button = tk.Button(self,text="Save command",command = self.save_command)
        self.save_button.pack(pady=5, side=tk.BOTTOM)
        self.current_widgets.append(self.command_type_menu)
        self.current_widgets.append(self.save_button)
        self.command_type.trace('w', self.show_correct_menu)

    def save_command(self):
        self.parameters["command_type"].set(self.command_type.get())
        self.selected_commands.append({key:self.parameters[key].get()  for key in self.parameters.keys()})
        self.reset_parameters()
        self.clear_window()
        self.save_all_button.pack(pady=5, side=tk.BOTTOM)
        self.add_new_command_button.pack(pady=5, side=tk.BOTTOM)
        self.current_widgets.append(self.add_new_command_button)
        self.current_widgets.append(self.save_all_button)
    
    def clear_window(self):
        for widget in self.current_widgets:
            widget.pack_forget()
        self.current_widgets.clear()
    
    def reset_parameters(self):
        self.command_type.set(COMMAND_TYPES[0])
        self.parameters["type"].set(CARTESIAN_TYPES[0])
        self.parameters["x"].set("0.0")
        self.parameters["y"].set("0.0")
        self.parameters["z"].set("0.0")
        self.parameters["v_max"].set("0.0")
        self.parameters["acc"].set("0.0")
        self.parameters["angle"].set("0.0")
        self.parameters["scan_type"].set(SCAN_TYPES[0])
        self.parameters["robot_moves"].set("")
        self.parameters["depth_or_color"].set(CAMERA_TYPES[0])
        self.parameters["object_width"].set("0.0095")
        self.parameters["starting_position"].set(STARTING_POSITIONS[0])
        self.parameters["yellow"].set("0")
        self.parameters["red"].set("0")
        self.parameters["green"].set("0")
        self.parameters["put_down_type"].set(PUT_DOWN_TYPES[0])
        self.parameters["force"].set("0.0")
        self.parameters["put_down_pose"].set(STARTING_POSITIONS[0])
        self.parameters["movement_type"].set(MOVEMENT_TYPES[0])
        self.parameters["name_pose"].set(STARTING_POSITIONS[1])
        self.parameters["conveyor_speed"].set(0.0)
        self.parameters["conveyor_direction"].set(CONVEYOR_DIRECTIONS[0])
        self.parameters["duration"].set(0.0)

    def show_correct_menu(self,_,__,___):
        self.clear_window()
        self.command_type_menu.pack(pady=5, side=tk.TOP)
        self.save_button.pack(pady=5, side=tk.BOTTOM)
        self.current_widgets.append(self.command_type_menu)
        self.current_widgets.append(self.save_button)
        if self.command_type.get()=="cartesian_movement":
            self.show_cartesian_menu()
        elif self.command_type.get()=="scanning":
            self.show_scanning_menu()
        elif self.command_type.get()=="pick_up_single_gear":
            self.single_gear_pick_menu()
        elif self.command_type.get()=="pick_up_multiple_gears":
            self.multiple_gears_pick_menu()
        elif self.command_type.get()=="put_down_gear":
            self.show_put_down_menu()
        elif self.command_type.get()=="moving_gears":
            self.show_moving_gears_menu()
        elif self.command_type.get()=="move_to_named_pose":
            self.show_named_pose_menu()
        elif self.command_type.get()=="move_conveyor":
            self.show_move_conveyor_menu()
        elif self.command_type.get()=="sleep":
            self.show_sleep_menu()
    
    def show_cartesian_menu(self):
        cartesian_type_label = tk.Label(self, text="Select the type of cartesian movement")
        cartesian_type_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_type_label)
        cartesian_type_menu = tk.OptionMenu(self,self.parameters["type"],*CARTESIAN_TYPES)
        cartesian_type_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_type_menu)

        cartesian_x_label = tk.Label(self, text="Enter the x value for the cartesian movement:")
        cartesian_x_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_x_label)
        cartesian_x_entry = tk.Entry(self,textvariable=self.parameters["x"])
        cartesian_x_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_x_entry)

        cartesian_y_label = tk.Label(self, text="Enter the y value for the cartesian movement:")
        cartesian_y_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_y_label)
        cartesian_y_entry = tk.Entry(self,textvariable=self.parameters["y"])
        cartesian_y_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_y_entry)

        cartesian_z_label = tk.Label(self, text="Enter the z value for the cartesian movement:")
        cartesian_z_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_z_label)
        cartesian_z_entry = tk.Entry(self,textvariable=self.parameters["z"])
        cartesian_z_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_z_entry)

        cartesian_acc_label = tk.Label(self, text="Enter the acceleration value for the cartesian movement:")
        cartesian_acc_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_acc_label)
        cartesian_acc_entry = tk.Entry(self,textvariable=self.parameters["acc"])
        cartesian_acc_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_acc_entry)

        cartesian_v_max_label = tk.Label(self, text="Enter the y value for the cartesian movement:")
        cartesian_v_max_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_v_max_label)
        cartesian_v_max_entry = tk.Entry(self,textvariable=self.parameters["v_max"])
        cartesian_v_max_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_v_max_entry)

        cartesian_angle_label = tk.Label(self, text="Enter the angle value for the cartesian movement:")
        cartesian_angle_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_angle_label)
        cartesian_angle_entry = tk.Entry(self,textvariable=self.parameters["angle"],state=tk.DISABLED)
        cartesian_angle_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(cartesian_angle_entry)

        angle_entry_enabled = partial(self.enable_disable_angle_entry,cartesian_angle_entry)
        self.parameters["type"].trace('w',angle_entry_enabled)

    def show_scanning_menu(self):
        scanning_type_label = tk.Label(self, text="Select the type scan")
        scanning_type_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(scanning_type_label)
        scanning_type_menu = tk.OptionMenu(self,self.parameters["scan_type"],*SCAN_TYPES)
        scanning_type_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(scanning_type_menu)

        robot_moves_label = tk.Label(self, text="Please enter the points for the grid search. Enter them as [x,y] seperated by a comma. Leave blank to do default 3x3 scan.")
        robot_moves_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(robot_moves_label)
        robot_moves_entry = tk.Entry(self, textvariable=self.parameters["robot_moves"],state=tk.DISABLED)
        robot_moves_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(robot_moves_entry)

        robot_moves_entry_enabled = partial(self.enable_disable_robot_moves_entry,robot_moves_entry)
        self.parameters["scan_type"].trace('w',robot_moves_entry_enabled)

    def single_gear_pick_menu(self):
        depth_or_color_label = tk.Label(self, text="Select whether the scan should be done using the depth or color image")
        depth_or_color_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(depth_or_color_label)
        depth_or_color_menu = tk.OptionMenu(self,self.parameters["depth_or_color"],*CAMERA_TYPES)
        depth_or_color_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(depth_or_color_menu)

        object_width_label = tk.Label(self, text="Please enter the object_width")
        object_width_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(object_width_label)
        object_width_entry = tk.Entry(self, textvariable=self.parameters["object_width"])
        object_width_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(object_width_entry)

        starting_position_label = tk.Label(self, text="Please choose the named position that the robot should pick up the gear")
        starting_position_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(starting_position_label)
        starting_position_menu = tk.OptionMenu(self,self.parameters["starting_position"],*STARTING_POSITIONS)
        starting_position_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(starting_position_menu)
    
    def multiple_gears_pick_menu(self):
        object_width_label = tk.Label(self, text="IMPORTANT NOTE: SCANNING MUST BE DONE BEFORE THIS OR IT WILL NOT HAVE GEARS TO PICK UP.\n\nPlease enter the object_width")
        object_width_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(object_width_label)
        object_width_entry = tk.Entry(self, textvariable=self.parameters["object_width"])
        object_width_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(object_width_entry)

        starting_position_label = tk.Label(self, text="Please choose the starting position. Choose current to use the current position")
        starting_position_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(starting_position_label)
        starting_position_menu = tk.OptionMenu(self,self.parameters["starting_position"],*STARTING_POSITIONS)
        starting_position_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(starting_position_menu)

        color_label = tk.Label(self,text="Select the colors that you would like the robot to pick up")
        color_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(color_label)
        yellow_button = tk.Checkbutton(self, text="Yellow", variable=self.parameters["yellow"], onvalue="1", offvalue="0", height=1, width=20)
        yellow_button.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(yellow_button)
        red_button = tk.Checkbutton(self, text="Red", variable=self.parameters["red"], onvalue="1", offvalue="0", height=1, width=20)
        red_button.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(red_button)
        green_button = tk.Checkbutton(self, text="Green", variable=self.parameters["green"], onvalue="1", offvalue="0", height=1, width=20)
        green_button.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(green_button)
        

        depth_or_color_label = tk.Label(self, text="Select whether the second check should be done using the depth or color image")
        depth_or_color_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(depth_or_color_label)
        depth_or_color_menu = tk.OptionMenu(self,self.parameters["depth_or_color"],*CAMERA_TYPES)
        depth_or_color_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(depth_or_color_menu)

        put_down_type_label = tk.Label(self,text="Choose the method to put the gear down.")
        put_down_type_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(put_down_type_label)
        put_down_type_menu =tk.OptionMenu(self,self.parameters["put_down_type"],*PUT_DOWN_TYPES)
        put_down_type_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(put_down_type_menu)

        force_label = tk.Label(self, text="Please enter the put down force")
        force_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(force_label)
        force_entry = tk.Entry(self, textvariable=self.parameters["force"])
        force_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(force_entry)

        put_down_pose_label = tk.Label(self, text="Please choose the starting position. Choose current to use the current position")
        put_down_pose_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(put_down_pose_label)
        put_down_pose_menu = tk.OptionMenu(self,self.parameters["put_down_pose"],*STARTING_POSITIONS)
        put_down_pose_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(put_down_pose_menu)

        force_entry_enabled = partial(self.enable_disable_force_entry,force_entry)
        self.parameters["put_down_type"].trace('w',force_entry_enabled)

    def show_put_down_menu(self):
        put_down_type_label = tk.Label(self,text="Choose the method to put the gear down.")
        put_down_type_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(put_down_type_label)
        put_down_type_menu =tk.OptionMenu(self,self.parameters["put_down_type"],*PUT_DOWN_TYPES)
        put_down_type_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(put_down_type_menu)

        z_label = tk.Label(self, text="Enter the z value down movement. For the table below the FR3, use -0.247.")
        z_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(z_label)
        z_entry = tk.Entry(self,textvariable=self.parameters["z"])
        z_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(z_entry)

        force_label = tk.Label(self, text="Please enter the put down force.")
        force_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(force_label)
        force_entry = tk.Entry(self, textvariable=self.parameters["force"])
        force_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(force_entry)

        update_menu = partial(self.update_put_down_menu,z_entry,force_entry)
        self.parameters["put_down_type"].trace('w',update_menu)

    def show_moving_gears_menu(self):
        movement_type_label = tk.Label(self, text="Select the type of movement")
        movement_type_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(movement_type_label)
        movement_type_menu = tk.OptionMenu(self,self.parameters["movement_type"],*MOVEMENT_TYPES)
        movement_type_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(movement_type_menu)

        object_width_label = tk.Label(self, text="Please enter the object_width")
        object_width_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(object_width_label)
        object_width_entry = tk.Entry(self, textvariable=self.parameters["object_width"])
        object_width_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(object_width_entry)

        object_width_entry_update = partial(self.update_movement_menu,object_width_entry)
        self.parameters["movement_type"].trace('w',object_width_entry_update)

    def show_named_pose_menu(self):
        name_pose_label = tk.Label(self, text="Please choose the named pose that you would like the robot to move to")
        name_pose_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(name_pose_label)
        name_pose_menu = tk.OptionMenu(self,self.parameters["name_pose"],*STARTING_POSITIONS[1:])
        name_pose_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(name_pose_menu)

    def show_move_conveyor_menu(self):
        conveyor_speed_label = tk.Label(self, text="Please enter the conveyor speed")
        conveyor_speed_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(conveyor_speed_label)
        conveyor_speed_entry = tk.Entry(self, textvariable=self.parameters["conveyor_speed"])
        conveyor_speed_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(conveyor_speed_entry)

        conveyor_direction_label = tk.Label(self,text="Select the direction for the conveyor belt")
        conveyor_direction_label.pack(pady=5,side=tk.TOP)
        self.current_widgets.append(conveyor_direction_label)
        conveyor_direction_menu = tk.OptionMenu(self, self.parameters["conveyor_direction"],*CONVEYOR_DIRECTIONS)
        conveyor_direction_menu.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(conveyor_direction_menu)

    def show_sleep_menu(self):
        sleep_label = tk.Label(self, text="Please enter duration for sleep")
        sleep_label.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(sleep_label)
        sleep_entry = tk.Entry(self, textvariable=self.parameters["duration"])
        sleep_entry.pack(pady=5, side=tk.TOP)
        self.current_widgets.append(sleep_entry)

    def enable_disable_angle_entry(self, entry_box,_,__,___):
        self.parameters["angle"].set("0.0")
        if self.parameters["type"].get()=="angle":
            entry_box["state"]=tk.NORMAL
        else:
            entry_box["state"]=tk.DISABLED
    
    def enable_disable_robot_moves_entry(self, entry_box,_,__,___):
        self.parameters["robot_moves"].set("")
        if self.parameters["scan_type"].get()=="grid":
            entry_box["state"]=tk.NORMAL
        else:
            entry_box["state"]=tk.DISABLED
    
    def enable_disable_force_entry(self, entry_box,_,__,___):
        self.parameters["force"].set("0.0")
        if self.parameters["put_down_type"].get()=="force":
            entry_box["state"]=tk.NORMAL
        else:
            entry_box["state"]=tk.DISABLED

    def update_put_down_menu(self, z_box, force_box,_,__,___):
        self.parameters["z"].set("-0.247")
        self.parameters["force"].set("0.0")
        if self.parameters["put_down_type"].get()=="force":
            force_box["state"]=tk.NORMAL
            z_box["state"]=tk.DISABLED
        else:
            force_box["state"]=tk.DISABLED
            z_box["state"]=tk.NORMAL
    
    def update_movement_menu(self, entry_box,_,__,___):
        self.parameters["object_width"].set("0.0095")
        if self.parameters["movement_type"].get()=="pick_up":
            entry_box["state"]=tk.NORMAL
        else:
            entry_box["state"]=tk.DISABLED



def main(args=None):
    app = FR3_GUI()
    app.mainloop()
    main_node = open(os.getcwd()+"/src/gear_place/gear_place/nodes/gear_place_node.py",'w')
    main_node.write("#!/usr/bin/env python3\n\nimport rclpy\n\nfrom gear_place.gear_place_classes import GearPlace, Error\n\nfrom time import sleep\n\nfrom math import pi\n\n\ndef main(args=None):"+
                    "\n\trclpy.init(args=args)\n\ttry:\n\t\tsupervisor = GearPlace()\n\t\tsupervisor.wait(5)")
    for command in app.selected_commands:
        if command["command_type"]=="open_gripper":
            main_node.write("\n\t\tsupervisor.call_open_gripper_service()")
        elif command["command_type"]=="cartesian_movement":
            if command["type"]=="standard":
                main_node.write(f"\n\t\tsupervisor.call_move_cartesian_service({command['x']},{command['y']},{command['z']},{command['v_max']},{command['acc']})")
            elif command["type"]=="angle":
                main_node.write(f"\n\t\tsupervisor.call_move_cartesian_angle_service({command['x']},{command['y']},{command['z']},{command['v_max']},{command['acc']},{command['angle']})")
            else:
                main_node.write(f"\n\t\tsupervisor.call_move_cartesian_smooth_service({command['x']},{command['y']},{command['z']},{command['v_max']},{command['acc']})")
        elif command["command_type"]=="scanning":
            main_node.write(f"\n\t\tdistances_from_home,updated_radius_vals = supervisor.select_scan(type_scan=\"{command['scan_type']}\""+("" if command['robot_moves']=="" or command['scan_type']=="single" else f",[\"{command['robot_moves']}\"]")+")")
        elif command["command_type"]=="pick_up_single_gear":
            main_node.write(f"\n\t\tsupervisor.call_pick_up_gear_service(\"{command['depth_or_color']}\",{command['object_width']},\"{command['starting_position']}\")")
        elif command["command_type"]=="pick_up_multiple_gears":
            comma_needed=False
            color_list = "["
            if command["yellow"]=='1':
                color_list+="yellow"
                comma_needed=True
            if command["red"]=='1':
                if comma_needed:
                    color_list+=","
                color_list+="red"
                comma_needed=True
            if command["green"]=='1':
                if comma_needed:
                    color_list+=","
                color_list+="green"
            color_list+="]"
            main_node.write(f"\n\t\tsupervisor.pick_up_multiple_gears(distances_from_home,updated_radius_vals,{command['object_width']},\"{command['starting_position']}\",\"{color_list}\",\"{command['depth_or_color']}\",\"{command['put_down_type']}\",{command['force']},\"{command['put_down_pose']}\")")
        elif command["command_type"]=="put_down_gear":
            main_node.write(f"\n\t\tsupervisor.put_gear_down_choose_type(\"{command['put_down_type']}\",{command['z']},{command['force']})")
        elif command["command_type"]=="moving_gears":
            if command["movement_type"]=="pick_up:":
                main_node.write(f"\n\t\tsupervisor.call_move_up_moving_gear_service({command['object_width']})")
            else:
                main_node.write("\n\t\tsupervisor.call_move_above_gear()")
        elif command["command_type"]=="move_to_named_pose":
            main_node.write(f"\n\t\tsupervisor.call_move_to_named_pose_service(\"{command['name_pose']}\")")
        elif command["command_type"]=="enable_conveyor":
            main_node.write(f"\n\t\tsupervisor.enable_conveyor_service(True)")
        elif command["command_type"]=="disable_conveyor":
            main_node.write(f"\n\t\tsupervisor.enable_conveyor_service(False)")
        elif command["command_type"]=="move_conveyor":
            main_node.write(f"\n\t\tsupervisor.set_conveyor_state_service({command['conveyor_speed']},{CONVEYOR_DIRECTIONS.index(command['conveyor_direction'])})")
        elif command["command_type"]=="sleep":
            main_node.write(f"\n\t\tsleep({command['duration']})")
    main_node.write("\n\texcept Error as e:\n\n\t\tprint(e)\n\nif __name__ == \"__main__\":\n\tmain()")
    main_node.close()
    os.system("cd ~/fr3_ws")
    os.system("colcon build")
    os.system("source install/setup.bash")
    os.system("ros2 launch gear_place gear.launch.py")


if __name__ == '__main__':
    main()