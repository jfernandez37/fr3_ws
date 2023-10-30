#!/usr/bin/env python3

import tkinter as tk
from functools import partial
import rclpy
import os
from gear_place.gui_utils import (
    decimal_val,
    validate_rotation_value
)
from math import pi

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
                 "rotate_single_joint",
                 "sleep"]
SCAN_TYPES = ["single", "grid"]
STARTING_POSITIONS = ["current","home","high_scan","rotate_scan_1","rotate_scan_2","above_conveyor","position_1","position_2"]
PUT_DOWN_TYPES = ["force", "camera", "value"]
CARTESIAN_TYPES = ["standard","angle","smooth"]
MOVEMENT_TYPES = ["pick_up","above"]
CONVEYOR_DIRECTIONS = ["forward","backward"]
JOINT_INDICES = [str(i) for i in range(7)]
ANGLE_TYPES = ["radians","degree"]
class FR3_GUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.selected_commands = []
        self.title('FR3 control gui')
        self.resizable(width=False, height=False)
        self.grid_columnconfigure(0, weight=1)

        # cancel flag
        self.cancel_flag = tk.StringVar()
        self.cancel_flag.set('0')

        # command counter
        self.command_counter = tk.StringVar()
        self.command_counter.set('0')
        self.command_counter.trace('w',self.update_label_and_remove_button)

        # label with all current commands
        self.selected_command_label = tk.Label(self,text="Current code:\n\n")
        self.selected_command_label.pack(pady=10, side=tk.TOP)

        self.parameters = {}

        # Get the current screen width and height
        self.screen_width = self.winfo_screenwidth()
        self.screen_height = self.winfo_screenheight()
        

        self.geometry('1000x750')

        self.cancel_button = tk.Button(self, text="Cancel", command = self.cancel_function)
        self.cancel_button.pack(pady=5, side=tk.BOTTOM)
        self.save_all_button = tk.Button(self, text="Save all", command=self.destroy)
        self.save_all_button.pack(pady=5, side=tk.BOTTOM)
        self.remove_command_button = tk.Button(self,text="Remove command", command=self.remove_command,state=tk.DISABLED)
        self.remove_command_button.pack(pady=5,side=tk.BOTTOM)
        self.add_new_command_button = tk.Button(self,text="Add command", command=self.add_command)
        self.add_new_command_button.pack(pady=5, side=tk.BOTTOM)
        
        self.current_widgets = [self.add_new_command_button,self.save_all_button, self.cancel_button,self.selected_command_label,self.remove_command_button]
        
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
        self.parameters["v_max"].set("0.15")

        self.parameters["acc"] = tk.StringVar()
        self.parameters["acc"].set("0.2")
        
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
        self.parameters["starting_position"].set("home")

        self.parameters["yellow"] = tk.StringVar()
        self.parameters["yellow"].set("0")

        self.parameters["orange"] = tk.StringVar()
        self.parameters["orange"].set("0")

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

        # tk rotate single joint parameters
        self.parameters['joint_index'] = tk.StringVar()
        self.parameters['joint_index'].set(JOINT_INDICES[0])

        self.parameters['joint_angle'] = tk.StringVar()
        self.parameters['joint_angle'].set(0.0)

        self.parameters["angle_type"] = tk.StringVar()
        self.parameters["angle_type"].set(ANGLE_TYPES[0])

        # tk sleep parameter
        self.parameters["duration"] = tk.StringVar()
        self.parameters["duration"].set(0.0)

        #validation functions
        validate_x = partial(decimal_val,self.parameters["x"])
        self.parameters["x"].trace('w',validate_x)

        validate_y = partial(decimal_val,self.parameters["y"])
        self.parameters["y"].trace('w',validate_y)

        validate_z = partial(decimal_val,self.parameters["z"])
        self.parameters["z"].trace('w',validate_z)

        validate_v_max = partial(decimal_val,self.parameters["v_max"])
        self.parameters["v_max"].trace('w',validate_v_max)

        validate_acc = partial(decimal_val,self.parameters["acc"])
        self.parameters["acc"].trace('w',validate_acc)

        validate_object_width = partial(decimal_val, self.parameters["object_width"])
        self.parameters["object_width"].trace('w',validate_object_width)

        validate_conveyor_speed = partial(decimal_val,self.parameters["conveyor_speed"])
        self.parameters["conveyor_speed"].trace('w',validate_conveyor_speed)

        validate_force = partial(decimal_val,self.parameters["force"])
        self.parameters["force"].trace('w',validate_force)

        validate_duration = partial(decimal_val, self.parameters["duration"])
        self.parameters["duration"].trace('w',validate_duration)
    
    def pack_and_append(self, widget):
        widget.pack(pady=5,side=tk.TOP)
        self.current_widgets.append(widget)

    def cancel_function(self):  # cancels at any point in the program
        self.cancel_flag.set('1')
        self.destroy()

    def add_command(self):
        self.clear_window()
        self.command_type_menu = tk.OptionMenu(self,self.command_type, *COMMAND_TYPES)
        self.command_type_menu.pack(pady=5, side=tk.TOP)
        self.back_button = tk.Button(self,text="Back",command = self.back_command)
        self.back_button.pack(pady=5,side=tk.BOTTOM)
        self.save_button = tk.Button(self,text="Save command",command = self.save_command)
        self.save_button.pack(pady=5, side=tk.BOTTOM)
        self.current_widgets.append(self.command_type_menu)
        self.current_widgets.append(self.save_button)
        self.current_widgets.append(self.back_button)
        self.command_type.trace('w', self.show_correct_menu)

    def save_command(self):
        self.parameters["command_type"].set(self.command_type.get())
        for key in ["angle","joint_angle"]:
            if self.parameters[key].get()!="0.0":
                try:
                    self.parameters[key].set(str(float(self.parameters[key])))
                except:
                    if self.parameters[key].get()=="pi":
                        self.parameters[key].set(str(round(pi,5)))
                    else:
                        multiplier = -1 if "-" in self.parameters[key].get() else 1
                        values = self.parameters[key].get().split("/")
                        try:
                            val_1 = float(values[0])
                        except:
                            val_1 = pi
                        try:
                            val_2 = float(values[1])
                        except:
                            val_2 = pi
                        self.parameters[key].set(str(round(val_1/val_2,5)*multiplier))
        self.selected_commands.append({key:self.parameters[key].get()  for key in self.parameters.keys()})
        self.command_counter.set(str(int(len(self.selected_commands))))
        self.reset_parameters(True)
        self.clear_window()
        self.cancel_button.pack(pady=5, side=tk.BOTTOM)
        self.save_all_button.pack(pady=5, side=tk.BOTTOM)
        self.remove_command_button.pack(pady=5,side=tk.BOTTOM)
        self.add_new_command_button.pack(pady=5, side=tk.BOTTOM)
        self.selected_command_label.pack(pady=10, side=tk.TOP)
        self.current_widgets.append(self.add_new_command_button)
        self.current_widgets.append(self.save_all_button)
        self.current_widgets.append(self.remove_command_button)
        self.current_widgets.append(self.cancel_button)
        self.current_widgets.append(self.selected_command_label)
    
    def back_command(self):
        self.reset_parameters(True)
        self.clear_window()
        self.cancel_button.pack(pady=5, side=tk.BOTTOM)
        self.save_all_button.pack(pady=5, side=tk.BOTTOM)
        self.remove_command_button.pack(pady=5,side=tk.BOTTOM)
        self.add_new_command_button.pack(pady=5, side=tk.BOTTOM)
        self.selected_command_label.pack(pady=10, side=tk.TOP)
        self.current_widgets.append(self.add_new_command_button)
        self.current_widgets.append(self.save_all_button)
        self.current_widgets.append(self.remove_command_button)
        self.current_widgets.append(self.cancel_button)
        self.current_widgets.append(self.selected_command_label)
    
    def clear_window(self):
        for widget in self.current_widgets:
            widget.pack_forget()
        self.current_widgets.clear()
    
    def reset_parameters(self, including_command_type:bool):
        if including_command_type:
            self.command_type.set(COMMAND_TYPES[0])
        self.parameters["type"].set(CARTESIAN_TYPES[0])
        self.parameters["x"].set("0.0")
        self.parameters["y"].set("0.0")
        self.parameters["z"].set("0.0")
        self.parameters["v_max"].set("0.15")
        self.parameters["acc"].set("0.2")
        self.parameters["angle"].set("0.0")
        self.parameters["scan_type"].set(SCAN_TYPES[0])
        self.parameters["robot_moves"].set("")
        self.parameters["depth_or_color"].set(CAMERA_TYPES[0])
        self.parameters["object_width"].set("0.0095")
        self.parameters["starting_position"].set(STARTING_POSITIONS[1])
        self.parameters["yellow"].set("0")
        self.parameters["orange"].set("0")
        self.parameters["green"].set("0")
        self.parameters["put_down_type"].set(PUT_DOWN_TYPES[0])
        self.parameters["force"].set("0.0")
        self.parameters["put_down_pose"].set(STARTING_POSITIONS[0])
        self.parameters["movement_type"].set(MOVEMENT_TYPES[0])
        self.parameters["name_pose"].set(STARTING_POSITIONS[1])
        self.parameters["conveyor_speed"].set(0.0)
        self.parameters["conveyor_direction"].set(CONVEYOR_DIRECTIONS[0])
        self.parameters["duration"].set(0.0)
        self.parameters['joint_index'].set(JOINT_INDICES[0])
        self.parameters['joint_angle'].set(0.0)
        self.parameters["angle_type"].set(ANGLE_TYPES[0])

    def show_correct_menu(self,_,__,___):
        self.clear_window()
        self.reset_parameters(False)
        self.command_type_menu.pack(pady=5, side=tk.TOP)
        self.back_button.pack(pady=5,side=tk.BOTTOM)
        self.save_button.pack(pady=5, side=tk.BOTTOM)
        self.current_widgets.append(self.command_type_menu)
        self.current_widgets.append(self.save_button)
        self.current_widgets.append(self.back_button)
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
        elif self.command_type.get()=="rotate_single_joint":
            self.show_single_joint_menu()
        elif self.command_type.get()=="sleep":
            self.show_sleep_menu()
    
    def show_cartesian_menu(self):
        cartesian_type_label = tk.Label(self, text="Select the type of cartesian movement")
        self.pack_and_append(cartesian_type_label)
        cartesian_type_menu = tk.OptionMenu(self,self.parameters["type"],*CARTESIAN_TYPES)
        self.pack_and_append(cartesian_type_menu)

        cartesian_x_label = tk.Label(self, text="Enter the x value for the cartesian movement:")
        self.pack_and_append(cartesian_x_label)
        cartesian_x_entry = tk.Entry(self,textvariable=self.parameters["x"])
        self.pack_and_append(cartesian_x_entry)

        cartesian_y_label = tk.Label(self, text="Enter the y value for the cartesian movement:")
        self.pack_and_append(cartesian_y_label)
        cartesian_y_entry = tk.Entry(self,textvariable=self.parameters["y"])
        self.pack_and_append(cartesian_y_entry)

        cartesian_z_label = tk.Label(self, text="Enter the z value for the cartesian movement:")
        self.pack_and_append(cartesian_z_label)
        cartesian_z_entry = tk.Entry(self,textvariable=self.parameters["z"])
        self.pack_and_append(cartesian_z_entry)

        cartesian_acc_label = tk.Label(self, text="Enter the acceleration value for the cartesian movement:")
        self.pack_and_append(cartesian_acc_label)
        cartesian_acc_entry = tk.Entry(self,textvariable=self.parameters["acc"])
        self.pack_and_append(cartesian_acc_entry)

        cartesian_v_max_label = tk.Label(self, text="Enter the maximum velocity value for the cartesian movement:")
        self.pack_and_append(cartesian_v_max_label)
        cartesian_v_max_entry = tk.Entry(self,textvariable=self.parameters["v_max"])
        self.pack_and_append(cartesian_v_max_entry)

        cartesian_angle_label = tk.Label(self, text="Enter the angle value for the cartesian movement:")
        self.pack_and_append(cartesian_angle_label)
        cartesian_angle_entry = tk.Entry(self,textvariable=self.parameters["angle"],state=tk.DISABLED)
        self.pack_and_append(cartesian_angle_entry)

        angle_entry_enabled = partial(self.enable_disable_angle_entry,cartesian_angle_entry)
        self.parameters["type"].trace('w',angle_entry_enabled)

        validate_cartesian_angle = partial(validate_rotation_value, self.parameters["angle"], self.save_button)
        self.parameters["angle"].trace('w', validate_cartesian_angle)

    def show_scanning_menu(self):
        scanning_type_label = tk.Label(self, text="Select the type scan")
        self.pack_and_append(scanning_type_label)
        scanning_type_menu = tk.OptionMenu(self,self.parameters["scan_type"],*SCAN_TYPES)
        self.pack_and_append(scanning_type_menu)

        robot_moves_label = tk.Label(self, text="Please enter the points for the grid search. Enter them as [x,y] seperated by a comma. Leave blank to do default 3x3 scan.")
        self.pack_and_append(robot_moves_label)
        robot_moves_entry = tk.Entry(self, textvariable=self.parameters["robot_moves"],state=tk.DISABLED)
        self.pack_and_append(robot_moves_entry)

        robot_moves_entry_enabled = partial(self.enable_disable_robot_moves_entry,robot_moves_entry)
        self.parameters["scan_type"].trace('w',robot_moves_entry_enabled)

    def single_gear_pick_menu(self):
        depth_or_color_label = tk.Label(self, text="Select whether the scan should be done using the depth or color image")
        self.pack_and_append(depth_or_color_label)
        depth_or_color_menu = tk.OptionMenu(self,self.parameters["depth_or_color"],*CAMERA_TYPES)
        self.pack_and_append(depth_or_color_menu)

        object_width_label = tk.Label(self, text="Please enter the object_width")
        self.pack_and_append(object_width_label)
        object_width_entry = tk.Entry(self, textvariable=self.parameters["object_width"])
        self.pack_and_append(object_width_entry)
        
        self.parameters["starting_position"].set(STARTING_POSITIONS[0])
        starting_position_label = tk.Label(self, text="Please choose the named position that the robot should pick up the gear")
        self.pack_and_append(starting_position_label)
        starting_position_menu = tk.OptionMenu(self,self.parameters["starting_position"],*STARTING_POSITIONS)
        self.pack_and_append(starting_position_menu)
    
    def multiple_gears_pick_menu(self):
        object_width_label = tk.Label(self, text="IMPORTANT NOTE: SCANNING MUST BE DONE BEFORE THIS OR IT WILL NOT HAVE GEARS TO PICK UP.\n\nPlease enter the object_width")
        self.pack_and_append(object_width_label)
        object_width_entry = tk.Entry(self, textvariable=self.parameters["object_width"])
        self.pack_and_append(object_width_entry)

        starting_position_label = tk.Label(self, text="Please choose the starting position. Choose current to use the current position")
        self.pack_and_append(starting_position_label)
        starting_position_menu = tk.OptionMenu(self,self.parameters["starting_position"],*STARTING_POSITIONS)
        self.pack_and_append(starting_position_menu)

        color_label = tk.Label(self,text="Select the colors that you would like the robot to pick up")
        self.pack_and_append(color_label)
        yellow_button = tk.Checkbutton(self, text="Yellow", variable=self.parameters["yellow"], onvalue="1", offvalue="0", height=1, width=20)
        self.pack_and_append(yellow_button)
        orange_button = tk.Checkbutton(self, text="Orange", variable=self.parameters["orange"], onvalue="1", offvalue="0", height=1, width=20)
        self.pack_and_append(orange_button)
        green_button = tk.Checkbutton(self, text="Green", variable=self.parameters["green"], onvalue="1", offvalue="0", height=1, width=20)
        self.pack_and_append(green_button)
        

        depth_or_color_label = tk.Label(self, text="Select whether the second check should be done using the depth or color image")
        self.pack_and_append(depth_or_color_label)
        depth_or_color_menu = tk.OptionMenu(self,self.parameters["depth_or_color"],*CAMERA_TYPES)
        self.pack_and_append(depth_or_color_menu)

        put_down_type_label = tk.Label(self,text="Choose the method to put the gear down.")
        self.pack_and_append(put_down_type_label)
        put_down_type_menu =tk.OptionMenu(self,self.parameters["put_down_type"],*PUT_DOWN_TYPES)
        self.pack_and_append(put_down_type_menu)

        force_label = tk.Label(self, text="Please enter the put down force")
        self.pack_and_append(force_label)
        force_entry = tk.Entry(self, textvariable=self.parameters["force"])
        self.pack_and_append(force_entry)

        put_down_pose_label = tk.Label(self, text="Please choose the pose to put down the gear. Choose current to use the current position")
        self.pack_and_append(put_down_pose_label)
        put_down_pose_menu = tk.OptionMenu(self,self.parameters["put_down_pose"],*STARTING_POSITIONS)
        self.pack_and_append(put_down_pose_menu)

        force_entry_enabled = partial(self.enable_disable_force_entry,force_entry)
        self.parameters["put_down_type"].trace('w',force_entry_enabled)

    def show_put_down_menu(self):
        put_down_type_label = tk.Label(self,text="Choose the method to put the gear down.")
        self.pack_and_append(put_down_type_label)
        put_down_type_menu =tk.OptionMenu(self,self.parameters["put_down_type"],*PUT_DOWN_TYPES)
        self.pack_and_append(put_down_type_menu)

        z_label = tk.Label(self, text="Enter the z value down movement. For the table below the FR3, use -0.247.")
        self.pack_and_append(z_label)
        z_entry = tk.Entry(self,textvariable=self.parameters["z"])
        self.pack_and_append(z_entry)

        force_label = tk.Label(self, text="Please enter the put down force.")
        self.pack_and_append(force_label)
        force_entry = tk.Entry(self, textvariable=self.parameters["force"])
        self.pack_and_append(force_entry)

        update_menu = partial(self.update_put_down_menu,z_entry,force_entry)
        self.parameters["put_down_type"].trace('w',update_menu)

    def show_moving_gears_menu(self):
        movement_type_label = tk.Label(self, text="Select the type of movement")
        self.pack_and_append(movement_type_label)
        movement_type_menu = tk.OptionMenu(self,self.parameters["movement_type"],*MOVEMENT_TYPES)
        self.pack_and_append(movement_type_menu)

        object_width_label = tk.Label(self, text="Please enter the object_width")
        self.pack_and_append(object_width_label)
        object_width_entry = tk.Entry(self, textvariable=self.parameters["object_width"])
        self.pack_and_append(object_width_entry)

        object_width_entry_update = partial(self.update_movement_menu,object_width_entry)
        self.parameters["movement_type"].trace('w',object_width_entry_update)

    def show_named_pose_menu(self):
        name_pose_label = tk.Label(self, text="Please choose the named pose that you would like the robot to move to")
        self.pack_and_append(name_pose_label)
        name_pose_menu = tk.OptionMenu(self,self.parameters["name_pose"],*STARTING_POSITIONS[1:])
        self.pack_and_append(name_pose_menu)

    def show_move_conveyor_menu(self):
        conveyor_speed_label = tk.Label(self, text="Please enter the conveyor speed")
        self.pack_and_append(conveyor_speed_label)
        conveyor_speed_entry = tk.Entry(self, textvariable=self.parameters["conveyor_speed"])
        self.pack_and_append(conveyor_speed_entry)

        conveyor_direction_label = tk.Label(self,text="Select the direction for the conveyor belt")
        self.pack_and_append(conveyor_direction_label)
        conveyor_direction_menu = tk.OptionMenu(self, self.parameters["conveyor_direction"],*CONVEYOR_DIRECTIONS)
        self.pack_and_append(conveyor_direction_menu)

    def show_single_joint_menu(self):
        joint_index_label = tk.Label(self,text="Select the index for the joint you would like to rotate")
        self.pack_and_append(joint_index_label)
        joint_index_menu = tk.OptionMenu(self, self.parameters["joint_index"],*JOINT_INDICES)
        self.pack_and_append(joint_index_menu)

        joint_angle_label = tk.Label(self,text="Enter the joint angle in radians or degrees:")
        self.pack_and_append(joint_angle_label)
        joint_angle_entry = tk.Entry(textvariable=self.parameters["joint_angle"])
        self.pack_and_append(joint_angle_entry)

        angle_type_label = tk.Label(self,text="Select the type of angle:")
        self.pack_and_append(angle_type_label)
        angle_type_menu = tk.OptionMenu(self, self.parameters["angle_type"],*ANGLE_TYPES)
        self.pack_and_append(angle_type_menu)

        validate_joint_angle = partial(validate_rotation_value, self.parameters["joint_angle"], self.save_button)
        self.parameters["joint_angle"].trace('w', validate_joint_angle)

    def show_sleep_menu(self):
        sleep_label = tk.Label(self, text="Please enter duration for sleep")
        self.pack_and_append(sleep_label)
        sleep_entry = tk.Entry(self, textvariable=self.parameters["duration"])
        self.pack_and_append(sleep_entry)

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
    
    def update_label_and_remove_button(self,_,__,___):
        if len(self.selected_commands)>0:
            self.remove_command_button["state"] = tk.NORMAL
        else:
            self.remove_command_button["state"] = tk.DISABLED
        updated_text="Current code:\n\n"
        for command in self.selected_commands:
            if command["command_type"]=="open_gripper":
                updated_text+=("\nsupervisor.call_open_gripper_service()")
            elif command["command_type"]=="cartesian_movement":
                if command["type"]=="standard":
                    updated_text+=(f"\nsupervisor.call_move_cartesian_service({command['x']},{command['y']},{command['z']},{command['v_max']},{command['acc']})")
                elif command["type"]=="angle":
                    updated_text+=(f"\nsupervisor.call_move_cartesian_angle_service({command['x']},{command['y']},{command['z']},{command['v_max']},{command['acc']},{command['angle']})")
                else:
                    updated_text+=(f"\nsupervisor.call_move_cartesian_smooth_service({command['x']},{command['y']},{command['z']},{command['v_max']},{command['acc']})")
            elif command["command_type"]=="scanning":
                updated_text+=(f"\ndistances_from_home,updated_radius_vals = supervisor.select_scan(type_scan=\"{command['scan_type']}\""+("" if command['robot_moves']=="" or command['scan_type']=="single" else f",[\"{command['robot_moves']}\"]")+")")
            elif command["command_type"]=="pick_up_single_gear":
                updated_text+=(f"\nsupervisor.call_pick_up_gear_service(\"{command['depth_or_color']}\",{command['object_width']},\"{command['starting_position']}\")")
            elif command["command_type"]=="pick_up_multiple_gears":
                comma_needed=False
                color_list = "["
                if command["yellow"]=='1':
                    color_list+="yellow"
                    comma_needed=True
                if command["orange"]=='1':
                    if comma_needed:
                        color_list+=","
                    color_list+="orange"
                    comma_needed=True
                if command["green"]=='1':
                    if comma_needed:
                        color_list+=","
                    color_list+="green"
                color_list+="]"
                updated_text+=(f"\nsupervisor.pick_up_multiple_gears(distances_from_home,updated_radius_vals,{command['object_width']},\"{command['starting_position']}\",\"{color_list}\",\"{command['depth_or_color']}\",\"{command['put_down_type']}\",{command['force']},\"{command['put_down_pose']}\")")
            elif command["command_type"]=="put_down_gear":
                updated_text+=(f"\nsupervisor.put_gear_down_choose_type(\"{command['put_down_type']}\",{command['z']},{command['force']})")
            elif command["command_type"]=="moving_gears":
                if command["movement_type"]=="pick_up:":
                    updated_text+=(f"\nsupervisor.call_move_up_moving_gear_service({command['object_width']})")
                else:
                    updated_text+=("\nsupervisor.call_move_above_gear()")
            elif command["command_type"]=="move_to_named_pose":
                updated_text+=(f"\nsupervisor.call_move_to_named_pose_service(\"{command['name_pose']}\")")
            elif command["command_type"]=="enable_conveyor":
                updated_text+=(f"\nsupervisor.enable_conveyor_service(True)")
            elif command["command_type"]=="disable_conveyor":
                updated_text+=(f"\nsupervisor.enable_conveyor_service(False)")
            elif command["command_type"]=="move_conveyor":
                updated_text+=(f"\nsupervisor.set_conveyor_state_service({command['conveyor_speed']},{CONVEYOR_DIRECTIONS.index(command['conveyor_direction'])})")
            elif command["command_type"]=="rotate_single_joint":
                updated_text+=(f"\nsupervisor.call_rotate_single_joint({command['joint_index']}, {command['joint_angle']}, {'True' if command['angle_type']=='radians' else 'False'})")
            elif command["command_type"]=="sleep":
                updated_text+=(f"\nsleep({command['duration']})")
        self.selected_command_label.config(text=updated_text)

    def remove_command(self):
        self.clear_window()
        list_of_commands = "Current commands:\n"
        current_commands = []
        index = 0
        for command in self.selected_commands:
            current_commands.append(str(index)+" "+command["command_type"])
            index+=1
            if command["command_type"]=="open_gripper":
                list_of_commands+=("\nsupervisor.call_open_gripper_service()")
            elif command["command_type"]=="cartesian_movement":
                if command["type"]=="standard":
                    list_of_commands+=(f"\nsupervisor.call_move_cartesian_service({command['x']},{command['y']},{command['z']},{command['v_max']},{command['acc']})")
                elif command["type"]=="angle":
                    list_of_commands+=(f"\nsupervisor.call_move_cartesian_angle_service({command['x']},{command['y']},{command['z']},{command['v_max']},{command['acc']},{command['angle']})")
                else:
                    list_of_commands+=(f"\nsupervisor.call_move_cartesian_smooth_service({command['x']},{command['y']},{command['z']},{command['v_max']},{command['acc']})")
            elif command["command_type"]=="scanning":
                list_of_commands+=(f"\ndistances_from_home,updated_radius_vals = supervisor.select_scan(type_scan=\"{command['scan_type']}\""+("" if command['robot_moves']=="" or command['scan_type']=="single" else f",[\"{command['robot_moves']}\"]")+")")
            elif command["command_type"]=="pick_up_single_gear":
                list_of_commands+=(f"\nsupervisor.call_pick_up_gear_service(\"{command['depth_or_color']}\",{command['object_width']},\"{command['starting_position']}\")")
            elif command["command_type"]=="pick_up_multiple_gears":
                comma_needed=False
                color_list = "["
                if command["yellow"]=='1':
                    color_list+="yellow"
                    comma_needed=True
                if command["orange"]=='1':
                    if comma_needed:
                        color_list+=","
                    color_list+="orange"
                    comma_needed=True
                if command["green"]=='1':
                    if comma_needed:
                        color_list+=","
                    color_list+="green"
                color_list+="]"
                list_of_commands+=(f"\nsupervisor.pick_up_multiple_gears(distances_from_home,updated_radius_vals,{command['object_width']},\"{command['starting_position']}\",\"{color_list}\",\"{command['depth_or_color']}\",\"{command['put_down_type']}\",{command['force']},\"{command['put_down_pose']}\")")
            elif command["command_type"]=="put_down_gear":
                list_of_commands+=(f"\nsupervisor.put_gear_down_choose_type(\"{command['put_down_type']}\",{command['z']},{command['force']})")
            elif command["command_type"]=="moving_gears":
                if command["movement_type"]=="pick_up:":
                    list_of_commands+=(f"\nsupervisor.call_move_up_moving_gear_service({command['object_width']})")
                else:
                    list_of_commands+=("\nsupervisor.call_move_above_gear()")
            elif command["command_type"]=="move_to_named_pose":
                list_of_commands+=(f"\nsupervisor.call_move_to_named_pose_service(\"{command['name_pose']}\")")
            elif command["command_type"]=="enable_conveyor":
                list_of_commands+=(f"\nsupervisor.enable_conveyor_service(True)")
            elif command["command_type"]=="disable_conveyor":
                list_of_commands+=(f"\nsupervisor.enable_conveyor_service(False)")
            elif command["command_type"]=="move_conveyor":
                list_of_commands+=(f"\nsupervisor.set_conveyor_state_service({command['conveyor_speed']},{CONVEYOR_DIRECTIONS.index(command['conveyor_direction'])})")
            elif command["command_type"]=="rotate_single_joint":
                list_of_commands+=(f"\nsupervisor.call_rotate_single_joint({command['joint_index']}, {command['joint_angle']}, {'True' if command['angle_type']=='radians' else 'False'})")
            elif command["command_type"]=="sleep":
                list_of_commands+=(f"\nsleep({command['duration']})")
        current_commands_label = tk.Label(self,text=list_of_commands)
        current_commands_label.pack(pady=5,side=tk.TOP)
        self.current_widgets.append(current_commands_label)
        back_button = tk.Button(self,text="Back",command = self.back_command)
        back_button.pack(pady=5,side=tk.BOTTOM)
        self.current_widgets.append(back_button)
        command_to_remove = tk.StringVar()
        command_to_remove.set(current_commands[0])
        remove_selected = partial(self.remove_and_home, command_to_remove)
        remove_button = tk.Button(self,text="Remove selection", command=remove_selected)
        remove_button.pack(pady=5,side=tk.BOTTOM)
        self.current_widgets.append(remove_button)
        command_menu = tk.OptionMenu(self,command_to_remove, *current_commands)
        command_menu.pack(pady=5,side=tk.BOTTOM)
        self.current_widgets.append(command_menu)
        remove_command_label = tk.Label(self,text="Select the command you would like to remove. The options are numbered in order of appearance and the index selected will be deleted.")
        remove_command_label.pack(pady=5,side=tk.BOTTOM)
        self.current_widgets.append(remove_command_label)
        
    
    def remove_and_home(self, selection):
        del self.selected_commands[int(selection.get().split()[0])]
        self.command_counter.set(len(self.selected_commands))
        self.back_command()

        



def main(args=None):
    conveyor_enabled = False
    scan_already = False
    app = FR3_GUI()
    app.mainloop()
    command_order =[command['command_type'] for command in app.selected_commands]
    print(command_order)
    if app.cancel_flag.get()=="0":
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
                scan_already = True
                main_node.write(f"\n\t\tdistances_from_home,updated_radius_vals = supervisor.select_scan(type_scan=\"{command['scan_type']}\""+("" if command['robot_moves']=="" or command['scan_type']=="single" else f",[\"{command['robot_moves']}\"]")+")")
            elif command["command_type"]=="pick_up_single_gear":
                main_node.write(f"\n\t\tsupervisor.call_pick_up_gear_service(\"{command['depth_or_color']}\",{command['object_width']},\"{command['starting_position']}\")")
            elif command["command_type"]=="pick_up_multiple_gears":
                if not scan_already:
                    main_node.write(f"\n\t\tdistances_from_home,updated_radius_vals = supervisor.select_scan(type_scan=\"grid\")")
                comma_needed=False
                color_list = "["
                if command["yellow"]=='1':
                    color_list+="\"yellow\""
                    comma_needed=True
                if command["orange"]=='1':
                    if comma_needed:
                        color_list+=","
                    color_list+="\"orange\""
                    comma_needed=True
                if command["green"]=='1':
                    if comma_needed:
                        color_list+=","
                    color_list+="\"green\""
                color_list+="]"
                main_node.write(f"\n\t\tsupervisor.pick_up_multiple_gears(distances_from_home,updated_radius_vals,{command['object_width']},\"{command['starting_position']}\",{color_list},\"{command['depth_or_color']}\",\"{command['put_down_type']}\",{command['force']},\"{command['put_down_pose']}\")")
                scan_already = False
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
                conveyor_enabled = True
                main_node.write(f"\n\t\tsupervisor.enable_conveyor_service(True)")
            elif command["command_type"]=="disable_conveyor":
                conveyor_enabled = False
                main_node.write(f"\n\t\tsupervisor.enable_conveyor_service(False)")
            elif command["command_type"]=="move_conveyor":
                if not conveyor_enabled:
                    main_node.write(f"\n\t\tsupervisor.enable_conveyor_service(True)")
                    conveyor_enabled = True
                main_node.write(f"\n\t\tsupervisor.set_conveyor_state_service({command['conveyor_speed']},{CONVEYOR_DIRECTIONS.index(command['conveyor_direction'])})")
            elif command["command_type"]=="rotate_single_joint":
                main_node.write(f"\n\t\tsupervisor.call_rotate_single_joint({command['joint_index']}, {command['joint_angle']}, {'True' if command['angle_type']=='radians' else 'False'})")
            elif command["command_type"]=="sleep":
                main_node.write(f"\n\t\tsleep({command['duration']})")
        if conveyor_enabled:
            main_node.write(f"\n\t\tsupervisor.enable_conveyor_service(False)")
        main_node.write("\n\texcept Error as e:\n\n\t\tprint(e)\n\nif __name__ == \"__main__\":\n\tmain()")
        main_node.close()
        os.system("cd ~/fr3_ws")
        os.system("colcon build")
        os.system("source install/setup.bash")
        os.system("ros2 launch gear_place gear.launch.py")


if __name__ == '__main__':
    main()