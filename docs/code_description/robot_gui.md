# robot_gui.py

## Overview

This file contains the code for the gui used to create and run nodes to control the robot. Each part of the gui is a different instruction for the robot and each parameter can be input to do exact inputs. This gui also does not allow incorrect inputs, so the file will always run correctly.

## Important functions

* pack_and_append

    This function takes in a widget as a parameter. It then packs that widget onto the main window and adds it to the `self.current_widgets` list. This list keeps track of all of the widgets currently in the window so they can be removed later/

* cancel_function

    This function sets the cancel flag to 1 and destroys the window. The cancel flag determines if the saved data is written to the file or not.

* add_command

    This function goes from the "main" window to the add_command window, which allows you to choose the type of command and it will display the correct menu depending on which you choose.

* save_command

    This function takes the chosen parameters from the add_command menu and saves them to the `self.selected_commands` list as a dictionary. Then, all of the parameters are cleared, meaning all menus are reset and you are taken to the home menu. This also updates the command_counter, which updates the label for the current commands.

* back_command
    
    This function is the same as the save_command funciton, but it does not save the information. It just clears the data and takes you back to the home menu.

* clear_window

    This function takes all of the widgets in `self.current_widgets` and "forgets" them, meaning that they are no longer on the window. After this, the list of current widgets is cleared.

* reset_parameters

    This function resets all of the parameters that you have chosen back to their original value. This is used to reset the gui after you go back or after a command is saved. This function allows the program to choose whether the command type is reset or not. This is needed for the `show_correct_menu` function so the menu can change.

* show_correct_menu

    This function is run every time you choose a new command type in the `add_command` menu. When the selection is changed, the window is cleared, all parameters except the command type is reset, and the correct menu is shown for the command type.

* SHOW MENU COMMANDS

    All of these functions are very similar, so this will cover all of them. Tkinter widgets are made for the menus, packed using the `pack_and_append` command, and trace commands are added to ensure that the input is validated.

* ENABLE DISABLE COMMANDS

    All of these functions control if certain widgets are enabled or disabled depending on other options. For example, for the cartesian movement, the standard and smooth movement does not need an angle, so the angle entry box will be disabled. But if you selected angle movement, the angle box is then enabled, allowing you to enter an angle.

* update_label_and_remove_button

    This function serves two main purposes. The first is to enable or disable the `remove_command` depending on if there are commands that are able to be removed. The second is to update the list of commands on the main menu. This is updated every time the `command_counter` goes up or down.

* remove_command

    This function starts by saving a list of all of the commands currently available and prints out the command how they would look like in the node. Then, a menu is created with the list of commands currently available and a remove selection button.

* remove_and_home

    This function is used when the remove selection button is pressed. It takes in a the selection and removes the command based on the index. The command counter is then updated and you are returned to the home menu.

* main

    This funciton starts by making an instance of the gui class and running it in a mainloop, which keeps the window open until it is destroyed. Then, if the cancel button was not pressed, the file is created in `~/fr3_ws/src/gear_place/gear_place/nodes`, which will allow it to be launched. The file is then written to by looping through the selected commands and writing each. There is also error checking which ensures that neccessary command run before others. For example, if you select the `move_conveyor_belt` command and the coneyor belt has not been enabled yet, it will be enabled. Finally, the program builds the workspace, sources it, and launches the node.