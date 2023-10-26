import tkinter as tk
from tkinter import ttk

CAMERA_TYPES = ["depth","color"]
COMMAND_TYPES = ["open_gripper","cartesian_movement","scanning","pick_up_single_gear","pick_up_multiple_gears","put_down_gears","moving_gears","conveyor_belt","move_to_named_pose"]

class FR3_GUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.selected_commands = []
        self.title('FR3 control gui')
        self.current_commands = []
        self.resizable(width=False, height=False)
        self.grid_columnconfigure(0, weight=1)

        # Get the current screen width and height
        self.screen_width = self.winfo_screenwidth()
        self.screen_height = self.winfo_screenheight()
        

        self.geometry('800x550')

        self.add_new_command_button = tk.Button(self,text="Add command", command=self.add_command)
        self.add_new_command_button.pack(side=tk.BOTTOM)
        
        self.current_widgets = [self.add_new_command_button]
    
    def add_command(self):
        self.clear_window()
        command_type = tk.StringVar()
        command_type.set(COMMAND_TYPES[0])
        selected_parameters = {}
        self.command_type_menu = tk.OptionMenu(self,command_type, *COMMAND_TYPES)
        self.command_type_menu.pack(side=tk.TOP)
        self.save_button = tk.Button(self,text="Save command",command = self.save_command)
        self.save_button.pack(side=tk.BOTTOM)
        self.current_widgets.append(self.command_type_menu)
        self.current_widgets.append(self.save_button)
        self.selected_commands.append(selected_parameters)

    def save_command(self):
        self.clear_window()
        self.add_new_command_button.pack(side=tk.BOTTOM)
        self.current_widgets.append(self.add_new_command_button)
    
    def clear_window(self):
        for widget in self.current_widgets:
            widget.pack_forget()

        

def main():
    app = FR3_GUI()
    app.mainloop()


if __name__ == '__main__':
    main()