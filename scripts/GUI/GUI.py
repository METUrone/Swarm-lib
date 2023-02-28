#pip install customrkinter


## BEFORE TO RUN THE GUI.py PLEASE READ COMMENTS AND SET YOUR ENV. CAREFULLY
## ACCESING SOME FOLDERS MIGHT GIVE SOME ERRORS

##PAY ATTENTION TO mission_planner.py FOLDER, SINCE GUI.py FOLDER OVERWRITES ONTO IT. 


import tkinter as tk
from tkinter.ttk import *
import tkinter.messagebox
import customtkinter
from PIL import Image, ImageTk
import cv2
import os.path

customtkinter.set_appearance_mode("System")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"


class App(customtkinter.CTk):

    with open(os.path.dirname(__file__) +"/../mission_planner.py","r+") as fp:
        lines = fp.readlines()# read an store all lines into list
        fp.seek(0)# move file pointer to the beginning of a file
        fp.truncate()# truncate the file
        fp.writelines(lines[:9])#start writing first 10 lines, there must be no given mission before first 10 lines.


    def __init__(self):
        super().__init__()

        #FUNCTIONS --> Swarm
    
        def Add_mission():
                
            if self.type_shape=="prism":
                num=int(self.edge_entry.get())
                    
            else:
                num="'"+self.type_shape+"'"
                print(num)

            rad=float(self.radius_entry.get())
            h=float(self.height_entry.get())
            obj_h=float(self.top_entry.get()) 
            with open(os.path.dirname(__file__) +"/../mission_planner.py", "a") as f:     # write missions one by one (order can be changed later)
                        f.write(f"\napf.form_3d({rad}, {num},{h},{obj_h})" )
                        f.write(f"\ntime.sleep({self.delay1_entry.get()})")
                        f.write(f"\napf.go({(self.go_entry.get())})" )
                        f.write(f"\ntime.sleep({self.delay2_entry.get()})")
                        f.write(f"\napf.rotate({(self.rotation_entry.get())})" )
        def Run():
            Add_mission()
            import os
            print(os.path.dirname(__file__))
            os.system(os.path.dirname(__file__) +"/3d_gui.sh")  #import this sh file to os.system to maintain alternative option about runing our code in terminal
        
        def Add_fly(in_num,out_num,id=195,x=1.0,y=1.0,z=0.0): #append new flies (!!!set crazyflies.yaml file to be have only 3 flies and last one's id must be 195!!!)
            print(in_num,out_num)
            while in_num!=out_num:
                id+=1
                x=x+1 if x!=1 else (-1)
                if in_num%3==0: #should be fixed!!
                    y+=1
                with open(os.path.dirname(__file__) +"/../../config/crazyflies.yaml", "a") as f:  ###  Adding new flies.. !! DON NOT TOUCH/CHANGE ANYTHING ! ! !
                    f.write(f"""\n- channel: 125
  id: {id}
  initialPosition:      
  - {x}
  - {y}
  - {z}
  type: default""".format(id=id,x=x,y=y,z=z))
                in_num-=1  

        def Num_3D():
            with open(os.path.dirname(__file__) +"/../../config/crazyflies.yaml","r+") as yaml:
                lines = yaml.readlines()# read an store all lines into list
                yaml.seek(0)# move file pointer to the beginning of a file
                yaml.truncate()# truncate the file
                yaml.writelines(lines[:23])#start writing first 23 lines 

            if self.type_shape=="prism":
                num=(2*int(self.edge_entry.get()))
                if num<6:
                    print("Number of Drones must be bigger than 2")
                    #messagebox.showerror("Num_3D","Number of Drones must be bigger than 2")
                    #answer.config(text="Number of Drones must be bigger than 2")
                elif num-3>0: 
                    id,x,y,z=195,-1.0,1.0,0.0  #our data about new drone
                    Add_fly(num,3)

            elif self.type_shape=="pyramid":
                Add_fly(int(self.edge_entry.get())+1,3)

            elif self.type_shape=="cylinder":
                Add_fly(int(self.edge_entry.get())*2+2,3)
            

                    
    
        # configure window
        self.title("MetuRone Swarm")
        self.geometry(f"{800}x{580}")

        # configure grid layout (4x4)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1, 2), weight=1)

        # create sidebar frame with widgets
        self.sidebar_frame = customtkinter.CTkFrame(self, width=140, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)
        self.logo_label = customtkinter.CTkLabel(self.sidebar_frame, text="Görevler", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        self.sidebar_button_1 = customtkinter.CTkButton(self.sidebar_frame,text="Görevler" ,command=self.sidebar_button_event)
        self.sidebar_button_1.grid(row=1, column=0, padx=20, pady=10)
        self.sidebar_button_2 = customtkinter.CTkButton(self.sidebar_frame,text="Sifirla", command=self.sidebar_button_event)
        self.sidebar_button_2.grid(row=2, column=0, padx=20, pady=10)
        self.sidebar_button_3 = customtkinter.CTkButton(self.sidebar_frame, command=self.sidebar_button_event)
        self.sidebar_button_3.grid(row=3, column=0, padx=20, pady=10)
        self.appearance_mode_label = customtkinter.CTkLabel(self.sidebar_frame, text="Appearance Mode:", anchor="w")
        self.appearance_mode_label.grid(row=5, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["Light", "Dark", "System"],
                                                                       command=self.change_appearance_mode_event)
        self.appearance_mode_optionemenu.grid(row=6, column=0, padx=20, pady=(10, 10))
        self.scaling_label = customtkinter.CTkLabel(self.sidebar_frame, text="UI Scaling:", anchor="w")
        self.scaling_label.grid(row=7, column=0, padx=20, pady=(10, 0))
        self.scaling_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["80%", "90%", "100%", "110%", "120%"],
                                                               command=self.change_scaling_event)
        self.scaling_optionemenu.grid(row=8, column=0, padx=20, pady=(10, 20))

        
        # create tabview
        self.tabview = customtkinter.CTkTabview(self, width=400,height=500)
        self.tabview.grid(row=0, column=1, padx=(5, 0), pady=(5, 0), sticky="nsew")
        self.tabview.add("3B")
        self.tabview.add("Yangin")
        self.tabview.add("Engel") 
        self.tabview.tab("3B").grid_columnconfigure(0, weight=2)  # configure grid of individual tabs
        self.tabview.tab("Yangin").grid_columnconfigure(0, weight=2)

        #column0 of the first tab
        i,y=0,0 #order of entries
        self.type_shape=""
        segemented_button = customtkinter.CTkSegmentedButton(self.tabview.tab("3B"),width=250,
                                                     values=["Prism", "Pyramid", "Cylinder"],
                                                     command=self.segmented_button_callback)
        segemented_button.grid(row=i, column=0, padx=0, pady=(0, 0))
        segemented_button.set("Shape")  # set initial value
        i+=1
        #self.string_input_button = customtkinter.CTkButton(self.tabview.tab("3B"), text="Input",
        #                                                   command=self.open_input_dialog_event)
        #self.string_input_button.grid(row=1, column=0, padx=5, pady=(5, 0))

        
        self.delay1_entry = customtkinter.CTkEntry(self.tabview.tab("3B"),width=250, height=50,
                                                   placeholder_text="First Delay",)
        self.delay1_entry.grid(row=i, column=y, padx=(5, 5), pady=(5, 0), sticky="nsew")
        i+=1
        
        self.edge_entry = customtkinter.CTkEntry(self.tabview.tab("3B"), width=250, height=50,
                                                placeholder_text="Num of Edges",)
        self.edge_entry.grid(row=i, column=y, padx=(5, 5), pady=(5, 0), sticky="nsew")
        i+=1
        self.radius_entry = customtkinter.CTkEntry(self.tabview.tab("3B"), width=250, height=50,
                                                   placeholder_text="Value of Radius",)
        self.radius_entry.grid(row=i, column=y, padx=(5, 5), pady=(5, 0), sticky="nsew")
        i+=1
        self.height_entry = customtkinter.CTkEntry(self.tabview.tab("3B"), width=250, height=50,
                                                   placeholder_text="Value of Height",)
        self.height_entry.grid(row=i, column=y, padx=(5, 5), pady=(5, 0), sticky="nsew")
        i+=1
        self.delay2_entry = customtkinter.CTkEntry(self.tabview.tab("3B"), width=250, height=50,
                                                   placeholder_text="Second Delay",)
        self.delay2_entry.grid(row=i, column=y, padx=(5, 5), pady=(5, 0), sticky="nsew")
        

        #coloumn2 of the first tab
        self.add_Button = customtkinter.CTkButton(self.tabview.tab("3B"),text="Add to list", width=250, height=50,command=Num_3D)
        self.add_Button.grid(row=i, column=1 ,padx=0, pady=0)
        i,y=1,1
        self.go_entry = customtkinter.CTkEntry(self.tabview.tab("3B"), width=250, height=50,
                                                   placeholder_text="Goal",)
        self.go_entry.grid(row=i, column=y, padx=(5, 5), pady=(0, 0), sticky="nsew")
        i+=1
        self.rotation_entry = customtkinter.CTkEntry(self.tabview.tab("3B"), width=250, height=50,
                                                   placeholder_text="Rotation",)
        self.rotation_entry.grid(row=i, column=y, padx=(5, 5), pady=(5, 0), sticky="nsew")
        i+=1
        self.top_entry = customtkinter.CTkEntry(self.tabview.tab("3B"), width=250, height=50,
                                                   placeholder_text="Altetiude of top layer",)
        self.top_entry.grid(row=i, column=y, padx=(5, 5), pady=(5, 0), sticky="nsew")
        i+=1


        #2nd TAB
        self.label_tab_2 = customtkinter.CTkLabel(self.tabview.tab("Yangin"), text="Yakinda yüklenecek")
        self.label_tab_2.grid(row=0, column=0, padx=20, pady=20)

        self.add_Button = customtkinter.CTkButton(self.tabview.tab("Yangin"),text="Open WebCam", width=250, height=50,font=customtkinter.CTkFont(size=17),command=self.openNewWindow)
        self.add_Button.grid(row=1, column=0 ,padx=0, pady=0)



        # Run Button
        self.run_button_3B = customtkinter.CTkButton(self,width=500,height=100, text="Run", font=customtkinter.CTkFont(size=20, weight="bold"),command=Run)
        self.run_button_3B.grid(row=1, column=1 ,padx=5, pady=20)



        #left part
        self.sidebar_button_3.configure(state="disabled", text="Disabled CTkButton")
        self.appearance_mode_optionemenu.set("Dark")
        self.scaling_optionemenu.set("100%")

        
    #FUNCTIONS --> GUI
        
    def openNewWindow(self):
    
        # Toplevel object which will
        # be treated as a new window
        newWindow = tk.Toplevel()
        # sets the title of the
        # Toplevel widget
        newWindow.title("WebCam")
        # sets the geometry of toplevel
        newWindow.geometry("650x530")
        # A Label widget to show in toplevel
        label=Label(newWindow
          )
        label.grid(row=0, column=0)  
                  
        i=0

        def mirror():
            nonlocal i
            i=i^1
        
        cap= cv2.VideoCapture(0)
        
        # Define function to show frame
        def show_frames():
           nonlocal i
           # Get the latest frame and convert into Image
           cv2image= cv2.cvtColor(cap.read()[1],cv2.COLOR_BGR2RGB)
           cv2image=cv2image[:,::-1] if i==1 else cv2image[:,::]
           img = Image.fromarray(cv2image)
           # Convert image to PhotoImage
           imgtk = ImageTk.PhotoImage(image = img)
           label.imgtk = imgtk
           label.configure(image=imgtk)
           # Repeat after an interval to capture continiously
           label.after(20, show_frames)
        
        show_frames()
        button1 = tk.Button(newWindow, text="Mirror",
                                    command=mirror)
        button1.grid(row=1,column=0) 
        #newWindow.mainloop()
        
        
    def open_input_dialog_event(self):
        dialog = customtkinter.CTkInputDialog(text="Type in a number:", title="Input")
        print("CTkInputDialog:", dialog.get_input())

    def segmented_button_callback(self,value):
        if value=="Pyramid":
            self.type_shape="pyramid"
        elif value=="Cylinder":
            self.type_shape="cylinder"
        else:
            self.type_shape="prism"
        

        print("segmented button clicked:", value)

    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 90
        customtkinter.set_widget_scaling(new_scaling_float)

    def sidebar_button_event(self):
        print("sidebar_button click")

    


if __name__ == "__main__":
    app = App()
    app.mainloop()