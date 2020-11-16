from tkinter import *
import time
import sys

try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')
    sys.exit(-1)


class App(Frame):
    def __init__(self, root):
        super(App, self).__init__(root)
        self.root = root
        self.pack()
        self.root.geometry('600x578')
        self.root.title('sim_API')
        self.image = PhotoImage(file='images/sim_background.png')
        self.root.background_label = Label(root, image=self.image)
        self.root.background_label.place(x=0, y=0, relwidth=1, relheight=1)
        self.root.iconbitmap('images/coppeliaSim.ico')
        self.clientID = -1

        # Redefine X button
        self.root.protocol('WM_DELETE_WINDOW', self.quit)

        # Text settings.
        self.text = Label(root, text='', font='arial 10')

        # Adding buttons.
        self.button_connect = Button(root, text='connect', width=10, height=2, bg='green', font='arial 18',
                                     command=self.connect)
        self.button_start = Button(root, text='start', width=10, height=2, bg='white', font='arial 18',
                                   command=self.start_simulation)
        self.button_stop = Button(root, text='stop', width=10, height=2, bg='white', font='arial 18',
                                  command=self.stop_simulation)
        self.button_getvalue = Button(root, text='get velocity', width=10, height=2, bg='white', font='arial 18',
                                      command=self.get_value)
        # Create a grid and put buttons there.
        self.button_connect.pack(side='top', expand=1)
        self.text.pack(side='bottom', expand=1)
        self.button_start.pack(side='left', expand=1)
        self.button_stop.pack(side='left', expand=1)
        self.button_getvalue.pack(side='left', expand=1)

    def connect(self):
        # Connecting to copeliasim via port 19997.
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
        if self.clientID != -1:
            self.text.config(text='Connected to remote API server', bg='green')

            # Now try to retrieve data in a blocking fashion (i.e. a service call):
            res, objs = sim.simxGetObjects(self.clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
            if res == sim.simx_return_ok:
                print('Number of objects in the scene: ', len(objs))
            else:
                self.text.config(text='Remote API function call returned with error code: ' + str(res), bg='red')

            time.sleep(2)

    def start_simulation(self):
        if self.clientID != -1:
            if 0 <= sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot) <= 1:
                self.text.config(text='Simulation started.', bg='white')
            else:
                self.text.config(text='Problem with starting simulation', bg='red')

    def stop_simulation(self):
        if self.clientID != -1:
            if 0 <= sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot) <= 1:
                self.text.config(text='Simulation stopped', bg='white')
            else:
                self.text.config(bg='black')

    def get_value(self):
        res, v0 = sim.simxGetObjectHandle(self.clientID, 'bubbleRob', sim.simx_opmode_oneshot_wait)
        res, linear_velocity, angular_velocity = sim.simxGetObjectVelocity(self.clientID, v0,
                                                                           sim.simx_opmode_oneshot_wait)
        self.text.config(text=str(linear_velocity))
        sim.simxSetObjectFloatParameter(self.clientID, v0, sim.sim_shapefloatparam_init_velocity_x, 10000,
                                        sim.simx_opmode_oneshot_wait)

    def quit(self):
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        # You can guarantee this with (for example):
        sim.simxGetPingTime(self.clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(self.clientID)
        sys.exit(0)

