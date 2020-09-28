import time

def _clamp(value, limits):
    """
    Saturate value according to limits, where limits=(lower limit, upper limit)
    """
    lower = limits[0]
    upper = limits[1]
    if value < lower and lower is not None:
        value = lower
    elif value > upper and upper is not None:
        value = upper
    return value


class PID():
    """
    Basic PID controller.
    - Accepts saturation limits to prevent integral windup
    - Minimizes derivative kick from changing setPoint
    - Can be manually adjusted (*not yet tested)  
    """

    ## INITALIZE
    def __init__(self, Kp=1, Ki=0, Kd=0, setPoint=0, outLimits=None, intLimits=None, manualAdjust = False, sampleTime=0.1, dt=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setPoint = setPoint
        self.sampleTime = sampleTime
        self.outLimits =outLimits # saturation limit for final output value
        self.intLimits =intLimits # saturation limits for integral gain, must be <? Kp*currError to prevent integral windup, limits the maximum effect of the resulting I gain i.e. intLimits=(-5, 5) means the I term can account for at most +-5 in the output contrtol signal, in general it is ok for this limit to be small as I is typically most helpful for small changes anyway
        self.manualAdjust = manualAdjust
        self._dt = dt # time step, if not given default is None and actual time is calculated
       
        self.reset() # initialize carry-over variables (for integral and derivative terms) 


    ## COMPUTE OUTPUT
    def compute(self, currInput):
        """
        Compute and return the PID control output. 
        """

        # if in manual adjustment mode, return last output without computing anything else
        if self.manualAdjust:
            print("Manual adjustment mode activated.")
            if self._lastOutput is None:
                return 0 
            return self._lastOutput
 

        # check current time
        if self._dt is None:
            dt = time.monotonic() - self._lastTime
        else:
            dt = self._dt

        # return previous output if sampleTime has not elapsed        
        if dt < self.sampleTime and self._lastOutput is not None: 
            print("[WARN] Loop time {}".format(dt),"faster than sample time {}".format(self.sampleTime)+".", "Decrease sample time.")
            return self._lastOutput      

        # compute error 
        currError = self.setPoint - currInput

        # proportional term
        self._errP = self.Kp*currError
    
        # integral term         
        self._errI += self.Ki*currError*dt 
        if self.intLimits is not None:     
            self._errI = _clamp(self._errI, self.intLimits) # saturate to prevent integral windup

        # derivative term 
        if self._lastInput is None:
            self._lastInput = currInput
        dErrorKickback = -1*(currInput - self._lastInput) # use Derivative on Measurement to prevent kickback from changing setPoint
        self._errD = self.Kd * dErrorKickback / dt

        # output
        currOutput = self._errP + self._errI + self._errD
        if self.outLimits is not None:        
            currOutput = _clamp(currOutput, self.outLimits) # saturate to final output

        # update variables for next loop
        self._lastError = currError
        self._lastTime = time.monotonic()
        self._lastOutput = currOutput
        self._lastInput = currInput

        return currOutput

    def switch_mode(self, autoAdjust, lastOutput):
        """
        Switch between automatic computation and manual adjustment modes. 
        """

        # switch from manual to automatic
        if self.manualAdjust and autoAdjust:
            tempLastInput = self._lastInput            
            self.reset()
            self._lastInput = tempLastInput # last input from manual that automatic control should start with
            if lastOutput is not None:
                self._errI = lastOutput # last output from manual that the automatic control should start with
            self._errI = _clamp(self._errI, self.intLimits)

        self.manualAdjust = not autoAdjust

        # switch from automatic to manual
        
    def update_gains(self, Kp_new=None, Ki_new=None, Kd_new=None):
        if Kp_new != None:
            self.Kp = Kp_new
        if Ki_new != None:
            self.Ki = Ki_new
        if Kd_new != None:
            self.Kd = Kd_new

    def reset(self):
        """
        Reset internal controller parameters for errors, time, and input/output.
        - Use on startup and for switching between auto/manual control.
        """

        self._errP = 0 # proportional error term
        self._errI = 0 # integral error term
        self._errD = 0 # derivative error term

        self._lastError = 0
        self._lastTime = time.monotonic() # user monotonic time so never have negative/wrap-around time
        self._lastOutput = None
        self._lastInput = None

if __name__ == "__main__":
    from sys import platform
    if platform == "darwin": # need to use different backend to blit with MacOS
        import matplotlib
        matplotlib.use('Qt5Agg')
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, Button
    import numpy as np

    #### PID controller ####
    # To run demo, run pid.py from terminal. Close figure to stop simulation (click 'x', press 'q', etc.)
    # Windows and Linux users, this should work right away.
    # Mac users (with MacOSX backend), make sure you're using Qt5Agg GUI backend: https://vincentstudio.info/2019/02/08/012_Use_Qt5Agg_GUI_backend_for_matplotlib_on_MacOS/
    #
    # To implement this controller in other code, import as module (ex. import pid as PID) and create object (ex. myPID=PID(...))
    # For more reading, see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
    

    # set initial PID gains
    Kp = 0.8                # good= 0.8, demo= 0.1 
    Ki = 0.1               # good= 0.1, demo= 0.04
    Kd = 0.0               # good= 0, demo= 0.03

    # set some other PID parameters
    setPoint = 100          # set point to control to
    sampleTime = 0.00001    # sample time of PID controller, this must be less (faster) than the loop time (simulated or real) 
    outLimits = None        # limits for control output (ex. =(-100,100))
    intLimits = None        # limits for integral term to prevent integral windup (ex. =(-10, 10))
    manualAdjust = False    # the switch for this would need to be built, may be helpful for use with live systems 

    # simulation parameters
    currState = 0           # intial state
    simulationTime = 100    # time to run simulation for
    dtSim = 0.1             # note that the gains will change if this time changes
    dt = 0                  # initial time

    # initialize PID object
    demoPID = PID(Kp=Kp, Ki=Ki, Kd=Kd, setPoint=setPoint, sampleTime=sampleTime, outLimits=outLimits, intLimits=intLimits, manualAdjust=manualAdjust, dt=dtSim) # use dt=None for real system

    # plotting, slows down slightly from simulation time with a large amount of data to plot 
    plot_setPoint = []
    plot_currState = []
    plot_controlOutput = []    

    fig, ax1 = plt.subplots(figsize=(15,8))
    ax1.set_ylim([-50,200])
    ax1background = fig.canvas.copy_from_bbox(ax1.bbox)
    
    # setpoint slider
    def update_slider(val):
        global setPoint
        setPoint = val

    def reset_slider(event):
        slider_setPoint.reset()

    widgetColor = 'tan'
    sliderSetPoint = plt.axes([0.15, 0.29, 0.65, 0.03], facecolor=widgetColor)
    slider_setPoint = Slider(sliderSetPoint, 'set point', 0.0, 200.0, valinit=setPoint)
    slider_setPoint.on_changed(update_slider)

    resetSlider = plt.axes([0.9, 0.29, 0.07, 0.03])
    button_resetSlider = Button(resetSlider, 'Reset', color=widgetColor, hovercolor='0.975')
    button_resetSlider.on_clicked(reset_slider)
    
    
    # Kp slider
    def update_slider_Kp(val):
        global Kp
        Kp = val

    def reset_slider_Kp(event):
        slider_Kp.reset()

    widgetColor = 'tan'
    sliderKp = plt.axes([0.15, 0.21, 0.65, 0.03], facecolor=widgetColor)
    slider_Kp = Slider(sliderKp, 'Kp', 0.0, 2.5, valinit=Kp)
    slider_Kp.on_changed(update_slider_Kp)

    resetSliderKp = plt.axes([0.9, 0.21, 0.07, 0.03])
    button_resetSliderKp = Button(resetSliderKp, 'Reset', color=widgetColor, hovercolor='0.975')
    button_resetSliderKp.on_clicked(reset_slider_Kp)
    
        # Ki slider
    def update_slider_Ki(val):
        global Ki
        Ki = val

    def reset_slider_Ki(event):
        slider_Ki.reset()

    widgetColor = 'tan'
    sliderKi = plt.axes([0.15, 0.13, 0.65, 0.03], facecolor=widgetColor)
    slider_Ki = Slider(sliderKi, 'Ki', 0.0, 0.4, valinit=Ki)
    slider_Ki.on_changed(update_slider_Ki)

    resetSliderKi = plt.axes([0.9, 0.13, 0.07, 0.03])
    button_resetSliderKi = Button(resetSliderKi, 'Reset', color=widgetColor, hovercolor='0.975')
    button_resetSliderKi.on_clicked(reset_slider_Ki)
    
        # Kd slider
    def update_slider_Kd(val):
        global Kd
        Kd = val

    def reset_slider_Kd(event):
        slider_Kd.reset()

    widgetColor = 'tan'
    sliderKd = plt.axes([0.15, 0.05, 0.65, 0.03], facecolor=widgetColor)
    slider_Kd = Slider(sliderKd, 'Kd', 0.0, 0.1, valinit=Kd)
    slider_Kd.on_changed(update_slider_Kd)

    resetSliderKd = plt.axes([0.9, 0.05, 0.07, 0.03])
    button_resetSliderKd = Button(resetSliderKd, 'Reset', color=widgetColor, hovercolor='0.975')
    button_resetSliderKd.on_clicked(reset_slider_Kd)


    fig.subplots_adjust(bottom=0.4)
    fig.canvas.draw()   # note that the first draw comes before setting data
    firstPlot = True 


    # loop for all time steps
    while dt < simulationTime:

        # compute new control output 
        demoPID.setPoint = setPoint # if setPoint does not change then this is not needed
        demoPID.update_gains(Kp,Ki,Kd) # if gains do not change then this is not needed
        controlOutput = demoPID.compute(currState)
        
        # log for plotting (or data collection)
        plot_setPoint.append(demoPID.setPoint)
        plot_currState.append(currState)
        plot_controlOutput.append(controlOutput)

        # simulate 
        newState = currState + controlOutput # how the control output is applied often varies between systems
        
        time.sleep(dtSim)
        dt += dtSim

        # read in new current state, this would typically be from a filtered measurement from a sensor
        currState = newState

        # plot
        plot_dt = np.linspace(0, int(dt), len(plot_controlOutput))

        h1, = ax1.plot(plot_dt, color="orange", label="set point")
        h2, = ax1.plot(plot_dt, color="green", label="current state")
        h3, = ax1.plot(plot_dt, color="blue", label="control output")

        h1.set_ydata(plot_setPoint)
        h2.set_ydata(plot_currState)
        h3.set_ydata(plot_controlOutput)

        fig.canvas.restore_region(ax1background)
        fig.canvas.draw()

        ax1.draw_artist(h1)
        ax1.draw_artist(h2)
        ax1.draw_artist(h3)

        fig.canvas.blit(ax1.bbox)

        plt.pause(0.000000001)
    
        if firstPlot:
            fig.suptitle("PID Controller Demo", fontsize=20)
            ax1.set_title("Good Values are: P=0.8, I=0.1, D=0")
            ax1.set_xlabel("Time")
            ax1.set_ylabel("Control/State Value [same units]")
            fig.legend()
            firstPlot = False  
        

        
    

























