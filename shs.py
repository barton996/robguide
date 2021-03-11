from gpiozero import LED
import numpy as np
from time import sleep
import picamera
import matplotlib.pyplot as plt
from scipy import signal
import csv
import os
from multiprocessing import Process, current_process
from remote_pdb import RemotePdb

# Default peak detection variables
minimum_peak_width = 1
minimum_peak_height = 50
minimum_peak_distance = 10
STRIP_ROWS = 1

#LED Pins
LED_STATUS_RED = 18
LED_STATUS_GREEN = 15
LED_STATUS_BLUE = 14
LED_SCAN_0 = 21
LED_SCAN_1 = 20
LED_SCAN_2 = 16
LED_SCAN_TOGGLE = 19

# Scan resolution
H_RESOLUTION = 1280
V_RESOLUTION = 32
H_RESOLUTION_STEREO = 2 * H_RESOLUTION

# Extrinsic Calibration parameters
default_calibration_initial = -30
default_calibration_final = 30
default_calibration_steps = 21

# Sensor intrisic calibration file path
SENSOR_CALIBRATION = 'cal.csv'

# Set plot style
plt.style.use('dark_background')


class Project:
    def __init__(self):
        self.name = 'robguide-project'
        self.job_list = []
        self.add_job()
        
    def add_job(self):
        self.job_list.append(Job())
        
    def remove_job(self, job_number):    
        self.job_list.pop(job_number)
          
    def rename(self, name):    
        self.name = name
        
        
class Job:
    def __init__(self):
        self.name = 'robguide-job'
        self.sequence_list = []
        self.add_sequence()   
        # Extrinsic calibration settings (mm)
        self.calibration_initial = default_calibration_initial
        self.calibration_final = default_calibration_final
        self.calibration_steps = default_calibration_steps
        self.calibration_positions = np.linspace(self.calibration_initial, self.calibration_final, self.calibration_steps)

    def add_sequence(self):
        self.sequence_list.append(Sequence())

    def remove_sequence(self, sequence_number):
        self.sequence_list.pop(sequence_number)
        
    def calibrate(self):
        for sequence in self.sequence_list:
            self.sequence_list[sequence].calibrate(self.calibration_positions)


class Sequence:
    def __init__(self):
        self.sensor = None
        self.name = 'robguide-sequence'    
        # Extrinsic Calibration object
        self.calibration = ExtrinsicCalibration()
        self.peaks = Peaks()
        self.plot = Plot()
        
        # 000 Settings that determine the exposure mode of the sensor. 
        # Best to write this once I have redone the Sensor/Shs class to include AGC.

    def set_sensor(self, sensor):
        self.sensor = sensor
        
    def remove_sensor(self):
        self.sensor = None
        
    def rename(self, name):
        self.name = name
        
    def calibrate(self, calibration_positions):
        if self.sensor is None:
            print('Sequence has no specified sensor.')
        else:
            for cal_axis in calibration.raw_data:
                for cal_pos in calibration_positions:
                    # 000 Send position to robot
                    # 000 Wait for reply from robot
                    np.append(calibration.raw_data[cal_axis], self.sensor.measure(), axis=0)
                    
                input('All calibration data captured. Press Enter to plot graphs.')
                # 000 Now plot graphs
                # Produce fits
                # Generate fit values
                self.calibration.set(calibration_values) # 000 This should be a 12 x 1 nparray

    def live_mode(self):
        #breakpoint()
        process = Process(target=self.sensor.grab_scan_continuous, args=(), daemon=True)
        process.start()
        #self.sensor.grab_scan_continuous()
        while True:
            print(self.sensor.scan_available)
            if self.sensor.scan_available:
                self.plot.update_scan_only(self.sensor.current_scan_left, self.sensor.current_scan_right)
    
       
class ExtrinsicCalibration:
    def __init__(self):
        self.X_data = np.empty((0, 2))
        self.Y_data = np.empty((0, 2))
        self.Z_data = np.empty((0, 2))
        self.A_data = np.empty((0, 2))
        self.B_data = np.empty((0, 2))
        self.C_data = np.empty((0, 2)) 
        self.raw_data = [self.X_data, self.Y_data, self.Z_data, self.A_data, self.B_data, self.C_data]
            
        # Extrinsic calibration values
        # (dx/dX, dy/dX, dx/dY, dy/dY, dx/dZ, dy/dZ, dx/dA, dy/dA, dx/dB, dy/dB, dx/dC, dy/dC)
        # xy are sensor coordinates. XYZABC are robot coordinates.
        self.values = np.zeros((12, 2))
        self.valid = False
        
    def set(self, calibration_values):
        self.values = calibration_values
        self.valid = True

    def reset(self):
        self.X_data = np.empty((0, 2))
        self.Y_data = np.empty((0, 2))
        self.Z_data = np.empty((0, 2))
        self.A_data = np.empty((0, 2))
        self.B_data = np.empty((0, 2))
        self.C_data = np.empty((0, 2))
                
        self.values = np.zeros((12, 2))
        self.valid = False

class Plot:
    def __init__(self):
        #Can improve efficieny of plotting function using knowledge at: https://bastibe.de/2013-05-30-speeding-up-matplotlib.html
        #Define pixel array for plotting purposes
        self.pixel_number = np.linspace(1, H_RESOLUTION, num=H_RESOLUTION)
        #Generate figure and axes and set size of figure
        self.fig, self.axs = plt.subplots(2,1, squeeze=True, sharex=True)
        self.fig.set_size_inches(12, 4)
        #Initialise left and right scan lines with random data for scan profile. Left is top and right is bottom, from perspective of sensor.
        self.scan_data_left, = self.axs[0].plot(self.pixel_number, np.random.rand(H_RESOLUTION), color='red')
        self.scan_data_right, = self.axs[1].plot(self.pixel_number, np.random.rand(H_RESOLUTION), color='red')
        #Initialise left and right master profile line with random data. Left is top and right is bottom, from perspective of sensor.
        #self.master_profile_left, = self.axs[0].plot(self.pixel_number, np.random.rand(H_RESOLUTION), color='silver')
        #self.master_profile_right, = self.axs[1].plot(self.pixel_number, np.random.rand(H_RESOLUTION), color='silver')
        #Initialise left and right peak markers with random data. Left is top and right is bottom, from perspective of sensor.
        self.peak_markers_left, = self.axs[0].plot(np.random.rand(1), np.random.rand(1), 'y|')
        self.peak_markers_right, = self.axs[1].plot(np.random.rand(1), np.random.rand(1), 'y|')
        #Set axis limits on the graphs
        self.axs[0].set_ylim(0, 265)
        self.axs[0].set_xlim(0, H_RESOLUTION)
        self.axs[1].set_ylim(0, 265)
        self.axs[1].set_xlim(0, H_RESOLUTION)
        #Show the figure (block=False disables the program break that is default for plt.show())
        plt.show(block=False)
        
    def update(self, scan_data_left, scan_data_right, current_peaks_left, current_peaks_right):
        #Refresh data for scan lines
        self.scan_data_left.set_ydata(scan_data_left)
        self.scan_data_right.set_ydata(scan_data_right)
        #Refresh data for left and right peaks
        self.peak_markers_left.set_xdata(current_peaks_left)
        self.peak_markers_left.set_ydata(np.full(current_peaks_left.shape, 260))
        self.peak_markers_right.set_xdata(current_peaks_right)
        self.peak_markers_right.set_ydata(np.full(current_peaks_right.shape, 260))
        #self.master_profile_left.set_ydata(master_profile_left)
        #self.master_profile_right.set_ydata(master_profile_right)
        #Redraw the current figure with the new data
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    #EXPERIMENTAL
    def update_scan_only(self, scan_data_left, scan_data_right):
        #Refresh data for scan lines
        self.scan_data_left.set_ydata(scan_data_left)
        self.scan_data_right.set_ydata(scan_data_right)
        #Redraw the current figure with the new data
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


class Peaks():
    def __init__(self):
        # Peak detection settings
        self.minimum_peak_width = minimum_peak_width
        self.minimum_peak_height = minimum_peak_height
        self.minimum_peak_distance = minimum_peak_distance
    
    def find_peaks(self, scan_data_left, scan_data_right):
        peaks_left, _ = signal.find_peaks(scan_data_left.flatten(), height=self.minimum_peak_height, width=self.minimum_peak_width, distance=self.minimum_peak_distance)
        peaks_right, _ = signal.find_peaks(scan_data_right.flatten(), height=self.minimum_peak_height, width=self.minimum_peak_width, distance=self.minimum_peak_distance)
        return scan_data_left, scan_data_right, peaks_left, peaks_right
              
    def set_peak_height(self, height):
        self.minimum_peak_height = height
    
    def set_peak_width(self, width):
        self.minimum_peak_width = width
        
    def set_peak_distance(self, distance):
        self.minimum_peak_distance = distance


class Sensor():
    def __init__(self):
        # SHS configuration
        self.camera = picamera.PiCamera(stereo_mode ='side-by-side', stereo_decimate=False, resolution=(H_RESOLUTION_STEREO, V_RESOLUTION))
        self.camera.awb_mode = 'off'
        self.camera.exposure_mode = 'off'
        self.camera.shutter_speed = 10000
        self.video_port_enable = True
        # Sensor calibration
        self.calibration = IntrinsicCalibration(SENSOR_CALIBRATION)
        # Scan LEDs
        self.scan_leds = ScanLED()
        # SensorOutput object
        self.output = SensorOutput()
        #Current scan
        self.current_scan_left = np.zeros(H_RESOLUTION)
        self.current_scan_right = np.zeros(H_RESOLUTION)
        #Scan available flag
        self.scan_available = False
        
    def grab_scan(self):
        # Enable scan LEDs
        self.scan_leds.toggle.on()
        # Capture LED ON scan
        try:
            self.camera.capture(self.output, 'yuv', use_video_port=self.video_port_enable)
        except IOError:
            pass
        sleep(0.05)
        scan_leds_on = self.output.scan
        # Disable scan LEDs
        self.scan_leds.toggle.off()
        # Capture LED OFF scan
        try:
            self.camera.capture(self.output, 'yuv', use_video_port=self.video_port_enable)
        except IOError:
            pass
        sleep(0.05)
        scan_leds_off = self.output.scan
        # Subtract the LED OFF scan from LED ON scan to reduce noise
        scan_data = np.transpose(np.subtract(scan_leds_on, scan_leds_off.astype(np.int16)).clip(0, 255).astype(np.uint8))
        # Split the data into left and right channels
        self.current_scan_left = scan_data[:H_RESOLUTION]
        self.current_scan_right = scan_data[H_RESOLUTION:]
        self.scan_available = True
        #return scan_data_left, scan_data_right

    def grab_scan_continuous(self):
        #RemotePdb('127.0.0.1', 4444).set_trace()
        self.scan_leds.toggle.on()
        for frame in self.camera.capture_continuous(self.output, format='yuv', use_video_port=self.video_port_enable):
            RemotePdb('127.0.0.1', 4444).set_trace()
            if self.scan_leds.toggle.is_lit:
                scan_leds_on = self.output.scan
                self.scan_leds.toggle.off()
            else:
                scan_leds_off = self.output.scan
                scan_data = np.transpose(np.subtract(scan_leds_on, scan_leds_off.astype(np.int16)).clip(0, 255).astype(np.uint8))
                self.current_scan_left = scan_data[:H_RESOLUTION]
                self.current_scan_right = scan_data[H_RESOLUTION:]
                #print(signal.find_peaks(scan_data.flatten(), minimum_peak_height, minimum_peak_width, minimum_peak_distance))
                self.scan_available = True
                self.scan_leds.toggle.on()
        

class SensorOutput(object):
    def _init_(self):
        self.scan = np.empty(1, H_RESOLUTION_STEREO, dtype=np.uint8)
    
    def write(self, buf):
        # write will be called once for each frame of output. buf is a bytes
        # object containing the frame data in YUV420 format; we can construct a
        # numpy array on top of the Y plane of this data quite easily:
        y_data = np.frombuffer(buf, dtype=np.uint8, count=H_RESOLUTION_STEREO*V_RESOLUTION).reshape((V_RESOLUTION, H_RESOLUTION_STEREO))
        self.scan = y_data[0, :H_RESOLUTION_STEREO]
            
    def flush(self):
        # this will be called at the end of the recording; do whatever you want
        # here
        pass


class StatusLED:
    def __init__(self):
        self.red = LED(LED_STATUS_RED)
        self.green = LED(LED_STATUS_GREEN)
        self.blue = LED(LED_STATUS_BLUE)
        
    def reset(self):
        # Disable all colours
        self.red.off()
        self.blue.off()
        self.green.off()
        
    def idle(self):
        # Reset LED
        self.reset()
        # Enable idle (green) status LED
        self.green.on()
        
    def active(self):
        # Reset LED
        self.reset()
        # Enable active (blue) staus LED
        self.blue.on()
        
    def scan(self):
        # Reset LED
        self.reset()
        # Enable scan (red) status LED
        self.red.on()


class ScanLED:
    def __init__(self):
        # Define scan LED pins
        self.top = LED(LED_SCAN_0)
        self.middle = LED(LED_SCAN_1)
        self.bottom = LED(LED_SCAN_2)
        self.toggle = LED(LED_SCAN_TOGGLE)
        self.top.on()
        self.middle.on()
        self.bottom.on()

       
class IntrinsicCalibration:
    def __init__(self, intrinsic_calibration_path):
        with open(intrinsic_calibration_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                # Left camera
                self.x_l = float(row[0])
                self.y_l = float(row[1])
                self.beta_0_l = float(row[2])
                self.focal_l = float(row[3])
                # Right camera
                self.x_r = float(row[4])
                self.y_r = float(row[5])
                self.beta_0_r = float(row[6])
                self.focal_r = float(row[7])
                # Pixel Size
                self.pixel_size = float(row[8])
                
        # Angular field of view
        self.afov_l = 2 * np.tan((H_RESOLUTION * self.pixel_size)/(2 * self.focal_l))
        self.afov_r = 2 * np.tan((H_RESOLUTION * self.pixel_size)/(2 * self.focal_r))




        

    
        
        
        
        	
        






		


		
		

            
            
           
            
