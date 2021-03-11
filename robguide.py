import shs

#Create robguide project
#By default this project has one job and one sequence
main = shs.Project()
#Create a sensor
sensor = shs.Sensor()
#Assign the sensor just created to the first sequence of the first job
main.job_list[0].sequence_list[0].set_sensor(sensor)
#Put the sequence into live mode
main.job_list[0].sequence_list[0].live_mode()


#while True:
    #plot.update(*peak.find_peaks(*sensor.grab_scan()))
    #shutter_speed = input("Enter a shutter speed:\n")
    #sensor.camera.shutter_speed = int(shutter_speed)
    #print(sensor.camera.shutter_speed)
    #plot.update(*peak.find_peaks(*sensor.grab_scan()))
    #plot.update(*peak.find_peaks(*sensor.grab_scan()))

#sensor.grab_scan_continuous()


