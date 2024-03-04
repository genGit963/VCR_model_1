from gpiozero import Servo, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

#------------------------- Ultrasonic setup ---------------

# ------------------------- Motor Setup -------------------
SERVO1_PIN = 26
SERVO2_PIN = 16
SERVO3_PIN = 20
SERVO4_PIN = 21
factory = PiGPIOFactory()

servo1 = AngularServo(SERVO1_PIN, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023,pin_factory=factory)
servo2 = AngularServo(SERVO2_PIN, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023,pin_factory=factory)
servo3 = AngularServo(SERVO3_PIN, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023,pin_factory=factory)
clamper = Servo(SERVO4_PIN, min_angle=0, max_angle=90, min_pulse_width=0.0006, max_pulse_width=0.0023, pin_factory=factory)

dof2 = 50 + 10
dof3 = 115 -10
pi_dof3 = 300-dof3
hpi_dof3 = 90+dof3



# ------------ Clamper ----------------
clamper_open_time = 2  # Time to keep clamper open (in seconds)
clamper_close_time = 4  # Time to keep clamper closed (in seconds)

clamper_open = True  # Flag to indicate whether the clamper is currently open or closed
clamper_last_toggle_time = time.time()  # Initialize last toggle time

try:
    for i in range(3):
        
        print("\n-------->servo-top<-----------") 
        clamper.angle = 90
        time.sleep(2)       
        servo1.angle = 100
        time.sleep(2)
        #dof2 += 5
        servo2.angle = dof2
        time.sleep(2)

        print("\n----->servo3", pi_dof3, "<----------")
        #pi_dof3 += 5
        servo3.angle = pi_dof3
        time.sleep(2)

        current_time = time.time()
        if clamper_open and current_time - clamper_last_toggle_time >= clamper_close_time:
            clamper.angle = 90  # Open clamper
            clamper_open = True
            clamper_last_toggle_time = current_time
            # clamped = input("Will it clamp? : ")
            time.sleep(2)
            clamper.angle = 0
            time.sleep(2)
            servo3.angle = 100
            time.sleep(2)
            servo2.angle = 100
            time.sleep(2)
            servo1.angle = 20 #dustbin
            time.sleep(2)
            servo3.angle = 170
            time.sleep(2)
            clamper.angle = 90 #open clamper
            time.sleep(2)
            print("Done !!")
            break
               

        elif clamper_open and current_time - clamper_last_toggle_time >= clamper_open_time:
            clamper.angle = 0  # Close clamper
            clamper_open = False
            clamper_last_toggle_time = current_time
            
                
finally:
    # Cleanup
    servo1.close()
    servo3.close()
    servo2.close()
    clamper.close()
