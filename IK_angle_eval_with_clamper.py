from gpiozero import Servo, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import time
import cv2

SERVO2_PIN = 16
SERVO3_PIN = 20
SERVO4_PIN = 21
factory = PiGPIOFactory()

servo2 = AngularServo(SERVO2_PIN, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023,pin_factory=factory)
servo3 = AngularServo(SERVO3_PIN, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023,pin_factory=factory)
clamper = AngularServo(SERVO4_PIN, min_angle=0, max_angle=90, min_pulse_width=0.0006, max_pulse_width=0.0023, pin_factory=factory)

# dof2 = 60
# dof3 = 105
# dw_dof3 = 300-dof3
# hpi_dof3 = 90+dof3


# def on_trackbar(val):
#     print("Tracbar value: ", val)
# cv2.namedWindow('TrackBars')
# cv2.resizeWindow('TrackBars', 1200, 600)
# cv2.createTrackbar("ClamperTime", "TrackBars", 2, 20, on_trackbar)

# ------------ Clamper ----------------
clamper_open_time = 2  # Time to keep clamper open (in seconds)
# clamper_close_time = cv2.getTrackbarPos('ClamperTime', "TrackBars")  # Time to keep clamper closed (in seconds)
clamper_close_time = 5

clamper_open = True  # Flag to indicate whether the clamper is currently open or closed
clamper_last_toggle_time = time.time()  # Initialize last toggle time

def ServoControlling(dof2, dw_dof3):

    try:
        for i in range(100):
            # clamper_close_time = cv2.getTrackbarPos('ClamperTime', "TrackBars")
            print("\n-------->servo2", dof2 ,"<-----------")
            servo2.angle = dof2
            time.sleep(1)

            print("\n----->servo3", pi_dof3, "<----------")
            pi_dof3 += 5
            servo3.angle = pi_dof3

            current_time = time.time()
            if clamper_open and current_time - clamper_last_toggle_time >= clamper_open_time:
                clamper.angle = 0  # Close clamper
                clamper_open = False
                clamper_last_toggle_time = current_time
            elif not clamper_open and current_time - clamper_last_toggle_time >= clamper_close_time:
                clamper.angle = 90  # Open clamper
                clamper_open = True
                clamper_last_toggle_time = current_time

            # status = input("is clamped ? ")
            # if(status=="Y" or status=="y"):
            #     servo2.angle = dof2 + 45
            #     time.sleep(4)
            #     servo3.angle = dof3 - 45
            #     time.sleep(4)
            #     break
            # else:
            #     continue

    finally:
        # Cleanup
        servo3.close()
        servo2.close()


