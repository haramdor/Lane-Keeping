#Vesrion of 21.1.2021#
import cv2
import numpy as np
import picamera
from picamera.array import PiRGBArray
import serial #serial communication
from time import strftime
import time

###########################################################

## FUNCTION DEFINITION ####################################

# This function takes the linear lines that we found and make an avrrage of them and puts them back. So instead of few
# different direction lines we get just one. This one line will start at the bottom and will go 0.5 of the image size. 
def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    if slope > 0:
        x1 = int(640)
        x2 = int(x1*0.7)
        y1 = int(slope*x1 + intercept)
        y2 = int(slope*x2 + intercept)
    else:
        x1 = 0
        x2 = int(640*0.3)
        y1 = int(slope*x1 + intercept)
        y2 = int(slope*x2 + intercept)

    return np.array([x1, y1, x2, y2])

#This function execute a filter to the detected lines, so the only left and right lines that will be calculated in the
# following algorithm will only be the lines that are closest to the middle of the image.
def filter_lines(given_lines):
#This value defines which lines we will calculate and which will not.
    LIMIT_VALUE = 9
    cluster_array = []
    avg_array = []
    middle_of_frame_x = 320
    middle_of_frame_y = 480

    for line in given_lines:
        #This flag will allow us to see if we need to open a new bucket in the end of this inner loop
        line_added_to_bucket = False
        for index, bucket in enumerate(cluster_array):
            #Calculate the grade of this current line against the avg[index].
            grade_for_cur_bucket = calc_grade_for_bucket(line, avg_array, index)
            if (grade_for_cur_bucket < LIMIT_VALUE):
                #Since the grade made the condition add this line to the current bucket.
                bucket.append(line)
                #Update avg of the bucket now that we added a new line to it.
                avg_array[index][0] = (avg_array[index][0] * (len(bucket) - 1) + line[0]) / len(bucket)
                avg_array[index][1] = (avg_array[index][1] * (len(bucket) - 1) + line[1]) / len(bucket)
                #Update flag to show that we have found a bucket for this line, and do not need to make a new one.
                line_added_to_bucket = True
                #Breaks the inner loop since we already found a match.
                break
        #This condiotion will be met if we have not found a match in the loop above, hence we need to create a new bucket.
        if (not line_added_to_bucket):
            #Created new bucket by creating an array that contains this line (which means it looks like this:
            # [[[s1,i1],[s2,i2],.....],.....,[[slope,intercpt]]])
            cluster_array.append([line])
            #Creating an average cell for this new cluster.
            avg_array.append(list(line))

    #Select the best cluster and return it.
    best_score = 1000000
    best_bucket = 0
    for index, avg in enumerate(avg_array):
        score_of_cur_avg = abs((-1 * avg[0] * middle_of_frame_x + middle_of_frame_y - avg[1])) / (
            np.sqrt(avg[0] * avg[0] + 1))
        #Calculate the distance of the line from the middle and top pixel of the image
        if (score_of_cur_avg < best_score):
            best_score = score_of_cur_avg
            best_bucket = index
    if (not cluster_array):
        return []
    return cluster_array[best_bucket]

def calc_grade_for_bucket(line, avg_array, index):
    slope_grade = np.abs(line[0] - avg_array[index][0])
    intercept_grade = np.abs(line[1] - avg_array[index][1])
    return (slope_grade + intercept_grade)/2

#This function gets all the lines that hough lines finds. Hough lines finds all the linear lines in the image.
def average_slope_intercept(image, lines, averaged_lines, previous_lines):
    left_fit = []
    right_fit = []
    garbage = []
    if lines is None:
        return previous['lines'], previous['error']

    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1,x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if 0.3 > slope > -0.3:
            garbage.append((slope, intercept)) #bad lines goes to the garbage
        elif slope < 0:
            left_fit.append((slope, int(intercept)))#finds left lines
        else:
            right_fit.append((slope, int(intercept)))#finds right lines
    left_fit = filter_lines(left_fit)
    right_fit = filter_lines(right_fit)

#Here we have many cases for errors
    #If there is no detection for the left line, takes the last left line.
    if not left_fit:
        left_line = averaged_lines[0]
        left_fit_average = previous['left_average']
    else:
        left_fit_average = np.average(left_fit, axis= 0)
        left_line = make_coordinates(image, left_fit_average)
        previous['left_average'] = left_fit_average

    #If there is no detection for the right line, takes the last right line.
    if not right_fit:
        right_line = averaged_lines[1]
        right_fit_average = previous['right_average']
    else:
        right_fit_average = np.average(right_fit, axis= 0)
        right_line = make_coordinates(image, right_fit_average)
        previous['right_average'] = right_fit_average

#Here the error is defined
    middle_line, nose_angle = mid_line(left_fit_average, right_fit_average, image)
    error_distance = distance2line(image, left_fit_average, right_fit_average, middle_line)
#    error_distance = logi(error_distance)
    previous['lines'] = [left_line, right_line, middle_line]
    return [np.array(previous['lines']), error_distance, nose_angle, middle_line, left_fit, right_fit]

#Here we aplly 'Lan' function on the error signal
def logi(error):
    if error < 0:
        error = error * (-1)
        error = np.log(error)
        error = error * (-1)
    else:
        error = np.log(error)
    return error

#This function calculates the middle line
def mid_line(left_line, right_line, image):
#Here we calculate the intercept of the left line and right line
    x_inter = (left_line[1] - right_line[1]) / (right_line[0] - left_line[0])
    y_inter = left_line[0] * x_inter + left_line[1]
#Here we calculate the angle between the left line and the right line
    theta = np.abs(np.arctan((left_line[0] - right_line[0]) / (left_line[0]*right_line[0] + 1)))
#Here we calculate the bisection
    half_tan_theta = np.tan(np.pi - theta/2)
#For defining the middle line in cartesian method, we do this with the slope of the line, and the intercept of the line with the Y axis.
    mid_slope = -1*(right_line[0] - half_tan_theta) / (right_line[0] * half_tan_theta + 1)
    mid_inter = y_inter - mid_slope * x_inter
#Here there is a definition of the actual line that will display
    y1 = 480
    x1 = int((y1 - mid_inter) / mid_slope)
    y2 = int(0.6 * y1)
    x2 = int((y2 - mid_inter) / mid_slope)
#The nose angle of the car with respect to the lane is defined in here. This angle is actually the error that we are look for
    nose_angle = np.arctan(1 / mid_slope)
    return [np.array([x1, y1, x2, y2]), nose_angle]

#This function calculates the distance from the two sided lines (left & right) to the point of the middle line that is
# at the bottom of the image (y=480)
def distance2line(image, left_line, right_line, middle_line):
    #x = int(640 / 2)
    y = middle_line[1]
    x = middle_line[0]
    left_distance = abs((-1 * left_line[0] * x + y - left_line[1])) / (np.sqrt(left_line[0] * left_line[0] + 1))
    right_distance = abs((-1 * right_line[0] * x + y - right_line[1])) / (np.sqrt(right_line[0] * right_line[0] + 1))
    error = left_distance - right_distance
    return error

# This function changes an image to gray scale, than applies a gaussian blur and canny kernel on it
def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)#gives only B&W
    blur = cv2.GaussianBlur(gray, (5,5), 0) #bluring the image to not confusing
    canny = cv2.Canny(blur, 50, 150)
    return canny

# This function makes a new image of zeroes, then it applies to itself the lines that were defiend by Hough.
def display_lines(image, lines):
    line_image= np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(line_image, (x1,y1), (x2,y2), (0, 0, 255), 10)
    return line_image

# This function makes a triangle polygon, in out region of interest. Out side this polygon - all will be black.
# Inside this polygon - all will be white. After that it combines the real image and the polygon to finally show
# just our region of interst of our image
def region_of_interest(image):
    height = image.shape[0]
    width = image.shape[1]
    polygons = np.array([
    [(0,height), (0, 240), (130, 200), (510, 200), (width,200), (width, height)]
    ])
    #(0,height) is the left-down corner.
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image
    
def interpolate(a1, a2, poly_deg = 2, n_points = 100):
    min_a1_y, max_a1_y = min(a1[0,:]), max(a1[0,:])
    new_a1_y = np.linspace(min_a1_y, max_a1_y, n_points)
    a1_coef = np.polyfit(a1[0,:], a1[1,:], poly_deg)
    new_a1_x = np.polyval(a1_coef, new_a1_y)

    min_a2_y, max_a2_y = min(a2[0,:]), max(a2[0,:])
    new_a2_y = np.linspace(min_a2_y, max_a2_y, n_points)
    a2_coef = np.polyfit(a2[0,:], a2[1,:], poly_deg)
    new_a2_x = np.polyval(a2_coef, new_a2_y)

    midx = [np.mean([new_a1_x[i], new_a2_x[i]]) for i in range(n_points)]
    midy = [np.mean([new_a1_y[i], new_a2_y[i]]) for i in range(n_points)]

    mid_line = np.array([[x,y] for x, y in zip(midx, midy)])
    return mid_line

#Calculate the mean of how many errors that 'loop' is equal to
def mean_errors(error, prev_errors, loop):
    if loop < pulse_softner:
        prev_errors[loop] = error
        mean_prev_errors = np.mean(prev_errors)
        loop = loop + 1
    else:
        prev_errors[0] = error
        mean_prev_errors = np.mean(prev_errors)
        loop = 0
    return mean_prev_errors, loop

#This function defines the angle error that will be sent to the function PID_angle
def angle_compare(averaged_lines):
    left_slope = (averaged_lines[0,3] - averaged_lines[0,1]) / (averaged_lines[0,2] - averaged_lines[0,0])
    right_slope = (averaged_lines[1,3] - averaged_lines[1,1]) / (averaged_lines[1,2] - averaged_lines[1,0])
    angle_error = left_slope + right_slope
    return angle_error

#If this function is called (at the For-Loop), then a video window will appear on the RPI screen when the car will be driven.
def showing_the_video(rotated_image, line_image, cropped_image):
#Combines the two images together
    combo_image = cv2.addWeighted(rotated_image, 0.8, line_image, 1, 1)
    result.write(combo_image)
#Here it is possible to change 'combo_image' to 'cropped_image' in order to see only the region of interest
    cv2.imshow('result', combo_image)

#PID function: It uses the location error that the algorithm recieves (in the veriable called 'error')
def PID(error, iteration_time, error_prior, Kp, Kd):
#Here we defines the derivative as a discrete time signal.
    derivative = (error - error_prior) / iteration_time
#Here we process the signal: The constant defines the straight wheels angle. According to the error the angle of the wheels
# veries from being smaller than this constant (the wheels turn left), or bigger than it (the wheels turn right).
    output = int(78 + Kp * error + Kd * derivative)
#For some reason, if the output gets 0 value, it makes troubles
    if output == 0: output = 1
    error_prior = error
    return [output, error_prior]

#PID function: It uses the angle error that the algorithm recieves (in the veriable called 'angle_error')
def PID_angle(angle_error, iteration_time, last_angle_error, Kp , Kd):
    derivative = (angle_error - last_angle_error) / iteration_time
    output = int(78 + Kp * error + Kd * derivative)
#For some reason, if the output gets 0 value, it makes troubles
    if output == 0: output = 1
    last_angle_error = angle_error
    return [output, last_angle_error]

#This function recieves an integer that was calculated to be the final steering angle
def sendInt(servo_angle):
#This integer has to be a string in order to attach the markers <an##> to it
    send_string = str(servo_angle)
    send_string += ">\n"
    send_string = "<an" + send_string
#Transforming the full string to 8-bits, because this is how serial communication is preformed.
    ser.write(send_string.encode('utf-8'))
    ser.flush()


## GENERAL SETUP ###########################################
averaged_lines = np.array([[0, 281, 322, 180],
                            [1074, 334, 751, 183]])
previous = {
        'lines' : [],
        'left_average' : [-0.313326, 281.70313],
        'right_average' : [0.46858, -168.3927],
        'error' : []
    }
iteration_time = 0.2
PID_signal = 78
error_prior = 0
pulse_softner = 2
prev_errors = 78 * (np.ones((pulse_softner), dtype = int))
loop = 3
last_angle_error = 0

## CONNECTION WITH ARDUINO SETUP############################
port0 = "/dev/ttyACM0"
port1 = "/dev/ttyACM1"
baudrate = 115200
ser = serial.Serial(port0, baudrate, timeout = 1)
ser.flush()

## CAMERA AND VIDEO SETUP #################################
camera = picamera.PiCamera()
size = (640, 480)
camera.resolution = size
#camera.framerate = 30
#Defines color scales
rawCapture = PiRGBArray(camera)
time.sleep(1)
#The codec is defined
fourcc = cv2.VideoWriter_fourcc('M','J', 'P', 'G')
result = cv2.VideoWriter('from_picamera.avi', fourcc, 30, size)#, isColor = False) - for B&W videos

###########################################################

## MAIN ###################################################
#The loop continues as long as the camera is operated
for frame in camera.capture_continuous(rawCapture, format = 'bgr', use_video_port = True):
    #For knowing how much time it takes for each iteration - here the count is started
    start_time = time.time()
    image = frame.array
    #The camera is installed upside down, so there is a need to rotate the image
    rotated_image = cv2.rotate(image, cv2.ROTATE_180)
    canny_image = canny(rotated_image)
    cropped_image = region_of_interest(canny_image)
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength= 40, maxLineGap= 50) #minlinelenght can be changed so the RPI will know what is the minimum lenght that will be detected.
    #Here we have the critical calculations for the nose angle & error, and averaged_lines. averaged_lines have 6 elements that gives information about the right, left, and the middle lines.
    try: averaged_lines, error, nose_angle, middle_line, left_fit, right_fit = average_slope_intercept(frame, lines, averaged_lines, previous)
    except: break
    line_image = display_lines(rotated_image, averaged_lines)
    showing_the_video(rotated_image, line_image, cropped_image)
    angle_error = angle_compare(averaged_lines)
    rawCapture.truncate(0)
    with open("/home/pi/Documents/Python/time_error.csv", "a") as log:
        log.write("{0},{1}\n".format(strftime('%H%M%S.%f')[:-3],str(error)))
    mean_prev_errors, loop = mean_errors(error, prev_errors, loop)
    #For knowing how much time it takes for each iteration - here the count is stopped
    duration = time.time() - start_time
    #Use PID function that generates the signal from the location error
#    PID_signal, error_prior = PID(error, iteration_time, error_prior, Kp, Kd)
    #Use PID function that generates the signal from the angle error
    PID_signal, error_prior = PID_angle(error, duration, error_prior, Kp, Kd)

    print('The steering angle is ' + str(PID_signal))
    sendInt(PID_signal)
    if cv2.waitKey(1) == ord('q'):
        break
camera.close()
result.release()
cv2.destroyAllWindows()
sendInt('quit') #A special sign is sent to the Arduino, so it will stop the DC motor and turn the wheels to be straight
ser.close()