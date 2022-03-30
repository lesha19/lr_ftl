import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar
import cv2
import os
import time
# f_name = "FTL_" + time.strftime("%Y-%m-%d_%H:%M:%S")
# os.system(f"mkdir '{f_name}'")


bridge = CvBridge()
rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), yaw_rate=0, speed=0.5, \
        frame_id='body', tolerance=0.2, auto_arm=False):

    res = navigate(x=x, y=y, z=z, yaw=yaw, yaw_rate=yaw_rate, speed=speed, \
        frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)



img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
start = get_telemetry(frame_id='aruco_map')
start_coords = [start.x, start.y]
navigate_wait(z=0.5, frame_id='body', auto_arm=True)

x, y = 0, 0
barcodes = pyzbar.decode(img)
for barcode in barcodes:
    b_data = barcode.data.decode("utf-8")
    x, y = [float(a) for a in b_data.split(" ")]

z = 1.5
print((x == 0 and y == 0) and z <= 2.3)
while (x == 0 and y == 0) and z <= 2.3:
    time.sleep(2)
    z+=0.1
    # navigate_wait(z=0.1, frame_id='body')
    
    navigate_wait(1.62, 1.42, z, frame_id='aruco_map')
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    barcodes = pyzbar.decode(img)
    # print("\n\n\nimportant ", barcodes)
    for barcode in barcodes:
        b_data = barcode.data.decode("utf-8")
        x, y = [float(a) for a in b_data.split(" ")]
    # cv2.imwrite(f_name+ "/qr-" + time.strftime("%Y-%m-%d_%H:%M:%S")+ ".jpg", img)

print("Navigation area x={}, y={}".format(x, y))
if x == 0 and y == 0:
    navigate_wait(2.5, 0.5, 1, frame_id='aruco_map')
else:
    navigate_wait(x, y, 1, frame_id='aruco_map')
# cv2.imwrite(f_name+ "/qr-" + time.strftime("%Y-%m-%d_%H:%M:%S")+ ".jpg", img)
# navigate_wait(2.5, 2.5, 2, frame_id='aruco_map')
# cv2.imwrite(f_name+ "/qr-" + time.strftime("%Y-%m-%d_%H:%M:%S")+ ".jpg", img)
# navigate_wait(x = 1,y=1, z = 2, frame_id='aruco_map')
# os.system(f"zip -r FTL.zip '{f_name}'")
time.sleep(5)
print("Start navigating...")


def dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return ( (x2 - x1 ) **2  + ((y2-y1)**2) )**0.5

# image_pub = rospy.Publisher("/oil_detect", Image)

def image_callback(data):
    global curr_save
    try:
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        orange = cv2.inRange(hsv, (3, 46, 57), (19, 146, 222))
        line = cv2.inRange(hsv, (28, 55, 87), (65, 134, 248))
        # line_img = cv2.bitwise_and(cv_image, cv_image ,mask = line)
        contours_blk, _ = cv2.findContours(line.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk.sort(key=cv2.minAreaRect)
        # cv2.imwrite(f_name+ "/" + time.strftime("%Y-%m-%d_%H:%M:%S")+ ".jpg", cv_image)
        # curr_save += time.strftime("%Y-%m-%d_%H:%M:%S") + "  "
        # curr_save += f'{telemetry.x, telemetry.y, telemetry.z}' + "\n"
        # os.system(f"zip -r FTL.zip '{f_name}'")
        # backup(curr_save)




        # line_img = cv2.bitwise_and(img, img ,mask = line)
        # line_img = cv2.inRange(line_img, (0, 0, 0), (60, 60, 60))
        # contours_destroid, _ = cv2.findContours(line_img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # correct_destroid = contours_destroid[0]
        # print(len(contours_destroid))
        # n = 0
        # for contour in contours_destroid:
        #     draw = img.copy()
        #     cv2.drawContours(draw, contour, -1, (0+ n,0  + n,0+ n), 3)
        #     n+=40
        #     i = 0
        #     n = 0
        #     for point in contour:
        #         result = cv2.pointPolygonTest(contour, (round(point[0][0]), round(point[0][1])), False)
        #         n+=1
        #         i += result
        #     if i > 0:
        #         correct_destroid = contour
        #         break


        contours_razliv, _ = cv2.findContours(orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours_razliv) > 0:
            sorted_contours= sorted(contours_razliv, key=cv2.contourArea, reverse= True)
            telemetry = get_telemetry(frame_id='aruco_map')
            M = cv2.moments(sorted_contours[0])
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            if dist((cX,cY), (cv_image.shape[1],cv_image.shape[0])) < 40:
                print(f"Разлив топлива x={telemetry.x} y={telemetry.y}")
                res = cv2.drawContours(cv_image.copy(), [sorted_contours[0]], 0, (255,0,0), 3)
                # image_pub.publish(res)
        set_velocity(vx=0.1, vy=0, vz=0, yaw=float('nan'), yaw_rate=0, frame_id='aruco_map')
        if len(contours_blk) > 0:
            cnt = contours_blk[0]
            if cv2.contourArea(cnt) > 300:
                rect = cv2.minAreaRect(cnt)
                (x_min, y_min), (w_min, h_min), angle = rect
                if angle < -45:
                    angle = 90 + angle
                if w_min < h_min and angle > 0:
                    angle = (90 - angle) * -1
                if w_min > h_min and angle < 0:
                    angle = 90 + angle

                center = cv_image.shape[1] / 2
                error = x_min - center
                # print(round(angle, 2), error)
                set_velocity(vx=0.1, vy=error*(-0.01), vz=0, yaw=float('nan'), yaw_rate=angle*(-0.008), frame_id='aruco_map')

    except Exception as e:
        print(e)

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

print('Land')
navigate_wait(start_coords[0], start_coords[1], 1.5, frame_id='aruco_map')
land()
