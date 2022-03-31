from cmath import nan
import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
import traceback
from cv_bridge import CvBridge
from pyzbar import pyzbar
import cv2
import time
import numpy as np
import pickle
import json

#############################################################################
################################### LOGGING #################################
#############################################################################


def get_timestamp():
  return time.strftime("%Y-%m-%d_%H:%M:%S")


f_name = f"FTL_{get_timestamp()}.pkl"
log_file = open(f_name, "wb")


def log(*msgs):
  msg = ' '.join([str(a) for a in msgs])
  pickle.dump(("LOG", msg), log_file)
  print(msg)


#############################################################################
################################ COMMUNICATION ##############################
#############################################################################
bridge = CvBridge()
rospy.init_node('flight')

get_telemetry_ros = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def get_telemetry(frame_id):
  try:
    res = get_telemetry_ros(frame_id=frame_id)

    pickle.dump(("TELEMETRY",
                 json.dumps({
                     "frame_id": res.frame_id,
                     "yaw": res.yaw,
                     "x": res.x,
                     "y": res.y,
                     "z": res.z,
                     "vx": res.vx,
                     "vy": res.vy,
                     "vz": res.vz
                 })), log_file)
    return res
  except Exception as e:
    log("ERROR: Exception while getting telemetry: {}".format(e))
    return None


def navigate_arrived(tolerance=0.2):
  telem = get_telemetry(frame_id='navigate_target')
  if telem is None:
    return False

  if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
    return True

  return False


def navigate_wait(x=0,
                  y=0,
                  z=0,
                  yaw=float('nan'),
                  yaw_rate=0,
                  speed=0.5,
                  frame_id='body',
                  tolerance=0.2,
                  auto_arm=False):

  res = navigate(x=x,
                 y=y,
                 z=z,
                 yaw=yaw,
                 yaw_rate=yaw_rate,
                 speed=speed,
                 frame_id=frame_id,
                 auto_arm=auto_arm)
  print(x, y, z)
  if not res.success:
    log("ERROR: Failed to navigate", res)
    raise Exception("Failed to navigate")

  while not rospy.is_shutdown():
    if navigate_arrived(tolerance):
      break
    rospy.sleep(0.2)


last_img = np.zeros((240, 320, 3))


def image_callback(ros_img):
  global last_img
  cv_image = bridge.imgmsg_to_cv2(ros_img, 'bgr8')
  last_img = cv_image


def get_frame():
  img = last_img.copy()
  encoded_image = cv2.imencode('.png', img)[1]
  pickle.dump(("IMG", encoded_image), log_file)
  return img


image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

#############################################################################
############################### ALGORITHM ###################################
#############################################################################



def detect_razliv(img):
  global was_razliv, oil_detect_pub
  try:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    orange = cv2.inRange(hsv, (3, 46, 57), (19, 146, 222))
    contours_razliv, _ = cv2.findContours(orange, cv2.RETR_TREE,
                                          cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_razliv) > 0:
      sorted_contours = sorted(contours_razliv,
                               key=cv2.contourArea,
                               reverse=True)
      telemetry = get_telemetry(frame_id='aruco_map')
      M = cv2.moments(sorted_contours[0])
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      razliv_debug = img.copy()
      rasliv_area = cv2.contourArea(sorted_contours[0])
      if rasliv_area > 20 and not was_razliv:
        was_razliv = 1
        print(f"Разлив топлива  x={telemetry.x} y={telemetry.y}")
        cv2.drawContours(razliv_debug, [sorted_contours[0]], 0, (255, 0, 0), 3)
        oil_detect_pub.publish(bridge.cv2_to_imgmsg(img, encoding="bgr8"))
  except Exception as e:
    log(f"Error in detect_razliv: {''.join(traceback.TracebackException.from_exception(e).format())}"
        )


def detect_destroided(img):
  try:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    line = cv2.inRange(hsv, (28, 55, 87), (65, 134, 248))
    line_img = cv2.bitwise_and(img, img, mask=line)
    line_img = cv2.inRange(line_img, (0, 0, 0), (60, 60, 60))
    contours_destroid, _ = cv2.findContours(line_img.copy(), cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
    correct_destroid = contours_destroid[0]
    print(len(contours_destroid))
    n = 0
    for contour in contours_destroid:
      draw = img.copy()
      cv2.drawContours(draw, contour, -1, (0 + n, 0 + n, 0 + n), 3)
      n += 40
      i = 0
      n = 0
      for point in contour:
        result = cv2.pointPolygonTest(contour,
                                      (round(point[0][0]), round(point[0][1])),
                                      False)
        n += 1
        i += result
      if i > 0:
        correct_destroid = contour
        break
  except Exception as e:
    log(f"Error in detect_destroided: {''.join(traceback.TracebackException.from_exception(e).format())}"
        )


def find_decode_qrcode(img):
  barcodes = pyzbar.decode(img)
  b_data = None
  for barcode in barcodes:
    b_data = barcode.data.decode("utf-8")
    log("DEBUG: Found qrcode, data: {}".format(b_data))
    b_data = [float(a) for a in b_data.split(" ")]
  return b_data


navigate_wait(z=0.5, frame_id='body', auto_arm=True)
rospy.sleep(1)

start_telemetry = get_telemetry(frame_id='aruco_map')
start_x = start_telemetry.x
start_y = start_telemetry.y
start_z = start_telemetry.z

while start_x == None or start_y == None or math.isnan(start_x) or math.isnan(
    start_y):
  start_telemetry = get_telemetry(frame_id='aruco_map')
  start_x = start_telemetry.x
  start_y = start_telemetry.y
  start_z = start_telemetry.z

fr = get_frame()
b_data = find_decode_qrcode(fr)

z = start_z
while b_data is None and z <= 2.3:
  time.sleep(2)
  z += 0.1

  navigate_wait(start_x, start_y, z, frame_id='aruco_map')
  rospy.sleep(0.2)

  img = get_frame()
  b_data = find_decode_qrcode(img)

if b_data is None:
  log("WARNING: No qrcode found using stub data")
  b_data = (2.5, 0.5)

log("Navigation area x={}, y={}".format(b_data[0], b_data[1]))
navigate_wait(x=b_data[0], y=b_data[1], z=1, frame_id='aruco_map')

time.sleep(5)
log("DEBUG: Start navigating...")


def dist(p1, p2):
  x1, y1 = p1
  x2, y2 = p2
  return ((x2 - x1)**2 + ((y2 - y1)**2))**0.5


oil_detect_pub = rospy.Publisher("/oil_detect", Image)

was_razliv = 0

# while True:

#   try:
#     img = get_frame()
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#     line = cv2.inRange(hsv, (28, 55, 87), (65, 134, 248))
#     # line_img = cv2.bitwise_and(cv_image, cv_image ,mask = line)
#     contours_blk, _ = cv2.findContours(line.copy(), cv2.RETR_TREE,
#                                        cv2.CHAIN_APPROX_SIMPLE)
#     contours_blk.sort(key=cv2.minAreaRect)
#     detect_destroided(img)
#     detect_razliv(img)
#     set_velocity(vx=0.1, vy=0, vz=0, yaw=float('nan'), yaw_rate=0)
#     if len(contours_blk) > 0:
#       cnt = contours_blk[0]
#       if cv2.contourArea(cnt) > 300:
#         rect = cv2.minAreaRect(cnt)
#         (x_min, y_min), (w_min, h_min), angle = rect
#         if angle < -45:
#           angle = 90 + angle
#         if w_min < h_min and angle > 0:
#           angle = (90 - angle) * -1
#         if w_min > h_min and angle < 0:
#           angle = 90 + angle
#         center = img.shape[1] / 2
#         error = x_min - center
#         # print(round(angle, 2), error)
#         set_velocity(vx=0.1,
#                      vy=error * (-0.01),
#                      vz=0,
#                      yaw=float('nan'),
#                      yaw_rate=angle * (-0.008))

#     else:
#       log("WARNING: Countours not found, going to landing")
#       break
#   except Exception as e:
#     log(f"Error in main loop: {''.join(traceback.TracebackException.from_exception(e).format())}"
#         )

log("DEBUG: Goind to landing")
navigate_wait(start_x, start_y, 1.5, frame_id='aruco_map')
log("DEBUG: Landing...")
land()
