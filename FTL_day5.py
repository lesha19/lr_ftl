import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge
from pyzbar import pyzbar

# Иницируем ноду
rospy.init_node('flight')
bridge = CvBridge()

# Получаем нужные сервисы
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


# Вспомогательная функция для навигации
def navigate_wait(x=0,
                  y=0,
                  z=0,
                  yaw=float('nan'),
                  speed=0.5,
                  frame_id='',
                  auto_arm=False,
                  tolerance=0.2):
  print(f"Navigating to {x} {y} {z} of {frame_id}")
  navigate(x=x,
           y=y,
           z=z,
           yaw=yaw,
           speed=speed,
           frame_id=frame_id,
           auto_arm=auto_arm)

  while not rospy.is_shutdown():
    telem = get_telemetry(frame_id='navigate_target')
    if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
      break
    rospy.sleep(0.2)
  rospy.sleep(0.6)
  print("Arrived to destination")


# Вспомогательная функция для посадкаи
def land_wait():
  land()
  while get_telemetry().armed:
    rospy.sleep(0.2)


# Получение нового кадра
def get_frame():
  ros_img = rospy.wait_for_message("/main_camera/image_raw", Image)
  cv_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
  return cv_img


navigate_wait(x=0, y=0, z=0.5, frame_id='body', auto_arm=True)  # Взлёт
rospy.sleep(2)
# Пробуем получить стартовые кординаты
start_telemetry = None
while start_telemetry is None or math.isnan(start_telemetry.x):
  start_telemetry = get_telemetry(frame_id='aruco_map')
  rospy.sleep(0.2)

print("Got start telemetry", start_telemetry.x, start_telemetry.y,
      start_telemetry.z)


def find_decode_qrcode(img):
  barcodes = pyzbar.decode(img)
  b_data = None
  for barcode in barcodes:
    b_data = barcode.data.decode("utf-8")
    print("DEBUG: Found qrcode, data: {}".format(b_data))
    b_data = [float(a) for a in b_data.split()]
  return b_data


# Пробуем распознать qr код
qrcode = find_decode_qrcode(get_frame())
current_z = start_telemetry.z
while qrcode is None:
  current_z += 0.2
  if current_z > 2.3:
    current_z = start_telemetry.z
  navigate_wait(x=start_telemetry.x,
                y=start_telemetry.y,
                z=current_z,
                frame_id='aruco_map')

  qrcode = find_decode_qrcode(get_frame())

print('--------------------------------')
print("QRCode data:")
print(f"Navigation area: {qrcode[0]} {qrcode[1]}")
print(f"Lake area: {qrcode[2]} {qrcode[3]}")
print('--------------------------------')

navigate_wait(x=start_telemetry.x,
              y=start_telemetry.y,
              z=start_telemetry.z,
              frame_id='aruco_map')

# Летим к старту линии
print("Going to navigation area")
navigate_wait(x=qrcode[0],
              y=qrcode[1],
              z=start_telemetry.z,
              frame_id='aruco_map')
print("Arrived to navigation area")
rospy.sleep(5)

# Летим к озеру
print("Navigating to lake")
navigate_wait(x=qrcode[2],
              y=qrcode[3],
              z=start_telemetry.z,
              frame_id='aruco_map')
rospy.sleep(1)
# Опускаемся над озером
range_data = rospy.wait_for_message('rangefinder/range', Range).range
navigate_wait(x=0, y=0, z=(0.6 - range_data), frame_id='body')

print("Arrived to lake")
rospy.sleep(6)
print('--------------------------------')
print("Successful water withdrawal")
print('--------------------------------')
rospy.sleep(1)

print("Navigating to landing")
# Поднимаемся с озера
navigate_wait(x=0, y=0, z=(range_data - 0.6), frame_id='body')

navigate_wait(x=qrcode[2],
              y=qrcode[3],
              z=start_telemetry.z,
              frame_id='aruco_map')

# Летим обратно
navigate_wait(x=start_telemetry.x,
              y=start_telemetry.y,
              z=start_telemetry.z,
              frame_id='aruco_map')
rospy.sleep(2)
print("Arrived to landing")
land_wait()  # Посадка
print("Landing successful")