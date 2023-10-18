from djitellopy import tello
from time import sleep
from droneblocksutils.aruco_utils import detect_markers_in_image, detect_distance_from_image_center
import cv2
import logging
LOGGER = logging.getLogger()
LOGGER.setLevel(logging.ERROR) # change to logging.DEBUG for more verbose logging

# Maximum speed sent to send_rc_control
MAX_SPEED = 50
MIN_DISTANCE = 80
MIN_AREA = 7000

def find_marker(drone, target_id):
    marker_details = None
    detected = False
    while not(detected):
    
        img = drone.get_frame_read().frame
        show_cam(img)
        _, marker_details = detect_markers_in_image(img, draw_center=True, draw_reference_corner=True,
                                                        target_id=target_id)
        print(marker_details)
        if len(marker_details)==0:
            drone.send_rc_control(0,0,0,-20)
            continue
        detected = True

    marker_center_x, marker_center_y = marker_details[0][0]

    marker_center_y = marker_center_y + 90
    
    return marker_center_x, marker_center_y
    
def drone_towrad_marker_center(drone, target_id, return_area = False):
    marker_center_x, marker_center_y = find_marker(drone, target_id)
    img = drone.get_frame_read().frame
    show_cam(img)
    img, dx, dy, d = detect_distance_from_image_center(img, marker_center_x, marker_center_y, show_center=True, show_detail=True, show_center_arrow=True)
    valid = False

    while d > MIN_DISTANCE:
        valid = True
        print("target: ", target_id)
        img = drone.get_frame_read().frame
        show_cam(img)

        _, marker_details = detect_markers_in_image(img, draw_center=True, draw_reference_corner=True,
                                                        target_id=target_id)
        print(marker_details)

        if len(marker_details) == 0:
            continue

        marker_center_x, marker_center_y = marker_details[0][0]
    
        marker_center_y = marker_center_y + 90
        img, dx, dy, d = detect_distance_from_image_center(img, marker_center_x, marker_center_y, show_center=True, show_detail=True, show_center_arrow=True)

        l_r_speed = int((MAX_SPEED * dx) / (W // 2))

        u_d_speed = int((MAX_SPEED * dy / (H // 2)) * -1)

        yaw_speed = int((MAX_SPEED * dx) / (W // 2))

        try:
            if d > 200: 
                drone.send_rc_control(int(l_r_speed / 3), 0, int(u_d_speed / 3), yaw_speed)
            # we are not close enough to the ArUco marker, so keep flying
            else: 
                drone.send_rc_control(l_r_speed, 0, u_d_speed, 0)

        except Exception as exc:
            LOGGER.error(f"send_rc_control exception: {exc}")
        
    if return_area and valid:
        return marker_details[0][2]
    
def drone_forward_using_area(drone, target_id):
    img = drone.get_frame_read().frame
    show_cam(img)
    _, marker_details = detect_markers_in_image(img, draw_center=True, draw_reference_corner=True,
                                                        target_id=target_id)
        
    area_of_marker = marker_details[0][2]
    
    print("area_of_marker: ", area_of_marker)
    
    while not((area_of_marker > MIN_AREA + 1000) and (area_of_marker < MIN_AREA - 1000)):

        difference_area = abs(MIN_AREA - area_of_marker)
        
        speed = int(difference_area / 95)

        if speed > 70:
            speed = 70
        elif speed < 20:
            speed = 20

        if area_of_marker < MIN_AREA + 1000:
            drone.move_forward(speed)
            print("speed: ", speed)

        elif area_of_marker > MIN_AREA - 1000:
            drone.move_back(speed)
            print("speed: ", speed)

        returned_area = drone_towrad_marker_center(drone, target_id, return_area=True) 
        if returned_area != None:
            area_of_marker = returned_area

        print("area_of_marker: ", area_of_marker)

        if area_of_marker != None:
            if (area_of_marker < MIN_AREA + 1000) and (area_of_marker > MIN_AREA - 1000):
                break

        img = drone.get_frame_read().frame
        show_cam(img)
        _, marker_details = detect_markers_in_image(img, draw_center=True, draw_reference_corner=True,
                                                            target_id=target_id)
        if len(marker_details) == 0:
                continue

        area_of_marker = marker_details[0][2]

        print("area_of_marker: ", area_of_marker)
        if (area_of_marker < MIN_AREA + 1000) and (area_of_marker > MIN_AREA - 1000):
            break

    
    drone.move_forward(30)
        
def show_cam(img):
    img = cv2.resize(img, (360, 240))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imshow("Tello Camera", img)
    cv2.waitKey(1)
    
if __name__ == "__main__":
    drone = tello.Tello()
    drone.connect()
    print(drone.get_battery())
    drone.streamon()
    # 처음에 딱 떠
    drone.takeoff()

    (H, W) = drone.get_frame_read().frame.shape[:2]
    sleep(2)
    # target 0을 찾고, 시점을 맞춰.
    drone_towrad_marker_center(drone, 0)
    sleep(2)
    # target 0 앞까지 가. 
    drone_forward_using_area(drone, 0)
    sleep(2)
    # target 1을 찾고, 시점을 맞춰.
    drone_towrad_marker_center(drone, 1)
    sleep(2)
    # target 1 앞까지 가. 
    drone_forward_using_area(drone, 1)
    sleep(2)
    drone.move_forward(30)
    # target 2을 찾고, 시점을 맞춰.
    drone_towrad_marker_center(drone, 2)
    # target 2 앞까지 가. 
    sleep(2)
    drone_forward_using_area(drone, 2)
    sleep(2)
    drone.rotate_counter_clockwise(3600)
    sleep(2)
    drone.land()