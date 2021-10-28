import rospy
from std_msgs.msg import Int16
import cv2
import numpy as np
from math import *
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

display = True

image_height = 308  # 308
image_width = 410  # 410

params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.filterByInertia = False
params.filterByConvexity = False
params.blobColor = 0
params.minArea = 20
params.maxArea = 2000
params.minCircularity = 0.0
params.maxCircularity = 1.0

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

blob_border = 4
blob_line_sectors = 10
blob_max_delta_angle = pi / 3


# TODO: Come up with and write a smarter filtering of the nearest points.
def get_chain_from_keypoints(keypoints):
    blob_height = 1.0 * image_height / blob_line_sectors

    def keypoints_first(keypoints_array):
        for keypoints_id in range(len(keypoints_array)):
            if len(keypoints_array[keypoints_id]) != 0:
                return keypoints_id
        return -1

    def find_nearest_upper_point(keypoints_array, start_y, start_x):
        start_y_new = None
        start_x_new = None
        for i in range(start_y, blob_line_sectors):
            for x in keypoints_array[i]:
                if (start_y_new is None or start_x_new is None) or \
                        ((i - start_y) * blob_height) ** 2 + (x - start_x) ** 2 < \
                        ((start_y_new - start_y) * blob_height) ** 2 + (start_x_new - start_x) ** 2:
                    start_y_new = i
                    start_x_new = x
        return [start_y_new, start_x_new]

    for i in range(len(keypoints)):
        keypoints[i].sort()
    keypoints = keypoints[::-1]

    if keypoints_first(keypoints) == -1:
        return []

    temp = find_nearest_upper_point(keypoints, keypoints_first(keypoints), image_width / 2.0)
    start_y_new = temp[0]
    start_x_new = temp[1]

    x_chain = []
    x_prev_2 = start_x_new
    x_prev = start_x_new
    for i in range(start_y_new):
        x_chain.append(x_prev)
    i = start_y_new
    while i < blob_line_sectors:
        x_arr_now = keypoints[i]

        x_arr_predict = 2 * x_arr_now - x_prev
        np.abs(x_arr_predict - x_arr_now)
        x_id_next = (np.argmin(np.abs(x_arr_predict - x_arr_now)) if len(x_arr_now) != 0 else -1)

        # TODO Stupidity place
        if len(x_arr_now) == 0 or abs(atan2(blob_height, x_arr_now[x_id_next] - x_prev) - \
                                      atan2(blob_height, x_prev - x_prev_2)) > blob_max_delta_angle:
            return x_chain
        else:
            x_prev_2 = x_prev
            x_prev = x_arr_now[x_id_next]
        x_chain.append(x_prev)
        i += 1
    return x_chain


def image_callback(msg):
    try:
        cv2_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  # np.ndarray
    except CvBridgeError, e:
        print(e)
        return
    cv2.flip(cv2_img, -1, cv2_img)
    if display:
        cv2.imshow('camera_image.jpeg', cv2_img)
    (T, thresh) = cv2.threshold(cv2.GaussianBlur(cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY), (5, 5), 0), 0, 255,
                                cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    if display:
        cv2.imshow("Otsu Thresholding", thresh)

    blob_height = 1.0 * thresh.shape[0] / blob_line_sectors

    # TODO: Do detection inverted color as well.
    blob_thresh = []
    for i in range(blob_line_sectors):
        blob_container = thresh[int(round(i * blob_height)):int(round((i + 1) * blob_height))]
        blob_container = cv2.copyMakeBorder(blob_container, top=blob_border, bottom=blob_border, left=blob_border,
                                            right=blob_border, borderType=cv2.BORDER_CONSTANT,
                                            value=255)
        blob_thresh.append(blob_container)
    keypoints = []
    for i in range(blob_line_sectors):
        keypoints.append([])
    for i in range(blob_line_sectors):
        new_keypoints = cv2.KeyPoint_convert(detector.detect(blob_thresh[i]))
        for j in range(len(new_keypoints)):
            keypoints[i].append(new_keypoints[j][0] - blob_border)
    keypoints = [np.array(keypoints[i], dtype=np.float32) for i in range(blob_line_sectors)]

    keypoints_old = keypoints

    chain_x = get_chain_from_keypoints(keypoints)

    if len(keypoints_old) > 0:
        im_with_keypoints = cv2_img.copy()
        for y_id in range(len(keypoints_old)):
            for x in keypoints_old[y_id]:
                point = (int(round(x)), int(round(y_id * blob_height + blob_height / 2)))
                cv2.circle(im_with_keypoints, point, 10, (0, 0, 255))
        for y_id in range(len(chain_x)):
            point = (int(round(chain_x[y_id])), int(round(image_height - y_id * blob_height - blob_height / 2)))
            cv2.circle(im_with_keypoints, point, 10, (0, 255, 0))
        if display:
            cv2.imshow("Keypoints", im_with_keypoints)

    angle_calc_k = 0.3

    error_dx = (np.array(chain_x[0], dtype=np.float32) - image_width / 2.0) / image_width * 2.0
    error_angle_calc_y_id = int(round(blob_line_sectors * angle_calc_k))
    error_angle_calc_y_id = min(len(chain_x) - 1, error_angle_calc_y_id)
    error_angle = atan2((error_angle_calc_y_id + 1) * blob_height,
                        chain_x[error_angle_calc_y_id] - chain_x[0]) - pi / 2

    control_callback(error_dx, error_angle, chain_x)

    cv2.waitKey(1)


control_pwm_max = 160
control_pwm_target = 160
control_dx_k_P = control_pwm_target * 1.0
control_angle_k_P = control_pwm_target * 2.5

motor_cmd_left = rospy.Publisher('/omegabot/cmd/motor/left', Int16, queue_size=1)
motor_cmd_right = rospy.Publisher('/omegabot/cmd/motor/right', Int16, queue_size=1)
servo_cmd = [rospy.Publisher('/omegabot/cmd/servo/{}'.format(i), Int16, queue_size=1) for i in range(1, 2 + 1)]

global servo_set
servo_set = False


def control_callback(error_dx, error_angle, chain_x):
    global servo_set
    control_dx = np.array([error_dx * control_dx_k_P, -error_dx * control_dx_k_P], dtype=np.float32)
    control_angle = np.array([-error_angle * control_angle_k_P, error_angle * control_angle_k_P], dtype=np.float32)
    control = control_dx + control_angle
    if control_pwm_target > np.max(control):
        control += control_pwm_target - np.max(control)
    control = np.clip(control, -control_pwm_max, control_pwm_max)

    if not servo_set:
        servo_cmd[0].publish(94)
        servo_cmd[1].publish(65)
        servo_set = True

    control = np.ndarray.astype(np.round(control), dtype=np.int)
    rospy.loginfo((error_dx, error_angle, control))

    if len(chain_x) > 1:
        pass
        motor_cmd_left.publish(control[0])
        motor_cmd_right.publish(control[1])
    else:
        motor_cmd_left.publish(0)
        motor_cmd_right.publish(0)


def main():
    rospy.init_node('image_listener')
    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, image_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
