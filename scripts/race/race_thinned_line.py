import rospy
from std_msgs.msg import Int16
import time
import cv2
import numpy as np
from math import *
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

from skimage.morphology import *

bridge = CvBridge()

"""
RASPI = False
DEBUG = True
RAW = True

RASPI = True
DEBUG = False
RAW = True
"""
RASPI = False
DEBUG = True
RAW = True

debug_skeletonize = True
debug_lines_hor_ver = False

image_compress_factor = 3.0

# adaptive_threshold_param_1 = 21
# adaptive_threshold_param_2 = 10

bordersize = 3
blur_kernel = (11, 11)  # (11, 11)
blur_k = 5  # 8

horizontal_morphological_filter_size = 3
horizontal_dilate_filter_size = (6, 6)
vertical_morphological_filter_size = 3
vertical_dilate_filter_size = (6, 6)

height_sectors_min_area = 10
height_sectors_len = 2

global last_time, fps_counter, processing_time
processing_time = 0.0
fps_timeout = 1.0
last_time = 0.0
fps_counter = 0


def skeletonize(cv2_img):
    image_width = np.shape(cv2_img)[1]
    image_height = np.shape(cv2_img)[0]

    cv2_img_blured = cv2.GaussianBlur(cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY), blur_kernel, blur_k)
    cv2_img_with_border = cv2.copyMakeBorder(cv2_img_blured,
                                             top=bordersize,
                                             bottom=bordersize,
                                             left=bordersize,
                                             right=bordersize,
                                             borderType=cv2.BORDER_REPLICATE)

    (T, thresh) = cv2.threshold(cv2_img_with_border, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    # (T, thresh) = cv2.threshold(cv2_img_with_border, 80, 255, cv2.THRESH_BINARY_INV)
    # thresh = cv2.adaptiveThreshold(cv2_img_with_border, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 6)

    if debug_skeletonize:
        cv2.imshow('cv2_img_compressed', cv2.resize(cv2_img, (image_width, image_height)))
        cv2.imshow('cv2_img_blured', cv2.resize(cv2_img_blured, (image_width, image_height)))
        cv2.imshow("Otsu Thresholding", cv2.resize(thresh, (image_width, image_height)))

    skeletonized = cv2.ximgproc.thinning(thresh, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
    skeletonized = skeletonized[bordersize:np.shape(cv2_img)[0] + bordersize,
                   bordersize:np.shape(cv2_img)[1] + bordersize]
    if debug_skeletonize:
        cv2.imshow("skeletonize", cv2.resize(skeletonized, (image_width, image_height)))

    return skeletonized


def max_pooling(skeletonized):
    (numLabels, labels, stats, centroids) = cv2.connectedComponentsWithStats(skeletonized, connectivity=8,
                                                                             ltype=cv2.CV_32S)

    if len(stats) < 2:
        labels = np.zeros(np.shape(labels), dtype=skeletonized.dtype)
    else:
        area = np.array(stats)[:, 4]
        max_area_id = np.argmax(area)
        area[max_area_id] = -1
        max_area_id = np.argmax(area)
        labels = (labels == max_area_id)
        labels = np.uint8(np.clip(labels, 0, 255) * 255)

    return labels


def image_callback(msg):
    global last_time, fps_counter, processing_time, control_last
    time_start_timer = time.time()
    try:
        if RAW:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")  # np.ndarray
        else:
            cv2_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  # np.ndarray
    except CvBridgeError, e:
        print(e)
        return

    cv2.flip(cv2_img, -1, cv2_img)
    if DEBUG:
        cv2.imshow('cv2_img', cv2_img)

    image_width = np.shape(cv2_img)[1]
    image_height = np.shape(cv2_img)[0]
    image_width_compressed = int(round(image_width / image_compress_factor))
    image_height_compressed = int(round(image_height / image_compress_factor))

    cv2_img_compressed = cv2.resize(cv2_img, (image_width_compressed, image_height_compressed),
                                    interpolation=cv2.INTER_CUBIC)

    skeletonized = skeletonize(cv2_img_compressed)
    skeletonized = max_pooling(skeletonized)
    if DEBUG:
        cv2.imshow("skeletonized", cv2.resize(skeletonized, (image_width, image_height)))

    horizontal = skeletonized.copy()
    horizontal_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (horizontal_morphological_filter_size, 1))
    cv2.erode(horizontal, horizontal_structure, dst=horizontal)
    cv2.dilate(horizontal, horizontal_structure, dst=horizontal)

    vertical = skeletonized.copy()
    vertical_structure = cv2.getStructuringElement(cv2.MORPH_RECT, (1, vertical_morphological_filter_size))
    cv2.erode(vertical, vertical_structure, dst=vertical)
    cv2.dilate(vertical, vertical_structure, dst=vertical)

    horizontal_expanded = horizontal.copy()
    cv2.dilate(horizontal_expanded, cv2.getStructuringElement(cv2.MORPH_OPEN, horizontal_dilate_filter_size),
               dst=horizontal_expanded)

    vertical_expanded = vertical.copy()
    cv2.dilate(vertical_expanded, cv2.getStructuringElement(cv2.MORPH_OPEN, vertical_dilate_filter_size),
               dst=vertical_expanded)

    skeletonize_vertical = np.bitwise_and(skeletonized, np.bitwise_not(horizontal_expanded))
    skeletonize_horizontal = np.bitwise_and(skeletonized, np.bitwise_not(vertical_expanded))

    crossroads = np.bitwise_and(horizontal_expanded, vertical_expanded)

    vertical_line = np.bitwise_or(skeletonize_vertical, crossroads)
    vertical_line = cv2.ximgproc.thinning(vertical_line, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN)
    vertical_line = max_pooling(vertical_line)

    sectors_height = 1.0 * vertical_line.shape[0] / height_sectors_len
    vertical_lines = []
    for i in range(height_sectors_len):
        vertical_lines.append(vertical_line[int(round(i * sectors_height)):int(round((i + 1) * sectors_height))])

    if debug_lines_hor_ver:
        cv2.imshow("horizontal", cv2.resize(horizontal, (image_width, image_height)))
        cv2.imshow("vertical", cv2.resize(vertical, (image_width, image_height)))
        cv2.imshow("horizontal_expanded", cv2.resize(horizontal_expanded, (image_width, image_height)))
        cv2.imshow("vertical_expanded", cv2.resize(vertical_expanded, (image_width, image_height)))
        cv2.imshow("skeletonize_vertical", cv2.resize(skeletonize_vertical, (image_width, image_height)))
        cv2.imshow("skeletonize_horizontal", cv2.resize(skeletonize_horizontal, (image_width, image_height)))
        cv2.imshow("crossroads", cv2.resize(crossroads, (image_width, image_height)))
    if DEBUG:
        cv2.imshow("vertical_line", cv2.resize(vertical_line, (image_width, image_height)))

    contours_vertical_lines = []
    if RASPI:
        for i in range(height_sectors_len):
            contours_vertical_line = cv2.findContours(vertical_lines[i], cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            contours_vertical_lines.append(contours_vertical_line[1])
    else:
        for i in range(height_sectors_len):
            contours_vertical_line, hierarchy_vertical_line = cv2.findContours(vertical_lines[i], cv2.RETR_LIST,
                                                                               cv2.CHAIN_APPROX_NONE)
            contours_vertical_lines.append(contours_vertical_line)

    contours_vertical_lines_all = []
    for i in range(height_sectors_len):
        contours_vertical_lines_all.append([])
        for contour in contours_vertical_lines[i]:
            contours_vertical_lines_all[i].extend(contour)

    sectors = []
    if DEBUG:
        image_with_line = cv2_img_compressed.copy()
    for i in range(height_sectors_len):
        if len(contours_vertical_lines_all[i]) < height_sectors_min_area:
            sectors.append([np.inf, np.inf, np.inf])
            continue
        [vx, vy, x, y] = cv2.fitLine(np.float32(contours_vertical_lines_all[i]), cv2.DIST_L2, 0, 0.01, 0.01)
        line_angle = atan2(vy[0], vx[0]) + pi / 2
        while line_angle < pi / 2:
            line_angle += pi
        while line_angle > pi / 2:
            line_angle -= pi
        y[0] += int(round(i * sectors_height))

        sectors.append([2.0 * (x[0] / image_width_compressed - 0.5), 1.0 - y[0] / image_height_compressed,
                        -line_angle])  # vx[0], vy[0], x[0], y[0], line_angle -> x[0], y[0], line_angle
        if DEBUG:
            x1 = int(x[0] + image_width / height_sectors_len / image_compress_factor / 2 * (-np.sin(line_angle)))
            y1 = int(y[0] + image_width / height_sectors_len / image_compress_factor / 2 * (np.cos(line_angle)))
            x2 = int(x[0] - image_width / height_sectors_len / image_compress_factor / 2 * (-np.sin(line_angle)))
            y2 = int(y[0] - image_width / height_sectors_len / image_compress_factor / 2 * (np.cos(line_angle)))
            cv2.line(image_with_line, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.circle(image_with_line, (x, y), 1, (0, 0, 255))
    sectors = sectors[::-1]
    if DEBUG:
        cv2.imshow("image_with_vertical_lines", cv2.resize(image_with_line, (image_width, image_height)))

    if DEBUG:
        cv2.waitKey(1)

    control(sectors)

    fps_counter += 1
    time_now = time.time()
    processing_time += time_now - time_start_timer
    if time_now > last_time + fps_timeout:
        rospy.loginfo("fps: {:.3f}\t| proc: {:2.2%}\t|{}".format(fps_counter / (time_now - last_time),
                                                                 processing_time / (time_now - last_time),
                                                                 control_last))
        fps_counter = 0
        processing_time = 0.0
        last_time = time_now


def control(sectors):
    [error_dx, error_angle, pwm_target, error_normal] = control_calc(sectors)

    control_callback(error_dx, error_angle / pi, pwm_target, error_normal, True)


control_pwm_max = 220
control_pwm_lost = 200

control_pwm_forward = [0, 120, 220]
control_dx_windup = [0, 120, 220]
control_angle_windup = [0, 220, 0]

control_dx_k_P = [0, 1.2 * control_pwm_max, 1.2 * control_pwm_max]  # 255 is 1
control_dx_k_D = [0, 0.0 * control_pwm_max, 0.0 * control_pwm_max]  # 255 is 1
control_dx_k_I = [0, 0.0 * control_pwm_max, 0.0 * control_pwm_max]  # 255 is 1
control_dx_k_I_windup = [0, 0, 0]  # 1 is 1
control_angle_k_P = [0, 3.0 * control_pwm_max, 0.0 * control_pwm_max]  # 255 is 1

control_low_pass_kilter_k = 0.0

motor_cmd_left = rospy.Publisher('/omegabot/cmd/motor/left', Int16, queue_size=1)
motor_cmd_right = rospy.Publisher('/omegabot/cmd/motor/right', Int16, queue_size=1)
servo_cmd = [rospy.Publisher('/omegabot/cmd/servo/{}'.format(i), Int16, queue_size=1) for i in range(1, 2 + 1)]


def control_calc(sectors):
    if len(sectors) == 0:
        return [0, 0, 0, []]

    error_normal = []
    for sector_id in range(len(sectors)):
        if not np.isinf(sectors[sector_id][2]):
            error_normal.append(sector_id)
    if len(error_normal) == 0:
        return [0, 0, 0, []]

    """
    error_dx = 0.0
    for sector_id in error_normal:
        error_dx += sectors[sector_id][0]
    error_dx /= len(error_normal)
    """
    error_dx = sectors[error_normal[0]][0]

    if len(error_normal) == 1:
        error_angle = sectors[error_normal[0]][2]
    else:
        error_angle = 0.0
        for sector_id in range(len(error_normal) - 1):
            error_angle += sectors[error_normal[sector_id + 1]][2] - sectors[error_normal[sector_id]][2]
        error_angle /= len(error_normal) - 1

    pwm_target = control_pwm_forward[len(error_normal)]

    return [error_dx, error_angle, pwm_target, error_normal]


global servo_set
servo_set = False
global control_dx_I, control_dx_last, control_dx_time_last
control_dx_I = 0.0
control_dx_last = None
control_dx_time_last = None
control_last = np.array([0.0, 0.0])


def control_callback(error_dx, error_angle, pwm_target, error_normal, is_move):
    global servo_set, control_dx_I, control_dx_last, control_dx_time_last, control_last

    # print(control_dx_I)
    if len(error_normal) == 0:
        if control_dx_last is not None:
            if control_dx_last > 0.0:
                motor_cmd_left.publish(control_pwm_lost)
                motor_cmd_right.publish(-control_pwm_lost)
                if control_dx_I < 0.0:
                    control_dx_I = 0.0
                control_dx_time_last = time.time()
            else:
                motor_cmd_left.publish(-control_pwm_lost)
                motor_cmd_right.publish(control_pwm_lost)
                if control_dx_I > 0.0:
                    control_dx_I = 0.0
                control_dx_time_last = time.time()
        else:
            motor_cmd_left.publish(0)
            motor_cmd_right.publish(0)
        return
    control_dx_last = error_dx

    if control_dx_time_last is None:
        control_dx_time_last = time.time()
        control_dx_D = 0.0
    else:
        control_dx_time_now = time.time()
        control_dx_D = (error_dx - control_dx_last) / (control_dx_time_now - control_dx_time_last)
        control_dx_time_last = control_dx_time_now

    control_dx_I += error_dx
    control_dx_I = np.clip(control_dx_I, -control_dx_k_I_windup[len(error_normal)],
                           control_dx_k_I_windup[len(error_normal)])

    # control_fwd = np.array([pwm_target, pwm_target], dtype=np.float32)

    control_dx = np.array([error_dx * control_dx_k_P[len(error_normal)] + control_dx_I * control_dx_k_I[
        len(error_normal)] + control_dx_D * control_dx_k_D[len(error_normal)],
                           -error_dx * control_dx_k_P[len(error_normal)] - control_dx_I * control_dx_k_I[
                               len(error_normal)] - control_dx_D * control_dx_k_D[len(error_normal)]],
                          dtype=np.float32)

    control_angle = np.array([-error_angle * control_angle_k_P[len(error_normal)],
                              error_angle * control_angle_k_P[len(error_normal)]], dtype=np.float32)

    control_dx = np.clip(control_dx, -control_dx_windup[len(error_normal)], control_dx_windup[len(error_normal)])
    control_angle = np.clip(control_angle, -control_angle_windup[len(error_normal)],
                            control_angle_windup[len(error_normal)])

    control = control_angle + control_dx
    if pwm_target > np.max(control):
        control += pwm_target - np.max(control)
    control = np.clip(control, -control_pwm_max, control_pwm_max)
    control = control * (1.0 - control_low_pass_kilter_k) + control_last * control_low_pass_kilter_k
    control_last = control

    # print(control)

    if not servo_set:
        servo_cmd[0].publish(94)
        servo_cmd[1].publish(60)  # 60
        servo_set = True

    # TODO REMOVE
    # control = np.array([0.0, 0.0])

    control_cmd = np.ndarray.astype(np.round(control), dtype=np.int)

    print(control_cmd)
    if is_move:
        motor_cmd_left.publish(control_cmd[0])
        motor_cmd_right.publish(control_cmd[1])
    else:
        motor_cmd_left.publish(0)
        motor_cmd_right.publish(0)


def main():
    rospy.init_node('image_listener')
    if RAW:
        rospy.Subscriber("/raspicam_node/image", Image, image_callback)
    else:
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, image_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
