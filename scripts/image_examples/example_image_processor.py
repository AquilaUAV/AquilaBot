import rospy
from std_msgs.msg import Int16
import cv2
import numpy as np
from math import *
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

from skimage.morphology import *

bridge = CvBridge()

RASPI = False
DEBUG = True
RAW = False

debug_skeletonize_low_res = False
debug_lines_hor_ver = False

image_compress_factor = 4.0

# adaptive_threshold_param_1 = 21
# adaptive_threshold_param_2 = 10

bordersize = 3
blur_kernel = (11, 11)  # (11, 11)
blur_k = 5  # 8

horizontal_morphological_filter_size = 3
horizontal_dilate_filter_size = (9, 9)
vertical_morphological_filter_size = 3
vertical_dilate_filter_size = (9, 9)

height_sectors_min_area = 20
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

    if debug_skeletonize_low_res:
        cv2.imshow('cv2_img_compressed', cv2.resize(cv2_img, (image_width, image_height)))
        cv2.imshow('cv2_img_blured', cv2.resize(cv2_img_blured, (image_width, image_height)))
        cv2.imshow("Otsu Thresholding", cv2.resize(thresh, (image_width, image_height)))

    skeletonized = cv2.ximgproc.thinning(thresh, thinningType=cv2.ximgproc.THINNING_GUOHALL)
    skeletonized = skeletonized[bordersize:np.shape(cv2_img)[0] + bordersize,
                   bordersize:np.shape(cv2_img)[1] + bordersize]
    if debug_skeletonize_low_res:
        cv2.imshow("skeletonize", cv2.resize(skeletonized, (image_width, image_height)))

    return skeletonized


def max_pooling(skeletonized):
    (numLabels, labels, stats, centroids) = cv2.connectedComponentsWithStats(skeletonized, connectivity=8,
                                                                             ltype=cv2.CV_32S)

    if len(stats) < 2:
        labels = np.zeros(np.shape(labels))
    else:
        area = np.array(stats)[:, 4]
        max_area_id = np.argmax(area)
        area[max_area_id] = -1
        max_area_id = np.argmax(area)
        labels = (labels == max_area_id)
        labels = np.uint8(np.clip(labels, 0, 255) * 255)

    return labels


def image_callback(msg):
    global last_time, fps_counter, processing_time
    time_start_timer = rospy.get_time()
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
    horizontal_expanded = cv2.dilate(horizontal_expanded,
                                     cv2.getStructuringElement(cv2.MORPH_OPEN, horizontal_dilate_filter_size))

    vertical_expanded = vertical.copy()
    vertical_expanded = cv2.dilate(vertical_expanded,
                                   cv2.getStructuringElement(cv2.MORPH_OPEN, vertical_dilate_filter_size))

    skeletonize_vertical = skeletonized * (255 - horizontal_expanded) * 255
    skeletonize_horizontal = skeletonized * (255 - vertical_expanded) * 255

    crossroads = horizontal_expanded * vertical_expanded * 255

    vertical_line = np.clip(skeletonize_vertical + crossroads, 0, 255)
    vertical_line = cv2.ximgproc.thinning(vertical_line, thinningType=cv2.ximgproc.THINNING_GUOHALL)
    vertical_line = max_pooling(vertical_line)

    # vertical_line = [vertical_line[0:np.shape(vertical_line)[0], :], vertical_line[np.shape(vertical_line)[0]:, :]]
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
            contours_vertical_lines.append(contours_vertical_line)
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
            continue
        [vx, vy, x, y] = cv2.fitLine(np.float32(contours_vertical_lines_all[i]), cv2.DIST_L2, 0, 0.01, 0.01)
        line_angle = atan2(vy[0], vx[0]) + pi / 2
        while line_angle < pi / 2:
            line_angle += pi
        while line_angle > pi / 2:
            line_angle -= pi
        y[0] += int(round(i * sectors_height))

        sectors.append([x[0], y[0], line_angle]) # vx[0], vy[0], x[0], y[0], line_angle -> x[0], y[0], line_angle
        if DEBUG:
            x1 = int(x[0] + image_width / height_sectors_len / image_compress_factor / 2 * (-np.sin(line_angle)))
            y1 = int(y[0] + image_width / height_sectors_len / image_compress_factor / 2 * (np.cos(line_angle)))
            x2 = int(x[0] - image_width / height_sectors_len / image_compress_factor / 2 * (-np.sin(line_angle)))
            y2 = int(y[0] - image_width / height_sectors_len / image_compress_factor / 2 * (np.cos(line_angle)))
            cv2.line(image_with_line, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.circle(image_with_line, (x, y), 1, (0, 0, 255))
    if DEBUG:
        cv2.imshow("image_with_vertical_lines", cv2.resize(image_with_line, (image_width, image_height)))

    print(sectors)



    if DEBUG:
        cv2.waitKey(1)

    fps_counter += 1
    time_now = rospy.get_time()
    processing_time += time_now - time_start_timer
    if time_now > last_time + fps_timeout:
        rospy.loginfo("fps: {:.3f}\t| proc: {:2.2%}".format(fps_counter / (time_now - last_time),
                                                            processing_time / (time_now - last_time)))
        fps_counter = 0
        processing_time = 0.0
        last_time = time_now


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
    if RAW:
        rospy.Subscriber("/raspicam_node/image", Image, image_callback)
    else:
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, image_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
