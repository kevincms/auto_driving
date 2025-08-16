import numpy as np
import cv2

def define_light(obj, min_ratio=0.003, morph=True):
    # 위쪽 1/3만 사용
    img_bgr=obj.cv_image
    h = img_bgr.shape[0]
    img_bgr_roi = img_bgr[0 : h // 3, :]  # 세로 방향 위쪽 1/3

    hsv = cv2.cvtColor(img_bgr_roi, cv2.COLOR_BGR2HSV)

    # 색 범위(조명에 따라 조정)
    # Green
    lower_green = np.array([40, 80, 80], np.uint8)
    upper_green = np.array([85, 255, 255], np.uint8)
    mask_g = cv2.inRange(hsv, lower_green, upper_green)

    # Yellow
    lower_yel = np.array([10, 120, 120], np.uint8)
    upper_yel = np.array([40, 255, 255], np.uint8)
    mask_y = cv2.inRange(hsv, lower_yel, upper_yel)

    # Red
    lower_red1 = np.array([0, 80, 80], np.uint8)
    upper_red1 = np.array([8, 255, 255], np.uint8)
    lower_red2 = np.array([175, 80, 80], np.uint8)
    upper_red2 = np.array([180, 255, 255], np.uint8)
    mask_r = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2),
    )

    if morph:
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_OPEN, k3, iterations=1)
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, k3, iterations=1)
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, k3, iterations=1)

    total = img_bgr_roi.shape[0] * img_bgr_roi.shape[1]
    ratios = {
        "green": cv2.countNonZero(mask_g) / total,
        "yellow": cv2.countNonZero(mask_y) / total,
        "red": cv2.countNonZero(mask_r) / total,
    }

    # 가장 큰 비율의 색을 선택(최소 비율 미만이면 none)
    color = max(ratios, key=ratios.get)
    if ratios[color] >= min_ratio:
        return color
    else:
        return "none"