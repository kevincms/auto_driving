import numpy as np
import cv2


def define_light(obj, min_ratio=0.003, morph=True, check_light=False):

    # 1. ROI 설정 및 원형 신호등 전구 찾기
    img_bgr = obj.cv_image
    h, w, _ = img_bgr.shape
    img_bgr_roi = img_bgr[0 : h // 3, :]
    rh, rw, _ = img_bgr_roi.shape

    gray = cv2.cvtColor(img_bgr_roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)

    # 허프 원 검출 (param2, minRadius, maxRadius 등은 환경에 맞게 튜닝 필요)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(10, rh // 4),
        param1=100,
        param2=18,
        minRadius=int(rh * 0.1),
        maxRadius=int(rh * 0.4),
    )

    best_circle = None
    if circles is not None:
        circles = np.uint16(np.around(circles[0]))
        hsv = cv2.cvtColor(img_bgr_roi, cv2.COLOR_BGR2HSV)
        V = hsv[:, :, 2]  # 밝기(Value) 채널

        # 검출된 원들 중 가장 밝은 원을 선택 (실제 켜진 전구일 확률이 높음)
        best_score = -1
        for x, y, r in circles:
            mask = np.zeros_like(V)
            cv2.circle(mask, (x, y), r, 255, -1)
            mean_v = cv2.mean(V, mask=mask)[0]  # 원 내부의 평균 밝기

            if mean_v > best_score:
                best_score = mean_v
                best_circle = (x, y, r)

    # 원을 찾지 못했으면 바로 종료
    if best_circle is None:
        if check_light:  # 디버깅 모드에서 원이 없음을 표시
            img_display = img_bgr.copy()
            txt = "Circle: NOT FOUND"
            cv2.putText(
                img_display,
                txt,
                (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2,
            )
            cv2.imshow("Debug Light Detection", img_display)
            cv2.waitKey(1)
        return "none"

    # 2. 찾은 원 내부에서만 색상 분석
    x, y, r = best_circle
    circle_mask = np.zeros((rh, rw), np.uint8)
    cv2.circle(circle_mask, (x, y), r, 255, -1)

    hsv_roi = cv2.cvtColor(img_bgr_roi, cv2.COLOR_BGR2HSV)

    # 색상 범위 정의
    lower_green, upper_green = np.array([40, 80, 80]), np.array([85, 255, 255])
    lower_yel, upper_yel = np.array([10, 120, 120]), np.array([40, 255, 255])
    lower_red1, upper_red1 = np.array([0, 80, 80]), np.array([8, 255, 255])
    lower_red2, upper_red2 = np.array([175, 80, 80]), np.array([180, 255, 255])

    # 색상 마스크 생성 후, 원 마스크와 AND 연산하여 최종 영역 확정
    mask_g = cv2.bitwise_and(
        cv2.inRange(hsv_roi, lower_green, upper_green), circle_mask
    )
    mask_y = cv2.bitwise_and(cv2.inRange(hsv_roi, lower_yel, upper_yel), circle_mask)
    mask_r1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
    mask_r2 = cv2.inRange(hsv_roi, lower_red2, upper_red2)
    mask_r = cv2.bitwise_and(cv2.bitwise_or(mask_r1, mask_r2), circle_mask)

    # 모폴로지 연산
    if morph:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_OPEN, kernel)
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, kernel)
        mask_r = cv2.morphologyEx(mask_r, cv2.MORPH_OPEN, kernel)

    # 비율 계산 (분모가 전체 ROI가 아닌 원의 넓이)
    total_circle_area = cv2.countNonZero(circle_mask)
    if total_circle_area == 0:
        return "none"  # 원이 너무 작으면 오류 방지

    ratios = {
        "green": cv2.countNonZero(mask_g) / total_circle_area,
        "yellow": cv2.countNonZero(mask_y) / total_circle_area,
        "red": cv2.countNonZero(mask_r) / total_circle_area,
    }

    # 3. 최종 색상 결정 및 디버깅 출력
    color = "none"
    max_color = max(ratios, key=ratios.get)
    if ratios[max_color] >= min_ratio:
        color = max_color

    if check_light:
        img_display = img_bgr.copy()

        # 텍스트 내용과 색상 설정
        txt = f"Pred: {color.upper()} (G:{ratios['green']:.2f} Y:{ratios['yellow']:.2f} R:{ratios['red']:.2f})"
        cv2.putText(
            img_display,
            txt,
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
        )

        # 찾은 원을 디버깅 이미지에 그리기
        # (x, y)는 ROI 기준 좌표이므로 전체 이미지에도 동일하게 적용 가능
        cv2.circle(img_display, (x, y), r, (0, 255, 255), 3)  # 노란색 원으로 표시

        # 디버깅 창 표시
        cv2.imshow("Debug Light Detection", img_display)
        cv2.waitKey(1)

    return color
