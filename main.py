import cv2
import numpy as np
import time
import math

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

detector = cv2.QRCodeDetector()

if not cap.isOpened():
    raise IOError("Cannot open webcam")

while cap.isOpened():
    ret, frame = cap.read()

    if frame is not None:
        value, points, qrcode = detector.detectAndDecode(frame)
        cv2.imshow('Input', cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))

        # QR code detection
        start = time.perf_counter()

        if value != "":
            x1 = points[0][0][0]
            y1 = points[0][0][1]
            x2 = points[0][2][0]
            y2 = points[0][2][1]

            x_center = (x2 - x1) / 2 + x1
            y_center = (y2 - y1) / 2 + y1

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 5)

            cv2.circle(frame, (int(x_center), int(y_center)), 3, (0, 0, 255), 3)

            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            cv2.putText(frame, f'Distance: {distance:.2f}', (30, 170), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2,
                        cv2.LINE_AA)
            cv2.putText(frame, str(value), (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)

        end = time.perf_counter()
        totalTime = end - start
        fps = 1 / totalTime

        cv2.putText(frame, f'FPS: {int(fps)}', (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow('img', frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cap.release()
cv2.destroyAllWindows()