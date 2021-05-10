import time

import numpy as np

from CameraCapture import CameraCapture

import cv2
from threading import Thread, Lock
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QObject
from PyQt5.QtGui import QImage

import rospy
from geometry_msgs.msg import TwistStamped


class PID:
    def __init__(self):
        self.P = 0
        self.I = 0
        self.D = 0
        self.Multiplier = 0


class DroneControl(QObject):
    changePixmap1 = pyqtSignal(QImage)
    changePixmap2 = pyqtSignal(QImage)
    changePixmap3 = pyqtSignal(QImage)
    changeFpsCounter = pyqtSignal(int)

    def __init__(self):
        super().__init__()

        print("drone control created")
        self.pid_x = PID()
        self.pid_y = PID()
        self.pid_z = PID()

        self.low_HSV = []
        self.high_HSV = []
        self.low_HSV2 = []
        self.high_HSV2 = []

        self.erode_size = 0
        self.dilate_size = 0
        self.erode_size2 = 0
        self.dilate_size2 = 0

        self.landing_area = []

        self.error_x_before = 0
        self.error_y_before = 0
        self.error_z_before = 0
        self.is_running = True

        self.initVar()

        self.attitude_pub = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel',TwistStamped,queue_size=1)

        self.openCamera(0)

        self.param_mutex = Lock()


    def initVar(self):
        self.low_HSV.append(0)
        self.low_HSV.append(0)
        self.low_HSV.append(0)
        self.high_HSV.append(255)
        self.high_HSV.append(255)
        self.high_HSV.append(255)

        self.low_HSV2.append(0)
        self.low_HSV2.append(0)
        self.low_HSV2.append(0)
        self.high_HSV2.append(255)
        self.high_HSV2.append(255)
        self.high_HSV2.append(255)

        self.landing_area.append((260,180))
        self.landing_area.append((380,300))

        self.center_landing_area = (((self.landing_area[0][0] + self.landing_area[1][0]) // 2),
                                    ((self.landing_area[1][0] + self.landing_area[1][1]) // 2))


    def onPIDChanged(self, attribute, axis, val):
        self.param_mutex.acquire()
        if attribute == 'p':
            if axis == 'x':
                self.pid_x.P = val
            elif axis == 'y':
                self.pid_y.P = val
            elif axis == 'z':
                self.pid_z.P = val
        elif attribute == 'i':
            if axis == 'x':
                self.pid_x.I = val
            elif axis == 'y':
                self.pid_y.I = val
            elif axis == 'z':
                self.pid_z.I = val
        elif attribute == 'd':
            if axis == 'x':
                self.pid_x.D = val
            elif axis == 'y':
                self.pid_y.D = val
            elif axis == 'z':
                self.pid_z.D = val
        elif attribute == 'm':
            if axis == 'x':
                self.pid_x.Multiplier = val
            elif axis == 'y':
                self.pid_y.Multiplier = val
            elif axis == 'z':
                self.pid_z.Multiplier = val

        self.param_mutex.release()

    def onHSVChanged(self, attribute, name, value):
        self.param_mutex.acquire()

        if attribute == 'h':
            if name == 'low':
                self.low_HSV[0] = value
            elif name == 'high':
                self.high_HSV[0] = value
            elif name == 'low2':
                self.low_HSV2[0] = value
            elif name == 'high2':
                self.high_HSV2[0] = value
        elif attribute == 's':
            if name == 'low':
                self.low_HSV[1] = value
            elif name == 'high':
                self.high_HSV[1] = value
            elif name == 'low2':
                self.low_HSV2[1] = value
            elif name == 'high2':
                self.high_HSV2[1] = value
        elif attribute == 'v':
            if name == 'low':
                self.low_HSV[2] = value
            elif name == 'high':
                self.high_HSV[2] = value
            elif name == 'low2':
                self.low_HSV2[2] = value
            elif name == 'high2':
                self.high_HSV2[2] = value

        self.param_mutex.release()

    def onMorphChange(self, attribute, name, value):
        self.param_mutex.acquire()
        if attribute == 'erode':
            if name == 'erode1':
                self.erode_size = value
            elif name == 'erode2':
                self.erode_size2 = value
        elif attribute == 'dilate':
            if name == 'dilate1':
                self.dilate_size = value
            elif name == 'dilate2':
                self.dilate_size2 = value
        self.param_mutex.release()

    def onLandingAreaChanged(self, x, y, z):
        # self.landing_area[0][0] = 260 - z + x
        # self.landing_area[0][1] = 180 - z + y
        # self.landing_area[1][0] = 380 + z + x
        # self.landing_area[1][1] = 300 + z + y

        self.landing_area[0] = ((260 - z + x), (180 - z + y))
        self.landing_area[1] = ((380 + z + x), (300 + z + y))

        self.center_landing_area = (((self.landing_area[0][0] + self.landing_area[1][0]) // 2),
                               ((self.landing_area[1][0] + self.landing_area[1][1]) // 2))

    def calculatePIDError(self, pid_: PID, error, error_prev, acc_error):
        #clamping dulu

        return np.clip(pid_.Multiplier*(pid_.P * error + pid_.I * (acc_error) + pid_.D * (error - error_prev)),-2,2)


    def openCamera(self, camera):
        self.camera_thread = Thread(target=self.openCameraThread, args=(camera,))
        self.camera_thread.start()
        print("opening camera")

    def publishControlToMavros(self, vx, vy, vz):
        msg = TwistStamped()
        msg.twist.linear.x = vx # kiri 
        msg.twist.linear.y = -vy # maju
        msg.twist.linear.z = -vz
        self.attitude_pub.publish(msg)
        print("V ", vx, " ", vy, " ", vz)
        pass

    def stopAllThread(self):
        self.param_mutex.acquire()
        self.is_running = False
        self.param_mutex.release()

    def openCameraThread(self, camera):
        print("opening camera Thread")

        self.camera_capture = CameraCapture(camera)
        prev_frame_time = 0
        error_x = 0
        error_y = 0
        error_z = 0

        while self.camera_capture.isOpened() and self.is_running:
            # print("camera strating")
            new_frame_time = time.time()
            ret, img = self.camera_capture.read()

            if not ret:
                continue
            frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            frame_threshold = cv2.inRange(frame_HSV, (self.low_HSV[0], self.low_HSV[1], self.low_HSV[2]),
                                          (self.high_HSV[0], self.high_HSV[1], self.high_HSV[2]))

            element = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * self.erode_size + 1, 2 * self.erode_size + 1),
                                               (self.erode_size, self.erode_size))
            erosion_dst = cv2.erode(frame_threshold, element)
            element = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * self.dilate_size + 1, 2 * self.dilate_size + 1),
                                                (self.dilate_size, self.dilate_size))
            frame_morph = cv2.dilate(erosion_dst, element)

            contours, _ = cv2.findContours(frame_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            contours_poly = [None] * len(contours)
            boundRect = [None] * len(contours)
            min_area = 400
            i = 0
            for q, c in enumerate(contours):
                if cv2.contourArea(c) < min_area:
                    continue

                contours_poly[i] = cv2.approxPolyDP(c, 3, True)
                boundRect[i] = cv2.boundingRect(contours_poly[i])
                # print(boundRect[i])
                cv2.rectangle(img, (int(boundRect[i][0]), int(boundRect[i][1])), \
                             (int(boundRect[i][0] + boundRect[i][2]), int(boundRect[i][1] + boundRect[i][3])), (255,0,0), 2)
                i = i + 1
###################################--------------------------------------------------

            frame_threshold2 = cv2.inRange(frame_HSV, (self.low_HSV2[0], self.low_HSV2[1], self.low_HSV2[2]),
                                          (self.high_HSV2[0], self.high_HSV2[1], self.high_HSV2[2]))

            element2 = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * self.erode_size2 + 1, 2 * self.erode_size2 + 1),
                                                (self.erode_size2, self.erode_size2))
            erosion_dst2 = cv2.erode(frame_threshold2, element2)
            element2 = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * self.dilate_size2 + 1, 2 * self.dilate_size2 + 1),
                                                (self.dilate_size2, self.dilate_size2))
            frame_morph2 = cv2.dilate(erosion_dst2, element2)

            contours2, _ = cv2.findContours(frame_morph2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            contours_poly2 = [None] * len(contours2)
            boundRect2 = [None] * len(contours2)
            min_area2 = 400
            i = 0
            for q, c in enumerate(contours2):
                if cv2.contourArea(c) < min_area2:
                    continue

                contours_poly2[i] = cv2.approxPolyDP(c, 3, True)
                boundRect2[i] = cv2.boundingRect(contours_poly2[i])
                cv2.rectangle(img, (int(boundRect2[i][0]), int(boundRect2[i][1])), \
                              (int(boundRect2[i][0] + boundRect2[i][2]), int(boundRect2[i][1] + boundRect2[i][3])),
                              (0, 255, 0), 2)
                i = i + 1
###################################--------------------------------------------------
            landing_marker = []
            if len(boundRect) < 50 or len(boundRect2) < 50:
                for i in boundRect:
                    if i == None:
                        continue
                    tl = (int(i[0]), int(i[1]))
                    br = (int(i[0] + i[2]), int(i[1] + i[3]))
                    for y in boundRect2:
                        if y == None:
                            continue
                        tl2 = (int(y[0]), int(y[1]))
                        br2 = (int(y[0] + y[2]), int(y[1] + y[3]))

                        if (tl[0] <= tl2[0]) and (br[0] >= br2[0]) \
                                and (tl[1] <= tl2[1]) and (br[1] >= br2[1]):
                            cv2.rectangle(img, tl, \
                                          br,
                                          (0, 0, 255), 2)
                            landing_marker.append((tl, br))

            controlled_landing_marker = None
            if len(landing_marker):
                controlled_landing_marker = (landing_marker[0])

            cv2.rectangle(img, (self.landing_area[0][0],self.landing_area[0][1]),
                          (self.landing_area[1][0],self.landing_area[1][1]), (255, 255, 255), 2)

            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # try:
            # except Exception:
            #     print(Exception)
            h, w, ch = img.shape
            bytesPerLine = ch * w
            convertToQtFormat = QImage(img.data, w, h, bytesPerLine, QImage.Format_RGB888)
            # p = convertToQtFormat.scaled(640, 480)
            p = convertToQtFormat

            self.changePixmap1.emit(p)

            img2 = cv2.cvtColor(frame_morph, cv2.COLOR_GRAY2BGR)
            h, w , ch = img2.shape
            bytesPerLine = w * ch
            convertToQtFormat2 = QImage(img2.data, w, h, bytesPerLine, QImage.Format_RGB888)
            # p = convertToQtFormat.scaled(201, 151)
            p2 = convertToQtFormat2
            self.changePixmap2.emit(p2)

            img3 = cv2.cvtColor(frame_morph2, cv2.COLOR_GRAY2BGR)
            h, w, ch = img3.shape
            bytesPerLine = w * ch
            convertToQtFormat3 = QImage(img3.data, w, h, bytesPerLine, QImage.Format_RGB888)
            # p = convertToQtFormat.scaled(201, 151)
            p3 = convertToQtFormat3
            self.changePixmap3.emit(p3)

            # cv2.imshow('seg', frame_threshold)
            # cv2.imshow('morhp', frame_morph)

            # cv2.imshow('seg2', frame_threshold2)
            # cv2.imshow('morhp2', frame_morph2)
            # cv2.waitKey(1)

            #COMPUTING ERROR, menggunakan titik tengah marker dan landing area
            center_landing_marker = None
            if controlled_landing_marker != None and len(controlled_landing_marker):
                center_landing_marker = ( ((controlled_landing_marker[0][0] + controlled_landing_marker[1][0]) // 2),
                                          ((controlled_landing_marker[0][1] + controlled_landing_marker[1][1]) // 2))
                # print("CENTER Marker ",center_landing_marker)

            if center_landing_marker != None:
                error_x = center_landing_marker[0] - self.center_landing_area[0]
                error_y = center_landing_marker[1] - self.center_landing_area[1]

                if ((controlled_landing_marker[0][0] >= self.landing_area[0][0]) \
                    and (controlled_landing_marker[1][0] <= self.landing_area[1][0]) \
                    and (controlled_landing_marker[0][1] >= self.landing_area[0][1]) \
                    and (controlled_landing_marker[1][1] <= self.landing_area[1][1])) \
                        or ((controlled_landing_marker[0][0] <= self.landing_area[0][0]) \
                    and (controlled_landing_marker[1][0] >= self.landing_area[1][0]) \
                    and (controlled_landing_marker[0][1] <= self.landing_area[0][1]) \
                    and (controlled_landing_marker[1][1] >= self.landing_area[1][1])):
                        # print("TURUN")
                        error_z = 10


                # print("Error X Y ", error_x, " ", error_y)


            else:
                error_x = 0
                error_y = 0
                error_z = 0 #BISA DINAIKKAN DRONE NYA CARI LAGI

            vx = self.calculatePIDError(self.pid_x, error_x, self.error_x_before, error_x + self.error_x_before)
            vy = self.calculatePIDError(self.pid_y, error_y, self.error_y_before, error_y + self.error_y_before)
            vz = self.calculatePIDError(self.pid_z, error_z, self.error_z_before, error_z + self.error_z_before)

            self.error_x_before = error_x
            self.error_y_before = error_y
            self.error_z_before = error_z

            velocity_divider = 100
            self.publishControlToMavros(vx / velocity_divider, vy / velocity_divider, vz / velocity_divider)

            fps = 1 / (new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time
            self.changeFpsCounter.emit(int(fps))

        self.camera_capture.release()
    #signal

