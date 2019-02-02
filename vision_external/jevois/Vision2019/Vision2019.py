import libjevois as jevois
import cv2
import numpy as np
import math
## Angle displacement to tape.
#
#
# @author Dhruv Shah
#
# @videomapping YUYV 640 480 30 YUYV 640 480 30 MSET649 FollowTheTape
# @email dhruv.shah@gmail.com
# @address 123 first street, Los Angeles CA 90012, USA
# @copyright Copyright (C) 2018 by Dhruv Shah
# @mainurl example.com
# @supporturl example.com
# @otherurl example.com
# @license GPL v3
# @distribution Unrestricted
# @restrictions None
# @ingroup modules

DEBUG = 0

class Vision2019:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        # HSV color range to use:
        #
        # H: 0=red/do not use because of wraparound, 30=yellow, 45=light green, 60=green, 75=green cyan, 90=cyan,
        #      105=light blue, 120=blue, 135=purple, 150=pink
        # S: 0 for unsaturated (whitish discolored object) to 255 for fully saturated (solid color)
        # V: 0 for dark to 255 for maximally bright
        self.HLSmin = np.array([ 35, 50, 200], dtype=np.uint8)
        self.HLSmax = np.array([ 70, 255, 255], dtype=np.uint8)
        # Other processing parameters:
        self.epsilon = 0.02              # Shape smoothing factor (higher for smoother)
        self.hullarea = ( 2*5.5, 160*120 ) # Range of object area (in pixels) to track
        self.hullfill = 97                 # Min fill ratio of the convex hull (percent)
        self.ethresh = 5                 # Shape error threshold (lower is stricter for exact shape)
        self.margin = 0                    # Margin from from frame borders (pixels)
        self.bound_ratio_error = 80        # % error on bounding aspect ratio
        self.bound_target_ratio = (0.52419)    # target on bounding aspect ratio
        self.rot_ratio_error = 20           # % error on rotated bounding aspect ratio
        self.rot_target_ratio = (0.444, 2.333) # the two rotated rectangle aspect ratios (one horizontal, one for vertical)

        self.angle_data = np.array([])

        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("Vision2019", 100, jevois.LOG_INFO)

        # Print out config values
        threshold_debug = "HLS threshold range: H={}-{} L={}-{} S={}-{} ".format(self.HLSmin[0], self.HLSmax[0], self.HLSmin[1], self.HLSmax[1], self.HLSmin[2], self.HLSmax[2])
        jevois.LINFO(threshold_debug)
        # CAUTION: The constructor is a time-critical code section. Taking too long here could upset USB timings and/or
        # video capture software running on the host computer. Only init the strict minimum here, and do not use OpenCV,
        # read files, etc
    # ###################################################################################################
    ## Load camera calibration from JeVois share directory
    def loadCameraCalibration(self, w, h):
        cpf = "/jevois/share/camera/calibration{}x{}.yaml".format(w, h)
        fs = cv2.FileStorage(cpf, cv2.FILE_STORAGE_READ)
        if (fs.isOpened()):
            self.camMatrix = fs.getNode("camera_matrix").mat()
            self.distCoeffs = fs.getNode("distortion_coefficients").mat()
            jevois.LINFO("Loaded camera calibration from {}".format(cpf))
        else:
            jevois.LFATAL("Failed to read camera parameters from file [{}]".format(cpf))

    # ###################################################################################################
    ## Detect objects within our HSV range
    def detect(self, imgbgr, outimg = None):
        maxn = 3 # max number of objects we will consider
        h, w, chans = imgbgr.shape
        # Convert input image to HSV:
        imghls = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HLS)

        # Create structuring elements for morpho maths:
        if not hasattr(self, 'erodeElement'):
            self.erodeElement = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            self.dilateElement = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))

        # Apply morphological operations to cleanup the image noise:
        imghls = cv2.erode(imghls, self.erodeElement)
        imghls = cv2.dilate(imghls, self.dilateElement)

        # Isolate pixels inside our desired HLS range:
        imgth = cv2.inRange(imghls, self.HLSmin, self.HLSmax)
        imgth = cv2.blur(imgth, (3, 3))

        # display threshold
        #resultBGR = cv2.bitwise_and(imgbgr, imgbgr, mask = imgth)
        #display = cv2.cvtColor(resultBGR, cv2.COLOR_BGR2RGB)
        jevois.convertCvGRAYtoRawImage(imgth, outimg, 100)

        # Detect objects by finding contours:
        contours, hierarchy = cv2.findContours(imgth, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Only consider the N biggest objects by area:
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[:maxn]

        # Array of pairs of (match string, center)
        contour_matches = []
        # Array of cost, center
        contour_costs = []

        # Identify the "good" objects:
        for c in contours:
            match = ""
            cost = 0
            # Compute contour area:
            area = cv2.contourArea(c, oriented = False)
            # Compute convex hull:
            rawhull = cv2.convexHull(c, clockwise = True)
            rawhullperi = cv2.arcLength(rawhull, closed = True)
            hull = cv2.approxPolyDP(rawhull, epsilon = self.epsilon * rawhullperi * 3.0, closed = True)
            self.logMessage(hull.shape, "hushape ")
            # Is it the right shape?
            if (hull.shape != (4,1,2)):
                match += "" # 4 vertices for the rectangular convex outline (shows as a trapezoid)
                continue
            else:
                match += "H" # Hull is quadrilateral
            

            huarea = cv2.contourArea(hull, oriented = False)
            self.logMessage(huarea, "huarea ")
            if huarea == 0.0:
                continue
            elif (huarea < self.hullarea[0] or huarea > self.hullarea[1]):
                match += ""
                continue
            else:
                match += "A" # Hull area ok
            
            cost += pow((2000 / huarea), 3.0)

            hufill = area / huarea * 100.0
            self.logMessage(hufill, "hullfill ")
            if hufill < self.hullfill:
                match += ""
                continue
            else:
                match += "F" # Fill is ok

            # Check object shape:
            peri = cv2.arcLength(c, closed = True)
            approx = cv2.approxPolyDP(c, epsilon = self.epsilon * peri, closed = True)
            self.logMessage(len(approx), "approx ")
            if len(approx) < 4 or len(approx) > 8:
                match += ""  # 4 vertices for a recangle shape
            else:
                match += "S" # Shape is ok

            # Compute contour serr:
            serr = 100.0 * cv2.matchShapes(c, approx, cv2.CONTOURS_MATCH_I1, 0.0)
            self.logMessage(serr, "shape error")
            if serr > self.ethresh:
                match += ""
            else:
                match += "E" # Shape error is ok

            # Get bounding reectangle (no rotation) for aspect ratio
            _, _, bound_rect_w, bound_rect_h = cv2.boundingRect(c)
            bound_aspect_ratio = float(bound_rect_w)/float(bound_rect_h)
            bound_ratio_err = round(abs(bound_aspect_ratio - self.bound_target_ratio)/bound_aspect_ratio * 100)
            self.logMessage(bound_ratio_err, "bounding aspect ratio error ")
            if bound_ratio_err > self.bound_ratio_error:
                match += ""
            else:
                match += "B" # Bounding ratio error is ok
                
            cost += math.sqrt(bound_ratio_err)
                            
            # Get bounding box for rotated rectangle
            rot_box = cv2.minAreaRect(c) # ((x,y), (w, h), theta)
            rot_w,rot_h = rot_box[1]
            rot_w = float(rot_w)
            rot_h = float(rot_h)
            theta = rot_box[2]
            self.logMessage((rot_w/rot_h, theta), "ROT RECT ")
            
            rot_aspect_ratio = float(rot_w)/float(rot_h)
            rot_ratio_err = round(abs(min(rot_aspect_ratio - self.rot_target_ratio[0], rot_aspect_ratio - self.rot_target_ratio[1]))/rot_aspect_ratio * 100)
            self.logMessage(rot_ratio_err, "rotation aspect ratio error ")
            if rot_ratio_err > self.rot_ratio_error:
                match += ""
            else:
                match += "R" # Rot rect ratio error is ok
                
            cost += rot_ratio_err

                
            # Calculate the center of the contour
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            self.logMessage(match)
            self.logMessage(cost, "cost ")

            cost += (10 - len(match))
            
            contour_matches.append((match, center))
            contour_costs.append((cost, center))
            
        if len(contour_matches) == 0:
            jevois.LERROR("NO CONTOUR DETECTED")
            return None, None
        contour_costs.sort(key=self.cms)
        centers = np.array([center for (match, center) in contour_matches[:2]]) # two lowest cost contours
        center = tuple(np.mean(centers, axis=0, dtype=np.float64).astype(np.int))
        center = [a.item() for a in center]
        return centers, center
    # ###################################################################################################
    ## Send angle over serial
    def calculateAngle(self, w, h, center):
        if center == None:
            return
        stdCenterX = self.stdX(float(center[0]), float(w))
        angle = round(math.degrees(math.atan(stdCenterX/1732.050808)), 2) # 60 degree lens
        #jevois.sendSerial("ANGLE {}".format(angle))
        return angle

    def logMessage(self, msg, prefix_msg = ""):
        if DEBUG != 1: return
        jevois.LINFO("{}{}".format(prefix_msg, msg))

    # ###################################################################################################
    ## Draw circle at detected center
    def drawDetections(self, outimg, centers, center):
        if centers == None:
            return
        jevois.drawCircle(outimg, int(center[0]), int(center[1]), 10, 1, jevois.YUYV.MedPink)
        for c in centers:
            jevois.drawCircle(outimg, int(c[0]), int(c[1]), 3, 1, jevois.YUYV.MedPink)

    # ###################################################################################################
    ## Process function with no USB output
    def processNoUSB(self, inframe):
        # Get the next camera image (may block until it is captured) as OpenCV BGR:
        imgbgr = inframe.getCvBGR()
        h, w, chans = imgbgr.shape
        # Start measuring image processing time:
        self.timer.start()
        # Get a list of quadrilateral convex hulls for all good objects:
        center = self.detect(imgbgr)
        # Load camera calibration if needed:
        if not hasattr(self, 'camMatrix'): self.loadCameraCalibration(w, h)
        # Send all serial messages:
        self.sendAllSerial(w, h, hlist, rvecs, tvecs)
        # Log frames/s info (will go to serlog serial port, default is None):
        self.timer.stop()

    # ###################################################################################################
    ## Process function with USB output
    def process(self, inframe, outframe):
        # Get the next camera image (may block until it is captured). To avoid wasting much time assembling a composite
        # output image with multiple panels by concatenating numpy arrays, in this module we use raw YUYV images and
        # fast paste and draw operations provided by JeVois on those images:
        inimg = inframe.get()
        # Let camera know we are done using the input image:
        inframe.done()

        # Start measuring image processing time:
        self.timer.start()
        # Convert input image to BGR24:
        imgbgr = jevois.convertToCvBGR(inimg)
        h, w, chans = imgbgr.shape

        # Load camera calibration if needed:
        if not hasattr(self, 'camMatrix'): self.loadCameraCalibration(w, h)

        # Get pre-allocated but blank output image which we will send over USB:
        outimg = outframe.get()
        outimg.require("output", w, h, jevois.V4L2_PIX_FMT_YUYV)
        jevois.paste(inimg, outimg, 0, 0)

        # Get the centers of the vision targets and the center of the pair
        centers, center = self.detect(imgbgr, outimg)

        if center != None:
            # Send serial messages
            angle = self.calculateAngle(w, h, center)
            self.angle_data = np.append(self.angle_data, angle)
            # Draw detections on image
            self.drawDetections(outimg, centers, center)
        while len(self.angle_data) > 5:
                self.angle_data = np.delete(self.angle_data, 0)
        jevois.sendSerial("ANGLE {}".format(np.median(self.angle_data)))
        # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
        fps = self.timer.stop()

        jevois.writeText(outimg, fps, 3, h-10, jevois.YUYV.White, jevois.Font.Font6x10)
        # jevois.writeText(outimg, desc, 3, h-20, jevois.YUYV.White, jevois.Font.Font6x10)
        # self.logMessage(desc)
        # We are done with the output, ready to send it to host over USB:
        outframe.send()

    def stdX(self, x, w):
        stdX = float(2000.0 * x/w - 1000.0)
        eps = 0.1
        stdX = round(stdX / eps) * eps
        return stdX

    def cms(self, item):
        return item[0]
