import cv2
from networktables import NetworkTables
NetworkTables.initialize(server='roborio-6445-frc.local')
sd = NetworkTables.getTable('SmartDashboard')
Jetson = NetworkTables.getTable('Jetson')
from enum import Enum

class Red(object):
    """
    An OpenCV pipeline
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsl_threshold_0_hue = [67.98561151079136, 129.31740614334473]
        self.__hsl_threshold_0_saturation = [215.55755395683454, 255.0]
        self.__hsl_threshold_0_luminance = [197.21223021582736, 255.0]

        self.hsl_threshold_0_output = None

        self.__hsl_threshold_1_hue = [55.03597122302158, 180.0]
        self.__hsl_threshold_1_saturation = [0.0, 255.0]
        self.__hsl_threshold_1_luminance = [0.0, 255.0]

        self.hsl_threshold_1_output = None

        self.__blur_input = self.hsl_threshold_0_output
        self.__blur_type = BlurType.Median_Filter
        self.__blur_radius = 22.522522522522518

        self.blur_output = None

        self.__find_contours_input = self.blur_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        :rtype: object
        """
        # Step HSL_Threshold0:
        self.__hsl_threshold_0_input = source0
        (self.hsl_threshold_0_output) = self.__hsl_threshold(self.__hsl_threshold_0_input, self.__hsl_threshold_0_hue,
                                                             self.__hsl_threshold_0_saturation,
                                                             self.__hsl_threshold_0_luminance)

        # Step HSL_Threshold1:
        self.__hsl_threshold_1_input = source0
        (self.hsl_threshold_1_output) = self.__hsl_threshold(self.__hsl_threshold_1_input, self.__hsl_threshold_1_hue,
                                                             self.__hsl_threshold_1_saturation,
                                                             self.__hsl_threshold_1_luminance)

        # Step Blur0:
        self.__blur_input = self.hsl_threshold_0_output
        (self.blur_output) = self.__blur(self.__blur_input, self.__blur_type, self.__blur_radius)

        # Step Find_Contours0:
        self.__find_contours_input = self.blur_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input,
                                                           self.__find_contours_external_only)

    @staticmethod
    def __hsl_threshold(input, hue, sat, lum):
        """Segment an image based on hue, saturation, and luminance ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max luminance.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HLS)
        return cv2.inRange(out, (hue[0], lum[0], sat[0]), (hue[1], lum[1], sat[1]))

    @staticmethod
    def __blur(src, type, radius):
        """Softens an image using one of several filters.
        Args:
            src: The source mat (numpy.ndarray).
            type: The blurType to perform represented as an int.
            radius: The radius for the blur as a float.
        Returns:
            A numpy.ndarray that has been blurred.
        """
        if (type is BlurType.Box_Blur):
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif (type is BlurType.Gaussian_Blur):
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif (type is BlurType.Median_Filter):
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if (external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy = cv2.findContours(input, mode=mode, method=method)
        return contours


BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

class Blue(object):
    """
    An OpenCV pipeline
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsl_threshold_hue = [0.0, 37.16723549488056]
        self.__hsl_threshold_saturation = [0.0, 255.0]
        self.__hsl_threshold_luminance = [158.22841726618708, 255.0]

        self.hsl_threshold_output = None

        self.__blur_input = self.hsl_threshold_output
        self.__blur_type = BlurType.Median_Filter
        self.__blur_radius = 39.63963963963963

        self.blur_output = None

        self.__find_contours_input = self.blur_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__get_mat_info_input = self.blur_output

        self.get_mat_info_size = None

        self.get_mat_info_empty = None

        self.get_mat_info_channels = None

        self.get_mat_info_cols = None

        self.get_mat_info_rows = None

        self.get_mat_info_high_value = None


    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        :rtype: object
        """
        # Step HSL_Threshold0:
        self.__hsl_threshold_input = source0
        (self.hsl_threshold_output) = self.__hsl_threshold(self.__hsl_threshold_input, self.__hsl_threshold_hue, self.__hsl_threshold_saturation, self.__hsl_threshold_luminance)

        # Step Blur0:
        self.__blur_input = self.hsl_threshold_output
        (self.blur_output) = self.__blur(self.__blur_input, self.__blur_type, self.__blur_radius)

        # Step Find_Contours0:
        self.__find_contours_input = self.blur_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input, self.__find_contours_external_only)

        # Step Get_Mat_Info0:
        self.__get_mat_info_input = self.blur_output
        (self.get_mat_info_size,self.get_mat_info_empty,self.get_mat_info_channels,self.get_mat_info_cols,self.get_mat_info_rows,self.get_mat_info_high_value) = self.__get_mat_info(self.__get_mat_info_input)


    @staticmethod
    def __hsl_threshold(input, hue, sat, lum):
        """Segment an image based on hue, saturation, and luminance ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max luminance.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HLS)
        return cv2.inRange(out, (hue[0], lum[0], sat[0]),  (hue[1], lum[1], sat[1]))

    @staticmethod
    def __blur(src, type, radius):
        """Softens an image using one of several filters.
        Args:
            src: The source mat (numpy.ndarray).
            type: The blurType to perform represented as an int.
            radius: The radius for the blur as a float.
        Returns:
            A numpy.ndarray that has been blurred.
        """
        if(type is BlurType.Box_Blur):
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif(type is BlurType.Gaussian_Blur):
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif(type is BlurType.Median_Filter):
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if(external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
        return contours

    @staticmethod
    def __get_mat_info(src):
        """Gets information about given Mat.
        Args:
            src: A numpy.ndarray.
        Returns:
            The size of the mat as a list of two numbers.
            A boolean that is true if the mat is empty.
            The number of the channels in the mat.
            The number of columns.
            The number of rows.
            The highest value in the mat.
        """
        cols, rows, channels = src.shape
        empty = (src.size==0)
        lowest_value, highest_value = cv2.minMaxLoc(src if (channels == 1)
                            else cv2.cvtColor(src, cv2.COLOR_BGR2GRAY))
        mat_size = (rows, cols)
        return mat_size, empty, channels, cols, rows, highest_value


BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

def get_distance_from_sensor():
    return sd.getvalue("Distance_Sensor")
def get_realsense_distance():
    # Do the things

def program_1():
    while get_distance_from_sensor() < 10 and get_realsense_distance():
        # do something
def program_1a():

def program_1b():

def program_1c():

def program_2():
    while get_distance_from_sensor() < 10 and get_realsense_distance():
        # do something
def program_2a():

def program_2b():

def program_2c():

def program_3():
    while get_distance_from_sensor() < 10 and get_realsense_distance():
        # do something
def program_3a():

def program_3b():

def program_3c():
while True:
    color = sd.getvalue("Alliance_Color")
    position = sd.getvalue("Auto_Mode")
    if "blue" in color:
        blue = Blue()
        Blue.process(blue, cv2.VideoCapture(0))
    if "red" in color:
        red = Red()
        Red.process(red, cv2.VideoCapture(0))
    if position is 1:
        program_1()
    if position is 2:
        program_2()
    if position is 3:
        program_3()
