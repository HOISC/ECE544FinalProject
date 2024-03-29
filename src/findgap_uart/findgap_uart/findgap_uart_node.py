import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import serial

class FindGapAndUart(Node):
    bubbleRadius = 8       # 20 data points. Tunable parameter
    threshold_dist = 1.75   # Threshold distance to start counting a gap
    def __init__(self):
        super().__init__("findgap_uart_node")
        self.subsciber = self.create_subscription(
            LaserScan, "lidar_data", self.callback_process_gap, 100
        )
        self.get_logger().info("gap_uart node has started")

        self.serial = serial.Serial('/dev/ttyACM0', baudrate=9600)
    
    def callback_process_gap(self, msg):
        #PI = 3.14159
        self.get_logger().info("IN UART NODE")
        self.get_logger().info(str(msg.ranges))
        self.get_logger().info("ANGLES")
        self.get_logger().info(str(msg.intensities))

        modifiedArray = self.bubble(msg.ranges)
        distThreshold = self.findThreshold(modifiedArray)
        steeringindx = self.findGap(modifiedArray, distThreshold)

        steeringAngle = msg.intensities[steeringindx]
        self.get_logger().info(str(distThreshold))
        self.get_logger().info(str(steeringAngle))


        speed_write = b''
        angle_write = b''
        speed_write = ('$R' + str(steeringAngle) + ',' + 'F25@').encode('ascii')
        self.serial.write(speed_write)

    def findMin(self, origArray):
        curr_min = 20000
        min_indx = -1

        for indxArr in range(0, len(origArray)):
            if str(origArray[indxArr]) != "nan":
                if origArray[indxArr] < curr_min:
                    curr_min = origArray[indxArr]
                    min_indx = indxArr
                    self.get_logger().info("Minimum Current: " + str(min_indx))

        return min_indx
    
    def bubble(self, fullArray):
        self.get_logger().info("Making bubble in array")
        modified_array = fullArray
        min_idx = self.findMin(modified_array)
        self.get_logger().info("min range: " + str(min_idx))
        self.get_logger().info("length Modified: " + str(len(modified_array)))
        

        begin_indx = min_idx - self.bubbleRadius
        last_indx = min_idx + self.bubbleRadius

        if min_idx != -1:
            if begin_indx < 0: begin_indx = 0
            if last_indx > len(modified_array): last_indx = len(modified_array)

            for i in range(begin_indx, last_indx):
                modified_array[i] = 0
            self.get_logger().info("After bubble: " + str(modified_array))
        else:
            self.get_logger().info("no min indx")
        
        return modified_array
    
    def countBetween(self, bubbledArray, val1, val2):
        counts = sum((i >= val1 and i < val2) for i in bubbledArray)
        self.get_logger().info(str(counts))
        return counts
    
    def findThreshold(self, bubbledArray):
        counts_zero = self.countBetween(bubbledArray, 0, 0.5)
        counts_half = self.countBetween(bubbledArray, 0.5, 1)
        counts_one = self.countBetween(bubbledArray, 1, 1.5)
        counts_one_half = self.countBetween(bubbledArray, 1.5, 2)
        counts_two = self.countBetween(bubbledArray, 2, 2.5)
        counts_two_half = self.countBetween(bubbledArray, 2.5, 3)
        counts_three = self.countBetween(bubbledArray, 3, 20)
        
#        counts_zero = count_gt_half - count_gt_zero
#        counts_half = count_gt_one - count_gt_half
#        counts_one = count_gt_one_half - count_gt_one
#        counts_one_half = count_gt_two - count_gt_one_half
#        counts_two = count_gt_two_half - count_gt_two
#        counts_two_half = count_gt_three - count_gt_two_half
#        counts_three = count_gt_three
        
        array_counts = [counts_zero, counts_half, counts_one, counts_one_half, counts_two, counts_two_half, counts_three]
        max_val = max(array_counts)
        self.get_logger().info("max " + str(max_val))
        max_indx = array_counts.index(max_val)
        self.get_logger().info("ind " + str(max_indx))

        threshold = 1.75

        #if max_indx == 0:
        #    threshold = 3
        #elif max_indx == 1:
        #    threshold = 2.5
        #elif max_indx == 2:
        #    threshold = 2
        #elif max_indx == 3:
        #    threshold = 1.5
        #elif max_indx == 4:
        #    threshold = 1
        #elif max_indx == 5:
        #    threshold = 0.5
        #elif max_indx == 6:
        #    threshold = 0.15
        #else:
        #    threshold = 1.75

        if counts_three >=7:
            threshold = 3
        elif counts_two_half >= 10:
            threshold = 2.5
        elif counts_two >= 18:
            threshold = 2
        elif counts_one_half >= 30:
            threshold = 1.5
        elif counts_one >= 40:
            threshold = 1
        elif counts_half >= 70:
            threshold = 0.5
        elif counts_zero >= 100:
            threshold = 0.15

        return threshold
    
    def findGap(self, bubbledArray, threshold):
        min_idx = -1
        max_idx = -1
        min_idx_curr = -1
        gap = 0
        biggestGap = 0
        indxSteer = 0
        #threshold = self.threshold_dist
        self.get_logger().info("bubble length: " + str(len(bubbledArray)))
        for j in range(0,10):
            #self.get_logger().info("In while loop")
            i = 0
            for i in range(0,len(bubbledArray)):
                #self.get_logger().info("In for loop " + str(i) + " " + str(threshold))
                if bubbledArray[i] != "nan":
                    if bubbledArray[i] >= threshold:
                        self.get_logger().info("gap starting: " + str(bubbledArray[i]) + "  " + str(threshold))
                        if gap == 0:
                            min_idx_curr = i
                        gap = gap + 1
                        if gap > biggestGap:
                            biggestGap = gap
                            min_idx = min_idx_curr
                            max_idx = i
                    else:
                        gap = 0
                        min_idx_curr = -1
                else:
                    gap = 0
                    min_idx_curr = -1
            if (threshold > 0.5):
                if (min_idx == -1 and max_idx == -1):
                    threshold = threshold - 0.25
                elif (min_idx != -1 and max_idx != -1):
                    indxSteer = int((min_idx + max_idx)/2)
                    break
            else:
                if (min_idx != -1 and max_idx != -1):
                    indxSteer = int((min_idx + max_idx)/2)
                    break
                elif (min_idx == -1 and max_idx == -1):
                    indxSteer = int(len(bubbledArray)/2)
                    break
        self.get_logger().info("gap done: " + str(min_idx) + "  " + str(max_idx))
        
        return (indxSteer)

def main(args=None):
    rclpy.init(args=args)
    node = FindGapAndUart()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
