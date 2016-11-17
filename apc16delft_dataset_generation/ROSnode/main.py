import rospy
from ../objectDetection/objectDetection import detect

def handle_object_detection(req):
    
    return 1 
    
    
def object_detection_server():
    ## Define a topic on which the results will be published
    #pub = rospy.Publisher("boundingBox", String, queue_size=10)

    # Communicate the name of the node to the ROScore
    rospy.init_node('object_detection_server')
    
    # forward all service calls to the service handler
    s = rospy.Service('object_detection', ObjectDetection, handle_object_detection)
    
    # keep code from exiting untill the service is shut down
    rospy.spin()
        
        
if __name__ == "__main__":
    object_detection_server()