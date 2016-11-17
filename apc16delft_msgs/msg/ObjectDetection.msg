Object object                        # The detected object.
float64 score                        # The score of the detection (0 to 1).
sensor_msgs/PointCloud2 point_cloud  # Cropped and processed point cloud containing the detected object.
geometry_msgs/Point centroid         # Centroid of the detected object.
apc16delft_msgs/BoundingBox2D box    # The bounding box of the object
