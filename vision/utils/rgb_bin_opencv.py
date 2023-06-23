import sys
import numpy as np
import cv2

def show_rgba_image(file_path):
    # Read binary file
    with open(file_path, "rb") as file:
        image_data = file.read()
    # Convert bytes to numpy array
    image_array = np.frombuffer(image_data, dtype=np.uint8)

    # Reshape the array to obtain image dimensions
    # image_shape = (int(len(image_array) / 4), 1, 4)
    height = 480  # Specify the height of the matrix
    width = 640   # Specify the width of the matrix
    rgba_matrix = image_array.reshape((height, width, 4))
    # Convert RGBA to BGR
    image_bgr = cv2.cvtColor(rgba_matrix, cv2.COLOR_RGBA2BGR)

    # Display the image
    cv2.imshow("RGBA Image", image_bgr)
    cv2.imshow("RGBA Image2", rgba_matrix)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if len(sys.argv) < 2:
    exit(-1)
file_path = sys.argv[1]
print(file_path)
show_rgba_image(file_path)
