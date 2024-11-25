import cv2
from ament_index_python.packages import get_package_share_directory
bringup_dir = get_package_share_directory('coordinator')
# Global variables
image_path =  bringup_dir + "/map/" + "map" + '.png'
zoom_factor = 1.0
zoom_increment = 0.1

def update_zoom(event, x, y, flags, param):
    global zoom_factor
    print(event)
    if event == cv2.EVENT_MOUSEWHEEL:
        if flags > 0:
            zoom_factor += zoom_increment
        else:
            zoom_factor -= zoom_increment

        # Keep the zoom factor within bounds
        zoom_factor = max(zoom_factor, zoom_increment)
    
    display_image()

def display_image():
    global zoom_factor

    original_image = cv2.imread(image_path)
    resized_image = cv2.resize(original_image, None, fx=zoom_factor, fy=zoom_factor, interpolation=cv2.INTER_LINEAR)

    cv2.imshow('Zoomable Image', resized_image)

# Load the image from a file
cv2.namedWindow('Zoomable Image')
cv2.setMouseCallback('Zoomable Image', update_zoom)

display_image()

cv2.waitKey(0)
cv2.destroyAllWindows()
