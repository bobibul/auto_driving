import ginyung as fl
import cv2
import os

def get_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # Convert BGR to HSV and get the HSV value at the clicked point
        hsv_value = cv2.cvtColor(param, cv2.COLOR_BGR2HSV)[y, x]
        print(f"Clicked coordinates: ({x}, {y})")
        print(f"HSV value: {hsv_value}")

if __name__ == "__main__":
    env_light = fl.libCAMERA(cam_num=0)

    # Ensure the 'img' directory exists
    if not os.path.exists('img'):
        os.makedirs('img')
    
    img_count = 0  # Counter for saved images
    
    while True:
        ret, env_light.frame = env_light.cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Resize the frame to make the display larger (e.g., double the original size)
        scale_percent = 200  # Percentage of the original size
        width = int(env_light.frame.shape[1] * scale_percent / 100)
        height = int(env_light.frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        
        # Resize the image
        resized_frame = cv2.resize(env_light.frame, dim, interpolation=cv2.INTER_LINEAR)

        # Set the window to be resizable and specify the size
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("img", width, height)

        # Set the mouse callback to the resized frame
        cv2.setMouseCallback("img", get_hsv_value, param=resized_frame)
        
        # Display the resized image
        cv2.imshow('img', resized_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # Press 'q' to exit the loop
            break
        elif key == ord('c'):  # Press 'c' to capture and save the image
            img_filename = f"img/captured_image_{img_count}.png"
            cv2.imwrite(img_filename, resized_frame)
            print(f"Image saved as {img_filename}")
            img_count += 1  # Increment the counter for the next image

    cv2.destroyAllWindows()