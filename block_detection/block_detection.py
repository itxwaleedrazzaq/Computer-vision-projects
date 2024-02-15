import cv2
import numpy as np

def detect_all_blocks(image_path):
    original_image = cv2.imread(image_path)
    resized_image = cv2.resize(original_image, (800, 800))
    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
    # Apply adaptive thresholding
    _, thresh = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(resized_image, contours, -1, (255, 0, 0), 2)

    cv2.imshow('Detected Blocks', resized_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    image_path = 'masterplan.jpg'
    detect_all_blocks(image_path)
