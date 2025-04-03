from ultralytics import YOLO
from fuzzywuzzy import fuzz
import os
import numpy as np
import cv2
import easyocr

DIR = "test/ocr_tests/images"
BOOK_ID = 0
num_files = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])
segmentation_model = YOLO("test/ocr_tests/yolo_custom_books.pt")
reader = easyocr.Reader(['en'])
ocr_results = []
accurate_detections = 0
inaccurate_detections = 0
COUNT = 0
KNOWN_BOOKS = ["ripe",
               "s is for science",
               "the open laboratory",
               "interactive science",
               "the science of the bottom line",
               "arithmetic optimization techniques for hardware and software design",
               "arrival of the fittest",
               "guns, germs and steel",
               "computer concepts",
               "lem",
               "template matching techniques in computer vision",
               "the philosophy of computer games",
               "the visual display of quantitative information",
               "cognitive robotics",
               "the jungle book",
               "the sherlock holmes handbook",
               "pride and prejudice"]

# Function to run segmentation and OCR
def run_segmentation():
    global COUNT, accurate_detections,inaccurate_detections
    while COUNT != 15:
        for i in range(1, num_files + 1):

            results = segmentation_model(f"test/ocr_tests/images/shelf_{i}.jpg")
            
            for result in results:
                boxes = result.boxes.xywh  
                

                img = cv2.imread(f"test/ocr_tests/images/shelf_{i}.jpg")
                
                for box in boxes:

                    x_center, y_center, width, height = box.cpu().numpy()

                    # Convert to top-left (x_min, y_min) and bottom-right (x_max, y_max) coordinates
                    x_min = int(x_center - width / 2)
                    y_min = int(y_center - height / 2)
                    x_max = int(x_center + width / 2)
                    y_max = int(y_center + height / 2)
                    
                    # Draw the bounding box on the image
                    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  # Green box with thickness 2

                    # Crop the detected region using the bounding box
                    cropped_img = img[y_min:y_max, x_min:x_max]

                    # Run OCR on the cropped image
                    text = reader.readtext(cropped_img, detail=0)  
                    wordsMatched = ' '.join(text).lower()  
                    test_similarity(wordsMatched)
        COUNT += 1  
        ocr_results.append([accurate_detections,inaccurate_detections])
        accurate_detections = 0
        inaccurate_detections = 0
    print(ocr_results)

def test_similarity(wordsFound):
    global accurate_detections,inaccurate_detections
    for book in KNOWN_BOOKS:
        print(wordsFound)
        similarity_score = fuzz.token_sort_ratio(book, wordsFound)
        print(similarity_score)
        if similarity_score >= 50:
            accurate_detections += 1
            return True
    inaccurate_detections += 1
    return False

if __name__ == "__main__":
    run_segmentation()