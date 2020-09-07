import cv2
import numpy as np
import torch

# Take in an identified section of the image,
# return a cropped and resized section which our model can use
def extract_digit(frame, img, rect, pad = 10, SIZE=28):
    x, y, w, h = rect

    # STEP 1: Do some filtering on given rectancles which 
    #         are very small (Hint: check line 7)

    # STEP 2: Use the input rectangle's dimensions to crop out
    #         area of interest within the image
    #         HINT: You can take out a slice of an image using
    #         the syntax: img[y1:y2, x1:x2]
    cropped_digit = img / 255.0
    corpped_digit = cropped_digit[1, 1] # you might want to change this

    # STEP 3: resize and return our image to one of size SIZE x SIZE
    #         HINT: check out cv2.resize()

    return None


# Do intial filtering on frame to make
# our other vision possible
def img_to_mnist(frame):
    # STEP 4: Turn image into grayscale using cv2.cvtColor and cv2.COLOR_BGR2GRAY
    gray_img = frame # you might want to change this

    # STEP 5: Blur grayscale image using cv2.GaussianBlur
    gray_img = frame # you might want to change this
 
    gray_img = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, blockSize = 321, C = 28)

    return gray_img


# Take in a frame, return an integer 
# representing a number found in the frame
def find_label(frame, network):
        # STEP 6: Apply some filtering to our initial video fram
        #         to make image processing possible
        #         (Hint: read comments for each function)
        final_img = frame  # change me!

        # STEP 7: Find the contours within final_img using cv2.findContours
        _, contours, _ = None  # Write different code here

        # STEP 8: Draw a bounding box around each of the found contours
        rects = [None, None]  # You might want to change this
        rects = [rect for rect in rects if rect[2] >= 3 and rect[3] >= 8]

        ret = -1

        # Identify the largest rectangle to draw (if there is one)
        if len(rects) > 0:
            max_rect_i = np.argmax([rect[2] * rect[3] for rect in rects])
            rect = rects[max_rect_i]

            # STEP 9: Cut out rect from the image
            mnist_frame = frame  # You might want to change this

            if mnist_frame is not None:
                model_in = torch.tensor(mnist_frame).float()
                # lift model into appropriate dim
                model_in = model_in.unsqueeze(0).unsqueeze(0)

                with torch.no_grad():
                    output = network(model_in)
                    label = output.data.max(1, keepdim=True)[1][0][0]
                ret = int(label)
        return ret