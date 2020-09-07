import cv2
import numpy as np
import torch

# Take in an identified section of the image,
# return a cropped and resized section which our model can use
def extract_digit(frame, img, rect, pad = 10, SIZE=28):
    x, y, w, h = rect
    cropped_digit = img[y-pad:y+h+pad, x-pad:x+w+pad]
    cropped_digit = cropped_digit/255.0

    #only look at images that are somewhat big:
    if cropped_digit.shape[0] < 32 or cropped_digit.shape[1] < 32:
        return

    return cv2.resize(cropped_digit, (SIZE, SIZE))


# Do intial filtering on frame to make
# our other vision possible
def img_to_mnist(frame):
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
    #adaptive here does better with variable lighting:
    gray_img = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, blockSize = 321, C = 28)

    return gray_img


# Take in a frame, return an integer 
# representing a number found in the frame
def find_label(frame, network):
    final_img = img_to_mnist(frame)
    contours, _ = cv2.findContours(final_img.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)

    rects = [cv2.boundingRect(contour) for contour in contours]
    rects = [rect for rect in rects if rect[2] >= 3 and rect[3] >= 8]

    ret = -1

    # Identify the largest rectangle to draw (if there is one)
    if len(rects) > 0:
        max_rect_i = np.argmax([rect[2] * rect[3] for rect in rects])
        rect = rects[max_rect_i]

        #Draw the rectangle we singled out
        mnist_frame = extract_digit(frame, final_img, rect, pad = 15)
        # cv2.imshow('seen', mnist_frame)
        if mnist_frame is not None:
            model_in = torch.tensor(mnist_frame).float()
            # lift model into appropriate dim
            model_in = model_in.unsqueeze(0).unsqueeze(0)

            with torch.no_grad():
                output = network(model_in)
                label = output.data.max(1, keepdim=True)[1][0][0]
            ret = int(label)
    return ret