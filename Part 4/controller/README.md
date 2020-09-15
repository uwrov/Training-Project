# Buliding cam_control.py

Now we'll apply our newfound ML knowledge to a ROS controller.

Here's our roadmap for this file
- Make a new thread to stream video from a source
- Load a trained model to recognized handwritten digits
- Publish findings to '/wheely_boi/wheely_boi/cmd'


## Threading
The vm we're running our image recognition from doesn't have that many resources. So if we try to toss every frame of our video stream into the model, we'll be met with poor performance and a lot of lag.

To avoid this, we'll create a new thread which is dedicated to streaming the video, and occasionally grab and process a frame from it every second or so.

Now let's jump into the code.

```Python
from threading import Thread
from tools.VideoStream import VideoStream

...

stream = VideoStream(src)
stream.start()
```

This chunk of code creates a new VideoStream object, and that's where the threading magic happens.

```Python
class VideoStream(Thread):
    def __init__(self, src=0):
        Thread.__init__(self)
```
`VideoStream` is an object which extends `Thread`, which means it has support for threading. We call `Thread.__init__(self)` to call the superclass' constructor for the object.

Now when run `stream.start()`, we'll start a new thread which will run `VideoStream.run()`, which is a while loop which will just read from the video source.

Now back in the while loop in `cam_control.py` we'll grab a frame with `frame=stream.frame`. This is the frame that we'll throw into our model for prediction.

## Digit Recognition
It's a little hidden in this function but all our digit recognition happens within a call to `find_label(frame, network)` within our while loop.

`find_label()` is a function which lives in `cam_util.py`, a utility file which contains some useful functions.

Let's walk through a call to `find_label()`:
```Py
final_img = img_to_mnist(frame)

def img_to_mnist(frame):
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
    #adaptive here does better with variable lighting:
    gray_img = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, blockSize = 321, C = 28)

    return gray_img
```
First we'll do some filtering on our input frame with `img_to_mnist()`. Recall that the images in the MNIST dataset are all in black and white, but our camera pictures things in color, so we'll apply some filters to it to get in a usable state. Going play by play:
1. `cv2.CvtColor`: First we'll convert our image to grayscale
2. `cv2.GaussianBlur`: Next we'll apply a blur to the image to reduce noise. This is helpful as it'll reduce the amount of contours we'll have to sift through later.
3. `cv2.adaptiveThreashold`: This function will make our image close to black and white. It's responsible for making our image look like an MNIST data point.

```Python
contours, _ = cv2.findContours(final_img.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
```
Next, we'll apply contours and identify the boundaries in our image. This will hopefully give us an outline of our handwritten digit if `img_to_mnist` did its job correctly.

```py
rects = [cv2.boundingRect(contour) for contour in contours]
rects = [rect for rect in rects if rect[2] >= 3 and rect[3] >= 8]

...

max_rect_i = np.argmax([rect[2] * rect[3] for rect in rects])
rect = rects[max_rect_i]

```
Here we are singling out the largest rectangle present in the image (if there is one) and drawing a bounding box around it.

```py
mnist_frame = extract_digit(frame, final_img, rect, pad = 15)

def extract_digit(frame, img, rect, pad = 10, SIZE=28):
    cropped_digit = img[y-pad:y+h+pad, x-pad:x+w+pad]
    cropped_digit = cropped_digit/255.0

    #only look at images that are somewhat big:
    if cropped_digit.shape[0] < 32 or cropped_digit.shape[1] < 32:
        return

    return cv2.resize(cropped_digit, (SIZE, SIZE))
```
Given the predetermined rectangle and its contents, the image is cropped and resized to better fit our model using the `extract_digit()` method. Smaller images are ignored.

```py
model_in = torch.tensor(mnist_frame).float()
# lift model into appropriate dim
model_in = model_in.unsqueeze(0).unsqueeze(0)

with torch.no_grad():
    output = network(model_in)
    label = output.data.max(1, keepdim=True)[1][0][0]
ret = int(label)
```
Now we'll run into some PyTorch voodoo. First we'll spend some time getting an appropriate `model_in` into a format that our neural network will accept.

Next, we'll do `output = network(model_in)` to get an output from our network, then extract the corresponding label from the output and return it.

When you run `cam_control.py`, you might notice that the program has pretty poor accuracy. How can this happen when our network achieves >90% accuracy on the test data?

This inaccuracy most likely comes from the cropped images not looking enough like MNIST data points. So our next step in developing this controller would be to find the differences between our data and MNIST, then make appropriate changes based on that.

## Publishing
Our publishing is essentially the same as it was in `key_in.py`, but with different signals.

In our while loop, we'll simply tell ROS to publish a message according to what our model decides the output should be.
```py
publish(find_label(frame, network), t, velocity_publisher)

def publish(signal, t, velocity_publisher):
    if (signal == 8):
        t.linear.x = min(t.linear.x + 0.1, 1.0)
    elif (signal == 4):
        t.angular.z = min(t.angular.z + 0.1, 1.0)
    elif (signal == 6):
        t.linear.x = max(t.linear.x - 0.1, -1.0)
    elif (signal == 2):
        t.angular.z = max(t.angular.z - 0.1, -1.0)
    else:
        t.linear.x = 0
        t.angular.z = 0

    t.linear.x = round(t.linear.x, 1)
    t.angular.z = round(t.angular.z, 1)

    rospy.loginfo("Sending Command v:" + str(t.linear.x)
                    + ", y:" + str(t.angular.z))
    velocity_publisher.publish(t)
```

`publish(signal, t, velocity_publisher)` is essentially lifted from `key_in.py`.