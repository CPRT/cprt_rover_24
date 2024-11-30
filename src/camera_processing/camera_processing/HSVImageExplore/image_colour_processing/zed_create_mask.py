import cv2


def display(img_to_display, name):
    resized_img = cv2.resize(img_to_display, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_AREA)
    cv2.imshow(name, resized_img)
    cv2.waitKey(100)


img = cv2.imread('/home/erik/Documents/ZED/ImageToCreateMask/ZedMaskStep1.png')
print(f"Size: {img.shape}")
display(img, "Original Image")

ret, threshold_image = cv2.threshold(img, 255, 127, cv2.THRESH_BINARY)
print(f"threshold_image.shape: {threshold_image.shape}")

display(threshold_image, "Threshold Image")



# image, cnts, hierarchy = cv2.findContours(threshold_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
# cnt = sorted(cnts, key=cv2.contourArea)

# cv2.drawContours(threshold_image, cnt, 1, (0, 255, 0), 3)

# display(threshold_image)



try:
    while True:
        pass
except KeyboardInterrupt:
    cv2.destroyAllWindows()