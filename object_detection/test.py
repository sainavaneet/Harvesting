import cv2

# Create a black image
image = cv2.imread('/home/dexweaver/Github/cucumber-harvesting/object_detection/simple.webp')  # Ensure you provide a valid path to an image file

cv2.imshow('Test Window', image)
cv2.waitKey(0)
cv2.destroyAllWindows()