import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

# Load the image
img = mpimg.imread('controllers/main/map.png')

# Convert the image to grayscale
img_gray = np.dot(img[...,:3], [0.2989, 0.5870, 0.1140])

# Flip the image
flipped_img = np.flip(img_gray, 1)
print(flipped_img)
# Display the flipped image
plt.imshow(flipped_img, vmin=-1, vmax=1, cmap='gray', origin='lower')
plt.show()