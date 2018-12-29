#!/usr/bin/env python
import cv2
import os
import numpy as np

if __name__ == '__main__':
	map = cv2.imread('../map.pgm', -1)
	np.set_printoptions(threshold=np.inf)
	print(map)
	print(os.getcwd())