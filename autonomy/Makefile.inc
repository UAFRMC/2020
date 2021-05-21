
# OpenCV flags: Ubuntu 20.04 OpenCV4 version
CVCFLAGS=`pkg-config opencv4 --cflags`
CVLINK=-lpthread `pkg-config opencv4 --cflags --libs`

# Old OpenCV flags for older machines
#CVCFLAGS=`pkg-config opencv --cflags`
#CVLINK=-lpthread `pkg-config opencv --cflags --libs`

