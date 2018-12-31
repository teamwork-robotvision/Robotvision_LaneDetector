cmake_minimum_required(VERSION 3.10)
project(Robotvision_LaneDetector)

set(CMAKE_CXX_STANDARD 17)

add_executable(Robotvision_LaneDetector main1.cpp advance_lane_detect.cpp lane_process.cpp
        lanedetector.h LaneDetector.cpp LaneDetector.h roadsign.cpp RoadSign.h)

include_directories(/usr/local/include/opencv/include
        /usr/local/include/opencv/include/opencv2
        /usr/local/include/opencv/include/opencv)

link_directories(/usr/local/lib/opencv/lib)

target_link_libraries(Robotvision_LaneDetector -I/usr/local/include/opencv/include
        -I/usr/local/include/opencv/include/opencv
        -I/usr/local/include/opencv/include/opencv2)

target_link_libraries(Robotvision_LaneDetector /home/zhaobin/software/opencv/lib/libopencv_aruco.so
        /home/zhaobin/software/opencv/lib/libopencv_bgsegm.so
        /home/zhaobin/software/opencv/lib/libopencv_bioinspired.so
        /home/zhaobin/software/opencv/lib/libopencv_calib3d.so
        /home/zhaobin/software/opencv/lib/libopencv_ccalib.so
        /home/zhaobin/software/opencv/lib/libopencv_core.so
        /home/zhaobin/software/opencv/lib/libopencv_datasets.so
        /home/zhaobin/software/opencv/lib/libopencv_dnn_objdetect.so
        /home/zhaobin/software/opencv/lib/libopencv_dnn.so
        /home/zhaobin/software/opencv/lib/libopencv_dpm.so
        /home/zhaobin/software/opencv/lib/libopencv_face.so
        /home/zhaobin/software/opencv/lib/libopencv_features2d.so
        /home/zhaobin/software/opencv/lib/libopencv_flann.so
        /home/zhaobin/software/opencv/lib/libopencv_freetype.so
        /home/zhaobin/software/opencv/lib/libopencv_fuzzy.so
        /home/zhaobin/software/opencv/lib/libopencv_hfs.so
        /home/zhaobin/software/opencv/lib/libopencv_highgui.so
        /home/zhaobin/software/opencv/lib/libopencv_imgcodecs.so
        /home/zhaobin/software/opencv/lib/libopencv_img_hash.so
        /home/zhaobin/software/opencv/lib/libopencv_imgproc.so
        /home/zhaobin/software/opencv/lib/libopencv_line_descriptor.so
        /home/zhaobin/software/opencv/lib/libopencv_ml.so
        /home/zhaobin/software/opencv/lib/libopencv_objdetect.so
        /home/zhaobin/software/opencv/lib/libopencv_optflow.so
        /home/zhaobin/software/opencv/lib/libopencv_phase_unwrapping.so
        /home/zhaobin/software/opencv/lib/libopencv_photo.so
        /home/zhaobin/software/opencv/lib/libopencv_plot.so
        /home/zhaobin/software/opencv/lib/libopencv_reg.so
        /home/zhaobin/software/opencv/lib/libopencv_rgbd.so
        /home/zhaobin/software/opencv/lib/libopencv_saliency.so
        /home/zhaobin/software/opencv/lib/libopencv_shape.so
        /home/zhaobin/software/opencv/lib/libopencv_stereo.so
        /home/zhaobin/software/opencv/lib/libopencv_stitching.so
        /home/zhaobin/software/opencv/lib/libopencv_structured_light.so
        /home/zhaobin/software/opencv/lib/libopencv_superres.so
        /home/zhaobin/software/opencv/lib/libopencv_surface_matching.so
        /home/zhaobin/software/opencv/lib/libopencv_text.so
        /home/zhaobin/software/opencv/lib/libopencv_tracking.so
        /home/zhaobin/software/opencv/lib/libopencv_videoio.so
        /home/zhaobin/software/opencv/lib/libopencv_video.so
        /home/zhaobin/software/opencv/lib/libopencv_videostab.so
        /home/zhaobin/software/opencv/lib/libopencv_xfeatures2d.so
        /home/zhaobin/software/opencv/lib/libopencv_ximgproc.so
        /home/zhaobin/software/opencv/lib/libopencv_xobjdetect.so
        /home/zhaobin/software/opencv/lib/libopencv_xphoto.so)