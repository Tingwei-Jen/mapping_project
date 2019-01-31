## vins mono on euroc
roslaunch vins_estimator euroc.launch 
roslaunch vins_estimator vins_rviz.launch
rosbag play datasets_euroc_kitti/MH_01/MH_01_easy.bag
# with ground truth
roslaunch benchmark_publisher publish.launch  sequence_name:=MH_01_easy



## vins mono on kitti
roslaunch vins_estimator kitti.launch 
roslaunch vins_estimator vins_rviz.launch
rosbag play datasets_euroc_kitti/kitti/city0093/kitti_2011_09_26_drive_0093_synced.bag



## vins mono on MYNT-EYE
roslaunch vins_estimator mynteye.launch 
roslaunch vins_estimator vins_rviz.launch
roslaunch mynteye_wrapper_d mynteye.launch

# record data 
rosbag record -O subset /mynteye/imu/data_raw /mynteye/left/image_mono /vins_estimator/camera_pose


## extract bag file
rosrun rosbag_to_csv rosbag_to_csv.py
python export.py


## realsense d435i vins
# sync imu and camera, gyro and accl
# In rs_camera.launch, enable_sync-->true; unite_imu_method-->copy 
# publish message only when accl
roslaunch vins_estimator realsense_d435i.launch
roslaunch vins_estimator vins_rviz.launch
roslaunch realsense2_camera rs_camera.launch

## record
rosbag record -O subset /camera/color/image_raw /camera/imu 
rosbag record -O subset /camera/color/image_raw /camera/gyro/sample /camera/accel/sample

## check ros bag
rosrun rqt_bag rqt_bag


## test realsense SDK
realsense-viewer
./librealsense/build/examples/capture/rs-capture
