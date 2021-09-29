cd /sys/class/gpio
echo 448 > export
cd /home/amov/Prometheus/Scripts
sudo chmod 666 /dev/ttyTHS0
cd /home/amov/Prometheus/Modules/object_detection_yolov5tensorrt
{
gnome-terminal -t "2" -x bash -c "python3 yolov5_trt_webcam.py; exec bash"
}

{
gnome-terminal -t "2" -x bash -c "roslaunch p450_experiment p450_vio_onboard.launch; exec bash"
}

{
gnome-terminal -t "1" -x bash -c "sleep 10s; roslaunch prometheus_detection yolov5_test.launch; exec bash"
}






