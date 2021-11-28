#!/usr/bin/env roseus

(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "audio_common_msgs")

(load "package://audio_video_recorder/euslisp/audio-video-recorder-client.l")

(ros::roseus "audio_video_recorder_client_demo")

(setq destination (ros::get-param "~destination" "/tmp/"))
(setq file-name-01 (concatenate string destination "audio_video_recorder_server_demo_01.avi"))
(setq file-name-02 (concatenate string destination "audio_video_recorder_server_demo_02.avi"))

(setq msg-image (one-shot-subscribe "/usb_cam_node/image_raw" sensor_msgs::Image :timeout 10000))
(setq msg-audio (one-shot-subscribe "/audio" audio_common_msgs::AudioData :timeout 10000))

(if (or (not msg-image) (not msg-audio))
  (progn
    (ros::ros-error "Failed to get image or audio message")
    (exit 1)))

(ros::ros-info "Start a recording task")
(call-start-record-service "/audio" "/usb_cam_node/image_raw" file-name-01 30)
(unix::sleep 5)
(ros::ros-info "Start another recording task")
(call-start-record-service "/audio" "/usb_cam_node/image_raw" file-name-02 30)
(unix::sleep 2)
(ros::ros-info "Get a recording task array")
(setq task-array (get-recording-task-array))
(ros::ros-info " : ~A" task-array)
(unix::sleep 3)
(ros::ros-info "Stopped the first recording task.")
(call-stop-record-service file-name-01)
(unix::sleep 5)
(ros::ros-info "Stopped the second recording task.")
(call-stop-record-service file-name-02)

(exit 0)
