(ros::load-ros-manifest "audio_video_recorder")
(ros::load-ros-manifest "sensor_msgs")


(defun call-start-record-service
    (audio-topic-name
     image-topic-name
     file-name
     video-framerate
     &key
     (queue-size 100)
     (file-format "avi")
     (audio-format "mp3")
     (audio-sample-format "S16LE")
     (audio-channels 1)
     (audio-depth 16)
     (audio-sample-rate 16000)
     (video-encoding "RGB")
     (video-height nil)
     (video-width nil))
 (let (msg req res)
   (if (or (not video-height) (not video-width))
     (progn
       (setq msg (one-shot-subscribe image-topic-name sensor_msgs::Image :timeout 5000))
       (if (not msg)
         (progn
           (ros::ros-error "Image is not published from ~A" image-topic-name)
           (return-from call-start-record-service nil)))
       (setq video-height (send msg :height))
       (setq video-width (send msg :width))))
   (setq req (instance audio_video_recorder::StartRecordRequest :init))
   (send req :task :audio_topic_name audio-topic-name)
   (send req :task :image_topic_name image-topic-name)
   (send req :task :queue_size queue-size)
   (send req :task :file_name file-name)
   (send req :task :file_format file-format)
   (send req :task :audio_format audio-format)
   (send req :task :audio_sample_format audio-sample-format)
   (send req :task :audio_channels audio-channels)
   (send req :task :audio_depth audio-depth)
   (send req :task :audio_sample_rate audio-sample-rate)
   (send req :task :video_encoding video-encoding)
   (send req :task :video_height video-height)
   (send req :task :video_width video-width)
   (send req :task :video_framerate video-framerate)
   (setq res (ros::service-call "/audio_video_recorder_server/start_record" req t))
   (if (not (send res :success))
     (progn
       (ros::ros-error "Failed to start recording: ~A" (send res :message))
       (return-from call-start-record-service nil)))
   (send res :success)
   ))

(defun call-stop-record-service
  (file-name)
  (let (req res)
    (setq req (instance audio_video_recorder::StopRecordRequest :init))
    (send req :file_name file-name)
    (setq res (ros::service-call "/audio_video_recorder_server/stop_record" req t))
    (if (not (send res :success))
      (progn
        (ros::ros-error "Failed to stop recording: ~A" (send res :message))
        (return-from call-stop-record-service nil)))
    (send res :success)
    ))

(defun get-recording-task-array
  ()
  (let (msg)
    (setq msg (one-shot-subscribe "/audio_video_recorder_server/record_tasks" audio_video_recorder::RecordTaskArray :timeout 10000))
    (if (not msg)
      (progn
        (ros::ros-error "Failed to get a task array message.")
        (return-from get-recording-task-array nil)))
    (send msg :array)
    ))
