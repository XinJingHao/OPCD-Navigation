;; Auto-generated. Do not edit!


(when (boundp 'gazebo_sfm_plugin::ped_state)
  (if (not (find-package "GAZEBO_SFM_PLUGIN"))
    (make-package "GAZEBO_SFM_PLUGIN"))
  (shadow 'ped_state (find-package "GAZEBO_SFM_PLUGIN")))
(unless (find-package "GAZEBO_SFM_PLUGIN::PED_STATE")
  (make-package "GAZEBO_SFM_PLUGIN::PED_STATE"))
(unless (find-package "GAZEBO_SFM_PLUGIN::PED_STATEREQUEST")
  (make-package "GAZEBO_SFM_PLUGIN::PED_STATEREQUEST"))
(unless (find-package "GAZEBO_SFM_PLUGIN::PED_STATERESPONSE")
  (make-package "GAZEBO_SFM_PLUGIN::PED_STATERESPONSE"))

(in-package "ROS")





(defclass gazebo_sfm_plugin::ped_stateRequest
  :super ros::object
  :slots (_name ))

(defmethod gazebo_sfm_plugin::ped_stateRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass gazebo_sfm_plugin::ped_stateResponse
  :super ros::object
  :slots (_px _py _pz _vx _vy _theta ))

(defmethod gazebo_sfm_plugin::ped_stateResponse
  (:init
   (&key
    ((:px __px) 0.0)
    ((:py __py) 0.0)
    ((:pz __pz) 0.0)
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:theta __theta) 0.0)
    )
   (send-super :init)
   (setq _px (float __px))
   (setq _py (float __py))
   (setq _pz (float __pz))
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _theta (float __theta))
   self)
  (:px
   (&optional __px)
   (if __px (setq _px __px)) _px)
  (:py
   (&optional __py)
   (if __py (setq _py __py)) _py)
  (:pz
   (&optional __pz)
   (if __pz (setq _pz __pz)) _pz)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:serialization-length
   ()
   (+
    ;; float64 _px
    8
    ;; float64 _py
    8
    ;; float64 _pz
    8
    ;; float64 _vx
    8
    ;; float64 _vy
    8
    ;; float64 _theta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _px
       (sys::poke _px (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _py
       (sys::poke _py (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pz
       (sys::poke _pz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _px
     (setq _px (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _py
     (setq _py (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pz
     (setq _pz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vx
     (setq _vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vy
     (setq _vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta
     (setq _theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass gazebo_sfm_plugin::ped_state
  :super ros::object
  :slots ())

(setf (get gazebo_sfm_plugin::ped_state :md5sum-) "05beeb87454630bfc7ea88a89fd9b876")
(setf (get gazebo_sfm_plugin::ped_state :datatype-) "gazebo_sfm_plugin/ped_state")
(setf (get gazebo_sfm_plugin::ped_state :request) gazebo_sfm_plugin::ped_stateRequest)
(setf (get gazebo_sfm_plugin::ped_state :response) gazebo_sfm_plugin::ped_stateResponse)

(defmethod gazebo_sfm_plugin::ped_stateRequest
  (:response () (instance gazebo_sfm_plugin::ped_stateResponse :init)))

(setf (get gazebo_sfm_plugin::ped_stateRequest :md5sum-) "05beeb87454630bfc7ea88a89fd9b876")
(setf (get gazebo_sfm_plugin::ped_stateRequest :datatype-) "gazebo_sfm_plugin/ped_stateRequest")
(setf (get gazebo_sfm_plugin::ped_stateRequest :definition-)
      "# client
string name
---
# server
float64 px
float64 py
float64 pz
float64 vx
float64 vy
float64 theta
")

(setf (get gazebo_sfm_plugin::ped_stateResponse :md5sum-) "05beeb87454630bfc7ea88a89fd9b876")
(setf (get gazebo_sfm_plugin::ped_stateResponse :datatype-) "gazebo_sfm_plugin/ped_stateResponse")
(setf (get gazebo_sfm_plugin::ped_stateResponse :definition-)
      "# client
string name
---
# server
float64 px
float64 py
float64 pz
float64 vx
float64 vy
float64 theta
")



(provide :gazebo_sfm_plugin/ped_state "05beeb87454630bfc7ea88a89fd9b876")


