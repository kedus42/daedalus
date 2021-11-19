rostopic pub /daedalus_leg_l_even_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0.01,0.01,0.01,0.01,0.01,0.01,0.14,0.14,0.14,0.14,0.14,0.14]" &

rostopic pub /daedalus_leg_r_even_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0.01,0.01,0.01,0.01,0.01,0.01,0.14,0.14,0.14,0.14,0.14,0.14]" &

rostopic pub /daedalus_leg_l_uneven_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0.01,0.01,0.01,0.01,0.01,0.01,0.14,0.14,0.14,0.01,0.01,0.01]" &

rostopic pub /daedalus_leg_r_uneven_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0.01,0.01,0.01,0.01,0.01,0.01,0.14,0.14,0.14,0.01,0.01,0.01]" &
