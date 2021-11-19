from gazebo_msgs.srv import SetPhysicsProperties
from std_msgs.msg import Float64    
import rospy, geometry_msgs, gazebo_msgs

set_gravity = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

time_step = Float64(0.001)
max_update_rate = Float64(1000.0)
gravity = geometry_msgs.msg.Vector3()
gravity.x = 0.0
gravity.y = 0.0
gravity.z = 0.0
ode_config = gazebo_msgs.msg.ODEPhysics()
ode_config.auto_disable_bodies = False
ode_config.sor_pgs_precon_iters = 0
ode_config.sor_pgs_iters = 50
ode_config.sor_pgs_w = 1.3
ode_config.sor_pgs_rms_error_tol = 0.0
ode_config.contact_surface_layer = 0.001
ode_config.contact_max_correcting_vel = 100.0
ode_config.cfm = 0.0
ode_config.erp = 0.2
ode_config.max_contacts = 20
set_gravity(time_step.data, max_update_rate.data, gravity, ode_config)