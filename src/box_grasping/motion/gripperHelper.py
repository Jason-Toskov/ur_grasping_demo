def getWidthString(width, force):
    WIDTH = width
    FORCE = force
    return TEMP.format(width, force)


TEMP = """    
def unnamed():
  set_standard_analog_input_domain(0, 1)
  set_standard_analog_input_domain(1, 1)
  set_tool_analog_input_domain(0, 1)
  set_tool_analog_input_domain(1, 1)
  set_analog_outputdomain(0, 0)
  set_analog_outputdomain(1, 0)
  set_input_actions_to_default()
  step_count_0699da5f_db83_48b1_9e97_4ce0fa23bd6c = 0.0
  thread Step_Counter_Thread_37ed5b5d_9fc1_4b28_a53e_db2ae0ec4f61():
    while (True):
      step_count_0699da5f_db83_48b1_9e97_4ce0fa23bd6c = step_count_0699da5f_db83_48b1_9e97_4ce0fa23bd6c + 1.0
      sync()
    end
  end
  run Step_Counter_Thread_37ed5b5d_9fc1_4b28_a53e_db2ae0ec4f61()
  set_tcp(p[0.0015344856138343379,-1.2348096930715063E-4,0.14636652558513413,-0.0274,0.0171,-1.5074])
  set_safety_mode_transition_hardness(1)
  set_tool_communication(True, 1000000, 2, 1, 1.5, 3.5)
  set_tool_output_mode(0)
  set_tool_digital_output_mode(0, 1)
  set_tool_digital_output_mode(1, 1)
  set_tool_voltage(24)
  set_target_payload(0.890000, [-0.002000, -0.002000, 0.071000], [0.001268, 0.001268, 0.001268, 0.000000, 0.000000, 0.000000])
  set_gravity([0.0, 0.0, 9.82])
  global Line_1=p[0.0,0.0,0.0,0.0,0.0,0.0]
  # begin: URCap Installation Node
  #   Source: Remote TCP & Toolpath, 1.2.65.build36, Universal Robots A/S
  #   Type: Remote TCP & Toolpath
  mc_check_divergence = True
  mc_lookahead = 0.03
  mc_gain = 2000
  mc_dt = 0.004
  
  mc_conv_tolnc = 0.001
  mc_conv_tol_s = 0.001
  mc_conv_loops = 100
  
  mc_ee_set_id = 2
  
  MC_GROUP_STANDBY = 0
  MC_GROUP_STOPPING = 2
  MC_GROUP_ERROR_STOP = 3
  
  MC_ERROR_PATHDIVERGED = 0
  MC_ERROR_COLLISION = 1
  MC_ERROR_JOINTLIMIT = 2
  MC_ERROR_SINGULARITY = 4
  MC_ERROR_PLANNINGFAILED = 16
  MC_ERROR_DAEMONSTOPPED = 1024
  MC_ERROR_SERVER_BUSY = 1025
  MC_ERROR_NOT_INITIALIZED = 2000
  
  MC_TOOLPATH_ERROR_NONE = 0
  MC_TOOLPATH_ERROR_FILELOAD = 1
  MC_TOOLPATH_ERROR_PARSE = 2
  MC_TOOLPATH_ERROR_EMPTYTOOLPATH = 3
  MC_TOOLPATH_ERROR_UNSUPPORTEDTYPE = 4
  
  MC_GROUP_STATUS_INPUT = 24
  MC_CURRENT_MOTION_ID_INPUT = 25
  MC_JOINT_VALUE_INPUT = 24
  MC_ERROR_CODE_INPUT = 26
  MC_WATCHDOG_INPUT = 30
  MC_PATH_SPEED_INPUT = 31
  MC_SPEEDFACTOR_OUTPUT = 24
  
  mc_last_watchdog = 0
  mc_watchdog_counter = 0
  mc_watchdog_counter_limit = 100
  
  mc_last_motion_id = -1
  mc_group_status = MC_GROUP_STOPPING
  mc_server_started = False
  mc_rtcp_moving = False
  mc_debug_msg = False
  mc_gmm_initialized = False
  
  
  def mc_initialize(mode, tcp, doc=6):
  	mc_check_busy()
  
  	if (mode == 0 and doc == 6):
  		mc_ee_set_id = 0
  	end
  
  	if (mode == 1 and doc == 6):
  		mc_ee_set_id = 2
  	end
  
  	if (mode == 0 and doc == 5):
  		mc_ee_set_id = 1
  	end
  
  	if (mode == 1 and doc == 5):
  		mc_ee_set_id = 3
  	end
  
  	mc_initialize_internal(tcp)
  end
  
  def mc_initialize_internal(tcp):
  	if (mc_server_started == False):
  		global mc_server = rpc_factory("xmlrpc", "127.0.0.1:7828")
  		mc_server_started = True
  	end
  
  	mc_server.setDataStoreBoolean("IS_ON", "/robot/check_divergence", mc_check_divergence)
  	mc_server.reset()
  	mc_group_status = MC_GROUP_STANDBY
  	mc_set_speed_factor(1.0)
  	mc_server.setKinTransform(mc_ee_set_id, 0, tcp)
  	mc_gmm_initialized = False
  end
  
  def mc_check_busy():
  	enter_critical
  	if (mc_rtcp_moving == False):
  		mc_rtcp_moving = True
  	else:
  		mc_error_stop("", "", MC_ERROR_SERVER_BUSY)
  	end
  	exit_critical
  end
  
  def mc_add_linear(pose, a, v, r):
  	mc_check_ready()
  	ee_id = 0
  	buffer_mode = "BLENDING_NEXT"
  	transition_mode = "CORNER_DISTANCE"
  	id = mc_server.moveLinearAbsolute(pose, mc_ee_set_id, ee_id, v, a, buffer_mode, transition_mode, r)
  
  	if (id == -1):
  		mc_error_stop("Compute node returned ID ", id)
  	end
  
  	mc_last_motion_id = id
  
  	return id
  end
  
  def mc_add_circular(pose_via, pose_to, a, v, r, mode = 0):
  	mc_check_ready()
  	ee_id = 0
  	buffer_mode = "BLENDING_NEXT"
  	transition_mode = "CORNER_DISTANCE"
  	id = mc_server.moveCircularBorderAbsolute(pose_via, pose_to, mode, mc_ee_set_id, ee_id, v, a, buffer_mode, transition_mode, r)
  
  	if (id == -1):
  		mc_error_stop("Compute node returned ID ", id)
  	end
  
  	mc_last_motion_id = id
  
  	return id
  end
  
  def mc_add_path(path_id, a, v, r):
  	mc_check_ready()
  	ee_id = 0
  	buffer_mode = "BLENDING_NEXT"
  	transition_mode = "CORNER_DISTANCE"
  	id = mc_server.movePath(path_id, mc_ee_set_id, ee_id, v, a, buffer_mode, transition_mode, r)
  
  	if (id == -1):
  		result = mc_server.getErrorCode()
  		mc_toolpath_error_stop(result[0], result[1])
  	end
  
  	mc_last_motion_id = id
  
  	return id
  end
  
  def mc_load_path(nc_file, use_feedrate = False):
  	id = mc_server.loadPath(nc_file, use_feedrate)
  
  	if (id < 0):
  		result = mc_server.getErrorCode()
  		mc_toolpath_error_stop(result[0], result[1])
  	end
  
  	return id
  end
  
  def mc_get_target_rtcp_speed():
  	return read_input_float_register(MC_PATH_SPEED_INPUT)
  end
  
  def mc_check_ready():
  	if mc_group_status != MC_GROUP_STANDBY:
  		mc_error_stop("", "", MC_ERROR_NOT_INITIALIZED)
  	end
  end
  
  def mc_set_pcs(pcs):
  	ee_id = 0
  	mc_server.setCoordinateTransform(mc_ee_set_id, ee_id, pcs, "ABORTING")
  end
  
  def mc_run_motion(id=-1):
  	if (id == -1):
  		id = mc_last_motion_id
  	end
  
  	mc_debugMsg("Waiting for ID ", id)
  
  	if (mc_gmm_initialized == False):
  		mc_initialize_gmm()
  	end
  
  	mc_current_motion_id = read_input_integer_register(MC_CURRENT_MOTION_ID_INPUT)
  	mc_group_status = read_input_integer_register(MC_GROUP_STATUS_INPUT)
  	joint_targets = get_actual_joint_positions()
  	while (mc_current_motion_id <= id and mc_current_motion_id != -2 and mc_group_status != MC_GROUP_STOPPING and mc_group_status != MC_GROUP_ERROR_STOP):
  		mc_current_motion_id = read_input_integer_register(MC_CURRENT_MOTION_ID_INPUT)
  		mc_group_status = read_input_integer_register(MC_GROUP_STATUS_INPUT)
  
  		joint_targets = mc_get_joint_targets(joint_targets)
  		servoj(joint_targets, t = mc_dt, lookahead_time = mc_lookahead, gain = mc_gain)
  
  		mc_check_watchdog()
  	end
  
  	if (mc_group_status == MC_GROUP_STOPPING or mc_group_status == MC_GROUP_ERROR_STOP):
  		stopj(45)
  
  		mc_debugMsg("Stopping due to error after ID ", id)
  
  		mc_server.setDataStoreBoolean("IS_RTCP_RUNNING", "/group_motion_manager", False)
  
  		error_code = read_input_integer_register(MC_ERROR_CODE_INPUT)
  		mc_error_stop("Compute node returned group status ", mc_group_status, error_code)
  
  		mc_rtcp_moving = False
  		mc_group_status = MC_GROUP_STOPPING
  	end
  
  	if (id == mc_last_motion_id):
  		mc_debugMsg("Stopping after ID ", id)
  		mc_check_convergence(joint_targets)
  
  		mc_server.setDataStoreBoolean("IS_RTCP_RUNNING", "/group_motion_manager", False)
  
  		mc_rtcp_moving = False
  		mc_group_status = MC_GROUP_STOPPING
  	end
  
  	mc_debugMsg("Completed ID ", id)
  end
  
  def mc_set_speed_factor(s):
  	write_output_float_register(MC_SPEEDFACTOR_OUTPUT, s)
  end
  
  def mc_initialize_gmm():
  	mc_current_motion_id = read_input_integer_register(MC_CURRENT_MOTION_ID_INPUT)
  	while (mc_current_motion_id != -1):
  		mc_current_motion_id = read_input_integer_register(MC_CURRENT_MOTION_ID_INPUT)
  		sleep(mc_dt)
  		mc_check_watchdog()
  	end
  
  	mc_server.setDataStoreBoolean("IS_RTCP_RUNNING", "/group_motion_manager", True)
  
  	while (mc_current_motion_id == -1):
  		mc_current_motion_id = read_input_integer_register(MC_CURRENT_MOTION_ID_INPUT)
  		sleep(mc_dt)
  		mc_check_watchdog()
  	end
  
  	mc_gmm_initialized = True
  end
  
  def mc_check_convergence(joint_targets):
  	loopcounter = 0
  	converged = False
  	while (converged == False and loopcounter < mc_conv_loops):
  
  		joint_targets = mc_get_joint_targets(joint_targets)
  		diff = mc_get_diff_joints(get_actual_joint_positions(), joint_targets)
  		joint_speeds = get_actual_joint_speeds()
  
  		position_converged = diff[0] <= mc_conv_tolnc and diff[1] <= mc_conv_tolnc and diff[2] <= mc_conv_tolnc and diff[3] <= mc_conv_tolnc and diff[4] <= mc_conv_tolnc and diff[5] <= mc_conv_tolnc
  		speed_converged = joint_speeds[0] <= mc_conv_tol_s and joint_speeds[1] <= mc_conv_tol_s and joint_speeds[2] <= mc_conv_tol_s and joint_speeds[3] <= mc_conv_tol_s and joint_speeds[4] <= mc_conv_tol_s and joint_speeds[5] <= mc_conv_tol_s
  		converged = position_converged and speed_converged
  
  		servoj(joint_targets, t = mc_dt, lookahead_time = mc_lookahead, gain = mc_gain)
  
  		loopcounter = loopcounter + 1
  	end
  
  	mc_debugMsg("Convergence loop count: ", loopcounter)
  end
  
  def mc_check_watchdog():
  	current_watchdog = read_input_float_register(MC_WATCHDOG_INPUT)
  
  	if (current_watchdog > mc_last_watchdog):
  		mc_last_watchdog = current_watchdog
  		mc_watchdog_counter = 0
  	else:
  		mc_watchdog_counter = mc_watchdog_counter + 1
  		if (mc_watchdog_counter > mc_watchdog_counter_limit):
  			mc_error_stop("", "", MC_ERROR_DAEMONSTOPPED)
  		end
  	end
  end
  
  def mc_get_joint_targets(joint_targets):
  	joint_targets[0] = read_input_float_register(MC_JOINT_VALUE_INPUT)
  	joint_targets[1] = read_input_float_register(MC_JOINT_VALUE_INPUT + 1)
  	joint_targets[2] = read_input_float_register(MC_JOINT_VALUE_INPUT + 2)
  	joint_targets[3] = read_input_float_register(MC_JOINT_VALUE_INPUT + 3)
  	joint_targets[4] = read_input_float_register(MC_JOINT_VALUE_INPUT + 4)
  	joint_targets[5] = read_input_float_register(MC_JOINT_VALUE_INPUT + 5)
  	return joint_targets
  end
  
  def mc_get_diff_joints(x, y):
  	diff = x
  	diff[0] = norm(x[0] - y[0])
  	diff[1] = norm(x[1] - y[1])
  	diff[2] = norm(x[2] - y[2])
  	diff[3] = norm(x[3] - y[3])
  	diff[4] = norm(x[4] - y[4])
  	diff[5] = norm(x[5] - y[5])
  	return diff
  end
  
  def mc_error_stop(msg, param="", error_code=MC_ERROR_PLANNINGFAILED):
  	if (msg != ""):
  		textmsg(msg, param)
  	end
  
  	textmsg("Error code: ", error_code)
  	error_msg = mc_getErrorMessage(error_code)
  	textmsg(error_msg)
  
  	popup(error_msg, title="Remote TCP", error=True, blocking=False)
  	halt
  end
  
  def mc_toolpath_error_stop(error_code, param):
  	msg = mc_getToolpathMessage(error_code)
  	textmsg(msg + " - Line: ", param)
  
  	popup(msg, title="Toolpath Validation", error=True, blocking=True)
  	halt
  end
  
  def mc_debugMsg(msg, param=""):
  	if (mc_debug_msg == True):
  		textmsg(msg, param)
  	end
  end
  
  def mc_getErrorMessage(error_code):
  	if error_code == MC_ERROR_PATHDIVERGED:
  		return "<html>Robot cannot maintain tool speed. You may avoid this issue by:<ul><li>reducing the tool speed;</li><li>avoiding sharp corners;</li><li>staying away from singularities.</li></ul></html>"
  	elif error_code == MC_ERROR_COLLISION:
  		return "Robot is in collision."
  	elif error_code == MC_ERROR_JOINTLIMIT:
  		return "Robot is at a joint limit."
  	elif error_code == MC_ERROR_SINGULARITY:
  		return "Robot is at a singularity."
  	elif error_code == MC_ERROR_DAEMONSTOPPED:
  		return "The controller stopped."
  	elif error_code == MC_ERROR_SERVER_BUSY:
  		return "Another Remote TCP Move node is running."
  	elif error_code == MC_ERROR_NOT_INITIALIZED:
  		return "Group motion manager is not initialized."
  	else:
  		return "Motion planning failed."
  	end
  end
  
  def mc_getToolpathMessage(message_code):
  	if message_code == MC_TOOLPATH_ERROR_NONE:
  		return "Toolpath file is valid."
  	elif message_code == MC_TOOLPATH_ERROR_FILELOAD:
  		return "File cannot be loaded."
  	elif message_code == MC_TOOLPATH_ERROR_PARSE:
  		return "Parsing error"
  	elif message_code == MC_TOOLPATH_ERROR_EMPTYTOOLPATH:
  		return "Empty toolpath"
  	elif message_code == MC_TOOLPATH_ERROR_UNSUPPORTEDTYPE:
  		return "Unsupported type"
  	else:
  		return "Unknown toolpath validation error"
  	end
  end
  # end: URCap Installation Node
  # begin: URCap Installation Node
  #   Source: External Control, 1.0.5, FZI Research Center for Information Technology
  #   Type: External Control
  # end: URCap Installation Node
  # begin: URCap Installation Node
  #   Source: rs485, 1.0.0, FZI Research Center for Information Technology
  #   Type: RS485
  global rs485=rpc_factory("xmlrpc", "http://127.0.0.1:40404/RPC2")
  # end: URCap Installation Node
  # begin: URCap Installation Node
  #   Source: OnRobot, 6.0.0, OnRobot A/S
  #   Type: OnRobot Setup
  if (False):
    global rg_Busy = 0
    global rg_Depth = 0
    global rg_DepthRel = 0
    global rg_Grip_detected = 0
    global rg_Width = 0
  end
  ON_CONN_SHIFT_BOOL = 64
  ON_CONN_SHIFT_INT = 44
  ON_CONN_SHIFT_FLOAT = 45
  ON_CONN_REG_SUM_BOOL = 7
  ON_CONN_REG_SUM_INT = 4
  ON_CONN_REG_SUM_FLOAT = 3
  ON_TOOL_SHIFT_BOOL = 64
  ON_TOOL_SHIFT_INT = 45
  ON_TOOL_SHIFT_FLOAT = 45
  ON_TOOL_SHIFT_BOOL_ARR = [64, 71, 71]
  ON_TOOL_SHIFT_INT_ARR = [45, 48, 48]
  ON_TOOL_SHIFT_FLOAT_ARR = [45, 48, 48]
  ON_TOOL_REG_SUM_BOOL = 7
  ON_TOOL_REG_SUM_INT = 3
  ON_TOOL_REG_SUM_FLOAT = 3
  ON_REGISTERS_SPEEDL_FLOAT = 0
  ON_REG_USE_TOOL = False
  ON_DI_SINGLE = 0
  ON_DI_PRIMARY = 1
  ON_DI_SECONDARY = 2
  ON_DI_DUAL = 3
  on_robot_type = 3
  on_robot_cycle = 2.0
  on_conn_ip = "localhost"
  on_tool_ip = "localhost"
  on_device_socket_port = 51234
  on_conn_xmlrpc = rpc_factory("xmlrpc", "http://localhost:41414")
  on_tool_xmlrpc = rpc_factory("xmlrpc", "http://localhost:41414")
  on_RPC = rpc_factory("xmlrpc", "http://127.0.0.1:31416")
  ON_DEBUG_LOG = False
  on_isMetric = True
  on_toolConnector = True
  on_ioqc = False
  on_dual = False
  on_computebox = False
  on_devices = 1
  on_gripper = [False, False, False]
  on_custom_tcp_id = "none"
  on_custom_tcp_enabled = False
  on_ft = False
  rg_index = 0
  on_gripper[rg_index] = True
  #======    OnRobot Globals    ======#
  
  ON_MATH_PI=3.141593
  ON_ZEROPOSE=p[0.0,0.0,0.0,0.0,0.0,0.0]
  ON_ZEROFRAME=p[0.0,0.0,0.0,0.0,0.0,0.0]
  ON_ZERO3D=[0.0,0.0,0.0]
  ON_ZERO6D=[0.0,0.0,0.0,0.0,0.0,0.0]
  ON_ZERO8D=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
  ON_FALSE3D=[False,False,False]
  ON_FALSE6D=[False,False,False,False,False,False]
  ON_FALSE8D=[False,False,False,False,False,False,False,False]
  global on_return=0
  on_speedL=ON_ZERO6D
  on_speedBase=ON_ZERO6D
  on_speedVect=ON_ZERO6D
  on_speedExtra=ON_ZERO6D
  on_speedCB=ON_ZERO6D
  on_speedGecko=ON_ZERO6D
  on_RTDE_error=0
  on_RTDE_tool_error=0
  on_dataProcess_thrd=0
  ON_DEVICE_ID_MISSING=0
  ON_INIT_WATCHDOG_HZ=5
  ON_INIT_TIMEOUT=2000
  on_robot_TCP_offset=ON_ZEROFRAME
  on_tcp_offset_actual=ON_ZEROFRAME
  on_cog_actual=ON_ZEROFRAME
  on_mass_actual=0.0
  on_robot_mount=[0.0,0.0]
  
  #======    End of OnRobot Globals    ======#
  #======    OnRobot RG Globals    ======#
  
  RG_DEVICE_ID_RG2=32
  RG_DEVICE_ID_RG6=33
  if ON_DEBUG_LOG:
  textmsg("RG Dual: False, Index: ",rg_index)
  if False:
  rg_index=-1
  end 
  end 
  rg_Width_arr=[0,0,0]
  rg_Depth_arr=[0,0,0]
  rg_DepthRel_arr=[0,0,0]
  rg_product_code_arr=[0,0,0]
  rg_device_id_arr=[0,0,0]
  rg_Status_arr=[0,0,0]
  rg_Grip_detected_arr=[False,False,False]
  rg_Busy_arr=[False,False,False]
  rg_S1_pushed_arr=[False,False,False]
  rg_S1_triggered_arr=[False,False,False]
  rg_S2_pushed_arr=[False,False,False]
  rg_S2_triggered_arr=[False,False,False]
  rg_Safety_error_arr=[False,False,False]
  rg_Speed_arr=[0,0,0]
  rg_Angle_arr=[0,0,0]
  rg_Angle_speed_arr=[0,0,0]
  rg_Depth_prev_arr=[0,0,0]
  rg_speedDC=ON_ZERO6D
  rg_data_error_arr=[0,0,0]
  rg_mounting_angle_arr=[0,0,0]
  rg_fingertip_arr=[0,0,0]
  rg_Grip_guard_arr=[False,False,False]
  
  rg_Width=0
  rg_Depth=0
  rg_DepthRel=0
  rg_Busy=False
  rg_Grip_detected=False
  def get_rg_Width():
  return rg_Width
  end 
  def get_rg_Depth():
  return rg_Depth
  end 
  def get_rg_DepthRel():
  return rg_DepthRel
  end 
  def get_rg_Busy():
  return rg_Busy
  end 
  def get_rg_Grip_detected():
  return rg_Grip_detected
  end 
  
  rg2_mount_bracket_offset=35.0/1000.0
  rg2_mount_body_offset=p[0.0,0.0,0.180,0.0,0.0,0.0]
  rg2_mount_cog_offset=p[0.0,0.0,0.065,0.0,0.0,0.0]
  rg6_mount_bracket_offset=35.0/1000.0
  rg6_mount_body_offset=p[0.0,0.0,0.230,0.0,0.0,0.0]
  rg6_mount_cog_offset=p[0.0,0.0,0.080,0.0,0.0,0.0]
  
  #======    End of OnRobot RG Globals    ======#
  #======    OnRobot Interface Messages    ======#
  
  on_devices_primary_log="OnRobot Devices"
  on_devices_secondary_log="OnRobot Secondary Device"
  on_program_halted="<br>Program halted."
  on_device_error_title="OnRobot - Device error"
  on_install_error="OnRobot Setup is not correct.<br>Check the status in the OnRobot Setup page in the Installation Tab.<br>Program halted."
  on_device_missing="No device connected.<br>Program halted."
  cb_device_missing="No OnRobot Compute Box connected.<br>Program halted."
  ft_device_missing="No OnRobot FT sensor or license detected.<br>Program halted."
  hex_device_missing="No HEX sensor connected.<br>Program halted."
  rg2ft_device_missing="No RG2-FT gripper connected.<br>Program halted."
  rg_device_missing="No RG gripper connected.<br>Program halted."
  vg_device_missing="No VG gripper connected.<br>Program halted."
  gg_device_missing="No Gecko Gripper connected.<br>Program halted."
  sg_device_missing="No Soft Gripper connected.<br>Program halted."
  tfg_device_missing="No 3FG gripper connected.<br>Program halted."
  sdr_device_missing="No OnRobot Sander connected.<br>Program halted."
  twofg_device_missing="No 2FG gripper connected.<br>Program halted."
  vgp_device_missing="No VGP gripper connected.<br>Program halted."
  mg_device_missing="No Magnetic Gripper connected.<br>Program halted."
  fgp_device_missing="No 2FGP20 Gripper connected.<br>Program halted."
  eyes_device_missing="No OnRobot Eyes connected.<br>Program halted."
  on_xmlrpc_start_ip="Connecting to OnRobot XML-RPC Server:"
  on_java_comm_error_textmsg_title="OnRobot - Communication error:"
  on_java_comm_controlsocket_open_error="Socket 'javaSocket' opening was unsuccessful."
  on_java_comm_error_title="OnRobot - Communication error"
  on_java_comm_socket_open_error="Establishing connection with the URCap was timed out.<br>Check the status in the OnRobot Setup page in the Installation Tab.<br>Program halted."
  on_rtde_feed_error_textmsg_title="OnRobot - RTDE error:"
  on_rtde_feed_error="RTDE feed error. OnRobot device count mismatch.<br>Program halted."
  on_rtde_feed_tool_error="Tool RTDE feed error. OnRobot device count mismatch.<br>Program halted."
  on_rtde_feed_open_error_textmsg="Socket 'rtdeFeed' opening was unsuccessful."
  on_rtde_feed_error_title="OnRobot - RTDE error"
  on_rtde_feed_count_error="Invalid RTDE offset setup detected. Please check RTDE Offsets at the OnRobot Setup page in the Installation Tab.<br>Program halted."
  on_rtde_feed_open_error="Establishing connection with the devices was timed out.<br>Ensure that the OnRobot devices are operational, and check the status in the OnRobot Setup page in the Installation Tab."
  
  #======    End of OnRobot Interface Messages    ======#
  #======    OnRobot Interface    ======#
  
  on_portopened_javaSocket=False
  on_rtde_feed_opened=False
  on_dataProcess_running=False
  def on_missing():
  popup(on_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def cb_missing():
  popup(cb_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def ft_missing():
  popup(ft_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def hex_missing():
  popup(hex_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def rg2ft_missing():
  popup(rg2ft_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def rg_missing():
  popup(rg_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def vg_missing():
  popup(vg_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def gg_missing():
  popup(gg_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def sg_missing():
  popup(sg_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def tfg_missing():
  popup(tfg_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def sdr_missing():
  popup(sdr_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def twofg_missing():
  popup(twofg_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def vgp_missing():
  popup(vgp_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def mg_missing():
  popup(mg_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def fgp_missing():
  popup(fgp_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def eyes_missing():
  popup(eyes_device_missing,title=on_device_error_title,error=True,blocking=False)
  halt
  end 
  def on_portclose_javaSocket():
  socket_close("javaSocket")
  on_portopened_javaSocket=False
  end 
  def on_portopen_javaSocket():
  on_portclose_javaSocket()
  on_portopened_javaSocket=socket_open("127.0.0.1",44005,"javaSocket")
  if not on_portopened_javaSocket:
  textmsg(on_java_comm_error_textmsg_title,on_java_comm_controlsocket_open_error)
  popup(on_java_comm_socket_open_error,title=on_java_comm_error_title,error=True,blocking=False)
  halt
  end 
  end 
  on_conn_rtde_feed_name="rtdeFeedConn"
  on_tool_rtde_feed_name="rtdeFeedTool"
  def on_rtde_feed_close(rtdeFeedName):
  socket_close(rtdeFeedName)
  on_rtde_feed_opened=False
  end 
  def on_rtde_feed_open(deviceIP,rtdeFeedName,regStart,regSum,regSpeedl):
  on_rtde_feed_close(rtdeFeedName)
  if((regStart[0]+regSum[0])>128)or((regStart[1]+regSum[1])>48)or((regStart[2]+regSum[2])>48):
  popup(on_rtde_feed_count_error,title=on_rtde_feed_error_title,error=True,blocking=False)
  textmsg(str_cat("RegStart: ",regStart),str_cat("  -  RegSum: ",regSum))
  halt
  end 
  on_rtde_feed_opened=socket_open(deviceIP,on_device_socket_port,rtdeFeedName)
  if not on_rtde_feed_opened:
  on_rtde_feed_opened=socket_open(deviceIP,on_device_socket_port,rtdeFeedName)
  end 
  if not on_rtde_feed_opened:
  textmsg(on_rtde_feed_error_textmsg_title,on_rtde_feed_open_error_textmsg)
  popup(on_rtde_feed_open_error,title=on_rtde_feed_error_title,error=True,blocking=False)
  halt
  end 
  socket_send_int(regStart[0],rtdeFeedName)
  socket_send_int(regSum[0],rtdeFeedName)
  socket_send_int(regStart[1],rtdeFeedName)
  socket_send_int(regSum[1],rtdeFeedName)
  socket_send_int(regStart[2],rtdeFeedName)
  socket_send_int(regSum[2],rtdeFeedName)
  socket_send_int(regSpeedl,rtdeFeedName)
  socket_send_int(on_devices,rtdeFeedName)
  end 
  def on_dataRead():
  enter_critical
  on_RTDE_error=read_input_integer_register(ON_CONN_SHIFT_INT)
  if(ON_REG_USE_TOOL):
  on_RTDE_tool_error=read_input_integer_register(ON_TOOL_SHIFT_INT_ARR[0])
  end
  exit_critical
  end 
  def on_set_rtde_watchdog(updateHz=ON_INIT_WATCHDOG_HZ):
  local effect="stop"
  if(updateHz<1):
  effect="ignore"
  end 
  watchdog_conn_reg_str=str_cat("input_int_register_",ON_CONN_SHIFT_INT)
  rtde_set_watchdog(watchdog_conn_reg_str,updateHz,effect)
  if(ON_REG_USE_TOOL):
  watchdog_tool_reg_str=str_cat("input_int_register_",ON_TOOL_SHIFT_INT_ARR[0])
  rtde_set_watchdog(watchdog_tool_reg_str,updateHz,effect)
  end 
  if ON_DEBUG_LOG:
  local update_str=str_cat(" "+effect+" watchdog set to [Hz]: ",updateHz)
  textmsg(watchdog_conn_reg_str,update_str)
  if(ON_REG_USE_TOOL):
  local update_str=str_cat(" "+effect+" watchdog set to [Hz]: ",updateHz)
  textmsg(watchdog_tool_reg_str,update_str)
  end 
  end 
  end 
  def on_speedCB_get():
  return[on_speedCB[0],on_speedCB[1],on_speedCB[2],on_speedCB[3],on_speedCB[4],on_speedCB[5]]
  end 
  def on_speedGecko_get():
  local speedExtra=ON_ZERO6D
  if on_speedl_for_gecko:
  speedExtra=[on_speedGecko[0],on_speedGecko[1],on_speedGecko[2],on_speedGecko[3],on_speedGecko[4],on_speedGecko[5]]
  end 
  return speedExtra
  end 
  def on_speedGecko_set(speedGecko):
  on_speedGecko=[speedGecko[0],speedGecko[1],speedGecko[2],speedGecko[3],speedGecko[4],speedGecko[5]]
  end 
  thread on_dataProcess_thread():
  if ON_DEBUG_LOG:
  textmsg("Starting on_dataRead thread")
  end 
  local error=False
  on_dataProcess_running=True
  sync()
  while on_dataProcess_running:
  sync()
  on_dataRead()
  error=on_error((on_RTDE_error<0),on_rtde_feed_error,on_rtde_feed_error_title,error)
  error=on_error((ON_REG_USE_TOOL and(on_RTDE_tool_error<0)),on_rtde_feed_tool_error,on_rtde_feed_error_title,error)
  on_dataProcess_running=not error
  end 
  if error:
  halt
  end 
  if ON_DEBUG_LOG:
  textmsg("Stopping on_dataRead thread")
  end 
  end 
  thread on_set_watchdog_thread():
  sleep(2)
  on_set_rtde_watchdog(updateHz=ON_INIT_WATCHDOG_HZ)
  sleep(1/ON_INIT_WATCHDOG_HZ)
  on_dataProcess_running=False
  kill on_dataProcess_thrd
  end 
  
  #======    End of OnRobot Interface    ======#
  #======    OnRobot QC Setup Tool Connector    ======#
  
  def tc_setup_tool():
  if ON_DEBUG_LOG:
  textmsg("QC Setup Tool Connector start...")
  end 
  set_tool_voltage(24)
  
  set_tool_communication(True,1000000,2,1,1.5,3.5)
  
  if ON_DEBUG_LOG:
  textmsg("QC Setup Tool Connector end.")
  end 
  end 
  
  #======    End of OnRobot QC Setup Tool Connector    ======#
  #======    OnRobot RG Interface    ======#
  
  rg_dataRead_running=False
  def rg_dataRead_RTDE(tool_index):
  local reg_offset_bool=ON_TOOL_SHIFT_BOOL_ARR[tool_index]
  local reg_offset_int=ON_TOOL_SHIFT_INT_ARR[tool_index]
  local reg_offset_float=ON_TOOL_SHIFT_FLOAT_ARR[tool_index]
  enter_critical
  floatRegDummy=read_input_float_register(reg_offset_float+0)
  rg_Width_arr[tool_index]=floatRegDummy
  floatRegDummy=read_input_float_register(reg_offset_float+1)
  rg_Depth_arr[tool_index]=floatRegDummy
  floatRegDummy=read_input_float_register(reg_offset_float+2)
  rg_DepthRel_arr[tool_index]=floatRegDummy
  intRegDummy=read_input_integer_register(reg_offset_int+0)
  rg_device_id_arr[tool_index]=intRegDummy
  intRegDummy=read_input_integer_register(reg_offset_int+1)
  rg_product_code_arr[tool_index]=intRegDummy
  intRegDummy=read_input_integer_register(reg_offset_int+2)
  rg_Status_arr[tool_index]=intRegDummy
  boolRegDummy=read_input_boolean_register(reg_offset_bool+0)
  rg_Busy_arr[tool_index]=boolRegDummy
  boolRegDummy=read_input_boolean_register(reg_offset_bool+1)
  rg_Grip_detected_arr[tool_index]=boolRegDummy
  boolRegDummy=read_input_boolean_register(reg_offset_bool+2)
  rg_S1_pushed_arr[tool_index]=boolRegDummy
  boolRegDummy=read_input_boolean_register(reg_offset_bool+3)
  rg_S1_triggered_arr[tool_index]=boolRegDummy
  boolRegDummy=read_input_boolean_register(reg_offset_bool+4)
  rg_S2_pushed_arr[tool_index]=boolRegDummy
  boolRegDummy=read_input_boolean_register(reg_offset_bool+5)
  rg_S2_triggered_arr[tool_index]=boolRegDummy
  boolRegDummy=read_input_boolean_register(reg_offset_bool+6)
  rg_Safety_error_arr[tool_index]=boolRegDummy
  exit_critical
  end 
  thread rg_dataRead_thread():
  if ON_DEBUG_LOG:
  textmsg("Starting rg_dataRead thread")
  end 
  while rg_dataRead_running:
  sync()
  rg_Depth_prev_arr=rg_Depth_arr
  if(rg_index==ON_DI_DUAL):
  rg_dataRead_RTDE(ON_DI_PRIMARY)
  rg_dataRead_RTDE(ON_DI_SECONDARY)
  else:
  rg_dataRead_RTDE(rg_index)
  end 
  end 
  if ON_DEBUG_LOG:
  textmsg("Stopping rg_dataRead thread")
  end 
  end 
  
  #======    End of OnRobot RG Interface    ======#
  #======    OnRobot TCP Messages    ======#
  
  on_tcp_log_msg_default="OnRobot: Robot TCP Offset set to:"
  on_tcp_log_msg_primary="OnRobot: Robot TCP Offset set to Primary:"
  on_tcp_log_msg_secondary="OnRobot: Robot TCP Offset set to Secondary:"
  
  #======    End of OnRobot TCP Messages    ======#
  #======    OnRobot TCP    ======#
  
  on_tcp_offset_actual=ON_ZEROFRAME
  on_tcp_offset_primary=ON_ZEROFRAME
  on_tcp_static_primary=ON_ZEROFRAME
  on_tcp_dynamic_primary=ON_ZEROFRAME
  on_tcp_adapters=ON_ZEROFRAME
  on_tcp_qc_primary=ON_ZEROFRAME
  on_tcp_base_primary=ON_ZEROFRAME
  on_tcp_gripper_static_primary=ON_ZEROFRAME
  on_tcp_workpiece_primary=ON_ZEROFRAME
  on_tcp_gripper_dynamic_primary=ON_ZEROFRAME
  on_tcp_gripper_primary=ON_ZEROFRAME
  on_tcp_offset_secondary=ON_ZEROFRAME
  on_tcp_static_secondary=ON_ZEROFRAME
  on_tcp_dynamic_secondary=ON_ZEROFRAME
  on_tcp_qc_secondary=ON_ZEROFRAME
  on_tcp_base_secondary=ON_ZEROFRAME
  on_tcp_gripper_static_secondary=ON_ZEROFRAME
  on_tcp_workpiece_secondary=ON_ZEROFRAME
  on_tcp_gripper_dynamic_secondary=ON_ZEROFRAME
  on_tcp_gripper_secondary=ON_ZEROFRAME
  on_tcp_custom_preset_assigned=False
  on_tcp_custom_preset=ON_ZEROFRAME
  def on_tcp_init_adapters():
  if ON_DEBUG_LOG:
  textmsg("TCP Init Adapters start..")
  end 
  enter_critical
  on_tcp_adapters=ON_ZEROFRAME
  local adapterCount=length(on_tcp_adapters_array)
  local i=0
  while(i<adapterCount):
  on_tcp_adapters=pose_trans(on_tcp_adapters,on_tcp_adapters_array[i])
  i=i+1
  end
  exit_critical
  if ON_DEBUG_LOG:
  textmsg("TCP Init Adapters end.")
  end 
  end 
  def on_tcp_update_primary():
  if ON_DEBUG_LOG:
  textmsg("TCP Update Primary start..")
  end 
  on_tcp_gripper_dynamic_primary=rg_tcp_dynamic_get(0)
  enter_critical
  on_tcp_dynamic_primary=pose_trans(on_tcp_static_primary,on_tcp_gripper_dynamic_primary)
  on_tcp_offset_primary=pose_trans(on_tcp_dynamic_primary,on_tcp_workpiece_primary)
  exit_critical
  if ON_DEBUG_LOG:
  textmsg("TCP Update Primary end.")
  end 
  end 
  def on_tcp_init_primary():
  if ON_DEBUG_LOG:
  textmsg("TCP Init Primary start..")
  end 
  enter_critical
  on_tcp_base_primary=pose_trans(on_tcp_adapters,on_tcp_qc_primary)
  on_tcp_static_primary=pose_trans(on_tcp_base_primary,on_tcp_gripper_static_primary)
  exit_critical
  on_tcp_update_primary()
  if ON_DEBUG_LOG:
  textmsg("TCP Init Primary end.")
  end 
  end 
  def on_tcp_offset_set(TCP_offset):
  if ON_DEBUG_LOG:
  textmsg("TCP Offset set started!")
  end 
  on_tcp_offset_send(TCP_offset)
  if(on_follow_tcp):
  set_tcp(TCP_offset)
  end 
  on_robot_TCP_offset=TCP_offset
  if ON_DEBUG_LOG:
  textmsg("TCP Offset set to: ",on_robot_TCP_offset)
  end 
  end 
  def on_tcp_offset_forced_set_actual(isPrimary=True):
  if(not on_follow_tcp):
  
  local tcp_log_msg=on_tcp_log_msg_default
  
  textmsg(tcp_log_msg,on_tcp_offset_actual)
  set_tcp(on_tcp_offset_actual)
  end 
  end 
  def on_tcp_update(isPrimary=True):
  on_tcp_update_primary()
  
  on_tcp_set_actual_to(isPrimary)
  end 
  def on_tcp_set_actual_to(isPrimary=True):
  
  on_tcp_offset_actual=on_tcp_offset_primary
  if(on_follow_tcp):
  textmsg(on_tcp_log_msg_default,on_tcp_offset_actual)
  end 
  
  on_tcp_active_is_primary=isPrimary
  on_tcp_offset_set(on_tcp_offset_actual)
  end 
  def on_tcp_update_workpiece_primary(workpiece_offset):
  if ON_DEBUG_LOG:
  textmsg("TCP Update Tool Primary start..")
  end 
  on_tcp_workpiece_primary=workpiece_offset
  on_tcp_update_primary()
  if ON_DEBUG_LOG:
  textmsg("TCP Update Tool Primary end.")
  end 
  end 
  def on_tcp_workpiece_rpy_get(xyz=[0.0,0.0,0.0],rpy=[0.0,0.0,0.0]):
  local rotvec=rpy2rotvec(rpy)
  local workpiece_offset=p[xyz[0],xyz[1],xyz[2],rotvec[0],rotvec[1],rotvec[2]]
  return workpiece_offset
  end 
  
  def on_tcp_update_workpiece(workpiece_offset,tool_index=0):
  if ON_DEBUG_LOG:
  textmsg("TCP Update Tool start..")
  end 
  if(tool_index!=ON_DI_SECONDARY):
  on_tcp_update_workpiece_primary(workpiece_offset)
  local isPrimary=True
  
  end 
  on_tcp_set_actual_to(isPrimary)
  if ON_DEBUG_LOG:
  textmsg("TCP Update Tool end.")
  end 
  end 
  def on_get_tcp_for(tool_index):
  if tool_index==ON_DI_SECONDARY:
  local actual_tcp=on_tcp_static_secondary
  else:
  local actual_tcp=on_tcp_static_primary
  end 
  return actual_tcp
  end 
  def on_tcp_custom_unused():
  return ON_ZEROFRAME
  end 
  def on_tcp_update_custom():
  if ON_DEBUG_LOG:
  textmsg("TCP Update Custom start..")
  end 
  if(on_custom_tcp_enabled):
  on_tcp_custom_unused()
  on_tcp_custom_unused()
  on_tcp_custom_unused()
  on_tcp_custom_unused()
  
  end 
  if ON_DEBUG_LOG:
  textmsg("TCP Update Custom end.")
  end 
  end 
  
  #======    End of OnRobot TCP    ======#
  #======    OnRobot Payload Messages    ======#
  
  on_mass_log_msg="OnRobot: Robot Payload Mass set to:"
  on_cog_log_msg="OnRobot: Robot Payload Center of Gravity set to:"
  
  #======    End of OnRobot Payload Messages    ======#
  #======    OnRobot Payload    ======#
  
  on_cog_actual=ON_ZEROFRAME
  on_mass_actual=0.0
  on_cog_primary=ON_ZEROFRAME
  on_cog_L1L2_primary=ON_ZEROFRAME
  on_mass_primary=0.0
  on_mass_L1L2_primary=0.0
  on_cog_adapters=ON_ZEROFRAME
  on_cog_qc_primary=ON_ZEROFRAME
  on_cog_base_primary=ON_ZEROFRAME
  on_cog_gripper_primary=ON_ZEROFRAME
  on_cog_workpiece_primary=ON_ZEROFRAME
  on_mass_adapters=0.0
  on_mass_qc_primary=0.0
  on_mass_base_primary=0.0
  on_mass_gripper_primary=0.0
  on_mass_workpiece_primary=0.0
  on_cog_secondary=ON_ZEROFRAME
  on_mass_secondary=0.0
  on_cog_gripper_secondary=ON_ZEROFRAME
  on_mass_gripper_secondary=0.0
  on_cog_workpiece_secondary=ON_ZEROFRAME
  on_mass_workpiece_secondary=0.0
  ON_LB2KG001=0.45359237/100
  def on_payload_init_adapters():
  if ON_DEBUG_LOG:
  textmsg("TCP Init Adapters start..")
  end 
  enter_critical
  on_cog_adapters=ON_ZEROFRAME
  on_mass_adapters=0.0
  local massLAL0Ratio=0
  local adapterAbsCOG=ON_ZEROFRAME
  local tcp_adapter_offset=ON_ZEROFRAME
  local adapterCount=length(on_tcp_adapters_array)
  local i=0
  while(i<adapterCount):
  on_mass_adapters=on_mass_adapters+on_mass_adapters_array[i]
  if(on_mass_adapters<=0):
  massLAL0Ratio=0
  else:
  massLAL0Ratio=on_mass_adapters_array[i]/on_mass_adapters
  end
  adapterAbsCOG=pose_trans(tcp_adapter_offset,on_tcp_adapters_array[i])
  adapterAbsCOG=p[adapterAbsCOG[0],adapterAbsCOG[1],adapterAbsCOG[2],0.0,0.0,0.0]
  on_cog_adapters=interpolate_pose(on_cog_adapters,adapterAbsCOG,massLAL0Ratio)
  on_cog_adapters=p[on_cog_adapters[0],on_cog_adapters[1],on_cog_adapters[2],0.0,0.0,0.0]
  tcp_adapter_offset=pose_trans(tcp_adapter_offset,on_tcp_adapters_array[i])
  i=i+1
  sync()
  end
  exit_critical
  if ON_DEBUG_LOG:
  textmsg("TCP Init Adapters end.")
  end 
  end 
  def on_payload_update_primary():
  if ON_DEBUG_LOG:
  textmsg("Payload Update Primary start..")
  end 
  enter_critical
  on_mass_primary=on_mass_L1L2_primary+on_mass_workpiece_primary
  local massL1L2L3Ratio=on_mass_workpiece_primary/on_mass_primary
  local workpieceAbsCOG=pose_trans(on_tcp_dynamic_primary,on_cog_workpiece_primary)
  workpieceAbsCOG=p[workpieceAbsCOG[0],workpieceAbsCOG[1],workpieceAbsCOG[2],0.0,0.0,0.0]
  on_cog_primary=interpolate_pose(on_cog_L1L2_primary,workpieceAbsCOG,massL1L2L3Ratio)
  on_cog_primary=p[on_cog_primary[0],on_cog_primary[1],on_cog_primary[2],0.0,0.0,0.0]
  exit_critical
  if ON_DEBUG_LOG:
  textmsg("Payload Update Primary end.")
  end 
  end 
  def on_payload_init_primary():
  if ON_DEBUG_LOG:
  textmsg("Payload Init Primary start..")
  end 
  enter_critical
  on_mass_base_primary=on_mass_adapters+on_mass_qc_primary
  local massL0L1Ratio=on_mass_qc_primary/on_mass_base_primary
  on_mass_L1L2_primary=on_mass_base_primary+on_mass_gripper_primary
  local massL1L2Ratio=on_mass_gripper_primary/on_mass_L1L2_primary
  local qcAbsCOG=pose_trans(on_tcp_adapters,on_cog_qc_primary)
  qcAbsCOG=p[qcAbsCOG[0],qcAbsCOG[1],qcAbsCOG[2],0.0,0.0,0.0]
  on_cog_base_primary=interpolate_pose(on_cog_adapters,qcAbsCOG,massL0L1Ratio)
  on_cog_base_primary=p[on_cog_base_primary[0],on_cog_base_primary[1],on_cog_base_primary[2],0.0,0.0,0.0]
  local gripperAbsCOG=pose_trans(on_tcp_base_primary,on_cog_gripper_primary)
  gripperAbsCOG=p[gripperAbsCOG[0],gripperAbsCOG[1],gripperAbsCOG[2],0.0,0.0,0.0]
  on_cog_L1L2_primary=interpolate_pose(on_cog_base_primary,gripperAbsCOG,massL1L2Ratio)
  on_cog_L1L2_primary=p[on_cog_L1L2_primary[0],on_cog_L1L2_primary[1],on_cog_L1L2_primary[2],0.0,0.0,0.0]
  exit_critical
  on_payload_update_primary()
  if ON_DEBUG_LOG:
  textmsg("Payload Init Primary end.")
  end 
  end 
  def on_payload_update():
  on_payload_update_primary()
  
  on_mass_actual =on_mass_primary
  on_cog_actual=on_cog_primary
  
  on_payload_set_actual()
  end 
  def on_payload_set_actual():
  
  on_mass_actual =on_mass_primary
  on_cog_actual=on_cog_primary
  
  local CoG=[on_cog_actual[0],on_cog_actual[1],on_cog_actual[2]]
  on_payload_set(on_mass_actual,CoG)
  end 
  def on_payload_update_workpiece_primary(workpiece_mass=0,workpiece_cog=[0.0,0.0,0.0]):
  if ON_DEBUG_LOG:
  textmsg("Payload Update Workpiece Primary start..")
  end 
  on_cog_workpiece_primary=p[workpiece_cog[0],workpiece_cog[1],workpiece_cog[2],0.0,0.0,0.0]
  on_mass_workpiece_primary=workpiece_mass
  on_payload_update_primary()
  if ON_DEBUG_LOG:
  textmsg("Payload Update Workpiece Primary end.")
  end 
  end 
  def on_payload_set(mass,CoG):
  if ON_DEBUG_LOG:
  textmsg("Payload set started!")
  end 
  local center_of_gravity=[CoG[0],CoG[1],CoG[2]]
  on_payload_send(center_of_gravity,mass)
  if(on_isMetric):
  local roundedMass=floor(mass*100+0.5)/100
  else:
  local roundedMass=floor(mass/ON_LB2KG001+0.5)*ON_LB2KG001
  end 
  set_payload(roundedMass,center_of_gravity)
  on_mass_actual=roundedMass
  on_cog_actual=p[CoG[0],CoG[1],CoG[2],0.0,0.0,0.0]
  if ON_DEBUG_LOG:
  textmsg("Payload mass, CoG set to: ",str_cat(str_cat(mass,", "),CoG))
  end 
  end 
  
  def on_payload_update_secondary():
  end 
  def on_payload_init_secondary():
  end 
  
  def on_payload_update_workpiece(workpiece_mass,workpiece_cog=[0.0,0.0,0.0],tool_index=0):
  if ON_DEBUG_LOG:
  textmsg("Payload Update Workpiece start..")
  end 
  if(tool_index!=ON_DI_SECONDARY):
  on_cog_workpiece_primary=p[workpiece_cog[0],workpiece_cog[1],workpiece_cog[2],0.0,0.0,0.0]
  on_mass_workpiece_primary=workpiece_mass
  else:
  on_cog_workpiece_secondary=p[workpiece_cog[0],workpiece_cog[1],workpiece_cog[2],0.0,0.0,0.0]
  on_mass_workpiece_secondary=workpiece_mass
  end 
  on_payload_update()
  if ON_DEBUG_LOG:
  textmsg("Payload Update Workpiece end.")
  end 
  end 
  def on_payload_get_cog2tcp_workpiece(tool_index=0):
  if(tool_index!=ON_DI_SECONDARY):
  local cog_tool=[on_tcp_workpiece_primary[0],on_tcp_workpiece_primary[1],on_tcp_workpiece_primary[2]]
  else:
  local cog_tool=[on_tcp_workpiece_secondary[0],on_tcp_workpiece_secondary[1],on_tcp_workpiece_secondary[2]]
  end 
  return cog_tool
  end 
  
  #======    End of OnRobot Payload    ======#
  #======    OnRobot QC TCP    ======#
  
  onrobotmini=0
  
  def on_tcp_offset_send(TCP_offset):
  on_robot_TCP_offset=TCP_offset
  end 
  def on_payload_send(CoG,mass):
  on_robot_payload_cog=CoG
  on_robot_payload_mass=mass
  end 
  
  
  #======    End of OnRobot QC TCP    ======#
  #======    OnRobot RG TCP    ======#
  
  def rg_mount_tcp(gripper_angle,isRG2=True):
  if ON_DEBUG_LOG:
  textmsg("RG Mount TCP command starting..")
  end 
  if(isRG2):
  local gripperBracket=rg2_mount_bracket_offset
  local gripperBody=rg2_mount_body_offset
  else:
  local gripperBracket=rg6_mount_bracket_offset
  local gripperBody=rg6_mount_body_offset
  end 
  local gripperRotVec=rpy2rotvec([gripper_angle,0.0,0.0])
  local gripperMountOffset=p[0.0,0.0,gripperBracket,gripperRotVec[0],gripperRotVec[1],gripperRotVec[2]]
  local gripperOffsetStatic=pose_trans(gripperMountOffset,gripperBody)
  if ON_DEBUG_LOG:
  textmsg("RG Mount TCP command ended.")
  end 
  return gripperOffsetStatic
  end 
  def rg_mount_cog(gripper_angle,isRG2=True):
  if ON_DEBUG_LOG:
  textmsg("RG Mount CoG command starting..")
  end 
  if(isRG2):
  local gripperBracket=rg2_mount_bracket_offset
  local gripperCoG=rg2_mount_cog_offset
  else:
  local gripperBracket=rg6_mount_bracket_offset
  local gripperCoG=rg6_mount_cog_offset
  end 
  local gripperRotVec=rpy2rotvec([gripper_angle,0.0,0.0])
  local gripperMountOffset=p[0.0,0.0,gripperBracket,gripperRotVec[0],gripperRotVec[1],gripperRotVec[2]]
  local gripperCOGOffset=pose_trans(gripperMountOffset,gripperCoG)
  gripperCOGOffset=p[gripperCOGOffset[0],gripperCOGOffset[1],gripperCOGOffset[2],0.0,0.0,0.0]
  if ON_DEBUG_LOG:
  textmsg("RG Mount CoG command ended.")
  end 
  return gripperCOGOffset
  end 
  def rg_fcp_depth_calc_frame(tool_index=0):
  local gripperDepth=rg_Depth_arr[tool_index]
  local z_offset=-gripperDepth/1000.0
  return p[0.0,0.0,z_offset,0.0,0.0,0.0]
  end 
  def rg_tcp_dynamic_get(tool_index=0):
  local offset=rg_fcp_depth_calc_frame(tool_index)
  return offset
  end 
  def rg_payload_set(mass,tool_index=0,use_guard=False):
  if ON_DEBUG_LOG:
  textmsg("RG Payload mass command starting..")
  end 
  if(tool_index==ON_DI_SECONDARY):
  local cogTool=[on_cog_workpiece_secondary[0],on_cog_workpiece_secondary[1],on_cog_workpiece_secondary[2]]
  else:
  local cogTool=[on_cog_workpiece_primary[0],on_cog_workpiece_primary[1],on_cog_workpiece_primary[2]]
  end 
  if(use_guard and rg_Grip_detected_arr[tool_index]):
  local mass2set=mass
  rg_Grip_guard_arr[tool_index]=not on_ioqc
  else:
  local mass2set=0.0
  rg_Grip_guard_arr[tool_index]=False
  end 
  on_payload_update_workpiece(mass2set,cogTool,tool_index)
  if ON_DEBUG_LOG:
  textmsg("RG Payload mass command ended.")
  end 
  end 
  
  #======    End of OnRobot RG TCP    ======#
  #======    OnRobot  Speedl    ======#
  
  on_speedl_thread_handler=0
  on_speedl_is_enabled=False
  on_speedl_is_running=False
  on_speedl_acc=0.0
  ft_speedl_hg_caranteen_reached=False
  ft_speedl_hg_caranteen_safemode=False
  on_speedl_for_ftcontrol=False
  on_speedl_for_move=False
  on_speedl_for_handguide=False
  on_speedl_for_insertpart=False
  on_speedl_for_depthcompensation=False
  on_speedl_for_center=False
  on_speedl_for_gecko=False
  on_speedl_acc=10000.0 
  on_speedl_acc_to_zero=3.0 
  ON_SPEEDL_FTCONTROL=1
  ON_SPEEDL_HANDGUIDE=2
  ON_SPEEDL_TRAJECTORY=3
  ON_SPEEDL_MOVE=ON_SPEEDL_TRAJECTORY
  ON_SPEEDL_INSERTPART=4
  ON_SPEEDL_DEPTHCOMP=5
  ON_SPEEDL_CENTER=6
  ON_SPEEDL_GECKO=7
  def on_wait_ms(time_ms):
  local sync_time=ceil(norm(time_ms/2.0))
  while(sync_time>0):
  sync_time=sync_time-1
  sync()
  end
  end 
  def on_error(status_flag,message,title,stop_var=False):
  if(status_flag):
  popup(message,title=title,error=True,blocking=False)
  stop_var=True
  end 
  return stop_var
  end 
  def on_warning(status_flag,message,title,isPopupNeeded=False,is_shown=False):
  if(status_flag):
  if(not is_shown):
  if(isPopupNeeded):
  popup(message,title=title,warning=True,blocking=False)
  else:
  textmsg(title+": ",message)
  end 
  is_shown=True
  end 
  else:
  is_shown=False
  end 
  return is_shown
  end 
  def on_speedl_add_extra(speedBase,speedExtra):
  speedSum=ON_ZERO6D
  speedBaseP=p[speedBase[0],speedBase[1],speedBase[2],speedBase[3],speedBase[4],speedBase[5]]
  speedExtraP=p[speedExtra[0],speedExtra[1],speedExtra[2],speedExtra[3],speedExtra[4],speedExtra[5]]
  speedSumP=pose_add(speedExtraP,speedBaseP)
  speedSum=[speedSumP[0],speedSumP[1],speedSumP[2],speedSumP[3],speedSumP[4],speedSumP[5]]
  return speedSum
  end 
  
  thread on_speedl_thread():
  if ON_DEBUG_LOG:
  textmsg("Speedl thread started..")
  end 
  enter_critical
  on_speedl_is_running=True
  on_speedl_is_enabled=True
  exit_critical
  on_speedL=ON_ZERO6D
  on_speedL_last=ON_ZERO6D
  
  while on_speedl_is_running:
  on_speedBase=on_speedCB_get()
  on_speedL=on_speedBase
  
  
  if on_speedl_is_enabled:
  if(on_speedL==ON_ZERO6D):
  on_speedL=[on_speedL_last[0]/100,on_speedL_last[1]/100,on_speedL_last[2]/100,on_speedL_last[3]/100,on_speedL_last[4]/100,on_speedL_last[5]/100]
  end
  speedl(on_speedL,a=on_speedl_acc,t=0.001)
  else:
  speedl(ON_ZERO6D,a=on_speedl_acc_to_zero,t=0.001)
  end 
  on_speedL_last=on_speedL
  end 
  
  on_speedL=ON_ZERO6D
  if ON_DEBUG_LOG:
  textmsg("Speedl thread ended.")
  end 
  end 
  def on_speedl_integer_get():
  local speedl_integer=binary_list_to_integer([on_speedl_for_ftcontrol,on_speedl_for_handguide,on_speedl_for_move,on_speedl_for_insertpart,on_speedl_for_depthcompensation,on_speedl_for_center,on_speedl_for_gecko])
  return speedl_integer
  end 
  def on_speedl_start_for(speedl_id):
  if(speedl_id==ON_SPEEDL_FTCONTROL):
  on_speedl_for_ftcontrol=True
  elif(speedl_id==ON_SPEEDL_HANDGUIDE):
  on_speedl_for_handguide=True
  elif(speedl_id==ON_SPEEDL_TRAJECTORY):
  on_speedl_for_move=True
  elif(speedl_id==ON_SPEEDL_INSERTPART):
  on_speedl_for_insertpart=True
  elif(speedl_id==ON_SPEEDL_DEPTHCOMP):
  on_speedl_for_depthcompensation=True
  elif(speedl_id==ON_SPEEDL_CENTER):
  on_speedl_for_center=True
  elif(speedl_id==ON_SPEEDL_GECKO):
  on_speedl_for_gecko=True
  else:
  textmsg("Unknown Start speedl ID received: ",speedl_id)
  end 
  local speedl_integer=on_speedl_integer_get()
  if not(speedl_integer==0):
  if not on_speedl_is_running:
  on_speedl_thread_handler=run on_speedl_thread()
  end 
  end 
  return on_speedl_is_running
  end 
  def on_speedl_stop_for(speedl_id,brake=10,brakeRot=10):
  if(speedl_id==ON_SPEEDL_FTCONTROL):
  on_speedl_for_ftcontrol=False
  elif(speedl_id==ON_SPEEDL_HANDGUIDE):
  on_speedl_for_handguide=False
  elif(speedl_id==ON_SPEEDL_TRAJECTORY):
  on_speedl_for_move=False
  elif(speedl_id==ON_SPEEDL_INSERTPART):
  on_speedl_for_insertpart=False
  elif(speedl_id==ON_SPEEDL_DEPTHCOMP):
  on_speedl_for_depthcompensation=False
  elif(speedl_id==ON_SPEEDL_CENTER):
  on_speedl_for_center=False
  elif(speedl_id==ON_SPEEDL_GECKO):
  on_speedl_for_gecko=False
  on_speedGecko_set(ON_ZERO6D)
  else:
  textmsg("Unknown Stop speedl ID received: ",speedl_id)
  end 
  local speedl_integer=on_speedl_integer_get()
  if(speedl_integer==0):
  if(on_speedl_is_running):
  kill on_speedl_thread_handler
  enter_critical
  on_speedl_is_running=False
  exit_critical
  
  end 
  if ON_DEBUG_LOG:
  textmsg("Speedl thread stopped")
  end 
  stopl(brake,brakeRot)
  end 
  return on_speedl_is_running
  end 
  def on_speedl_pause():
  on_speedl_is_enabled=False
  return on_speedl_integer_get()
  end 
  def on_speedl_resume():
  on_speedl_is_enabled=True
  return on_speedl_integer_get()
  end 
  
  #======    End of OnRobot  Speedl    ======#
  #======    OnRobot RG Engine Messages    ======#
  
  rg_error_title="OnRobot - RG error"
  rg_device_id_waiting="Waiting for valid OnRobot RG ID..."
  rg_device_id_timeout="Waiting for valid OnRobot RG ID timed out.<br>Program halted."
  rg_data_error_title="OnRobot - RG Data error"
  rg_data_error_type="No or unknown Gripper connected."
  rg_data_warning_title="OnRobot - RG Data warning:"
  rg_status_error_title="OnRobot - RG Status error"
  rg_status_error_missing="Communication problem with an RG gripper. Please check the device.<br>Program halted."
  rg_else_error="Unknown error code:"
  rg_status_msg_single="RG status:"
  rg_data_error_bit0_single="Wrong size of float data received."
  rg_data_error_bit1_single="Wrong size of integer data received."
  rg_data_error_bit2_single="Wrong size of boolean data received."
  rg_status_error_mismatch_single="The RG's Status is not matching with its Safety Switches' state."
  rg_status_error_s1_triggered_single="The S1 Safety Switch has been triggered.<br>Please restart the RG gripper."
  rg_status_error_s1_pushed_single="The S1 Safety Switch has been pushed."
  rg_status_error_s2_triggered_single="The S2 Safety Switch has been triggered.<br>Please restart the RG gripper."
  rg_status_error_s2_pushed_single="The S2 Safety Switch has been pushed."
  rg_status_error_safety_system_single="Safety System error occured.<br>Please restart the RG gripper."
  rg_status_msg_primary="RG Primary status:"
  rg_data_error_bit0_primary="Wrong size of float data received for Primary RG."
  rg_data_error_bit1_primary="Wrong size of integer data received for Primary RG."
  rg_data_error_bit2_primary="Wrong size of boolean data received for Primary RG."
  rg_status_error_mismatch_primary="The Primary RG's Status is not matching with its Safety Switches' state."
  rg_status_error_s1_pushed_primary="The S1 Safety Switch has been pushed on Primary RG."
  rg_status_error_s1_triggered_primary="The S1 Safety Switch has been triggered on Primary RG.<br>Please restart the Primary RG gripper."
  rg_status_error_s2_pushed_primary="The S2 Safety Switch has been pushed on Primary RG."
  rg_status_error_s2_triggered_primary="The S2 Safety Switch has been triggered on Primary RG.<br>Please restart the Primary RG gripper."
  rg_status_error_safety_system_primary="Safety System error occured.<br>Please restart the Primary RG gripper."
  rg_status_msg_secondary="RG Secondary status:"
  rg_data_error_bit0_secondary="Wrong size of float data received for Secondary RG."
  rg_data_error_bit1_secondary="Wrong size of integer data received for Secondary RG."
  rg_data_error_bit2_secondary="Wrong size of boolean data received for Secondary RG."
  rg_status_error_mismatch_secondary="The Secondary RG's Status is not matching with its Safety Switches' state."
  rg_status_error_s1_pushed_secondary="The S1 Safety Switch has been pushed on Secondary RG."
  rg_status_error_s1_triggered_secondary="The S1 Safety Switch has been triggered on Secondary RG.<br>Please restart the Secondary RG gripper."
  rg_status_error_s2_pushed_secondary="The S2 Safety Switch has been pushed on Secondary RG."
  rg_status_error_s2_triggered_secondary="The S2 Safety Switch has been triggered on Secondary RG.<br>Please restart the Secondary RG gripper."
  rg_status_error_safety_system_secondary="The Safety System error.<br>Please restart the Secondary RG gripper."
  
  #======    End of OnRobot RG Engine Messages    ======#
  #======    OnRobot RG Engine    ======#
  
  rg_dataProcess_running=False
  rg_start_flange=ON_ZEROPOSE
  rg_start_pose=ON_ZEROPOSE
  RG_STATUS_IGNORE_SHIFT=4
  def rg_status_int_get(tool_index):
  local status_int=binary_list_to_integer([rg_S1_pushed_arr[tool_index],rg_S1_triggered_arr[tool_index],rg_S2_pushed_arr[tool_index],rg_S2_triggered_arr[tool_index],rg_Safety_error_arr[tool_index]])
  return status_int
  end 
  def rg_dataProcess_status_errors(rg_stop=False):
  enter_critical
  local tool_index=rg_index
  if(tool_index==ON_DI_DUAL):
  tool_index=ON_DI_SECONDARY
  rg_stop=on_error((rg_device_id_arr[tool_index]!=tool_index),rg_status_error_missing,rg_status_error_title,rg_stop)
  rg_stop=on_error((rg_product_code_arr[tool_index]==ON_DEVICE_ID_MISSING),rg_status_error_missing,rg_status_error_title,rg_stop)
  tool_index=ON_DI_PRIMARY
  end 
  rg_stop=on_error((rg_device_id_arr[tool_index]!=tool_index),rg_status_error_missing,rg_status_error_title,rg_stop)
  rg_stop=on_error((rg_product_code_arr[tool_index]==ON_DEVICE_ID_MISSING),rg_status_error_missing,rg_status_error_title,rg_stop)
  
  local switch_error_single=rg_status_int_get(tool_index)
  local status_error_single=floor(rg_Status_arr[tool_index]/RG_STATUS_IGNORE_SHIFT)
  if(status_error_single!=switch_error_single):
  on_warning(((status_error_single-switch_error_single)>0),str_cat(rg_else_error,rg_Status_arr[tool_index]),rg_status_error_title)
  end 
  if(switch_error_single!=0):
  rg_stop=on_error(rg_S1_pushed_arr[tool_index],rg_status_error_s1_pushed_single,rg_status_error_title,rg_stop)
  rg_stop=on_error(rg_S1_triggered_arr[tool_index],rg_status_error_s1_triggered_single,rg_status_error_title,rg_stop)
  rg_stop=on_error(rg_S2_pushed_arr[tool_index],rg_status_error_s2_pushed_single,rg_status_error_title,rg_stop)
  rg_stop=on_error(rg_S2_triggered_arr[tool_index],rg_status_error_s2_triggered_single,rg_status_error_title,rg_stop)
  rg_stop=on_error(rg_Safety_error_arr[tool_index],rg_status_error_safety_system_single,rg_status_error_title,rg_stop)
  end 
  
  exit_critical
  return rg_stop
  end 
  def rg_dataProcess():
  enter_critical
  local tool_index=rg_index
  if tool_index==ON_DI_DUAL:
  rg_Width_primary=rg_Width_arr[ON_DI_PRIMARY]
  rg_Depth_primary=rg_Depth_arr[ON_DI_PRIMARY]
  rg_DepthRel_primary=rg_DepthRel_arr[ON_DI_PRIMARY]
  rg_Busy_primary=rg_Busy_arr[ON_DI_PRIMARY]
  rg_Grip_detected_primary=rg_Grip_detected_arr[ON_DI_PRIMARY]
  rg_Width_secondary=rg_Width_arr[ON_DI_SECONDARY]
  rg_Depth_secondary=rg_Depth_arr[ON_DI_SECONDARY]
  rg_DepthRel_secondary=rg_DepthRel_arr[ON_DI_SECONDARY]
  rg_Busy_secondary=rg_Busy_arr[ON_DI_SECONDARY]
  rg_Grip_detected_secondary=rg_Grip_detected_arr[ON_DI_SECONDARY]
  else:
  rg_Width=rg_Width_arr[tool_index]
  rg_Depth=rg_Depth_arr[tool_index]
  rg_DepthRel=rg_DepthRel_arr[tool_index]
  rg_Busy=rg_Busy_arr[tool_index]
  rg_Grip_detected=rg_Grip_detected_arr[tool_index]
  end 
  exit_critical
  end 
  thread rg_dataProcess_thread():
  if ON_DEBUG_LOG:
  textmsg("Starting rg_dataProcess thread")
  end 
  while rg_dataProcess_running:
  sync()
  local rg_stop=False
  rg_stop=rg_dataProcess_status_errors(rg_stop)
  if rg_stop:
  halt
  end 
  rg_dataProcess()
  end 
  if ON_DEBUG_LOG:
  textmsg("Stopping rg_dataProcess thread")
  end 
  end 
  def rg_index_get():
  return rg_index
  end 
  
  #======    End of OnRobot RG Engine    ======#
  rg_mounting_angle_arr[0] = 0.0
  rg_fingertip_arr[0] = 4.599999904632568
  rg_Depth_arr[0] = 23.0
  textmsg(on_devices_primary_log, ": Quick Changer + RG2 + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
  on_follow_tcp = True
  on_tcp_active_is_primary = True
  on_tcp_adapters_array = []
  on_cog_adapters_array = []
  on_mass_adapters_array = []
  on_tcp_qc_primary = p[0.0, 0.0, 0.0136, 0.0, 0.0, 0.0]
  on_cog_qc_primary = p[0.0, 0.0, 0.004, 0.0, 0.0, 0.0]
  on_mass_qc_primary = 0.06
  on_tcp_gripper_static_primary = rg_mount_tcp(rg_mounting_angle_arr[0])
  on_cog_gripper_primary = rg_mount_cog(rg_mounting_angle_arr[0])
  on_mass_gripper_primary = 0.78
  on_tcp_workpiece_primary = p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  on_cog_workpiece_primary = p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  on_mass_workpiece_primary = 0.0
  
  #======    End of OnRobot RG Depth    ======#
  #======    OnRobot RG Width Messages    ======#
  
  rg_grip_title="OnRobot - RG Grip"
  rg_grip_return_error_n1="RG Grip command returned with error.<br>Please be sure that the required width is between the limits.<br>Program halted."
  
  #======    End of OnRobot RG Width Messages    ======#
  #======    OnRobot RG Width    ======#
  
  RG_MIN_WIDTH=0
  RG_MAX_WIDTH_RG2=110
  RG_MAX_WIDTH_RG6=160
  RG_MIN_FORCE=0
  RG_MAX_FORCE_RG2=40
  RG_MAX_FORCE_RG6=120
  rg__grip_param_warning_width="The parameter 'width' is out of the limits. Limited value sent: "
  rg__grip_param_warning_force="The parameter 'force' is out of the limits. Limited value sent: "
  def rg_depth_compensate(tool_index,start_depth_mm):
  local timeout=0
  while not rg_Busy_arr[tool_index]:
  sleep(0.008)
  timeout=timeout+1
  if timeout>20:
  break
  end
  end
  local start_pose=get_forward_kin()
  if(tool_index==ON_DI_SECONDARY):
  local tcp_static=on_tcp_static_secondary
  else:
  local tcp_static=on_tcp_static_primary
  end 
  local t_w_rg=pose_trans(get_actual_tool_flange_pose(),tcp_static)
  local t_rg_w=pose_inv(t_w_rg)
  local compensation_depth_mm=0
  local after_ready_continue_count=10
  while True:
  local busy=rg_Busy_arr[tool_index]
  if not busy:
  if after_ready_continue_count>0:
  after_ready_continue_count=after_ready_continue_count-1
  else:
  break
  end
  end
  local measure_depth_mm=rg_Depth_arr[tool_index]
  compensation_depth_mm=measure_depth_mm-start_depth_mm
  local target_pose=pose_add(start_pose,pose_trans(pose_trans(t_w_rg,p[0,0,compensation_depth_mm/1000.0,0,0,0]),t_rg_w))
  servoj(get_inverse_kin(target_pose),t=0.008,lookahead_time=0.033,gain=1500)
  end
  stopj(20)
  end
  def rg_grip(width,force,tool_index=0,blocking=True,depth_comp=False,popupmsg=True):
  if ON_DEBUG_LOG:
  textmsg("RG Grip start..")
  end 
  local retVal=0
  local limitOffset=(2.0*rg_fingertip_arr[tool_index])
  local width2send=width
  if limitOffset>0:
  local minWidth=RG_MIN_WIDTH
  else:
  local minWidth=RG_MIN_WIDTH-limitOffset
  end 
  if not(width2send>=minWidth):
  width2send=minWidth
  textmsg(rg__grip_param_warning_width,width2send)
  elif not(force>=RG_MIN_FORCE):
  force=RG_MIN_FORCE
  textmsg(rg__grip_param_warning_force,force)
  end 
  if(rg_product_code_arr[tool_index]==RG_DEVICE_ID_RG6):
  if not(width2send<=RG_MAX_WIDTH_RG6-limitOffset):
  width2send=RG_MAX_WIDTH_RG6-limitOffset
  textmsg(rg__grip_param_warning_width,width2send)
  elif not(force<=RG_MAX_FORCE_RG6):
  force=RG_MAX_FORCE_RG6
  textmsg(rg__grip_param_warning_force,force)
  end 
  else:
  if not(width2send<=RG_MAX_WIDTH_RG2-limitOffset):
  width2send=RG_MAX_WIDTH_RG2-limitOffset
  textmsg(rg__grip_param_warning_width,width2send)
  elif not((force<=RG_MAX_FORCE_RG2)):
  force=RG_MAX_FORCE_RG2
  textmsg(rg__grip_param_warning_force,force)
  end 
  end 
  rg_Grip_guard_arr[tool_index]=False
  sync()
  if(tool_index==ON_DI_SECONDARY):
  local isPrimary=False
  else:
  local isPrimary=True
  end 
  if(on_follow_tcp):
  on_tcp_set_actual_to(isPrimary)
  end 
  if depth_comp:
  local start_depth_mm=rg_Depth_arr[tool_index]
  end 
  sync()
  retVal=on_tool_xmlrpc.rg_grip(tool_index,width2send+0.0,force+0.0)
  if(retVal!=0):
  popup(rg_grip_return_error_n1,rg_grip_title,error=True,blocking=False)
  halt
  end 
  if depth_comp:
  rg_depth_compensate(tool_index,start_depth_mm)
  end 
  if blocking:
  if not depth_comp:
  local timeout=0
  while not rg_Busy_arr[tool_index]:
  sleep(0.008)
  timeout=timeout+1
  if timeout>20:
  break
  end 
  end 
  end 
  while(rg_Busy_arr[tool_index]==True):
  sync()
  end 
  end 
  if(on_follow_tcp):
  on_tcp_update(isPrimary)
  end 
  if ON_DEBUG_LOG:
  textmsg("RG Grip ended.")
  end 
  return retVal
  end 
  
  #======    End of OnRobot RG Width    ======#
  #======    OnRobot VG Monitor Messages    ======#
  
  rg_monitor_error_title="OnRobot - RG Grip lost"
  rg_monitor_grip_lost_error="Grip lost detected.<br>Program halted."
  
  #======    End of OnRobot VG Monitor Messages    ======#
  #======    OnRobot RG Monitor    ======#
  
  thread rg_monitor_thread():
  while True:
  sync()
  local rg_error=False
  if(rg_index==ON_DI_DUAL):
  local tool_index=ON_DI_SECONDARY
  rg_error=rg_monitor_check(tool_index)
  local tool_index=ON_DI_PRIMARY
  else:
  local tool_index=rg_index
  end 
  rg_error=rg_error or rg_monitor_check(tool_index)
  if rg_error:
  halt
  end 
  end 
  end 
  def rg_monitor_check(tool_index=0):
  local error=False
  if rg_Grip_guard_arr[tool_index]and not rg_Grip_detected_arr[tool_index]:
  popup(rg_monitor_grip_lost_error,title=rg_monitor_error_title,error=True,blocking=False)
  rg_payload_set(0,tool_index=tool_index)
  error=True
  end 
  return error
  end 
  
  #======    End of OnRobot RG Monitor    ======#
  #======    OnRobot Run    ======#
  
  on_portopen_javaSocket()
  sync()
  textmsg(on_xmlrpc_start_ip,on_conn_ip)
  if(ON_REG_USE_TOOL):
  on_regStart_conn=[ON_CONN_SHIFT_BOOL,ON_CONN_SHIFT_INT,ON_CONN_SHIFT_FLOAT]
  on_regSum_conn=[ON_CONN_REG_SUM_BOOL,ON_CONN_REG_SUM_INT,ON_CONN_REG_SUM_FLOAT]
  on_rtde_feed_open(on_conn_ip,on_conn_rtde_feed_name,on_regStart_conn,on_regSum_conn,ON_REGISTERS_SPEEDL_FLOAT)
  sync()
  textmsg(on_xmlrpc_start_ip,on_tool_ip)
  on_regStart_tool=[ON_TOOL_SHIFT_BOOL,ON_TOOL_SHIFT_INT,ON_TOOL_SHIFT_FLOAT]
  on_regSum_tool=[ON_TOOL_REG_SUM_BOOL,ON_TOOL_REG_SUM_INT,ON_TOOL_REG_SUM_FLOAT]
  on_rtde_feed_open(on_tool_ip,on_tool_rtde_feed_name,on_regStart_tool,on_regSum_tool,0)
  sync()
  else:
  on_regStart_conn=[ON_CONN_SHIFT_BOOL,ON_CONN_SHIFT_INT,ON_CONN_SHIFT_FLOAT]
  on_regSum_conn=[ON_CONN_REG_SUM_BOOL,ON_CONN_REG_SUM_INT,ON_CONN_REG_SUM_FLOAT]
  on_rtde_feed_open(on_conn_ip,on_conn_rtde_feed_name,on_regStart_conn,on_regSum_conn,ON_REGISTERS_SPEEDL_FLOAT)
  sync()
  end
  on_set_rtde_watchdog(updateHz=0.2)
  sync()
  on_dataProcess_thrd=run on_dataProcess_thread()
  sync()
  on_tcp_init_adapters()
  on_payload_init_adapters()
  on_tcp_init_primary()
  on_payload_init_primary()
  
  on_tcp_update_custom()
  on_tcp_set_actual_to(on_tcp_active_is_primary)
  if(on_follow_tcp):
  on_payload_set_actual()
  end 
  on_watchdog_thrd=run on_set_watchdog_thread()
  sync()
  
  #======    End of OnRobot Run    ======#
  #======    OnRobot QC Start    ======#
  
  if(on_toolConnector):
  tc_setup_tool()
  end 
  sync()
  
  #======    End of OnRobot QC Start    ======#
  #======    OnRobot RG Run    ======#
  
  def rg_wait_for_init(tool_index):
  local rg_timeout=0
  while not((rg_product_code_arr[tool_index]==RG_DEVICE_ID_RG2)or(rg_product_code_arr[tool_index]==RG_DEVICE_ID_RG6)):
  sync()
  rg_timeout=rg_timeout+1
  if(rg_timeout>ON_INIT_TIMEOUT):
  popup(rg_device_id_timeout,rg_error_title,error=True,blocking=False)
  halt
  end 
  end 
  if ON_DEBUG_LOG:
  textmsg("Identified RG: ",rg_product_code_arr[tool_index])
  end 
  end 
  rg_dataRead_running=True
  sync()
  rg_dataRead_thrd=run rg_dataRead_thread()
  sync()
  textmsg(rg_device_id_waiting)
  if(rg_index==ON_DI_DUAL):
  rg_wait_for_init(ON_DI_PRIMARY)
  rg_wait_for_init(ON_DI_SECONDARY)
  else:
  rg_wait_for_init(rg_index)
  end 
  sync()
  rg_dataProcess_running=True
  sync()
  rg_dataProcess_thrd=run rg_dataProcess_thread()
  sync()
  rg_monitor_thread_handle=run rg_monitor_thread()
  sync()
  
  #======    End of OnRobot RG Run    ======#
  # end: URCap Installation Node
  #while (True):
    $ 1 "Robot Program"
    $ 2 "rg_grip({0},{1})"
    rg_grip({0},{1}, blocking = True, depth_comp = True)
    # begin: URCap Program Node
    #   Source: OnRobot, 6.0.0, OnRobot A/S
    #   Type: RG Grip
    #$ 3 "RG Grip"
#on_return = rg_grip(24, 58, tool_index = 0, blocking = True, depth_comp = True, popupmsg = True)
rg_payload_set(mass = 0.0, tool_index = 0, use_guard = True)
    # end: URCap Program Node
  end
end
"""
