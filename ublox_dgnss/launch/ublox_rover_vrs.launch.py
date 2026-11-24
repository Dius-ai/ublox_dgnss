"""Launch a ZED-F9P rover for VRS NTRIP corrections through the pty link.

The driver opens a pseudo-terminal at `pty_link` that a serial NTRIP client (e.g.
rtv_ntrip_bridge) uses as if it were the receiver: it reads the receiver's GGA there (a VRS
caster needs the rover position) and writes the caster's RTCM 3 back, which the driver forwards
to the receiver over USB. Corrections therefore do not pass through ROS, and the
/ntrip_client/rtcm subscription is off by default.

Receiver profile: output on USB only (UART1/UART2/I2C/SPI silenced, their factory-default NMEA
fills the shared TX buffer at higher rates), UBX + RTCM 3 in, UBX + NMEA GGA out, RTK fixed
mode, every-epoch navigation messages plus NAV-PL protection levels.
"""
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  """Generate launch description for a VRS rover."""

  namespace = LaunchConfiguration('namespace')
  device_serial_string = LaunchConfiguration('device_serial_string')
  frame_id = LaunchConfiguration('frame_id')
  pty_link = LaunchConfiguration('pty_link')
  rate_meas_ms = LaunchConfiguration('rate_meas_ms')
  one_hz_divider = LaunchConfiguration('one_hz_divider')
  dyn_model = LaunchConfiguration('dyn_model')
  rtcm_input_enabled = LaunchConfiguration('rtcm_input_enabled')

  args = [
    DeclareLaunchArgument('log_level', default_value=TextSubstitution(text='INFO')),
    DeclareLaunchArgument('namespace', default_value=''),
    DeclareLaunchArgument(
      'device_serial_string', default_value='',
      description='Serial string of the device to use (empty: first F9P found)'),
    DeclareLaunchArgument(
      'frame_id', default_value='ubx',
      description='The frame_id to use in header of published messages'),
    DeclareLaunchArgument(
      'pty_link',
      default_value=PathJoinSubstitution(
        [EnvironmentVariable('XDG_RUNTIME_DIR', default_value='/tmp'), 'ublox_dgnss']),
      description='Symlink to the pty the NTRIP client opens as its serial device'),
    DeclareLaunchArgument(
      'rate_meas_ms', default_value='200',
      description='Measurement period in ms (200 = 5 Hz)'),
    DeclareLaunchArgument(
      'one_hz_divider', default_value='5',
      description='Epochs per 1 Hz message (GGA, NAV-SAT/SIG/STATUS); 1000 / rate_meas_ms'),
    DeclareLaunchArgument(
      'dyn_model', default_value='4',
      description='CFG_NAVSPG_DYNMODEL: 0 portable, 2 stationary, 3 pedestrian, 4 automotive'),
    DeclareLaunchArgument(
      'rtcm_input_enabled', default_value='false',
      description='Also accept RTCM from /ntrip_client/rtcm (off: corrections come via the pty)'),
  ]

  params = [{
    'DEVICE_FAMILY': 'F9P',
    'DEVICE_SERIAL_STRING': device_serial_string,
    'FRAME_ID': frame_id,
    'PTY_LINK_PATH': pty_link,
    'RTCM_INPUT_ENABLED': ParameterValue(rtcm_input_enabled, value_type=bool),

    # output on USB only
    'CFG_UART1OUTPROT_UBX': False,
    'CFG_UART1OUTPROT_NMEA': False,
    'CFG_UART1OUTPROT_RTCM3X': False,
    'CFG_UART2OUTPROT_UBX': False,
    'CFG_UART2OUTPROT_NMEA': False,
    'CFG_UART2OUTPROT_RTCM3X': False,
    'CFG_I2COUTPROT_UBX': False,
    'CFG_I2COUTPROT_NMEA': False,
    'CFG_I2COUTPROT_RTCM3X': False,
    'CFG_SPIOUTPROT_UBX': False,
    'CFG_SPIOUTPROT_NMEA': False,
    'CFG_SPIOUTPROT_RTCM3X': False,
    # USB: UBX + RTCM 3 in, UBX + NMEA (GGA for the NTRIP client) out, no RTCM out
    'CFG_USBINPROT_UBX': True,
    'CFG_USBINPROT_NMEA': False,
    'CFG_USBINPROT_RTCM3X': True,
    'CFG_USBOUTPROT_UBX': True,
    'CFG_USBOUTPROT_NMEA': True,
    'CFG_USBOUTPROT_RTCM3X': False,

    # navigation
    'CFG_RATE_MEAS': ParameterValue(rate_meas_ms, value_type=int),
    'CFG_RATE_NAV': 1,
    'CFG_NAVSPG_DYNMODEL': ParameterValue(dyn_model, value_type=int),
    'CFG_NAVHPG_DGNSSMODE': 3,  # RTK fixed: ambiguities fixed when possible

    # every epoch
    'CFG_MSGOUT_UBX_NAV_PVT_USB': 1,
    'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1,
    'CFG_MSGOUT_UBX_NAV_COV_USB': 1,
    'CFG_MSGOUT_UBX_NAV_DOP_USB': 1,
    'CFG_MSGOUT_UBX_NAV_PL_USB': 1,
    'CFG_MSGOUT_UBX_NAV_EOE_USB': 1,
    'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1,  # one per received RTCM message
    # 1 Hz
    'CFG_MSGOUT_UBX_NAV_STATUS_USB': ParameterValue(one_hz_divider, value_type=int),
    'CFG_MSGOUT_UBX_NAV_SAT_USB': ParameterValue(one_hz_divider, value_type=int),
    'CFG_MSGOUT_UBX_NAV_SIG_USB': ParameterValue(one_hz_divider, value_type=int),
    'CFG_MSGOUT_NMEA_ID_GGA_USB': ParameterValue(one_hz_divider, value_type=int),
    'CFG_MSGOUT_UBX_MON_COMMS_USB': 1,
    # factory-default NMEA nobody needs
    'CFG_MSGOUT_NMEA_ID_GLL_USB': 0,
    'CFG_MSGOUT_NMEA_ID_GSA_USB': 0,
    'CFG_MSGOUT_NMEA_ID_GSV_USB': 0,
    'CFG_MSGOUT_NMEA_ID_RMC_USB': 0,
    'CFG_MSGOUT_NMEA_ID_VTG_USB': 0,
    'CFG_MSGOUT_NMEA_ID_GST_USB': 0,
    'CFG_MSGOUT_NMEA_ID_ZDA_USB': 0,
  }]

  container1 = ComposableNodeContainer(
    name='ublox_dgnss_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_dgnss_node',
        plugin='ublox_dgnss::UbloxDGNSSNode',
        name='ublox_dgnss',
        namespace=namespace,
        parameters=params
      )
    ]
  )

  container2 = ComposableNodeContainer(
    name='ublox_nav_sat_fix_hp_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_nav_sat_fix_hp_node',
        plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
        name='ublox_nav_sat_fix_hp',
        namespace=namespace
      )
    ]
  )

  return launch.LaunchDescription(args + [container1, container2])
