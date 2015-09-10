/*! \class VCC50iPtz
 *  \file vc_c50i_ptz_node.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2015
 *  \brief Class for the node to control the PTZ of the VC-C50i camera
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <string.h>
#include <vector>
#include <stdint.h>
#include <ros/ros.h>
#include <math.h>
#include <cstdlib>


#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
#include <vc_c50i_ptz/serial_device.h>
#include <robotnik_msgs/ptz.h>

#define VCC50IPTZ_MIN_COMMAND_REC_FREQ 		1.0
#define VCC50IPTZ_MAX_COMMAND_REC_FREQ 		10.0
#define VCC50IPTZ_DEFAULT_FREQ				50.0

#define VCC50I_TRANSFERRATE 				9600
#define VCC50I_PARITY 						"odd" //"even" "odd" "none"
#define VCC50I_DATA_SIZE					8



//
// Constants
//
const int MAX_COMMAND_LENGTH = 19;
const int MAX_REQUEST_LENGTH = 17;
const int COMMAND_RESPONSE_BYTES = 6;
const int PACKET_TIMEOUT = 300;
const int SLEEP_TIME_USEC = 300000;
const int PAN_THRESH = 1;
const int TILT_THRESH = 1;
const int ZOOM_THRESH = 1;

using namespace std;

class VCC50iPtz
{

public:

	self_test::TestRunner self_test_;
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	double desired_freq_;
	int baudrate_;

	diagnostic_updater::Updater diagnostic_;							// General status diagnostic updater
	diagnostic_updater::FrequencyStatus freq_diag_;		         		// Component frequency diagnostics
  diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; 	// Topic reception frequency diagnostics
	ros::Time last_command_time_;										// Last moment when the component received a command
	ros::Time last_read_time_;											// Last moment when the positions were read
  diagnostic_updater::FunctionDiagnosticTask command_freq_;


	//! Node running
	bool running;

	/* EXAMPLES */
	// Ros service stop
	// ros::ServiceServer srv_stop_;
    // Publishers
    // ros::Publisher joint_state_pub_;
	// Subscribers
	ros::Subscriber ptz_commands_sub;

	//! Flag to inform about shutdown
	bool shutting_down_;
	bool bReceiving_;
	//! Read write cycle measured frequency
	double cycle_freq_;
	//! Serial Device iface
	SerialDevice *serial;
	string serial_port_;
	int connected_;
  int max_zoom_;
	int pan_, tilt_, zoom_;
	robotnik_msgs::ptz current_state_;

	enum Command {
    DELIM = 0x00, ///<Delimeter character
    DEVICEID = 0x30, ///<Default device ID
    PANSLEW = 0x50, ///<Sets the pan slew
    TILTSLEW = 0x51, ///<Sets the tilt slew
    STOP = 0x53, ///<Stops current pan/tilt motion
    INIT = 0x58, ///<Initializes the camera
    SLEWREQ = 0x59, ///<Request pan/tilt min/max slew
    ANGLEREQ = 0x5c, ///<Request pan/tilt min/max angle
    PANTILT = 0x62, ///<Pan/tilt command
    SETRANGE = 0x64, ///<Pan/tilt min/max range assignment
    PANTILTREQ = 0x63, ///<Request pan/tilt position
    INFRARED = 0x76, ///<Controls operation of IR lighting
    PRODUCTNAME = 0x87, ///<Requests the product name
    LEDCONTROL = 0x8E, ///<Controls LED status
    CONTROL = 0x90, ///<Puts camera in Control mode
    POWER = 0xA0, ///<Turns on/off power
    AUTOFOCUS = 0xA1, ///<Controls auto-focusing functions
    ZOOMSTOP = 0xA2, ///<Stops zoom motion
    GAIN = 0xA5, ///<Sets gain adjustment on camera
    FOCUS = 0xB0, ///<Manual focus adjustment
    ZOOM = 0xB3, ///<Zooms camera lens
    ZOOMREQ = 0xB4, ///<Requests max zoom position
    IRCUTFILTER = 0xB5, ///<Controls the IR cut filter
    DIGITALZOOM = 0xB7, ///<Controls the digital zoom amount
    FOOTER = 0xEF, ///<Packet Footer
    RESPONSE = 0xFE, ///<Packet header for response
    HEADER = 0xFF ///<Packet Header
  };

	// the various error states that the camera can return
	enum Error {
		CAM_ERROR_NONE = 0x30, ///<No error
		CAM_ERROR_BUSY = 0x31, ///<Camera busy, will not execute the command
		CAM_ERROR_PARAM = 0x35, ///<Illegal parameters to function call
		CAM_ERROR_MODE = 0x39,  ///<Not in host control mode
		CAM_ERROR_UNKNOWN = 0xFF ///<Unknown error condition.  Should never happen
	};

	// Types for turning on and off the camera
  enum Power {
    POWER_OFF = 0,
    POWER_ON = 1
  };

	// preset limits on movements.  Based on empirical data
	enum Param {
		MAX_PAN = 98,		// 875 units is max pan assignment
		MIN_PAN = -98,		// -875 units is min pan assignment
		MAX_TILT = 88,		// 790 units is max tilt assignment
		MIN_TILT = -30,		// -267 units is min tilt assignment
		MAX_PAN_SLEW = 90,		// 800 positions per sec (PPS)
		MIN_PAN_SLEW = 1,		// 8 positions per sec (PPS)
		MAX_TILT_SLEW = 69,		// 662 positions per sec (PPS)
		MIN_TILT_SLEW = 1,		// 8 positions per sec (PPS)
		MAX_ZOOM_OPTIC = 1960,
		MIN_ZOOM = 0
	};

/*!	\fn VCC50iPtz::VCC50iPtz()
 * 	\brief Public constructor
*/
VCC50iPtz(ros::NodeHandle h) : self_test_(), diagnostic_(),
  node_handle_(h), private_node_handle_("~"),
  desired_freq_(VCC50IPTZ_DEFAULT_FREQ),
	max_zoom_(MAX_ZOOM_OPTIC),
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
  command_freq_("Command frequency check", boost::bind(&VCC50iPtz::checkCommandSubscriber, this, _1))
{
    running = false;
    ros::NodeHandle VCC50iPtz(node_handle_, "VCC50iPtz");

    // READ ROS PARAMS
    private_node_handle_.param<double>("desired_freq", desired_freq_, VCC50IPTZ_DEFAULT_FREQ);
    private_node_handle_.param<string>("port", serial_port_, "/dev/ttyUSB0");
    private_node_handle_.param<int>("baudrate", baudrate_, VCC50I_TRANSFERRATE);
    // Self test
    self_test_.add("Connect Test", this, &VCC50iPtz::connectTest);

    // Component Setup
    // Opens/setups devices

	/* EXAMPLES */
    // Services and Topics
    // Service to stop robot
	// srv_stop_ = private_node_handle_.advertiseService("stop", &VCC50iPtz::srvCallbackStop, this);
	// Publish joint states for wheel motion visualization
	// joint_state_pub_ = private_node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 10);

	// Subscribing
    ptz_commands_sub = private_node_handle_.subscribe<robotnik_msgs::ptz>("/vc_c50i/ptz_command", 10, &VCC50iPtz::cmdPtzCommandCallback, this);

    // Component frequency diagnostics
    diagnostic_.setHardwareID("rscomponent");
    diagnostic_.add("VCC50iPtz Diagnostic", this, &VCC50iPtz::controllerDiagnostic);
    diagnostic_.add( freq_diag_ );
    diagnostic_.add( command_freq_ );

    // Topics freq control for command topics
    double min_freq = VCC50IPTZ_MIN_COMMAND_REC_FREQ; // If you update these values, the
    double max_freq = VCC50IPTZ_MAX_COMMAND_REC_FREQ; // HeaderlessTopicDiagnostic will use the new values.
    ROS_INFO("Desired freq %5.2f", desired_freq_);

    // Sets the topic frequency we want to monitorize
    subs_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joint_commands", diagnostic_,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
    subs_command_freq->addTask(&command_freq_); // Adding an additional task to the control

	shutting_down_ = false;

	bReceiving_ = false;

	cycle_freq_ = 0.0;

	//Creates the serial device
	serial= new SerialDevice(serial_port_.c_str(), baudrate_, VCC50I_PARITY, VCC50I_DATA_SIZE);

}


/*!	\fn VCC50iPtz::checkCommandSubscriber
 * 	\brief Checks that the controller is receiving at a correct frequency the command messages. Diagnostics
*/
void checkCommandSubscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	ros::Time current_time = ros::Time::now();

	double diff = (current_time - last_command_time_).toSec();

	if(diff > 0.25){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
        // ROS_INFO("checkCommandSubscriber: cmd %lf seconds without commands", diff);
		// If this happens in safe velocity, deactivate flag in order to stop motors.
		// This condition is not relevant in position or position velocity commands
		bReceiving_ = false;

	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
	}
}

/*!	\fn VCC50iPtz::connectTest()
 * 	\brief Test
*/
void connectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
{
	// Connection test or ping test
	// IF OK
	if (this->open() == SERIAL_OK)
	{
	status.summary(0, "Connected successfully.");
  }
}

/*!	\fn VCC50iPtz::controllerDiagnostic
 * 	\brief Checks the status of the driver Diagnostics
*/
void controllerDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	// add and addf are used to append key-value pairs.
	// stat.addf("Controller StatusWord (HEX)", "%x", sw ); // Internal controller status

	// TODO => Add diagnostic info
	// stat.add("Encoder position left", (int) position_left_pps_ );
	// stat.add("Encoder position right", (int) position_right_pps_ );
}

/*!	\fn VCC50iPtz::~VCC50iPtz()
 * 	\brief Public destructor
*/
~VCC50iPtz(){

    delete subs_command_freq;
}

/*!	\fn int VCC50iPtz::start()
 * 	\brief Starts the component
*/
int start(){

	freq_diag_.clear();
	running = true;
  this->open();

	sendInit();
	usleep(10000);

  sendPowerOn();
  usleep(10000);

	return 0;
}


/*!	\fn int VCC50iPtz::stop()
 * 	\brief Stops the component
*/
int stop(){

}



/*! \fn read_and_publish
 *  Do some stuff and publishes state/messages to ROS
 */
int read_and_publish(){

	// Time measurement
	ros::Time current_time = ros::Time::now();
	double diff = (current_time - last_read_time_).toSec();
	last_read_time_ = current_time;
	if (diff>0.0) cycle_freq_ = 1.0 / diff;


	int read_bytes=0;			//Number of received bytes
	int ret = SERIAL_ERROR;
	char cReadBuffer[64] = "\0";
	// Read controller messages
	if (serial->ReadPort(cReadBuffer, &read_bytes, 64)==SERIAL_ERROR) {
			ROS_ERROR("VCC50iPtz::ReadControllerMsgs: Error reading port");
			return SERIAL_ERROR;
				}
	for (int i=0;i<read_bytes;i++)
	{

	ROS_INFO("VCC50iPtz::read_and_publish: %X ", cReadBuffer[i]);
	}
	/*
	while(read_bytes > 0){
		ret = ProcessMsg(cReadBuffer); // returns ERROR or Number of bytes !
		if(ret == ERROR) return ERROR;
		else ret = OK;
								if (serial->ReadPort(cReadBuffer, &read_bytes, 64)==ERROR) {
			//ROS_ERROR("VCC50iPtz::read_and_publish: Error after 1st port read");
			//return ERROR;
									ret = OK;
								}
				}
   */

	/* EXAMPLE */
	// Publish message
    // joint_states_.header.stamp = ros::Time::now();
	// joint_state_pub_.publish( joint_states_ );
}


/*! \fn bool spin()
  *  Main Loop
*/
bool spin()
{
    ros::Rate r(desired_freq_);

    while (!(shutting_down_=ros::isShuttingDown())) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
		if (start() == 0)
		{
			while(ros::ok() && node_handle_.ok()) {

				read_and_publish();

				self_test_.checkTest();

				diagnostic_.update();

				ros::spinOnce();
				r.sleep();
			}

		   ROS_INFO("VCC50iPtz::spin - END OF ros::ok() !!!");
	   } else {
		   // No need for diagnostic here since a broadcast occurs in start
		   // when there is an error.
		   usleep(1000000);
		   self_test_.checkTest();
		   ros::spinOnce();
	  }
   }

   return true;
}

/*!	\fn int VCC50iPtz::Open(char *dev)
 * 	\brief Open serial port
 * 	\returns -1 Error
 * 	\returns 0 Ok
*/
int open()
{

	// Setup serial device
	if (this->serial->OpenPort2() == SERIAL_ERROR) {
          ROS_ERROR("VCC50iPtz::Open: Error Opening Serial Port");
	  // iErrorType = DSPIC_ERROR_OPENING;
	  return SERIAL_ERROR;
          }
	ROS_INFO("VCC50iPtz::Open: serial port opened at %s", serial->GetDevice());

	// Send 1st command to verify device is working
        //usleep(50000);
        //SendCalibrateOffsetGyro();
	return SERIAL_OK;
}


/*! \fn  void cmdPtzCommandCallback(const robotnik_msgs::ptz::ConstPtr &msg)
  * Receives ptz commands
*/
void cmdPtzCommandCallback(const robotnik_msgs::ptz::ConstPtr &cmd)
{
	ROS_DEBUG("cmdPtzCommandCallback: Received commands: pan = %.2lf, tilt = %.2lf, zoom = %.2lf, relative = %d", cmd->pan, cmd->tilt, cmd->zoom, cmd->relative);

  // sendHome();              DEBUG
  // usleep(10000);

	robotnik_msgs::ptz to_send;
  bool change_pan_tilt = false;
  bool change_zoom = false;
  to_send.pan = pan_;
  to_send.tilt = tilt_;
  to_send.zoom = zoom_;

  // Check if the command is relative to the current position
  if (cmd->relative)
  {
    if ( abs(cmd->pan) > PAN_THRESH)
    {
      to_send.pan = cmd->pan + pan_;
      change_pan_tilt = true;
    }
    if ( abs(cmd->tilt) > TILT_THRESH)
    {
      to_send.tilt = cmd->tilt + tilt_;
      change_pan_tilt = true;
    }
    if ( abs(cmd->zoom) > ZOOM_THRESH)
    {
      to_send.zoom = cmd->zoom + zoom_;
      change_zoom = true;
    }
  }
  else
  {
    if ( abs(cmd->pan - pan_) > PAN_THRESH)
    {
      to_send.pan = cmd->pan;
      change_pan_tilt = true;
    }
    if ( abs(cmd->tilt - tilt_) > TILT_THRESH)
    {
      to_send.tilt = cmd->tilt;
      change_pan_tilt = true;
    }
    if ( abs(cmd->zoom - zoom_) > ZOOM_THRESH)
    {
      to_send.zoom = cmd->zoom;
      change_zoom = true;
    }
  }

  if (change_pan_tilt)
  {
    sendAbsPanTilt(to_send.pan, to_send.tilt);
  }
  if (change_zoom)
  {
    sendAbsZoom(to_send.zoom);
  }

  current_state_.pan = pan_;
  current_state_.zoom = zoom_;
  current_state_.tilt = tilt_;

	ROS_ERROR("VCC50iPtz::Zoom STATE to_send.zoom = %d", cmd->zoom);

		return;
}

int sendInit()
{
  char command[MAX_COMMAND_LENGTH];
  int written_bytes=0;

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = CONTROL;
  command[5] = DEVICEID;
  command[6] = FOOTER;

	// Sends the message
	if(serial->WritePort(command, &written_bytes, 7) != SERIAL_OK) {
		ROS_ERROR("VCC50iPtz::sendInit: Error sending message");
        }

  	ROS_DEBUG("VCC50iPtz::sendInit: %X %X %X %X %X %X %X  %d", command[0],command[1],command[2],command[3],command[4],command[5],command[6], written_bytes);
		return 0;
}

int sendHome()
{
  char command[MAX_COMMAND_LENGTH];
  int written_bytes=0;

	command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = 0x00;
  command[4] = 0x57;
  command[5] = FOOTER;

	// Sends the message
	if(serial->WritePort(command, &written_bytes, 6) != SERIAL_OK) {
		ROS_ERROR("VCC50iPtz::sendHome: Error sending message");
        }

  	ROS_DEBUG("VCC50iPtz::sendHome: %X %X %X %X %X %X %d", command[0],command[1],command[2],command[3],command[4],command[5], written_bytes);
		return 0;
}


int sendPowerOn()
{
  char command[MAX_COMMAND_LENGTH];
  int written_bytes=0;

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = POWER;
  command[5] = DEVICEID +1;
  command[6] = FOOTER;

	// Sends the message
	if(serial->WritePort(command, &written_bytes, 7) != SERIAL_OK) {
		ROS_ERROR("VCC50iPtz::sendPowerOn: Error sending message");
        }

  	ROS_DEBUG("VCC50iPtz::sendPowerOn: %X %X %X %X %X %X %X  %d", command[0],command[1],command[2],command[3],command[4],command[5],command[6], written_bytes);
		return 0;
}


int sendAbsPanTilt(int pan, int tilt)
{
  char command[MAX_COMMAND_LENGTH];
  int convpan, convtilt;
  char buf[5];
  int ppan, ttilt;
	int written_bytes=0;

  ppan = pan; ttilt = tilt;
  if(pan < MIN_PAN)
  {
    ppan = (int)MIN_PAN;
  }
  else if(pan > MAX_PAN)
  {
      ppan = (int)MAX_PAN;
  }

  if (tilt > MAX_TILT)
  {
    ttilt = (int)MAX_TILT;
  }
  else if(tilt < MIN_TILT)
  {
      ttilt = (int)MIN_TILT;
  }
  //puts("Camera pan angle thresholded");

  //puts("Camera tilt angle thresholded");

  convpan = (int)floor(ppan/.1125) + 0x8000;
  convtilt = (int)floor(ttilt/.1125) + 0x8000;
  //   fprintf(stdout, "ppan: %d ttilt: %d conpan: %d contilt: %d\n",
  // 	  ppan,ttilt,convpan,convtilt);
  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = PANTILT;
  // pan position

  snprintf((char *)buf,sizeof(buf), "%X", convpan);

  command[5] = buf[0];
  command[6] = buf[1];
  command[7] = buf[2];
  command[8] = buf[3];
  // tilt position
  snprintf((char *)buf,sizeof(buf), "%X", convtilt);
  command[9]  = buf[0];
  command[10] = buf[1];
  command[11] = buf[2];
  command[12] = buf[3];
  command[13] = (unsigned char) FOOTER;

  tilt_ = ttilt;
  pan_ = ppan;

	if(serial->WritePort(command, &written_bytes, 14) != SERIAL_OK)
	{
	  ROS_ERROR("VCC50iPtz::sendAbsPanTilt: Error sending message");
  }

  ROS_DEBUG("VCC50iPtz::sendAbsPanTilt: %X %X %X %X %X %X %X  %d", command[0],command[1],command[2],command[3],command[4],command[5],command[6], written_bytes);

	return 0;
}


int sendAbsZoom(int zoom)
{
  char command[MAX_COMMAND_LENGTH];
  char buf[5];
  int i;
  int written_bytes=0;

  if(zoom < 0)
    zoom = 0;

  else
    if(zoom > max_zoom_){
      zoom = max_zoom_;
    }

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = ZOOM;

  sprintf((char *)buf, "%4X", zoom);

  for (i=0;i<3;i++)
    if (buf[i] == ' ')
      buf[i] = '0';

  // zoom position
  command[5] = buf[0];
  command[6] = buf[1];
  command[7] = buf[2];
  command[8] = buf[3];
  command[9] = FOOTER;

  zoom_ = zoom;

	ROS_ERROR("VCC50iPtz::Zoom STATE zoom_ = %d", zoom_);

	if(serial->WritePort(command, &written_bytes, 14) != SERIAL_OK)
	{
	  ROS_ERROR("VCC50iPtz::sendAbsZoom: Error sending message");
  }

  ROS_DEBUG("VCC50iPtz::sendAbsZoom: %X %X %X %X %X %X %X  %d", command[0],command[1],command[2],command[3],command[4],command[5],command[6], written_bytes);
  //return (receiveCommandAnswer(COMMAND_RESPONSE_BYTES));
}

/*
int P2OSPtz::getAbsPanTilt(int* pan, int* tilt)
{
  unsigned char command[MAX_COMMAND_LENGTH];
  unsigned char reply[MAX_REQUEST_LENGTH];
  int reply_len;
  char buf[4];
  char byte;
  unsigned int u_val;
  int val;
  int i;

  command[0] = HEADER;
  command[1] = DEVICEID;
  command[2] = DEVICEID;
  command[3] = DELIM;
  command[4] = PANTILTREQ;
  command[5] = FOOTER;

  if (sendRequest(command, 6, reply))
    return(-1);
  if (bidirectional_com_)
  {
    reply_len = receiveRequestAnswer(reply,14,0);
  }
  else
  {
    return 0;
  }

  if ( reply_len != 14 ) {
    ROS_ERROR("Reply Len = %i; should equal 14", reply_len);
    return -1;
  }

  // remove the ascii encoding, and put into 4-byte array
  for (i = 0; i < 4; i++)
  {
    byte = reply[i+5];
    if (byte < 0x40)
      byte = byte - 0x30;
    else
      byte = byte - 'A' + 10;
    buf[i] = byte;
  }

  // convert the 4-bytes into a number
  u_val = buf[0] * 0x1000 + buf[1] * 0x100 + buf[2] * 0x10 + buf[3];

  // convert the number to a value that's meaningful, based on camera specs
  val = (int)(((int)u_val - (int)0x8000) * 0.1125);

  // now set myPan to the response received for where the camera thinks it is
  *pan = val;

  // repeat the steps for the tilt value
  for (i = 0; i < 4; i++)
  {
    byte = reply[i+9];
    if (byte < 0x40)
      byte = byte - 0x30;
    else
      byte = byte - 'A' + 10;
    buf[i] = byte;
  }
  u_val = buf[0] * 0x1000 + buf[1] * 0x100 + buf[2] * 0x10 + buf[3];
  val =(int)(((int)u_val  - (int)0x8000) * 0.1125);
  *tilt = val;

  return(0);
}
*/


}; // class VCC50iPtz


// MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "vc_c50i_ptz_component");

	ros::NodeHandle n;
  	VCC50iPtz ctrl(n);

  	ctrl.spin();

	return (0);
}
// EOF
