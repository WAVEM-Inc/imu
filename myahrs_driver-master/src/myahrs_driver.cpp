//------------------------------------------------------------------------------
// Copyright (c) 2015, Yoonseok Pyo
// All rights reserved.

// License: BSD

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of myahrs_driver nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------
#include <myahrs_driver/myahrs_plus.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <mpc_navigation/Gyro.h>

//------------------------------------------------------------------------------
using namespace WithRobot;

//------------------------------------------------------------------------------
class MyAhrsDriverForROS : public iMyAhrsPlus
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Publisher imu_data_raw_pub_;
  ros::Publisher imu_data_pub_;
  ros::Publisher imu_mag_pub_;
  ros::Publisher imu_temperature_pub_;

  ros::Publisher ready_pub;  //!< gyro 토픽 발행자


  tf::TransformBroadcaster broadcaster_;

  Platform::Mutex lock_;
  SensorData sensor_data_;

  std::string parent_frame_id_;
  std::string frame_id_;
  double linear_acceleration_stddev_;
  double angular_velocity_stddev_;
  double magnetic_field_stddev_;
  double orientation_stddev_;

  ros::Time current_time ;  //!< 현재 시각
  ros::Time last_time;	 //!< 가장 최근 오도메트리 공표 시각: 델타 계산에 필요
  double th;  //!< 현재 위치에서의 회전각
  double sum_th;
  int count;

  //!low pass filter
  double filter_y;
  double prev_y;
  double prev_theta;
  double filter_y_dt;
  double tau;
  double binary_i;
  double prev_i;
  double ic;
  double c1;
  double rc;
  double ri;
  double prev_ri;
  double threshold;
  bool ideal_zero;
  double max_theta;


  mpc_navigation::Gyro gyro;
  const char *OBS_FRAME_ID = "/mpc/gyro";  //!< gyro  프레임 식별자
  unsigned char ready_ = 0;




  // iMyAhrsPlus::OnSensorData()를 재정의
  void OnSensorData(int sensor_id, SensorData data)
  {
    //ROS_ERROR("OnSensorData");
    LockGuard _l(lock_);
    sensor_data_ = data;
    publish_topic(sensor_id);
  }

  // iMyAhrsPlus::OnAttributeChange()를 재정의
  void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
  {
    printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
  }

public:
  MyAhrsDriverForROS(std::string port="", int baud_rate=115200)
  : iMyAhrsPlus(port, baud_rate),
    nh_priv_("~")
  {
    // dependent on user device
    nh_priv_.setParam("port", port);
    nh_priv_.setParam("baud_rate", baud_rate);
    // default frame id
    nh_priv_.param("frame_id", frame_id_, std::string("imu_link"));
    // for testing the tf
    nh_priv_.param("parent_frame_id_", parent_frame_id_, std::string("base_link"));

    // defaults obtained experimentally from device
    nh_priv_.param("linear_acceleration_stddev", linear_acceleration_stddev_, 0.026831);
    nh_priv_.param("angular_velocity_stddev", angular_velocity_stddev_, 0.002428);
    nh_priv_.param("magnetic_field_stddev", magnetic_field_stddev_, 0.00000327486);
    nh_priv_.param("orientation_stddev", orientation_stddev_, 0.002143);
    // publisher for streaming
    imu_data_raw_pub_   = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    imu_data_pub_       = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
    imu_mag_pub_        = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
    imu_temperature_pub_= nh_.advertise<std_msgs::Float64>("imu/temperature", 1);


    ready_pub = nh_.advertise<mpc_navigation::Gyro>(OBS_FRAME_ID, 10);

    current_time = last_time = ros::Time::now();
    th = 0.0;
    sum_th = 0.0;
    count = 0;

    filter_y = 0.0;
    prev_y = 0.0;
    prev_theta =0.0; 
    tau = 0.01; // 0.01 //0.00015
    filter_y_dt = 0.0;
    binary_i = 0.0;
    prev_i = 0.0;
    ic = 0.00000005; //0.00000015
    c1 = 0.001; // 0.001
    rc = 1.0;
    ri = 0.0;
    prev_ri = 0.0;
    threshold = 15;
    ideal_zero = false;
    max_theta = 0.0;

  }

  ~MyAhrsDriverForROS()
  {}

  bool initialize()
  {
    bool ok = false;


    // 용도가 정의되어 있으므로 센서 초기화 과정을 method로 만듬
    do
    {
      if(start() == false) break;
      //Euler angle(x, y, z axis)
      //IMU(linear_acceleration, angular_velocity, magnetic_field)
      if(cmd_binary_data_format("EULER, IMU") == false) break;
      // 100Hz
      if(cmd_divider("1") == false) break;
      // Binary and Continue mode
      if(cmd_mode("BC") == false) break;
      ok = true;
    } while(0);

    return ok;
  }

  inline void get_data(SensorData& data)
  {
    LockGuard _l(lock_);
    data = sensor_data_;
  }

  inline SensorData get_data()
  {
    LockGuard _l(lock_);
    return sensor_data_;
  }

  int SIGN(double v){
    if (v > 0) return 1;
    if (v < 0) return -1;
    return 0;
  }

  void publish_topic(int sensor_id)
  {
    sensor_msgs::Imu imu_data_raw_msg;
    sensor_msgs::Imu imu_data_msg;
    sensor_msgs::MagneticField imu_magnetic_msg;
    std_msgs::Float64 imu_temperature_msg;



    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    double linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
    double angular_velocity_cov    = angular_velocity_stddev_ * angular_velocity_stddev_;
    double magnetic_field_cov      = magnetic_field_stddev_ * magnetic_field_stddev_;
    double orientation_cov         = orientation_stddev_ * orientation_stddev_;

    imu_data_raw_msg.linear_acceleration_covariance[0] =
    imu_data_raw_msg.linear_acceleration_covariance[4] =
    imu_data_raw_msg.linear_acceleration_covariance[8] =
    imu_data_msg.linear_acceleration_covariance[0] =
    imu_data_msg.linear_acceleration_covariance[4] =
    imu_data_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;

    imu_data_raw_msg.angular_velocity_covariance[0] =
    imu_data_raw_msg.angular_velocity_covariance[4] =
    imu_data_raw_msg.angular_velocity_covariance[8] =
    imu_data_msg.angular_velocity_covariance[0] =
    imu_data_msg.angular_velocity_covariance[4] =
    imu_data_msg.angular_velocity_covariance[8] = angular_velocity_cov;

    imu_data_msg.orientation_covariance[0] =
    imu_data_msg.orientation_covariance[4] =
    imu_data_msg.orientation_covariance[8] = orientation_cov;

    imu_magnetic_msg.magnetic_field_covariance[0] =
    imu_magnetic_msg.magnetic_field_covariance[4] =
    imu_magnetic_msg.magnetic_field_covariance[8] = magnetic_field_cov;

    static double convertor_g2a  = 9.80665;    // for linear_acceleration (g to m/s^2)
    static double convertor_d2r  = M_PI/180.0; // for angular_velocity (degree to radian)
    static double convertor_r2d  = 180.0/M_PI; // for easy understanding (radian to degree)
    static double convertor_ut2t = 1000000;    // for magnetic_field (uT to Tesla)
    static double convertor_c    = 1.0;        // for temperature (celsius)

    double roll, pitch, yaw;

    // original sensor data used the degree unit, convert to radian (see ROS REP103)
    // we used the ROS's axes orientation like x forward, y left and z up
    // so changed the y and z aixs of myAHRS+ board
    roll  =  sensor_data_.euler_angle.roll*convertor_d2r;
    pitch = -sensor_data_.euler_angle.pitch*convertor_d2r;
    yaw   = -sensor_data_.euler_angle.yaw*convertor_d2r;

    ImuData<float>& imu = sensor_data_.imu;

    tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);

    ros::Time now = ros::Time::now();

    imu_data_raw_msg.header.stamp =
    imu_data_msg.header.stamp     =
    imu_magnetic_msg.header.stamp = now;

    imu_data_raw_msg.header.frame_id =
    imu_data_msg.header.frame_id     =
    imu_magnetic_msg.header.frame_id = frame_id_;

    // orientation
    imu_data_msg.orientation.x = orientation[0];
    imu_data_msg.orientation.y = orientation[1];
    imu_data_msg.orientation.z = orientation[2];
    imu_data_msg.orientation.w = orientation[3];

    // original data used the g unit, convert to m/s^2
    imu_data_raw_msg.linear_acceleration.x =
    imu_data_msg.linear_acceleration.x     =  imu.ax * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.y =
    imu_data_msg.linear_acceleration.y     = -imu.ay * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.z =
    imu_data_msg.linear_acceleration.z     = -imu.az * convertor_g2a;

    // original data used the degree/s unit, convert to radian/s
    imu_data_raw_msg.angular_velocity.x =
    imu_data_msg.angular_velocity.x     =  imu.gx * convertor_d2r;
    imu_data_raw_msg.angular_velocity.y =
    imu_data_msg.angular_velocity.y     = -imu.gy * convertor_d2r;
    imu_data_raw_msg.angular_velocity.z =
    imu_data_msg.angular_velocity.z     = -imu.gz * convertor_d2r;

    // original data used the uTesla unit, convert to Tesla
    imu_magnetic_msg.magnetic_field.x =  imu.mx / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.y = -imu.my / convertor_ut2t;
    imu_magnetic_msg.magnetic_field.z = -imu.mz / convertor_ut2t;

    // original data used the celsius unit
    imu_temperature_msg.data = imu.temperature;


    //201610- KKY -begin
    double theta = imu_data_msg.angular_velocity.z;
    double dtheta = 0.0;




    //static bias erase
    if(count < 1000){
        sum_th += theta;

        if(fabs(max_theta) < fabs(theta) ){
            max_theta = theta;
        }

//        ROS_ERROR("max_th : %f", max_theta);

//        ROS_ERROR("sum_th : %f", sum_th);


    }else if(count == 1000){
        sum_th /= count;

        sum_th = (sum_th + max_theta) / 2;

        theta = prev_y = prev_theta = theta - sum_th;

        ROS_ERROR("bias OK : %f", sum_th);
        ready_ = 1;

    }else{
//        ROS_ERROR("bias : %f", sum_th);

        theta = theta -sum_th;


 	//low pass filter
        //! tau 0.001 0.01 0.1,
        //!filter_y = (tau * prev_y + ts * raw_data ) / ( tau + ts)
        filter_y = (tau * prev_y + dt * theta) / (tau + dt);
        //ROS_ERROR("dtheta : %f", dtheta);
//        ROS_ERROR("filter_y : %f", filter_y);
//        ROS_ERROR("prev_y : %f", prev_y);
        //ROS_ERROR("dt : %f", dt);
        theta = filter_y;


	prev_y = filter_y;

	//repetition attenuator
	if( SIGN(prev_theta) == SIGN(theta) ){
		ri = prev_ri+1.0;  
	}else{
		ri = 1.0;
	}
	prev_ri = ri;
	rc = (1.0 + c1) / (1.0 +(c1 * ri));



        //Binary I-controller
        //! ic = 0.001
        //! SIGN( x > 0 : 1, x = 0 : 0, x < 0 : -1)
        //! Ii = Ii-1 - SIGN(Wi-1)ic
        binary_i = prev_i -SIGN(prev_theta)*ic*rc;
//        ROS_ERROR("pre bi : %f", binary_i);
        //고정증분값 제거 ed 20160118

	prev_i = binary_i;

	theta = theta + binary_i;
	prev_theta = theta;


        if((theta > 0.0 && theta < 0.004 ) || (theta < 0.0 && theta > -0.004)){
            dtheta = 0.0;
        }else{
            dtheta = theta * dt;
        }
	


//        ROS_ERROR("theta : %f", theta);
//	ROS_ERROR("ri : %f, rc : %f", ri, rc);
//	ROS_ERROR("binary_i : %f", binary_i);
//        ROS_ERROR("imu_data_msg.angular_velocity.z : %f", imu_data_msg.angular_velocity.z);
//        ROS_ERROR("dtheta : %f", dtheta);

        //max-min +- 180/3.14 (degree/radian)
        if(th >= M_PI){
            th = -2*M_PI + th + dtheta;
        }else if(th <= -M_PI){
            th = 2*M_PI + th + dtheta;
        }else{
            th += dtheta;
        }


        //ROS_ERROR("th : %f", th);



        imu_data_msg.angular_velocity.z = th;
        //201610- KKY -end

        // publish the IMU data
        imu_data_raw_pub_.publish(imu_data_raw_msg);
        imu_data_pub_.publish(imu_data_msg);
        imu_mag_pub_.publish(imu_magnetic_msg);
        imu_temperature_pub_.publish(imu_temperature_msg);


        //! obstacle 메시지 값 채우기
        gyro.header.stamp = current_time;
        gyro.header.frame_id = OBS_FRAME_ID;
        // ROS_INFO("publishObstacle");

        //! 위치 값 채우기
        gyro.ready = ready_;

        //! gyro 토픽 공표
        ready_pub.publish(gyro);


    }



//default
/*
      if((theta > 0.0 && theta < 0.005 ) || (theta < 0.0 && theta > -0.005)){
            dtheta = 0.0;
        }else{
            dtheta = theta * dt;
        }
*/

    //dtheta = theta;

//    if(dtheta == 0.0){
//        ideal_zero = true;
//    }

//    //bias erase
//    if(count < 1000){
//        sum_th += dtheta;

////        if(fabs(max_theta) < fabs(dtheta) ){
////            max_theta = dtheta;
////        }
//        ROS_ERROR("sum_th : %f", sum_th);
//        ROS_ERROR("max_th : %f", max_theta);

//    }else if(count == 1000){
//        sum_th /= count;

//        //sum_th = (sum_th + max_theta) / 2;

//        prev_y = dtheta - sum_th;
//        prev_theta = dtheta;

//    }else{
//        ROS_ERROR("==============s===============");
//        ROS_ERROR("imu_data_msg.angular_velocity.z : %f", imu_data_msg.angular_velocity.z);
//        ROS_ERROR("bias : %f", sum_th);
//        ROS_ERROR("dtheta : %f", dtheta);

//        //repetition attenuator
////        if(SIGN(prev_theta) == SIGN(dtheta)){
////            ic += 0.000001;
////        }else{
////            ic -= 0.000001;
////        }


//        //if(dtheta != 0.0){
//            dtheta = (dtheta - sum_th);
//        //}

//        ROS_ERROR("dtheta_f : %f", dtheta);
//        //th += dtheta;
//        //dtheta = dtheta - sum_th;
//        //100hz -> ?ms

//        //low pass filter
//        //! tau 0.001 0.01 0.1,
//        //!filter_y = (tau * prev_y + ts * raw_data ) / ( tau + ts)
//        filter_y = (tau * prev_y + dt * dtheta) / (tau + dt);
//        //ROS_ERROR("dtheta : %f", dtheta);
//        ROS_ERROR("filter_y : %f", filter_y);
//        ROS_ERROR("prev_y : %f", prev_y);
//        //ROS_ERROR("dt : %f", dt);
//        //filter_y_dt = filter_y * dt;



//        //Binary I-controller
//        //! ic = 0.001
//        //! SIGN( x > 0 : 1, x = 0 : 0, x < 0 : -1)
//        //! Ii = Ii-1 - SIGN(Wi-1)ic
//        binary_i = prev_i -SIGN(prev_theta)*ic;
//        ROS_ERROR("pre bi : %f", binary_i);
//        //고정증분값 제거 ed 20160118

//        if(binary_i < 0.0)
//            binary_i *= 0.955425;

//        dtheta = filter_y + binary_i;

//        filter_y_dt = dtheta * dt;
//        ROS_ERROR("ic : %f", ic );
//        ROS_ERROR("-SIGN() : %d", -SIGN(prev_theta));
//        ROS_ERROR("binary_i : %f", binary_i);
//        ROS_ERROR("prev_i : %f", prev_i);
//        ROS_ERROR("dtheta_F : %f", dtheta);
//        ROS_ERROR("filter_y_dt : %f", filter_y_dt);
//        ROS_ERROR("==============e===============");


//        if(fabs(filter_y_dt) < 0.00003 ){

//        }else{
//        th += filter_y_dt;
//        }

//        prev_y = filter_y;
//        prev_i = binary_i;

//        prev_theta = dtheta;

//    }





/*
    broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw),
                                                                  tf::Vector3(0.0, 0.0, 0.0)),
                                                    ros::Time::now(), frame_id_, parent_frame_id_));
*/

    last_time = current_time;
    count++;
    ROS_INFO("th : %f", th);
    }

};


//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "myahrs_driver");

  std::string port = std::string("/dev/ttyACM0");
  int baud_rate    = 115200;

  ros::param::get("~port", port);
  ros::param::get("~baud_rate", baud_rate);

  MyAhrsDriverForROS sensor(port, baud_rate);


  if(sensor.initialize() == false)
  {
    ROS_ERROR("%s\n", "Initialize() returns false, please check your devices.");
    return 0;
  }
  else
  {
    ROS_ERROR("Initialization OK!\n");
  }

  ros::spin();

  return 0;
}

//------------------------------------------------------------------------------
