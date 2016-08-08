#ifndef FT_SIM_HPP
#define FT_SIM_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <Eigen/Dense>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <string>
#include <fstream>
#include <streambuf>
#include <memory>
#include <functional>
#include <rst-rt/dynamics/Wrench.hpp>

#include <boost/shared_ptr.hpp>

namespace cogimon {

class ftSim: public RTT::TaskContext {
public:
	ftSim(std::string const& name);bool configureHook();
	void updateHook();bool getModel(const std::string& name);
//    void WorldUpdateBegin();
//    void WorldUpdateEnd();
	virtual ~ftSim() {
	}
protected:
	void gazeboUpdateHook(gazebo::msgs::WrenchStamped msg);
	void gazeboUpdateHook2();
//    virtual void OnUpdate();
	bool gazeboConfigureHook(gazebo::physics::ModelPtr model);

	gazebo::physics::ModelPtr model;
	gazebo::msgs::WrenchStamped msg;
	gazebo::sensors::ForceTorqueSensor parentSensor;
	gazebo::sensors::ForceTorqueSensorPtr ft_sensor;
	gazebo::event::ConnectionPtr updateConnection;
	gazebo::event::ConnectionPtr updateConnection2;
	gazebo::sensors::SensorManager *mgr;

	RTT::SendHandle<gazebo::physics::ModelPtr(const std::string&, double)> get_model_handle;

	gazebo::physics::Joint_V gazebo_joints_;
	gazebo::physics::Link_V model_links_;
	std::string ft_sensor_name;
	rstrt::dynamics::Wrench ft_wrench;
	RTT::OutputPort<rstrt::dynamics::Wrench> ft_wrench_output_port;
	std::string forceTorqueSensorString =
	"<sdf version='1.4'> <sensor name='force_torque' type='force_torque'> <update_rate>30</update_rate> <always_on>true</always_on> </sensor> </sdf>";
	gazebo::physics::LinkPtr link;
	gazebo::event::ConnectionPtr connection;
private:
	bool is_configured;
	bool tmep;
};

}
#endif
