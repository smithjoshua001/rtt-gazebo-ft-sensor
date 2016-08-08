#include <rtt/Operation.hpp>
#include <rtt-gazebo-ft-sim.hpp>

using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

ftSim::ftSim(const std::string &name) :
		TaskContext(name), is_configured(false) {
	this->addOperation("getModel", &ftSim::getModel, this, ClientThread);
	ft_sensor_name = "placeholder";
	this->addProperty("ft_sensor_name", ft_sensor_name).doc(
			"Force Torque Sensor Link name");
	tmep = false;

}
bool ftSim::getModel(const std::string& model_name) {
	if (model) {
		log(Warning) << "Model [" << model_name << "] already loaded !"
				<< endlog();
		return true;
	}
	gazebo::printVersion();
	if (!gazebo::physics::get_world()) {
		log(Error) << "getWorldPtr does not seem to exists" << endlog();
		return false;
	}
	model = gazebo::physics::get_world()->GetModel(model_name);
	if (model) {
		log(Info) << "Model [" << model_name << "] successfully loaded !"
				<< endlog();
		return true;
	}
	return bool(model);
}

void ftSim::updateHook() {
//	if(!tmep){
//	connection = ft_sensor->ConnectUpdate(boost::bind(&ftSim::gazeboUpdateHook, this, _1));
//	tmep=true;
//	}
}

bool ftSim::configureHook() {
	this->is_configured = gazeboConfigureHook(model);
	return is_configured;
}
//void ftSim::gazeboUpdateHook(gazebo::physics::ModelPtr model) {
//	if (model.get() == NULL) {
//		return;
//	}
//	gazebo::physics::LinkPtr ft_link = model->GetLink(ft_link_name);
//
//	gazebo::sensors::ForceTorqueSensorPtr ft_sensor =
//			gazebo::sensors::get_sensor(ft_link->GetSensorName(0));
//
//	ft_sensor->ConnectUpdate(boost::bind(&ftSim::gazeboUpdateHook, this, &msg));
//
//}

void ftSim::gazeboUpdateHook(gazebo::msgs::WrenchStamped msg) {
	if(this->isRunning()){
	ft_wrench.forces(0) = msg.wrench().force().x();
	ft_wrench.forces(1) = msg.wrench().force().y();
	ft_wrench.forces(2) = msg.wrench().force().z();
	ft_wrench.torques(0) = msg.wrench().torque().x();
	ft_wrench.torques(1) = msg.wrench().torque().y();
	ft_wrench.torques(2) = msg.wrench().torque().z();
	ft_wrench_output_port.write(ft_wrench);
	}
//	RTT::log(RTT::Info) << link->GetRelativeForce().x<<", "<<link->GetRelativeForce().y<<", "<<link->GetRelativeForce().z<<RTT::endlog();

}
//void ftSim::gazeboUpdateHook2() {
//	RTT::log(RTT::Info) << link->GetRelativeForce().x<<", "<<link->GetRelativeForce().y<<", "<<link->GetRelativeForce().z<<RTT::endlog();

//}

//Here we already have the model!
bool ftSim::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
	if (model.get() == NULL ) {
		RTT::log(RTT::Error) << "No model could be loaded" << RTT::endlog();
		return false;
	}else if(ft_sensor_name.compare("placeholder") == 0){
		RTT::log(RTT::Error) << "No sensor could be loaded" << RTT::endlog();
				return false;
	}
//	if(!ft_wrench_output_port.connected()){
//		return false;
//	}

	ft_wrench = rstrt::dynamics::Wrench();
	ft_wrench.forces.setZero();
	ft_wrench.torques.setZero();
	ft_wrench_output_port.setName(ft_sensor_name+"_output");
	ft_wrench_output_port.setDataSample(ft_wrench);
	ports()->addPort(ft_wrench_output_port).doc(
			"Output port for wrench reading from the force_torque sensor: "
					+ ft_sensor_name + ".");
//
//	// Get the joints
//	gazebo_joints_ = model->GetJoints();
//	model_links_ = model->GetLinks();
//
//	RTT::log(RTT::Info) << "Model name " << model->GetName() << RTT::endlog();
//	RTT::log(RTT::Info) << "Model has " << gazebo_joints_.size() << " joints"
//			<< RTT::endlog();
//	RTT::log(RTT::Info) << "Model has " << model_links_.size() << " links"
//			<< RTT::endlog();
//
//	hardcoded_chains chains;
//	std::map<std::string, std::vector<std::string>>::iterator it;
//	for (it = chains.map_chains_joints.begin();
//			it != chains.map_chains_joints.end(); it++)
//		kinematic_chains.insert(
//				std::pair<std::string, boost::shared_ptr<KinematicChain>>(
//						it->first,
//						boost::shared_ptr<KinematicChain>(
//								new KinematicChain(it->first, it->second,
//										*(this->ports()), model))));
//	RTT::log(RTT::Info) << "Kinematic Chains map created!" << RTT::endlog();
//
//	for (std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it =
//			kinematic_chains.begin(); it != kinematic_chains.end(); it++) {
//		if (!(it->second->initKinematicChain())) {
//			RTT::log(RTT::Warning) << "Problem Init Kinematic Chain"
//					<< it->second->getKinematicChainName() << RTT::endlog();
//			return false;
//		}
//	}
//	RTT::log(RTT::Info) << "Kinematic Chains Initialized!" << RTT::endlog();
//
//	RTT::log(RTT::Warning) << "Done configuring component" << RTT::endlog();

//	gazebo::physics::JointPtr ft_link = model->GetJoint(ft_link_name);
//	ft_link->GetSensorName(0)

//RTT::log(RTT::Info)<<"HERE"<<gazebo::sensors::SensorManager::Instance()->GetSensors().at(0)->GetName()<<RTT::endlog();
//	gazebo::physics::JointPtr joint = model->GetJoint("lwr_arm_6_joint");
//	link = model->GetLink("lwr_arm_7_link");

	//	std::vector<gazebo::physics::JointPtr>::iterator it;
//	for(gazebo::physics::JointPtr temp:model->GetJoints()){
//		RTT::log(RTT::Info) << temp->GetName() << RTT::endlog();
//	}
//	if(!joint){
//		RTT::log(RTT::Info) << "Debug7" << RTT::endlog();
//	}
//	mgr =
//			gazebo::sensors::SensorManager::Instance();
//	RTT::log(RTT::Info) << "Debug1==" << RTT::endlog();
//	sdf::ElementPtr sdf(new sdf::Element);
//	sdf::initFile("sensor.sdf", sdf);
//	RTT::log(RTT::Info) << "Debug2" << RTT::endlog();
//	sdf::readString(forceTorqueSensorString, sdf);
//	RTT::log(RTT::Info) << "Debug3"<<joint->GetId() << RTT::endlog();
//	std::string sensorName = mgr->CreateSensor(sdf, "default", "lwr_arm_6_joint",
//			joint->GetId());
//	RTT::log(RTT::Info) << "Debug4" << RTT::endlog();
//	mgr->Update();
//	RTT::log(RTT::Info) << "Debug5"<< RTT::endlog();
	gazebo::sensors::SensorManager* sm = gazebo::sensors::SensorManager::Instance();
	for(gazebo::sensors::SensorPtr temp: sm->GetSensors()){
		RTT::log(RTT::Info) << "HERE"<<temp->GetName() << RTT::endlog();
	}
//	if(sm->GetSensors().at(0)==NULL){
		RTT::log(RTT::Info) << "HERE!!!!!!!!!"<<sm->GetSensor(ft_sensor_name)->GetName()<<RTT::endlog();
//	}
	ft_sensor = boost::dynamic_pointer_cast<gazebo::sensors::ForceTorqueSensor>(sm->GetSensor(ft_sensor_name));
//			boost::dynamic_pointer_cast<gazebo::sensors::ForceTorqueSensor>(//gazebo::default::kuka-lwr-4plus-ft::lwr_arm_6_joint::
//					sm->GetSensors().at(0));
	RTT::log(RTT::Info) << "HERE!!!!!!!!!33333333"<<RTT::endlog();
	if (ft_sensor == NULL) {
		RTT::log(RTT::Info) << "HERE2" << RTT::endlog();
		return false;
	}
	connection = ft_sensor->ConnectUpdate(boost::bind(&ftSim::gazeboUpdateHook, this, _1));
//	updateConnection2 = ft_sensor->ConnectUpdated(std::bind(&ftSim::gazeboUpdateHook2, this));
	RTT::log(RTT::Info) << "HERE!!!!!!!!!3322233"<<RTT::endlog();
//	mgr->Update();
//	ft_sensor->SetActive(true);
	RTT::log(RTT::Info) << "HERE3=_"<<ft_sensor->IsActive()<<ft_sensor->Force() << RTT::endlog();
	return true;
}

ORO_CREATE_COMPONENT_LIBRARY()
//ORO_CREATE_COMPONENT(cogimon::robotSim)
ORO_LIST_COMPONENT_TYPE(cogimon::ftSim)

