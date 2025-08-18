#include "HapticsPlugin.h"

HapticsPlugin::HapticsPlugin(void)
{
	// initialize plugin
	InitializeHapticDevices();
	InitializeVariables();
}


HapticsPlugin::~HapticsPlugin(void)
{

}

void HapticsPlugin::InitializeHapticDevices(void)
{
	// create a haptic device handler
	handler = new cHapticDeviceHandler();
	// get number of haptic devices
	numHapticDevices = handler->getNumDevices();
	// setup each haptic device
	for (int i = 0; i<numHapticDevices; i++)
	{
		// get a handle to the first haptic device
		handler->getDevice(hapticDevice[i], i);
		// open a connection to haptic device
		hapticDevice[i]->open();
		// calibrate device (if necessary)
		hapticDevice[i]->calibrate();
		// retrieve information about the current haptic device
		info[i] = hapticDevice[i]->getSpecifications();
		// if the device has a gripper, enable the gripper to simulate a user switch
		hapticDevice[i]->setEnableGripperUserSwitch(true);
	}
}

void HapticsPlugin::InitializeVariables(void)
{
	for (int i = 0; i < numHapticDevices; i++)
	{
		// variables for haptic device   
		position[i] = cVector3d(0, 0 ,0);
		rotation[i] = cMatrix3d(1, 0, 0,
								0, 1, 0,
								0, 0, 1);
		gripperAngle[i] = 0.0;
		linearVelocity[i] = cVector3d(0, 0, 0);
		angularVelocity[i] = cVector3d(0, 0, 0);
		gripperAngularVelocity[i] = 0.0;
		button0[i] = false,
		button1[i] = false, 
		button2[i] = false, 
		button3[i] = false;

		// variables for force feedback
		force[i] = cVector3d(0, 0, 0);
		torque[i] = cVector3d(0, 0, 0);
		gripperForce[i] = 0.0;
	}
}

void HapticsPlugin::EndHapticDevices(void)
{
	// close haptic device
	for (int i = 0; i<numHapticDevices; i++)
	{
		hapticDevice[i]->close();
	}
	delete handler;
}

glm::mat3 HapticsPlugin::ConvertChaiMatToGLM(cMatrix3d chaiMat)
{
	return glm::mat3(chaiMat(0, 0), chaiMat(0, 1), chaiMat(0, 2),
		chaiMat(1, 0), chaiMat(1, 1), chaiMat(1, 2),
		chaiMat(2, 0), chaiMat(2, 1), chaiMat(2, 2));
}

int HapticsPlugin::GetHapticDevicesDetected(void)
{
	return numHapticDevices;
}

double HapticsPlugin::GetHapticDeviceInfo(int numHapDev, int var)
{
	double temp;
	switch (var) {
	case 1:
		temp = info[numHapDev].m_maxAngularTorque;
		break;
	case 2:
		temp = info[numHapDev].m_maxGripperForce;
		break;
	case 3:
		temp = info[numHapDev].m_maxLinearStiffness;
		break;
	case 4:
		temp = info[numHapDev].m_maxAngularStiffness;
		break;
	case 5:
		temp = info[numHapDev].m_maxGripperLinearStiffness;
		break;
	case 6:
		temp = info[numHapDev].m_maxLinearDamping;
		break;
	case 7:
		temp = info[numHapDev].m_maxAngularDamping;
		break;
	case 8:
		temp = info[numHapDev].m_maxGripperAngularDamping;
		break;
	default:
		temp = info[numHapDev].m_maxLinearForce;
		break;
	}

	return temp;
}

Vector3 HapticsPlugin::GetHapticPositions(int numHapDev)
{
	Vector3 temp;
	temp.x = (float)position[numHapDev].y();
	temp.y = (float)position[numHapDev].z();
	temp.z = (float)-position[numHapDev].x();

	return temp;
}

Quaternion HapticsPlugin::GetHapticOrientation(int numHapDev)
{
	glm::mat3 aux = ConvertChaiMatToGLM(rotation[numHapDev]);
	glm::mat3 rotateXY = glm::rotate(glm::radians(-90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	glm::mat3 rotateYZ = glm::rotate(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

	glm::mat3 rotateYX = glm::rotate(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	glm::mat3 rotateZY = glm::rotate(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

	glm::mat3 m_RotateChaiCoordinateSystemToOpenGLSystem = (rotateYZ * rotateXY);

	glm::mat3 finalOrientation = m_RotateChaiCoordinateSystemToOpenGLSystem * aux;

	Quaternion temp;
	temp.w = (float)(sqrt(1 + finalOrientation[0][0] + finalOrientation[1][1] + finalOrientation[2][2]) / 2);
	temp.x = (float)((finalOrientation[2][1] - finalOrientation[1][2]) / (4 * temp.w));
	temp.y = (float)((finalOrientation[0][2] - finalOrientation[2][0]) / (4 * temp.w));
	temp.z = (float)((finalOrientation[1][0] - finalOrientation[0][1]) / (4 * temp.w));
	return temp;
}

double HapticsPlugin::GetHapticGripperAngle(int numHapDev)
{
	return gripperAngle[numHapDev];
}

Vector3 HapticsPlugin::GetHapticLinearVelocity(int numHapDev)
{
	Vector3 temp;
	temp.x = (float)linearVelocity[numHapDev].y();
	temp.y = (float)linearVelocity[numHapDev].z();
	temp.z = (float)-linearVelocity[numHapDev].x();

	return temp;
}

Vector3 HapticsPlugin::GetHapticAngularVelocity(int numHapDev)
{
	Vector3 temp;
	temp.x = (float)angularVelocity[numHapDev].y();
	temp.y = (float)angularVelocity[numHapDev].z();
	temp.z = (float)-angularVelocity[numHapDev].x();
	
	return temp;
}

double HapticsPlugin::GetHapticGripperAngularVelocity(int numHapDev)
{
	return gripperAngularVelocity[numHapDev];
}

bool HapticsPlugin::GetHapticButton(int numHapDev, int button)
{
	bool temp;
	switch (button) {
	case 2:
		temp = button1[numHapDev];
		break;
	case 3:
		temp = button2[numHapDev];
		break;
	case 4:
		temp = button3[numHapDev];
		break;
	default:
		temp = button0[numHapDev];
		break;
	}

	return temp;
}

void HapticsPlugin::SetHapticForceFeedback(int numHapDev, cVector3d sentForce)
{
	force[numHapDev] = force[numHapDev] + sentForce;
}

void HapticsPlugin::SetHapticTorqueFeedback(int numHapDev, cVector3d sentTorque)
{
	torque[numHapDev] = torque[numHapDev] + sentTorque;
}

void HapticsPlugin::SetHapticGripperFeedback(int numHapDev, double sentGripperForce)
{
	gripperForce[numHapDev] = gripperForce[numHapDev] + sentGripperForce;
}

void HapticsPlugin::UpdateHaptics(int numHapDev)
{
	/////////////////////////////////////////////////////////////////////
	// READ HAPTIC DEVICE
	/////////////////////////////////////////////////////////////////////

	// read haptic device specifications
	info[numHapDev] = hapticDevice[numHapDev]->getSpecifications();
	// read position 
	hapticDevice[numHapDev]->getPosition(position[numHapDev]);
	// read orientation 
	hapticDevice[numHapDev]->getRotation(rotation[numHapDev]);
	// read gripper position
	hapticDevice[numHapDev]->getGripperAngleRad(gripperAngle[numHapDev]);
	// read linear velocity 
	hapticDevice[numHapDev]->getLinearVelocity(linearVelocity[numHapDev]);
	// read angular velocity
	hapticDevice[numHapDev]->getAngularVelocity(angularVelocity[numHapDev]);
	// read gripper angular velocity
	hapticDevice[numHapDev]->getGripperAngularVelocity(gripperAngularVelocity[numHapDev]);
	// read user-switch status
	hapticDevice[numHapDev]->getUserSwitch(0, button0[numHapDev]);
	hapticDevice[numHapDev]->getUserSwitch(1, button1[numHapDev]);
	hapticDevice[numHapDev]->getUserSwitch(2, button2[numHapDev]);
	hapticDevice[numHapDev]->getUserSwitch(3, button3[numHapDev]);

	/////////////////////////////////////////////////////////////////////
	// COMPUTE AND APPLY FORCES
	/////////////////////////////////////////////////////////////////////

	// send computed force, torque, and gripper force to haptic 
	hapticDevice[numHapDev]->setForceAndTorqueAndGripperForce(force[numHapDev], torque[numHapDev], gripperForce[numHapDev]);

	// clear forces
	force[numHapDev] = cVector3d(0, 0, 0);
	torque[numHapDev] = cVector3d(0, 0, 0);
	gripperForce[numHapDev] = 0;
}

/////////////////////////////////////////////////////////////////////
// Plugin Functions
/////////////////////////////////////////////////////////////////////

HAPTICS_PLUGIN HapticsPlugin* CreateHapticDevices(void) {
	return new HapticsPlugin();
}

HAPTICS_PLUGIN void DeleteHapticDevices(HapticsPlugin* hapticPlugin) {
	// close haptic device
	hapticPlugin->EndHapticDevices();

	// delete resource
	delete hapticPlugin;
}

HAPTICS_PLUGIN int GetHapticsDetected(HapticsPlugin* hapticPlugin) {
	return hapticPlugin->GetHapticDevicesDetected();
}

HAPTICS_PLUGIN double GetHapticsDeviceInfo(HapticsPlugin * hapticPlugin, int numHapDev, int var)
{
	return hapticPlugin->GetHapticDeviceInfo(numHapDev, var);
}

HAPTICS_PLUGIN Vector3 GetHapticsPositions(HapticsPlugin * hapticPlugin, int numHapDev)
{
	return hapticPlugin->GetHapticPositions(numHapDev);
}

HAPTICS_PLUGIN Quaternion GetHapticsOrientations(HapticsPlugin * hapticPlugin, int numHapDev)
{
	return hapticPlugin->GetHapticOrientation(numHapDev);
}

HAPTICS_PLUGIN double GetHapticsGripperAngle(HapticsPlugin * hapticPlugin, int numHapDev)
{
	return hapticPlugin->GetHapticGripperAngle(numHapDev);
}

HAPTICS_PLUGIN Vector3 GetHapticsLinearVelocity(HapticsPlugin * hapticPlugin, int numHapDev)
{
	return hapticPlugin->GetHapticLinearVelocity(numHapDev);
}

HAPTICS_PLUGIN Vector3 GetHapticsAngularVelocity(HapticsPlugin * hapticPlugin, int numHapDev)
{
	return hapticPlugin->GetHapticAngularVelocity(numHapDev);
}

HAPTICS_PLUGIN double GetHapticsGripperAngularVelocity(HapticsPlugin * hapticPlugin, int numHapDev)
{
	return hapticPlugin->GetHapticGripperAngularVelocity(numHapDev);
}

HAPTICS_PLUGIN double GetHapticsButtons(HapticsPlugin * hapticPlugin, int numHapDev, int button)
{
	return hapticPlugin->GetHapticButton(numHapDev, button);
}

HAPTICS_PLUGIN void SetHapticsForce(HapticsPlugin * hapticPlugin, int numHapDev, Vector3 sentForce)
{
	cVector3d tempF = cVector3d(-sentForce.z, sentForce.x, sentForce.y);

	hapticPlugin->SetHapticForceFeedback(numHapDev, tempF);
}

HAPTICS_PLUGIN void SetHapticsTorque(HapticsPlugin * hapticPlugin, int numHapDev, Vector3 sentTorque)
{
	cVector3d tempT = cVector3d(-sentTorque.z, sentTorque.x, sentTorque.y);

	hapticPlugin->SetHapticForceFeedback(numHapDev, tempT);
}

HAPTICS_PLUGIN void SetHapticsGripperForce(HapticsPlugin * hapticPlugin, int numHapDev, double sentGripperForce)
{
	hapticPlugin->SetHapticGripperFeedback(numHapDev, sentGripperForce);
}

HAPTICS_PLUGIN void UpdateHapticDevices(HapticsPlugin * hapticPlugin, int numHapDev)
{
	hapticPlugin->UpdateHaptics(numHapDev);
}