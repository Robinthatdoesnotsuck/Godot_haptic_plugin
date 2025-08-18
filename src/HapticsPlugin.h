#pragma once

#include <glm/glm.hpp> // glm::vec2, glm::mat4
#include <glm/gtc/matrix_transform.hpp>  // glm::rotate, glm::translate
#include <glm/gtx/transform.hpp> //
#include <glm/gtc/type_ptr.hpp> // glm::value_ptr
#include <vector>

// Chai3D Library
#include <math/CVector3d.h>
#include <math/CMatrix3d.h>
#include <devices/CGenericHapticDevice.h>
#include <devices/CPhantomDevices.h>
#include <devices/CHapticDeviceHandler.h>

#define HAPTICS_PLUGIN __attribute__((visibility("default"))) 
#define MAX_DEVICES 16

using namespace chai3d;

using namespace std;

struct Vector3
{
	float x;
	float y;
	float z;
};

struct Quaternion {
	float x;
	float y;
	float z;
	float w;
};

class HapticsPlugin
{
public:
	// constructor
	HapticsPlugin(void);
	// destructor
	~HapticsPlugin(void);

	// initialization of plugin
	void InitializeHapticDevices(void);
	void InitializeVariables(void);

	// end of plugin
	void EndHapticDevices(void);

	// auxiliary function
	glm::mat3 ConvertChaiMatToGLM(cMatrix3d);

	// get haptic device info
	int GetHapticDevicesDetected(void);
	double GetHapticDeviceInfo(int, int);
	Vector3 GetHapticPositions(int);
	Quaternion GetHapticOrientation(int);
	double GetHapticGripperAngle(int);
	Vector3 GetHapticLinearVelocity(int);
	Vector3 GetHapticAngularVelocity(int);
	double GetHapticGripperAngularVelocity(int);
	bool GetHapticButton(int, int);

	// set haptic device forces
	void SetHapticForceFeedback(int, cVector3d);
	void SetHapticTorqueFeedback(int, cVector3d);
	void SetHapticGripperFeedback(int, double);

	// main haptics rendering loop
	void UpdateHaptics(int);

private:
	// number of haptic devices detected
	int numHapticDevices = 0;
	// a haptic device handler
	cHapticDeviceHandler* handler;
	// a pointer to the current haptic device
	cGenericHapticDevicePtr hapticDevice[MAX_DEVICES];
	// information about the haptic devices detected
	cHapticDeviceInfo info[MAX_DEVICES];
	// haptic device variables
	cVector3d position[MAX_DEVICES];
	cMatrix3d rotation[MAX_DEVICES];
	double gripperAngle[MAX_DEVICES];
	cVector3d linearVelocity[MAX_DEVICES];
	cVector3d angularVelocity[MAX_DEVICES];
	double gripperAngularVelocity[MAX_DEVICES];
	bool button0[MAX_DEVICES];
	bool button1[MAX_DEVICES];
	bool button2[MAX_DEVICES];
	bool button3[MAX_DEVICES];
	// force feedback variables  
	cVector3d force[MAX_DEVICES];
	cVector3d torque[MAX_DEVICES];
	double gripperForce[MAX_DEVICES];
};

extern "C" {
    // constructor
	HAPTICS_PLUGIN HapticsPlugin* CreateHapticDevices(void);
	// destructor
	HAPTICS_PLUGIN void DeleteHapticDevices(HapticsPlugin* hapticPlugin);

	// get haptic device information
	HAPTICS_PLUGIN int GetHapticsDetected(HapticsPlugin* hapticPlugin);
	HAPTICS_PLUGIN double GetHapticsDeviceInfo(HapticsPlugin* hapticPlugin, int numHapDev, int var);
	HAPTICS_PLUGIN Vector3 GetHapticsPositions(HapticsPlugin* hapticPlugin, int numHapDev);
	HAPTICS_PLUGIN Quaternion GetHapticsOrientations(HapticsPlugin* hapticPlugin, int numHapDev);
	HAPTICS_PLUGIN double GetHapticsGripperAngle(HapticsPlugin* hapticPlugin, int numHapDev);
	HAPTICS_PLUGIN Vector3 GetHapticsLinearVelocity(HapticsPlugin* hapticPlugin, int numHapDev);
	HAPTICS_PLUGIN Vector3 GetHapticsAngularVelocity(HapticsPlugin* hapticPlugin, int numHapDev);
	HAPTICS_PLUGIN double GetHapticsGripperAngularVelocity(HapticsPlugin* hapticPlugin, int numHapDev);
	HAPTICS_PLUGIN double GetHapticsButtons(HapticsPlugin* hapticPlugin, int numHapDev, int button);

	// set haptic device forces
	HAPTICS_PLUGIN void SetHapticsForce(HapticsPlugin* hapticPlugin, int numHapDev, Vector3 sentForce);
	HAPTICS_PLUGIN void SetHapticsTorque(HapticsPlugin* hapticPlugin, int numHapDev, Vector3 sentTorque);
	HAPTICS_PLUGIN void SetHapticsGripperForce(HapticsPlugin* hapticPlugin, int numHapDev, double sentGripperForce);
	
	// main haptics rendering loop
	HAPTICS_PLUGIN void UpdateHapticDevices(HapticsPlugin* hapticPlugin, int numHapDev);
}