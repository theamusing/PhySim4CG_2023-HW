#ifndef RIGIDBODY_h
#define RIGIDBODY_h
#include "Simulator.h"
struct ExternalForce
{
	Vec3 loc;
	Vec3 force;
	ExternalForce(Vec3 _loc, Vec3 _force)
	{
		loc = _loc;
		force = _force;
	}
};
struct RigidBody
{
	Vec3 position;
	Vec3 size;
	Quat orientation;
	float mass;
	Vec3 velocity;
	Vec3 angular_velocity;
	Mat4d inertia;
	bool fixed;
	std::vector<ExternalForce> external_forces;
	RigidBody(Vec3 _position, Vec3 _size, int _mass, bool _fixed = false)
	{
		position = _position;
		size = _size;
		mass = _mass;
		fixed = _fixed;
		orientation = Quat(0,0,0,1);
		velocity = Vec3(0.0);
		angular_velocity = Vec3(0.0);
		external_forces.clear();
		inertia.initScaling(size.y * size.y + size.z * size.z, size.x * size.x + size.z + size.z, size.x + size.x + size.y + size.y);
		inertia = inertia / 12.0f * mass;
	}
	Mat4 WorldMat()
	{
		Mat4 rotMat, scaleMat, translatMat;
		rotMat = orientation.getRotMat();
		scaleMat.initScaling(size.x, size.y, size.z);
		translatMat.initTranslation(position.x, position.y, position.z);
		return scaleMat * rotMat * translatMat;
	}
};

#endif
