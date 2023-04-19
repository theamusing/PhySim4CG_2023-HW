
#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"
// Construtors
RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	RigidBodyList = std::vector<RigidBody>();
	m_externalForce = Vec3(0.0f);
	g = Vec3(0.0f, 0.0f, 0.0f);
}
// Functions
const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "RigidBody,Collision,Complex";
}
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}
void RigidBodySystemSimulator::reset()
{
	RigidBodyList.clear();
	m_externalForce = Vec3(0.0f);
	g = Vec3(0.0f, 0.0f, 0.0f);
}
void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

	for each (RigidBody box in RigidBodyList)
	{
		DUC->drawRigidBody(box.WorldMat());
	}
}
void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	reset();
	m_testCase = testCase;
	switch (testCase)
	{
	case 0://single rigidbody
		addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		applyForceOnBody(0, Vec3(0.0, 0.0f, 0.0), Vec3(0, 0, 200));
		break;
	case 1://collision
		addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f);
		addRigidBody(Vec3(0.0f, 0.5f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0);
		setOrientationOf(1, Quat(Vec3(1.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
		setVelocityOf(1, Vec3(0.0f, -0.03f, 0.0f));
		break;
	case 2://complex
		addRigidBody(Vec3(0.0f, -1.0f, 0.0f), Vec3(2.0f, 0.1f, 2.0f), 10000000.0f, true);//floor
		addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.2f, 0.2f, 0.2f), 10.0f);
		addRigidBody(Vec3(0.1f, 0.2f, 0.1f), Vec3(0.2f, 0.2f, 0.2f), 10.0f);
		addRigidBody(Vec3(0.3f, -0.1f, 0.1f), Vec3(0.2f, 0.2f, 0.2f), 10.0f);
		addRigidBody(Vec3(-0.1f, 0.0f, -0.1f), Vec3(0.2f, 0.2f, 0.2f), 10.0f);
		setOrientationOf(1, Quat(Vec3(1.0f, 1.0f, 1.0f), (float)(M_PI) * 0.25f));
		setOrientationOf(2, Quat(Vec3(1.0f, 0.0f, 0.0f), (float)(M_PI) * 0.25f));
		setOrientationOf(3, Quat(Vec3(1.0f, 0.0f, -1.0f), (float)(M_PI) * 0.25f));
		setOrientationOf(4, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
		//setVelocityOf(1, Vec3(0.0f, -0.03f, 0.0f));
		//setVelocityOf(2, Vec3(0.01f, -0.05f, 0.0f));
		//setVelocityOf(3, Vec3(-0.01f, -0.03f, 0.0f));
		//setVelocityOf(4, Vec3(0.0f, -0.04f, 0.1f));
		g = Vec3(0.0f,-0.1f,0.0f);
		break;
	}
}
void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	switch (m_testCase)
	{
	case 0:
		Point2D mouseDiff;
		mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
		mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
		if (mouseDiff.x != 0 || mouseDiff.y != 0)
		{
			Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
			worldViewInv = worldViewInv.inverse();
			Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
			Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
			// find a proper scale!
			float inputScale = 0.1f;
			inputWorld = inputWorld * inputScale * timeElapsed;
			applyForceOnBody(0, Vec3(0, 0, 0), inputWorld);
		}
		break;
	default:
		break;
	}

}
void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	for (int i = 0; i < RigidBodyList.size(); i++)
	{
		if (RigidBodyList[i].fixed)
			continue;

		RigidBodyList[i].position += RigidBodyList[i].velocity * timeStep;
		Mat4d RotMat = RigidBodyList[i].orientation.getRotMat();
		Vec3 Torque = Vec3(0.0f);
		Mat4d RotMat_T = RotMat;
		RotMat_T.transpose();
		Mat4 inertia_cur = RotMat * RigidBodyList[i].inertia * RotMat_T;
		for each (auto force in RigidBodyList[i].external_forces)
		{
			RigidBodyList[i].velocity += force.force / RigidBodyList[i].mass * timeStep;			
			Torque += cross(RotMat.transformVector(force.loc - RigidBodyList[i].position), force.force);
		}
		RigidBodyList[i].velocity += g * timeStep;//apply gravity

		RigidBodyList[i].angular_velocity += inertia_cur.inverse().transformVector(Torque) * timeStep;
		Quat Rot_cur;
		Vec3 W_cur = RigidBodyList[i].angular_velocity * timeStep / 2;
		Rot_cur.w = 0;
		Rot_cur.x = W_cur.x;
		Rot_cur.y = W_cur.y;
		Rot_cur.z = W_cur.z;
		RigidBodyList[i].orientation += Rot_cur * RigidBodyList[i].orientation;
		RigidBodyList[i].orientation = RigidBodyList[i].orientation.unit();
		RigidBodyList[i].external_forces.clear();
	}
	/* calculate collision */

	for (int i = 0; i < RigidBodyList.size(); i++)
		for (int j = i + 1; j < RigidBodyList.size(); j++)
		{
			CollisionInfo info = checkCollisionSAT(RigidBodyList[i].WorldMat(), RigidBodyList[j].WorldMat());
			if (!info.isValid)
				continue;
			Vec3 v_rel = RigidBodyList[i].velocity - RigidBodyList[j].velocity;
			if (dot(v_rel, info.normalWorld) > 0)
				continue;
			float c = 1;
			float J = -(1 + c) * dot(v_rel, info.normalWorld) / 
				(1 / RigidBodyList[i].mass + 1 / RigidBodyList[j].mass + dot(
				cross(RigidBodyList[i].inertia.inverse().transformVector(cross(info.collisionPointWorld - RigidBodyList[i].position, info.normalWorld)), info.collisionPointWorld - RigidBodyList[i].position) +
				cross(RigidBodyList[j].inertia.inverse().transformVector(cross(info.collisionPointWorld - RigidBodyList[j].position, info.normalWorld)), info.collisionPointWorld - RigidBodyList[j].position),
				info.normalWorld));
			if (!RigidBodyList[i].fixed)
			{
				RigidBodyList[i].velocity += J * info.normalWorld / RigidBodyList[i].mass;
				RigidBodyList[i].angular_velocity += cross(info.collisionPointWorld - RigidBodyList[i].position, J * info.normalWorld);
			}
			if (!RigidBodyList[j].fixed)
			{
				RigidBodyList[j].velocity -= J * info.normalWorld / RigidBodyList[j].mass;
				RigidBodyList[j].angular_velocity -= cross(info.collisionPointWorld - RigidBodyList[j].position, J * info.normalWorld);
			}			
		}
}
void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return RigidBodyList.size();
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	if (i < RigidBodyList.size())
		return RigidBodyList[i].position;
	else
		return Vec3(0.f);
}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	if (i < RigidBodyList.size())
		return RigidBodyList[i].velocity;
	else
		return Vec3(0.f);
}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	if (i < RigidBodyList.size())
		return RigidBodyList[i].angular_velocity;
	else
		return Vec3(0.f);
}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	if (i < RigidBodyList.size())
		RigidBodyList[i].external_forces.push_back(ExternalForce(loc, force));
}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass, bool fixed)
{
	RigidBodyList.push_back(RigidBody(position, size, mass,fixed));
}
void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	if (i < RigidBodyList.size())
		RigidBodyList[i].orientation = orientation.unit();
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	if (i < RigidBodyList.size())
		RigidBodyList[i].velocity = velocity;
}


