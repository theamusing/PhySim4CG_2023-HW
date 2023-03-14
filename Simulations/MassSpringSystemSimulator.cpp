#include "MassSpringSystemSimulator.h"

// Construtor
MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	springList = vector<Spring>();
	pointList = vector<Point>();
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
	m_iIntegrator = 0;
	m_externalForce = 0;
}

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "MassSpring";
}
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Method", TW_TYPE_UINT8 , &m_iIntegrator,"min=0 max=1");
}
void MassSpringSystemSimulator::reset()
{
	pointList.clear();
	springList.clear();
	test(); // build test case
}
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	for each (auto s in springList)
	{
		Vec3 pos1 = pointList[s.point1].position;
		Vec3 pos2 = pointList[s.point2].position;
		Vec3 color1 = Vec3(1, 0, 0);
		Vec3 color2 = Vec3(0, 1, 0);
		DUC->beginLine();
		DUC->drawLine(pos1, color1, pos2, color2);
		DUC->endLine();
	}
}
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	reset();
}
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{

}
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	for (int i = 0; i < pointList.size(); i++)
	{
		pointList[i].force = m_externalForce;
	}

	switch (m_iIntegrator)
	{
	case EULER:
		simulateTimestep_Euler(timeStep);
		break;
	case MIDPOINT:
		simulateTimestep_midpoint(timeStep);
		break;
	default:
		simulateTimestep_Euler(timeStep);
		break;
	}
}
void MassSpringSystemSimulator::simulateTimestep_Euler(float timeStep)
{
	for (int i = 0; i < springList.size(); i++)
	{
		Vec3 pos1 = pointList[springList[i].point1].position;
		Vec3 pos2 = pointList[springList[i].point2].position;
		springList[i].currentLength = norm(pos1 - pos2);
		Vec3 force = springList[i].stiffness * (springList[i].currentLength - springList[i].initialLength) * (pos2 - pos1) / springList[i].currentLength;
		pointList[springList[i].point1].force += force;
		pointList[springList[i].point2].force -= force;
	}

	for(int i=0;i<pointList.size();i++)
	{
		if (!pointList[i].fixed)
		{
			pointList[i].position += pointList[i].velocity * timeStep;
			pointList[i].velocity += pointList[i].force / pointList[i].mass * timeStep;
		}
		if (pointList[i].position.y < -1)
		{
			pointList[i].position.y = -1;
			pointList[i].velocity.y = -pointList[i].velocity.y;
		}
		cout << "id: " << i << ", position: " << pointList[i].position << ", velosity : " << pointList[i].velocity << endl;
	}
}
void MassSpringSystemSimulator::simulateTimestep_midpoint(float timeStep)
{
	vector<Point> midpointList = pointList;

	for (int i = 0; i < springList.size(); i++)
	{
		Vec3 pos1 = pointList[springList[i].point1].position;
		Vec3 pos2 = pointList[springList[i].point2].position;
		springList[i].currentLength = norm(pos1 - pos2);
		Vec3 force = springList[i].stiffness * (springList[i].currentLength - springList[i].initialLength) * (pos2 - pos1) / springList[i].currentLength;
		pointList[springList[i].point1].force += force;
		pointList[springList[i].point2].force -= force;
	}

	for (int i = 0; i < midpointList.size(); i++)
	{
		if (!midpointList[i].fixed)
		{
			midpointList[i].position = pointList[i].position + pointList[i].velocity * timeStep / 2;
			midpointList[i].velocity = pointList[i].velocity + pointList[i].force / pointList[i].mass * timeStep / 2;
		}
	}

	for (int i = 0; i < springList.size(); i++)
	{
		Vec3 pos1 = midpointList[springList[i].point1].position;
		Vec3 pos2 = midpointList[springList[i].point2].position;
		springList[i].currentLength = norm(pos1 - pos2);
		Vec3 force = springList[i].stiffness * (springList[i].currentLength - springList[i].initialLength) * (pos2 - pos1) / springList[i].currentLength;
		midpointList[springList[i].point1].force += force;
		midpointList[springList[i].point2].force -= force;
	}

	for (int i = 0; i < pointList.size(); i++)
	{
		if (!pointList[i].fixed)
		{
			pointList[i].position += midpointList[i].velocity * timeStep;
			pointList[i].velocity += midpointList[i].force / pointList[i].mass * timeStep;
		}
		if (pointList[i].position.y < -1)
		{
			pointList[i].position.y = -1;
			pointList[i].velocity.y = -pointList[i].velocity.y;
		}
	}
}
void MassSpringSystemSimulator::onClick(int x, int y)
{

}
void MassSpringSystemSimulator::onMouse(int x, int y)
{

}

// Specific Functions
void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
	for(int i=0;i<pointList.size();i++)
	{
		pointList[i].mass = mass;
	}
}
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
	for (int i = 0; i < springList.size(); i++)
	{
		springList[i].stiffness = stiffness;
	}
}
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
	for (int i = 0; i < springList.size(); i++)
	{
		springList[i].dampingFactor = damping;
	}
}
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	Point p = Point();
	p.position = position;
	p.velocity = Velocity;
	p.fixed = isFixed;
	p.mass = m_fMass;
	pointList.push_back(p);
	return pointList.size() - 1;
}
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring s = Spring();
	s.point1 = masspoint1;
	s.point2 = masspoint2;
	s.initialLength = initialLength;
	s.stiffness = m_fStiffness;
	s.dampingFactor = m_fDamping;
	springList.push_back(s);
}
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return pointList.size();
}
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springList.size();
}
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return pointList[index].position;
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return pointList[index].velocity;
}
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
}
 
// Test Functions
void MassSpringSystemSimulator::test()
{	
	setMass(1);
	setStiffness(40);
	setDampingFactor(0);
	applyExternalForce(Vec3(0, -3, 0));

	//int p1 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	//int p2 = addMassPoint(Vec3(0, 1, 0), Vec3(0, 0, 0), false);
	//addSpring(p1, p2, 1);

	
	int num = 4;
	addMassPoint(Vec3(0, 0.1, 0), Vec3(0, 0, 0), false);
	addMassPoint(Vec3(0, -0.1, 0), Vec3(0, 0, 0), false);
	for(int i = 0; i < num; i++)
		for (int j = 0; j < num/2 - 1; j++)
		{
			float angle = M_PI / 2 - 2 * M_PI / num * j - 2 * M_PI / num;
			int p = addMassPoint(0.1 * getNormalized(Vec3(cosf(2 * M_PI / num * i) * cosf(angle), sinf(angle), sinf(2 * M_PI / num * i) * cosf(angle))), Vec3(0, 0, 0), false);
		}
	for (int i = 0; i < num * (num / 2 - 1) + 2; i++)
	{
		for (int j = i + 1; j < num * (num / 2 - 1) + 2; j++)
		{
			float length = norm(pointList[i].position - pointList[j].position);
			if(length < 1.f)
				addSpring(i, j, length);
		}
			
	}

}