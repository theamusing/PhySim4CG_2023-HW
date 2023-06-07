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
	return "SoftBody";
}
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT , &m_fStiffness,"min=5000 max=500000");
}
void MassSpringSystemSimulator::reset()
{
	pointList.clear();
	springList.clear();
	test(); // build test case
}
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	//for each (auto s in springList)
	//{
	//	Vec3 pos1 = pointList[s.point1].position;
	//	Vec3 pos2 = pointList[s.point2].position;
	//	Vec3 color1 = Vec3(1, 0, 0);
	//	Vec3 color2 = Vec3(0, 1, 0);
	//	DUC->beginLine();
	//	DUC->drawLine(pos1, color1, pos2, color2);
	//	DUC->endLine();
	//}
	for each (auto v in volumeList)
	{
		Vec3 color1 = Vec3(1, 0, 0);
		Vec3 color2 = Vec3(0, 1, 0);
		Vec3 p1 = pointList[v.point1].position;
		Vec3 p2 = pointList[v.point2].position;
		Vec3 p3 = pointList[v.point3].position;
		Vec3 p4 = pointList[v.point4].position;
		DUC->DrawTriangle(p4, p2, p1);
		DUC->DrawTriangle(p4, p3, p2);
		DUC->DrawTriangle(p4, p1, p3);
		DUC->DrawTriangle(p1, p2, p3);
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
	vector<Vec3> p;
	for (int i = 0; i < pointList.size(); i++)
	{
		pointList[i].velocity += timeStep * pointList[i].force / pointList[i].mass;
		p.push_back(pointList[i].position);
		pointList[i].position += timeStep * pointList[i].velocity;
	}
	for (int i = 0; i < springList.size(); i++)// distance construction
	{
		Spring s = springList[i];
		Vec3 p1 = pointList[s.point1].position;
		Vec3 p2 = pointList[s.point2].position;
		float w1 = 1 / pointList[s.point1].mass;
		float w2 = 1 / pointList[s.point2].mass;
		s.currentLength = norm(p1 - p2);
		float C = s.currentLength - s.initialLength;
		Vec3 C1 = -(p2 - p1) / s.currentLength;
		Vec3 C2 = -(p1 - p2) / s.currentLength;
		float alpha = 1 / m_fStiffness;
		float lambda = -C / (w1 * dot(C1, C1) + w2 * dot(C2, C2) + alpha / (timeStep * timeStep));
		pointList[s.point1].position += lambda * w1 * C1;
		pointList[s.point2].position += lambda * w2 * C2;
	}
	for (int i = 0; i < volumeList.size(); i++)// volume construction
	{
		Volume Vo = volumeList[i];
		Vec3 p1 = pointList[Vo.point1].position;
		Vec3 p2 = pointList[Vo.point2].position;
		Vec3 p3 = pointList[Vo.point3].position;
		Vec3 p4 = pointList[Vo.point4].position;
		float w1 = 1 / pointList[Vo.point1].mass;
		float w2 = 1 / pointList[Vo.point2].mass;
		float w3 = 1 / pointList[Vo.point3].mass;
		float w4 = 1 / pointList[Vo.point4].mass;
		float V = 1.0f / 6.0f * dot(cross((p2 - p1), (p3 - p1)), p4 - p1);
		float C = 6 * (V - volumeList[i].V);
		Vec3 C1 = cross(p4 - p2, p3 - p2);
		Vec3 C2 = cross(p3 - p1, p4 - p1);
		Vec3 C3 = cross(p4 - p1, p2 - p1);
		Vec3 C4 = cross(p2 - p1, p3 - p1);
		float alpha = 1 / m_fStiffness;
		float lambda = -C / (w1 * dot(C1, C1) + w2 * dot(C2, C2) + w3 * dot(C3, C3) + w4 * dot(C4, C4) + alpha / (timeStep * timeStep));
		pointList[Vo.point1].position += lambda * w1 * C1;
		pointList[Vo.point2].position += lambda * w2 * C2;
		pointList[Vo.point3].position += lambda * w3 * C3;
		pointList[Vo.point4].position += lambda * w4 * C4;
	}
	for (int i = 0; i < pointList.size(); i++)
	{
		pointList[i].velocity = (pointList[i].position - p[i]) / timeStep;
	}
	for (int i = 0; i < pointList.size(); i++)
	{
		if (pointList[i].position.y < -1.0)// hit ground
		{
			pointList[i].position.y = -1.0;
			pointList[i].velocity.y = abs(pointList[i].velocity.y);
		}
	}
	//switch (m_iIntegrator)
	//{
	//case EULER:
	//	simulateTimestep_Euler(timeStep);
	//	break;
	//case MIDPOINT:
	//	simulateTimestep_midpoint(timeStep);
	//	break;
	//default:
	//	simulateTimestep_Euler(timeStep);
	//	break;
	//}
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
void MassSpringSystemSimulator::addVolume(int masspoint1, int masspoint2, int masspoint3, int masspoint4)
{
	Volume v = Volume();
	v.point1 = masspoint1;
	v.point2 = masspoint2;
	v.point3 = masspoint3;
	v.point4 = masspoint4;
	Vec3 p1 = pointList[masspoint1].position;
	Vec3 p2 = pointList[masspoint2].position;
	Vec3 p3 = pointList[masspoint3].position;
	Vec3 p4 = pointList[masspoint4].position;
	float Vrest = 1.0f / 6.0f * dot(cross((p2 - p1), (p3 - p1)),p4 - p1);
	v.V = Vrest;
	volumeList.push_back(v);
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
	setMass(100);
	setStiffness(5000);
	setDampingFactor(0);
	applyExternalForce(Vec3(0, -1, 0));

	//addMassPoint(Vec3(0.2, 0, 0.2), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(0.2, 0, -0.2), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(-0.2, 0, 0.2), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(-0.2, 0, -0.2), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(0.2, 0.4, 0.2), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(0.2, 0.4, -0.2), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(-0.2, 0.4, 0.2), Vec3(0, 0, 0), false);
	//addMassPoint(Vec3(-0.2, 0.4, -0.2), Vec3(0, 0, 0), false);
	//addSpring(0, 1, 0.4);
	//addSpring(0, 2, 0.4);
	//addSpring(0, 4, 0.4);
	//addSpring(1, 2, 0.565);
	//addSpring(1, 4, 0.565);
	//addSpring(4, 2, 0.565);
	//addSpring(2, 3, 0.4);
	//addSpring(2, 6, 0.4);
	//addSpring(3, 7, 0.4);
	//addSpring(3, 1, 0.4);
	//addSpring(5, 1, 0.4);
	//addSpring(5, 7, 0.4);
	//addSpring(5, 4, 0.4);
	//addSpring(7, 6, 0.4);
	//addSpring(4, 6, 0.4);

	//addVolume(1, 7, 2, 3);
	//addVolume(1, 7, 6, 2);
	//addVolume(1, 6, 7, 5);
	//addVolume(1, 2, 4, 0);
	//addVolume(1, 4, 2, 5);
	//addVolume(2, 4, 6, 5);

	//addVolume(1, 2, 4, 0);	
	//addVolume(0, 5, 3, 1);
	//addVolume(0, 3, 6, 2);
	//addVolume(1, 7, 2, 3);
	//addVolume(0, 6, 5, 4);
	//addVolume(1, 4, 7, 5);
	//addVolume(2, 7, 4, 6);
	//addVolume(3, 5, 6, 7);

	int num = 6;
	for (int i = 0; i < num; i++)
	{
		float angle = 2 * M_PI / num;
		addMassPoint(Vec3(0.4 * sin(angle * i), 0.4 * cos(angle * i), -0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.2 * sin(angle * i), 0.2 * cos(angle * i), -0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.4 * sin(angle * i), 0.4 * cos(angle * i), 0.1), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.2 * sin(angle * i), 0.2 * cos(angle * i), 0.1), Vec3(0, 0, 0), false);
	}
	for (int i = 0; i < num; i++)
	{
		float angle = 2 * M_PI / num;
		int p0 = 4 * ((i + 1) % num);
		int p1 = 4 * ((i + 1) % num) + 1;
		int p2 = 4 * i;
		int p3 = 4 * i + 1;
		int p4 = 4 * ((i + 1) % num) + 2;
		int p5 = 4 * ((i + 1) % num) + 3;
		int p6 = 4 * i + 2;
		int p7 = 4 * i + 3;
		addSpring(p0, p1, 0.2);
		addSpring(p1, p3, 0.2);
		addSpring(p1, p5, 0.2);
		addSpring(p5, p7, 0.2);
		addSpring(p5, p4, 0.2);
		addSpring(p0, p4, 0.2);
		addSpring(p0, p2, 0.4);
		addSpring(p6, p4, 0.4);
		addVolume(p1, p7, p2, p3);
		addVolume(p1, p7, p6, p2);
		addVolume(p1, p6, p7, p5);
		addVolume(p1, p2, p4, p0);
		addVolume(p1, p4, p2, p5);
		addVolume(p2, p4, p6, p5);
	}


	//int p1 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	//int p2 = addMassPoint(Vec3(0, 1, 0), Vec3(0, 0, 0), false);
	//addSpring(p1, p2, 1);

	
	/*int num = 6;
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
			if(length <= 0.1f)
				addSpring(i, j, length);
		}
			
	}*/

}