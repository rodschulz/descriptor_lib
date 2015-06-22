/**
 * Author: rodrigo
 * 2015
 */
#include "Test.h"
#include <stdio.h>

using namespace std;

Test::Test()
{
}

Test::~Test()
{
}

bool Test::Check()
{
	bool status = true;
	int testId = 0;

	Vector3f v1(1, 1, 0);
	Vector3f v2(1, 1, 1);
	Vector3f normal = v1.cross(v2).normalized();
	double angle12 = DEG2RAD(35.26438968);

	if (!(status = angleTest<Vector3f>(v1, v2, angle12)))
		cout << "Failed test " << testId++ << endl;

	if (!(status = angleSignedTest<Vector3f>(v1, v2, normal, angle12)))
		cout << "Failed test " << testId++ << endl;

	if (!(status = angleSignedTest<Vector3f>(v2, v1, normal, -angle12)))
		cout << "Failed test " << testId++ << endl;

	Vector3f v3(-1, -0.5, 0);
	normal = Vector3f(0, 0, 1);
	double angle13 = DEG2RAD(161.565051);

	if (!(status = angleTest<Vector3f>(v1, v3, angle13)))
		cout << "Failed test " << testId++ << endl;

	if (!(status = angleSignedTest<Vector3f>(v1, v3, normal, angle13)))
		cout << "Failed test " << testId++ << endl;

	if (!(status = angleSignedTest<Vector3f>(v3, v1, normal, -angle13)))
		cout << "Failed test " << testId++ << endl;

	Vector3f v4(1, 0.5, 0);
	double angle14 = DEG2RAD(18.4349487);

	if (!(status = angleTest<Vector3f>(v1, v4, angle14)))
		cout << "Failed test " << testId++ << endl;

	if (!(status = angleSignedTest<Vector3f>(v1, v4, normal, -angle14)))
		cout << "Failed test " << testId++ << endl;

	if (!(status = angleSignedTest<Vector3f>(v4, v1, normal, angle14)))
		cout << "Failed test " << testId++ << endl;

	Vector3f v5(-1, -1, 0);
	double angle15 = M_PI;

	if (!(status = angleSignedTest<Vector3f>(v1, v5, normal, angle15)))
		cout << "Failed test " << testId++ << endl;

	Vector3f v6(1, 1, 0);
	double angle16 = 0;

	if (!(status = angleSignedTest<Vector3f>(v1, v6, normal, angle16)))
		cout << "Failed test " << testId++ << endl;

	return status;
}
