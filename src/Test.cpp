/**
 * Author: rodrigo
 * 2015
 */
#include "Test.h"
#include <stdio.h>

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

	Eigen::Vector3f v1(1, 1, 0);
	Eigen::Vector3f v2(1, 1, 1);
	Eigen::Vector3f normal = v1.cross(v2).normalized();
	double angle12 = DEG2RAD(35.26438968);

	if (!(status = angleTest<Eigen::Vector3f>(v1, v2, angle12)))
		std::cout << "Failed test " << testId++ << std::endl;

	if (!(status = angleSignedTest<Eigen::Vector3f>(v1, v2, normal, angle12)))
		std::cout << "Failed test " << testId++ << std::endl;

	if (!(status = angleSignedTest<Eigen::Vector3f>(v2, v1, normal, -angle12)))
		std::cout << "Failed test " << testId++ << std::endl;

	Eigen::Vector3f v3(-1, -0.5, 0);
	normal = Eigen::Vector3f(0, 0, 1);
	double angle13 = DEG2RAD(161.565051);

	if (!(status = angleTest<Eigen::Vector3f>(v1, v3, angle13)))
		std::cout << "Failed test " << testId++ << std::endl;

	if (!(status = angleSignedTest<Eigen::Vector3f>(v1, v3, normal, angle13)))
		std::cout << "Failed test " << testId++ << std::endl;

	if (!(status = angleSignedTest<Eigen::Vector3f>(v3, v1, normal, -angle13)))
		std::cout << "Failed test " << testId++ << std::endl;

	Eigen::Vector3f v4(1, 0.5, 0);
	double angle14 = DEG2RAD(18.4349487);

	if (!(status = angleTest<Eigen::Vector3f>(v1, v4, angle14)))
		std::cout << "Failed test " << testId++ << std::endl;

	if (!(status = angleSignedTest<Eigen::Vector3f>(v1, v4, normal, -angle14)))
		std::cout << "Failed test " << testId++ << std::endl;

	if (!(status = angleSignedTest<Eigen::Vector3f>(v4, v1, normal, angle14)))
		std::cout << "Failed test " << testId++ << std::endl;

	Eigen::Vector3f v5(-1, -1, 0);
	double angle15 = M_PI;

	if (!(status = angleSignedTest<Eigen::Vector3f>(v1, v5, normal, angle15)))
		std::cout << "Failed test " << testId++ << std::endl;

	Eigen::Vector3f v6(1, 1, 0);
	double angle16 = 0;

	if (!(status = angleSignedTest<Eigen::Vector3f>(v1, v6, normal, angle16)))
		std::cout << "Failed test " << testId++ << std::endl;

	return status;
}
