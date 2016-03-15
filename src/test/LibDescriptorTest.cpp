/**
 * Author: rodrigo
 * 2016
 */
#include <boost/test/unit_test.hpp>
#include "../descriptor/Calculator.hpp"
#include "../descriptor/Extractor.hpp"

/**************************************************/
BOOST_AUTO_TEST_SUITE(Calculator_class_suite)

BOOST_AUTO_TEST_CASE(getSequenceChar)
{
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(1, 5), '0');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(-1, 5), '0');

	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(7, 5), 'A');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(12, 5), 'B');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(19, 5), 'C');

	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(-7, 5), 'a');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(-12, 5), 'b');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(-19, 5), 'c');
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(Extractor_class_suite)

BOOST_AUTO_TEST_CASE(getLongitudinalBands)
{
}

BOOST_AUTO_TEST_SUITE_END()
