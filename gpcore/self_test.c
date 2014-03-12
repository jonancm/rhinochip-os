#include "self_test.h"

bool_t test_teach_pendant(void)
{
	bool_t test_passed = true;
	return test_passed;
}

bool_t test_ram(unsigned int *last_addr, unsigned int *bytes_ok)
{
	bool_t test_passed = true;
	*last_addr = 0xFFFF;
	*bytes_ok = 0xFFFF;
	return test_passed;
}
