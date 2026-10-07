// Standalone runner for an existing lockstep SITL build (no simulator needed).
#include <gtest/gtest.h>
#include <uORB/uORBManager.hpp>
#include <platforms/posix/apps.h>

void init_app_map(apps_map_type &) {}
void list_builtins(apps_map_type &) {}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	uORB::Manager::initialize();
	return RUN_ALL_TESTS();
}
