#include <zephyr/ztest.h>
#include "state_machine.h"

ZTEST(state_machine, test_initial_state)
{
	DoodleBotState sm;
	zassert_equal(sm.currentStateEnum(), State::Idle, "Initial state should be Idle");
	zassert_equal(sm.lastCommand(), Command::DeviceSleep, "Initial command should be DeviceSleep");
}

ZTEST(state_machine, test_erase_transition)
{
	DoodleBotState sm;
	sm.disbatch(Event::EraseAcknowledge);
	zassert_equal(sm.currentStateEnum(), State::Erasing, "Should transition to Erasing on EraseAcknowledge");
	zassert_equal(sm.lastCommand(), Command::StartErase, "Should issue StartErase command");
}

ZTEST(state_machine, test_erase_to_idle_transition)
{
	DoodleBotState sm;
	sm.disbatch(Event::EraseAcknowledge); // to Erasing
	sm.disbatch(Event::TransformationDone); // back to Idle
	zassert_equal(sm.currentStateEnum(), State::Idle, "Should return to Idle after TransformationDone");
	zassert_equal(sm.lastCommand(), Command::DeviceSleep, "Should issue DeviceSleep command");
}

ZTEST_SUITE(state_machine, NULL, NULL, NULL, NULL, NULL);