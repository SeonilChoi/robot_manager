"""
Tests for FSM scheduler: transition table and FsmScheduler.

Run with: pytest tests/test_scheduler.py -v
"""
import unittest

from robot_manager.scheduler.fsm_scheduler import (
    Action,
    State,
    FsmScheduler,
    FsmAction,
    get_next_state,
)


class TestGetNextState(unittest.TestCase):
    """Tests for get_next_state transition table (State x Action -> State)."""

    def test_stopped_transitions(self):
        self.assertEqual(get_next_state(State.STOPPED, Action.STOP), State.STOPPED)
        self.assertEqual(get_next_state(State.STOPPED, Action.MOVE), State.OPERATING)
        self.assertEqual(get_next_state(State.STOPPED, Action.HOME), State.HOMING)

    def test_operating_transitions(self):
        self.assertEqual(get_next_state(State.OPERATING, Action.STOP), State.STOPPED)
        self.assertEqual(get_next_state(State.OPERATING, Action.MOVE), State.OPERATING)
        self.assertEqual(get_next_state(State.OPERATING, Action.HOME), State.INVALID)

    def test_homing_transitions(self):
        self.assertEqual(get_next_state(State.HOMING, Action.STOP), State.STOPPED)
        self.assertEqual(get_next_state(State.HOMING, Action.MOVE), State.INVALID)
        self.assertEqual(get_next_state(State.HOMING, Action.HOME), State.HOMING)


class TestFsmScheduler(unittest.TestCase):
    """Tests for FsmScheduler: initial state, step, tick, reset, invalid transition."""

    def setUp(self):
        self.dt = 0.1
        self.scheduler = FsmScheduler(self.dt)

    def test_initial_state_is_stopped(self):
        self.assertEqual(self.scheduler._state, State.STOPPED)

    def test_step_increments_time(self):
        self.scheduler.step()
        self.assertAlmostEqual(self.scheduler._t, self.dt)
        self.scheduler.step()
        self.assertAlmostEqual(self.scheduler._t, 2 * self.dt)

    def test_tick_stopped_to_operating_on_move(self):
        changed, fsm_state = self.scheduler.tick(FsmAction(Action.MOVE.value, duration=1.0))
        self.assertTrue(changed)
        self.assertEqual(fsm_state.state, State.OPERATING.value)
        self.assertEqual(self.scheduler._state, State.OPERATING)

    def test_tick_operating_to_stopped_on_stop(self):
        self.scheduler.tick(FsmAction(Action.MOVE.value, duration=1.0))
        changed, fsm_state = self.scheduler.tick(FsmAction(Action.STOP.value, duration=0.0))
        self.assertTrue(changed)
        self.assertEqual(fsm_state.state, State.STOPPED.value)
        self.assertEqual(self.scheduler._state, State.STOPPED)

    def test_tick_invalid_transition_raises(self):
        self.scheduler.tick(FsmAction(Action.MOVE.value, duration=1.0))
        with self.assertRaises(ValueError):
            self.scheduler.tick(FsmAction(Action.HOME.value, duration=1.0))

    def test_reset_sets_stopped_and_zero_time(self):
        self.scheduler._state = State.OPERATING
        self.scheduler._t = 1.0
        self.scheduler.reset()
        self.assertEqual(self.scheduler._state, State.STOPPED)
        self.assertEqual(self.scheduler._t, 0.0)

    def test_tick_home_until_progress_one_then_state_stopped(self):
        duration = 1.0
        self.scheduler.tick(FsmAction(Action.HOME.value, duration=duration))
        self.assertEqual(self.scheduler._state, State.HOMING)
        for _ in range(20):
            if self.scheduler._state == State.STOPPED:
                break
            self.scheduler.tick(FsmAction(Action.HOME.value, duration=duration))
            self.scheduler.step()
        self.assertEqual(self.scheduler._state, State.STOPPED)


if __name__ == "__main__":
    unittest.main()
