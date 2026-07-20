from types import SimpleNamespace
from unittest.mock import MagicMock

from joystick.joystick_node import JoystickNode


def _node(calibration_mode=False):
    node = JoystickNode.__new__(JoystickNode)
    node.calibration_mode = calibration_mode
    node.manual_mode = False
    node.e_stop_latched = False
    node._prev_nudge_b_pressed = False
    node._prev_nudge_y_pressed = False
    node.nudge_pub = MagicMock()
    logger = MagicMock()
    node.get_logger = MagicMock(return_value=logger)
    return node, logger


def _buttons(b=False, y=False):
    return SimpleNamespace(button_b=b, button_y=y)


def test_b_and_y_publish_one_signed_event_per_press():
    """B should publish LEFT once and Y should publish RIGHT once."""
    node, _ = _node()

    node.update_nudge_from_buttons(_buttons(b=True))
    node.update_nudge_from_buttons(_buttons(b=True))
    node.update_nudge_from_buttons(_buttons())
    node.update_nudge_from_buttons(_buttons(y=True))

    directions = [call.args[0].data for call in node.nudge_pub.publish.call_args_list]
    assert directions == [1, -1]


def test_simultaneous_buttons_are_ignored():
    """A simultaneous B/Y edge should not choose an arbitrary direction."""
    node, logger = _node()

    node.update_nudge_from_buttons(_buttons(b=True, y=True))

    node.nudge_pub.publish.assert_not_called()
    logger.warning.assert_called_once()


def test_calibration_mode_suppresses_nudge():
    """Calibration should retain B/Y exclusively for steering trim."""
    node, _ = _node(calibration_mode=True)

    node.update_nudge_from_buttons(_buttons(b=True))

    node.nudge_pub.publish.assert_not_called()


def test_estop_suppresses_nudge():
    """An X-button stop should take priority over a simultaneous nudge."""
    node, _ = _node()
    buttons = SimpleNamespace(button_b=True, button_y=False, button_x=True)

    node.update_nudge_from_buttons(buttons)

    node.nudge_pub.publish.assert_not_called()
