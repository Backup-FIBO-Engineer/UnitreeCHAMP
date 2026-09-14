from xbox_one_s_teleop.sdl_devices import choose_joystick_id, parse_joy_enumerate

_SAMPLE = """
ID : GUID                             : GamePad : Mapped : Joystick Device Name
-------------------------------------------------------------------------------
 0 : 030000007e0500000920000011000000 :   false :  false : RustDesk UInput Keyboard
 1 : 050000005e040000e002000003090000 :    true :   true : Xbox Wireless Controller
"""


def test_parse_joy_enumerate_skips_header():
    devices = parse_joy_enumerate(_SAMPLE)
    assert devices == [
        (0, 'RustDesk UInput Keyboard'),
        (1, 'Xbox Wireless Controller'),
    ]


def test_auto_prefers_xbox_over_rustdesk():
    devices = parse_joy_enumerate(_SAMPLE)
    assert choose_joystick_id(devices, 'auto') == 1


def test_explicit_id_is_honored():
    devices = parse_joy_enumerate(_SAMPLE)
    assert choose_joystick_id(devices, '0') == 0
    assert choose_joystick_id(devices, '1') == 1


def test_auto_skips_virtual_when_no_xbox_name():
    devices = [
        (0, 'RustDesk UInput Keyboard'),
        (1, 'Logitech Gamepad F710'),
    ]
    assert choose_joystick_id(devices, 'auto') == 1


def test_auto_picks_xpadneo_xbox_360_name():
    devices = [
        (0, 'Xbox 360 Controller'),
        (1, 'Xbox 360 Controller'),
        (2, 'RustDesk UInput Keyboard'),
    ]
    assert choose_joystick_id(devices, 'auto') == 0


def test_auto_only_rustdesk_is_none():
    devices = [(0, 'RustDesk UInput Keyboard')]
    assert choose_joystick_id(devices, 'auto') is None


def test_auto_empty_list_is_none():
    assert choose_joystick_id([], 'auto') is None
