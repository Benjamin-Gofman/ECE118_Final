"""Transport-only bridge between CoppeliaSim and the original ECE118 C navigation.

Reads six downward-facing vision sensors, one banner-facing sensor, and contact force.
Calls the compiled C controller with seven digital inputs and applies C motor PWM.
No navigation logic or movement sequence is implemented in Python.
"""
from __future__ import annotations

import argparse
import ctypes
import math
import sys
import time
from pathlib import Path


# Assign a consistent terminal color to each navigation state.
STATE_COLORS = {
    'FIND_TAPE': '36',          # Cyan
    'FOLLOW_TAPE': '32',        # Green
    'BUMPED': '91',             # Bright red
    'SHOOT_AT_TOWER': '35',     # Magenta
    'FIND_OPPONENT': '34',      # Blue
    'OPP_SEARCH': '94',         # Bright blue
    'OPP_APPROACH': '92',       # Bright green
    'OPP_BACKOFF': '93',        # Bright yellow
    'OPP_SPINUP': '95',         # Bright magenta
    'OPP_SHOOT': '96',          # Bright cyan
    'OPP_DONE': '37',           # White
    'OPP_FAILED': '91',         # Bright red
    'APPROACH_OPPONENT': '92',  # Bright green
    'COLLISION_BACKOFF': '93',  # Bright yellow
    'SHOOT_AT_OPPONENT': '96',  # Bright cyan
    'DONE': '37',               # White
    'BACK_UP': '93',            # Bright yellow
    'GO_RIGHT': '94',           # Bright blue
    'GO_FORWARD': '92',         # Bright green
    'GO_LEFT': '95',            # Bright magenta
    'PROBE_FORWARD': '96',      # Bright cyan
    'DETOUR_ALIGN': '94',       # Bright magenta
    'DETOUR_STRAIGHT': '92',    # Bright green
    'DETOUR_EXIT_RIGHT': '94',
    'CHECK_CONTACT': '96',
    'SCAN_RIGHT': '96',
    'RETURN_HEADING': '96',
    'BARRIER_BACKTRACK': '93',
    'BARRIER_REVERSE': '93',  # Bright blue
}


def enable_state_colors():
    """Enable ANSI colors in Windows consoles when supported."""
    if not sys.stdout.isatty():
        return False
    if sys.platform != 'win32':
        return True
    try:
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
        mode = ctypes.c_ulong()
        if not kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            return False
        return bool(kernel32.SetConsoleMode(handle, mode.value | 0x0004))
    except (AttributeError, OSError):
        return False


def colored_state(state):
    """Color only the state name; leave the rest of the log unchanged."""
    color = STATE_COLORS.get(state)
    if not STATE_COLOR_ENABLED or color is None:
        return state
    return f'\x1b[{color}m{state}\x1b[0m'


STATE_COLOR_ENABLED = False


def get_remote_client():
    """Load the CoppeliaSim ZeroMQ API client."""
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    except ImportError:
        from zmqRemoteApi import RemoteAPIClient
    return RemoteAPIClient()


def get_binary_sensor(sim, sensor, threshold, detected_when_dark):
    """Convert a vision sensor's average RGB brightness into one digital input.

    Black tape is expected to be darker than the white floor.
    The banner uses the opposite polarity by default (bright patch = 1).
    These are *camera approximations*, not physically accurate IR reflectance.
    """
    image, resolution = sim.getVisionSensorImg(sensor)
    if not image:
        raise RuntimeError('Vision sensor returned no image; check scene/sensor settings')
    # Sampling RGB data directly avoids adding NumPy/Pillow dependencies.
    # Bytes are RGB888: three values per pixel.
    if len(image) != int(resolution[0]) * int(resolution[1]) * 3:
        raise RuntimeError('Unexpected vision sensor image length or resolution')
    mean_brightness = sum(image) / len(image)
    detected = mean_brightness < threshold if detected_when_dark else mean_brightness > threshold
    return int(detected), mean_brightness



def get_white_banner_sensor(sim, sensor, rgb_min, min_fraction):
    """Detect a white banner by RGB color instead of average brightness.

    A pixel counts as white only when its red, green, AND blue channels are
    above rgb_min. This avoids detecting bright red/green/blue tower faces
    as white. Detection requires a minimum fraction of white image pixels.

    The camera must be aimed at the banner: a white wall or other white
    object in its view could also trigger detection.
    """
    image, resolution = sim.getVisionSensorImg(sensor)
    if not image:
        raise RuntimeError('Banner vision sensor returned no image')

    pixel_count = int(resolution[0]) * int(resolution[1])
    if len(image) != pixel_count * 3:
        raise RuntimeError('Unexpected banner image length or resolution')

    white_count = 0
    for index in range(0, len(image), 3):
        r, g, b = image[index], image[index + 1], image[index + 2]
        if r >= rgb_min and g >= rgb_min and b >= rgb_min:
            white_count += 1

    white_fraction = white_count / pixel_count
    mean_brightness = sum(image) / len(image)
    detected = int(white_fraction >= min_fraction)
    return detected, mean_brightness, white_fraction



def optional_proximity_sensor(sim, path):
    """Look up an optional opponent or bumper proximity sensor."""
    try:
        return sim.getObject(path)
    except Exception:
        return None


def bumper_pressed(sim, handle, threshold):
    """Convert physical bumper force into the digital contact input for C."""
    result, force, _torque = sim.readForceSensor(handle)
    if result <= 0 or not force:
        return 0
    return int(math.sqrt(sum(component * component for component in force)) >= threshold)


def proximity_detected(sim, handle):
    """Forward ray counts ONLY an actual Opponent scene object, not a wall.

    This is sensor identification/transport, not navigation logic. C decides
    between two shots and obstacle avoidance from its digital input.
    """
    result = sim.readProximitySensor(handle)
    if not isinstance(result, (tuple, list)) or not result or not result[0]:
        return 0
    # CoppeliaSim returns the detected object handle in slot 3.
    if len(result) < 4:
        return 0
    obj = result[3]
    if obj is None or obj < 0:
        return 0
    while obj >= 0:
        alias = sim.getObjectAlias(obj, -1)  # Naked alias; option 1 returns an object path
        if alias == 'Opponent':
            return 1
        obj = sim.getObjectParent(obj)
    return 0


def space_pressed():
    """Read Space without blocking the CoppeliaSim bridge (Windows console).

    Windows msvcrt reads console keystrokes immediately; no Enter is needed.
    Other platforms do not support this console shortcut in this version.
    """
    if sys.platform != 'win32':
        return False
    import msvcrt
    pressed = False
    while msvcrt.kbhit():
        key = msvcrt.getwch()
        if key in ('\x00', '\xe0'):
            # Ignore the second code of a function/arrow key.
            if msvcrt.kbhit():
                msvcrt.getwch()
        elif key == ' ':
            pressed = not pressed
    return pressed


def main(start_pose=None, verbose=True):
    global STATE_COLOR_ENABLED
    STATE_COLOR_ENABLED = enable_state_colors()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=120.0, help='Simulation duration')
    parser.add_argument('--rpm-full', type=float, default=100.0,
                        help='Wheel RPM corresponding to a PWM of 1000')
    parser.add_argument('--left-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--right-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--tape-threshold', type=float, default=110.0,
                        help='Mean RGB threshold for dark tape (calibrate for your scene)')
    parser.add_argument('--banner-white-min', type=int, default=180,
                        help='Minimum value in each RGB channel for white')
    parser.add_argument('--banner-min-fraction', type=float, default=0.05,
                        help='Minimum fraction of white pixels in Top camera')
    parser.add_argument('--bumper-force-threshold', type=float, default=1.0,
                        help='Force in newtons required to press either bumper')
    args = parser.parse_args([] if start_pose is not None else None)
    if not math.isfinite(args.bumper_force_threshold) or args.bumper_force_threshold <= 0:
        parser.error('--bumper-force-threshold must be a positive finite number')
    if not 0 <= args.banner_white_min <= 255:
        parser.error('--banner-white-min must be 0..255')
    if not 0 < args.banner_min_fraction <= 1:
        parser.error('--banner-min-fraction must be (0,1]')

    # Link the Python transport to the compiled, original C navigation.
    lib = Path(__file__).resolve().parent / (
        'sim_robot.dll' if sys.platform == 'win32' else 'libsim_robot.so'
    )
    if not lib.exists():
        raise SystemExit(f'Missing {lib.name}; run the build script first')
    dll = ctypes.CDLL(str(lib))
    dll.SimHost_Init.argtypes = []
    dll.SimHost_Init.restype = None
    dll.SimHost_SetSensors.argtypes = [ctypes.c_ubyte] * 7
    dll.SimHost_SetSensors.restype = None
    dll.SimHost_SetOpponentDetected.argtypes = [ctypes.c_ubyte]
    dll.SimHost_SetOpponentDetected.restype = None
    dll.SimHost_SetBumperPressed.argtypes = [ctypes.c_ubyte]
    dll.SimHost_SetBumperPressed.restype = None
    dll.SimHost_SetRearBumperPressed.argtypes = [ctypes.c_ubyte]
    dll.SimHost_SetRearBumperPressed.restype = None
    dll.SimHost_Step.argtypes = [ctypes.c_double]
    dll.SimHost_Step.restype = None
    dll.SimHost_GetMotors.argtypes = [ctypes.POINTER(ctypes.c_int)] * 2
    dll.SimHost_GetMotors.restype = None
    # The returned string is owned by the C library; ctypes decodes a copy.
    dll.SimHost_GetStateName.argtypes = []
    dll.SimHost_GetStateName.restype = ctypes.c_char_p

    client = get_remote_client()
    sim = client.require('sim')

    # Each scene path is declared explicitly for easy maintenance.
    # Motors
    LEFT_MOTOR = sim.getObject('/Robot/LeftMotor')
    RIGHT_MOTOR = sim.getObject('/Robot/RightMotor')

    # Six tape sensors: their ordering must match SimHost_SetSensors().
    FRONT_LEFT = sim.getObject('/Robot/TapeSensors/FrontLeft')
    FRONT_MIDDLE = sim.getObject('/Robot/TapeSensors/FrontMiddle')
    FRONT_RIGHT = sim.getObject('/Robot/TapeSensors/FrontRight')
    REAR_LEFT = sim.getObject('/Robot/TapeSensors/RearLeft')
    REAR_MIDDLE = sim.getObject('/Robot/TapeSensors/RearMiddle')
    REAR_RIGHT = sim.getObject('/Robot/TapeSensors/RearRight')

    # 'Top' is the actual banner vision sensor name in Robot.simscene.xml.
    BANNER = sim.getObject('/Robot/TapeSensors/Top')

    # Forward is a proximity sensor; the bumper is a physical Force Sensor.
    OPPONENT_DETECTOR = optional_proximity_sensor(sim, '/Robot/TapeSensors/Forward')
    FRONT_BUMPER = optional_proximity_sensor(sim, '/Robot/FrontBumperForce')
    REAR_BUMPER = optional_proximity_sensor(sim, '/Robot/RearBumperForce')
    if REAR_BUMPER is None:
        raise SystemExit('Missing /Robot/RearBumperForce. Open the updated scene.')
    if sim.getObjectType(REAR_BUMPER) != sim.object_forcesensor_type:
        raise SystemExit('Robot/RearBumperForce exists but is not a Force Sensor.')

    if OPPONENT_DETECTOR is not None and sim.getObjectType(OPPONENT_DETECTOR) != sim.object_proximitysensor_type:
        raise SystemExit('Robot/TapeSensors/Forward exists but is not a Proximity Sensor. '
                         'Change Forward to a Ray Proximity Sensor or provide a vision classifier.')
    if FRONT_BUMPER is not None and sim.getObjectType(FRONT_BUMPER) != sim.object_forcesensor_type:
        raise SystemExit('Robot/FrontBumperForce exists but is not a Force Sensor.')

    if sim.getSimulationState() != sim.simulation_stopped:
        raise SystemExit('Stop the scene simulation before starting bridge.py')

    print('Connected: LeftMotor, RightMotor, six tape sensors, Top (white RGB banner)')
    print('Forward:', 'connected' if OPPONENT_DETECTOR is not None else 'MISSING (OPP=N/A)')
    print('FrontBumperForce:', f'connected (threshold={args.bumper_force_threshold:g} N)'
          if FRONT_BUMPER is not None else 'MISSING (BUMPER=N/A)')
    print('RearBumperForce:', f'connected (threshold={args.bumper_force_threshold:g} N)')
    print('Controls: SPACE = pause/resume, Ctrl+C = exit (Windows terminal).')
    client.setStepping(True)
    left_pwm, right_pwm = ctypes.c_int(), ctypes.c_int()
    previous_log_bucket = -1
    previous_state = None
    previous_bumper = None
    previous_rear_bumper = None
    previous_opponent = None
    paused = False
    dll.SimHost_Init()
    if start_pose is not None:
        x, y, angle = start_pose
        robot = sim.getObject('/Robot')
        position = sim.getObjectPosition(robot, -1)
        sim.setObjectPosition(robot, -1, [float(x), float(y), position[2]])
        orientation = sim.getObjectOrientation(robot, -1)
        sim.setObjectOrientation(robot, -1, [orientation[0], orientation[1], math.radians(angle)])
        print(f'START: x={x:+.2f} y={y:+.2f} yaw={angle} degrees')
    result = {'done': False, 'state': 'UNKNOWN', 'time': 0.0}
    sim.startSimulation()
    try:
        while float(sim.getSimulationTime()) < args.seconds:
            # In stepping mode, CoppeliaSim advances ONLY when client.step()
            # is called. Skipping that call freezes physics, simulation time,
            # motor movement, and the C navigation clock without resetting PWM.
            if space_pressed():
                paused = not paused
                print('PAUSED - press SPACE to resume.' if paused else 'RESUMED.')
            if paused:
                time.sleep(0.04)  # Avoid busy-waiting while paused.
                continue

            now = float(sim.getSimulationTime())

            # Read all seven sensors BEFORE running the C navigation tick.
            # This order matches TapeSensor_ReadValues() in your C project.
            readings = [
                get_binary_sensor(sim, FRONT_LEFT, args.tape_threshold, True),
                get_binary_sensor(sim, FRONT_MIDDLE, args.tape_threshold, True),
                get_binary_sensor(sim, FRONT_RIGHT, args.tape_threshold, True),
                get_binary_sensor(sim, REAR_LEFT, args.tape_threshold, True),
                get_binary_sensor(sim, REAR_MIDDLE, args.tape_threshold, True),
                get_binary_sensor(sim, REAR_RIGHT, args.tape_threshold, True),
            ]
            banner_bit, banner_brightness, banner_white_fraction = get_white_banner_sensor(
                sim, BANNER, args.banner_white_min, args.banner_min_fraction)
            bits = [v for v, brightness in readings] + [banner_bit]
            dll.SimHost_SetSensors(*bits)
            # Forward indicates an Opponent object only; decisions stay in C.
            opp = proximity_detected(sim, OPPONENT_DETECTOR) if OPPONENT_DETECTOR is not None else 0
            bumper = bumper_pressed(sim, FRONT_BUMPER, args.bumper_force_threshold) if FRONT_BUMPER is not None else 0
            rear_bumper = bumper_pressed(sim, REAR_BUMPER, args.bumper_force_threshold)
            if previous_rear_bumper is not None and rear_bumper != previous_rear_bumper:
                print(f't={now:6.2f}s  REAR_BUMPER {previous_rear_bumper} -> {rear_bumper}', flush=True)
            previous_rear_bumper = rear_bumper
            # Print sensor transitions immediately, even between periodic log lines.
            if FRONT_BUMPER is not None and previous_bumper is not None and bumper != previous_bumper:
                print(f't={now:6.2f}s  BUMPER {previous_bumper} -> {bumper}', flush=True)
            if OPPONENT_DETECTOR is not None and previous_opponent is not None and opp != previous_opponent:
                print(f't={now:6.2f}s  OPP {previous_opponent} -> {opp}', flush=True)
            previous_bumper = bumper
            previous_opponent = opp
            dll.SimHost_SetOpponentDetected(opp)
            dll.SimHost_SetBumperPressed(bumper)
            dll.SimHost_SetRearBumperPressed(rear_bumper)
            dll.SimHost_Step(now)  # Executes Navigation.c (motor steering is handled by Linefollowing.c).
            dll.SimHost_GetMotors(ctypes.byref(left_pwm), ctypes.byref(right_pwm))
            # Observe C state after the navigation tick; no control decisions in Python.
            state = dll.SimHost_GetStateName().decode('ascii')
            result.update(state=state, time=now)
            if state != previous_state:
                print(f't={now:6.2f}s  STATE {colored_state(previous_state or "START")} -> {colored_state(state)}', flush=True)
                previous_state = state

            # PWM -> joint angular speed. This is transport mapping, NOT navigation.
            radians_per_sec_full = args.rpm_full * 2.0 * math.pi / 60.0
            left_speed = args.left_sign * (left_pwm.value / 1000.0) * radians_per_sec_full
            right_speed = args.right_sign * (right_pwm.value / 1000.0) * radians_per_sec_full
            sim.setJointTargetVelocity(LEFT_MOTOR, left_speed)
            sim.setJointTargetVelocity(RIGHT_MOTOR, right_speed)

            # Stop the game as soon as the C navigation reaches DONE.
            if state == 'DONE':
                result['done'] = True
                print(f't={now:6.2f}s  GAME OVER - DONE. Motors stopped; ending simulation.', flush=True)
                break

            # Print both raw brightness and digital readings for calibration.
            bucket = int(now * 2.0)
            if bucket != previous_log_bucket:
                avg = [round(b, 1) for bit, b in readings]
                print(f't={now:6.2f}s  sensors FL FM FR RL RM RR TOP={bits}'
                      f'  brightness={avg}'
                      f'  OPP={opp if OPPONENT_DETECTOR is not None else "N/A"}'
                      f'  BUMPER={bumper if FRONT_BUMPER is not None else "N/A"}'
                      f'  REAR_BUMPER={rear_bumper}'
                      f'  STATE={colored_state(state)}'
                      f'  C PWM L/R={left_pwm.value:+d}/{right_pwm.value:+d}')
                previous_log_bucket = bucket

            client.step()
    finally:
        # Always stop the motors, even if an API call or C step fails.
        try:
            sim.setJointTargetVelocity(LEFT_MOTOR, 0.0)
            sim.setJointTargetVelocity(RIGHT_MOTOR, 0.0)
            sim.stopSimulation()
        except Exception as error:
            print('Cleanup warning:', error)
    print('Bridge stopped.')
    return result


if __name__ == '__main__':
    main()
