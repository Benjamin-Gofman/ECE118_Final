"""Run all starting headings with the same C navigation and bridge.py."""
import contextlib
import csv
import io
import os
import sys
import time
from pathlib import Path

from bridge import get_remote_client, main

CUSTOM_OFFSET = 0.40
STEP_DEGREES = 10
POSITIONS = {
    1: (-CUSTOM_OFFSET, +CUSTOM_OFFSET),
    2: (-CUSTOM_OFFSET, -CUSTOM_OFFSET),
    3: (+CUSTOM_OFFSET, +CUSTOM_OFFSET),
    4: (+CUSTOM_OFFSET, -CUSTOM_OFFSET),
}


def ask_location():
    print('Locations (top view):')
    print('  +---------+')
    print('  | 1  | 3  |')
    print('  |----+----|')
    print('  | 2  | 4  |')
    print('  +---------+')
    try:
        location = int(input('Enter location (1-4): ').strip())
    except (EOFError, ValueError):
        location = 1
    return location if location in POSITIONS else 1


class AngleLog(io.TextIOBase):
    """Print state transitions and tape sensor values; retain full logs for failures."""
    def __init__(self, output):
        self.output = output
        self.pending = ''
        self.lines = []

    def writable(self):
        return True

    def write(self, value):
        self.pending += value
        while '\n' in self.pending:
            line, self.pending = self.pending.split('\n', 1)
            self.lines.append(line)
            if ('C STATE:' in line or 'sensors FL FM FR RL RM RR TOP=' in line or
                    'DONE - STOP MOTORS' in line or
                    line.startswith('START:') or line.startswith('PAUSED') or
                    line.startswith('RESUMED')):
                self.output.write(line + '\n')
                self.output.flush()
        return len(value)

    def flush(self):
        self.output.flush()


def wait_until_stopped(sim, timeout=15):
    deadline = time.monotonic() + timeout
    while sim.getSimulationState() != sim.simulation_stopped:
        if time.monotonic() > deadline:
            raise RuntimeError('CoppeliaSim has not stopped; press Stop in the simulator')
        time.sleep(0.1)


def main_test():
    location = ask_location()
    x, y = POSITIONS[location]
    only = os.environ.get('ECE118_ONLY_ANGLES', '').strip()
    angles = ([int(value.strip()) for value in only.split(',') if value.strip()]
              if only else list(range(0, 360, STEP_DEGREES)))
    if not angles or any(angle < 0 or angle >= 360 for angle in angles):
        raise ValueError('Angles must be in the range 0..359')

    sim = get_remote_client().require('sim')
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError('Press Stop in CoppeliaSim before starting the angle sweep')
    if not (Path(__file__).parent / 'sim_robot.dll').is_file():
        raise RuntimeError('Missing sim_robot.dll; use test_all_angles.bat')

    print(f'\nTesting location {location} at x={x:+.2f}, y={y:+.2f}')
    print(f'Angles: {angles}; maximum simulation time per run: 120 s')
    print('Live output: C STATE changes and FL FM FR RL RM RR TOP sensors. Full log shown on failure.\n')

    csv_path = Path(__file__).resolve().parent / 'all_angles_results.csv'
    results = []
    try:
        for i, angle in enumerate(angles, 1):
            wait_until_stopped(sim)
            print(f'[{i:02d}/{len(angles)}] location={location} angle={angle:3d} degrees', flush=True)
            log = AngleLog(sys.stdout)
            started = time.monotonic()
            try:
                with contextlib.redirect_stdout(log):
                    data = main(start_pose=(x, y, angle), verbose=True)
                status = 'PASS' if data['done'] else 'TIME_LIMIT'
                final_state = data['state']
                sim_time = f"{data['time']:.2f}"
                error = ''
            except KeyboardInterrupt:
                print('Interrupted by user.')
                break
            except Exception as exc:
                status = 'ERROR'
                final_state = 'UNKNOWN'
                sim_time = ''
                error = f'{type(exc).__name__}: {exc}'
                print('  ERROR:', error, flush=True)
                try:
                    sim.stopSimulation()
                except Exception:
                    pass
            results.append([location, x, y, angle, status, final_state, sim_time, error])
            print(f'  {status}: state={final_state}, sim_time={sim_time}s, '
                  f'real_time={time.monotonic()-started:.1f}s', flush=True)
            if status != 'PASS':
                print('  --- Last 40 log lines ---')
                for line in log.lines[-40:]:
                    print('  ' + line)
                print('  --- End log ---')
            if status == 'ERROR':
                print('Stopping sweep due to setup/runtime error.')
                break
            print()
    finally:
        with csv_path.open('w', newline='', encoding='utf-8') as out:
            writer = csv.writer(out)
            writer.writerow(['location', 'start_x', 'start_y', 'angle_degrees',
                             'result', 'final_state', 'sim_time_s', 'error'])
            writer.writerows(results)
        print(f'Results saved: {csv_path}')
        print('Summary:', ', '.join(f'{state}={sum(r[4] == state for r in results)}'
                                  for state in ('PASS', 'TIME_LIMIT', 'ERROR')))
        print('PASS means C navigation reached DONE; it does not verify real ball hits.')


if __name__ == '__main__':
    main_test()
