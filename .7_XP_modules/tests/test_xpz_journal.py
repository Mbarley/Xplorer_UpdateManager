# Standalone test for xpz_journal.py using mock Klipper objects (no Klipper needed).
#   python3 .7_XP_modules/tests/test_xpz_journal.py
# NOT copied to klippy/extras: the [xplorer] loader only copies top-level .py files.
import importlib.util, json, os, re, sys, tempfile

sys.dont_write_bytecode = True  # do not leave a __pycache__ next to the module
HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "xpz_journal", os.path.join(HERE, "..", "xpz_journal.py"))
xpz = importlib.util.module_from_spec(spec); spec.loader.exec_module(xpz)

class GcodeError(Exception): pass
class Stepper:
    def __init__(s, name, z=True): s.name, s.z = name, z
    def get_name(s): return s.name
    def is_active_axis(s, a): return a == 'z' and s.z
class Kin:
    def __init__(s, steppers): s.steppers = steppers
    def get_steppers(s): return s.steppers
class Reactor:
    def monotonic(s): return 0.
class Toolhead:
    def __init__(s, kin): s.kin, s.homed = kin, "xyz"
    def get_kinematics(s): return s.kin
    def get_status(s, t): return {'homed_axes': s.homed}
    def wait_moves(s): pass
class World:  # shared physical world; survives "restarts" like the real machine
    def __init__(s): s.log = []
class ForceMove:
    def __init__(s, world): s.world = world; s.steppers = {}
    def manual_move(s, stepper, dist, speed, accel=0.):
        s.world.log.append(('move', stepper.get_name(), round(dist, 6)))
class ZHelper:
    world = None
    def adjust_steppers(self, adjustments, speed):
        ZHelper.world.log.append(('tilt', [round(a, 6) for a in adjustments]))
class ZTilt:
    def __init__(s): s.z_helper = ZHelper()
class Vsd:
    def __init__(s, p): s.p = p
    def file_path(s): return s.p
class Gcode:
    def __init__(s, printer): s.cmds = {}; s.printer = printer; s.out = []
    def register_command(s, name, fn, desc=None): s.cmds[name] = fn
    def run_script_from_command(s, script):
        m = re.match(r'FORCE_MOVE STEPPER="([^"]+)" DISTANCE=(\S+) VELOCITY=(\S+) ACCEL=(\S+)', script)
        assert m, script
        fm = s.printer.objs['force_move']
        fm.manual_move(Stepper(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)))
    error = GcodeError
class Gcmd:
    def __init__(s, gcode, **p): s.g, s.p = gcode, p
    def get(s, k, d=None): return s.p.get(k, d)
    def respond_info(s, m): s.g.out.append(m)
    def error(s, m): return GcodeError(m)
class Printer:
    def __init__(s, world, vsd_path):
        s.handlers = {}
        s.steppers = [Stepper("stepper z"), Stepper("stepper z1"), Stepper("stepper z2"), Stepper("stepper x", z=False)]
        s.objs = {'toolhead': Toolhead(Kin(s.steppers)), 'z_tilt': ZTilt(),
                  'force_move': ForceMove(world), 'virtual_sdcard': Vsd(vsd_path)}
        s.objs['gcode'] = Gcode(s)
    def get_reactor(s): return Reactor()
    def lookup_object(s, n, d=None): return s.objs.get(n, d)
    def register_event_handler(s, ev, fn): s.handlers.setdefault(ev, []).append(fn)
    def send(s, ev, *a):
        for fn in s.handlers.get(ev, []): fn(*a)
class Config:
    def __init__(s, printer, path): s.p, s.path = printer, path
    def get_printer(s): return s.p
    def get(s, k, d=None): return s.path if k == 'journal_file' else d
class Homing:
    def __init__(s, axes): s.axes = axes
    def get_axes(s): return s.axes

def boot(world, path, vsd):
    ZHelper.world = world
    p = Printer(world, vsd)
    j = xpz.load_config(Config(p, path))
    p.send('klippy:connect')
    return p, j

tmp = tempfile.mkdtemp(); path = os.path.join(tmp, "z_journal.json")
world = World()
p, j = boot(world, path, "/gcodes/tests/amp_plate.gcode")
fm, zt, g = p.objs['force_move'], p.objs['z_tilt'], p.objs['gcode']

# --- original session: home, apply_zmotors_offsets, 2 tilt passes ---
p.send('homing:home_rails_end', Homing([0, 1, 2]), [])
fm.manual_move(p.steppers[1], -0.468, 5., 0.)
fm.manual_move(p.steppers[2], 0.015, 5., 0.)
fm.manual_move(p.steppers[3], 9.9, 5., 0.)           # non-Z stepper must be ignored
zt.z_helper.adjust_steppers([-0.714854, -1.129914, -0.350488], 5.)
zt.z_helper.adjust_steppers([0.045486, 0.172667, -0.106007], 5.)
assert j.get_status(0) == {'events': 4, 'tilt_passes': 2, 'last_replay': 'none'}, j.get_status(0)
original = list(world.log)
g.cmds['SAVE_Z_JOURNAL'](Gcmd(g)); print(g.out[-1])
saved = json.load(open(path)); assert saved['file'] == 'amp_plate.gcode' and len(saved['events']) == 4

# --- simulated FIRMWARE_RESTART in the same process: new Printer/journal, class patch persists ---
world.log.clear()
p2, j2 = boot(world, path, None)
fm2, zt2, g2 = p2.objs['force_move'], p2.objs['z_tilt'], p2.objs['gcode']
p2.send('homing:home_rails_end', Homing([0, 1, 2]), [])
assert not hasattr(type(zt2.z_helper).adjust_steppers._xpz_orig, '_xpz_orig'), "double patched"
g2.cmds['REPLAY_Z_JOURNAL'](Gcmd(g2, FILE='amp_plate.gcode')); print(g2.out[-1])
assert world.log == [e for e in original if not (e[0] == 'move' and e[1] == 'stepper x')], world.log
assert j2.get_status(0) == {'events': 0, 'tilt_passes': 0, 'last_replay': 'ok'}, j2.get_status(0)  # replay not re-recorded
# ...and the OLD instance is no longer recording
zt.z_helper.adjust_steppers([0, 0, 0], 5.); assert j.get_status(0)['events'] == 4

# --- guards ---
g2.out.clear(); world.log.clear()
g2.cmds['REPLAY_Z_JOURNAL'](Gcmd(g2, FILE='other_part.gcode')); assert j2.last_replay == 'mismatch' and not world.log, g2.out
os.remove(path)
g2.cmds['REPLAY_Z_JOURNAL'](Gcmd(g2, FILE='x.gcode')); assert j2.last_replay == 'missing' and not world.log
open(path, 'w').write('{garbage'); g2.cmds['REPLAY_Z_JOURNAL'](Gcmd(g2)); assert j2.last_replay == 'failed'
# Z not homed -> hard error, nothing moved
json.dump({'version': 1, 'file': 'a.gcode', 'events': [{'t': 'tilt', 'adj': [0,0,0], 'speed': 5.}]}, open(path, 'w'))
p2.objs['toolhead'].homed = "xy"
try: g2.cmds['REPLAY_Z_JOURNAL'](Gcmd(g2)); raise SystemExit("expected error")
except GcodeError as e: print("unhomed ->", e)
p2.objs['toolhead'].homed = "xyz"
# SAVE with no tilt: non-fatal, removes stale journal
p2.send('stepper_enable:motor_off', 0.); assert j2.get_status(0)['events'] == 0
g2.cmds['SAVE_Z_JOURNAL'](Gcmd(g2)); assert not os.path.exists(path); print(g2.out[-1])
# partial failure stops and flags
class Boom(Exception): pass
json.dump({'version': 1, 'file': None, 'events': [{'t': 'move', 'stepper': 'stepper z1', 'dist': 1., 'speed': 5., 'accel': 0.},
                                                  {'t': 'bogus'}]}, open(path, 'w'))
try: g2.cmds['REPLAY_Z_JOURNAL'](Gcmd(g2)); raise SystemExit("expected error")
except GcodeError: pass
assert j2.last_replay == 'failed' and not j2.replaying
print("ALL OK")
