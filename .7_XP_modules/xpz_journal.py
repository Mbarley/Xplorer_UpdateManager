# Z-stepper state journal for Power Loss Recovery (PLR).
#
# Problem: Z_TILT_ADJUST (and apply_zmotors_offsets) physically move the three
# Z steppers individually, but Klipper forgets that: ZAdjustHelper.adjust_steppers
# ends with toolhead.set_position(), which re-bases every Z stepper to the same
# logical position. Nothing records the physical result, and a resume must re-home
# Z (G28), which throws the tilt away. Re-running Z_TILT_ADJUST at resume time is
# not possible with a part on the bed.
#
# Solution: record, in order, every physical Z-stepper move made since Z was last
# homed (each Z_TILT_ADJUST pass and each FORCE_MOVE of a Z stepper), save that
# list at print start, and replay the same calls in the same order after the
# resume's G28. Replaying the same relative calls from the same homed state gives
# the same physical stepper positions.
#
# Commands:
#   SAVE_Z_JOURNAL                 save the journal to journal_file
#   REPLAY_Z_JOURNAL [FILE=name]   replay it (refuses if FILE does not match the
#                                  gcode file the journal was saved for)
#   Z_JOURNAL_STATUS               report what is currently recorded / saved
#
# Status (readable from macros as printer.xpz_journal):
#   events, tilt_passes, last_replay ('none' | 'ok' | 'missing' | 'mismatch' | 'failed')
#
# Config:
#   [xpz_journal]
#   journal_file: ~/printer_data/config/z_journal.json   (optional)
#
# This module deliberately imports nothing from Klipper so it can be imported
# (by the [xplorer] module loader) and unit tested outside of Klipper.

import json
import logging
import os
import time

JOURNAL_VERSION = 1

# The journal instance Klipper is currently using. The z_tilt patch below is
# applied to a class, which survives a FIRMWARE_RESTART inside the same process,
# so the patched function must look the live instance up here instead of holding
# a reference to the instance that existed when it was patched.
_active = None


def _patched_adjust_steppers(orig):
    def adjust_steppers(helper, adjustments, speed):
        orig(helper, adjustments, speed)
        # Record only after the move succeeded, and never let journaling break
        # the tilt itself.
        try:
            if _active is not None:
                _active.record_tilt(helper, adjustments, speed)
        except Exception:
            logging.exception("xpz_journal: failed to record Z adjustment")
    adjust_steppers._xpz_orig = orig
    return adjust_steppers


class XpZJournal:
    def __init__(self, config):
        global _active
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.journal_file = os.path.expanduser(
            config.get('journal_file', '~/printer_data/config/z_journal.json'))
        self.events = []
        self.replaying = False
        self.last_replay = 'none'
        self.z_names = set()
        self.z_tilt = None
        _active = self

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        # A Z homing, or the motors being switched off, starts a new history.
        self.printer.register_event_handler("homing:home_rails_end",
                                            self._handle_home_rails_end)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._handle_motor_off)

        self.gcode.register_command(
            "SAVE_Z_JOURNAL", self.cmd_SAVE_Z_JOURNAL,
            desc="Save the Z-stepper move journal for power loss recovery")
        self.gcode.register_command(
            "REPLAY_Z_JOURNAL", self.cmd_REPLAY_Z_JOURNAL,
            desc="Replay the saved Z-stepper move journal after homing")
        self.gcode.register_command(
            "Z_JOURNAL_STATUS", self.cmd_Z_JOURNAL_STATUS,
            desc="Report the recorded and saved Z-stepper move journal")

    # ------------------------------------------------------------------
    # Hooks
    # ------------------------------------------------------------------
    def _handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        self.z_names = set(s.get_name() for s in kin.get_steppers()
                           if s.is_active_axis('z'))

        # Z_TILT_ADJUST: patch ZAdjustHelper.adjust_steppers (class level, once).
        self.z_tilt = self.printer.lookup_object('z_tilt', None)
        if self.z_tilt is not None:
            helper_cls = type(self.z_tilt.z_helper)
            current = helper_cls.adjust_steppers
            if not hasattr(current, '_xpz_orig'):
                helper_cls.adjust_steppers = _patched_adjust_steppers(current)
        else:
            logging.info("xpz_journal: no [z_tilt] section, tilt passes will "
                         "not be recorded")

        # FORCE_MOVE (used by apply_zmotors_offsets): wrap manual_move on this
        # ForceMove instance. cmd_FORCE_MOVE calls self.manual_move, so an
        # instance attribute is picked up. The instance is recreated on restart.
        force_move = self.printer.lookup_object('force_move', None)
        if force_move is not None:
            orig_manual_move = force_move.manual_move

            def manual_move(stepper, dist, speed, accel=0.):
                orig_manual_move(stepper, dist, speed, accel)
                try:
                    self.record_move(stepper, dist, speed, accel)
                except Exception:
                    logging.exception("xpz_journal: failed to record move")
            force_move.manual_move = manual_move

    def _handle_home_rails_end(self, homing_state, rails):
        if 2 in homing_state.get_axes():
            self.events = []

    def _handle_motor_off(self, print_time=None):
        self.events = []

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------
    def record_tilt(self, helper, adjustments, speed):
        if self.replaying or self.z_tilt is None:
            return
        if helper is not self.z_tilt.z_helper:
            return
        self.events.append({'t': 'tilt',
                            'adj': [float(a) for a in adjustments],
                            'speed': float(speed)})

    def record_move(self, stepper, dist, speed, accel):
        if self.replaying:
            return
        name = stepper.get_name()
        if name not in self.z_names:
            return
        self.events.append({'t': 'move', 'stepper': name,
                            'dist': float(dist), 'speed': float(speed),
                            'accel': float(accel)})

    def _tilt_passes(self, events):
        return sum(1 for e in events if e.get('t') == 'tilt')

    def _current_file(self):
        vsd = self.printer.lookup_object('virtual_sdcard', None)
        path = vsd.file_path() if vsd is not None else None
        return os.path.basename(path) if path else None

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------
    def _write_journal(self, data):
        tmp = self.journal_file + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, self.journal_file)

    def _read_journal(self):
        with open(self.journal_file) as f:
            return json.load(f)

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def cmd_SAVE_Z_JOURNAL(self, gcmd):
        passes = self._tilt_passes(self.events)
        if not passes:
            # Not fatal: this runs inside START_PRINT. Remove any older journal
            # so a later resume cannot silently replay one from a different print.
            try:
                os.remove(self.journal_file)
            except OSError:
                pass
            gcmd.respond_info(
                "SAVE_Z_JOURNAL: no Z_TILT_ADJUST recorded since Z was last "
                "homed, nothing saved (old journal removed)")
            return
        data = {'version': JOURNAL_VERSION,
                'saved_at': time.time(),
                'file': self._current_file(),
                'events': self.events}
        self._write_journal(data)
        gcmd.respond_info("SAVE_Z_JOURNAL: saved %d events (%d Z_TILT_ADJUST "
                          "passes) for %s"
                          % (len(self.events), passes, data['file']))

    def cmd_REPLAY_Z_JOURNAL(self, gcmd):
        want_file = gcmd.get('FILE', None)
        self.last_replay = 'none'
        try:
            data = self._read_journal()
        except FileNotFoundError:
            self.last_replay = 'missing'
            gcmd.respond_info("REPLAY_Z_JOURNAL: no saved journal, skipping")
            return
        except (OSError, ValueError) as e:
            self.last_replay = 'failed'
            gcmd.respond_info("REPLAY_Z_JOURNAL: could not read journal (%s), "
                              "skipping" % (e,))
            return
        if data.get('version') != JOURNAL_VERSION:
            self.last_replay = 'failed'
            gcmd.respond_info("REPLAY_Z_JOURNAL: unsupported journal version "
                              "%r, skipping" % (data.get('version'),))
            return
        saved_file = data.get('file')
        if want_file and saved_file and \
                os.path.basename(want_file) != saved_file:
            self.last_replay = 'mismatch'
            gcmd.respond_info("REPLAY_Z_JOURNAL: journal is for %s, not %s, "
                              "skipping" % (saved_file, want_file))
            return

        toolhead = self.printer.lookup_object('toolhead')
        homed = toolhead.get_status(
            self.printer.get_reactor().monotonic())['homed_axes']
        if 'z' not in homed:
            raise gcmd.error("REPLAY_Z_JOURNAL: Z must be homed first")
        if self.z_tilt is None:
            raise gcmd.error("REPLAY_Z_JOURNAL: no [z_tilt] section")

        events = data.get('events', [])
        gcmd.respond_info("REPLAY_Z_JOURNAL: replaying %d events (%d Z_TILT_"
                          "ADJUST passes)" % (len(events),
                                              self._tilt_passes(events)))
        self.replaying = True
        try:
            for ev in events:
                if ev['t'] == 'tilt':
                    self.z_tilt.z_helper.adjust_steppers(ev['adj'], ev['speed'])
                elif ev['t'] == 'move':
                    self.gcode.run_script_from_command(
                        'FORCE_MOVE STEPPER="%s" DISTANCE=%.6f VELOCITY=%.6f '
                        'ACCEL=%.6f' % (ev['stepper'], ev['dist'], ev['speed'],
                                        ev['accel']))
                else:
                    raise gcmd.error("REPLAY_Z_JOURNAL: unknown event %r"
                                     % (ev,))
            toolhead.wait_moves()
        except Exception:
            # Stop here rather than resuming on a half-replayed Z state.
            self.last_replay = 'failed'
            raise
        finally:
            self.replaying = False
        self.last_replay = 'ok'
        gcmd.respond_info("REPLAY_Z_JOURNAL: done")

    def cmd_Z_JOURNAL_STATUS(self, gcmd):
        gcmd.respond_info("xpz_journal: %d events recorded since Z was last "
                          "homed (%d Z_TILT_ADJUST passes); last replay: %s"
                          % (len(self.events), self._tilt_passes(self.events),
                             self.last_replay))
        try:
            data = self._read_journal()
            gcmd.respond_info("saved journal: %d events for %s"
                              % (len(data.get('events', [])), data.get('file')))
        except (OSError, ValueError):
            gcmd.respond_info("saved journal: none")

    def get_status(self, eventtime):
        return {'events': len(self.events),
                'tilt_passes': self._tilt_passes(self.events),
                'last_replay': self.last_replay}


def load_config(config):
    return XpZJournal(config)
