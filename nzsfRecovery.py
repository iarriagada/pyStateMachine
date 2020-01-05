#!/usr/bin/env python3.5

import os
import argparse
import epics
import time
import h5py
import re
import numpy as np
import collections

from datetime import datetime, timedelta
from StateMachineLib import StateMachine, State

ERROR_TIME = 1.5*60
Recs = {}
inputs = {}
outputs = {}

# Conditions for transitions of the State Machine
# Az and EL non zero speed fault
inputs['nzsAz'] = epics.PV('gis:az:azns:aznssums.VAL')
inputs['nzsEl'] = epics.PV('gis:alt:altns:altnssums.VAL')
# Voltage set-point for Az and El motors
inputs['voltAz'] = epics.PV('gis:mon:azmon3:azdspdspc.VAL')
inputs['voltEl'] = epics.PV('gis:mon:altmon3:altdspdspc.VAL')
# MCS follow mode state
inputs['mcsFollow'] = epics.PV('mc:FollowL')
# Az and El drives assert state
inputs['azDriveCond'] = epics.PV('mc:azDriveCondition')
inputs['elDriveCond'] = epics.PV('mc:elDriveCondition')
inputs['azPosErr'] = epics.PV('mc:azPosError')
inputs['elPosErr'] = epics.PV('mc:elPosError')
inputs['prevState'] = ''

# Actions taken by each state of the State Machine
# TCS Follow directive
outputs['tcsMCSFollow'] = epics.PV('tcs:mcFollow.A')
# TCS Apply directives
outputs['tcsApply'] = epics.PV('tcs:apply.DIR')
# F1 Reset
outputs['f1Reset'] = epics.PV('gis:tsrs:gisReset.PROC')
# MCS E-stop for motors
outputs['eStop'] = epics.PV('mc:azEstop.PROC')
# Disable MCS ability to go into Follow Mode
outputs['mcsTrackDis'] = epics.PV('mc:followTrackingOn.DISA')
# Assert/Disassert Az and El drives
outputs['azDriveEn'] = epics.PV('mc:azDriveEnable')
outputs['elDriveEn'] = epics.PV('mc:elDriveEnable')

Recs['Input'] = inputs
Recs['Output'] = outputs

states = \
    [['start', 'SS'],
     ['no_fault', 'ES'],
     'follow_off',
     'voltage_zero',
     'clear_nzsf',
     'fault_cleared',
     'az_disassert',
     'el_disassert',
     'disable_tracking',
     'az_assert',
     'enable_tracking',
     'el_assert',
     'follow_on',
     ['rec_success', 'ES'],
     ['rec_error', 'ES']]

def start_handler(recs):
    print('Initiating Non Zero Speed Fault Recovery')

start_trans = \
    [['no_fault', ['nzsAz','nzsEl'],
      lambda n,i: not(i[n[0]].value) and not(i[n[1]].value),
      False, ''],
     ['follow_off', ['mcsFollow'],
      lambda n,i: i[n[0]].value,
      False, ''],
     ['voltage_zero', ['voltAz','voltEl'],
      lambda n,i: ((abs(i[n[0]].value) > 0.5)
                   or (abs(i[n[1]]).value > 0.5)),
      False, ''],
     ['clear_nzsf', [''],
      lambda n,i: True,
      False, '']]

def no_fault_handler(recs):
    print('Non Zero Speed Fault not present, ending sequence')

no_fault_trans = []

def follow_off_handler(recs):
    inp = recs['Input']
    out = recs['Output']
    out['tcsMCSFollow'].put('Off')
    out['tcsApply'].put(3)

follow_off_trans = \
    [['rec_error', ['mcsFollow'],
      lambda n,i: i[n[0]].value,
      True, 'Error: Could not disable MCS Tracking'],
     ['follow_on', ['prevState'],
      lambda n,i: i[n[0]] == 'follow_on',
      False, ''],
     ['voltage_zero', ['voltAz','voltEl'],
      lambda n,i: ((abs(i[n[0]].value) > 0.5)
                   or (abs(i[n[1]]).value > 0.5)),
      False, ''],
     ['clear_nzsf', [''],
      lambda n,i: True,
      False, '']]

def voltage_zero_handler(recs):
    inp = recs['Input']
    out = recs['Output']
    out['f1Reset'].put(1)
    # out['eStop'].put(1)
    time.sleep(0.2)
    # out['eStop'].put(0)

voltage_zero_trans = \
    [['rec_error', ['voltAz','voltEl'],
      lambda n,i: ((abs(i[n[0]].value) > 0.1)
                   or (abs(i[n[1]]).value > 0.1)),
      True, 'Error: Unable to zero reference voltage'],
     ['clear_nzsf', [''],
      lambda n,i: True,
      False, '']]

def clear_nzsf_handler(recs):
    inp = recs['Input']
    out = recs['Output']
    out['f1Reset'].put(1)

clear_nzsf_trans = \
    [['rec_error', ['nzsAz','nzsEl'],
      lambda n,i: i[n[0]].value or i[n[1]].value,
      True, 'Error: Unable to clear Non Zero Speed Fault from GIS'],
     ['fault_cleared', [''],
      lambda n,i: True,
      False, '']]

def fault_cleared_handler(recs):
    pass

fault_cleared_trans = \
    [['az_disassert', ['azDriveCond'],
      lambda n,i: i[n[0]].value == 2,
      False, ''],
     ['el_disassert', ['elDriveCond'],
      lambda n,i: i[n[0]].value == 2,
      False, ''],
     ['disable_tracking', [''],
      lambda n,i: True,
      False, '']]

def az_disassert_handler(recs):
    inp = recs['Input']
    out = recs['Output']
    out['azDriveEn'].put(1)

az_disassert_trans = \
    [['rec_error', ['azDriveCond'],
      lambda n,i: i[n[0]].value != 1,
      True, 'Error: Azimuth Drive did not disassert'],
     ['el_disassert', [''],
      lambda n,i: True,
      False, '']]

def el_disassert_handler(recs):
    inp = recs['Input']
    out = recs['Output']
    out['elDriveEn'].put(1)

el_disassert_trans = \
    [['rec_error', ['elDriveCond'],
      lambda n,i: i[n[0]].value != 1,
      True, 'Error: Elevation Drive did not disassert'],
     ['disable_tracking', [''],
      lambda n,i: True,
      False, '']]

def disable_tracking_handler(recs):
    out = recs['Output']
    out['mcsTrackDis'].put(1)

disable_tracking_trans = \
    [['az_assert', [''],
      lambda n,i: True,
      False, '']]

def az_assert_handler(recs):
    inp = recs['Input']
    out = recs['Output']
    out['azDriveEn'].put(2)

az_assert_trans = \
    [['rec_error', ['azDriveCond'],
      lambda n,i: i[n[0]].value != 2,
      True, 'Error: Azimuth Drive did not assert'],
     ['enable_tracking', [''],
      lambda n,i: True,
      False, '']]

def enable_tracking_handler(recs):
    out = recs['Output']
    out['mcsTrackDis'].put(0)

enable_tracking_trans = \
    [['el_assert', [''],
      lambda n,i: True,
      False, '']]

def el_assert_handler(recs):
    inp = recs['Input']
    out = recs['Output']
    out['elDriveEn'].put(2)

el_assert_trans = \
    [['rec_error', ['elDriveCond'],
      lambda n,i: i[n[0]].value != 2,
      True, 'Error: Elevation Drive did not assert'],
     ['follow_on', [''],
      lambda n,i: True,
      False, '']]

def follow_on_handler(recs):
    inp = recs['Input']
    out = recs['Output']
    out['tcsMCSFollow'].put('On')
    out['tcsApply'].put(3)

follow_on_trans = \
    [['rec_error', ['mcsFollow'],
      lambda n,i: not(i[n[0]].value),
      True, 'Error: Could not disable MCS Tracking'],
     ['rec_success', ['voltAz','azPosErr','voltEl','elPosErr'],
      lambda n,i: ((((abs(i[n[0]].value) > 0.1) and (abs(i[n[2]]).value > 0.01))
                    or ((abs(i[n[0]].value) > 0.1) and (abs(i[n[2]]).value > 0.01)))
                   or ((abs(i[n[2]]).value < 0.01) and (abs(i[n[3]].value) < 0.01))),
      False, ''],
     ['rec_error', ['prevState'],
      lambda n,i: i[n[0]].value == 'follow_off',
      True, 'MCS Follow enabled but telescope not tracking'],
     ['follow_off', [''],
      lambda n,i: True,
      False, 'Resetting MCS Follow mode']]

def rec_success_handler(recs):
    print('Non Zero Speed Fault recovery successful')

rec_success_trans = []

def rec_error_handler(recs):
    out = recs['Output']
    out['mcsTrackDis'].put(0)
    print('Non Zero Speed Fault recovery ended in error')

rec_error_trans = []

if __name__ == '__main__':
    nzsfSM = StateMachine()
    state_array = []
    for sn in states:
        es = False
        ss = False
        n = sn
        if 'SS' in sn:
            ss = True
            n = sn[0]
        if 'ES' in sn:
            es = True
            n = sn[0]
        s = State(n,eval(n+'_handler'),eval(n+'_trans'),ss,es)
        print('State {} ready'.format(s.name))
        s.init_transitions()
        # state_array.append(s)
        nzsfSM.add_state(s)
    nzsfSM.run(Recs)






