#!/usr/bin/env python3.5

import os
import argparse
import epics
import time
import h5py
import re
import numpy as np

from datetime import datetime, timedelta

ERROR_TIME = 1.5*60
Recs = {}
conditions = {}
actions = {}

# Conditions for transitions of the State Machine
# Az and EL non zero speed fault
conditions['nzsAz'] = epics.PV('gis:az:azns:aznssums.VAL')
conditions['nzsEl'] = epics.PV('gis:alt:altns:altnssums.VAL')
# Voltage set-point for Az and El motors
conditions['voltAz'] = epics.PV('gis:mon:azmon3:azdspdspc.VAL')
conditions['voltEl'] = epics.PV('gis:mon:altmon3:altdspdspc.VAL')
# MCS follow mode state
conditions['mcsFollow'] = epics.PV('mc:FollowL')
# Az and El drives assert state
conditions['azDriveCond'] = epics.PV('mc:azDriveCondition')
conditions['elDriveCond'] = epics.PV('mc:elDriveCondition')
conditions['azPosErr'] = epics.PV('mc:azPosError')
conditions['elPosErr'] = epics.PV('mc:elPosError')
conditions['prevState'] = ''

# Actions taken by each state of the State Machine
# TCS Follow directive
actions['tcsMCSFollow'] = epics.PV('tcs:mcFollow.A')
# TCS Apply directives
actions['tcsApply'] = epics.PV('tcs:apply.DIR')
# F1 Reset
actions['f1Reset'] = epics.PV('gis:tsrs:gisReset.PROC')
# MCS E-stop for motors
actions['eStop'] = epics.PV('mc:azEstop.PROC')
# Disable MCS ability to go into Follow Mode
actions['mcsTrackDis'] = epics.PV('mc:followTrackingOn.DISA')
# Assert/Disassert Az and El drives
actions['azDriveEn'] = epics.PV('mc:azDriveEnable')
actions['elDriveEn'] = epics.PV('mc:elDriveEnable')

Recs['condition'] = conditions
Recs['action'] = actions

class InitializationError(Exception):
    def __init__(self, message):
        super().__init__()
        self.message = 'InitializationError: ' + message

class StateMachine:
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []
        self.prevState = ''

    def add_state(self, name, handler, end_state=False):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name.upper()

    def run(self, records):
        try:
            if not(self.startState):
                raise InitializationError('No Start State defined')
            if not(self.endStates):
                raise InitializationError('No End States defined')
        except InitializationError as err:
            print(err.message)
            exit(0)
        handler = self.handlers[self.startState]
        self.prevState = self.startState
        while True:
            (newState, records) = handler(records)
            records['condition']['prevState'] = self.prevState
            self.prevState = newState
            handler = self.handlers[newState.upper()]
            if newState.upper() in self.endStates:
                print("Recovery ended on {0}".format(newState.upper()))
                handler(records)
                break
            else:
                print("Recovery in {0} state".format(newState.upper()))

def start_transitions(recs):
    print('Initiating Non Zero Speed Fault Recovery')
    cond = recs['condition']
    if cond['nzsAz'].value or cond['nzsEl'].value:
        if cond['mcsFollow'].value:
            #Go to FOLLOW_OFF
            newState = 'follow_off'
        elif (abs(cond['voltAz'].value) > 0.5) or (abs(cond['voltEl'].value) > 0.5):
            #Go to VOLTAGE_ZERO
            newState = 'voltage_zero'
        else:
            newState = 'clear_nzsf'
    else:
        newState = 'no_fault'
    return (newState, recs)

def no_fault_actions(recs):
    print('Non Zero Speed Fault not present, ending sequence')

def follow_off_transitions(recs):
    waitTime = 0
    cond = recs['condition']
    actn = recs['action']
    actn['tcsMCSFollow'].put('Off')
    actn['tcsApply'].put(3)
    startTime = datetime.now()
    while True:
        if not(cond['mcsFollow'].value):
            break
        elif waitTime > ERROR_TIME:
            print('Error: Unable to disable MCS Tracking')
            newState = 'rec_error'
            return (newState, recs)
        waitTime = (datetime.now() - startTime).total_seconds()
    if cond['prevState'] == 'follow_on':
        newState = 'follow_on'
    elif (abs(cond['voltAz'].value) > 0.5) or (abs(cond['voltEl'].value) > 0.5):
        newState = 'voltage_zero'
    else:
        newState = 'clear_nzsf'
    return (newState, recs)

def voltage_zero_transitions(recs):
    waitTime = 0
    cond = recs['condition']
    actn = recs['action']
    actn['f1Reset'].put(1)
    # actn['eStop'].put(1)
    time.sleep(0.2)
    # actn['eStop'].put(0)
    startTime = datetime.now()
    while True:
        if (abs(cond['voltAz'].value) < 0.1) and (abs(cond['voltEl'].value) < 0.1):
            break
        elif waitTime > ERROR_TIME:
            print('Error: Unable to zero reference voltage')
            print('Az Volts: {0} - El Volts: {1}'.format(cond['voltAz'].value,
                                                        cond['voltEl'].value))
            newState = 'rec_error'
            return (newState, recs)
        waitTime = (datetime.now() - startTime).total_seconds()
    newState = 'clear_nzsf'
    time.sleep(1)
    return (newState, recs)

def clear_nzsf_transitions(recs):
    waitTime = 0
    cond = recs['condition']
    actn = recs['action']
    actn['f1Reset'].put(1)
    startTime = datetime.now()
    while True:
        if (not(cond['nzsAz'].value) and not(cond['nzsEl'].value)):
            break
        elif waitTime > ERROR_TIME:
            print('Error: Unable to clear Non Zero Speed Fault from GIS')
            newState = 'rec_error'
            return (newState, recs)
        waitTime = (datetime.now() - startTime).total_seconds()
    newState = 'fault_cleared'
    return (newState, recs)

def fault_cleared_transitions(recs):
    cond = recs['condition']
    if cond['azDriveCond'].value == 2:
        newState = 'az_disassert'
    elif cond['elDriveCond'].value == 2:
        newState = 'el_disassert'
    else:
        newState = 'disable_tracking'
    return (newState, recs)

def az_disassert_transitions(recs):
    waitTime = 0
    cond = recs['condition']
    actn = recs['action']
    actn['azDriveEn'].put(1)
    startTime = datetime.now()
    while True:
        if cond['azDriveCond'].value == 1:
            break
        elif waitTime > ERROR_TIME:
            print('Error: Azimuth Drive did not disassert')
            newState = 'rec_error'
            return (newState, recs)
        waitTime = (datetime.now() - startTime).total_seconds()
    newState = 'el_disassert'
    return (newState, recs)

def el_disassert_transitions(recs):
    waitTime = 0
    cond = recs['condition']
    actn = recs['action']
    actn['elDriveEn'].put(1)
    startTime = datetime.now()
    while True:
        if cond['elDriveCond'].value == 1:
            break
        elif waitTime > ERROR_TIME:
            print('Error: Elevation Drive did not disassert')
            newState = 'rec_error'
            return (newState, recs)
        waitTime = (datetime.now() - startTime).total_seconds()
    newState = 'disable_tracking'
    return (newState, recs)

def disable_tracking_transitions(recs):
    actn = recs['action']
    actn['mcsTrackDis'].put(1)
    newState = 'az_assert'
    return (newState, recs)

def az_assert_transitions(recs):
    waitTime = 0
    cond = recs['condition']
    actn = recs['action']
    actn['azDriveEn'].put(2)
    startTime = datetime.now()
    while True:
        if cond['azDriveCond'].value == 2:
            break
        elif waitTime > ERROR_TIME:
            print('Error: Azimuth Drive did not assert')
            newState = 'rec_error'
            return (newState, recs)
        waitTime = (datetime.now() - startTime).total_seconds()
    newState = 'enable_tracking'
    return (newState, recs)

def enable_tracking_transitions(recs):
    actn = recs['action']
    actn['mcsTrackDis'].put(0)
    newState = 'el_assert'
    return (newState, recs)

def el_assert_transitions(recs):
    waitTime = 0
    cond = recs['condition']
    actn = recs['action']
    actn['elDriveEn'].put(2)
    startTime = datetime.now()
    while True:
        if cond['elDriveCond'].value == 2:
            break
        elif waitTime > ERROR_TIME:
            print('Error: Elevation Drive did not assert')
            newState = 'rec_error'
            return (newState, recs)
        waitTime = (datetime.now() - startTime).total_seconds()
    newState = 'follow_on'
    return (newState, recs)

def follow_on_transitions(recs):
    waitTime = 0
    cond = recs['condition']
    actn = recs['action']
    actn['tcsMCSFollow'].put('On')
    actn['tcsApply'].put(3)
    startTime = datetime.now()
    while True:
        if cond['mcsFollow'].value:
            break
        elif waitTime > ERROR_TIME:
            print('Error: MCS did not start tracking')
            newState = 'rec_error'
            return (newState, recs)
        waitTime = (datetime.now() - startTime).total_seconds()
    if (((abs(cond['voltAz'].value) < 0.1)
         and (abs(cond['azPosErr'].value) > 0.01))
        or ((abs(cond['voltEl'].value) < 0.1)
            and (abs(cond['elPosErr'].value) > 0.01))):
        if cond['prevState'] == 'follow_off':
            print('Error: MCS Follow enabled but telescope not tracking')
            newState = 'rec_error'
        else:
            newState = 'follow_off'
    else:
        newState = 'rec_success'
    return (newState, recs)

def rec_success_actions(recs):
    print('Non Zero Speed Fault recovery successful')

def rec_error_actions(recs):
    actn = recs['action']
    actn['mcsTrackDis'].put(0)
    print('Non Zero Speed Fault recovery ended in error')

if __name__ == '__main__':
    nzsfSM = StateMachine()
    nzsfSM.add_state('start', start_transitions)
    nzsfSM.add_state('follow_off', follow_off_transitions)
    nzsfSM.add_state('voltage_zero', voltage_zero_transitions)
    nzsfSM.add_state('clear_nzsf', clear_nzsf_transitions)
    nzsfSM.add_state('fault_cleared', fault_cleared_transitions)
    nzsfSM.add_state('az_disassert', az_disassert_transitions)
    nzsfSM.add_state('el_disassert', el_disassert_transitions)
    nzsfSM.add_state('disable_tracking', disable_tracking_transitions)
    nzsfSM.add_state('az_assert', az_assert_transitions)
    nzsfSM.add_state('enable_tracking', enable_tracking_transitions)
    nzsfSM.add_state('el_assert', el_assert_transitions)
    nzsfSM.add_state('follow_on', follow_on_transitions)
    nzsfSM.add_state('no_fault', no_fault_actions, end_state=True)
    nzsfSM.add_state('rec_error', rec_error_actions, end_state=True)
    nzsfSM.add_state('rec_success', rec_success_actions, end_state=True)
    nzsfSM.set_start('start')
    nzsfSM.run(Recs)

