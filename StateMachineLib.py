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

ERROR_TIME = 1.5*60

class InitializationError(Exception):
    def __init__(self, message):
        super().__init__()
        self.message = 'InitializationError: ' + message

class State:
    '''
    This class defines a State object to be used by a StateMachine object
    Atributes:
        -name: State name
        -handler: user defined function that handles state actions
        Function name must follow this structure:
            def STATE_NAME_handler():
            Where STATE_NAME is the name of the current state
        -tarray: user array that defines transitions behavior
        The transitions array must have the following structure:
            STATE_NAME_trans =\
                [[NEXT_STATE, [INPUT_NAMES], CONDITION FUNCTION, ERROR, MESSAGE]]
            where:
                STATE_NAME: Name of the current state
                NEXT_STATE: String for name of the state to transition
                [INPUT_NAMES]:Array with the name of each input to be used on
                the condition evaluation function
                CONDITION_FUNCTION: Lambda function that process the transition
                based on input values
                ERROR: Boolean that indicates if the state to transition is an
                error state
                MESSAGE: A string to be displayed when transition occurs
        -transitions: Dictionary that contains transitions information for the
        State

    Methods:
        -init_transitions: Fills the State transitions dictionary
        -run_handler: Runs the handler function using a input/output dictionary
        -run_transitions: Executes a routines that tests each transition
        condition
    '''
    def __init__(self, Name, Handler, Tarray=[], sS=False, eS=False):
        self.name = Name
        self.handler = Handler
        self.tarray = Tarray
        # Very important that transitions are added in order
        self.transitions = collections.OrderedDict()
        self.startState = sS
        self.endState = eS

    def init_transitions(self):
        if not(self.endState):
            for st in self.tarray:
                self.transitions[st[0]] = {'inp':st[1],
                                           'cond':st[2],
                                           'error':st[3],
                                           'msg':st[4]}
                # print('Transition to {} state ready'.format(st[0]))

    def run_handler(self, iod):
        # outp = iod['Output']
        self.handler(iod)

    def run_transitions(self, iod):
        waitTime = 0
        inpt = iod['Input']
        tt = self.transitions
        startTime = datetime.now()
        while True:
            for ns in tt:
                if tt[ns]['cond'](tt[ns]['inp'],inpt):
                    nextState = ns
                    break
            waitTime = (datetime.now() - startTime).total_seconds()
            if not(tt[nextState]['error']) or waitTime > ERROR_TIME:
                break
        msg = tt[nextState]['msg']
        if msg:
            print(msg)
        return nextState

class StateMachine:
    def __init__(self):
        self.states = {}
        self.startState = None
        self.endStates = []
        self.prevState = ''

    def add_state(self, state):
        name = state.name.upper()
        self.states[name] = state
        print('Added {} state to State Machine'.format(name))
        if state.startState:
            try:
                if self.startState:
                    raise InitializationError('More than one Start State defined')
            except InitializationError as err:
                print(err.message)
                exit(0)
            self.startState = name
        if state.endState:
            self.endStates.append(name)

    def run(self, records):
        try:
            if not(self.startState):
                raise InitializationError('No Start State defined')
            if not(self.endStates):
                raise InitializationError('No End State defined')
        except InitializationError as err:
            print(err.message)
            exit(0)
        currState = self.states[self.startState]
        self.prevState = self.startState
        while True:
            # currState.run_handler(records)
            newState = currState.run_transitions(records)
            records['Input']['prevState'] = self.prevState
            self.prevState = newState
            currState = self.states[newState.upper()]
            if newState.upper() in self.endStates:
                print("Recovery ended on {0}".format(newState.upper()))
                # currState.run_handler(records)
                break
            else:
                print("Recovery in {0} state".format(newState.upper()))

if __name__ == '__main__':
    pass
