class TulipStrategy(object):
    """Mealy transducer.

    Internal states are integers, the current state
    is stored in the attribute "state".
    To take a transition, call method "move".

    The names of input variables are stored in the
    attribute "input_vars".

    Automatically generated by tulip.dumpsmach on 2018-06-26 01:06:30 UTC
    To learn more about TuLiP, visit http://tulip-control.org
    """
    def __init__(self):
        self.state = 17
        self.input_vars = ['Fire', 'StopSignal']

    def move(self, Fire, StopSignal):
        """Given inputs, take move and return outputs.

        @rtype: dict
        @return: dictionary with keys of the output variable names:
            ['sys_actions', 'loc', 'Base', 'GoalPos']
        """
        output = dict()
        if self.state == 0:
            if (Fire == False) and (StopSignal == False):
                self.state = 1

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 1:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 2:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 3:
            if (Fire == True) and (StopSignal == True):
                self.state = 16

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == False):
                self.state = 13

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == False):
                self.state = 14

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == True):
                self.state = 15

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 4:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 5:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 6:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 7:
            if (Fire == False) and (StopSignal == False):
                self.state = 9

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == False):
                self.state = 10

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == True):
                self.state = 11

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == True):
                self.state = 12

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 8:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 9:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 10:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 11:
            if (Fire == False) and (StopSignal == False):
                self.state = 9

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == False):
                self.state = 10

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == True):
                self.state = 11

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == True):
                self.state = 12

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 12:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 13:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 14:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 15:
            if (Fire == True) and (StopSignal == True):
                self.state = 16

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == False):
                self.state = 13

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == False):
                self.state = 14

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == True):
                self.state = 15

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 16:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 17:
            if (Fire == False) and (StopSignal == False):
                self.state = 0

                output["loc"] = 'Pos9_10Ori2'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 10

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == True):
                self.state = 11

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == True):
                self.state = 12

                output["loc"] = 'Pos9_8Ori4'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == False):
                self.state = 14

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == True):
                self.state = 15

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == True):
                self.state = 16

                output["loc"] = 'Pos10_9Ori3'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            else:
                self._error(Fire, StopSignal)
        else:
            raise Exception("Unrecognized internal state: " + str(self.state))
        return output

    def _error(self, Fire, StopSignal):
        raise ValueError("Unrecognized input: " + (
            "Fire = {Fire}; "
            "StopSignal = {StopSignal}; ").format(
                Fire=Fire,
                StopSignal=StopSignal))
