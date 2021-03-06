class TulipStrategy(object):
    """Mealy transducer.

    Internal states are integers, the current state
    is stored in the attribute "state".
    To take a transition, call method "move".

    The names of input variables are stored in the
    attribute "input_vars".

    Automatically generated by tulip.dumpsmach on 2018-06-26 01:09:32 UTC
    To learn more about TuLiP, visit http://tulip-control.org
    """
    def __init__(self):
        self.state = 9
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

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 1:
            if (Fire == False) and (StopSignal == False):
                self.state = 1

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 2:
            if (Fire == False) and (StopSignal == False):
                self.state = 1

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 3:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 4:
            if (Fire == False) and (StopSignal == False):
                self.state = 1

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 5:
            if (Fire == False) and (StopSignal == False):
                self.state = 1

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 6:
            if (Fire == False) and (StopSignal == False):
                self.state = 1

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 7:
            if (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == False):
                self.state = 5

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 8:
            if (Fire == False) and (StopSignal == False):
                self.state = 1

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            else:
                self._error(Fire, StopSignal)
        elif self.state == 9:
            if (Fire == False) and (StopSignal == False):
                self.state = 0

                output["loc"] = 'Pos4_3Ori2'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 2

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == False) and (StopSignal == True):
                self.state = 3

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == True):
                self.state = 4

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Go'
            elif (Fire == True) and (StopSignal == False):
                self.state = 6

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == False) and (StopSignal == True):
                self.state = 7

                output["loc"] = 'Pos5_4Ori1'
                output["Base"] = False
                output["GoalPos"] = False
                output["sys_actions"] = 'Stop'
            elif (Fire == True) and (StopSignal == True):
                self.state = 8

                output["loc"] = 'Pos5_4Ori1'
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
