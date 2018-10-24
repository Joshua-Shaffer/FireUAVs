import imp, sys

from guppy import hpy


ic_name = 'G3_3Pos1_2Ori1.py'

dir_name = 'ctrls3/Goal3_3/' + ic_name

#module_con = imp.load_source('TulipStrategy', dir_name)
#module_handle = module_con.TulipStrategy()
#del module_con
#print(module_handle)

fire_input = False
sync_vert = False
sync_signal = False
stop_signal = False
sync_hori = False
#print(sys.getsizeof(module_handle))
h = hpy()
print h.heap()
while True:
    print module_handle.move(fire_input, sync_vert, sync_signal, stop_signal, sync_hori)
    #fire_input = input('Fire signal')
    sync_hori = input('Horizontal sync signal')
    sync_vert = input('Vertical sync signal')
    #stop_signal = input('Stopping signal')
    #continue_si = input('Continue?')
    #if continue_si == 0:
    #    break
