import os

for X in range(1,11):
    for Y in range(1,11):
        names = 'Goal' + str(X) + '_' + str(Y)
        newpath = 'ctrls/' + names
        if not os.path.exists(newpath):
            os.makedirs(newpath)
