import re
import glob
import subprocess

# runs a command in a subprocess with optional hide output
def runCommand(command, hideOutput):
    if hideOutput:
        p = subprocess.Popen(command, 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL)
    else:
        p = subprocess.Popen(command)
    try:
        p.wait()
    except KeyboardInterrupt:
        exit()