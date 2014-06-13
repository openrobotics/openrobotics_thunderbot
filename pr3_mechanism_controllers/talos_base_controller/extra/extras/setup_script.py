#!/usr/bin/env python

#from subprocess import Popen, PIPE
import subprocess
proc = subprocess.Popen(["readlink", "/dev/arduino-mega"], bufsize=1, stdout=subprocess.PIPE)
std, err = proc.communicate()
std = std.rstrip()
print std
path = "/dev/"+std
proc = subprocess.Popen(["chmod", "a+xrw", path])
