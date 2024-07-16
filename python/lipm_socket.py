from socket import *

import numpy as np
import time
import os
from datetime import datetime

import subprocess
os.system('g++  lipm_socket.cpp -lpthread')
subprocess.Popen("./a.out")

while(1):
    k = 0