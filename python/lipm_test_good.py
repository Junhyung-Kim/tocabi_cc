import roslibpy
import pickle
import roslaunch
import numpy as np
import time
import control
from copy import copy
from sklearn.model_selection import train_test_split
import scipy.stats
from sklearn.decomposition import PCA
from sklearn.mixture import GaussianMixture
import matplotlib.pyplot as plt
from IPython.display import clear_output
import intel_extension_for_pytorch as ipex
import logging
import os
import torch
import pinocchio
import crocoddyl
from pinocchio.robot_wrapper import RobotWrapper

import sys
import numpy.matlib
np.set_printoptions(threshold=sys.maxsize)
global client
global learn_type
import torch
import torch.nn as nn
from torch.autograd import Variable
from torch.utils.data import Dataset
import torch.optim as optim
import torch.multiprocessing as multiprocessing
import ctypes
import pytorch_model_summary
from multiprocessing import shared_memory
import sysv_ipc

def calculatePreviewControlParams(A, B, C, Q, R, N):
    [P, _, _] = control.dare(A, B, C.T*Q*C, R)
    K = (R + B.T*P*B).I*(B.T*P*A)

    f = np.zeros((1, N))
    for i in range(N):
        f[0,i] = (R+B.T*P*B).I*B.T*(((A-B*K).T)**i)*C.T*Q

    return K, f

class CShmReader :
    def __init__(self) :
        pass
 
    def doReadShm(self , key) :
        memory = sysv_ipc.SharedMemory(key)
        memory_value = memory.read()
        c = np.ndarray((2,), dtype=np.double, buffer=memory_value)

    def doWriteShm(self, Input) :
        self.memory.write(Input)

def InversePCA(model, rbf_num, pca, Phi, tick, X, thread_manager):
    k = 0
    while True:
        if thread_manager[0] == 1:# and k == 0:
            c = torch.tensor(np.array(X[:]).reshape(1,1,43),dtype=torch.float32)
            w_traj = model[k].forward(c)[0].detach().numpy()
            w_traj = pca[k].inverse_transform([w_traj[None,:]])[0]
            w_traj = w_traj.reshape(rbf_num[k][3],-1)
            q_traj[:] = np.dot(Phi[k],w_traj)
            thread_manager[0] = 0
            k = k + 1
        else:
            if k == 149:
                k = 49

def InversePCA1(model, rbf_num, pca, Phi, tick, X, thread_manager):
    k = 0
    while True:
        if thread_manager[1] == 1:
            c = torch.tensor(np.array(X[:]).reshape(1,1,43),dtype=torch.float32)
            w_traj1 = model[k].forward(c).detach().numpy()
            w_traj1 = pca[k].inverse_transform([w_traj1[None,:]])[0]
            w_traj1 = w_traj1.reshape(rbf_num[k][3],-1)
            v_traj[:] = np.dot(Phi[k],w_traj1)
            a_traj[:] = np.subtract(v_traj[1:60,:], v_traj[0:59,:])/0.02
            thread_manager[1] = 0
            k = k + 1
        else:
            if k == 149:
                k = 49


def InversePCA2(model, rbf_num, pca, Phi, tick, X, thread_manager):
    k = 0
    while True:
        if thread_manager[2] == 1:# and k == 0:
            c = torch.tensor(np.array(X[:]).reshape(1,1,43),dtype=torch.float32)
            w_traj2 = model[k].forward(c).detach().numpy()
            w_traj2 = pca[k].inverse_transform([w_traj2[None,:]])[0]
            w_traj2 = w_traj2.reshape(rbf_num[k][3],-1)
            x_traj[:] = np.dot(Phi[k],w_traj2)
           
            u_traj[:,0:2] = np.subtract(x_traj[1:60,2:4], x_traj[0:59,2:4])/0.02
            u_traj[:,2:4] = np.subtract(x_traj[1:60,6:8], x_traj[0:59,6:8])/0.02
           
            #x_traj = np.dot(Phi[k],pca[k].inverse_transform([model[k].forward(c)[0].detach().numpy()[None,:]])[0].reshape(rbf_num,-1))
            thread_manager[2] = 0
            k = k + 1
        else:
            if k == 149:
                k = 49
           
class timeseries(Dataset):
    def __init__(self,x,y):
        self.x = torch.tensor(x,dtype=torch.float32)
        self.y = torch.tensor(y,dtype=torch.float32)
        self.len = x.shape[0]
       
    def __getitem__(self,idx):
        return self.x[idx],self.y[idx]
 
    def __len__(self):
        return self.len

class CNN(nn.Module):
    def __init__(self, input_size, output_size, device):
        super(CNN, self).__init__()
        self.layer1 = torch.nn.Sequential(
            torch.nn.Conv1d(1, 10, kernel_size=10, stride=1),
            torch.nn.ReLU(),
            )
        self.layer2 = torch.nn.Sequential(
            torch.nn.MaxPool1d(kernel_size=10, stride = 1),
            torch.nn.Flatten()
        )
        self.layer3 = torch.nn.Sequential(
            torch.nn.Linear(in_features = input_size, out_features= 50),
            torch.nn.LeakyReLU(),
            torch.nn.Linear(in_features = 50, out_features = output_size)
            )

    def forward(self, x):
        out = self.layer3(x)
        return out

class CNN1(nn.Module):
    def __init__(self, input_size, output_size, device):
        super(CNN1, self).__init__()
        self.layer1 = torch.nn.Sequential(
            torch.nn.Conv1d(1, 10, kernel_size=10, stride=1),
            torch.nn.ReLU(),
            )
        self.layer2 = torch.nn.Sequential(
            torch.nn.MaxPool1d(kernel_size=10, stride = 1),
            torch.nn.Flatten()
        )
        self.layer3 = torch.nn.Sequential(
            torch.nn.Linear(in_features = input_size, out_features= int(input_size*2/3) + output_size),
            torch.nn.ReLU(),
            torch.nn.Linear(in_features = int(input_size*2/3) + output_size, out_features = output_size)
            )

    def forward(self, x):
        out = self.layer3(x)
        return out

def define_RBF(dof=39, nbStates=60, offset=200, width=60, T=4000, coeff = 250):
    tList = np.arange(T)

    Mu = np.linspace(tList[0]-offset, tList[-1]+offset, nbStates)
    Sigma  = np.reshape(np.matlib.repmat(width, 1, nbStates),[1, 1, nbStates])
    Sigma.shape
    Phi = np.zeros((T, nbStates))
    for i in range(nbStates):
        Phi[:,i] = coeff*scipy.stats.norm(Mu[i], Sigma[0,0,i]).pdf(tList)
    return Phi

def apply_RBF(trajs, Phi, rcond=0.0001):
    w_trajs = []
    for traj in trajs:
        w,_,_,_ = np.linalg.lstsq(Phi, traj, rcond=0.0001)
        w_trajs.append(w.flatten())
    return np.array(w_trajs)
   
def inverse_transform(w_pca, pca, Phi, rbf_num):
    w = pca.inverse_transform(w_pca)
    w = w.reshape(rbf_num,-1)
    traj = np.dot(Phi,w)
    return traj

def constraint():  
    global zmp_refxssp2, zmp_refyssp2, array_boundxssp2, array_boundyssp2, array_boundRFssp2, array_boundLFssp2, zmp_refy, zmp_refx, array_boundLF, array_boundRF, array_boundx, array_boundy, array_boundxssp1, array_boundyssp1, array_boundRFssp1, array_boundLFssp1
    global cp_forEOS, cp_ssp1EOS, cp_ssp2EOS, cp_for, cp_ssp1, cp_ssp2
    print("start")
    f = open("/home/jhk/walkingdata/beforedata/fdyn/lfoot2_final.txt", 'r')
    f1 = open("/home/jhk/walkingdata/beforedata/fdyn/rfoot2_final.txt", 'r')
    f2 = open("/home/jhk/walkingdata/beforedata/fdyn/zmp2_ssp1_2.txt", 'r')
    f3 = open("/home/jhk/data/mpc/5_tocabi_data.txt", 'w')
    f4 = open("/home/jhk/data/mpc/6_tocabi_data.txt", 'w')
    f5 = open("/home/jhk/walkingdata/beforedata/fdyn/zmp2_ssp1_2.txt", 'r')

    lines = f.readlines()
    lines2 = f2.readlines()
    lines3 = f5.readlines()  
    lines1 = f1.readlines()

    N = 60

    array_boundx = [[] for i in range(int(len(lines2)))]
    array_boundy = [[] for i in range(int(len(lines2)))]

    array_boundx_ = [[] for i in range(N)]
    array_boundy_ = [[] for i in range(N)]

    array_boundRF = [[] for i in range(int(len(lines1)))]
    array_boundLF = [[] for i in range(int(len(lines1)))]

    cp_for = [[] for i in range(int(len(lines1)))]
    cp_ssp1 = [[] for i in range(int(len(lines1)))]
    cp_ssp2 = [[] for i in range(int(len(lines1)))]
    cp_forEOS = [[] for i in range(5)]
    cp_ssp1EOS = [[] for i in range(5)]
    cp_ssp2EOS = [[] for i in range(5)]

    array_boundRF_ = [[] for i in range(N)]
    array_boundLF_ = [[] for i in range(N)]

    zmp_refx = [[] for i in range(len(lines3))]
    zmp_refy = [[] for i in range(len(lines3))]

    zmp_refx_ = [[] for i in range(N)]
    zmp_refy_ = [[] for i in range(N)]

    lines_array = []
    for i in range(0, len(lines)):
        lines_array.append(lines[i].split())

    lines1_array = []
    for i in range(0, len(lines1)):
        lines1_array.append(lines1[i].split())

    lines2_array = []
    for i in range(0, len(lines2)):
        lines2_array.append(lines2[i].split())

    lines3_array = []
    for i in range(0, len(lines3)):
        lines3_array.append(lines3[i].split())

    for i in range(0, len(lines_array)):
        for j in range(0, len(lines_array[i])):
            if j == 0:
                array_boundRF[i].append(float(lines_array[i][j]))
            if j == 1:
                array_boundRF[i].append(float(lines_array[i][j]))
            if j == 2:
                array_boundRF[i].append(float(lines_array[i][j]))
   
    for i in range(0, len(lines1_array)):
        for j in range(0, len(lines1_array[i])):
            if j == 0:
                array_boundLF[i].append(float(lines1_array[i][j]))
            if j == 1:
                array_boundLF[i].append(float(lines1_array[i][j]))
            if j == 2:
                array_boundLF[i].append(float(lines1_array[i][j]))

    for i in range(0, len(lines_array)):
        array_boundRF[i] = np.sum([array_boundRF[i], [-0.03, 0.0, 0.15842]], axis = 0)
    for i in range(0, len(lines1_array)):
        array_boundLF[i] = np.sum([array_boundLF[i], [-0.03, 0.0, 0.15842]], axis = 0)
   
    for i in range(0, len(lines2_array)):
        for j in range(0, len(lines2_array[i])):
            if j == 0:
                array_boundx[i].append(float(lines2_array[i][j]))
            if j == 1:
                array_boundx[i].append(float(lines2_array[i][j]))
            if j == 2:
                array_boundy[i].append(float(lines2_array[i][j]))
            if j == 3:
                array_boundy[i].append(float(lines2_array[i][j]))

    for i in range(0, len(lines3_array)):
        for j in range(0, len(lines3_array[i])):
            if j == 0:
                zmp_refx[i].append(float(lines3_array[i][j]))
            if j == 1:
                [i].append(float(lines3_array[i][j]))

    f.close()
    f1.close()
    f2.close()

    f = open("/home/jhk/walkingdata/beforedata/ssp2/lfoot1.txt", 'r')
    f1 = open("/home/jhk/walkingdata/beforedata/ssp2/rfoot2.txt", 'r')
    f2 = open("/home/jhk/walkingdata/beforedata/ssp2/zmp3.txt", 'r')
    f3 = open("/home/jhk/data/mpc/5_tocabi_data.txt", 'w')
    f4 = open("/home/jhk/data/mpc/6_tocabi_data.txt", 'w')
    f5 = open("/home/jhk/walkingdata/beforedata/ssp2/zmp3.txt", 'r')

    lines = f.readlines()
    lines2 = f2.readlines()
    lines3 = f5.readlines()  
    lines1 = f1.readlines()

    N = 60
    array_boundxssp2 = [[] for i in range(int(len(lines2)))]
    array_boundyssp2 = [[] for i in range(int(len(lines2)))]

    array_boundx_ssp2_ = [[] for i in range(N)]
    array_boundy_ssp2_ = [[] for i in range(N)]

    array_boundRFssp2 = [[] for i in range(int(len(lines1)))]
    array_boundLFssp2 = [[] for i in range(int(len(lines1)))]

    array_boundRF_ssp2_ = [[] for i in range(N)]
    array_boundLF_ssp2_ = [[] for i in range(N)]

    zmp_refxssp2 = [[] for i in range(len(lines3))]
    zmp_refyssp2 = [[] for i in range(len(lines3))]

    zmp_refx_ssp2_ = [[] for i in range(N)]
    zmp_refy_ssp2_ = [[] for i in range(N)]

    lines_array = []
    for i in range(0, len(lines)):
        lines_array.append(lines[i].split())

    lines1_array = []
    for i in range(0, len(lines1)):
        lines1_array.append(lines1[i].split())

    lines2_array = []
    for i in range(0, len(lines2)):
        lines2_array.append(lines2[i].split())

    lines3_array = []
    for i in range(0, len(lines3)):
        lines3_array.append(lines3[i].split())

    for i in range(0, len(lines_array)):
        for j in range(0, len(lines_array[i])):
            if j == 0:
                array_boundRFssp2[i].append(float(lines_array[i][j]))
            if j == 1:
                array_boundRFssp2[i].append(float(lines_array[i][j]))
            if j == 2:
                array_boundRFssp2[i].append(float(lines_array[i][j]))
   
    for i in range(0, len(lines1_array)):
        for j in range(0, len(lines1_array[i])):
            if j == 0:
                array_boundLFssp2[i].append(float(lines1_array[i][j]))
            if j == 1:
                array_boundLFssp2[i].append(float(lines1_array[i][j]))
            if j == 2:
                array_boundLFssp2[i].append(float(lines1_array[i][j]))

    for i in range(0, len(lines_array)):
        array_boundRFssp2[i] = np.sum([array_boundRFssp2[i], [-0.03, 0.0, 0.15842]], axis = 0)
    for i in range(0, len(lines1_array)):
        array_boundLFssp2[i] = np.sum([array_boundLFssp2[i], [-0.03, 0.0, 0.15842]], axis = 0)
   
    for i in range(0, len(lines2_array)):
        for j in range(0, len(lines2_array[i])):
            if j == 0:
                array_boundxssp2[i].append(float(lines2_array[i][j]))
            if j == 1:
                array_boundxssp2[i].append(float(lines2_array[i][j]))
            if j == 2:
                array_boundyssp2[i].append(float(lines2_array[i][j]))
            if j == 3:
                array_boundyssp2[i].append(float(lines2_array[i][j]))

    for i in range(0, len(lines3_array)):
        for j in range(0, len(lines3_array[i])):
            if j == 0:
                zmp_refxssp2[i].append(float(lines3_array[i][j]))
            if j == 1:
                zmp_refyssp2[i].append(float(lines3_array[i][j]))

    f.close()
    f1.close()
    f2.close()

    f = open("/home/jhk/walkingdata/beforedata/ssp1/lfoot2.txt", 'r')
    f1 = open("/home/jhk/walkingdata/beforedata/ssp1/rfoot2.txt", 'r')
    f2 = open("/home/jhk/walkingdata/beforedata/ssp1/zmp3.txt", 'r')
    f3 = open("/home/jhk/data/mpc/5_tocabi_data.txt", 'w')
    f4 = open("/home/jhk/data/mpc/6_tocabi_data.txt", 'w')
    f5 = open("/home/jhk/walkingdata/beforedata/ssp1/zmp3.txt", 'r')

    lines = f.readlines()
    lines2 = f2.readlines()
    lines3 = f5.readlines()  
    lines1 = f1.readlines()

    N = 60
    array_boundxssp1 = [[] for i in range(int(len(lines2)))]
    array_boundyssp1 = [[] for i in range(int(len(lines2)))]
    array_boundx_ssp1_ = [[] for i in range(N)]
    array_boundy_ssp1_ = [[] for i in range(N)]

    array_boundRFssp1 = [[] for i in range(int(len(lines1)))]
    array_boundLFssp1 = [[] for i in range(int(len(lines1)))]

    array_boundRF_ssp1_ = [[] for i in range(N)]
    array_boundLF_ssp1_ = [[] for i in range(N)]

    zmp_refxssp1 = [[] for i in range(len(lines3))]
    zmp_refyssp1 = [[] for i in range(len(lines3))]

    zmp_refx_ssp1_ = [[] for i in range(N)]
    zmp_refy_ssp1_ = [[] for i in range(N)]

    lines_array = []
    for i in range(0, len(lines)):
        lines_array.append(lines[i].split())

    lines1_array = []
    for i in range(0, len(lines1)):
        lines1_array.append(lines1[i].split())

    lines2_array = []
    for i in range(0, len(lines2)):
        lines2_array.append(lines2[i].split())

    lines3_array = []
    for i in range(0, len(lines3)):
        lines3_array.append(lines3[i].split())

    for i in range(0, len(lines_array)):
        for j in range(0, len(lines_array[i])):
            if j == 0:
                array_boundRFssp1[i].append(float(lines_array[i][j]))
            if j == 1:
                array_boundRFssp1[i].append(float(lines_array[i][j]))
            if j == 2:
                array_boundRFssp1[i].append(float(lines_array[i][j]))
   
    for i in range(0, len(lines1_array)):
        for j in range(0, len(lines1_array[i])):
            if j == 0:
                array_boundLFssp1[i].append(float(lines1_array[i][j]))
            if j == 1:
                array_boundLFssp1[i].append(float(lines1_array[i][j]))
            if j == 2:
                array_boundLFssp1[i].append(float(lines1_array[i][j]))

    for i in range(0, len(lines_array)):
        array_boundRFssp1[i] = np.sum([array_boundRFssp1[i], [-0.03, 0.0, 0.15842]], axis = 0)
    for i in range(0, len(lines1_array)):
        array_boundLFssp1[i] = np.sum([array_boundLFssp1[i], [-0.03, 0.0, 0.15842]], axis = 0)
   
    for i in range(0, len(lines2_array)):
        for j in range(0, len(lines2_array[i])):
            if j == 0:
                array_boundxssp1[i].append(float(lines2_array[i][j]))
            if j == 1:
                array_boundxssp1[i].append(float(lines2_array[i][j]))
            if j == 2:
                array_boundyssp1[i].append(float(lines2_array[i][j]))
            if j == 3:
                array_boundyssp1[i].append(float(lines2_array[i][j]))
   
    for i in range(0, len(lines3_array)):
        for j in range(0, len(lines3_array[i])):
            if j == 0:
                zmp_refxssp1[i].append(float(lines3_array[i][j]))
            if j == 1:
                zmp_refyssp1[i].append(float(lines3_array[i][j]))

    f.close()
    f1.close()
    f2.close()
    '''
    for i in range(0, N):
        array_boundRF_[i] = array_boundRF[k*i + time_step]
        array_boundRF_ssp2_[i] = array_boundRFssp2[k*i + time_step]

    for i in range(0, N):        
        array_boundLF_[i] = array_boundLF[k*i + time_step]
        array_boundLF_ssp2_[i] = array_boundLFssp2[k*i + time_step]
           
    for i in range(0, N):
        array_boundx_[i] = array_boundx[k3*(i) + time_step]
        array_boundy_[i] = array_boundy[k3*(i) + time_step]
        array_boundx_ssp2_[i] = array_boundxssp2[k3*(i) + time_step]
        array_boundy_ssp2_[i] = array_boundyssp2[k3*(i) + time_step]
   
    for i in range(0, N):
        zmp_refx_[i] = zmp_refx[k*(i)+ time_step]
        zmp_refy_[i] = zmp_refy[k*(i)+ time_step]
        zmp_refx_ssp2_[i] = zmp_refxssp2[k*(i)+ time_step]
        zmp_refy_ssp2_[i] = zmp_refyssp2[k*(i)+ time_step]
    '''

def PCAlearning(time_step):
    global xs_pca_test, rbf_num
    global xs_pca
    global us_pca

    naming = [
        "timestep=0_finish_ssp2",  
"timestep=1_finish_ssp2",
"timestep=2_finish_ssp2",
"timestep=3_finish_ssp2",      
"timestep=4_finish_ssp2",    
"timestep=5_finish_ssp2",
"timestep=6_finish_ssp2",
"timestep=7_finish_ssp2",
"timestep=8_finish_ssp2",
"timestep=9_finish_ssp2",
"timestep=10_finish_ssp2",      
"timestep=12_finish_ssp2",
"timestep=12_finish_ssp2",      
"timestep=14_finish_ssp2",      
"timestep=14_finish_ssp2",      
"timestep=15_finish_ssp2",      
"timestep=16_finish_ssp2",      
"timestep=17_finish_ssp2",      
"timestep=21_finish_ssp2",
"timestep=21_finish_ssp2",    
"timestep=21_finish_ssp2", ##
"timestep=21_finish_ssp2",
"timestep=22_finish_ssp2",
"timestep=23_finish_ssp2",  
"timestep=24_finish_ssp2",
"timestep=24_finish_ssp2",
"timestep=26_finish_ssp2",
"timestep=27_finish_ssp2",
"timestep=28_finish_ssp2",
"timestep=29_finish_ssp2",
"timestep=30_finish_ssp2",
"timestep=31_finish_ssp2",
"timestep=32_finish_ssp2",
"timestep=33_finish_ssp2",
"timestep=34_finish_ssp2",    
"timestep=35_finish_ssp2",  
"timestep=36_finish_ssp2",      
"timestep=37_finish_ssp2",
"timestep=38_finish_ssp2",        
"timestep=39_finish_ssp2",
"timestep=40_finish_ssp2",  
"timestep=41_finish_ssp2",
"timestep=42_finish_ssp2",
"timestep=43_finish_ssp2",
"timestep=44_finish_ssp2",
"timestep=45_finish_ssp2",
"timestep=46_finish_ssp2",
"timestep=47_finish_ssp2",
"timestep=48_finish_ssp2",
"timestep=49_finish_ssp2",
##SSP1

    "timestep=0_finish_re",  
    "timestep=1_finish_re",
    "timestep=2_finish_re",
    "timestep=3_finish_re",      
    "timestep=4_finish_re",    
    "timestep=5_finish_re",
    "timestep=6_finish_re",
    "timestep=7_finish_re",
    "timestep=8_finish_re",
    "timestep=9_finish_re",
        "timestep=10_finish_re",
        "timestep=11_finish_re",
        "timestep=12_finish_re_add",
        "timestep=13_finish_re_add",
        ##14
        "timestep=13_finish_re_add",
        "timestep=15_finish_re_add",
        "timestep=16_finish_re_add",
        "timestep=17_finish_re_add",
        #"timestep=17_finish_re_add",
        "timestep=18_finish_re_add",
        "timestep=19_finish_re_add",
        #"timestep=20_finish_re_add",
        "timestep=21_finish_re_add",
        "timestep=21_finish_re_add",
        #"timestep=22_finish_re_add",
        "timestep=23_finish_re_add",
        "timestep=23_finish_re_add",
        "timestep=24_finish_re_add",
        "timestep=25_finish_re",
        "timestep=26_finish_re_add",
        "timestep=27_finish_re_add",
        "timestep=28_finish_re_add",
        "timestep=29_finish_re_add",
        "timestep=30_finish_re_add",
        "timestep=31_finish_re_add",
        "timestep=32_finish_re",
        "timestep=33_finish_re_add",
        "timestep=34_finish_re",
        "timestep=36_finish_re",
        "timestep=36_finish_re",
        "timestep=37_finish_re",
        "timestep=38_finish_re_add",
        "timestep=39_finish_re_add",
        "timestep=40_finish_re",
        "timestep=41_finish_re",
        "timestep=42_finish_re",
        "timestep=43_finish_re",
        "timestep=44_finish_re",
        "timestep=45_finish_re",
        "timestep=46_finish_re",
        "timestep=47_finish_re",
        "timestep=48_finish_re",
        "timestep=49_finish_re",
    ]

    naming1 = [
    "timestep=0_finish_ssp2",  
"timestep=1_finish_ssp2",
"timestep=2_finish_ssp2",
"timestep=3_finish_ssp2",      
"timestep=4_finish_ssp2",    
"timestep=5_finish_ssp2",
"timestep=6_finish_ssp2",
"timestep=7_finish_ssp2",
"timestep=8_finish_ssp2",
"timestep=9_finish_ssp2",
"timestep=10_finish_ssp2",      
"timestep=12_finish_ssp2",
"timestep=12_finish_ssp2",      
"timestep=13_finish_ssp2",      
"timestep=14_finish_ssp2",      
"timestep=15_finish_ssp2",      
"timestep=16_finish_ssp2",      
"timestep=17_finish_ssp2",      
"timestep=21_finish_ssp2",      
"timestep=21_finish_ssp2",    
"timestep=21_finish_ssp2",
"timestep=21_finish_ssp2",
"timestep=22_finish_ssp2",
"timestep=23_finish_ssp2",  
"timestep=24_finish_ssp2",
"timestep=24_finish_ssp2", #revise
"timestep=26_finish_ssp2",
"timestep=27_finish_ssp2",
"timestep=28_finish_ssp2",
"timestep=29_finish_ssp2",
"timestep=30_finish_ssp2",
"timestep=31_finish_ssp2",
"timestep=32_finish_ssp2",
"timestep=33_finish_ssp2",
"timestep=34_finish_ssp2",    
"timestep=35_finish_ssp2",  
"timestep=36_finish_ssp2",      
"timestep=37_finish_ssp2",
"timestep=38_finish_ssp2",        
"timestep=39_finish_ssp2",
"timestep=40_finish_ssp2",  
"timestep=41_finish_ssp2",
"timestep=42_finish_ssp2",
"timestep=43_finish_ssp2",
"timestep=44_finish_ssp2",
"timestep=45_finish_ssp2",
"timestep=46_finish_ssp2",
"timestep=47_finish_ssp2",
"timestep=48_finish_ssp2",
"timestep=49_finish_ssp2",
    ##SSP1
   
    "timestep=0_finish_re",  
    "timestep=1_finish_re",
    "timestep=2_finish_re",
    "timestep=3_finish_re",      
    "timestep=4_finish_re",    
    "timestep=5_finish_re",
    "timestep=6_finish_re",
    "timestep=7_finish_re",
    "timestep=8_finish_re",
    "timestep=9_finish_re",
        "timestep=10_finish_re",
        "timestep=11_finish_re",
        "timestep=12_finish_re_add",
        "timestep=13_finish_re_add",
        "timestep=13_finish_re_add",
        "timestep=15_finish_re_add",
        "timestep=16_finish_re_add",
        "timestep=17_finish_re_add",
        #"timestep=17_finish_re_add",
        "timestep=18_finish_re_add",
        "timestep=19_finish_re_add",
        #"timestep=20_finish_re_add",
        "timestep=21_finish_re_add",
        "timestep=21_finish_re_add",
        #"timestep=22_finish_re_add",
        "timestep=23_finish_re_add",
        "timestep=23_finish_re_add",
        "timestep=24_finish_re_add",
        "timestep=25_finish_re",
        "timestep=26_finish_re_add",
        "timestep=27_finish_re_add",
        "timestep=28_finish_re_add",
        "timestep=29_finish_re_add",
        "timestep=30_finish_re_add",
        "timestep=31_finish_re_add",
        "timestep=32_finish_re",
        "timestep=33_finish_re_add",
        "timestep=34_finish_re",
        "timestep=36_finish_re",
        "timestep=36_finish_re",
        "timestep=37_finish_re",
        "timestep=38_finish_re_add",
        "timestep=39_finish_re_add",
        "timestep=40_finish_re",
        "timestep=41_finish_re",
        "timestep=42_finish_re",
        "timestep=43_finish_re",
        "timestep=44_finish_re",
        "timestep=45_finish_re",
        "timestep=46_finish_re",
        "timestep=47_finish_re",
        "timestep=48_finish_re",
        "timestep=49_finish_re",
    ]
 
    param = [
        [2,1,19,55], #0
        [2,1,13,55], #1
        [1,1,33,52], #2
        [1,1,45,55], #3
        [1,1,3,54], #4
        [1,1,13,54], #5
        [1,1,27,54], #6
        [1,1,35,53], #7
        [2,1,17,55], #8
        [2,1,19,54], #9
        [2,1,3,53], #10
        #[2,1,23,53], #11
        [1,1,27,54], #12
        [1,1,27,54], #12
        #[2,1,23,53], #13
        [1,1,33,55], #13
        [1,1,33,55], #14
        [1,1,23,55], #15
        [1,1,33,55], #16
        [1,1,35,54], #17
       
        #[2,1,49,53], #18
        #[2,1,3,53], #19
        [2,1,25,52], #21
        [2,1,25,52], #21
        [2,1,25,52], #21
        [2,1,25,52], #21
        [1,1,19,54], #22
        [1,1,13,55], #23
        [1,1,39,55], #24
        [3,1,35,55], #25
        [1,1,19,54], #26
        [1,1,19,54], #27
        [2,1,49,53], #28
        [1,1,27,54], #29
        [2,1,13,55], #30
        [1,1,17,53], #31
        [2,1,15,52], #32
        [1,1,7,55], #33
        [1,1,45,55], #34
        #[1,1,7,55], #35
        [1,1,23,55], #36
        [1,1,23,55], #36
        [1,1,49,53], #37
        [2,1,7,55], #38
        [3,1,17,53], #39
        [1,1,13,55], #40
        [2,1,35,54], #41
        [2,1,7,53], #42
        [1,1,19,55], #43
        [2,1,43,53], #44
        [1,1,41,55], #45
        [2,1,11,53], #46
        [2,1,21,52], #47
        [2,1,11,53], #48
        [2,1,47,53], #49
       
        ##SSP1
        [2,1,19,55], #0
        [2,1,13,55], #1
        [1,1,33,52], #2
        [1,1,45,55], #3
        [1,1,3,54], #4
        [1,1,13,54], #5
        [1,1,27,54], #6
        [1,1,35,53], #7
        [2,1,17,55], #8
        [2,1,19,54], #9
        [2,1,3,53], #10
        [2,1,23,53], #11
        [1,1,27,54], #12
        [2,1,23,53], #13
        [2,1,23,53], #13
        #[1,1,33,55], #14
        [1,1,23,55], #15
        [1,1,33,55], #16
        [1,1,35,54], #17
        #[1,1,35,54], #17 65
        [2,1,49,53], #18 66
        [2,1,3,53], #19 67
        #[1,1,15,54], #20 68
        [2,1,25,52], #21
        [2,1,25,52], #21
        #[1,1,19,54], #22
        [1,1,13,55], #23
        [1,1,13,55], #23
        [1,1,39,55], #24
        [3,1,35,55], #25
        [1,1,19,54], #26
        [1,1,19,54], #27
        [2,1,49,53], #28
        [1,1,27,54], #29
        [2,1,13,55], #30
        [1,1,17,53], #31
        [2,1,15,52], #32
        [1,1,7,55], #33
        [1,1,45,55], #34
        [1,1,7,55], #35
        [1,1,23,55], #36
        [1,1,49,53], #37
        [2,1,7,55], #38
        [3,1,17,53], #39
        [1,1,13,55], #40
        [2,1,35,54], #41
        [2,1,7,53], #42
        [1,1,19,55], #43
        [2,1,43,53], #44
        [1,1,41,55], #45
        [2,1,11,53], #46
        [2,1,21,52], #47
        [2,1,11,53], #48
        [2,1,47,53], #49
    ]

    learn_type = 1
    learn_type1 = 1
   
    if time_step <= 48:
        file_name ='/home/jhk/kino_dynamic_learning/dataset/dataset1/'
    elif time_step <= 97:
        file_name ='/home/jhk/kino_dynamic_learning/dataset/dataset2/ssp2/'
    else:
        file_name ='/home/jhk/kino_dynamic_learning/dataset/dataset2/ssp1/'

    init_trajs = dict()
    trajs = dict()
    x_inputs_init = dict()
    vel_trajs = dict()
    x_inputs = dict()
    acc_trajs = dict()
    foot_poses = dict()
    u_trajs = dict()
    x_trajs = dict()

    new_trajs = dict()
    new_vel_trajs = dict()
    new_u_trajs = dict()
   
    w_trajs = dict()
    w_vel_trajs = dict()
    w_x_trajs = dict()
    w_acc_trajs = dict()
    w_u_trajs = dict()

    w_trajs_pca = dict()
    pca = dict()

    w_x_trajs_pca = dict()
    pca_x = dict()

    w_vel_trajs_pca = dict()
    pca_vel = dict()

    w_acc_trajs_pca = dict()
    pca_acc = dict()

    w_u_trajs_pca = dict()
    pca_u = dict()
   
    #define dataset
    num_desired = 11130
    keys = ['Right']
    num_data = dict()
    key = 'Right'
   
    timestep = 60
   
    if time_step <= 48:
        rbf_num = 47
        Phi = define_RBF(dof=19, nbStates =rbf_num, offset = 2, width = 1, T = timestep, coeff =47)
    else:
        rbf_num = param[time_step-49][3]
        Phi = define_RBF(dof=19, nbStates =rbf_num, offset = param[time_step-49][0], width = param[time_step-49][1], T = timestep, coeff =param[time_step-49][2])

    x_inputs_train = dict()
    x_inputs_test = dict()
    x_inputs_train_temp = dict()
    x_inputs_test_temp = dict()
    y_train = dict()
    y_test = dict()
    y_test_temp = dict()
    y_train_temp = dict()

    y_vel_train = dict()
    y_vel_test = dict()
    y_vel_test_temp = dict()
    y_vel_train_temp = dict()

    y_acc_train = dict()
    y_acc_test = dict()
    y_acc_train_temp = dict()
    y_acc_test_temp = dict()

    y_u_train = dict()
    y_u_test = dict()
    y_u_train_temp = dict()
    y_u_test_temp = dict()

    y_x_train = dict()
    y_x_test = dict()
    y_x_train_temp = dict()
    y_x_test_temp = dict()

    if learn_type1 == 0:
        for key in keys:
            w_trajs[key] = apply_RBF(trajs[key], Phi)
            w_vel_trajs[key] = apply_RBF(vel_trajs[key], Phi)
            w_x_trajs[key] = apply_RBF(x_trajs[key], Phi)
            w_u_trajs[key] = apply_RBF(u_trajs[key], Phi)    
            w_acc_trajs[key] = apply_RBF(acc_trajs[key], Phi)

        file_name2 = '/Phi.pkl'
        file_name3 = file_name + str(time_step) + file_name2
        pickle.dump(Phi, open(file_name3,"wb"))
       
        for key in keys:
            pca[key] = PCA(n_components = int(rbf_num))
            w_trajs_pca[key] = pca[key].fit_transform(w_trajs[key])
               
            pca_vel[key] = PCA(n_components=int(rbf_num))
            w_vel_trajs_pca[key] = pca_vel[key].fit_transform(w_vel_trajs[key])

            pca_x[key] = PCA(n_components= int(rbf_num))
            w_x_trajs_pca[key] = pca_x[key].fit_transform(w_x_trajs[key])
    else:
        if time_step <= 48:
            if time_step == 0:
                file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset1/'
                file_name2 = 'Phi'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                Phi = pickle.load(open(file_name4,"rb"))

                file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset1/'
                file_name2 = 'x_inputs_train_'
                file_name3 = '.pt'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                x_inputs_train = torch.load(file_name4)
                file_name2 = 'x_inputs_test_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                x_inputs_test = torch.load(file_name4)
                file_name2 = 'y_test_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                y_test = torch.load(file_name4)
                file_name2 = 'y_vel_test_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                y_vel_test = torch.load(file_name4)
                file_name2 = 'y_x_test_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                y_x_test = torch.load(file_name4)
                file_name2 = 'y_train_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                y_train = torch.load(file_name4)
                file_name2 = 'y_vel_train_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                y_vel_train = torch.load(file_name4)
                file_name2 = 'y_x_train_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                y_x_train = torch.load(file_name4)

                file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset1/'
                file_name2 = 'w_trajs_pca_'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                pca = pickle.load(open(file_name4,'rb'))
           
                file_name2 = 'w_vel_trajs_pca_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                pca_vel = pickle.load(open(file_name4,'rb'))
                file_name2 = 'w_x_trajs_pca_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                pca_x= pickle.load(open(file_name4,'rb'))
                file_name2 = 'w_u_trajs_pca_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                #pca_u = pickle.load(open(file_name4,'rb'))
                file_name2 = 'w_acc_trajs_pca_'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3
                print(file_name4)
            else:
                file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset1/'
                file_name2 = 'Phi'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                Phi = pickle.load(open(file_name4,"rb"))
                print(file_name4)

                file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset1/'
                file_name2 = 'x_inputs_train_'
                file_name3 = '.pt'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                x_inputs_train = torch.load(file_name4)
                file_name2 = 'x_inputs_test_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                x_inputs_test = torch.load(file_name4)
                file_name2 = 'y_test_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                y_test = torch.load(file_name4)
                file_name2 = 'y_vel_test_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                y_vel_test = torch.load(file_name4)
                file_name2 = 'y_x_test_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                y_x_test = torch.load(file_name4)
                file_name2 = 'y_train_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                y_train = torch.load(file_name4)
                file_name2 = 'y_vel_train_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                y_vel_train = torch.load(file_name4)
                file_name2 = 'y_x_train_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                y_x_train = torch.load(file_name4)

                file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset1/'
                file_name2 = 'w_trajs_pca_'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                pca = pickle.load(open(file_name4,'rb'))
           
                file_name2 = 'w_vel_trajs_pca_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                pca_vel = pickle.load(open(file_name4,'rb'))
                file_name2 = 'w_x_trajs_pca_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                pca_x= pickle.load(open(file_name4,'rb'))
                file_name2 = 'w_u_trajs_pca_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
                #pca_u = pickle.load(open(file_name4,'rb'))
                file_name2 = 'w_acc_trajs_pca_'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3
        elif time_step < 99:
            print(time_step)
            print(naming[time_step-49])
            file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset2/ssp2/'
            file_name2 = 'Phi'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49]+ file_name3
            print(file_name4)
            Phi = pickle.load(open(file_name4,"rb"))
           
            file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset2/ssp2/'
            file_name2 = 'w_trajs_pca_Early_'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49]+ file_name3
            pca = pickle.load(open(file_name4,'rb'))
           
            file_name2 = 'w_vel_trajs_pca_Early_'
            file_name4 = file_name  +file_name2+ naming[time_step-49]+ file_name3
            pca_vel = pickle.load(open(file_name4,'rb'))
            file_name2 = 'w_x_trajs_pca_Early_'
            file_name4 = file_name  +file_name2+ naming[time_step-49]+ file_name3
            pca_x= pickle.load(open(file_name4,'rb'))
        else:
            print(["ssp1",naming[time_step-49]])
            file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset2/ssp1/'
            file_name2 = 'Phinew'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49]+ file_name3
            Phi = pickle.load(open(file_name4,"rb"))
           
            file_name = '/home/jhk/kino_dynamic_learning/dataset/dataset2/ssp1/'
            file_name2 = 'w_trajs_pca_Early_new_'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49]+ file_name3
            pca = pickle.load(open(file_name4,'rb'))
            file_name2 = 'w_vel_trajs_pca_Early_new_'
            file_name4 = file_name  +file_name2+ naming[time_step-49]+ file_name3
            pca_vel = pickle.load(open(file_name4,'rb'))
            file_name2 = 'w_x_trajs_pca_Early_new_'
            file_name4 = file_name  +file_name2+ naming[time_step-49]+ file_name3
            pca_x= pickle.load(open(file_name4,'rb'))

   
    device = 'cpu'
    '''
    train_y = timeseries(x_inputs_train[key], y_train[key])
    test_y = timeseries(x_inputs_test[key], y_test[key])
    train_yvel = timeseries(x_inputs_train[key], y_vel_train[key])
    test_yvel = timeseries(x_inputs_test[key], y_vel_test[key])
    train_yx = timeseries(x_inputs_train[key], y_x_train[key])
    test_yx = timeseries(x_inputs_test[key], y_x_test[key])
   
    batch_size = 1
    train_loader = torch.utils.data.DataLoader(dataset=train_y, batch_size=batch_size, shuffle=True)
    test_loader = torch.utils.data.DataLoader(dataset=test_y, batch_size=batch_size, shuffle=True)
    train_vel_loader = torch.utils.data.DataLoader(dataset=train_yvel, batch_size=batch_size, shuffle=True)
    test_vel_loader = torch.utils.data.DataLoader(dataset=test_yvel, batch_size=batch_size, shuffle=True)
    train_x_loader = torch.utils.data.DataLoader(dataset=train_yx, batch_size=batch_size, shuffle=True)
    test_x_loader = torch.utils.data.DataLoader(dataset=test_yx, batch_size=batch_size, shuffle=True)
    '''
    #q
    input_size = 43
    if time_step <= 48:
        model = CNN(input_size=input_size,
                    output_size = rbf_num,
                    device=device).to(device)
       
        #qdot
        model1 = CNN(input_size=input_size,
                    output_size = rbf_num,
                    device=device).to(device)

        #x
        model2 = CNN(input_size=input_size,
                    output_size = rbf_num,
                    device=device).to(device)
    else:
        model = CNN1(input_size=input_size,
                    output_size = rbf_num,
                    device=device).to(device)
       
        #qdot
        model1 = CNN1(input_size=input_size,
                    output_size = rbf_num,
                    device=device).to(device)

        #x
        model2 = CNN1(input_size=input_size,
                    output_size = rbf_num,
                    device=device).to(device)

    if learn_type == 0:
        model.train()
        model1.train()
        model2.train()
       
    else:
        if time_step <= 48:
            if time_step == 0:
                file_name = '/home/jhk/ssd_mount/cnn'
                file_name2 = '0_'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3  
                model.load_state_dict(torch.load(file_name4))
                file_name2 = '1_'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3  
                model1.load_state_dict(torch.load(file_name4))
                file_name2 = '2_'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(1)+ file_name3  
                model2.load_state_dict(torch.load(file_name4))
            else:
                file_name = '/home/jhk/ssd_mount/cnn'
                file_name2 = '0_'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3  
                model.load_state_dict(torch.load(file_name4))
                file_name2 = '1_'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3  
                model1.load_state_dict(torch.load(file_name4))
                file_name2 = '2_'
                file_name3 = '.pkl'
                file_name4 = file_name  +file_name2+ str(time_step)+ file_name3  
            model2.load_state_dict(torch.load(file_name4))
        elif time_step < 99:
            file_name = '/home/jhk/ssd_mount/beforedata/ssp2/cnnEarly'
            file_name2 = '0_'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49] + file_name3  
            model.load_state_dict(torch.load(file_name4))
            file_name2 = '1_'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49] + file_name3  
            model1.load_state_dict(torch.load(file_name4))
            file_name2 = '2_'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49] + file_name3  
            model2.load_state_dict(torch.load(file_name4))
        else:
            file_name = '/home/jhk/ssd_mount/beforedata/ssp1/cnnEarlynew_'
            file_name2 = '0_'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49] + file_name3  
            model.load_state_dict(torch.load(file_name4))
            file_name2 = '1_'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49] + file_name3  
            model1.load_state_dict(torch.load(file_name4))
            file_name2 = '2_'
            file_name3 = '.pkl'
            file_name4 = file_name  +file_name2+ naming[time_step-49] + file_name3  
            model2.load_state_dict(torch.load(file_name4))


    PCA_.append(pca[key])
    PCA_VEL.append(pca_vel[key])
    PCA_X.append(pca_x[key])

    NN_.append(model)
    NN_VEL.append(model1)
    NN_X.append(model2)
   
    if time_step == 0:
        global X_INIT
        X_INIT = x_inputs_test[key]

    PHI_.append(Phi)

def talker():
    global xs_pca_test, xs_pca, us_pca, rbf_num, talk
    global q_traj, v_traj, a_traj, x_traj, u_traj, param
    global PCA_, NN_, PCA_VEL, NN_VEL, PCA_X, NN_X, X_INIT, PHI_
    global zmp_refxssp2, zmp_refyssp2, array_boundxssp2, array_boundyssp2, zmp_refy, zmp_refx, array_boundLF, array_boundRF, array_boundx, array_boundy, array_boundRFssp2, array_boundLFssp2, array_boundxssp1, array_boundxssp1, array_boundRFssp1, array_boundLFssp1
   
    param = [
        [2,1,19,47],#0
        [2,1,19,47],#1
        [2,1,19,47],#2
        [2,1,19,47],#3
        [2,1,19,47],#4
        [2,1,19,47],#5
        [2,1,19,47],#6
        [2,1,19,47],#7
        [2,1,19,47],#8
        [2,1,19,47],#9
        [2,1,19,47],#10
        [2,1,19,47],#11
        [2,1,19,47],#12
        [2,1,19,47],#13
        [2,1,19,47],#14
        [2,1,19,47],#15
        [2,1,19,47],#16
        [2,1,19,47],#17
        [2,1,19,47],#18
        [2,1,19,47],#19
        [2,1,19,47],#20
        [2,1,19,47],#21
        [2,1,19,47],#22
        [2,1,19,47],#23
        [2,1,19,47],#24
        [2,1,19,47],#25
        [2,1,19,47],#26
        [2,1,19,47],#27
        [2,1,19,47],#28
        [2,1,19,47],#29
        [2,1,19,47],#30
        [2,1,19,47],#31
        [2,1,19,47],#32
        [2,1,19,47],#33
        [2,1,19,47],#34
        [2,1,19,47],#35
        [2,1,19,47],#36
        [2,1,19,47],#37
        [2,1,19,47],#38
        [2,1,19,47],#39
        [2,1,19,47],#40
        [2,1,19,47],#41
        [2,1,19,47],#42
        [2,1,19,47],#43
        [2,1,19,47],#44
        [2,1,19,47],#45
        [2,1,19,47],#46
        [2,1,19,47],#47
        [2,1,19,47],#48
       
        ##SSP2
        [2,1,19,55], #0
        [2,1,13,55], #1
        [1,1,33,52], #2
        [1,1,45,55], #3
        [1,1,3,54], #4
        [1,1,13,54], #5
        [1,1,27,54], #6
        [1,1,35,53], #7
        [2,1,17,55], #8
        [2,1,19,54], #9
        [2,1,3,53], #10
        #[2,1,23,53], #11
        [1,1,27,54], #12
        [1,1,27,54], #12
        #[2,1,23,53], #13
        [1,1,33,55], #14
        [1,1,33,55], #14
        [1,1,23,55], #15
        [1,1,33,55], #16
        [1,1,35,54], #17
        #[2,1,49,53], #18
        #[2,1,3,53], #19
        [2,1,25,52], #21
        [2,1,25,52], #21
        [2,1,25,52], #20
        [2,1,25,52], #21
        [1,1,19,54], #22
        [1,1,13,55], #23
        [1,1,39,55], #24
        [3,1,35,55], #25
        [1,1,19,54], #26
        [1,1,19,54], #27
        [2,1,49,53], #28
        [1,1,27,54], #29
        [2,1,13,55], #30
        [1,1,17,53], #31
        [2,1,15,52], #32
        [1,1,7,55], #33
        [1,1,45,55], #34
        [1,1,7,55], #35
        [1,1,23,55], #36
        [1,1,49,53], #37
        [2,1,7,55], #38
        [3,1,17,53], #39
        [1,1,13,55], #40
        [2,1,35,54], #41
        [2,1,7,53], #42
        [1,1,19,55], #43
        [2,1,43,53], #44
        [1,1,41,55], #45
        [2,1,11,53], #46
        [2,1,21,52], #47
        [2,1,11,53], #48
        [2,1,47,53], #49
       
        ##SSP1
        [2,1,19,55], #0
        [2,1,13,55], #1
        [1,1,33,52], #2
        [1,1,45,55], #3
        [1,1,3,54], #4
        [1,1,13,54], #5
        [1,1,27,54], #6
        [1,1,35,53], #7
        [2,1,17,55], #8
        [2,1,19,54], #9
        [2,1,3,53], #10
        [2,1,23,53], #11
        [1,1,27,54], #12
        [2,1,23,53], #13
        [2,1,23,53], #13
        #[1,1,33,55], #14
        [1,1,23,55], #15
        [1,1,33,55], #16
        [1,1,35,54], #17
        #[1,1,35,54], #17 65
        [2,1,49,53], #18 66
        [2,1,3,53], #19 67
        #[1,1,15,54], #20 68
        [2,1,25,52], #21
        [2,1,25,52], #21
        #[1,1,19,54], #22
        [1,1,13,55], #23
        [1,1,13,55], #23
        [1,1,39,55], #24
        [3,1,35,55], #25
        [1,1,19,54], #26
        [1,1,19,54], #27
        [2,1,49,53], #28
        [1,1,27,54], #29
        [2,1,13,55], #30
        [1,1,17,53], #31
        [2,1,15,52], #32
        [1,1,7,55], #33
        [1,1,45,55], #34
        #[1,1,7,55], #35
        [1,1,23,55], #36
        [1,1,23,55], #36
        [1,1,49,53], #37
        [2,1,7,55], #38
        [3,1,17,53], #39
        [1,1,13,55], #40
        [2,1,35,54], #41
        [2,1,7,53], #42
        [1,1,19,55], #43
        [2,1,43,53], #44
        [1,1,41,55], #45
        [2,1,11,53], #46
        [2,1,21,52], #47
        [2,1,11,53], #48
        [2,1,47,53], #49
    ]
    PCA_ = []
    NN_ = []
    PCA_VEL = []
    NN_VEL = []
    PCA_X = []
    NN_X = []
    PHI_= []
    X_INIT = np.array([])
    N = 60
    T = 1
    MAXITER = 300
    dt_ = 1.2 / float(N)
    total_time = 255

    crocs_data = dict()
    crocs_data['left'] = dict()
    crocs_data['Right'] = dict()

    for key in crocs_data.keys():
        crocs_data[key]['foot_poses'] = []
        crocs_data[key]['trajs'] = []
        crocs_data[key]['acc_trajs'] = []
        crocs_data[key]['x_inputs'] = []
        crocs_data[key]['vel_trajs'] = []
        crocs_data[key]['x_state'] = []        
        crocs_data[key]['u_trajs'] = []
        crocs_data[key]['data_phases_set'] = []
        crocs_data[key]['costs'] = []
        crocs_data[key]['iters'] = []
   
    for i in range(0, 149):
        print("learning")
        print(i)
        PCAlearning(i)
   
    constraint()

    T = 1
    MAXITER = 300
    dt_ = 1.2 / float(N)
    k = 1
    k1 = 1
    k3 = 1
    #/usr/local/lib/python3.8/dist-packages/robot_properties_tocabi/resources/urdf/tocabi.urdf
    global model, foot_distance, data, LFframe_id, RFframe_id, PELVjoint_id, LHjoint_id, RHjoint_id, LFjoint_id, q_init, RFjoint_id, LFcframe_id, RFcframe_id, q, qdot, qddot, LF_tran, RF_tran, PELV_tran, LF_rot, RF_rot, PELV_rot, qdot_z, qddot_z, HRR_rot_init, HLR_rot_init, HRR_tran_init, HLR_tran_init, LF_rot_init, RF_rot_init, LF_tran_init, RF_tran_init, PELV_tran_init, PELV_rot_init, CPELV_tran_init, q_command, qdot_command, qddot_command, robotIginit, q_c
    model = RobotWrapper.BuildFromURDF("/usr/local/lib/python3.8/dist-packages/robot_properties_tocabi/resources/urdf/tocabi.urdf","/home/jhk/catkin_ws/src/dyros_tocabi_v2/tocabi_description/meshes",pinocchio.JointModelFreeFlyer())  
   
    pi = 3.14159265359
   
    jointsToLock = ['Waist1_Joint', 'Neck_Joint', 'Head_Joint',
    'L_Shoulder1_Joint', 'L_Shoulder2_Joint', 'L_Shoulder3_Joint', 'L_Armlink_Joint', 'L_Elbow_Joint', 'L_Forearm_Joint', 'L_Wrist1_Joint', 'L_Wrist2_Joint',
    'R_Shoulder1_Joint', 'R_Shoulder2_Joint', 'R_Shoulder3_Joint', 'R_Armlink_Joint', 'R_Elbow_Joint', 'R_Forearm_Joint', 'R_Wrist1_Joint', 'R_Wrist2_Joint']
    # Get the joint IDs
    jointsToLockIDs = []
   
    for jn in range(len(jointsToLock)):
        jointsToLockIDs.append(model.model.getJointId(jointsToLock[jn]))
    # Set initial configuration
   
    fixedJointConfig = np.matrix([0, 0, 0.82473, 0, 0, 0, 1,
    0.0, 0.0, -0.55, 1.26, -0.71, 0.0,
    0.0, 0.0, -0.55, 1.26, -0.71, 0.0,
    0, 0.0, 0.0,
    0.2, 0.6, 1.5, -1.47, -1, 0 ,-1, 0,
    0, 0,
    -0.2, -0.6 ,-1.5, 1.47, 1, 0, 1, 0]).T

    model = RobotWrapper.buildReducedRobot(model, jointsToLockIDs, fixedJointConfig)
    pi = 3.14159265359
   
    q = pinocchio.utils.zero(model.nq)
    qdot = pinocchio.utils.zero(model.nv)
    qdot_init = pinocchio.utils.zero(model.nv)
    qddot = pinocchio.utils.zero(model.nv)
    q_init = [0, 0, 0.82473, 0, 0, 0, 1, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0]

    state = crocoddyl.StateKinodynamic(model.model)
    actuation = crocoddyl.ActuationModelKinoBase(state)
    x0 = np.array([0.] * (state.nx + 8))
    u0 = np.array([0.] * (22))
    for i in range(0,len(q_init)):
        x0[i] = q_init[i]
        q[i] = q_init[i]
   
    RFjoint_id = model.model.getJointId("R_AnkleRoll_Joint")
    LFjoint_id = model.model.getJointId("L_AnkleRoll_Joint")
    LFframe_id = model.model.getFrameId("L_Foot_Link")
    RFframe_id = model.model.getFrameId("R_Foot_Link")  
    Pelvis_id = model.model.getFrameId("Waist1_Link")    
    data = model.model.createData()
    data1 = model.model.createData()

    pinocchio.forwardKinematics(model.model, data, q, qdot)
    pinocchio.updateFramePlacements(model.model,data)
    pinocchio.centerOfMass(model.model, data, q, False)
    pinocchio.computeCentroidalMomentum(model.model,data,q,qdot)
    LF_tran = data.oMf[LFframe_id]
    RF_tran = data.oMf[RFframe_id]

    LF_tran = data.oMi[LFjoint_id]
    RF_tran = data.oMi[RFjoint_id]

    x0[41] = data.com[0][0]
    x0[43] = data.com[0][0]
    x0[45] = data.com[0][1]
    x0[47] = data.com[0][1]

    N = 50
    T = 1
    mpc_cycle = 0
   
    MAXITER = 300
    dt_ = 1.0 / float(N)
    
    '''
    weight_quad_camx = 2.9
    weight_quad_camy = 2.9
    weight_quad_zmp = np.array([0.01, 0.01])#([weight_quad_zmpx] + [weight_quad_zmpy])
    weight_quad_zmp1 = np.array([3.0, 3.0])#np.array([3.0, 3.0]) ##5, 10
    weight_quad_zmp2 = np.array([3.0, 10.0])#np.array([3.0, 10.0]) ##11
    weight_quad_cam = np.array([0.03, 0.03])#([0.008, 0.008])([weight_quad_camy] + [weight_quad_camx])
    weight_quad_upper = np.array([25.0, 25.0])
    weight_quad_pelvis = np.array([130.0, 130.0, 0.005])
    weight_quad_com = np.array([30.0, 30.0, 5.0])#([weight_quad_comx] + [weight_quad_comy] + [weight_quad_comz])
    weight_quad_rf = np.array([13.0, 3.0, 5.0, 0.2, 0.2, 0.2])#np.array([weight_quad_rfx] + [weight_quad_rfy] + [weight_quad_rfz] + [weight_quad_rfroll] + [weight_quad_rfpitch] + [weight_quad_rfyaw])
    weight_quad_lf = np.array([13.0, 3.0, 5.0, 0.2, 0.2, 0.2])#np.array([weight_quad_lfx] + [weight_quad_lfy] + [weight_quad_lfz] + [weight_quad_lfroll] + [weight_quad_lfpitch] + [weight_quad_lfyaw])
    lb_ = np.ones([2, N])
    ub_ = np.ones([2, N])
    weight_quad_cp = np.array([0.001, 0.001])
    '''

    weight_quad_camx = 2.9
    weight_quad_camy = 2.9
    weight_quad_zmp = np.array([0.01, 0.01])#([weight_quad_zmpx] + [weight_quad_zmpy])
    weight_quad_zmp1 = np.array([3.0, 3.0])#np.array([3.0, 3.0]) ##5, 10
    weight_quad_zmp2 = np.array([10.0, 15.0])#np.array([3.0, 10.0]) ##11
    weight_quad_cam = np.array([0.03, 0.03])#([0.008, 0.008])([weight_quad_camy] + [weight_quad_camx])
    weight_quad_upper = np.array([25.0, 25.0])
    weight_quad_pelvis = np.array([130.0, 130.0, 0.005])
    weight_quad_com = np.array([30.0, 30.0, 5.0])#([weight_quad_comx] + [weight_quad_comy] + [weight_quad_comz])
    weight_quad_rf = np.array([13.0, 3.0, 5.0, 0.2, 0.2, 0.2])#np.array([weight_quad_rfx] + [weight_quad_rfy] + [weight_quad_rfz] + [weight_quad_rfroll] + [weight_quad_rfpitch] + [weight_quad_rfyaw])
    weight_quad_lf = np.array([13.0, 3.0, 5.0, 0.2, 0.2, 0.2])#np.array([weight_quad_lfx] + [weight_quad_lfy] + [weight_quad_lfz] + [weight_quad_lfroll] + [weight_quad_lfpitch] + [weight_quad_lfyaw])
    lb_ = np.ones([2, N])
    ub_ = np.ones([2, N])
    weight_quad_cp = np.array([80.0, 80.0])
    
    actuation_vector = [None] * (N)
    state_vector = [None] * (N)
    state_bounds = [None] * (N)
    state_bounds2 = [None] * (N)
    state_bounds3 = [None] * (N)
    state_activations = [None] * (N)
    state_activations1 = [None] * (N)
    state_activations2 = [None] * (N)
    state_activations3 = [None] * (N)
    xRegCost_vector = [None] * (N)
    uRegCost_vector = [None] * (N)
    stateBoundCost_vector = [None] * (N)
    stateBoundCost_vector1 = [None] * (N)
    stateBoundCost_vector2 = [None] * (N)
    stateBoundCost_vector3 = [None] * (N)
    camBoundCost_vector = [None] *  (N)
    comBoundCost_vector = [None] *  (N)
    rf_foot_pos_vector = [None] *  (N)
    lf_foot_pos_vector = [None] *  (N)
    pelvis_rot_vector = [None] *  (N)
    residual_FrameRF = [None] *  (N)
    residual_FramePelvis = [None] *  (N)
    residual_FrameLF = [None] *  (N)
    PelvisR = [None] *  (N)
    foot_trackR = [None] *  (N)
    foot_trackL = [None] *  (N)
    runningCostModel_vector = [None] * (N-1)
    runningDAM_vector = [None] * (N-1)
    runningModelWithRK4_vector = [None] * (N-1)
    xs = [None] * (N)
    us = [None] * (N-1)
    traj_= np.array([0.] * (state.nx + 8))
   
    rbf_num = 47
    tick = shared_memory.SharedMemory(create=True, size=sys.getsizeof(1))
    tick.buf[0] = 0
   
    time.sleep(1)

    print("Python start")
    mpc_signal = sysv_ipc.SharedMemory(1)
    mpc_signalv  = mpc_signal.read()
    mpc_signaldata =  np.ndarray(shape=(3,), dtype=np.int32, buffer=mpc_signalv)
    x_init = sysv_ipc.SharedMemory(2)
    x_initv  = x_init.read()
    statemachine = sysv_ipc.SharedMemory(3)
    statemachinedata = np.array([0, 0, 0], dtype=np.int8)
    statemachine.write(statemachinedata)
    desired_value = sysv_ipc.SharedMemory(4)
     
    thread_manager1 = []
    for i in range(0,3):
        thread_manager1.append(0)
   
    thread_manager = multiprocessing.Array(ctypes.c_int, thread_manager1)

    time_step = 0
    k = 1
    k3 = 1
    total_cost = []
    total_time_ = []
    cp_err = []
   
    tick = shared_memory.SharedMemory(create=True, size=sys.getsizeof(1))
    tick.buf[0] = 0
    X_temp = np.zeros(43)
    X_temp = X_temp.reshape(1,1,43)
    queue = multiprocessing.Array(ctypes.c_float, np.shape(X_temp)[2])
   
    a = np.ones([60,21])
    shm_q_traj = shared_memory.SharedMemory(create=True, size=a.nbytes)
    a = np.ones([60,20])
    shm_v_traj = shared_memory.SharedMemory(create=True, size=a.nbytes)
    a = np.ones([59,20])
    shm_acc_traj = shared_memory.SharedMemory(create=True, size=a.nbytes)
    a = np.ones([60,8])
    shm_x_traj = shared_memory.SharedMemory(create=True, size=a.nbytes)
    a = np.ones([59,4])
    shm_u_traj = shared_memory.SharedMemory(create=True, size=a.nbytes)
   
    KK = 0
    signal = False

    q_traj = np.ndarray(shape=(60,21), dtype=np.float32, buffer=shm_q_traj.buf)
    v_traj = np.ndarray(shape=(60,20), dtype=np.float32, buffer=shm_v_traj.buf)
    a_traj = np.ndarray(shape=(59,20), dtype=np.float32, buffer=shm_acc_traj.buf)
    x_traj = np.ndarray(shape=(60,8), dtype=np.float32, buffer=shm_x_traj.buf)
    u_traj = np.ndarray(shape=(59,4), dtype=np.float32, buffer=shm_u_traj.buf)
   
    p1 = multiprocessing.Process(target=InversePCA, args=(NN_, param, PCA_, PHI_, tick, queue, thread_manager))
    p2 = multiprocessing.Process(target=InversePCA1, args=(NN_VEL, param, PCA_VEL, PHI_, tick, queue, thread_manager))
    p3 = multiprocessing.Process(target=InversePCA2, args=(NN_X, param, PCA_X, PHI_, tick, queue, thread_manager))
   
    p1.start()
    p2.start()
    p3.start()


    capturePoint_ref_for = np.zeros([111, (state.nx + 8)])
    capturePoint_ref_ssp1 = np.zeros([111, (state.nx + 8)])
    capturePoint_ref_ssp2 = np.zeros([111, (state.nx + 8)])
    capturePoint_ref_ssp2_1 = np.zeros([111, (state.nx + 8)])
    
    z_c = data.com[0][2]
    g = 9.81
    dt = 0.02
    t_step = 1.0 # timing for one step
    t_preview = 1.2 # timing for preview
    N_preview = int(t_preview/dt) # preview length
    
    i = 0
    time_step = 0
    ZMP_x_ref = []
    ZMP_y_ref = []
    
    zmp_offset = [-0.05, -0.00]

    for t in np.arange(0, len(array_boundx)):
        ZMP_x_ref.append((array_boundx[t + time_step][0] + array_boundx[t + time_step][1])/2 + zmp_offset[0])
        if((array_boundy[t + time_step][0] + array_boundy[t + time_step][1])/2 > 0):
            ZMP_y_ref.append((array_boundy[t + time_step][0] + array_boundy[t + time_step][1])/2 - zmp_offset[1])
        else:
            ZMP_y_ref.append((array_boundy[t + time_step][0] + array_boundy[t + time_step][1])/2 + zmp_offset[1])
        
    A = np.mat(([1, dt, dt**2/2],
            [0, 1, dt],
            [0, 0, 1]))
    B = np.mat((dt**3/6, dt**2/2, dt)).T
    C = np.mat((1, 0, -z_c/g))
    Q = 1
    R = 1e-6

    # Calculate Preview control parameters
    K, f = calculatePreviewControlParams(A, B, C, Q, R, N_preview)

    N_simulation = int(2.2/dt)
    ux_1 = np.asmatrix(np.zeros((N_simulation, 1)))
    uy_1 = np.asmatrix(np.zeros((N_simulation, 1)))
    COM_x_1 = np.asmatrix(np.zeros((3, N_simulation+1)))
    COM_y_1 = np.asmatrix(np.zeros((3, N_simulation+1)))
    COM_x_2 = np.asmatrix(np.zeros((3, N_simulation+1)))
    COM_y_2 = np.asmatrix(np.zeros((3, N_simulation+1)))

    for k in range(0, N_simulation+1):
        COM_x_1[0, k] = data.com[0][0] - 0.02
        COM_y_1[0, k] = data.com[0][1] + 0.008
        
    # record data for plot
    COM_x_record_1 = []
    COM_y_record_1 = []
    ZMP_x_record_1 = []
    ZMP_y_record_1 = []
    
    for k in range(N_simulation):
        ZMP_x_preview = np.asmatrix(ZMP_x_ref[k:k+N_preview]).T
        ZMP_y_preview = np.asmatrix(ZMP_y_ref[k:k+N_preview]).T

        ZMP_x = C*COM_x_1[:,k]
        ZMP_y = C*COM_y_1[:,k]
        
        ux_1[k] = -K*COM_x_1[:, k] + f*ZMP_x_preview
        uy_1[k] = -K*COM_y_1[:, k] + f*ZMP_y_preview

        COM_x_1[:,k+1] = A*COM_x_1[:, k] + B*ux_1[k]
        COM_y_1[:,k+1] = A*COM_y_1[:, k] + B*uy_1[k]
        
        capturePoint_ref_for[k][0] = COM_x_1[0,k] + COM_x_1[1,k]/np.sqrt(g/z_c)
        capturePoint_ref_for[k][state.nx + 7] = COM_y_1[0,k] + COM_y_1[1,k]/np.sqrt(g/z_c)

        if(k > 2):
            capturePoint_ref_for[k-3][1] = ZMP_x
            capturePoint_ref_for[k-3][2] = ZMP_y
       
        if k == 50:
            COM_x_2  = copy(COM_x_1) 
            COM_y_2  = copy(COM_y_1)
            
        if k <= 50:  
            COM_x_record_1.append([capturePoint_ref_for[k][0],capturePoint_ref_for[k][state.nx + 7], COM_x_1[0,k], COM_y_1[0,k],COM_x_1[1,k], COM_y_1[1,k], COM_x_1[2,k], COM_y_1[2,k],ZMP_x[0,0], ZMP_y[0,0]])

    
    COM_x_1 = np.asmatrix(np.zeros((3, N_simulation+1)))
    COM_y_1 = np.asmatrix(np.zeros((3, N_simulation+1)))

    COM_x_1[:,0] = COM_x_2[:,50]
    COM_y_1[:,0] = COM_y_2[:,50]
    COM_x_1[0,0] = COM_x_1[0,0] - 0.04957
    
    ZMP_x_ref = []
    ZMP_y_ref = []

    for t in np.arange(0, len(array_boundx)):
        ZMP_x_ref.append((array_boundxssp2[t + time_step][0] + array_boundxssp2[t + time_step][1])/2 + zmp_offset[0]) 
        #ZMP_y_ref.append((array_boundyssp2[t + time_step][0] + array_boundyssp2[t + time_step][1])/2)

        if((array_boundyssp2[t + time_step][0] + array_boundyssp2[t + time_step][1])/2 > 0):
            ZMP_y_ref.append((array_boundyssp2[t + time_step][0] + array_boundyssp2[t + time_step][1])/2 - zmp_offset[1])
        else:
            ZMP_y_ref.append((array_boundyssp2[t + time_step][0] + array_boundyssp2[t + time_step][1])/2 + zmp_offset[1])
        

    for k in range(N_simulation):
        ZMP_x_preview = np.asmatrix(ZMP_x_ref[k:k+N_preview]).T
        ZMP_y_preview = np.asmatrix(ZMP_y_ref[k:k+N_preview]).T

        ZMP_x = C*COM_x_1[:,k]
        ZMP_y = C*COM_y_1[:,k]
        
        ux_1[k] = -K*COM_x_1[:, k] + f*ZMP_x_preview
        uy_1[k] = -K*COM_y_1[:, k] + f*ZMP_y_preview

        COM_x_1[:,k+1] = A*COM_x_1[:, k] + B*ux_1[k]
        COM_y_1[:,k+1] = A*COM_y_1[:, k] + B*uy_1[k]

        capturePoint_ref_ssp2[k][0] = COM_x_1[0,k] + COM_x_1[1,k]/np.sqrt(g/z_c)
        capturePoint_ref_ssp2[k][state.nx + 7] = COM_y_1[0,k] + COM_y_1[1,k]/np.sqrt(g/z_c)

        capturePoint_ref_ssp2[k][1] = ZMP_x
        capturePoint_ref_ssp2[k][2] = ZMP_y

        if k == 50:
            COM_x_2  = COM_x_1 
            COM_y_2  = COM_y_1
        if k <= 50:  
            COM_x_record_1.append([capturePoint_ref_ssp2[k][0] + 0.04957, COM_y_1[0,k],COM_x_1[1,k], COM_y_1[1,k], COM_x_1[2,k], COM_y_1[2,k],ZMP_x[0,0] + 0.04957, ZMP_y[0,0]])
    
    COM_x_1 = np.asmatrix(np.zeros((3, N_simulation+1)))
    COM_y_1 = np.asmatrix(np.zeros((3, N_simulation+1)))

    COM_x_1[:,0] = COM_x_2[:,50]
    COM_y_1[:,0] = COM_y_2[:,50]

    COM_x_1[0,0] = COM_x_1[0,0] - 0.1

    ZMP_x_ref = []
    ZMP_y_ref = []

    for t in np.arange(0, len(array_boundx)):
        ZMP_x_ref.append((array_boundxssp1[t + time_step][0] + array_boundxssp1[t + time_step][1])/2 + zmp_offset[0]) 
        #ZMP_y_ref.append((array_boundyssp1[t + time_step][0] + array_boundyssp1[t + time_step][1])/2)
        if((array_boundyssp1[t + time_step][0] + array_boundyssp1[t + time_step][1])/2 > 0):
            ZMP_y_ref.append((array_boundyssp1[t + time_step][0] + array_boundyssp1[t + time_step][1])/2 - zmp_offset[1])
        else:
            ZMP_y_ref.append((array_boundyssp1[t + time_step][0] + array_boundyssp1[t + time_step][1])/2 + zmp_offset[1])
        

    for k in range(N_simulation):
        ZMP_x_preview = np.asmatrix(ZMP_x_ref[k:k+N_preview]).T
        ZMP_y_preview = np.asmatrix(ZMP_y_ref[k:k+N_preview]).T

        ZMP_x = C*COM_x_1[:,k]
        ZMP_y = C*COM_y_1[:,k]
        
        ux_1[k] = -K*COM_x_1[:, k] + f*ZMP_x_preview
        uy_1[k] = -K*COM_y_1[:, k] + f*ZMP_y_preview

        COM_x_1[:,k+1] = A*COM_x_1[:, k] + B*ux_1[k]
        COM_y_1[:,k+1] = A*COM_y_1[:, k] + B*uy_1[k]

        capturePoint_ref_ssp1[k][0] = COM_x_1[0,k] + COM_x_1[1,k]/np.sqrt(g/z_c)
        capturePoint_ref_ssp1[k][state.nx + 7] = COM_y_1[0,k] + COM_y_1[1,k]/np.sqrt(g/z_c)

        capturePoint_ref_ssp1[k][1] = ZMP_x
        capturePoint_ref_ssp1[k][2] = ZMP_y

        if k == 50:
            COM_x_2  = COM_x_1 
            COM_y_2  = COM_y_1
        if k <= 50:  
            COM_x_record_1.append([capturePoint_ref_ssp1[k][0] + 0.1 + 0.04957, COM_y_1[0,k],COM_x_1[1,k], COM_y_1[1,k], COM_x_1[2,k], COM_y_1[2,k],ZMP_x[0,0] + 0.1 + 0.04957, ZMP_y[0,0]])

    COM_x_1 = np.asmatrix(np.zeros((3, N_simulation+1)))
    COM_y_1 = np.asmatrix(np.zeros((3, N_simulation+1)))

    COM_x_1[:,0] = COM_x_2[:,50]
    COM_y_1[:,0] = COM_y_2[:,50]

    COM_x_1[0,0] = COM_x_1[0,0] - 0.1

    ZMP_x_ref = []
    ZMP_y_ref = []

    for t in np.arange(0, len(array_boundx)):
        ZMP_x_ref.append((array_boundxssp2[t + time_step][0] + array_boundxssp2[t + time_step][1])/2 + zmp_offset[0])
        if((array_boundyssp2[t + time_step][0] + array_boundyssp2[t + time_step][1])/2 > 0):
            ZMP_y_ref.append((array_boundyssp2[t + time_step][0] + array_boundyssp2[t + time_step][1])/2 - zmp_offset[1])
        else:
            ZMP_y_ref.append((array_boundyssp2[t + time_step][0] + array_boundyssp2[t + time_step][1])/2 + zmp_offset[1])

    for k in range(N_simulation):
        ZMP_x_preview = np.asmatrix(ZMP_x_ref[k:k+N_preview]).T
        ZMP_y_preview = np.asmatrix(ZMP_y_ref[k:k+N_preview]).T

        ZMP_x = C*COM_x_1[:,k]
        ZMP_y = C*COM_y_1[:,k]
        
        ux_1[k] = -K*COM_x_1[:, k] + f*ZMP_x_preview
        uy_1[k] = -K*COM_y_1[:, k] + f*ZMP_y_preview

        COM_x_1[:,k+1] = A*COM_x_1[:, k] + B*ux_1[k]
        COM_y_1[:,k+1] = A*COM_y_1[:, k] + B*uy_1[k]

        capturePoint_ref_ssp2_1[k][0] = COM_x_1[0,k] + COM_x_1[1,k]/np.sqrt(g/z_c)
        capturePoint_ref_ssp2_1[k][state.nx + 7] = COM_y_1[0,k] + COM_y_1[1,k]/np.sqrt(g/z_c)

        capturePoint_ref_ssp2_1[k][1] = ZMP_x
        capturePoint_ref_ssp2_1[k][2] = ZMP_y

        if k == 50:
            COM_x_2  = COM_x_1 
            COM_y_2  = COM_y_1
        if k <= 50:  
            COM_x_record_1.append([capturePoint_ref_ssp2_1[k][0] + 0.2 + 0.04957, COM_y_1[0,k],COM_x_1[1,k], COM_y_1[1,k], COM_x_1[2,k], COM_y_1[2,k],ZMP_x[0,0] + 0.2 + 0.04957, ZMP_y[0,0]])

    for i in range(0,N-1):
        traj_[43] = (array_boundx[i][0] + array_boundx[i][1])/2 #zmp_refx_[i][0]
        traj_[47] = (array_boundy[i][0] + array_boundy[i][1])/2#zmp_refy_[i][0]
        state_vector[i] = crocoddyl.StateKinodynamic(model.model)
        actuation_vector[i] = crocoddyl.ActuationModelKinoBase(state_vector[i])
        state_bounds[i] = crocoddyl.ActivationBounds(lb_[:,i],ub_[:,i])
        state_activations[i] = crocoddyl.ActivationModelWeightedQuadraticBarrier(state_bounds[i], weight_quad_zmp)
        #state_activations1[i] = crocoddyl.ActivationModelWeightedQuadraticBarrier(state_bounds[i], weight_quad_upper)
        stateBoundCost_vector[i] = crocoddyl.CostModelResidual(state_vector[i], state_activations[i], crocoddyl.ResidualFlyState(state_vector[i], actuation_vector[i].nu + 4))
        if i != 1:
            stateBoundCost_vector1[i] = crocoddyl.CostModelResidual(state_vector[i], crocoddyl.ActivationModelWeightedQuad(weight_quad_zmp1), crocoddyl.ResidualFlyState(state_vector[i], traj_, actuation_vector[i].nu + 4))
        else:
            stateBoundCost_vector1[i] = crocoddyl.CostModelResidual(state_vector[i], crocoddyl.ActivationModelWeightedQuad(weight_quad_zmp2), crocoddyl.ResidualFlyState(state_vector[i], traj_, actuation_vector[i].nu + 4))
       
        stateBoundCost_vector2[i] = crocoddyl.CostModelResidual(state_vector[i], crocoddyl.ActivationModelWeightedQuad(weight_quad_upper), crocoddyl.ResidualFlyState1(state_vector[i], actuation_vector[i].nu + 4))
        camBoundCost_vector[i] = crocoddyl.CostModelResidual(state_vector[i], crocoddyl.ActivationModelWeightedQuad(weight_quad_cam), crocoddyl.ResidualModelCentroidalAngularMomentum(state_vector[i], actuation_vector[i].nu + 4))
        comBoundCost_vector[i] = crocoddyl.CostModelResidual(state_vector[i], crocoddyl.ActivationModelWeightedQuad(weight_quad_com), crocoddyl.ResidualModelCoMKinoPosition(state_vector[i], np.array([0.0, 0.0, data.com[0][2]]), actuation_vector[i].nu + 4))
        rf_foot_pos_vector[i] = pinocchio.SE3.Identity()
        rf_foot_pos_vector[i].translation = copy(RF_tran.translation)
        lf_foot_pos_vector[i] = pinocchio.SE3.Identity()
        pelvis_rot_vector[i] = pinocchio.SE3.Identity().rotation
       
        lf_foot_pos_vector[i].translation = copy(LF_tran.translation)
        residual_FramePelvis[i] = crocoddyl.ResidualKinoFrameRotation(state_vector[i], Pelvis_id, pelvis_rot_vector[i], actuation_vector[i].nu + 4)
        residual_FrameRF[i] = crocoddyl.ResidualKinoFramePlacement(state_vector[i], RFframe_id, rf_foot_pos_vector[i], actuation_vector[i].nu + 4)
        residual_FrameLF[i] = crocoddyl.ResidualKinoFramePlacement(state_vector[i], LFframe_id, lf_foot_pos_vector[i], actuation_vector[i].nu + 4)
        PelvisR[i] = crocoddyl.CostModelResidual(state_vector[i], crocoddyl.ActivationModelWeightedQuad(weight_quad_pelvis), residual_FramePelvis[i])
        foot_trackR[i] = crocoddyl.CostModelResidual(state_vector[i], crocoddyl.ActivationModelWeightedQuad(weight_quad_rf), residual_FrameRF[i])
        foot_trackL[i] = crocoddyl.CostModelResidual(state_vector[i], crocoddyl.ActivationModelWeightedQuad(weight_quad_lf), residual_FrameLF[i])
        runningCostModel_vector[i] = crocoddyl.CostModelSum(state_vector[i], actuation_vector[i].nu + 4)
        
        stateBoundCost_vector3[i] = crocoddyl.CostModelResidual(state_vector[i], crocoddyl.ActivationModelWeightedQuad(weight_quad_cp), crocoddyl.ResidualFlyState2(state_vector[i], actuation_vector[i].nu + 4))

        if i >= 1:
            #runningCostModel_vector[i].addCost("stateReg", stateBoundCost_vector[i], 1.0)
            runningCostModel_vector[i].addCost("comReg", comBoundCost_vector[i], 1.0)
            runningCostModel_vector[i].addCost("camReg", camBoundCost_vector[i], 1.0)
            runningCostModel_vector[i].addCost("footReg1", foot_trackR[i], 1.0)
            runningCostModel_vector[i].addCost("footReg2", foot_trackL[i], 1.0)
            runningCostModel_vector[i].addCost("stateReg1", stateBoundCost_vector1[i], 1.0)
            runningCostModel_vector[i].addCost("stateReg2", stateBoundCost_vector2[i], 1.0)
            runningCostModel_vector[i].addCost("stateReg3", stateBoundCost_vector3[i], 1.0)
        
            runningCostModel_vector[i].addCost("pelvReg2", PelvisR[i], 1.0)
           
        runningDAM_vector[i] = crocoddyl.DifferentialActionModelKinoDynamics(state_vector[i], actuation_vector[i], runningCostModel_vector[i])
        runningModelWithRK4_vector[i] = crocoddyl.IntegratedActionModelEuler(runningDAM_vector[i], dt_)
        
    state_vector[N-1] = crocoddyl.StateKinodynamic(model.model)
    actuation_vector[N-1] = crocoddyl.ActuationModelKinoBase(state_vector[N-1])
    state_bounds[N-1] = crocoddyl.ActivationBounds(lb_[:,N-1],ub_[:,N-1])
    state_activations[N-1] = crocoddyl.ActivationModelWeightedQuadraticBarrier(state_bounds[N-1], weight_quad_zmp)
    #state_activations1[N-1] = crocoddyl.ActivationModelWeightedQuadraticBarrier(state_bounds[N-1], weight_quad_upper)
    stateBoundCost_vector[N-1] = crocoddyl.CostModelResidual(state_vector[N-1], state_activations[N-1], crocoddyl.ResidualFlyState(state_vector[N-1], actuation_vector[N-1].nu + 4))
    stateBoundCost_vector1[N-1] = crocoddyl.CostModelResidual(state_vector[N-1], crocoddyl.ActivationModelWeightedQuad(weight_quad_zmp1), crocoddyl.ResidualFlyState(state_vector[N-1], traj_, actuation_vector[N-1].nu + 4))
    stateBoundCost_vector2[N-1] = crocoddyl.CostModelResidual(state_vector[N-1], crocoddyl.ActivationModelWeightedQuad(weight_quad_upper), crocoddyl.ResidualFlyState1(state_vector[N-1], actuation_vector[N-1].nu + 4))
    camBoundCost_vector[N-1] = crocoddyl.CostModelResidual(state_vector[N-1], crocoddyl.ActivationModelWeightedQuad(weight_quad_cam), crocoddyl.ResidualModelCentroidalAngularMomentum(state_vector[N-1], actuation_vector[N-1].nu + 4))
    comBoundCost_vector[N-1] = crocoddyl.CostModelResidual(state_vector[N-1], crocoddyl.ActivationModelWeightedQuad(weight_quad_com), crocoddyl.ResidualModelCoMKinoPosition(state_vector[N-1], np.array([0.0, 0.0, data.com[0][2]]), actuation_vector[N-1].nu + 4))
    rf_foot_pos_vector[N-1] = pinocchio.SE3.Identity()
    rf_foot_pos_vector[N-1].translation = copy(RF_tran.translation)
    lf_foot_pos_vector[N-1] = pinocchio.SE3.Identity()
    lf_foot_pos_vector[N-1].translation = copy(LF_tran.translation)
    #pelvis_rot_vector[N-1] = pinocchio.SE3.Identity().rotation()
    #residual_FramePelvis[N-1] = crocoddyl.ResidualFrameRotation(state_vector[N-1], Pelvis_id, pelvis_rot_vector[N-1], actuation_vector[N-1].nu + 4)
    #PelvisR[N-1] = crocoddyl.CostModelResidual(state_vector[N-1], crocoddyl.ActivationModelWeightedQuad(weight_quad_pelvis), residual_FramePelvis[N-1])
    residual_FrameRF[N-1] = crocoddyl.ResidualKinoFramePlacement(state_vector[N-1], RFframe_id, rf_foot_pos_vector[N-1], actuation_vector[N-1].nu + 4)
    residual_FrameLF[N-1] = crocoddyl.ResidualKinoFramePlacement(state_vector[N-1], LFframe_id, lf_foot_pos_vector[N-1], actuation_vector[N-1].nu + 4)
    foot_trackR[N-1] = crocoddyl.CostModelResidual(state_vector[N-1], crocoddyl.ActivationModelWeightedQuad(weight_quad_rf), residual_FrameRF[N-1])
    foot_trackL[N-1] = crocoddyl.CostModelResidual(state_vector[N-1], crocoddyl.ActivationModelWeightedQuad(weight_quad_lf), residual_FrameLF[N-1])
   
    terminalCostModel = crocoddyl.CostModelSum(state_vector[N-1], actuation_vector[N-1].nu + 4)
    #terminalCostModel.addCost("stateReg", stateBoundCost_vector[N-1], 1.0)
    #terminalCostModel.addCost("stateReg1", stateBoundCost_vector1[N-1], 1.0)
    #terminalCostModel.addCost("stateReg2", stateBoundCost_vector2[N-1], 1.0)
    terminalCostModel.addCost("comReg", comBoundCost_vector[N-1], 1.0)
    #terminalCostModel.addCost("camReg", camBoundCost_vector[N-1], 1.0)
    terminalCostModel.addCost("footReg1", foot_trackR[N-1], 1.0)
    terminalCostModel.addCost("footReg2", foot_trackL[N-1], 1.0)
    
    terminalDAM = crocoddyl.DifferentialActionModelKinoDynamics(state_vector[N-1], actuation_vector[N-1], terminalCostModel)
    terminalModel = crocoddyl.IntegratedActionModelEuler(terminalDAM, dt_)
    problemWithRK4 = crocoddyl.ShootingProblem(x0, runningModelWithRK4_vector, terminalModel)
    problemWithRK4.nthreads = 10

    ddp = crocoddyl.SolverFDDP(problemWithRK4)
    first_time = True
    ddp.th_stop = 0.00000001#0.00000005
    #ddp.th_stop = 0.00000001
   
    for time_step in range(0, total_time):
        xs_pca = []
        us_pca = []
        xs_pca_test = []
        ok_ = False
        if(time_step < 49):
            for i in range(1, N-1):
                traj_[43] = capturePoint_ref_for[i+time_step][1]#(array_boundx[i + time_step][0] + array_boundx[i + time_step][1])/2 #zmp_refx_[i][0]
                traj_[47] = capturePoint_ref_for[i+time_step][2]#(array_boundy[i + time_step][0] + array_boundy[i + time_step][1])/2#zmp_refy_[i][0]
                #capturePoint_ref_for[i+time_step][1]#
                #capturePoint_ref_for[i+time_step][1]
                runningCostModel_vector[i].costs["stateReg1"].cost.residual.reference = copy(traj_)
                
                rf_foot_pos_vector[i].translation[0] = copy(array_boundRF[i + time_step][0])
                rf_foot_pos_vector[i].translation[1] = copy(array_boundRF[i + time_step][1])
                rf_foot_pos_vector[i].translation[2] = copy(array_boundRF[i + time_step][2])
                lf_foot_pos_vector[i].translation[0] = copy(array_boundLF[i + time_step][0])
                lf_foot_pos_vector[i].translation[1] = copy(array_boundLF[i + time_step][1])
                lf_foot_pos_vector[i].translation[2] = copy(array_boundLF[i + time_step][2])
                
                runningCostModel_vector[i].costs["footReg1"].cost.residual.reference = copy(rf_foot_pos_vector[i])
                runningCostModel_vector[i].costs["footReg2"].cost.residual.reference = copy(lf_foot_pos_vector[i])

                runningCostModel_vector[i].costs["stateReg3"].cost.residual.reference = copy(capturePoint_ref_for[i + time_step])

            rf_foot_pos_vector[N-1].translation[0] = copy(array_boundRF[N-1 + time_step][0])
            rf_foot_pos_vector[N-1].translation[1] = copy(array_boundRF[N-1 + time_step][1])
            rf_foot_pos_vector[N-1].translation[2] = copy(array_boundRF[N-1 + time_step][2])
            lf_foot_pos_vector[N-1].translation[0] = copy(array_boundLF[N-1 + time_step][0])
            lf_foot_pos_vector[N-1].translation[1] = copy(array_boundLF[N-1 + time_step][1])
            lf_foot_pos_vector[N-1].translation[2] = copy(array_boundLF[N-1 + time_step][2])
            
        elif (time_step < 99):
            time_step_ssp2 = time_step - 49
            for i in range(1, N-1):
                traj_[43] = capturePoint_ref_ssp2[i+time_step_ssp2][1]#(array_boundxssp2[i + time_step_ssp2][0] + array_boundxssp2[i + time_step_ssp2][1])/2 #zmp_refx_[i][0]
                traj_[47] = capturePoint_ref_ssp2[i+time_step_ssp2][2]#(array_boundyssp2[i + time_step_ssp2][0] + array_boundyssp2[i + time_step_ssp2][1])/2#zmp_refy_[i][0]
                
                runningCostModel_vector[i].costs["stateReg1"].cost.residual.reference = traj_
                
                rf_foot_pos_vector[i].translation[0] = copy(array_boundRFssp2[i + time_step_ssp2][0])
                rf_foot_pos_vector[i].translation[1] = copy(array_boundRFssp2[i + time_step_ssp2][1])
                rf_foot_pos_vector[i].translation[2] = copy(array_boundRFssp2[i + time_step_ssp2][2])
                lf_foot_pos_vector[i].translation[0] = copy(array_boundLFssp2[i + time_step_ssp2][0])
                lf_foot_pos_vector[i].translation[1] = copy(array_boundLFssp2[i + time_step_ssp2][1])
                lf_foot_pos_vector[i].translation[2] = copy(array_boundLFssp2[i + time_step_ssp2][2])
                
                runningCostModel_vector[i].costs["footReg1"].cost.residual.reference = rf_foot_pos_vector[i]
                runningCostModel_vector[i].costs["footReg2"].cost.residual.reference = lf_foot_pos_vector[i]
                runningCostModel_vector[i].costs["stateReg3"].cost.residual.reference = capturePoint_ref_ssp2[i + time_step_ssp2]
           
            rf_foot_pos_vector[N-1].translation[0] = copy(array_boundRFssp2[N-1 + time_step_ssp2][0])
            rf_foot_pos_vector[N-1].translation[1] = copy(array_boundRFssp2[N-1 + time_step_ssp2][1])
            rf_foot_pos_vector[N-1].translation[2] = copy(array_boundRFssp2[N-1 + time_step_ssp2][2])
            lf_foot_pos_vector[N-1].translation[0] = copy(array_boundLFssp2[N-1 + time_step_ssp2][0])
            lf_foot_pos_vector[N-1].translation[1] = copy(array_boundLFssp2[N-1 + time_step_ssp2][1])
            lf_foot_pos_vector[N-1].translation[2] = copy(array_boundLFssp2[N-1 + time_step_ssp2][2])
            
        elif(time_step < 149):
            time_step_ssp1 = time_step - 99
            for i in range(1, N-1):
                traj_[43] = capturePoint_ref_ssp1[i+time_step_ssp1][1]#(array_boundxssp1[i + time_step_ssp1][0] + array_boundxssp1[i + time_step_ssp1][1])/2 #zmp_refx_[i][0]
                traj_[47] = capturePoint_ref_ssp1[i+time_step_ssp1][2]#(array_boundyssp1[i + time_step_ssp1][0] + array_boundyssp1[i + time_step_ssp1][1])/2 #zmp_refy_[i][0]
                 
                runningCostModel_vector[i].costs["stateReg1"].cost.residual.reference = traj_
                
                rf_foot_pos_vector[i].translation[0] = copy(array_boundRFssp1[i + time_step_ssp1][0])
                rf_foot_pos_vector[i].translation[1] = copy(array_boundRFssp1[i + time_step_ssp1][1])
                rf_foot_pos_vector[i].translation[2] = copy(array_boundRFssp1[i + time_step_ssp1][2])
                lf_foot_pos_vector[i].translation[0] = copy(array_boundLFssp1[i + time_step_ssp1][0])
                lf_foot_pos_vector[i].translation[1] = copy(array_boundLFssp1[i + time_step_ssp1][1])
                lf_foot_pos_vector[i].translation[2] = copy(array_boundLFssp1[i + time_step_ssp1][2])
                
                runningCostModel_vector[i].costs["footReg1"].cost.residual.reference = rf_foot_pos_vector[i]
                runningCostModel_vector[i].costs["footReg2"].cost.residual.reference = lf_foot_pos_vector[i]
                runningCostModel_vector[i].costs["stateReg3"].cost.residual.reference = capturePoint_ref_ssp1[i + time_step_ssp1]
           
            rf_foot_pos_vector[N-1].translation[0] = copy(array_boundRFssp1[N-1 + time_step_ssp1][0])
            rf_foot_pos_vector[N-1].translation[1] = copy(array_boundRFssp1[N-1 + time_step_ssp1][1])
            rf_foot_pos_vector[N-1].translation[2] = copy(array_boundRFssp1[N-1 + time_step_ssp1][2])
            lf_foot_pos_vector[N-1].translation[0] = copy(array_boundLFssp1[N-1 + time_step_ssp1][0])
            lf_foot_pos_vector[N-1].translation[1] = copy(array_boundLFssp1[N-1 + time_step_ssp1][1])
            lf_foot_pos_vector[N-1].translation[2] = copy(array_boundLFssp1[N-1 + time_step_ssp1][2])
                    
        else:
            contactswitch, time_step_num = divmod(time_step - 149, 50)
            if(contactswitch % 2 == 0):
                for i in range(1, N-1):
                    traj_[43] = capturePoint_ref_ssp2_1[i+time_step_num][1]#(array_boundxssp2[i + time_step_num][0] + array_boundxssp2[i + time_step_num][1])/2 #zmp_refx_[i][0]#capturePoint_ref_ssp2_1[i+time_step_num][1]#
                    traj_[47] = capturePoint_ref_ssp2_1[i+time_step_num][2]#(array_boundyssp2[i + time_step_num][0] + array_boundyssp2[i + time_step_num][1])/2#zmp_refy_[i][0]#capturePoint_ref_ssp2_1[i+time_step_num][2]#
                  
                    runningCostModel_vector[i].costs["stateReg1"].cost.residual.reference = traj_
                
                    rf_foot_pos_vector[i].translation[0] = copy(array_boundRFssp2[i + time_step_num][0])
                    rf_foot_pos_vector[i].translation[1] = copy(array_boundRFssp2[i + time_step_num][1])
                    rf_foot_pos_vector[i].translation[2] = copy(array_boundRFssp2[i + time_step_num][2])
                    lf_foot_pos_vector[i].translation[0] = copy(array_boundLFssp2[i + time_step_num][0])
                    lf_foot_pos_vector[i].translation[1] = copy(array_boundLFssp2[i + time_step_num][1])
                    lf_foot_pos_vector[i].translation[2] = copy(array_boundLFssp2[i + time_step_num][2])
                
                    runningCostModel_vector[i].costs["footReg1"].cost.residual.reference = rf_foot_pos_vector[i]
                    runningCostModel_vector[i].costs["footReg2"].cost.residual.reference = lf_foot_pos_vector[i]
                    runningCostModel_vector[i].costs["stateReg3"].cost.residual.reference = capturePoint_ref_ssp2_1[i + time_step_num]
             
                rf_foot_pos_vector[N-1].translation[0] = copy(array_boundRFssp2[N-1 + time_step_num][0])
                rf_foot_pos_vector[N-1].translation[1] = copy(array_boundRFssp2[N-1 + time_step_num][1])
                rf_foot_pos_vector[N-1].translation[2] = copy(array_boundRFssp2[N-1 + time_step_num][2])
                lf_foot_pos_vector[N-1].translation[0] = copy(array_boundLFssp2[N-1 + time_step_num][0])
                lf_foot_pos_vector[N-1].translation[1] = copy(array_boundLFssp2[N-1 + time_step_num][1])
                lf_foot_pos_vector[N-1].translation[2] = copy(array_boundLFssp2[N-1 + time_step_num][2])
               
            else:
                for i in range(1, N-1):
                    traj_[43] = capturePoint_ref_ssp1[i+time_step_num][1]#(array_boundxssp1[i + time_step_num][0] + array_boundxssp1[i + time_step_num][1])/2 #zmp_refx_[i][0]
                    traj_[47] = capturePoint_ref_ssp1[i+time_step_num][2]#(array_boundyssp1[i + time_step_num][0] + array_boundyssp1[i + time_step_num][1])/2 #zmp_refy_[i][0]
  
                    runningCostModel_vector[i].costs["stateReg1"].cost.residual.reference = traj_
                
                    rf_foot_pos_vector[i].translation[0] = copy(array_boundRFssp1[i + time_step_num][0])
                    rf_foot_pos_vector[i].translation[1] = copy(array_boundRFssp1[i + time_step_num][1])
                    rf_foot_pos_vector[i].translation[2] = copy(array_boundRFssp1[i + time_step_num][2])
                    lf_foot_pos_vector[i].translation[0] = copy(array_boundLFssp1[i + time_step_num][0])
                    lf_foot_pos_vector[i].translation[1] = copy(array_boundLFssp1[i + time_step_num][1])
                    lf_foot_pos_vector[i].translation[2] = copy(array_boundLFssp1[i + time_step_num][2])
                    
                    runningCostModel_vector[i].costs["footReg1"].cost.residual.reference = rf_foot_pos_vector[i]
                    runningCostModel_vector[i].costs["footReg2"].cost.residual.reference = lf_foot_pos_vector[i]
                    runningCostModel_vector[i].costs["stateReg3"].cost.residual.reference = capturePoint_ref_ssp1[i + time_step_num]
           
                rf_foot_pos_vector[N-1].translation[0] = copy(array_boundRFssp1[N-1 + time_step_num][0])
                rf_foot_pos_vector[N-1].translation[1] = copy(array_boundRFssp1[N-1 + time_step_num][1])
                rf_foot_pos_vector[N-1].translation[2] = copy(array_boundRFssp1[N-1 + time_step_num][2])
                lf_foot_pos_vector[N-1].translation[0] = copy(array_boundLFssp1[N-1 + time_step_num][0])
                lf_foot_pos_vector[N-1].translation[1] = copy(array_boundLFssp1[N-1 + time_step_num][1])
                lf_foot_pos_vector[N-1].translation[2] = copy(array_boundLFssp1[N-1 + time_step_num][2])
                
        terminalCostModel.costs["footReg1"].cost.residual.reference = rf_foot_pos_vector[N-1]
        terminalCostModel.costs["footReg2"].cost.residual.reference = lf_foot_pos_vector[N-1]
       
        while signal == False:
            mpc_signalv  = mpc_signal.read()
            mpc_signaldata =  np.ndarray(shape=(3,), dtype=np.int32, buffer=mpc_signalv)
            if(mpc_signaldata[0] == 4):
                signal = True

        while ok_ == False:
            mpc_signalv  = mpc_signal.read()
            mpc_signaldata =  np.ndarray(shape=(3,), dtype=np.int32, buffer=mpc_signalv)
           
            if mpc_signaldata[0] == 1:
                statemachine.write(np.array([2, 0, 0], dtype=np.int8))
                x_initv  = x_init.read()
                X = np.ndarray(shape=(50,), dtype=np.float64, buffer=x_initv)
                
                if time_step == 0:
                    '''
                    X = np.array([ 0.00000000e+00,  0.00000000e+00,  8.24730000e-01,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00,
    0.00000000e+00, -5.50000000e-01,  1.26000000e+00, -7.10000000e-01,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -5.50000000e-01,
    1.26000000e+00, -7.10000000e-01,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  8.61938268e-02,  0.00000000e+00,  8.61938268e-02,
    0.00000000e+00,  5.18219890e-06,  0.00000000e+00,  5.18219890e-06,
    0.00000000e+00])
                    '''
                    queue[:41] = X[:41]
                    queue[41] = X[43]
                    queue[42] = X[47]
                   
                    for i in range(0, len(x0)):
                        x0[i] = copy(X[i])
                    
                    for i in range(0, len(q)):
                        q[i] = x0[i]    
                    for i in range(0, len(qdot)):
                        qdot[i] = x0[i+len(q)]    

                    pinocchio.centerOfMass(model.model, data, q, qdot, False)                
                    
                    '''
                    pinocchio.forwardKinematics(model.model, data, q, qdot)
                    pinocchio.updateFramePlacements(model.model,data)
                    pinocchio.centerOfMass(model.model, data, q, qdot, False)
                    pinocchio.computeCentroidalMomentum(model.model,data,q,qdot)
                    '''
                    '''
                    x0[41] = data.com[0][0]
                    x0[45] = data.com[0][1]
               
                    x0[42] = data.vcom[0][0]
                    x0[46] = data.vcom[0][1]

                    x0[43] = data.com[0][0]
                    x0[47] = 0.0#data.com[0][1]

                    x0[44] = data.hg.angular[1]
                    x0[48] = data.hg.angular[0]
                    '''
                    '''
                    X = np.array([ 0.00000000e+00,  0.00000000e+00,  8.24730000e-01,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00,
    0.00000000e+00, -5.50000000e-01,  1.26000000e+00, -7.10000000e-01,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -5.50000000e-01,
    1.26000000e+00, -7.10000000e-01,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  8.61938268e-02,  0.00000000e+00,  8.61938268e-02,
    0.00000000e+00,  5.18219890e-06,  0.00000000e+00,  5.18219890e-06,
    0.00000000e+00, data.com[0][2]])
                    '''
                    for i in range(1, N-1):
                       runningCostModel_vector[i].costs["comReg"].cost.residual.reference = np.array([0, 0, data.com[0][2]])
                    terminalCostModel.costs["comReg"].cost.residual.reference = np.array([0, 0, data.com[0][2]])

                else:
                    x0 = copy(X[:49])
                    queue[:41] = x0[:41]
                    queue[41] = x0[43]
                    queue[42] = x0[47]
               
                thread_manager[:] = [1, 1, 1]    
                problemWithRK4.x0 = x0
                
                
                while True:
                    if (thread_manager[0] == 0 and thread_manager[1] == 0 and thread_manager[2] == 0):
                        for i in range(0, N):
                            xs_pca.append(np.concatenate([q_traj[i,:3], [0, 0, 0, 1], q_traj[i,7:19], [0.0, 0.0], v_traj[i,:18], [0.0, 0.0], x_traj[i,:]]))
                            if i != N-1:
                                us_pca.append(np.concatenate([a_traj[i,:18], [0.0, 0.0], u_traj[i,:]]))
                        break
                #print(xs_pca)
                xs_pca[0] = x0
                c_start = time.time()
                css = ddp.solve(xs_pca, us_pca, 15, False, 0.0000003)
                #css = ddp.solve(xs_pca, us_pca, 10, False, 0.0000003)
                c_end = time.time()
               
                if(ddp.cost < 0.03):
                    X = ddp.xs[1]
                    desired_value.write(X)
                    
                
                statemachine.write(np.array([1, 0, 0], dtype=np.int8)) #, 
                cp_err.append([time_step,runningCostModel_vector[1].costs["stateReg3"].cost.residual.reference[0], runningCostModel_vector[1].costs["stateReg1"].cost.residual.reference[43], runningCostModel_vector[1].costs["footReg1"].cost.residual.reference.translation[0], runningCostModel_vector[1].costs["footReg2"].cost.residual.reference.translation[0], (ddp.xs[1][41]+ddp.xs[1][42]/3.51462),(x0[41]+x0[42]/3.51462), runningCostModel_vector[1].costs["stateReg3"].cost.residual.reference[48], (ddp.xs[1][45]+ddp.xs[1][46]/3.51462),(x0[45]+ x0[46]/3.51462), x0[41], x0[45], ddp.xs[1][41], ddp.xs[1][45],ddp.xs[1][42], ddp.xs[1][46], ddp.xs[1][43], ddp.xs[1][47],x0[43], x0[47]])
                duration = (1e3 * (c_end - c_start))

                #if(time_step > 208):
                print('ddp.iter {0},{1},{2},{3},{4}'.format(time_step, ddp.iter, duration, css, ddp.cost))
                    #print([runningCostModel_vector[1].costs["stateReg3"].cost.residual.reference, ddp.xs[1][41]+ddp.xs[1][42]/3.51462, ddp.xs[1][45]+ddp.xs[1][46]/3.51462])
                total_time_.append([time_step, ddp.cost, duration, ddp.iter, divmod(time_step - 149, 50)[0]/10])
                #print(ddp.xs[1])
                ok_ = True
                
                #print(["time_step", time_step])
                #print(x0)

                #print(x0[0:21])
                #print(ddp.xs[1][0:21])
                #print(ddp.xs[1][21:41])
                '''
                print("ZMP")
                print(x0[43], x0[47])
                print(ddp.xs[1][43], ddp.xs[1][47])

                print(["time_step", time_step])
                print([x0[41],x0[45]])
                '''
                '''
                if(time_step >= 0):
                    for i in range(0, len(q)):
                        q[i] = x0[i]    
                    for i in range(0, len(qdot)):
                        qdot[i] = x0[i+len(q)]                    

                    pinocchio.forwardKinematics(model.model, data, q, qdot)
                    pinocchio.updateFramePlacements(model.model,data)
                    pinocchio.centerOfMass(model.model, data, q, qdot, False)
                    pinocchio.computeCentroidalMomentum(model.model,data,q,qdot)
                    print("aaa")
                    print(data.oMf[LFframe_id].translation)
                    print([lf_foot_pos_vector[1].translation[0],lf_foot_pos_vector[1].translation[1],lf_foot_pos_vector[1].translation[2]])
                    print(data.oMf[RFframe_id].translation)
                    print([rf_foot_pos_vector[1].translation[0],rf_foot_pos_vector[1].translation[1],rf_foot_pos_vector[1].translation[2]])
                    print("com")
                    print(data.com[0])
                    print([x0[41],x0[45], runningCostModel_vector[1].costs["comReg"].cost.residual.reference[2]])
                    
                    print(data.vcom[0])
                    print([x0[42],x0[46], 0.0])

                    print("ZMP")
                    print(x0[43], x0[47])
                    print(ddp.xs[1][43], ddp.xs[1][47])
                    
                    for i in range(0, len(q)):
                        q[i] = ddp.xs[1][i]    
                    for i in range(0, len(qdot)):
                        qdot[i] = ddp.xs[1][i+len(q)]                    
                    
                    pinocchio.forwardKinematics(model.model, data, q, qdot)
                    pinocchio.updateFramePlacements(model.model,data)
                    pinocchio.centerOfMass(model.model, data, q, qdot, False)
                    pinocchio.computeCentroidalMomentum(model.model,data,q,qdot)
                    print("bbb")
                    print(x0[44], x0[48])
                    print([ddp.xs[1][44], data.hg.angular[1]])
                    print([ddp.xs[1][48], data.hg.angular[0]])
                    print(data.oMf[LFframe_id].translation)
                    print([lf_foot_pos_vector[1].translation[0],lf_foot_pos_vector[1].translation[1],lf_foot_pos_vector[1].translation[2]])
                    print(data.oMf[RFframe_id].translation)
                    print([rf_foot_pos_vector[1].translation[0],rf_foot_pos_vector[1].translation[1],rf_foot_pos_vector[1].translation[2]])
                    print("com")
                    print(data.com[0])
                    print([ddp.xs[1][41],ddp.xs[1][45], runningCostModel_vector[1].costs["comReg"].cost.residual.reference[2]])
                    print(data.vcom[0])
                    print([ddp.xs[1][42],ddp.xs[1][46]])
                    print("ZMP")
                    print(x0[43], x0[47])
                    print(ddp.xs[1][43], ddp.xs[1][47])
                    print(ddp.xs[1])
                    print(x0)
                '''
                if time_step == total_time - 1:
                    time.sleep(0.002)
                    print(total_time_)
                    #print(cp_err)
                    np.savetxt('/home/jhk/data/walking/test.txt', cp_err)
                    np.savetxt('/home/jhk/data/walking/test1.txt', total_time_)
                    statemachine.write(np.array([3, 0, 0], dtype=np.int8))
                    time.sleep(1000)
               
                first_time = False
               
            elif mpc_signaldata[0] == 2:
                statemachine.write(np.array([2, 0, 0], dtype=np.int8))
       
def print_heard_talkling(message):    
    if  message['data'] == 'stateestimation':
        talk.publish(roslibpy.Message({'data': 'stateestimation'}))
    if  message['data'] == 'simvirtualjoint':
        talk.publish(roslibpy.Message({'data': 'simvirtualjoint'}))

   
if __name__=='__main__':
    global talk
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    #launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/jhk/catkin_ws/src/dyros_tocabi_v2/tocabi_controller/launch/simulation.launch'])
    #launch.start()
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()
    listener = roslibpy.Topic(client, '/tocabi/command', 'std_msgs/String')
    listener.subscribe(print_heard_talkling)
    talk = roslibpy.Topic(client, '/chatter', 'std_msgs/String')
    #talk.publish(roslibpy.Message({'data': 'Hello World!'}))
    #PCAlearning()
    talker()
