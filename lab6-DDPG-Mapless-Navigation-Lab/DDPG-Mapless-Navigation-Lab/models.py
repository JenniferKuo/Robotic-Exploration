import numpy as np
import random

import torch
import torch.nn as nn
import torch.nn.functional as F

# TODO(Lab-02): Complete the network model.
class PolicyNet(nn.Module):
    def __init__(self):
        super(PolicyNet, self).__init__()
        
        # inplace = true: 是否將之前計算的值覆蓋新的值，不用另外儲存其他變數
        self.relu = nn.ReLU(inplace=True)
        self.tanh = nn.Tanh(inplace=True)

        self.linear1 = nn.Linear(23,512)
        self.linear2 = nn.Linear(512,512)
        self.linear3 = nn.Linear(512,512)
        self.linear4 = nn.Linear(512,2) # action

    def forward(self, s):
        s = self.linear1(s)
        s = self.relu(s)
        s = self.linear2(s)
        s = self.relu(s)
        s = self.linear3(s)
        s = self.relu(s)
        s = self.linear4(s)
        s = self.tanh(s)

        return s

class QNet(nn.Module):
    def __init__(self):
        super(QNet, self).__init__()
        
        self.relu = nn.ReLU(inplace=True)

        self.linear1 = nn.Linear(23,514)
        self.linear2 = nn.Linear(514,512)
        self.linear3 = nn.Linear(512,512)
        self.linear4 = nn.Linear(512,1) # action value
        
    def forward(self, s, a):
        s = self.linear1(s)
        s = self.relu(s)
        s = self.linear2(s)
        s = self.relu(s)
        s = self.linear3(s)
        s = self.relu(s)
        s = self.linear4(s)
