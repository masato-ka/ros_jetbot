import torch
import torch.optim as optim
import torch.nn.functional as F
import torchvision
import torchvision.datasets as datasets
import torchvision.models as models
import torchvision.transforms as transforms
import glob
import PIL.Image
import os
import numpy as np

NUM_EPOCHS = 30
BEST_MODEL_PATH = '/home/jetbot/bst_model.pth'

model = torchvision.models.resnet18(pretrained=True)
model.fc = torch.nn.Linear(512,2)
device = torch.device('cuda')
model = model.to(device)
bset_loss = 1e9

optimizer = optim.Adam(model.parameters())

train_loder = []
test_loader = []

for epoch in range(NUM_EPOCHS):


    model.train()
    train_loss = 0.0

    for images, labels in iter(train_loder):

        images = images.to(device)
        labels = labels.to(device)
        optimizer.zero_grad()
        outputs = model(images)
        loss = F.mse_loss(outputs,labels)
        train_loss += loss
        loss.backward()
        optimizer.step()
    train_loss /= len(train_loder)

    model.eval()
    test_loss=0.0
    for images, labels in iter(test_loader):
        images = images.to(device)
        labels = images.to(device)
        outputs = model(images)
        loss = F.mse_loss(outputs, labels)
        test_loss += loss
    test_loss /= len(test_loader)


    print('%f,%f', (train_loss, test_loss))
    if test_loss < best_loss:
        torch.save(model.state_dict(), BEST_MODEL_PATH)
        best_loss = test_loss