# Copyright 2020 ROS-Industrial Consortium Asia Pacific
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy

from datetime import date

import glob
import os
import subprocess

import time

from PIL import Image

import torch
import torch.nn as nn
import torch.optim as optim

from torchvision import datasets, models, transforms


class P1Trainer:
    '''
    The Precision-Level 1 (P1) Trainer class object instantiates a training
    session using PyTorch for taking an input dataset and selected pretrained
    model from PyTorch model zoo in order to generate a custom-trained P1 ONNX
    model file.\n
    This model can then be deployed as a ROS2 package.
    '''
    def __init__(self, path_to_dataset, model_name, label_list):
        '''
        The constructor.
        Calls the initialize model function based on the requested pretrained
        Pytorch model from PyTorch model zoo.
        '''
        self.data_dir = path_to_dataset
        self.model_name = model_name
        self.num_classes = len(label_list)
        self.class_names = label_list

        self.path_to_trained_model = ('../data/model/' +
                                      self.model_name +
                                      '_' +
                                      str(date.today()) +
                                      '.onnx')

        self.feature_extract = True

        self.model_ft, input_size = self.initialize_model(self.model_name,
                                                          self.num_classes,
                                                          self.feature_extract)

        self.IMAGENET_MEAN = [0.485, 0.456, 0.406]
        self.IMAGENET_STD = [0.229, 0.224, 0.225]
        self.data_transforms = {
            'train': transforms.Compose([
                transforms.RandomResizedCrop(input_size),
                transforms.RandomHorizontalFlip(),
                transforms.ToTensor(),
                transforms.Normalize(self.IMAGENET_MEAN, self.IMAGENET_STD)
            ]),
            'val': transforms.Compose([
                transforms.Resize(input_size),
                transforms.CenterCrop(input_size),
                transforms.ToTensor(),
                transforms.Normalize(self.IMAGENET_MEAN, self.IMAGENET_STD)
            ]),
        }

    def train(self, debug):
        '''
        A Mutator function that sets a fixed 15-epoch training session given
        prior session configuration set during initialization, calls train_model
        function.
        '''
        if debug:
            self.batch_size = 8
            self.num_epochs = 2
        else:
            self.batch_size = 8
            self.num_epochs = 15

        image_datasets = {x: datasets.ImageFolder(
                          os.path.join(self.data_dir, x),
                          self.data_transforms[x]) for x in ['train', 'val']}

        dataloaders_dict = {x: torch.utils.data.DataLoader(image_datasets[x],
                            batch_size=self.batch_size,
                            shuffle=True,
                            num_workers=4) for x in ['train', 'val']}

        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

        model_ft = self.model_ft.to(self.device)

        params_to_update = model_ft.parameters()
        print('Parameters to learn:')
        if self.feature_extract:
            params_to_update = []
            for name, param in model_ft.named_parameters():
                if param.requires_grad is True:
                    params_to_update.append(param)
                    print('\t', name)
        # else:
        #     for name, param in model_ft.named_parameters():
        #         if param.requires_grad is True:
        #             print('\t', name)

        optimizer_ft = optim.SGD(params_to_update, lr=0.001, momentum=0.9)

        criterion = nn.CrossEntropyLoss()

        model_ft, hist = self.train_model(model_ft,
                                          dataloaders_dict,
                                          criterion,
                                          optimizer_ft,
                                          num_epochs=self.num_epochs,
                                          is_inception=(self.model_name == 'inception'))

        filename = self.model_name + '_' + str(date.today())

        p1 = subprocess.Popen(['mkdir', '-p', './trainer/P1TrainFarm'])
        p1.communicate()

        torch.save(model_ft.state_dict(), './trainer/P1TrainFarm/' + filename + '.pth')

        # Set up the image pre-processing.
        tfms = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(self.IMAGENET_MEAN, self.IMAGENET_STD)
        ])

        model_ft.load_state_dict(torch.load('./trainer/P1TrainFarm/' + filename + '.pth'))

        files = glob.glob(self.data_dir + '/**/*.png', recursive=True)
        # If unable to find any .png files, find .jpg file.
        if len(files) == 0:
            files = glob.glob(self.data_dir + '/**/*.jpg', recursive=True)
            files.append(glob.glob(self.data_dir + '/**/*.jpeg', recursive=True))

        random_train_image = files[0]
        # # Open the input image and move to GPU.
        img = tfms(Image.open(random_train_image)).unsqueeze(0)
        img = img.to(self.device)

        # Set the model to evaluation mode.
        model_ft.eval()

        # Run inference.
        with torch.no_grad():
            outputs = model_ft(img)
            _, preds = torch.max(outputs, 1)

        print('Testing ONNX model on image at: ', files[0])
        print('Object in image determined to be: ', self.class_names[preds[0]])

        # Export the model
        torch.onnx.export(  # model being run
                          model_ft,
                          # model input (or a tuple for multiple inputs)
                          img,
                          # where to save the model (can be a file or file-like object)
                          self.path_to_trained_model,
                          # store the trained parameter weights inside the model file
                          export_params=True,
                          # the ONNX version to export the model to
                          opset_version=10,
                          # whether to execute constant folding for optimization
                          do_constant_folding=True,
                          # the model's input names
                          input_names=['input'],
                          # the model's output names
                          output_names=['output'],
                          # variable length axes
                          dynamic_axes={'input': {0: 'batch_size'},
                                        'output': {0: 'batch_size'}})

        # # EXPORT End
        print('ONNX model exported to ', self.path_to_trained_model)

    def train_model(self,
                    model,
                    dataloaders,
                    criterion,
                    optimizer,
                    num_epochs=25,
                    is_inception=False):
        '''
        An internal Mutator function that conducts the training session
        given prior session configuration set during initialization.
        '''

        since = time.time()

        val_acc_history = []

        best_model_wts = copy.deepcopy(model.state_dict())
        best_acc = 0.0

        for epoch in range(num_epochs):
            print('Epoch {}/{}'.format(epoch, num_epochs - 1))
            print('-' * 10)

            # Each epoch has a training and validation phase
            for phase in ['train', 'val']:
                if phase == 'train':
                    model.train()  # Set model to training mode
                else:
                    model.eval()   # Set model to evaluate mode

                running_loss = 0.0
                running_corrects = 0

                # Iterate over data.
                for inputs, labels in dataloaders[phase]:
                    inputs = inputs.to(self.device)
                    labels = labels.to(self.device)

                    # zero the parameter gradients
                    optimizer.zero_grad()

                    # forward
                    # track history if only in train
                    with torch.set_grad_enabled(phase == 'train'):

                        if is_inception and phase == 'train':
                            outputs, aux_outputs = model(inputs)
                            loss1 = criterion(outputs, labels)
                            loss2 = criterion(aux_outputs, labels)
                            loss = loss1 + 0.4*loss2
                        else:
                            outputs = model(inputs)
                            loss = criterion(outputs, labels)

                        _, preds = torch.max(outputs, 1)

                        if phase == 'train':
                            loss.backward()
                            optimizer.step()

                    # statistics
                    running_loss += loss.item() * inputs.size(0)
                    running_corrects += torch.sum(preds == labels.data)

                epoch_loss = running_loss / len(dataloaders[phase].dataset)
                epoch_acc = running_corrects.double() / len(dataloaders[phase].dataset)

                print('{} Loss: {:.4f} Accuracy: {:.4f}'.format(phase, epoch_loss, epoch_acc))

                # deep copy the model
                if phase == 'val' and epoch_acc > best_acc:
                    best_acc = epoch_acc
                    best_model_wts = copy.deepcopy(model.state_dict())
                if phase == 'val':
                    val_acc_history.append(epoch_acc)

            print()

        time_elapsed = time.time() - since
        print('Training complete in {:.0f}m {:.0f}s'.format(time_elapsed // 60, time_elapsed % 60))
        print('Best val Acc: {:4f}'.format(best_acc))

        # load best model weights
        model.load_state_dict(best_model_wts)
        return model, val_acc_history

    def set_parameter_requires_grad(self,
                                    model,
                                    feature_extracting):
        '''
        A Mutator function that sets param.requires_grad boolean to false if
        feature_extracting boolean flag is set to True.
        '''
        if feature_extracting:
            for param in model.parameters():
                param.requires_grad = False

    def initialize_model(self,
                         model_name,
                         num_classes,
                         feature_extract,
                         use_pretrained=True):
        # Initialize these variables which will be set in this if statement. Each of these
        # variables is model specific.
        '''
        A Getter function that takes the requested session configuration and
        initializes corresponding pretrained model.\n
        Calls set_parameter_requires_grad function during initialization.
        '''
        model_ft = None
        input_size = 0

        if model_name == 'resnet':
            model_ft = models.resnet18(pretrained=use_pretrained)
            self.set_parameter_requires_grad(model_ft, feature_extract)
            num_ftrs = model_ft.fc.in_features
            model_ft.fc = nn.Linear(num_ftrs, num_classes)
            input_size = 224

        elif model_name == 'alexnet':
            model_ft = models.alexnet(pretrained=use_pretrained)
            self.set_parameter_requires_grad(model_ft, feature_extract)
            num_ftrs = model_ft.classifier[6].in_features
            model_ft.classifier[6] = nn.Linear(num_ftrs, num_classes)
            input_size = 224

        elif model_name == 'vgg':
            model_ft = models.vgg11_bn(pretrained=use_pretrained)
            self.set_parameter_requires_grad(model_ft, feature_extract)
            num_ftrs = model_ft.classifier[6].in_features
            model_ft.classifier[6] = nn.Linear(num_ftrs, num_classes)
            input_size = 224

        elif model_name == 'squeezenet':
            model_ft = models.squeezenet1_0(pretrained=use_pretrained)
            self.set_parameter_requires_grad(model_ft, feature_extract)
            model_ft.classifier[1] = nn.Conv2d(512,
                                               num_classes,
                                               kernel_size=(1, 1),
                                               stride=(1, 1))
            model_ft.num_classes = num_classes
            input_size = 224

        elif model_name == 'densenet':
            model_ft = models.densenet121(pretrained=use_pretrained)
            self.set_parameter_requires_grad(model_ft, feature_extract)
            num_ftrs = model_ft.classifier.in_features
            model_ft.classifier = nn.Linear(num_ftrs, num_classes)
            input_size = 224

        elif model_name == 'inception':
            model_ft = models.inception_v3(pretrained=use_pretrained)
            self.set_parameter_requires_grad(model_ft, feature_extract)
            # Handle the auxilary net
            num_ftrs = model_ft.AuxLogits.fc.in_features
            model_ft.AuxLogits.fc = nn.Linear(num_ftrs, num_classes)
            # Handle the primary net
            num_ftrs = model_ft.fc.in_features
            model_ft.fc = nn.Linear(num_ftrs, num_classes)
            input_size = 299

        elif model_name == 'mobilenet':
            model_ft = models.mobilenet_v2(pretrained=True)
            model_ft.classifier[1] = torch.nn.Linear(
                                in_features=model_ft.classifier[1].in_features,
                                out_features=num_classes)
            input_size = 224

        else:
            print('Invalid model name, exiting...')
            return None, 0

        return model_ft, input_size
