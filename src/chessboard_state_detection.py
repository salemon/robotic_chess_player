#!/usr/bin/env python3
import io
import cv2
import torch
import numpy as np
import torch.nn as nn
from torchvision import datasets, models, transforms


class ChessboardStateDetection:

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    processing = transforms.Compose([
            transforms.ToPILImage(),                         
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

    def __init__(self,nn_path):
        self.model = self.load_model(nn_path)

    @staticmethod
    def load_model(path):
        model = torch.load(path).eval()
        model = model.to(vision.device)
        return model

    def setSquareDict(self,square_dict):
        self.square_dict = square_dict

    def detecting(self,image):
        processing = vision.processing
        col = {'A':7,'B':6,'C':5,'D':4,'E':3,'F':2,'G':1,'H':0}
        row = {'1':0,'2':1,'3':2,'4':3,'5':4,'6':5,'7':6,'8':7}
        class_names = ['B', 'K', 'N', 'P', 'Q', 'R', '_', 'b', 'k', 'n', 'p', 'q', 'r']
        chessboard = np.zeros((8,8),dtype=str)
        img_list,key_list,count = [],[],0
        for key,value in self.square_dict.items():
            y1,y2,x1,x2 = value
            img = image[y1:y2,x1:x2,:]
            img_tt = vision.processing(img)
            img_list.append(img_tt)
            key_list.append(key)
            count += 1
            if count == 4:
                inputs = torch.stack(img_list,0)
                inputs = inputs.to(vision.device)
                outputs = self.model(inputs)
                _, preds = torch.max(outputs,1)
                for k in key_list:
                    chessboard[row[k[1]],col[k[0]]] = class_names[preds[key_list.index(k)]]
                img_list,key_list,count = [],[],0
        return chessboard

    def boardTOFen(self,board):
    # Use StringIO to build string more efficiently than concatenating
        with io.StringIO() as s:
            for row in board:
                empty = 0
                for cell in row:
                    c = cell[0]
                    if c in ('w', 'b'):
                        if empty > 0:
                            s.write(str(empty))
                            empty = 0
                        s.write(cell[1].upper() if c == 'w' else cell[1].lower())
                    else:
                        empty += 1
                if empty > 0:
                    s.write(str(empty))
                s.write('/')
            # Move one position back to overwrite last '/'
            s.seek(s.tell() - 1)
            # If you do not have the additional information choose what to put
            s.write(' b KQkq - 0 1')
            return s.getvalue()