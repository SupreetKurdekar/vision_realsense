# import some common detectron2 utilities
import detectron2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog
import cv2

import torchvision
print("torch version",torchvision.__version__)
import torch
print(torch.__version__)

import time

# get image
im = cv2.imread("/home/supreet/vision2/b_2.jpg")
# cv2.imshow("imput",im)
# cv2.waitKey()

cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 10
cfg.MODEL.WEIGHTS = "/home/supreet/vision2/dl_models/Drill_seg_1.pth"  # path to the model we just trained
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.75   # set a custom testing threshold
predictor = DefaultPredictor(cfg)

start = time.perf_counter()
outputs = predictor(im)  # format is documented at https://detectron2.readthedocs.io/tutorials/models.html#model-output-format
t = outputs["instances"].to("cpu")
s1 = time.perf_counter()
print(s1-start)
# v = Visualizer(im[:, :, ::-1],scale=1.2)
# out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
# cv2.imshow("display",out.get_image()[:, :, ::-1])
# cv2.waitKey(0)