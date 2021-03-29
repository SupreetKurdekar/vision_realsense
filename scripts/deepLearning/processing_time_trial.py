import detectron2
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog
import cv2

import time
import numpy
from statistics import mean,median,stdev

import os

folder = "/home/supreet/ObjectDatasetTools/mask_rcnn_data/bottom_side_data/JPEGImages"

cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.MODEL.ROI_HEADS.NUM_CLASSES = 10
cfg.MODEL.WEIGHTS = "/home/supreet/vision2/dl_models/Drill_seg_1.pth"  # path to the model we just trained
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.75   # set a custom testing threshold
cfg.MODEL.DEVICE = "cuda"
predictor = DefaultPredictor(cfg)

times = []

for f in os.listdir(folder):
    name = os.path.join(folder,f)
    im = cv2.imread(name)

    start = time.perf_counter()
    outputs = predictor(im)  # format is documented at https://detectron2.readthedocs.io/tutorials/models.html#model-output-format
    t = outputs["instances"].to("cpu")
    s1 = time.perf_counter()
    times.append(s1-start)

    v = Visualizer(im[:, :, ::-1],scale=1.2)
    out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
    cv2.imshow("display",out.get_image()[:, :, ::-1])
    cv2.waitKey(1)


print("mean: ",mean(times))
print("std : ",stdev(times))
print("median : ",median(times))
print("sum: ",sum(times))
print("count: ",len(times))
