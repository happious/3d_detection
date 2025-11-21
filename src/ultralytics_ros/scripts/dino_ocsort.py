#!/usr/bin/env python3
import os, sys, cv2, torch, rospy
import numpy as np
from PIL import Image
from contextlib import nullcontext
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage
import torchvision.transforms as T
from torchvision.ops import nms

import warnings
warnings.filterwarnings("ignore")

import rospkg
rp = rospkg.RosPack()
PKG_ROOT = rp.get_path("ultralytics_ros")

DINO_ROOT = os.path.join(PKG_ROOT, "DINO")
OCSORT_DIR = os.path.join(PKG_ROOT, "OC_SORT")

sys.path.append(DINO_ROOT)
sys.path.insert(0, OCSORT_DIR)

from models.dino.dino import build_dino
from util.box_ops import box_cxcywh_to_xyxy
from util.slconfig import SLConfig
from trackers.ocsort_tracker.ocsort import OCSort
from custom_msgs.msg import TrackingResult, TrackingResultArray

DEVICE      = "cuda"
CONF_THRES  = 0.7
NMS_IOU_THR = 0.40
USE_FP16    = True
TARGET_CLS  = 1

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "max_split_size_mb:128")
torch.backends.cudnn.benchmark = True

@torch.no_grad()
def load_model(cfg_path, ckpt_path, device="cuda"):
    rospy.loginfo(f"[DINO] Loading CFG:  {cfg_path}")
    rospy.loginfo(f"[DINO] Loading CKPT: {ckpt_path}")

    args = SLConfig.fromfile(cfg_path)
    args.device = device
    if hasattr(args, "use_checkpoint"):
        args.use_checkpoint = False
    if not hasattr(args, "num_classes"):
        args.num_classes = 91

    model, _, _ = build_dino(args)
    state = torch.load(ckpt_path, map_location=device)
    state = state.get("model", state)
    model.load_state_dict(state, strict=True)
    model.to(device).eval()

    if USE_FP16 and device.startswith("cuda"):
        model.half()

    rospy.loginfo("[DINO] Model loaded successfully")
    return model

def preprocess_bgr_for_dino(frame_bgr, device):
    img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(img_rgb)
    tfm = T.Compose([
        T.ToTensor(),
        T.Normalize([0.485, 0.456, 0.406],
                    [0.229, 0.224, 0.225]),
    ])
    tensor = tfm(pil).unsqueeze(0).to(device)

    if USE_FP16 and device.startswith("cuda"):
        tensor = tensor.half()

    return tensor

class DinoOCSortNode:
    def __init__(self):
        self.bridge = CvBridge()

        input_topic = rospy.get_param("~input_topic", "/ricoh/image_raw")
        cfg_path    = rospy.get_param("~cfg_path",  os.path.join(DINO_ROOT, "config/DINO/DINO_4scale_swin.py"))
        ckpt_path   = rospy.get_param("~ckpt_path", os.path.join(DINO_ROOT, "weights/checkpoint0029_4scale_swin.pth"))

        rospy.loginfo(f"[DINO_NODE] Subscribe topic : {input_topic}")
        rospy.loginfo(f"[DINO_NODE] CFG PATH        : {cfg_path}")
        rospy.loginfo(f"[DINO_NODE] CKPT PATH       : {ckpt_path}")

        self.model = load_model(cfg_path, ckpt_path, DEVICE)
        self.tracker = OCSort(det_thresh=CONF_THRES)
        self.amp_ctx = torch.cuda.amp.autocast if (USE_FP16 and DEVICE.startswith("cuda")) else nullcontext

        self.image_pub = rospy.Publisher("/tracking/image_raw", RosImage, queue_size=1)

        self.track_pub = rospy.Publisher("/tracking_result", TrackingResultArray, queue_size=1)

        self.sub = rospy.Subscriber(input_topic, RosImage, self.callback,
                                    queue_size=1, buff_size=2**24)

        rospy.loginfo("DINO + OC-SORT Node Initialized")

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = frame.shape[:2]
        img_info = [h, w, 3]
        img_size = (w, h)

        with self.amp_ctx():
            outputs = self.model(preprocess_bgr_for_dino(frame, DEVICE))

        logits = outputs["pred_logits"][0].softmax(-1)
        boxes  = outputs["pred_boxes"][0]
        scores, labels = logits[..., :-1].max(-1)
        keep = (scores > CONF_THRES) & (labels == TARGET_CLS)

        if keep.any():
            scores_kept = scores[keep]
            boxes_kept  = boxes[keep]
            scale_vec = torch.tensor([w, h, w, h], device=boxes_kept.device)
            boxes_xyxy = box_cxcywh_to_xyxy(boxes_kept) * scale_vec

            boxes_xyxy_cpu = boxes_xyxy.detach().float().cpu()
            scores_cpu     = scores_kept.detach().float().cpu()
            keep_idx = nms(boxes_xyxy_cpu, scores_cpu, NMS_IOU_THR)

            if keep_idx.numel() > 0:
                boxes_np  = boxes_xyxy_cpu[keep_idx].numpy()
                scores_np = scores_cpu[keep_idx].numpy()
                dets = np.concatenate([boxes_np, scores_np[:, None]], axis=1)
                tracks = self.tracker.update(dets, img_info, img_size)
            else:
                tracks = self.tracker.update(np.empty((0, 5)), img_info, img_size)
        else:
            tracks = self.tracker.update(np.empty((0, 5)), img_info, img_size)

        track_msg = TrackingResultArray()
        track_msg.header = msg.header

        GREEN = (0, 255, 0)

        for t in tracks:
            if len(t) >= 6:
                x1, y1, x2, y2, tid, since_update = t[:6]
                if since_update > 0:
                    continue
            else:
                x1, y1, x2, y2, tid = t[:5]

            tr = TrackingResult()
            tr.header   = msg.header
            tr.track_id = int(tid)
            tr.class_id = int(TARGET_CLS)
            tr.score    = 1.0

            tr.x1 = float(x1)
            tr.y1 = float(y1)
            tr.x2 = float(x2)
            tr.y2 = float(y2)

            track_msg.tracks.append(tr)

            x1i, y1i, x2i, y2i = map(int, [x1, y1, x2, y2])
            cv2.rectangle(frame, (x1i, y1i), (x2i, y2i), GREEN, 2)
            cv2.putText(frame, f'ID:{tid}', (x1i, y1i - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

        self.track_pub.publish(track_msg)

        out_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("dino_ocsort_node")
    node = DinoOCSortNode()
    rospy.spin()

