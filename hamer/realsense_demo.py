from pathlib import Path
import torch
import argparse
import os
import torch
import numpy as np
import pyrealsense2 as rs
import gc
import cv2

# PyTorch 2.6+ 호환성 패치: 가중치 로드 시 보안 제한 해제
import functools
torch.load = functools.partial(torch.load, weights_only=False)

from hamer.configs import CACHE_DIR_HAMER
from hamer.models import HAMER, download_models, load_hamer, DEFAULT_CHECKPOINT
from hamer.utils import recursive_to
from hamer.datasets.vitdet_dataset import ViTDetDataset, DEFAULT_MEAN, DEFAULT_STD
from hamer.utils.renderer import Renderer, cam_crop_to_full
from vitpose_model import ViTPoseModel

LIGHT_BLUE=(0.65098039,  0.74117647,  0.85882353)

def main():
    parser = argparse.ArgumentParser(description='HaMeR RealSense Light Demo')
    parser.add_argument('--checkpoint', type=str, default=DEFAULT_CHECKPOINT)
    # 기본 검출기를 regnety로 강제 설정하여 메모리 절약
    parser.add_argument('--body_detector', type=str, default='regnety')

    args = parser.parse_args()

    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    
    print("Loading HaMeR model... (1/3)")
    download_models(CACHE_DIR_HAMER)
    model, model_cfg = load_hamer(args.checkpoint)
    model = model.to(device)
    model.eval()
    
    # 메모리 정리
    gc.collect()
    torch.cuda.empty_cache() if torch.cuda.is_available() else None

    print("Loading Keypoint detector... (2/3)")
    cpm = ViTPoseModel(device)
    
    print("Loading Human detector... (3/3)")
    from hamer.utils.utils_detectron2 import DefaultPredictor_Lazy
    from detectron2 import model_zoo
    from detectron2.config import get_cfg
    print("  Fetching Detectron2 config...")
    # 4gf(Giga Flops)에서 400mf(Mega Flops)로 변경하여 연산량 약 1/10로 축소
    detectron2_cfg = model_zoo.get_config('new_baselines/mask_rcnn_regnety_400mf_dds_FPN_400ep_LSJ.py', trained=True)
    print("  Config fetched. Setting threshold...")
    detectron2_cfg.model.roi_heads.box_predictor.test_score_thresh = 0.5
    print("  Initializing Predictor...")
    detector = DefaultPredictor_Lazy(detectron2_cfg)
    print("  Human detector loaded.")

    print("Initializing Renderer...")
    renderer = Renderer(model_cfg, faces=model.mano.faces)
    print("Renderer initialized.")

    print("Starting RealSense Pipeline...")
    pipeline = rs.pipeline()
    config = rs.config()
    # 해상도를 더 낮춰서 CPU/GPU 부담 완화 (640x480 -> 424x240)
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
    try:
        pipeline.start(config)
        print("RealSense Pipeline Started!")
    except Exception as e:
        print(f"Error starting RealSense: {e}")
        return

    print("\n>>> RealSense Started! Press 'q' to quit.")

    frame_count = 0
    process_every_n_frames = 5 # 5프레임마다 1번 처리 (부하 더 감소)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame: continue

            frame_count += 1
            if frame_count % process_every_n_frames != 0:
                # 처리하지 않는 프레임은 원본만 보여주고 넘어감
                img_cv2 = np.asanyarray(color_frame.get_data())
                cv2.imshow('HaMeR Light Demo', img_cv2)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue

            img_cv2 = np.asanyarray(color_frame.get_data())
            img_rgb = img_cv2.copy()[:, :, ::-1]

            # Detectron2 실행 전 메모리 정리
            if torch.cuda.is_available(): torch.cuda.empty_cache()

            with torch.no_grad():
                det_out = detector(img_cv2)
            
            det_instances = det_out['instances']
            valid_idx = (det_instances.pred_classes==0) & (det_instances.scores > 0.6)
            pred_bboxes=det_instances.pred_boxes.tensor[valid_idx].cpu().numpy()
            pred_scores=det_instances.scores[valid_idx].cpu().numpy()

            if len(pred_bboxes) == 0:
                cv2.imshow('HaMeR Light Demo', img_cv2)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue

            # 가장 큰 사람 1명만 처리 (메모리 절약)
            areas = (pred_bboxes[:, 2] - pred_bboxes[:, 0]) * (pred_bboxes[:, 3] - pred_bboxes[:, 1])
            largest_idx = np.argmax(areas)
            pred_bboxes = pred_bboxes[largest_idx:largest_idx+1]
            pred_scores = pred_scores[largest_idx:largest_idx+1]

            vitposes_out = cpm.predict_pose(img_rgb, [np.concatenate([pred_bboxes, pred_scores[:, None]], axis=1)])

            bboxes, is_right = [], []
            for vitposes in vitposes_out:
                for hand_type, keyps in [('left', vitposes['keypoints'][-42:-21]), ('right', vitposes['keypoints'][-21:])]:
                    valid = keyps[:,2] > 0.5
                    if sum(valid) > 5:
                        bboxes.append([keyps[valid,0].min(), keyps[valid,1].min(), keyps[valid,0].max(), keyps[valid,1].max()])
                        is_right.append(1 if hand_type == 'right' else 0)

            if len(bboxes) == 0:
                # 메모리 정리 후 다음 프레임
                del det_out, vitposes_out
                gc.collect()
                cv2.imshow('HaMeR Light Demo', img_cv2)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue

            dataset = ViTDetDataset(model_cfg, img_cv2, np.stack(bboxes), np.stack(is_right), rescale_factor=2.0)
            dataloader = torch.utils.data.DataLoader(dataset, batch_size=len(bboxes), shuffle=False)

            all_verts, all_cam_t, all_right = [], [], []
            for batch in dataloader:
                batch = recursive_to(batch, device)
                with torch.no_grad():
                    out = model(batch)

                pred_cam = out['pred_cam']
                pred_cam[:,1] = (2*batch['right']-1)*pred_cam[:,1]
                img_size = batch["img_size"].float()
                scaled_focal_length = model_cfg.EXTRA.FOCAL_LENGTH / model_cfg.MODEL.IMAGE_SIZE * img_size.max()
                pred_cam_t_full = cam_crop_to_full(pred_cam, batch["box_center"].float(), batch["box_size"].float(), img_size, scaled_focal_length).detach().cpu().numpy()

                for n in range(batch['img'].shape[0]):
                    verts = out['pred_vertices'][n].detach().cpu().numpy()
                    is_r = batch['right'][n].cpu().numpy()
                    verts[:,0] = (2*is_r-1)*verts[:,0]
                    all_verts.append(verts); all_cam_t.append(pred_cam_t_full[n]); all_right.append(is_r)

            if len(all_verts) > 0:
                cam_view = renderer.render_rgba_multiple(all_verts, cam_t=all_cam_t, render_res=img_size[0], is_right=all_right, focal_length=scaled_focal_length, mesh_base_color=LIGHT_BLUE, scene_bg_color=(1, 1, 1))
                input_img = img_cv2.astype(np.float32)[:,:,::-1]/255.0
                input_img = np.concatenate([input_img, np.ones_like(input_img[:,:,:1])], axis=2)
                overlay = input_img[:,:,:3] * (1-cam_view[:,:,3:]) + cam_view[:,:,:3] * cam_view[:,:,3:]
                cv2.imshow('HaMeR Light Demo', (overlay * 255).astype(np.uint8)[:,:,::-1])
            else:
                cv2.imshow('HaMeR Light Demo', img_cv2)

            # 명시적 메모리 해제 및 가비지 컬렉션
            del det_out, vitposes_out, bboxes, is_right, all_verts, all_cam_t, all_right
            if 'out' in locals(): del out
            if 'batch' in locals(): del batch
            gc.collect()

            if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
