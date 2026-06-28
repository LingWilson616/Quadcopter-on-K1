#!/usr/bin/env python3
"""K1 人物检测快照 — 连续拍摄带检测框的照片, 用于竞赛演示素材."""
import sys, os, time

sys.path.insert(0, os.path.expanduser('~/spacemit-demo/examples/CV/yolov8/python'))
from utils import Yolov8Detection
import cv2

def main():
    model = os.path.expanduser(
        '~/spacemit-demo/examples/CV/yolov8/model/yolov8n_320x320.q.onnx')
    out_dir = '/tmp/demo_shots'
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    os.makedirs(out_dir, exist_ok=True)

    print(f'Loading YOLOv8...')
    d = Yolov8Detection(model, conf_threshold=0.25, iou_threshold=0.45)
    print(f'Model: 320x320 INT8  |  person-only')

    cap = cv2.VideoCapture('/dev/video20', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print(f'\nCamera: 1280x720 MJPG 25fps')
    print(f'Shots: {count} × {interval}s interval')
    print(f'Output: {out_dir}/shot_XX.jpg\n')

    for i in range(1, count + 1):
        # 丢前几帧让曝光稳定
        for _ in range(5):
            ret, frame = cap.read()
        if not ret:
            print(f'  [{i:02d}] CAM FAIL')
            continue

        img = frame.copy()
        tensor = d.preprocess(img)
        outputs = d.session.run(d.output_names, {d.input_name: tensor})
        boxes, classes, scores = d.postprocess(outputs)

        n = len(boxes) if boxes is not None else 0
        if n:
            result = d.draw_results(img, boxes, classes, scores)
        else:
            result = img
            cv2.putText(result, 'No person detected', (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 统计标签
        label = f'Shot {i:02d}  |  {n} person(s)'
        if n:
            confs = ' '.join(f'{s*100:.0f}%' for s in scores)
            label += f'  |  {confs}'
        cv2.putText(result, label, (20, 690),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        path = f'{out_dir}/shot_{i:02d}.jpg'
        cv2.imwrite(path, result, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print(f'  [{i:02d}] {n} person(s)  →  {path}')

        if i < count:
            time.sleep(interval)

    cap.release()
    print(f'\nDone — {count} shots saved to {out_dir}/')
    print(f'Copy: scp bianbu@10.171.220.9:{out_dir}/shot_*.jpg ./')

if __name__ == '__main__':
    main()
