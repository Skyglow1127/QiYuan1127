"""In-process version of the archived OpenCV red foreground route; no service."""
import base64
import hashlib
import time
import cv2
import numpy as np

HSV_RANGES = [([0, 80, 50], [10, 255, 255]), ([170, 80, 50], [179, 255, 255])]

def infer_remote(raw, metadata=None, endpoint=None, **unused):
    start = time.perf_counter()
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape != (480, 640, 3):
        raise ValueError('需要原始 640×480 RGB 图像')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    binary = np.zeros(image.shape[:2], np.uint8)
    for low, high in HSV_RANGES:
        binary |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    areas = stats[1:, cv2.CC_STAT_AREA]
    mask = np.zeros_like(binary)
    rejection = None
    significant = 0
    bounding_box = None
    clipped_edges = []
    rejection_code = None
    if not len(areas) or areas.max() < 100:
        rejection = '没有可拟合的红色目标'
        rejection_code = 'target_not_found'
    else:
        biggest = int(np.argmax(areas)) + 1
        significant = int(np.sum(areas > .2 * areas.max()))
        mask[labels == biggest] = 255
        bounding_box = [int(v) for v in stats[biggest, :4]]
        if significant > 1:
            rejection = '存在多个面积相近的红色目标'
            rejection_code = 'multiple_targets'
        for edge, pixels in [('top', mask[:2]), ('bottom', mask[-2:]), ('left', mask[:, :2]), ('right', mask[:, -2:])]:
            if np.any(pixels): clipped_edges.append(edge)
        if clipped_edges:
            names = {'top':'上','bottom':'下','left':'左','right':'右'}
            sides = '、'.join(names[e] for e in clipped_edges)
            rejection = f'红方块的{sides}沿接触腕部图像边界，完整轮廓不可用；请保持相机安装不变，使整个方块进入画面后重新准备'
            rejection_code = 'target_clipped'
    metadata = dict(metadata or {})
    if metadata.get('source_kind') == 'live_camera':
        age = metadata['board_age_at_fetch_ms'] + (time.monotonic() - metadata['received_host_monotonic']) * 1000
        if not 0 <= age <= 250:
            rejection = '图像已过期'
            rejection_code = 'stale_image'
    ok, png = cv2.imencode('.png', mask)
    if not ok:
        raise ValueError('无法编码前景掩码')
    payload = png.tobytes()
    return dict(schema='grasp-foreground-v1', backend='hsv', image_sha256=hashlib.sha256(raw).hexdigest(),
                status='REJECTED' if rejection else 'FOREGROUND_CANDIDATE', rejection_reason=rejection,
                rejection_code=rejection_code, bounding_box_xywh=bounding_box, clipped_edges=clipped_edges,
                image_size_wh=[640,480], significant_component_count=significant, mask_png_base64=base64.b64encode(payload).decode(),
                mask_png_sha256=hashlib.sha256(payload).hexdigest(), npu_execution_confirmed=False,
                timing_ms={'opencv_ms': (time.perf_counter()-start)*1000},
                method='archived_opencv_hsv_largest_component', ranges=HSV_RANGES)
