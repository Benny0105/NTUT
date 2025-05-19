#!/usr/bin/env python3
import os
import cv2
import numpy as np
import math
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from multiprocessing import cpu_count

# 匹配點門檻常數
MATCH_COUNT_THRESHOLD = 500

# 影像平移換算比例（pixel -> meter）
SCALE_FACTOR = 45 / 640.0

def preprocess_image(image):
    """影像預處理：先高斯模糊，再直方圖均衡化。"""
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    equalized = cv2.equalizeHist(blurred)
    return equalized

def calculate_similarity_and_transform(image1, image2):
    """
    ORB 特徵提取 + FLANN 匹配 + Lowe ratio 篩選，
    若匹配數 >= MATCH_COUNT_THRESHOLD，計算 Homography，
    然後計算影像中心的平移 (meters) 與旋轉角度 (degrees)。
    回傳 (match_count, H, shift_m, angle, good_matches)。
    """
    orb = cv2.ORB_create(nfeatures=3500)
    kp1, des1 = orb.detectAndCompute(image1, None)
    kp2, des2 = orb.detectAndCompute(image2, None)
    if des1 is None or des2 is None:
        return 0, None, None, None, None

    # FLANN 參數
    index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
    search_params = dict(checks=20)
    matcher = cv2.FlannBasedMatcher(index_params, search_params)
    knn_matches = matcher.knnMatch(des1, des2, k=2)

    good_matches = []
    for m, n in knn_matches:
        if m.distance < 0.7 * n.distance:
            good_matches.append(m)

    # 門檻過濾
    if len(good_matches) < MATCH_COUNT_THRESHOLD:
        return 0, None, None, None, None

    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1,1,2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1,1,2)
    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    if H is None:
        return 0, None, None, None, None

    # 計算中心平移
    h, w = image1.shape
    center = np.array([w/2, h/2], dtype=np.float32).reshape(-1,1,2)
    proj_center = cv2.perspectiveTransform(center, H)
    shift_px = np.linalg.norm(center - proj_center)
    shift_m = shift_px * SCALE_FACTOR

    # 計算旋轉角度
    angle = -math.degrees(math.atan2(H[1,0], H[0,0])) % 360

    return len(good_matches), H, shift_m, angle, good_matches

def compute_shift_and_angle_single(curr_path, ref_path):
    """
    單張對比：若匹配數達門檻，回傳 (shift_m, angle)，否則 (None, None)。
    """
    img1 = cv2.imread(curr_path, cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread(ref_path, cv2.IMREAD_GRAYSCALE)
    if img1 is None or img2 is None:
        return None, None
    img1 = preprocess_image(img1)
    img2 = preprocess_image(img2)
    match_count, H, shift, ang, _ = calculate_similarity_and_transform(img1, img2)
    if match_count == 0 or H is None:
        return None, None
    return shift, ang

def process_single_image(target_img, file_path):
    """平行化輔助函式：匹配並回傳結果元組或 None。"""
    folder_img = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
    if folder_img is None:
        return None
    folder_img = preprocess_image(folder_img)
    return calculate_similarity_and_transform(target_img, folder_img) + (file_path,)

def find_most_similar_images(target_path, folder_path, num_matches=1):
    """
    對 folder_path 下所有影像，並行計算匹配。
    回傳前 num_matches 名的列表及預處理後的 target_image。
    """
    target_img = cv2.imread(target_path, cv2.IMREAD_GRAYSCALE)
    if target_img is None:
        raise ValueError("目標影像讀取失敗")
    target_img = preprocess_image(target_img)

    results = []
    with ThreadPoolExecutor(max_workers=cpu_count()) as exe:
        futures = {exe.submit(process_single_image, target_img, os.path.join(folder_path, fn)): fn
                   for fn in os.listdir(folder_path)
                   if os.path.isfile(os.path.join(folder_path, fn))}
        for f in as_completed(futures):
            res = f.result()
            if res and res[0] >= MATCH_COUNT_THRESHOLD:
                results.append(res)

    # 根據匹配數排序
    results.sort(key=lambda x: x[0], reverse=True)
    return results[:num_matches], target_img

def compute_dx_dy_between_images(path1, path2):
    """
    直接對兩張影像做計算：若匹配數夠，回傳 (dx, dy)，否則 (None, None)。
    """
    img1 = cv2.imread(path1, cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread(path2, cv2.IMREAD_GRAYSCALE)
    if img1 is None or img2 is None:
        return None, None
    img1 = preprocess_image(img1)
    img2 = preprocess_image(img2)
    match_count, H, _, _, _ = calculate_similarity_and_transform(img1, img2)
    if match_count == 0 or H is None:
        return None, None
    # 計算中心位移
    h, w = img1.shape
    center = (w/2, h/2)
    pts = np.array([center], dtype=np.float32).reshape(-1,1,2)
    proj = cv2.perspectiveTransform(pts, H)[0][0]
    dx = (proj[0] - center[0]) * SCALE_FACTOR
    dy = (proj[1] - center[1]) * SCALE_FACTOR
    return dx, dy
