import numpy as np
import cv2
import glob
import argparse


# 코너 검출 정밀도를 높이기 위한 종료 조건 설정
# (반복 횟수 30회 또는 정밀도 0.001 도달 시 중단)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def calibrate(dirpath, prefix, image_format, square_size, width=9, height=6):
    """
    지정된 디렉토리의 체스판 이미지를 분석하여 카메라 캘리브레이션을 수행
    """
    # 1. 월드 좌표계(Real World Space)에서의 3D 점 설정 (0,0,0), (1,0,0) ...
    # 실제 체스판의 칸 크기(square_size)를 곱하여 물리적인 단위(mm 등)로 변환
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # 모든 이미지에서 검출된 3D 포인트와 2D 포인트를 저장할 리스트
    objpoints = []  # 실제 세계의 3D 점
    imgpoints = []  # 이미지 평면의 2D 점

    if dirpath[-1:] == '/':
        dirpath = dirpath[:-1]

    images = glob.glob(dirpath+'/' + prefix + '*.' + image_format)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 2. 체스판 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)


        if ret:
            objpoints.append(objp)

            # 3. 서브픽셀 단위로 코너 정밀도 개선 (Calibration 정확도 향상의 핵심)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # 코너 시각화 (선택 사항)
            # img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)

    # 4. 카메라 캘리브레이션 수행
    # mtx: 내부 파라미터(Intrinsic), dist: 왜곡 계수(Distortion)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]

def save_coefficients(mtx, dist, path):
    """ 캘리브레이션 결과(카메라 행렬 K, 왜곡 계수 D)를 YAML 파일로 저장"""
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    cv_file.release()

def load_coefficients(path):
    """ 저장된 YAML 파일로부터 카메라 파라미터를 로드"""
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--image_dir', type=str, required=True, help='image directory path')  
    parser.add_argument('--image_format', type=str, required=True,  help='image format, png/jpg') 
    parser.add_argument('--prefix', type=str, required=True, help='image prefix') 
    parser.add_argument('--square_size', type=float, required=False, help='chessboard square size')  
    parser.add_argument('--width', type=int, required=False, help='chessboard width size, default is 9') 
    parser.add_argument('--height', type=int, required=False, help='chessboard height size, default is 6')
    parser.add_argument('--save_file', type=str, required=True, help='YML file to save calibration matrices') 
    args = parser.parse_args()

    # 캘리브레이션 실행
    ret, mtx, dist, rvecs, tvecs = calibrate(args.image_dir, args.prefix, args.image_format, args.square_size, args.width, args.height)
    save_coefficients(mtx, dist, args.save_file)
    print("Calibration is finished. RMS: ", ret)