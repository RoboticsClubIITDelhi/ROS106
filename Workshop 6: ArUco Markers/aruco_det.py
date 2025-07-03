import cv2
import numpy as np
import os
import glob

class ArUcoDetector:
    def __init__(self, marker_size=0.05):
        self.marker_size = marker_size
        
        self.camera_matrix = np.array([
            [600, 0, 300],
            [0, 600, 300],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((4, 1))
        
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        
    def detect_markers(self, image_path):
        image = cv2.imread(image_path)
        if image is None:
            print(f"Could not load image: {image_path}")
            return None
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        try:
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, rejected = detector.detectMarkers(gray)
        except AttributeError:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        
        results = {
            'image_path': image_path,
            'corners': corners,
            'ids': ids,
            'poses': []
        }
        
        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            
            for i, marker_id in enumerate(ids.flatten()):
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                roll, pitch, yaw = self.rotation_matrix_to_euler_angles(rotation_matrix)
                x, y, z = tvecs[i][0]
                
                pose_info = {
                    'id': int(marker_id),
                    'translation': {
                        'x': float(x),
                        'y': float(y), 
                        'z': float(z),
                        'distance': float(np.linalg.norm(tvecs[i]))
                    },
                    'rotation': {
                        'roll': float(np.degrees(roll)),
                        'pitch': float(np.degrees(pitch)),
                        'yaw': float(np.degrees(yaw))
                    },
                    'rotation_vector': rvecs[i].flatten().tolist(),
                    'translation_vector': tvecs[i].flatten().tolist()
                }
                
                results['poses'].append(pose_info)
        
        return results
    
    def rotation_matrix_to_euler_angles(self, R):
        sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2,1], R[2,2])
            y = np.arctan2(-R[2,0], sy)
            z = np.arctan2(R[1,0], R[0,0])
        else:
            x = np.arctan2(-R[1,2], R[1,1])
            y = np.arctan2(-R[2,0], sy)
            z = 0
        
        return x, y, z

    def draw_axes_manual(self, image, camera_matrix, dist_coeffs, rvec, tvec, length):
        axes_points = np.float32([
            [0, 0, 0],
            [length, 0, 0],
            [0, length, 0],
            [0, 0, -length]
        ]).reshape(-1, 3)
        
        img_points, _ = cv2.projectPoints(axes_points, rvec, tvec, camera_matrix, dist_coeffs)
        img_points = np.int32(img_points).reshape(-1, 2)
        
        origin = tuple(img_points[0])
        x_end = tuple(img_points[1])
        y_end = tuple(img_points[2])
        z_end = tuple(img_points[3])
        
        cv2.line(image, origin, x_end, (0, 0, 255), 3)
        cv2.line(image, origin, y_end, (0, 255, 0), 3)
        cv2.line(image, origin, z_end, (255, 0, 0), 3)

    def visualize_detection(self, image_path, results):
        image = cv2.imread(image_path)
        if image is None:
            return None
        
        if results['ids'] is not None:
            cv2.aruco.drawDetectedMarkers(image, results['corners'], results['ids'])
            
            for i, pose in enumerate(results['poses']):
                rvec = np.array(pose['rotation_vector']).reshape(3, 1)
                tvec = np.array(pose['translation_vector']).reshape(3, 1)
                
                try:
                    cv2.aruco.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, 
                                          rvec, tvec, self.marker_size * 0.5)
                except AttributeError:
                    try:
                        cv2.aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, 
                                         rvec, tvec, self.marker_size * 0.5)
                    except AttributeError:
                        self.draw_axes_manual(image, self.camera_matrix, self.dist_coeffs, 
                                            rvec, tvec, self.marker_size * 0.5)
                
                corner = results['corners'][i][0][0]
                text_x, text_y = int(corner[0]), int(corner[1]) - 10
                
                cv2.putText(image, f"ID: {pose['id']}", 
                           (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(image, f"Dist: {pose['translation']['distance']:.2f}m", 
                           (text_x, text_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        output_path = image_path.replace('.png', '_detected.png')
        cv2.imwrite(output_path, image)
        return image

def process_images_in_folder(folder_path=".", marker_size=0.05):
    detector = ArUcoDetector(marker_size)
    
    image_files = []
    for ext in ["*.png", "*.jpg", "*.jpeg"]:
        image_files.extend(glob.glob(os.path.join(folder_path, ext)))
    
    if not image_files:
        print(f"No image files found in {folder_path}")
        return
    
    for image_path in sorted(image_files):
        print(f"Processing: {os.path.basename(image_path)}")
        results = detector.detect_markers(image_path)
        
        if results and results['ids'] is not None:
            for pose in results['poses']:
                print(f"Marker {pose['id']}: Distance={pose['translation']['distance']:.2f}m")
            detector.visualize_detection(image_path, results)

if __name__ == "__main__":
    process_images_in_folder()
