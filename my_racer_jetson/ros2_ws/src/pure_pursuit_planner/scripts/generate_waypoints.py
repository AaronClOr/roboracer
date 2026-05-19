import cv2
import numpy as np
import yaml
import csv
from skimage.morphology import skeletonize
from scipy.spatial.distance import cdist
from scipy.interpolate import splprep, splev

def load_map_info(yaml_file):
    """Parses the ROS standard map YAML file."""
    with open(yaml_file, 'r') as f:
        map_info = yaml.safe_load(f)
    return map_info['resolution'], map_info['origin']

def process_image(image_file):
    """Loads and heavily cleans the map to handle real-world SLAM noise."""
    img = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
    
    # 1. Threshold: In ROS, 255 is free space, 0 is obstacle, 205 is unknown.
    # We turn everything above 240 into purely drivable space (1), else (0).
    _, binary_map = cv2.threshold(img, 240, 1, cv2.THRESH_BINARY)
    
    # 2. Morphological Cleaning (Crucial for Real SLAM Maps)
    kernel = np.ones((5, 5), np.uint8)
    # Remove small black dots (noise) inside the white track
    clean_map = cv2.morphologyEx(binary_map, cv2.MORPH_CLOSE, kernel)
    # Remove small white branches sticking out of the track
    clean_map = cv2.morphologyEx(clean_map, cv2.MORPH_OPEN, kernel)
    
    return clean_map, img.shape

def sort_pixels(pixel_points):
    """Sorts the pixels into a continuous sequential line."""
    sorted_pixels = [pixel_points[0]]
    remaining_pixels = pixel_points[1:].tolist()
    
    print(f"Sorting {len(pixel_points)} raw points (this may take a few seconds)...")
    while remaining_pixels:
        current_point = sorted_pixels[-1]
        # Find the closest remaining pixel
        distances = cdist([current_point], remaining_pixels)[0]
        closest_idx = np.argmin(distances)
        # Move it to the sorted list
        sorted_pixels.append(remaining_pixels.pop(closest_idx))
        
    return np.array(sorted_pixels)

def generate_robust_waypoints(yaml_file, image_file, output_csv, waypoint_spacing=0.1):
    print("1. Loading map...")
    resolution, origin = load_map_info(yaml_file)
    clean_map, (height, width) = process_image(image_file)
    
    print("2. Extracting centerline...")
    skeleton = skeletonize(clean_map)
    pixel_points = np.column_stack(np.where(skeleton > 0)) # Format: [y, x]
    
    if len(pixel_points) == 0:
        print("Error: No drivable space detected.")
        return

    print("3. Connecting the dots...")
    sorted_pixels = sort_pixels(pixel_points)
    
    # Convert pixels to Real-World ROS Coordinates (meters)
    # ROS map origin is the bottom-left corner of the image
    real_x = (sorted_pixels[:, 1] * resolution) + origin[0]
    real_y = ((height - sorted_pixels[:, 0] - 1) * resolution) + origin[1]
    
    print("4. Smoothing and Resampling (B-Spline)...")
    # splprep fits a B-spline curve to our points. 
    # s controls the smoothness (higher = smoother, but cuts corners more)
    # per=True tells the algorithm this is a closed racing loop
    tck, u = splprep([real_x, real_y], s=2.0, per=True)
    
    # Calculate how many points we need to achieve our desired spacing (e.g. 0.1m)
    # Approximate track length by summing straight line distances
    track_length = np.sum(np.sqrt(np.diff(real_x)**2 + np.diff(real_y)**2))
    num_points = int(track_length / waypoint_spacing)
    
    # Generate the perfectly spaced, smoothed points
    u_new = np.linspace(0, 1.0, num_points)
    smooth_x, smooth_y = splev(u_new, tck)
    
    print("5. Saving to CSV...")
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y'])
        for i in range(len(smooth_x)):
            writer.writerow([smooth_x[i], smooth_y[i]])
            
    print(f"Success! Saved {num_points} smooth waypoints to {output_csv}")

# ==========================================
# CONFIGURATION
# ==========================================
MAP_YAML = 'levine_obs.yaml'  # Change to your file name
MAP_IMG = 'levine_obs.png'    # Change to your file name (.png or .pgm)
OUTPUT_FILE = 'global_waypoints.csv'
WAYPOINT_SPACING_METERS = 0.1 # Distance between each waypoint in meters

if __name__ == '__main__':
    generate_robust_waypoints(MAP_YAML, MAP_IMG, OUTPUT_FILE, WAYPOINT_SPACING_METERS)