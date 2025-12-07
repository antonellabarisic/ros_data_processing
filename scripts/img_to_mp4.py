import cv2
import os

def create_video_from_images():
    # --- CONFIGURATION ---
    image_folder = '/home/zdenka/elena/ASTRA/wind-farm/generated_data/instance'
    video_name = 'instance_output.mp4'
    fps = 24 
    # ---------------------

    valid_extensions = ('.png', '.jpg', '.jpeg', '.tiff', '.bmp')
    
    # 1. Get files
    files = [f for f in os.listdir(image_folder) if f.lower().endswith(valid_extensions)]
    
    # 2. Sort files 
    # This tries to sort numerically if filenames are like "1.png", "2.png"
    # Otherwise it falls back to alphabetical
    files.sort(key=lambda f: int(''.join(filter(str.isdigit, f))) if any(c.isdigit() for c in f) else f)

    if not files:
        print(f"No images found in: {image_folder}")
        return

    # 3. Read first image to get dimensions
    first_image_path = os.path.join(image_folder, files[0])
    frame = cv2.imread(first_image_path)
    
    if frame is None:
        print(f"Failed to read the first image: {first_image_path}")
        return

    height, width, layers = frame.shape

    # 4. Setup Video Writer
    # using 'mp4v' which is generally reliable on Ubuntu
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    video = cv2.VideoWriter(video_name, fourcc, fps, (width, height))

    print(f"Processing {len(files)} images from 'instance' folder...")

    # 5. Write frames
    for image in files:
        img_path = os.path.join(image_folder, image)
        frame = cv2.imread(img_path)
        
        # Safety check for dimensions
        if (frame.shape[1] != width) or (frame.shape[0] != height):
             frame = cv2.resize(frame, (width, height))
             
        video.write(frame)

    video.release()
    print(f"Video saved successfully as {video_name}")

if __name__ == "__main__":
    create_video_from_images()