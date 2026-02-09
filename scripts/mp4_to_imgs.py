"""Extract frames from a video or multiple videos at a specified rate.

Edit `INPUT_PATH` to point to either:
  - A single video file (e.g., "video.mp4")
  - A folder containing multiple .mp4 files

`FRAME_RATE` controls how many frames to extract per second.
Frames are saved to a new folder named after each video file.
"""

from pathlib import Path
import cv2

# Path to a single video file or a folder containing .mp4 files.
INPUT_PATH = "/home/antonella/Odonata/KAUFLAND/snimanje_17122025/"

# Number of frames to extract per second (e.g., 1 = 1 frame/sec, 5 = 5 frames/sec).
FRAME_RATE = 1


def extract_frames(video_path: str, frame_rate: float) -> None:
    video_path_obj = Path(video_path)
    if not video_path_obj.exists():
        print(f"Error: Video file '{video_path}' not found.")
        return

    # Create output folder named after the video (without extension).
    output_dir = video_path_obj.parent / video_path_obj.stem
    output_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"Error: Cannot open video '{video_path}'.")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        print("Error: Could not determine video FPS.")
        cap.release()
        return

    # Calculate the interval between frames to extract.
    frame_interval = int(fps / frame_rate)
    if frame_interval < 1:
        frame_interval = 1

    frame_idx = 0
    saved_count = 0
    video_name = video_path_obj.stem

    print(f"Extracting frames from '{video_path}' at {frame_rate} frames/sec (interval: {frame_interval})...")
    print(f"Output folder: {output_dir}")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx % frame_interval == 0:
            output_file = output_dir / f"{video_name}_{saved_count:06d}.jpg"
            cv2.imwrite(str(output_file), frame)
            saved_count += 1

        frame_idx += 1

    cap.release()
    print(f"Extracted {saved_count} frames to {output_dir}")


def main() -> None:
    input_path = Path(INPUT_PATH)
    
    if not input_path.exists():
        print(f"Error: Path '{INPUT_PATH}' does not exist.")
        return
    
    # Determine if it's a file or directory
    if input_path.is_file():
        # Process single video file
        if input_path.suffix.lower() == ".mp4":
            extract_frames(str(input_path), FRAME_RATE)
        else:
            print(f"Error: '{INPUT_PATH}' is not an MP4 file.")
    elif input_path.is_dir():
        # Process all .mp4 files in the directory
        video_files = sorted(input_path.glob("*.mp4")) + sorted(input_path.glob("*.MP4"))
        
        if not video_files:
            print(f"No .mp4 files found in '{INPUT_PATH}'.")
            return
        
        print(f"Found {len(video_files)} video(s) to process.\n")
        for video_file in video_files:
            extract_frames(str(video_file), FRAME_RATE)
            print()  # Blank line between videos
    else:
        print(f"Error: '{INPUT_PATH}' is neither a file nor a directory.")


if __name__ == "__main__":
    main()
