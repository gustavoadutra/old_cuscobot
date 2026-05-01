import os
import glob
import shutil
import numpy as np
from datetime import datetime

# ==========================================
# CONFIGURATION
# ==========================================
# The folder where your wrongly-timestamped images are right now
INPUT_FOLDER = "dataset_20260321_115430/rtsp_images" 

# The new folder where the corrected images will be saved
OUTPUT_FOLDER = "dataset_20260321_115430/rtsp_images_fixed"

# The EXACT date of the recording (Required to generate correct nanoseconds)
DATE_STRING = "2026-03-21" 

# The real start and end times you measured
START_TIME_STR = f"{DATE_STRING} 11:54:31"
END_TIME_STR   = f"{DATE_STRING} 11:55:11"

def main():
    if not os.path.exists(INPUT_FOLDER):
        print(f"[ERROR] Input folder not found: {INPUT_FOLDER}")
        return

    os.makedirs(OUTPUT_FOLDER, exist_ok=True)

    # 1. Convert your human times to Nanoseconds (Unix Epoch)
    fmt = "%Y-%m-%d %H:%M:%S"
    start_dt = datetime.strptime(START_TIME_STR, fmt)
    end_dt = datetime.strptime(END_TIME_STR, fmt)

    # Convert to seconds, then to nanoseconds (multiply by 1 billion)
    start_ns = int(start_dt.timestamp() * 1e9)
    end_ns = int(end_dt.timestamp() * 1e9)

    # 2. Get all images and sort them chronologically by their OLD, wrong timestamps
    # This ensures the sequence order remains perfectly intact
    image_paths = sorted(glob.glob(os.path.join(INPUT_FOLDER, "*.png")))
    total_images = len(image_paths)

    if total_images == 0:
        print("[ERROR] No images found in input folder.")
        return

    print(f"[INFO] Found {total_images} images.")
    print(f"[INFO] Interpolating timestamps from {START_TIME_STR} to {END_TIME_STR}...")

    # 3. Create a perfectly spaced array of new timestamps
    # np.linspace divides the 40 seconds evenly across all your frames
    new_timestamps = np.linspace(start_ns, end_ns, num=total_images, dtype=np.int64)

    # 4. Copy and Rename
    for i, old_path in enumerate(image_paths):
        new_filename = f"{new_timestamps[i]}.png"
        new_path = os.path.join(OUTPUT_FOLDER, new_filename)
        
        # Copy to the new folder (safe operation, doesn't delete originals)
        shutil.copy2(old_path, new_path)

        if i % 100 == 0:
            print(f"Processed {i}/{total_images} images...")

    print("=" * 50)
    print(f"[SUCCESS] Repacked {total_images} images!")
    print(f"FPS for this sequence was: {total_images / 40.0:.2f} fps")
    print(f"They are safely stored in: {OUTPUT_FOLDER}")
    print("=" * 50)
    print("NEXT STEP: Rename 'rtsp_images_fixed' to 'rtsp_images' before running your VO.")

if __name__ == "__main__":
    main()