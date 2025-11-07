import cv2, os, numpy as np, pyrealsense2 as rs

os.makedirs("dataset/images", exist_ok=True)
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg)

count = 0
while True:
    frames = pipe.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue
    img = np.asanyarray(color_frame.get_data())
    cv2.imshow('Capture', img)

    key = cv2.waitKey(1)
    if key == ord('s'):  # press 's' to save
        fname = f"dataset/images/frame_{count:04d}.jpg"
        cv2.imwrite(fname, img)
        print(f"Saved {fname}")
        count += 1
    elif key == ord('q'):
        break

pipe.stop()
cv2.destroyAllWindows()
