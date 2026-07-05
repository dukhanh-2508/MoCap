import cv2
import os

save_dir = "images"
os.makedirs(save_dir, exist_ok=True)

cap = cv2.VideoCapture(0)

count = 27

print("SPACE : capture")
print("ESC   : exit")

while True:
    ret, frame = cap.read()

    if not ret:
        break

    cv2.putText(frame,
                f"Captured: {count}",
                (20,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0,255,0),
                2)

    cv2.imshow("Capture", frame)

    key = cv2.waitKey(1)

    if key == 27:
        break

    elif key == ord(' '):
        filename = os.path.join(save_dir,
                                f"{count:03d}.png")
        cv2.imwrite(filename, frame)
        print(filename)
        count += 1

cap.release()
cv2.destroyAllWindows()