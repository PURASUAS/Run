import time

import cv2

cap = cv2.VideoCapture(0)

photo_count = 1000


while True:
    # Kameradan bir frame yakalama
    ret, frame = cap.read()


    # Klavyeden 'q' tuşuna basılırsa döngüyü sonlandır
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    photo_count+=1
    cv2.imwrite(f'{photo_count}_Photo.png', frame)
    time.sleep(0.5)
# Kamera kapatma
cap.release()
cv2.destroyAllWindows()
