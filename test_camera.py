import cv2

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while True:
    ret, img = cap.read()

    cv2.imshow('frame', img)
    res = cv2.waitKey(30)

    if res == ord('q'):
        break;

cap.release()
cv2.destroyAllWindows()