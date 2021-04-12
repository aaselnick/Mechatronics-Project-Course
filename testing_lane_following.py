import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow("TrackingYellow")
cv2.createTrackbar("LH", "TrackingYellow", 0, 255, nothing)
cv2.createTrackbar("LS", "TrackingYellow", 164, 255, nothing)
cv2.createTrackbar("LV", "TrackingYellow", 128, 255, nothing)
cv2.createTrackbar("UH", "TrackingYellow", 32, 255, nothing)
cv2.createTrackbar("US", "TrackingYellow", 255, 255, nothing)
cv2.createTrackbar("UV", "TrackingYellow", 194, 255, nothing)

cv2.namedWindow("TrackingPurple")
cv2.createTrackbar("LH", "TrackingPurple", 110, 255, nothing)
cv2.createTrackbar("LS", "TrackingPurple", 14, 255, nothing)
cv2.createTrackbar("LV", "TrackingPurple", 71, 255, nothing)
cv2.createTrackbar("UH", "TrackingPurple", 150, 255, nothing)
cv2.createTrackbar("US", "TrackingPurple", 112, 255, nothing)
cv2.createTrackbar("UV", "TrackingPurple", 167, 255, nothing)

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    #channel_count = img.shape[2]
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_the_lines(img,lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(blank_image, (x1,y1), (x2,y2), (0,255,0), thickness=3)
            
    img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    return img

def process(image):
    #print(image.shape)
    height = image.shape[0]
    width = image.shape[1]

    region_of_interest_vertices = [
        (0, height),
        ((width/2)+25, (height/2)-15),
        (width, height)
    ]

    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    canny_image = cv2.Canny(gray_image, 100, 200)
    cropped_image = region_of_interest(canny_image, np.array([region_of_interest_vertices], np.int32))

    lines = cv2.HoughLinesP(cropped_image,
                            rho=6,
                            theta=np.pi/60,
                            threshold=100,
                            lines=np.array([]),
                            minLineLength=50,
                            maxLineGap=15)
    
    #print(lines)
    if np.any(lines) == None:
        return image
    else:
        image_with_lines = draw_the_lines(image, lines)
    return image_with_lines

cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    ret, frame = cap.read()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    #YELLOW MASK###############################
    l_h = cv2.getTrackbarPos("LH", "TrackingYellow")
    l_s = cv2.getTrackbarPos("LS", "TrackingYellow")
    l_v = cv2.getTrackbarPos("LV", "TrackingYellow")
    
    u_h = cv2.getTrackbarPos("UH", "TrackingYellow")
    u_s = cv2.getTrackbarPos("US", "TrackingYellow")
    u_v = cv2.getTrackbarPos("UV", "TrackingYellow")
    
    l_y = np.array([l_h, l_s, l_v])
    u_y = np.array([u_h, u_s, u_v])
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    yellow_mask = cv2.inRange(hsv, l_y, u_y)

    res_yellow = cv2.bitwise_and(frame, frame, mask=yellow_mask)
    ###########################################
    
    
    #PURPLE MASK##############################
    l_h2 = cv2.getTrackbarPos("LH", "TrackingPurple")
    l_s2 = cv2.getTrackbarPos("LS", "TrackingPurple")
    l_v2 = cv2.getTrackbarPos("LV", "TrackingPurple")
    
    u_h2 = cv2.getTrackbarPos("UH", "TrackingPurple")
    u_s2 = cv2.getTrackbarPos("US", "TrackingPurple")
    u_v2 = cv2.getTrackbarPos("UV", "TrackingPurple")
    
    l_p = np.array([l_h2, l_s2, l_v2])
    u_p = np.array([u_h2, u_s2, u_v2])
    
    purple_mask = cv2.inRange(hsv, l_p, u_p)
    
    res_purple = cv2.bitwise_and(frame, frame, mask=purple_mask)
    ##########################################
    
    
    #cv2.imshow('frame', frame)
    yellow_lines = process(res_yellow)
    
    #cv2.imshow('purple_mask', purple_mask)
    #cv2.imshow('res_purple', res_purple)
    
    main_view = cv2.add(yellow_lines,frame)
    cv2.imshow('main_view', main_view)
    
    if cv2.waitKey(1) & 0xFF == 27:
        break
    
cap.release()    
cv2.destroyAllWindows()
