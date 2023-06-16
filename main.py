def photo_process(prev):
    import numpy as np
    import random
    import cv2

    image = cv2.imread("test2.png")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)[1]
    
    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=2)
    

    cnt = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    
    # If we have multiple contours, most likely closer to light source, so increase brightness cutoff
    if len(cnt) > 2:
        thresh = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=2)
        cnt = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    
    # If we still have multiple contours, check to see if we have recorded a previous area
    # If yes, then find areas in new list greater than previous but less than a max area of
    # 200. If no, then select the smallest area out of the group
    if len(cnt) > 1:
        areas = [0]*len(cnt)
        for idx, val in enumerate(cnt):
            areas[idx] = cv2.contourArea(val)
        if prev == 0:
            area = min(areas)
            M = cv2.moments(cnt[areas.index(area)])
        else:
            for idx, a in enumerate(areas):
                if a > prev and a < 200:
                    area = a
                    M = cv2.moments(cnt[idx])
                    break
    else:
        M = cv2.moments(thresh)
        area = cv2.contourArea(cnt[0])

    cv2.imshow("image", thresh)
    cv2.waitKey(0)
     
    # Calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    
    print("X: %d" %cX)
    print("Y: %d" %cY)
    print("Area: %d" %area)

    # put text
    cv2.circle(image, (cX, cY), 10, (0, 0, 255), 2)
    

    cv2.imshow("image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return cX, cY, area



def server_post (duty1, duty2, duty3, duty4):
    import MySQLdb
    
    db = MySQLdb.connect("localhost","root","","duty_cycle")
    insertrec = db.cursor()

    sqlquery = "INSERT INTO dht11 (duty1, duty2, duty3) VALUES (" + str(duty1) + ", " + str(duty2) + ", " + str(duty3) + ")"
    insertrec.execute(sqlquery)
    db.commit()
    db.close()
    print('Record created successfully') 


def photo_retrieve (URL):
    from selenium import webdriver
    import urllib.request
    import time
    
    driver = webdriver.Chrome()
    driver.get(URL) # "http://192.168.1.110/"
    driver.maximize_window()
    time.sleep(5)
    button = driver.find_element("id", "get-still")
    button.click()

    img = driver.find_element("id", "stream")
    src = img.get_attribute('src')
    urllib.request.urlretrieve(src, "test.png")

    driver.close()


if __name__ == "__main__":
    
    
    cx, cy, area = photo_process(20)