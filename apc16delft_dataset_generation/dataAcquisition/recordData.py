## Delft Robotics
# 03-03-2016
# X.Gerrmann
# 
# Changelog:
# 03-03-2016: classes are placed in seperate folders. The foldernames are formatted like this: class_day-month-year
# 03-03-2016: Added user input for classname and number of frames


import numpy as np
import cv2
import os
import datetime

imageDir = 'originalImages/'

print '\n#####\nStarting data acquisition\n#####\n'
cap = cv2.VideoCapture(1)

# Ask user for the number of frames to record
answer = 'n'
while(answer != 'y'): 
    number_frames = int(raw_input("Number of frames to record: \t")) 
    answer = raw_input('\nNumber of frames is: %d \nAgree? y/n \t' % number_frames) 

# Ask user for the classname
answer = 'n'
while((answer == 'n') & (answer != 'quit')): 
    class_name = raw_input("Give the class name: ") 
    answer = raw_input('\nClassname is: %s \nAgree? y/n or quit \t' % class_name) 

if(answer == 'y'):  
    print '\nStart acquisition'
    print '\nAbort with \'q\''
    
    # Create directory for data if it does not already exist
    now = datetime.datetime.now()
    dateString = now.strftime("%d-%m-%Y")
    classDir = imageDir+class_name+'_'+dateString+'/'
    if not os.path.exists(classDir):
        os.makedirs(classDir)
    
    # Start recording
    for frameNumber in range(1,number_frames+1) :
        # DELAY ????
        print 'Frame: %d'%frameNumber
        # Capture frame-by-frame
        ret, frame = cap.read()
    
        # Our operations on the frame come here
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Display the resulting frame
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        # output image to file
        #file = "~/shared/DRapc2016/PPMImages/%d.ppm" % frameNumber
        file = classDir+"%07d.ppm" % frameNumber
        print(file)
        
        # A nice feature of the imwrite method is that it will automatically choose the
        # correct format based on the file extension you provide. Convenient!
        cv2.imwrite(file, frame)

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    
else:
    print 'Quit data acquisition'
