import os
import pandas as pd
import time
import sys
import rospy

args = rospy.myargv(argv=sys.argv)

print("args", args)

id = int(args[1])

i = id
usr = str(args[2])




print("user",usr)

global DATA
	

try:
    os.system("sudo rm -r eece5554")
except:
    try:
        os.system("sudo rm -r EECE5554")
    except:
        pass

try :
    print("cloning git clone git@github.com:"+usr+"/eece5554.git")
    os.system("git clone git@github.com:"+usr+"/eece5554.git")
except:

    comment = "Can not clone repository. Have we been given the correct permissions ?"
    
    print(" ")
    print(" ")
    print(" ")
    print(comment)

    sys.exit(0)


    #return comment,penalty

try :
    os.chdir(os.getcwd()+"/eece5554/gnss")
except:
    print("except", os.getcwd())
    try :
        os.chdir(os.getcwd()+"/EECE5554/gnss")
        print("try os.getcwd=", os.getcwd())
    except :

        comment = "No gnss folder"
        penalty = 20
        
        print(" ")
        print(" ")
        print(comment)
        sys.exit(0)

        #return comment,penalty


# VERIFIED THAT LAB1 EXISTS


#os.system('catkin_make')
