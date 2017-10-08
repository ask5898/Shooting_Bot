#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
                        

def talker() :
  
	rospy.init_node('talker',anonymous=True) 
        pub=rospy.Publisher('chatter',String)
        
        rate=rospy.Rate(10)

        while not rospy.is_shutdown() :
	    msg = raw_input("Enetr")
            pub.publish(msg)
             
	    
        
if __name__=="__main__" :
    try :
        talker()

    except rospy.ROSInterruptException :
        pass



                                                                                                                                                                            
