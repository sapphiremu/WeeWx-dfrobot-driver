# WeeWx-dfrobot-driver

Original driver: https://github.com/weewx/weewx/blob/master/bin/weewx/drivers/ws1.py
ALL credit for the original file is clearly stated in the modded driver.

## Note regarding versions >= 0.05
This version of the driver includes an algorithm to "smooth" the wind direction which is not permitted in the WeeWX driver guide. It was included because the DFRobot hardware is somewhat archane in design and does not report accurate wind direction in the short term. The "smoothing" (by default) creates an average direction based upon the immediate previous 30 direction entries.
To disable this functionality, set the LOW_RES_VANE variable to False on line 54.
