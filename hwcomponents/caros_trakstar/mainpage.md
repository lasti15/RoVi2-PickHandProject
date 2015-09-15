\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc/generate-toc again -->
**Table of Contents**

- [caros_trakstar](#carostrakstar)
- [Using the trakstar node #](#using-the-trakstar-node-)
- [Calibration](#calibration)

<!-- markdown-toc end -->

# caros_trakstar #
The Trakstar component is a ros wrapper for the magnetic pose tracker system from TrakStar. It consist of 4 pose trackers and some digital io inputs.

IMPORTANT: the driver needs to be running. This is done by making sure ATCdaemon64 is running. Start it in the terminal and test if the connection is find using the APITest64 which is normally found in "/opt/3DGuidance.Rev.E.64/Binaries". 


# Using the trakstar node # 

The trakstar node is implemented as a nodelet and needs to be started as such. A standalone node is also available using the executable "caros_trakstar" which wraps the nodelet functionality. The trakstar node implements the Pose sensor interface, button interface and the caros node interface. 

To launch the node as a standalone nodelet do
	
	roslaunch caros_trakstar mytrakstar.launch
	
If the ATCdaemon64 is not running then the node will fail its initialization sequence and enter a caros fatal node error state, a restart of the node and making sure ATCdaemon is running, is required. 

There are a few parameters that matters to the trakstar driver

* *rate* - the rate in which pose samples are published with. The trakstar has a limitation depending on version and how many recievers that are used, however when using 4 recievers the fastest sampling rate is approximately 240hz.
* *frame* - the name of the frame in which the samples are described. This will default to **TrakstarBase**. 
* *calibration_data* - the path to the XML file containing calibration data for the Trakstar. **NOTICE:** This is currently not used.



# Calibration  #
TODO: The calibration is still not enabled in the new implementation.



