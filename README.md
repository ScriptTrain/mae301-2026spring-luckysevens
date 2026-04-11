1. Start SITL/MAVProxy
2.  Start Joey's Proxy
3. start ai_control.py
4. Open Unity/ poll/ Status

1. SIMULATION
   Need to create a linux virtual environment and install ubuntu since ArduPilo truns on linux and not winodws (also protects your computer)
   To run the simulation you will need to instal ceritan python libraries (cant remebver off the to o fhte dome)
2. INTERFACES
   All info going into and out of the proxy will be through port 14560
   All info going into the simulation will come from port 14550
   Port 14551 is the return path from 14550 to 14560
   
