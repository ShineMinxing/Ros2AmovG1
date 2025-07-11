sudo chmod 777 /dev/ttyUSB0  
https://docs.amovlab.com/gimbalwiki/#/src/G1/doc/AmovGimbalSDK4C++

sudo apt purge brltty
盲文软件会影响串口


ros2 run image_transport republish compressed raw \
  --ros-args \
    --remap in/compressed:=/SMX/GimbalCamera_Compressed \
    --remap out:=/SMX/Go2Camera_Compressed2Raw