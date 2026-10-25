# Solar illumination state of a vehicle, from STK's Lighting Times /
# Eclipse Times data providers.

std_msgs/Header header

string object_path

uint8 UMBRA = 0
uint8 PENUMBRA = 1
uint8 SUNLIT = 2

uint8 state
