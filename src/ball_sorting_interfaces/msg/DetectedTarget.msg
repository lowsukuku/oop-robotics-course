# Шар, обнаруженный системой технического зрения.

uint8 BLUE=0
uint8 RED=1

string name                     # идентификатор шара: ball_0 … ball_19
uint8 color                     # BLUE или RED
geometry_msgs/Point position    # центр шара в системе основания манипулятора, м
