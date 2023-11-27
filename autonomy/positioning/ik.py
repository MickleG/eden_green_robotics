import math

Length = float(350)  #linkage lengths
EndEffect = float(65) # center of end effector to outer linkage pin center 

linearStage = float(250) # travel of linear actuators
OuterPinDist = float(56.5) # distance from edge of carriage mount to center of 8mm pin (x dist)

ActuatorOffset = float(92.5 + 30) # edge of actuator (inside edge of aluminum bloc) to center of macron rail
Offset = float(ActuatorOffset + OuterPinDist) # center of macron rail to outer linkage pin center (fully ext)

Z_limit = float(50) #mm  how close endeffector can get to macron (3 inches offset)
X_max_limit = float(Offset + linearStage) #mm  how far can either (carriage mount outer pin) travel in x (fully fold)
X_min_limit = float(Offset) #mm  how close ...^ (from centerline of macron) (this is fully ext)

AB_max = float((EndEffect + math.sqrt((Length**2) - (Z_limit**2)))*2) # asssuming A = B what is max value of A and B (position of linear stages)

z_cord = float(input("Desired Z Coordinate (mm): "))

while(z_cord < Z_limit):
    z_cord = float(input("Error! Z coordinate will break posture. Please Try Again: "))

theta = math.asin(z_cord/Length) # theta(left) = theta(right) due to linkage configuration

AB = ((Length * math.cos(theta)) + EndEffect)*2 # distance from center of left carriage to center of right carriage

OutSpace = X_max_limit - (AB/2) # distance from either carriage to outer (fully folded) boundary
InSpace = (AB/2) - X_min_limit # disannce from either carriage to inner (fully ext) boundary
Space = min(OutSpace, InSpace) # whichever boundary is closest is the limiting factor of x movement

print("\n Possible X Range: ["+str(round(Space*-1, 4)) + "    " + str(round(Space, 4)) + "]\n")

x_cord = float(input("Input desired X coordinate (mm): "))

while(abs(x_cord) > Space):
    prompt = "Error! X coordinate will break posture. Please Try Again: "
    x_cord = float(input(prompt))

Ax = (-1*AB/2) + Offset + x_cord # left linear stage relative cord
Bx = (AB/2) - Offset + x_cord # right linear stage relative cord

print("\nRelative Carriage Coordinate (Left) :   " + str(round(Ax,4)))
print("Relative Carriage Coordinate (Right) :   " + str(round(Bx,4)))
