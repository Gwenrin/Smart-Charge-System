# Smart-Charge-System
# Abstract
This project tries to solve the problem of limited distance that a flying drone can make in one go. Nowadays drones are common tools to use in cases like surveillance, work in hazardous for man conditions, deliveries, etc. The distance that a drone can make in one round is quite limited. The time on one battery is oscillating in about 20 to 45 minutes on standard battery. This project is going to try solve these problems. 
# Defining problems
In companies like Amazon, drones are used in short cycles. A drones starts its journey in the main hub, then it's going all the path, drops the package and returns. Whole cycle lasts no longer than 60 minutes. After return, the battery is being replaced by a new, recharged one.
# 
This process seems quite uneffective. Imagine if we had a web of recharge stations on the roofs of the buildings. This solution become popular in modern publications. 
#
As it was mentioned before, the Amazon drones just drop the package midair. We would like to find the path for the drone to actually land on the spot. Moreover, it has to be in the right place on the spot itself to maximize charging effectivness. There is much more problems like maximum height of the landing pad, protection from dirt, snow, etc. Having this in mind we will focus on these topics:
- When drone is midair, how does it find the path to the charging spot and how to choose the best one (spot),
- When drone is near the spot, how does it perform the landing.
- If there is an obstacle, how the drone should react,
- How drone should react on other drones on the spot (queuing up the drones drones).

# How to find the path to the spot
In this section we will focus on pathfinding algorithms.


