# Vehicle_Controller_Design_Project
This project was completed on 12/10/2018, and was just uploaded to my Github account recently

In this project, I designed a LQR controller to control a car following a given trajectory. 
The goal of this project is to make the car finish the path as quickly as possible while still maintaining good accuracy.

The tasks for this project include:
1. Model the car using a bicycle model
2. linearize the model and decouple them into lateral and longitudinal models
3. Design a controller for both lateral and longitudinal models and tune the parameters to get the optimal peformance

The detailed modeling of the car can be found in the reference. The liearization process can be found in the "Calculation and result" folder

For this project, I designed 2 controllers. One is slow but accurate, the other is fast but barely satisfy the accuracy requirement. 
These two controllers can be found in controller_slow.py and controller_fast.py

Here are some simulation results:

### Fast Controller ###
* Total time:  74.55 s
* Maximun distance from reference:  5.7268493347214475 m
* Average distance from reference:  1.3019881974778402 m
![alt text](https://github.com/yymmaa0000/Vehicle_Controller_Design_Project/blob/master/Calculation%20and%20result/fast_controller_result_1.PNG)
![alt text](https://github.com/yymmaa0000/Vehicle_Controller_Design_Project/blob/master/Calculation%20and%20result/fast_controller_result_2.PNG)

### Slow Controller ###
* Total time:  157.45000000000002 s
* Maximun distance from reference:  5.6915563863996335 m
* Average distance from reference:  0.7979645575292 m
![alt text](https://github.com/yymmaa0000/Vehicle_Controller_Design_Project/blob/master/Calculation%20and%20result/slow_controller_result_1.PNG)
![alt text](https://github.com/yymmaa0000/Vehicle_Controller_Design_Project/blob/master/Calculation%20and%20result/slow_controller_result_2.PNG)


Reference
1. Rajamani Rajesh. Vehicle dynamics and control. Springer Science & Business Media,2011.
2. Kong Jason, et al. "Kinematic and dynamic vehicle models for autonomous driving
control design." Intelligent Vehicles Symposium, 2015.
