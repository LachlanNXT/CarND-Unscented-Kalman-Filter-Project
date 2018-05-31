# Extended Kalman Filter Project Writeup
Self-Driving Car Engineer Nanodegree Program

Things to note:
* works nice and easy under linux ubuntu vm following the instructions
* mostly used code from UKF lesson
* use linear KF equations for laser update
* use UKF transform for prediction, radar update
* check that angles are in domain -pi < x < pi !!!
* make sure to update the timestamp when initialising
* we have a good idea of initial position and speed, so P matrix can be initialised with smaller values
* make sure to check for numerical stability, i.e. no divide by zeros
* make sure to fill matrices with zeros before doing looping += assignments (got caught out on calculating S from Z_sigma points)!
* make variable names different enough... x vs x_ not great
* accuracy does seem to be a little better than EKF, but not much in it...

Here are examples of running the simulator with this code:

[//]: # (Image References)
[image1]: ./Screenshot.png
[image2]: ./Screenshot2.png

![alt text][image1]

![alt text][image2]
