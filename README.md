# AP_Experiments
Active Perception simulated experiments for my thesis

This code runs in matlab and simulates active perception for bearing only target localisation using an EKF and the offline AP framework presented in 

`N. Atanasov, J. Le Ny, K. Daniilidis and G. J. Pappas, "Information acquisition with sensing robots: Algorithms and error bounds," 2014 IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, 2014, pp. 6447-6454, doi: 10.1109/ICRA.2014.6907811.`

A camera moves through space to localise a single target in 3D simulating a fruit harvesting step. The camera trajectory ends at the target. Several trajectory types are tested. 

It requires some matlab toolboxes, including VR and Computer Vision. 

Start by running `buildExperiment.m`
