# self-driving-vehicle
The thesis presents the process of building the algorithm to follow the phanned trajectory, detect, track and avoid obstacles for autonomous vehicle model
We use the GPS RTK model to determine the location of the model, the STM32F4 microprocessor to control the orbit tracking model using the Stanley Controller algorithm,
together with the LoRa module to perform wireless data transmission between the Base station and Rover station as well as between model and PC. Fuzzy PID and PI controllers are used to get the best motor control results.

During the trajectory tracking model, we use images from the BumbleBee2 stereo camera to detect and track obstacles (such as: people, vehicles, lanes) and determine their position relative to the model
We use the Mobile-Net SSD model for obstacle detection, combined with the KCF tracking algorithm for obstacle tracking. When the obstacle is on the previously planned trajectory, we will perform obstacle avoidance based on The Curvature - Velocity Method. The algorithms for detecting, tracking and avoiding obstacles are implemented on The Robot Operating System (ROS) platform.
