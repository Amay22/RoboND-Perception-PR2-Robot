## Project: Perception Pick & Place

---

# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

The code for all the 3 steps can be found in [perception.py](https://github/amay22/) in the pcl

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

The RGB-D camera mounted on the robot gives us the Point-Cloud data. This Point-Cloud information is the most instrumental data in defining the objects around the robot. 
- First we reduce the noise by filtering out statistical outliers. The number of `neighbors` to analyze for each point is set to `5`, and the `standard deviation multiplier` to `0.05`. What this means is that all points who have a distance larger than 0.05 standard deviation and more than 5 neighboring points will be marked as outliers and removed.
- After we have removed the outliers we downsample the resulting Point-Cloud data using a voxel grid filter. I kept the `leaf size=0.005`. In downsampling if we keep  low enough leaf size we can retain more information. Voxel grid filter lets us downsample the data by taking a spatial average of the points in the point cloud data confined by each voxel. The set of points which lie within the bounds of a voxel are assigned to that voxel and statistically combined into one output point.
- We now have a voxel grid and we need to condense the point cloud data into the information in the region of interest by performing a pass-through filter in all three axes i.e. x, y and z. The pass-through filter allows us to crop any given 3D point cloud by specifying an axis with cut-off values along that axis.
- Now as we have the point cloud data narrowed down to the table and the objects on the table we can separate the table from the objects. This can be done by using the Ransac filter a.k.a Randome Sample Consenus filter. This filter correctly identifies the points in the data that belong to a particular model.
- We should only be left with point cloud data related to the objects at this point. We need to group the point cloud data into seperate objects. We can use the Euclidean clustering algorithm to group points into separate objects; it clubs all the nearby points into a cluster under the basic assumption that if points are close enough they will belong to the same object. This can be error prone if the objects are touching each other but since in most of the scenarios none of the points are touching each other this algo gives a fair estimate of the points.

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

In the previous exercise we have created a training model around the different type of objects that exist in the given problem namely  biscuits, soap, soap2, book, glue, sticky notes, snacks, and eraser.

SVM (Support Vector Machine) is the perfect solution to figure out what type of object we are looking at. The entire conecpt is based on Deep-Learning where we give the training model different type of objects and the training model analyzes the different type of features like shape, size, color, writings on object etc. The model analyzes also re-orients the same object into various random orientations so that when we pass an object that is not exactly the one the model is trained upon but maybe smaller or larger even then the SVM can recognize it.

- In this exercise we have spawned each object in 100 random orientations and computed it's features based on the point clouds resulting from each of the random orientations.
- The features are normalized histograms of the color and normal 3d vectors for each point in the point cloud captured by the virtual RGBD camera. The color is expressed in HSV format, because they capture the true color better, regardless of lighting conditions. The normal vectors for each point capture the shape of object. I used 64 bins for the histograms. They are normalized because it's the amount with respect to each other that matters not the actual amount.
- I then used the provided classifier which is a support vector machine classifier to condensed model that can be re-used while running the pr2 robot program.


Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.


Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



