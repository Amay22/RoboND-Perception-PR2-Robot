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

The code for all the 3 steps can be found in [perception.py](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/pr2_robot/scripts/perception.py).


The RGB-D camera mounted on the robot gives us the Point-Cloud data. This Point-Cloud information is the most instrumental data in defining the objects around the robot.
- First we reduce the noise by filtering out statistical outliers. The number of `neighbors` to analyze for each point is set to `5`, and the `standard deviation multiplier` to `0.05`. What this means is that all points who have a distance larger than 0.05 standard deviation and more than 5 neighboring points will be marked as outliers and removed.
- After we have removed the outliers we downsample the resulting Point-Cloud data using a voxel grid filter. I kept the `leaf size=0.005`. In downsampling if we keep  low enough leaf size we can retain more information. Voxel grid filter lets us downsample the data by taking a spatial average of the points in the point cloud data confined by each voxel. The set of points which lie within the bounds of a voxel are assigned to that voxel and statistically combined into one output point.
- We now have a voxel grid and we need to condense the point cloud data into the information in the region of interest by performing a pass-through filter in all three axes i.e. x, y and z. The pass-through filter allows us to crop any given 3D point cloud by specifying an axis with cut-off values along that axis.
- Now as we have the point cloud data narrowed down to the table and the objects on the table we can separate the table from the objects. This can be done by using the Ransac filter a.k.a Random Sample Consenus filter. This filter correctly identifies the points in the data that belong to a particular model.
- We should only be left with point cloud data related to the objects at this point.

Below is an example of the output of the voxelGrid output:

![voxelGrid pcl](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/images/voxelGrid_pcl.png)

Below is an example of the output of the passThrough filter output:

![passThrough filter](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/images/pass_through.png)

Below is an example of the output of the RANSAC algorithm using the function ransacFilter() filter output for the outliers i.e the points that do not match the background, mostly the objects:

![Ransac filter](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/images/ransac_outliers.png)

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

We have already deduced the point cloud data of the different objects in the previous exercise. Now we need to group the point cloud data into separate objects i.e. have a distinctive classification of objects in the PCL. We can use the Euclidean clustering algorithm to group points into separate objects; it clubs all the nearby points into a cluster under the basic assumption that if points are close enough they will belong to the same object. This can be error prone if the objects are touching each other but since in most of the scenarios none of the points are touching each other this algo gives a fair estimate of the points.

To identify which points in a point cloud belong to the same object we used the DBSCAN Algorithm (Density-Based Spatial Clustering of Applications with noise) aka Euclidian clustering to group points. The idea is that if a particular point belongs to a cluster, it should be near to lots of other points in that cluster.

I have used the kdtree search method as suggested in the lessons. KD tree is a very good spatial data espically point cloud data. I have used KD tree at work and in my self driving projects for Computer vision training data labelling and it works very well. The parameters used for Euclidean Clustering are as follows:

- Minimum Cluster Size : 50
- Maximum Cluster Size : 25000
- Cluster Tolerance : 0.02

As you can see the minimum and max are quite far apart. This is to make sure that we capture all the points of an object properly. Also very few objects so close by that this would cause a problem. Tolerance is also kept extremely low so that we can get a firm grasp of the object.

Below is an example of the output of the Euclidean Clustering algorithm i.e. the DBSCAN algorithm in this case:

![Euclidean Clustering](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/images/euclidean_clustering.png)


#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

In the previous project we have created a training model around the different type of objects that exist in the given problem namely  biscuits, soap, soap2, book, glue, sticky notes, snacks, and eraser. All the code for model building and feature extraction I have copied over into this project itself. It's in the [sensor_stick/scripts](https://github.com/Amay22/RoboND-Perception-PR2-Robot/tree/master/sensor_stick/scripts) folder.

The model can is computer here: [model.sav](https://github.com/Amay22/RoboND-Perception-PR2-Robot/tree/master/sensor_stick/scripts/model.sav) folder.


SVM (Support Vector Machine) is the perfect solution to figure out what type of object we are looking at. The entire conecpt is based on Deep-Learning where we give the training model different type of objects and the training model analyzes the different type of features like shape, size, color, writings on object etc. The model analyzes also re-orients the same object into various random orientations so that when we pass an object that is not exactly the one the model is trained upon but maybe smaller or larger even then the SVM can recognize it.

- In this exercise we have spawned each object in 100 random orientations and computed it's features based on the point clouds resulting from each of the random orientations.
- The features are normalized histograms of the color and normal 3d vectors for each point in the point cloud captured by the virtual RGBD camera. The color is expressed in HSV format, because they capture the true color better, regardless of lighting conditions. The normal vectors for each point capture the shape of object. I used 64 bins for the histograms. They are normalized because it's the amount with respect to each other that matters not the actual amount.
- I then used the provided classifier which is a support vector machine classifier to condensed model that can be re-used while running the pr2 robot program.

The resulting Confusion matrix with normalization can be found here:

![Normalized Confusion matrix](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/images/normalized_confusion_matrix.png)

The resulting Confusion matrix without normalization can be found here:

![Normalized Confusion matrix](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/images/non_normalized_confusion_matrix.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.


The test environment's cover three different combinations of various objects. The potential objects to be observed are:

- biscuits
- soap
- soap2
- book
- glue
- sticky_notes
- snacks
- eraser

#### Test World 1

Upon classification of the objects, a Picking List of the order in which an object is to be picked up and its resulting drop location was issued. Once we use the SVM model for clusterring and detection the code issues a command to the robot to move it's right or left arm to pick up the object and drop it off. The output file contains the information about the objects and the arm to use to pick up the object. It's quite seamless and fairly accurate as you can see in the image.

##### [OUTPUT 1](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/output/output_1.yaml)

Test World 1 resulted in 3 out of 3 accurate classification. Below is the screenshot of test1.world with the console output while I was doing it.

![Test World 1](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/images/test_world1.png)


#### Test World 2

This was actually very quick. Since there were more objects I was expecting it have some glitches but surprisingly it worked.

##### [OUTPUT 2](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/output/output_2.yaml)

Test World 2 resulted in 5 out of 5 accurate classification. Below is the screenshot of test1.world with the console output while I was doing it.

![Test World 2](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/images/test_world2.png)


#### Test World 3

Test world 3 by far has the most number of objects and some objects are partially behind the others so this is a challenging one.

##### [OUTPUT 3](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/output/output_3.yaml)

Test World 3 resulted in 7 out of 8 accurate classification. Below is the screenshot of test1.world with the console output while I was doing it. I am not particularly sad about not getting a perfect score because the glue is hiding right behind the book and it's kind of difficult to get.

![Test World 3](https://github.com/Amay22/RoboND-Perception-PR2-Robot/blob/master/images/test_world3.png)

#### Future Work and Enhancements


The position of the objects that we have captured in the output.yaml files is not extremely accurate. It gives a general bearing of the object which is quite good enough for the robot to go and get it. This is not a deal-breaker because the robot functions quite accurately. We should fix it though.

The project and code setup to look at the table-top which is on a proper horizontal ground but if we the table top had separate levels like a stair-case then code will not work. The project for classification probably won't work if two or more objects are stacked on top of another which is a very practical use-case in warehouses.

In Test world 3 we were not able to classify all the objects accurately because the objects were hiding behind another object. One way to improve is by extracting more feature like Histogram Of Gradient (HOG) or different colour spaces in our SVM. Also we could rotate the view and get true 3D PCL information and if we have that we can analyze it from all points of view; think of it like a hologram like they show in movies.
