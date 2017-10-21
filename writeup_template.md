## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

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

The first step in the pipeline is downsampling. It makes the most sense to do this first because it makes the reduces the load of the computations for the other algorithms. I found that a good leaf size that still preserved all the useful information was .01.

The second step was a passthrough filter to focus in on the interesting section of the image. I implemented a passthrough on 2 axes: y and z. The filter on z was to get rid of the table and the filter on y was to get rid of the edges of the table.

Lastly I used plane segmentation to get rid of the surface of the table. I found that setting the distance threshold to .01 did a reasonable job of filtering the table while preserving the integrity of the objects on top.


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

After separating out the objects in the image, I applied a statistical outlier filter to get rid of the noise in the image. I was able to set the cluster tolerance pretty low which got rid of a lot of the noise between objects. 

Applying the outlier filter made it a lot easier to cluster the pixels for each object.


#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

I used the sensor stick to generate training data for each of the objects in each world. I experimented with the number of training images until I ended up with an acceptable accuracy range for each object. I decided on a training set of 150 images which gave over 90% accuracy for each object:

[image1]: ./screenshots/accuracy1.PNG
[image2]: ./screenshots/accuracy2.PNG
[image3]: ./screenshots/accuracy3.PNG

![alt text][image1]
![alt text][image2]
![alt text][image3]

I then looped over each cluster and obtained both the normal and color histograms to create my feature vector. I can then input this feature vector into my trained classifier to get a label for each cluster.


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

I obtained the centroid for each point cloud by taking the average of all the coordinates. I then compared the labels from the list of expected objects to the ones detected by the machine learning. Finally I used that information to construct the appropriate request format and send it to the pick_place_routine.


[image4]: ./screenshots/labels1.PNG
[image5]: ./screenshots/labels2.PNG
[image6]: ./screenshots/labels3.PNG

![alt text][image4]
![alt text][image5]
![alt text][image6]



