# **Video mapping Node**

[//]: # (Image References)

[image1]: https://storage.googleapis.com/vision-198622-kiwibot-packages/cameras_mapping.png "Mapping"
[image2]: https://storage.googleapis.com/vision-198622-kiwibot-packages/video_threads.png "Threads"

## **Short Description**

This node is in charge of:

* Opening and reading the video stream of each camera connected through the USB hub and specified in the yaml file 'cam_ports'. Also handling errors of video stream readings.
* Using the N camera instant images from the video streams to map them into memory as a huge 3D array (see detailed description for this).

## **Topics/Services/Parameters/Env_vars**

This section describes the topics that the node uses and for what purpose.

### **Subscribers**

The node is subscribed to the following topics:

### **Publishers**     

The node publishes to the following topics:

### **Services**

The node exposes the following services:

 * /video_mapping/get_cameras_status_verbose
    * It checks the status of each of the camera connected to the robot and gives verbose status info
    * Doesn't receive any input param.
    * Returns a string array indicating a detailed status of the cameras.

### **Parameters**

This node doesn't use any specific parameters

### **Environment variables**

The node uses these environment variables:

 * MEMMAP_PATH: string, default: "/tmp"
    * Refers to the path that the library `easy_memmap` is going to use for handling the memory mapped video arrays.

 * CAM_PORTS_PATH: string, default: "$PWD/cam_ports.yaml"
    * Absolute path to camera ports file. Here the camera labels and ports are defined

 * VIDEO_WIDTH: string, default: 640
    * Video cameras streaming height

 * VIDEO_HEIGHT: string, default: 360
    * Video cameras streaming width


## **Technical/Implementation details**

### **Threads usage**

The main process spawns N(Cameras) different threads (one per camera) to read continuously the cameras stream. These threads are in the same context of the parent process, so this one can access their variables, i.e. the variables where the images are stored of the camera are stored. 

### **Mapping into memory**

Why mapping into memory? What does that even mean?
Sharing many images between processes is computational intensive, and adds many unwanted delays. That is why we decided to share images using shared memory between processes. That means that different processes can access a part of the memory that has the image arrays. That is done with `mmap` [Unix utility](https://en.wikipedia.org/wiki/Mmap). It maps a file into memory, and other processes that have the specific route to that file can also access that mapping.

It is relatively easy to use memory mapping on structured arrays. For that reason, we concatenate each BGR array (from each camera) into a big 3D array. We also concatenate the modified image that is going to be streamed to pilots console. The big array is then memory mapped using a library that internally uses `numpy.memmap`, which uses the Unix `mmap` seamless. 

After the mapping of that big array, each image can be read from memory in any process. For that, this node identifies each image within that 3D array with a label (see next section). We use a library for this for reading images in python, but there's also a C++ version of that in this repo, in utils/lib/easy_memmap.cpp.

### **External libraries**

The main library used to handle the memory mapping is [easy_memmap](https://github.com/charlielito/easy_memmap) that uses under the hood `numpy.memmap`. For pretty logging, the library [extended_rospylogs](https://github.com/charlielito/extended_rospylogs) is used, basically just for printing the `[ROS NODE NAME]` before each print sentence in order to be able to discriminate between print messages from different nodes. It also has other features that you can check on the library's page.

#### **easy_memmap**

This module exposes a class named `MultiImagesMemmap` that assumes the use of RGB image arrays, and each of them can be identified with a label. In our case, we use the following convention for naming: "CAM1", "CAM2"... "CAMN", that correspond to cameras from left to the right camera. The big 3D array that has all BGR images concatenated need to be also in the same order of the label array.

The constructor have the following signature (an example is provided):

```python
video_map = MultiImagesMemmap(mode = "w", name = "main_stream", labels = ["CAM1", "CAM2"... "CAMN"], memmap_path = "/tmp)
```
where the `name` is a predefined name that identifies that memory mapped array. (In our case, "main_stream" is used for this memmap sharing)


