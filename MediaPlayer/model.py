class data_reader:
    """
    Reads csv file inside 'path' and stores image locations in a 3-dimensional list whose indexes are defined as follows:
        - i: capture indexes, defining the capture associated to an image 
        - j: camera indexes, defining the camera used to take an image
        - k: timestamp indexes, the timestamp index at which the image was taken
    """
    def __init__(self, path):
        """
        Constructor file with initialization of fundamental variables: timestamps sequence, camera labels and images 3-d list
        """
        self.path = path # Absolute parent path to csv file  
        
        self.timestamps = [[]] # Sequence of timestamps, one per capture 
        
        self.camera_labels = {} # Camera labels definition
        
        self.images = [[[]]] # 3-dimensional tensor storing image paths according to capture index, camera index and timestamp index  

    def load_data(self):
        """
        Loads timestamps, camera labels and image data from the csv file
        """
                         
        camera_index = 0    # Camera index used to traverse the csv file according to the camera order
        # Open csv file using the absolute path
        with open(self.path + '/data.csv') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0

            current_capture = None # Current capture index
            current_timestamp = None # Current timestamp value
            are_camera_labels_stored = False # Flag used to define when the camera labels have been stored
            is_new_capture = True # Extends image array dimensionality after reading a new capture from file

            # Read csv file one row at a time
            for row in csv_reader:
                # If line is header line print the column names
                if line_count == 0:
                    print('Column names are {}'.format(", ".join(row)))
                    line_count += 1
                # Else read each new row and store values in the 3-d array self.images
                else:
                    
                    # If current_capture has not been defined, define it
                    if current_capture == None:
                        current_capture = int(row[0])
                    elif current_capture != int(row[0]):
                        current_capture = int(row[0])
                        self.images.append([[]])
                        self.timestamps.append([])
                        is_new_capture =  True
                        current_timestamp = None
                        print('There is a new capture!: {}'.format(current_capture))
                    
                    if current_timestamp == None:
                        current_timestamp = row[1]
                        timestamps[current_capture].append(current_timestamp)
                        camera_index = 0
                    elif current_timestamp != row[1]:
                        if not(camera_labels_stored):
                            camera_labels_stored = True
                        if new_capture:
                            new_capture = False
                        current_timestamp = row[1]
                        timestamps[current_capture].append(current_timestamp)
                        camera_index = 0
                    else:
                        if new_capture:
                            images[current_capture].append([])
                            print('-----------This just happened!-------------')
                    
                    if not(camera_labels_stored):
                        if row[2] not in camera_labels: 
                            camera_labels[row[2]] = camera_index

                    print(current_capture, camera_index, row[3])

                    images[current_capture][camera_index].append(row[3])
                    
                    camera_index += 1

                    print('Picture taken in capture {0} with timestamp {1} and camera {2}.\nLocation is: {3}'.format(row[0],row[1],row[2],row[3]))
                    line_count += 1
            print(camera_labels)
            print('\nSummary of storage\n')
            index = 0
            for capture in images:
                print('Capture {}\n'.format(index))
                print('{} Cameras\n'.format(len(capture)))
                print('{} timestamps in total\n'.format(len(capture[0])))
                index += 1
            print('Processed {} lines.'.format(line_count))
    
    def get_image(self, timestamp_idx, camera_idx, capture_idx):
        """
        Get an image given a timestamp index, camera type and capture index
        """
        pass

