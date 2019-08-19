import csv 

class data_reader:
    """
    Reads csv file inside 'path' and stores image locations in a 3-dimensional list whose indexes are defined as follows:
        - i: capture indexes, defining the capture associated to an image 
        - j: camera indexes, defining the camera used to take an image
        - k: timestamp indexes, the timestamp index at which the image was taken
    """
    def __init__(self):
        """
        Constructor file with initialization of fundamental variables: timestamps sequence, camera labels and images 3-d list
        """
        self.path = None # Absolute parent path to csv file  
        
        self.timestamps = [[]] # Sequence of timestamps, one per capture 
        
        self.camera_labels = {} # Camera labels definition
        
        self.images = [[[]]] # 3-dimensional list storing image paths according to capture index, camera index and timestamp index  
        
        self.current_capture = None # Current capture used to read images from 
        self.current_camera = None # Current camera used to read images from

        # There variables are used in the _str__ method overloading
        self.line_count = None # Used to keep track of read rows from the csv file
        self.header_format = None # used to keep track of header rwo from csv file

    def __str__(self):
        """
        Overloads __str__ method with custom object attributes with information about:
            - Camera labels - which are the same for a given set of captures stored in a csv file
            - Captures
                - Number of cameras per capture
                - Number of timestamps per capture
        """
        message = '----DATA READER SUMMARY----\n\n'
        message += 'Camera labels:\n'
        message += self.camera_labels.__str__() + '\n'
        
        index = 0   
        for capture in self.images:
            message += '\nCapture {}\n'.format(index)
            message += '{} Cameras\n'.format(len(capture))
            message += '{} timestamps in total\n'.format(len(capture[0]))
            index += 1
        message += self.header_format
        message += '\n----Total lines read from the csv file: {} ----'.format(self.line_count)
        return message

    def load_data(self, path):
        """
        Loads timestamps, camera labels and image data from the csv file
        """
        self.path = path

        camera_index = 0    # Camera index used to traverse the csv file according to the camera order
        # Open csv file using the absolute path
        with open(self.path + '/data.csv') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            self.line_count = 0

            current_capture = None # Current capture index
            current_timestamp = None # Current timestamp value
            are_camera_labels_stored = False # Flag used to define when the camera labels have been stored
            is_new_capture = True # Extends image array dimensionality after reading a new capture from file

            # Read csv file one row at a time
            for row in csv_reader:
                # If line is header line print the column names
                if self.line_count == 0:
                    self.header_format = '\n----Column names are: {}----'.format(" | ".join(row))
                    self.line_count += 1
                # Else read each new row and store values in the 3-d array self.images
                else:
                    
                    # If current_capture has not been defined, define it with current row capture value
                    if current_capture == None:
                        current_capture = int(row[0])
                    # When capture value changes
                    elif current_capture != int(row[0]):
                        current_capture = int(row[0]) # Stores current capture in row
                        self.images.append([[]]) # Adds a new 2-d list to include information of new capture
                        self.timestamps.append([]) # Adds new list to include timestamp sequence of new capture
                        is_new_capture =  True # Set flag to true, since a new capture is being stored
                        current_timestamp = None # Reset current timestamp
                    
                    # If current timestamp has not been defined
                    if current_timestamp == None:
                        current_timestamp = row[1] # Define timestamp of current row
                        self.timestamps[current_capture].append(current_timestamp) # Append timestamp to the list of timestamps of current capture
                        camera_index = 0 # Init camera index as 0
                    # When current timestamp changes
                    elif current_timestamp != row[1]:
                        # Set are_camera_labels_stores flag to true
                        if not(are_camera_labels_stored):
                            are_camera_labels_stored = True
                        # Set is_new_capture to false 
                        if is_new_capture:
                            is_new_capture = False
                        current_timestamp = row[1] # Define timestamp of current row
                        self.timestamps[current_capture].append(current_timestamp) # Append timestamp to the list of timestamps of current capture
                        camera_index = 0 # Init camera index as 0
                    else:
                        # If is_new_capture extend dimensonality of current image capture 2-d list
                        if is_new_capture:
                            self.images[current_capture].append([])
                    
                    # If camera labels have not been store, store them in camera_labels
                    if not(are_camera_labels_stored):
                        if row[2] not in self.camera_labels: 
                            self.camera_labels[row[2]] = camera_index
                    
                    # Store image path in current_capture, camera_index and timestamp index positions
                    self.images[current_capture][camera_index].append(row[3])
                    
                    # Increase camera i
                    camera_index += 1
                    self.line_count += 1

        self.current_capture = 0
        self.current_camera = len(self.camera_labels) - 1
        print(self)

    def get_image(self, timestamp_idx, camera_idx, capture_idx):
        """
        Get an image given a timestamp index, camera type and capture index
        """
        return self.images[capture_idx][camera_idx][timestamp_idx]

if __name__ == '__main__':
    reader = data_reader()
    reader.load_data('/media/overcode/eagle/data_capture-08-08-19')
    print(reader)