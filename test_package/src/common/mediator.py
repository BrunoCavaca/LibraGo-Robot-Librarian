"""
As the name suggests, mediates the communication between states
"""
class Mediator:
    def __init__(self):
        self.speechDetected = ""
        self.bookRequested = ""
        self.conversationMode = False

        self.navigationComplete = False

        self.robotPoseAtDetection = None
        self.detectionPerformed = False

        self.bookPosition = None

        self.reRunDetections = False
        self.detectionFailed = False

        self.gripperEmpty = False


    """
    In charge of holding pre-processed speech
    """
    def setSpeechDetected(self,speech):
        self.speechDetected = speech

    def getSpeechDetected(self):
        return self.speechDetected

    """
    In charge of setting conversational mode on/off and returning current status
    """
    def turnOnConversationalMode(self):
        self.conversationalMode = True

    def turnOffConversationalMode(self):
        self.conversationalMode = False


    def returnConversationalMode(self):
        return self.conversationalMode


    """
    In charge of getting/setting the book name requested
    """
    def setRequestedBookName(self,bookName):
        self.bookRequested = bookName

    def getRequestedBookName(self):
        return self.bookRequested


    """
    In charge of getting/setting if navigation is complete
    (mediates with the YOLO Detection module)
    """
    def setNavigationStatus(self):
        self.navigationComplete = not self.navigationComplete

    def getNavigationStatus(self):
        return self.navigationComplete


    """
    In charge of getting/setting the robot's pose at the point of detection
    """
    def getRobotPose(self):
        return self.robotPoseAtDetection

    def setRobotPose(self,pose):
        self.robotPoseAtDetection = pose


    """
    In charge of getting/setting the position of the book
    """
    def getBookPosition(self):
        return self.bookPosition

    def setBookPosition(self,pose):
        self.bookPosition = pose

    """
    Gets/Sets the detection status as complete (book found)
    """
    def setDetectionStatus(self):
        self.detectionPerformed = not self.detectionPerformed

    def getDetectionStatus(self):
        return self.detectionPerformed
    
    """
    In charge of getting/setting the book name requested
    Also holds the original pre-processed speech
    """
    def setDetectionFail(self):
        self.detectionFailed = True

    def getDetectionFail(self):
        return self.detectionFailed
        
    """
    In charge of seeing if gripper is empty or holds anything inside
    """
    def setGripperStatus(self):
        self.gripperEmpty = not self.gripperEmpty

    def getGripperStatus(self):
        return self.gripperEmpty
    


    """
    Resets all known values of the mediator class to allow for infinite looping
    """
    def reset(self):
        self.bookRequested = ""
        self.navigationComplete = False
        self.robotPoseAtDetection = None
        self.detectionPerformed = False
        self.bookPosition = None
        self.reRunDetections = False
        self.gripperEmpty = False
