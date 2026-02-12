class CvBridge:
    def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
        # In our mock runner, we will pass the numpy array directly as 'img_msg' 
        # to simplify things, or wrapper object.
        # But to be robust, let's assume the runner passes an object having 'data'
        if hasattr(img_msg, 'data_numpy'):
            return img_msg.data_numpy
        return img_msg

    def cv2_to_imgmsg(self, cvim, encoding="passthrough"):
        # Wrap it in a simple object
        class MockImgMsg:
            def __init__(self, data):
                self.data_numpy = data
        return MockImgMsg(cvim)
