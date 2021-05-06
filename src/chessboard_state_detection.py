

class ChessClassification:
    def __init__(self):
        '''
        load trained model
        '''
        pass

    def predict(self, image):
        '''
        predict the chess class given a square image
        '''
        pass



class ChessboardStateDetection:
    def __init__(self):
        self.classifier = ChessClassification()
        pass

    def detectState(self, img):
        '''
        detect the state of chessboard given image

        Args:
            img (numpy image): input image
        
        Returns:
            state (string): the state of chessboard in FEN coding format
        '''
        pass

    def extractSquare(self, image):
        '''
        output a list of square images for chess classification
        '''
        pass

    def setSquarePos(self, square_pos):
        '''
        set the image position for each square
        '''
        pass