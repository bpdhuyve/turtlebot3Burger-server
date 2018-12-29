# same name as custom package, is unfortunate choice..
import face_recognition
import cv2
import numpy as np
import time
from sklearn.metrics.pairwise import cosine_similarity
import time

# Note: needs dlib and face_recognition to be installed

class FaceRecognizer(object):
    '''
    Class to recognize faces
    - Detects faces from an image and if it recognizes categorizes the person
    - if it does not recognize, it adds the face to the database.
    '''

    def __init__(self):
        '''
        This class does not need any parameters 
        '''
        # Store known face_embeddings in an array
        self.face_embeddings = []
        # Process images for beter facedetection
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

    def _create_embedding(self, face):
        '''
        Encode a face into a single vector embedding 
        '''
        return np.asarray(face_recognition.face_encodings(face))

    def calculate_similarity(self, embedding_1, embedding_2):
        '''
        Returns the cosine similarity between two embeddings (between 0 and 1) 
        '''
        return cosine_similarity(embedding_1.reshape(1,-1), embedding_2.reshape(1,-1))

    def _find_most_similar(self, embedding, threshold):
        '''
        Searches all known embeddings for the best matching face.
        Does not take the face in account if the similarity is below the threshold
        '''
        most_similar = None
        best_similarity = 0
        print("="*100)
        for i, compare_embedding in enumerate(self.face_embeddings):
            compare_person_id = i
            similarity = self.calculate_similarity(embedding, compare_embedding)
            print("SIMILARITY WITH {}: {}".format(i, similarity))
            if(similarity > threshold):
                if(similarity > best_similarity):
                    most_similar = i
                    best_similarity = similarity
        print("="*100)
        return most_similar, best_similarity

    def detect_and_identify(self, frame, locs):
        '''
        High level function that 
        receives locations for faces within a frame
        cuts out faces and returns the detected ids 
        '''
        # loc returns [top, right, bottom, left]
        faces = [frame[l[0]:l[2], l[3]:l[1]] for l in locs]
        embeddings = [self._create_embedding(face) for face in faces]

        ids = []
        # Compare embeddings with known embeddings

        for emb in embeddings:
            if emb.shape == (1,128):
                id, sim = self._find_most_similar(emb, 0.90)
                if id is not None:
                    ids.append(id)
                else:
                    # Add embedding to the face_embeddings
                    self.face_embeddings.append(emb)
                    ids.append(len(self.face_embeddings) - 1)

                print("ID: {} detected with a similarity of {}%".format(ids[-1], sim*100))
                print("_"*100)

        return ids

    def recognize_people(self, frame):
        '''
        Detects people in a frame, and identifies them
        returns false if no faces are detected, otherwise
        returns a list of face locations and ids in the same 
        order 
        '''
        # HOG detection (CNN is too slow for realtime use)
        # And performs better than Viola-jones
        processed_detection = self.clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        locs = face_recognition.face_locations(processed_detection, model='hog')
        if len(locs) > 0:
            ids = self.detect_and_identify(frame, locs)
            return locs, ids
        return False

if __name__ == '__main__':

    fr = FaceRecognizer()
    cap = cv2.VideoCapture(0)
    while(True):
        frame_nr = 0

        ret, frame = cap.read()

        ret = fr.recognize_people(frame)
        if ret:
            locs, ids = ret
            for loc, id in zip(locs, ids):
                top, right, bottom, left = loc
                frame = cv2.putText(frame,'Person {}'.format(id),(left,bottom-3),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
                frame = cv2.rectangle(frame,(left,top),(right,bottom),(0,255,0),3)

        cv2.imshow('frame {}'.format(frame_nr), np.hstack([frame]) )

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
