import unittest
import cv2
import numpy as np
from calibrate_charuco_4k import make_board, detect


class CharucoTest(unittest.TestCase):
    def test_detects_generated_board_at_4k(self):
        dictionary, board = make_board()
        image = np.full((2160,3840),255,np.uint8)
        image[330:1830,870:2970] = board.draw((2100,1500))
        corners, ids = detect(image, dictionary, board)
        self.assertIsNotNone(corners)
        self.assertEqual(len(ids),24)


if __name__ == '__main__':
    unittest.main()
