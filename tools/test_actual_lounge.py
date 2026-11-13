"""Check real-export serialization axes independently of the reference scorer."""
import unittest
import numpy as np
from prepare_actual_lounge import exported_frame, normalize_pixels


class ActualLoungeTest(unittest.TestCase):
    def fixture(self):
        # 90deg yaw and nonzero translation reveal transpose/layout mistakes.
        t=np.array([[0,0,1,2],[0,1,0,3],[-1,0,0,4],[0,0,0,1]],dtype=float)
        k=np.array([[460,0,319.5],[0,460,239.5],[0,0,1.]])
        f=dict(cameraPoseARFrame=t.flatten().tolist(),intrinsics=k.flatten().tolist(),time=100.)
        s=dict(worldFromCameraTransform=t.flatten(order='F').tolist(),
               cameraFromWorldTransform=np.linalg.inv(t).flatten(order='F').tolist(),
               cameraIntrinsics=k.flatten(order='F').tolist(),timestamp=100.)
        return f,s

    def test_layout_and_optical_axes(self):
        f,s=self.fixture();pose,k,error=exported_frame(f,s)
        np.testing.assert_array_equal(pose[:3,3],[2,3,4])
        np.testing.assert_array_equal(pose[:3,0],[0,0,-1])
        np.testing.assert_array_equal(pose[:3,1],[0,-1,0])
        np.testing.assert_array_equal(pose[:3,2],[-1,0,0])
        self.assertEqual(max(error.values()),0.)
        image=np.arange(640*480,dtype=np.uint8).reshape(480,640)
        normalized,_=normalize_pixels(image,k)
        np.testing.assert_array_equal(normalized,image)

    def test_independent_metadata_disagreement_is_rejected(self):
        f,s=self.fixture();s['worldFromCameraTransform'][12]+=1
        with self.assertRaisesRegex(ValueError,'metadata mismatch'):
            exported_frame(f,s)


if __name__=='__main__':unittest.main()
