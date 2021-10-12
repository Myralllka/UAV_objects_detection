# Data labeling for [VoteNet](https://github.com/facebookresearch/votenet) network (objects detection from point clouds)

For labels storage I use a custom message.<br>
Each pointcloud should have an annotation as MetadataArray

Metadata includes:
- label a.e.                            'human'
- bounding box center |x y z| a.e.      'Point(-2.3, 0.1, -18)
- orientation (votenet accept only euler angle orientation along the up-axis; rotation radius from +X towards -Y; +X is 0 and -Y is pi/4). Just for the future and for python script for data preparation
- dimensions |l w h| a.e.               'Vector3(1.95, 2.4, 0.5)'
  - length along x axis
  - width along y
  - height along z
