import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def depth_image_to_point_cloud2(
    depth_frame,
    header,
    fx,
    fy,
    cx,
    cy,
    depth_scale,
    stride,
    max_depth_m
):
    stride = max(int(stride), 1)
    sampled_depth = depth_frame[::stride, ::stride].astype(np.float32) * float(depth_scale)

    rows, cols = sampled_depth.shape
    v_coords, u_coords = np.indices((rows, cols), dtype=np.float32)
    u_coords = u_coords * stride
    v_coords = v_coords * stride

    valid = (
        np.isfinite(sampled_depth) &
        (sampled_depth > 0.0) &
        (sampled_depth <= float(max_depth_m))
    )

    z = sampled_depth[valid]
    u = u_coords[valid]
    v = v_coords[valid]

    if z.size == 0:
        points = np.empty((0, 3), dtype=np.float32)
    else:
        x = (u - float(cx)) * z / float(fx)
        y = (v - float(cy)) * z / float(fy)
        points = np.column_stack((x, y, z)).astype(np.float32)

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = points.shape[0]
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * points.shape[0]
    cloud.is_dense = False
    cloud.data = points.tobytes()

    return cloud
