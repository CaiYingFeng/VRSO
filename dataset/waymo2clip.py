# flake8: noqa

import tensorflow as tf
if not tf.executing_eagerly():
  tf.compat.v1.enable_eager_execution()
import os
from os.path import join
import json
import logging
import cv2
import numpy as np
from tqdm import tqdm
from scipy.spatial.transform import Rotation
from waymo_open_dataset import dataset_pb2 as open_dataset
from waymo_open_dataset.utils import box_utils

logger = logging.getLogger()
logger.setLevel(logging.INFO)
logging.basicConfig(level=logging.INFO, format="%(asctime)-15s %(message)s")

class Waymo2Clip:
    def __init__(self, configs):

        self.configs = configs

        # converted sensor name
        self.camera_dict = {"FRONT": "camera_front",
                            "FRONT_LEFT": "camera_front_left",
                            "FRONT_RIGHT": "camera_front_right",
                            "SIDE_LEFT": "camera_rear_left",
                            "SIDE_RIGHT": "camera_rear_right"}

        self.box_type_dict = {1: "vehicle",
                              2: "pedestrian",
                              3: "sign",
                              4: "cyclist"}

    def get_sync_info(self, unsync_timestamps_dict, ref_sensor, max_diff):
        """sync the unsync timestamps with reference sensor

        Args:
            unsync_timestamps_dict (dict): each key is sensor name and its value is list of unsync timestamps in milliseconds
            ref_sensor (string): reference sensor name, must be in unsync_timestamps_dict
            max_diff (int): max_diff in milliseconds for synchronization

        Returns:
            dict: each key is sensor name and its value is list of sync timestamps in milliseconds
        """
        # assert ref_sensor in unsync_timestamps_dict
        sync_timestamps_dict = {}
        for sensor_name in unsync_timestamps_dict.keys():
            sync_timestamps_dict[sensor_name] = []
        for ref_timestamp in unsync_timestamps_dict[ref_sensor]:
            sync_timestamp = []
            for sensor in unsync_timestamps_dict.keys():
                if sensor == ref_sensor:
                    sync_timestamp.append(ref_timestamp)
                else:
                    # this may be slower than np.searchsorted theoretically
                    # but it handles boundary issues automatically and is more readable
                    # the length of unsync_timestamps is at the magnitude of 10^2 to 10^4
                    search_idx = np.abs(
                        np.asarray(unsync_timestamps_dict[sensor]) - ref_timestamp
                    ).argmin()
                    diff_time = abs(
                        unsync_timestamps_dict[sensor][search_idx] - ref_timestamp
                    )
                    if diff_time <= max_diff:
                        sync_timestamp.append(unsync_timestamps_dict[sensor][search_idx])
                    # else:
                    #     logger.warning(
                    #         f"skipped, diff time = {diff_time} is larger than max_diff = {max_diff}"
                    #     )
            if len(sync_timestamp) == len(unsync_timestamps_dict.keys()):
                for sensor_name, timestamp in zip(unsync_timestamps_dict, sync_timestamp):
                    sync_timestamps_dict[sensor_name].append(timestamp)
            # else:
            #     logger.info("skip frame {}".format(ref_timestamp))
        return sync_timestamps_dict

    def save_camera_image(self, frame, save_path, unsync):
        for image in frame.images:
            img = tf.image.decode_jpeg(image.image).numpy()
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            timestamp = round(image.camera_trigger_time*1000)
            # timestamp = round(image.pose_timestamp*1000)
            
            camera_name = self.camera_dict[open_dataset.CameraName.Name.Name(image.name)]
            unsync[camera_name].append(timestamp)
            img_save_path = join(save_path, camera_name)
            cv2.imwrite(join(img_save_path, f"{timestamp}.jpg"), img)
        return unsync

    def get_chassis_pose(self, frame):
        chassis2world = np.array(frame.pose.transform).reshape(4, 4)
        quaternion = Rotation.from_matrix(chassis2world[:3, :3]).as_quat()
        translation = chassis2world[:3, 3]
        tum_chassis_pose = np.concatenate(([frame.timestamp_micros/1000], translation, quaternion))
        return tum_chassis_pose

    def get_calibration(self, frame):
        calibration = {}
        for camera_calibration in frame.context.camera_calibrations:
            fx, fy, cx, cy, k1, k2, p1, p2, k3 = camera_calibration.intrinsic
            K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            distort = [k1, k2, p1, p2, k3]
            height = camera_calibration.height
            width = camera_calibration.width
            camera_name = self.camera_dict[open_dataset.CameraName.Name.Name(camera_calibration.name)]
            calibration[camera_name] = {"center_u": cx,
                                        "center_v": cy,
                                        "distort": distort,
                                        "focal_u": fx,
                                        "focal_v": fy,
                                        "image_height": height,
                                        "image_width": width,
                                        "K": K, "d": distort}

            cam2chassis = np.array(camera_calibration.extrinsic.transform).reshape(4,4)
            opencv2waymo = np.eye(4)
            opencv2waymo[:3, :3] = np.array(
                                [[0, 0, 1],
                                    [-1, 0, 0],
                                    [0, -1, 0]])

            calibration[f"{camera_name}_2_chassis"] = (cam2chassis @ opencv2waymo).tolist()

        return calibration

    def get_3dbox(self, frame, box3dgt):
        box3d = []
        for label in frame.laser_labels:
            box_type = self.box_type_dict[label.type]
            if box_type == "sign":
                box = label.camera_synced_box

                if not box.ByteSize():
                    continue  # Filter out labels that do not have a camera_synced_box.

                # Retrieve upright 3D box corners.
                box_coords = np.array([[
                    box.center_x, box.center_y, box.center_z, box.length, box.width,
                    box.height, box.heading
                ]])
                corners = box_utils.get_upright_3d_box_corners(
                    box_coords)[0].numpy()  # [8, 3]

                box3d.append({"type": box_type, "corners": corners.tolist(), "id": label.id})

        for image in frame.images:
            camera_name = self.camera_dict[open_dataset.CameraName.Name.Name(image.name)]
            timestamp = round(image.camera_trigger_time*1000)
            box3dgt[f"{camera_name}_{timestamp}"] = box3d
        return box3dgt

    def get_sign_3dbox(self, box3dgt):
        sign_box3d = []
        all_box = []
        exist_ids = set()
        for _, all_box3d in box3dgt.items():
            for box3d in all_box3d:
                if box3d["type"] == "sign":
                    all_box.append(box3d["corners"])
                if box3d["type"] == "sign" and box3d["id"] not in exist_ids:
                    sign_box3d.append(box3d["corners"])
                    exist_ids.add(box3d["id"])

        return sign_box3d

    def convert(self, segment_filename):

        # segment-15832924468527961_1564_160_1584_160_with_camera_labels.tfrecord
        dataset = tf.data.TFRecordDataset(join(self.configs["dataroot"], segment_filename), compression_type='')
        seg_name = segment_filename.split("-")[-1].split("_")[0]
        clip_root = join(self.configs["converted_dataroot"], seg_name)
        if not os.path.exists(clip_root):
            return

        # prepare output dir
        os.makedirs(join(clip_root, "odometry"), exist_ok=True)
        for _, camera_name in self.camera_dict.items():
            img_save_path = join(clip_root, camera_name)
            if not os.path.exists(img_save_path):
                os.makedirs(img_save_path)

        # init clip attribute dict
        attr_dict = {
            "status": "init",
            "calibration": dict(),
        }

        # init 3dbox gt
        box3d_dict = {}

        pose = []
        unsync = {}
        box3dgt = {}
        for _, camera_name in self.camera_dict.items():
            unsync[camera_name] = list()

        for frameid, data in enumerate(tqdm(dataset)):
            frame = open_dataset.Frame()
            frame.ParseFromString(bytearray(data.numpy()))
            frame.timestamp_micros = frame.timestamp_micros//1000
            # print(frame)
            if frameid < 151:
                continue

            # 5v image
            unsync = self.save_camera_image(frame, clip_root, unsync)
            # print(unsync)

            # # odometry
            tum_chassis_pose = self.get_chassis_pose(frame)
            pose.append(tum_chassis_pose)
            # print(pose)

            box3dgt = self.get_3dbox(frame, box3dgt)
            # print(box3dgt)
            if len(pose)>10:
                break

        box3d_dict.update({"box3d_info": box3dgt})

        sign_box3d = self.get_sign_3dbox(box3dgt)
        # print(sign_box3d)
        sign_num = len(sign_box3d)
        box3d_dict.update({"sign_box3d": box3dgt})
        # with open("/home/users/yingfeng.cai/some_tools/sign_num.txt", "a+") as f:
        #     f.write(segment_filename+" "+str(sign_num)+"\n")

        with open(os.path.join(clip_root, 'box3d.json'), 'w') as jf:
            json.dump(box3d_dict, jf, indent=4, ensure_ascii=False)

        np.savetxt(join(clip_root, "odometry/wigo.txt"), pose)

        attr_dict.update({"unsync": unsync})
        # print(len(unsync["camera_front"]))

        sync = self.get_sync_info(unsync, "camera_front", max_diff=50)
        # print(len(sync["camera_front"]))
        attr_dict.update({"sync": sync})

        calibration = self.get_calibration(frame)
        attr_dict.update({"calibration": calibration})

        wigo_offset = np.array(pose)
        location_utm = wigo_offset[len(pose)//2, 1:4].tolist()
        attr_dict.update({"location_utm": location_utm})
        wigo_offset[:, 1:4] -= location_utm
        np.savetxt(join(clip_root, "odometry/wigo_offset_clip.txt"), wigo_offset)

        attr_dict.update({"sign_num": sign_num})
        # print(attr_dict)

        with open(os.path.join(clip_root, 'attribute.json'), 'w') as jf:
            json.dump(attr_dict, jf, indent=4, ensure_ascii=False)

if __name__ == '__main__':

    configs = dict()
    configs["dataroot"] = '/home/users/yingfeng.cai/dev/waymo'
    configs["converted_dataroot"] = '/home/users/yingfeng.cai/dev'
    w2c = Waymo2Clip(configs)
    segment_filenames = os.listdir(configs["dataroot"])
    segment_filenames.sort()

    # segment_filenames= ["individual_files_training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord"]
    for segment_filename in segment_filenames:
        # segment_filenames= ["segment-1005081002024129653_1564_160_1584_160_with_camera_labels.tfrecord"]
        # print(segment_filename)
        if segment_filename.startswith("segment") or segment_filename.startswith("individual"):
            w2c.convert(segment_filename)
