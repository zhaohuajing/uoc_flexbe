#!/usr/bin/env python3
import json, os
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from unseen_obj_clst_ros2.srv import SegCloud
from sensor_msgs.msg import PointCloud2, CameraInfo

class UnseenObjSegCloudServiceState(EventState):
    """
    Calls 'run_segmentation_cloud' with a PointCloud2 (and optional CameraInfo).
    Outputs:
      seg_json (dict), result_dir (str), instance_ids (list), classes (list), bboxes (list), message (str)
    """
    def __init__(self,
                 cloud_service='run_segmentation_cloud',
                 service_timeout=5.0,
                 default_im_name='from_cloud'):
        super().__init__(
            outcomes=['finished', 'failed'],
            input_keys=['cloud_in', 'camera_info'], #, 'image_name'],
            output_keys=['seg_json','result_dir','instance_ids','classes','bboxes'] #,'message']
        )
        self._timeout = float(service_timeout)
        self._default_im_name = default_im_name
        self._cloud_srv_name = cloud_service
        self._srv = ProxyServiceCaller({ self._cloud_srv_name: SegCloud })
        self._res = None
        self._err = False

    def on_enter(self, userdata):
        self._res, self._err = None, False
        if not isinstance(getattr(userdata, 'cloud_in', None), PointCloud2):
            Logger.logerr("[SegCloudServiceState] Missing or invalid 'cloud_in' PointCloud2.")
            self._err = True
            return

        if not self._srv.is_available(self._cloud_srv_name, timeout=self._timeout):
            Logger.logerr(f"[SegCloudServiceState] Service '{self._cloud_srv_name}' not available after {self._timeout}s.")
            self._err = True
            return

        try:
            req = SegCloud.Request()
            req.cloud = userdata.cloud_in
            if isinstance(getattr(userdata, 'camera_info', None), CameraInfo):
                req.cam_info = userdata.camera_info
            # # optional im_name if your .srv includes it
            # try:
            #     im_name = getattr(userdata, 'image_name', None) or self._default_im_name
            #     req.im_name = im_name
            # except Exception:
            #     pass

            self._res = self._srv.call(self._cloud_srv_name, req)

        except Exception as e:
            Logger.logerr(f"[SegCloudServiceState] Service call failed: {e}")
            self._err = True

    def execute(self, userdata):
        if self._err or self._res is None:
            return 'failed'
        if not self._res.success:
            userdata.message = self._res.log_output or "Segmentation failed."
            return 'failed'

        try:
            seg_json = json.loads(self._res.json_result)
            classes = seg_json.get('classes', [])
            instance_ids = seg_json.get('instance_ids', [])
            bboxes = seg_json.get('bboxes', [])

            result_dir = seg_json.get('result_dir', '')
            if not result_dir:
                base_output_dir = seg_json.get('base_output_dir', "/tmp/ucn_io/out")
                # im_name = getattr(userdata, 'image_name', self._default_im_name)
                result_dir = os.path.join(base_output_dir, f"segmentation_output") #f"segmentation_{im_name}")

            userdata.seg_json = seg_json
            userdata.result_dir = result_dir
            userdata.classes = classes
            userdata.instance_ids = instance_ids
            userdata.bboxes = bboxes
            # userdata.message = self._res.log_output or ""
            return 'finished'

        except Exception as e:
            Logger.logerr(f"[SegCloudServiceState] Parse error: {e}")
            return 'failed'
