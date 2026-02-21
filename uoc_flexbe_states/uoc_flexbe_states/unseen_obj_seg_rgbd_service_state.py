#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import numpy as np

import time


from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from unseen_obj_clst_ros2.srv import SegImage

import subprocess, os


# Note: equivalent to running "ros2 service call /segmentation_rgbd unseen_obj_clst_ros2/srv/SegImage "{im_name: 'from_rgbd'}" in another terminal



class UnseenObjSegRGBDServiceState(EventState):
    """
    Call the /segmentation_rgbd SegImage service and expose instance IDs + masks.

    The RGBD segmentation server saves the latest RGB + depth, runs the UCN
    segmentation pipeline, and returns a JSON string (segmentation.json) plus
    a result directory.  This state:

      1) Calls /segmentation_rgbd (SegImage) with a chosen im_name.
      2) Parses the JSON returned in `json_result`.
      3) Extracts the 2D instance-id map and builds per-instance binary masks.
      4) Publishes everything on userdata for downstream states (e.g. CGN).

    -- service_name     string    Service name (default: '/segmentation_rgbd')
    -- service_timeout  float     Timeout for service discovery (sec)
    -- default_im_name  string    Fallback im_name if userdata.im_name is empty
    -- background_id    int       Label to treat as background (default: 0)

    ># im_name                      string   Optional override for im_name
    <# seg_json                     dict     Full segmentation JSON
    <# result_dir                   string   Output directory (as provided by server/JSON)
    <# instance_ids_2d              object   HxW np.ndarray of instance IDs (int32)
    <# instance_id_list             list     Sorted unique non-background IDs
    <# instance_masks               list     List of HxW np.uint8 masks (one per instance)
    <# message                      string   Log / debug text from server

    <= finished                     Segmentation succeeded and userdata filled
    <= failed                       Service call or JSON parsing failed
    """

    def __init__(self,
                 service_name: str = '/segmentation_rgbd',
                 service_timeout: float = 10.0,
                 default_im_name: str = 'from_rgbd',
                 background_id: int = 0):

        super(UnseenObjSegRGBDServiceState, self).__init__(
            outcomes=['finished', 'failed'],
            input_keys=['im_name'],
            output_keys=[
                'seg_json',
                'result_dir',
                'instance_ids_2d',
                'instance_id_list',
                'instance_masks',
                'message'
            ]
        )

        self._service_name = service_name
        self._timeout = float(service_timeout)
        self._default_im_name = str(default_im_name)
        self._background_id = int(background_id)

        # Proxy to the SegImage service
        self._srv = ProxyServiceCaller({self._service_name: SegImage})

        self._res = None
        self._had_error = False
        self._im_name_used = self._default_im_name

    # ------------------------------------------------------------------
    # FlexBE lifecycle
    # ------------------------------------------------------------------

    def on_enter(self, userdata):
        """Send the SegImage request when we enter the state."""
        self._res = None
        self._had_error = False

        # # Check service availability
        # if not self._srv.is_available(self._service_name, timeout=self._timeout):
        #     Logger.logerr(
        #         f"[{type(self).__name__}] Service '{self._service_name}' "
        #         f"not available after {self._timeout:.1f}s."
        #     )
        #     self._had_error = True
        #     return


        # Check service availability (manual timeout loop)
        start = time.time()
        while not self._srv.is_available(self._service_name):
            if time.time() - start > self._timeout:
                Logger.logerr(
                    f"[{type(self).__name__}] Service '{self._service_name}' "
                    f"not available after {self._timeout:.1f}s."
                )
                self._had_error = True
                return
            Logger.loginfo(
                f"[{type(self).__name__}] Waiting for service '{self._service_name}'..."
            )
            time.sleep(0.2)

        # Choose im_name: userdata.im_name or default
        im_name = getattr(userdata, 'im_name', None) or self._default_im_name
        self._im_name_used = im_name

        # Build request
        req = SegImage.Request()
        # SegImage server expects `im_name` as the field
        req.im_name = im_name

        try:
            self._res = self._srv.call(self._service_name, req)
            Logger.loginfo(
                f"[{type(self).__name__}] Called {self._service_name} "
                f"with im_name='{im_name}'."
            )

            cmd = [
                "python3",
                "/home/csrobot/graspnet_ws/src/unseen_obj_clst_ros2/compare_UnseenObjectClustering/segmentation_rgbd/visualize_segmentation.py",
            ]
            subprocess.check_call(cmd)
            
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Service call failed: {e}")
            self._had_error = True

    def execute(self, userdata):
        """Parse the response and fill userdata."""
        if self._had_error or self._res is None:
            return 'failed'

        # Check success flag from server
        if not getattr(self._res, 'success', False):
            msg = getattr(self._res, 'log_output', 'Segmentation failed.')
            Logger.logerr(f"[{type(self).__name__}] Segmentation reported failure: {msg}")
            userdata.message = msg
            return 'failed'

        # Parse JSON
        try:
            json_str = self._res.json_result
            seg_json = json.loads(json_str)
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to parse json_result: {e}")
            userdata.message = f"JSON parse error: {e}"
            return 'failed'

        # Extract instance-id map
        instance_ids = seg_json.get('instance_ids', None)
        if instance_ids is None:
            Logger.logerr(f"[{type(self).__name__}] 'instance_ids' missing in JSON.")
            userdata.message = "Segmentation JSON missing 'instance_ids'."
            return 'failed'

        # Convert to numpy array for easier mask construction
        arr = np.asarray(instance_ids, dtype=np.int32)
        h, w = arr.shape
        Logger.loginfo(
            f"[{type(self).__name__}] Received instance_ids map of shape {h}x{w}."
        )

        # Unique instance labels (excluding background)
        unique_ids = sorted(int(v) for v in np.unique(arr) if v != self._background_id)
        Logger.loginfo(
            f"[{type(self).__name__}] Unique instance IDs (no background): {unique_ids}"
        )

        # Build binary masks (HxW np.uint8, 1 where pixel == instance_id)
        masks = []
        for inst_id in unique_ids:
            mask = (arr == inst_id).astype(np.uint8)
            masks.append(mask)

        # Result directory: prefer JSON's 'result_dir', fall back to response field
        result_dir = seg_json.get('result_dir', '') or getattr(self._res, 'result_dir', '')
        if not result_dir:
            # As absolute last resort, try base_output_dir + im_name
            base_output_dir = seg_json.get('base_output_dir', '')
            if base_output_dir:
                result_dir = os.path.join(base_output_dir, f"segmentation_{self._im_name_used}")

        # Fill userdata
        userdata.seg_json = seg_json
        userdata.result_dir = result_dir
        userdata.instance_ids_2d = arr
        userdata.instance_id_list = unique_ids
        userdata.instance_masks = masks
        userdata.message = getattr(self._res, 'log_output', '')

        return 'finished'

    def on_exit(self, userdata):
        """No special cleanup needed."""
        pass
