from __future__ import absolute_import, division, print_function

from collections import defaultdict

import marv
import numpy as np

@marv.node()
@marv.input('messages', filter=['*:auv_msgs/NavSts'])
def navsts(messages):
    coords = defaultdict(list)
    erroneous = defaultdict(int)
    for topic, msg, _ in messages:
        # skip erroneous messages
        if not hasattr(msg, 'status') or \
           np.isnan(msg.global_position.longitude) or \
           np.isnan(msg.global_position.latitude) or \
           np.isnan(msg.altitude):
            erroneous[topic] += 1
        else:
            coords[topic].append((msg.status, (msg.global_position.longitude, msg.global_position.latitude)))

    if erroneous:
        marv.log_warn('Skipped erroneous GNSS messages %r', erroneous.items())

    return {'coordinates': coords} if coords else None


@marv.node()
@marv.input('navsts')
def geo_json_navsts_trajectory(navsts):
    features = []
    prev_quality = None
    coordinates = navsts.coordinates.values()[0]  # Only one topic for now
    for status, coord in coordinates:
        #  uint8 STATUS_FAULT = 0                   -> color id 0 = red
        #  uint8 STATUS_LOCAL_FRAME_OK = 1          -> color id 1 = orange
        #  uint8 STATUS_GLOBAL_FRAME_OK = 2         -> color id 2 = blue
        #  uint8 STATUS_POSITION_OK = 3             -> color id 3 = green
        #  uint8 STATUS_VELOCITY_OK = 4             -> color id 4 = black
        #  uint8 STATUS_ESTIMATION_ERROR_OK  = 8    -> color id 4 = black
        #  uint8 STATUS_ALL_OK = 15                 -> color id 4 = black
        if 0 <= status <= 4:
            quality = status + 1
        else:
            quality = 4
        if quality != prev_quality:
            color = ('#f00', '#ffa500', '#00f', '#0f0', '#000')[quality]
            coords = []
            feat = {'type': 'Feature',
                    'properties': {'style': {'color': color}},
                    'geometry': {'type': 'LineString', 'coordinates': coords}}
            features.append(feat)
            prev_quality = quality
        coords.append(coord)
    return {'type': 'FeatureCollection', 'features': features} if features else None
