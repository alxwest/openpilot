"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import pyray as rl


class RadarTracks:
  def draw_radar_tracks(self, live_tracks, map_to_screen, path_offset_z, track_size=6):
    for track in live_tracks.points:
      d_rel = track.dRel
      y_rel = track.yRel if math.isfinite(track.yRel) else 0.0
      if not math.isfinite(d_rel):
        continue

      pt = map_to_screen(d_rel, -y_rel, path_offset_z)
      if pt is None:
        continue

      x, y = pt
      rl.draw_circle(int(x), int(y), track_size, rl.Color(0, 255, 64, 255))
