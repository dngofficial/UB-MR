from pathlib import Path
import shutil
import tempfile
import unittest
import numpy as np
from rotation_integration.georeference import inspect_lanelet, opendrive_georeference, project_xy


class GeographicEvidenceTests(unittest.TestCase):
    def test_empty_geographic_tags_do_not_establish_a_map_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.osm'
            path.write_text('<osm><node id="1" lat="" lon=""><tag k="local_x" v="1"/><tag k="local_y" v="2"/></node></osm>')
            report, points = inspect_lanelet(path)
            self.assertEqual(report['paired_geographic_anchor_nodes'], 0)
            np.testing.assert_equal(points, [[1, 2]])

    def test_nonzero_opendrive_offset_cannot_be_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.xodr'
            path.write_text('<OpenDRIVE><header><geoReference>+proj=tmerc</geoReference><offset x="1"/></header></OpenDRIVE>')
            with self.assertRaisesRegex(ValueError, 'offset'):
                opendrive_georeference(path)

    @unittest.skipUnless(shutil.which('cs2cs'), 'PROJ cs2cs is optional for geographic auditing')
    def test_local_projection_origin_and_utm_roundtrip(self):
        geographic = '+proj=longlat +datum=WGS84 +no_defs'
        local = '+proj=tmerc +lat_0=42.9926180599693 +lon_0=-78.7924945927909 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs'
        utm = '+proj=utm +zone=17 +datum=WGS84 +units=m +no_defs'
        origin = [[-78.7924945927909, 42.9926180599693]]
        np.testing.assert_allclose(project_xy(origin, geographic, local), [[0, 0]], atol=1e-6)
        fix = [[-78.79302490093541, 42.993243137132964]]
        through_utm = project_xy(project_xy(fix, geographic, utm), utm, local)
        np.testing.assert_allclose(through_utm, project_xy(fix, geographic, local), atol=1e-6)

    def test_projection_rejects_nonfinite_input(self):
        if not shutil.which('cs2cs'):
            self.skipTest('PROJ cs2cs not installed')
        with self.assertRaises(ValueError):
            project_xy([[float('nan'), 1]], '+proj=longlat', '+proj=utm +zone=17')
