"""Hardware-free controller checks for the camera-only v6 notebook."""
import ast
import json
from pathlib import Path
import unittest

NOTEBOOK = Path(__file__).resolve().parents[1] / 'notebooks/v6.ipynb'
SOURCES = [''.join(c['source']) for c in json.loads(NOTEBOOK.read_text())['cells'] if c['cell_type'] == 'code']
CENTER = dict(near=0., middle=0., far=0.)
AWAY = dict(near=.7, middle=.7, far=.7)
BLANK = dict(near=None, middle=None, far=None)

class V6Tests(unittest.TestCase):
    def setUp(self):
        self.ns = {'LINE_WEIGHTS': dict(near=.45, middle=.35, far=.20)}
        for s in SOURCES:
            if any(k in s for k in ('class CurveController:', 'class GapCurveController(', 'class GreenNavigator:')):
                exec(s, self.ns)
        self.c = self.ns['GreenNavigator']()

    def trigger(self, direction='LEFT'):
        for t in (0., .03):
            self.assertEqual(self.c.step(CENTER, direction, t)[2], 'FOLLOW')
        return self.c.step(CENTER, direction, .06)

    def begin(self, direction='LEFT'):
        self.assertEqual(self.trigger(direction)[2], 'GREEN_STOP')
        self.assertEqual(self.c.step(CENTER, direction, .10)[:2], (0., 0.))
        return self.c.step(CENTER, direction, .19)

    def test_cells_compile_and_no_outputs(self):
        for s in SOURCES:
            compile(s, str(NOTEBOOK), 'exec')
        for cell in json.loads(NOTEBOOK.read_text())['cells']:
            if cell['cell_type'] == 'code':
                self.assertEqual(cell['outputs'], [])
                self.assertIsNone(cell['execution_count'])

    def test_stationary_inner_wheel_and_no_advance(self):
        for d, expected in [('LEFT', (0., .15)), ('RIGHT', (.15, 0.))]:
            self.c = self.ns['GreenNavigator']()
            self.assertEqual(self.begin(d)[:2], expected)

    def test_old_line_cannot_finish_turn(self):
        self.begin()
        for t in (.3, .5, .8, 1.):
            self.assertEqual(self.c.step(CENTER, 'LEFT', t)[2], 'GREEN_TURN_LEFT')

    def test_one_bad_frame_cannot_confirm_departure(self):
        self.begin()
        self.c.step(BLANK, 'LEFT', .2)
        for t in (.21, .22, .23, .24):
            self.assertEqual(self.c.step(CENTER, 'LEFT', t)[2], 'GREEN_TURN_LEFT')
        self.assertFalse(self.c.departed)

    def test_visual_completion_without_minimum_duration(self):
        self.begin()
        for t in (.20, .21, .22): self.c.step(AWAY, 'LEFT', t)
        for t in (.23, .24):
            self.assertEqual(self.c.step(CENTER, 'LEFT', t)[2], 'GREEN_TURN_LEFT')
        self.assertEqual(self.c.step(CENTER, 'LEFT', .25)[2], 'GREEN_DONE')
        self.assertFalse(self.c.armed)
        for t in (.3, .4, .5, .6, .7): self.c.step(CENTER, 'NONE', t)
        self.assertFalse(self.c.armed)
        self.c.step(CENTER, 'NONE', 1.1)
        self.assertTrue(self.c.armed)

    def test_slanted_line_and_interrupted_reacquisition(self):
        self.begin()
        for t in (.2, .21, .22): self.c.step(AWAY, 'LEFT', t)
        for t in (.3, .4, .5):
            self.assertEqual(self.c.step(dict(near=-.2, middle=.2, far=.2), 'NONE', t)[2], 'GREEN_TURN_LEFT')
        self.c.step(CENTER, 'NONE', .6)
        self.c.step(BLANK, 'NONE', .7)
        self.c.step(CENTER, 'NONE', .8)
        self.assertEqual(self.c.step(CENTER, 'NONE', .9)[2], 'GREEN_TURN_LEFT')
        self.assertEqual(self.c.step(CENTER, 'NONE', 1.)[2], 'GREEN_DONE')

    def test_timeout_latches(self):
        self.begin()
        self.assertTrue(self.c.step(AWAY, 'NONE', 1.7)[2].startswith('STOP'))
        self.assertEqual(self.c.step(CENTER, 'RIGHT', 1.8)[:2], (0., 0.))

    def test_both_is_ambiguous(self):
        self.assertTrue(self.trigger('BOTH')[2].startswith('STOP'))
        self.assertTrue(self.c.stopped)

    def test_gap_success_and_timeout(self):
        for t in (0., .05, .1): self.c.step(CENTER, 'NONE', t)
        self.assertTrue(self.c.step(BLANK, 'NONE', .15)[2].startswith('GAP'))
        self.assertTrue(self.c.step(CENTER, 'NONE', .2)[2].startswith('GAP'))
        self.assertEqual(self.c.step(CENTER, 'NONE', .25)[2], 'FOLLOW')
        for t in (.3, .35, .4): self.c.step(CENTER, 'NONE', t)
        self.c.step(BLANK, 'NONE', .45)
        self.assertTrue(self.c.step(BLANK, 'NONE', 1.5)[2].startswith('STOP'))

    def test_optional_band_follow_and_gap(self):
        self.ns['LINE_WEIGHTS'] = dict(near=.45, middle=.30, far=.20, extra_far=.05)
        p = dict(CENTER, extra_far=0.)
        for t in (0., .05, .1):
            self.assertEqual(self.c.step(p, 'NONE', t)[2], 'FOLLOW')
        self.assertTrue(self.c.step(dict(BLANK, extra_far=None), 'NONE', .15)[2].startswith('GAP'))
        curve = self.ns['CurveController']()
        self.assertEqual(curve.step(dict(BLANK, extra_far=.1), 0.)[2], 'FOLLOW')

if __name__ == '__main__':
    unittest.main()
