"""Hardware-free regression tests. Run: python3 -m unittest discover -s tests -p 'test_v4_repaired.py'."""
import ast
import json
from pathlib import Path
import unittest

NOTEBOOK = Path(__file__).resolve().parents[1] / 'notebooks' / 'v4_repaired.ipynb'
SOURCES = [''.join(c['source']) for c in json.loads(NOTEBOOK.read_text())['cells'] if c['cell_type']=='code']
CENTER = {'near':0., 'middle':0., 'far':0.}
AWAY = {'near':.7, 'middle':.7, 'far':.7}
BLANK = {'near':None, 'middle':None, 'far':None}

def namespace():
    ns = {}
    for s in SOURCES:
        if any(k in s for k in ('class CurveController:', 'class GapCurveController(', 'class GreenNavigator:')):
            exec(s, ns)
    return ns

class V4Tests(unittest.TestCase):
    def setUp(self):
        self.ns = namespace()
        self.c = self.ns['GreenNavigator']()

    def trigger(self, direction='LEFT'):
        for t in (0., .03):
            self.assertEqual(self.c.step(CENTER, direction, t)[2], 'FOLLOW')
        self.assertEqual(self.c.step(CENTER, direction, .06)[2], 'GREEN_STOP')

    def begin_turn(self, direction='LEFT'):
        self.trigger(direction)
        self.assertEqual(self.c.step(CENTER, direction, .19)[2], 'GREEN_ADVANCE')
        return self.c.step(CENTER, direction, .35)

    def test_all_cells_compile(self):
        for s in SOURCES:
            compile(s, str(NOTEBOOK), 'exec')

    def test_confirmation_resets(self):
        for t,d in ((0,'LEFT'),(.03,'NONE'),(.06,'LEFT'),(.09,'RIGHT')):
            self.assertEqual(self.c.step(CENTER,d,t)[2], 'FOLLOW')

    def test_full_stop_delay_and_turn_signs(self):
        for direction in ('LEFT','RIGHT','BOTH'):
            self.c = self.ns['GreenNavigator']()
            self.trigger(direction)
            self.assertEqual(self.c.step(CENTER,direction,.10)[:2], (0.,0.))
            self.assertEqual(self.c.step(CENTER,direction,.17)[2], 'GREEN_STOP')
            self.assertEqual(self.c.step(CENTER,direction,.19)[2], 'GREEN_ADVANCE')
            l,r,state = self.c.step(CENTER,direction,.35)
            self.assertTrue(l>0 and r<0 if direction=='RIGHT' else l<0 and r>0)

    def test_no_premature_reacquisition(self):
        self.begin_turn()
        for t in (.65,.70,.75):
            self.assertEqual(self.c.step(CENTER,'LEFT',t)[2], 'GREEN_TURN_LEFT')

    def test_reacquisition_and_marker_cooldown(self):
        self.begin_turn()
        self.c.step(AWAY,'LEFT',.40)
        self.assertEqual(self.c.step(CENTER,'LEFT',.65)[2], 'GREEN_TURN_LEFT')
        self.assertEqual(self.c.step(CENTER,'LEFT',.70)[2], 'GREEN_TURN_LEFT')
        self.assertEqual(self.c.step(CENTER,'LEFT',.75)[2], 'GREEN_DONE')
        for t in (1.,1.6,1.7,1.8):
            self.assertEqual(self.c.step(CENTER,'LEFT',t)[2], 'FOLLOW')
        self.assertFalse(self.c.armed)
        for t in (2.,2.05,2.1,2.15,2.2): self.c.step(CENTER,'NONE',t)
        self.assertTrue(self.c.armed)
        for t in (2.3,2.35):self.c.step(CENTER,'RIGHT',t)
        self.assertEqual(self.c.step(CENTER,'RIGHT',2.4)[2], 'GREEN_STOP')

    def test_turn_timeout_latches_stop(self):
        self.begin_turn()
        self.assertTrue(self.c.step(AWAY,'LEFT',1.9)[2].startswith('STOP'))
        self.assertEqual(self.c.step(CENTER,'RIGHT',2.)[:2], (0.,0.))

    def test_uturn_minimum_and_three_frames(self):
        self.begin_turn('BOTH')
        self.c.step(AWAY,'BOTH',.40)
        for t in (.7,.8,.9):self.assertEqual(self.c.step(CENTER,'BOTH',t)[2], 'GREEN_U_TURN')
        for t in (1.2,1.25):self.assertEqual(self.c.step(CENTER,'BOTH',t)[2], 'GREEN_U_TURN')
        self.assertEqual(self.c.step(CENTER,'BOTH',1.3)[2], 'GREEN_DONE')

    def test_gap_and_reacquisition(self):
        for t in (0.,.05,.1):self.c.step(CENTER,'NONE',t)
        self.assertTrue(self.c.step(BLANK,'NONE',.15)[2].startswith('GAP'))
        self.assertTrue(self.c.step(CENTER,'NONE',.2)[2].startswith('GAP'))
        self.assertEqual(self.c.step(CENTER,'NONE',.25)[2], 'FOLLOW')

    def test_gap_timeout_stops(self):
        for t in (0.,.05,.1):self.c.step(CENTER,'NONE',t)
        self.c.step(BLANK,'NONE',.15)
        self.assertTrue(self.c.step(BLANK,'NONE',1.2)[2].startswith('STOP'))

    def test_marker_geometry(self):
        ns={'BANDS':[('near',.72,.88),('middle',.54,.70),('far',.36,.52)],
            'GREEN_TRIGGER_Y':.65,'GREEN_SIDE_MARGIN':8.,'GREEN_MAX_SIDE_DISTANCE':65.,'GREEN_PAIR_Y_TOLERANCE':.15}
        s=next(s for s in SOURCES if 'def classify_green_direction' in s)
        for node in ast.parse(s).body:
            if isinstance(node,ast.FunctionDef) and node.name in ('get_line_points','classify_green_direction'):
                module = ast.parse('')
                module.body = [node]
                exec(compile(module,'<geometry>','exec'),ns)
        classify=ns['classify_green_direction']
        left={'x':80.,'y':165.};right={'x':144.,'y':165.}
        self.assertEqual(classify(CENTER,[left],224,224),'LEFT')
        self.assertEqual(classify(CENTER,[right],224,224),'RIGHT')
        self.assertEqual(classify(CENTER,[left,right],224,224),'BOTH')
        self.assertEqual(classify(BLANK,[left],224,224),'UNKNOWN')
        self.assertEqual(classify(CENTER,[],224,224),'NONE')
        self.assertEqual(classify(CENTER,[{'x':80.,'y':90.}],224,224),'CENTER')

if __name__ == '__main__': unittest.main()
