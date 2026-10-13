"""Behavior-preserving v4 -> v9 controller refactor checks; no robot imports."""
import json
from pathlib import Path
import random
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def cells(name):
    return json.loads((ROOT/'notebooks'/name).read_text())['cells']

def namespace(name):
    ns={}
    for cell in cells(name):
        s=''.join(cell['source'])
        if cell['cell_type']=='code' and any('class '+n in s for n in (
            'CurveController','GapCurveController','GreenNavigator','BlackLineController',
            'GapCrossingController','GreenMarkerTurnController','NavigationCoordinator')):
            exec(s,ns)
    return ns

CENTER=dict(near=0.,middle=0.,far=0.)
AWAY=dict(near=.7,middle=.7,far=.7)
BLANK=dict.fromkeys(CENTER)

class V9Tests(unittest.TestCase):
    def setUp(self):
        self.old=namespace('v4_repaired.ipynb')['GreenNavigator']()
        self.ns=namespace('v9.ipynb')
        self.new=self.ns['NavigationCoordinator']()

    def compare(self, sequence):
        for p,d,t in sequence:
            self.assertEqual(self.old.step(p,d,t),self.new.step(p,d,t),(p,d,t))
            self.assertEqual(self.old.stopped,self.new.stopped)

    def test_all_cells_compile_and_diagram_is_valid_svg(self):
        for cell in cells('v9.ipynb'):
            if cell['cell_type']=='code':
                compile(''.join(cell['source']),'v9','exec')
                self.assertIsNone(cell['execution_count'])
                self.assertEqual(cell['outputs'],[])
        ET.parse(ROOT/'notebooks/v9_controller_overview.svg')

    def test_green_sequence_completion_and_rearming(self):
        sequence=[(CENTER,'LEFT',t) for t in (0.,.03,.06,.10,.17,.19,.35)]
        sequence += [(AWAY,'LEFT',.4)]
        sequence += [(CENTER,'LEFT',t) for t in (.65,.70,.75,1.,1.6,1.7,1.8)]
        sequence += [(CENTER,'NONE',t) for t in (2.,2.05,2.1,2.15,2.2)]
        sequence += [(CENTER,'RIGHT',t) for t in (2.3,2.35,2.4,2.6,2.8,4.5,4.6)]
        self.compare(sequence)

    def test_both_and_failure_latch(self):
        self.compare([(CENTER,'BOTH',t) for t in (0.,.03,.06,.19,.35)]+
                     [(AWAY,'BOTH',.4)]+[(CENTER,'BOTH',t) for t in (.7,.8,.9,1.2,1.25,1.3)])
        self.assertEqual(self.new.active_controller,'GREEN_TURN')
        self.compare([(BLANK,'NONE',1.4),(CENTER,'LEFT',1.5)])

    def test_gap_success_and_failure(self):
        self.compare([(CENTER,'NONE',t) for t in (0.,.05,.1)]+[(BLANK,'NONE',.15)])
        self.assertEqual(self.new.active_controller,'GAP_CROSSING')
        self.compare([(CENTER,'NONE',.2),(CENTER,'NONE',.25)])
        self.assertEqual(self.new.active_controller,'BLACK_LINE')
        self.compare([(CENTER,'NONE',t) for t in (.3,.35,.4)]+[(BLANK,'NONE',.45),(BLANK,'NONE',1.5),(CENTER,'LEFT',1.6)])
        self.assertEqual(self.new.active_controller,'STOPPED')

    def test_plain_turn_and_search(self):
        self.compare([(AWAY,'NONE',0.),(BLANK,'RIGHT',.1),(AWAY,'RIGHT',.2),
                      (CENTER,'NONE',.3),(CENTER,'NONE',.4),(AWAY,'NONE',.5),
                      (AWAY,'NONE',1.6),(CENTER,'NONE',1.7)])

    def test_randomized_v4_equivalence(self):
        rng=random.Random(20261013)
        values=[None,-.9,-.55,-.21,0.,.21,.55,.9]
        for _ in range(150):
            self.old=namespace('v4_repaired.ipynb')['GreenNavigator']()
            self.new=self.ns['NavigationCoordinator']()
            now=0.
            sequence=[]
            for i in range(60):
                now+=rng.choice([.01,.03,.1,.4])
                p={k:rng.choice(values) for k in CENTER}
                sequence.append((p,rng.choice(['NONE','LEFT','RIGHT','BOTH','CENTER','UNKNOWN']),now))
            self.compare(sequence)

    def test_vision_and_calibration_preserved(self):
        old=cells('v4_repaired.ipynb')
        new=cells('v9.ipynb')
        old_vision=next(''.join(c['source']) for c in old if 'def detect_curve' in ''.join(c['source']))
        new_vision=next(''.join(c['source']) for c in new if 'def detect_curve' in ''.join(c['source']))
        self.assertEqual(old_vision,new_vision)
        ns=namespace('v4_repaired.ipynb')
        for key,value in ns.items():
            if key.isupper():self.assertEqual(value,self.ns[key],key)

if __name__=='__main__':unittest.main()
