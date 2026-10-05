"""No hardware or third-party packages required: python3 -m unittest discover -s tests."""
import ast
import contextlib
import io
import json
from pathlib import Path
import types
import unittest

PATH=Path(__file__).resolve().parents[1]/'notebooks/v5.ipynb'
SOURCES=[''.join(c['source']) for c in json.loads(PATH.read_text())['cells'] if c['cell_type']=='code']
CENTER={'near':0.,'middle':0.,'far':0.}
BLANK={'near':None,'middle':None,'far':None}
AWAY={'near':.7,'middle':.7,'far':.7}

def load():
    ns={}
    for s in SOURCES:
        if any(k in s for k in ('def pixel_runs','class CurveController:','class GapCurveController(',
                                'class V5LineController(', 'class GreenNavigator:')):
            exec(s,ns)
    return ns

def image(rectangles):
    rows=[[0]*224 for _ in range(224)]
    for x0,y0,x1,y1 in rectangles:
        for y in range(y0,y1):
            for x in range(x0,x1):rows[y][x]=255
    return rows

class GeometryTests(unittest.TestCase):
    def setUp(self):self.ns=load()
    def test_straight(self):
        o=self.ns['analyze_track'](image([(106,0,118,224)]))
        self.assertFalse(o['junction']);self.assertEqual(o['corner'],0)
        self.assertTrue(all(abs(v)<.02 for v in o['positions'].values()))
    def test_cross_and_t(self):
        for stem in [(106,0,118,224),(106,135,118,224)]:
            o=self.ns['analyze_track'](image([stem,(10,130,214,145)]))
            self.assertTrue(o['junction']);self.assertEqual(o['corner'],0)
    def test_single_branch_straight_exit(self):
        for bar in [(10,130,118,145),(106,130,214,145)]:
            o=self.ns['analyze_track'](image([(106,0,118,224),bar]))
            self.assertTrue(o['junction'])
            self.assertAlmostEqual(o['positions']['near'],0.,delta=.02)
    def test_corners_both_directions(self):
        for bar,direction in [((10,130,118,145),-1),((106,130,214,145),1)]:
            o=self.ns['analyze_track'](image([(106,135,118,224),bar]))
            self.assertFalse(o['junction']);self.assertEqual(o['corner'],direction)
    def test_candidate_association_ignores_large_side_line(self):
        o=self.ns['analyze_track'](image([(106,0,118,224),(20,0,55,224)]))
        self.assertTrue(all(abs(v)<.02 for v in o['positions'].values()))
    def test_blank(self):
        o=self.ns['analyze_track'](image([]))
        self.assertEqual(o['positions'],BLANK);self.assertFalse(o['junction'])

class ControlTests(unittest.TestCase):
    def setUp(self):self.ns=load();self.c=self.ns['V5LineController']()
    def test_notebook_compiles(self):
        for s in SOURCES:compile(s,str(PATH),'exec')
    def test_unmarked_junction_ignores_side_error(self):
        self.c.observation={'junction':True}
        l,r,s=self.c.step({'near':0.,'middle':.8,'far':.8},0)
        self.assertEqual(s,'INTERSECTION_STRAIGHT');self.assertEqual(l,r)
    def test_t_no_exit_stops_instead_of_turning(self):
        self.c.observation={'junction':True};self.c.step(CENTER,0)
        self.c.observation={}
        l,r,s=self.c.step(BLANK,.2)
        self.assertEqual(l,r);self.assertEqual(s,'INTERSECTION_STRAIGHT')
        self.assertTrue(self.c.step(BLANK,1.9)[2].startswith('STOP'))
    def test_junction_exit_requires_three_frames(self):
        self.c.observation={'junction':True};self.c.step(CENTER,0)
        self.c.observation={}
        for t in (.3,.35):self.assertEqual(self.c.step(CENTER,t)[2],'INTERSECTION_STRAIGHT')
        self.assertEqual(self.c.step(CENTER,.4)[2],'FOLLOW')
    def turn(self,c,direction,t=0):
        c.observation={'corner':direction}
        self.assertEqual(c.step(CENTER,t)[2],'SHARP_APPROACH')
        advanced=c.step(CENTER,t+.05)
        self.assertTrue(advanced[2].startswith('SHARP_ADVANCE'))
        self.assertEqual(advanced[0],advanced[1])
        return c.step(AWAY,t+.30)
    def test_advance_latches_direction_and_survives_missing_tape(self):
        self.c.observation={'corner':1}
        self.c.step(CENTER,0.)
        l,r,state=self.c.step(CENTER,.05)
        self.assertEqual(state,'SHARP_ADVANCE_RIGHT');self.assertEqual(l,r)
        self.c.observation={'corner':-1}
        l,r,state=self.c.step(BLANK,.20)
        self.assertEqual(state,'SHARP_ADVANCE_RIGHT');self.assertEqual(l,r)
        l,r,state=self.c.step(BLANK,.30)
        self.assertEqual(state,'SHARP_RIGHT');self.assertGreater(l,0);self.assertLess(r,0)
        self.assertEqual(self.c.turn_started,.30)

    def test_pivot_must_leave_original_alignment(self):
        self.c.observation={'corner':1}
        self.c.step(CENTER,0.);self.c.step(CENTER,.05)
        self.c.observation={}
        self.c.step(CENTER,.30)
        for t in (.7,.75,.8):self.assertEqual(self.c.step(CENTER,t)[2],'SHARP_RIGHT')
        self.c.step(AWAY,.85)
        self.c.step(CENTER,.9);self.c.step(CENTER,.95)
        self.assertEqual(self.c.step(CENTER,1.)[2],'SHARP_DONE')

    def test_direction_specific_advance_times(self):
        self.ns['SHARP_LEFT_ADVANCE_TIME']=.4
        self.ns['SHARP_RIGHT_ADVANCE_TIME']=.1
        for d,expected in ((-1,'SHARP_ADVANCE_LEFT'),(1,'SHARP_RIGHT')):
            c=self.ns['V5LineController']();c.observation={'corner':d}
            c.step(CENTER,0.);c.step(CENTER,.05)
            self.assertEqual(c.step(AWAY,.20)[2],expected)

    def test_sharp_right_survives_old_loss_timeout(self):
        l,r,s=self.turn(self.c,1)
        self.assertTrue(l>0 and r<0)
        self.c.observation={}
        self.assertEqual(self.c.step(BLANK,.6)[2],'SHARP_RIGHT')
        self.assertTrue(self.c.step(BLANK,2.2)[2].startswith('STOP'))
    def test_both_turns_need_stable_heading(self):
        for direction in (-1,1):
            c=self.ns['V5LineController']();self.turn(c,direction);c.observation={}
            tilted={'near':-.15,'middle':.15,'far':.15}
            self.assertTrue(c.step(tilted,.4)[2].startswith('SHARP'))
            for t in (.60,.65):self.assertNotEqual(c.step(CENTER,t)[2],'SHARP_DONE')
            self.assertEqual(c.step(CENTER,.70)[2],'SHARP_DONE')
    def test_dense_curve_slowdown(self):
        for start in (0.,1.):
            self.turn(self.c,1,start);self.c.observation={}
            for delta in (.60,.65,.70):self.c.step(CENTER,start+delta)
        l,r,s=self.c.step(CENTER,1.8)
        self.assertEqual(s,'FOLLOW_SLOW');self.assertLess(l,self.ns['BASE_SPEED'])
        self.assertEqual(self.c.step(CENTER,8.)[2],'FOLLOW')
    def test_gap_preserved(self):
        for t in (0.,.05,.1):self.c.step(CENTER,t)
        self.assertTrue(self.c.step(BLANK,.15)[2].startswith('GAP'))
        self.c.step(CENTER,.2)
        self.assertEqual(self.c.step(CENTER,.25)[2],'FOLLOW')
    def test_green_overrides_junction(self):
        c=self.ns['GreenNavigator']()
        for t in (0.,.03,.06):
            c.line.observation={'junction':True}
            result=c.step(CENTER,'RIGHT',t)
        self.assertEqual(result[2],'GREEN_STOP')
        self.assertEqual(c.step(CENTER,'RIGHT',.1)[2],'GREEN_STOP')
        c.step(CENTER,'RIGHT',.19)
        l,r,s=c.step(CENTER,'RIGHT',.35)
        self.assertEqual(s,'GREEN_TURN_RIGHT');self.assertTrue(l>0 and r<0)
    def test_duplicate_frames_still_timeout(self):
        ns=self.ns
        class Clock:
            t=0.
            def monotonic(self):self.t+=.025;return self.t
            def sleep(self,dt):self.t+=dt
        class Robot:
            def __init__(self):self.commands=[];self.stops=0
            def stop(self):self.stops+=1
            def set_motors(self,l,r):self.commands.append((l,r))
        class Frame:shape=(224,224,3)
        class Detector:
            def detect(self,frame):return {'positions':CENTER,'junction':False,'corner':0},[],None,None,None
        clock=Clock();robot=Robot()
        ns.update(time=clock,robot=robot,TrackDetector=Detector,latest={'sample':(Frame(),0.)},
                  classify_green_direction=lambda *a:'NONE',bgr8_to_jpeg=lambda *a:b'',
                  cv2=types.SimpleNamespace(putText=lambda *a:None,cvtColor=lambda *a:None,
                      FONT_HERSHEY_SIMPLEX=0,COLOR_GRAY2BGR=0),
                  overlay_view=types.SimpleNamespace(),mask_view=types.SimpleNamespace(),status_view=types.SimpleNamespace())
        runner=next(s for s in SOURCES if 'def run_v5' in s)
        module=ast.parse('');module.body=[node for node in ast.parse(runner).body if isinstance(node,ast.FunctionDef)]
        exec(compile(module,'runner','exec'),ns)
        out=io.StringIO()
        with contextlib.redirect_stdout(out):ns['run_v5'](5,True)
        self.assertIn('stale camera frame',out.getvalue())
        self.assertEqual(len(robot.commands),1);self.assertEqual(robot.stops,2)
        self.assertLess(clock.t,1.)

if __name__=='__main__':unittest.main()
