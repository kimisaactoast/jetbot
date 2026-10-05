"""V7 hardware-free tests. OpenCV image tests run when cv2/numpy are installed."""
import ast
import json
from pathlib import Path
import unittest
from types import SimpleNamespace
from contextlib import redirect_stdout
from io import StringIO

NOTEBOOK = Path(__file__).resolve().parents[1] / 'notebooks/v7.ipynb'
CELLS = json.loads(NOTEBOOK.read_text())['cells']
CENTER = dict(near=0., middle=0., far=0.)
RIGHT = dict(near=.75, middle=.75, far=.75)
LEFT = dict(near=-.75, middle=-.75, far=-.75)
BLANK = dict.fromkeys(CENTER)

def namespace(images=False):
    ns = {}
    if images:
        import cv2
        import numpy as np
        ns.update(cv2=cv2,np=np)
    for i in (4,6,8,10):
        tree = ast.parse(''.join(CELLS[i]['source']))
        if not images:
            tree.body = [node for node in tree.body if not (isinstance(node,ast.Assign) and
                         any(isinstance(t,ast.Name) and t.id in ('GREEN_LOWER','GREEN_UPPER') for t in node.targets))]
        exec(compile(tree,str(NOTEBOOK),'exec'),ns)
    return ns

def marker(x=80.,y=125.,area=100.):
    return dict(x=x,y=y,area=area,w=10,h=10,box=(int(x-5),int(y-5),10,10))

class V7Tests(unittest.TestCase):
    def setUp(self):
        self.ns = namespace()
        self.c = self.ns['GreenNavigator']()

    def begin(self,direction='LEFT',ids=(1,),start=0.):
        for dt in (0.,.03): self.c.step(CENTER,direction,start+dt,ids)
        self.assertEqual(self.c.step(CENTER,direction,start+.06,ids)[2],'GREEN_STOP')
        return self.c.step(CENTER,direction,start+.19,ids)

    def complete(self):
        for t in (.2,.21,.22): self.c.step(BLANK,'NONE',t)
        for t in (.23,.24,.25): self.c.step(CENTER,'NONE',t)
        self.assertEqual(self.c.step(CENTER,'NONE',.26)[2],'GREEN_DONE')

    def test_compile_and_no_saved_outputs(self):
        for cell in CELLS:
            if cell['cell_type']=='code':
                compile(''.join(cell['source']),str(NOTEBOOK),'exec')
                self.assertEqual(cell['outputs'],[])
                self.assertIsNone(cell['execution_count'])
        self.assertIn('# run_robot(seconds=2)',''.join(CELLS[16]['source']))

    def test_green_pivot_and_gradual_capture(self):
        self.assertEqual(self.begin()[:2],(0.,.15))
        for t in (.2,.21,.22): self.c.step(BLANK,'NONE',t)
        p=dict(near=-.3,middle=-.2,far=-.1)
        self.c.step(p,'NONE',.23)
        l,r,state=self.c.step(p,'NONE',.24)
        self.assertEqual(state,'GREEN_ALIGN')
        self.assertGreater(l,0.)
        self.assertGreater(r,l)
        self.assertLessEqual(r,.15)

    def test_green_old_centered_line_does_not_complete(self):
        self.begin()
        for t in (.2,.3,.5,.7):
            self.assertEqual(self.c.step(CENTER,'NONE',t)[2],'GREEN_TURN_LEFT')

    def test_new_marker_can_trigger_immediately_after_turn(self):
        self.begin()
        self.complete()
        for t in (.27,.28):
            self.assertEqual(self.c.step(CENTER,'RIGHT',t,(2,))[2],'FOLLOW')
        self.assertEqual(self.c.step(CENTER,'RIGHT',.29,(2,))[2],'GREEN_STOP')
        self.assertEqual(self.c.consumed_ids,(2,))

    def test_green_does_not_capture_old_line_on_opposite_side(self):
        self.begin('RIGHT')
        for t in (.20,.21,.22,.23,.24,.25):
            self.assertEqual(self.c.step(LEFT,'NONE',t)[2],'GREEN_TURN_RIGHT')
        for t in (.26,.27):
            result=self.c.step(dict(near=.3,middle=.2,far=.1),'NONE',t)
        self.assertEqual(result[2],'GREEN_ALIGN')
        self.assertGreater(result[0],result[1])

    def test_ambiguity_clears_gap_approach_evidence(self):
        for t in (0.,.05,.1): self.c.step(CENTER,'NONE',t)
        self.c.step(BLANK,'NONE',.15,reliable=False)
        self.assertTrue(self.c.step(BLANK,'NONE',.2)[2].startswith('STOP'))

    def test_identity_change_resets_confirmation(self):
        self.c.step(CENTER,'LEFT',0.,(1,))
        self.c.step(CENTER,'LEFT',.03,(2,))
        self.assertEqual(self.c.step(CENTER,'LEFT',.06,(2,))[2],'FOLLOW')
        self.assertEqual(self.c.step(CENTER,'LEFT',.09,(2,))[2],'GREEN_STOP')

    def test_green_timeout_covers_alignment_and_reloss(self):
        self.begin()
        for t in (.2,.21,.22): self.c.step(BLANK,'NONE',t)
        self.c.step(CENTER,'NONE',.23)
        self.c.step(CENTER,'NONE',.24)
        self.assertTrue(self.c.step(BLANK,'NONE',.25)[2].startswith('GREEN_TURN'))
        self.assertTrue(self.c.step(CENTER,'NONE',1.7)[2].startswith('STOP'))
        self.assertEqual(self.c.step(CENTER,'NONE',1.8)[:2],(0.,0.))

    def test_unmarked_opposite_bends_change_steering(self):
        c=self.ns['CurveController']()
        l,r,_=c.step(RIGHT,0.)
        self.assertGreater(l,r)
        l,r,state=c.step(dict(near=.2,middle=.1,far=0.),.05)
        self.assertEqual(state,'CURVE_ALIGN')
        self.assertGreater(r,0.)
        l,r,_=c.step(LEFT,.1)
        self.assertGreater(r,l)
        l,r,state=c.step(BLANK,.15)
        self.assertEqual(state,'SEARCH')
        self.assertEqual(l,0.)
        self.assertGreater(r,0.)
        self.assertTrue(c.step(BLANK,.51)[2].startswith('STOP'))

    def test_local_completion_not_blocked_by_next_far_bend(self):
        c=self.ns['CurveController']()
        c.step(RIGHT,0.)
        p=dict(near=0.,middle=0.,far=-.8)
        c.step(p,.1)
        self.assertEqual(c.step(p,.2)[2],'FOLLOW')
        self.assertIsNone(c.turn_started)
        self.assertGreater(c.step(LEFT,.3)[1],c.step(LEFT,.31)[0])

    def test_motor_commands_never_reverse(self):
        for offset in (-1.,-.8,-.3,0.,.3,.8,1.):
            p=dict.fromkeys(CENTER,offset)
            for result in (self.ns['alignment_commands'](p),self.ns['CurveController']().step(p,0.)[:2]):
                self.assertTrue(all(0.<=v<=self.ns['MAX_SPEED'] for v in result))

    def test_ambiguity_holds_then_latches(self):
        self.assertEqual(self.c.step(BLANK,'NONE',0.,reliable=False)[:2],(0.,0.))
        self.assertTrue(self.c.step(BLANK,'NONE',.36,reliable=False)[2].startswith('STOP'))
        self.assertEqual(self.c.step(CENTER,'NONE',.4)[:2],(0.,0.))

    def test_gap_recovery_retained(self):
        for t in (0.,.05,.1): self.c.step(CENTER,'NONE',t)
        self.assertTrue(self.c.step(BLANK,'NONE',.15)[2].startswith('GAP'))
        self.assertTrue(self.c.step(CENTER,'NONE',.2)[2].startswith('GAP'))
        self.assertEqual(self.c.step(CENTER,'NONE',.25)[2],'FOLLOW')

    def test_tracker_rejects_equally_plausible_paths(self):
        t=self.ns['LineTracker']()
        candidates={name:[(-.4,1),(.4,2)] for name in CENTER}
        self.assertEqual(t.select(candidates,0.),BLANK)
        self.assertTrue(t.ambiguous)

    def test_tracker_temporal_and_marker_branch_preference(self):
        t=self.ns['LineTracker']()
        t.select({name:[(-.4,1)] for name in CENTER},0.)
        choices={name:[(-.4,1),(.4,2)] for name in CENTER}
        self.assertTrue(all(v<0 for v in t.select(choices,.05).values()))
        fresh=self.ns['LineTracker']()
        self.assertTrue(all(v>0 for v in fresh.select(choices,0.,preferred=1).values()))

    def test_tracker_does_not_force_disconnected_jump(self):
        t=self.ns['LineTracker']()
        result=t.select(dict(near=[(-.7,1)],middle=[(.7,2)],far=[(.7,2)]),0.)
        self.assertIsNone(result['near'])
        self.assertGreater(result['middle'],0.)

    def test_marker_identity_retention_and_distinct_new_marker(self):
        t=self.ns['MarkerTracker']()
        a=t.update([marker(y=145)],0.,224,224)[0]
        t.consume((a['track_id'],))
        t.update([], .1,224,224)
        b=t.update([marker(y=155),marker(x=145,y=110)],.2,224,224)
        self.assertEqual(b[0]['track_id'],a['track_id'])
        self.assertFalse(b[0]['eligible'])
        self.assertTrue(b[1]['eligible'])
        c=t.update([marker(y=160),marker(x=145,y=140)],.3,224,224)
        direction,ids=t.instruction(CENTER,c,224,224)
        self.assertEqual(direction,'RIGHT')
        self.assertEqual(ids,(b[1]['track_id'],))

    def test_unknown_lower_entry_is_not_rearmed(self):
        t=self.ns['MarkerTracker']()
        t.update([],0.,224,224)
        self.assertFalse(t.update([marker(y=170)],.1,224,224)[0]['eligible'])

    def test_both_stops_and_unknown_does_not_trigger(self):
        for time in (0.,.03): self.c.step(CENTER,'BOTH',time,(1,2))
        self.assertTrue(self.c.step(CENTER,'BOTH',.06,(1,2))[2].startswith('STOP'))

try:
    import cv2
    import numpy as np
    HAS_IMAGES=True
except ImportError:
    HAS_IMAGES=False

@unittest.skipUnless(HAS_IMAGES,'OpenCV and numpy needed for generated image tests')
class V7ImageTests(unittest.TestCase):
    def setUp(self):
        self.ns=namespace(images=True)
        self.t=self.ns['LineTracker']()

    def frame(self):
        return np.full((224,224,3),255,np.uint8)

    def test_straight_line_and_blank(self):
        frame=self.frame()
        cv2.line(frame,(112,223),(112,0),(0,0,0),10)
        positions,overlay,mask=self.t.detect(frame,0.)
        self.assertTrue(all(abs(v)<.03 for v in positions.values()))
        self.assertFalse(self.t.ambiguous)
        self.assertEqual(self.t.detect(self.frame(),.1)[0],BLANK)

    def test_two_branches_and_temporal_tracking(self):
        frame=self.frame()
        cv2.line(frame,(65,223),(65,0),(0,0,0),10)
        self.t.detect(frame,0.)
        cv2.line(frame,(158,223),(158,0),(0,0,0),18)
        positions,_,_=self.t.detect(frame,.05)
        self.assertFalse(self.t.ambiguous)
        self.assertTrue(all(v<0 for v in positions.values()))
        fresh=self.ns['LineTracker']()
        fresh.detect(frame,0.)
        self.assertTrue(fresh.ambiguous)

    def test_slanted_continuous_line(self):
        frame=self.frame()
        cv2.line(frame,(60,223),(165,0),(0,0,0),9)
        p,_,_=self.t.detect(frame,0.)
        self.assertTrue(all(v is not None for v in p.values()))
        self.assertLess(p['near'],p['middle'])
        self.assertLess(p['middle'],p['far'])

    def test_horizontal_patch_not_averaged_to_center(self):
        frame=self.frame()
        cv2.rectangle(frame,(0,160),(223,198),(0,0,0),-1)
        p,_,_=self.t.detect(frame,0.)
        self.assertIsNone(p['near'])

    def runner(self,drive=False,stale=False,fail=False):
        ns=self.ns
        frame=self.frame()
        cv2.line(frame,(112,223),(112,0),(0,0,0),10)
        ns['latest']={'sample':(frame,0.)}
        commands=[]
        stops=[]
        def set_motors(l,r):
            commands.append((l,r))
            if fail:
                raise RuntimeError('simulated driver failure')
        ns['robot']=SimpleNamespace(stop=lambda:stops.append(True),set_motors=set_motors)
        clock=[.6 if stale else 0.]
        def monotonic():
            clock[0]+=.02
            if not stale:
                ns['latest']['sample']=(frame,clock[0])
            return clock[0]
        ns['time']=SimpleNamespace(monotonic=monotonic,sleep=lambda _:None)
        for name in ('overlay_view','mask_view','status_view'):
            ns[name]=SimpleNamespace(value=None)
        ns['bgr8_to_jpeg']=lambda _:b'jpeg'
        tree=ast.parse(''.join(CELLS[12]['source']))
        tree.body=[node for node in tree.body if isinstance(node,ast.FunctionDef)]
        exec(compile(tree,'runner','exec'),ns)
        with redirect_stdout(StringIO()):
            if fail:
                with self.assertRaises(RuntimeError): ns['run_v7'](seconds=.4,drive=drive)
            else:
                ns['run_v7'](seconds=.4,drive=drive)
        self.assertGreaterEqual(len(stops),2)
        return commands

    def test_runner_preview_never_drives(self):
        self.assertEqual(self.runner(),[])

    def test_runner_drive_stops_on_exit(self):
        self.assertTrue(self.runner(drive=True))

    def test_runner_stale_camera_never_drives(self):
        self.assertEqual(self.runner(drive=True,stale=True),[])

    def test_runner_driver_exception_still_stops(self):
        self.assertTrue(self.runner(drive=True,fail=True))

    def test_green_detector_and_ids(self):
        frame=self.frame()
        cv2.rectangle(frame,(73,145),(87,160),(0,200,0),-1)
        found,_=self.ns['detect_green'](frame)
        self.assertEqual(len(found),1)
        tracker=self.ns['MarkerTracker']()
        markers=tracker.update(found,0.,224,224)
        direction,ids=tracker.instruction(CENTER,markers,224,224)
        self.assertEqual(direction,'LEFT')
        self.assertTrue(ids)

if __name__=='__main__':
    unittest.main()
