#!/usr/bin/env python3
"""Regenerate all deliverables from one CAD build; stop on failed verification.
Use --no-video to skip the optional 44-second MP4 render.
"""
import importlib,sys
import model

def main():
    model.build()
    modules={n:importlib.import_module(n) for n in [
        'build','export_gazebo','check_structure','check_controls','timing',
        'motion','validate','check_batch_fixture','render','make_report','summarize_validation']}
    # All modules share model.PARTS/LINKS. Avoid rebuilding the same B-reps ten times.
    for m in modules.values():
        if hasattr(m,'build'):m.build=lambda:None
    for name in ['build','export_gazebo','check_structure','check_controls','timing',
                 'motion','validate','check_batch_fixture']:
        print('Running',name,flush=True);modules[name].main()
    modules['render'].sheet()
    if '--no-video' not in sys.argv:modules['render'].video()
    modules['summarize_validation'].main()
    modules['make_report'].main()
    print('REGENERATION COMPLETE',flush=True)

if __name__=='__main__':main()
