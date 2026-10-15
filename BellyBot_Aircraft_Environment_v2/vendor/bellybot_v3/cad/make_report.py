#!/usr/bin/env python3
"""Seven-page concept report, assembled from the checked revision and CAD renders."""
import json
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,Image,PageBreak
from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
ROOT=Path(__file__).resolve().parents[1];D=ROOT/'docs'
pdfmetrics.registerFont(TTFont('DejaVu','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'))
pdfmetrics.registerFont(TTFont('DejaVuBold','/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'))
pdfmetrics.registerFontFamily('DejaVu',normal='DejaVu',bold='DejaVuBold')
S=getSampleStyleSheet()
for key in ['Normal','BodyText','Heading1','Heading2','Title']:
    S[key].fontName='DejaVu';S[key].textColor=colors.HexColor('#203e36')
S['BodyText'].fontSize=9.6;S['BodyText'].leading=14;S['BodyText'].spaceAfter=8
S['Heading1'].fontName='DejaVuBold';S['Heading1'].fontSize=22;S['Heading1'].leading=27;S['Heading1'].spaceAfter=14
S['Heading2'].fontName='DejaVuBold';S['Heading2'].fontSize=12;S['Heading2'].leading=16;S['Heading2'].spaceBefore=10
S['Title'].fontName='DejaVuBold';S['Title'].fontSize=31;S['Title'].leading=35
S.add(ParagraphStyle('Small',parent=S['BodyText'],fontSize=8.2,leading=11))
S.add(ParagraphStyle('Kicker',parent=S['BodyText'],fontName='DejaVuBold',fontSize=10,textColor=colors.HexColor('#198666'),spaceAfter=15))
story=[]
def p(text,style='BodyText'):story.append(Paragraph(text,S[style]))
def title(n,text):p('BELLYBOT / CONCEPT C / REVISION 3','Kicker');p(f'{n}. {text}','Heading1')
def pic(name,width=500,height=265):
    im=Image(str(D/name));im._restrictSize(width,height);story.append(im);story.append(Spacer(1,5))
def table(rows,widths):
    t=Table([[Paragraph(str(v),S['Small']) for v in row] for row in rows],colWidths=widths,hAlign='LEFT')
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dbeae2')),('VALIGN',(0,0),(-1,-1),'TOP'),
        ('LINEBELOW',(0,0),(-1,0),.7,colors.HexColor('#75a48e')),('LINEBELOW',(0,1),(-1,-1),.3,colors.HexColor('#d3dfd7')),
        ('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7)]))
    story.append(t);story.append(Spacer(1,9))
def page():story.append(PageBreak())
def footer(c,doc):
    c.saveState();c.setStrokeColor(colors.HexColor('#bfd0c6'));c.line(42,43,553,43)
    c.setFont('DejaVu',8);c.setFillColor(colors.HexColor('#637b70'))
    c.drawString(42,29,'C-DockStack-v3.0 | Concept engineering draft | 10 October 2026')
    c.drawRightString(553,29,f'{doc.page} / 7');c.restoreState()
def main():
    v=json.loads((D/'validation.json').read_text());tim=json.loads((D/'timing.json').read_text())
    cases=v['floor_pressure_screen']['cases'];peak=max(c['factored_max_patch_kg_m2'] for c in cases)
    p('HACKATHON CONCEPT DESIGN / DRAFT','Kicker')
    p('BELLYBOT<br/>Dock &amp; Stack','Title')
    p('A low-profile crawler with a telescopic lift, rotating tray and pitching floor-access cassette.','BodyText')
    pic('placement.png',500,290)
    table([['Required cover information','Complete before submission'],['Team name / institute','[Team name] / [Institute]'],['Team lead','[Registered lead name] / [Email] / [Contact]'],['Team members','[Names and email/contact details]']],[165,335])
    p('<b>What this revision demonstrates:</b> sketch-derived CAD, a connected yaw bearing/frame, retained slide guides, independent limited joints, geometric floor reach and offline checks.')
    p('<b>What it does not demonstrate:</b> successful physical baggage acquisition, verified motors/structure, sustained 12 bags/minute, aircraft-floor compliance, or a Gazebo/RViz runtime pass.','Small')
    page()
    title(1,'Concept and architecture')
    pic('home.png',500,235)
    p('The concept retains the sketch’s teal body, black crawler tracks with five wheels per side, central nested mast, round yaw plate, thin horizontal belt cassette and separately supported rear conveyor. Dimensions are engineering choices because the illustration has no scale.')
    table([['Element','Revised dimension / role'],['Body and tracks','Body length 1,320 mm; crawler width 1,100 mm; track contact length 1,320 mm. Rounded ends extend the crawler length to about 1,488 mm.'],['Lift and reach','Three 200 mm lift stages; two 500 mm horizontal stages. Nominal belt surface 495 mm above the floor when stowed.'],['Cassette and pitch','900 × 600 mm tray envelope; 25 mm cassette; rear pitch axis; 3 mm tapered nose.'],['Infeed','400 mm horizontal adjustment plus new 250 mm vertical adjustment.'],['Payload model','760 × 480 × 280 mm rigid test bag, assigned 32 kg. Robot assigned 173 kg.']],[125,375])
    p('The mobile robot, infeed and free test bag are separate Gazebo models. Robot state publishers expose the joint transforms to RViz. A supervisory node sequences motion; it does not supply luggage perception, path planning or a certified stop. Track surface controllers provide simulated traction, while the ten continuous wheel joints provide wheel rotation.')
    p('The visible camera and emergency-stop button are CAD housings. Camera processing, bag classification, force feedback and physical safety circuits remain integration work.','Small')
    page()
    title(2,'Corrected yaw support and guides')
    pic('yaw_mechanism.png',500,195)
    p('Revision 2 grouped disconnected solids into a rigid yaw link. V3 adds a continuous load path from the mast bearing through the carrier flange, four cranked ribs, crossbeams and rail pedestals. The rendered view removes the tray to expose that structure.')
    table([['Connection','Implemented geometry'],['Yaw bearing mount','Separate ring envelopes, mast plate, 380 mm rotating flange, 12 nominal M8 mounting positions per ring and coaxial drive envelope.'],['Retained slides','Two dovetail/gib interfaces; two 54 mm blocks per side per stage, end stops and rear ties that clear the moving runners.'],['Pitch hinge','Driven/idler stub shafts, hinge ears, bearing housings and a connected motor bridge.']],[125,375])
    p('Yaw is limited to ±95°; tray pitch to 0–32° downward. Three lift joints each travel 200 mm and two extension joints each travel 500 mm. Wheel and conveyor-drum rotation is continuous. Serial URDF joints have no mimic dependency; bag support is not faked with an attachment.')
    p('Before yaw: level the cassette, retract the infeed to at most 150 mm stroke, raise at least 80 mm and retract the horizontal stages. Independent viewer sliders bypass these multi-joint guards. Roller joints spin only the drums; yaw swings the upper frame.')
    p('Bearing dimensions use the THK RU178X mounting envelope (115/240/28 mm; inner/outer bolt circles 139/217 mm). Raceways, preload, threads and drive internals are simplified. Custom guides, joints and mounting faces still need strength, wear, fastening and actuator engineering. Source: thk.com Cross Roller Ring RU catalog.','Small')
    page()
    title(3,'Floor loading and unloading')
    pic('floor_side.png',500,205)
    p('The support correction raises the home belt to 495 mm and the rear hinge to 503.5 mm. The extended cassette creates a floor-access ramp. At <b>30.922°</b> pitch, the nose underside is <b>2 mm</b> above the nominal floor. At 840 mm extension it is 1.182 m ahead of the base origin; at 1,000 mm extension it is 1.342 m ahead.')
    p('Loading from the floor','Heading2')
    p('Align to an accessible bag edge; stop the base; extend and pitch the empty cassette. Advance the last 160 mm while running the tray belt toward the robot. Confirm that the bag is fully supported before leveling the cassette and retracting. The supplied floor world uses a free bag; no attachment or teleportation establishes success.')
    p('Unloading or retrieval','Heading2')
    p('For floor deposition, first establish support and a clear path, then extend, pitch and run the belt outward while monitoring contact. Retrieval reverses the belt direction and insertion sequence. At an elevated layer the tray remains level: extend over a suitable support surface and coordinate belt advance with tray withdrawal.')
    table([['32 kg bag estimate','Result / meaning'],['Uphill friction at constant speed','Minimum μ ≈ 0.599 before insertion losses or acceleration. The simulated belt μ = 0.7 is an assumption.'],['Uphill gravity force','About 161 N; additional resistance and acceleration increase traction demand.'],['Pitch moment from bag alone','About 135 N·m at a 430 mm centre distance. Provisional 450 N·m joint effort is not a selected actuator.']],[175,325])
    p('A 2 mm CAD clearance is sensitive to floor variation and structural sag. A compliant nose, contact-force limits and measured height control are needed. Soft bags, inaccessible undersides and straps remain unresolved handling cases.','Small')
    page()
    title(4,'Speed improvement and its limits')
    p('The new infeed height adjustment removes a repeated mast round trip when several bags are transferred at the same layer. The fast planning case keeps yaw at zero, both surfaces at one height and the bag already staged at the infeed.')
    table([['Prepared layer-batch phase','Analytic duration'],['Receive through 900 mm belt travel','1.316 s'],['Extend tray by 800 mm','1.359 s'],['Retract 800 mm while advancing belt 1,425 mm','1.781 s'],['Command / verification allowance','0.200 s'],['Nominal total','4.6566 s, or 12.89 bags/min before additional delays']],[365,135])
    p('These are finite-acceleration quintic profiles, using up to 1.4 m/s total extension speed and 2.5 m/s² acceleration, and 1.5 m/s belt speed and 3.0 m/s² acceleration. The default commissioning speed scale is 0.25. Increasing software limits does not prove the motors can deliver the profile.')
    p('Why belt travel exceeds tray withdrawal','Heading2')
    p('The 1.78125 belt-to-retraction ratio advances the bag relative to the receiving surface as well as withdrawing the tray beneath it. In the no-slip geometric model, the bag’s rear edge clears the nose about 22 mm beyond the assumed stack front at 780 mm. The 550 mm receiver width leaves 42 mm clearance to each outboard rail. Clear channels beside the bag column are required. Its 555 mm top leaves 15 mm below the cassette and a 40 mm drop from the belt. Contact, deformation and settling can invalidate the estimate.')
    table([['Other operating case','Timing implication'],['65° yaw route','About 16.81 s before a separate deposit allowance; below 12/min.'],['Floor approach and return','About 6.73 s before bag transfer; below 12/min.'],['Prepared batch at 90% productive time','About 11.60 bags/min; below the target. More than 93.132% is needed under the exact model.']],[215,285])
    p('<b>Requirement status:</b> the design has a plausible fast mode, not proven mission-wide throughput. The test node waits for real bag-ready, supported and placed decisions, plus completed-motion acknowledgement; it logs completed transfers rather than treating an animation as a successful operation.')
    page()
    title(5,'Offline verification and floor loads')
    p(f'<b>{v["part_groups"]} CAD part groups</b> were checked for valid solids. STEP reimport passed. Five URDF variants passed independent forward-kinematics comparison across 40 random poses each, with consistent mesh paths and positive inertias. Thirty named configurations showed no cross-link solid intersections above the checker threshold.')
    p('The yaw carrier, middle slide, tip carriage and tray each pass a solid-level connectivity check. Retained guide engagement is checked through the 500 mm stroke. The unloaded 44-second demonstration is sampled at 2,641 times for joint/velocity limits, floor-tip height and robot height. Seventeen control checks cover sequencing and rejected invalid commands. These are not a continuous collision proof or a physical load test.')
    p('Floor-pressure screening','Heading2')
    p('The revised mass budget is 173 kg, or 205 kg with the 32 kg bag. Track contact is 1,320 × 350 mm per side. The following screen includes a 1.5 multiplier and assumes four equal-compliance, uniform-pressure half-track patches.')
    rows=[['Named pose','Factored peak kg/m²','Screen vs 600']]
    for c in cases:rows.append([c['pose'].replace('_',' '),f'{c["factored_max_patch_kg_m2"]:.1f}','Within' if c['within_screen_limit'] else 'Exceeds'])
    table(rows,[220,150,130])
    p(f'The largest named-pose result is {peak:.1f} kg/m². Real pressure is not uniform: individual wheel/track reactions, floor compliance, bag offsets, motion and contact loss need measurement. Passing this approximation does not certify aircraft-floor compatibility.')
    p('Verification boundary','Heading2')
    p('Gazebo, ROS 2 and RViz were not run in the authoring environment. SDF XML references and plugin target names were checked; backend behavior, conveyor direction, free-bag contact, joint tracking, odometry and actuator force limits still need runtime commissioning. The assigned masses are estimates, not a materials bill of quantities.')
    page()
    title(6,'Requirements and submission handoff')
    table([['Hackathon requirement','Evidence and remaining work'],['Height below 1.2 m','Named CAD poses and an unloaded motion envelope checked. Loaded roof guard supplied. Check actual curved hold and routes.'],['20–32 kg irregular / fragile bags','Rigid 32 kg case and preliminary force estimates only. Test grip, damage, soft bags, handles and straps.'],['At least 12 bags/min','4.66 s modeled prepared-layer cycle. Sustained mixed operations, retries and setup remain unverified.'],['Floor loading ≤600 kg/m²','See five recalculated mass/COM screening cases on page 6. Actual local pressure and aircraft-floor qualification remain open.'],['Floor loading / unloading','Geometric access now exists. Free-bag acquisition, support and controlled deposit need contact tests.']],[170,330])
    p('Next engineering gates','Heading2')
    p('Commission the supplied bench and floor worlds at low speed. Verify separate joints, belt direction, gravity and contact. Add sensing for bag support and stable placement. Measure insertion forces, deflection and drive effort with representative 20/32 kg luggage. Then run repeated batches and a complete floor/stack mission, including layer changes, repositioning, failures and recovery.')
    p('Safety and human interaction','Heading2')
    p('Keep people clear of pinch, shear and overturning zones. The supplied software stop and command watchdog are commissioning features; they are not an independent, safety-rated stop or a physical holding brake. Final engineering needs actuator brakes, contact sensing and human-access controls appropriate to the actual installation.')
    p('Included handoff and submission preparation','Heading2')
    p('The ZIP contains editable CadQuery source, STEP/GLB assemblies, independent URDF links and meshes, Gazebo worlds, ROS launch/control files, a 44-second unloaded CAD MP4 and this seven-page draft. Follow QUICKSTART.md. Complete the cover details and rename the submission ZIP and numbered STP/MP4/PDF files using the guideline’s team/registered-lead convention.')
    p('Requirement source: user-supplied “Hackathon Guidelines for Design Submission_Oct 2.pdf”, page 1. Deadline stated there: 19 October 2026. The team lead is responsible for final submission. This package has not been submitted.','Small')
    doc=SimpleDocTemplate(str(D/'BellyBot_DockStack_v3_report.pdf'),pagesize=(595.28,841.89),rightMargin=47,leftMargin=47,topMargin=40,bottomMargin=57,
        title='BellyBot Dock & Stack — revision 3 concept report',author='Team details to be completed')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    print(D/'BellyBot_DockStack_v3_report.pdf')
if __name__=='__main__':main()
