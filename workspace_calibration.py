from compliance_calib import *
np.set_printoptions(suppress=True)

idx = 0
p = []
point_order = ['origin', 'x-axis', 'y-axis']
tormach_calib = calib_module()
tormach_calib.start(homing = True)
while idx < 3:
    print('***Now place the tool on the ',
          point_order[idx], ' in the parts frame***')
    key = input('Press <ENTER> to confirm the x,y location, press <q> then <ENTER> to quite the calibration')
    if key == 'q':
        print('Calibration aborted')
        break
    tormach_calib.touch_off_on = True
    while tormach_calib.touch_off_on:
        time.sleep(0.1)
    p.append(tormach_calib.touch_off_loc)
    idx += 1
    print('     ')
tormach_calib.stop()


if idx == 3:  # calibration finished
    H_BP = np.eye(4)
    x = (p[1]-p[0])/np.linalg.norm(p[1]-p[0])
    y_temp = p[2]-p[0]
    z = np.cross(x,y_temp)/np.linalg.norm(np.cross(x,y_temp))
    y = np.cross(z,x)
    R_BP = np.array([x,y,z]).T
    H_BP[:3,:3] = R_BP
    H_BP[:3,-1] = p[0]
    print('Calibration H_(base)_(part)=\n',
          H_BP)

    with open('config/workspace_H.yaml', 'w') as f:
        yaml.dump(H_BP.tolist(), f)

with open('config/workspace_H.yaml') as f:
    loaded_H = yaml.load(f)
loaded = np.array(loaded)
print loaded